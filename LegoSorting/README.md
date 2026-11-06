# LegoSorting

A Meca500 sorts LEGO bricks into piles by colour, guided by webcams. Everything is
measured in the robot's base frame, so the cameras can be moved freely. See
[PLAN.md](PLAN.md) for the full design and stages.

## Status

**Stage 1 (camera calibration + base tracking): code done, tested on synthetic
images and in simulation; live run pending.** The real C920 opens and its controls
lock; the mat has not yet been printed and viewed.

## Layout

```
legosort/        library
  boards.py      ChArUco board (intrinsics) and base mat of ArUco markers
  camera.py      webcam capture, locking focus/exposure/white balance (v4l2-ctl)
  intrinsics.py  ChArUco detection and calibration, save/load
  tracker.py     per-frame camera pose in the robot frame; pixel <-> robot table point
  geometry.py    transforms, projection, ray/plane intersection
  synth.py       synthetic camera images, for tests
  sim.py         simulation in the VRRobotController twin: viewer client, meshes,
                 virtual camera, robot with injectable joint errors
  simserver.py   starts the viewer headless (server + Chrome) for tests and sim_view
  robotmodel.py  the Meca500's link meshes posed by joint angles; robot silhouette masks
  robotcal.py    lens + camera pose from the robot arm's outline (safe pose planning, fit)
  lego.py        brick sizes, colours, poses and shape (shell, studs, tubes)
  physics.py     MuJoCo simulation: gravity, contact between bricks, mat, arm and gripper fingers
  pick.py        grippable bricks in a pile, and the pick-and-place motion
  megp25e.py     MEGP 25E gripper + STAP01 adapter from the vendor STEP (models/megp25e/)
  overlay.py     drawing helpers
apps/            scripts (run from the project folder)
config/          base_mat.json: marker layout + mat position in the robot frame
prints/          generated PDFs to print (not in git; regenerate with make_targets.py)
data/            per-camera settings, intrinsics, captured frames (not in git)
tests/           synthetic end-to-end tests
```

Needs Python 3 with `opencv-python` ≥ 4.7 (ArUco), `numpy`, `Pillow`, `mujoco` (piles
only: `python3 -m pip install --user mujoco`), and `v4l2-ctl`
(`sudo apt install v4l-utils`) to lock the camera controls.

## Stage 1: step by step

1. **Make the targets.**
   `python3 apps/make_targets.py` → `prints/charuco_A4.pdf`, `prints/base_mat_A3.pdf`.
   Print both at **100% / actual size**. Check each 100 mm bar with a ruler. Glue the
   ChArUco sheet to something flat and rigid (foamboard or glass).

2. **Lay the mat.** Put it on the table in front of the robot, with the edge marked
   *TOP EDGE TOWARDS ROBOT* facing the robot and centred on its X axis, and tape it
   down. Measure the distance from the robot base axis (J1) to the mat's centre cross,
   and enter it as `board_in_robot.xyz[0]` in `config/base_mat.json` (default
   230 mm). This hand measurement is only a first guess: stage 2 fits it properly.

3. **Set up the camera.** Mount it where it will normally sit, looking at the mat.
   `python3 apps/camera_setup.py`
   On first run it locks exposure/white balance from the camera's auto mode. Adjust
   focus with `f`/`F` until the sharpness number peaks on the mat, then press `s`.

4. **Calibrate the intrinsics.** Measure the printed ChArUco square (across 9 squares
   ÷ 9), then
   `python3 apps/calibrate_intrinsics.py --square-mm 30.0`
   Hold the board still in about 20–30 places, tilted up to ~45° and reaching every
   image corner, until the coverage grid is all green. Press `c`. Aim for RMS < 0.5 px.

5. **Live check.**
   `python3 apps/live_view.py`
   The robot axes should appear at the robot base, and the grid on the mat. Click a
   few spots on the table to drop pins, then move the camera by hand.
   **Stage 1 is done when** the pins and grid stay on the same physical spots, and
   the camera jitter is well below 1 mm while the camera is still.

Changing focus or zoom invalidates the intrinsics: `live_view` warns when the stored
settings differ from those used for calibration.

### Alternative to step 4: calibrate from the robot itself

No board to hold: the arm moves through ~15 poses while the camera watches, and the
lens and camera pose are fitted to the arm's outline plus the mat markers. It needs
the mat in view, steady lighting, and the arm on the twin through `meca500_bridge.py`.

```bash
python3 apps/calibrate_from_robot.py --camera c920 --real --url ws://127.0.0.1:8099/ws --session ID
```

It refuses to move a real arm without `--real`, shows the planned joint poses and
waits for `yes`. Every pose and every move between poses (checked in 5° steps) keeps
the arm and a 90 x 50 mm gripper cylinder 40 mm above the table and 150 mm from the
camera (`--clearance`, `--tool-length`, `--tool-radius`, `--camera-clear`). The arm
is fed ≤8° steps so it follows the checked path, and Ctrl+C holds it where it is.
The gripper is not in the model, so by default the flange link's outline is left
out of the fit (`--fit-flange` to use it). Output: `data/intrinsics_c920.json`, and
`data/robotcal_c920/` with the captures, `result.json` and `overlay.png` (fitted
outline in green). The run fails, and says so, if the median outline distance is above 0.6 px
after restarts; check `overlay.png`. `--replay data/robotcal_c920` refits saved
captures without moving anything. To rehearse without the arm, run
`meca500_bridge.py --sim` and use `--sim --real` (virtual camera, moves through the bridge).

## Simulation (VRRobotController twin)

`../VRRobotController` renders the robot, a table, the base mat and (later) bricks.
A virtual camera captures exact images through the viewer's `captureImage`
command (added for this project, with `addObject`), and the vision code runs on
them unchanged.

```bash
python3 apps/calibrate_from_robot.py --sim --distort     # lens from the robot outline
python3 apps/sim_view.py                                 # own headless viewer
python3 apps/sim_view.py --url ws://127.0.0.1:8080/ws    # a viewer you have open
```

Orbit the virtual camera (`a`/`d`, `w`/`s`, `+`/`-`), click to drop pins, and move
the arm (`j`). The overlay comes from the tracked pose only, and the true camera
error is shown alongside.

- **Frames:** everything in LegoSorting is in the robot frame (= GNKinematics). The
  viewer API reports positions with **Y mirrored**; `legosort.sim.to_api` /
  `from_api` convert, and a test checks this against GNKinematics.
- **Exposure:** the viewer's lighting overexposes matte objects; captures use
  `exposure=0.15`.
- **Inaccurate robot:** `SimRobot(viewer, joint_offsets=...)` adds fixed errors to
  every joint command, so stage 2 calibration can be tested against known errors.
- Objects added by the sim are named `ls_*` and are replaced on each run. In a
  viewer tab you use yourself they persist (IndexedDB) until removed.

### Piles of bricks (physics)

Bricks dropped in a pile are simulated in MuJoCo (`legosort/physics.py`): gravity,
friction, and contact between bricks and with the mat. Each brick collides as the
same shape the viewer draws: a hollow shell (1.2 mm walls, 1 mm top), studs, and the
underside tubes (pins on 1-wide bricks). So studs can catch inside another brick's
underside, and an upside-down brick shows its tubes to the camera.

```bash
python3 apps/sim_pile.py --n 30 --seed 1                  # headless; data/sim_pile*.png
python3 apps/sim_pile.py --url ws://127.0.0.1:8099/ws     # in a viewer you have open
python3 apps/sim_pile.py --remove 5                       # then lift the top 5 off, re-settling each time
```

```python
sim = PileSim(bricks)          # lego.Brick list: sizes and colours
sim.pour(centre=(230, 0))      # dropped one by one, random orientation, then settled
pile = sim.bricks()            # Brick with full pose: b.T (4x4), b.R, b.tilt (0 = studs up)
sim.remove(i); sim.settle()    # a pick: the rest of the pile slumps
```

Notes: 30 bricks pour and settle in about 6 s. The contact is stiff (2 ms time
constant, 0.25 ms step), so bricks sink < 0.01 mm. Drops are kept short (30 mm above
the pile) so a falling brick moves well under a wall thickness per step. Bricks not
yet poured, or removed, are parked away from the scene, so the model is never rebuilt.
Real clutch power (studs gripping) is not modelled: stacked bricks only sit, held
by friction. `detect.py` still assumes studs-up, separate bricks.

### The robot in the physics

`PileSim(bricks, robot=True)` adds the Meca500 (the viewer's own joint chain; each
link collides as its convex hull) and the Mecademic MEGP 25E, built from the vendor
STEP files (`legosort/megp25e.py`). The arm follows its joint path exactly and
pushes whatever it meets. Each jaw is a force-limited servo (10 N), so a brick is
held by friction only. The physics TCP matches GNKinematics FK to < 0.1 mm.

The gripper, from the CAD: the bare jaws open to 17.0 mm and close to 11.4 mm
(5.6 mm stroke), so they grip 2-wide bricks (15.8 mm) with no extra fingers. The
jaws stand only 8.2 mm proud of the body, so a grasp is skipped when the body would
land on the brick or its neighbours before the jaws reach the brick's middle.

Mounting (`MEGP25E(mount="side")`, the default, as on the real arm): no adapter. A
printed spacer joins the flange to the gripper's wide side face, so the jaws point
at 90 deg to the flange axis. Estimated from a photo, to be measured: spacer 7 mm
thick and 20 mm across (`spacer`, `spacer_d`); flange axis 21 mm above the jaw tips
(`axis_to_tips`). That puts the TCP 16 mm out along the flange axis and 21 mm off
it. Unknown: how the gripper is turned about the flange axis at J6 = 0
(`mount_angle`).

**Sorting a pile** (`apps/sim_pick.py`): runs until every brick is on its colour's spot,
or nothing more can be done. Each step:

1. **Grip from the side** if any brick can be gripped (`pick.side_grasps` + `pick_side`),
   outermost first. With the gripper side-mounted, gripping from the top would put the
   flange axis level and the wrist down at jaw height, so the jaws come in horizontally
   from outside, with the flange axis vertical and the wrist above. The closing axis is
   horizontal, across a 15.8 mm side, with at least 4 mm of jaw on the brick. A grip is
   used only if the gripper's whole path (down, in, up) misses every other brick, checked
   against the real gripper shape plus the wrist (`pick.path_clear`). The brick's centre
   is set down on the spot.
2. Otherwise **push** a low brick with the gripper (`pick.plan_push` + `push`): jaws
   first and open, so a brick up to 17 mm across sits between them and the flat end of
   the body pushes it. First choice is a clean push toward its spot, up to 120 mm per leg,
   steering up to 80° around other bricks, touching nothing but its own colour's heap.
   Failing that, **spread the pile**: of all low bricks and 12 directions, the push that
   leaves a brick the most free space, shoving neighbours if need be, but never toward a
   spot, out of reach, or in toward the base.

The bare jaws (11.4–17.0 mm) can never grip a 1x1, 1x3 or 1x4, or a 2xN on its side,
so those travel by pushing only. Drop spots: one per colour (up to 10), every 15° on an
arc 235 mm from the base. The default pile is at (165, 0). No home pose: travel height
is the pile's top + 25 mm.

Results (20 bricks, seeds 1/3/5, `--no-viewer`):

| gripper | sorted | grips | pushes | robot time |
|---|---|---|---|---|
| bare jaws | 13 / 7-13 / 12-13 | 7-12 | 39-91 | 3.5-8 min |
| stepped fingers (`--fingers`) | 16 / 17 / 15 | 18-20 (about half on the narrow step) | 10-26 | 2.2-3.2 min |

With the fingers nearly every brick is grippable, so pushing only separates clumps.
What's left is 2-5 bricks pressed together where the gripper can't come down
beside them (its footprint is about 57 x 27 mm plus the wrist), or the odd brick knocked out
of reach (1 drop in 55 landed 65 mm off its spot). Small changes reshape the pile
chaotically, so results vary between runs.

Physics safeguard: the arm is driven with unlimited force, so a brick pinched against
the mat could be shot out at metres per second. `PileSim` caps brick speed across the
mat and upward at 0.4 m/s (`max_brick_speed`), except for bricks touching the fingers.

### Stepped fingers (to 3D print)

The bare jaws grip only 2-wide bricks. The printed fingers in `models/megp25e/fingers/`
bolt onto the jaw blocks and give two grip ranges:

| step | where | range | grips |
|---|---|---|---|
| narrow | deep in, 2.4 mm pads on the jaw faces | 6.6-12.2 mm | 1-wide bricks (7.8), any brick across its height (9.6; 11.3 over the studs) |
| wide | the 10 mm at the tips, flush with the bare jaws | 11.4-17.0 mm | 2-wide bricks (15.8), 1x2 lengthways |

The step between them stops a 2-wide brick going in too far. The fingers stick 12 mm
out beyond the jaw tips and reach 3.5 mm lower than the jaws (1.9 mm above the mat at
the lowest grip). So every brick has a grippable side however it lies.

**Printing** (PLA): `finger_a_print.stl` and `finger_b_print.stl` (one of each; they
differ slightly, like the jaws), already laid flat with the jaw side down and the
countersink up, so they need no supports. 0.2 mm layers, 100% infill (they are small:
13.5 x 20 x 6.4 mm, ~1 cm³). Check the pad thickness (2.4 mm) with calipers; it sets the
narrow range.

**Fitting**: one M3 x 10 countersunk screw (DIN 7991) per finger, from the gripping
face through the pad and the jaw's 3.2 mm hole, M3 nut on the outside of the jaw. The
two 1.6 mm holes take optional 1.5 mm dowel pins to stop the finger turning; without
them, tighten the screw with the finger's back edge square to the gripper body. Fit
them with the gripper open. Then close it by hand: the fingers should not touch the body
(0.2 mm gap) or each other.

`fingers.step` has both fingers in the gripper's STEP frame, and `fit.png` shows them on
the gripper. To change the design, edit the constants at the top of
`tools/megp25e_fingers.py` and run it in FreeCAD (MCP bridge or the macro console); it
checks for interference with the gripper open and closed, and rewrites everything here.

In the simulation: `MEGP25E(fingers=True)`, or `apps/sim_pick.py --fingers`.

To change the CAD, re-run `tools/megp25e_export.py` in FreeCAD (see its docstring): it
meshes the open `MEGP_fingers2` document (or the vendor STEP) and writes
`models/megp25e/megp25e_parts.npz`, about 57k triangles, which the viewer draws as is.
`PhysicsView` plays the simulation in the viewer: joints, gripper and
bricks moved every frame.

```bash
python3 apps/sim_pick.py --url ws://127.0.0.1:8099/ws     # watch it sort live
python3 apps/sim_pick.py --video                          # headless; data/sim_pick.mp4
python3 apps/sim_pick.py --seed 3 --no-viewer             # physics only, fast; prints each step
```

Reach: with the tool pointing down, the Meca500 reaches only about 260 mm from its
axis at the mat (245 mm at 80 mm up). The mat is centred at 230 mm, so its far half
is out of reach. The pile defaults to (165, 0), with drop spots inside the reach.
Grasp choice uses the simulator's true poses for now, not the camera.

**Driving the arm yourself** (`apps/sim_teleop.py`): closes the loop, so you move the
robot in the viewer and it interacts with the bricks. The app attaches to the viewer
as a robot bridge (like `meca500_bridge.py --sim`), but the robot is the physics arm:

```bash
python3 apps/sim_teleop.py --url ws://127.0.0.1:8099/ws             # 12 bricks
python3 apps/sim_teleop.py --url ws://127.0.0.1:8099/ws --n 20 --fingers
```

- The viewer's joints (sliders, IK gizmo, VR, macros) are the command. The physics
  arm follows them at the Meca500's joint speeds, times the bridge panel's speed scale
  (default 100 %), and pushes whatever it meets. It updates 60 times a second.
- The gripper panel (Open, Close, opening slider) and the VR trigger work the physics
  jaws. "holding part" shows when a brick touches both jaws. Gripping force is fixed
  in the sim (`MEGP25E.force`), so the force slider does nothing.
- Bricks and gripper are drawn where the physics has them. The viewer's arm shows the
  command, so after a big jump (Home, a typed angle) the gripper trails the drawn arm
  until the physics catches up.
- Disable and E-stop freeze the arm; Enable or Resume carry on. Reset pours a new pile.
- The arm is kinematic, so it never stalls: it can drive through the mat or a heap.
- Dropped bricks bounce (`--bounce`, the coefficient of restitution, default 0.5;
  `--bounce 0` for the old dead contacts). In `PileSim(restitution=e)` only a brick
  moving faster than 50 mm/s gets bouncy contacts, for 40 ms after it slows. Bouncy
  contacts all the time never let a pile come to rest: the bricks jitter at 1-2 mm/s,
  slide off the heap and never sleep.
- It refuses to start if another bridge (possibly the real arm) is attached to the viewer.
- Bricks at rest sleep (`PileSim(sleep=True)`): MuJoCo skips them until the arm or
  another brick touches them. A 60 Hz frame then costs about 3 ms of physics with
  20 bricks (37 ms without). The physics step is 0.5 ms (0.25 ms in `sim_pick`, which
  leaves sleep off so its results match earlier runs). It warns if it falls below 90 %
  of real time.

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
```

`tests/test_sim.py` starts the viewer headless and is skipped if
`../VRRobotController` (or `$VRROBOT_DIR`) or Chrome is missing.

(The environment variable works around a broken `anyio` pytest plugin on this
machine.) The tests render the targets through a synthetic camera with lens
distortion, then check that the intrinsics are recovered, that the camera pose
is recovered to < 2 mm / 0.15°, that tracking survives half the mat being hidden,
and that a fixed table point gets the same robot coordinates (< 1 mm) from
different camera positions. The simulation tests repeat the pose checks on
viewer-rendered images (camera < 1 mm / 0.1°) and check the viewer's FK
against GNKinematics.
