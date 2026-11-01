# LegoSorting — Plan

Use a Meca500 arm to sort LEGO bricks into piles by colour, with webcams as the
feedback. Everything is measured in the **robot frame**, never the camera frame, so
a camera that is moved or bumped is simply re-solved on the next frame.

## Core idea

Each frame:

1. Find the camera pose relative to the robot base (markers fixed to the robot/table).
2. Detect bricks in the image and project them onto the known table plane.
3. Express brick position, yaw, type and colour in robot coordinates.
4. Plan a grasp across the brick's narrow side and send the move to the robot.

The camera extrinsics drop out; only the camera intrinsics must be fixed and known.

## Locating the robot in the image

| Option | Method | Notes |
|---|---|---|
| 1. Base markers (start here) | ArUco/ChArUco board on the robot base plate or table, `cv2.solvePnP` per frame | ~1 mm at 0.5–1 m; markers share the robot frame |
| 2. Flange marker + FK | Move to ~20 poses, observe a marker on the flange, compare with `f_kinematics`, least-squares fit for the base-marker → robot transform | Uses the robot dimensions; no hand measurement; also detects drift during operation |
| 3. Markerless (later) | Fit the robot CAD model / keypoints to the image | The "physical AI" stage; less accurate initially |

**Visual servoing for the final approach:** when the camera sees both the gripper
marker and the brick, close the error measured in the image. This cancels most
remaining calibration error.

## Brick detection

- **Position:** a single calibrated camera is enough. Intersect each pixel ray with
  the table plane; the brick height is known (brick 9.6 mm, plate 3.2 mm).
  A second camera adds robustness and coverage, but is not needed for 3D.
- **Orientation:** colour-segment each blob and fit `cv2.minAreaRect` to get the yaw.
- **Type:** measured footprint ÷ 8 mm stud pitch → 2×2, 2×4, 1×4, …
- **Grasp symmetry:** yaw is taken modulo 180° (modulo 90° for square bricks); pick the
  equivalent angle that needs the smallest wrist rotation.
- **Colour:** cluster in Lab space under fixed lighting; keep a white/grey reference
  patch on the board for per-frame white-balance correction.

## Robot side

The Meca500's own Cartesian moves are not accurate enough, so **do not use the
controller's IK** (`MovePose`). Instead:

- Solve the IK with **GNKinematics** from `../RobotVisualisation`
  (`GNKinematics/kinematics.py`, robot definition `Meca500_kin` in
  `RobotDefinitions.py`): `kin.setEulerTarget([x, y, z], a, b, g)` → 6 joint angles
  (ZYX Euler convention, same as the viewer), `kin.f_kinematics(angles)` for FK.
- Send the result with `mecademicpy` **`MoveJoints`** only, so the controller does no
  Cartesian maths of its own.
- **This only helps if our model is better than the controller's.** The nominal
  `Meca500_kin` uses the catalogue dimensions, so on its own it is no more accurate.
  The gain comes from calibrating it (stage 2): fit joint offsets (and later link
  lengths and the tool vector) from camera observations, then load the fitted values
  into the kinematics object (`setMotorOffset`, `setToolOffset`, …).
- `../RobotCalibration` (`robotcal`) is a calibration fork of the same kinematics
  core: `eight_poses()` enumerates the IK branches for a pose and `calibrate()` fits
  joint offsets by least squares. Reuse its solver, using our webcam/marker
  observations as the measurements. Its limits apply here too: a J1 offset can't be
  seen without an absolute reference (here the base markers provide one), and J6 needs
  targets at differing orientations.
- **Sign convention check:** the viewer config gives Meca500 J4 `apiSign: -1`.
  Confirm that GNKinematics joint signs and zeros match the Mecademic `MoveJoints`
  convention for every joint before any closed-loop motion. Test by sending small
  single-joint moves and comparing with the camera.
- Use the same model for reachability checks, picking a configuration, and avoiding
  the wrist singularity near J5 ≈ 0.
- The digital twin is now `../VRRobotController` (see Simulation). It carries its
  own copy of GNKinematics/RobotDefinitions; check that the two copies agree before
  relying on either.

## Simulation

The VRRobotController viewer (`../VRRobotController`, branch `vr-meca500`) is the
digital twin. It already drives the real arm through `meca500_bridge.py`
(mecademicpy, joint-velocity following with a 0.05° deadband) and the MEGP 25E
gripper, so the same viewer API serves simulation and hardware.

- Two commands were added to the viewer for this: `captureImage` with an exact camera
  (pose, fov, size; returns K) and `addObject` (GLB/STL bytes). The scene is
  built from Python (`legosort/sim.py`).
- The viewer API's Y axis is mirrored relative to GNKinematics (measured). Convert
  at the API boundary only.
- `SimRobot` adds joint offsets to every command. This stands in for the real arm's
  inaccuracy when developing stage 2.
- Still to add: bricks (with studs, LEGO colours), and the gripper. In the sim,
  "grip" = parent the brick to the flange link (L5) when the fingers close on it.

## Gripper constraint

The MEGP 25E opens only 0–5.6 mm, so each finger set grips **one** brick width
(±~2.8 mm). Fingers sized for 15.8 mm handle 2×N bricks across their width and
1×2 bricks lengthways; 1×N (7.8 mm) bricks would need a second finger set or
a different strategy. Choose the fingers before stage 4.

## Pitfalls

- **Lock webcam autofocus, zoom and exposure.** Autofocus changes the intrinsics and
  makes the 3D estimates drift.
- **Gripper fit:** a 2×4 brick is 15.8 mm across. The fingers must open wider than that
  and have pads shaped for LEGO. Suction does not seal well on studs.
- **Contrast:** dark grey and black bricks need a light, matte background.
- **Scope creep:** start with bricks spread out and studs-up. Touching, stacked or
  overturned bricks come later (for example by nudging them apart first).

## Stages

1. **Camera calibration + base tracking.** Calibrate the intrinsics (ChArUco), then
   track the base markers and show the camera pose in the robot frame live.
   *Done when:* the robot-frame axes overlay stays fixed on the robot while the camera
   is moved by hand.
2. **Robot calibration with the cameras.** First confirm the joint sign/zero
   convention against `MoveJoints`. Then command about 20–40 diverse joint
   configurations (several IK branches per pose where joint limits allow), observe
   the flange marker, and fit the base-marker → robot transform together with the
   GNKinematics joint offsets (reusing the `robotcal` solver).
   *Done when:* the predicted flange position (calibrated FK) agrees with the camera
   to well under the nominal model's error at held-out poses, and the fitted offsets
   are saved to `data/` and loaded at start-up.
3. **Brick detection.** Colour, type, position and yaw in robot coordinates, shown as
   an overlay.
   *Done when:* bricks at known positions are reported to within about 1–2 mm and a few
   degrees.
4. **Single pick-and-place, open loop.** Hover, descend, grip, lift, place.
   All motion uses GNKinematics IK → `MoveJoints`.
   *Done when:* the arm reliably picks one brick placed anywhere in the workspace.
5. **Full sort.** Loop over all bricks into colour piles; add visual servoing for the
   final approach.
6. **Harder cases.** Touching or overturned bricks, more colours, markerless robot
   tracking, or a learned detector trained on synthetic renders.

## Open questions

- **Gripper:** which one (for example the Mecademic MEGP 25), and what finger design?
- **Cameras:** how many webcams, and what models? Can focus and exposure be locked?
- **Controller:** can this PC reach the Meca500 over the network?
- **Workspace:** table size, background colour and lighting.

## Layout

See [README.md](README.md): library in `legosort/`, scripts in `apps/`, mat layout in
`config/`, per-camera calibration in `data/`. Stages 2+ will add robot control
(mecademicpy `MoveJoints` + GNKinematics with calibrated offsets) and brick detection
modules alongside.

## Progress

- **2026-11-01 — Stepped fingers (to print in PLA).** `tools/megp25e_fingers.py`
  (FreeCAD, built on the vendor jaw blocks; zero interference open/closed) ->
  `models/megp25e/fingers/` (print STLs, STEP, fit.png, fingers.json). Narrow step
  6.6-12.2 mm deep in (1-wide, any brick across its height), wide step 11.4-17.0 at
  the tips; 12 mm beyond the jaws; M3x10 countersunk + nut per finger.
  `MEGP25E(fingers=True)`, `sim_pick.py --fingers`. Seeds 1/3/5: 16/17/15 of 20
  sorted, 10-26 pushes, 2-3 min (bare: 13/7/13, 39-91 pushes). Left over: tight
  2-3 brick clumps the gripper can't reach beside. Not yet printed/tested.

- **2026-11-01 — Complete sort: grip, else push.** User: "run a complete sort".
  Corridor box replaced by a swept check of the real gripper + wrist shape
  (`pick.path_clear`). Bricks the jaws can't hold are pushed jaws-first with the jaws
  open (`pick.plan_push`/`push`): a clean push toward the spot, else a spreading push
  (most free space, may shove others, never toward a spot / out of reach / inward).
  One spot per colour on a 235 mm arc; pile default moved to (165, 0). Brick
  speed cap in `PileSim` (pinched bricks were shot out up to 2.7 m). Result: 12-14
  of 20 sorted, then stuck on a tight clump; chaotic between runs. Next: stepped
  fingers would make most bricks grippable.

- **2026-11-01 — Side grips, outermost first; no home pose.** User: with the side
  mount, grip from the sides and work inward; don't return to a start pose; base the
  travel height on the remaining pile. `pick.side_grasps`/`pick_side`: jaws
  horizontal, wrist above, corridor check, outermost first; travel height = pile top
  + 25 mm. Seed 1: 4/4 held at 4.4-6.0 s per brick (top grips took 11-14 s). It
  stops when only 1-wide, 1x1 or 2xN-on-their-side bricks remain (no 15.8 mm side
  horizontal). Fixes on the way: cap height in the body clearance (sign of grip y),
  jaw height from the gripped part of a tilted brick, min 4 mm jaw contact, jaw mass.

- **2026-11-01 — Gripper side-mounted, as on the real arm (user's photo).** No STAP01:
  a yellow printed spacer (~7 mm, ~Ø20) on the gripper's wide side face, jaws at 90 deg
  to the flange axis, axis ~21 mm above the jaw tips (estimates). Grasps are planned
  in a grip frame (`MEGP25E.T_tool_grip`). The wrist hangs as low as the jaw tips when
  the jaws point down, so `pick.orient` tilts the jaws and `sim.clearance` checks the
  arm against the mat. Seed 1 pile: 2/2 held. To measure: spacer thickness, axis to
  jaw tips, and the jaw direction at J6 = 0.

- **2026-11-01 — Real gripper (MEGP 25E + STAP01 from the vendor STEP).** FreeCAD
  (snap; it can only read ~/snap/freecad/common) exports `models/megp25e/megp25e_parts.npz`;
  `legosort/megp25e.py` builds collision hulls + render meshes in the tool frame.
  Bare jaws: 11.4-17.0 mm, TCP 75.2 mm, jaws 8.2 mm long. Grasps now keep the body
  above whatever is under it, and bricks are set down 2 mm above the drop spot (a
  2x4 dropped from 20 mm tumbled). Seed 1 pile: 3/3 held, then only bricks on their
  side or tilted are left: the short jaws can't reach their middles. Longer fingers
  would help; open: how the gripper is clocked on the real flange (mount_angle).

- **2026-11-01 — Robot in the physics (`PileSim(robot=True)`, `legosort/pick.py`,
  `apps/sim_pick.py`, `sim.PhysicsView`).** Kinematic arm (link hulls), MEGP-like
  gripper with force-limited fingers. Seed 1, 20-brick pile: 6/6 picks held, then no
  grippable brick left (the rest lie on their sides or are 1-wide). Bugs found: MuJoCo
  also needs `body_contype/conaffinity` changed at runtime; a tilted brick must be
  gripped at its centre height, not just below its top. Mat far half is out of reach
  with the tool vertical. Open: picking bricks on their side, 1-wide bricks, grasps
  from camera detections, and a tool tilt to reach further.

- **2026-11-01 — Physics for piles (`legosort/physics.py`, `apps/sim_pile.py`).**
  MuJoCo 3.17 (installed with `--user`). Bricks collide as hollow shells + studs +
  tubes (`Brick.parts`, also used by `sim.brick_glb`, which now draws any 3D pose via
  `Brick.R`). `PileSim.pour` drops bricks one by one; `remove` + `settle` models a
  pick. Seed 1, 30 bricks: 2 studs up, 4 upside down, 19 on their side, 5 tilted.
  `tests/test_physics.py` (3 pass). Detection does not yet handle piles.

- **2026-10-31 — `apps/calibrate_from_robot.py` (library: `legosort/robotcal.py`).**
  The outline calibration as a procedure for the twin or the real arm via
  `meca500_bridge.py`: starting focal length from one tilted view of the mat, 15
  poses planned for image coverage and safety (table and camera clearance with a
  gripper cylinder, every move checked in 5° steps and fed to the bridge in ≤8° steps),
  fit, restart from fx ±8 % if the median outline distance is > 0.6 px, saves
  intrinsics + overlay. Sim, webcam-like lens, starting from a 70° FOV guess, 3 pose
  sets: fx within 0.13 %, median 0.25–0.26 px first time, max error 0.31–0.39 mm at
  z=0 and 0.57–0.72 mm at z=200 (true lens 0.29 / 0.50). Leaving out the flange
  link (the real default, gripper not modelled) made no difference (0.31 / 0.59).
  Rehearsed end to end through `meca500_bridge.py --sim`. Viewer changes: `getState`
  now carries `bridge` (null or the bridge status) and echoes `_reqId`; the Python
  client drains broadcast state messages before each request (they had caused stale
  replies while a bridge was attached).
- **2026-10-31 — Lens calibration from the robot's outline, no sticker (sim,
  `experiments/robot_silhouette.py`, model in `legosort/robotmodel.py`).** FK is
  assumed exact (the user calibrates the joint offsets separately). The robot is
  segmented by median background (fixed camera, moving arm) plus Canny edges near
  that outline. The fit covers lens + camera pose (ICP-style, Huber), starting with
  the focal length 12 % off and no distortion. With the mat: fx within 0.12 %,
  principal point within 3 px, k1/k2 recovered; max point error 0.28–0.32 mm at z=0
  and 0.13–0.31 mm at z=200, the same as the true lens (0.29 / 0.50). Outline coverage
  is 42–50 % of the image (sticker 17–33 %). One 9-pose run fell into a wrong
  minimum, flagged by its median residual (1.0 px vs 0.25 px): **use about 15 poses
  and check the residual.** Outline only (no mat) is unreliable (1–2.6 mm on the
  table, principal point 40–60 px off). Real-world caveats: the gripper and cables are
  not in the model (drop the L5 outline or model the gripper); the arm must move
  enough for the median background; lighting must stay steady during the sequence.
- **2026-10-31 — Lens calibration by the robot (sim, `experiments/robot_intrinsics.py`).**
  An 80 mm ChArUco plate on the flange, posed by GNKinematics IK, a known webcam
  distortion applied to the images, and 3 random pose sets. Worst error of points
  located through the mat (true lens: 0.29 mm at z=0, 0.50 mm at z=200):
  9 poses 0.40–0.95 / 0.41–1.15 mm; 9 poses + mat view 0.34–0.35 / 0.36–1.32 mm;
  15 poses no better. The limit is image coverage (the plate reaches only 17–33%
  of the image grid). Ignoring distortion gives 0.57 / 1.68 mm. **Conclusion:** the
  robot can replace the hand-held board (about 1 mm or better); use the mat as an
  extra view, a larger plate, and poses spread across the image. Joint limits: plan
  within the intersection of the viewer, GNKinematics and bridge limits (the viewer
  clamps silently).
- **2026-10-31 — Brick detection (sim).** Found all bricks, < 1 mm, ~2°, correct stud
  counts, no false detections. Colour grouping is still fragile (glare on glossy black
  vs the small lime/yellow gap); 2 tests in `tests/test_detect.py` fail on this.
- **2026-10-31 — Simulation.** The VRRobotController twin renders the mat and robot
  for a virtual camera. Stage 1 runs on these images unchanged (camera pose
  0.1–0.9 mm, pins fixed across views, including with the arm hiding markers). 16
  tests pass (11 synthetic + 5 sim). **Next in sim:** bricks + detection (stage 3)
  and calibration against injected joint errors (stage 2).
- **2026-10-31 — Stage 1 code done.** Targets generated (`prints/`), tracker,
  intrinsics capture, camera setup and live view written; 11 synthetic tests pass
  (pose < 2 mm / 0.15°, robot-frame point invariant to camera motion < 1 mm).
  The C920 opens at 1920×1080 and its controls lock. Its auto-mode gain readings are
  unreliable, so exposure is locked by matching brightness. **Next:** print the
  targets and run the five steps in README.md on the real setup.
