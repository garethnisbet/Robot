"""Rigid-body physics for the bricks and the robot (MuJoCo): gravity and contact
between bricks, the mat, the arm and the gripper fingers.

Bricks are built from the same primitives the renderer draws (``Brick.parts``): a
hollow shell, studs and underside tubes, so studs can catch in an open underside and
a brick on its side rests on its wall. Mass comes from ABS density over that volume
(a 2x4 is about 2.7 g; a real one is about 2.3 g).

The arm (optional) is the viewer's own Meca500: the same joint chain and link meshes
as ``robotmodel.RobotModel``, each link colliding as its convex hull. It is driven
kinematically, exactly along the commanded joint path, as a stiff position-controlled
arm is: bricks cannot push it back. The gripper is the Mecademic MEGP 25E on its
STAP01 adapter, from the vendor CAD (``megp25e.py``): body, cap, connector and plate
as convex hulls, and the two jaws' gripping blocks on slide joints, each closed by a
force-limited position servo, so it squeezes a brick with a set force and the brick
is held by friction alone.

    sim = PileSim(bricks, robot=True)  # bricks: lego.Brick list; poses are ignored
    sim.pour(centre=(230, 0))          # drop them one by one onto a pile, wait for it to settle
    pile = sim.bricks()                # Brick list with full poses (R, x, y, z), robot frame mm
    sim.move_joints(q)                 # arm moves, pushing anything in its way
    sim.move_linear(p_tcp, R_grip)     # straight-line move of the grip frame (IK every mm)
    sim.grip(0.0)                      # close; sim.held() lists the bricks between the fingers
    sim.remove(3); sim.settle()        # take one away by hand and let the pile slump

Units: the robot frame in mm outside, metres inside MuJoCo. The mat top is z = 0.
Bricks not yet poured, or removed, are parked: moved far away, made non-colliding
and gravity-compensated, so they cost nothing and the model never needs rebuilding.
"""
import cv2
import mujoco
import numpy as np

from .lego import Brick
from .megp25e import MEGP25E

MM = 1e-3
ABS_DENSITY = 1050.0          # kg/m^3
PARK_X = 50.0                 # m, where parked bricks wait

# contype / conaffinity bits: bricks collide with everything; the arm and gripper
# collide with bricks only (not the mat, the fences or themselves).
BRICK_CT, BRICK_CA = 1, 3
ROBOT_CT, ROBOT_CA = 2, 0


def _quat_to_R(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def _R_to_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel())
    return q


def random_rotation(rng):
    """Uniformly random 3x3 rotation."""
    q = rng.normal(size=4)
    return _quat_to_R(q / np.linalg.norm(q))


def _v(t, scale=MM):
    return " ".join(f"{x * scale:.6g}" for x in t)


class PileSim:
    def __init__(self, bricks, friction=0.4, mat_friction=0.5, timestep=2.5e-4, hollow=True,
                 walls=None, robot=False, gripper=None, q0=None, sleep=False, restitution=None):
        """bricks: lego.Brick list (sizes and colours; poses are set by pour/place).

        friction: ABS on ABS (sliding); mat_friction: ABS on the paper mat.
        walls: optional (xmin, xmax, ymin, ymax) mm, invisible fences that keep the pile on the mat.
        robot: add the Meca500 with ``gripper`` (default megp25e.MEGP25E()), starting at joints q0.
        sleep: let bricks at rest sleep (MuJoCo skips them until something touches or moves
            them; the arm never sleeps). 3-5x faster with a resting pile; off by default so
            results match earlier runs.
        restitution: make bricks bounce, roughly this coefficient of restitution on the mat
            and on each other (ABS on a table is about 0.4-0.5). None (default): the
            original critically damped contacts, which hardly bounce. Only a brick moving
            faster than BOUNCE_SPEED (falling, tumbling) gets the bouncy contacts, kept for
            BOUNCE_HOLD after it slows so the rebound completes: lightly damped contacts
            never come to rest in a pile (the bricks jitter, creep and never sleep). Robot
            contacts stay critically damped, and only bricks touching the arm or gripper
            body are speed-capped (otherwise the cap would clip every rebound).
        """
        self.proto = [Brick(b.nx, b.ny, b.colour, height=b.height) for b in bricks]
        self.hollow = hollow
        self.gripper = (gripper or MEGP25E()) if robot else None
        self._robot_model = None
        if robot:
            from .robotmodel import RobotModel
            self._robot_model = RobotModel()
        self.model = mujoco.MjModel.from_xml_string(self._xml(friction, mat_friction, timestep, walls))
        if sleep:
            self.model.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_SLEEP
        self.restitution = restitution
        self.data = mujoco.MjData(self.model)
        m = self.model
        self._qadr = [m.jnt_qposadr[m.body_jntadr[m.body(f"b{i}").id]] for i in range(len(bricks))]
        self._vadr = [m.jnt_dofadr[m.body_jntadr[m.body(f"b{i}").id]] for i in range(len(bricks))]
        self._lin = np.array([[d, d + 1, d + 2] for d in self._vadr], int).reshape(-1, 3)
        self._ang = self._lin + 3
        self._geoms = [np.where(m.geom_bodyid == m.body(f"b{i}").id)[0] for i in range(len(bricks))]
        self._brick_body = {m.body(f"b{i}").id: i for i in range(len(bricks))}
        if restitution is not None:
            self._set_bounce(restitution)
        self.active = np.zeros(len(bricks), bool)
        for i in range(len(bricks)):
            self._park(i)
        self.time = 0.0
        self.on_frame = None          # callback(sim) at frame_rate while time advances
        self.frame_rate = 30.0
        self._next_frame = 0.0
        if robot:
            self._arm_q = [m.jnt_qposadr[m.joint(f"j{k}").id] for k in range(6)]
            self._arm_v = [m.jnt_dofadr[m.joint(f"j{k}").id] for k in range(6)]
            self._fingers = [m.body("finger_a").id, m.body("finger_b").id]
            # per geom: touches-the-fingers flag and brick index (-1: not a brick), for _cap
            self._geom_finger = np.isin(m.geom_bodyid, self._fingers)
            self._geom_brick = np.full(m.ngeom, -1)
            for i, g in enumerate(self._geoms):
                self._geom_brick[g] = i
            self._geom_robot = (m.geom_bodyid != 0) & (self._geom_brick < 0) & ~self._geom_finger
            self._finger_q = [m.jnt_qposadr[m.joint(n).id] for n in ("fa", "fb")]
            self.q = np.zeros(6) if q0 is None else np.asarray(q0, float)
            self._q_start = self.q.copy()
            self._qd = np.zeros(6)
            self.data.ctrl[:] = self.gripper.stroke / 2 * MM          # start open
            self.data.qpos[self._finger_q] = self.gripper.stroke / 2 * MM
            self._write_arm()
        mujoco.mj_forward(m, self.data)

    # ------------------------------------------------------------ model

    def _xml(self, friction, mat_friction, timestep, walls):
        # Stiff, small-scale contacts: the default 20 ms time constant lets a 1 mm wall sink
        # several mm, so use 2 ms (needs timestep <= 1 ms). No margin: a margin makes bricks hover.
        geom_defaults = (f'<geom contype="1" conaffinity="1" friction="{friction} 0.005 0.0001" '
                         f'solref="0.002 1" solimp="0.95 0.99 0.0005" margin="0" '
                         f'density="{ABS_DENSITY}" condim="4"/>')
        bodies = []
        bmask = f'contype="{BRICK_CT}" conaffinity="{BRICK_CA}"'
        for i, b in enumerate(self.proto):
            geoms = []
            for part in b.parts(hollow=self.hollow):
                if part[0] == "box":
                    _, c, h = part
                    geoms.append(f'<geom type="box" pos="{_v(c)}" size="{_v(h)}" {bmask}/>')
                else:
                    _, c, r, hh = part
                    geoms.append(f'<geom type="cylinder" pos="{_v(c)}" size="{r * MM:.6g} {hh * MM:.6g}" {bmask}/>')
            bodies.append(f'<body name="b{i}" pos="{PARK_X + i * 0.1} 0 1" gravcomp="1"><freejoint/>'
                          f'{"".join(geoms)}</body>')
        fence = ""
        if walls:
            x0, x1, y0, y1 = (w * MM for w in walls)
            cx, cy, hx, hy, t, hz = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2, 0.005, 0.05
            fence = "".join(f'<geom type="box" pos="{p}" size="{s}" rgba="0 0 0 0"/>' for p, s in [
                (f"{x0 - t} {cy} {hz}", f"{t} {hy + 2 * t} {hz}"), (f"{x1 + t} {cy} {hz}", f"{t} {hy + 2 * t} {hz}"),
                (f"{cx} {y0 - t} {hz}", f"{hx + 2 * t} {t} {hz}"), (f"{cx} {y1 + t} {hz}", f"{hx + 2 * t} {t} {hz}")])
        assets, robot, actuators = (self._robot_xml() if self.gripper else ("", "", ""))
        return f"""
<mujoco model="lego_pile">
  <option timestep="{timestep}" gravity="0 0 -9.81" cone="elliptic" impratio="10" integrator="implicitfast"/>
  <default>{geom_defaults}</default>
  <asset>{assets}</asset>
  <worldbody>
    <geom name="mat" type="plane" size="0 0 0.01" friction="{mat_friction} 0.005 0.0001"/>
    {fence}
    {robot}
    {"".join(bodies)}
  </worldbody>
  <actuator>{actuators}</actuator>
</mujoco>"""

    def _robot_xml(self):
        """The arm from the viewer's config (three.js frame, metres), plus the gripper."""
        import trimesh
        rm = self._robot_model
        assets, hull = [], {}
        for j, (v, f) in rm.parts.items():
            h = trimesh.Trimesh(v, f, process=False).convex_hull.vertices
            if len(h) > 400:
                h = h[np.linspace(0, len(h) - 1, 400).astype(int)]
            name = f"link{j + 1}"
            assets.append(f'<mesh name="{name}" vertex="{" ".join(f"{x:.6g}" for x in h.ravel())}"/>')
            hull[j] = name
        rgeom = f'contype="{ROBOT_CT}" conaffinity="{ROBOT_CA}"'
        inertial = '<inertial pos="0 0 0" mass="0.5" diaginertia="1e-3 1e-3 1e-3"/>'
        g_assets, g_xml = self._gripper_xml(rm.cfg.get("eeOffset", [0, 0, 0]))
        assets += g_assets
        children = {}
        for k, d in enumerate(rm.joints):
            children.setdefault(d.get("parent", k - 1), []).append(k)

        def body(k):
            d = rm.joints[k]
            axis = np.asarray(d["axis"], float) * rm.sign[k]
            geom = f'<geom type="mesh" mesh="{hull[k]}" {rgeom}/>' if k in hull else ""
            inner = "".join(body(c) for c in children.get(k, []))
            if k == rm.n - 1:
                inner += g_xml
            return (f'<body name="link{k}" pos="{_v(d["restPos"], 1)}" quat="{_v(d["restQuat"], 1)}" '
                    f'gravcomp="1">{inertial}<joint name="j{k}" type="hinge" axis="{_v(axis, 1)}" '
                    f'limited="false"/>{geom}{inner}</body>')

        base = f'<geom type="mesh" mesh="{hull[-1]}" {rgeom}/>' if -1 in hull else ""
        # three.js world -> robot frame: +90 deg about x
        robot = (f'<body name="robot" quat="0.70710678 0.70710678 0 0">{base}'
                 + "".join(body(c) for c in children.get(-1, [])) + "</body>")
        g = self.gripper
        act = "".join(f'<position name="{n}" joint="{n}" kp="{g.force / (g.stroke / 2 * MM):.6g}" '
                      f'forcelimited="true" forcerange="{-g.force} {g.force}" ctrlrange="0 {g.stroke / 2 * MM}"/>'
                      for n in ("fa", "fb"))
        return "".join(assets), robot, act

    def _gripper_xml(self, ee_offset):
        """Gripper in the flange link: the tool frame is the link frame turned 180 deg about x
        (GNKinematics tool x = link x, tool z = -link z), at the flange face.
        Returns (mesh assets, body xml)."""
        g = self.gripper
        rgeom = f'contype="{ROBOT_CT}" conaffinity="{ROBOT_CA}"'

        def mesh(name, v):
            return f'<mesh name="{name}" vertex="{_v(np.ravel(v))}"/>'
        assets, geoms = [], []
        for k, v in enumerate(g.collision_static()):
            assets.append(mesh(f"grip{k}", v))
            geoms.append(f'<geom type="mesh" mesh="grip{k}" {rgeom}/>')
        jaws = []
        for side, name, sign in (("a", "fa", 1), ("b", "fb", -1)):
            pieces = g.collision_jaw_parts(side)
            jgeoms = ""
            for k, v in enumerate(pieces):
                assets.append(mesh(f"jaw_{side}{k}", v))
                jgeoms += (f'<geom type="mesh" mesh="jaw_{side}{k}" friction="{g.friction} 0.005 0.0001" '
                           f'{rgeom}/>')
            ax = sign * np.asarray(g.axis)
            # the whole moving jaw (slider inside the body + block) weighs ~20 g; with only the
            # ~1 g block the squeeze on a 2.7 g brick chatters
            c = pieces[0].mean(axis=0)
            jaws.append(f'<body name="finger_{side}" gravcomp="1">'
                        f'<inertial pos="{_v(c)}" mass="0.02" diaginertia="2e-6 2e-6 2e-6"/>'
                        f'<joint name="{name}" type="slide" axis="{_v(ax, 1)}" range="0 {g.stroke / 2 * MM}" '
                        f'damping="60"/>{jgeoms}</body>')
        xml = (f'<body name="gripper" pos="{_v(ee_offset, 1)}" quat="0 1 0 0" gravcomp="1">'
               f'<inertial pos="0 0 0.03" mass="0.15" diaginertia="1e-4 1e-4 1e-4"/>'
               + "".join(geoms) + "".join(jaws) + '</body>')
        return assets, xml

    # ------------------------------------------------------------ state

    def _set_pose(self, i, R, p_mm, v=(0, 0, 0), w=(0, 0, 0)):
        a, d = self._qadr[i], self._vadr[i]
        self.data.qpos[a:a + 3] = np.asarray(p_mm, float) * MM
        self.data.qpos[a + 3:a + 7] = _R_to_quat(R)
        self.data.qvel[d:d + 3] = v
        self.data.qvel[d + 3:d + 6] = w

    def _park(self, i):
        self.active[i] = False
        self._collide(i, 0, 0)
        self.model.body_gravcomp[self.model.body(f"b{i}").id] = 1.0
        self._set_pose(i, np.eye(3), (PARK_X / MM + i * 100.0, 0, 1000.0))

    def _activate(self, i):
        self.active[i] = True
        self._collide(i, BRICK_CT, BRICK_CA)
        self.model.body_gravcomp[self.model.body(f"b{i}").id] = 0.0

    def _collide(self, i, ct, ca):
        # the broadphase also filters on per-body masks (the OR over the body's geoms)
        b = self.model.body(f"b{i}").id
        self.model.geom_contype[self._geoms[i]] = ct
        self.model.geom_conaffinity[self._geoms[i]] = ca
        self.model.body_contype[b] = ct
        self.model.body_conaffinity[b] = ca

    def snapshot(self):
        """The full state (arrays), to put back with ``restore`` into this sim or a fresh one
        built from the same bricks and gripper."""
        return {"qpos": self.data.qpos.copy(), "qvel": self.data.qvel.copy(), "ctrl": self.data.ctrl.copy(),
                "active": self.active.copy(), "time": np.array([self.data.time, self.time]),
                "q": np.array(getattr(self, "q", np.zeros(6)), float)}

    def restore(self, s):
        for i, a in enumerate(s["active"]):
            if a and not self.active[i]:
                self._activate(i)
            elif not a and self.active[i]:
                self._park(i)
        self.data.qpos[:], self.data.qvel[:], self.data.ctrl[:] = s["qpos"], s["qvel"], s["ctrl"]
        self.data.time, self.time = (float(t) for t in s["time"])
        if self.gripper is not None:
            self.q = np.asarray(s["q"], float).copy()
            self._qd = np.zeros(6)
        mujoco.mj_forward(self.model, self.data)

    def pose(self, i):
        """(R, origin mm) of brick i's frame (origin at its bottom centre)."""
        a = self._qadr[i]
        return _quat_to_R(self.data.qpos[a + 3:a + 7]), self.data.qpos[a:a + 3] / MM

    def body_pose(self, name):
        """(R, origin mm) of a named body (e.g. 'gripper', 'finger_a', 'link3') in the robot frame."""
        b = self.model.body(name).id
        return self.data.xmat[b].reshape(3, 3).copy(), self.data.xpos[b] / MM

    def speed(self):
        """Fastest linear (mm/s) and angular (deg/s) speed over the active bricks."""
        if not self.active.any():
            return 0.0, 0.0
        v = np.array([self.data.qvel[d:d + 6] for d, a in zip(self._vadr, self.active) if a])
        return (float(np.linalg.norm(v[:, :3], axis=1).max() / MM),
                float(np.degrees(np.linalg.norm(v[:, 3:], axis=1).max())))

    def bricks(self, include_inactive=False):
        """Bricks with their current poses (robot frame, mm; z = bottom-centre height above the mat)."""
        out = []
        for i, b in enumerate(self.proto):
            if not (self.active[i] or include_inactive):
                continue
            R, p = self.pose(i)
            yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
            out.append(Brick(b.nx, b.ny, b.colour, float(p[0]), float(p[1]), yaw, b.height, float(p[2]), R))
        return out

    def indices(self):
        """Simulator index of each brick returned by ``bricks()`` (active ones, in order)."""
        return np.flatnonzero(self.active)

    # ------------------------------------------------------------ time

    # Contact time constant x damping ratio fixes MuJoCo's contact stiffness; the default
    # (2 ms, 1) is kept for the stiffness, so a bouncier contact gets a longer time constant
    # (lowering the damping ratio alone makes the contact stiffer and the 0.5 ms step unstable).
    # Measured: damping 0.25 / 8 ms gives e ~0.49 for a 2x4 dropped flat on the mat.
    _CONTACT_STIFFNESS_TC = 0.002

    BOUNCE_SPEED = 0.05       # m/s: a brick faster than this gets the bouncy contacts ...
    BOUNCE_HOLD = 0.04        # s: ... and keeps them this long after it slows (an impact lasts ~15 ms)

    def _set_bounce(self, e):
        m = self.model
        L = -np.log(np.clip(e, 0.01, 0.9))
        zeta = L / np.sqrt(np.pi ** 2 + L ** 2)          # damped-oscillator restitution
        self._bouncy_ref = (self._CONTACT_STIFFNESS_TC / zeta, zeta)
        self._calm_ref = tuple(m.geom_solref[self._geoms[0][0]])
        self._bouncy = np.zeros(len(self.proto), bool)
        self._bouncy_until = np.zeros(len(self.proto))
        # contacts mix the two geoms' solref weighted by solmix: robot contacts take the
        # robot's (stiff, damped) one; a bouncing brick's wins against the mat or a resting brick
        robot = (m.geom_bodyid != 0) & ~np.isin(np.arange(m.ngeom), np.concatenate(self._geoms))
        m.geom_solmix[robot] = 1e3

    def _bounce(self):
        if self.restitution is None:
            return
        v = self.data.qvel[self._lin]
        fast = np.einsum("ij,ij->i", v, v) > self.BOUNCE_SPEED ** 2
        self._bouncy_until[fast] = self.time + self.BOUNCE_HOLD
        want = self._bouncy_until > self.time
        for i in np.where(want != self._bouncy)[0]:
            g = self._geoms[i]
            self.model.geom_solref[g] = self._bouncy_ref if want[i] else self._calm_ref
            self.model.geom_solmix[g] = 10.0 if want[i] else 1.0
        self._bouncy = want

    def _write_arm(self):
        if self.gripper is None:
            return
        self.data.qpos[self._arm_q] = np.radians(self.q)
        self.data.qvel[self._arm_v] = np.radians(self._qd)

    # The arm is driven kinematically (unlimited force), so a brick pinched between it and
    # the mat or another brick can be shot out at metres per second, which the real arm
    # (it would stall) never does. Cap each brick's speed across the mat and upwards (falling
    # is left alone) and its spin, except for bricks touching the fingers (a held brick travels
    # with the arm, faster than this in a joint move).
    max_brick_speed = 0.4         # m/s, horizontal (the arm pushes at most 0.15 m/s)
    max_brick_spin = 60.0         # rad/s

    def _cap(self):
        q = self.data.qvel
        robot_only = self.restitution is not None
        free = np.full(len(self._lin), not robot_only)
        if self.gripper is not None and self.data.ncon:
            c = self.data.contact[:self.data.ncon]
            g1, g2 = c.geom1, c.geom2
            if robot_only:          # cap only bricks the arm or gripper body is pushing
                r = self._geom_robot[g1] | self._geom_robot[g2]
                if r.any():
                    k = np.r_[self._geom_brick[g1[r]], self._geom_brick[g2[r]]]
                    free[k[k >= 0]] = True
            f = self._geom_finger[g1] | self._geom_finger[g2]
            if f.any():
                k = np.r_[self._geom_brick[g1[f]], self._geom_brick[g2[f]]]
                free[k[k >= 0]] = False
        if not free.any():
            return
        for idx, cap in ((self._lin[:, :2], self.max_brick_speed), (self._ang, self.max_brick_spin)):
            v = q[idx]
            n = np.linalg.norm(v, axis=1)
            fast = (n > cap) & free
            if fast.any():
                q[idx[fast]] = v[fast] * (cap / n[fast])[:, None]
        up = self._lin[:, 2]
        fast = (q[up] > self.max_brick_speed) & free          # thrown upwards (falling is fine)
        if fast.any():
            q[up[fast]] = self.max_brick_speed

    def _tick(self):
        if self.on_frame is not None and self.time >= self._next_frame:
            self._next_frame = self.time + 1.0 / self.frame_rate
            self.on_frame(self)

    def step(self, seconds):
        """Advance time with the arm held where it is."""
        n = max(1, int(round(seconds / self.model.opt.timestep)))
        dt = self.model.opt.timestep
        if self.gripper is None and self.on_frame is None and self.restitution is None:
            mujoco.mj_step(self.model, self.data, nstep=n)
            self.time += n * dt
            return
        if self.gripper is not None:
            self._qd[:] = 0
        for _ in range(n):
            self._write_arm()
            mujoco.mj_step(self.model, self.data)
            self._cap()
            self._bounce()
            self.time += dt
            self._tick()

    def settle(self, max_seconds=5.0, v_tol=2.0, w_tol=5.0, check=0.05):
        """Run until every active brick is slower than v_tol mm/s and w_tol deg/s. Returns the sim time."""
        t = 0.0
        while t < max_seconds:
            self.step(check)
            t += check
            v, w = self.speed()
            if v < v_tol and w < w_tol:
                break
        return t

    # ------------------------------------------------------------ bricks

    def place(self, i, R, p_mm):
        """Put brick i at a pose (R, bottom-centre mm), at rest, and make it part of the scene."""
        self._activate(i)
        self._set_pose(i, R, p_mm)
        mujoco.mj_forward(self.model, self.data)

    def remove(self, i):
        """Take brick i out of the scene (by hand: it simply disappears)."""
        self._park(i)
        mujoco.mj_forward(self.model, self.data)

    def drop(self, i, xy_mm, height_mm, rng, spin=3.0):
        """Release brick i at random orientation, its centre height_mm above the mat."""
        R = random_rotation(rng)
        b = self.proto[i]
        p = np.array([xy_mm[0], xy_mm[1], height_mm]) - R @ [0, 0, b.height / 2]
        self._activate(i)
        self._set_pose(i, R, p, w=rng.normal(scale=spin, size=3))

    def top(self, xy_mm, radius_mm):
        """Highest point (mm) of any active brick whose centre is within radius of xy."""
        z = 0.0
        for i in np.flatnonzero(self.active):
            R, p = self.pose(i)
            b = self.proto[i]
            c = p + R @ [0, 0, b.height / 2]
            if np.hypot(c[0] - xy_mm[0], c[1] - xy_mm[1]) < radius_mm:
                z = max(z, c[2] + np.hypot(b.length, b.width) / 2)
        return z

    def pour(self, centre=(230.0, 0.0), spread=25.0, drop_above=30.0, interval=0.12, rng=None,
             settle=True):
        """Drop the bricks one by one at random orientations onto a pile at ``centre`` (mm).

        Each brick starts drop_above mm over the current top of the pile, within ``spread``
        mm (1 sigma) of the centre; the next follows ``interval`` s later, so earlier ones
        are still tumbling, much like tipping them out of a cup. Short falls also keep
        each step's travel well under the 1 mm wall thickness.
        """
        rng = rng or np.random.default_rng()
        for i in range(len(self.proto)):
            if self.active[i]:
                continue
            xy = np.asarray(centre, float) + rng.normal(scale=spread, size=2)
            z = self.top(xy, 40.0) + drop_above
            self.drop(i, xy, z, rng)
            self.step(interval)
        return self.settle() if settle else 0.0

    # ------------------------------------------------------------ robot

    def tcp_pose(self):
        """(R_grip, p_tcp mm): the grip frame (z along the jaws, x the closing direction)
        and its origin midway between the jaw tips, robot frame."""
        R, p = self.body_pose("gripper")             # the tool (flange) frame
        T = self.gripper.T_tool_grip
        return R @ T[:3, :3], p + R @ T[:3, 3]

    def clearance(self, q, first_link=2):
        """Lowest point (mm, robot frame) of the arm from link ``first_link`` out and the gripper
        body (not the jaws, which are meant to go low), at joints q (deg). Below 0 means the
        arm would hit the mat."""
        m = self.model
        if not hasattr(self, "_kin_data"):
            self._kin_data = mujoco.MjData(m)
            bodies = [f"link{k}" for k in range(first_link, 6)] + ["gripper"]
            ids = {m.body(b).id for b in bodies}
            self._kin_geoms = [g for g in range(m.ngeom)
                               if m.geom_bodyid[g] in ids and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
        d = self._kin_data
        d.qpos[:] = self.data.qpos
        d.qpos[self._arm_q] = np.radians(q)
        mujoco.mj_kinematics(m, d)
        low = np.inf
        for g in self._kin_geoms:
            k = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
            z = d.geom_xpos[g][2] + v @ d.geom_xmat[g].reshape(3, 3)[2]
            low = min(low, z.min())
        return float(low / MM)

    def opening(self):
        """Current finger opening, mm (0 = closed, stroke = fully open)."""
        return float(sum(self.data.qpos[self._finger_q]) / MM)

    def held(self):
        """Bricks touching both fingers (index list)."""
        touch = [set(), set()]
        c = self.data.contact[:self.data.ncon]
        for g1, g2 in zip(c.geom1, c.geom2):
            b1, b2 = self.model.geom_bodyid[g1], self.model.geom_bodyid[g2]
            for k, f in enumerate(self._fingers):
                if b1 == f and b2 in self._brick_body:
                    touch[k].add(self._brick_body[b2])
                elif b2 == f and b1 in self._brick_body:
                    touch[k].add(self._brick_body[b1])
        return sorted(touch[0] & touch[1])

    def set_joints(self, q):
        """Put the arm at q (deg) at once (use only where it can't hit anything)."""
        self.q = np.asarray(q, float).copy()
        self._qd[:] = 0
        self._write_arm()
        mujoco.mj_forward(self.model, self.data)

    def follow(self, qs, ts):
        """Drive the arm along joint waypoints qs (deg) at times ts (s, from 0), linearly
        between them, stepping the physics; anything in the way is pushed."""
        qs, ts = np.asarray(qs, float), np.asarray(ts, float)
        dt = self.model.opt.timestep
        n = int(np.ceil(ts[-1] / dt))
        for k in range(1, n + 1):
            t = min(k * dt, ts[-1])
            s = min(np.searchsorted(ts, t, side="right") - 1, len(ts) - 2)
            a = (t - ts[s]) / max(ts[s + 1] - ts[s], 1e-12)
            self.q = qs[s] + a * (qs[s + 1] - qs[s])
            self._qd = (qs[s + 1] - qs[s]) / max(ts[s + 1] - ts[s], 1e-12)
            self._write_arm()
            mujoco.mj_step(self.model, self.data)
            self._cap()
            self._bounce()
            self.time += dt
            self._tick()
        self.q = qs[-1].copy()
        self._qd[:] = 0

    def move_joints(self, q, speed=60.0):
        """Joint move with a smooth (cosine) speed profile; speed = peak joint speed, deg/s."""
        q0, q1 = self.q.copy(), np.asarray(q, float)
        span = np.abs(q1 - q0).max()
        if span < 1e-6:
            return
        T = np.pi / 2 * span / speed
        s = np.linspace(0, 1, max(8, int(T * 200)))
        prof = 0.5 - 0.5 * np.cos(np.pi * s)
        self.follow(q0 + prof[:, None] * (q1 - q0), s * T)

    def move_linear(self, p_tcp, R_grip=None, speed=40.0, step_mm=1.0):
        """Straight-line move of the grip frame to p_tcp (mm), keeping the orientation (or
        turning to R_grip), at ``speed`` mm/s with smooth ends. Raises if IK fails on the way."""
        R0, p0 = self.tcp_pose()
        R1 = R0 if R_grip is None else np.asarray(R_grip, float)
        p1 = np.asarray(p_tcp, float)
        dist = np.linalg.norm(p1 - p0)
        rvec = cv2.Rodrigues(R0.T @ R1)[0].ravel()
        n = max(2, int(np.ceil(max(dist, np.degrees(np.linalg.norm(rvec))) / step_mm)) + 1)
        s = np.linspace(0, 1, n)
        prof = 0.5 - 0.5 * np.cos(np.pi * s)
        qs, q = [], self.q
        for a in prof:
            R = R0 @ cv2.Rodrigues(rvec * a)[0]
            q = self.ik(p0 + a * (p1 - p0), R, q)
            qs.append(q)
        T = np.pi / 2 * max(dist, 1e-3) / speed
        self.follow(qs, s * T)

    def ik(self, p_tcp, R_grip, seed=None):
        """Joint angles (deg) putting the grip frame at R_grip with its origin at p_tcp (mm)."""
        from .robotcal import _kin
        k = _kin()
        T = self.gripper.T_tool_grip
        R_tool = np.asarray(R_grip, float) @ T[:3, :3].T
        flange = np.asarray(p_tcp, float) - R_tool @ T[:3, 3]
        target = np.array([flange, R_tool[:, 0], R_tool[:, 1], R_tool[:, 2]])
        # The solver starts from a seed and can fail from one that is far from the answer
        # (a plan made from one arm pose, then run from another): retry from the start pose.
        for s in (self.q if seed is None else seed, self._q_start):
            k.storeCurrentPosition(np.asarray(s, float))
            q = k.i_kinematics(target)
            if not np.any(np.isnan(q)):
                return np.asarray(q, float)
        raise ValueError(f"no IK solution for TCP at {np.round(p_tcp, 1)}")

    def grip(self, opening_mm, max_seconds=1.0):
        """Command the finger opening (mm, 0 = closed) and wait until the fingers stop."""
        self.data.ctrl[:] = np.clip(opening_mm, 0, self.gripper.stroke) / 2 * MM
        t = 0.0
        while t < max_seconds:
            self.step(0.02)
            t += 0.02
            if t > 0.06 and np.abs(self.data.qvel[[self.model.jnt_dofadr[self.model.joint(n).id]
                                                   for n in ("fa", "fb")]]).max() < 0.5 * MM:
                break
        return self.opening()
