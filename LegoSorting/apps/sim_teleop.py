"""Drive the physics-simulated arm from the viewer and play with a pile of LEGO.

    python3 apps/sim_teleop.py --url ws://127.0.0.1:8099/ws        # your viewer
    python3 apps/sim_teleop.py --url ws://127.0.0.1:8099/ws --n 20 --fingers

This closes the loop between the viewer and the physics (legosort/physics.py). It
attaches to the viewer as a robot bridge, like ``meca500_bridge.py --sim``, but the
"robot" is the MuJoCo arm with the MEGP 25E gripper:

* the viewer's joints (sliders, IK gizmo, VR, macros) are the command; the physics arm
  follows them at the Meca500's joint speeds (times the panel's speed scale) and
  pushes whatever it meets;
* the viewer's gripper panel (Open / Close / opening slider) and the VR trigger squeeze
  the physics fingers; "holding part" lights up when a brick touches both jaws;
* the bricks and the gripper are drawn where the physics has them. The viewer's own
  arm shows the command, so after a big jump (Home, a typed angle) the gripper trails
  the drawn arm until the physics arm catches up;
* Disable / E-stop freeze the arm (the physics keeps running), Enable resumes, and
  Reset pours a fresh pile.

The arm is kinematic (it never stalls), so it can drive straight through the mat or a
heap; bricks it pinches are slowed down rather than shot away (PileSim's speed cap).
Ctrl+C quits and removes the bricks and gripper from the viewer.

It refuses to start if another bridge (possibly the real arm) is attached to the viewer:
that bridge would follow the same joints.
"""
import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from legosort.boards import BaseMat  # noqa: E402
from legosort.megp25e import MEGP25E  # noqa: E402
from legosort.physics import MM, PileSim  # noqa: E402
from legosort.pick import HOME  # noqa: E402
from legosort.sim import SIM_PREFIX, PhysicsView, Viewer, build_scene, to_api  # noqa: E402
from sim_pile import random_bricks, summary  # noqa: E402

# meca500_bridge.py's limits, so the sim moves like the real arm would
JOINT_LIMITS = np.array([(-175, 175), (-70, 90), (-135, 70), (-170, 170), (-115, 115), (-36000, 36000)], float)   # J6 is multi-turn
MAX_JOINT_VEL = np.array([150, 150, 180, 300, 300, 500], float)        # deg/s at 100 %


class PhysicsBridge:
    def __init__(self, v: Viewer, args):
        self.v, self.args = v, args
        self.rng = np.random.default_rng(args.seed)
        self.enabled, self.paused = True, False
        self.vel_scale = args.vel_scale
        self.grip_force, self.grip_vel = 50, 50
        self.target = np.array(HOME, float)
        self.sim = self.view = None
        self._repour = False
        v.on_message = self.on_message

    # ------------------------------------------------------------ scene

    def pour(self):
        a = self.args
        if self.view is not None:
            self.v.remove_objects(SIM_PREFIX + "brick_")
        q0 = self.sim.q.copy() if self.sim is not None else np.array(HOME, float)
        t = time.time()
        # the pile is poured with the arm parked at HOME, out of the way
        sim = PileSim(random_bricks(a.n, self.rng), robot=True, q0=HOME, timestep=a.timestep,
                      gripper=MEGP25E(fingers=a.fingers), sleep=True,
                      restitution=a.bounce or None)
        sim.pour(centre=a.centre, spread=a.spread, rng=self.rng)
        if sim.clearance(q0) > 0:
            sim.set_joints(q0)
        self.sim = sim
        self.view = PhysicsView(self.v, sim, drive_arm=False)
        print(f"poured in {time.time() - t:.1f} s: {summary(sim.bricks())}")

    # ------------------------------------------------------------ viewer -> bridge

    def on_message(self, d):
        if d.get("type") != "bridgeCommand":
            return
        cmd = d.get("cmd")
        g = self.sim.gripper if self.sim else None
        if cmd in ("enable", "resume"):
            self.enabled, self.paused = True, False
            print("enabled")
        elif cmd == "disable":
            self.enabled = False
            print("disabled: the arm holds still")
        elif cmd == "estop":
            self.paused = True
            print("E-STOP: the arm holds still (Resume to carry on)")
        elif cmd == "reset":
            self._repour = True
        elif cmd == "setVelScale":
            self.vel_scale = float(np.clip(d.get("scale", 0.25), 0.01, 1.0))
            print(f"speed {self.vel_scale * 100:.0f} %")
        elif g is None:
            return
        elif cmd == "gripperOpen":
            self.set_opening(g.stroke)
        elif cmd == "gripperClose":
            self.set_opening(0.0)
        elif cmd == "setGripper":
            if "mm" in d:
                self.set_opening(float(d["mm"]))
            else:
                self.set_opening(float(np.clip(d.get("opening", 1.0), 0, 1)) * g.stroke)
        elif cmd == "setGripperForce":
            self.grip_force = int(np.clip(d.get("force", 50), 5, 100))
            print("gripper force is fixed in the sim (MEGP25E.force); ignoring the slider")
        elif cmd == "setGripperVel":
            self.grip_vel = int(np.clip(d.get("vel", 50), 5, 100))

    def set_opening(self, mm):
        g = self.sim.gripper
        self.sim.data.ctrl[:] = float(np.clip(mm, 0, g.stroke)) / 2 * MM

    # ------------------------------------------------------------ bridge -> viewer

    def status(self):
        sim = self.sim
        g = sim.gripper
        at = bool(np.abs(self.target - sim.q).max() < 0.5)
        return {
            "cmd": "bridgeStatus", "robotConnected": True, "robotConnecting": False, "robotHomed": True,
            "enabled": self.enabled, "paused": self.paused, "collisionStopped": False,
            "realJoints": [round(float(j), 2) for j in sim.q], "velScale": round(self.vel_scale, 2),
            "atTarget": at, "sim": True, "robotIp": "physics",
            "gripper": {"present": True, "tool": "MEGP 25E (physics)", "homed": True, "error": False,
                        "pos": round(sim.opening(), 2), "min": 0.0, "max": round(g.stroke, 2),
                        "force": self.grip_force, "vel": self.grip_vel, "holding": bool(sim.held())},
        }

    # ------------------------------------------------------------ loop

    def run(self):
        a = self.args
        period = 1.0 / a.rate
        frames, t_report, behind = 0, time.monotonic() + 5.0, 0.0
        next_status = 0.0
        while True:
            t0 = time.monotonic()
            if self._repour:
                self._repour = False
                print("reset: pouring a new pile")
                self.pour()
            try:
                st = self.v.request({"cmd": "getState"}, "state", timeout=5.0, strict=True)
            except TimeoutError:
                print("viewer not answering (tab closed or reloaded?); waiting")
                continue
            if st.get("joints") and len(st["joints"]) == 6:
                self.target = np.clip(np.asarray(st["joints"], float), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
            sim = self.sim
            if self.enabled and not self.paused:
                lim = MAX_JOINT_VEL * self.vel_scale * period
                q1 = sim.q + np.clip(self.target - sim.q, -lim, lim)
                sim.follow([sim.q.copy(), q1], [0.0, period])
            else:
                sim.step(period)
            self.view.update()
            if t0 >= next_status:
                next_status = t0 + 0.1
                self.v.send(self.status())

            frames += 1
            spent = time.monotonic() - t0
            behind += max(0.0, spent - period)
            if spent < period:
                time.sleep(period - spent)
            if time.monotonic() > t_report:
                if behind > 0.1 * frames * period:          # below ~90 % of real time
                    print(f"physics is running at about {frames * period / (frames * period + behind):.0%} "
                          f"of real time (fewer bricks, --timestep or a lower --rate would help)")
                frames, behind, t_report = 0, 0.0, time.monotonic() + 30.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="ws://127.0.0.1:8099/ws", help="viewer WebSocket URL")
    ap.add_argument("--session", help="viewer session id (default: the first one)")
    ap.add_argument("--n", type=int, default=12, help="bricks in the pile")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--centre", type=float, nargs=2, default=(165.0, 0.0), metavar=("X", "Y"))
    ap.add_argument("--spread", type=float, default=15.0, help="mm: how widely the pile is poured")
    ap.add_argument("--fingers", action="store_true", help="printed stepped fingers on the jaws")
    ap.add_argument("--rate", type=float, default=60.0, help="Hz: viewer updates per second")
    ap.add_argument("--timestep", type=float, default=5e-4,
                    help="s: physics step (sim_pick uses 2.5e-4; 5e-4 is twice as fast and stable at rest)")
    ap.add_argument("--bounce", type=float, default=0.5,
                    help="coefficient of restitution of the bricks (0: the old dead contacts)")
    ap.add_argument("--vel-scale", type=float, default=1.0, help="joint speed as a fraction of the Meca500's")
    args = ap.parse_args()

    session = args.session
    if not session:
        http = args.url.replace("ws://", "http://").replace("wss://", "https://").rsplit("/ws", 1)[0]
        sessions = json.loads(urllib.request.urlopen(http + "/sessions", timeout=5).read())
        if not sessions:
            raise SystemExit(f"no viewer tab is connected to {http}")
        session = sessions[0]["id"]
    v = Viewer(args.url, session)
    other = v.state().get("bridge")
    if other:
        raise SystemExit(f"another robot bridge is attached to this viewer ({other}); "
                         "stop it first, or it would follow the same joints")
    b = PhysicsBridge(v, args)
    build_scene(v, BaseMat.load())
    v.set_joints(HOME)
    cx, cy = args.centre
    v.request({"cmd": "setCamera", "position": to_api([cx + 330, cy - 330, 330]),
               "target": to_api([cx - 40, cy, 40])}, "camera")
    b.pour()
    print("ready: move the arm in the viewer (sliders, IK gizmo or VR); the gripper panel works the jaws; "
          "Reset pours a new pile; Ctrl+C quits")
    try:
        b.run()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        try:
            v.on_message = None
            v.remove_objects(SIM_PREFIX + "brick_")
            v.remove_objects(SIM_PREFIX + "grip_")
            v.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
