"""Calibrate a camera's lens and pose from the robot arm's outline: no target on the arm.

The arm moves through ~15 planned joint poses (MoveJoints only) while the camera
watches it over the base mat. The lens (fx, fy, cx, cy, k1, k2) and the camera pose
are fitted so the arm's predicted outline (forward kinematics, taken as exact) lies on
the observed outline in every image and the mat markers reproject where they were seen.
A fit whose median outline distance stays high is restarted (see legosort.robotcal).

  In the twin, with a virtual camera (headless, or a running viewer with --url):
    python3 apps/calibrate_from_robot.py --sim [--distort] [--view -90 50 620]
    python3 apps/calibrate_from_robot.py --sim --url ws://127.0.0.1:8099/ws --session ID

  A real webcam, with the real arm driven through the viewer by meca500_bridge.py:
    python3 apps/calibrate_from_robot.py --camera c920 --real --url ws://127.0.0.1:8099/ws --session ID

  Rehearse the real run with the virtual camera, the moves going through the bridge
  (start meca500_bridge.py --sim for no real arm):
    python3 apps/calibrate_from_robot.py --sim --real --url ws://127.0.0.1:8099/ws --session ID

  Refit saved captures without moving anything:
    python3 apps/calibrate_from_robot.py --replay data/robotcal_c920

Writes data/intrinsics_<name>.json (used by the tracker like a ChArUco calibration),
and data/robotcal_<name>/ with the captures, result.json and overlay.png (fitted outline
in green on the images; orange is the part of the arm left out of the fit).

Real arm safety: the app refuses to run if a bridge is reporting unless --real is
given, and with --real it shows the plan and waits for "yes". Every pose and the path
to it (in <=5 degree steps) keeps the arm and a gripper cylinder --clearance mm above
the table and --camera-clear mm from the camera; the arm is fed small steps so it
follows the checked path. Ctrl+C holds the arm where it is.
"""
import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from legosort import robotcal, synth  # noqa: E402
from legosort.boards import BaseMat  # noqa: E402
from legosort.camera import DATA  # noqa: E402
from legosort.intrinsics import Intrinsics  # noqa: E402
from legosort.robotmodel import RobotModel  # noqa: E402
from legosort.sim import SIM_PREFIX, Viewer, build_scene  # noqa: E402
from legosort.tracker import BaseTracker  # noqa: E402
from sim_view import SAFE_POSE, camera_at  # noqa: E402

TEST_DIST = np.array([0.08, -0.15, 0.0005, -0.0004, 0.0])   # webcam-like lens for --distort
STEP_DEG = 8.0          # real arm: largest joint change per step sent to the bridge
FLANGE = 5              # joint index of the flange link


# ------------------------------------------------------------------ arm

class Arm:
    """Moves the twin, and through it the real arm when a bridge is attached."""

    def __init__(self, viewer, real, settle):
        self.v, self.real, self.settle = viewer, real, settle

    def bridge(self):
        return self.v.state().get("bridge")

    def joints(self):
        if self.real:
            return np.array(self._check(self.bridge())["realJoints"], float)
        return np.array(self.v.state()["joints"], float)

    @staticmethod
    def _check(b):
        if not b:
            raise RuntimeError("the bridge stopped reporting")
        if b["collisionStopped"] or b["paused"] or not b["enabled"] or not b["connected"]:
            raise RuntimeError(f"the bridge is not driving the arm: {b}")
        return b

    def move(self, q):
        q = np.asarray(q, float)
        if not self.real:
            self.v.set_joints(q)
            return
        cur = self.joints()
        n = max(1, int(np.ceil(np.abs(q - cur).max() / STEP_DEG)))
        for k in range(1, n + 1):
            w = cur + (q - cur) * k / n
            self.v.set_joints(w)
            self._wait(w, 0.3 if k == n else 3.0)
        time.sleep(self.settle)

    def _wait(self, w, tol, timeout=60.0):
        t = time.time()
        while time.time() - t < timeout:
            b = self._check(self.bridge())
            if np.abs(np.array(b["realJoints"]) - w).max() < tol:
                return
            time.sleep(0.05)
        raise TimeoutError(f"the arm did not reach {np.round(w, 1)}")

    def hold(self):
        """Stop where the arm is now (the bridge tracks the viewer's joints)."""
        try:
            b = self.bridge()
            if b:
                self.v.set_joints(b["realJoints"], settle=False)
        except Exception as e:
            print(f"could not hold the arm: {e}")


# ------------------------------------------------------------------ cameras

class SimCam:
    def __init__(self, viewer, cam, distort):
        self.v, self.cam, self.distort = viewer, cam, distort

    def grab(self):
        img, _ = self.v.capture(self.cam, exposure=0.15)
        return synth.distort(img, self.cam.K(), TEST_DIST) if self.distort else img


class RealCam:
    def __init__(self, name, frames):
        from legosort.camera import Camera, CameraSettings
        self.settings = CameraSettings.load(name)
        self.cam = Camera(self.settings)
        self.frames = frames

    def grab(self):
        for _ in range(3):                    # flush frames buffered during the move
            self.cam.read()
        stack = [self.cam.read() for _ in range(self.frames)]
        return np.median(np.stack(stack), axis=0).astype(np.uint8)

    def close(self):
        drift = self.cam.check_locked()
        if drift:
            print(f"warning: camera controls drifted during the run: {drift}")
        self.cam.close()


# ------------------------------------------------------------------ steps

def connect(args):
    session = args.session
    if not session:
        http = args.url.replace("ws://", "http://").rsplit("/ws", 1)[0]
        session = json.loads(urllib.request.urlopen(http + "/sessions").read())[0]["id"]
    return Viewer(args.url, session)


def check_bridge(v, args):
    st = v.state()
    if "bridge" not in st:
        sys.exit("The viewer page does not report bridge status (old js/websocket.js). Reload the "
                 "viewer page so the app can tell whether a real arm is attached.")
    b = st["bridge"]
    if b and not args.real:
        sys.exit("A real-robot bridge is attached to this viewer session, so these moves would drive "
                 "the arm. Pass --real to do that on purpose, or use another session.")
    if args.real and not b:
        sys.exit("--real given, but no bridge is reporting on this viewer session.")
    if args.real:
        Arm._check(b)
        print(f"bridge: connected, enabled{' (bridge in --sim mode)' if b['sim'] else ''}; "
              f"arm at {np.round(b['realJoints'], 1)}")


def capture(v, args, rm, mat):
    """Plan, move, capture. Returns (poses, images, mat image, extra meta)."""
    arm = Arm(v, args.real, args.settle)
    if args.sim:
        cam = camera_at(*args.view, fov=args.fov)
        cam.width, cam.height = 1920, 1080
        source = SimCam(v, cam, args.distort)
        v.set_joints(SAFE_POSE)
        build_scene(v, mat)
        for prefix in ("brick_", "det_", "target"):
            v.remove_objects(SIM_PREFIX + prefix)
        truth = {"K": cam.K().tolist(), "dist": (TEST_DIST if args.distort else np.zeros(5)).tolist(),
                 "eye": list(cam.eye), "T_cam_robot": cam.T_cam_robot.tolist()}
    else:
        source = RealCam(args.camera, args.frames)
        truth = None

    try:
        start = arm.joints()
        safety = robotcal.Safety(clearance=args.clearance, camera_clear=args.camera_clear,
                                 tool_length=args.tool_length, tool_radius=args.tool_radius)
        if safety.why_unsafe(rm, start) and args.real:
            sys.exit(f"the arm's current pose is already unsafe ({safety.why_unsafe(rm, start)}); "
                     "move it somewhere clear first")
        home = np.array(SAFE_POSE, float)
        if args.real and not safety.path_ok(rm, start, home):
            sys.exit(f"no safe straight move from {np.round(start, 1)} to the start pose {SAFE_POSE}")
        arm.move(home)
        img_mat = source.grab()
        size = (img_mat.shape[1], img_mat.shape[0])

        K0, d0 = guess_intrinsics(args, size)
        robot_pts, board_pts, px = robotcal.mat_observations(mat, img_mat)
        f0 = robotcal.focal_from_mat(board_pts, px, size, K0[0, 0]) if not args.intrinsics else K0[0, 0]
        K0[0, 0] = K0[1, 1] = f0
        pose = BaseTracker(mat, K0, d0).estimate(img_mat)
        safety.camera = pose.camera_position
        print(f"camera seen at {np.round(pose.camera_position)} mm (robot frame), "
              f"{pose.n_markers} mat markers, starting fx {f0:.0f} px")

        rng = np.random.default_rng(args.seed)
        poses, cover = robotcal.plan_poses(rm, K0, d0, pose.T_cam_robot, size, args.poses, rng, safety,
                                           j1_span=args.j1_span, start=home)
        print(f"planned {len(poses)} poses; the outline reaches {100 * cover:.0f}% of an 8x6 image grid")
        if len(poses) < 9:
            sys.exit("too few safe poses show the arm to this camera; move the camera back or "
                     "relax --clearance / --camera-clear")
        if args.real:
            print(f"\nThe REAL arm will move through {len(poses)} poses, starting and ending at {SAFE_POSE}:")
            for q in poses:
                print("   ", np.round(q, 1))
            print(f"clearance {args.clearance:.0f} mm above the table, {args.camera_clear:.0f} mm from the camera, "
                  f"gripper cylinder {args.tool_length:.0f} x {2 * args.tool_radius:.0f} mm. Ctrl+C holds the arm.")
            if input('Type "yes" to start: ').strip().lower() != "yes":
                sys.exit("cancelled")

        images = []
        for i, q in enumerate(poses):
            arm.move(q)
            images.append(source.grab())
            print(f"  pose {i + 1:2d}/{len(poses)}: {np.round(q, 1)}", flush=True)
        arm.move(home)
    except KeyboardInterrupt:
        if args.real:
            arm.hold()
        sys.exit("\ninterrupted: arm held where it was")
    except Exception:
        if args.real:
            arm.hold()
        raise
    finally:
        if isinstance(source, RealCam):
            source.close()

    meta = {"truth": truth, "camera": args.camera, "f_guess": f0,
            "camera_settings": None if args.sim else source.settings.__dict__}
    return [list(map(float, q)) for q in poses], images, img_mat, meta


def guess_intrinsics(args, size):
    if args.intrinsics:
        intr = Intrinsics.load(args.intrinsics)
        return intr.K.copy(), intr.dist.ravel().copy()
    f = (size[0] / 2) / np.tan(np.radians(args.fov_guess) / 2)
    return np.array([[f, 0, size[0] / 2 - 0.5], [0, f, size[1] / 2 - 0.5], [0, 0, 1.0]]), np.zeros(5)


def save_captures(out, poses, images, img_mat, meta):
    out.mkdir(parents=True, exist_ok=True)
    for i, img in enumerate(images):
        cv2.imwrite(str(out / f"pose_{i:02d}.png"), img)
    cv2.imwrite(str(out / "mat.png"), img_mat)
    (out / "captures.json").write_text(json.dumps({"poses": poses, **meta}, indent=2, default=str))


def load_captures(out):
    meta = json.loads((out / "captures.json").read_text())
    images = [cv2.imread(str(out / f"pose_{i:02d}.png")) for i in range(len(meta["poses"]))]
    return meta.pop("poses"), images, cv2.imread(str(out / "mat.png")), meta


def contact_sheet(images, rm, poses, f, max_joint, n=6, width=640):
    idx = np.linspace(0, len(images) - 1, min(n, len(images))).round().astype(int)
    tiles = []
    for i in idx:
        o = robotcal.overlay(images[i], rm, poses[i], f, max_joint)
        tiles.append(cv2.resize(o, (width, int(width * o.shape[0] / o.shape[1])), interpolation=cv2.INTER_AREA))
    while len(tiles) % 3:
        tiles.append(np.zeros_like(tiles[0]))
    return np.vstack([np.hstack(tiles[r:r + 3]) for r in range(0, len(tiles), 3)])


def report_truth(f, truth, img_mat, mat):
    """Sim only: compare with the virtual camera, and locate grid points at four heights."""
    Kt, dt, Tt = np.array(truth["K"]), np.array(truth["dist"]), np.array(truth["T_cam_robot"])
    print(f"\nagainst the virtual camera: fx {100 * (f.K[0, 0] / Kt[0, 0] - 1):+.2f}%  "
          f"cx {f.K[0, 2] - Kt[0, 2]:+.1f} px  cy {f.K[1, 2] - Kt[1, 2]:+.1f} px  "
          f"k1 {f.dist[0]:.3f} (true {dt[0]:.3f})  k2 {f.dist[1]:.3f} (true {dt[1]:.3f})")
    for label, K, d in (("fitted lens", f.K, f.dist), ("true lens", Kt, dt)):
        tr = BaseTracker(mat, K, d)
        pose = tr.estimate(img_mat)
        xs, ys = np.meshgrid(np.linspace(145, 315, 7), np.linspace(-145, 145, 9))
        errs = []
        for z in (0.0, 10.0, 100.0, 200.0):
            pts = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)])
            from legosort.geometry import project
            got = tr.pixel_to_robot(pose, project(pts, Kt, dt, Tt), z=z)
            errs.append(np.linalg.norm(got - pts, axis=1).max())
        print(f"  {label:12s} max error locating points, mm:  "
              + "  ".join(f"z={int(z)}: {e:.2f}" for z, e in zip((0, 10, 100, 200), errs)))


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--sim", action="store_true", help="virtual camera in the twin")
    src.add_argument("--camera", help="real webcam settings name (data/camera_<name>.json)")
    src.add_argument("--replay", type=Path, help="refit the captures saved in this folder")
    ap.add_argument("--url", help="viewer WebSocket URL (sim default: start a headless viewer)")
    ap.add_argument("--session", help="viewer session id (default: the first one)")
    ap.add_argument("--real", action="store_true", help="drive the real arm through the bridge")
    ap.add_argument("--name", help="output name (default: the camera name, or 'sim')")
    ap.add_argument("--poses", type=int, default=15)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--intrinsics", help="start from these intrinsics instead of --fov-guess")
    ap.add_argument("--fov-guess", type=float, default=70.0, help="horizontal field of view guess, deg")
    ap.add_argument("--clearance", type=float, help="mm above the table (default 15 sim, 40 real)")
    ap.add_argument("--camera-clear", type=float, default=150.0, help="mm from the camera")
    ap.add_argument("--tool-length", type=float, help="gripper cylinder length, mm (default 0 sim, 90 real)")
    ap.add_argument("--tool-radius", type=float, default=25.0)
    ap.add_argument("--j1-span", type=float, default=60.0, help="J1 within +- this of facing the mat")
    ap.add_argument("--fit-flange", dest="fit_flange", action="store_true", default=None,
                    help="use the flange link's outline (default: only in the sim; a real gripper hides it)")
    ap.add_argument("--no-fit-flange", dest="fit_flange", action="store_false")
    ap.add_argument("--residual-limit", type=float, default=0.6, help="median outline px for a good fit")
    ap.add_argument("--settle", type=float, default=1.0, help="real arm: seconds to settle before a picture")
    ap.add_argument("--frames", type=int, default=3, help="real camera: frames per pose (median)")
    ap.add_argument("--view", type=float, nargs=3, default=(-90, 50, 620), metavar=("AZ", "EL", "DIST"))
    ap.add_argument("--fov", type=float, default=50.0, help="sim camera vertical field of view, deg")
    ap.add_argument("--distort", action="store_true", help="sim: give the virtual camera a webcam-like lens")
    args = ap.parse_args()
    if (args.camera or args.real) and not args.url:
        ap.error("--camera and --real need --url (the viewer session the bridge follows)")
    real_like = bool(args.camera)
    args.clearance = args.clearance if args.clearance is not None else (40.0 if real_like else 15.0)
    args.tool_length = args.tool_length if args.tool_length is not None else (90.0 if real_like else 0.0)
    if args.fit_flange is None:
        args.fit_flange = not real_like

    rm, mat = RobotModel(), BaseMat.load()
    if args.replay:
        out = args.replay
        poses, images, img_mat, meta = load_captures(out)
        name = args.name or out.name.removeprefix("robotcal_")
    else:
        name = args.name or args.camera or "sim"
        out = DATA / f"robotcal_{name}"
        if args.url:
            v = connect(args)
            try:
                check_bridge(v, args)
                poses, images, img_mat, meta = capture(v, args, rm, mat)
            finally:
                v.close()
        else:
            from legosort.simserver import HeadlessViewer
            with HeadlessViewer() as hv:
                v = Viewer(hv.ws_url, hv.session)
                try:
                    check_bridge(v, args)
                    poses, images, img_mat, meta = capture(v, args, rm, mat)
                finally:
                    v.close()
        save_captures(out, poses, images, img_mat, meta)
        print(f"captures saved to {out}")

    max_joint = None if args.fit_flange else FLANGE - 1
    print(f"\nfitting {len(poses)} outlines + the mat"
          f"{'' if args.fit_flange else ' (flange link left out: gripper not in the model)'}")
    t = time.time()
    f, fits = robotcal.calibrate(rm, poses, images, img_mat, mat, meta["f_guess"], max_joint,
                                 args.residual_limit)
    good = f.median_px <= args.residual_limit
    print(f"\nfit in {time.time() - t:.0f} s: median outline distance {f.median_px:.2f} px, "
          f"mat corners {f.mat_rms_px:.2f} px RMS, {f.n_points} outline points"
          + ("" if good else f"  ** above {args.residual_limit} px: check overlay.png before using it **"))
    print(f"  fx {f.K[0, 0]:.1f}  fy {f.K[1, 1]:.1f}  cx {f.K[0, 2]:.1f}  cy {f.K[1, 2]:.1f}  "
          f"k1 {f.dist[0]:.4f}  k2 {f.dist[1]:.4f}")
    print(f"  camera at {np.round(f.camera_position, 1)} mm (robot frame)")
    if meta.get("truth"):
        report_truth(f, meta["truth"], img_mat, mat)

    size = (images[0].shape[1], images[0].shape[0])
    intr = Intrinsics(f.K, f.dist, size, rms=f.median_px, n_views=len(poses),
                      camera=meta.get("camera_settings") or {},
                      board={"method": "robot outline + base mat", "fit_flange": args.fit_flange,
                             "median_outline_px": f.median_px, "mat_rms_px": f.mat_rms_px, "good": good},
                      created=time.strftime("%Y-%m-%d %H:%M:%S"))
    p = intr.save(name)
    (out / "result.json").write_text(json.dumps({
        "K": f.K.tolist(), "dist": f.dist.tolist(), "T_cam_robot": f.T_cam_robot.tolist(),
        "camera_position": f.camera_position.tolist(), "median_outline_px": f.median_px,
        "mat_rms_px": f.mat_rms_px, "good": good,
        "fits": [{"start_fx": x.start_fx, "median_px": x.median_px, "log": x.log} for x in fits]}, indent=2))
    cv2.imwrite(str(out / "overlay.png"), contact_sheet(images, rm, poses, f, max_joint))
    print(f"\nwrote {p}\n      {out / 'result.json'}\n      {out / 'overlay.png'}")
    if not good:
        sys.exit(1)


if __name__ == "__main__":
    main()
