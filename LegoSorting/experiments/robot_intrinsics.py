"""Can the robot replace the hand-held board for lens calibration?

The twin carries a small ChArUco plate on its flange to N poses in front of a fixed
virtual camera. A known lens distortion is applied to every captured image (the
viewer renders an ideal pinhole), then the intrinsics are calibrated from those
views alone and compared with the truth: directly (K, distortion) and by what
matters downstream, the robot-frame position of table points located through the
base mat.

    python3 experiments/robot_intrinsics.py [--url ws://127.0.0.1:8099/ws --session ID]
"""
import argparse
import json
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "apps"))
from legosort import synth  # noqa: E402
from legosort.boards import BaseMat, CharucoSpec  # noqa: E402
from legosort.geometry import make_T, project, transform_points  # noqa: E402
from legosort.intrinsics import calibrate, detect_charuco, view_diversity  # noqa: E402
from legosort.sim import SIM_PREFIX, MeshBuilder, Viewer, build_scene  # noqa: E402
from legosort.simserver import VIEWER_DIR, HeadlessViewer  # noqa: E402
from legosort.tracker import BaseTracker  # noqa: E402
from sim_view import SAFE_POSE, camera_at  # noqa: E402

sys.path.insert(0, str(VIEWER_DIR))
from RobotDefinitions import Meca500_kin as KIN  # noqa: E402

TRUE_DIST = np.array([0.08, -0.15, 0.0005, -0.0004, 0.0])   # typical webcam barrel
TARGET = CharucoSpec(5, 5, 16.0, 12.0, "DICT_4X4_50")       # 80 mm plate
STANDOFF = 20.0                                              # plate face from flange, mm


# ------------------------------------------------------------------ kinematics

def fk(q):
    f = KIN.f_kinematics(np.asarray(q, float))
    return np.column_stack([f[5], f[6], f[7]]), f[8]          # tool axes (columns), flange point


def joint_limits(margin=2.0):
    """Intersection of the GNKinematics and viewer limits (the viewer clamps silently)."""
    lo, hi = np.array(KIN.motor_limits, float).T
    cfg = json.loads((VIEWER_DIR / "meca500_config.json").read_text())
    vl = np.array([j["limits"] for j in cfg["joints"]], float)
    return np.maximum(lo, vl[:, 0]) + margin, np.minimum(hi, vl[:, 1]) - margin


LIMITS = joint_limits()


def ik(R_goal, p_goal, seeds):
    lo, hi = LIMITS

    def res(q):
        R, p = fk(q)
        dR = cv2.Rodrigues(R_goal.T @ R)[0].ravel()
        return np.concatenate([p - p_goal, 100.0 * dR])

    best = None
    for s in seeds:
        r = least_squares(res, np.clip(s, lo, hi), bounds=(lo, hi), xtol=1e-12, ftol=1e-12)
        if best is None or r.cost < best.cost:
            best = r
        if np.sqrt(2 * best.cost) < 1e-3:
            break
    return best.x if best is not None and np.sqrt(2 * best.cost) < 1e-3 else None


# ------------------------------------------------------------------ target

def target_points_local():
    """Inner ChArUco corners in the flange frame (for planning only)."""
    w = TARGET.squares_x * TARGET.square_mm
    g = (np.arange(1, TARGET.squares_x) * TARGET.square_mm) - w / 2
    xx, yy = np.meshgrid(g, g)
    return np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, STANDOFF)])


def target_glb(R, p):
    """The plate as a vertex-coloured mesh at flange pose (R, p): face along tool +z."""
    cell = 2.0                                       # 16 mm squares, 12 mm markers -> 2 mm grid
    n = int(TARGET.squares_x * TARGET.square_mm / cell)
    img = TARGET.board().generateImage((n, n), marginSize=0, borderBits=1)
    T = make_T(R, p)
    w = n * cell / 2
    mb = MeshBuilder()
    m = 3.0
    back = transform_points(T, [[-w - m, -w - m, STANDOFF - 0.1], [w + m, -w - m, STANDOFF - 0.1],
                                [w + m, w + m, STANDOFF - 0.1], [-w - m, w + m, STANDOFF - 0.1]])
    mb.quad(back, (1.0, 1.0, 1.0))
    mb.box(T @ make_T(t=[0, 0, STANDOFF / 2 - 0.2]), (12, 12, STANDOFF - 0.4), (0.3, 0.3, 0.3))
    for r in range(n):
        c = 0
        while c < n:                                 # merge runs of black cells
            if img[r, c] > 127:
                c += 1
                continue
            c0 = c
            while c < n and img[r, c] <= 127:
                c += 1
            x0, x1 = -w + c0 * cell, -w + c * cell
            y1, y0 = w - r * cell, w - (r + 1) * cell
            mb.quad(transform_points(T, [[x0, y0, STANDOFF], [x1, y0, STANDOFF],
                                         [x1, y1, STANDOFF], [x0, y1, STANDOFF]]), (0.02, 0.02, 0.02))
    return mb.glb()


# ------------------------------------------------------------------ pose planning

def plan_poses(cam, n, rng, tries=4000):
    """Flange poses that show the plate to the camera, tilted in varied directions."""
    K = cam.K()
    T_cam_robot = cam.T_cam_robot
    pts_local = target_points_local()
    cands = []
    seeds = [np.array(SAFE_POSE, float), np.array([0, 0, 0, 0, 45, 0.0]), np.array([-30, 10, 10, 0, 60, 0.0])]
    for _ in range(tries):
        p = rng.uniform([170, -150, 110], [320, 90, 280])
        to_cam = np.asarray(cam.eye) - p
        to_cam /= np.linalg.norm(to_cam)
        tilt = rng.uniform(15, 40)
        axis = np.cross(to_cam, rng.normal(size=3))
        axis /= np.linalg.norm(axis)
        z = cv2.Rodrigues(axis * np.radians(tilt))[0] @ to_cam
        x = np.cross(rng.normal(size=3), z)
        x /= np.linalg.norm(x)
        R = np.column_stack([x, np.cross(z, x), z])
        q = ik(R, p, seeds)
        if q is None:
            continue
        pix = project(transform_points(make_T(R, p), pts_local), K, np.zeros(5), T_cam_robot)
        if (pix.min(axis=0) < 40).any() or pix[:, 0].max() > cam.width - 40 or pix[:, 1].max() > cam.height - 40:
            continue
        cands.append((q, R, p, z))
        if len(cands) >= 6 * n:
            break
    # greedy: spread the plate normals (and positions) as much as possible
    chosen = [cands[0]]
    while len(chosen) < n and len(chosen) < len(cands):
        def score(c):
            return min(np.degrees(np.arccos(np.clip(c[3] @ o[3], -1, 1))) + 0.05 * np.linalg.norm(c[2] - o[2])
                       for o in chosen)
        chosen.append(max((c for c in cands if not any(c is o for o in chosen)), key=score))
    return chosen


# ------------------------------------------------------------------ evaluation

HEIGHTS = (0.0, 10.0, 100.0, 200.0)


def table_error(K_est, d_est, img_mat, mat, cam):
    """Max robot-frame error of points located with the estimated intrinsics.

    Points on horizontal planes at HEIGHTS above the table (table, brick tops,
    gripper heights), seen by the real camera, back-projected with the estimate.
    Returns (max error per height..., camera position error).
    """
    tr = BaseTracker(mat, K_est, d_est)
    pose = tr.estimate(img_mat)
    if pose is None:
        return tuple([np.inf] * (len(HEIGHTS) + 1))
    xs, ys = np.meshgrid(np.linspace(145, 315, 7), np.linspace(-145, 145, 9))
    out = []
    for z in HEIGHTS:
        pts = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)])
        pix = project(pts, cam.K(), TRUE_DIST, cam.T_cam_robot)
        got = tr.pixel_to_robot(pose, pix, z=z)
        out.append(float(np.linalg.norm(got - pts, axis=1).max()))
    return tuple(out) + (float(np.linalg.norm(pose.camera_position - cam.eye)),)


def run(v, args):
    for seed in args.seed:
        print(f"\n=== pose set {seed} ===")
        run_one(v, args, seed)


def run_one(v, args, seed):
    rng = np.random.default_rng(seed)
    mat = BaseMat.load()
    cam = camera_at(-90, 50, 620)
    cam.width, cam.height = 1920, 1080
    K_true = cam.K()
    v.set_joints(SAFE_POSE)
    build_scene(v, mat)
    v.remove_objects(SIM_PREFIX + "brick_")
    v.remove_objects(SIM_PREFIX + "det_")
    v.remove_objects(SIM_PREFIX + "target")

    img_mat, _ = v.capture(cam, exposure=0.15)
    img_mat = synth.distort(img_mat, K_true, TRUE_DIST)

    poses = plan_poses(cam, args.max_poses, rng)
    print(f"planned {len(poses)} reachable poses showing the plate")
    views, shots = [], []
    for i, (q, R, p, z) in enumerate(poses):
        v.set_joints(q)
        v.remove_objects(SIM_PREFIX + "target")
        v.add_glb(SIM_PREFIX + "target", target_glb(R, p), {"opacity": 1.0, "roughness": 0.9, "metalness": 0.0})
        img, _ = v.capture(cam, exposure=0.15)
        img = synth.distort(img, K_true, TRUE_DIST)
        det = detect_charuco(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), TARGET)
        n = 0 if det is None else len(det[0])
        print(f"  pose {i + 1:2d}: joints {np.round(q, 1)}  corners {n}")
        if det is not None and n >= 8:
            views.append(det)
        shots.append(img)
    v.remove_objects(SIM_PREFIX + "target")
    v.set_joints(SAFE_POSE)
    if args.save:
        out = ROOT / "data" / "robot_intrinsics"
        out.mkdir(parents=True, exist_ok=True)
        for i, im in enumerate(shots):
            cv2.imwrite(str(out / f"pose_{i + 1:02d}.png"), im)

    # the mat as one more planar view
    tr = BaseTracker(mat, K_true, np.zeros(5))
    corners, ids = tr.detect(img_mat)
    mo, mi = tr.board.matchImagePoints(corners, ids.reshape(-1, 1))
    mat_view = (mo.reshape(-1, 3).astype(np.float32), mi.reshape(-1, 2).astype(np.float32))

    size = (cam.width, cam.height)
    rows = []
    base = table_error(K_true, TRUE_DIST, img_mat, mat, cam)
    rows.append(("true intrinsics (reference)", None, 0, base))
    rows.append(("ignore distortion (true K)", None, 0, table_error(K_true, np.zeros(5), img_mat, mat, cam)))
    for label, vs in [("9 robot poses", views[:9]), ("15 robot poses", views[:15]),
                      ("9 robot poses + mat", views[:9] + [mat_view]),
                      ("15 robot poses + mat", views[:15] + [mat_view])]:
        if len(vs) < 4:
            continue
        intr, kept = calibrate(vs, size)
        rows.append((label, intr, view_diversity(size, vs), table_error(intr.K, intr.dist, img_mat, mat, cam)))

    print(f"\ntrue: fx {K_true[0, 0]:.1f}  cx {K_true[0, 2]:.1f}  cy {K_true[1, 2]:.1f}  dist {TRUE_DIST[:2]}")
    hdr = "  ".join(f"z={int(z)}" for z in HEIGHTS)
    print(f"{'method':28s} {'RMS':>5s} {'fx%':>6s} {'cx,cy px':>11s} {'cover':>5s} | max error mm: {hdr} | cam pos")
    for label, intr, cover, errs in rows:
        e = "  ".join(f"{x:5.2f}" for x in errs[:-1])
        if intr is None:
            print(f"{label:28s} {'':>5s} {'':>6s} {'':>11s} {'':>5s} | {e} | {errs[-1]:6.2f}")
            continue
        K = intr.K
        print(f"{label:28s} {intr.rms:5.2f} {100 * (K[0, 0] / K_true[0, 0] - 1):6.2f} "
              f"{K[0, 2] - K_true[0, 2]:5.1f},{K[1, 2] - K_true[1, 2]:5.1f} {100 * cover:4.0f}% | {e} | {errs[-1]:6.2f}")
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", help="viewer WebSocket URL (default: headless)")
    ap.add_argument("--session")
    ap.add_argument("--seed", type=int, nargs="+", default=[0])
    ap.add_argument("--max-poses", type=int, default=15)
    ap.add_argument("--save", action="store_true", help="save the pose images to data/robot_intrinsics/")
    args = ap.parse_args()
    if args.url:
        session = args.session
        if not session:
            http = args.url.replace("ws://", "http://").rsplit("/ws", 1)[0]
            session = json.loads(urllib.request.urlopen(http + "/sessions").read())[0]["id"]
        v = Viewer(args.url, session)
        try:
            run(v, args)
        finally:
            v.close()
    else:
        with HeadlessViewer() as hv:
            v = Viewer(hv.ws_url, hv.session)
            try:
                run(v, args)
            finally:
                v.close()


if __name__ == "__main__":
    main()
