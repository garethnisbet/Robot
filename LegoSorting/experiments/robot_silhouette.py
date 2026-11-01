"""Lens calibration from the robot's own outline (no sticker), in the simulation.

The arm moves to N joint poses in front of a fixed virtual camera. Forward
kinematics (assumed exact: the link models are accurate and the joint offsets are
calibrated separately) places the robot's meshes; the camera intrinsics and pose
are fitted so the predicted outline lies on the observed outline in every image,
together with the base mat's marker corners.

Observed outline, using only what a real setup has:
  background = per-pixel median over all images (the camera is fixed, the arm moves),
  foreground = pixels that differ from it, and the edges kept are Canny edges near
  the foreground boundary (drops the mat's printing, soft shadows and the arm's
  interior detail).

A known webcam distortion is applied to the rendered images; the fit starts from a
focal length 12 % off and zero distortion.

    python3 experiments/robot_silhouette.py [--seed 0 1 2] [--url ws://127.0.0.1:8099/ws --session ID]
"""
import argparse
import json
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "apps"))
sys.path.insert(0, str(ROOT / "experiments"))
from legosort import synth  # noqa: E402
from legosort.boards import BaseMat  # noqa: E402
from legosort.geometry import project, rvec_tvec_to_T, T_to_rvec_tvec  # noqa: E402
from legosort.robotmodel import RobotModel  # noqa: E402
from legosort.sim import SIM_PREFIX, Viewer, build_scene  # noqa: E402
from legosort.simserver import HeadlessViewer  # noqa: E402
from legosort.tracker import BaseTracker  # noqa: E402
from robot_intrinsics import HEIGHTS, LIMITS, TRUE_DIST, table_error  # noqa: E402
from sim_view import SAFE_POSE, camera_at  # noqa: E402

SIZE = (1920, 1080)
BORDER = 6          # outline within this many px of the image edge is the frame, not the robot


# ------------------------------------------------------------------ poses

def plan_poses(rm, cam, n, rng, tries=400):
    """Random joint poses with the arm clear of the table and spread over the image."""
    lo, hi = LIMITS
    K = cam.K()
    cands = []
    for _ in range(tries):
        q = rng.uniform(lo, hi)
        q[0] = rng.uniform(-60, 60)                       # stay on the camera's side
        v, _ = rm.vertices(q, min_joint=1)
        if v[:, 2].min() < 15:                            # moving links clear of the table / mat
            continue
        m = rm.mask(q, K, np.zeros(5), cam.T_cam_robot, SIZE)
        edge = cv2.morphologyEx(m, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
        edge[:BORDER], edge[-BORDER:], edge[:, :BORDER], edge[:, -BORDER:] = False, False, False, False
        ys, xs = np.nonzero(edge)
        if len(xs) < 500:
            continue
        cells = set(zip((xs * 8 // SIZE[0]).tolist(), (ys * 6 // SIZE[1]).tolist()))
        cands.append((q, cells))
        if len(cands) >= 5 * n:
            break
    chosen, covered = [], set()
    while len(chosen) < n and cands:
        best = max(cands, key=lambda c: len(c[1] - covered) + 0.01 * rng.random())
        chosen.append(best[0])
        covered |= best[1]
        cands = [c for c in cands if c is not best]
    return chosen, len(covered) / 48.0


# ------------------------------------------------------------------ observation

def observed_edges(images):
    """Distance-to-edge maps of each image's robot outline (see module docstring)."""
    stack = np.stack(images)
    bg = np.median(stack, axis=0).astype(np.int16)
    out = []
    for img in images:
        diff = np.abs(img.astype(np.int16) - bg).max(axis=2)
        fg = (diff > 30).astype(np.uint8) * 255
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        band = cv2.dilate(cv2.morphologyEx(fg, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)),
                          np.ones((9, 9), np.uint8))
        edges = cv2.Canny(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 40, 120)
        edges = cv2.bitwise_and(edges, band)
        dt = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 5)
        out.append(dt)
    return out


# ------------------------------------------------------------------ fitting

def unpack(x):
    fx, fy, cx, cy, k1, k2 = x[:6]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    dist = np.array([k1, k2, 0, 0, 0])
    T = rvec_tvec_to_T(x[6:9], x[9:12])
    return K, dist, T


def silhouette_points(rm, q, K, dist, T, per_image=500, rng=None):
    """3D robot-frame points on the predicted outer outline for joint pose q."""
    m = rm.mask(q, K, dist, T, SIZE)
    cnt, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    line = np.zeros_like(m)
    cv2.drawContours(line, cnt, -1, 255, 1)
    dt = cv2.distanceTransform(255 - line, cv2.DIST_L2, 5)
    v, _ = rm.vertices(q)
    pix = project(v, K, dist, T)
    ok = ((pix[:, 0] >= BORDER) & (pix[:, 0] < SIZE[0] - BORDER)
          & (pix[:, 1] >= BORDER) & (pix[:, 1] < SIZE[1] - BORDER))
    v, pix = v[ok], pix[ok]
    d = map_coordinates(dt, [pix[:, 1], pix[:, 0]], order=1)
    v = v[d < 0.7]
    if rng is not None and len(v) > per_image:
        v = v[rng.choice(len(v), per_image, replace=False)]
    return v


def calibrate_from_robot(rm, poses, images, mat_obs, x0, use_mat=True, iters=8, log=print):
    dts = observed_edges(images)
    rng = np.random.default_rng(0)
    x = x0.copy()
    gate = 25.0
    for it in range(iters):
        K, dist, T = unpack(x)
        pts, maps = [], []
        for q, dt in zip(poses, dts):
            p3 = silhouette_points(rm, q, K, dist, T, rng=rng)
            d = map_coordinates(dt, [project(p3, K, dist, T)[:, 1], project(p3, K, dist, T)[:, 0]], order=1)
            pts.append(p3[d < gate])                      # ICP-style outlier gate
            maps.append(dt)
        n_pts = sum(len(p) for p in pts)

        def residuals(x):
            K, dist, T = unpack(x)
            r = []
            for p3, dt in zip(pts, maps):
                pix = project(p3, K, dist, T)
                r.append(map_coordinates(dt, [pix[:, 1], pix[:, 0]], order=1, mode="nearest"))
            if use_mat:
                obj, img = mat_obs
                r.append(3.0 * (project(obj, K, dist, T) - img).ravel())   # corners: sharper
            return np.concatenate(r)

        sol = least_squares(residuals, x, loss="huber", f_scale=1.5, x_scale="jac", max_nfev=60)
        step = np.abs(sol.x - x)
        x = sol.x
        K, dist, _ = unpack(x)
        log(f"    iter {it + 1}: {n_pts} outline points, gate {gate:4.1f} px, "
            f"median |r| {np.median(np.abs(sol.fun)):.2f} px, fx {K[0, 0]:.1f} cx {K[0, 2]:.1f} "
            f"cy {K[1, 2]:.1f} k1 {dist[0]:.3f} k2 {dist[1]:.3f}")
        gate = max(4.0, gate * 0.5)
        if it > 2 and step[:4].max() < 0.05:
            break
    return unpack(x)


# ------------------------------------------------------------------ experiment

def run_one(v, rm, seed, n_list, log=print):
    rng = np.random.default_rng(seed)
    mat = BaseMat.load()
    cam = camera_at(-90, 50, 620)
    cam.width, cam.height = SIZE
    K_true = cam.K()
    poses, cover = plan_poses(rm, cam, max(n_list), rng)
    log(f"planned {len(poses)} poses, outline covers {100 * cover:.0f}% of an 8x6 image grid")

    v.set_joints(SAFE_POSE)
    build_scene(v, mat)
    for prefix in ("brick_", "det_", "target"):
        v.remove_objects(SIM_PREFIX + prefix)
    images = []
    for q in poses:
        v.set_joints(q)
        img, _ = v.capture(cam, exposure=0.15)
        images.append(synth.distort(img, K_true, TRUE_DIST))
    v.set_joints(SAFE_POSE)
    img_mat, _ = v.capture(cam, exposure=0.15)
    img_mat = synth.distort(img_mat, K_true, TRUE_DIST)

    # starting point: focal length 12 % off, centred principal point, no distortion
    K0 = np.array([[1.12 * K_true[0, 0], 0, SIZE[0] / 2], [0, 1.12 * K_true[1, 1], SIZE[1] / 2], [0, 0, 1.0]])
    tr0 = BaseTracker(mat, K0, np.zeros(5))
    corners, ids = tr0.detect(img_mat)
    obj, img = tr0.board.matchImagePoints(corners, ids.reshape(-1, 1))
    obj = (obj.reshape(-1, 3) @ mat.T_robot_board[:3, :3].T + mat.T_robot_board[:3, 3])   # robot frame
    mat_obs = (obj, img.reshape(-1, 2).astype(float))
    pose0 = tr0.estimate(img_mat)
    rv, tv = T_to_rvec_tvec(pose0.T_cam_robot)
    x0 = np.concatenate([[K0[0, 0], K0[1, 1], K0[0, 2], K0[1, 2], 0.0, 0.0], rv, tv])

    rows = [("true lens (reference)", table_error(K_true, TRUE_DIST, img_mat, mat, cam)),
            ("starting guess", table_error(K0, np.zeros(5), img_mat, mat, cam))]
    for n in n_list:
        for use_mat in (True, False):
            label = f"robot outline x{n}" + (" + mat" if use_mat else " only")
            log(f"  {label}")
            K, dist, T = calibrate_from_robot(rm, poses[:n], images[:n], mat_obs, x0, use_mat, log=log)
            rows.append((label, table_error(K, dist, img_mat, mat, cam), K, dist))
    hdr = "  ".join(f"z={int(z)}" for z in HEIGHTS)
    log(f"\n  {'method':28s} {'fx%':>6s} {'cx,cy px':>11s} {'k1,k2':>13s} | max error mm: {hdr} | cam pos")
    for r in rows:
        label, errs = r[0], r[1]
        e = "  ".join(f"{x:5.2f}" for x in errs[:-1])
        if len(r) == 2:
            log(f"  {label:28s} {'':>6s} {'':>11s} {'':>13s} | {e} | {errs[-1]:6.2f}")
        else:
            K, dist = r[2], r[3]
            log(f"  {label:28s} {100 * (K[0, 0] / K_true[0, 0] - 1):6.2f} "
                f"{K[0, 2] - K_true[0, 2]:5.1f},{K[1, 2] - K_true[1, 2]:5.1f} "
                f"{dist[0]:6.3f},{dist[1]:6.3f} | {e} | {errs[-1]:6.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url")
    ap.add_argument("--session")
    ap.add_argument("--seed", type=int, nargs="+", default=[0])
    ap.add_argument("--poses", type=int, nargs="+", default=[9, 15])
    args = ap.parse_args()
    rm = RobotModel()

    def go(v):
        for s in args.seed:
            print(f"\n=== pose set {s} ===", flush=True)
            run_one(v, rm, s, args.poses, log=lambda *a: print(*a, flush=True))

    if args.url:
        session = args.session
        if not session:
            http = args.url.replace("ws://", "http://").rsplit("/ws", 1)[0]
            session = json.loads(urllib.request.urlopen(http + "/sessions").read())[0]["id"]
        v = Viewer(args.url, session)
        try:
            go(v)
        finally:
            v.close()
    else:
        with HeadlessViewer() as hv:
            v = Viewer(hv.ws_url, hv.session)
            try:
                go(v)
            finally:
                v.close()


if __name__ == "__main__":
    main()
