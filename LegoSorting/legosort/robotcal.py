"""Camera calibration from the robot's own outline, with no target on the arm.

The arm moves through joint poses in front of a fixed camera. Forward kinematics
(taken as exact: the link models are accurate and the joint offsets are calibrated
separately) places the robot's meshes. The camera intrinsics (fx, fy, cx, cy, k1, k2)
and the camera pose are then fitted so that

  * the predicted outline of the arm lies on the observed outline in every image, and
  * the base mat's marker corners reproject onto where they were detected.

The observed outline uses only what a real setup has: the per-pixel median over all
images is the empty background (the camera is fixed, the arm moves), the foreground
is what differs from it, and the edges kept are Canny edges near the foreground
boundary. That drops the mat's printing, soft shadows and the arm's interior detail.

In the simulation this matched calibrating with the true lens (PLAN.md, progress
log). The mat is essential: the outline alone settles on a wrong principal point.
"""
import json
import sys
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares

from .boards import BaseMat
from .geometry import project, rvec_tvec_to_T, T_to_rvec_tvec
from .simserver import VIEWER_DIR
from .tracker import BaseTracker

BORDER = 6            # outline within this many px of the image edge is the frame, not the robot

# meca500_bridge.py clamps to these before driving the real arm
BRIDGE_LIMITS = [(-175, 175), (-70, 90), (-135, 70), (-170, 170), (-115, 115), (-36000, 36000)]


# ------------------------------------------------------------------ kinematics

def _kin():
    if str(VIEWER_DIR) not in sys.path:
        sys.path.insert(0, str(VIEWER_DIR))
    from RobotDefinitions import Meca500_kin
    return Meca500_kin


def joint_limits(margin=2.0):
    """Intersection of the GNKinematics, viewer and bridge limits, less a margin.

    The viewer and the bridge both clamp silently, so poses are planned inside all three.
    """
    lo, hi = np.array(_kin().motor_limits, float).T
    cfg = json.loads((VIEWER_DIR / "meca500_config.json").read_text())
    vl = np.array([j["limits"] for j in cfg["joints"]], float)
    bl = np.array(BRIDGE_LIMITS, float)
    return (np.maximum.reduce([lo, vl[:, 0], bl[:, 0]]) + margin,
            np.minimum.reduce([hi, vl[:, 1], bl[:, 1]]) - margin)


def tool_points(q, length, radius):
    """Points on a cylinder standing in for the gripper: from the flange along the tool axis."""
    f = _kin().f_kinematics(np.asarray(q, float))
    R = np.column_stack([f[5], f[6], f[7]])
    p = np.asarray(f[8], float)
    pts = [p]
    if length > 0:
        for s in np.linspace(0, length, 4):
            for a in np.linspace(0, 2 * np.pi, 8, endpoint=False):
                pts.append(p + R[:, 2] * s + radius * (np.cos(a) * R[:, 0] + np.sin(a) * R[:, 1]))
    return np.array(pts)


# ------------------------------------------------------------------ safe poses

@dataclass
class Safety:
    """Keep-out rules for poses and the moves between them (robot frame, mm)."""
    clearance: float = 15.0          # lowest moving link / tool point above the table
    camera: np.ndarray = None        # camera centre, if known
    camera_clear: float = 150.0      # no part of the arm or tool closer than this to it
    tool_length: float = 0.0         # gripper cylinder beyond the flange
    tool_radius: float = 0.0

    def why_unsafe(self, rm, q):
        v, _ = rm.vertices(q, min_joint=1)
        pts = np.vstack([v[:: max(1, len(v) // 3000)], tool_points(q, self.tool_length, self.tool_radius)])
        if pts[:, 2].min() < self.clearance:
            return f"{pts[:, 2].min():.0f} mm above the table"
        if self.camera is not None:
            d = np.linalg.norm(pts - self.camera, axis=1).min()
            if d < self.camera_clear:
                return f"{d:.0f} mm from the camera"
        return None

    def path_ok(self, rm, qa, qb, step_deg=5.0):
        qa, qb = np.asarray(qa, float), np.asarray(qb, float)
        n = max(1, int(np.ceil(np.abs(qb - qa).max() / step_deg)))
        return all(self.why_unsafe(rm, qa + (qb - qa) * k / n) is None for k in range(1, n + 1))


def outline_cells(rm, q, K, dist, T_cam_robot, size, grid=(8, 6)):
    """Image-grid cells the arm's outline passes through (for coverage)."""
    m = rm.mask(q, K, dist, T_cam_robot, size)
    edge = cv2.morphologyEx(m, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
    edge[:BORDER], edge[-BORDER:], edge[:, :BORDER], edge[:, -BORDER:] = False, False, False, False
    ys, xs = np.nonzero(edge)
    return len(xs), set(zip((xs * grid[0] // size[0]).tolist(), (ys * grid[1] // size[1]).tolist()))


def plan_poses(rm, K, dist, T_cam_robot, size, n, rng, safety: Safety, limits=None,
               j1_span=60.0, start=None, tries=600):
    """Choose n safe joint poses whose outlines spread over the image, in visiting order.

    Random poses within the limits (J1 within +-j1_span of facing the mat) are kept if
    they are safe and show enough outline; then poses are picked greedily for new
    image coverage, and finally ordered so that each move from the previous pose
    (from ``start`` for the first) is safe along the way.
    Returns (poses, coverage fraction of the image grid).
    """
    lo, hi = limits if limits is not None else joint_limits()
    cands = []
    for _ in range(tries):
        q = rng.uniform(lo, hi)
        q[0] = rng.uniform(max(lo[0], -j1_span), min(hi[0], j1_span))
        q[5] = rng.uniform(-90, 90)
        if safety.why_unsafe(rm, q):
            continue
        npx, cells = outline_cells(rm, q, K, dist, T_cam_robot, size)
        if npx < 500:
            continue
        cands.append((q, cells))
        if len(cands) >= 5 * n:
            break
    chosen, covered = [], set()
    while len(chosen) < n and cands:
        best = max(cands, key=lambda c: len(c[1] - covered) + 0.01 * rng.random())
        chosen.append(best[0])
        covered |= best[1]
        cands = [c for c in cands if c is not best]
    return order_poses(rm, chosen, start, safety), len(covered) / 48.0


def order_poses(rm, poses, start, safety: Safety):
    """Nearest-neighbour order with every move checked; unreachable poses are dropped."""
    cur = np.asarray(start if start is not None else poses[0], float)
    left, out = list(poses), []
    while left:
        left.sort(key=lambda q: np.abs(np.asarray(q) - cur).max())
        nxt = next((q for q in left if safety.path_ok(rm, cur, q)), None)
        if nxt is None:
            break
        out.append(nxt)
        left = [q for q in left if q is not nxt]
        cur = np.asarray(nxt, float)
    if start is not None and out and not safety.path_ok(rm, cur, start):
        out.pop()                 # must be able to get back
    return out


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
        out.append(cv2.distanceTransform(255 - edges, cv2.DIST_L2, 5))
    return out


def mat_observations(mat: BaseMat, image):
    """Detected mat marker corners: (robot-frame points, board-frame points, pixels)."""
    tr = BaseTracker(mat, np.eye(3), np.zeros(5))
    corners, ids = tr.detect(image)
    if len(ids) < 2:
        raise RuntimeError(f"only {len(ids)} mat markers visible; need at least 2")
    obj, img = tr.board.matchImagePoints(corners, ids.reshape(-1, 1))
    obj = obj.reshape(-1, 3).astype(float)
    robot = obj @ mat.T_robot_board[:3, :3].T + mat.T_robot_board[:3, 3]
    return robot, obj, img.reshape(-1, 2).astype(float)


def focal_from_mat(board_pts, pixels, size, f_guess):
    """Focal length from one tilted view of the flat mat (principal point at the centre,
    no distortion). Returns f_guess if the view is too square-on to tell."""
    K0 = np.array([[f_guess, 0, size[0] / 2 - 0.5], [0, f_guess, size[1] / 2 - 0.5], [0, 0, 1.0]])
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO
             | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    try:
        _, K, _, _, _ = cv2.calibrateCamera([board_pts.astype(np.float32)], [pixels.astype(np.float32)],
                                            size, K0, np.zeros(5), flags=flags)
    except cv2.error:
        return f_guess
    f = float(K[0, 0])
    return f if 0.6 * f_guess < f < 1.6 * f_guess else f_guess


# ------------------------------------------------------------------ fitting

def unpack(x):
    fx, fy, cx, cy, k1, k2 = x[:6]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    dist = np.array([k1, k2, 0, 0, 0])
    return K, dist, rvec_tvec_to_T(x[6:9], x[9:12])


def pack(K, dist, T):
    rv, tv = T_to_rvec_tvec(T)
    return np.concatenate([[K[0, 0], K[1, 1], K[0, 2], K[1, 2], dist[0], dist[1]], np.ravel(rv), np.ravel(tv)])


def outline_points(rm, q, K, dist, T, size, max_joint=None, per_image=500, rng=None):
    """Robot-frame mesh points on the predicted outer outline for joint pose q.

    The whole arm decides what is outline; only links up to ``max_joint`` supply
    points (leave out the flange link when a gripper the model lacks is fitted).
    """
    m = rm.mask(q, K, dist, T, size)
    cnt, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    line = np.zeros_like(m)
    cv2.drawContours(line, cnt, -1, 255, 1)
    dt = cv2.distanceTransform(255 - line, cv2.DIST_L2, 5)
    v, _ = rm.vertices(q, min_joint=0, max_joint=max_joint)
    pix = project(v, K, dist, T)
    ok = ((pix[:, 0] >= BORDER) & (pix[:, 0] < size[0] - BORDER)
          & (pix[:, 1] >= BORDER) & (pix[:, 1] < size[1] - BORDER))
    v, pix = v[ok], pix[ok]
    v = v[map_coordinates(dt, [pix[:, 1], pix[:, 0]], order=1) < 0.7]
    if rng is not None and len(v) > per_image:
        v = v[rng.choice(len(v), per_image, replace=False)]
    return v


@dataclass
class Fit:
    K: np.ndarray
    dist: np.ndarray
    T_cam_robot: np.ndarray
    median_px: float              # median outline distance, px (good fits ~0.25 in the sim)
    mat_rms_px: float             # mat corner reprojection RMS
    n_points: int                 # outline points used in the last iteration
    start_fx: float = 0.0
    log: list = field(default_factory=list)

    @property
    def camera_position(self):
        R, t = self.T_cam_robot[:3, :3], self.T_cam_robot[:3, 3]
        return -R.T @ t


def fit(rm, poses, dts, size, mat_obs, x0, max_joint=None, iters=8, mat_weight=3.0, log=print):
    """Iterative closest-outline fit of the 12 camera parameters (see module docstring)."""
    rng = np.random.default_rng(0)
    x = np.asarray(x0, float).copy()
    gate, lines = 25.0, []
    robot_pts, _, mat_px = mat_obs
    for it in range(iters):
        K, dist, T = unpack(x)
        pts = []
        for q, dt in zip(poses, dts):
            p3 = outline_points(rm, q, K, dist, T, size, max_joint, rng=rng)
            pix = project(p3, K, dist, T)
            d = map_coordinates(dt, [pix[:, 1], pix[:, 0]], order=1, mode="nearest")
            pts.append(p3[d < gate])                      # ICP-style outlier gate
        n_pts = sum(len(p) for p in pts)

        def residuals(x):
            K, dist, T = unpack(x)
            r = []
            for p3, dt in zip(pts, dts):
                pix = project(p3, K, dist, T)
                r.append(map_coordinates(dt, [pix[:, 1], pix[:, 0]], order=1, mode="nearest"))
            r.append(mat_weight * (project(robot_pts, K, dist, T) - mat_px).ravel())
            return np.concatenate(r)

        sol = least_squares(residuals, x, loss="huber", f_scale=1.5, x_scale="jac", max_nfev=60)
        step = np.abs(sol.x - x)
        x = sol.x
        K, dist, T = unpack(x)
        outline_r = sol.fun[:n_pts]
        med = float(np.median(np.abs(outline_r))) if n_pts else float("inf")
        line = (f"iter {it + 1}: {n_pts} outline points, gate {gate:4.1f} px, median {med:.2f} px, "
                f"fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f} "
                f"k1 {dist[0]:.3f} k2 {dist[1]:.3f}")
        lines.append(line)
        log("    " + line)
        gate = max(4.0, gate * 0.5)
        if it > 2 and step[:4].max() < 0.05:
            break
    mat_r = project(robot_pts, K, dist, T) - mat_px
    return Fit(K, dist, T, med, float(np.sqrt(np.mean(np.sum(mat_r ** 2, axis=1)))), n_pts,
               float(x0[0]), lines)


def calibrate(rm, poses, images, mat_image, mat: BaseMat, f_guess, max_joint=None,
              residual_limit=0.6, log=print):
    """Full calibration: starting guess from the mat, outline fit, and a residual check.

    If the median outline distance is above ``residual_limit`` px the fit has probably
    found a wrong minimum, so it is restarted from focal lengths 8 % either side and
    the fit with the smallest residual is kept. Returns (best Fit, all fits).
    """
    size = (images[0].shape[1], images[0].shape[0])
    mat_obs = mat_observations(mat, mat_image)
    f0 = focal_from_mat(mat_obs[1], mat_obs[2], size, f_guess)
    log(f"  starting focal length {f0:.0f} px (guess {f_guess:.0f}) from {len(mat_obs[2]) // 4} mat markers")
    dts = observed_edges(images)

    def start(f):
        K = np.array([[f, 0, size[0] / 2 - 0.5], [0, f, size[1] / 2 - 0.5], [0, 0, 1.0]])
        pose = BaseTracker(mat, K, np.zeros(5)).estimate(mat_image)
        return pack(K, np.zeros(5), pose.T_cam_robot)

    fits = []
    for scale in (1.0, 0.92, 1.08):
        if fits:
            log(f"  median {fits[-1].median_px:.2f} px is above {residual_limit} px: "
                f"restarting from fx x{scale:.2f}")
        fits.append(fit(rm, poses, dts, size, mat_obs, start(f0 * scale), max_joint, log=log))
        if fits[-1].median_px <= residual_limit:
            break
    return min(fits, key=lambda f: f.median_px), fits


def overlay(image, rm, q, f: Fit, max_joint=None):
    """The image with the fitted outline drawn (green: used links, orange: left out)."""
    size = (image.shape[1], image.shape[0])
    out = image.copy()
    m_all = rm.mask(q, f.K, f.dist, f.T_cam_robot, size)
    cnt, _ = cv2.findContours(m_all, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(out, cnt, -1, (0, 160, 255), 3, cv2.LINE_AA)
    pts = outline_points(rm, q, f.K, f.dist, f.T_cam_robot, size, max_joint)
    for u, v in project(pts, f.K, f.dist, f.T_cam_robot):
        cv2.circle(out, (int(round(u)), int(round(v))), 3, (0, 255, 0), -1)
    return out
