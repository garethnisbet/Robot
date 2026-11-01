"""Find studs-up bricks on the mat: position, yaw, size (studs) and colour, in the robot frame.

Method (one calibrated camera, pose from the base mat):
1. Segment brick pixels in the image: anything darker than the empty mat should
   look there, or coloured. The camera pose is known, so the printed mat (markers,
   check bar, cross) is predicted per pixel and does not count as a brick. Bricks of
   every colour except white/tan are darker than the white mat.
2. Re-project the mask onto two horizontal planes in the robot frame: the table
   (z = 0) and the brick tops (z = h). Seen at an angle, a brick spills away from
   the camera on the z = 0 view and towards it on the z = h view, so their
   intersection is the true footprint whatever the viewing angle.
3. Fit a rotated rectangle per footprint -> centre, yaw, length/width -> studs.
4. Sample the colour on the top face (projected back into the image).
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

from .geometry import project
from .lego import BRICK_H, GAP, PITCH


@dataclass
class Detection:
    x: float
    y: float
    yaw: float              # deg, long axis from robot X, in (-90, 90]
    length: float           # mm, measured
    width: float
    nx: int                 # studs along the long axis
    ny: int
    lab: np.ndarray         # CIELAB of the top face (L 0-100)
    bgr: np.ndarray
    area_mm2: float
    size_error: float       # mm, worst |measured - nominal| over length and width
    cluster: int = -1
    corners: np.ndarray = field(default=None, repr=False)   # footprint corners, robot XY


@dataclass
class DetectConfig:
    region: tuple = (138.0, 322.0, -153.0, 153.0)   # robot x/y limits: the mat interior
    px_per_mm: float = 3.0
    height: float = BRICK_H
    z_base: float = 0.3                              # mat surface
    dark_thresh: float = 14.0                        # L* below the local mat white
    chroma_thresh: float = 18.0
    bg_kernel_mm: float = 70.0                       # larger than any brick
    min_area_mm2: float = 60.0
    mat_px_per_mm: float = 4.0                       # resolution of the predicted mat


BAND = (0.0, 1.0)     # L-percentile band of top-face samples used for colour


def _lab(img_bgr):
    return cv2.cvtColor(img_bgr.astype(np.float32) / 255.0, cv2.COLOR_BGR2Lab)


def mat_reflectance(mat, K, dist, T_cam_robot, shape, px_per_mm=4.0, step=4):
    """Predicted reflectance of the empty mat at every image pixel: 1 white, ~0 black.

    Pixels that do not see the mat get 0 (never counted as darker than expected).
    """
    from .geometry import invert_T, pixels_to_plane
    h, w = shape[:2]
    gv, gu = np.mgrid[0:h + step:step, 0:w + step:step].astype(np.float64)
    T_board_cam = invert_T(mat.T_robot_board) @ invert_T(T_cam_robot)
    b = pixels_to_plane(np.column_stack([gu.ravel(), gv.ravel()]), K, dist, T_board_cam, 0.0)
    tu = (b[:, 0] + mat.width_mm / 2) * px_per_mm - 0.5
    tv = (mat.height_mm / 2 - b[:, 1]) * px_per_mm - 0.5
    tu = np.nan_to_num(tu, nan=-1e6).reshape(gu.shape).astype(np.float32)
    tv = np.nan_to_num(tv, nan=-1e6).reshape(gu.shape).astype(np.float32)
    # grid -> full resolution (the mapping is smooth)
    mu = cv2.resize(tu, (gu.shape[1] * step, gu.shape[0] * step), interpolation=cv2.INTER_LINEAR)[:h, :w]
    mv = cv2.resize(tv, (gu.shape[1] * step, gu.shape[0] * step), interpolation=cv2.INTER_LINEAR)[:h, :w]
    tex = mat.render(px_per_mm).astype(np.float32) / 255.0
    return cv2.remap(tex, mu, mv, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def foreground(img, cfg: DetectConfig, kernel_px, reflectance=None):
    """Pixels darker than the mat should look there, or clearly coloured.

    ``reflectance`` (from mat_reflectance) is the predicted mat; without it the mat
    is assumed white everywhere, so its markers count as foreground.
    """
    lab = _lab(img)
    L = lab[..., 0]
    k = int(kernel_px) | 1
    white = cv2.dilate(L, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    expected = white if reflectance is None else white * reflectance
    chroma = np.hypot(lab[..., 1], lab[..., 2])
    fg = ((expected - L) > cfg.dark_thresh) | (chroma > cfg.chroma_thresh)
    return fg.astype(np.uint8) * 255, lab


def plane_maps(K, dist, T_cam_robot, cfg: DetectConfig, z):
    """remap() maps from a top view of the plane z (robot frame) to image pixels.

    Top-view pixel (col u, row v) is robot (x = xmin + v / s, y = ymin + u / s).
    """
    xmin, xmax, ymin, ymax = cfg.region
    s = cfg.px_per_mm
    rows, cols = int(round((xmax - xmin) * s)), int(round((ymax - ymin) * s))
    v, u = np.mgrid[0:rows, 0:cols].astype(np.float64)
    pts = np.column_stack([xmin + v.ravel() / s, ymin + u.ravel() / s, np.full(v.size, z)])
    pix = project(pts, K, dist, T_cam_robot).reshape(rows, cols, 2).astype(np.float32)
    return pix[..., 0], pix[..., 1]


def topview_to_robot(uv, cfg: DetectConfig):
    uv = np.asarray(uv, float).reshape(-1, 2)
    xmin, _, ymin, _ = cfg.region
    return np.column_stack([xmin + uv[:, 1] / cfg.px_per_mm, ymin + uv[:, 0] / cfg.px_per_mm])


def studs(mm):
    return max(1, int(round((mm + GAP) / PITCH)))


def detect_bricks(img, K, dist, T_cam_robot, cfg: DetectConfig = None, debug=None, mat=None):
    """Detections for one image. ``debug`` (a dict) receives the intermediate masks.
    Pass the BaseMat as ``mat`` so its printing is not mistaken for bricks."""
    cfg = cfg or DetectConfig()
    # brick size in image pixels, roughly, to size the background filter
    centre = np.array([[np.mean(cfg.region[:2]), np.mean(cfg.region[2:]), 0.0]])
    p = project(np.vstack([centre, centre + [cfg.bg_kernel_mm, 0, 0]]), K, dist, T_cam_robot)
    kernel_px = max(15, np.linalg.norm(p[1] - p[0]))
    refl = None
    if mat is not None:
        refl = mat_reflectance(mat, K, dist, T_cam_robot, img.shape, cfg.mat_px_per_mm)
    fg, lab = foreground(img, cfg, kernel_px, refl)

    z0, zh = cfg.z_base, cfg.z_base + cfg.height
    m0 = cv2.remap(fg, *plane_maps(K, dist, T_cam_robot, cfg, z0), cv2.INTER_NEAREST, borderValue=0)
    mh = cv2.remap(fg, *plane_maps(K, dist, T_cam_robot, cfg, zh), cv2.INTER_NEAREST, borderValue=0)
    foot = cv2.bitwise_and(m0, mh)
    k = max(3, int(round(cfg.px_per_mm * 1.5)) | 1)
    foot = cv2.morphologyEx(foot, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    if debug is not None:
        debug.update(fg=fg, m0=m0, mh=mh, foot=foot)

    n, labels, stats, _ = cv2.connectedComponentsWithStats(foot, connectivity=8)
    s2 = cfg.px_per_mm ** 2
    out = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA] / s2
        if area < cfg.min_area_mm2:
            continue
        ys, xs = np.nonzero(labels == i)
        rect = cv2.minAreaRect(np.column_stack([xs, ys]).astype(np.float32))
        box = topview_to_robot(cv2.boxPoints(rect), cfg)          # 4 corners, robot XY
        e1, e2 = box[1] - box[0], box[2] - box[1]
        l1, l2 = np.linalg.norm(e1), np.linalg.norm(e2)
        # minAreaRect measures pixel extents (+1 px per side); remove that bias
        l1, l2 = max(l1 - 1 / cfg.px_per_mm, 0), max(l2 - 1 / cfg.px_per_mm, 0)
        long_v, length, width = (e1, l1, l2) if l1 >= l2 else (e2, l2, l1)
        yaw = float(np.degrees(np.arctan2(long_v[1], long_v[0])))
        yaw = (yaw + 90) % 180 - 90
        cx, cy = box.mean(axis=0)
        nx, ny = studs(length), studs(width)
        size_err = max(abs(length - (nx * PITCH - GAP)), abs(width - (ny * PITCH - GAP)))

        # colour: top face, inner 70%, sampled in the original image. ABS is glossy:
        # highlights add white, so use the darker part of the face (10-40th
        # percentile of L), which is the brick's own (diffuse) colour.
        g = np.linspace(-0.35, 0.35, 11)
        uu, vv = np.meshgrid(g, g)
        c, sn = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        local = np.column_stack([uu.ravel() * length, vv.ravel() * width])
        xy = local @ np.array([[c, sn], [-sn, c]]) + [cx, cy]
        pix = project(np.column_stack([xy, np.full(len(xy), zh)]), K, dist, T_cam_robot)
        pix = np.round(pix).astype(int)
        h, w = img.shape[:2]
        ok = (pix[:, 0] >= 0) & (pix[:, 0] < w) & (pix[:, 1] >= 0) & (pix[:, 1] < h)
        if not ok.any():
            continue
        samp_lab = lab[pix[ok, 1], pix[ok, 0]]
        samp_bgr = img[pix[ok, 1], pix[ok, 0]].astype(float)
        order = np.argsort(samp_lab[:, 0])
        n = len(order)
        band = order[int(BAND[0] * n): max(int(BAND[1] * n), int(BAND[0] * n) + 1)]
        samp_lab, samp_bgr = samp_lab[band], samp_bgr[band]
        out.append(Detection(float(cx), float(cy), yaw, float(length), float(width), nx, ny,
                             np.median(samp_lab, axis=0), np.median(samp_bgr, axis=0),
                             float(area), float(size_err), corners=box))
    return out


def delta_e(a, b):
    return float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float)))


def cluster_colours(dets, threshold=12.0):
    """Group detections by top-face colour (greedy, CIE76 delta E). Sets .cluster; returns centres."""
    centres, members = [], []
    for d in sorted(dets, key=lambda d: -d.lab[0]):
        if centres:
            dist = [delta_e(d.lab, c) for c in centres]
            j = int(np.argmin(dist))
            if dist[j] < threshold:
                members[j].append(d)
                centres[j] = np.mean([m.lab for m in members[j]], axis=0)
                d.cluster = j
                continue
        centres.append(d.lab.copy())
        members.append([d])
        d.cluster = len(centres) - 1
    return centres
