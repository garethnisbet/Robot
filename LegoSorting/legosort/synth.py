"""Synthetic camera images of planar targets, for tests that need no hardware."""
import cv2
import numpy as np

SUPERSAMPLE = 2


def intrinsics_matrix(fx, fy, cx, cy):
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)


def render_plane(texture, px_per_mm, K, T_cam_board, image_size, dist=None, background=110):
    """Image of a flat texture lying in the board's Z=0 plane, centred on the board origin.

    ``texture`` is a grayscale image at ``px_per_mm``, with image-up = board +Y.
    Renders at SUPERSAMPLE× then box-filters down to limit aliasing.
    """
    w, h = image_size
    s = SUPERSAMPLE
    th, tw = texture.shape[:2]
    # texture pixel centre (u, v) -> board (x, y)
    A = np.array([[1 / px_per_mm, 0, 0.5 / px_per_mm - tw / (2 * px_per_mm)],
                  [0, -1 / px_per_mm, th / (2 * px_per_mm) - 0.5 / px_per_mm],
                  [0, 0, 1]])
    # high-res camera: u_hi = s * u + (s - 1) / 2
    S = np.array([[s, 0, (s - 1) / 2], [0, s, (s - 1) / 2], [0, 0, 1]])
    R, t = T_cam_board[:3, :3], T_cam_board[:3, 3]
    P = np.column_stack([R[:, 0], R[:, 1], t])
    H = S @ K @ P @ A
    hi = cv2.warpPerspective(texture, H, (w * s, h * s), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=background)
    img = cv2.resize(hi, (w, h), interpolation=cv2.INTER_AREA)
    if dist is not None and np.any(dist):
        img = distort(img, K, dist)
    return img


def distort(img, K, dist):
    """Apply lens distortion to an ideal pinhole image."""
    h, w = img.shape[:2]
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    pts = np.stack([u.ravel(), v.ravel()], axis=1).reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts, K, np.asarray(dist, float), P=K).reshape(h, w, 2)
    return cv2.remap(img, und[..., 0], und[..., 1], cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=110)


def add_noise(img, sigma=2.0, seed=0):
    rng = np.random.default_rng(seed)
    out = img.astype(np.float32) + rng.normal(0, sigma, img.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def random_view(rng, target=(0, 0, 0), distance=(350, 700), tilt_deg=(0, 45), aim_mm=40):
    """T_cam_board for a camera looking roughly at ``target`` from above the board."""
    from .geometry import invert_T, look_at
    d = rng.uniform(*distance)
    tilt = np.radians(rng.uniform(*tilt_deg))
    az = rng.uniform(0, 2 * np.pi)
    eye = np.asarray(target, float) + d * np.array(
        [np.sin(tilt) * np.cos(az), np.sin(tilt) * np.sin(az), np.cos(tilt)])
    aim = np.asarray(target, float) + rng.uniform(-aim_mm, aim_mm, 3) * [1, 1, 0]
    up = [np.cos(az + rng.uniform(-1, 1)), np.sin(az + rng.uniform(-1, 1)), 0.3]
    return invert_T(look_at(eye, aim, up))
