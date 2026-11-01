"""Rigid transforms and camera-ray geometry.

Conventions: 4x4 homogeneous matrices, millimetres. ``T_a_b`` maps points
expressed in frame b into frame a (p_a = T_a_b @ p_b).
"""
import cv2
import numpy as np


def make_T(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.ravel(t)
    return T


def invert_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rvec_tvec_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    return make_T(R, np.ravel(tvec))


def T_to_rvec_tvec(T):
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return rvec.reshape(3), T[:3, 3].copy()


def rot_x(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def transform_points(T, pts):
    pts = np.atleast_2d(np.asarray(pts, dtype=float))
    return pts @ T[:3, :3].T + T[:3, 3]


def rotation_angle_deg(R):
    """Angle of a rotation matrix, in degrees."""
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """T_world_cam for an OpenCV camera (z forward, y down) at ``eye`` looking at ``target``."""
    eye, target, up = (np.asarray(v, dtype=float) for v in (eye, target, up))
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-9:
        x = np.cross(z, [0.0, 1.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return make_T(np.column_stack([x, y, z]), eye)


def pixel_rays(pixels, K, dist):
    """Unit-depth ray directions (camera frame) for distorted pixel coordinates."""
    pix = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pix, K, dist).reshape(-1, 2)
    return np.column_stack([norm, np.ones(len(norm))])


def pixels_to_plane(pixels, K, dist, T_frame_cam, z=0.0):
    """Intersect pixel rays with the plane Z = z of some frame (e.g. the robot base frame).

    Returns an (N, 3) array in that frame; rows are NaN where the ray is parallel to
    the plane or the plane is behind the camera.
    """
    d = pixel_rays(pixels, K, dist) @ T_frame_cam[:3, :3].T
    o = T_frame_cam[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (z - o[2]) / d[:, 2]
    s[~np.isfinite(s) | (s <= 0)] = np.nan
    return o + s[:, None] * d


def project(points, K, dist, T_cam_frame):
    """Project points given in some frame into the image. Returns (N, 2) pixels."""
    rvec, tvec = T_to_rvec_tvec(T_cam_frame)
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 1, 3)
    img, _ = cv2.projectPoints(pts, rvec, tvec, K, dist)
    return img.reshape(-1, 2)
