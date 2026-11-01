"""Per-frame camera pose in the robot frame, from the base mat of ArUco markers."""
from dataclasses import dataclass

import cv2
import numpy as np

from .boards import BaseMat
from .geometry import invert_T, pixels_to_plane, project, rvec_tvec_to_T


@dataclass
class BasePose:
    T_cam_board: np.ndarray
    T_cam_robot: np.ndarray
    ids: np.ndarray            # marker ids used
    n_markers: int
    rms_px: float              # reprojection error of the mat corners
    corners: list              # raw detected marker corners (for drawing)

    @property
    def T_robot_cam(self):
        return invert_T(self.T_cam_robot)

    @property
    def camera_position(self):
        """Camera centre in the robot frame, mm."""
        return self.T_robot_cam[:3, 3]


class BaseTracker:
    def __init__(self, mat: BaseMat, K, dist, min_markers=2):
        self.mat = mat
        self.K = np.asarray(K, float)
        self.dist = np.asarray(dist, float)
        self.min_markers = min_markers
        self.board = mat.board()
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.01
        self.detector = cv2.aruco.ArucoDetector(self.board.getDictionary(), params)
        self.T_robot_board = mat.T_robot_board
        self.T_board_robot = invert_T(mat.T_robot_board)
        self.known_ids = {m.id for m in mat.markers}

    def detect(self, image):
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return [], np.zeros(0, int)
        ids = ids.ravel()
        keep = [i for i, k in enumerate(ids) if k in self.known_ids]
        return [corners[i] for i in keep], ids[keep]

    def estimate(self, image):
        """Return a BasePose, or None if too few mat markers are visible."""
        corners, ids = self.detect(image)
        if len(ids) < self.min_markers:
            return None
        obj, img = self.board.matchImagePoints(corners, ids.reshape(-1, 1))
        obj = obj.reshape(-1, 3).astype(np.float64)
        img = img.reshape(-1, 2).astype(np.float64)
        # SQPnP gives a global solution without an initial guess; LM then polishes it.
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.dist, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.K, self.dist, rvec, tvec)
        T_cam_board = rvec_tvec_to_T(rvec, tvec)
        resid = project(obj, self.K, self.dist, T_cam_board) - img
        rms = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))
        return BasePose(T_cam_board, T_cam_board @ self.T_board_robot, ids,
                        len(ids), rms, corners)

    # Helpers that work in the robot frame -------------------------------------
    def pixel_to_robot(self, pose: BasePose, pixels, z=0.0):
        """Robot-frame points where pixel rays meet the plane Z = z (table is z = 0)."""
        return pixels_to_plane(pixels, self.K, self.dist, pose.T_robot_cam, z)

    def robot_to_pixel(self, pose: BasePose, points):
        return project(points, self.K, self.dist, pose.T_cam_robot)
