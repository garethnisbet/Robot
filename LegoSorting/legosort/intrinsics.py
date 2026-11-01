"""Camera intrinsic calibration from ChArUco views."""
import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .boards import CharucoSpec
from .camera import DATA


def detect_charuco(gray, spec: CharucoSpec, detector=None):
    """Return (object_points, image_points) for the board corners seen, or None."""
    board = spec.board()
    detector = detector or cv2.aruco.CharucoDetector(board)
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < 6:
        return None
    obj, img = board.matchImagePoints(corners, ids)
    if obj is None or len(obj) < 6:
        return None
    return obj.reshape(-1, 3).astype(np.float32), img.reshape(-1, 2).astype(np.float32)


@dataclass
class Intrinsics:
    K: np.ndarray
    dist: np.ndarray
    image_size: tuple            # (width, height)
    rms: float = float("nan")
    n_views: int = 0
    per_view_rms: list = field(default_factory=list)
    camera: dict = field(default_factory=dict)   # settings it was calibrated with
    board: dict = field(default_factory=dict)
    created: str = ""

    @staticmethod
    def path_for(name):
        return DATA / f"intrinsics_{name}.json"

    def save(self, name):
        DATA.mkdir(exist_ok=True)
        p = self.path_for(name)
        p.write_text(json.dumps({
            "K": self.K.tolist(), "dist": self.dist.ravel().tolist(),
            "image_size": list(self.image_size), "rms": self.rms,
            "n_views": self.n_views, "per_view_rms": self.per_view_rms,
            "camera": self.camera, "board": self.board, "created": self.created,
        }, indent=2))
        return p

    @classmethod
    def load(cls, name_or_path):
        p = Path(name_or_path)
        if not p.suffix:
            p = cls.path_for(name_or_path)
        d = json.loads(p.read_text())
        return cls(np.array(d["K"]), np.array(d["dist"]), tuple(d["image_size"]),
                   d.get("rms", float("nan")), d.get("n_views", 0),
                   d.get("per_view_rms", []), d.get("camera", {}), d.get("board", {}),
                   d.get("created", ""))

    def mismatch(self, settings):
        """Reasons these intrinsics may not apply to a camera with ``settings``."""
        out = []
        if tuple(self.image_size) != (settings.width, settings.height):
            out.append(f"resolution {self.image_size} vs {settings.width}x{settings.height}")
        cal = self.camera.get("controls", {})
        for k in ("focus_absolute", "zoom_absolute", "focus_automatic_continuous"):
            if k in cal and cal[k] != settings.controls.get(k):
                out.append(f"{k} calibrated at {cal[k]}, now {settings.controls.get(k)}")
        return out


def calibrate(views, image_size, reject_factor=3.0, fix_k3=True):
    """Calibrate from a list of (obj, img) views.

    Views with a reprojection error above ``reject_factor`` × median are dropped once
    and the calibration is repeated (motion blur and misdetections show up this way).
    k3 is fixed at zero by default: webcam lenses have mild distortion and a free k3
    extrapolates badly into image corners the board never reached.
    Returns (Intrinsics, kept_indices).
    """
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    keep = list(range(len(views)))
    for _ in range(2):
        obj = [views[i][0] for i in keep]
        img = [views[i][1] for i in keep]
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, image_size, None, None, flags=flags)
        errs = []
        for o, im, r, t in zip(obj, img, rvecs, tvecs):
            p, _ = cv2.projectPoints(o, r, t, K, dist)
            errs.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - im) ** 2, axis=1)))))
        med = float(np.median(errs))
        bad = [k for k, e in zip(keep, errs) if e > reject_factor * med]
        if not bad or len(keep) - len(bad) < 8:
            break
        keep = [k for k in keep if k not in bad]
    intr = Intrinsics(K, dist.ravel(), tuple(image_size), float(rms), len(keep),
                      [round(e, 4) for e in errs],
                      created=time.strftime("%Y-%m-%d %H:%M:%S"))
    return intr, keep


def view_diversity(image_size, views):
    """Simple coverage summary: fraction of a 6x4 image grid with at least one corner."""
    w, h = image_size
    grid = np.zeros((4, 6), bool)
    for _, img in views:
        gx = np.clip((img[:, 0] / w * 6).astype(int), 0, 5)
        gy = np.clip((img[:, 1] / h * 4).astype(int), 0, 3)
        grid[gy, gx] = True
    return float(grid.mean())
