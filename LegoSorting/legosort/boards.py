"""Printed targets: the ChArUco board for intrinsics and the base mat of ArUco markers.

Board frame convention (both targets): origin at the centre of the sheet, X to the
right and Y towards the top edge of the printed image, Z out of the paper.
"""
import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .geometry import make_T, rot_z

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MAT = ROOT / "config" / "base_mat.json"


def dictionary(name):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def check_bar(canvas, px_per_mm, x_mm, y_mm, label):
    """100 mm bar with 10 mm ticks and a label; (x_mm, y_mm) from the image top-left."""
    p = lambda mm: int(round(mm * px_per_mm))  # noqa: E731
    t = max(1, int(round(0.25 * px_per_mm)))
    y = p(y_mm)
    cv2.line(canvas, (p(x_mm), y), (p(x_mm + 100), y), 0, t)
    for i in range(11):
        cv2.line(canvas, (p(x_mm + 10 * i), y - p(2 if i % 5 else 4)), (p(x_mm + 10 * i), y), 0, t)
    cv2.putText(canvas, label, (p(x_mm + 104), y), cv2.FONT_HERSHEY_SIMPLEX,
                0.12 * px_per_mm, 0, t)


# --------------------------------------------------------------------- ChArUco

@dataclass
class CharucoSpec:
    squares_x: int = 9
    squares_y: int = 6
    square_mm: float = 30.0
    marker_mm: float = 22.0
    dict_name: str = "DICT_4X4_50"

    def board(self):
        return cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y), self.square_mm, self.marker_mm,
            dictionary(self.dict_name))

    def size_mm(self):
        return self.squares_x * self.square_mm, self.squares_y * self.square_mm

    def render(self, px_per_mm, margin_mm=0.0):
        w, h = self.size_mm()
        m = int(round(margin_mm * px_per_mm))
        size = (int(round(w * px_per_mm)) + 2 * m, int(round(h * px_per_mm)) + 2 * m)
        return self.board().generateImage(size, marginSize=m, borderBits=1)


# --------------------------------------------------------------------- Base mat

@dataclass
class Marker:
    id: int
    x: float          # centre, board frame, mm
    y: float
    size: float       # outer black edge length, mm
    rot_deg: float = 0.0

    def corners(self):
        """Corners in board frame, in ArUco order: TL, TR, BR, BL of the printed marker."""
        h = self.size / 2.0
        local = np.array([[-h, h], [h, h], [h, -h], [-h, -h]])
        R = rot_z(self.rot_deg)[:2, :2]
        xy = local @ R.T + [self.x, self.y]
        return np.column_stack([xy, np.zeros(4)]).astype(np.float32)


@dataclass
class BaseMat:
    width_mm: float
    height_mm: float
    dict_name: str
    markers: list
    T_robot_board: np.ndarray = field(default_factory=lambda: np.eye(4))
    source: str = ""

    # ---- io
    @classmethod
    def load(cls, path=DEFAULT_MAT):
        path = Path(path)
        d = json.loads(path.read_text())
        markers = [Marker(**m) for m in d["markers"]]
        if "T_robot_board" in d:
            T = np.array(d["T_robot_board"], dtype=float)
        else:
            p = d["board_in_robot"]
            T = make_T(rot_z(p.get("rz_deg", 0.0)), p["xyz"])
        return cls(d["width_mm"], d["height_mm"], d["dict_name"], markers, T, str(path))

    def to_dict(self):
        return {
            "width_mm": self.width_mm,
            "height_mm": self.height_mm,
            "dict_name": self.dict_name,
            "markers": [m.__dict__ for m in self.markers],
            "T_robot_board": np.round(self.T_robot_board, 6).tolist(),
        }

    # ---- opencv
    def board(self):
        obj = [m.corners() for m in self.markers]
        ids = np.array([m.id for m in self.markers], dtype=np.int32)
        return cv2.aruco.Board(obj, dictionary(self.dict_name), ids)

    def outline(self):
        w, h = self.width_mm / 2, self.height_mm / 2
        return np.array([[-w, -h, 0], [w, -h, 0], [w, h, 0], [-w, h, 0]], dtype=float)

    # ---- printing
    def render(self, px_per_mm, decorations=True):
        """Image of the whole mat at true scale (white background).

        ``decorations`` adds the printed 100 mm check bar and centre cross, so the
        detector's model of the empty mat matches the print."""
        W = int(round(self.width_mm * px_per_mm))
        H = int(round(self.height_mm * px_per_mm))
        img = np.full((H, W), 255, np.uint8)
        d = dictionary(self.dict_name)
        for m in self.markers:
            side = int(round(m.size * px_per_mm))
            tile = cv2.aruco.generateImageMarker(d, m.id, side, borderBits=1)
            if m.rot_deg % 360:
                # Rotate about the tile centre; only multiples of 90 deg are exact.
                k = int(round(m.rot_deg / 90.0)) % 4
                tile = np.rot90(tile, k)
            u0 = int(round((m.x - m.size / 2 + self.width_mm / 2) * px_per_mm))
            v0 = int(round((self.height_mm / 2 - (m.y + m.size / 2)) * px_per_mm))
            img[v0:v0 + side, u0:u0 + side] = tile
        if decorations:
            check_bar(img, px_per_mm, 70, 75, "100 mm   base mat - TOP EDGE TOWARDS ROBOT")
            c = (W // 2, H // 2)
            cv2.drawMarker(img, c, 160, cv2.MARKER_CROSS, int(20 * px_per_mm),
                           max(1, int(round(0.17 * px_per_mm))))
        return img


def default_mat_layout(width=420.0, height=297.0, size=40.0, margin=12.0,
                       n_long=6, n_short_inner=2, dict_name="DICT_5X5_100"):
    """Markers around the border of a sheet (A3 landscape by default), interior left clear."""
    c = margin + size / 2
    xs = np.linspace(-width / 2 + c, width / 2 - c, n_long)
    ys = np.linspace(-height / 2 + c, height / 2 - c, n_short_inner + 2)[1:-1]
    pts = [(x, -height / 2 + c) for x in xs]
    pts += [(width / 2 - c, y) for y in ys]
    pts += [(x, height / 2 - c) for x in xs[::-1]]
    pts += [(-width / 2 + c, y) for y in ys[::-1]]
    markers = [Marker(i, round(float(x), 3), round(float(y), 3), size) for i, (x, y) in enumerate(pts)]
    return width, height, dict_name, markers
