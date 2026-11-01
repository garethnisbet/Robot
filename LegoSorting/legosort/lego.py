"""LEGO brick geometry, colours and random layouts (robot frame, mm)."""
from dataclasses import dataclass

import numpy as np

PITCH = 8.0            # stud spacing
GAP = 0.2              # bricks are 0.2 mm short of n * pitch
BRICK_H = 9.6
PLATE_H = 3.2
STUD_D = 4.8
STUD_H = 1.7
WALL = 1.2             # side wall thickness
TOP = 1.0              # top (or plate floor) thickness
TUBE_OD = 6.51         # underside tubes between stud rows (2-wide and up)
PIN_D = 3.2            # underside pins of 1-wide bricks

# Common LEGO colours, sRGB 0-255 (BrickLink/Rebrickable reference values).
COLOURS = {
    "red": (201, 26, 9),
    "blue": (0, 85, 191),
    "yellow": (242, 205, 55),
    "green": (35, 120, 65),
    "orange": (254, 138, 24),
    "black": (27, 42, 52),
    "dark_bluish_grey": (108, 110, 104),
    "light_bluish_grey": (160, 165, 169),
    "lime": (187, 233, 11),
    "dark_red": (114, 14, 15),
    "tan": (228, 205, 158),
    "white": (244, 244, 244),
}


@dataclass
class Brick:
    nx: int                # studs along the brick's own x (length)
    ny: int                # studs along its own y (width)
    colour: str
    x: float = 0.0         # centre, robot frame
    y: float = 0.0
    yaw: float = 0.0       # deg, brick x axis from robot X
    height: float = BRICK_H
    z: float = 0.0         # bottom
    R: np.ndarray = None   # full 3x3 orientation (a brick in a pile); None = studs up at ``yaw``

    @property
    def T(self):
        """Pose of the brick frame (origin at the bottom centre, z up through the studs)."""
        if self.R is None:
            c, s = np.cos(np.radians(self.yaw)), np.sin(np.radians(self.yaw))
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        else:
            R = self.R
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, (self.x, self.y, self.z)
        return T

    @property
    def tilt(self):
        """Angle (deg) between the brick's stud axis and robot +Z: 0 studs up, 180 upside down."""
        if self.R is None:
            return 0.0
        return float(np.degrees(np.arccos(np.clip(self.R[2, 2], -1, 1))))

    def parts(self, hollow=True):
        """The brick as primitives in its own frame (mm), shared by the physics and the renderer.

        Returns a list of ("box", centre, half_size) and ("cyl", centre, radius, half_height)
        with cylinders along the brick's z. Hollow = a real brick: top, four walls, studs,
        and the underside tubes (2-wide and up) or pins (1-wide).
        """
        L, W, H = self.length, self.width, self.height
        out = []
        if hollow:
            out.append(("box", (0, 0, H - TOP / 2), (L / 2, W / 2, TOP / 2)))
            hz = (H - TOP) / 2
            out.append(("box", ((L - WALL) / 2, 0, hz), (WALL / 2, W / 2, hz)))
            out.append(("box", (-(L - WALL) / 2, 0, hz), (WALL / 2, W / 2, hz)))
            out.append(("box", (0, (W - WALL) / 2, hz), (L / 2 - WALL, WALL / 2, hz)))
            out.append(("box", (0, -(W - WALL) / 2, hz), (L / 2 - WALL, WALL / 2, hz)))
            if self.ny >= 2 or self.nx >= 2:
                long_n, short_n = max(self.nx, self.ny), min(self.nx, self.ny)
                d = TUBE_OD if short_n >= 2 else PIN_D
                # tubes sit between stud rows; 1-wide pins sit between studs along the length
                us = (np.arange(1, self.nx) - self.nx / 2) * PITCH if self.nx > 1 else [0.0]
                vs = (np.arange(1, self.ny) - self.ny / 2) * PITCH if self.ny > 1 else [0.0]
                for u in us:
                    for v in vs:
                        out.append(("cyl", (u, v, hz), d / 2, hz))
        else:
            out.append(("box", (0, 0, H / 2), (L / 2, W / 2, H / 2)))
        u = (np.arange(self.nx) + 0.5) * PITCH - self.nx * PITCH / 2
        v = (np.arange(self.ny) + 0.5) * PITCH - self.ny * PITCH / 2
        for a in u:
            for b in v:
                out.append(("cyl", (a, b, H + STUD_H / 2), STUD_D / 2, STUD_H / 2))
        return out

    @property
    def length(self):
        return self.nx * PITCH - GAP

    @property
    def width(self):
        return self.ny * PITCH - GAP

    @property
    def rgb(self):
        return COLOURS[self.colour]

    def footprint(self):
        """Corners of the base (robot frame XY), counter-clockwise."""
        c, s = np.cos(np.radians(self.yaw)), np.sin(np.radians(self.yaw))
        hl, hw = self.length / 2, self.width / 2
        local = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
        return local @ np.array([[c, s], [-s, c]]) + [self.x, self.y]

    def stud_centres(self):
        """Stud centres in the robot frame (XY)."""
        c, s = np.cos(np.radians(self.yaw)), np.sin(np.radians(self.yaw))
        u = (np.arange(self.nx) + 0.5) * PITCH - self.nx * PITCH / 2
        v = (np.arange(self.ny) + 0.5) * PITCH - self.ny * PITCH / 2
        uu, vv = np.meshgrid(u, v)
        local = np.column_stack([uu.ravel(), vv.ravel()])
        return local @ np.array([[c, s], [-s, c]]) + [self.x, self.y]


def yaw_diff(a, b, period=180.0):
    """Smallest difference between two yaw angles with the given symmetry period."""
    d = (a - b) % period
    return min(d, period - d)


def random_layout(n, region, rng, sizes=((2, 4), (2, 2), (2, 3), (1, 2), (2, 6), (1, 4)),
                  colours=None, min_gap=8.0, max_tries=5000):
    """n non-overlapping bricks lying studs-up inside region = (xmin, xmax, ymin, ymax)."""
    colours = colours or [c for c in COLOURS if c not in ("white", "tan")]
    xmin, xmax, ymin, ymax = region
    out = []
    for _ in range(max_tries):
        if len(out) == n:
            break
        nx, ny = sizes[rng.integers(len(sizes))]
        b = Brick(nx, ny, colours[rng.integers(len(colours))],
                  rng.uniform(xmin, xmax), rng.uniform(ymin, ymax), rng.uniform(-180, 180))
        fp = b.footprint()
        if fp[:, 0].min() < xmin or fp[:, 0].max() > xmax or fp[:, 1].min() < ymin or fp[:, 1].max() > ymax:
            continue
        r = np.hypot(b.length, b.width) / 2
        if all(np.hypot(b.x - o.x, b.y - o.y) > r + np.hypot(o.length, o.width) / 2 + min_gap for o in out):
            out.append(b)
    return out
