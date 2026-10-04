#!/usr/bin/env python3
"""
Robot Path Planner — RRT-Connect with capsule collision detection

Replicates the viewer's FK chain from a device config (any serial arm), then runs
RRT-Connect in joint space with capsule self-collision and obstacle checks.

Usage (CLI):
    python3 planner.py --start "0 0 0 0 0 0" --goal "30 -45 60 0 30 0"
    python3 planner.py --start "0 0 0 0 0 0" --goal "30 -45 60 0 30 0" --obstacles obstacles.json

Library usage:
    from planner import RobotPlanner
    p = RobotPlanner("meca500_config.json")
    path = p.plan([0,0,0,0,0,0], [30,-45,60,0,30,0])  # degrees
    # path is list of joint-angle lists (degrees), or None if failed
"""

import argparse
import json
import math
import os
import random
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# ---------------------------------------------------------------------------
# Quaternion helpers  (format: [w, x, y, z]  — matches Three.js / config)
# ---------------------------------------------------------------------------

def qmul(a, b):
    """Multiply two quaternions [w,x,y,z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])

def qrot(q, v):
    """Rotate vector v by unit quaternion q [w,x,y,z]."""
    # p' = q * [0,v] * q_conj
    vq = np.array([0.0, v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    r = qmul(qmul(q, vq), qc)
    return r[1:]

def qmat(q):
    """Rotation matrix of unit quaternion q [w,x,y,z]."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def qfrom_axis_angle(axis, angle_rad):
    """Quaternion [w,x,y,z] for rotation of angle_rad around unit axis."""
    s = math.sin(angle_rad / 2)
    return np.array([math.cos(angle_rad / 2), axis[0]*s, axis[1]*s, axis[2]*s])

# ---------------------------------------------------------------------------
# Forward kinematics  (matches Three.js Object3D hierarchy)
# ---------------------------------------------------------------------------

def api_pose_to_three(position_mm, rotation_deg):
    """A pose as the viewer reports it (worldPosition in mm and worldRotation
    in degrees, both in API axis order [x, z, y] of Three.js) to a Three.js
    position (m) and quaternion [w, x, y, z]."""
    pos = np.array([position_mm[0], position_mm[2], position_mm[1]], dtype=float) / 1000.0
    ex, ey, ez = (math.radians(rotation_deg[0]), math.radians(rotation_deg[2]),
                  math.radians(rotation_deg[1]))
    # Three.js Euler 'XYZ': q = qx · qy · qz
    q = qmul(qmul(qfrom_axis_angle(np.array([1.0, 0, 0]), ex),
                  qfrom_axis_angle(np.array([0, 1.0, 0]), ey)),
             qfrom_axis_angle(np.array([0, 0, 1.0]), ez))
    return pos, q


def qinv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def fk_world(joints_cfg, angles_deg):
    """
    World pose of every joint in the config, matching the viewer's chain
    (js/chain.js) for the same API angles.

    joints_cfg: the full joint list from the device config JSON, fixed
                joints included — they carry real offsets and rotations
                (a GP arm's base column is one)
    angles_deg: one angle per movable joint, in degrees, in the WebSocket
                API convention (the viewer multiplies each by its apiSign)

    Returns one (position_3d, quaternion_wxyz) per config joint: the frame
    its link meshes hang in (the viewer's jointRotGroups[i]).
    """
    angles = iter(angles_deg)
    world = []

    for i, jcfg in enumerate(joints_cfg):
        parent = jcfg.get("parent", i - 1)
        if parent < 0:
            pos, ori = np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
        else:
            pos, ori = world[parent]

        # Step the parent frame forward by restPos, then apply restQuat
        pos = pos + qrot(ori, np.array(jcfg["restPos"], dtype=float))
        ori = qmul(ori, np.array(jcfg["restQuat"], dtype=float))

        # Fixed joints stay at rest; movable ones turn about their local axis
        if jcfg.get("fixed"):
            ang_deg = 0.0
        else:
            ang_deg = jcfg.get("apiSign", 1) * next(angles)
        axis = np.array(jcfg["axis"], dtype=float)
        ori = qmul(ori, qfrom_axis_angle(axis, math.radians(ang_deg)))

        world.append((pos, ori))

    return world


def fk(joints_cfg, angles_deg):
    """
    Frames of the movable joints: frame 0 is the world origin, then one
    (position_3d, quaternion_wxyz) per movable joint, in config order.
    See fk_world for the arguments.
    """
    world = fk_world(joints_cfg, angles_deg)
    frames = [(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))]
    for jcfg, (pos, ori) in zip(joints_cfg, world):
        if not jcfg.get("fixed"):
            frames.append((pos.copy(), ori.copy()))
    return frames  # length = movable joints + 1


# ---------------------------------------------------------------------------
# Capsule collision
# ---------------------------------------------------------------------------

@dataclass
class Capsule:
    """A capsule defined by two endpoints and a radius (all in metres)."""
    p0: np.ndarray
    p1: np.ndarray
    radius: float

    def closest_point_on_segment(self, point):
        """Closest point on segment p0-p1 to `point`."""
        d = self.p1 - self.p0
        t = np.dot(point - self.p0, d)
        len2 = np.dot(d, d)
        if len2 < 1e-12:
            return self.p0.copy()
        t = max(0.0, min(1.0, t / len2))
        return self.p0 + t * d


def _seg_seg_dist(p0, p1, q0, q1):
    """
    Minimum distance between two line segments p0-p1 and q0-q1.
    Returns (distance, t_p, t_q) where t is parameter in [0,1].
    """
    d1 = p1 - p0
    d2 = q1 - q0
    r  = p0 - q0
    a  = np.dot(d1, d1)
    e  = np.dot(d2, d2)
    f  = np.dot(d2, r)

    if a < 1e-12 and e < 1e-12:
        return np.linalg.norm(r), 0.0, 0.0

    if a < 1e-12:
        tp, tq = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = np.dot(d1, r)
        if e < 1e-12:
            tq, tp = 0.0, max(0.0, min(1.0, -c / a))
        else:
            b  = np.dot(d1, d2)
            denom = a * e - b * b
            if abs(denom) > 1e-12:
                tp = max(0.0, min(1.0, (b * f - c * e) / denom))
            else:
                tp = 0.0
            tq = (b * tp + f) / e
            if tq < 0.0:
                tq, tp = 0.0, max(0.0, min(1.0, -c / a))
            elif tq > 1.0:
                tq, tp = 1.0, max(0.0, min(1.0, (b - c) / a))

    cp = p0 + tp * d1
    cq = q0 + tq * d2
    return np.linalg.norm(cp - cq), tp, tq


def capsules_collide(c1: Capsule, c2: Capsule) -> bool:
    dist, _, _ = _seg_seg_dist(c1.p0, c1.p1, c2.p0, c2.p1)
    return dist < (c1.radius + c2.radius)


def capsule_sphere_collide(c: Capsule, centre: np.ndarray, radius: float) -> bool:
    cp = c.closest_point_on_segment(centre)
    return np.linalg.norm(cp - centre) < (c.radius + radius)


# ---------------------------------------------------------------------------
# Capsule fitting  (headless/fit-capsules.mjs, for payload the viewer carries)
# ---------------------------------------------------------------------------

def _segment_distances(pts, a, b):
    """Distance from each of pts (N x 3) to segment a-b."""
    ab = b - a
    len2 = float(ab @ ab)
    t = np.clip((pts - a) @ ab / len2, 0.0, 1.0) if len2 > 0 else np.zeros(len(pts))
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


def _capsule_volume(r, length):
    return math.pi * r * r * length + 4.0 / 3.0 * math.pi * r ** 3


def _capsule_along(pts, centre, axis):
    """The smallest-volume enclosing capsule along one axis: the segment spans
    the projections, pulled in from both ends by s; the radius is then the
    farthest point from the segment. s is scanned."""
    d = pts - centre
    t = d @ axis
    perp2 = np.sum((d - t[:, None] * axis) ** 2, axis=1)
    tmin, tmax = float(t.min()), float(t.max())
    half = (tmax - tmin) / 2
    best = None
    for k in range(41):
        s = half * k / 40
        a, b = tmin + s, tmax - s
        over = np.maximum(a - t, 0.0) + np.maximum(t - b, 0.0)
        r = math.sqrt(float(np.max(perp2 + over * over)))
        volume = _capsule_volume(r, b - a)
        if best is None or volume < best[0]:
            best = (volume, centre + a * axis, centre + b * axis)
    return best


def fit_capsule(pts):
    """One capsule enclosing every point: (p0, p1, radius), rounded outward
    (endpoints to 0.1 mm, radius up to the next 0.1 mm after re-measuring)."""
    centre = pts.mean(axis=0)
    _, vecs = np.linalg.eigh((pts - centre).T @ (pts - centre))
    _, p0, p1 = min((_capsule_along(pts, centre, vecs[:, i]) for i in range(3)),
                    key=lambda c: c[0])
    p0, p1 = np.round(p0, 4), np.round(p1, 4)
    r = float(_segment_distances(pts, p0, p1).max())
    return p0, p1, math.ceil((r + 1e-6) * 1e4) / 1e4


def _as_groups(pts):
    """Triangles (T x 3 x 3) as they are; bare points (N x 3) as one-point
    groups. The fitters below split a mesh by whole groups, so every
    triangle lies in one part's shape: enclosing its three vertices, a
    convex shape encloses the triangle, which the viewer's check tests."""
    g = np.asarray(pts, dtype=np.float64)
    return g.reshape(-1, 1, 3) if g.ndim == 2 else g


def fit_parts(tris, max_parts=12, enough=1.10, min_points=32):
    """Several capsules whose union encloses the triangles (or points), as
    fitParts in headless/fit-capsules.mjs: the largest part is split at the
    median along whichever principal axis leaves the smaller total, up to
    max_parts; then the fewest parts whose total volume is within `enough`
    of the best seen are kept. Triangles are kept whole, by centroid."""
    groups = _as_groups(tris)
    # Shared vertices once, and each triangle as indices into them: a mesh
    # sent as triangles repeats every vertex about six times.
    verts, inverse = np.unique(groups.reshape(-1, 3), axis=0, return_inverse=True)
    index = inverse.reshape(len(groups), -1)

    def part(rows):
        cap = fit_capsule(verts[np.unique(index[rows])])
        return {"rows": rows, "g": groups[rows], "cap": cap,
                "volume": _capsule_volume(cap[2], np.linalg.norm(cap[1] - cap[0]))}

    parts = [part(np.arange(len(groups)))]
    history = [parts]
    while len(parts) < max_parts:
        open_ = [p for p in parts if len(p["g"]) >= 2 * min_points]
        if not open_:
            break
        big = max(open_, key=lambda p: p["volume"])
        cent = big["g"].mean(axis=1)
        centre = cent.mean(axis=0)
        _, vecs = np.linalg.eigh((cent - centre).T @ (cent - centre))
        best = None
        for i in range(3):
            t = (cent - centre) @ vecs[:, i]
            below = t < np.sort(t)[len(t) >> 1]
            if below.sum() < min_points or (~below).sum() < min_points:
                continue
            halves = [part(big["rows"][below]), part(big["rows"][~below])]
            volume = halves[0]["volume"] + halves[1]["volume"]
            if best is None or volume < best[0]:
                best = (volume, halves)
        if best is None:
            break
        parts = [p for p in parts if p is not big] + best[1]
        history.append(parts)
    total = lambda ps: sum(p["volume"] for p in ps)
    least = min(total(ps) for ps in history)
    return [p["cap"] for p in next(ps for ps in history if total(ps) <= enough * least)]


def _occupied_cells(lo, hi, origin, cell, dims):
    """Grid cells touched by any of the boxes [lo, hi] (a triangle's box
    encloses the triangle, so these cells cover the mesh)."""
    occ = np.zeros(dims, dtype=bool)
    i0 = np.clip(np.floor((lo - origin) / cell).astype(np.int64), 0, dims - 1)
    i1 = np.clip(np.floor((hi - origin) / cell).astype(np.int64), 0, dims - 1)
    small = np.all(i1 - i0 <= 1, axis=1)
    for dx in (0, 1):                       # most triangles span ≤ 2 cells a side
        for dy in (0, 1):
            for dz in (0, 1):
                ijk = np.minimum(i0[small] + [dx, dy, dz], i1[small])
                occ[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True
    for a, b in zip(i0[~small], i1[~small]):
        occ[a[0]:b[0] + 1, a[1]:b[1] + 1, a[2]:b[2] + 1] = True
    return occ


def _fill_enclosed(occ):
    """Occupied cells plus the empty ones no path of empty cells joins to
    the outside of the grid: the inside of a closed body. An open frame's
    inside joins the outside and stays empty."""
    pad = np.pad(occ, 1)
    outside = np.zeros_like(pad)
    outside[0, :, :] = outside[-1, :, :] = True
    outside[:, 0, :] = outside[:, -1, :] = True
    outside[:, :, 0] = outside[:, :, -1] = True
    outside &= ~pad
    while True:
        grown = outside.copy()
        grown[1:] |= outside[:-1]
        grown[:-1] |= outside[1:]
        grown[:, 1:] |= outside[:, :-1]
        grown[:, :-1] |= outside[:, 1:]
        grown[:, :, 1:] |= outside[:, :, :-1]
        grown[:, :, :-1] |= outside[:, :, 1:]
        grown &= ~pad
        if np.array_equal(grown, outside):
            break
        outside = grown
    return ~outside[1:-1, 1:-1, 1:-1]


def _cell_boxes(occ):
    """Cover the occupied cells with boxes [(i0, i1)] (inclusive cell
    ranges): from each cell not yet covered, grow along x, then y, then z
    while the cells stay occupied (covered or not; boxes may overlap)."""
    covered = np.zeros_like(occ)
    boxes = []
    for i, j, k in np.argwhere(occ):
        if covered[i, j, k]:
            continue
        i1 = i
        while i1 + 1 < occ.shape[0] and occ[i1 + 1, j, k]:
            i1 += 1
        j1 = j
        while j1 + 1 < occ.shape[1] and occ[i:i1 + 1, j1 + 1, k].all():
            j1 += 1
        k1 = k
        while k1 + 1 < occ.shape[2] and occ[i:i1 + 1, j:j1 + 1, k1 + 1].all():
            k1 += 1
        covered[i:i1 + 1, j:j1 + 1, k:k1 + 1] = True
        boxes.append((np.array([i, j, k]), np.array([i1, j1, k1])))
    return boxes


def fit_boxes(tris, max_parts=16, cells=48):
    """Axis-aligned boxes (in the triangles' frame) whose union encloses the
    triangles: [(centre, half)].

    The mesh is voxelised conservatively (every cell a triangle's box
    touches), with the inside of a closed body filled (_fill_enclosed: a
    solid detector is a solid block, not six slabs something could slip
    between between samples), the occupied cells are covered with boxes,
    and boxes are
    merged, the pair whose joint box adds least volume first, down to
    max_parts. Each box is then shrunk to the triangle material inside it.
    Every triangle lies in its occupied cells and every such cell in some
    box, so the union encloses the mesh. An open frame (a detector's cage)
    comes out as boxes along its bars, which cutting the mesh in two, one
    cut at a time, does not find: no single cut of a hollow box saves much."""
    groups = _as_groups(tris)
    lo_t, hi_t = groups.min(axis=1), groups.max(axis=1)
    lo, hi = lo_t.min(axis=0), hi_t.max(axis=0)
    # A ragged mesh (or a scatter of points) breaks into many small cell
    # boxes, and merging costs the cube of their number; the grid is
    # coarsened until there are few enough.
    while True:
        cell = max(float((hi - lo).max()) / cells, 1e-6)
        dims = np.maximum(np.ceil((hi - lo) / cell).astype(np.int64), 1)
        occ = _fill_enclosed(_occupied_cells(lo_t, hi_t, lo, cell, dims))
        found = _cell_boxes(occ)
        if len(found) <= 256 or cells <= 4:
            break
        cells = max(4, int(cells / 1.5))
    boxes = [(lo + a * cell, lo + (b + 1) * cell) for a, b in found]

    vol = lambda b: float(np.prod(b[1] - b[0]))
    while len(boxes) > max_parts:
        L = np.array([b[0] for b in boxes])
        H = np.array([b[1] for b in boxes])
        V = np.prod(H - L, axis=1)
        jl = np.minimum(L[:, None], L[None])
        jh = np.maximum(H[:, None], H[None])
        cost = np.prod(jh - jl, axis=2) - V[:, None] - V[None]
        np.fill_diagonal(cost, np.inf)
        a, b = np.unravel_index(np.argmin(cost), cost.shape)
        merged = (jl[a, b], jh[a, b])
        boxes = [x for k, x in enumerate(boxes) if k not in (a, b)] + [merged]

    # Shrink each box to what the triangles put in it: the union of each
    # triangle's box clipped to it. A triangle point inside a box lies in
    # that clipped part, so nothing is lost.
    out = []
    for blo, bhi in boxes:
        inside = np.all(hi_t >= blo, axis=1) & np.all(lo_t <= bhi, axis=1)
        if not inside.any():
            continue
        clo = np.maximum(lo_t[inside], blo).min(axis=0)
        chi = np.minimum(hi_t[inside], bhi).max(axis=0)
        out.append(((clo + chi) / 2, (chi - clo) / 2))
    return out


# ---------------------------------------------------------------------------
# Robot planner
# ---------------------------------------------------------------------------

@dataclass
class Obstacle:
    """Sphere obstacle in world space (metres)."""
    centre: np.ndarray
    radius: float
    name: str = ""
    _from_viewer: bool = False


@dataclass
class AABBObstacle:
    """
    Axis-aligned bounding box obstacle in world space (metres), in the same
    Y-up frame as fk(). Built automatically from the viewer's listObjects
    response, which reports Z-up and is converted by sync_from_viewer_objects.
    """
    min: np.ndarray   # [x, y, z] lower corner
    max: np.ndarray   # [x, y, z] upper corner
    name: str = ""
    _from_viewer: bool = False


class CapsuleSetObstacle:
    """Another device, fixed in place: the capsules around its links (their
    fitted parts), with each capsule's bounding box precomputed so a test
    measures only the few capsules near the body."""

    def __init__(self, capsules, names, name=""):
        self.capsules = list(capsules)
        self.names = list(names)
        self.name = name
        self._from_viewer = False
        self.lo = np.array([np.minimum(c.p0, c.p1) - c.radius for c in self.capsules]).reshape(-1, 3)
        self.hi = np.array([np.maximum(c.p0, c.p1) + c.radius for c in self.capsules]).reshape(-1, 3)

    def _near(self, lo, hi):
        return np.nonzero(np.all(self.hi >= lo, axis=1) & np.all(self.lo <= hi, axis=1))[0]

    def hit_by_capsule(self, cap):
        """Name of the first capsule `cap` meets, or None."""
        lo = np.minimum(cap.p0, cap.p1) - cap.radius
        hi = np.maximum(cap.p0, cap.p1) + cap.radius
        for k in self._near(lo, hi):
            if capsules_collide(cap, self.capsules[k]):
                return self.names[k]
        return None

    def hit_by_box(self, box):
        """Name of the first capsule the OrientedBox `box` meets, or None."""
        for k in self._near(box.lo, box.hi):
            if box.meets_capsule(self.capsules[k]):
                return self.names[k]
        return None


class OrientedBox:
    """A box in world space (metres): centre, unit axes (the columns of
    `axes`) and half-extents along them, with its world AABB precomputed."""

    def __init__(self, centre, axes, half):
        self.centre = np.asarray(centre, dtype=float)
        self.axes = np.asarray(axes, dtype=float)
        self.half = np.asarray(half, dtype=float)
        reach = np.abs(self.axes) @ self.half
        self.lo, self.hi = self.centre - reach, self.centre + reach

    def to_local(self, pts):
        """Points (N x 3 or 3) in the box's frame, where it spans [-half, half]."""
        return (np.asarray(pts, dtype=float) - self.centre) @ self.axes

    def overlaps_aabb(self, lo, hi):
        """Separating-axis test against an axis-aligned box (touching counts)."""
        ha, t = (hi - lo) / 2, self.centre - (lo + hi) / 2
        R, hb = self.axes, self.half        # R[i, j]: world axis i · box axis j
        A = np.abs(R) + 1e-12
        if np.any(np.abs(t) > ha + A @ hb):
            return False
        if np.any(np.abs(t @ R) > ha @ A + hb):
            return False
        for i in range(3):                  # world axis i × box axis j
            i1, i2 = (i + 1) % 3, (i + 2) % 3
            for j in range(3):
                j1, j2 = (j + 1) % 3, (j + 2) % 3
                ra = ha[i1] * A[i2, j] + ha[i2] * A[i1, j]
                rb = hb[j1] * A[i, j2] + hb[j2] * A[i, j1]
                if abs(t[i2] * R[i1, j] - t[i1] * R[i2, j]) > ra + rb:
                    return False
        return True

    def meets_capsule(self, cap):
        if not _aabb_overlap(self.lo, self.hi, np.minimum(cap.p0, cap.p1) - cap.radius,
                             np.maximum(cap.p0, cap.p1) + cap.radius):
            return False
        p0, p1 = self.to_local([cap.p0, cap.p1])
        return _segment_aabb_min_dist(p0, p1, -self.half, self.half) < cap.radius

    def meets_sphere(self, centre, radius):
        over = np.maximum(0.0, np.abs(self.to_local(centre)) - self.half)
        return float(over @ over) < radius * radius


@dataclass
class CarriedCapsule:
    """A capsule around payload carried by a link of the planning device (a
    detector on the flange, say), in that link's joint frame."""
    name: str
    joint: int
    p0: np.ndarray
    p1: np.ndarray
    radius: float


@dataclass
class CarriedBox:
    """A box around carried payload, in its link's joint frame: centre, unit
    axes (columns) and half-extents."""
    name: str
    joint: int
    centre: np.ndarray
    axes: np.ndarray
    half: np.ndarray


# The viewer counts a point cloud as touching a mesh when a point lies
# within this distance of the mesh surface (js/point-grid.js).
POINT_CLOUD_CONTACT = 0.04


class PointCloudObstacle:
    """
    A point cloud (a scan) in world space, tested the way the viewer tests it:
    contact when a point lies within POINT_CLOUD_CONTACT of the body. Against
    a capsule that encloses a link's mesh, "within r + POINT_CLOUD_CONTACT of
    the capsule's axis" covers every point the viewer could count, so the
    planner never passes what the viewer would reject.

    The points are indexed in a uniform grid; a test visits only the cells
    near the body, then measures the actual points (no cell-size margin).
    """

    MAX_CELLS = 20_000_000

    def __init__(self, points, name="", cell=0.05, contact=POINT_CLOUD_CONTACT):
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        self.name = name
        self.contact = contact
        self.points = pts
        self.lo = pts.min(axis=0) if len(pts) else np.zeros(3)
        extent = (pts.max(axis=0) - self.lo) if len(pts) else np.zeros(3)
        while np.prod(np.floor(extent / cell) + 1) > self.MAX_CELLS:
            cell *= 1.5
        self.cell = cell
        self.dims = (np.floor(extent / cell) + 1).astype(np.int64)
        ijk = np.floor((pts - self.lo) / cell).astype(np.int64)
        ids = (ijk[:, 0] * self.dims[1] + ijk[:, 1]) * self.dims[2] + ijk[:, 2]
        self.order = np.argsort(ids, kind="stable")
        counts = np.bincount(ids, minlength=int(np.prod(self.dims)))
        self.starts = np.concatenate([[0], np.cumsum(counts)])
        self.occupied = (counts > 0).reshape(tuple(self.dims))

    def _points_near_box(self, lo, hi):
        """Points in the cells overlapping the box [lo, hi] (a superset)."""
        i0 = np.maximum(np.floor((lo - self.lo) / self.cell).astype(np.int64), 0)
        i1 = np.minimum(np.floor((hi - self.lo) / self.cell).astype(np.int64), self.dims - 1)
        if np.any(i1 < i0):
            return None
        sub = self.occupied[i0[0]:i1[0] + 1, i0[1]:i1[1] + 1, i0[2]:i1[2] + 1]
        cells = np.argwhere(sub)
        if len(cells) == 0:
            return None
        cells += i0
        ids = (cells[:, 0] * self.dims[1] + cells[:, 1]) * self.dims[2] + cells[:, 2]
        begin, end = self.starts[ids], self.starts[ids + 1]
        n = end - begin
        idx = np.repeat(begin - np.concatenate([[0], np.cumsum(n)[:-1]]), n) + np.arange(n.sum())
        return self.points[self.order[idx]]

    def hits_capsule(self, cap):
        reach = cap.radius + self.contact
        lo = np.minimum(cap.p0, cap.p1) - reach
        hi = np.maximum(cap.p0, cap.p1) + reach
        pts = self._points_near_box(lo, hi)
        if pts is None:
            return False
        d = cap.p1 - cap.p0
        L2 = float(d @ d)
        t = np.clip((pts - cap.p0) @ d / L2, 0.0, 1.0) if L2 > 0 else np.zeros(len(pts))
        dist2 = np.sum((pts - (cap.p0 + t[:, None] * d)) ** 2, axis=1)
        return bool(np.any(dist2 <= reach * reach))

    def hits_box(self, box):
        """Any point within the contact distance of the OrientedBox `box`?"""
        pts = self._points_near_box(box.lo - self.contact, box.hi + self.contact)
        if pts is None:
            return False
        over = np.maximum(0.0, np.abs(box.to_local(pts)) - box.half)
        return bool(np.any(np.sum(over * over, axis=1) <= self.contact ** 2))


# Clouds already indexed, by (object id, world pose): indexing millions of
# points takes seconds, and a scan rarely moves between plans.
_CLOUD_CACHE: dict = {}


def point_cloud_obstacle(obj, local_points):
    """A PointCloudObstacle for one listObjects entry and its local points
    (the viewer's exportObjectPoints, in the object's frame)."""
    key = (obj["id"], tuple(obj["worldPosition"]), tuple(obj["worldRotation"]), tuple(obj["scale"]))
    if key not in _CLOUD_CACHE:
        pos, q = api_pose_to_three(obj["worldPosition"], obj["worldRotation"])
        R = qmat(q)
        pts = np.asarray(local_points, dtype=np.float64).reshape(-1, 3) * np.asarray(obj["scale"], float)
        if len(_CLOUD_CACHE) >= 4:
            _CLOUD_CACHE.pop(next(iter(_CLOUD_CACHE)))
        _CLOUD_CACHE[key] = PointCloudObstacle(pts @ R.T + pos, obj.get("name", ""))
    cloud = _CLOUD_CACHE[key]
    cloud._from_viewer = True
    return cloud


# Shapes fitted to carried meshes, by (object id, vertex count, the
# object's scale): fitting takes a second or so for a detailed mesh, and
# depends only on the shape, not on where the arm has taken it.
_PAYLOAD_FIT_CACHE: dict = {}


def payload_shapes(obj, local_vertices):
    """World shapes (Three.js frame, metres) whose union encloses a carried
    mesh: boxes in its own axes (fit_boxes), or capsules (fit_parts),
    whichever encloses less volume. Boxes suit a detector and its cage;
    capsules a long or bent tool. `obj` is its listObjects entry, which must
    carry matrixWorld; `local_vertices` are its triangles, three vertices
    each (the viewer's exportObjectPoints with vertices: true).

    matrixWorld = R·S plus a translation (polar decomposition). The fit is
    made on the triangles scaled by S and carried out by R and the
    translation, so it is reused for any pose of the same object. S keeps
    the mesh's own axes, bar a shear from a stretched parent, which leaves
    the boxes enclosing but less snug."""
    M = np.asarray(obj["matrixWorld"], dtype=float).reshape(4, 4).T
    U, sigma, Vt = np.linalg.svd(M[:3, :3])
    R, S = U @ Vt, Vt.T @ np.diag(sigma) @ Vt
    verts = np.asarray(local_vertices, dtype=np.float64).reshape(-1, 3)
    key = (obj["id"], len(verts), tuple(np.round(S, 9).ravel()), tuple(obj.get("origin") or ()))
    if key not in _PAYLOAD_FIT_CACHE:
        if len(_PAYLOAD_FIT_CACHE) >= 16:
            _PAYLOAD_FIT_CACHE.pop(next(iter(_PAYLOAD_FIT_CACHE)))
        scaled = verts @ S.T
        tris = scaled.reshape(-1, 3, 3) if len(scaled) % 3 == 0 else scaled
        # Out by a micrometre: float32 vertices, float64 here.
        boxes = [(c, h + 1e-6) for c, h in fit_boxes(tris)]
        box_volume = sum(8 * np.prod(h) for _, h in boxes)
        caps = fit_parts(tris)
        cap_volume = sum(_capsule_volume(r, np.linalg.norm(p1 - p0)) for p0, p1, r in caps)
        _PAYLOAD_FIT_CACHE[key] = ("boxes", boxes) if box_volume <= cap_volume else ("capsules", caps)
    kind, fit = _PAYLOAD_FIT_CACHE[key]
    t = M[:3, 3]
    if kind == "boxes":
        return [OrientedBox(R @ c + t, R, h) for c, h in fit]
    return [Capsule(R @ p0 + t, R @ p1 + t, r) for p0, p1, r in fit]


def _aabb_overlap(lo1, hi1, lo2, hi2):
    return bool(np.all(hi1 >= lo2) and np.all(hi2 >= lo1))


def _segment_aabb_min_dist(p0, p1, aabb_min, aabb_max):
    """Minimum distance from segment p0-p1 to AABB [aabb_min, aabb_max].

    Exact to float precision: distance to a convex set is convex along a
    line, so a ternary search over the segment parameter finds its minimum.
    (Sampling points along the segment instead would overstate the distance
    between samples, and let a small box slip past a long capsule.)
    """
    d = p1 - p0

    def dist(t):
        pt = p0 + t * d
        return float(np.linalg.norm(np.maximum(0.0, np.maximum(aabb_min - pt, pt - aabb_max))))

    lo, hi = 0.0, 1.0
    for _ in range(60):
        m1 = lo + (hi - lo) / 3.0
        m2 = hi - (hi - lo) / 3.0
        if dist(m1) <= dist(m2):
            hi = m2
        else:
            lo = m1
    return min(dist(lo), dist(hi))


def capsule_aabb_collide(c: Capsule, aabb: AABBObstacle) -> bool:
    # Cheap reject: the capsule's own bounding box misses the obstacle's.
    lo = np.minimum(c.p0, c.p1) - c.radius
    hi = np.maximum(c.p0, c.p1) + c.radius
    if np.any(hi < aabb.min) or np.any(lo > aabb.max):
        return False
    return _segment_aabb_min_dist(c.p0, c.p1, aabb.min, aabb.max) < c.radius


def _lowest(shape):
    """Lowest height (world Y) of a Capsule or OrientedBox."""
    if isinstance(shape, OrientedBox):
        return shape.lo[1]
    return min(shape.p0[1], shape.p1[1]) - shape.radius


def _meets_capsule(shape, cap):
    """Does a Capsule or OrientedBox meet the capsule `cap`?"""
    if isinstance(shape, OrientedBox):
        return shape.meets_capsule(cap)
    return capsules_collide(shape, cap)


def _obstacle_hit(shape, obs):
    """What a Capsule or OrientedBox meets in `obs`: the obstacle's name (a
    device's part, for another device), or None."""
    name = obs.name or "obstacle"
    if isinstance(obs, CapsuleSetObstacle):
        return obs.hit_by_box(shape) if isinstance(shape, OrientedBox) else obs.hit_by_capsule(shape)
    if isinstance(shape, OrientedBox):
        if isinstance(obs, AABBObstacle):
            hit = shape.overlaps_aabb(obs.min, obs.max)
        elif isinstance(obs, PointCloudObstacle):
            hit = obs.hits_box(shape)
        else:
            hit = shape.meets_sphere(obs.centre, obs.radius)
    elif isinstance(obs, AABBObstacle):
        hit = capsule_aabb_collide(shape, obs)
    elif isinstance(obs, PointCloudObstacle):
        hit = obs.hits_capsule(shape)
    else:
        hit = capsule_sphere_collide(shape, obs.centre, obs.radius)
    return name if hit else None


class RobotPlanner:
    """
    RRT-Connect path planner for a robot described by a device config JSON.

    Parameters
    ----------
    config_path : str
        Path to device config JSON (e.g. meca500_config.json).
    capsule_radii : list[float] | float | None
        Per-link capsule radius (metres). If a single float, used for all links.
        If None, defaults to 18 mm for all links (tuned for Meca500 geometry).
    obstacles : list[Obstacle]
        Sphere obstacles in world space.
    step_deg : float
        RRT extension step size in degrees (joint-space Linf norm).
    max_iter : int
        Maximum RRT-Connect iterations.
    goal_bias : float
        Probability of sampling the goal directly (0–1).
    """

    def __init__(
        self,
        config_path: str = "meca500_config.json",
        capsule_radii=None,
        obstacles: list = None,
        step_deg: float = 5.0,
        max_iter: int = 5000,
        goal_bias: float = 0.10,
    ):
        with open(config_path) as f:
            cfg = json.load(f)
        self.config_path = config_path
        self._config = cfg

        self.all_joints = cfg["joints"]
        self.joints_cfg = [j for j in self.all_joints if not j.get("fixed")]
        self.n = len(self.joints_cfg)
        # Config limits are in the model's convention; the planner works in
        # API angles, so a joint with apiSign −1 has its range mirrored.
        self.limits = np.array([
            j["limits"] if j.get("apiSign", 1) > 0 else [-j["limits"][1], -j["limits"][0]]
            for j in self.joints_cfg
        ], dtype=float)

        # The arm's collision shapes. Fitted capsules (one per link, enclosing
        # its mesh; see headless/fit-capsules.mjs) make this check stricter
        # than the viewer's, never looser. Without them, or when radii are
        # given explicitly, it falls back to thin joint-to-joint capsules.
        capsule_file = config_path.replace("_config.json", "_capsules.json")
        if capsule_radii is None and capsule_file != config_path and os.path.exists(capsule_file):
            with open(capsule_file) as f:
                fitted = json.load(f)
            self.capsule_source = "fitted"
            self._links = [(l["name"], l["joint"], np.array(l["p0"], dtype=float),
                            np.array(l["p1"], dtype=float), float(l["radius"]))
                           for l in fitted["links"]]
            # The tighter set per link (its union also encloses the link),
            # used when this device is an obstacle to another.
            self._parts = [(l["name"], l["joint"],
                            [(np.array(c["p0"], dtype=float), np.array(c["p1"], dtype=float),
                              float(c["radius"])) for c in l.get("parts") or [l]])
                           for l in fitted["links"]]
            # Self-collision is left to the exact check on the finished path.
            # Links two apart (L1–L3, L2–L4) nest into each other at a compact
            # elbow or wrist, so capsules that enclose them overlap in nearly
            # every pose while the meshes almost never touch (measured: Meca500
            # 300/300 poses vs 2/300, GP280 300/300 vs 0/300, and still 244/300
            # with four capsules per link). Checked here, they would reject
            # every start pose.
            self._self_pairs = []
            # A link resting on the floor by design (a base) would otherwise
            # always report a floor contact; only the others are checked.
            home = self._link_capsules(np.zeros(self.n))
            self._floor_checked = [c.p0[1] - c.radius >= 0 and c.p1[1] - c.radius >= 0 for c in home]
        else:
            self.capsule_source = "joint-to-joint"
            if capsule_radii is None:
                self._capsule_radii = [0.018] * self.n
            elif isinstance(capsule_radii, (int, float)):
                self._capsule_radii = [float(capsule_radii)] * self.n
            else:
                self._capsule_radii = list(capsule_radii)
            # Neighbouring capsules share a joint, so only pairs two apart count.
            self._self_pairs = [(i, j) for i in range(self.n) for j in range(i + 2, self.n)]
            self._floor_checked = [False] * self.n

        self.obstacles: list = obstacles or []
        # Where the device stands in the world (Three.js frame). Its capsules
        # are computed in its own frame and carried out by this pose.
        self.base_pos = np.zeros(3)
        self.base_quat = np.array([1.0, 0.0, 0.0, 0.0])
        # Payload carried by its links (CarriedCapsule / CarriedBox, in the
        # joint frame), and whether each shape is kept off the floor.
        self.carried: list = []
        self._carried_floor: list = []
        # Which carried shapes are checked against which of the arm's own
        # link parts: (carried index, part index), chosen at sync time.
        self._payload_pairs: list = []
        self.step_deg   = step_deg
        self.max_iter   = max_iter
        self.goal_bias  = goal_bias

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(self, start_deg, goal_deg, verbose=True):
        """
        Plan a collision-free path from start to goal (both in degrees).

        Returns list of joint-angle lists (degrees) from start to goal,
        or None if no path found within max_iter iterations.
        """
        start = np.array(start_deg, dtype=float)
        goal  = np.array(goal_deg,  dtype=float)

        # These respect verbose like every other message here: a library caller
        # (the MCP server speaks JSON-RPC over stdout) cannot afford stray prints.
        if not self._valid(start):
            if verbose:
                print("  [planner] start config is in collision or out of limits")
            return None
        if not self._valid(goal):
            if verbose:
                print("  [planner] goal config is in collision or out of limits")
            return None

        t0 = time.time()

        # Each tree: list of (config, parent_index)
        tree_a = [(start, -1)]
        tree_b = [(goal,  -1)]

        for i in range(self.max_iter):
            # Bias toward goal of tree_b
            if random.random() < self.goal_bias:
                q_rand = tree_b[0][0].copy()
            else:
                q_rand = self._sample()

            # Extend tree_a toward q_rand
            result_a = self._extend(tree_a, q_rand)
            if result_a == "trapped":
                tree_a, tree_b = tree_b, tree_a
                continue

            # Try to connect tree_b to the new node in tree_a
            q_new = tree_a[-1][0]
            result_b = self._connect(tree_b, q_new)

            if result_b == "reached":
                path = self._extract_path(tree_a, tree_b)
                path = self._smooth(path)
                elapsed = time.time() - t0
                if verbose:
                    print(f"  [planner] found path: {len(path)} waypoints "
                          f"in {i+1} iterations ({elapsed:.2f}s)")
                return path

            tree_a, tree_b = tree_b, tree_a

        elapsed = time.time() - t0
        if verbose:
            print(f"  [planner] failed after {self.max_iter} iterations ({elapsed:.2f}s)")
        return None

    def add_obstacle(self, centre, radius, name=""):
        """Add a sphere obstacle (centre in metres, radius in metres)."""
        self.obstacles.append(Obstacle(np.array(centre, dtype=float), radius, name))

    def remove_obstacle(self, name):
        self.obstacles = [o for o in self.obstacles if o.name != name]

    def sync_from_viewer_objects(self, objects_data):
        """
        Rebuild viewer-sourced obstacles from a listObjects response.

        objects_data: the 'objects' list from the viewer's {"type":"objects",...} message.
        Each entry needs 'visible' and 'worldBB' (added in buildObjectInfo).
        Manually added obstacles (add_obstacle / --obstacles file) are preserved.

        The viewer reports worldBB in the API's Z-up convention, [x, z, y] of
        the underlying Three.js frame, while fk() here works directly from the
        config's restPos/restQuat and so stays in Three.js Y-up. The axes are
        swapped back on the way in; without that the obstacles sit in a
        different space from the arm and never register a hit.
        """
        # Drop previously synced viewer obstacles; keep manual ones
        self.obstacles = [o for o in self.obstacles if not getattr(o, '_from_viewer', False)]

        added = 0
        for obj in objects_data:
            if not obj.get('visible', True):
                continue
            bb = obj.get('worldBB')
            if bb is None:
                continue  # point cloud or no geometry
            lo, hi = bb['min'], bb['max']
            obs = AABBObstacle(
                min=np.array([lo[0], lo[2], lo[1]], dtype=float),
                max=np.array([hi[0], hi[2], hi[1]], dtype=float),
                name=obj.get('name', ''),
            )
            obs._from_viewer = True
            self.obstacles.append(obs)
            added += 1
        return added

    def sync_from_viewer(self, devices, objects, device_index, fetch_points=None,
                         fetch_vertices=None):
        """
        Take the scene from the viewer's listDevices and listObjects replies,
        for planning the device at `device_index` (its place in that list):

          * the device's world pose;
          * every other visible serial device as fixed capsules, at its
            current joints and pose (its fitted parts, whose union encloses it);
          * objects parented to a link of this device as payload carried by
            that link: a box or capsules fitted to its mesh when
            `fetch_vertices(id)` is given (it returns the mesh's vertices in
            its local frame, the viewer's exportObjectPoints with
            vertices: true), else its world box as it is now;
          * every other visible object as a fixed box;
          * visible point clouds (and PLY splats) as their points, when
            `fetch_points(id)` is given: it returns the object's collision
            points in its local frame (the viewer's exportObjectPoints).

        Returns the number of obstacles and carried objects taken in.
        """
        me = devices[device_index]
        self.set_base_pose(*api_pose_to_three(me["worldPosition"], me["worldRotation"]))
        self.obstacles = [o for o in self.obstacles if not getattr(o, '_from_viewer', False)]
        self.carried = []
        self._carried_floor = []
        self._payload_pairs = []
        here = os.path.dirname(os.path.abspath(self.config_path))

        for i, d in enumerate(devices):
            if i == device_index or not d.get("visible", True) or d.get("deviceType") != "serial":
                continue
            config = os.path.join(here, d["config"])
            if not os.path.exists(config.replace("_config.json", "_capsules.json")):
                continue        # no enclosing shapes to stand in for it
            other = RobotPlanner(config)
            other.set_base_pose(*api_pose_to_three(d["worldPosition"], d["worldRotation"]))
            parts = other.part_capsules(np.asarray(d["joints"], dtype=float))
            obs = CapsuleSetObstacle([c for _, c in parts],
                                     [f"{d['name']}:{link}" for link, _ in parts], d["name"])
            obs._from_viewer = True
            self.obstacles.append(obs)

        current = np.asarray(me["joints"], dtype=float)
        world_now = fk_world(self.all_joints, current)
        link_joint = {l["name"]: l["joint"] for l in self._config["links"]}
        n_carried = 0
        for obj in objects:
            if not obj.get("visible", True):
                continue
            if obj.get("hasCollisionPoints"):
                if fetch_points is not None:
                    self.obstacles.append(point_cloud_obstacle(obj, fetch_points(obj["id"])))
                continue
            if obj.get("worldBB") is None:
                continue
            lo, hi = obj["worldBB"]["min"], obj["worldBB"]["max"]
            lo = np.array([lo[0], lo[2], lo[1]], dtype=float)
            hi = np.array([hi[0], hi[2], hi[1]], dtype=float)
            parent = obj.get("parent") or ""
            dev_id, _, link = parent.partition(":")
            if dev_id == me["id"] and link in link_joint:
                j = link_joint[link]
                pos, ori = world_now[j]
                R_joint = qmat(qinv(ori)) @ qmat(qinv(self.base_quat))
                to_joint = lambda w: qrot(qinv(ori), qrot(qinv(self.base_quat), w - self.base_pos) - pos)
                # Carried: shapes fitted to its mesh (see payload_shapes),
                # fixed in the link's joint frame. They enclose every vertex,
                # so, as for the links, this check is stricter than the
                # viewer's. Without its vertices, its world box as it is now.
                shapes = None
                if fetch_vertices is not None and obj.get("matrixWorld"):
                    try:
                        shapes = payload_shapes(obj, fetch_vertices(obj["id"]))
                    except Exception:
                        shapes = None     # no vertices to be had; use its box
                if not shapes:
                    shapes = [OrientedBox((lo + hi) / 2, np.eye(3), (hi - lo) / 2)]
                n_carried += 1
                name = obj.get("name", "")
                for s in shapes:
                    if isinstance(s, OrientedBox):
                        self.carried.append(CarriedBox(name, j, to_joint(s.centre), R_joint @ s.axes, s.half))
                    else:
                        self.carried.append(CarriedCapsule(name, j, to_joint(s.p0), to_joint(s.p1), s.radius))
                    # Kept off the floor unless it already reaches it.
                    self._carried_floor.append(_lowest(s) >= 0)
            else:
                obs = AABBObstacle(min=lo, max=hi, name=obj.get("name", ""))
                obs._from_viewer = True
                self.obstacles.append(obs)
        self._payload_pairs = self._clear_payload_pairs(current)
        return len(self.obstacles) + n_carried

    def _clear_payload_pairs(self, q):
        """Pairs of carried shape and the arm's own link part (the fitted
        parts, whose union encloses each link) to check: those apart at pose
        q, the pose the plan starts from. A pair already touching there (the
        payload against the link that holds it, say) is left out, as a base
        on the floor is: checked, it would reject every pose."""
        if self.capsule_source != "fitted" or not self.carried:
            return []
        parts = self.part_capsules(q)
        return [(k, i) for k, (_, shape) in enumerate(self._carried_shapes(q))
                for i, (_, cap) in enumerate(parts) if not _meets_capsule(shape, cap)]

    def fk_frames(self, angles_deg):
        """Return FK frames for given joint angles (degrees)."""
        return fk(self.all_joints, angles_deg)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _sample(self):
        """Uniform random sample within joint limits."""
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        return lo + np.random.rand(self.n) * (hi - lo)

    def _clamp(self, q):
        return np.clip(q, self.limits[:, 0], self.limits[:, 1])

    def _step_toward(self, q_from, q_to):
        """
        Move from q_from toward q_to by at most step_deg (Linf).
        Returns new config and whether goal was reached.
        """
        delta = q_to - q_from
        max_d = np.max(np.abs(delta))
        if max_d < 1e-6:
            return q_to.copy(), True
        if max_d <= self.step_deg:
            return q_to.copy(), True
        return q_from + delta * (self.step_deg / max_d), False

    def _extend(self, tree, q_target):
        """Extend tree one step toward q_target. Returns 'advanced' or 'trapped'."""
        nearest_idx, q_near = self._nearest(tree, q_target)
        q_new, _ = self._step_toward(q_near, q_target)
        q_new = self._clamp(q_new)
        if self._valid(q_new):
            tree.append((q_new, nearest_idx))
            return "advanced"
        return "trapped"

    def _connect(self, tree, q_target):
        """Repeatedly extend tree toward q_target until reached or trapped."""
        while True:
            nearest_idx, q_near = self._nearest(tree, q_target)
            q_new, reached = self._step_toward(q_near, q_target)
            q_new = self._clamp(q_new)
            if not self._valid(q_new):
                return "trapped"
            tree.append((q_new, nearest_idx))
            if reached:
                return "reached"

    def _nearest(self, tree, q):
        """Find index and config of nearest node in tree (Linf distance)."""
        best_idx, best_dist = 0, np.inf
        for i, (node, _) in enumerate(tree):
            d = np.max(np.abs(node - q))
            if d < best_dist:
                best_dist, best_idx = d, i
        return best_idx, tree[best_idx][0]

    def _extract_path(self, tree_a, tree_b):
        """
        Concatenate path from tree_a root → tip and tree_b tip → root.
        Both trees grew toward each other; tree_b was swapped so we need to
        trace tree_b tip → root and reverse.
        """
        def trace(tree, idx):
            path = []
            while idx != -1:
                path.append(tree[idx][0])
                idx = tree[idx][1]
            return path[::-1]

        path_a = trace(tree_a, len(tree_a) - 1)
        path_b = trace(tree_b, len(tree_b) - 1)
        return path_a + path_b[::-1]

    def _smooth(self, path, attempts=200):
        """
        Shortcut smoothing: pick two random waypoints, replace segment with
        straight line if collision-free.
        """
        path = [p.copy() for p in path]
        for _ in range(attempts):
            if len(path) <= 2:
                break
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            if j - i <= 1:
                continue
            if self._edge_valid(path[i], path[j]):
                path = path[:i+1] + path[j:]
        return path

    def _edge_valid(self, q1, q2, steps=None):
        """Check all intermediate configs along straight edge in joint space."""
        if steps is None:
            max_d = np.max(np.abs(q2 - q1))
            steps = max(2, int(math.ceil(max_d / self.step_deg)))
        for k in range(1, steps):
            t = k / steps
            q = q1 + t * (q2 - q1)
            if not self._valid(q):
                return False
        return True

    # ------------------------------------------------------------------
    # Collision checking
    # ------------------------------------------------------------------

    def _valid(self, q):
        """Returns True if config q is within limits and collision-free."""
        return self._problem(np.asarray(q, dtype=float)) is None

    def diagnose(self, q):
        """Explain why config q is invalid, or return None if it is fine.

        _valid answers yes/no, which is all RRT needs. A caller reporting to a
        person (or an agent) needs to know which link hit what; this names the
        first failure found.
        """
        q = np.asarray(q, dtype=float)
        if len(q) != self.n:
            return f"expected {self.n} joint angles, got {len(q)}"
        return self._problem(q)

    def _problem(self, q):
        for i, (lo, hi) in enumerate(self.limits):
            if q[i] < lo or q[i] > hi:
                name = self.joints_cfg[i].get("name", f"joint {i}")
                return (f"{name} at {q[i]:.2f} deg is outside its limits "
                        f"[{lo:g}, {hi:g}]")

        capsules = self._capsules(q)
        names = self._body_names()

        for i, j in self._self_pairs:
            if capsules_collide(capsules[i], capsules[j]):
                return f"self-collision between {names[i]} and {names[j]}"

        # The links, then any payload they carry, against the floor and the
        # obstacles. (Payload against the arm is left to the exact check,
        # like the arm against itself.)
        carried = self._carried_shapes(q)
        bodies = zip(capsules + [s for _, s in carried],
                     names + [f"{name} (carried)" for name, _ in carried],
                     list(self._floor_checked) + self._carried_floor)
        for shape, name, floor_checked in bodies:
            if floor_checked and _lowest(shape) < 0:
                return f"{name} goes below the floor"
            for obs in self.obstacles:
                hit = _obstacle_hit(shape, obs)
                if hit:
                    return f"{name} collides with {hit}"

        # Payload against the arm that carries it. (The arm against itself is
        # left to the exact check: see __init__.)
        if self._payload_pairs:
            parts = self.part_capsules(q)
            for k, i in self._payload_pairs:
                name, shape = carried[k]
                if _meets_capsule(shape, parts[i][1]):
                    return f"{name} (carried) collides with {parts[i][0]}"

        return None

    def _body_names(self):
        if self.capsule_source == "fitted":
            return [l[0] for l in self._links]
        return [j.get("name", f"link {i}") for i, j in enumerate(self.joints_cfg)]

    def set_base_pose(self, position, quaternion_wxyz):
        """Place the device in the world (Three.js frame, metres)."""
        self.base_pos = np.asarray(position, dtype=float)
        self.base_quat = np.asarray(quaternion_wxyz, dtype=float)

    def _to_world(self, p):
        return self.base_pos + qrot(self.base_quat, p)

    def _capsules(self, q):
        if self.capsule_source == "fitted":
            local = self._link_capsules(q)
        else:
            local = self._make_capsules(fk(self.all_joints, q))
        return [Capsule(self._to_world(c.p0), self._to_world(c.p1), c.radius) for c in local]

    def _carried_shapes(self, q):
        """(name, world Capsule or OrientedBox) for the carried payload at pose q."""
        if not self.carried:
            return []
        world = fk_world(self.all_joints, q)
        R_base = qmat(self.base_quat)
        out = []
        for c in self.carried:
            pos, ori = world[c.joint]
            if isinstance(c, CarriedBox):
                shape = OrientedBox(self._to_world(pos + qrot(ori, c.centre)),
                                    R_base @ qmat(ori) @ c.axes, c.half)
            else:
                shape = Capsule(self._to_world(pos + qrot(ori, c.p0)),
                                self._to_world(pos + qrot(ori, c.p1)), c.radius)
            out.append((c.name, shape))
        return out

    def part_capsules(self, q):
        """(name, world capsule) for every fitted part of every link at pose q,
        placed by the base pose. Their union encloses each link's mesh."""
        world = fk_world(self.all_joints, q)
        out = []
        for name, joint, parts in self._parts:
            pos, ori = world[joint]
            for p0, p1, r in parts:
                out.append((name, Capsule(self._to_world(pos + qrot(ori, p0)),
                                          self._to_world(pos + qrot(ori, p1)), r)))
        return out

    def _link_capsules(self, q):
        """Each link's fitted capsule, placed by its joint's world frame."""
        world = fk_world(self.all_joints, q)
        caps = []
        for _, joint, p0, p1, r in self._links:
            pos, ori = world[joint]
            caps.append(Capsule(pos + qrot(ori, p0), pos + qrot(ori, p1), r))
        return caps

    def _make_capsules(self, frames):
        """Joint-to-joint capsules from FK frames (no fitted capsules)."""
        capsules = []
        for i in range(self.n):
            p0 = frames[i][0]
            p1 = frames[i + 1][0]
            r  = self._capsule_radii[i]
            capsules.append(Capsule(p0.copy(), p1.copy(), r))
        return capsules


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_angles(s):
    return [float(x) for x in s.split()]


def main():
    parser = argparse.ArgumentParser(description="RRT-Connect path planner for robot configs")
    parser.add_argument("--config", default="meca500_config.json")
    parser.add_argument("--start", required=True, help='Joint angles in degrees e.g. "0 0 0 0 0 0"')
    parser.add_argument("--goal",  required=True, help='Goal joint angles in degrees')
    parser.add_argument("--obstacles", default=None,
                        help='JSON file: [{"centre":[x,y,z],"radius":r,"name":"..."},...] (metres)')
    parser.add_argument("--step",  type=float, default=5.0,  help="Step size in degrees (default 5)")
    parser.add_argument("--iter",  type=int,   default=5000, help="Max iterations (default 5000)")
    parser.add_argument("--radius", type=float, default=0.018, help="Capsule radius in metres (default 0.018)")
    parser.add_argument("--smooth", type=int,  default=200,  help="Smoothing passes (default 200)")
    parser.add_argument("--output", default=None, help="Save path to JSON file")
    args = parser.parse_args()

    obstacles = []
    if args.obstacles:
        with open(args.obstacles) as f:
            for o in json.load(f):
                obstacles.append(Obstacle(
                    np.array(o["centre"]), o["radius"], o.get("name", "")
                ))

    planner = RobotPlanner(
        config_path=args.config,
        capsule_radii=args.radius,
        obstacles=obstacles,
        step_deg=args.step,
        max_iter=args.iter,
    )
    planner._smooth_attempts = args.smooth

    start = _parse_angles(args.start)
    goal  = _parse_angles(args.goal)

    print(f"Planning: {start} → {goal}")
    path = planner.plan(start, goal)

    if path is None:
        print("No path found.")
        return

    print(f"\nPath ({len(path)} waypoints):")
    for i, q in enumerate(path):
        print(f"  [{i:3d}] " + "  ".join(f"{a:7.2f}" for a in q))

    if args.output:
        with open(args.output, "w") as f:
            json.dump([[round(a, 4) for a in q] for q in path], f, indent=2)
        print(f"\nSaved to {args.output}")


if __name__ == "__main__":
    main()
