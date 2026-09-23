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

    def hit_by_box(self, lo, hi):
        box = AABBObstacle(min=lo, max=hi)
        for k in self._near(lo, hi):
            if capsule_aabb_collide(self.capsules[k], box):
                return self.names[k]
        return None


@dataclass
class AttachedBox:
    """A box carried by a link of the planning device (payload such as a
    detector on the flange), in that link's joint frame."""
    joint: int
    corners: np.ndarray        # 8 x 3, joint frame, metres
    name: str = ""


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

    def hits_box(self, lo, hi):
        """Any point within the contact distance of the box [lo, hi]?"""
        pts = self._points_near_box(lo - self.contact, hi + self.contact)
        if pts is None:
            return False
        over = np.maximum(0.0, np.maximum(lo - pts, pts - hi))
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
        w, x, y, z = q
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        pts = np.asarray(local_points, dtype=np.float64).reshape(-1, 3) * np.asarray(obj["scale"], float)
        if len(_CLOUD_CACHE) >= 4:
            _CLOUD_CACHE.pop(next(iter(_CLOUD_CACHE)))
        _CLOUD_CACHE[key] = PointCloudObstacle(pts @ R.T + pos, obj.get("name", ""))
    cloud = _CLOUD_CACHE[key]
    cloud._from_viewer = True
    return cloud


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
        self.attached: list[AttachedBox] = []
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

    def sync_from_viewer(self, devices, objects, device_index, fetch_points=None):
        """
        Take the scene from the viewer's listDevices and listObjects replies,
        for planning the device at `device_index` (its place in that list):

          * the device's world pose;
          * every other visible serial device as fixed capsules, at its
            current joints and pose (its fitted parts, whose union encloses it);
          * objects parented to a link of this device as payload carried by
            that link; every other visible object as a fixed box;
          * visible point clouds (and PLY splats) as their points, when
            `fetch_points(id)` is given: it returns the object's collision
            points in its local frame (the viewer's exportObjectPoints).

        Returns the number of obstacles and carried objects taken in.
        """
        me = devices[device_index]
        self.set_base_pose(*api_pose_to_three(me["worldPosition"], me["worldRotation"]))
        self.obstacles = [o for o in self.obstacles if not getattr(o, '_from_viewer', False)]
        self.attached = []
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
                # Carried: its box, fixed in the link's joint frame.
                j = link_joint[link]
                pos, ori = world_now[j]
                corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                                    for z in (lo[2], hi[2])])
                to_joint = [qrot(qinv(ori), qrot(qinv(self.base_quat), c - self.base_pos) - pos)
                            for c in corners]
                self.attached.append(AttachedBox(j, np.array(to_joint), obj.get("name", "")))
            else:
                obs = AABBObstacle(min=lo, max=hi, name=obj.get("name", ""))
                obs._from_viewer = True
                self.obstacles.append(obs)
        return len(self.obstacles) + len(self.attached)

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

        for i, cap in enumerate(capsules):
            if self._floor_checked[i] and min(cap.p0[1], cap.p1[1]) - cap.radius < 0:
                return f"{names[i]} goes below the floor"
            for obs in self.obstacles:
                if isinstance(obs, AABBObstacle):
                    hit = capsule_aabb_collide(cap, obs)
                elif isinstance(obs, CapsuleSetObstacle):
                    part = obs.hit_by_capsule(cap)
                    if part:
                        return f"{names[i]} collides with {part}"
                    hit = False
                elif isinstance(obs, PointCloudObstacle):
                    hit = obs.hits_capsule(cap)
                else:
                    hit = capsule_sphere_collide(cap, obs.centre, obs.radius)
                if hit:
                    return f"{names[i]} collides with {obs.name or 'obstacle'}"

        for lo, hi, name in self._attached_boxes(q):
            for obs in self.obstacles:
                if isinstance(obs, AABBObstacle):
                    hit = _aabb_overlap(lo, hi, obs.min, obs.max)
                elif isinstance(obs, CapsuleSetObstacle):
                    part = obs.hit_by_box(lo, hi)
                    if part:
                        return f"{name} (carried) collides with {part}"
                    hit = False
                elif isinstance(obs, PointCloudObstacle):
                    hit = obs.hits_box(lo, hi)
                else:
                    hit = capsule_aabb_collide(Capsule(obs.centre, obs.centre, obs.radius),
                                               AABBObstacle(min=lo, max=hi))
                if hit:
                    return f"{name} (carried) collides with {obs.name or 'obstacle'}"

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

    def _attached_boxes(self, q):
        """World AABBs of the attached payload at pose q."""
        if not self.attached:
            return []
        world = fk_world(self.all_joints, q)
        boxes = []
        for box in self.attached:
            pos, ori = world[box.joint]
            pts = np.array([self._to_world(pos + qrot(ori, c)) for c in box.corners])
            boxes.append((pts.min(axis=0), pts.max(axis=0), box.name))
        return boxes

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
