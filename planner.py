#!/usr/bin/env python3
"""
Robot Path Planner — RRT-Connect with capsule collision detection

Replicates the Three.js FK chain from robot_config.json, then runs
RRT-Connect in joint space with capsule self-collision and obstacle checks.

Usage (CLI):
    python3 planner.py --start "0 0 0 0 0 0" --goal "30 -45 60 0 30 0"
    python3 planner.py --start "0 0 0 0 0 0" --goal "30 -45 60 0 30 0" --obstacles obstacles.json

Library usage:
    from planner import RobotPlanner
    p = RobotPlanner("robot_config.json")
    path = p.plan([0,0,0,0,0,0], [30,-45,60,0,30,0])  # degrees
    # path is list of joint-angle lists (degrees), or None if failed
"""

import argparse
import json
import math
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

def fk(joints_cfg, angles_deg):
    """
    Compute world-space positions (and orientations) of each joint frame.

    joints_cfg: list of joint dicts from robot_config.json
    angles_deg: list of joint angles in degrees (same length as joints_cfg)

    Returns list of (position_3d, quaternion_wxyz) for each joint,
    plus one extra entry for the end-effector (tip of last link chain).
    The 0-th frame is the world origin (identity).
    """
    pos = np.zeros(3)
    ori = np.array([1.0, 0.0, 0.0, 0.0])  # identity [w,x,y,z]

    frames = [(pos.copy(), ori.copy())]  # frame 0 = world origin

    for jcfg, ang_deg in zip(joints_cfg, angles_deg):
        rest_pos  = np.array(jcfg["restPos"],  dtype=float)
        rest_quat = np.array(jcfg["restQuat"], dtype=float)
        axis      = np.array(jcfg["axis"],     dtype=float)

        # Step parent frame forward by restPos, then apply restQuat
        pos = pos + qrot(ori, rest_pos)
        ori = qmul(ori, rest_quat)

        # Apply joint rotation around local axis
        joint_q = qfrom_axis_angle(axis, math.radians(ang_deg))
        ori = qmul(ori, joint_q)

        frames.append((pos.copy(), ori.copy()))

    return frames  # length = num_joints + 1

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
    Axis-aligned bounding box obstacle in world space (Z-up, metres).
    Built automatically from the viewer's listObjects response.
    """
    min: np.ndarray   # [x, y, z] lower corner
    max: np.ndarray   # [x, y, z] upper corner
    name: str = ""
    _from_viewer: bool = False


def _segment_aabb_min_dist(p0, p1, aabb_min, aabb_max, n=12):
    """Minimum distance from segment p0-p1 to AABB [aabb_min, aabb_max]."""
    min_dist = np.inf
    for k in range(n):
        t = k / (n - 1)
        pt = p0 + t * (p1 - p0)
        # Distance from point to AABB = length of over-shoot vector
        d = np.maximum(0.0, np.maximum(aabb_min - pt, pt - aabb_max))
        min_dist = min(min_dist, np.linalg.norm(d))
        if min_dist == 0.0:
            return 0.0
    return min_dist


def capsule_aabb_collide(c: Capsule, aabb: AABBObstacle) -> bool:
    dist = _segment_aabb_min_dist(c.p0, c.p1, aabb.min, aabb.max)
    return dist < c.radius


class RobotPlanner:
    """
    RRT-Connect path planner for a robot described by robot_config.json.

    Parameters
    ----------
    config_path : str
        Path to robot_config.json.
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
        config_path: str = "robot_config.json",
        capsule_radii=None,
        obstacles: list = None,
        step_deg: float = 5.0,
        max_iter: int = 5000,
        goal_bias: float = 0.10,
    ):
        with open(config_path) as f:
            cfg = json.load(f)

        self.joints_cfg = [j for j in cfg["joints"] if not j.get("fixed")]
        self.n = len(self.joints_cfg)
        self.limits = np.array([j["limits"] for j in self.joints_cfg], dtype=float)

        if capsule_radii is None:
            self._capsule_radii = [0.018] * self.n
        elif isinstance(capsule_radii, (int, float)):
            self._capsule_radii = [float(capsule_radii)] * self.n
        else:
            self._capsule_radii = list(capsule_radii)

        self.obstacles: list[Obstacle] = obstacles or []
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

        if not self._valid(start):
            print("  [planner] start config is in collision or out of limits")
            return None
        if not self._valid(goal):
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
            obs = AABBObstacle(
                min=np.array(bb['min'], dtype=float),
                max=np.array(bb['max'], dtype=float),
                name=obj.get('name', ''),
            )
            obs._from_viewer = True
            self.obstacles.append(obs)
            added += 1
        return added

    def fk_frames(self, angles_deg):
        """Return FK frames for given joint angles (degrees)."""
        return fk(self.joints_cfg, angles_deg)

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
        # Limits check
        if np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1]):
            return False

        frames = fk(self.joints_cfg, q)
        capsules = self._make_capsules(frames)

        # Self-collision: skip adjacent pairs (they share a joint)
        n = len(capsules)
        for i in range(n):
            for j in range(i + 2, n):  # skip i, i+1
                if capsules_collide(capsules[i], capsules[j]):
                    return False

        # Obstacle collision
        for cap in capsules:
            for obs in self.obstacles:
                if isinstance(obs, AABBObstacle):
                    if capsule_aabb_collide(cap, obs):
                        return False
                else:
                    if capsule_sphere_collide(cap, obs.centre, obs.radius):
                        return False

        return True

    def _make_capsules(self, frames):
        """Build one capsule per link from FK frames."""
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
    parser = argparse.ArgumentParser(description="RRT-Connect path planner for robot_config.json")
    parser.add_argument("--config", default="robot_config.json")
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
