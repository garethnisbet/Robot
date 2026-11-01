"""The Meca500's link meshes, posed by joint angles, in the robot frame (mm).

Built from the viewer's own config and GLB (VRRobotController), placing each link
the way the viewer does: joint i is a fixed rest transform followed by a rotation
about its axis; each link mesh keeps its rest-pose (all joints 0) placement relative
to its joint; meshes under no link (the base casting) are fixed to the root.
"""
import json
from pathlib import Path

import cv2
import numpy as np
import trimesh

from .geometry import project
from .simserver import VIEWER_DIR


def _quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _axis_angle(axis, a):
    return cv2.Rodrigues(np.asarray(axis, float) / np.linalg.norm(axis) * a)[0]


# three.js world (metres, Y up) -> robot frame (mm, Z up): inverse of sim.robot_to_three
THREE_TO_ROBOT = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]]) * \
    np.array([[1000], [1000], [1000], [1]])


class RobotModel:
    def __init__(self, config=None, max_faces_per_link=None):
        config = Path(config or VIEWER_DIR / "meca500_config.json")
        self.cfg = json.loads(config.read_text())
        self.joints = self.cfg["joints"]
        self.n = len(self.joints)
        self.sign = np.array([j.get("apiSign", 1) for j in self.joints], float)
        link_joint = {l["name"]: l["joint"] for l in self.cfg["links"]}

        sc = trimesh.load(config.parent / self.cfg["model"], force="scene")
        parents = sc.graph.transforms.parents
        hide = {"Icosphere", "Cross"}
        G0 = self._chain(np.zeros(self.n))                  # joint groups at rest
        parts = {}                                          # joint index (-1 = root) -> [verts, faces]
        for node in sc.graph.nodes_geometry:
            T, gname = sc.graph[node]
            chain, n = [], node
            while n is not None:
                chain.append(n)
                n = parents.get(n)
            if hide & set(chain):
                continue
            j = next((link_joint[c] for c in chain if c in link_joint), -1)
            m = sc.geometry[gname]
            if max_faces_per_link and len(m.faces) > max_faces_per_link:
                m = m.simplify_quadric_decimation(face_count=max_faces_per_link)
            v = trimesh.transform_points(m.vertices, T)     # three.js world at rest
            local = v if j < 0 else trimesh.transform_points(v, np.linalg.inv(G0[j]))
            p = parts.setdefault(j, [[], [], 0])
            p[0].append(local)
            p[1].append(np.asarray(m.faces) + p[2])
            p[2] += len(local)
        self.parts = {j: (np.vstack(v), np.vstack(f)) for j, (v, f, _) in parts.items()}

    def _chain(self, q_deg):
        """World matrices (three.js) of each joint's rotation group."""
        G = []
        for i, d in enumerate(self.joints):
            p = d.get("parent", i - 1)
            parent = np.eye(4) if p < 0 else G[p]
            rest = np.eye(4)
            rest[:3, :3] = _quat_wxyz_to_R(d["restQuat"])
            rest[:3, 3] = d["restPos"]
            rot = np.eye(4)
            rot[:3, :3] = _axis_angle(d["axis"], np.radians(self.sign[i] * q_deg[i]))
            G.append(parent @ rest @ rot)
        return G

    def link_transforms(self, q_deg):
        """{joint index: 4x4 local -> robot-frame (mm) transform}, -1 for the root."""
        G = self._chain(np.asarray(q_deg, float))
        out = {-1: THREE_TO_ROBOT.copy()}
        for j in range(self.n):
            out[j] = THREE_TO_ROBOT @ G[j]
        return out

    def vertices(self, q_deg, min_joint=-1, max_joint=None):
        """Vertices in the robot frame (mm) and matching faces, for links with
        min_joint <= joint index <= max_joint (-1 = the fixed base casting,
        1 = the links that move above it, 5 = the flange link)."""
        Ts = self.link_transforms(q_deg)
        vs, fs, off = [], [], 0
        for j, (v, f) in sorted(self.parts.items()):
            if j < min_joint or (max_joint is not None and j > max_joint):
                continue
            T = Ts[j]
            vs.append(v @ T[:3, :3].T + T[:3, 3])
            fs.append(f + off)
            off += len(v)
        return np.vstack(vs), np.vstack(fs)

    def mask(self, q_deg, K, dist, T_cam_robot, size):
        """Binary silhouette (uint8, 255 = robot) of the robot as the camera sees it.

        cv2.fillPoly fills a polygon list with the even-odd rule, so overlapping
        triangles would cancel. Draw only camera-facing triangles, one link at a
        time, fill each link's outline solid, and OR the links together.
        """
        Ts = self.link_transforms(q_deg)
        cam_centre = -T_cam_robot[:3, :3].T @ T_cam_robot[:3, 3]
        out = np.zeros((size[1], size[0]), np.uint8)
        for j, (v, f) in self.parts.items():
            T = Ts[j]
            vw = v @ T[:3, :3].T + T[:3, 3]
            tri = vw[f]
            n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            front = np.einsum("ij,ij->i", n, cam_centre - tri[:, 0]) > 0
            pix = project(vw, K, dist, T_cam_robot)
            poly = np.round(pix[f[front]] * 4).astype(np.int32)
            m = np.zeros_like(out)
            cv2.fillPoly(m, list(poly), 255, lineType=cv2.LINE_8, shift=2)
            cnt, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(m, cnt, -1, 255, -1)
            out |= m
        return out
