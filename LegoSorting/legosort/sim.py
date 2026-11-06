"""Simulation in the VRRobotController viewer (digital twin of the Meca500).

The viewer renders the robot, the table, the base mat and the bricks; a simulated
camera captures exact images through the viewer's ``captureImage`` command, and the
stage-1 vision code runs on them unchanged.

Frames
------
Everything here is in the **robot frame** (Meca500 base frame, mm, Z up, right
handed), the same frame as GNKinematics. The viewer's WebSocket API reports
positions with Y mirrored (API = (x, -y, z)); this was measured on 2026-10-31 by
comparing the viewer's EE position with GNKinematics FK (J1 = 30 deg gives
Y = +95 mm in GNKinematics, -95 mm from the API). ``to_api`` converts. Mesh
vertices are sent in three.js world coordinates (metres, Y up), which relate to
the robot frame by the proper rotation (x, y, z) -> (x, z, -y).
"""
import base64
import json
import struct
import time
from dataclasses import dataclass, field

import cv2
import numpy as np
from websockets.sync.client import connect

from .boards import BaseMat, dictionary
from .geometry import invert_T, look_at, transform_points

DEFAULT_URL = "ws://127.0.0.1:8080/ws"
API_MIRROR = np.array([1.0, -1.0, 1.0])


def to_api(p):
    return (np.asarray(p, float) * API_MIRROR).tolist()


def from_api(p):
    return np.asarray(p, float) * API_MIRROR


def robot_to_three(pts_mm):
    """Robot-frame mm -> three.js world metres (a proper rotation)."""
    p = np.asarray(pts_mm, float) / 1000.0
    return np.column_stack([p[:, 0], p[:, 2], -p[:, 1]])


# ------------------------------------------------------------------ viewer client

class Viewer:
    """Minimal synchronous client for the viewer's WebSocket API."""

    def __init__(self, url=DEFAULT_URL, session=None, timeout=20.0):
        if session:
            url += ("&" if "?" in url else "?") + f"role=controller&session={session}"
        else:
            url += ("&" if "?" in url else "?") + "role=controller"
        self.ws = connect(url, max_size=None, open_timeout=timeout)
        self.timeout = timeout
        self._req = 0
        # unique per client: replies go to every controller on the session, so two clients
        # numbering their requests alike could take each other's replies
        self._rid = f"ls{np.random.default_rng().integers(1 << 32):08x}_"
        self.on_message = None        # callback(dict) for messages nobody asked for (e.g. bridgeCommand)

    def close(self):
        self.ws.close()

    def send(self, msg):
        self.ws.send(json.dumps(msg))

    def _drain(self):
        """Drop messages already waiting: the viewer also broadcasts unrequested 'state'
        updates (several a second while a robot bridge is attached), which would
        otherwise be taken as the reply to a later getState."""
        while True:
            try:
                m = self.ws.recv(timeout=0)
            except TimeoutError:
                return
            self._unasked(m)

    def _unasked(self, m):
        if self.on_message is not None:
            d = json.loads(m) if isinstance(m, (str, bytes)) else m
            if d.get("type") != "state":
                self.on_message(d)

    def request(self, msg, want, timeout=None, strict=False):
        """Send ``msg`` and return the first reply of type ``want`` (raises on 'error').
        strict: only the reply to this request (not a broadcast of the same type, which
        may be older)."""
        self._drain()
        self._req += 1
        msg = dict(msg, _reqId=f"{self._rid}{self._req}")
        self.send(msg)
        end = time.monotonic() + (timeout or self.timeout)
        while True:
            left = end - time.monotonic()
            if left <= 0:
                raise TimeoutError(f"no '{want}' reply to {msg.get('cmd')}")
            d = json.loads(self.ws.recv(timeout=left))
            rid = d.get("_reqId")
            if rid not in (None, msg["_reqId"]) or (strict and rid is None and d.get("type") == want):
                continue
            if d.get("type") == "bridgeCommand":
                self._unasked(d)
                continue
            if d.get("type") == "error" and rid == msg["_reqId"]:
                raise RuntimeError(f"viewer error for {msg.get('cmd')}: {d.get('error')}")
            if d.get("type") == want:
                return d

    # ---- robot
    def set_joints(self, angles, settle=True):
        angles = [float(a) for a in angles]
        self.send({"cmd": "setJoints", "angles": angles})
        if settle:
            for _ in range(50):
                st = self.request({"cmd": "getState"}, "state")
                if np.allclose(st["joints"], angles, atol=1e-3):
                    return st
                time.sleep(0.02)
            raise TimeoutError("viewer did not reach the commanded joints")

    def state(self):
        return self.request({"cmd": "getState"}, "state")

    # ---- objects
    def list_objects(self):
        return self.request({"cmd": "listObjects"}, "objects").get("objects", [])

    def remove_objects(self, prefix):
        """Remove every imported object whose name starts with ``prefix``."""
        n = 0
        while True:
            names = [o["name"] for o in self.list_objects() if o.get("name", "").startswith(prefix)]
            if not names:
                return n
            for name in names:
                self.request({"cmd": "removeObject", "name": name}, "objectRemoved")
                n += 1

    def add_glb(self, name, glb, material=None):
        msg = {"cmd": "addObject", "format": "glb", "name": name,
               "data": base64.b64encode(glb).decode()}
        if material:
            msg["material"] = material
        return self.request(msg, "objectAdded")

    # ---- camera
    def capture(self, cam: "SimCamera", background=None, fmt="png", exposure=None):
        eye, target, up = cam.eye, cam.target, cam.up
        msg = {"cmd": "captureImage", "format": fmt, "camera": {
            "position": to_api(eye), "target": to_api(target), "up": to_api(up),
            "fov": cam.fov_deg, "width": cam.width, "height": cam.height}}
        if background:
            msg["background"] = background
        if exposure is not None:
            msg["exposure"] = float(exposure)
        d = self.request(msg, "image", timeout=60)
        buf = np.frombuffer(base64.b64decode(d["data"]), np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img, np.array(d["K"], float)


# ------------------------------------------------------------------ meshes

def glb_bytes(positions, colors):
    """A GLB of flat-shaded, vertex-coloured triangles.

    positions: (3N, 3) three.js metres, each consecutive 3 rows one triangle.
    colors:    (3N, 3) linear RGB 0..1.
    """
    pos = np.asarray(positions, np.float32).reshape(-1, 3)
    col = np.asarray(colors, np.float32).reshape(-1, 3)
    tri = pos.reshape(-1, 3, 3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    nor = np.repeat(n, 3, axis=0).astype(np.float32)
    blobs = [pos.tobytes(), nor.tobytes(), col.tobytes()]
    offsets = np.cumsum([0] + [len(b) for b in blobs])
    bin_chunk = b"".join(blobs)
    count = len(pos)
    gltf = {
        "asset": {"version": "2.0", "generator": "legosort.sim"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1, "COLOR_0": 2}}]}],
        "buffers": [{"byteLength": len(bin_chunk)}],
        "bufferViews": [{"buffer": 0, "byteOffset": int(offsets[i]), "byteLength": len(blobs[i]),
                         "target": 34962} for i in range(3)],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": count, "type": "VEC3",
             "min": pos.min(axis=0).tolist(), "max": pos.max(axis=0).tolist()},
            {"bufferView": 1, "componentType": 5126, "count": count, "type": "VEC3"},
            {"bufferView": 2, "componentType": 5126, "count": count, "type": "VEC3"},
        ],
    }
    js = json.dumps(gltf).encode()
    js += b" " * (-len(js) % 4)
    bin_chunk += b"\0" * (-len(bin_chunk) % 4)
    total = 12 + 8 + len(js) + 8 + len(bin_chunk)
    return (struct.pack("<III", 0x46546C67, 2, total)
            + struct.pack("<II", len(js), 0x4E4F534A) + js
            + struct.pack("<II", len(bin_chunk), 0x004E4942) + bin_chunk)


def srgb_to_linear(c):
    c = np.asarray(c, float)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


class MeshBuilder:
    """Collects triangles in the robot frame (mm), then emits a GLB."""

    def __init__(self):
        self.pos, self.col = [], []

    def quad(self, corners_mm, rgb):
        """Corners in counter-clockwise order seen from the side the face points to."""
        a, b, c, d = np.asarray(corners_mm, float)
        self.pos += [a, b, c, a, c, d]
        self.col += [rgb] * 6

    def box(self, T, size, rgb):
        """Axis-aligned box in its own frame (centre at origin), placed by 4x4 ``T``."""
        sx, sy, sz = np.asarray(size, float) / 2
        v = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
        v = transform_points(T, v)
        faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
        centre = v.mean(axis=0)
        for f in faces:
            q = v[list(f)]
            n = np.cross(q[1] - q[0], q[2] - q[0])
            if np.dot(n, q.mean(axis=0) - centre) < 0:      # wind every face outwards
                q = q[::-1]
            self.quad(q, rgb)

    def cylinder(self, centre, radius, z0, z1, rgb, segments=20):
        """Vertical cylinder with a top cap (no bottom), robot frame."""
        cx, cy = centre
        t = np.linspace(0, 2 * np.pi, segments + 1)
        ring = np.column_stack([cx + radius * np.cos(t), cy + radius * np.sin(t)])
        for i in range(segments):
            (xa, ya), (xb, yb) = ring[i], ring[i + 1]
            self.quad([[xa, ya, z0], [xb, yb, z0], [xb, yb, z1], [xa, ya, z1]], rgb)
            self.pos += [[cx, cy, z1], [xa, ya, z1], [xb, yb, z1]]
            self.col += [rgb] * 3

    def mesh(self, vertices, faces, rgb):
        """Indexed triangles (robot-frame mm vertices)."""
        tri = np.asarray(vertices, float)[np.asarray(faces)]
        self.pos += list(tri.reshape(-1, 3))
        self.col += [rgb] * (3 * len(tri))

    def cylinder_T(self, T, radius, half_h, rgb, segments=20, bottom=True):
        """Closed cylinder along its own z (centre at origin), placed by 4x4 ``T``."""
        t = np.linspace(0, 2 * np.pi, segments + 1)
        ring = np.column_stack([radius * np.cos(t), radius * np.sin(t)])
        lo = transform_points(T, np.column_stack([ring, np.full(len(ring), -half_h)]))
        hi = transform_points(T, np.column_stack([ring, np.full(len(ring), half_h)]))
        top, bot = transform_points(T, [[0, 0, half_h], [0, 0, -half_h]])
        for i in range(segments):
            self.quad([lo[i], lo[i + 1], hi[i + 1], hi[i]], rgb)
            self.pos += [top, hi[i], hi[i + 1]]
            self.col += [rgb] * 3
            if bottom:
                self.pos += [bot, lo[i + 1], lo[i]]
                self.col += [rgb] * 3

    def glb(self):
        return glb_bytes(robot_to_three(np.array(self.pos)), srgb_to_linear(np.array(self.col)))


def table_glb(size=(4000.0, 4000.0), centre=(250.0, 0.0), rgb=(0.45, 0.42, 0.38), z=-0.5):
    mb = MeshBuilder()
    x0, y0 = centre[0] - size[0] / 2, centre[1] - size[1] / 2
    x1, y1 = x0 + size[0], y0 + size[1]
    mb.quad([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]], rgb)
    return mb.glb()


def mat_glb(mat: BaseMat, z=0.2):
    """The base mat: white sheet plus black marker cells, placed by mat.T_robot_board."""
    mb = MeshBuilder()
    T = mat.T_robot_board
    w, h = mat.width_mm / 2, mat.height_mm / 2
    sheet = transform_points(T, [[-w, -h, z], [w, -h, z], [w, h, z], [-w, h, z]])
    mb.quad(sheet, (1.0, 1.0, 1.0))
    d = dictionary(mat.dict_name)
    nbits = d.markerSize + 2
    for m in mat.markers:
        img = cv2.aruco.generateImageMarker(d, m.id, nbits, borderBits=1)
        if m.rot_deg % 360:
            img = np.rot90(img, int(round(m.rot_deg / 90.0)) % 4)
        cell = m.size / nbits
        x_left, y_top = m.x - m.size / 2, m.y + m.size / 2
        for r in range(nbits):
            for c in range(nbits):
                if img[r, c] > 127:
                    continue
                xa, xb = x_left + c * cell, x_left + (c + 1) * cell
                ya, yb = y_top - (r + 1) * cell, y_top - r * cell
                q = transform_points(T, [[xa, ya, z + 0.1], [xb, ya, z + 0.1],
                                         [xb, yb, z + 0.1], [xa, yb, z + 0.1]])
                mb.quad(q, (0.02, 0.02, 0.02))
    return mb.glb()


def brick_glb(b, z_base=0.3, hollow=True):
    """A brick (lego.Brick) at its pose, any orientation, on the mat surface (z_base)."""
    from .geometry import make_T
    mb = MeshBuilder()
    rgb = np.array(b.rgb) / 255.0
    T = b.T
    T[2, 3] += z_base
    for part in b.parts(hollow=hollow):
        if part[0] == "box":
            _, c, h = part
            mb.box(T @ make_T(None, c), 2 * np.asarray(h), rgb)
        else:
            _, c, r, hh = part
            mb.cylinder_T(T @ make_T(None, c), r, hh, rgb, segments=16)
    return mb.glb()


# ------------------------------------------------------------------ camera, robot

@dataclass
class SimCamera:
    eye: tuple = (230.0, -420.0, 480.0)     # robot frame, mm
    target: tuple = (230.0, 0.0, 0.0)
    up: tuple = (1.0, 0.0, 0.0)
    fov_deg: float = 45.0                    # vertical
    width: int = 1280
    height: int = 720

    @property
    def T_robot_cam(self):
        """OpenCV camera (x right, y down, z forward) in the robot frame."""
        return look_at(self.eye, self.target, self.up)

    @property
    def T_cam_robot(self):
        return invert_T(self.T_robot_cam)

    def K(self):
        fy = (self.height / 2) / np.tan(np.radians(self.fov_deg) / 2)
        return np.array([[fy, 0, self.width / 2 - 0.5], [0, fy, self.height / 2 - 0.5], [0, 0, 1.0]])


@dataclass
class SimRobot:
    """The twin as a stand-in for the real arm, optionally with joint errors.

    Commands are in the GNKinematics/Mecademic joint convention (they matched the
    viewer at every pose tested). ``joint_offsets`` are added to every command,
    which makes the twin behave like an arm whose zeros are off: the 'true' pose
    differs from what a nominal model predicts, as on the real Meca500.
    """
    viewer: Viewer
    joint_offsets: np.ndarray = field(default_factory=lambda: np.zeros(6))

    def move_joints(self, angles):
        true = np.asarray(angles, float) + self.joint_offsets
        return self.viewer.set_joints(true)


# ------------------------------------------------------------------ scene

SIM_PREFIX = "ls_"


def build_scene(viewer: Viewer, mat: BaseMat):
    """(Re)create the table and the mat in the viewer."""
    viewer.remove_objects(SIM_PREFIX)
    matte = {"opacity": 1.0, "roughness": 0.9, "metalness": 0.0}
    viewer.add_glb(SIM_PREFIX + "table", table_glb(), matte)
    viewer.add_glb(SIM_PREFIX + "mat", mat_glb(mat), matte)


BRICK_MATERIAL = {"opacity": 1.0, "roughness": 0.35, "metalness": 0.0}   # ABS is glossy


def place_bricks(viewer: Viewer, bricks):
    """Replace all simulated bricks with ``bricks``; returns their viewer names."""
    viewer.remove_objects(SIM_PREFIX + "brick_")
    names = []
    for i, b in enumerate(bricks):
        name = f"{SIM_PREFIX}brick_{i:02d}_{b.colour}_{b.nx}x{b.ny}"
        viewer.add_glb(name, brick_glb(b), BRICK_MATERIAL)
        names.append(name)
    return names


# ------------------------------------------------------------------ physics playback

_M = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.0]])        # robot frame -> three.js axes


def _euler_xyz(R):
    """three.js 'XYZ' Euler angles (rad) of a rotation matrix."""
    ey = np.arcsin(np.clip(R[0, 2], -1, 1))
    if abs(R[0, 2]) < 0.9999999:
        return np.arctan2(-R[1, 2], R[2, 2]), ey, np.arctan2(-R[0, 1], R[0, 0])
    return np.arctan2(R[2, 1], R[1, 1]), ey, 0.0


def object_pose_msg(name, R, p):
    """setObject message putting an object, built in its own frame with MeshBuilder,
    at pose (R, p mm) in the robot frame."""
    ex, ey, ez = np.degrees(_euler_xyz(_M @ np.asarray(R, float) @ _M.T))
    # setObject takes rotation as [x, z, y] three.js Euler angles (it swaps y and z)
    return {"cmd": "setObject", "name": name, "space": "world", "position": to_api(p),
            "rotation": [float(ex), float(ez), float(ey)]}


class PhysicsView:
    """Shows a physics.PileSim in the viewer: the arm's joints, the gripper and every brick,
    updated whenever ``update()`` is called (e.g. as sim.on_frame).

    Bricks are added once, each in its own frame, then only moved (setObject), so a frame
    costs one small message per brick that moved.
    """

    def __init__(self, viewer: Viewer, sim, on_frame=None, drive_arm=True):
        """drive_arm=False: leave the viewer's robot joints alone (they are the operator's
        command, and the physics arm follows them; see apps/sim_teleop.py)."""
        self.v, self.sim, self.extra, self.drive_arm = viewer, sim, on_frame, drive_arm
        self._last = {}
        viewer.remove_objects(SIM_PREFIX + "brick_")
        viewer.remove_objects(SIM_PREFIX + "grip_")
        self.bricks = []
        from .lego import Brick
        for i, b in enumerate(sim.proto):
            name = f"{SIM_PREFIX}brick_{i:02d}_{b.colour}_{b.nx}x{b.ny}"
            viewer.add_glb(name, brick_glb(Brick(b.nx, b.ny, b.colour, height=b.height), z_base=0.0),
                           BRICK_MATERIAL)
            self.bricks.append(name)
        self.parts = []
        if sim.gripper is not None:
            g = sim.gripper
            metal = {"opacity": 1.0, "roughness": 0.45, "metalness": 0.5}
            for name, parts in (("grip_body", g.render_static()), ("grip_finger_a", g.render_jaw("a")),
                                ("grip_finger_b", g.render_jaw("b"))):
                mb = MeshBuilder()
                for v, f, rgb in parts:
                    mb.mesh(v, f, rgb)
                viewer.add_glb(SIM_PREFIX + name, mb.glb(), metal)
            self.parts = [("gripper", SIM_PREFIX + "grip_body"),
                          ("finger_a", SIM_PREFIX + "grip_finger_a"), ("finger_b", SIM_PREFIX + "grip_finger_b")]
        self.update(force=True)

    def _move(self, name, R, p, force):
        key = np.concatenate([np.ravel(R), p])
        last = self._last.get(name)
        if not force and last is not None and np.abs(key - last).max() < 0.02:
            return
        self._last[name] = key
        self.v.send(object_pose_msg(name, R, p))

    def update(self, sim=None, force=False):
        sim = self.sim
        if sim.gripper is not None:
            if self.drive_arm:
                self.v.send({"cmd": "setJoints", "angles": [float(a) for a in sim.q]})
            for body, name in self.parts:
                R, p = sim.body_pose(body)
                self._move(name, R, p, force)
        for i, name in enumerate(self.bricks):
            vis = bool(sim.active[i])
            if force or self._last.get(name + "/vis") != vis:
                self._last[name + "/vis"] = vis
                self.v.send({"cmd": "setObject", "name": name, "visible": vis})
            if vis:
                R, p = sim.pose(i)
                self._move(name, R, p + [0, 0, 0.3], force)      # physics z = 0 is the mat top
        self.v._drain()
        if self.extra:
            self.extra(sim)
