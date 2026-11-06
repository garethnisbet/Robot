"""The Mecademic MEGP 25E gripper, from the vendor STEP files, and how it is mounted.

Meshes come from models/megp25e/megp25e_parts.npz (meshed in FreeCAD by
tools/megp25e_export.py from the vendor STEP model, drawn without decimation). Everything here is in the TOOL frame, mm:
the GNKinematics flange frame, origin on the flange face, z out of the flange.

The GRIP frame (``T_tool_grip``) is what grasps are planned in: origin midway between
the jaw tips (the TCP), z the way the jaws point, x the direction they close along.

Read from the CAD:
  * The jaws are L-shaped sliders. Their gripping blocks stand 8.2 mm proud of the body
    end, and the jaw faces are 17.0 mm apart in the model (fully open). With the
    5.6 mm stroke (meca500_bridge's range), closed is 11.4 mm, so the bare jaws grip
    a 2-wide brick (15.8 mm) with no extra fingers.
  * The keypad (round button) and the M8 connector are on the same side face (-Z in
    the STEP file), across the closing direction.

Mounts:
  * "side" (the user's arm, photo 2026-11-01): no adapter. A printed spacer (yellow,
    about 7 mm thick, about 20 mm across) joins the flange to one of the two wide side
    faces next to the keypad face (+X or -X in the STEP file: ``face``). So the gripper
    sits across the flange: the jaws point at 90 deg to the flange axis, which meets
    the gripper ``axis_to_tips`` mm above the jaw tips. Spacer size, axis_to_tips and
    face are estimates from the photo.
  * "axial": on the M500 STAP01 plate (5 mm), jaws pointing straight out of the
    flange; TCP 75.2 mm from the flange face.
  * Neither file says how the gripper is turned about the flange axis relative to
    J6 = 0: ``mount_angle``.

Fingers (``fingers=True``): printed stepped fingers bolted on the jaw blocks (designed in
tools/megp25e_fingers.py, read from models/megp25e/fingers/). A narrow step deep in
(6.6-12.2 mm: 1-wide bricks, and any brick across its height) and a wide step at the tips
(11.4-17.0 mm: 2-wide bricks); the TCP moves out to the finger tips. ``steps`` lists the
grip ranges either way.
"""
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import json

import numpy as np
import trimesh

PARTS = Path(__file__).resolve().parent.parent / "models" / "megp25e" / "megp25e_parts.npz"
FINGERS = PARTS.parent / "fingers"           # stepped fingers, from tools/megp25e_fingers.py
PLATE = 5.0                    # STAP01 thickness between flange face and gripper base, mm

BODY = "gripper/Part__Feature"
JAW_B = ["gripper/Part__Feature001", "gripper/Part__Feature002", "gripper/Part__Feature003",
         "gripper/Part__Feature004", "gripper/Part__Feature005"]          # jaw on the -x side
JAW_A = ["gripper/Part__Feature006", "gripper/Part__Feature007", "gripper/Part__Feature008",
         "gripper/Part__Feature009", "gripper/Part__Feature010"]          # jaw on the +x side
CAP = "gripper/Part__Feature011"
KEYPAD = "gripper/Part__Feature012"
SCREWS = ["gripper/Part__Feature013", "gripper/Part__Feature014"]
CONNECTOR = ["gripper/Part__Feature015", "gripper/Part__Feature016", "gripper/Part__Feature017"]
ADAPTER = "adapter/Part__Feature"

COLOURS = {"body": (0.12, 0.12, 0.13), "cap": (0.08, 0.08, 0.09), "keypad": (0.3, 0.3, 0.32),
           "screw": (0.55, 0.55, 0.57), "connector": (0.6, 0.6, 0.62), "adapter": (0.7, 0.71, 0.73),
           "jaw": (0.78, 0.79, 0.81), "spacer": (0.95, 0.85, 0.1), "finger": (0.95, 0.45, 0.1)}


@lru_cache(maxsize=1)
def _fingers():
    """(info dict, {side: (vertices, faces)}) of the stepped fingers, STEP frame, jaws open."""
    info = json.loads((FINGERS / "fingers.json").read_text())
    meshes = {}
    for side in ("a", "b"):
        m = trimesh.load(FINGERS / f"finger_{side}.stl", force="mesh")
        meshes[side] = (np.asarray(m.vertices, float), np.asarray(m.faces))
    return info, meshes


@lru_cache(maxsize=1)
def _raw():
    d = np.load(PARTS)
    return {k[:-2]: (d[k].astype(float), d[k[:-2] + "_f"]) for k in d.files if k.endswith("_v")}


@dataclass
class MEGP25E:
    stroke: float = 5.6                     # mm, total (both jaws)
    force: float = 10.0                     # N per jaw (MEGP 25E: up to 40 N)
    friction: float = 0.6
    mount: str = "side"                     # "side" (printed spacer) or "axial" (STAP01)
    mount_angle: float = 0.0                # deg about the flange axis
    spacer: float = 7.0                     # side: spacer thickness, mm (estimate)
    spacer_d: float = 20.0                  # side: spacer diameter, mm (estimate)
    axis_to_tips: float = 21.0              # side: flange axis to jaw tips along the jaws, mm (estimate)
    face: int = 1                           # side: spacer on the +X (1) or -X (-1) face of the STEP body
    plate: float = PLATE
    fingers: bool = False                   # printed stepped fingers on the jaws
    _cache: dict = field(default_factory=dict, repr=False)

    # ---------------------------------------------------------- frame

    def __post_init__(self):
        raw = _raw()
        body_v = raw[BODY][0]
        cap_v = raw[CAP][0]
        jb, ja = raw[JAW_B[0]][0], raw[JAW_A[0]][0]
        tips_y = min(jb[:, 1].min(), ja[:, 1].min())
        jaw_tips_y = tips_y
        self._body_end_y = body_v[:, 1].min()
        # jaw faces: the -x jaw's block faces +Z_step at its top, the +x jaw's faces -Z_step
        block_b = jb[jb[:, 1] < self._body_end_y - 0.5]
        block_a = ja[ja[:, 1] < self._body_end_y - 0.5]
        face_b, face_a = block_b[:, 2].max(), block_a[:, 2].min()
        self.open_gap = float(face_a - face_b)
        x_c = (body_v[:, 0].min() + body_v[:, 0].max()) / 2
        z_c = (face_a + face_b) / 2
        if self.mount == "axial":
            centre = np.array([x_c, cap_v[:, 1].max() + self.plate, z_c])
            # x_tool = +Z_step (closing), y_tool = -X_step, z_tool = -Y_step (base -> jaws)
            R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.0]])
        elif self.mount == "side":
            f = 1 if self.face >= 0 else -1
            x_face = body_v[:, 0].max() if f > 0 else body_v[:, 0].min()
            centre = np.array([x_face + f * self.spacer, tips_y + self.axis_to_tips, z_c])
            # x_tool = +Z_step (closing), y_tool = f*Y_step, z_tool = -f*X_step (flange -> gripper)
            R = np.array([[0, 0, 1], [0, f, 0], [-f, 0, 0.0]])
        else:
            raise ValueError(f"mount must be 'side' or 'axial', not {self.mount!r}")
        c, s = np.cos(np.radians(self.mount_angle)), np.sin(np.radians(self.mount_angle))
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        self._R, self._c, self._Rz = Rz @ R, centre, Rz
        if self.fingers:
            info = _fingers()[0]
            tips_y = info["tip_y"]
        # tips to the body end, along the jaws (with fingers: the finger tips)
        self.jaw_length = float(self._body_end_y - tips_y)
        bare = float(self._body_end_y - jaw_tips_y)
        # side-grip ranges: (gap when open, near-face depth range from the tips)
        if self.fingers:
            wide = info["ext"] - info["shoulder"]
            self.steps = [(info["wide_open"], 0.0, wide), (info["narrow_open"], wide, self.jaw_length)]
            bottom = info["bottom_x"]
        else:
            self.steps = [(self.open_gap, 0.0, bare)]
            bottom = min(block_a[:, 0].min(), block_b[:, 0].min())
        top = max(block_a[:, 0].max(), block_b[:, 0].max())
        # how far the jaws (fingers) reach up and down from the TCP, side mount (grip y = -X_step)
        self.jaw_up, self.jaw_down = float(top - x_c), float(x_c - bottom)
        # grip frame: origin between the jaw tips, z along the jaws, x the closing direction
        xg = self._R @ [0, 0, 1.0]
        zg = self._R @ [0, -1.0, 0]
        T = np.eye(4)
        T[:3, :3] = np.column_stack([xg, np.cross(zg, xg), zg])
        T[:3, 3] = self.to_tool([x_c, tips_y, z_c])[0]
        self.T_tool_grip = T

    @property
    def closed_gap(self):
        return self.open_gap - self.stroke

    @property
    def axis(self):
        """Opening direction of the +x jaw (tool frame); the other jaw mirrors it."""
        return self._R @ [0, 0, 1.0]

    def to_tool(self, v_step):
        return np.atleast_2d(np.asarray(v_step, float) - self._c) @ self._R.T

    def _adapter_to_tool(self, v):
        # adapter file: plate in z 0..5 with bosses -2..0 and 5..7; flange face at z = 0
        return np.asarray(v, float) @ self._Rz.T

    def _mount_part(self):
        """(vertices, faces) of the adapter plate or the printed spacer, tool frame."""
        if self.mount == "axial":
            v, f = _raw()[ADAPTER]
            return self._adapter_to_tool(v), f
        m = trimesh.creation.cylinder(radius=self.spacer_d / 2, height=self.spacer, sections=32)
        return np.asarray(m.vertices) + [0, 0, self.spacer / 2], np.asarray(m.faces)

    # ---------------------------------------------------------- meshes (tool frame)

    def render_static(self):
        """[(vertices, faces, rgb)] for everything that does not move."""
        if "static" not in self._cache:
            raw, out = _raw(), []
            # drawn as meshed in FreeCAD (tools/megp25e_export.py): decimating distorts them
            groups = [([BODY], "body"), ([CAP], "cap"), ([KEYPAD], "keypad"),
                      (SCREWS, "screw"), (CONNECTOR, "connector")]
            for names, col in groups:
                for n in names:
                    v, f = raw[n]
                    out.append((self.to_tool(v), f, COLOURS[col]))
            v, f = self._mount_part()
            out.append((v, f, COLOURS["adapter" if self.mount == "axial" else "spacer"]))
            self._cache["static"] = out
        return self._cache["static"]

    def render_jaw(self, side):
        """[(vertices, faces, rgb)] of jaw 'a' (+x) or 'b' (-x) in its own frame: the tool
        frame with the jaw at its CLOSED position (it opens by stroke/2 along its axis)."""
        key = "jaw_" + side
        if key not in self._cache:
            raw, out = _raw(), []
            shift = (self.axis if side == "a" else -self.axis) * self.stroke / 2
            for n in (JAW_A if side == "a" else JAW_B):
                v, f = raw[n]
                out.append((self.to_tool(v) - shift, f, COLOURS["jaw"]))
            if self.fingers:
                v, f = _fingers()[1][side]
                out.append((self.to_tool(v) - shift, f, COLOURS["finger"]))
            self._cache[key] = out
        return self._cache[key]

    def collision_static(self):
        """Convex pieces (vertex arrays, tool frame) for the body, cap, connector and plate."""
        raw = _raw()
        hulls = [self.to_tool(trimesh.Trimesh(*raw[BODY]).convex_hull.vertices),
                 self.to_tool(trimesh.Trimesh(*raw[CAP]).convex_hull.vertices),
                 self.to_tool(np.vstack([raw[n][0] for n in CONNECTOR])),
                 self._mount_part()[0]]
        out = []
        for h in hulls:
            h = trimesh.convex.convex_hull(h).vertices
            out.append(h[np.linspace(0, len(h) - 1, min(len(h), 200)).astype(int)])
        return out

    def collision_jaw(self, side):
        """Convex gripping block of a jaw (the part outside the body), in its closed-position frame."""
        raw = _raw()
        v, f = raw[(JAW_A if side == "a" else JAW_B)[0]]
        m = trimesh.Trimesh(v, f)
        y = self._body_end_y - 0.3          # just outside the body: the slider starts at its end
        cut = m.section(plane_origin=[0, y, 0], plane_normal=[0, 1, 0]).vertices
        pts = np.vstack([v[v[:, 1] < y], cut])
        shift = (self.axis if side == "a" else -self.axis) * self.stroke / 2
        return self.to_tool(trimesh.convex.convex_hull(pts).vertices) - shift

    def collision_jaw_parts(self, side):
        """Convex pieces of a moving jaw (its block, plus the finger's boxes if fitted),
        in its closed-position frame."""
        out = [self.collision_jaw(side)]
        if self.fingers:
            info = _fingers()[0]
            shift = (self.axis if side == "a" else -self.axis) * self.stroke / 2
            for name, (x0, x1, y0, y1, z0, z1) in info["boxes"][side].items():
                if name == "tip":                  # the tip section runs back to the jaw tip
                    y1 = info["jaw_tip_y"] - 0.2
                c = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])
                out.append(self.to_tool(c) - shift)
        return out

    def push_depth(self, width):
        """How far (mm from the tips) a brick ``width`` mm across goes between the open
        jaws before it meets something: the body end, the fingers' step, or the tips."""
        depth = 0.0
        for gap, lo, hi in sorted(self.steps, key=lambda s: -s[0]):
            if width < gap - 0.5:
                depth = hi
            else:
                return lo if depth else 0.0
        return self.jaw_length
