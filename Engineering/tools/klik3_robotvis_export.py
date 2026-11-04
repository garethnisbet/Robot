"""Export KLiK3 as a RobotVisualisation device: klik3_config.json + klik3_scene.glb.

Run inside FreeCAD with KLiK3.FCStd open (MCP bridge or macro console):
    exec(open('/FastDrive/Dropbox/ClaudeCodeProjects/Engineering/tools/klik3_robotvis_export.py').read())

The model is exported at the zero pose (the joint sheet is zeroed for the export
and restored afterwards). Each visible part goes to the GLB node of the link it
rides on (L0 = Link1 ... L5 = Link6); parts outside any link (base, J1 actuator,
cables) go to a static "Base" node. Parts are merged per colour.

Frames: FreeCAD is Z-up in mm, the viewer is Y-up in metres:
    (x, y, z)_freecad -> (x, z, -y) / 1000
The robot root (KLiK3 part, WORLD_Z) maps to the viewer origin shifted so the
base plate top sits at y = 0, as in FreeCAD world.

Import in the viewer: pick or drop both output files together.
"""
import json, math, os, struct
import numpy as np
import FreeCAD as App
import MeshPart

OUT_DIR = "/FastDrive/Dropbox/ClaudeCodeProjects/Engineering/robotvis"
NAME = "klik3"
DOC = "KLiK3"
LIN_TOL = 0.1           # tessellation: chord deviation (mm)
ANG_TOL = 0.3           # and angle per segment (rad); the GUI mesh is ~4x finer
LINKS = ["Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]
SKIP_ROOTS = {"Meca500_ref"}

doc = App.getDocument(DOC)
sheet = doc.getObject("Joints")


def to_view(p):
    """FreeCAD point/vector (mm, Z-up) -> viewer (m, Y-up)."""
    return (p[0] / 1000.0, p[2] / 1000.0, -p[1] / 1000.0)


def visible(o):
    try:
        if not o.ViewObject.Visibility:
            return False
    except Exception:
        if not getattr(o, "Visibility", True):
            return False
    return True


def link_of(o):
    """Index of the nearest LinkN ancestor (0..5), -1 for static, None to skip."""
    cur = o
    chain_visible = True
    while True:
        parents = [p for p in cur.InList if p.TypeId == "App::Part"]
        if not parents:
            return (None if cur.Name in SKIP_ROOTS else -1), chain_visible
        p = parents[0]
        if not visible(p):
            chain_visible = False
        if p.Name in LINKS:
            # still need the root check for skipping
            r = p
            while True:
                pp = [q for q in r.InList if q.TypeId == "App::Part"]
                if not pp:
                    break
                r = pp[0]
            return (None if r.Name in SKIP_ROOTS else LINKS.index(p.Name)), chain_visible
        cur = p


def face_colours(o, n_faces):
    """Per-face (r, g, b, a) from the view provider."""
    vo = o.ViewObject
    cols = None
    try:
        app = vo.ShapeAppearance
        cols = [(m.DiffuseColor[0], m.DiffuseColor[1], m.DiffuseColor[2], 1.0 - m.Transparency) for m in app]
    except Exception:
        try:
            cols = [(c[0], c[1], c[2], 1.0 - (c[3] if len(c) > 3 else 0.0)) for c in vo.DiffuseColor]
        except Exception:
            cols = [(0.8, 0.8, 0.8, 1.0)]
    if len(cols) != n_faces:
        cols = [cols[0]] * n_faces
    return cols


def tessellate_face(f):
    pts, tris = f.tessellate(LIN_TOL)
    if not tris:
        return None
    v = np.array([to_view(p) for p in pts], dtype=np.float64)
    t = np.array(tris, dtype=np.int64)
    # smooth normals within the face, sharp between faces
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    fn = np.cross(b - a, c - a)
    n = np.zeros_like(v)
    for k in range(3):
        np.add.at(n, t[:, k], fn)
    ln = np.linalg.norm(n, axis=1)
    ln[ln == 0] = 1
    n /= ln[:, None]
    return v, n, t


# group key: (link index or -1, colour) -> list of (v, n, t)
groups = {}

saved_q = [sheet.get(f"B{i}") for i in range(2, 8)]
try:
    for i in range(2, 8):
        sheet.set(f"B{i}", "0")
    doc.recompute()

    # joint frames at the zero pose (read before the sheet is restored)
    world = [doc.getObject(nm).getGlobalPlacement().Base for nm in LINKS]
    # IK target on the flange's mounting face: the Flange part's far end
    # along the J6 axis (world +X at the zero pose)
    fl = doc.getObject("Flange")
    fs = fl.Shape.copy(); fs.Placement = fl.getGlobalPlacement()
    flange_face = fs.BoundBox.XMax - world[5].x      # mm beyond the J6 origin

    for o in doc.Objects:
        if not o.isDerivedFrom("Part::Feature") or o.TypeId == "Spreadsheet::Sheet":
            continue
        if not visible(o):
            continue
        li, chain_vis = link_of(o)
        if li is None or not chain_vis:
            continue
        if o.Shape.isNull() or not o.Shape.Faces:
            continue
        shp = o.Shape.copy(False)        # without the GUI's triangulation
        shp.Placement = o.getGlobalPlacement()
        # mesh the whole shape once (shared edges, no cracks); the faces keep it
        MeshPart.meshFromShape(Shape=shp, LinearDeflection=LIN_TOL,
                               AngularDeflection=ANG_TOL, Relative=False)
        cols = face_colours(o, len(o.Shape.Faces))
        for f, col in zip(shp.Faces, cols):
            r = tessellate_face(f)
            if r is None:
                continue
            key = (li, tuple(round(x, 3) for x in col))
            groups.setdefault(key, []).append(r)
finally:
    for i, q in enumerate(saved_q):
        sheet.set(f"B{i + 2}", str(q))
    doc.recompute()


# ---------------------------------------------------------------- GLB writer
bin_chunks = []
bin_len = 0
buffer_views, accessors, materials, meshes = [], [], [], []
mat_index = {}


def add_view(arr, target):
    global bin_len
    data = arr.tobytes()
    pad = (-len(data)) % 4
    bin_chunks.append(data + b"\0" * pad)
    buffer_views.append({"buffer": 0, "byteOffset": bin_len, "byteLength": len(data), "target": target})
    bin_len += len(data) + pad
    return len(buffer_views) - 1


def add_accessor(arr, comp, typ, target, minmax=False):
    bv = add_view(arr, target)
    acc = {"bufferView": bv, "componentType": comp, "count": int(arr.shape[0]), "type": typ}
    if minmax:
        acc["min"] = arr.min(axis=0).tolist()
        acc["max"] = arr.max(axis=0).tolist()
    accessors.append(acc)
    return len(accessors) - 1


def material(col):
    if col in mat_index:
        return mat_index[col]
    r, g, b, a = col
    grey = abs(r - g) < 0.06 and abs(g - b) < 0.08
    bright = (r + g + b) / 3
    metal = grey and bright > 0.55          # aluminium / steel parts
    m = {
        "name": f"c{len(materials)}",
        "pbrMetallicRoughness": {
            # FreeCAD colours are sRGB; glTF wants linear
            "baseColorFactor": [((x + 0.055) / 1.055) ** 2.4 if x > 0.04045 else x / 12.92 for x in (r, g, b)] + [a],
            "metallicFactor": 0.6 if metal else 0.0,
            "roughnessFactor": 0.35 if metal else 0.55,
        },
        "doubleSided": True,
    }
    if a < 0.999:
        m["alphaMode"] = "BLEND"
    materials.append(m)
    mat_index[col] = len(materials) - 1
    return mat_index[col]


node_names = ["Base"] + [f"L{i}" for i in range(6)]
mesh_of_node = {}
tri_count = 0
for li in range(-1, 6):
    prims = []
    for (gli, col), parts in sorted(groups.items()):
        if gli != li:
            continue
        vs, ns, ts, off = [], [], [], 0
        for v, n, t in parts:
            vs.append(v); ns.append(n); ts.append(t + off); off += len(v)
        v = np.concatenate(vs).astype(np.float32)
        n = np.concatenate(ns).astype(np.float32)
        t = np.concatenate(ts).astype(np.uint32).reshape(-1)
        tri_count += len(t) // 3
        prims.append({
            "attributes": {"POSITION": add_accessor(v, 5126, "VEC3", 34962, True),
                           "NORMAL": add_accessor(n, 5126, "VEC3", 34962)},
            "indices": add_accessor(t, 5125, "SCALAR", 34963),
            "material": material(col),
        })
    if prims:
        meshes.append({"name": node_names[li + 1], "primitives": prims})
        mesh_of_node[li + 1] = len(meshes) - 1

nodes = [{"name": "KLiK3", "children": []}]
for k, nm in enumerate(node_names):
    if k in mesh_of_node:
        nodes.append({"name": nm, "mesh": mesh_of_node[k]})
        nodes[0]["children"].append(len(nodes) - 1)

gltf = {
    "asset": {"version": "2.0", "generator": "klik3_robotvis_export.py"},
    "scene": 0, "scenes": [{"name": "Scene", "nodes": [0]}],
    "nodes": nodes, "meshes": meshes, "materials": materials,
    "accessors": accessors, "bufferViews": buffer_views,
    "buffers": [{"byteLength": bin_len}],
}
js = json.dumps(gltf, separators=(",", ":")).encode()
js += b" " * ((-len(js)) % 4)
binb = b"".join(bin_chunks)
os.makedirs(OUT_DIR, exist_ok=True)
glb_path = os.path.join(OUT_DIR, f"{NAME}_scene.glb")
with open(glb_path, "wb") as fh:
    fh.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(binb)))
    fh.write(struct.pack("<II", len(js), 0x4E4F534A)); fh.write(js)
    fh.write(struct.pack("<II", len(binb), 0x004E4942)); fh.write(binb)

# ---------------------------------------------------------------- config
# Joint frames at the zero pose, from the link placements (each rotates about
# its own frame's X, Y or Z, as in the FreeCAD placement expressions).
AXES = {"Link1": (0, 0, 1), "Link2": (0, 1, 0), "Link3": (0, 1, 0),
        "Link4": (1, 0, 0), "Link5": (0, 1, 0), "Link6": (1, 0, 0)}
LABELS = ["J1 Base", "J2 Shoulder", "J3 Elbow", "J4 Wrist Roll", "J5 Wrist Pitch", "J6 Flange"]
try:
    import klik3_freecad as k3
    limits = [list(l) for l in k3.LIMITS]
except Exception:
    limits = [[-175, 175], [-115, 105], [-153, 75], [-170, 170], [-130, 130], [-3600, 3600]]
limits[5] = [max(limits[5][0], -360), min(limits[5][1], 360)]   # usable slider range

joints = []
prev = App.Vector(0, 0, 0)
for i, nm in enumerate(LINKS):
    p = world[i]
    if i == 0:
        p = App.Vector(0, 0, 0)          # any point on the J1 axis; the base plate top
    d = to_view(p - prev)
    joints.append({
        "name": LABELS[i], "bone": LABELS[i],
        "restPos": [round(x, 6) + 0.0 for x in d],
        "restQuat": [1.0, 0.0, 0.0, 0.0],
        "axis": [int(x) for x in to_view([1000 * a for a in AXES[nm]])],
        "limits": limits[i],
        "parent": i - 1,
    })
    prev = p

config = {
    "name": "KLiK3",
    "model": f"{NAME}_scene.glb",
    "joints": joints,
    "links": [{"name": f"L{i}", "label": f"L{i}", "joint": i} for i in range(6)],
    "eeOffset": [round(flange_face / 1000.0, 6), 0, 0],   # J6 frame (= world axes at zero) -> flange face
    # tool frame (Meca500 convention): z out of the flange, x down; listed z, y, x
    "eeAxes": [[1, 0, 0], [0, 0, -1], [0, -1, 0]],
    "demoPose": [0, 6, 36, 0, 48, 0],
}
cfg_path = os.path.join(OUT_DIR, f"{NAME}_config.json")
with open(cfg_path, "w") as fh:
    json.dump(config, fh, indent=1)

print(f"wrote {glb_path} ({os.path.getsize(glb_path) / 1e6:.1f} MB, {tri_count} triangles, "
      f"{len(materials)} materials) and {cfg_path}")
for j in joints:
    print(j["name"], j["restPos"], j["axis"], j["limits"])
