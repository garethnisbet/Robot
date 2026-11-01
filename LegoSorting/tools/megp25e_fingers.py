"""Stepped fingers for the MEGP 25E, to grip both 1-wide and 2-wide LEGO bricks (run in FreeCAD).

    exec(open("/FastDrive/Dropbox/ClaudeCodeProjects/LegoSorting/tools/megp25e_fingers.py").read())

The bare jaws span 11.4-17.0 mm (5.6 mm stroke): 2-wide bricks (15.8) only. Each finger
bolts on a jaw's gripping block and gives two steps:

  * NARROW, deep in (over the jaw block): a PAD mm plate on the jaw face, so the fingers
    span 17.0 - 2*PAD - 5.6 .. 17.0 - 2*PAD = 6.6 .. 12.2 mm: 1-wide bricks (7.8) and any
    brick across its height (9.6, or 11.3 over the studs).
  * WIDE, at the tips: EXT - SHOULDER mm long, flush with the bare jaw face (11.4-17.0):
    2-wide bricks (15.8). The step between them stops a 2-wide brick going in too far.

The fingers reach down to DROP mm above the gripper body's underside level, so they hold
low bricks over more of their height. Mounting: M3 x 10 countersunk screw through the pad
and the jaw's 3.2 mm hole, M3 nut on the outside of the jaw; optional 1.5 mm dowels.

Everything is built in the STEP file's frame from the jaw geometry itself. Writes to
models/megp25e/fingers/: finger_a.stl / finger_b.stl (STEP frame, as fitted, jaws open),
finger_a_print.stl / finger_b_print.stl (flat face down, ready to slice), fingers.step,
fingers.json (the collision boxes and steps for legosort/megp25e.py), fit.png.
"""
import json
import os

import FreeCAD
import Import
import Mesh
import Part
from FreeCAD import Vector

PROJECT = "/FastDrive/Dropbox/ClaudeCodeProjects/LegoSorting"
STEP = "/FastDrive/MEGP_25E_electric_gripper/MEGP 25E.stp"
OUT = os.path.join(PROJECT, "models", "megp25e", "fingers")

PAD = 2.4          # narrow-step plate on each jaw face, mm
EXT = 12.0         # finger length beyond the jaw tips, mm
SHOULDER = 2.0     # the pad starts this far beyond the jaw tips (joins pad and tip section)
DROP = 0.5         # finger bottom this far above the body's underside level, mm
GAP = 0.2          # clearance to the jaw and body
TIP_CHAMFER = 1.0  # lead-in on the tips' inner edges
SCREW_D, CSK_D = 3.4, 6.4      # M3 clearance, countersink diameter (90 deg)
DOWEL_D = 1.6

os.makedirs(OUT, exist_ok=True)
doc = FreeCAD.newDocument("MEGP_fingers")
Import.insert(STEP, doc.Name)
doc.recompute()
solids = {o.Name: o for o in doc.Objects if o.TypeId == "Part::Feature" and o.Shape.Solids}


def shape(name):
    s = solids[name].Shape.copy()
    s.Placement = solids[name].getGlobalPlacement()
    return s


body = shape("Part__Feature")
bb_body = body.BoundBox
body_end = bb_body.YMin                     # jaws stick out below this y
jaws = {"a": shape("Part__Feature006"), "b": shape("Part__Feature001")}


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def block_of(jaw):
    """Bounding box of the jaw's gripping block (the part outside the body)."""
    cut = jaw.common(box(-1e3, 1e3, -1e3, body_end - 0.3, -1e3, 1e3))
    return cut.BoundBox


blk = {s: block_of(j) for s, j in jaws.items()}
# jaw a's block faces -z (towards b), jaw b's faces +z
inner = {"a": blk["a"].ZMin, "b": blk["b"].ZMax}
inward = {"a": -1.0, "b": 1.0}
open_gap = inner["a"] - inner["b"]
z_mid = (inner["a"] + inner["b"]) / 2
x_c = (bb_body.XMin + bb_body.XMax) / 2     # TCP height level (middle of the body)
print("open gap", round(open_gap, 2), "block", blk["a"])

parts, boxes = {}, {}
for s in ("a", "b"):
    b, k, zi = blk[s], inward[s], inner[s]
    zo = zi - k * (b.ZLength)                                    # jaw block's outer face
    x0, x1 = bb_body.XMin + DROP, b.XMax
    tip = b.YMin
    pieces = {
        # pad on the jaw face, from SHOULDER beyond the tip back to the body (less a gap)
        "pad": (x0, x1, tip - SHOULDER, body_end - GAP, min(zi, zi + k * PAD), max(zi, zi + k * PAD)),
        # tip section, flush with the jaw face, as thick as the jaw block
        "tip": (x0, x1, tip - EXT, tip - SHOULDER, min(zi, zo), max(zi, zo)),
        # web under the jaw block joining the two (clear of the block by GAP)
        "web": (x0, b.XMin - GAP, tip - SHOULDER, body_end - GAP, min(zi, zo), max(zi, zo)),
        # the tip section continues back over the shoulder so the tip and pad overlap
        "join": (x0, x1, tip - SHOULDER, tip - GAP, min(zi, zo), max(zi, zo)),
    }
    solid = None
    for name, p in pieces.items():
        bx = box(*p)
        solid = bx if solid is None else solid.fuse(bx)
    solid = solid.removeSplitter()
    # the join piece must not overlap the jaw block itself
    solid = solid.cut(jaws[s])
    # lead-in chamfer on the tip's inner vertical edge
    edges = [e for e in solid.Edges
             if abs(e.BoundBox.YMin - (tip - EXT)) < 1e-6 and abs(e.BoundBox.YMax - (tip - EXT)) < 1e-6
             and abs(e.BoundBox.ZMin - zi) < 1e-6 and abs(e.BoundBox.ZMax - zi) < 1e-6]
    if edges and TIP_CHAMFER:
        solid = solid.makeChamfer(TIP_CHAMFER, edges)
    # screw and dowel holes: find them in the jaw block (cylindrical faces along z)
    holes = []
    for f in jaws[s].Faces:
        if f.Surface.TypeId == "Part::GeomCylinder" and abs(abs(f.Surface.Axis.z) - 1) < 1e-6:
            c, r = f.Surface.Center, f.Surface.Radius
            if r > 0.6 and c.y < body_end and (round(c.x, 2), round(c.y, 2), round(r, 2)) not in [(h[0], h[1], h[2]) for h in holes]:
                holes.append((round(c.x, 2), round(c.y, 2), round(r, 2)))
    zface = zi + k * PAD                                         # pad's gripping face
    for hx, hy, r in holes:
        if r > 1.2:                                              # the M3 hole: through + countersink
            cyl = Part.makeCylinder(SCREW_D / 2, 30, Vector(hx, hy, zi - 15))
            cone = Part.makeCone(SCREW_D / 2, CSK_D / 2 + 0.6, (CSK_D - SCREW_D) / 2 + 0.6,
                                 Vector(hx, hy, zface - k * ((CSK_D - SCREW_D) / 2)), Vector(0, 0, k))
            solid = solid.cut(cyl).cut(cone)
        else:
            solid = solid.cut(Part.makeCylinder(DOWEL_D / 2, 30, Vector(hx, hy, zi - 15)))
    parts[s] = solid
    boxes[s] = {n: p for n, p in pieces.items() if n != "join"}
    print(s, "holes", holes, "volume", round(solid.Volume, 1), "valid", solid.isValid())

# fit check: open (as modelled) and closed (each jaw moves stroke/2 inward)
STROKE = 5.6
report = {}
for state, d in (("open", 0.0), ("closed", STROKE / 2)):
    fa = parts["a"].copy(); fa.translate(Vector(0, 0, -d))
    fb = parts["b"].copy(); fb.translate(Vector(0, 0, d))
    ja = jaws["a"].copy(); ja.translate(Vector(0, 0, -d))
    jb = jaws["b"].copy(); jb.translate(Vector(0, 0, d))
    hits = {"a-body": fa.common(body).Volume, "b-body": fb.common(body).Volume,
            "a-jaw_a": fa.common(ja).Volume, "b-jaw_b": fb.common(jb).Volume,
            "a-b": fa.common(fb).Volume, "a-jaw_b": fa.common(jb).Volume, "b-jaw_a": fb.common(ja).Volume}
    report[state] = {k: round(v, 3) for k, v in hits.items()}
print("interference mm^3", report)

# outputs
for s, solid in parts.items():
    m = Mesh.Mesh(solid.tessellate(0.02))
    m.write(os.path.join(OUT, f"finger_{s}.stl"))
    p = solid.copy()                                  # print: jaw-side (outer) face down
    if inward[s] < 0:
        p.rotate(Vector(0, 0, 0), Vector(1, 0, 0), 180)
    bbp = p.BoundBox
    p.translate(Vector(-bbp.XMin, -bbp.YMin, -bbp.ZMin))
    Mesh.Mesh(p.tessellate(0.02)).write(os.path.join(OUT, f"finger_{s}_print.stl"))
Part.makeCompound([parts["a"], parts["b"]]).exportStep(os.path.join(OUT, "fingers.step"))
info = {"pad": PAD, "ext": EXT, "shoulder": SHOULDER, "open_gap": open_gap,
        "narrow_open": open_gap - 2 * PAD, "wide_open": open_gap,
        "tip_y": blk["a"].YMin - EXT, "jaw_tip_y": blk["a"].YMin, "body_end_y": body_end,
        "bottom_x": bb_body.XMin + DROP, "x_c": x_c, "z_mid": z_mid,
        "boxes": boxes, "interference": report}
with open(os.path.join(OUT, "fingers.json"), "w") as f:
    json.dump(info, f, indent=1)

for s, solid in parts.items():
    o = doc.addObject("Part::Feature", f"finger_{s}")
    o.Shape = solid
    if FreeCAD.GuiUp:
        o.ViewObject.ShapeColor = (0.95, 0.45, 0.1)
doc.recompute()
if FreeCAD.GuiUp:
    import FreeCADGui as Gui
    Gui.SendMsgToActiveView("ViewFit")
    v = Gui.ActiveDocument.ActiveView
    v.viewIsometric()
    v.fitAll()
    v.saveImage(os.path.join(OUT, "fit.png"), 1600, 1200, "White")
print("wrote", OUT)
