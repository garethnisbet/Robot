"""Build a jointed Meca500-like 6-axis arm with harmonic-drive joints in FreeCAD.

Geometry follows Kinematics/Meca500.py (mm, home pose):
  J1 axis Z at origin, J2 axis Y at z=135, J3 axis Y at z=270,
  J4 axis X at z=308 (38 mm offset), J5 axis Y at x=120, flange at
  x=120+WRIST (Meca500 itself: WRIST=70, flange at x=190).

Each link lives in a nested App::Part whose Placement is an expression of the
joint angles in the "Joints" spreadsheet (cells B2:B7, aliases q1..q6), so
editing those cells poses the arm (or call pose(j1, ..., j6), or use the
slider panel from show_panel()).

Every joint is a cup-type strain-wave (harmonic) drive with a frameless motor,
and a crossed-roller bearing across the running gap that carries the link's
moment and axial loads (the flexspline only transmits torque):
  circular spline (CS)  - fixed to the housing link, N+2 internal teeth
  flexspline (FS)       - cup whose diaphragm drives the output link, N teeth
  wave generator (WG)   - elliptical plug + flexible bearing on the motor shaft
With the CS fixed, output = -WG / RATIO, so each WG turns at -RATIO x its joint
angle. Proportions are scaled from the size code (pitch diameter ~ size x
2.54 mm); they are representative, not catalogue dimensions.

Run inside FreeCAD (macro, or via the MCP bridge's execute_python_file).
"""
import math

import FreeCAD as App
import Part
from FreeCAD import Vector as V

WHITE = (0.93, 0.93, 0.92)
DARK = (0.18, 0.18, 0.20)
METAL = (0.65, 0.66, 0.68)
STEEL = (0.80, 0.81, 0.84)
BRONZE = (0.80, 0.62, 0.30)
MOTOR = (0.25, 0.27, 0.30)
BEARING = (0.45, 0.50, 0.60)
# Industrial styling (brushed-aluminium links, black pedestal, orange logo).
ALU = (0.76, 0.77, 0.79)
CAP = (0.60, 0.61, 0.63)
BLACK = (0.07, 0.07, 0.08)
ORANGE = (0.93, 0.55, 0.12)
BLUE = (0.10, 0.35, 0.85)
LABEL = (0.25, 0.25, 0.27)
METALLIC = {ALU, CAP, METAL, STEEL}
LOGO = "KLiK"
LOGO_FONT = "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"

X, Y, Z = V(1, 0, 0), V(0, 1, 0), V(0, 0, 1)
RATIO = 100
LIMITS = [(-175, 175), (-70, 90), (-135, 70), (-170, 170), (-115, 115), (-3600, 3600)]
HD_SIZES = [17, 20, 14, 11, 8, 5]
# J5 axis to tool flange (mm). Below ~50 the J6 motor runs into the J5 drive.
WRIST = 50
# Forearm slot between the fork arms starts here (x from J4), so J5 reaches +-115.
SLOT_X = 30
# Double-supported J2/J3: a second upper-arm plate on +Y (y = 40..40+PLATE_T)
# rides on trunnions from the turret and the elbow, so each clevis joint is
# held on both sides (drive + CRB on -Y, support bearing on +Y). The +Y plate
# starts at y = 40 because the size-20 J2 motor runs to y = +35.5.
SUPPORT_R = 12          # trunnion radius
SUPPORT_Y = 40          # inner face of the +Y plate

# Outer shells made see-through by xray() so the drives show.
HOUSINGS = ["Base", "Base_ring", "Turret", "Shoulder_covers", "UpperArm", "UpperArm_covers",
            "Elbow", "Elbow_covers", "Forearm", "Ring4", "Wrist",
            "Logo_base", "Logo_upper", "Logo_forearm", "Label_turret",
            "Upper_cover", "Turret_covers", "Forearm_cover", "Elbow_cover"]   # last row: klik_freecad   # absent ones are skipped


def cyl(r, h, base, axis):
    return Part.makeCylinder(r, h, base, axis)


def ycyl(r, length, centre):
    """Cylinder along Y, centred on `centre`."""
    return cyl(r, length, centre - V(0, length / 2, 0), Y)


def box(lx, ly, lz, corner):
    return Part.makeBox(lx, ly, lz, corner)


def fuse(*shapes):
    s = shapes[0]
    for t in shapes[1:]:
        s = s.fuse(t)
    return s.removeSplitter()


def moved(shape, placement):
    s = shape.copy()
    s.transformShape(placement.toMatrix())
    return s


def soft(shape, r, edges=None):
    """Fillet `edges` (default all) at r, backing off if OCC refuses."""
    edges = shape.Edges if edges is None else edges
    for rr in (r, 0.6 * r, 0.35 * r):
        try:
            out = shape.makeFillet(rr, edges)
            if out.isValid():
                return out
        except Exception:
            pass
    App.Console.PrintWarning("fillet r=%g failed, left sharp\n" % r)
    return shape


def is_along(edge, axis):
    d = edge.Vertexes[-1].Point - edge.Vertexes[0].Point
    return len(edge.Vertexes) == 2 and d.Length > 0 and abs(abs(d.normalize() * axis) - 1) < 1e-6


def on_plane(edge, x=None, y=None, z=None):
    bb = edge.BoundBox
    return all(v is None or (abs(lo - v) < 1e-6 and abs(hi - v) < 1e-6)
               for v, lo, hi in ((x, bb.XMin, bb.XMax), (y, bb.YMin, bb.YMax), (z, bb.ZMin, bb.ZMax)))


def hull_xz(c1, r1, c2, r2, width):
    """Tangent hull of two circles in the XZ plane, extruded `width` along Y
    (centred) - a tapered link joining two Y-axis bosses."""
    d = c2 - c1
    phi = math.atan2(d.z, d.x)
    alpha = math.acos((r1 - r2) / d.Length)

    def at(c, r, ang):
        return V(c.x + r * math.cos(ang), 0, c.z + r * math.sin(ang))

    a, m1, b = at(c1, r1, phi + alpha), at(c1, r1, phi + math.pi), at(c1, r1, phi - alpha)
    c, m2, e = at(c2, r2, phi - alpha), at(c2, r2, phi), at(c2, r2, phi + alpha)
    wire = Part.Wire([Part.Arc(a, m1, b).toShape(), Part.LineSegment(b, c).toShape(),
                      Part.Arc(c, m2, e).toShape(), Part.LineSegment(e, a).toShape()])
    return Part.Face(wire).extrude(V(0, width, 0)).translated(V(0, -width / 2, 0))


def revolve_z(before, arc, after):
    """Revolve a closed XZ profile (polyline, 3-point arc, polyline) about Z."""
    edges = [Part.LineSegment(p, q).toShape() for p, q in zip(before, before[1:])]
    edges.append(Part.Arc(*arc).toShape())
    pts = after + [before[0]]
    edges += [Part.LineSegment(p, q).toShape() for p, q in zip(pts, pts[1:])]
    return Part.Face(Part.Wire(edges)).revolve(V(0, 0, 0), Z, 360)


def rrect_wire(half, r, z):
    """Rounded-square wire (side 2*half, corner radius r) in the plane z."""
    b = box(2 * half, 2 * half, 1, V(-half, -half, z))
    b = soft(b, r, [e for e in b.Edges if is_along(e, Z)])
    return [f for f in b.Faces if abs(f.CenterOfMass.z - z) < 1e-6][0].OuterWire


def logo(height, depth, origin, up, normal, text=LOGO):
    """Raised lettering: `text` centred on `origin`, letter tops toward `up`,
    standing `depth` along `normal` (half of it sunk into the face), so it
    reads correctly when the face is viewed from outside."""
    x_dir = up.cross(normal)
    import os
    if not os.path.exists(LOGO_FONT):
        return None
    try:
        chars = Part.makeWireString(text, LOGO_FONT, height, 0)
    except TypeError:
        d, f = os.path.split(LOGO_FONT)
        chars = Part.makeWireString(text, d + "/", f, height, 0)
    wires = [w for ch in chars for w in ch]
    face = Part.makeFace(wires, "Part::FaceMakerBullseye")
    bb = face.BoundBox
    face.translate(V(-(bb.XMin + bb.XMax) / 2, -(bb.YMin + bb.YMax) / 2, -depth / 2))
    solid = face.extrude(V(0, 0, depth))
    rot = App.Rotation(x_dir, normal.cross(x_dir), normal, "ZXY")
    return moved(solid, App.Placement(origin, rot))


def rrect_x(hy, hz, r, x0, length):
    """Box along X (y +-hy, z +-hz) with its long edges rounded to r."""
    b = box(length, 2 * hy, 2 * hz, V(x0, -hy, -hz))
    return soft(b, r, [e for e in b.Edges if is_along(e, X)])


def add_shape(doc, container, name, shape, colour):
    f = doc.addObject("Part::Feature", name)
    f.Shape = shape
    container.addObject(f)
    if App.GuiUp:
        f.ViewObject.ShapeColor = colour
        try:                                    # brushed-metal sheen
            mat = App.Material()
            mat.DiffuseColor = colour
            mat.AmbientColor = tuple(0.3 * c for c in colour)
            spec = 0.55 if colour in METALLIC else 0.15
            mat.SpecularColor = (spec, spec, spec)
            mat.Shininess = 0.45 if colour in METALLIC else 0.2
            f.ViewObject.ShapeAppearance = (mat,)
        except Exception:
            pass
    return f


def rot_placement_expr(offset, axis, angle):
    return ("create(<<placement>>; create(<<vector>>; {}; {}; {}); "
            "create(<<rotation>>; create(<<vector>>; {}; {}; {}); {}))").format(
        offset.x, offset.y, offset.z, axis.x, axis.y, axis.z, angle)


def joint_part(doc, parent, name, offset, axis, q):
    p = doc.addObject("App::Part", name)
    parent.addObject(p)
    p.setExpression("Placement", rot_placement_expr(offset, axis, "Joints." + q))
    return p


# --- harmonic drive -------------------------------------------------------

def toothed(ra, rb, n, sx=1.0, sy=1.0):
    """Closed tooth outline: radius ra on the tooth flats, rb in the gaps."""
    pts = []
    for i in range(n):
        for frac, r in ((0.0, rb), (0.25, ra), (0.5, ra), (0.75, rb)):
            a = 2 * math.pi * (i + frac) / n
            pts.append(V(sx * r * math.cos(a), sy * r * math.sin(a), 0))
    pts.append(pts[0])
    return Part.makePolygon(pts)


def prism(wire, z0, h):
    return Part.Face(wire).extrude(V(0, 0, h)).translated(V(0, 0, z0))


def ellipse_prism(a, b, z0, h):
    e = Part.Ellipse(V(0, 0, 0), a, b).toShape()
    return Part.Face(Part.Wire(e)).extrude(V(0, 0, h)).translated(V(0, 0, z0))


def drive_shapes(size, motor_len=None):
    """Harmonic drive + frameless motor + crossed-roller bearing in the drive
    frame: axis +Z from motor (input) towards the output, CS mid-plane at z=0.
    The output hub stands 4 mm proud of the running gap, which sits at
    z = z_out - 4; the crossed-roller bearing (CRB) sits just inboard of the
    gap, outer race in the housing link, inner race on the output."""
    p = 2.54 * size                 # FS pitch diameter
    n = 2 * RATIO                   # FS teeth; CS has n + 2
    m = p / n                       # module
    rp = p / 2
    k = m / rp                      # elliptical deflection (radial = 1 module)
    w = 0.3 * p                     # tooth face width
    t = max(0.02 * p, 0.4)          # flexspline wall
    rcs = rp + m
    r_in = rp - 0.7 * m - t         # FS bore = WG bearing outer
    rc = rp - 0.7 * m               # FS cup outer radius
    lc = 0.75 * p                   # cup length
    rm, lm = 0.62 * p, motor_len or max(0.35 * p, 10)   # short, wide frameless motor
    z_mf = -w / 2 - 2               # motor front face
    z_out = lc + 2 * t + 3          # output face
    b = 0.2 * p                     # CRB width
    rb_i, rb_o = rc + 1, 0.7 * p    # CRB bore / OD
    rb_m = (rb_i + rb_o) / 2
    z_b = z_out - 5 - b             # CRB inboard face

    s = {"z_out": z_out}
    s["cs"] = cyl(0.7 * p, w, V(0, 0, -w / 2), Z).cut(
        prism(toothed(rcs - 0.7 * m, rcs + 0.7 * m, n + 2), -w / 2 - 1, w + 2))
    s["fs_rim"] = prism(toothed(rp + 0.7 * m, rp - 0.7 * m, n, 1 + k, 1 - k), -w / 2, w).cut(
        ellipse_prism(r_in * (1 + k), r_in * (1 - k), -w / 2 - 1, w + 2))
    s["fs_cup"] = fuse(
        cyl(rc, lc - w / 2, V(0, 0, w / 2), Z).cut(cyl(rc - t, lc, V(0, 0, w / 2 - 1), Z)),
        cyl(rc, 2 * t, V(0, 0, lc), Z),
        cyl(0.22 * p, z_out - lc, V(0, 0, lc), Z),
    )
    rwb = r_in - 0.05 * p
    s["wg_bearing"] = ellipse_prism(r_in * (1 + k), r_in * (1 - k), -0.45 * w, 0.9 * w).cut(
        ellipse_prism(rwb * (1 + k), rwb * (1 - k), -0.45 * w - 1, 0.9 * w + 2))
    s["wg_plug"] = fuse(
        ellipse_prism(rwb * (1 + k), rwb * (1 - k), -0.45 * w, 0.9 * w),
        cyl(max(0.06 * p, 1), -z_mf + 1, V(0, 0, z_mf - 1), Z),
    )
    s["motor"] = fuse(
        cyl(rm, lm, V(0, 0, z_mf - lm), Z),
        cyl(rm * 0.5, 1, V(0, 0, z_mf - 0.5), Z),
    )
    s["crb_outer"] = cyl(rb_o, b, V(0, 0, z_b), Z).cut(cyl(rb_m + 0.15, b + 2, V(0, 0, z_b - 1), Z))
    s["crb_inner"] = cyl(rb_m - 0.15, b, V(0, 0, z_b), Z).cut(cyl(rb_i, b + 2, V(0, 0, z_b - 1), Z))
    s["envelope"] = cyl(0.7 * p + 0.5, z_out - z_mf + lm + 1, V(0, 0, z_mf - lm - 0.5), Z)
    # Without the coaxial motor (motor offset, e.g. belt-driven): gear only.
    s["envelope_nomotor"] = cyl(0.7 * p + 0.5, z_out - z_mf + 2.5, V(0, 0, z_mf - 1.5), Z)
    s["z_mf"] = z_mf
    return s


def add_drive(doc, j, housing, output, gap, out_offset, toward, axis, q_sign=1, size=None, motor=True,
              motor_len=None):
    """Place joint j's drive across the running gap at `gap` (housing frame).

    housing    - link holding the CS, motor and CRB outer race
    output     - link driven by the FS cup, carrying the CRB inner race
    out_offset - add to housing-frame coords to get output-frame coords (q=0)
    toward     - unit vector from housing to output along the joint axis
    q_sign     - +1 if the output turns +q about `axis` relative to the
                 housing (output outboard), -1 if it turns -q (output inboard)
    Returns the drive envelope in housing and output frames, for boring."""
    s = drive_shapes(size or HD_SIZES[j - 1], motor_len)
    rot = App.Rotation(Z, toward)
    origin = gap - toward * (s["z_out"] - 4)
    pl_h = App.Placement(origin, rot)
    pl_o = App.Placement(origin + out_offset, rot)

    drv = doc.addObject("App::Part", "J%d_drive" % j)
    housing.addObject(drv)
    drv.Placement = pl_h
    add_shape(doc, drv, "J%d_CS" % j, s["cs"], METAL)
    if motor:
        add_shape(doc, drv, "J%d_motor" % j, s["motor"], MOTOR)
    add_shape(doc, drv, "J%d_CRB_outer" % j, s["crb_outer"], BEARING)

    # WG turns -RATIO x the output's rotation about the drive's +Z.
    coef = -RATIO * q_sign * round(toward * axis)
    wg = doc.addObject("App::Part", "J%d_wave" % j)
    drv.addObject(wg)
    wg.setExpression("Placement", rot_placement_expr(V(0, 0, 0), Z, "%d * Joints.q%d" % (coef, j)))
    add_shape(doc, wg, "J%d_WG_plug" % j, s["wg_plug"], BRONZE)
    add_shape(doc, wg, "J%d_WG_bearing" % j, s["wg_bearing"], STEEL)
    add_shape(doc, wg, "J%d_FS_teeth" % j, s["fs_rim"], STEEL)

    add_shape(doc, output, "J%d_FS_cup" % j, moved(s["fs_cup"], pl_o), STEEL)
    add_shape(doc, output, "J%d_CRB_inner" % j, moved(s["crb_inner"], pl_o), BEARING)
    env = s["envelope"] if motor else s["envelope_nomotor"]
    return moved(env, pl_h), moved(env, pl_o)


# --- arm ------------------------------------------------------------------

def build(doc_name="Meca500", xray_on=True, double=False, plate_t=32, hand=1, gripper=True):
    """double=True adds the second upper-arm plate and J2/J3 support trunnions;
    plate_t is the thickness of each upper-arm plate (baseline 32).
    hand=+1 puts the side link (and the J5 motor) on +Y, as in the KLiK
    reference photo; hand=-1 is the original -Y layout the FEA study used.
    gripper=True fits a parallel gripper to the flange."""
    M = -hand                       # multiplies every y written for the -Y layout
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)

    sheet = doc.addObject("Spreadsheet::Sheet", "Joints")
    for c, h in zip("ABCDEF", ("Joint", "Angle (deg)", "Min", "Max", "HD size", "Ratio")):
        sheet.set(c + "1", h)
    for i, (lo, hi) in enumerate(LIMITS, 1):
        r = str(i + 1)
        sheet.set("A" + r, "J%d" % i)
        sheet.set("B" + r, "0")
        sheet.set("C" + r, str(lo))
        sheet.set("D" + r, str(hi))
        sheet.set("E" + r, str(HD_SIZES[i - 1]))
        sheet.set("F" + r, str(RATIO))
        sheet.setAlias("B" + r, "q%d" % i)

    root = doc.addObject("App::Part", "Meca500")
    l1 = joint_part(doc, root, "Link1", V(0, 0, 0), Z, "q1")
    l2 = joint_part(doc, l1, "Link2", V(0, 0, 135), Y, "q2")
    l3 = joint_part(doc, l2, "Link3", V(0, 0, 135), Y, "q3")
    l4 = joint_part(doc, l3, "Link4", V(60, 0, 38), X, "q4")
    l5 = joint_part(doc, l4, "Link5", V(60, 0, 0), Y, "q5")
    l6 = joint_part(doc, l5, "Link6", V(WRIST, 0, 0), X, "q6")

    # Drives: (joint, parent, child, drive origin in parent, joint origin, axis)
    e1p, e1c = add_drive(doc, 1, root, l1, V(0, 0, 90), V(0, 0, 0), Z, Z)
    # J2/J3: the upper arm is a side link (|y| 32..64) beside the centred
    # turret and elbow (|y| <= 30), so each drive bridges a 2 mm running gap.
    e2p, e2c = add_drive(doc, 2, l1, l2, V(0, -31 * M, 135), V(0, 0, -135), -Y * M, Y)
    # J3 is housed in the 60 mm-wide elbow and drives the (thin) upper arm.
    e3c, e3p = add_drive(doc, 3, l3, l2, V(0, -31 * M, 0), V(0, 0, 135), -Y * M, Y, q_sign=-1)
    e4p, e4c = add_drive(doc, 4, l3, l4, V(60, 0, 38), V(-60, 0, -38), X, X)
    e5p, e5c = add_drive(doc, 5, l4, l5, V(60, -16 * M, 0), V(-60, 0, 0), Y * M, Y)
    e6p, e6c = add_drive(doc, 6, l5, l6, V(WRIST - 6, 0, 0), V(-WRIST, 0, 0), X, X)

    # Base (fixed): black rounded-square pedestal on a slightly larger foot.
    foot = box(124, 124, 8, V(-62, -62, 0))
    foot = soft(foot, 14, [e for e in foot.Edges if is_along(e, Z)])
    foot = soft(foot, 2.5, [e for e in foot.Edges if on_plane(e, z=8)])
    pedestal = Part.makeLoft([rrect_wire(56, 18, 8), rrect_wire(54, 17, 84)], True)
    pedestal = soft(pedestal, 5, [e for e in pedestal.Edges if on_plane(e, z=84)])
    add_shape(doc, root, "Base", foot.fuse(pedestal).removeSplitter().cut(e1p), BLACK)
    add_shape(doc, root, "Base_ring", cyl(47, 6, V(0, 0, 84), Z).cut(e1p), CAP)
    lb = logo(23, 1.5, V(55.3, 0, 38), Z, X)
    if lb:
        add_shape(doc, root, "Logo_base", lb, ORANGE)

    # Link 1: turret - J1 disc plus a vertical block rising into the shoulder
    # boss (|y| -30..38; runs long on the motor side for the size-20 J2 motor).
    yb = min(-30 * M, 38 * M)
    block = box(84, 68, 40, V(-42, yb, 95))
    block = soft(block, 10, [e for e in block.Edges if is_along(e, Y) and on_plane(e, z=95)])
    turret = soft(fuse(cyl(45, 5, V(0, 0, 90), Z), block, ycyl(42, 68, V(0, 4 * M, 135))), 4)
    turret = turret.cut(e1c).cut(e2p)
    lt = logo(7, 1.0, V(-42.1, 0, 113), Z, -X, text="MDC 1")
    if lt:
        add_shape(doc, l1, "Label_turret", lt, LABEL)
    if double:
        trunnion = ycyl(SUPPORT_R, 12, V(0, 44 * M, 135))      # |y| 38..50
        add_shape(doc, l1, "Turret", turret.fuse(trunnion).removeSplitter(), ALU)
    else:
        add_shape(doc, l1, "Turret", turret, ALU)
        add_shape(doc, l1, "Shoulder_covers", ycyl(34, 3, V(0, 39.5 * M, 135)), CAP)

    # Link 2: parallel-sided stadium side link at |y| 32..64, clearing the
    # turret and the elbow so both J2 and J3 can turn freely; the r38
    # shoulder boss (size-20 J2 output) clears the J1 disc by 2 mm.
    profile = soft(hull_xz(V(0, 0, 0), 38, V(0, 0, 135), 36, plate_t), 10)
    upper = profile.translated(V(0, (-32 - plate_t / 2) * M, 0)).cut(e2c).cut(e3p)
    yc = (-32 - plate_t - 1.5) * M
    covers = ycyl(35, 3, V(0, yc, 0)).fuse(ycyl(33, 3, V(0, yc, 135)))
    if double:
        plate = profile.translated(V(0, (SUPPORT_Y + plate_t / 2) * M, 0))
        for z in (0, 135):                                      # blind bearing bores
            plate = plate.cut(ycyl(SUPPORT_R + 0.5, 12, V(0, (SUPPORT_Y + 6) * M, z)))
        upper = upper.fuse(plate)
        yc = (SUPPORT_Y + plate_t + 1.5) * M
        covers = covers.fuse(ycyl(35, 3, V(0, yc, 0))).fuse(ycyl(33, 3, V(0, yc, 135)))
    add_shape(doc, l2, "UpperArm", upper, ALU)
    add_shape(doc, l2, "UpperArm_covers", covers, CAP)
    # Lettering runs down the arm from the elbow, tops toward the back.
    lu = logo(28, 1.5, V(0, (-32 - plate_t + 0.25) * M, 67.5), -X, -Y * M)
    if lu:
        add_shape(doc, l2, "Logo_upper", lu, ORANGE)

    # Link 3: elbow sweeping up and forward into a square-section J4 housing.
    side = soft(hull_xz(V(0, 0, 0), 30, V(6, 0, 38), 14, 52), 8)   # stays inside the nose
    nose = rrect_x(30, 28, 11, -12, 72).translated(V(0, 0, 38))
    nose = soft(nose, 3, [e for e in nose.Edges if on_plane(e, x=60)])
    nose = soft(nose, 8, [e for e in nose.Edges if on_plane(e, x=-12)])
    elbow = fuse(ycyl(32, 60, V(0, 0, 0)), side, nose).cut(e3c).cut(e4p)
    if double:
        trunnion = ycyl(SUPPORT_R, 20, V(0, 40 * M, 0))         # |y| 30..50
        add_shape(doc, l3, "Elbow", elbow.fuse(trunnion).removeSplitter(), ALU)
    else:
        add_shape(doc, l3, "Elbow", elbow, ALU)
        add_shape(doc, l3, "Elbow_covers", ycyl(28, 3, V(0, 31.5 * M, 0)), CAP)

    # Link 4: square-section forearm continuing the elbow housing, ending in
    # a rounded wrist fork; J5 motor boss on the side-link side.
    body = rrect_x(25, 24, 9, 0, 40)
    fork = soft(hull_xz(V(36, 0, 0), 24, V(60, 0, 0), 24, 9), 3)
    boss = ycyl(16, 23, V(60, -36.5 * M, 0))
    boss = soft(boss, 3, [e for e in boss.Edges if on_plane(e, y=-48 * M)])
    forearm = fuse(body, fork.translated(V(0, 20.5, 0)), fork.translated(V(0, -20.5, 0)), boss)
    forearm = forearm.cut(ycyl(17.5, 32, V(60, 0, 0)))
    forearm = forearm.cut(box(50, 32, 52, V(SLOT_X, -16, -26)))   # J5 swing slot
    # J5 outboard support: plain bearing pair on the far fork arm, so the
    # wrist is carried on both sides instead of cantilevered off the J5 drive.
    forearm = forearm.cut(ycyl(9.2, 7, V(60, 19.5 * M, 0)))
    add_shape(doc, l4, "J5_support_outer",
              ycyl(9, 5, V(60, 19.5 * M, 0)).cut(ycyl(7.15, 7, V(60, 19.5 * M, 0))), BEARING)
    add_shape(doc, l4, "Forearm", forearm.cut(e4c).cut(e5p), ALU)
    add_shape(doc, l4, "Ring4", rrect_x(25.6, 24.6, 9.3, 0, 3).cut(e4c), DARK)
    lf = logo(11, 1.2, V(18, (-25 + 0.2) * M, 0), Z, -Y * M)
    if lf:
        add_shape(doc, l4, "Logo_forearm", lf, ORANGE)

    # Link 5: wrist hub with a tapering nose to the flange.
    hub = soft(ycyl(17, 31, V(0, 0, 0)), 3)
    nose = Part.makeCone(15, 12, WRIST - 6, V(0, 0, 0), X)
    wrist = fuse(hub, nose)
    add_shape(doc, l5, "J5_support_inner",
              fuse(ycyl(5, 8, V(0, 19 * M, 0)), ycyl(6.85, 5, V(0, 19.5 * M, 0))), BEARING)
    add_shape(doc, l5, "Wrist", wrist.cut(e5c).cut(e6p), ALU)

    # Link 6: ISO-style tool flange.
    flange = cyl(16, 6, V(-6, 0, 0), X)
    for i in range(4):
        a = math.radians(45 + 90 * i)
        flange = flange.cut(cyl(1.6, 8, V(-7, 10 * math.cos(a), 10 * math.sin(a)), X))
    flange = flange.cut(cyl(3, 8, V(-7, 0, 0), X))
    add_shape(doc, l6, "Flange", flange, METAL)
    add_shape(doc, l6, "Index_pin", cyl(1.5, 3, V(0, 0, 13), X), DARK)
    if gripper:
        # Parallel gripper: adaptor, black collar, body, two blue fingers.
        add_shape(doc, l6, "Gripper_adaptor", cyl(15, 10, V(0, 0, 0), X), METAL)
        add_shape(doc, l6, "Gripper_collar", cyl(16, 4, V(10, 0, 0), X), DARK)
        gbody = soft(box(30, 38, 30, V(14, -19, -15)), 4)
        add_shape(doc, l6, "Gripper_body", gbody, ALU)
        lg = logo(5, 0.6, V(29, 0, 15.1), -X, Z, text="6E7 43")
        if lg:
            add_shape(doc, l6, "Gripper_label", lg, LABEL)
        for sgn in (1, -1):
            finger = soft(box(26, 9, 16, V(44, (3 if sgn > 0 else -12), -8)), 3)
            add_shape(doc, l6, "Gripper_finger_%s" % ("a" if sgn > 0 else "b"), finger, BLUE)

    doc.recompute()
    if App.GuiUp:
        import FreeCADGui as Gui
        xray(xray_on, doc_name)
        Gui.ActiveDocument.ActiveView.viewIsometric()
        Gui.SendMsgToActiveView("ViewFit")
        refresh_panel(doc_name)
    return doc


def refresh_panel(doc_name):
    """After a rebuild, reopen the slider panel if one is showing, so its
    values match the new document."""
    if not App.GuiUp:
        return
    import FreeCADGui as Gui
    from PySide import QtWidgets
    if Gui.getMainWindow().findChild(QtWidgets.QDockWidget, "Meca500Joints"):
        show_panel(doc_name)


def xray(on=True, doc_name="Meca500"):
    """Make the outer housings see-through (on) or opaque (off)."""
    import re
    doc = App.getDocument(doc_name)
    # Everything solid except the drives/actuators (J<n>_...), the cabling (flex strips,
    # connector boards) and the Meca500 reference model: a fixed name list missed most
    # of the KLiK3 parts (split upper arm, cheeks, lid, collar, covers, screws...).
    keep = re.compile(r"^(J\d_(?!cap)|Flex|Board|Meca)")   # (J5_cap is wrist housing)
    names = set(HOUSINGS) | {o.Name for o in doc.Objects
                             if o.TypeId.startswith("Part::") and not keep.match(o.Name)}
    for name in names:
        obj = doc.getObject(name)
        if obj and getattr(obj, "ViewObject", None) is not None and hasattr(obj.ViewObject, "Transparency"):
            obj.ViewObject.Transparency = 75 if on else 0


def photo_view(width=1376, height=768, path=None, doc_name="Meca500"):
    """Pose the arm and camera like the KLiK reference photo (J1 turned so
    the side link faces the camera, wrist pointing down), optionally saving
    a render to `path`."""
    import FreeCADGui as Gui
    pose(-90, 27, 40, 0, 23, 0, doc_name=doc_name)
    v = Gui.getDocument(doc_name).ActiveView
    v.setCameraType("Perspective")
    az, el = math.radians(18), math.radians(14)
    zc = V(math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
    xc = Z.cross(zc).normalize()
    v.setCameraOrientation(App.Rotation(xc, zc.cross(xc), zc, "ZXY"))
    v.fitAll()
    if path:
        v.saveImage(path, width, height, "White")


def pose(*angles, doc_name="Meca500"):
    """pose(j1, j2, j3, j4, j5, j6) in degrees."""
    doc = App.getDocument(doc_name)
    sheet = doc.getObject("Joints")
    for i, a in enumerate(angles, 1):
        sheet.set("B%d" % (i + 1), str(a))
    doc.recompute()


# --- slider panel ---------------------------------------------------------

def show_panel(doc_name="Meca500"):
    """Dock a panel with one slider + spin box per joint."""
    import FreeCADGui as Gui
    from PySide import QtCore, QtWidgets

    mw = Gui.getMainWindow()
    old = mw.findChild(QtWidgets.QDockWidget, "Meca500Joints")
    if old:
        old.setObjectName("")           # so lookups find the new panel, not this one
        mw.removeDockWidget(old)
        old.deleteLater()

    dock = QtWidgets.QDockWidget("Meca500 joints", mw)
    dock.setObjectName("Meca500Joints")
    body = QtWidgets.QWidget()
    grid = QtWidgets.QGridLayout(body)
    sheet = App.getDocument(doc_name).getObject("Joints")
    rows = []

    def push():
        # Look the sheet up every time: a rebuild replaces the document, and a
        # captured sheet would then be a deleted object.
        if doc_name not in App.listDocuments():
            return
        doc = App.getDocument(doc_name)
        sh = doc.getObject("Joints")
        for i, (_, spin) in enumerate(rows, 1):
            sh.set("B%d" % (i + 1), str(spin.value()))
        doc.recompute()

    lims = []
    for i, (lo, hi) in enumerate(LIMITS, 1):
        try:                            # the document's own limits, if its sheet has them
            lo, hi = int(float(sheet.get("C%d" % (i + 1)))), int(float(sheet.get("D%d" % (i + 1))))
        except Exception:
            pass
        if i == 6:
            lo, hi = -360, 360          # J6 is +-3600; keep the slider usable
        lims.append((lo, hi))
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(lo, hi)
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(1)
        spin.setSuffix(" °")
        start = float(sheet.get("q%d" % i))
        slider.setValue(round(start))
        spin.setValue(start)
        slider.valueChanged.connect(lambda v, s=spin: s.setValue(v))
        spin.valueChanged.connect(
            lambda v, sl=slider: (sl.blockSignals(True), sl.setValue(round(v)),
                                  sl.blockSignals(False), push()))
        grid.addWidget(QtWidgets.QLabel("J%d" % i), i - 1, 0)
        grid.addWidget(slider, i - 1, 1)
        grid.addWidget(spin, i - 1, 2)
        rows.append((slider, spin))

    home = QtWidgets.QPushButton("Home")
    home.clicked.connect(lambda: [spin.setValue(0) for _, spin in rows])
    xr = QtWidgets.QCheckBox("X-ray housings")
    xr.setChecked(App.getDocument(doc_name).getObject("Turret").ViewObject.Transparency > 0)
    xr.toggled.connect(lambda on: xray(on, doc_name))
    grid.addWidget(home, 6, 1)
    grid.addWidget(xr, 6, 2)

    def set_free(on):
        # explore beyond the joint limits (sheet C/D are left as the real limits)
        for (slider, spin), (lo, hi), j in zip(rows, lims, range(1, 7)):
            if on:
                lo, hi = (-360, 360) if j == 6 else (-180, 180)
            for w in (slider, spin):
                w.blockSignals(True)
                w.setRange(lo, hi)
                w.blockSignals(False)
            slider.blockSignals(True)
            slider.setValue(round(spin.value()))
            slider.blockSignals(False)
        push()                          # a value clamped back into range moves the model too
    free = QtWidgets.QCheckBox("Free (ignore joint limits)")
    free.setObjectName("FreeLimits")
    free.toggled.connect(set_free)
    grid.addWidget(free, 7, 1, 1, 2)
    grid.setRowStretch(8, 1)

    dock.setWidget(body)
    mw.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
    dock.show()
    return dock


if __name__ == "__main__":
    build()
    if App.GuiUp:
        show_panel()
