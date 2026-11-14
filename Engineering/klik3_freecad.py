"""KLiK3: a from-scratch model of the KLiK robot, matched to the studio views
(front, back, left, right and a 3/4 view).

Proportions are measured off the photos, scaled so J2-J3 = 135 mm (which
puts J3-J5 at 127 mm, matching the Meca500-style layout). The robot:

  - base: black rounded-square block with a lower connector box behind it
    (KLiK on its right-hand face), on a chamfered square plate and a round
    foot disc; two cables loop from the connectors to the disc
  - turret: a clevis whose two cheeks are round-topped about J2, each with a
    stadium cover
  - upper arm: a centre spine inside the clevis, splitting into two side
    plates up to J3, each with a long stadium KLiK cover
  - elbow: a round J3 drum between the upper-arm plates and a narrower
    bevelled box along the J4 axis, reaching back past J3
  - forearm: a rounded-edged box with a semicircular end about J5, a wide
    window through the end for the wrist, and a pill-shaped pocket in each
    side holding a KLiK cover; J5 is direct: gear and coaxial motor sit in
    the wrist drum, with a support bearing in the near forearm wall
  - wrist drum and block and a stepped tool flange

Covers are on the +Y side as "right", matching the photo labelled RIGHT SIDE.
Reuses the drive and helper code from meca500_freecad / klik_freecad.

Run inside FreeCAD (or via the MCP bridge): build(), photo_views(), show_panel().
"""
import importlib
import math

import FreeCAD as App
import Part
from FreeCAD import Vector as V

import meca500_freecad as m
import klik_freecad as k1
from meca500_freecad import (X, Y, Z, add_drive, add_shape, box, cyl, drive_shapes, fuse,
                             hull_xz, is_along, joint_part, logo, on_plane, soft, ycyl)

DOC = "KLiK3"
WORLD_Z = -8                      # whole robot shifted down so the base plate top (8 thick) sits at world z = 0
STAINLESS, COVER = k1.STAINLESS, k1.COVER
ANODISED_GREEN = (0.780, 0.835, 0.470)   # #C7D578 (lifted from #B4C06E: FreeCAD shading read it olive): lime/pistachio anodised aluminium, sampled from the user's coffee-pod photo 20261111_153830.jpg (lit faces #C2C876..#A1A365) (user 2026-11-12)
ARM_FINISH = COVER_FINISH = ANODISED_GREEN   # link bodies, turret and covers; Flange, screws, bearings stay stainless/steel
m.METALLIC.add(ANODISED_GREEN)
LOGO = (0.988, 0.059, 0.753)       # KLiK lettering: shocking pink (#FC0FC0)
BLACK, PLATE, CABLE = k1.BLACK, k1.PLATE, k1.CABLE

# --- dimensions (mm), from the photos ------------------------------------------
DISC_T = 0                        # (foot disc removed: the square plate sits on the ground)
PLATE_X, PLATE_Y, PLATE_T = (-83, 60), 58, 8   # (was (-83, 50), 48: grown with the r51 column, 2026-10-25)
BASE_TOP = 84                     # top of the black block (plate top at 18; photo: 66 tall)
BASE_HALF = round(math.hypot(40, 32.5), 1) + 0.5   # black column about J1, flush with the turret collar (COLLAR_R + 0.5; 40 = CHEEK_R)
BOX_X, BOX_Y, BOX_TOP = -71, (-9, BASE_HALF), 72     # connector box behind the column, side flush with it (top 12 below)
J1_GAP = BASE_TOP + 1
J2X = -20                         # J2 axis this far behind J1 (along the arm plane's x)
J2Z = J1_GAP + 78                 # J2 height (photo: 50 above the turret bottom; +10 taller cheeks so the arm clears the collar to J2 -95; +3 for the r44 FHA-11C hub over the collar; +15 taller cheeks so the hub doesn't sit straight on the collar, user 2026-10-25)
CHEEK_R, TURRET_HALF, SLOT_Y = 40, 47, 32.5      # (45/30.5 -> 47/32.5: arm widened 2 mm a side so the J2/J3 bus can pass through the -Y support bearings' centres, 2026-10-26) turret cheeks about J2 (photo cover r ~31.6), clevis slot (was 28: 56.5 mm RSA-8A at J2; TURRET_HALF 41 -> 45: 16 mm cheeks for stiffness)
J2_BARREL_Y = SLOT_Y - 1.5   # J2 drum face (= SPINE_Y now the whole spine is this wide; the local barrel boss was dropped) J2 barrel on the spine (like the elbow's J3 drum): closes the J2 actuator's -Y end (2.5 mm skin); slot widened to 30.5 for it
COLLAR_H = 16                     # turret's round collar under the flat-sided cheeks
COLLAR_R = math.hypot(CHEEK_R, SLOT_Y)   # collar radius: its flats at |y| = SLOT_Y come out exactly the cheek width (+-CHEEK_R)
CHEEK_FIX_X, CHEEK_FIX_Z = 33, (8,)   # cheek plates bolt to the collar flats: bolts at x = +-20, these heights above the collar bottom
CHEEK_FIX_D = 4                   # M4 cheek bolts (counterbored under the covers, tapped 10 mm into the collar)
CHEEK_DOWELS = [(20, 12), (-14, 12)]   # 3 mm dowels locating each cheek on its collar flat (x, z above the collar bottom); clear of the bolts and the ribbon riser (x -26); z 4 -> 12: above the J1 support bearing (2026-10-25)
TURRET_POCKET_HI = 95             # the turret clearance pocket in the upper arm is swept only up to this J2 (user: no notch in the arms' end arc)
CHEEK_KINK_R = 25                  # round where the cheek's front flank meets the straight drop to the collar (was 10; user: smoother contour)
CHEEK_KINK_R_BACK = 25             # same at the back flank (was a sharp ~30 deg corner)
CHEEK_COVER_IN = 3.5               # turret cheek covers: this much smaller than the cheek all round, recessed flush
L2 = 135                          # J2 -> J3
SPINE_Y, PLATE_IN, PLATE_OUT = 31, 31.5, 47  # (+2 each, 2026-10-26: bus through the -Y support bearings) upper arm: spine (27.5 -> 29: whole section thickened, 2.5 mm skin over the J2 actuator end incl. its connector box; replaces the local J2 barrel), side plates (were 26.5, 26.5: RSA-8A at J2/J3; PLATE_OUT 41 -> 45: 15.5 mm fork plates for stiffness)
PLATE_R = 34                      # upper-arm J3 end radius (photo cover r ~33.7)
ARM_H0 = 28                       # upper arm: half depth of the band (grows as t^2 to PLATE_R at J3)
J2_HUB_R, HUB_T = 44, 0.4         # J2 hub round the FHA-11C (80 across corners + 0.5 clearance + 3.5 wall); the band swells to it, from t = HUB_T down to J2 (smoothstep: leaves the hub tangentially, no kink)
ARM_SAG = 12                      # upper arm centre line bowed this far to the back at mid-span (Denso-style banana; tried 0 = straight and put back, user 2026-11-07)
ARM_FRONT_LINE = True             # upper arm front (fold-side) edge: one straight line tangent to the J2 hub and J3 end arcs, no concave (user 2026-11-07, with the taller elbow)
CROTCH_HUB_ARC = True             # spine behind the crotch (|y| < PLATE_IN) trimmed to the J2 hub arc r J2_HUB_R, as the turret cheeks, back to where it meets the crotch floor (user 2026-11-07)
PLATE_END_HULL = True             # fork plates' lower end: the cover-concentric end circle wrapped by one tangent line onto a round at the front corner, just clear of the turret cheeks' sweep (replaces the stepped pocket edge; user 2026-11-07)
PLATE_END_RF, PLATE_END_CLEAR = 10, 1.0   # that front-corner round, and its clearance to the cheeks' sweep
PLATE_END_FLOW = False           # OFF 2026-11-12 (user: the cheek should overlap the spine face this cut exposed). fork plates' lower edge = the spine's hub-arc blend line carried straight on from F (user 2026-11-12: "make the upper arm cheeks flow from the highlighted line")
PLATE_FLOW_RF, PLATE_FLOW_RK = 10, 6   # its rounds: into the straight front edge, and into the plates' back end circle
PLATE_BACK_LINE = True           # fork plates' back edge: straight, parallel to the arm's front face and tangent to the r PLATE_R J3 end (parallel-sided plates; user 2026-11-12)
PLATE_BACK_R = 10                # round where that back edge meets the plates' lower end circle
PLATE_STRAP_CUT = True           # fork plates cut behind the line through the lower end's tangent point (the user's highlighted edge, ~(-6, 51.6)) parallel to the front face, keeping the full r PLATE_R disc round J3: back edge runs straight up into the elbow circle (user's modelled cut on a copy, 2026-11-12)
UA_CABLE_DX = 18                 # -Y fork board + fork flex moved this far toward the front (along the arm normal) so they stay inside the strap-cut plate
COVER_FOLLOWS_PLATE = True      # upper-arm covers + their pockets = the fork plate's actual side outline inset 3.5 / 3.2 (as the turret cheek covers), not the old banana pill (user 2026-11-12: cheeks didn't flow with the straight front / hull end)
UA_FORK_POCKETS = False          # rib / floor pockets under the upper-arm covers (removed, user 2026-11-07)
EDGE_CH = 3.0          # chamfer theme (user 2026-11-13: chamfers, not fillets): elbow, Ring4, forearm collar/taper/side edges
PLATE_OUTLINE_CH = 1.0 # chamfer on the fork plates' outer-face outline, applied to the split halves (user 2026-11-13; was a 1 mm round)
# 2026-11-14 (user): break on each fork plate's convex Y-edge at the foot of the crotch-side
# face (upper-arm local x -6.04, z 51.61, y +-31..46). Was a 5 mm chamfer, then (same day) an
# r10 fillet matching the crotch-back fillets. OCC makeChamfer gives an invalid solid there, so
# it is a profile cut along Y (prism_chamfer_y). ((x, z), size, "fillet"|"chamfer"); [] = sharp
FORK_CORNER_CH = [((-6.03619, 51.6131), 10.0, "fillet")]
# 2026-11-14 (user, after FEA rev D put the upper-arm peak there): 1 mm chamfer on the sharp
# concave crotch corners, where each fork plate's inner face (y = +-PLATE_IN) meets the crotch
# floor and the front crotch chamfer face. makeChamfer fails, so it is a lofted filler wedge
# (concave_corner_fill). 0 = sharp
CROTCH_CORNER_CH = 0     # was 1.0 (2026-11-14); user: "looks untidy", undone
ARM_OUTER_R = 0        # fork-plate outer-edge round (y = +-PLATE_OUT) + lower-end ring; 0 = sharp (user 2026-11-13, was 1)
CROTCH_FRONT_CH = 10   # chamfer (legs mm) on the front crotch edge (arm front face / crotch floor, across the spine); user 2026-11-13; 0 = sharp
CROTCH_EDGE_R = ()     # was (10, 8, 6, 5, 4, 3, 2); () = sharp (user 2026-11-12: remove the inner fillets). front/back crotch edges across the spine (where its faces meet the crotch floor): largest that works
CROTCH_FILLET = 0                 # was 4; 0 = sharp (user 2026-11-12). crotch floor fillets (to the arms' inner faces and the front/back faces)
ARM_T0 = 0.6                      # fork arms (and their covers) start here along the centre line (0 = J2, 1 = J3); was 0.45, moved up clear of the r44 cheek hub
J4Z = 74                          # J4 axis above J3 (photo 31; +13 lifts the forearm over the upper arm so J3 reaches 74 at any J4 roll; +30 taller elbow, user 2026-11-07; Meca500 38)
ELBOW_BACK, ELBOW_HH, ELBOW_HW = 34, 28, 24     # elbow box along the J4 axis: reach behind J3, half height, half width (was 21 = photo 42 wide; 24 makes room for the J4 cable loop)
ELBOW_FRONT_FILL = True           # under the elbow box: a vertical face at x = DRUM_R, square to the underside, runs down tangent into the J3 drum (user 2026-11-07)
ELBOW_FRONT_R = 8                 # fillet where that face meets the underside (user 2026-11-07); 0 = sharp
ELBOW_R = 45                      # elbow box top-rear fillet: large, to blend into the upper arm's r34 end
ELBOW_Y, DRUM_R = 29.5, 34        # (27.5 -> 29.5 with the wider arm, 2026-10-26) J3 drum between the upper-arm plates (was 24.5; RSA-8A at J3); r34 = PLATE_R, same as the arm's J3 end (user 2026-10-25, was 32)
J4_GAP_X = 51                     # J4 running gap, along X from J3 (photo seam ~45; +4 so the J4 motor clears the J3 drive)
FW, FH = 38, 28.5                 # forearm half width (y; was 32 = photo; 35 = 70 wide was the minimum for the J5 RSA-8A, 38 = 76 wide gives 6.5 mm side walls; the J5 flex coil lives under the domed +Y cover) and half height (z; photo 57)
COLLAR4_L, TAPER4_L = 13, 18      # forearm: elbow-width collar behind J4, then taper to FW (clears the upper-arm plates)
J4X = 64                          # Link4 frame origin along the forearm from J3 (elbow +4 with J4_GAP_X)
J5X = 75                          # J4 -> J5 along the forearm (photo: J5 135 from J3 along the arm)
SLOT_X, J5_GAP_Y = 28, 29.5       # wrist window: starts 28 past J4, 59 wide (was 43 = photo; RSA-8A at J5)
WINDOW_CH = 10                    # chamfer on the window back face's top and bottom edges (user 2026-11-07); 0 = none
WRIST_R, WRIST_Y, WRIST_BY, WRIST_BZ = 32, 28.5, 18, 21    # J5 drum radius (= forearm end radius; was FH 28.5: the RSA-8A reaches r28.4, its bore broke through; 2026-10-25) / half width, block half width / half depth
FLANGE_FACE = 11.0                # J6 origin -> flange mounting face (panel/IK point when the Flange part isn't there)
WRIST = 71                        # J5 -> J6 origin (was 46.5; the 48.5 mm RSF-5B at J6 must clear the J5 RSA-8A; Meca500: 70)
HD_SIZES = [17, 14, 14, 8, 11, 5]   # J2 14: slim cheeks; J4 8: clears the J3 gear (J4 sits 31 above J3); J5 11 in the r28.5 drum
                                  # (Meca500 actuator sizes [8, 8, 8, 8, 8, 5] tried 2026-10-22: J2 too weak for this arm)
USE_ACTUATORS = True              # Meca500-style Harmonic Drive actuators with integrated servo drive + CANopen
# per joint: (model, connector-box direction in the housing frame); False = the component HD sets in HD_SIZES
J2_ROT = 15                       # J2 actuator turned this far about +Y (clockwise looking along +Y): connector box 15 deg off the arm axis toward the front (user 2026-11-07)
ACTUATORS = [("FHA-8C", V(-math.cos(math.radians(25)), math.sin(math.radians(25)), 0)),   # J1: connector into the rear connector box (turned 25 deg so its corner stays inside the r42 column)
             ("FHA-11C", V(math.sin(math.radians(J2_ROT)), 0, math.cos(math.radians(J2_ROT)))),       # J2 (turned J2_ROT about +Y): in the shoulder hub, connector up the arm into the hollow spine (was RSA-8A: ~2 N.m continuous, J2 needs ~4.4 at full stretch)
             ("RSA-8A", -X),       # J3: in the elbow drum, connector to the back
             ("RSA-8A", -Z),       # J4: in the elbow box, connector down toward J3 (up, it broke through the elbow lid; ~4 mm over the J3 actuator)
             ("RSA-8A", -X),       # J5: in the wrist drum, connector toward J4
             ("RSF-5B", Z)]        # J6: in the wrist block
ACT_SIZES = [8, 11, 8, 8, 8, 5]
ACT_CLEAR = 0.5                   # bore clearance round the actuators
ACT_PL = {}                       # joint -> actuator frame placement in its housing link (for the insertion slots)
ELBOW_LID_Z = J4Z + 21                  # J4 actuator drops in from above: elbow top is a bolt-on lid above this height
NOSE_T = 4                        # J6 actuator slides in from the flange end: front NOSE_T of the wrist block is a bolt-on nose ring
FLEX_W = 4.0                      # flex PCB strip width (cabling.py)
BOARD_RECESS = 4.0                # Meca500-style connector boards under the +Y covers (cabling.py): recess depth below the cover-pocket floor
BACK_BLEND = (0.1, 0.6)           # back edge of the upper arm between these t is one Hermite curve from the hub's line into the fork's (no waist between the J2 hub and the banana's back bulge; user 2026-10-25)
UA_BOARD_T = 0.74                 # upper-arm board centre along the arm (0 = J2, 1 = J3), just below the J3 coil as in the Meca500
J2_COIL_R = 31                    # J2 flex coil cavity inner radius (r 31..37): outside the FHA-11C's 59 mm output disc, inside the r44 hub
DOME_TOP = FW - 2 + FLEX_W + 1.6 + 1.5   # +Y forearm cover is raised to here: the J5 flex coil sits under it (1.5 mm cover wall)
CABLES = True                     # internal flex-PCB bus (CAN + 24 V), clock-spring coil at each joint (see cabling.py)
TRI_POCKETS = True                # Meca500-style triangular rib pockets on the hidden faces (under covers, forearm end)
TRI_CELL, TRI_RIB, TRI_R = 24, 3, 2      # triangle side, rib width, pocket corner radius
HOLLOW_UPPER = True               # lighten the upper arm (see hollow_upper_arm)
UA_WALL, UA_WEB = 3.0, 3          # spine tube wall (3.5 -> 3.0: FEA peak 11 MPa, 2026-10-25); fork pocket inner web
UA_RING_WALL = 6.8                # solid metal kept round the J2 drive and J3 output/support seat (rings grow with the drive size)
UA_CORE_T1 = 0.4                  # spine core runs from J2 to here (0..1 along the arm); its round end stays sealed below the crotch floor (0.8 broke through it)
# Split upper arm: two mirror halves joined on the y = 0 plane, so each half-core is an open
# pocket milled from the joint face (a one-piece sealed core can't be machined).
SPLIT_UPPER = True
_UA_SPLIT_DRUM0 = [(-36, -11), (36, -11), (-11, -36), (11, -36), (-36, 11), (36, 11)]   # M3 x 30 in the hub, beside the flats of the square FHA-11C bore (x, z in Link2); none on top (connector box)
_c2, _s2 = math.cos(math.radians(J2_ROT)), math.sin(math.radians(J2_ROT))
UA_SPLIT_DRUM = [(x * _c2 + z * _s2, -x * _s2 + z * _c2) for x, z in _UA_SPLIT_DRUM0]   # turned with the J2 actuator
UA_SPLIT_COVER = [(16, 69), (28, 65)]  # 2026-11-13: (0, 67) was above the CROTCH_HUB_ARC floor (hole through the -Y plate into the crotch, never reached R); new sites have >= 4.2 mm metal round them through both halves. (12, 76) dropped 2026-11-13: its hole sat on the crotch floor (open half-hole, screw exposed).  (-20, 64), (-11, 64) dropped 2026-11-12: behind the PLATE_STRAP_CUT edge (no plate left under their heads). Older: (-24, 64) -> (-20, 64): clear of the PLATE_BACK_LINE edge; (2, 64) -> (0, 67): 8 mm off the PLATE_END_FLOW edge so its counterbore stays inside the cover pocket (2026-11-12)  # M3 x 45, heads under the -Y cover (lands kept in the rib pockets); moved up from z 55 (2026-10-25: with the forks starting at ARM_T0 0.6 the outer two floated off the spine); (12, 76): the one solid spot under the J3 -150 crotch floor
UA_BULKHEAD_Z = (61, 69)                                           # bulkhead across the core (z range) carrying the mid-length bolts; splits the core into two pockets (was 51..59)
UA_DOWELS = [(0, -37), (29, 92)]   # (-34, 66) -> (29, 92) 2026-11-12: it sat in the crotch void (27 mm from metal) since the 11-07 CROTCH_HUB_ARC trim, exposed by PLATE_BACK_LINE; (29, 92) = solid spine front corner. Old note:                                 # 3 mm dowels locating the halves; (-34, 66) on the bulkhead's back end (was (14, 82): left in the thin wall under the crotch once DRUM_R went to 34, stuck out; 2026-10-25)
UA_CORNER_R, UA_FLOOR_R = 6, 2    # spine core long-edge radii (12 mm cutter, L/D ~6); fork pocket floor radius
LIMITS = [(-175, 175), (-145, 105), (-153, 82), (-170, 170), (-132, 132), (-3600, 3600)]   # J2 -145..105 (was -115, user 2026-11-14, unchecked); J3 -153..82 (was ..80, user 2026-11-14, unchecked; was ..75, user 2026-11-13; was -150..74, user 2026-10-25); J5 +-132 (was 130, user 2026-11-07)
# Support bearings (FEA report recommendations, 2026-10-25): take bending moment off the actuators' own bearings
J1_SUPPORT = True                 # thin-section 4-point-contact ring (Kaydon KA025XP0 class: 63.5 bore x 69.85 OD x 6.35) on a base spigot round the J1 output disc, outer race in the turret
J1_BRG = (31.75, 34.925, 6.35)    # bore radius, outside radius, width
J1_SPIGOT_RI = 25.5               # spigot bore: 0.5 clear of the FHA-8C output disc envelope (r 25)
# Parallel joints J2/J3/J5: a second crossed-roller bearing on the side opposite the actuator's
# output flange, so both cheeks / plates / walls carry the joint (IKO CRBT super-slim, 5 wide)
J2_SUPPORT = True                 # CRBT205A (20 x 31 x 5) in the -Y cheek, on a stub from the upper arm's -Y hub face (was a 61804)
J2_BRG = (10, 15.5, 5)            # bore radius, outside radius, width
J3_SUPPORT = True                 # CRBT205A in the upper arm's -Y plate, on a stub from a cap over the J3 actuator's tail (elbow -Y face)
J3_BRG = (10, 15.5, 5)
J5_SUPPORT = True                 # CRBT155A (15 x 26 x 5) in the forearm's +Y wall, on the J5 cap's hollow flex hub
J5_BRG = (7.5, 13, 5)
LIGHTEN = True                    # mass cuts: base column ring pockets (from below), wrist drum -Y face pockets, deeper forearm back-face pockets
FA_POCKET_X0 = -3                 # forearm back-face rib pockets run from SLOT_X down to here (was SLOT_X - 20 = 8)
KLIK_FONT = "/usr/share/fonts/opentype/cantarell/Cantarell-ExtraBold.otf"   # heavy, like the photo lettering
OK_LOGO = True                    # Oxford Kinematics "OK" linkage mark (+ OXFORD KINEMATICS) replaces the KLiK lettering (user 2026-11-12; oxford-kinematics-logo.svg, image_e0d1ebee.jpg)
OK_FONT = "/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyreadventor-regular.otf"   # geometric sans like the plate's lettering
MACHINED = (0.87, 0.89, 0.90)     # bare machined aluminium (the mark is cut through the anodising)
m.METALLIC.add(MACHINED)


def _trace_face(poly, f):
    """Part face from a traced polygon {'outer': [[x, y]...], 'holes': [...]} via f(x, y) -> Vector."""
    def wire(pts):
        vs = [f(*q) for q in pts]
        return Part.makePolygon(vs + [vs[0]])
    face = Part.Face(wire(poly["outer"]))
    for h in poly["holes"]:
        face = face.cut(Part.Face(wire(h)))
    return face


def ok_logo(width, depth, origin, up, normal, text=False, proud=0.08):
    """Oxford Kinematics mark, traced from the user's photo image_e0d1ebee.jpg (perspective
    removed; ok_logo_trace.json, pixels of the rectified plate), `width` across, centred on
    `origin`, tops toward `up`, standing `proud` above the face along `normal`. Returns
    (silver, dark, green): bare-aluminium islands (+ OXFORD KINEMATICS below, at the photo's
    spacing, if `text`), dark grooves + pivot bores, anodised-green pivot rings."""
    import json, os
    T = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ok_logo_trace.json")))
    x0, y0, w, h = T["mark_bbox"]
    cx, cy = x0 + w / 2.0, y0 + h / 2.0
    s = width / float(w)
    f = lambda x, y: V((x - cx) * s, -(y - cy) * s, 0)
    P = _trace_face(T["mark"][0], f)
    disc = lambda c, r: Part.Face(Part.Wire(Part.makeCircle(r * s, f(*c), Z)))
    ring = lambda c, r0, r1: disc(c, r1).cut(disc(c, r0))
    piv = T["piv"]                                   # TL, TR, MR, BL, BR (px)
    grooves = [ring(c, 69, 75) for c in piv]         # pivot discs' outline grooves
    (tl, tr, _, bl, br) = piv
    for (ya, yb), pa, pb in ((T["grooves"]["top"], tl, tr), (T["grooves"]["bottom"], bl, br)):   # bar / O seams
        grooves.append(Part.Face(Part.makePolygon([f(pa[0], ya), f(pb[0], ya), f(pb[0], yb), f(pa[0], yb), f(pa[0], ya)])))
    grooves += [_trace_face(p, f) for p in T.get("inset", [])]   # inset outline lines round the O, bars and link
    bores = [disc(c, 13) for c in piv]
    greens = [ring(c, 28, 37) for c in piv]
    dark = P.common(grooves[0].fuse(grooves[1:])).fuse(bores)
    green = greens[0].fuse(greens[1:])
    silver = P.cut(dark).cut(green)
    if text:
        for poly in T["text"]:
            silver = silver.fuse(_trace_face(poly, f))
    lift = lambda face, top: face.extrude(V(0, 0, depth)).translated(V(0, 0, top - depth))
    x_dir = up.cross(normal)
    pl = App.Placement(origin, App.Rotation(x_dir, normal.cross(x_dir), normal, "ZXY"))
    return (m.moved(lift(silver, proud), pl), m.moved(lift(dark, proud * 0.4), pl), m.moved(lift(green, proud * 0.6), pl))


def add_ok_logo(doc, container, name, parts_list):
    """Add the three layers of one or more ok_logo() results as name, name_dark, name_green."""
    for k, (suffix, col) in enumerate((("", MACHINED), ("_dark", m.DARK), ("_green", ARM_FINISH))):
        sh = parts_list[0][k]
        for p in parts_list[1:]:
            sh = sh.fuse(p[k])
        add_shape(doc, container, name + suffix, sh, col)


def klogo(*args, **kw):
    """logo() in the heavy KLiK lettering font (other labels keep the default)."""
    import os
    old = m.LOGO_FONT
    if os.path.exists(KLIK_FONT):
        m.LOGO_FONT = KLIK_FONT
    try:
        return logo(*args, **kw)
    finally:
        m.LOGO_FONT = old


PHOTO_POSE = (0, 6, 36, 0, 48, 0)       # measured off the side photo (forearm 27 deg down, flange down)


SCREW_SS = m.STEEL                # polished stainless screw heads (metallic sheen)


def cap_screw(p, n, d=3.0):
    """ISO 4762 socket head cap screw head (stainless), seated on the surface at p,
    head along n: head dia 1.83d, height d, hex socket 0.83d AF, 0.6d deep, top edge chamfered."""
    n = V(n).normalize()
    hd, hh, af, sd = 1.83 * d, d, 0.83 * d, 0.6 * d
    head = Part.makeCylinder(hd / 2, hh + 0.1, p - n * 0.1, n)
    try:
        top = [e for e in head.Edges if abs((e.BoundBox.Center - (p + n * hh)).dot(n)) < 1e-6]
        head = head.makeChamfer(0.1 * d, top)
    except Exception:
        pass
    u = n.cross(V(0, 0, 1)) if abs(n.z) < 0.9 else n.cross(V(1, 0, 0))
    u.normalize()
    w = n.cross(u)
    r = af / math.sqrt(3)                                # hexagon circumradius
    c = p + n * (hh - sd)
    hexw = Part.makePolygon([c + (u * math.cos(math.radians(60 * i)) + w * math.sin(math.radians(60 * i))) * r
                             for i in range(7)])
    return head.cut(Part.Face(hexw).extrude(n * (sd + 0.5)))


def cs_screw(p, n, d=3.0):
    """ISO 10642 countersunk socket screw head, flush at p (surface), sunk along -n:
    90 deg head dk 2.24d deep k 0.62d, hex socket 0.67d AF, 0.37d deep."""
    n = V(n).normalize()
    dk, k, af, sd = 2.24 * d, 0.62 * d, 0.67 * d, 0.37 * d
    head = Part.makeCone(d / 2, dk / 2, k, p - n * k, n)
    u = n.cross(V(0, 0, 1)) if abs(n.z) < 0.9 else n.cross(V(1, 0, 0))
    u.normalize()
    w = n.cross(u)
    r = af / math.sqrt(3)
    c = p - n * sd
    hexw = Part.makePolygon([c + (u * math.cos(math.radians(60 * i)) + w * math.sin(math.radians(60 * i))) * r
                             for i in range(7)])
    return head.cut(Part.Face(hexw).extrude(n * (sd + 0.5)))


def countersinks(pts, n, t, d=3.0):
    """Cutter for countersunk screws at pts (surface, head along n) through a t thick
    part: 90 deg countersink (0.1 over the head) plus the 1.13d clearance hole."""
    n = V(n).normalize()
    dk, k = 2.24 * d + 0.2, 0.62 * d + 0.1
    sol = []
    for p in pts:
        cs = Part.makeCone(d / 2, dk / 2, k, p - n * k, n).fuse(Part.makeCylinder(dk / 2, 0.5, p, n))
        sol.append(cs.fuse(Part.makeCylinder(0.565 * d, t + 1, p - n * (t + 0.5), n)))
    return Part.makeCompound(sol)


def cbores(pts, n, r=3.25, depth=3.3, hole_r=0.0, hole_len=0.0):
    """Counterbore cutter at surface points pts (head along n): r x depth (+1 mm above the
    surface), plus an optional clearance hole hole_r deep hole_len below the counterbore."""
    n = V(n).normalize()
    sol = []
    for p in pts:
        c = Part.makeCylinder(r, depth + 1, p - n * depth, n)
        if hole_r:
            c = c.fuse(Part.makeCylinder(hole_r, hole_len + 0.1, p - n * (depth + hole_len), n))
        sol.append(c)
    return Part.makeCompound(sol)


def cs_screws(doc, link, name, pts, normal):
    """Stainless M3 countersunk screws, flush at pts."""
    add_shape(doc, link, name, Part.makeCompound([cs_screw(p, normal) for p in pts]), SCREW_SS)


def screws(doc, link, name, pts, normal, d=3.0):
    """Stainless socket cap screws (M3 default) at pts (on the surface), heads along normal."""
    sol = [cap_screw(p, normal, d) for p in pts]
    add_shape(doc, link, name, Part.makeCompound(sol), SCREW_SS)


def make_sheet(doc):
    sheet = doc.addObject("Spreadsheet::Sheet", "Joints")
    for c, h in zip("ABCDEF", ("Joint", "Angle (deg)", "Min", "Max", "HD size", "Ratio")):
        sheet.set(c + "1", h)
    for i, (lo, hi) in enumerate(LIMITS, 1):
        r = str(i + 1)
        size = ACT_SIZES[i - 1] if USE_ACTUATORS else HD_SIZES[i - 1]
        for c, v in zip("ABCDEF", ("J%d" % i, "0", lo, hi, size, m.RATIO)):
            sheet.set(c + r, str(v))
        sheet.setAlias("B" + r, "q%d" % i)


def arm_centre(t):
    """Upper-arm centre line at t (0 = J2, 1 = J3): point, unit normal toward the
    front (+X, the concave side) and half depth."""
    s = min(max(t / HUB_T, 0.0), 1.0)
    hub = (J2_HUB_R - ARM_H0) * (1 - s) ** 2 * (1 + 2 * s)   # zero slope at both ends: G1 into the round hub cap and into the plain band
    h = ARM_H0 + (PLATE_R - ARM_H0) * t * t + hub
    if abs(ARM_SAG) < 1e-6:                              # straight arm: J2 -> J3 along +Z
        return V(0, 0, t * L2), V(1, 0, 0), h
    R = (L2 ** 2 / 4 + ARM_SAG ** 2) / (2 * ARM_SAG)
    cx, cz = R - ARM_SAG, L2 / 2                         # centre of curvature, in front of the arm
    a0, a1 = math.atan2(-cz, -cx), math.atan2(L2 - cz, -cx)
    if a0 < 0 < a1:                                      # go the short way, round the back (a = pi)
        a0 += 2 * math.pi
    a = a0 + t * (a1 - a0)
    p = V(cx + R * math.cos(a), 0, cz + R * math.sin(a))
    n = V(-math.cos(a), 0, -math.sin(a))
    s = min(max(t / HUB_T, 0.0), 1.0)
    hub = (J2_HUB_R - ARM_H0) * (1 - s) ** 2 * (1 + 2 * s)   # zero slope at both ends: G1 into the round hub cap and into the plain band
    return p, n, ARM_H0 + (PLATE_R - ARM_H0) * t * t + hub


def _back_edge_raw(t, inset):
    c, nn, h = arm_centre(t)
    return c - nn * (h - inset)


def _back_edge(t, inset):
    """Back (convex) edge of the arm band at t. Between BACK_BLEND[0] and [1] it is a cubic
    Hermite from the hub-side edge to the fork-side edge, matching both points and
    tangents, so the line flows straight from the forks into the J2 hub without the
    waist the hub swell and the banana bulge leave between them."""
    a, b = BACK_BLEND
    if not a < t < b:
        return _back_edge_raw(t, inset)
    e = 1e-4
    P1, P2 = _back_edge_raw(a, inset), _back_edge_raw(b, inset)
    D1 = (_back_edge_raw(a + e, inset) - _back_edge_raw(a - e, inset)) * (0.5 / e) * (b - a)
    D2 = (_back_edge_raw(b + e, inset) - _back_edge_raw(b - e, inset)) * (0.5 / e) * (b - a)
    s = (t - a) / (b - a)
    h00, h10, h01, h11 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s, -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
    return P1 * h00 + D1 * h10 + P2 * h01 + D2 * h11


def front_line(inset):
    """Straight front edge of the upper arm: the line tangent (front side) to the J2 hub
    circle and the J3 end circle, both shrunk by `inset`. Returns its tangent points on
    the J2 and J3 circles and its unit outward normal."""
    (c0, _, h0), (c1, _, h1) = arm_centre(0), arm_centre(1)
    r0, r1 = h0 - inset, h1 - inset
    d = c1 - c0
    L = d.Length
    u = d * (1 / L)
    w = V(u.z, 0, -u.x)                                  # perpendicular to the J2-J3 chord
    if w.x < 0:
        w = w * -1                                       # toward the front (+X)
    k = (r0 - r1) / L
    m = u * k + w * math.sqrt(1 - k * k)
    return c0 + m * r0, c1 + m * r1, m


def arm_profile(t0, t1, inset, n=24):
    """Face in the XZ plane: the banana band along the arm centre line from t0
    to t1, half depth minus `inset`, with round ends."""
    ts = [t0 + (t1 - t0) * i / n for i in range(n + 1)]
    pts = [arm_centre(t) for t in ts]
    front = [c + nn * (h - inset) for c, nn, h in pts]
    back = [_back_edge(t, inset) for t in ts]

    def spline(q):
        b = Part.BSplineCurve()
        b.interpolate(q)
        return b.toShape()

    def cap(c, nn, h, out, pf, pb):
        # round end from the front end point pf to the back end point pb (these are the
        # actual edge ends: inside BACK_BLEND the back point is off the plain offset)
        tg = V(nn.z, 0, -nn.x) * out                     # along the arm, outward at this end
        r = h - inset
        return Part.Arc(pf, c + tg * r, pb).toShape()

    (c0, n0, h0), (c1, n1, h1) = pts[0], pts[-1]
    # tangent along +t is n rotated: check its sign against the chord
    sgn = 1 if (V(n1.z, 0, -n1.x)).dot(c1 - c0) > 0 else -1
    if ARM_FRONT_LINE and t0 == 0 and t1 == 1:
        A, B, _ = front_line(inset)
        edges = [Part.LineSegment(A, B).toShape(), cap(c1, n1, h1, sgn, B, back[-1]), spline(back[::-1]),
                 cap(c0, n0, h0, -sgn, A, back[0])]
        return Part.Face(Part.Wire(Part.__sortEdges__(edges)))
    edges = [spline(front), cap(c1, n1, h1, sgn, front[-1], back[-1]), spline(back[::-1]),
             cap(c0, n0, h0, -sgn, front[0], back[0])]
    return Part.Face(Part.Wire(Part.__sortEdges__(edges)))


def _fillet_y_planes(solid, r, ys):
    """Round the edges lying in the planes y = ys (a tool solid's edges there become
    concave internal radii once it is cut). Returns the solid unchanged if OCC fails."""
    try:
        f = solid.makeFillet(r, [e for e in solid.Edges if any(on_plane(e, y=y) for y in ys)])
        return f if f.isValid() else solid
    except Exception:
        return solid


def _radial_extent(shape, centre):
    """Largest distance of `shape` from the Y axis through `centre` (XZ plane)."""
    pts = [p for e in shape.Edges for p in e.discretize(24)]
    return max(math.hypot(p.x - centre.x, p.z - centre.z) for p in pts)


def add_actuator(doc, j, kind, housing, output, gap, out_offset, toward, conn):
    """Harmonic Drive actuator with integrated servo drive (CANopen) at joint j.
    Output face on the running gap at `gap` (housing frame), body back into the
    housing along -toward, connector box pointing along `conn`. The stator goes in
    the housing link, the output flange (rotor) in the output link. Returns the
    clearance envelope (+ACT_CLEAR) in the housing and output frames, for boring."""
    import actuator_fit as af
    zt = V(toward).normalize()
    xt = V(conn) - zt * V(conn).dot(zt)
    xt.normalize()
    rot = App.Rotation(xt, zt.cross(xt), zt, "ZXY")
    pl_h = App.Placement(gap, rot)
    pl_o = App.Placement(gap + out_offset, rot)
    stator, rotor = af.parts(kind)
    act = doc.addObject("App::Part", "J%d_act" % j)
    housing.addObject(act)
    act.Placement = pl_h
    ACT_PL[j] = pl_h
    act.Label = "J%d %s (integrated drive)" % (j, kind)
    add_shape(doc, act, "J%d_actuator" % j, stator, m.MOTOR)
    add_shape(doc, output, "J%d_output" % j, m.moved(rotor, pl_o), m.STEEL)
    env = af.envelope(kind, ACT_CLEAR)
    return m.moved(env, pl_h), m.moved(env, pl_o)


def disc(c, r, n):
    """Planar disc (face) of radius r about c, normal n."""
    return Part.Face(Part.Wire(Part.makeCircle(r, c, n)))


def tri_pockets(region, n, depth, keep=(), u=Z, cell=None, rib=None, r=None, margin=None, min_in=2.5):
    """Meca500-style lightening: a triangular rib grid (isogrid) of pockets on a hidden
    planar face. `region` is the face (in the pocket opening plane) the pockets may use,
    `n` the outward normal there (pockets go `depth` along -n), `keep` faces in the same
    plane that stay solid (bosses, screw lands), `u` the grid direction. Ribs are `rib`
    wide, pocket corners rounded `r` (cutter radius); a `margin` wall is kept round the
    region edge. Returns a compound of pocket solids to cut (None if nothing fits)."""
    cell, rib, r = cell or TRI_CELL, rib or TRI_RIB, r or TRI_R
    margin = rib if margin is None else margin
    n = V(n).normalize()
    u = V(u) - n * V(u).dot(n)
    u.normalize()
    v = n.cross(u)
    inner = region.makeOffset2D(-margin) if margin > 0 else region
    for k in keep:
        inner = inner.cut(k)
    if inner.Area < 1:
        return None
    c0, p0 = inner.BoundBox.Center, inner.Vertexes[0].Point
    o = c0 - n * (c0 - p0).dot(n)                        # grid origin in the face plane
    pts = [p for e in inner.Edges for p in e.discretize(16)]
    us = [(p - o).dot(u) for p in pts]
    vs = [(p - o).dot(v) for p in pts]
    s, h = cell, cell * math.sqrt(3) / 2
    k = (s - math.sqrt(3) * rib) / s                     # shrink about the centroid: rib/2 off each side
    P = lambda a, b: o + u * a + v * b

    def rounded(tri):
        """Triangle (2D corner list) shrunk by rib/2 with corners rounded r."""
        c = [sum(t[0] for t in tri) / 3, sum(t[1] for t in tri) / 3]
        q = [(c[0] + (a - c[0]) * k, c[1] + (b - c[1]) * k) for a, b in tri]
        f = Part.Face(Part.makePolygon([P(*q[0]), P(*q[1]), P(*q[2]), P(*q[0])]))
        try:
            g = f.makeOffset2D(-r).makeOffset2D(r)
            return g.Faces[0] if g.Faces else f
        except Exception:
            return f

    cut = []
    j = 0
    vb = min(vs) - h
    while vb < max(vs) + h:
        a = min(us) - 2 * s + (j % 2) * s / 2
        while a < max(us) + s:
            for tri in (((a, vb), (a + s, vb), (a + s / 2, vb + h)),
                        ((a + s / 2, vb + h), (a + 3 * s / 2, vb + h), (a + s, vb))):
                t = rounded(tri)
                tb = t.BoundBox
                tb.enlarge(0.5)                            # flat boxes: zero thickness, so pad before the overlap test
                pieces = []
                for F in inner.Faces:                      # clip face by face
                    if tb.intersect(F.BoundBox):
                        pieces += t.common(F).Faces
                for f in pieces:
                    if f.Area < 4 * r * r or 2 * f.Area / f.Length < min_in:   # too small/thin to machine
                        continue
                    try:
                        g = f.makeOffset2D(-r).makeOffset2D(r)            # round corners left by the clip
                        f = g.Faces[0] if g.Faces and g.Faces[0].Area > 0.5 * f.Area else f
                    except Exception:
                        pass
                    cut.append(f.extrude(n * -depth))
            a += s
        vb += h
        j += 1
    return Part.makeCompound(cut) if cut else None


LOG = []                          # build messages (the MCP bridge doesn't show prints from module code)


def log(msg):
    LOG.append(msg)
    print(msg)


def refine(shape):
    """removeSplitter, kept only if the result is valid and the volume unchanged
    (on some cored shapes it returns garbage)."""
    try:
        r = shape.removeSplitter()
        if r.isValid() and len(r.Solids) == len(shape.Solids) and abs(r.Volume - shape.Volume) < 0.01 * shape.Volume:
            return r
    except Exception:
        pass
    return shape


def checked_cut(shape, tool, name=""):
    """shape.cut(tool), verified: the cut must stay valid and actually remove (most of)
    what the tool overlaps; OCC sometimes returns the shape unchanged without an error.
    Retried with fuzzy booleans; logged either way."""
    want = shape.common(tool).Volume
    if want < 0.01:
        log("checked_cut %s: no overlap (shape %.0f, tool %.0f mm3)" % (name, shape.Volume, tool.Volume))
        return shape
    for fuzz in (None, 0.001, 0.01, 0.05):
        try:
            r = shape.cut(tool) if fuzz is None else shape.cut([tool], fuzz)
        except Exception:
            continue
        got = shape.Volume - r.Volume
        if r.isValid() and len(r.Solids) == len(shape.Solids) and got > 0.9 * want:
            log("checked_cut %s: removed %.0f of %.0f overlap%s" % (name, got, want, "" if fuzz is None else " (fuzz %s)" % fuzz))
            return r
    log("checked_cut %s: FAILED (overlap %.0f mm3 left in)" % (name, want))
    return shape


def crotch_front_chamfer(upper, spot, c):
    """Chamfer the front crotch edge (along Y at `spot` = (x, z), Link2 frame) with legs `c`:
    a triangular prism through the corner and the points c mm along the arm's front face and
    along the crotch floor, cut across |y| <= PLATE_IN (makeFillet/makeChamfer fail on this
    edge). Stores the y = 0 triangle in CROTCH_CH_XZ for the hollow core."""
    global CROTCH_CH_XZ
    x, z = spot
    sec = upper.slice(Y, 0.0)
    cv = V(x, 0, z)
    es = [e for w in sec for e in w.Edges if min((v.Point - cv).Length for v in e.Vertexes) < 0.5]
    if len(es) != 2:
        log("crotch front chamfer: corner at (%.1f, %.1f) has %d edges, skipped" % (x, z, len(es)))
        return upper
    corner = min((v.Point for v in es[0].Vertexes), key=lambda p: (p - cv).Length)
    legs = []
    for e in es:
        pts = e.discretize(Distance=0.2)
        if (pts[0] - corner).Length > (pts[-1] - corner).Length:
            pts = pts[::-1]
        acc, q = 0.0, pts[-1]
        for a, b in zip(pts, pts[1:]):
            acc += (b - a).Length
            if acc >= c:
                q = b
                break
        legs.append(q)
    m = (legs[0] + legs[1]) * 0.5
    n = corner - m
    n.normalize()                                          # outward, from the chamfer face to the corner
    u = legs[1] - legs[0]
    u.normalize()
    A, B = legs[0] - u * 1.0, legs[1] + u * 1.0            # chamfer line run 1 mm past both legs: the crotch floor
    tri = Part.Face(Part.makePolygon([A, B, B + n * 15, A + n * 15, A]))   # curves near its leg and left a sliver
    tool = tri.extrude(V(0, 2 * PLATE_IN, 0)).translated(V(0, -PLATE_IN, 0))
    res = checked_cut(upper, tool, "front crotch chamfer %g" % c)
    if res is not upper and res.isValid():
        CROTCH_CH_XZ = Part.Face(Part.makePolygon([legs[0], corner, legs[1], legs[0]]))
        log("crotch front chamfer %g at (%.1f, %.1f): legs to (%.1f, %.1f) / (%.1f, %.1f)" % (c, corner.x, corner.z, legs[0].x, legs[0].z, legs[1].x, legs[1].z))
    return res


def round_corner_2d(shape, cx, cz, r, half_y, win=14):
    """Round a convex edge running along Y at (cx, cz) when makeFillet refuses it: take
    the y = 0 outline near the corner, round its convex corners by an opening (offset in
    by r, back out by r, arc joins), and cut the sliver that removes, within win of the
    corner, straight through |y| < half_y. Returns None if it fails."""
    try:
        sec = shape.slice(Y, 0.0)
        outer = max(sec, key=lambda w: w.BoundBox.DiagonalLength)
        F = Part.Face(outer)
        sq = lambda h: Part.Face(Part.makePolygon([V(cx - h, 0, cz - h), V(cx + h, 0, cz - h), V(cx + h, 0, cz + h),
                                                     V(cx - h, 0, cz + h), V(cx - h, 0, cz - h)]))
        Fc = F.common(sq(3 * win))
        Fc = max(Fc.Faces, key=lambda f: f.Area) if len(Fc.Faces) > 1 else Fc.Faces[0]
        op = Fc.makeOffset2D(-r, join=0).makeOffset2D(r, join=0)
        sl = Fc.cut(op).common(sq(win))
        if not sl.Faces or sl.Area < 0.01:
            return None
        tool = Part.makeCompound([f.extrude(V(0, 2 * half_y, 0)) for f in sl.Faces]).translated(V(0, -half_y, 0))
        r_ = shape.cut(tool)
        if r_.isValid() and len(r_.Solids) == len(shape.Solids) and 0 < shape.Volume - r_.Volume < 0.01 * shape.Volume:
            return r_
    except Exception as e:
        log("round_corner_2d failed: %s" % e)
    return None


def cut_pockets(shape, tp, name="", pre_refine=True):
    """Cut the pocket solids one at a time (a single cut with the whole compound
    silently skips some of them); a pocket whose cut fails, breaks the solid or removes
    nothing is retried with a fuzzy boolean, then left out."""
    if tp is None:
        return shape

    def run(shape):
        skipped = []
        for i, p in enumerate(tp.Solids):
            want = shape.common(p).Volume
            if want < 1:
                continue
            ok = False
            for fuzz in (None, 0.001, 0.01, 0.05):
                try:
                    r = shape.cut(p) if fuzz is None else shape.cut([p], fuzz)
                except Exception as e:
                    continue
                if len(r.Solids) > len(shape.Solids):          # drop slivers the cut chipped off
                    big = [q for q in r.Solids if q.Volume > 1]
                    if len(big) == len(shape.Solids):
                        r = big[0] if len(big) == 1 else Part.makeCompound(big)
                if r.isValid() and len(r.Solids) == len(shape.Solids) and shape.Volume - r.Volume > 0.7 * want:
                    shape, ok = r, True
                    break
            if not ok:
                skipped.append((i, round(want)))
        return shape, skipped

    res, skipped = run(refine(shape) if pre_refine else shape)
    if skipped:
        # removeSplitter can leave a valid-looking solid that later booleans choke on
        # (the upper arm after its first side of pockets, 2026-10-25): retry unrefined
        res2, sk2 = run(shape)
        if len(sk2) < len(skipped):
            log("cut_pockets %s: unrefined retry, skipped %d -> %d" % (name, len(skipped), len(sk2)))
            res, skipped = res2, sk2
    log("cut_pockets %s: %d pockets, skipped %s" % (name, len(tp.Solids), skipped))
    return res


def hollow_upper_arm(upper, e2h, e3o):
    """Lightening for a Meca500-class (size-8) J2, laid out for micron-level stability:
    - spine: a closed-section tube (UA_WALL walls), cored from the crotch end, so the
      J2->J3 load path stays a torsion box rather than an open channel;
    - forks: pockets from the outer faces under the covers, UA_WEB inner web; the
      covers sit on a ledge and stay non-structural (no bolted/bonded joint in the
      load path to micro-slip or creep);
    - solid rings (drive envelope + UA_RING_WALL) keep both bearing seats in thick metal, so
      releasing pocket stresses can't distort the bores (finish-bore them last);
    - left/right identical and centred, so a uniform temperature change only grows the
      arm in its own plane; uniform walls give one thermal time constant.
    """
    if USE_ACTUATORS:
        import actuator_fit as af
        r2 = _radial_extent(e2h, V(0, 0, 0)) + UA_WALL                       # full envelope incl. the connector box (corner radius alone let the box break into the core)
        r3 = af.corner_radius(ACTUATORS[2][0]) + ACT_CLEAR + UA_RING_WALL
    else:
        r2 = _radial_extent(e2h, V(0, 0, 0)) + UA_RING_WALL
        r3 = max(_radial_extent(e3o, V(0, 0, L2)), 18.2) + UA_RING_WALL       # 18.2: J3 support bearing seat
    c2 = Part.Face(Part.Wire(Part.makeCircle(r2, V(0, 0, 0), Y)))
    c3 = Part.Face(Part.Wire(Part.makeCircle(r3, V(0, 0, L2), Y)))
    # spine tube: profile inset by the wall, minus the J2 ring, open past the crotch floor
    hy = SPINE_Y - UA_WALL
    core = arm_profile(0, UA_CORE_T1, UA_WALL).cut(c2)
    if CROTCH_XZ is not None:                           # stay UA_WALL below the crotch pocket (it deepens with the J3 limit)
        core = core.cut(CROTCH_XZ.makeOffset2D(UA_WALL, join=0))
        if CROTCH_CH_XZ is not None:
            core = core.cut(CROTCH_CH_XZ.makeOffset2D(UA_WALL, join=0))
        core = max(core.Faces, key=lambda f: f.Area) if len(core.Faces) > 1 else core.Faces[0]
    core = core.extrude(V(0, 2 * hy, 0)).translated(V(0, -hy, 0))
    core = _fillet_y_planes(core, UA_CORNER_R, (hy, -hy))
    tools = [core]
    if SPLIT_UPPER:
        # bulkhead across the core for the mid-length bolts (also a torsion diaphragm):
        # the core becomes two separate pockets, each cut on its own
        z0, z1 = UA_BULKHEAD_Z
        tools = [t for t in core.cut(box(200, 2 * hy + 4, z1 - z0, V(-100, -hy - 2, z0))).Solids if t.Volume > 1000]   # (a sliver under the bulkhead won't cut cleanly)
    for i, tool in enumerate(tools):
        for fuzz in (None, 0.001, 0.01):
            r = upper.cut(tool) if fuzz is None else upper.cut([tool], fuzz)
            if r.isValid() and len(r.Solids) == 1 and upper.Volume - r.Volume > 0.99 * tool.Volume:
                log("upper arm core %d/%d: fuzz %s" % (i + 1, len(tools), fuzz))
                upper = r
                break
            log("upper arm core %d/%d: fuzz %s rejected: valid %s, solids %d, removed %.0f of %.0f" % (
                i + 1, len(tools), fuzz, r.isValid(), len(r.Solids), upper.Volume - r.Volume, tool.Volume))
        else:
            log("upper arm core %d/%d: CUT FAILED" % (i + 1, len(tools)))
    # fork pockets: 3 mm ledge inside the cover pocket (cover outline at inset 3.5)
    if not UA_FORK_POCKETS:
        return upper
    floor = PLATE_IN + UA_WEB
    top = PLATE_OUT - 2.5                                       # cover pocket floor
    if TRI_POCKETS:
        # Meca500 style: triangular rib pockets under the covers (ribs tie the fork's
        # outer skin to the inner web; screw lands and the J3 ring stay solid)
        for sy in (1, -1):
            yp = sy * top
            region = arm_profile(ARM_T0, 1, 3.5 + 3).translated(V(0, yp, 0))
            keep = [disc(V(0, yp, L2), (28 if CABLES else 21) if USE_ACTUATORS else r3, Y)]   # J3: RSA-8A output flange land (+ cable loop cavity)
            if CABLES:
                cb_, _, _ = arm_centre(UA_BOARD_T)
                keep.append(disc(V(cb_.x, yp, cb_.z), 15, Y))                   # solid land for the connector-board recess
            for t in (ARM_T0, 1.0):
                c, nn, h = arm_centre(t)
                rr = h - 3.5 - 5
                keep += [disc(V(c.x + sgn * nn.x * rr, yp, c.z + sgn * nn.z * rr), 5, Y) for sgn in (1, -1)]
            if SPLIT_UPPER:
                keep += [disc(V(x, yp, z), 5, Y) for x, z in UA_SPLIT_COVER]   # split-bolt head lands (both sides: halves stay mirror images)
            upper = cut_pockets(upper, tri_pockets(region, sy * Y, top - floor, keep, u=Z, margin=0), "upper arm", pre_refine=False)   # refine here broke later cuts (FHA-11C hub, 2026-10-25)
        return upper
    fp = arm_profile(ARM_T0, 1, 3.5 + 3).cut(c3).extrude(V(0, top - floor + 0.01, 0))
    fp = _fillet_y_planes(fp.translated(V(0, floor, 0)), UA_FLOOR_R, (floor,))
    upper = upper.cut(fp).cut(fp.mirror(V(0, 0, 0), Y))
    return upper


def rrect_yz(x, hy, hz, r):
    """Closed rounded-rectangle wire in the plane at `x` (y +-hy, z +-hz, corner radius r)."""
    k = math.sqrt(0.5)
    edges, pts = [], []
    for sy, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1)):   # corners anticlockwise from +y+z
        cy, cz = sy * (hy - r), sz * (hz - r)
        pts.append((V(x, cy + sy * r, cz) if sy * sz > 0 else V(x, cy, cz + sz * r),
                    V(x, cy + sy * r * k, cz + sz * r * k),
                    V(x, cy, cz + sz * r) if sy * sz > 0 else V(x, cy + sy * r, cz)))
    for i, (a0, am, a1) in enumerate(pts):
        edges.append(Part.Arc(a0, am, a1).toShape())
        edges.append(Part.LineSegment(a1, pts[(i + 1) % 4][0]).toShape())
    return Part.Wire(edges)


def bevel(shape, c, edges=None):
    """Chamfer `edges` (default all) at c, backing off if OCC refuses (chamfer twin of soft())."""
    edges = shape.Edges if edges is None else edges
    for cc in (c, 0.6 * c, 0.35 * c):
        try:
            out = shape.makeChamfer(cc, edges)
            if out.isValid():
                if cc != c:
                    log("bevel %g failed, used %g" % (c, cc))
                return out
        except Exception:
            pass
    log("bevel %g failed, left sharp" % c)
    return shape


def crect_yz(x, hy, hz, c):
    """Closed chamfered-rectangle wire in the plane at `x` (y +-hy, z +-hz, 45 deg corner legs c)."""
    p = [V(x, hy, hz - c), V(x, hy - c, hz), V(x, -hy + c, hz), V(x, -hy, hz - c),
         V(x, -hy, -hz + c), V(x, -hy + c, -hz), V(x, hy - c, -hz), V(x, hy, -hz + c)]
    return Part.makePolygon(p + [p[0]])


def rounded_square(half, r, z0, h):
    b = box(2 * half, 2 * half, h, V(-half, -half, z0))
    return soft(b, r, [e for e in b.Edges if is_along(e, Z)])


SMOOTH_HELPER = r"""
import json, sys
from shapely.geometry import Polygon
d = json.load(sys.stdin)
p = Polygon(d["pts"]).buffer(d["r"], join_style=1).buffer(-d["r"], join_style=1)   # closing: fill notches
p = p.buffer(d["grow"], join_style=1).buffer(-d["ro"], join_style=1).buffer(d["ro"], join_style=1)   # opening: round sharp outer corners (<= ~1 mm of the clearance)
if p.geom_type != "Polygon":
    p = max(p.geoms, key=lambda g: g.area)
ring = p.exterior
n = max(int(ring.length / d["ds"]), 12)
json.dump([list(ring.interpolate(i * ring.length / n).coords[0]) for i in range(n)], sys.stdout)
"""


CROTCH_XZ = None
CROTCH_CH_XZ = None     # y = 0 face of the front crotch chamfer cut (hollow_upper_arm keeps the core a wall off it)
PLATE_XZ = None      # fork plate side outline (Link2 XZ, y = 0), set by build() after the side trims


def plate_end_flow(upper, F, d):
    """Fork plates' lower edge carried on from the spine's hub-arc blend: the straight line
    through F (where the blend meets the crotch floor) along its direction d continues as
    the plates' lower edge, rounded PLATE_FLOW_RF into the straight front edge and
    PLATE_FLOW_RK into the plates' back end circle. Cuts |y| >= SPINE_Y only, then
    re-takes PLATE_XZ so the covers follow (user 2026-11-12)."""
    global PLATE_XZ
    c0, _, h0 = arm_centre(ARM_T0)
    nU = V(-d.z, 0, d.x)                                     # normal toward the plate (up/back)
    if nU.z < 0:
        nU = nU * -1
    A, B, m = front_line(0)
    uf = B - A
    uf.normalize()

    def meet(p1, d1, p2, d2):                                # intersection of two lines in XZ
        den = d1.x * d2.z - d1.z * d2.x
        s = ((p2.x - p1.x) * d2.z - (p2.z - p1.z) * d2.x) / den
        return p1 + d1 * s
    rf, rk = PLATE_FLOW_RF, PLATE_FLOW_RK
    C = meet(F + nU * rf, d, A - m * rf, uf)                 # front corner round
    T1, T2 = C - nU * rf, C + m * rf
    # back kink: circle r rk tangent to the flow line (plate side) and inside the end circle
    best = None
    P0 = F + nU * rk - c0
    a2, b2, cc = 1.0, 2 * P0.dot(d), P0.dot(P0) - (h0 - rk) ** 2
    disc = b2 * b2 - 4 * a2 * cc
    for s in ((-b2 + math.sqrt(disc)) / 2, (-b2 - math.sqrt(disc)) / 2) if disc >= 0 else ():
        K = F + nU * rk + d * s
        if s > -1e-6 and (best is None or s < best[0]):
            best = (s, K)
    if best is None:
        log("plate end flow: no back-kink round, cut skipped")
        return upper
    K = best[1]
    Tl = K - nU * rk
    rad = K - c0
    rad.normalize()
    Ta = c0 + rad * h0
    mid = lambda c, p, q, r: c + ((p + q) * 0.5 - c).normalize() * r
    P1 = T2 + m * 40
    P4 = Ta + rad * 40
    edges = [Part.Arc(Ta, mid(K, Ta, Tl, rk), Tl).toShape(), Part.LineSegment(Tl, T1).toShape(),
             Part.Arc(T1, mid(C, T1, T2, rf), T2).toShape(), Part.LineSegment(T2, P1).toShape(),
             Part.LineSegment(P1, V(P1.x, 0, -200)).toShape(), Part.LineSegment(V(P1.x, 0, -200), V(P4.x, 0, -200)).toShape(),
             Part.LineSegment(V(P4.x, 0, -200), P4).toShape(), Part.LineSegment(P4, Ta).toShape()]
    face = Part.Face(Part.Wire(edges))
    t = PLATE_OUT - SPINE_Y + 8
    for sy in (1, -1):
        upper = checked_cut(upper, face.extrude(V(0, t, 0)).translated(V(0, SPINE_Y if sy > 0 else -SPINE_Y - t, 0)), "plate end flow %+d" % sy)
    log("plate end flow: line from F (%.1f, %.1f) at %.1f deg, front round r%s at (%.1f, %.1f), back round r%s at (%.1f, %.1f)"
        % (F.x, F.z, math.degrees(math.atan2(d.z, d.x)), rf, C.x, C.z, rk, K.x, K.z))
    ws = [w for w in upper.slice(Y, PLATE_OUT - 1.5) if w.isClosed()]
    if ws and COVER_FOLLOWS_PLATE:
        PLATE_XZ = max((Part.Face(w) for w in ws), key=lambda f: f.Area).translated(V(0, -(PLATE_OUT - 1.5), 0))
        log("upper arm plate outline (after flow cut): %.0f mm2, %d edges" % (PLATE_XZ.Area, len(PLATE_XZ.Edges)))
    return upper


def plate_back_line(upper):
    """Cut the fork plates (|y| >= SPINE_Y) behind the line parallel to the arm's straight
    front face and tangent to the J3 end circle (r PLATE_R) on the back, so the plates are
    parallel-sided; PLATE_BACK_R round into the lower end circle. Re-takes PLATE_XZ."""
    global PLATE_XZ
    A, B, m = front_line(0)
    uf = B - A
    uf.normalize()
    if uf.z < 0:
        uf = uf * -1                                         # up the arm
    J3c = arm_centre(1)[0]
    P = J3c - m * PLATE_R                                    # back tangent point on the J3 end
    c0, _, h0 = arm_centre(ARM_T0)
    rb = PLATE_BACK_R
    # round: centre rb in front of the line, inside the end circle (|K - c0| = h0 - rb)
    Q0 = P + m * rb - c0
    b2, cc = 2 * Q0.dot(uf), Q0.dot(Q0) - (h0 - rb) ** 2
    disc = b2 * b2 - 4 * cc
    if disc < 0:
        log("plate back line: end circle not cut by the line, nothing to do")
        return upper
    K = min((P + m * rb + uf * s for s in ((-b2 + math.sqrt(disc)) / 2, (-b2 - math.sqrt(disc)) / 2)), key=lambda k: k.z)
    Tb = K - m * rb
    rad = K - c0
    rad.normalize()
    Ta = c0 + rad * h0
    mid = ((Ta + Tb) * 0.5 - K)
    mid.normalize()
    U = P + uf * 200
    Pd = Ta + rad * 60
    edges = [Part.Arc(Ta, K + mid * rb, Tb).toShape(), Part.LineSegment(Tb, U).toShape(),
             Part.LineSegment(U, U - m * 150).toShape(), Part.LineSegment(U - m * 150, Pd - m * 150).toShape(),
             Part.LineSegment(Pd - m * 150, Pd).toShape(), Part.LineSegment(Pd, Ta).toShape()]
    face = Part.Face(Part.Wire(edges))
    t = PLATE_OUT - SPINE_Y + 8
    for sy in (1, -1):
        upper = checked_cut(upper, face.extrude(V(0, t, 0)).translated(V(0, SPINE_Y if sy > 0 else -SPINE_Y - t, 0)), "plate back line %+d" % sy)
    log("plate back line: tangent to the J3 end at (%.1f, %.1f), r%s round into the end circle at (%.1f, %.1f)" % (P.x, P.z, rb, K.x, K.z))
    ws = [w for w in upper.slice(Y, PLATE_OUT - 1.5) if w.isClosed()]
    if ws and COVER_FOLLOWS_PLATE:
        PLATE_XZ = max((Part.Face(w) for w in ws), key=lambda f: f.Area).translated(V(0, -(PLATE_OUT - 1.5), 0))
        log("upper arm plate outline (after back line): %.0f mm2, %d edges" % (PLATE_XZ.Area, len(PLATE_XZ.Edges)))
    return upper


def plate_strap_cut(upper, E):
    """Remove the fork plates (|y| >= SPINE_Y) behind the line through E parallel to the
    arm's straight front face, from E's level up, except the r PLATE_R disc round J3: the
    plates' back edge runs straight from E up into the elbow circle. Re-takes PLATE_XZ."""
    global PLATE_XZ
    A, B, m = front_line(0)
    uf = B - A
    uf.normalize()
    if uf.z < 0:
        uf = uf * -1
    E = V(E.x, 0, E.z)
    E = E - uf * 6                                  # start below the tangent point: else a 0.5 mm wedge of the hull edge survives behind it (stray face, user 2026-11-12)
    poly = Part.Face(Part.makePolygon([E, E + uf * 250, E + uf * 250 - m * 150, E - m * 150, E]))
    J3c = arm_centre(1)[0]
    face = poly.cut(Part.Face(Part.Wire(Part.makeCircle(PLATE_R, J3c, Y))))
    t = PLATE_OUT - SPINE_Y + 8
    for sy in (1, -1):
        upper = checked_cut(upper, face.extrude(V(0, t, 0)).translated(V(0, SPINE_Y if sy > 0 else -SPINE_Y - t, 0)), "plate strap cut %+d" % sy)
    log("plate strap cut: line from (%.1f, %.1f) along (%.4f, %.4f), r%s J3 disc kept" % (E.x, E.z, uf.x, uf.z, PLATE_R))
    ws = [w for w in upper.slice(Y, PLATE_OUT - 1.5) if w.isClosed()]
    if ws and COVER_FOLLOWS_PLATE:
        PLATE_XZ = max((Part.Face(w) for w in ws), key=lambda f: f.Area).translated(V(0, -(PLATE_OUT - 1.5), 0))
        log("upper arm plate outline (after strap cut): %.0f mm2, %d edges" % (PLATE_XZ.Area, len(PLATE_XZ.Edges)))
    return upper


def cover_outline(inset):
    """XZ face of the upper-arm cover (inset 3.5) or its pocket (3.2): the plate outline
    PLATE_XZ shrunk by `inset` so the border round the cover is even, like the turret
    cheeks'; falls back to the old banana band when there is no outline."""
    if COVER_FOLLOWS_PLATE and PLATE_XZ is not None:
        try:
            f = PLATE_XZ.makeOffset2D(-inset, join=0)
            f = f.Faces[0] if f.Faces else None
            if f is not None and f.isValid() and f.Area > 0.5 * PLATE_XZ.Area:
                return f
            log("cover_outline(%s): offset gave a bad face, falling back" % inset)
        except Exception as e:
            log("cover_outline(%s): offset failed (%s), falling back" % (inset, e))
    return arm_profile(ARM_T0, 1, inset)


def elbow_pocket(elbow, clear=2.5, half_y=PLATE_IN, step=1, smooth=0):
    """True swept area of the elbow (y = 0 section) and the forearm root over
    the J3 range, merged in the system Python with shapely (same helper as
    klik_freecad), cut across |y| < half_y."""
    import json, os, subprocess
    wires = elbow.slice(Y, 0.0)
    outer = max(wires, key=lambda w: w.BoundBox.DiagonalLength)
    outline = [(p.x, p.z) for p in outer.discretize(Distance=0.5)]
    RE = max(FH, WRIST_R)
    rects = [(J4_GAP_X - 1, J4Z - RE, J4X + J5X + RE, J4Z + RE)]   # forearm at J4 = 0, to the J5 end (its round end is r WRIST_R)
    lo, hi = LIMITS[2]
    job = json.dumps({"outline": outline, "rects": rects, "clear": clear,
                      "angles": list(range(lo, hi, step)) + [hi]})
    env = {kk: v for kk, v in os.environ.items()
           if not kk.startswith("PYTHON") and kk not in ("LD_LIBRARY_PATH", "LD_PRELOAD", "APPDIR", "APPIMAGE")}
    env["PATH"] = "/usr/bin:/bin"
    out = subprocess.run(["/usr/bin/python3", "-c", k1.POCKET_HELPER], input=job,
                         capture_output=True, text=True, timeout=120, env=env)
    if out.returncode:
        raise RuntimeError("elbow_pocket helper failed: " + out.stderr[-500:])
    xz = json.loads(out.stdout)
    if smooth:
        # Fill the facet notches (closing, so clearance only grows) and fit one smooth
        # periodic spline through the outline instead of ~1-degree polygon facets.
        out = subprocess.run(["/usr/bin/python3", "-c", SMOOTH_HELPER],
                             input=json.dumps({"pts": xz, "r": smooth, "grow": 0.3, "ro": 6.0, "ds": 2.0}),
                             capture_output=True, text=True, timeout=120, env=env)
        if out.returncode:
            raise RuntimeError("elbow_pocket smoothing failed: " + out.stderr[-500:])
        sp = [V(x, 0, z) for x, z in json.loads(out.stdout)]
        bs = Part.BSplineCurve()
        bs.interpolate(sp, PeriodicFlag=True)
        # Re-fit as an approximated spline (0.05 mm) through 1 mm samples of it: booleans
        # between the interpolated periodic spline's extrusion and the FHA-11C hub arm
        # silently returned nothing (2026-10-25); the approximated one cuts cleanly.
        pts = Part.Wire(bs.toShape()).discretize(Distance=1.0)
        ba = Part.BSplineCurve()
        ba.approximate(Points=pts[:-1] + [pts[0]], Tolerance=0.05)
        ea = ba.toShape()
        p0, p1 = ea.Vertexes[0].Point, ea.Vertexes[-1].Point
        edges = [ea] if (p1 - p0).Length < 1e-7 else [ea, Part.LineSegment(p1, p0).toShape()]   # close the end gap
        face = Part.Face(Part.Wire(edges))
        xz_face = Part.Face(Part.Wire(bs.toShape()))    # the periodic one offsets cleanly (hollow_upper_arm)
    else:
        face = Part.Face(Part.makePolygon([V(x, 0, z) for x, z in xz]))
        xz_face = face
    global CROTCH_XZ
    CROTCH_XZ = xz_face                                    # y = 0 outline: hollow_upper_arm keeps the core a wall below it
    sol = face.extrude(V(0, 2 * half_y, 0)).translated(V(0, -half_y, 0))
    if not 0.99 * face.Area * 2 * half_y < sol.Volume < 1.01 * face.Area * 2 * half_y:
        log("elbow_pocket: WARNING solid volume %.0f vs face %.0f x %.0f" % (sol.Volume, face.Area, 2 * half_y))
    return sol

def cheek_face():
    """XZ face (y = 0, Link1 frame) of a turret cheek: the J2 hub (r J2_HUB_R about J2,
    concentric with the upper arm's hub so the two read as one round shoulder), then
    flanks tangent to it down to the edges of the collar flat (|x| = CHEEK_R about J1)
    at the collar top, then straight down to the collar bottom.
    (Before the FHA-11C hub, 2026-10-25, the flanks continued the arm band's edges.)"""
    tb, R = J1_GAP, CHEEK_R
    ct = J1_GAP + COLLAR_H
    c = V(J2X, 0, J2Z)
    r = J2_HUB_R

    def tangent(p):
        # tangent point on the hub circle from p, on the side that keeps the outline
        # counter-clockwise (front: up the +x side; back: down the -x side)
        d = p - c
        L = d.Length
        a = math.acos(r / L)
        base = math.atan2(d.z, d.x)
        cands = [c + V(math.cos(base + s * a), 0, math.sin(base + s * a)) * r for s in (1, -1)]
        # front (p.x > 0): the point with the larger x; back: the smaller x
        return max(cands, key=lambda q: q.x) if p.x > c.x else min(cands, key=lambda q: q.x)

    f_bot, b_bot = V(R, 0, ct), V(-R, 0, ct)
    f_top, b_top = tangent(f_bot), tangent(b_bot)
    top = Part.Arc(f_top, c + V(0, 0, r), b_top).toShape()             # over the top of the hub
    w = Part.Wire(Part.__sortEdges__([
        Part.LineSegment(V(-R, 0, tb), V(R, 0, tb)).toShape(),
        Part.LineSegment(V(R, 0, tb), f_bot).toShape(),
        Part.LineSegment(f_bot, f_top).toShape(), top,
        Part.LineSegment(b_top, b_bot).toShape(),
        Part.LineSegment(b_bot, V(-R, 0, tb)).toShape()]))
    face = Part.Face(w)
    # Round the kinks where the flanks meet the straight drops to the collar (front
    # CHEEK_KINK_R, back CHEEK_KINK_R_BACK): fillet a thin extrusion, take its y = 0 face back.
    for x, pt, rad in ((R, f_bot, CHEEK_KINK_R), (-R, b_bot, CHEEK_KINK_R_BACK)):
        sol = face.extrude(V(0, 1, 0))
        kink = [e for e in sol.Edges if is_along(e, Y) and abs(e.Vertexes[0].Point.x - x) < 1e-6
                and abs(e.Vertexes[0].Point.z - pt.z) < 1e-6]
        try:
            sf = sol.makeFillet(rad, kink)
            if sf.isValid() and kink:
                face = [f for f in sf.Faces if f.BoundBox.YMax < 1e-6][0]
            else:
                log("cheek_face: kink fillet r%s at x %s not applied" % (rad, x))
        except Exception as e:
            log("cheek_face: kink fillet r%s at x %s failed: %s" % (rad, x, e))
    return face


def cheek_cover_face(inset, zb):
    """The cheek outline shrunk by `inset` and cut off at z = zb."""
    f = cheek_face().makeOffset2D(-inset).Faces[0]
    return f.common(Part.Face(Part.makePolygon([V(-90, 0, zb), V(90, 0, zb), V(90, 0, J2Z + 50),
                                                V(-90, 0, J2Z + 50), V(-90, 0, zb)]))).Faces[0]


def cheek_cover_solid(inset, zb, y0, t, r=4):
    """cheek_cover_face extruded t from y0, its two bottom corners rounded r."""
    sol = cheek_cover_face(inset, zb).extrude(V(0, t, 0)).translated(V(0, y0, 0))
    return soft(sol, r, [e for e in sol.Edges if is_along(e, Y) and e.BoundBox.ZMax < zb + 0.1])


def turret_pocket(clear=1.5, step=1):
    """XZ face (Link2 frame) of the region the turret cheeks sweep relative to the
    upper arm over the J2 range, plus `clear`: the arms (|y| > SPINE_Y) must stay
    out of it. Same shapely merge as elbow_pocket."""
    import json, os, subprocess
    tb, ct = J1_GAP - J2Z, J1_GAP + COLLAR_H - J2Z      # turret bottom / collar top, relative to J2
    disk = [(p.x - J2X, p.z - J2Z) for p in cheek_face().OuterWire.discretize(Distance=1.0)]   # cheek outline about J2
    cw = math.sqrt(COLLAR_R ** 2 - SPINE_Y ** 2)          # collar half width at the arms' inner face
    rects = [(-cw - J2X, tb, cw - J2X, ct)]
    lo, hi = LIMITS[1][0], min(LIMITS[1][1], TURRET_POCKET_HI)   # capped so the arms' r29 end arc stays whole
    # The helper rotates like App.Rotation(Y, q) and then adds 135 to z; the turret
    # sits in Link2 rotated by -q2 about J2, so pass -q2 and take the 135 back off.
    job = json.dumps({"outline": disk, "rects": rects, "clear": clear,
                      "angles": [-q for q in list(range(lo, hi, step)) + [hi]]})
    env = {kk: v for kk, v in os.environ.items()
           if not kk.startswith("PYTHON") and kk not in ("LD_LIBRARY_PATH", "LD_PRELOAD", "APPDIR", "APPIMAGE")}
    env["PATH"] = "/usr/bin:/bin"
    out = subprocess.run(["/usr/bin/python3", "-c", k1.POCKET_HELPER], input=job,
                         capture_output=True, text=True, timeout=120, env=env)
    if out.returncode:
        raise RuntimeError("turret_pocket helper failed: " + out.stderr[-500:])
    pts = [V(x, 0, z - 135.0) for x, z in json.loads(out.stdout)]
    return Part.Face(Part.makePolygon(pts))


def build(doc_name=DOC):
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)
    make_sheet(doc)

    root = doc.addObject("App::Part", "KLiK3")
    root.Placement = App.Placement(V(0, 0, WORLD_Z), App.Rotation())
    l1 = joint_part(doc, root, "Link1", V(0, 0, 0), Z, "q1")
    l2 = joint_part(doc, l1, "Link2", V(J2X, 0, J2Z), Y, "q2")
    l3 = joint_part(doc, l2, "Link3", V(0, 0, L2), Y, "q3")
    l4 = joint_part(doc, l3, "Link4", V(J4X, 0, J4Z), X, "q4")
    l5 = joint_part(doc, l4, "Link5", V(J5X, 0, 0), Y, "q5")
    l6 = joint_part(doc, l5, "Link6", V(WRIST, 0, 0), X, "q6")

    # --- drives ---------------------------------------------------------------
    hd = HD_SIZES
    if USE_ACTUATORS:
        # Meca500 set. Each output face sits on the output link's face; J2/J3/J5 also get a
        # crossed-roller support bearing on the far side (J2_SUPPORT, J3_SUPPORT, J5_SUPPORT).
        G2 = SLOT_Y
        J = [(root, l1, V(0, 0, J1_GAP), V(0, 0, 0), Z),
             (l2, l1, V(0, SLOT_Y, 0), V(J2X, 0, J2Z), Y),             # J2 in the spine, output on the near cheek
             (l3, l2, V(0, PLATE_IN, 0), V(0, 0, L2), Y),             # J3 in the elbow, output on the near plate
             (l3, l4, V(J4_GAP_X, 0, J4Z), V(-J4X, 0, -J4Z), X),
             (l5, l4, V(0, -J5_GAP_Y, 0), V(J5X, 0, 0), -Y),          # J5 in the wrist drum, output on the far wall
             (l5, l6, V(WRIST - 6, 0, 0), V(-WRIST, 0, 0), X)]
        envs = [add_actuator(doc, j, ACTUATORS[j - 1][0], h, o, gp, off, tw, ACTUATORS[j - 1][1])
                for j, (h, o, gp, off, tw) in enumerate(J, 1)]
        (e1h, e1o), (e2h, _), (e3h, e3o), (e4h, e4o), (e5h, e5o), (e6h, e6o) = envs
    else:
        e1h, e1o = add_drive(doc, 1, root, l1, V(0, 0, J1_GAP), V(0, 0, 0), Z, Z, size=hd[0])
        # J2 in the upper-arm spine, output to the near cheek; motor tail in the far cheek.
        G2 = SPINE_Y + 1
        e2h, _ = add_drive(doc, 2, l2, l1, V(0, G2, 0), V(J2X, 0, J2Z), Y, Y, q_sign=-1, size=hd[1])
        # J3 in the elbow, output to the near side plate.
        e3h, e3o = add_drive(doc, 3, l3, l2, V(0, ELBOW_Y + 1, 0), V(0, 0, L2), Y, Y, q_sign=-1, size=hd[2])
        e4h, e4o = add_drive(doc, 4, l3, l4, V(J4_GAP_X, 0, J4Z), V(-J4X, 0, -J4Z), X, X, size=hd[3])
        # J5 direct: gear and coaxial motor inside the wrist drum, output to the far
        # forearm wall. J6 gets a 6 mm pancake motor so it clears the J5 motor (1.4 mm).
        e5h, e5o = add_drive(doc, 5, l5, l4, V(0, -J5_GAP_Y, 0), V(J5X, 0, 0), -Y, Y, q_sign=-1, size=hd[4])
        e6h, e6o = add_drive(doc, 6, l5, l6, V(WRIST - 6, 0, 0), V(-WRIST, 0, 0), X, X, size=hd[5], motor_len=6)

    # --- base (fixed) -------------------------------------------------------------
    plate = box(PLATE_X[1] - PLATE_X[0], 2 * PLATE_Y, PLATE_T, V(PLATE_X[0], -PLATE_Y, DISC_T))
    plate = plate.makeChamfer(12, [e for e in plate.Edges if is_along(e, Z)])
    pt = DISC_T + PLATE_T
    bpts = [V(bx, by, pt) for bx in (PLATE_X[0] + 9, PLATE_X[1] - 9) for by in (-PLATE_Y + 9, PLATE_Y - 9)]
    plate = plate.cut(cbores(bpts, Z, 4.5, 3.8, 2.75, PLATE_T - 3.8))        # counterbored M5 floor bolts
    add_shape(doc, root, "Base_plate", plate, PLATE)
    bolts = None
    for bp in bpts:
        bx, by = bp.x, bp.y
        if True:
            b = cyl(4, 3.5, V(bx, by, pt - 3.8), Z).cut(cyl(2, 3, V(bx, by, pt - 2.3), Z))
            bolts = b if bolts is None else bolts.fuse(b)
    add_shape(doc, root, "Base_bolts", bolts, k1.SCREW)
    block = cyl(BASE_HALF, BASE_TOP - pt, V(0, 0, pt), Z)
    cbox = box(-BOX_X, BOX_Y[1] - BOX_Y[0], BOX_TOP - pt, V(BOX_X, BOX_Y[0], pt))
    cbox = soft(cbox, 4, [e for e in cbox.Edges if is_along(e, Z) and e.BoundBox.XMax < BOX_X + 1])
    base = block.fuse(cbox).removeSplitter()
    base = soft(base, 2, [e for e in base.Edges if on_plane(e, z=BASE_TOP)])
    base = base.cut(e1h)
    if USE_ACTUATORS:
        # J1 actuator loads from below: hollow the base under it (straight slot of its outline,
        # connector box included) down to the base plate, which is the bottom cover
        import actuator_fit as af
        kind = ACTUATORS[0][0]
        M1 = af.FHA[int(kind[4:-1])][4] if kind.startswith("FHA") else 2.5
        base = base.cut(m.moved(af.prism(kind, ACT_CLEAR, -(J1_GAP - pt) - 1, -M1), ACT_PL[1]))
    if LIGHTEN:
        # ring pockets in the column from its underside (the base plate closes them): 4 mm outer
        # wall, 2 mm round the actuator bore, stop 6 below the J1 coil cavity; the sector toward
        # the connector box (flex, actuator connector) stays solid
        z1 = BASE_TOP - FLEX_W - 1.6 - 6
        ring = cyl(BASE_HALF - 4, z1 - pt, V(0, 0, pt - 0.01), Z).cut(cyl(33.5 + 2, z1 - pt + 1, V(0, 0, pt - 0.5), Z))
        secs = None
        for a0, a1 in ((-150, -95), (-85, -30), (-20, 35), (45, 100)):
            w = Part.Face(Part.makePolygon([V(0, 0, 0)] + [V(80 * math.cos(math.radians(a)), 80 * math.sin(math.radians(a)), 0)
                                                           for a in (a0, (a0 + a1) / 2, a1)] + [V(0, 0, 0)])).extrude(V(0, 0, 200)).translated(V(0, 0, -50))
            secs = w if secs is None else secs.fuse(w)
        base = checked_cut(base, soft(ring.common(secs), 2), "base column pockets")
    if J1_SUPPORT:
        # spigot round the J1 output disc carrying the support bearing's inner race; outer race in the turret
        zr0 = J1_GAP + 1
        spig = cyl(J1_BRG[0], zr0 + J1_BRG[2] - BASE_TOP + 0.5, V(0, 0, BASE_TOP - 0.5), Z).fuse(cyl(J1_BRG[0] + 1.5, zr0 - BASE_TOP + 0.5, V(0, 0, BASE_TOP - 0.5), Z))
        base = base.fuse(spig.cut(cyl(J1_SPIGOT_RI, 40, V(0, 0, BASE_TOP - 5), Z))).removeSplitter()
        rr = (J1_BRG[0] + J1_BRG[1]) / 2
        add_shape(doc, root, "J1_support_inner", cyl(rr - 0.15, J1_BRG[2], V(0, 0, zr0), Z).cut(cyl(J1_BRG[0], 20, V(0, 0, zr0 - 1), Z)), m.BEARING)
    add_shape(doc, root, "Base", base, BLACK)
    add_shape(doc, root, "Base_ring", cyl(40, 1, V(0, 0, BASE_TOP), Z).cut(e1h).cut(cyl(J1_BRG[1] + 0.6 if J1_SUPPORT else 0.01, 3, V(0, 0, BASE_TOP - 1), Z)), m.DARK)
    # ~13 mm letters centred on the flat side face of the connector box (x BOX_X+4..0 past its corner round)
    if OK_LOGO:      # OK mark over OXFORD KINEMATICS, bare aluminium on the black box
        add_ok_logo(doc, root, "Logo_base", [ok_logo(36, 0.6, V((BOX_X + 4) / 2, BOX_Y[1], (pt + BOX_TOP) / 2 + 9), Z, Y, text=True)])
        lb = None
    else:
        lb = klogo(6.75, 0.6, V((BOX_X + 4) / 2, BOX_Y[1], (pt + BOX_TOP) / 2), Z, Y)
    if lb:
        add_shape(doc, root, "Logo_base", lb, LOGO)
    conn = None
    for z in (36, 54):
        c = fuse(cyl(7, 4, V(BOX_X - 4, 30, z), X), cyl(5.5, 10, V(BOX_X - 14, 30, z), X))
        conn = c if conn is None else conn.fuse(c)
    add_shape(doc, root, "Connectors", conn, m.METAL)
    # Cables drop from the connectors and run along the ground (waypoints spaced so the
    # spline doesn't dip below z = 0).
    cab = k1.cable([V(BOX_X - 14, 30, 54), V(BOX_X - 30, 34, 50), V(BOX_X - 42, 46, 34), V(BOX_X - 42, 58, 14),
                    V(BOX_X - 28, 64, 6), V(BOX_X - 6, 64, 3.2), V(-20, 58, 2.7)], 2.6)
    cab = cab.fuse(k1.cable([V(BOX_X - 14, 30, 36), V(BOX_X - 26, 36, 32), V(BOX_X - 34, 46, 20),
                             V(BOX_X - 30, 56, 7), V(BOX_X - 12, 58, 3.2), V(-36, 52, 2.7)], 2.6))
    add_shape(doc, root, "Cables", cab, CABLE)

    # --- link 1: clevis turret -------------------------------------------------------
    d2 = drive_shapes(hd[1])
    p2 = 2.54 * hd[1]
    y0 = G2 - (d2["z_out"] - 4)
    rm, lm = 0.62 * p2, max(0.35 * p2, 10)
    motor_back = y0 + d2["z_mf"] - lm
    tb = J1_GAP
    # Round collar about J1 (as in the photos), flattened on both sides at |y| = SLOT_Y; the
    # two cheek plates (round-topped about J2) bolt onto those flats and run down to the
    # collar bottom. The flats come out 60 wide - the cheek width.
    ct = tb + COLLAR_H
    turret = cyl(COLLAR_R, COLLAR_H, V(0, 0, tb), Z).common(
        box(2 * COLLAR_R, 2 * SLOT_Y, COLLAR_H, V(-COLLAR_R, -SLOT_Y, tb)))
    # (collar weight pocket removed 2026-10-23 at user request: solid collar)
    turret = bevel(turret, 2).cut(e1o)                 # chamfer theme (2026-11-13; was r2)
    cheeks = None
    for sy in (1, -1):
        y_in = SLOT_Y if sy > 0 else -TURRET_HALF
        ch = cheek_face().extrude(V(0, TURRET_HALF - SLOT_Y, 0)).translated(V(0, y_in, 0))
        ch = bevel(ch, 2)                              # chamfer theme (2026-11-13; was r2)
        cheeks = ch if cheeks is None else cheeks.fuse(ch)
    cheeks = cheeks.cut(e1o)
    if J1_SUPPORT:
        zr0 = J1_GAP + 1
        rr = (J1_BRG[0] + J1_BRG[1]) / 2
        b1 = cyl(rr + 0.3, zr0 + J1_BRG[2] + 0.6 - tb + 0.1, V(0, 0, tb - 0.1), Z).cut(cyl(J1_SPIGOT_RI - 0.5, 40, V(0, 0, tb - 5), Z))   # spigot + inner-race clearance (0.6 over them)
        b1 = b1.fuse(cyl(J1_BRG[1], zr0 + J1_BRG[2] + 0.1 - tb + 0.1, V(0, 0, tb - 0.1), Z))                                         # outer race bore
        turret, cheeks = checked_cut(turret, b1, "turret J1 bearing bore"), cheeks.cut(b1)
        rr = (J1_BRG[0] + J1_BRG[1]) / 2
        add_shape(doc, l1, "J1_support_outer", cyl(J1_BRG[1], J1_BRG[2], V(0, 0, zr0), Z).cut(cyl(rr + 0.15, 20, V(0, 0, zr0 - 1), Z)), m.BEARING)
    if J2_SUPPORT:
        # J2 support bearing in the -Y cheek (1 mm in from its inner face)
        yb = SLOT_Y + 1
        cheeks = cheeks.cut(ycyl(J2_BRG[1], J2_BRG[2] + 2.0, V(J2X, -(SLOT_Y + (J2_BRG[2] + 2.0) / 2 - 0.1), J2Z)))   # 0.9 past the stub end
        add_shape(doc, l1, "J2_support_outer", ycyl(J2_BRG[1], J2_BRG[2], V(J2X, -(yb + J2_BRG[2] / 2), J2Z)).cut(ycyl(sum(J2_BRG[:2]) / 2 + 0.15, 10, V(J2X, -(yb + J2_BRG[2] / 2), J2Z))), m.BEARING)
    if not USE_ACTUATORS:
        cheeks = cheeks.cut(ycyl(0.22 * p2 + 0.6, 6, V(J2X, G2 + 2, J2Z)))          # J2 output hub
    if not USE_ACTUATORS and motor_back < -SLOT_Y:                                                      # motor tail bore
        cheeks = cheeks.cut(ycyl(rm + 1.5, -SLOT_Y - motor_back + 1.5, V(J2X, (motor_back - 1.5 - SLOT_Y) / 2, J2Z)))
    dfx = CHEEK_FIX_D
    for sy in (1, -1):
        for x in (-CHEEK_FIX_X, CHEEK_FIX_X):                                     # cheek-to-collar bolts
            for z in CHEEK_FIX_Z:
                cheeks = cheeks.cut(ycyl(0.5625 * dfx, TURRET_HALF - SLOT_Y + 0.2, V(x, sy * (TURRET_HALF + SLOT_Y) / 2, tb + z)))   # clearance
                turret = turret.cut(ycyl(0.42 * dfx, 10.2, V(x, sy * (SLOT_Y - 5), tb + z)))                                   # tap drill, 10 deep
        for x, z in CHEEK_DOWELS:                                                 # dowels: 5 mm each side of the joint
            pin = ycyl(1.5, 10, V(x, sy * SLOT_Y, tb + z))
            cheeks, turret = cheeks.cut(pin), turret.cut(pin)
    add_shape(doc, l1, "Turret", turret.removeSplitter(), ARM_FINISH)
    # Covers following the cheek outline (curved front included), CHEEK_COVER_IN smaller
    # all round, set flush into a 2.5 mm pocket in each cheek (0.3 mm clearance round the edge).
    cw, cb = CHEEK_R - CHEEK_COVER_IN, ct + CHEEK_COVER_IN                        # cover half width, bottom
    cov_f = cheek_cover_face(CHEEK_COVER_IN, cb)
    cbc = tb + CHEEK_COVER_IN                                                     # covers run down over the cheek-to-collar bolts
    covers = None
    for sy in (1, -1):
        y0 = TURRET_HALF - 2.5 if sy > 0 else -TURRET_HALF
        cheeks = cheeks.cut(cheek_cover_solid(CHEEK_COVER_IN - 0.3, cbc - 0.3, y0 if sy > 0 else y0 - 0.01, 2.51, 4.3))   # pocket breaks the outer face
        c = cheek_cover_solid(CHEEK_COVER_IN, cbc, y0, 2.5)
        covers = c if covers is None else covers.fuse(c)
    yo = TURRET_HALF + 0.1
    zs = cb + 5.5
    xs = [v.Point.x for v in cov_f.section(Part.LineSegment(V(-90, 0, zs), V(90, 0, zs)).toShape()).Vertexes]
    sp = [V(min(cw, max(xs)) - 5.5, yo - 0.1, zs), V(max(-cw, min(xs)) + 5.5, yo - 0.1, zs),
          V(J2X + 0.62 * (cw - 5.5), yo - 0.1, J2Z + 0.78 * (cw - 5.5)), V(J2X - 0.62 * (cw - 5.5), yo - 0.1, J2Z + 0.78 * (cw - 5.5))]
    if TRI_POCKETS:
        # triangular rib pockets under the cheek covers (cover sits on a 2.5 mm ledge),
        # leaving a 3.5 mm inner skin; solid boss round J2 and lands under the cover screws
        yfl = TURRET_HALF - 2.5
        depth = yfl - SLOT_Y - (2.5 if LIGHTEN else 3.5)                       # inner skin (3.5 -> 2.5: FEA, 2026-10-25)
        for sy in (1, -1):
            region = cheek_cover_face(CHEEK_COVER_IN + 2.5, cb + 2.5).translated(V(0, sy * yfl, 0))
            keep = [disc(V(J2X, sy * yfl, J2Z), J2_COIL_R + 10 if CABLES else 18, Y)]   # J2 boss (+ cable coil cavity behind it, outside the FHA-11C output disc)
            keep += [disc(V(p.x, sy * yfl, p.z), 5, Y) for p in sp]              # lands under the cover screws
            cheeks = cut_pockets(cheeks, tri_pockets(region, sy * Y, depth, keep, u=Z, margin=0), "cheeks")
    add_shape(doc, l1, "Turret_cheeks", cheeks, ARM_FINISH)
    sp = [V(p.x, TURRET_HALF, p.z) for p in sp]                                  # flush countersunk heads on the cover face
    spf = [V(p.x, -p.y, p.z) for p in sp]
    covers = covers.cut(countersinks(sp, Y, 2.5)).cut(countersinks(spf, -Y, 2.5))
    add_shape(doc, l1, "Turret_covers", covers, COVER_FINISH)
    # KLiK lettering on both cheek covers, centred on the J2 hub, upright at home
    yo = TURRET_HALF + 0.1
    if OK_LOGO:      # OK mark centred on J2, OXFORD KINEMATICS below it (between the cover screws)
        add_ok_logo(doc, l1, "Logo_turret", [ok_logo(23, 0.6, V(J2X, TURRET_HALF, J2Z + 2), Z, Y, text=True),
                                             ok_logo(23, 0.6, V(J2X, -TURRET_HALF, J2Z + 2), Z, -Y, text=True)])
    else:
        lt = klogo(6.75, 0.6, V(J2X, yo, J2Z), Z, Y)
        if lt:
            add_shape(doc, l1, "Logo_turret", lt.fuse(klogo(6.75, 0.6, V(J2X, -yo, J2Z), Z, -Y)), LOGO)
    cs_screws(doc, l1, "Turret_screws", sp, Y)
    cs_screws(doc, l1, "Turret_screws_far", spf, -Y)
    fix = [V(sx * CHEEK_FIX_X, TURRET_HALF - 2.5, tb + z) for sx in (1, -1) for z in CHEEK_FIX_Z]   # on the cover-pocket floor (under the cover)
    fixf = [V(p.x, -p.y, p.z) for p in fix]
    tc = doc.getObject("Turret_cheeks")
    cbd = CHEEK_FIX_D + 0.3                                                       # counterbore depth: heads 0.3 below the pocket floor
    tc.Shape = tc.Shape.cut(cbores(fix, Y, CHEEK_FIX_D, cbd)).cut(cbores(fixf, -Y, CHEEK_FIX_D, cbd))
    screws(doc, l1, "Cheek_bolts", [p - Y * cbd for p in fix], Y, CHEEK_FIX_D)
    screws(doc, l1, "Cheek_bolts_far", [p + Y * cbd for p in fixf], -Y, CHEEK_FIX_D)
    add_shape(doc, l1, "Cheek_dowels", Part.makeCompound([ycyl(1.5, 10, V(x, sy * SLOT_Y, tb + z)) for sy in (1, -1) for x, z in CHEEK_DOWELS]), m.STEEL)

    # --- link 3 shape first (the upper arm's slot is cut to its sweep) ------------------
    # Elbow (as in the photos): a round drum about J3 between the upper-arm
    # plates, and a narrower bevelled box along the J4 axis that runs from the
    # J4 seam back past J3, its top-rear corner rounded ELBOW_R.
    drum = soft(ycyl(DRUM_R, 2 * ELBOW_Y, V(0, 0, 0)), 2)
    hz0 = J4Z - ELBOW_HH
    # side profile: the box along J4, plus a fill behind J3 down to J3 height, so its back
    # face (x = -ELBOW_BACK = -DRUM_R) runs down tangent into the drum (user 2026-10-25)
    xe, zt = J4_GAP_X - 0.5, hz0 + 2 * ELBOW_HH
    xb = -ELBOW_BACK
    zb = 0.0 if ELBOW_BACK <= DRUM_R + 1e-6 else hz0
    # front: with ELBOW_FRONT_FILL the underside stops at x = DRUM_R and a face square to it
    # runs down to J3 height, tangent to the drum there (as the back face does)
    xf = DRUM_R if ELBOW_FRONT_FILL else 0.0
    prof = Part.Face(Part.makePolygon([V(xb, -ELBOW_HW, zb), V(xf, -ELBOW_HW, zb), V(xf, -ELBOW_HW, hz0), V(xe, -ELBOW_HW, hz0),
                                       V(xe, -ELBOW_HW, zt), V(xb, -ELBOW_HW, zt), V(xb, -ELBOW_HW, zb)]))
    top = prof.extrude(V(0, 2 * ELBOW_HW, 0))
    top = top.makeFillet(ELBOW_R, [e for e in top.Edges if is_along(e, Y) and on_plane(e, x=-ELBOW_BACK)
                               and e.BoundBox.ZMin > J4Z])
    if xf > 0 and ELBOW_FRONT_R > 0:
        top = top.makeFillet(ELBOW_FRONT_R, [e for e in top.Edges if is_along(e, Y) and on_plane(e, x=xf) and on_plane(e, z=hz0)])
    front = [e for e in top.Edges if xf > 0 and (is_along(e, Z) and on_plane(e, x=xf) or                 # its side edges (and the
             isinstance(e.Curve, Part.Circle) and abs(e.Curve.Radius - ELBOW_FRONT_R) < 1e-6 and          # fillet's), chamfered
             abs(abs(e.BoundBox.YMin) - ELBOW_HW) < 1e-6 and e.BoundBox.XMax < xe)]                       # like the underside's
    top = top.makeChamfer(EDGE_CH, [e for e in top.Edges if is_along(e, X)] + front)
    elbow = drum.fuse(top).removeSplitter()
    if not USE_ACTUATORS:
        elbow = elbow.fuse(ycyl(12, 10, V(0, -ELBOW_Y - 5, 0))).removeSplitter()     # J3 support trunnion
    elbow = elbow.cut(e3h).cut(e4h)
    if J3_SUPPORT and USE_ACTUATORS:
        # cap over the J3 actuator's tail (its bore breaks through the drum's -Y face), proud
        # 1.5 into the 2 mm running gap; a boss and stub on it carry the support bearing's
        # inner race in the upper arm's -Y plate
        y3 = PLATE_IN + 1.5                                     # bearing inner face (|y|)
        cap3 = ycyl(27.2, 1.5, V(0, -(ELBOW_Y + 0.75), 0)).fuse(ycyl(J3_BRG[0] + 3, y3 - 0.3 - ELBOW_Y - 1.5, V(0, -(ELBOW_Y + 1.5 + y3 - 0.3) / 2, 0)))
        cap3 = cap3.fuse(ycyl(J3_BRG[0], J3_BRG[2] + 0.3, V(0, -(y3 - 0.3 + J3_BRG[2] / 2 + 0.15), 0)))
        for i in range(6):                                              # M2 countersunk cap screws on r 30.5
            a = math.radians(30 + 60 * i)
            cap3 = cap3.cut(ycyl(1.1, 2, V(30.5 * math.cos(a), -(ELBOW_Y + 0.75), 30.5 * math.sin(a))))
            elbow = elbow.cut(ycyl(0.8, 6, V(30.5 * math.cos(a), -(ELBOW_Y - 3), 30.5 * math.sin(a))))
        add_shape(doc, l3, "J3_cap", cap3.removeSplitter(), ARM_FINISH)
        add_shape(doc, l3, "J3_support_inner", ycyl(sum(J3_BRG[:2]) / 2 - 0.15, J3_BRG[2], V(0, -(y3 + J3_BRG[2] / 2), 0)).cut(ycyl(J3_BRG[0], 10, V(0, -(y3 + J3_BRG[2] / 2), 0))), m.BEARING)

    # --- link 2: one-piece Denso-style upper arm -----------------------------------------
    # A J2 drum in the clevis (|y| < SPINE_Y) and two banana-curved fork arms out to
    # PLATE_OUT, all one body: the side profile is a band along a centre line bowed
    # ARM_SAG to the back, so the front (fold) edge is concave. The arms grow out of
    # the drum where a cylinder cut keeps them 2 mm off the turret cheeks; a U crotch
    # and the swept elbow pocket open the gap between them.
    # Drum + spine across |y| < SPINE_Y; the arms outside it are pills that follow the
    # covers (same outline, 3.5 mm larger, lower end concentric with the cover's).
    # (Built as one extrusion then trimmed: fusing pieces that share the spline faces
    # silently drops one of them.)
    upper = arm_profile(0, 1, 0).extrude(V(0, 2 * PLATE_OUT, 0)).translated(V(0, -PLATE_OUT, 0))
    # Round the outer edges now, while the slab is a plain extrusion; after the cuts below,
    # OCC fails or returns a broken solid on the spline sides.
    if ARM_OUTER_R:
        upper = upper.makeFillet(ARM_OUTER_R, [e for e in upper.Edges if on_plane(e, y=PLATE_OUT) or on_plane(e, y=-PLATE_OUT)])
    c0, n0, h0 = arm_centre(ARM_T0)
    tg = V(n0.z, 0, -n0.x)                                                       # along the arm
    if tg.dot(arm_centre(1)[0] - c0) < 0:
        tg = tg * -1
    tp = turret_pocket(clear=0.1)        # turret cheeks' sweep over J2; 0.1 so the arms' r29 end arc stays whole (0.13 gap at J2 -95)
    T0 = None
    if PLATE_END_HULL:
        # keep = hull of the end circle (c0, h0) and a PLATE_END_RF round tangent to the
        # straight front edge, open upward; the round sits as low as the cheeks allow
        A, Bf, m_ = front_line(0) if ARM_FRONT_LINE else (None, None, None)
        def keep_for(zf):
            if ARM_FRONT_LINE:
                u_ = Bf - A
                cf = A + u_ * ((zf - A.z) / u_.z) - m_ * PLATE_END_RF
            else:
                c_, n_, h_ = arm_centre(0.5)
                cf = V(c_.x + h_ - PLATE_END_RF, 0, zf)
            d = cf - c0
            L = d.Length
            du = d * (1 / L)
            w = V(du.z, 0, -du.x)
            if w.z > 0:
                w = w * -1                                      # the lower side
            k = (h0 - PLATE_END_RF) / L
            nn = du * k + w * math.sqrt(1 - k * k)
            t0, t1 = c0 + nn * h0, cf + nn * PLATE_END_RF
            top = max(c0.z, cf.z) + 300
            poly = Part.Face(Part.makePolygon([t0, t1, cf, V(cf.x + PLATE_END_RF, 0, cf.z), V(cf.x + PLATE_END_RF, 0, top),
                                               V(c0.x - h0, 0, top), V(c0.x - h0, 0, c0.z), c0, t0]))
            K = poly.fuse([Part.Face(Part.Wire(Part.makeCircle(h0, c0, Y))),
                           Part.Face(Part.Wire(Part.makeCircle(PLATE_END_RF, cf, Y)))]).removeSplitter()
            return K, t0, cf
        for zf in range(int(c0.z) - 40, int(c0.z) + 40):
            K, T0, cf = keep_for(zf)
            if K.distToShape(tp)[0] >= PLATE_END_CLEAR:
                break
        log("upper arm plate ends: front round r%s at (%.1f, %.1f), clear of the cheek sweep by %.1f" % (PLATE_END_RF, cf.x, cf.z, K.distToShape(tp)[0]))
        trim = Part.Face(Part.makePolygon([V(-300, 0, -300), V(300, 0, -300), V(300, 0, 600), V(-300, 0, 600), V(-300, 0, -300)])).cut(K)
    else:
        # half-plane on the J2 side of the cover's lower-end centre, minus that end's disk
        trim = Part.Face(Part.makePolygon([c0 + n0 * 300, c0 + n0 * 300 - tg * 300, c0 - n0 * 300 - tg * 300,
                                           c0 - n0 * 300, c0 + n0 * 300])).cut(
            Part.Face(Part.Wire(Part.makeCircle(h0, c0, Y))))
    # One fused cutter per side (trim + cheek clearance cylinder + turret pocket), so the
    # arms' inner faces at |y| = SPINE_Y come out as single faces, not split along the
    # cutters' seams (refine can't merge those afterwards).
    t = PLATE_OUT - SPINE_Y + 8
    side = Part.Face(Part.Wire(Part.makeCircle(max(CHEEK_R, J2_HUB_R) + 2, V(0, 0, 0), Y))).fuse([trim, tp]).removeSplitter()
    side = side.Faces[0] if len(side.Faces) == 1 else side
    for sy in (1, -1):
        upper = checked_cut(upper, side.extrude(V(0, t, 0)).translated(V(0, SPINE_Y if sy > 0 else -SPINE_Y - t, 0)), "arm side trim %+d" % sy)
    # 1 mm round on the plates' lower-end arc (the trim disk) where it meets the outer faces.
    # makeFillet fails there (the arc runs into the outer-edge fillets), so cut it instead:
    # a quarter-round ring revolved about the disk axis over the J2-side half.
    dirv, angs = n0, (180, -180)
    if T0 is not None:                    # only the arc from the back edge round to the tangent line
        dirv = (T0 - c0) * (1 / h0)
        a = math.degrees(math.acos(max(-1.0, min(1.0, dirv.dot(n0 * -1)))))
        angs = sorted((a, -a, 360 - a, a - 360), key=abs)

    def end_round(yo, sy, r=1.0):
        P = lambda rho, dy: c0 + dirv * rho + V(0, yo + sy * dy, 0)
        k = math.sqrt(0.5)
        f = Part.Face(Part.Wire([Part.LineSegment(P(h0 - r, 0.5), P(h0 - r, 0)).toShape(),
                                 Part.Arc(P(h0 - r, 0), P(h0 - r + r * k, -r + r * k), P(h0, -r)).toShape(),
                                 Part.LineSegment(P(h0, -r), P(h0 + 0.5, -r)).toShape(),
                                 Part.LineSegment(P(h0 + 0.5, -r), P(h0 + 0.5, 0.5)).toShape(),
                                 Part.LineSegment(P(h0 + 0.5, 0.5), P(h0 - r, 0.5)).toShape()]))
        for ang in angs:
            ring = f.revolve(c0, Y, ang)
            if ring.isInside(c0 - tg * (h0 + 0.2) + V(0, yo - sy * 0.2, 0), 1e-6, True):
                return ring
    for sy in ((1, -1) if ARM_OUTER_R else ()):
        upper = upper.cut(end_round(sy * PLATE_OUT, sy, ARM_OUTER_R))
    global PLATE_XZ
    PLATE_XZ = None
    if COVER_FOLLOWS_PLATE:
        # the fork plate's side outline (below the 1 mm face rounds), in the y = 0 plane
        ws = [w for w in upper.slice(Y, PLATE_OUT - 1.5) if w.isClosed()]
        if ws:
            PLATE_XZ = max((Part.Face(w) for w in ws), key=lambda f: f.Area).translated(V(0, -(PLATE_OUT - 1.5), 0))
            log("upper arm plate outline: %.0f mm2, %d edges" % (PLATE_XZ.Area, len(PLATE_XZ.Edges)))
        else:
            log("upper arm plate outline: slice failed, covers fall back to the arm band")
    # Crotch: one smooth cut - the elbow's swept pocket, notches filled and fitted with
    # a spline, its ends rounded so the floor blends into the arms' inner faces with a
    # true fillet; then its front/back edges rounded too.
    pk = elbow_pocket(elbow, smooth=8)
    try:
        if not CROTCH_FILLET: raise ValueError("crotch fillet off")
        pf = pk.makeFillet(CROTCH_FILLET, [e for e in pk.Edges if on_plane(e, y=PLATE_IN) or on_plane(e, y=-PLATE_IN)])
        # a fillet can come back valid but inside out (volume of everything else): only
        # take it if it is a slightly bigger version of the pocket
        if pf.isValid() and pk.Volume < pf.Volume < 1.2 * pk.Volume:
            pk = pf
        else:
            log("crotch fillet rejected: pocket %.0f, filleted %.0f mm3" % (pk.Volume, pf.Volume))
    except Exception:
        pass
    upper = checked_cut(upper, pk, "upper arm crotch")
    hub_seams = []
    if CROTCH_HUB_ARC:
        # Behind J2 the crotch floor stands above the hub circle. Bring it down to the arc:
        # the floor follows the arc from the back face (tangent there) to Q, then one smooth
        # curve (cubic Hermite, tangent to the arc at Q and to the floor at F) into the floor.
        R = J2_HUB_R
        sec = max((Part.Face(w) for w in upper.slice(Y, 0) if w.isClosed()), key=lambda f: f.Area)
        q = [p for p in sec.OuterWire.discretize(Distance=0.25) if p.z > 30 and -30 < p.x < 30]
        q.sort(key=lambda p: p.x)

        def floor_at(x):
            for a, b in zip(q, q[1:]):
                if a.x <= x <= b.x and b.x > a.x:
                    return a.z + (b.z - a.z) * (x - a.x) / (b.x - a.x)
            return None

        def herm(P0, D0, P1, D1, n=40):
            out = []
            for k in range(n + 1):
                t = k / n
                h00, h10, h01, h11 = 2 * t ** 3 - 3 * t ** 2 + 1, t ** 3 - 2 * t ** 2 + t, -2 * t ** 3 + 3 * t ** 2, t ** 3 - t ** 2
                out.append(P0 * h00 + D0 * h10 + P1 * h01 + D1 * h11)
            return out

        best = None
        for fx in [x * 0.5 for x in range(-36, 1)]:                # F between x -18 and 0
            fz = floor_at(fx)
            fz2 = floor_at(fx + 0.5)
            if fz is None or fz2 is None:
                continue
            F = V(fx, 0, fz)
            tF = V(0.5, 0, fz2 - fz)
            tF.normalize()
            for ai in range(0, 41):
                al = math.radians(100 + ai * 1.25)                  # Q on the arc, 100..150 deg
                Q = V(R * math.cos(al), 0, R * math.sin(al))
                tQ = V(math.sin(al), 0, -math.cos(al))              # along the arc, toward the top
                L = (F - Q).Length
                pts = herm(Q, tQ * L, F, tF * L)
                ok = all(b.x > a.x for a, b in zip(pts, pts[1:]))
                ok = ok and all(math.hypot(p.x, p.z) >= R - 0.05 for p in pts)
                ok = ok and all((floor_at(p.x) or -1e9) >= p.z - 0.05 for p in pts[1:-1])
                if not ok:
                    continue
                turn = 0.0                                          # sharpest bend (deg per mm)
                for a, b, c in zip(pts, pts[1:], pts[2:]):
                    u1, u2 = b - a, c - b
                    ang = math.degrees(math.atan2(u1.x * u2.z - u1.z * u2.x, u1.dot(u2)))
                    turn = max(turn, abs(ang) / max(1e-6, (u1.Length + u2.Length) / 2))
                if best is None or turn < best[0]:
                    best = (turn, Q, F, pts)
        if best is not None:
            turn, Q, F, pts = best
            cur = Part.BSplineCurve()
            cur.interpolate(pts[::-1])
            edges = [Part.LineSegment(V(-80, 0, 0), V(-80, 0, 120)).toShape(), Part.LineSegment(V(-80, 0, 120), V(F.x, 0, 120)).toShape(),
                     Part.LineSegment(V(F.x, 0, 120), F).toShape(), cur.toShape(),
                     Part.LineSegment(Q, V(0, 0, 0)).toShape(), Part.LineSegment(V(0, 0, 0), V(-80, 0, 0)).toShape()]
            tool = Part.Face(Part.Wire(edges)).cut(Part.Face(Part.Wire(Part.makeCircle(R, V(0, 0, 0), Y))))
            upper = checked_cut(upper, tool.extrude(V(0, 2 * PLATE_IN, 0)).translated(V(0, -PLATE_IN, 0)), "spine to the J2 hub arc")
            hub_seams = [(Q.x, Q.z), (F.x, F.z)]
            if PLATE_END_FLOW:
                dF = pts[-1] - pts[-2]
                dF.normalize()
                upper = plate_end_flow(upper, F, dF)
            if PLATE_BACK_LINE:
                upper = plate_back_line(upper)
            if PLATE_STRAP_CUT and T0 is not None:
                upper = plate_strap_cut(upper, T0)
            if FORK_CORNER_CH:
                upper = fork_corners(upper)
            log("spine floor on the J2 hub arc to (%.1f, %.1f), smooth curve into the crotch at (%.1f, %.1f), max bend %.1f deg/mm" % (Q.x, Q.z, F.x, F.z, turn))
        else:
            log("spine hub arc: blend NOT found")
    edge_fb = [e for e in upper.Edges if 40 < e.BoundBox.ZMin and e.BoundBox.ZMax < 120 and e.BoundBox.XLength < 1.5
               and e.BoundBox.ZLength < 1.5 and abs(e.BoundBox.YLength - 2 * PLATE_IN) < 12]
    # Round them one at a time (largest radius that works), so one failure can't silently
    # drop the others: the back one is where the spine's back face meets the crotch floor.
    spots = [(e.BoundBox.Center.x, e.BoundBox.Center.z) for e in edge_fb]
    spots = [(x, z) for x, z in spots if all(math.hypot(x - a, z - b) > 2 for a, b in hub_seams)]   # tangent seams: nothing to round
    for x, z in (spots if CROTCH_EDGE_R else []):
        done = False
        for rad in CROTCH_EDGE_R:
            es = [e for e in upper.Edges if abs(e.BoundBox.Center.x - x) < 0.5 and abs(e.BoundBox.Center.z - z) < 0.5
                  and e.BoundBox.YLength > 40]
            try:
                uf = upper.makeFillet(rad, es)
                if uf.isValid() and abs(uf.Volume - upper.Volume) < 0.05 * upper.Volume:
                    upper, done = uf, True
                    log("crotch edge (x %.1f, z %.1f): fillet r%s" % (x, z, rad))
                    break
            except Exception:
                pass
        if not done:
            r2d = round_corner_2d(upper, x, z, 6, PLATE_IN)
            if r2d is not None:
                upper = r2d
                log("crotch edge (x %.1f, z %.1f): fillet failed, rounded r6 in 2D instead" % (x, z))
            else:
                log("crotch edge (x %.1f, z %.1f): fillet FAILED" % (x, z))
    if CROTCH_FRONT_CH and spots:
        upper = crotch_front_chamfer(upper, max(spots), CROTCH_FRONT_CH)
    upper = checked_cut(checked_cut(upper, e2h, "upper arm J2 bore"), e3o, "upper arm J3 seat")
    if not USE_ACTUATORS:
        upper = upper.cut(ycyl(18.2, 8.5, V(0, -PLATE_IN - 4.25, L2)))           # J3 support seat
    # Pockets for the flush covers: cover outline + 0.3 mm, 2.5 mm deep in each arm face.
    pocket = cover_outline(3.5 - 0.3).extrude(V(0, 2.51, 0))
    upper = checked_cut(checked_cut(upper, pocket.translated(V(0, PLATE_OUT - 2.5, 0)), "cover pocket +Y"), pocket.translated(V(0, -PLATE_OUT - 0.01, 0)), "cover pocket -Y")
    if HOLLOW_UPPER:
        upper = hollow_upper_arm(upper, e2h, e3o)
    if J3_SUPPORT and USE_ACTUATORS:
        # -Y plate: outer-race bore from the plate face (clears the cap's boss), stub-end relief
        y3 = PLATE_IN + 1.5
        tool = ycyl(J3_BRG[1], y3 + J3_BRG[2] - PLATE_IN + 0.01, V(0, -(PLATE_IN + y3 + J3_BRG[2]) / 2 + 0.005, L2))
        tool = tool.fuse(ycyl(sum(J3_BRG[:2]) / 2 + 0.5, J3_BRG[2] + 1.0, V(0, -(y3 + J3_BRG[2] / 2 + 0.5), L2)))
        upper = checked_cut(upper, tool, "upper arm J3 support bore")
        add_shape(doc, l2, "J3_support_outer", ycyl(J3_BRG[1], J3_BRG[2], V(0, -(y3 + J3_BRG[2] / 2), L2)).cut(ycyl(sum(J3_BRG[:2]) / 2 + 0.15, 10, V(0, -(y3 + J3_BRG[2] / 2), L2))), m.BEARING)
    if J2_SUPPORT:
        # stub on the -Y hub face (the spine skin over the FHA-11C's end) into the -Y cheek's support bearing
        y0, y1 = J2_BARREL_Y - 0.5, SLOT_Y + 1 + J2_BRG[2]
        upper = upper.fuse(ycyl(J2_BRG[0], y1 - y0, V(0, -(y0 + y1) / 2, 0)))
        add_shape(doc, l2, "J2_support_inner", ycyl(sum(J2_BRG[:2]) / 2 - 0.15, J2_BRG[2], V(0, -(SLOT_Y + 1 + J2_BRG[2] / 2), 0)).cut(ycyl(J2_BRG[0], 10, V(0, -(SLOT_Y + 1 + J2_BRG[2] / 2), 0))), m.BEARING)
    add_shape(doc, l2, "UpperArm", upper, ARM_FINISH)   # (no refine: on the FHA-11C hub arm it left a solid later booleans fail on)
    if not USE_ACTUATORS:
        add_shape(doc, l2, "J3_support",
                  ycyl(18, 7, V(0, -PLATE_IN - 4.5, L2)).cut(ycyl(12.15, 9, V(0, -PLATE_IN - 4.5, L2))), m.BEARING)
    # Long pill covers following the arm's curve, recessed flush, KLiK logo on each.
    t0 = ARM_T0
    cov_xz = cover_outline(3.5)
    cov = cov_xz.extrude(V(0, 2.5, 0)).translated(V(0, -1.25, 0))
    try:
        cf = cov.makeFillet(0.8, cov.Edges)
        if cf.isValid():
            cov = cf
    except Exception:
        pass
    add_shape(doc, l2, "Upper_covers", cov.translated(V(0, PLATE_OUT - 1.25, 0)).fuse(       # flush in the arm faces
        cov.translated(V(0, -PLATE_OUT + 1.25, 0))), COVER_FINISH)
    zo = PLATE_OUT + 0.1
    tm = (t0 + 1) / 2
    cm, nm, _ = arm_centre(tm)
    up = Z                                                                       # level in the home pose
    # Upright and horizontal at home (letter tops up), sized to fit the cover width.
    # (KLiK logo moved from the upper-arm covers to the turret cheek covers, user 2026-10-25)
    sp = []
    for t in (t0, 1.0):
        c, n, h = arm_centre(t)
        r = h - 3.5 - 5
        sp += [V(c.x + n.x * r, zo - 0.1, c.z + n.z * r), V(c.x - n.x * r, zo - 0.1, c.z - n.z * r)]
    if PLATE_XZ is not None:
        # keep each screw >= 5 mm inside the new cover edge: pull it onto that inset outline if not
        try:
            inner = cov_xz.makeOffset2D(-5)
            sp = [p if inner.isInside(V(p.x, 0, p.z), 1e-6, True) else
                  inner.OuterWire.distToShape(Part.Vertex(V(p.x, 0, p.z)))[1][0][0] for p in sp]
            if math.hypot(sp[0].x - sp[1].x, sp[0].z - sp[1].z) < 20:
                # lower pair crowded (narrow strap-cut cover): spread them across the cover's
                # lower end, on the 5 mm inset line 8 mm above its bottom
                # (raised until both are >= 8 mm off the split bolts, whose heads sit under the covers)
                for dz in range(8, 40):
                    zl = inner.BoundBox.ZMin + dz
                    cut = inner.OuterWire.section(Part.makeLine(V(-200, 0, zl), V(200, 0, zl)))
                    xs = sorted(v.Point.x for v in cut.Vertexes)
                    if len(xs) >= 2 and all(math.hypot(x - bx, zl - bz) >= 8 for x in (xs[0], xs[-1]) for bx, bz in (UA_SPLIT_COVER if SPLIT_UPPER else [])):
                        sp[0], sp[1] = V(xs[-1], sp[0].y, zl), V(xs[0], sp[1].y, zl)
                        log("upper cover screws: lower pair spread to x %.1f / %.1f at z %.1f" % (xs[-1], xs[0], zl))
                        break
        except Exception as e:
            log("upper cover screws: inset check failed: %s" % e)
    sp = [V(p.x, PLATE_OUT, p.z) for p in sp]                                  # flush countersunk heads on the cover face
    spf = [V(p.x, -p.y, p.z) for p in sp]
    uc = doc.getObject("Upper_covers")
    uc.Shape = uc.Shape.cut(countersinks(sp, Y, 2.5)).cut(countersinks(spf, -Y, 2.5))
    cs_screws(doc, l2, "Upper_screws", sp, Y)
    cs_screws(doc, l2, "Upper_screws_far", spf, -Y)

    lid_sy = 13
    if USE_ACTUATORS:
        # J4 actuator drops in from above (its connector box blocks sliding it in axially):
        # clear the space over it up to ELBOW_LID_Z, open to the seam face (the forearm collar
        # covers that), and make everything above ELBOW_LID_Z from the actuator's back end
        # forward a bolt-on lid, screwed into the side walls
        x_a = J4_GAP_X - 56.5 - ACT_CLEAR - 0.5
        hw = 15 + ACT_CLEAR
        elbow = elbow.cut(box(J4_GAP_X + 1 - x_a, 2 * hw, ELBOW_LID_Z - J4Z, V(x_a, -hw, J4Z)))
        lidbox = box(J4_GAP_X + 1 - x_a, 2 * ELBOW_HW + 10, 40, V(x_a, -ELBOW_HW - 5, ELBOW_LID_Z))
        add_shape(doc, l3, "Elbow_lid", elbow.common(lidbox), ARM_FINISH)
        elbow = elbow.cut(lidbox)
        lid_sy = (hw + ELBOW_HW) / 2                                             # screws into the side walls
    add_shape(doc, l3, "Elbow", elbow, ARM_FINISH)
    ep = [V(x, sy * lid_sy, J4Z + ELBOW_HH) for i in range(3) for x in (-ELBOW_BACK + ELBOW_R + 5 + i * (J4_GAP_X - 6 + ELBOW_BACK - ELBOW_R - 5) / 2,) for sy in (1, -1)]
    lid = doc.getObject("Elbow_lid") if USE_ACTUATORS else doc.getObject("Elbow")
    lid.Shape = lid.Shape.cut(cbores(ep, Z, hole_r=1.7, hole_len=(J4Z + ELBOW_HH - 3.3 - ELBOW_LID_Z) if USE_ACTUATORS else 0))   # counterbore + clearance through the lid
    screws(doc, l3, "Elbow_screws", [p - Z * 3.3 for p in ep], Z)

    # --- link 4: rounded forearm with the wrist window ------------------------------------
    # As in the photos: a rounded-edged box with a semicircular end about J5,
    # a 43 mm window through the end (top, bottom and front) for the wrist,
    # and a pill-shaped pocket in each side with a shorter KLiK cover in it.
    x0 = J4_GAP_X - J4X                                   # J4 face in the forearm frame (-13)
    xf = x0 + 2                                           # forearm body starts behind the dark seam ring
    # side profile: FH tall at the J4 end, straight tangent lines out to the r WRIST_R round
    # end about J5 (the J5 drum is bigger than the forearm's half height)
    RE = max(FH, WRIST_R)
    if RE > FH + 1e-6:
        # tangent point from P = (xf, FH) to the circle about (J5X, 0), on its upper side
        cands = []
        for sgn in (1, -1):
            ang = math.atan2(FH, xf - J5X) + sgn * math.acos(RE / math.hypot(xf - J5X, FH))
            cands.append((J5X + RE * math.cos(ang), RE * math.sin(ang)))
        tx, tz = max(cands, key=lambda q: q[1])
        FA_SLOPE = (tz - FH) / (tx - xf)
        P = lambda x, z: V(x, -FW, z)
        w = Part.Wire(Part.__sortEdges__([Part.LineSegment(P(xf, -FH), P(xf, FH)).toShape(),
                                         Part.LineSegment(P(xf, FH), P(tx, tz)).toShape(),
                                         Part.Arc(P(tx, tz), P(J5X + RE, 0), P(tx, -tz)).toShape(),
                                         Part.LineSegment(P(tx, -tz), P(xf, -FH)).toShape()]))
        prof = Part.Face(w).extrude(V(0, 2 * FW, 0))
    else:
        FA_SLOPE = 0.0
        prof = fuse(box(J5X - xf, 2 * FW, 2 * FH, V(xf, -FW, -FH)), ycyl(FH, 2 * FW, V(J5X, 0, 0))).removeSplitter()
    fa_h = lambda x: FH + FA_SLOPE * (x - xf)               # forearm half height along x (the tapered part)
    side_edges = [e for e in prof.Edges if (on_plane(e, y=FW) or on_plane(e, y=-FW)) and e.BoundBox.XMax > xf + 0.5]
    forearm = prof.makeChamfer(EDGE_CH, side_edges)        # chamfer theme (2026-11-13; was an r7 round)
    try:
        forearm = forearm.makeChamfer(1.5, [e for e in forearm.Edges if on_plane(e, x=xf)])
    except Exception:
        pass
    # Elbow-width collar then a taper out to the full forearm width (as in the
    # photos): keeps the wide forearm clear of the upper-arm side plates when J3 folds.
    xc, xt = xf + COLLAR4_L, xf + COLLAR4_L + TAPER4_L
    hz = fa_h(xc) + 0.01                                  # just over the forearm so top/bottom stay its own faces
    cr = EDGE_CH                                          # collar + taper corners chamfered like the forearm/elbow (2026-11-13; were r5 -> r7)
    env = Part.Face(crect_yz(x0 - 1, ELBOW_HW, hz, cr)).extrude(V(xc - x0 + 1, 0, 0))
    env = env.fuse(Part.makeLoft([crect_yz(xc, ELBOW_HW, hz, cr), crect_yz(xt, FW, fa_h(xt) + 0.01, cr)], True, True))
    env = env.fuse(box(J5X + RE + 10 - xt, 2 * FW + 10, 2 * RE + 10, V(xt, -FW - 5, -RE - 5))).removeSplitter()
    fc0 = forearm.common(env)
    forearm = fc0 if fc0.Volume > 1 else forearm.common(env, 0.001)   # the steep taper loft can defeat the plain boolean
    slot = box(RE + J5X - SLOT_X + 10, 2 * J5_GAP_Y, 2 * RE + 20, V(SLOT_X, -J5_GAP_Y, -RE - 10))
    slot = soft(slot, 6, [e for e in slot.Edges if is_along(e, Z) and on_plane(e, x=SLOT_X)])
    forearm = forearm.cut(slot)
    if WINDOW_CH > 0:
        # chamfer the window back face's top/bottom edges: slide the window's section (with
        # its r6 corners) back along the chamfer line, so the chamfer follows the corners
        zt = fa_h(SLOT_X)
        cs = 1 / math.sqrt(1 + FA_SLOPE ** 2)
        d = V(-WINDOW_CH * cs, 0, WINDOW_CH * (1 - FA_SLOPE * cs))                # back-face point -> top-face point
        sec = Part.Face(Part.Wire(slot.slice(Z, zt - WINDOW_CH)[0]))
        tool = sec.extrude(d * 1.6)
        tool = tool.fuse(sec.translated(d * 1.6).extrude(V(0, 0, 20)))
        tool = tool.fuse(tool.mirror(V(0, 0, 0), Z))
        forearm = forearm.cut(tool)
    POCKET = (55, J5X, 18.5)                              # side pocket: arc centres and radius (concentric with the cover: 2 mm round it; was 40, too close to the J4 collar taper)
    COVER_X = (55, J5X, 16.5)                             # KLiK cover in the far end of the pocket
    for sy in (1, -1):
        forearm = forearm.cut(hull_xz(V(POCKET[0], 0, 0), POCKET[2], V(POCKET[1], 0, 0), POCKET[2], 4).translated(V(0, sy * FW, 0)))
    # J5: gear + motor in the wrist drum, output to the far wall (e5o); the near
    # wall carries an outboard support bearing on a stub from the drum.
    if not USE_ACTUATORS:
        forearm = forearm.cut(ycyl(9.2, 3.6, V(J5X, J5_GAP_Y + 1.8, 0)))              # J5 outboard bearing seat
        add_shape(doc, l4, "J5_support_outer",
                  ycyl(9, 3.5, V(J5X, J5_GAP_Y + 1.75, 0)).cut(ycyl(7.15, 5, V(J5X, J5_GAP_Y + 1.75, 0))), m.BEARING)
    forearm = forearm.cut(e4o).cut(e5o)
    if TRI_POCKETS:
        # triangular rib pockets into the window's back face (hidden behind the wrist
        # drum), 20 deep: 3 mm top/bottom skins, side walls >= 5 mm where the taper starts
        region = Part.Face(rrect_yz(SLOT_X, J5_GAP_Y - 6, fa_h(SLOT_X) - (WINDOW_CH + 1.5 if WINDOW_CH > 0 else 0), 4))   # below the window chamfers
        depth = SLOT_X - FA_POCKET_X0 if LIGHTEN else 20
        keep = [Part.Face(Part.makePolygon([V(SLOT_X, y, z) for y, z in ((12, 8), (26, 8), (26, 24), (12, 24), (12, 8))]))] if LIGHTEN and CABLES else []   # J4->J5 flex runs here (y 13-24, z 14-18)
        forearm = cut_pockets(forearm, tri_pockets(region, X, depth, keep, u=Y, cell=18, rib=3, margin=3), "forearm")
    if CABLES and USE_ACTUATORS:
        forearm = forearm.cut(ycyl(7, FW - 2 - J5_GAP_Y + 0.5, V(J5X, (J5_GAP_Y + FW - 2) / 2, 0)))   # wall hole for the wrist's hollow flex hub, through to the cover-pocket floor
    if J5_SUPPORT and CABLES and USE_ACTUATORS:
        y5 = J5_GAP_Y + 0.3                                                       # outer race from the wall's inner face, 1.2 lip under the cover pocket
        y_a, y_b = J5_GAP_Y - 0.01, y5 + J5_BRG[2] + 0.1                          # bore from the wall face (clears the cap's boss)
        forearm = forearm.cut(ycyl(J5_BRG[1], y_b - y_a, V(J5X, (y_a + y_b) / 2, 0)))
        add_shape(doc, l4, "J5_support_outer", ycyl(J5_BRG[1], J5_BRG[2], V(J5X, y5 + J5_BRG[2] / 2, 0)).cut(ycyl(sum(J5_BRG[:2]) / 2 + 0.15, 10, V(J5X, y5 + J5_BRG[2] / 2, 0))), m.BEARING)
    rear = box(xc - x0 + 1, 2 * FW + 10, 2 * RE + 10, V(x0 - 1, -FW - 5, -RE - 5))
    add_shape(doc, l4, "Forearm_collar", forearm.common(rear), ARM_FINISH)
    add_shape(doc, l4, "Forearm", forearm.cut(rear), ARM_FINISH)
    ring = box(2, 2 * ELBOW_HW, 2 * ELBOW_HH, V(x0, -ELBOW_HW, -ELBOW_HH))
    ring = ring.makeChamfer(EDGE_CH, [e for e in ring.Edges if is_along(e, X)])
    add_shape(doc, l4, "Ring4", ring.cut(e4o), m.DARK)
    fc = hull_xz(V(COVER_X[0], 0, 0), COVER_X[2], V(COVER_X[1], 0, 0), COVER_X[2], 2.3)
    fc = soft(fc, 0.8)
    covers = fc.translated(V(0, -FW + 0.85, 0))
    ytop = FW + 0.3
    if CABLES and USE_ACTUATORS:
        # +Y cover raised into a dome: the J5 flex coil sits in a cavity under it, round the
        # wrist's hollow hub that comes out through the forearm wall on the J5 axis
        h = DOME_TOP - (FW - 2)
        dome = hull_xz(V(COVER_X[0], 0, 0), COVER_X[2], V(COVER_X[1], 0, 0), COVER_X[2], h).translated(V(0, FW - 2 + h / 2, 0))
        dome = soft(dome, 3, [e for e in dome.Edges if on_plane(e, y=DOME_TOP)])
        dome = soft(dome, 0.8, [e for e in dome.Edges if on_plane(e, y=FW - 2)])
        dome = dome.cut(ycyl(14.5, FLEX_W + 1.7, V(J5X, FW - 2 + (FLEX_W + 1.7) / 2 - 0.05, 0)))
        covers = covers.fuse(dome)
        ytop = DOME_TOP
    else:
        covers = covers.fuse(fc.translated(V(0, FW - 0.85, 0)))
    add_shape(doc, l4, "Forearm_covers", covers, COVER_FINISH)
    xl = (COVER_X[0] + COVER_X[1]) / 2
    if OK_LOGO:      # mark only on the small oval covers
        add_ok_logo(doc, l4, "Logo_forearm", [ok_logo(15, 0.6, V(xl, ytop, 0), Z, Y), ok_logo(15, 0.6, V(xl, -FW - 0.3, 0), Z, -Y)])   # halved 2026-11-13 (was 30; turret 46 -> 23)
        lf = None
    else:
        lf = klogo(5, 0.6, V(xl, ytop + 0.05, 0), Z, Y)
    if lf:
        add_shape(doc, l4, "Logo_forearm", lf.fuse(klogo(5, 0.6, V(xl, -FW - 0.35, 0), Z, -Y)), LOGO)

    # --- link 5: wrist drum + block -------------------------------------------------------
    drum5 = ycyl(WRIST_R, 2 * WRIST_Y, V(0, 0, 0))
    blk = box(WRIST, 2 * WRIST_BY, 2 * WRIST_BZ - 1, V(0, -WRIST_BY, -WRIST_BZ + 0.5))
    blk = bevel(blk, EDGE_CH, [e for e in blk.Edges if is_along(e, X)])     # chamfer theme (2026-11-13; was r3)
    wrist = bevel(drum5, 2).fuse(blk).removeSplitter()                      # (was r2)
    wrist = wrist.cut(cyl(12.6, 8, V(WRIST - 6, 0, 0), X))
    if not USE_ACTUATORS:
        wrist = wrist.fuse(ycyl(3.2, J5_GAP_Y + 3.5 - WRIST_Y + 1, V(0, (J5_GAP_Y + 3.5 + WRIST_Y - 1) / 2, 0)))   # support-bearing stub
        add_shape(doc, l5, "J5_support_inner",
                  ycyl(6.85, 3.5, V(0, J5_GAP_Y + 1.75, 0)).cut(ycyl(3.2, 5, V(0, J5_GAP_Y + 1.75, 0))), m.BEARING)
    wrist = wrist.cut(e5h).cut(e6h)
    if USE_ACTUATORS:
        import actuator_fit as af
        # J5 actuator slides in from the drum's +Y face (connector end last): open the bore there
        wrist = wrist.cut(m.moved(af.prism(ACTUATORS[4][0], ACT_CLEAR, -56.9, -64), ACT_PL[5]))
        # J6 actuator slides in from the flange end: straight slot of its outline out through
        # the front, closed by a bolt-on nose ring (front NOSE_T of the block)
        if CABLES:
            # thin cap closing the J5 loading opening (recessed in the drum's +Y face), carrying
            # the hollow hub that takes the flex out through the forearm wall to the J5 coil
            wrist = wrist.cut(ycyl(27.3, 1.2, V(0, 28.1, 0)))
            cap = ycyl(27.2, 0.95, V(0, 28.025, 0)).fuse(ycyl(6, FW - 2 + 1.6 - 28.4, V(0, (28.4 + FW - 0.4) / 2, 0))).cut(ycyl(4, FW - 20, V(0, (28.4 + FW - 0.4) / 2, 0)))   # hub up to 1.6 past the pocket floor
            if J5_SUPPORT:
                # boss off the cap (inner-race shoulder) and the r7.5 seat for the support bearing's inner race
                y5 = J5_GAP_Y + 0.3
                cap = cap.fuse(ycyl(J5_BRG[0] + 2.5, y5 - 28.5, V(0, (28.5 + y5) / 2, 0))).fuse(ycyl(J5_BRG[0], J5_BRG[2], V(0, y5 + J5_BRG[2] / 2, 0)))
                cap = cap.cut(ycyl(4, 30, V(0, 30, 0)))
                add_shape(doc, l5, "J5_support_inner", ycyl(sum(J5_BRG[:2]) / 2 - 0.15, J5_BRG[2], V(0, y5 + J5_BRG[2] / 2, 0)).cut(ycyl(J5_BRG[0], 10, V(0, y5 + J5_BRG[2] / 2, 0))), m.BEARING)
            add_shape(doc, l5, "J5_cap", cap.removeSplitter(), ARM_FINISH)
        if LIGHTEN:
            # pockets in the drum ring from its -Y face (hidden: it faces the forearm's far wall):
            # 3 mm outer skin, 2 mm round the J5 actuator, solid where the block joins (x > 8), 6 ribs
            sil = m.moved(af.silhouette(ACTUATORS[4][0], ACT_CLEAR + 2), ACT_PL[5]).translated(V(0, J5_GAP_Y - WRIST_Y, 0))
            reg = Part.Face(Part.Wire(Part.makeCircle(WRIST_R - 3, V(0, -WRIST_Y, 0), Y))).cut(sil)
            reg = reg.cut(Part.Face(Part.makePolygon([V(8, -WRIST_Y, -60), V(60, -WRIST_Y, -60), V(60, -WRIST_Y, 60), V(8, -WRIST_Y, 60), V(8, -WRIST_Y, -60)])))
            for i in range(6):
                a = math.radians(30 + 60 * i)
                d_, n_ = V(math.cos(a), 0, math.sin(a)), V(-math.sin(a), 0, math.cos(a))
                reg = reg.cut(Part.Face(Part.makePolygon([V(0, -WRIST_Y, 0) + n_ * s1 * 1.5 + d_ * s2 * 40 for s1, s2 in ((-1, 0), (1, 0), (1, 1), (-1, 1), (-1, 0))])))
            pk = [f.extrude(V(0, 24, 0)) for f in reg.Faces if f.Area > 20]
            if pk:
                wrist = checked_cut(wrist, soft(Part.makeCompound(pk).translated(V(0, -0.01, 0)), 1.5) if False else Part.makeCompound(pk).translated(V(0, -0.01, 0)), "wrist drum pockets")
        nosebox = box(20, 100, 100, V(WRIST - NOSE_T, -50, -50))
        add_shape(doc, l5, "Wrist_nose", wrist.common(nosebox), ARM_FINISH)
        wrist = wrist.cut(nosebox).cut(m.moved(af.prism(ACTUATORS[5][0], ACT_CLEAR, -49.5, 10), ACT_PL[6]))
    add_shape(doc, l5, "Wrist", wrist, ARM_FINISH)

    # --- link 6: stepped tool flange ----------------------------------------------------------
    flange = fuse(cyl(12, 6, V(-6, 0, 0), X), cyl(16.5, 6, V(0, 0, 0), X),
                  cyl(15, 1.5, V(6, 0, 0), X), cyl(16, 3.5, V(7.5, 0, 0), X))
    flange = flange.cut(cyl(4, 20, V(-7, 0, 0), X))
    for i in range(4):
        a = math.radians(45 + 90 * i)
        flange = flange.cut(cyl(1.6, 8, V(4, 10 * math.cos(a), 10 * math.sin(a)), X))
    add_shape(doc, l6, "Flange", flange, STAINLESS)

    if CABLES and USE_ACTUATORS:
        import cabling
        importlib.reload(cabling)
        cabling.apply(doc, log)
    if SPLIT_UPPER:
        split_upper_arm(doc, l2)
    doc.recompute()
    if App.GuiUp:
        import FreeCADGui as Gui
        Gui.ActiveDocument.ActiveView.viewIsometric()
        Gui.SendMsgToActiveView("ViewFit")
        m.refresh_panel(doc_name)
    return doc


def corner_tool_y(p, t1, t2, c, y_len, kind="chamfer"):
    """Material to remove at a convex corner along Y: p = corner point (at the start y),
    t1/t2 = unit in-face directions away from the edge (in XZ), y_len = signed extrusion.
    chamfer: triangle with legs c. fillet: the region between the corner and a radius-c arc
    tangent to both faces (any corner angle)."""
    if kind == "fillet":
        th = math.acos(max(-1.0, min(1.0, t1.dot(t2))))
        a1, a2 = p + t1 * (c / math.tan(th / 2)), p + t2 * (c / math.tan(th / 2))
        bis = (t1 + t2); bis.normalize()
        ctr = p + bis * (c / math.sin(th / 2))
        quad = Part.Face(Part.makePolygon([p, a1, ctr, a2, p])).extrude(V(0, y_len, 0))
        disk = Part.Face(Part.Wire(Part.makeCircle(c, ctr, V(0, 1, 0)))).extrude(V(0, y_len, 0))
        return quad.cut(disk)
    return Part.Face(Part.makePolygon([p, p + t1 * c, p + t2 * c, p])).extrude(V(0, y_len, 0))


def prism_chamfer_y(shape, x, z, c, side, label, kind="chamfer"):
    """Chamfer (legs c) or fillet (radius c) the straight Y-direction edge(s) at (x, z) on the
    y-`side` of `shape` by cutting a profile along Y (makeChamfer fails on some fork-plate
    corners). Checks the removed volume; returns the shape unchanged (logged) if the edge is
    missing or the cut is bad."""
    edges = [e for e in shape.Edges if e.BoundBox.XLength < 0.01 and e.BoundBox.ZLength < 0.01
             and e.BoundBox.YLength > 0.5 and abs(e.BoundBox.XMin - x) < 0.05 and abs(e.BoundBox.ZMin - z) < 0.05
             and side * e.BoundBox.Center.y > 0]
    if not edges:
        log("%s %s: edge at x %.2f z %.2f not found, left sharp" % (label, kind, x, z))
        return shape
    out = shape
    for e in edges:
        q = e.valueAt(0.5 * (e.FirstParameter + e.LastParameter))
        dirs = []
        for f in shape.ancestorsOfType(e, Part.Face):
            n = f.normalAt(*f.Surface.parameter(q))
            t = V(-n.z, 0, n.x)                             # in-face, perpendicular to the Y edge
            t.normalize()
            if f.distToShape(Part.Vertex(q + t * 0.5))[0] > f.distToShape(Part.Vertex(q - t * 0.5))[0]:
                t = -t
            dirs.append(t)
        if len(dirs) != 2:
            log("%s %s: %d faces on the edge, left sharp" % (label, kind, len(dirs)))
            continue
        b = e.BoundBox
        y0, y1 = (b.YMin, b.YMax + 1) if side > 0 else (b.YMax, b.YMin - 1)   # +1 clears the outer-face outline chamfer
        tool = corner_tool_y(V(b.XMin, y0, b.ZMin), dirs[0], dirs[1], c, y1 - y0, kind)
        r = out.cut(tool)
        dv = out.Volume - r.Volume
        if r.isValid() and len(r.Solids) == len(out.Solids) and 0 < dv <= tool.Volume + 0.1:
            log("%s %s %g: removed %.1f mm3" % (label, kind, c, dv))
            out = r
        else:
            log("%s %s %g rejected (valid %s, dV %.1f)" % (label, kind, c, r.isValid(), dv))
    return out


def concave_corner_fill(shape, side, c, label, box=(-10, 40, 45, 110)):
    """Chamfer (legs c) the concave edges where the fork plate's inner face (y = side*PLATE_IN)
    meets the crotch floor, inside the XZ box (xmin, xmax, zmin, zmax), by fusing a filler wedge
    lofted along each edge (makeChamfer refuses these). Checks the added volume."""
    walls = [f for f in shape.Faces if f.Surface.__class__.__name__ == "Plane"
             and abs(f.BoundBox.YMin - side * PLATE_IN) < 0.01 and abs(f.BoundBox.YMax - side * PLATE_IN) < 0.01]
    nw = V(0, -side, 0)                                   # wall normal, into the crotch
    wedges, length = [], 0.0
    for w in walls:
        for e in w.Edges:
            b = e.BoundBox
            if not (b.XMin > box[0] and b.XMax < box[1] and b.ZMin > box[2] and b.ZMax < box[3]):
                continue
            others = [f for f in shape.ancestorsOfType(e, Part.Face) if not f.isSame(w)]
            if not others:
                continue
            o = others[0]
            q = e.valueAt(0.5 * (e.FirstParameter + e.LastParameter))
            if not shape.isInside(q + nw * 0.3 - o.normalAt(*o.Surface.parameter(q)) * 0.3, 1e-4, False):
                continue                                  # convex: not a crotch corner
            secs = []
            for p in e.discretize(Number=max(4, int(e.Length / 3) + 1)):      # smooth loft, sections ~3 mm apart
                no = o.normalAt(*o.Surface.parameter(p))
                no.y = 0
                no.normalize()
                a = p - nw * 0.05 - no * 0.05             # 0.05 into the metal so the fuse is clean
                secs.append(Part.makePolygon([a, p + no * c - nw * 0.05, p + nw * c - no * 0.05, a]))
            wedges.append(Part.makeLoft(secs, True, False))
            length += e.Length
    if not wedges:
        log("%s chamfer: no concave corner found, left sharp" % label)
        return shape
    r = shape.fuse(wedges)
    added = r.cut(shape).Volume
    if r.isValid() and len(r.Solids) == len(shape.Solids) and 0 < added <= 0.5 * c * c * length * 1.3:
        log("%s chamfer %g: %d edges, %.1f mm long, added %.1f mm3" % (label, c, len(wedges), length, added))
        return r
    log("%s chamfer %g rejected (valid %s, added %.1f)" % (label, c, r.isValid(), added))
    return shape


def fork_corners(upper):
    """FORK_CORNER_CH breaks on the fork plates (both sides), applied before the covers so that
    PLATE_XZ (the outline the covers, their pockets and the outer-face outline chamfer follow)
    gets the same corner (user 2026-11-14: keep cover + edge chamfer consistent with it)."""
    global PLATE_XZ
    for (cx, cz), c, kind in FORK_CORNER_CH:
        for side in (-1, 1):
            upper = prism_chamfer_y(upper, cx, cz, c, side, "fork corner %s" % "LR"[side > 0], kind)
        if PLATE_XZ is None:
            continue
        py = PLATE_XZ.BoundBox.YMin
        vs = [v for v in PLATE_XZ.Vertexes if math.hypot(v.X - cx, v.Z - cz) < 0.05]
        if not vs:
            log("fork corner: no PLATE_XZ vertex at (%.2f, %.2f); covers keep the sharp corner" % (cx, cz))
            continue
        v = vs[0]
        dirs = []
        for e in PLATE_XZ.OuterWire.Edges:
            for par, sgn in ((e.FirstParameter, 1), (e.LastParameter, -1)):
                if e.valueAt(par).distanceToPoint(v.Point) < 0.05:
                    t = e.tangentAt(par) * sgn
                    t.y = 0
                    t.normalize()
                    dirs.append(t)
        if len(dirs) != 2:
            log("fork corner: PLATE_XZ vertex has %d edges, left sharp" % len(dirs))
            continue
        tool = corner_tool_y(V(v.X, py - 1, v.Z), dirs[0], dirs[1], c, 2, kind)
        f = PLATE_XZ.cut(tool)
        if f.Faces and abs(f.Faces[0].Area - PLATE_XZ.Area) < tool.Volume:
            log("fork corner: PLATE_XZ %s %g, area %.1f -> %.1f mm2" % (kind, c, PLATE_XZ.Area, f.Faces[0].Area))
            PLATE_XZ = f.Faces[0]
        else:
            log("fork corner: PLATE_XZ cut failed, covers keep the sharp corner")
    return upper


def split_upper_arm(doc, l2):
    """Split UpperArm on y = 0 into UpperArm_L (-Y) and UpperArm_R (+Y), with M3 socket cap
    screws from the -Y side (counterbored, clearance in L, tapped in R) and two dowels."""
    ua = doc.getObject("UpperArm")
    shape = ua.Shape
    tools, heads = [], []
    for pts, face in ((UA_SPLIT_DRUM, -J2_BARREL_Y), (UA_SPLIT_COVER, -(PLATE_OUT - 2.5))):
        for x, z in pts:
            cb, seat = 3.3, face + 3.3
            t = ycyl(3.25, cb + 1, V(x, face + (cb - 1) / 2, z))            # counterbore (head 5.5 dia, 3 tall)
            t = t.fuse(ycyl(1.7, -seat + 0.02, V(x, seat / 2, z)))          # 3.4 clearance to the joint
            t = t.fuse(ycyl(1.25, 9, V(x, 4.5, z)))                         # 2.5 tap drill, 6 mm thread in the +Y half
            tools.append(t.removeSplitter())
            heads.append(V(x, seat, z))
    for x, z in UA_DOWELS:
        tools.append(ycyl(1.5, 10, V(x, 0, z)))
    # halve first (a plain common), then drill each half, no refine: on the FHA-11C hub arm
    # (2026-10-25) cutting the holes first, or refining, left unorientable halves
    tc = Part.makeCompound(tools)
    neg = cut_pockets(shape.common(box(300, 150, 400, V(-150, -150, -100))), tc, "upper arm split holes L", pre_refine=False)
    pos = cut_pockets(shape.common(box(300, 150, 400, V(-150, 0, -100))), tc, "upper arm split holes R", pre_refine=False)
    # 2026-11-13: merge the coplanar faces the CROTCH_HUB_ARC tool outline leaves on the fork
    # plates' inner faces (stray seam at x -5.5 / z 120). removeSplitter mutated its source in an
    # earlier try (both halves went invalid), so refine a copy and keep the original on failure.
    for nm in ("neg", "pos"):
        h = locals()[nm]
        r = refine(h.copy())
        if r is not h and h.isValid() and r.isValid():
            log("upper arm %s refined: %d -> %d faces, dV %.1f mm3" % (nm, len(h.Faces), len(r.Faces), r.Volume - h.Volume))
            if nm == "neg":
                neg = r
            else:
                pos = r
    # 2026-11-13 (user): 1 mm round on the outline of each fork plate's outer face (y = +-PLATE_OUT),
    # done on the finished halves (the outline only exists after the strap/back-line cuts)
    if PLATE_OUTLINE_CH:
        def _round_outline(h, yy):
            fs = [g for g in h.Faces if g.Surface.__class__.__name__ == "Plane" and abs(g.BoundBox.YMin - yy) < 0.01
                  and abs(g.BoundBox.YMax - yy) < 0.01]
            if not fs:
                return h
            try:
                r = h.makeChamfer(PLATE_OUTLINE_CH, max(fs, key=lambda g: g.Area).OuterWire.Edges)
                if r.isValid() and len(r.Solids) == len(h.Solids):
                    log("plate outline chamfer y %+.1f: dV %.1f mm3" % (yy, r.Volume - h.Volume))
                    return r
            except Exception as ex:
                log("plate outline chamfer y %+.1f failed: %s" % (yy, ex))
            return h
        neg = _round_outline(neg, -PLATE_OUT)
        pos = _round_outline(pos, PLATE_OUT)
    if CROTCH_CORNER_CH:
        neg = concave_corner_fill(neg, -1, CROTCH_CORNER_CH, "crotch corner L")
        pos = concave_corner_fill(pos, +1, CROTCH_CORNER_CH, "crotch corner R")
    log("upper arm halves valid: L %s R %s" % (neg.isValid(), pos.isValid()))
    doc.removeObject("UpperArm")
    add_shape(doc, l2, "UpperArm_L", neg, ARM_FINISH)
    add_shape(doc, l2, "UpperArm_R", pos, ARM_FINISH)
    screws(doc, l2, "Split_screws", heads, -Y)
    add_shape(doc, l2, "Split_dowels", Part.makeCompound([ycyl(1.5, 10, V(x, 0, z)) for x, z in UA_DOWELS]), m.STEEL)
    log("upper arm split: L %.0f R %.0f mm3, %d screws, %d dowels" % (neg.Volume, pos.Volume, len(heads), len(UA_DOWELS)))


def _settle(v):
    """FreeCAD animates camera moves; turn that off and let the view update,
    or a render taken straight after a view change shows the old angle."""
    from PySide import QtCore
    try:
        v.setAnimationEnabled(False)
    except Exception:
        pass
    for _ in range(5):
        QtCore.QCoreApplication.processEvents()


def set_view(az_deg, el_deg, doc_name=DOC, ortho=True):
    import FreeCADGui as Gui
    v = Gui.getDocument(doc_name).ActiveView
    _settle(v)
    v.setCameraType("Orthographic" if ortho else "Perspective")
    a, e = math.radians(az_deg), math.radians(el_deg)
    zc = V(math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))
    xc = Z.cross(zc).normalize()
    v.setCameraOrientation(App.Rotation(xc, zc.cross(xc), zc, "ZXY"))
    v.fitAll()
    _settle(v)
    return v


def photo_views(prefix=None, doc_name=DOC):
    """Pose as in the photos and (optionally) save renders named after the
    photo labels. FreeCAD's standard views: photo FRONT (camera ahead, +X) =
    viewRight, BACK = viewLeft, RIGHT SIDE (camera at +Y) = viewRear,
    LEFT SIDE = viewFront."""
    import FreeCADGui as Gui
    m.pose(*PHOTO_POSE, doc_name=doc_name)
    v = Gui.getDocument(doc_name).ActiveView
    v.setCameraType("Orthographic")
    shots = {"front": v.viewRight, "back": v.viewLeft, "right": v.viewRear, "left": v.viewFront}
    _settle(v)
    for name, fn in shots.items():
        fn()
        v.fitAll()
        _settle(v)
        if prefix:
            v.saveImage("%s_%s.png" % (prefix, name), 700, 760, "White")
    v.viewIsometric()
    v.fitAll()
    return list(shots)

def kinematics(doc_name=DOC):
    """IK/FK solver for this arm, using the joint limits in the document's sheet."""
    from klik3_ik import KLiK3Kinematics
    lims = list(LIMITS)
    tool = FLANGE_FACE
    if doc_name in App.listDocuments():
        doc = App.getDocument(doc_name)
        sh = doc.getObject("Joints")
        try:
            lims = [(float(sh.get("C%d" % r)), float(sh.get("D%d" % r))) for r in range(2, 8)]
        except Exception:
            pass
        fl = doc.getObject("Flange")
        if fl is not None and fl.Shape.Volume > 0:
            tool = fl.Shape.BoundBox.XMax           # mounting face, in the Link6 (J6) frame
    # reports the flange mounting face (as RobotVisualisation does), not the J6 origin
    return KLiK3Kinematics(J2Z + WORLD_Z, L2, J4X, J4Z, J5X, WRIST, lims, J2X=J2X, TOOL=tool)   # world frame


def show_panel(doc_name=DOC):
    """Joint sliders (shared panel) plus a Cartesian IK section: flange X/Y/Z and
    tool-frame mobile XYZ Euler angles, Meca500 convention, at the flange mounting face
    (zero joints = 201, 0, 334, 0, 90, 0, as in RobotVisualisation)."""
    from PySide import QtWidgets
    dock = m.show_panel(doc_name)
    grid = dock.widget().layout()
    joints = [grid.itemAtPosition(i, 2).widget() for i in range(6)]
    sliders = [grid.itemAtPosition(i, 1).widget() for i in range(6)]
    for sp in joints:
        sp.setDecimals(3)                   # 0.1 deg rounding would put the flange ~0.1 mm off target
    kin = kinematics(doc_name)
    real_limits = list(kin.limits)
    free = dock.findChild(QtWidgets.QCheckBox, "FreeLimits")
    if free is not None:                    # IK follows the panel's "Free" box: any angle within +-180 (J6 +-360)
        free.toggled.connect(lambda on: setattr(kin, "limits", [(-180.0, 180.0)] * 5 + [(-360.0, 360.0)] if on else real_limits))

    box_ = QtWidgets.QGroupBox("Cartesian (IK) - flange in base frame")
    g = QtWidgets.QGridLayout(box_)
    cart = []
    for i, (name, unit, lo, hi) in enumerate((("X", " mm", -500, 500), ("Y", " mm", -500, 500), ("Z", " mm", -200, 600),
                                              ("α", " °", -180, 180), ("β", " °", -180, 180), ("γ", " °", -180, 180))):
        sp = QtWidgets.QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setDecimals(3)                   # 2 dp angle rounding lets the flange creep ~2 µm per jog
        sp.setSuffix(unit)
        sp.setKeyboardTracking(False)       # solve on Enter / arrows, not every keystroke
        if i >= 3:
            sp.setWrapping(True)
        g.addWidget(QtWidgets.QLabel(name), i % 3, 2 * (i // 3))
        g.addWidget(sp, i % 3, 2 * (i // 3) + 1)
        cart.append(sp)
    step = QtWidgets.QComboBox()
    step.addItems(["0.1", "1", "5", "10"])
    step.setCurrentIndex(1)
    frame = QtWidgets.QComboBox()
    frame.addItems(["Base", "Tool"])
    frame.setToolTip("Frame for the jogs: base axes, or the tool's own axes (tool Z = flange normal).\n"
                     "Rotations always pivot about the flange centre.")
    status = QtWidgets.QLabel()
    sagbox = QtWidgets.QCheckBox("Gravity-sag compensation, payload")
    sagbox.setToolTip("Correct the joint command for gear wind-up and output-bearing tilt under gravity\n"
                      "(klik3_ik.SagModel: link masses from klik3_mass.json, Harmonic Drive stiffness).\n"
                      "Pose boxes then show where the flange really is, sag included.")
    payload = QtWidgets.QDoubleSpinBox()
    payload.setRange(0, 1.0)
    payload.setSingleStep(0.1)
    payload.setValue(0.5)
    payload.setSuffix(" kg")
    g.addWidget(sagbox, 6, 0, 1, 3)
    g.addWidget(payload, 6, 3)
    sag_model = {}

    def sag():
        if not sagbox.isChecked():
            return None
        from klik3_ik import SagModel
        key = round(payload.value(), 3)
        if key not in sag_model:
            sag_model.clear()
            sag_model[key] = SagModel(kin, payload_kg=key)
        return sag_model[key]
    g.addWidget(QtWidgets.QLabel("Step"), 3, 0)
    g.addWidget(step, 3, 1)
    g.addWidget(QtWidgets.QLabel("Jog"), 3, 2)
    g.addWidget(frame, 3, 3)
    g.addWidget(status, 5, 0, 1, 4)

    def set_step(t):
        for sp in cart:
            sp.setSingleStep(float(t))
    step.currentTextChanged.connect(set_step)
    set_step(step.currentText())

    def lin_jog(axis, sign):
        # Move the flange one step along a base axis, or along the tool's axis (a column of R_tool).
        d = sign * float(step.currentText())
        if frame.currentText() == "Base":
            cart[axis].setValue(cart[axis].value() + d)
            return
        from klik3_ik import euler_to_matrix
        R = euler_to_matrix(*[sp.value() for sp in cart[3:]])
        for k, sp in enumerate(cart[:3]):
            sp.blockSignals(True)
            sp.setValue(sp.value() + d * R[k][axis])
            sp.blockSignals(False)
        solve()

    # Jog buttons: one step per click; hold to repeat.
    jog = QtWidgets.QGridLayout()
    for i, name in enumerate("XYZ"):
        for col, sign in ((0, -1), (1, 1)):
            b = QtWidgets.QPushButton("%s%s" % (name, "−" if sign < 0 else "+"))
            b.setAutoRepeat(True)
            b.setAutoRepeatDelay(400)
            b.setAutoRepeatInterval(80)
            b.clicked.connect(lambda _=False, a=i, s=sign: lin_jog(a, s))
            jog.addWidget(b, i, col)

    def rot_jog(axis, sign):
        # Turn the tool about a base axis (pre-multiply) or its own axis (post-multiply),
        # pivoting on the flange centre. Euler-angle steps would be degenerate at
        # b = +-90, which is where the arm sits at zero joints.
        from klik3_ik import euler_to_matrix, matrix_to_euler, _mm, _rx, _ry, _rz
        d = math.radians(sign * float(step.currentText()))
        R0, Rd = euler_to_matrix(*[sp.value() for sp in cart[3:]]), (_rx, _ry, _rz)[axis](d)
        R = _mm(Rd, R0) if frame.currentText() == "Base" else _mm(R0, Rd)
        for sp, v in zip(cart[3:], matrix_to_euler(R, a_hint=cart[3].value())):
            sp.blockSignals(True)
            sp.setValue(v)
            sp.blockSignals(False)
        solve()

    for i, name in enumerate(("Rx", "Ry", "Rz")):
        for col, sign in ((2, -1), (3, 1)):
            b = QtWidgets.QPushButton("%s%s" % (name, "−" if sign < 0 else "+"))
            b.setToolTip("Rotate about the %s axis (Jog frame) by one step in degrees, pivoting on the flange centre" % name[1].upper())
            b.setAutoRepeat(True)
            b.setAutoRepeatDelay(400)
            b.setAutoRepeatInterval(80)
            b.clicked.connect(lambda _=False, a=i, s=sign: rot_jog(a, s))
            jog.addWidget(b, i, col)
    g.addLayout(jog, 4, 0, 1, 4)

    def show_fk(*_):
        q = [sp.value() for sp in joints]
        pose = kin.fk(q, a_hint=cart[3].value())
        sm = sag()
        if sm is not None:                  # where the sagging flange really is
            from klik3_ik import _mm, _rotvec_exp, matrix_to_euler, RY90
            p, R = kin.flange(q)
            dp, dw = sm.predict(q)
            pose = tuple(a + b for a, b in zip(p, dp)) + matrix_to_euler(_mm(_mm(_rotvec_exp(dw), R), RY90), cart[3].value())
        for sp, v in zip(cart, pose):
            sp.blockSignals(True)
            sp.setValue(v)
            sp.blockSignals(False)
        status.setStyleSheet("")
        status.setText("")

    def solve(*_):
        q0 = [sp.value() for sp in joints]
        sm = sag()
        if sm is not None:
            q, ep, ew = sm.compensated_ik([sp.value() for sp in cart], q0)
        else:
            q = kin.ik([sp.value() for sp in cart], q0)
        if q is None:
            show_fk()                       # boxes back to where the arm really is
            status.setStyleSheet("color: #c00")
            status.setText("unreachable / outside limits")
            return
        for sp, sl, v in zip(joints, sliders, q):
            for w in (sp, sl):
                w.blockSignals(True)
            sp.setValue(v)
            sl.setValue(round(v))
            for w in (sp, sl):
                w.blockSignals(False)
        status.setStyleSheet("color: #080")
        if sm is not None:
            dp, dw = sm.predict(q)
            status.setText("ok, sag-compensated (sag here %.2f mm, %.2f mrad)" % (math.sqrt(sum(v * v for v in dp)), 1e3 * math.sqrt(sum(v * v for v in dw))))
        else:
            status.setText("ok")
        if doc_name in App.listDocuments():
            doc = App.getDocument(doc_name)
            sh = doc.getObject("Joints")
            for i, sp in enumerate(joints, 2):
                sh.set("B%d" % i, str(sp.value()))
            doc.recompute()

    sagbox.toggled.connect(show_fk)
    payload.valueChanged.connect(show_fk)
    for sp in joints:
        sp.valueChanged.connect(show_fk)
    for sp in cart:
        sp.valueChanged.connect(solve)
    grid.addWidget(box_, 8, 0, 1, 3)
    grid.setRowStretch(8, 0)
    grid.setRowStretch(9, 1)
    show_fk()
    return dock


if __name__ == "__main__":
    build()
    if App.GuiUp:
        photo_views()
        show_panel()
