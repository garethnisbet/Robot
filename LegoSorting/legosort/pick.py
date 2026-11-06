"""Choosing a brick to pick from a pile, and the pick-and-place motion (robot frame, mm).

The gripper closes along the tool x axis and only spans closed_gap..closed_gap+stroke,
so a brick can be gripped across a side whose size falls in that range, and only if that
side lies (nearly) horizontal. The tool points straight down, so the brick must be uncovered.
"""
import itertools
from dataclasses import dataclass

import numpy as np

HOME = [0.0, -20.0, 30.0, 0.0, 80.0, 0.0]      # arm up and back, clear of the mat


def corners(b, studs=False):
    """Brick box corners in the robot frame (studs adds the stud height on top)."""
    from .lego import STUD_H
    L, W, H = b.length, b.width, b.height + (STUD_H if studs else 0)
    c = np.array([[x, y, z] for x in (-L / 2, L / 2) for y in (-W / 2, W / 2) for z in (0, H)])
    T = b.T
    return c @ T[:3, :3].T + T[:3, 3]


def sample_points(b, step=2.0):
    """Points filling the brick's box (studs included), robot frame."""
    from .lego import STUD_H
    L, W, H = b.length, b.width, b.height + STUD_H
    g = [np.linspace(-a / 2, a / 2, max(2, int(np.ceil(a / step)) + 1)) for a in (L, W)]
    z = np.linspace(0, H, max(2, int(np.ceil(H / step)) + 1))
    p = np.stack(np.meshgrid(g[0], g[1], z, indexing="ij"), -1).reshape(-1, 3)
    T = b.T
    return p @ T[:3, :3].T + T[:3, 3]


def body_footprint(gripper, margin=1.5):
    """Half sizes (tool x, tool y) of the gripper body seen from below."""
    v = gripper.collision_static()[0]
    return np.abs(v[:, :2]).max(axis=0) + margin


@dataclass(eq=False)
class Grasp:
    index: int                 # position in the brick list
    centre: np.ndarray         # grasp point (TCP target), mm
    R: np.ndarray              # grip frame: x closing, z along the jaws (down)
    width: float               # brick size across the fingers, mm
    top: float                 # top of the brick, mm
    below: float = 0.0         # how far the brick reaches below the TCP, mm
    approach: np.ndarray = None   # side grips: horizontal direction the jaws come in along
    radius: float = 0.0           # side grips: distance from the pile centre
    step: float = 0.0             # side grips: open gap of the finger step used, mm


def grasps(bricks, gripper, margin=0.6, max_tilt=12.0, depth=6.0, clear=2.0):
    """Grippable bricks, highest first.

    margin: keep the brick width this far inside the finger range.
    max_tilt: deg, how far the gripped side may be off horizontal.
    depth: how far below the brick's top the fingertips go, but at least down to 1 mm below
        its centre (a tilted brick's top is a far corner), and never below 1.5 mm above the mat.
    clear: another brick reaching within this of the top, over the footprint, covers it.
    """
    lo, hi = gripper.closed_gap + margin, gripper.closed_gap + gripper.stroke - margin
    out = []
    boxes = [corners(b, studs=True) for b in bricks]
    clouds = np.vstack([sample_points(b) for b in bricks])
    half = body_footprint(gripper)
    for i, b in enumerate(bricks):
        T = b.T
        for axis, size in ((0, b.length), (1, b.width)):
            if not lo <= size <= hi:
                continue
            a = T[:3, axis]
            if abs(np.degrees(np.arcsin(np.clip(abs(a[2]), 0, 1)))) > max_tilt:
                continue
            c = T[:3, :3] @ [0, 0, b.height / 2] + T[:3, 3]
            top = boxes[i][:, 2].max()
            covered = False
            r = 0.5 * np.hypot(b.length, b.width)
            for j, other in enumerate(boxes):
                if j == i:
                    continue
                near = np.hypot(*(other[:, :2] - c[:2]).T) < r + 4
                if near.any() and other[near, 2].max() > top + clear and other[:, 2].min() > c[2] - 2:
                    covered = True
                    break
            if covered:
                continue
            x = np.array([a[0], a[1], 0.0])
            x /= np.linalg.norm(x)
            if x[0] > 0:                   # J6 near zero has tool x along -robot x
                x = -x
            z = np.array([0, 0, -1.0])
            R = np.column_stack([x, np.cross(z, x), z])
            # the jaws stand only jaw_length proud of the gripper body, so the body must stay
            # above everything under it (this brick and its neighbours)
            local = (clouds - [c[0], c[1], 0])[:, :2] @ R[:2, :2]
            under = clouds[(np.abs(local) <= half).all(axis=1), 2]
            floor = (under.max() if len(under) else top) - (gripper.jaw_length - 0.6)
            tip = max(min(top - depth, c[2] - 1.0), floor, 1.5)
            if tip > c[2] + 1.5:          # can't reach down to the brick's middle
                continue
            out.append(Grasp(i, np.array([c[0], c[1], tip]), R, size, top, tip - boxes[i][:, 2].min()))
    out.sort(key=lambda g: -g.top)
    return out


def orient(sim, grasp, tilts=(0, 10, -10, 20, -20, 30, -30, 40, -40, 50, -50), min_clear=3.0,
           approach=40.0):
    """Tilt the jaws about the closing axis (deg, smallest first) until the arm and gripper body
    clear the mat by min_clear mm at the grasp, with IK at the grasp and above it.

    With the gripper side-mounted the flange axis is level when the jaws point straight
    down, and the wrist then hangs about as low as the jaw tips; tilting lifts it.
    Returns a new Grasp, or None.
    """
    from .geometry import rot_x
    above = grasp.centre + [0, 0, approach + grasp.top - grasp.centre[2]]
    for t in tilts:
        R = grasp.R @ rot_x(t)
        try:
            q = sim.ik(grasp.centre, R)
            sim.ik(above, R, q)
        except ValueError:
            continue
        if sim.clearance(q) >= min_clear:
            return Grasp(grasp.index, grasp.centre, R, grasp.width, grasp.top, grasp.below)
    return None


def drop_orientation(sim, R, p, yaws=(0, 90, -90, 180, 45, -45, 135, -135), min_clear=3.0):
    """Grip-frame orientation for setting a brick down at p: R turned about the vertical
    (the brick's yaw on the drop spot doesn't matter) until IK works and the arm clears the mat."""
    from .geometry import rot_z
    for yaw in yaws:
        Ry = rot_z(yaw) @ R
        try:
            q = sim.ik(p, Ry)
            sim.ik(np.asarray(p) + [0, 0, 15], Ry, q)
        except ValueError:
            continue
        if sim.clearance(q) >= min_clear:
            return Ry
    raise ValueError(f"can't reach the drop spot {np.round(p, 1)}")


WRIST_R = 24.0      # mm, radius of the arm's wrist above the flange, for path checks
WRIST_LEN = 60.0    # mm of wrist above the flange that can sweep near the pile


def gripper_shapes(gripper):
    """Convex pieces (vertex arrays, grip frame) that sweep through the pile on a side grip:
    the gripper body, cap, connector, spacer, both jaws (open) and the wrist above the flange."""
    if "grip_shapes" in gripper._cache:
        return gripper._cache["grip_shapes"]
    Tg = np.linalg.inv(gripper.T_tool_grip)
    tool = list(gripper.collision_static())
    for side, sgn in (("a", 1), ("b", -1)):
        for piece in gripper.collision_jaw_parts(side):
            tool.append(piece + sgn * gripper.axis * gripper.stroke / 2)
    t = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    ring = np.column_stack([WRIST_R * np.cos(t), WRIST_R * np.sin(t), np.zeros_like(t)])
    tool.append(np.vstack([ring, ring - [0, 0, WRIST_LEN]]))       # tool z points flange -> gripper
    out = [h @ Tg[:3, :3].T + Tg[:3, 3] for h in tool]
    gripper._cache["grip_shapes"] = out
    return out


def _inside_hull(pts, hull_pts, margin):
    from scipy.spatial import ConvexHull
    lo, hi = hull_pts.min(axis=0) - margin, hull_pts.max(axis=0) + margin
    pts = pts[((pts >= lo) & (pts <= hi)).all(axis=1)]
    if not len(pts):
        return False
    eq = ConvexHull(hull_pts).equations
    return bool(((pts @ eq[:, :3].T + eq[:, 3]) <= margin).all(axis=1).any())


def path_clear(gripper, R, waypoints, points, margin=1.0):
    """True if the gripper (and wrist), held at orientation R (grip frame), can move in
    straight lines through ``waypoints`` (TCP, mm) without any of ``points`` coming within
    ``margin`` mm of it."""
    if not len(points):
        return True
    shapes = gripper_shapes(gripper)
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        for h in shapes:
            w = h @ R.T
            if _inside_hull(points, np.vstack([w + a, w + b]), margin):
                return False
    return True


def side_grasps(bricks, gripper, pile_centre, margin=0.6, max_tilt=12.0, body_clear=1.0,
                travel_z=None, back=20.0, min_contact=4.0, path_margin=1.0, nudge=None, only=None):
    """Side grips, outermost first: the jaws point horizontally, inward from outside the pile.

    For a side-mounted gripper this keeps the wrist above the gripper (flange axis vertical,
    grip y pointing down), so the arm never reaches down beside the jaws. The closing axis
    is horizontal, across a brick side (width, length, or height over the studs) that fits
    one of the gripper's ``steps``: bare jaws 11.4-17.0 mm (2-wide bricks, 1x2 lengthways);
    with the stepped fingers also 6.6-12.2 mm deep in (1-wide bricks, any brick across its
    height). On the wide (tip) step the jaws reach up to the step's depth into the brick;
    on the narrow step the brick goes in as far as the pads allow. The TCP sits at the
    middle height of the gripped part, but high enough for the gripper body to clear the mat.

    A grip is kept only if the gripper's path is free of other bricks (``path_clear``, the
    real gripper shape plus the wrist): down from ``travel_z`` to ``back`` mm outside the
    brick, in to the grip, and back up to ``travel_z``. The jaws (``jaw_up``/``jaw_down``
    about the TCP) must overlap the brick by at least ``min_contact`` mm of height.

    ``nudge`` {i: neighbours} (for a clump of touching bricks): brick i's grip may push those
    neighbours aside as the jaws slide in and lift, but the gripper must still come down on
    clear mat. ``only``: just these brick indices.
    Returns Grasps with R = the grip frame.
    """
    from .lego import STUD_H
    Tg = np.linalg.inv(gripper.T_tool_grip)
    parts = [h @ Tg[:3, :3].T + Tg[:3, 3] for h in gripper.collision_static()]   # grip frame
    below = max(p[:, 1].max() for p in parts)           # lowest part below the TCP (grip y points down)
    clouds = [sample_points(b) for b in bricks]
    boxes = [corners(b, studs=False) for b in bricks]
    out = []
    for i, b in enumerate(bricks):
        if only is not None and i not in only:
            continue
        T = b.T
        others = np.vstack([clouds[j] for j in range(len(bricks)) if j != i] or [np.zeros((0, 3))])
        pushed = set(nudge.get(i, ())) if nudge else set()
        rest = (np.vstack([clouds[j] for j in range(len(bricks)) if j != i and j not in pushed]
                          or [np.zeros((0, 3))]) if pushed else others)
        # across the width, the length, or the height (over the studs: they touch the jaw)
        sides = ((0, b.length), (1, b.width), (2, b.height + STUD_H))
        for (axis, size), (gap, d_lo, d_hi) in itertools.product(sides, gripper.steps):
            if not gap - gripper.stroke + margin <= size <= gap - margin:
                continue
            a = T[:3, axis]
            if np.degrees(np.arcsin(min(1.0, abs(a[2])))) > max_tilt:
                continue
            x = np.array([a[0], a[1], 0.0]) / np.hypot(a[0], a[1])
            c = boxes[i].mean(axis=0)
            for sgn in (1, -1):
                d = sgn * np.cross([0, 0, 1.0], x)       # approach direction (horizontal)
                y = np.array([0, 0, -1.0])
                R = np.column_stack([np.cross(y, d), y, d])          # x = y cross z keeps it right-handed
                proj = (boxes[i] - c) @ d
                length = proj.max() - proj.min()
                # how deep the brick's near face goes past the tips: as deep as the step allows
                # (a short brick stops 1 mm short of the tips passing its far face, on the wide step)
                reach = d_hi - 0.5 if d_lo > 0 else min(d_hi - 1.0, length - 1.0)
                if reach - max(reach - length, d_lo) < 3.0:        # too little of it on this step
                    continue
                tcp = c + d * (proj.min() + reach)
                own = clouds[i]
                along = (own - c) @ d
                held = own[(along >= proj.min() - 0.5) & (along <= proj.min() + reach + 0.5)]
                zlo, zhi = held[:, 2].min(), held[:, 2].max()
                z = max((zlo + zhi) / 2, below + body_clear)
                if min(zhi, z + gripper.jaw_up) - max(zlo, z - gripper.jaw_down) < min_contact:
                    continue                             # too little jaw on the brick
                tcp[2] = z
                hi_z = max(travel_z if travel_z is not None else 0.0, z + 15.0)
                out_p = tcp - d * back
                path = [np.r_[out_p[:2], hi_z], out_p, tcp, np.r_[tcp[:2], hi_z]]
                if not path_clear(gripper, R, path[:2], others, path_margin):
                    continue                             # coming down: clear of everything
                if not path_clear(gripper, R, path[1:], rest, path_margin):
                    continue
                g = Grasp(i, tcp, R, size, boxes[i][:, 2].max(), z - boxes[i][:, 2].min())
                g.step = gap
                g.approach = d
                g.radius = float(np.hypot(*(c[:2] - pile_centre)))
                out.append(g)
    out.sort(key=lambda g: -g.radius)
    return out


def pick_side(sim, idx, grasp, drop_xy, travel_z, back=20.0, release=2.0, log=print,
              fast=150.0, lin=150.0, slow=50.0, free=True):
    """Side grip: come down outside the pile, slide in along the approach direction, close,
    lift to travel height and set the brick down at drop_xy. With ``free`` it goes on clear
    mat near drop_xy (``free_drop``), else on top of whatever is there (``release`` mm above
    it). Ends over the drop at travel height. Returns True if it was held."""
    g = sim.gripper
    d = grasp.approach
    out = grasp.centre - d * back
    sim.grip(g.stroke)
    sim.move_joints(sim.ik([out[0], out[1], max(travel_z, out[2])], grasp.R), speed=fast)
    sim.move_linear(out, speed=lin)
    sim.move_linear(grasp.centre, speed=slow)
    sim.grip(0.0)
    sim.move_linear(grasp.centre + [0, 0, 6], speed=slow)       # free it from what it sat on
    up = grasp.centre.copy()
    up[2] = max(travel_z, grasp.centre[2] + 15)
    sim.move_linear(up, speed=lin)
    ok = idx in sim.held()
    log(f"  {'holding' if ok else 'missed'} {sim.proto[idx].colour} {sim.proto[idx].nx}x{sim.proto[idx].ny} "
        f"(opening {sim.opening():.2f} mm)")
    spot = free_drop(sim, idx, grasp.R, drop_xy, travel_z, release) if ok and free else None
    if spot is not None:         # clear mat beside whatever is already on the spot
        drop_xy, drop_z, R = spot
        hi = max(travel_z, drop_z + 15)
    else:
        floor = 0.0
        for j, b in zip(sim.indices(), sim.bricks()):
            c = corners(b, studs=True)
            if j != idx and (np.hypot(*(c[:, :2] - drop_xy).T) < 25).any():
                floor = max(floor, c[:, 2].max())
        drop_z = max(floor + grasp.below + release, grasp.centre[2])
        hi = max(travel_z, drop_z + 15)
        R = drop_orientation(sim, grasp.R, [drop_xy[0], drop_xy[1], drop_z], min_clear=0.5)
        if ok:                       # put the brick's centre on the spot, not the jaw tips
            Rg, p = sim.tcp_pose()
            rel = Rg.T @ (corners(sim.bricks()[list(sim.indices()).index(idx)]).mean(axis=0) - p)
            shifted = np.asarray(drop_xy, float) - (R @ rel)[:2]
            try:
                q = sim.ik([shifted[0], shifted[1], drop_z], R)
                if sim.clearance(q) >= 0.5:
                    drop_xy = shifted
            except ValueError:
                pass
    sim.move_joints(sim.ik([drop_xy[0], drop_xy[1], hi], R), speed=fast)
    sim.move_linear([drop_xy[0], drop_xy[1], drop_z], speed=lin)
    sim.grip(g.stroke)
    sim.step(0.1)
    sim.move_linear([drop_xy[0], drop_xy[1], hi], speed=lin)
    return ok


def free_drop(sim, idx, R, spot_xy, travel_z, release=2.0, rings=(0.0, 9.0, 15.0, 21.0, 26.0), n_dir=12,
              yaws=(0, 90, -90, 180, 45, -45, 135, -135), passes=((3.0, 21.0), (1.5, 21.0), (3.0, 26.0))):
    """Where to set the held brick idx down on clear mat near spot_xy, so it is not dropped
    onto a heap it could slide off (out of the arm's reach, past the spot arc).

    Tries brick-centre positions on ``rings`` (mm) round the spot, nearest first and the
    robot's side of the spot before the far side; for each, grip orientations turned about
    the vertical by ``yaws``. Kept if the held brick ends ``clear`` mm from every other
    brick, the gripper comes straight down from travel height without touching one, and IK
    works; ``passes`` are the (clear, furthest ring) tried in turn. Returns (tcp_xy, tcp_z, R)
    or None.
    """
    Rg, p = sim.tcp_pose()
    i_list = list(sim.indices())
    held = sample_points(sim.bricks()[i_list.index(idx)])
    rel = (held - p) @ Rg                                  # grip frame
    rel_c = rel.mean(axis=0)
    others = [sample_points(b) for j, b in zip(i_list, sim.bricks()) if j != idx]
    others = np.vstack(others) if others else np.zeros((0, 3))
    spot = np.asarray(spot_xy, float)
    out_dir = spot / max(np.linalg.norm(spot), 1e-9)
    cands = [spot]
    for r in rings[1:]:
        for a in np.linspace(0, 2 * np.pi, n_dir, endpoint=False):
            cands.append(spot + r * np.array([np.cos(a), np.sin(a)]))
    cands.sort(key=lambda c: np.linalg.norm(c - spot) + 0.7 * max(0.0, (c - spot) @ out_dir))
    for clear, ring in passes:
        for c in cands:
            if np.linalg.norm(c - spot) > ring + 1e-6:
                continue
            r = _drop_at(sim, c, R, rel, rel_c, others, travel_z, release, yaws, clear)
            if r is not None:
                return r
    return None


def _drop_at(sim, c, R, rel, rel_c, others, travel_z, release, yaws, clear):
    """free_drop at one brick-centre position c: (tcp_xy, tcp_z, R) or None."""
    from .geometry import rot_z
    near = others[np.hypot(*(others[:, :2] - c).T) < 60] if len(others) else others
    for yaw in yaws:
        Ry = rot_z(yaw) @ R
        w = rel @ Ry.T                                 # held brick about the TCP, robot axes
        z = release - w[:, 2].min()                    # brick bottom ``release`` above the mat
        tcp = np.r_[c - (Ry @ rel_c)[:2], z]
        brick = w + tcp
        hi = np.r_[tcp[:2], max(travel_z, z + 15)]
        if len(near) and (_inside_hull(near, np.vstack([brick, brick + hi - tcp]), clear)
                          or not path_clear(sim.gripper, Ry, [hi, tcp], near, clear)):
            continue
        try:
            q = sim.ik(tcp, Ry)
            sim.ik(hi, Ry, q)
        except ValueError:
            continue
        if sim.clearance(q) >= 0.5:
            return tcp[:2], tcp[2], Ry
    return None


def pile_top(sim, centre, radius=90.0):
    """Highest point (mm, studs included) of the bricks within radius of the pile centre."""
    z = 0.0
    for b in sim.bricks():
        c = corners(b, studs=True)
        if (np.hypot(*(c[:, :2] - centre).T) < radius).any():
            z = max(z, c[:, 2].max())
    return z


def hover(sim, xy, z, tilts=(0, 10, -10, 20, -20, 30, -30), min_clear=3.0):
    """Joint angles with the jaws down (tilted if needed) and the TCP at (xy, z)."""
    from .geometry import rot_x
    zdn = np.array([0, 0, -1.0])
    x = np.array([-1.0, 0, 0])
    R0 = np.column_stack([x, np.cross(zdn, x), zdn])
    for t in tilts:
        try:
            q = sim.ik([xy[0], xy[1], z], R0 @ rot_x(t))
        except ValueError:
            continue
        if sim.clearance(q) >= min_clear:
            return q
    raise ValueError(f"can't hover at {np.round([xy[0], xy[1], z], 1)}")


def pick_and_place(sim, idx, grasp, drop_xy, travel_z, release=2.0, log=print):
    """Pick brick ``idx`` (simulator index) with ``grasp`` and set it down at drop_xy, letting go
    ``release`` mm above whatever is already there (a dropped brick held by its ends tumbles).

    The arm travels at ``travel_z`` (TCP, mm): set it from the pile's current height, so
    it never climbs higher than it needs to. It ends just above the drop spot, at travel
    height, ready to go straight to the next brick. Returns True if the brick was held
    after lifting. The arm pushes anything it meets.
    """
    g = sim.gripper
    above = np.array([grasp.centre[0], grasp.centre[1], max(travel_z, grasp.top + 10)])
    sim.grip(g.stroke)
    sim.move_joints(sim.ik(above, grasp.R), speed=90)
    sim.move_linear(grasp.centre, speed=30)
    sim.grip(0.0)
    sim.move_linear(above, speed=40)
    ok = idx in sim.held()
    log(f"  {'holding' if ok else 'missed'} {sim.proto[idx].colour} {sim.proto[idx].nx}x{sim.proto[idx].ny} "
        f"(opening {sim.opening():.2f} mm)")
    floor = 0.0
    for j, b in zip(sim.indices(), sim.bricks()):
        c = corners(b, studs=True)
        if j != idx and (np.hypot(*(c[:, :2] - drop_xy).T) < 25).any():
            floor = max(floor, c[:, 2].max())
    drop_z = floor + grasp.below + release
    up = max(travel_z, drop_z + 15)
    R = drop_orientation(sim, grasp.R, [drop_xy[0], drop_xy[1], drop_z])
    sim.move_joints(sim.ik([drop_xy[0], drop_xy[1], up], R), speed=90)
    sim.move_linear([drop_xy[0], drop_xy[1], drop_z], speed=30)
    sim.grip(g.stroke)
    sim.step(0.15)
    sim.move_linear([drop_xy[0], drop_xy[1], up], speed=60)
    return ok


# ------------------------------------------------------------------ pushing

def push_frame(direction):
    """Grip frame for pushing along a horizontal direction jaws first (jaws horizontal and
    pointing along it, wrist above, as for a side grip). With the jaws open, a brick up to
    17 mm across sits between them and is pushed by the flat end of the gripper body (1 to
    19 mm above the mat), the jaws keeping it in line; a wider one is pushed by the two
    jaw tips. (Closed jaws wedge on a low brick's top edge and pinch it into the mat.)"""
    z = np.array([direction[0], direction[1], 0.0])
    z /= np.linalg.norm(z)
    y = np.array([0, 0, -1.0])
    return np.column_stack([np.cross(y, z), y, z])


def plan_push(sim, bricks, i, goal_xy, travel_z, legs=(120.0, 60.0, 30.0),
              turns=(0, 20, -20, 40, -40, 60, -60, 80, -80), body_clear=1.0, gap=4.0, keep_off=8.0,
              path_margin=1.0, max_low=4.0, contact=False, spots=(), spot_clear=50.0,
              band=(125.0, 225.0), sector=105.0, ignore=(), best=False, outward_cost=0.0, why=None):
    """One straight push of brick i towards goal_xy, jaws first, steering round other bricks.

    The brick must sit low (lowest corner within ``max_low`` mm of the mat) so the gripper,
    held just above the mat, meets it. Tries the goal direction first, then turned by
    ``turns`` deg, each with leg lengths ``legs`` (capped at the distance left). A push is
    kept if the gripper's whole path (down, push, up) and the brick's swept footprint
    miss every other brick (except ``ignore``: e.g. the heap it is being pushed onto), the
    brick ends ``keep_off`` mm from the others, ``band`` mm from the robot axis and within
    ``sector`` deg of the robot's x axis, and IK works all along.

    contact=True (for a pile too dense for a clean push): the brick and the gripper may
    shove other bricks, but the gripper must still come down on clear mat, the leg is only
    the shortest of ``legs``, and the brick (and anything it shoves) must end
    ``spot_clear`` mm from every one of ``spots`` (the drop spots), so nothing is shoved
    onto a sorted heap.

    best=True: try every direction (``turns``) and return the push that leaves the brick
    furthest from all others, as (plan, gap_mm), or None; for spreading a pile. Each mm the
    brick ends further from the robot costs ``outward_cost`` mm of that gap (outward pushes
    are the easy ones to reach, so without it a pile drifts out of reach as it is spread).
    Returns (R, start_tcp, end_tcp, hi_z) or None.
    """
    why = {} if why is None else why            # reason -> count, for diagnosis

    def no(r):
        why[r] = why.get(r, 0) + 1

    b = bricks[i]
    box = corners(b, studs=True)
    if box[:, 2].min() > max_low:
        no("not low")
        return None
    below = max(h[:, 1].max() for h in gripper_shapes(sim.gripper)[:4])
    c = box.mean(axis=0)
    to_goal = np.asarray(goal_xy, float) - c[:2]
    dist = np.linalg.norm(to_goal)
    if dist < 1.0:
        no("at goal")
        return None
    clouds = [sample_points(o) for o in bricks]
    skip = set(ignore) | {i}
    others = np.vstack([clouds[j] for j in range(len(bricks)) if j not in skip] or [np.zeros((0, 3))])
    others_xy = [corners(o)[:, :2] for j, o in enumerate(bricks) if j not in skip]
    z = below + body_clear
    hi_z = max(travel_z, z + 15.0)
    base = np.arctan2(to_goal[1], to_goal[0])
    found = None
    for t in turns:
        a = base + np.radians(t)
        p = np.array([np.cos(a), np.sin(a), 0.0])
        R = push_frame(p)
        lo = ((box - c) @ p).min()                   # the brick's back face, from its centre
        across = np.ptp((box - c) @ R[:, 0])
        jaw = sim.gripper.push_depth(across)         # where it meets the gripper: body end, step or tips
        start = np.r_[c[:2], z] + p * (lo - gap)     # jaw tips just behind it
        for L in (legs[-1:] if contact else legs):
            L = min(L, dist) if t == 0 else L
            shift = p * L
            end = np.r_[c[:2], z] + shift + p * (lo + jaw)   # body end against the brick's back
            moved = box + shift
            m_xy = moved[:, :2].mean(axis=0)
            r_now, r_end = np.hypot(*c[:2]), np.hypot(*m_xy)
            hi_r = max(band[1], r_now + 2.0) if contact else band[1]    # sideways is fine out there
            lo_r = min(band[0], r_now + 2.0)                            # in close: only outward
            if not lo_r <= r_end <= hi_r or abs(np.degrees(np.arctan2(m_xy[1], m_xy[0]))) > sector:
                no("out of band")
                continue
            path = [np.r_[start[:2], hi_z], start, end, np.r_[end[:2], hi_z]]
            if contact:
                reach = moved[:, :2].mean(axis=0) + p[:2] * 15     # shoved bricks end up ahead of it
                if not lo_r - 10 <= np.hypot(*reach) <= hi_r + 10:
                    no("shoves out of reach")
                    continue
                if any(np.hypot(*(reach - s_)) < spot_clear for s_ in spots):
                    no("near a spot")
                    continue
                if not path_clear(sim.gripper, R, path[:2], others, path_margin):
                    no("lands on a brick")
                    continue
            else:
                if any(np.min(np.linalg.norm(o[:, None, :] - moved[None, :, :2], axis=2)) < keep_off
                       for o in others_xy) or _inside_hull(others, np.vstack([box, moved]), path_margin):
                    no("brick path")
                    continue
                if not path_clear(sim.gripper, R, path, others, path_margin):
                    no("gripper path")
                    continue
            try:                      # IK all along the path (move_linear needs every step)
                q = sim.ik(path[0], R)
                q = sim.ik(start, R, q)
                worst = sim.clearance(q)
                for f in np.linspace(0, 1, max(2, int(L / 10) + 1))[1:]:
                    q = sim.ik(start + (end - start) * f, R, q)
                    worst = min(worst, sim.clearance(q))
                sim.ik(path[3], R, q)
            except ValueError:
                no("IK")
                continue
            if worst < 0.5:
                no("arm hits mat")
                continue
            if not best:
                return R, start, end, hi_z
            room = min((np.min(np.linalg.norm(o[:, None, :] - moved[None, :, :2], axis=2)) for o in others_xy),
                       default=1e9)
            score = room - outward_cost * max(0.0, r_end - r_now)
            if found is None or score > found[2]:
                found = ((R, start, end, hi_z), room, score)
    return found[:2] if found else None


def push(sim, plan, fast=150.0, lin=150.0, slow=40.0):
    """Carry out a plan_push plan: down behind the brick, push, back up."""
    R, start, end, hi_z = plan
    sim.grip(sim.gripper.stroke)      # jaws open: a small brick passes between them to the body end
    sim.move_joints(sim.ik(np.r_[start[:2], hi_z], R), speed=fast)
    try:
        sim.move_linear(start, speed=lin)
        sim.move_linear(end, speed=slow)
        sim.move_linear(end - R[:, 2] * 3.0, speed=slow)   # back off the brick before lifting
    finally:                                           # always leave at travel height
        _, p = sim.tcp_pose()
        sim.move_linear(np.r_[p[:2], hi_z], speed=lin)
