"""RRT-Connect planner and its capsule geometry."""
import random

import numpy as np
import pytest

import planner
from planner import Capsule, RobotPlanner


def test_segment_distance_parallel_and_crossing():
    d, _, _ = planner._seg_seg_dist(np.zeros(3), np.array([1., 0, 0]),
                                    np.array([0., 1, 0]), np.array([1., 1, 0]))
    assert d == pytest.approx(1)
    d, _, _ = planner._seg_seg_dist(np.array([-1., 0, 0]), np.array([1., 0, 0]),
                                    np.array([0., -1, 0.5]), np.array([0., 1, 0.5]))
    assert d == pytest.approx(0.5)


def test_capsule_collisions():
    a = Capsule(np.zeros(3), np.array([1., 0, 0]), 0.1)
    b = Capsule(np.array([0., 0.15, 0]), np.array([1., 0.15, 0]), 0.1)
    c = Capsule(np.array([0., 0.25, 0]), np.array([1., 0.25, 0]), 0.1)
    assert planner.capsules_collide(a, b)
    assert not planner.capsules_collide(a, c)
    assert planner.capsule_sphere_collide(a, np.array([2.05, 0, 0]), 1.0)
    assert not planner.capsule_sphere_collide(a, np.array([0.5, 0.5, 0]), 0.3)


def test_capsule_box_test_is_exact_between_sample_points():
    # A 1.1 m capsule of radius 20 mm, and a 20 mm box whose nearest point
    # is 15 mm from the capsule's axis, midway between where 12 evenly
    # spaced samples would fall. The capsule overlaps it.
    cap = Capsule(np.zeros(3), np.array([1.1, 0, 0]), 0.02)
    x = 1.1 * 0.5 / 11          # halfway between samples 0 and 1
    box = planner.AABBObstacle(min=np.array([x - 0.01, 0.015, -0.01]),
                               max=np.array([x + 0.01, 0.035, 0.01]))
    assert planner.capsule_aabb_collide(cap, box)
    far = planner.AABBObstacle(min=np.array([x - 0.01, 0.025, -0.01]),
                               max=np.array([x + 0.01, 0.045, 0.01]))
    assert not planner.capsule_aabb_collide(cap, far)


def test_meca500_fk_home_flange(config_path):
    p = RobotPlanner(config_path("meca500"))
    flange = p.fk_frames([0] * 6)[-1][0]
    # The flange joint sits on the 308 mm line of the home pose.
    assert flange[1] == pytest.approx(0.308, abs=1e-3)


def test_plan_in_free_space_connects_start_and_goal(config_path):
    random.seed(0)
    p = RobotPlanner(config_path("meca500"), step_deg=5.0)
    start, goal = [0, 0, 0, 0, 0, 0], [60, -20, 20, 30, 30, 0]
    path = p.plan(start, goal, verbose=False)
    assert path is not None
    np.testing.assert_allclose(path[0], start)
    np.testing.assert_allclose(path[-1], goal)
    for a, b in zip(path, path[1:]):
        assert p._edge_valid(np.asarray(a), np.asarray(b))


def test_plan_refuses_a_goal_inside_an_obstacle(config_path):
    p = RobotPlanner(config_path("meca500"))
    goal = [0] * 6
    flange = p.fk_frames(goal)[-1][0]
    p.add_obstacle(flange, 0.05, "block")
    assert p.plan([30, 0, 0, 0, 0, 0], goal, verbose=False) is None


# The planner's arm must be the arm the viewer draws. The viewer is tied to
# GNKinematics by test/js/websocket-api.test.mjs; this ties the planner to
# GNKinematics too. The wrist centre (J5 origin) is GNKinematics' v3, row 3
# of f_kinematics. Planner frames are Three.js Y-up in metres; the
# kinematic frame is [x, −z, y] of that, in mm. GP225's config and
# RobotDefinitions differ by a few mm; every other arm agrees to well under
# 0.1 mm. The bug this guards against (fixed joints dropped, apiSign
# ignored) put the wrist metres away.
WRIST_TOLERANCE_MM = {"meca500": 0.1, "gp180": 0.1, "gp225": 3.0, "gp280": 0.1, "motomini": 0.1}


@pytest.mark.parametrize("name", WRIST_TOLERANCE_MM)
def test_wrist_centre_matches_gnkinematics(name, config_path):
    import RobotDefinitions as rd
    kin = {"meca500": rd.Meca500_kin, "gp180": rd.GP180_120_kin, "gp225": rd.GP225_kin,
           "gp280": rd.GP280_kin, "motomini": rd.MotoMini_kin}[name]
    p = RobotPlanner(config_path(name))
    rng = np.random.default_rng(4)
    for _ in range(20):
        q = p.limits[:, 0] + rng.uniform(0, 1, p.n) * (p.limits[:, 1] - p.limits[:, 0])
        wrist = p.fk_frames(q)[5][0]
        wrist_kin = np.array([wrist[0], -wrist[2], wrist[1]]) * 1000
        err = np.linalg.norm(wrist_kin - kin.f_kinematics(q)[3])
        assert err < WRIST_TOLERANCE_MM[name], f"{name} at {q.round(1)}: wrist {err:.1f} mm off"


def test_limits_are_in_api_convention(config_path):
    # GP225 J2 is stored as [-76, 60] in the model's convention with
    # apiSign -1; in API angles that is [-60, 76], as RobotDefinitions has it.
    p = RobotPlanner(config_path("gp225"))
    np.testing.assert_allclose(p.limits[1], [-60, 76])
    np.testing.assert_allclose(p.limits[0], [-180, 180])     # apiSign +1: unchanged


def test_fixed_joints_are_part_of_the_arm(config_path):
    # GP280's base column is a fixed joint: the shoulder sits 650 mm up.
    shoulder = RobotPlanner(config_path("gp280")).fk_frames([0] * 6)[2][0]
    assert shoulder[1] == pytest.approx(0.65, abs=1e-3)


def test_point_cloud_contact_matches_the_viewer_rule():
    # Contact is a point within radius + 40 mm of the capsule's axis.
    cap = Capsule(np.zeros(3), np.array([0.5, 0, 0]), 0.05)
    reach = 0.05 + planner.POINT_CLOUD_CONTACT
    inside = planner.PointCloudObstacle([[0.25, reach - 1e-6, 0], [3, 3, 3]])
    outside = planner.PointCloudObstacle([[0.25, reach + 1e-6, 0], [3, 3, 3]])
    beyond_end = planner.PointCloudObstacle([[0.5 + reach + 1e-6, 0, 0]])
    assert inside.hits_capsule(cap)
    assert not outside.hits_capsule(cap)
    assert not beyond_end.hits_capsule(cap)


def test_point_cloud_index_finds_points_in_every_cell():
    # Random points across many cells: the grid must agree with brute force.
    rng = np.random.default_rng(3)
    pts = rng.uniform(-1, 1, (20000, 3))
    cloud = planner.PointCloudObstacle(pts, cell=0.07)
    for _ in range(200):
        p0, p1 = rng.uniform(-1.2, 1.2, 3), rng.uniform(-1.2, 1.2, 3)
        cap = Capsule(p0, p1, rng.uniform(0, 0.05))
        d = p1 - p0
        t = np.clip((pts - p0) @ d / (d @ d), 0, 1)
        brute = np.min(np.linalg.norm(pts - (p0 + t[:, None] * d), axis=1)) <= cap.radius + cloud.contact
        assert cloud.hits_capsule(cap) == brute


# ── Carried payload: fitted capsules or its own box ──────────────────────────

def detector_vertices(rng):
    """A flat box with a cable stub: the shape of a detector on a flange."""
    box = rng.uniform([-0.15, -0.05, -0.12], [0.15, 0.05, 0.12], size=(3000, 3))
    stub = rng.uniform([-0.02, 0.05, -0.02], [0.02, 0.20, 0.02], size=(500, 3))
    return np.vstack([box, stub])


def tool_vertices(rng):
    """Two thin rods at a right angle: a bent sample stick."""
    t = rng.uniform(0, 1, size=(2000, 1))
    a = t * [0.3, 0, 0] + rng.normal(scale=0.004, size=(2000, 3))
    b = [0.3, 0, 0] + t * [0, 0.2, 0.1] + rng.normal(scale=0.004, size=(2000, 3))
    return np.vstack([a, b])


def rod_vertices(rng):
    """A thin rod lying diagonally in its own frame (a sample stick modelled
    askew), which no set of boxes in that frame follows well."""
    t = rng.uniform(0, 1, size=(3000, 1))
    return t * [0.3, 0.25, 0.2] + rng.normal(scale=0.004, size=(3000, 3))


def inside_union(pts, shapes):
    """Is every point inside at least one Capsule or OrientedBox? (Slack for
    float32 vertices.) Also returns the worst excess."""
    def excess(s):
        if isinstance(s, planner.OrientedBox):
            return np.linalg.norm(np.maximum(0, np.abs(s.to_local(pts)) - s.half), axis=1)
        return planner._segment_distances(pts, s.p0, s.p1) - s.radius
    d = np.min([excess(s) for s in shapes], axis=0)
    return bool(np.all(d <= 1e-7)), float(d.max())


def random_rotation(rng):
    q = rng.normal(size=4)
    return planner.qmat(q / np.linalg.norm(q))


def test_fitted_parts_enclose_every_point_and_beat_one_capsule_on_a_bent_tool():
    for pts in (tool_vertices(np.random.default_rng(1)), detector_vertices(np.random.default_rng(1))):
        parts = planner.fit_parts(pts)
        ok, worst = inside_union(pts, [planner.Capsule(np.asarray(a), np.asarray(b), r) for a, b, r in parts])
        assert ok, f"a point lies {worst * 1000:.3f} mm outside the parts"
    pts = tool_vertices(np.random.default_rng(1))
    one = planner.fit_capsule(pts)
    parts = planner.fit_parts(pts)
    volume = lambda caps: sum(planner._capsule_volume(r, np.linalg.norm(np.subtract(b, a))) for a, b, r in caps)
    assert len(parts) > 1 and volume(parts) < 0.3 * volume([one])


@pytest.mark.parametrize("shape,kind", [("detector", planner.OrientedBox), ("rod", planner.Capsule)])
def test_payload_is_fitted_in_its_own_frame_and_follows_any_pose(shape, kind):
    """A detector gets boxes, an askew rod capsules. Fitted once in the
    object's scaled frame, then carried by its pose: any rotation,
    translation and non-uniform scale still encloses it."""
    rng = np.random.default_rng(2)
    local = ((detector_vertices if shape == "detector" else rod_vertices)(rng) * 1000).astype("<f4")
    planner._PAYLOAD_FIT_CACHE.clear()
    for k in range(4):
        M = np.eye(4)
        M[:3, :3] = random_rotation(rng) @ np.diag([0.001, 0.0015, 0.00254])
        M[:3, 3] = rng.uniform(-1, 1, 3)
        obj = {"id": "p", "matrixWorld": list(M.T.ravel())}      # column-major, as Three.js
        shapes = planner.payload_shapes(obj, local)
        assert all(isinstance(s, kind) for s in shapes)
        ok, worst = inside_union(local.astype(float) @ M[:3, :3].T + M[:3, 3], shapes)
        assert ok, f"pose {k}: a vertex lies {worst * 1000:.3f} mm outside"
    assert len(planner._PAYLOAD_FIT_CACHE) == 1      # one shape, one fit


def cube_tris(lo, hi):
    """The 12 triangles of a box's surface."""
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    quads = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return np.array([t for a, b, cc, d in quads for t in ([c[a], c[b], c[cc]], [c[a], c[cc], c[d]])])


def frame_tris():
    """An open cage: the twelve edges of a 0.4 m cube as 20 mm bars, and a
    back panel of two large triangles, the kind a cut through vertices
    alone would leave uncovered."""
    s, w = 0.4, 0.02
    bars = []
    for axis in range(3):
        for u in (0, s - w):
            for v in (0, s - w):
                lo = np.zeros(3); hi = np.zeros(3)
                lo[axis], hi[axis] = 0, s
                others = [a for a in range(3) if a != axis]
                lo[others[0]], hi[others[0]] = u, u + w
                lo[others[1]], hi[others[1]] = v, v + w
                bars.append(cube_tris(lo, hi))
    panel = np.array([[[0, 0, s], [s, 0, s], [s, s, s]], [[0, 0, s], [s, s, s], [0, s, s]]], float)
    return np.vstack(bars + [panel])


def triangle_samples(tris, n=40, seed=0):
    """Points spread over every triangle (corners, edges and inside)."""
    rng = np.random.default_rng(seed)
    w = rng.dirichlet([1, 1, 1], size=n)
    w = np.vstack([np.eye(3), [[.5, .5, 0], [0, .5, .5], [.5, 0, .5]], w])
    return np.einsum("kj,tjd->tkd", w, tris).reshape(-1, 3)


def test_box_fit_encloses_whole_triangles_and_opens_up_a_frame():
    tris = frame_tris()
    boxes = planner.fit_boxes(tris)
    shapes = [planner.OrientedBox(c, np.eye(3), h) for c, h in boxes]
    ok, worst = inside_union(triangle_samples(tris), shapes)
    assert ok, f"a point of a triangle lies {worst * 1000:.3f} mm outside the boxes"
    volume = sum(8 * np.prod(h) for _, h in boxes)
    assert volume < 0.5 * 0.4 ** 3, f"{volume:.4f} m3: the frame's inside was not opened up"
    # A closed box's surface is a solid box, not six slabs.
    [(c, h)] = planner.fit_boxes(cube_tris([0, 0, 0], [0.1, 0.2, 0.3]))
    assert np.allclose(c, [0.05, 0.1, 0.15]) and np.allclose(h, [0.05, 0.1, 0.15])


def test_capsule_parts_keep_triangles_whole():
    """Big triangles of a bent tool: the capsules must enclose every point
    of each, not just its corners."""
    rng = np.random.default_rng(4)
    spine = np.vstack([np.linspace([0, 0, 0], [0.3, 0, 0], 40), np.linspace([0.3, 0, 0], [0.3, 0.2, 0.1], 30)])
    tris = np.array([[p, p + rng.normal(scale=0.01, size=3), spine[(i + 7) % len(spine)]]
                     for i, p in enumerate(spine)])
    caps = [planner.Capsule(np.asarray(a), np.asarray(b), r) for a, b, r in planner.fit_parts(tris, min_points=8)]
    assert len(caps) > 1
    ok, worst = inside_union(triangle_samples(tris), caps)
    assert ok, f"a point of a triangle lies {worst * 1000:.3f} mm outside the capsules"


def test_oriented_box_tests_are_exact():
    rng = np.random.default_rng(3)
    # 45° about Z: the box's world AABB reaches a corner block the box misses.
    s = np.sqrt(0.5)
    box = planner.OrientedBox([0, 0, 0], [[s, -s, 0], [s, s, 0], [0, 0, 1]], [0.1, 0.1, 0.1])
    corner = (np.array([0.09, 0.09, -0.05]), np.array([0.2, 0.2, 0.05]))
    assert planner._aabb_overlap(box.lo, box.hi, *corner) and not box.overlaps_aabb(*corner)
    assert box.overlaps_aabb(np.array([0.13, -0.01, -0.01]), np.array([0.3, 0.01, 0.01]))
    # (0.15, 0.15) lies 0.2121 along the diagonal, which is the normal of a
    # face 0.1 out: 0.1121 from the box.
    cap = planner.Capsule(np.array([0.15, 0.15, -1.0]), np.array([0.15, 0.15, 1.0]), 0.11)
    assert not box.meets_capsule(cap)
    cap.radius = 0.1125
    assert box.meets_capsule(cap)
    assert not box.meets_sphere(np.array([0.15, 0.15, 0]), 0.11)
    assert box.meets_sphere(np.array([0.15, 0.15, 0]), 0.1125)
    # Random boxes: a sampled point of the box inside the AABB means overlap
    # (the test never misses), and the SAT never reports a gap as contact
    # when the world AABBs are apart.
    for _ in range(300):
        box = planner.OrientedBox(rng.uniform(-0.3, 0.3, 3), random_rotation(rng), rng.uniform(0.02, 0.2, 3))
        c, h = rng.uniform(-0.3, 0.3, 3), rng.uniform(0.02, 0.2, 3)
        lo, hi = c - h, c + h
        pts = box.centre + (rng.uniform(-1, 1, (4000, 3)) * box.half) @ box.axes.T
        if np.any(np.all((pts >= lo) & (pts <= hi), axis=1)):
            assert box.overlaps_aabb(lo, hi)
        if not planner._aabb_overlap(box.lo, box.hi, lo, hi):
            assert not box.overlaps_aabb(lo, hi)
