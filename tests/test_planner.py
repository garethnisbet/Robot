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
