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
