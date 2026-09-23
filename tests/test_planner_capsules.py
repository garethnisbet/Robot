"""The planner's fitted capsules against the viewer's exact check.

The capsules enclose each link's mesh (test/js/capsules.test.mjs), so the
planner may call a pose blocked when the viewer would not, but never the
reverse. These tests hold that end to end, through the headless engine.
"""
import random

import numpy as np
import pytest

from headless_client import HeadlessEngine
from planner import RobotPlanner

pytestmark = pytest.mark.skipif(not HeadlessEngine.available(),
                                reason="needs node and node_modules (npm ci)")


@pytest.fixture(scope="module")
def engine():
    with HeadlessEngine() as e:
        yield e


def reply(replies, kind):
    return next(r for r in replies if r["type"] == kind)


def fresh(engine, config):
    engine.request({"cmd": "loadScene", "scene": {"version": 1, "devices": [], "stls": []}})
    engine.request({"cmd": "addDevice", "config": config})
    engine.request({"cmd": "addPrimitive", "type": "cube"})
    engine.request({"cmd": "setFloorCollision", "enabled": False})


def cube_blocks(planner, q):
    """Does any of the arm's capsules meet the cube's box? (Only the cube:
    diagnose stops at the first problem, which may be the floor.)"""
    from planner import capsule_aabb_collide
    return any(capsule_aabb_collide(c, o) for c in planner._capsules(np.asarray(q, float))
               for o in planner.obstacles)


def test_fitted_capsules_are_used_when_present(config_path):
    assert RobotPlanner(config_path("meca500")).capsule_source == "fitted"
    assert RobotPlanner(config_path("meca500"), capsule_radii=0.018).capsule_source == "joint-to-joint"


@pytest.mark.parametrize("name,scale", [("meca500", 1), ("gp280", 6)])
def test_planner_never_misses_a_contact_the_viewer_sees(engine, config_path, name, scale):
    fresh(engine, f"{name}_config.json")
    engine.request({"cmd": "setObject", "index": 0, "scale": [scale] * 3})
    p = RobotPlanner(config_path(name))
    rng = np.random.default_rng(5)
    contacts = misses = stricter = 0
    for _ in range(120):
        q = [round(float(v), 3) for v in p.limits[:, 0] + rng.uniform(0, 1, p.n) * (p.limits[:, 1] - p.limits[:, 0])]
        ee = reply(engine.request({"cmd": "setJoints", "angles": q}), "state")["eePosition"]
        # Near the end effector, where a few mm decide contact.
        d = rng.normal(size=3)
        pos = [ee[i] + d[i] / np.linalg.norm(d) * rng.uniform(0, 120) * scale for i in range(3)]
        engine.request({"cmd": "setObject", "index": 0, "position": pos})
        pairs = reply(engine.request({"cmd": "getCollisions"}), "collisions")["pairs"]
        viewer_hit = any("Cube" in (pp["link"], pp["object"]) for pp in pairs)
        p.obstacles = []
        p.sync_from_viewer_objects(reply(engine.request({"cmd": "listObjects"}), "objects")["objects"])
        planner_hit = cube_blocks(p, q)
        contacts += viewer_hit
        misses += viewer_hit and not planner_hit
        stricter += planner_hit and not viewer_hit
    assert contacts >= 20, f"only {contacts} contacts sampled"
    assert misses == 0, f"planner missed {misses} of {contacts} contacts the viewer saw"
    assert stricter > 0 or contacts == 120   # it does err on the safe side


def test_planned_paths_round_a_cube_pass_the_exact_check(engine, config_path):
    fresh(engine, "meca500_config.json")
    engine.request({"cmd": "setFloorCollision", "enabled": True})
    at45 = reply(engine.request({"cmd": "setJoints", "angles": [45, 0, 0, 0, 0, 0]}), "state")["eePosition"]
    engine.request({"cmd": "home"})
    engine.request({"cmd": "setObject", "index": 0, "position": at45})
    objects = reply(engine.request({"cmd": "listObjects"}), "objects")["objects"]
    assert not engine.check_path(0, [[0] * 6, [90, 0, 0, 0, 0, 0]])["ok"], "the straight sweep must hit the cube"
    for seed in range(5):
        random.seed(seed)
        np.random.seed(seed)
        p = RobotPlanner(config_path("meca500"))
        p.sync_from_viewer_objects(objects)
        path = p.plan([0] * 6, [90, 0, 0, 0, 0, 0], verbose=False)
        assert path is not None
        verdict = engine.check_path(0, path, 1.0)
        assert verdict["ok"], f"seed {seed}: {verdict['reason']}"
