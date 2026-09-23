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


# ── The scene around the planning device, from the viewer's replies ──────────

def scene_planner(engine, config_path, name="meca500", index=0):
    """A planner for device `index` built from the engine's listDevices and
    listObjects, as the MCP server builds it from the viewer's. The floor is
    left out so the only contacts are with the scene."""
    devices = reply(engine.request({"cmd": "listDevices"}), "devices")["devices"]
    objects = reply(engine.request({"cmd": "listObjects"}), "objects")["objects"]
    p = RobotPlanner(config_path(name))
    p.sync_from_viewer(devices, objects, index)
    p._floor_checked = [False] * len(p._floor_checked)
    return p


def scene_contact(engine):
    """Any contact the viewer sees other than a device touching itself."""
    pairs = reply(engine.request({"cmd": "getCollisions"}), "collisions")["pairs"]
    def device(side):
        return side.split(":")[0] if ":" in side else None
    return [pp for pp in pairs
            if not (device(pp["link"]) and device(pp["link"]) == device(pp["object"]))]


def random_pose(p, rng, bands=None):
    """Random joints within limits; `bands` narrows some joints {index: (lo, hi)}."""
    lo, hi = p.limits[:, 0].copy(), p.limits[:, 1].copy()
    for i, (a, b) in (bands or {}).items():
        lo[i], hi[i] = max(lo[i], a), min(hi[i], b)
    return [round(float(v), 3) for v in lo + rng.uniform(0, 1, p.n) * (hi - lo)]


# Meca500 poses leaning forward (+X of its base), towards an arm it faces.
REACHING = {0: (-35, 35), 1: (10, 90), 2: (-60, 30)}


def test_a_moved_device_is_planned_where_it_stands(engine, config_path):
    fresh(engine, "meca500_config.json")
    engine.request({"cmd": "setDeviceOrigin", "position": [300, 200, 50], "rotation": [0, 0, 70]})
    rng = np.random.default_rng(8)
    contacts = misses = 0
    for _ in range(100):
        p = scene_planner(engine, config_path)
        q = random_pose(p, rng)
        ee = reply(engine.request({"cmd": "setJoints", "angles": q}), "state")["eePosition"]
        d = rng.normal(size=3)
        pos = [ee[i] + d[i] / np.linalg.norm(d) * rng.uniform(0, 120) for i in range(3)]
        engine.request({"cmd": "setObject", "index": 0, "position": pos})
        p = scene_planner(engine, config_path)
        seen = bool(scene_contact(engine))
        contacts += seen
        misses += seen and p.diagnose(q) is None
    assert contacts >= 20, contacts
    assert misses == 0, f"planner missed {misses} of {contacts}"


def test_another_device_is_an_obstacle(engine, config_path):
    engine.request({"cmd": "loadScene", "scene": {"version": 1, "devices": [], "stls": []}})
    engine.request({"cmd": "setFloorCollision", "enabled": False})
    engine.request({"cmd": "addDevice", "config": "meca500_config.json"})
    [b] = [r for r in engine.request({"cmd": "addDevice", "config": "meca500_config.json"})
           if r["type"] == "deviceAdded"]
    engine.request({"cmd": "setDeviceOrigin", "device": b["id"], "position": [380, 0, 0], "rotation": [0, 0, 180]})
    # Distinct names, so a contact between the two is not read as one arm touching itself.
    engine.request({"cmd": "renameDevice", "device": b["id"], "name": "Other"})
    rng = np.random.default_rng(9)
    contacts = misses = 0
    for _ in range(150):
        p = scene_planner(engine, config_path)
        qa, qb = random_pose(p, rng, REACHING), random_pose(p, rng, REACHING)
        engine.request({"cmd": "setJoints", "device": "dev_0", "angles": qa})
        engine.request({"cmd": "setJoints", "device": b["id"], "angles": qb})
        p = scene_planner(engine, config_path)
        seen = bool(scene_contact(engine))
        contacts += seen
        misses += seen and p.diagnose(qa) is None
    assert contacts >= 20, contacts
    assert misses == 0, f"planner missed {misses} of {contacts}"


def test_carried_payload_moves_with_its_link(engine, config_path):
    fresh(engine, "meca500_config.json")                 # cube 0: the payload
    engine.request({"cmd": "addPrimitive", "type": "cube"})  # cube 1: fixed in the world
    engine.request({"cmd": "setObject", "index": 0, "name": "Payload", "scale": [1.5, 1.5, 1.5]})
    engine.request({"cmd": "setObject", "index": 1, "name": "Block", "scale": [2, 2, 2]})
    dev = reply(engine.request({"cmd": "listDevices"}), "devices")["devices"][0]
    # Payload just past the flange, carried by the last link.
    ee = reply(engine.request({"cmd": "home"}), "state")["eePosition"]
    engine.request({"cmd": "setObject", "index": 0, "position": [ee[0] + 45, ee[1], ee[2]],
                    "parent": f"{dev['id']}:L5"})
    rng = np.random.default_rng(10)
    p = scene_planner(engine, config_path)
    assert [b.name for b in p.attached] == ["Payload"]
    contacts = misses = 0
    for _ in range(150):
        q = random_pose(p, rng)
        state = reply(engine.request({"cmd": "setJoints", "angles": q}), "state")
        payload = reply(engine.request({"cmd": "getObject", "index": 0}), "object")["worldPosition"]
        d = rng.normal(size=3)
        pos = [payload[i] + d[i] / np.linalg.norm(d) * rng.uniform(0, 150) for i in range(3)]
        engine.request({"cmd": "setObject", "index": 1, "position": pos})
        # The planner is built once, at home, and must follow the payload itself.
        objects = reply(engine.request({"cmd": "listObjects"}), "objects")["objects"]
        block = [o for o in objects if o["name"] == "Block"]
        p.obstacles = []
        p.sync_from_viewer_objects(block)
        seen = any("Payload" in (pp["link"], pp["object"]) and "Block" in (pp["link"], pp["object"])
                   for pp in scene_contact(engine))
        contacts += seen
        misses += seen and p.diagnose(q) is None
    engine.request({"cmd": "home"})
    assert contacts >= 15, contacts
    assert misses == 0, f"planner missed {misses} of {contacts} payload contacts"
