"""The headless engine driven from Python, as the MCP server and robot.plan use it.

Scenes are given in the viewer's exportScene format, so no viewer is needed.
Skipped where Node or node_modules is missing.
"""
import base64
import copy
import struct

import pytest

from headless_client import HeadlessEngine, HeadlessEngineError

pytestmark = pytest.mark.skipif(not HeadlessEngine.available(),
                                reason="needs node and node_modules (npm ci)")


def cube_stl(size=0.05):
    """Binary STL of an axis-aligned cube centred on the origin (metres)."""
    h = size / 2
    v = [(x, y, z) for x in (-h, h) for y in (-h, h) for z in (-h, h)]
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    out = bytearray(80) + struct.pack("<I", len(faces))
    for f in faces:
        out += struct.pack("<3f", 0, 0, 0)
        for i in f:
            out += struct.pack("<3f", *v[i])
        out += b"\0\0"
    return bytes(out)


def api_to_three(p_mm):
    """API position (mm, Z-up) to a saved-scene position (m, Three.js Y-up)."""
    return [p_mm[0] / 1000, p_mm[2] / 1000, p_mm[1] / 1000]


def scene(cube_at=None, extra=()):
    s = {"version": 1,
         "devices": [{"configFile": "meca500_config.json", "name": "Meca500",
                      "jointAngles": [0] * 6, "position": [0, 0, 0],
                      "rotation": [0, 0, 0], "visible": True, "parentLink": None}],
         "stls": list(extra)}
    if cube_at is not None:
        s["stls"].insert(0, {"id": "c1", "name": "Cube", "fileType": "stl",
                             "buffer": base64.b64encode(cube_stl()).decode(),
                             "position": api_to_three(cube_at), "rotation": [0, 0, 0],
                             "scale": [1, 1, 1], "visible": True, "parentLink": None})
    return s


def exporter(payload):
    """Stand-in for the viewer's exportScene: geometry only when asked."""
    def export(buffers):
        p = copy.deepcopy(payload)
        if not buffers:
            for r in p["stls"]:
                r.pop("buffer", None)
        return p
    return export


@pytest.fixture(scope="module")
def engine():
    with HeadlessEngine() as e:
        yield e


def ee_at(engine, angles):
    """Where the Meca500's EE is at `angles` (API mm), asked of the engine."""
    engine.sync_scene(exporter(scene()))
    state = next(r for r in engine.request({"cmd": "setJoints", "device": "Meca500",
                                            "angles": angles}) if r["type"] == "state")
    engine.request({"cmd": "home", "device": "Meca500"})
    return state["eePosition"]


def test_geometry_is_sent_only_when_the_scene_changes(engine):
    first = engine.sync_scene(exporter(scene(cube_at=[500, 500, 500])))
    assert first["rebuilt"] and first["objects"] == 1
    moved = engine.sync_scene(exporter(scene(cube_at=[190, 0, 308])))
    assert moved["rebuilt"] is False
    hit = engine.check_path("Meca500", [[0] * 6])
    assert not hit["ok"] and any(p["object"] == "Cube" for p in hit["pairs"])


def test_a_sweep_is_stopped_where_it_first_meets_the_cube(engine):
    target = ee_at(engine, [45, 0, 0, 0, 0, 0])
    engine.sync_scene(exporter(scene(cube_at=target)))
    towards = engine.check_path("Meca500", [[0] * 6, [90, 0, 0, 0, 0, 0]], resolution_deg=1)
    assert not towards["ok"]
    assert any(p["object"] == "Cube" for p in towards["pairs"])
    assert 20 < towards["angles"][0] <= 45, towards
    assert towards["segment"] == [0, 1]
    away = engine.check_path("Meca500", [[0] * 6, [-90, 0, 0, 0, 0, 0]])
    assert away["ok"], away
    # The check leaves the device where it was.
    assert engine.check_path("Meca500", [[0] * 6])["ok"] is not None
    state = next(r for r in engine.request({"cmd": "getState", "device": "Meca500"}))
    assert state["joints"] == [0] * 6


def test_endpoints_alone_would_miss_what_the_sweep_hits(engine):
    target = ee_at(engine, [45, 0, 0, 0, 0, 0])
    engine.sync_scene(exporter(scene(cube_at=target)))
    assert engine.check_path("Meca500", [[0] * 6])["ok"]
    assert engine.check_path("Meca500", [[90, 0, 0, 0, 0, 0]])["ok"]
    assert not engine.check_path("Meca500", [[0] * 6, [90, 0, 0, 0, 0, 0]])["ok"]


def test_limits_are_checked_in_api_degrees(engine):
    engine.sync_scene(exporter(scene()))
    # J1 sweeps freely at home, so the only thing stopping it is its limit.
    r = engine.check_path("Meca500", [[0] * 6, [200, 0, 0, 0, 0, 0]])
    assert not r["ok"] and "outside its limits [-175, 175]" in r["reason"]
    assert 175 < r["angles"][0] <= 176


def test_self_collision_counts_against_the_path(engine):
    engine.sync_scene(exporter(scene()))
    r = engine.check_path("Meca500", [[0, 60, 60, 0, 60, 0]])
    assert not r["ok"] and {"link": "L0", "object": "L4"} in r["pairs"]


def test_what_the_engine_cannot_see_is_reported(engine):
    cloud = {"id": "pc", "name": "Room scan", "fileType": "ply", "isPointCloud": True,
             "sourceFile": "room.ply", "position": [0, 0, 0], "rotation": [0, 0, 0],
             "scale": [1, 1, 1], "visible": True, "parentLink": None}
    engine.sync_scene(exporter(scene(extra=[cloud])))
    assert engine.check_path("Meca500", [[0] * 6])["unchecked"] == ["Room scan (point cloud)"]
    hidden = dict(cloud, visible=False)
    engine.sync_scene(exporter(scene(extra=[hidden])))
    assert engine.check_path("Meca500", [[0] * 6])["unchecked"] == []


def test_wrong_joint_count_is_refused(engine):
    engine.sync_scene(exporter(scene()))
    r = engine.check_path("Meca500", [[0, 0, 0]])
    assert not r["ok"] and "6 joints" in r["reason"]
