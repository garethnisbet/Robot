"""Scan and axis-name helpers in the IPython client."""
import pytest

import robot_client as ri
from robot_client import RobotClient


@pytest.mark.parametrize("start,end,step,want", [
    (0, 10, 5, [0, 5, 10]),
    (0, 10, 4, [0, 4, 8, 10]),            # always ends exactly on `end`
    (10, 0, 5, [10, 5, 0]),               # descending
    (10, 0, -5, [10, 5, 0]),              # step sign is irrelevant
    (3, 3, 1, [3]),
])
def test_build_axis_vals(start, end, step, want):
    assert ri._build_axis_vals(start, end, step) == want


def test_densify_path_respects_step_and_keeps_waypoints():
    path = [[0, 0], [10, 0], [10, -3]]
    dense = ri._densify_path(path, 2.5)
    assert dense[0] == [0, 0] and dense[-1] == [10, -3]
    assert [10, 0] in dense
    for a, b in zip(dense, dense[1:]):
        assert max(abs(x - y) for x, y in zip(a, b)) <= 2.5 + 1e-12


@pytest.mark.parametrize("token,want", [
    (3, {"index": 3}),
    ("#4", {"index": 4}),
    ("Table", {"object": "Table"}),
    ("#tag", {"object": "#tag"}),
])
def test_parse_obj_ref(token, want):
    assert ri._parse_obj_ref(token) == want


def test_movable_joints_skip_fixed(config_path):
    config = ri._load_config(config_path("gp225"))
    names = [n for _, n in ri._get_movable_joints(config)]
    assert len(names) == 6
    assert all(not j.get("fixed") for j in config["joints"] if j["name"] in names)


def test_resolve_axis_for_device_by_name_prefix_and_number():
    joints = [(0, "J1 Base"), (1, "J2 Shoulder")]
    assert ri._resolve_axis_for_device("j2", joints) == 1
    assert ri._resolve_axis_for_device("J1 Base", joints) == 0
    assert ri._resolve_axis_for_device("2", joints) == 1
    assert ri._resolve_axis_for_device("3", joints) is None
    assert ri._resolve_axis_for_device("elbow", joints) is None


class TestResolveAxis:
    names = ["delta", "eta", "chi", "phi", "mu"]

    def test_by_index_and_negative_index(self):
        assert RobotClient._resolve_axis(1, 5, self.names) == 1
        assert RobotClient._resolve_axis(-1, 5, self.names) == 4
        assert RobotClient._resolve_axis("2", 5, self.names) == 2

    def test_by_name_and_unique_prefix(self):
        assert RobotClient._resolve_axis("Chi", 5, self.names) == 2
        assert RobotClient._resolve_axis("m", 5, self.names) == 4

    @pytest.mark.parametrize("bad", ["nope", 9, True, 1.5])
    def test_rejects_unknown_and_bad_types(self, bad):
        with pytest.raises(ValueError):
            RobotClient._resolve_axis(bad, 5, self.names)

    def test_rejects_an_ambiguous_prefix(self):
        with pytest.raises(ValueError):
            RobotClient._resolve_axis("p", 3, ["phi", "psi", "mu"])


@pytest.mark.parametrize("key,want", [(0, 0), (5, 5), ("x", 0), ("G", 5), ("ee:z", 2)])
def test_resolve_ee_key(key, want):
    assert RobotClient._resolve_ee_key(key) == want


@pytest.mark.parametrize("bad", [6, -1, "q", "ee:w"])
def test_resolve_ee_key_rejects(bad):
    with pytest.raises(ValueError):
        RobotClient._resolve_ee_key(bad)


def test_prefixed_axis_names():
    assert RobotClient._parse_virtual_axis("V:Chi") == "chi"
    assert RobotClient._parse_virtual_axis("chi") is None
    assert RobotClient._parse_ee_axis("ee:b") == 4
    assert RobotClient._parse_ee_axis("ee:q") is None
