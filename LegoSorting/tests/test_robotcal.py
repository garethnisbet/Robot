"""Safety rules for the poses calibrate_from_robot drives the arm through (no viewer needed)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from legosort import simserver  # noqa: E402

pytestmark = pytest.mark.skipif(not simserver.VIEWER_DIR.exists(), reason="VRRobotController not found")

SAFE_POSE = [0, -20, 20, 0, 60, 0]


@pytest.fixture(scope="module")
def rm():
    from legosort.robotmodel import RobotModel
    return RobotModel()


def test_limits_inside_every_clamp():
    from legosort import robotcal
    lo, hi = robotcal.joint_limits()
    bl = np.array(robotcal.BRIDGE_LIMITS)
    assert np.all(lo > bl[:, 0]) and np.all(hi < bl[:, 1]) and np.all(lo < hi)


def test_gripper_into_table_is_unsafe(rm):
    from legosort.robotcal import Safety
    s = Safety(clearance=15, tool_length=90, tool_radius=25)
    assert s.why_unsafe(rm, SAFE_POSE) is None
    assert "table" in s.why_unsafe(rm, [0, 60, 30, 0, 60, 0])
    assert not s.path_ok(rm, SAFE_POSE, [0, 60, 30, 0, 60, 0])


def test_camera_keep_out(rm):
    from legosort.robotcal import Safety, tool_points
    tip = tool_points(SAFE_POSE, 0, 0)[0]
    assert "camera" in Safety(camera=tip + [0, 0, 100], camera_clear=150).why_unsafe(rm, SAFE_POSE)
    assert Safety(camera=tip + [0, 0, 400], camera_clear=150).why_unsafe(rm, SAFE_POSE) is None
