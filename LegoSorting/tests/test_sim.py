"""Stage 1 against the VRRobotController twin, rendered headless.

Skipped when ../VRRobotController or Chrome is missing. About 15 s.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from legosort import simserver  # noqa: E402
from legosort.boards import BaseMat  # noqa: E402
from legosort.geometry import project, rotation_angle_deg  # noqa: E402
from legosort.tracker import BaseTracker  # noqa: E402

pytestmark = pytest.mark.skipif(not simserver.available(), reason="VRRobotController viewer or Chrome not available")


@pytest.fixture(scope="module")
def viewer():
    from legosort.sim import Viewer, build_scene
    with simserver.HeadlessViewer() as hv:
        v = Viewer(hv.ws_url, hv.session)
        v.set_joints([0, -20, 20, 0, 60, 0])
        build_scene(v, BaseMat.load())
        yield v
        v.close()


@pytest.fixture(scope="module")
def kin():
    sys.path.insert(0, str(simserver.VIEWER_DIR))
    from RobotDefinitions import Meca500_kin
    return Meca500_kin


def test_viewer_matches_gnkinematics_with_mirrored_y(viewer, kin):
    """The API reports EE positions as (x, -y, z) of the GNKinematics robot frame."""
    from legosort.sim import from_api
    rng = np.random.default_rng(0)
    for _ in range(5):
        a = rng.uniform([-90, -40, -60, -90, -60, -90], [90, 40, 30, 90, 60, 90])
        st = viewer.set_joints(a)
        fk = kin.f_kinematics(a)[-2]
        assert np.allclose(from_api(st["eePosition"]), fk, atol=0.05), (a, st["eePosition"], fk)
    viewer.set_joints([0, -20, 20, 0, 60, 0])


@pytest.mark.parametrize("eye,up", [
    ((230, -420, 480), (1, 0, 0)),
    ((520, 260, 420), (-1, 0, 0.3)),
    ((-120, -250, 600), (1, 0.5, 0)),
])
def test_base_pose_from_rendered_view(viewer, eye, up):
    from legosort.sim import SimCamera
    mat = BaseMat.load()
    cam = SimCamera(eye=eye, up=up, fov_deg=50)
    img, K = viewer.capture(cam, exposure=0.15)
    assert np.allclose(K, cam.K())
    pose = BaseTracker(mat, K, np.zeros(5)).estimate(img)
    assert pose is not None and pose.n_markers >= 8
    assert pose.rms_px < 0.5
    assert np.linalg.norm(pose.camera_position - np.asarray(eye, float)) < 1.0
    assert rotation_angle_deg(pose.T_cam_robot[:3, :3] @ cam.T_cam_robot[:3, :3].T) < 0.1
    # table points: true pixel -> estimated robot coordinates
    pts = np.array([[200.0, -60.0, 0.0], [300.0, 70.0, 0.0]])
    pix = project(pts, K, np.zeros(5), cam.T_cam_robot)
    got = BaseTracker(mat, K, np.zeros(5)).pixel_to_robot(pose, pix)
    assert np.max(np.linalg.norm(got - pts, axis=1)) < 0.5


def test_sim_robot_applies_joint_offsets(viewer):
    from legosort.sim import SimRobot
    off = np.array([0.0, 0.2, -0.15, 0.3, -0.1, 0.25])
    st = SimRobot(viewer, off).move_joints([10, 0, 10, 0, 45, 0])
    assert np.allclose(st["joints"], np.array([10, 0, 10, 0, 45, 0]) + off, atol=1e-3)
    viewer.set_joints([0, -20, 20, 0, 60, 0])
