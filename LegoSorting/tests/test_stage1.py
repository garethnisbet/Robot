"""Stage 1 on synthetic images: intrinsics, base-mat pose, robot-frame invariance.

Run:  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from legosort import synth  # noqa: E402
from legosort.boards import BaseMat, CharucoSpec  # noqa: E402
from legosort.geometry import (invert_T, look_at, make_T, project,  # noqa: E402
                               rotation_angle_deg, rot_x, rot_z, transform_points)
from legosort.intrinsics import calibrate, detect_charuco  # noqa: E402
from legosort.tracker import BaseTracker  # noqa: E402

SIZE = (1280, 720)
K = synth.intrinsics_matrix(920.0, 918.0, 645.0, 356.0)
DIST = np.array([0.08, -0.15, 0.0005, -0.0004, 0.0])   # k3 = 0, as calibrate() assumes
MAT_PX_PER_MM = 4.0


@pytest.fixture(scope="module")
def mat():
    return BaseMat.load()


@pytest.fixture(scope="module")
def mat_texture(mat):
    return mat.render(MAT_PX_PER_MM)


def camera_over_mat(mat, eye_robot, target_robot=(230, 0, 0)):
    """T_cam_board for a camera at ``eye_robot`` (robot frame) looking at a robot-frame point."""
    T_robot_cam = look_at(eye_robot, target_robot, up=(1, 0, 0))
    return invert_T(T_robot_cam) @ mat.T_robot_board


# ----------------------------------------------------------------- geometry

def test_transform_roundtrip():
    T = make_T(rot_x(20) @ rot_z(-35), [10, -4, 300])
    assert np.allclose(invert_T(T) @ T, np.eye(4))
    p = np.array([[1.0, 2.0, 3.0]])
    assert np.allclose(transform_points(invert_T(T), transform_points(T, p)), p)


def test_mat_markers_unique_and_inside(mat):
    ids = [m.id for m in mat.markers]
    assert len(ids) == len(set(ids)) >= 4
    for m in mat.markers:
        c = m.corners()
        assert np.all(np.abs(c[:, 0]) <= mat.width_mm / 2)
        assert np.all(np.abs(c[:, 1]) <= mat.height_mm / 2)


def test_default_mat_axes(mat):
    """Board +Y points at the robot (robot -X); board +X is robot +Y."""
    R = mat.T_robot_board[:3, :3]
    assert np.allclose(R @ [0, 1, 0], [-1, 0, 0])
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])


# ----------------------------------------------------------------- intrinsics

def test_intrinsics_recovered_from_synthetic_views():
    spec = CharucoSpec()
    tex = spec.render(px_per_mm=5.0)
    rng = np.random.default_rng(3)
    views = []
    for i in range(80):
        T = synth.random_view(rng, distance=(380, 650), tilt_deg=(5, 40), aim_mm=170)
        img = synth.add_noise(synth.render_plane(tex, 5.0, K, T, SIZE, DIST), 1.5, seed=i)
        v = detect_charuco(img, spec)
        if v is not None and len(v[0]) >= 10:
            views.append(v)
        if len(views) == 25:
            break
    assert len(views) >= 15
    from legosort.intrinsics import view_diversity
    assert view_diversity(SIZE, views) > 0.75
    intr, kept = calibrate(views, SIZE)
    assert intr.rms < 0.5
    assert abs(intr.K[0, 0] / K[0, 0] - 1) < 0.01
    assert abs(intr.K[1, 1] / K[1, 1] - 1) < 0.01
    assert abs(intr.K[0, 2] - K[0, 2]) < 5
    assert abs(intr.K[1, 2] - K[1, 2]) < 5
    # Distortion: compare the effect, not the coefficients (they trade off).
    edge = np.array([[30.0, 30.0], [1250.0, 690.0], [640.0, 20.0]])
    from legosort.geometry import pixel_rays
    assert np.allclose(pixel_rays(edge, intr.K, intr.dist), pixel_rays(edge, K, DIST), atol=2e-3)


# ----------------------------------------------------------------- base mat pose

@pytest.mark.parametrize("eye", [(230, -350, 450), (500, 250, 380), (-150, 120, 500), (230, 0, 620)])
def test_base_pose_recovered(mat, mat_texture, eye):
    T_true = camera_over_mat(mat, eye)
    img = synth.add_noise(synth.render_plane(mat_texture, MAT_PX_PER_MM, K, T_true, SIZE, DIST))
    tr = BaseTracker(mat, K, DIST)
    pose = tr.estimate(img)
    assert pose is not None and pose.n_markers >= 8
    assert pose.rms_px < 0.5
    dR = pose.T_cam_board[:3, :3] @ T_true[:3, :3].T
    assert rotation_angle_deg(dR) < 0.15
    # camera position error, robot frame
    assert np.linalg.norm(pose.camera_position - np.asarray(eye)) < 2.0


def test_robot_frame_invariant_to_camera_motion(mat, mat_texture):
    """The core claim: a fixed table point gets the same robot coordinates from any camera pose."""
    table_pts = np.array([[200.0, -60.0, 0.0], [300.0, 70.0, 0.0], [250.0, 0.0, 0.0]])
    tr = BaseTracker(mat, K, DIST)
    results = []
    for eye in [(230, -380, 420), (520, 200, 360), (60, 260, 520)]:
        T_true = camera_over_mat(mat, eye)
        img = synth.add_noise(synth.render_plane(mat_texture, MAT_PX_PER_MM, K, T_true, SIZE, DIST))
        pose = tr.estimate(img)
        assert pose is not None
        # where the points really appear in this view
        pix = project(table_pts, K, DIST, T_true @ invert_T(mat.T_robot_board))
        results.append(tr.pixel_to_robot(pose, pix))
    for r in results:
        assert np.max(np.linalg.norm(r - table_pts, axis=1)) < 1.0
    assert np.max(np.linalg.norm(results[0] - results[1], axis=1)) < 1.0


def test_tracking_survives_occlusion(mat, mat_texture):
    """Half the mat hidden (e.g. by the arm) still gives a good pose."""
    T_true = camera_over_mat(mat, (230, -300, 480))
    img = synth.render_plane(mat_texture, MAT_PX_PER_MM, K, T_true, SIZE, DIST)
    img[:, : SIZE[0] // 2] = 90
    pose = BaseTracker(mat, K, DIST).estimate(img)
    assert pose is not None and 2 <= pose.n_markers < len(mat.markers)
    assert rotation_angle_deg(pose.T_cam_board[:3, :3] @ T_true[:3, :3].T) < 0.5


def test_no_mat_gives_none(mat):
    img = np.full((SIZE[1], SIZE[0]), 128, np.uint8)
    assert BaseTracker(mat, K, DIST).estimate(img) is None
