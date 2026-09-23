"""Frame conversions between the API (Z-up) and Three.js (Y-up)."""
import numpy as np
import pytest

import robot_client as ri

rng = np.random.default_rng(0)


def test_vec_round_trip():
    for v in rng.uniform(-1000, 1000, (50, 3)):
        np.testing.assert_allclose(ri._three_to_api_vec(ri._api_to_three_vec(v)), v)


def test_api_up_is_three_up():
    np.testing.assert_allclose(ri._api_to_three_vec([0, 0, 1]), [0, 1, 0])


def test_euler_round_trip():
    for rot in rng.uniform(-170, 170, (50, 3)):
        np.testing.assert_allclose(ri._three_euler_to_api(*ri._api_euler_to_three(rot)), rot, atol=1e-9)


def test_matrix_to_euler_inverts_rot_xyz():
    for ex, ey, ez in rng.uniform(-3, 3, (100, 3)):
        ey = np.clip(ey, -1.5, 1.5)                  # stay off gimbal lock
        R = ri._rot_xyz_three(ex, ey, ez)
        np.testing.assert_allclose(ri._rot_xyz_three(*ri._euler_xyz_from_matrix(R)), R, atol=1e-12)


def test_matrix_to_euler_at_gimbal_lock_is_still_the_same_rotation():
    R = ri._rot_xyz_three(0.3, np.pi / 2, 0.2)
    np.testing.assert_allclose(ri._rot_xyz_three(*ri._euler_xyz_from_matrix(R)), R, atol=1e-9)


# World (viewer) coordinates in, kinematic-frame coordinates out: the
# kinematic Y axis is the viewer's −Y.
world_to_local = ri.RobotClient._world_to_local_core


def test_world_to_local_with_device_at_origin_only_flips_y():
    pos, ori = world_to_local([100, 200, 300], [10, 20, 30], [0, 0, 0], [0, 0, 0])
    np.testing.assert_allclose(pos, [100, -200, 300], atol=1e-9)
    np.testing.assert_allclose(ori, [10, 20, 30], atol=1e-9)


def test_world_to_local_removes_translation():
    pos, _ = world_to_local([150, 250, 330], None, [50, 50, 30], [0, 0, 0])
    np.testing.assert_allclose(pos, [100, -200, 300], atol=1e-9)


def test_world_to_local_of_device_origin_is_zero_whatever_its_rotation():
    dev_pos, dev_rot = [120, -40, 75], [15, -30, 60]
    pos, _ = world_to_local(dev_pos, None, dev_pos, dev_rot)
    np.testing.assert_allclose(pos, [0, 0, 0], atol=1e-9)


def test_world_to_local_preserves_distances():
    dev_pos, dev_rot = [120, -40, 75], [15, -30, 60]
    a, _ = world_to_local([0, 0, 0], None, dev_pos, dev_rot)
    b, _ = world_to_local([300, 400, 0], None, dev_pos, dev_rot)
    assert np.linalg.norm(np.subtract(a, b)) == pytest.approx(500)


def test_world_to_local_passes_none_through():
    assert world_to_local(None, None, [0, 0, 0], [0, 0, 0]) == (None, None)
