"""Regression tests for the analytic IK the client uses for 'ee:' moves.

Conventions: f_kinematics(joints)[-2] is the EE position (mm) and [-1] its
ZYX Euler angles in RADIANS; setEulerTarget wants the angles in DEGREES.
Joint angles are in the WebSocket API convention.
"""
import numpy as np
import pytest

from RobotDefinitions import GP180_120_kin, GP225_kin, GP280_kin, Meca500_kin, MotoMini_kin

ARMS = {
    "Meca500": Meca500_kin,
    "GP180_120": GP180_120_kin,
    "GP225": GP225_kin,
    "GP280": GP280_kin,
    "MotoMini": MotoMini_kin,
}


def pose(kin, joints):
    out = kin.f_kinematics(np.asarray(joints, dtype=float))
    return np.concatenate([out[-2], np.degrees(out[-1])])


def random_joints(kin, rng, margin=0.8, bands=()):
    """Joints inside the limits; `bands` maps a joint index to a narrower range."""
    lo, hi = kin.motor_limits[:, 0].copy(), kin.motor_limits[:, 1].copy()
    for i, (b_lo, b_hi) in dict(bands).items():
        lo[i], hi[i] = max(lo[i], b_lo), min(hi[i], b_hi)
    mid, half = (lo + hi) / 2, (hi - lo) / 2 * margin
    return mid + rng.uniform(-1, 1, 6) * half


def solve(kin, target, seed):
    kin.storeCurrentPosition(np.asarray(seed, dtype=float))
    with np.errstate(invalid="ignore"):
        return np.asarray(kin.setEulerTarget(list(target)), dtype=float).ravel()


def test_meca500_home_matches_the_manual():
    np.testing.assert_allclose(pose(Meca500_kin, [0] * 6)[:3], [190, 0, 308], atol=1e-6)


@pytest.mark.parametrize("name", ARMS)
def test_fk_ik_round_trip(name):
    kin, rng = ARMS[name], np.random.default_rng(1)
    for _ in range(50):
        target = pose(kin, random_joints(kin, rng, bands={2: (-90, 90), 4: (-90, 90)}))
        sol = solve(kin, target, np.zeros(6))
        assert not np.isnan(sol).any(), f"no solution for reachable pose {target}"
        # 1 mm is the solver's own acceptance tolerance; near a wrist
        # singularity (J5 ≈ 0) it lands a few tenths of a mm off.
        np.testing.assert_allclose(pose(kin, sol)[:3], target[:3], atol=1.0)


# Known gaps: the solver misses some poses the arm does reach when the
# elbow is folded past 90° (GP280/MotoMini allow J3 up to 197°) or the
# Meca500 wrist is bent past ~93° (J5 goes to 115°). Sampling the full
# limits, GP280 fails ~17%, MotoMini ~13%, Meca500 ~1%. strict=True turns
# these red once the solver is fixed, so they can be dropped then.
@pytest.mark.xfail(strict=True, reason="IK misses reachable poses in this joint band")
@pytest.mark.parametrize("name,bands", [
    ("GP280", {2: (100, 197)}),
    ("MotoMini", {2: (100, 197)}),
    ("Meca500", {4: (95, 115)}),
])
def test_fk_ik_round_trip_known_gaps(name, bands):
    kin, rng = ARMS[name], np.random.default_rng(1)
    for _ in range(50):
        target = pose(kin, random_joints(kin, rng, margin=1.0, bands=bands))
        assert not np.isnan(solve(kin, target, np.zeros(6))).any()


@pytest.mark.parametrize("name", ["Meca500", "GP180_120"])
def test_solution_does_not_depend_on_the_seed(name):
    # Regression: only the strategy's first pick used to be FK-checked, so
    # some seeds reported "no solution" for a reachable pose.
    kin, rng = ARMS[name], np.random.default_rng(2)
    target = pose(kin, random_joints(kin, rng))
    for _ in range(10):
        sol = solve(kin, target, random_joints(kin, rng))
        assert not np.isnan(sol).any()


def test_alpha_just_off_180_still_solves():
    # Regression: readback orientations arrive as −179.999, not ±180, and
    # a 1e-4 mm FK tolerance used to reject every candidate there.
    kin = GP180_120_kin
    home = pose(kin, np.zeros(6))
    target = [home[0] - 300, home[1] + 50, home[2] - 200, -179.999, home[4], home[5]]
    sol = solve(kin, target, np.zeros(6))
    assert not np.isnan(sol).any()
    np.testing.assert_allclose(pose(kin, sol)[:3], target[:3], atol=0.05)


def test_unreachable_pose_is_nan():
    sol = solve(Meca500_kin, [5000, 0, 0, 0, 90, 0], np.zeros(6))
    assert np.isnan(sol).all()
