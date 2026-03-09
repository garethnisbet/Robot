"""
Numerical inverse kinematics using damped least-squares (Levenberg-Marquardt).

Solves for joint angles q such that the end-effector position matches target_pos.
Orientation is optionally included via a quaternion error term.
"""

import numpy as np
from .robot import Robot


def _rotation_error(R_current: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """Return a 3-vector angular error from current to target rotation."""
    R_err = R_target @ R_current.T
    # Extract axis-angle from the skew-symmetric part
    angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
    if abs(angle) < 1e-8:
        return np.zeros(3)
    axis = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ]) / (2 * np.sin(angle))
    return axis * angle


def inverse_kinematics(
    robot: Robot,
    target_pos: np.ndarray,
    target_rot: np.ndarray | None = None,
    max_iter: int = 200,
    tol: float = 1e-4,
    damping: float = 0.05,
    q0: np.ndarray | None = None,
) -> tuple[np.ndarray, bool]:
    """
    Solve IK for a target end-effector position (and optionally orientation).

    Parameters
    ----------
    robot      : Robot instance (not modified)
    target_pos : desired end-effector position (3,)
    target_rot : desired end-effector rotation (3,3), or None for position-only IK
    max_iter   : maximum number of iterations
    tol        : convergence threshold on error norm
    damping    : damping factor lambda for DLS
    q0         : initial joint configuration; defaults to robot.q

    Returns
    -------
    q_sol  : joint angles that achieve the target (n_dof,)
    converged : True if the solution is within tolerance
    """
    target_pos = np.asarray(target_pos, dtype=float)
    q = robot.q if q0 is None else np.asarray(q0, dtype=float).copy()

    for _ in range(max_iter):
        # Compute FK
        T_ee = _fk_for_q(robot, q)
        pos_err = target_pos - T_ee[:3, 3]

        if target_rot is not None:
            rot_err = _rotation_error(T_ee[:3, :3], target_rot)
            err = np.concatenate([pos_err, rot_err])
            rows = 6
        else:
            err = pos_err
            rows = 3

        if np.linalg.norm(err) < tol:
            return q, True

        # Jacobian (full 6xN); slice to used rows
        J = _jacobian_for_q(robot, q)[:rows, :]

        # Damped least-squares step
        JJT = J @ J.T
        dq = J.T @ np.linalg.solve(JJT + damping ** 2 * np.eye(rows), err)

        q = q + dq

        # Enforce joint limits
        for i, joint in enumerate(robot.joints):
            q[i] = np.clip(q[i], joint.limits[0], joint.limits[1])

    return q, np.linalg.norm(err) < tol


def _fk_for_q(robot: Robot, q: np.ndarray) -> np.ndarray:
    from .kinematics import forward_kinematics
    return forward_kinematics(robot.joints, q)[-1]


def _jacobian_for_q(robot: Robot, q: np.ndarray) -> np.ndarray:
    from .kinematics import jacobian
    return jacobian(robot.joints, q)
