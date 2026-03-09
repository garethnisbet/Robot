"""
DH-parameter-based forward kinematics and Jacobian computation.

Standard DH convention: T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
"""

import numpy as np


def dh_matrix(d: float, theta: float, a: float, alpha: float) -> np.ndarray:
    """Return the 4x4 homogeneous transform for one DH link."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ],
    ])


def forward_kinematics(joints, joint_angles: np.ndarray):
    """
    Compute FK for a list of Joint objects and joint angles.

    Returns
    -------
    transforms : list of np.ndarray, shape (4,4)
        T_0_i for i = 0..n  (base frame is identity at index 0)
    """
    T = np.eye(4)
    transforms = [T.copy()]
    for joint, q in zip(joints, joint_angles):
        theta = q + joint.theta_offset
        T = T @ dh_matrix(joint.d, theta, joint.a, joint.alpha)
        transforms.append(T.copy())
    return transforms


def jacobian(joints, joint_angles: np.ndarray) -> np.ndarray:
    """
    Compute the 6xN geometric Jacobian (linear + angular velocity).

    Rows 0-2: linear velocity components
    Rows 3-5: angular velocity components
    """
    transforms = forward_kinematics(joints, joint_angles)
    n = len(joints)
    J = np.zeros((6, n))

    p_ee = transforms[-1][:3, 3]

    for i, joint in enumerate(joints):
        T_i = transforms[i]
        z_i = T_i[:3, 2]   # z-axis of frame i
        p_i = T_i[:3, 3]   # origin of frame i

        if joint.joint_type == "revolute":
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i
        elif joint.joint_type == "prismatic":
            J[:3, i] = z_i
            J[3:, i] = np.zeros(3)

    return J
