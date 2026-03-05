"""
Analytical inverse kinematics for the UR5 arm.

Algorithm (8 solutions = 2 shoulder × 2 wrist-flip × 2 elbow)
--------------------------------------------------------------
The UR5 has a "shoulder-wrist" decoupling: the first joint rotates a
vertical plane, and joints 4-5-6 form a non-centred spherical wrist.
The correct decoupling order is:

  1. θ1  – from wrist-centre (x,y) in the horizontal plane.
  2. θ5  – from the z-component of the approach axis in frame 1.
  3. θ6  – from the remaining orientation elements in frame 1.
  4. T14  = T16 · inv(T46)  →  frame-4 position in frame 1.
  5. θ3  – from law of cosines on (r_arm, s_arm) = T14[0,3], T14[1,3].
  6. θ2  – from 2-link geometry.
  7. θ4  – from θ2+θ3+θ4 encoded in T16 z-column.

DH parameters (matching robot.py make_6dof_arm):
    d1=0.089159  d4=0.10915  d5=0.09465  d6=0.0823
    a2=-0.425    a3=-0.39225
    alpha: π/2, 0, 0, π/2, −π/2, 0

Key geometric facts exploited
------------------------------
• Frame-1 y-axis = global z  →  frame-4 y1-coord = a2·s2 + a3·s23  (s_arm).
• z4 ⊥ z1  →  d5 along z4 contributes zero to z1-component of EE.
• T16[2,2] = cos θ5,  T16[2,0] = s5·c6,  T16[2,1] = −s5·s6.
• θ1 constraint: pw_x·sin θ1 − pw_y·cos θ1 = d4
  → θ1 = atan2(pw_y, pw_x) ± atan2(d4, √(pw_x²+pw_y²−d4²)).
"""

import numpy as np
from .kinematics import dh_matrix

# UR5 kinematic constants
_d1 = 0.089159
_d4 = 0.10915
_d5 = 0.09465
_d6 = 0.0823
_a2 = -0.425
_a3 = -0.39225


def _wrap(q: np.ndarray) -> np.ndarray:
    return (q + np.pi) % (2 * np.pi) - np.pi


def _t01_inv(c1: float, s1: float) -> np.ndarray:
    """Inverse of T_0_1(θ1) — maps base frame to frame 1."""
    return np.array([
        [ c1,  s1,  0,  0   ],
        [  0,   0,  1, -_d1 ],
        [ s1, -c1,  0,  0   ],
        [  0,   0,  0,  1   ],
    ])


def _theta6_from_T16(T16: np.ndarray, s5: float) -> float:
    """Extract θ6 from T16[2,0] = s5·c6 and T16[2,1] = −s5·s6."""
    if s5 > 0:
        return np.arctan2(-T16[2, 1], T16[2, 0])
    else:
        return np.arctan2( T16[2, 1], -T16[2, 0])


def _theta234_from_T16(T16: np.ndarray, s5: float) -> float:
    """
    Extract θ2+θ3+θ4 from T16[0,2] = −c234·s5, T16[1,2] = −s234·s5.
    Only valid when |s5| is not near zero.
    """
    if s5 > 0:
        return np.arctan2(-T16[1, 2], -T16[0, 2])
    else:
        return np.arctan2( T16[1, 2],  T16[0, 2])


def ur5_ik(T: np.ndarray) -> list[np.ndarray]:
    """
    Analytical IK for the UR5.

    Parameters
    ----------
    T : (4,4) homogeneous target pose in the base frame.

    Returns
    -------
    List of up to 8 joint-angle arrays (6,), each wrapped to [−π, π].
    Returns an empty list if the target is geometrically unreachable.
    """
    R = T[:3, :3]
    p = T[:3, 3]

    # ── Wrist centre (frame-5 origin) ────────────────────────────────────────
    # Moving d6 back along the approach axis gives the intersection of
    # joints 4 and 5 axes, which is the correct decoupling point for UR5.
    pw = p - _d6 * R[:, 2]
    pw_x, pw_y = pw[0], pw[1]

    solutions = []

    # ── θ1 ───────────────────────────────────────────────────────────────────
    # Constraint: pw_x·sin θ1 − pw_y·cos θ1 = d4
    # → √(pw_x²+pw_y²) · sin(θ1 − φ1) = d4  where φ1 = atan2(pw_y, pw_x)
    # → θ1 = φ1 + arcsin(d4 / R_xy)   (two solutions: +arcsin, π−arcsin)
    r_xy_sq = pw_x**2 + pw_y**2
    if r_xy_sq < _d4**2:
        return []   # wrist centre inside shoulder axis — singular

    phi1 = np.arctan2(pw_y, pw_x)
    phi2 = np.arctan2(_d4, np.sqrt(r_xy_sq - _d4**2))   # = arcsin(d4/R)

    for theta1 in (phi1 + phi2, phi1 + np.pi - phi2):
        c1, s1 = np.cos(theta1), np.sin(theta1)
        T16 = _t01_inv(c1, s1) @ T   # target expressed in frame 1

        # ── θ5 ───────────────────────────────────────────────────────────────
        # T16[2,2] = cos θ5  (z-component of z5 in frame 1)
        c5 = float(np.clip(T16[2, 2], -1.0, 1.0))

        for theta5 in (np.arccos(c5), -np.arccos(c5)):
            s5 = np.sin(theta5)

            # ── θ6 ───────────────────────────────────────────────────────────
            if abs(s5) < 1e-8:
                # Gimbal lock: θ4+θ6 (or θ4−θ6) is determined, not each alone.
                # Fix θ4 to zero; θ6 is resolved when θ4 is computed below.
                theta6 = 0.0
            else:
                theta6 = _theta6_from_T16(T16, s5)

            # ── T14 = T16 · inv(T46) ─────────────────────────────────────────
            # T46 depends only on θ5,θ6 (now known).
            T45 = dh_matrix(_d5, theta5, 0.0, -np.pi / 2)
            T56 = dh_matrix(_d6, theta6, 0.0,  0.0)
            T14 = T16 @ np.linalg.inv(T45 @ T56)

            # Frame-4 position in frame-1:  x1 = r_arm, y1 = s_arm, z1 = d4
            r_arm = T14[0, 3]
            s_arm = T14[1, 3]

            # ── θ2, θ3 from law of cosines ────────────────────────────────────
            D = (r_arm**2 + s_arm**2 - _a2**2 - _a3**2) / (2 * _a2 * _a3)
            if abs(D) > 1.0 + 1e-8:
                continue   # not reachable
            D = np.clip(D, -1.0, 1.0)

            for elbow_sign in (+1, -1):
                theta3 = np.arctan2(elbow_sign * np.sqrt(1.0 - D**2), D)
                c3, s3 = np.cos(theta3), np.sin(theta3)

                A = _a2 + _a3 * c3
                B = _a3 * s3
                theta2 = np.arctan2(A * s_arm - B * r_arm,
                                    A * r_arm + B * s_arm)

                # ── θ4 ───────────────────────────────────────────────────────
                if abs(s5) < 1e-8:
                    # Gimbal lock: use T14 rotation to get θ234, then θ4.
                    theta234 = np.arctan2(T14[1, 0], T14[0, 0])
                else:
                    theta234 = _theta234_from_T16(T16, s5)

                theta4 = theta234 - theta2 - theta3

                q = _wrap(np.array([theta1, theta2, theta3,
                                    theta4, theta5, theta6]))
                solutions.append(q)

    return solutions


def best_ur5_solution(
    T: np.ndarray,
    q_current: np.ndarray,
    joint_limits: list[tuple[float, float]] | None = None,
) -> np.ndarray | None:
    """
    Return the analytical IK solution closest to q_current.

    Parameters
    ----------
    T            : (4,4) target pose.
    q_current    : (6,) current joint angles used as reference.
    joint_limits : list of (lo, hi) per joint; solutions violating limits are
                   discarded.  Pass None to skip limit checking.

    Returns None if no valid solution exists.
    """
    candidates = ur5_ik(T)
    if not candidates:
        return None

    if joint_limits is not None:
        valid = [q for q in candidates
                 if all(lo <= qi <= hi
                        for qi, (lo, hi) in zip(q, joint_limits))]
        candidates = valid or candidates   # fall back if none in limits

    return min(candidates, key=lambda q: np.linalg.norm(q - q_current))
