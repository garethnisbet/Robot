from dataclasses import dataclass, field
from typing import Literal, Tuple
import numpy as np

from .kinematics import forward_kinematics, jacobian


@dataclass
class Joint:
    """
    DH parameters for a single joint.

    d            : link offset along previous z-axis
    a            : link length along x-axis
    alpha        : twist angle between z-axes (radians)
    theta_offset : fixed offset added to the variable joint angle (radians)
    joint_type   : "revolute" or "prismatic"
    limits       : (min, max) in radians (revolute) or metres (prismatic)
    name         : optional label
    """
    d: float = 0.0
    a: float = 0.0
    alpha: float = 0.0
    theta_offset: float = 0.0
    joint_type: Literal["revolute", "prismatic"] = "revolute"
    limits: Tuple[float, float] = (-np.pi, np.pi)
    name: str = ""


class Robot:
    """A serial manipulator defined by a sequence of DH joints."""

    def __init__(self, joints: list[Joint], name: str = "Robot"):
        self.joints = joints
        self.name = name
        self._q = np.zeros(len(joints))

    @property
    def n_dof(self) -> int:
        return len(self.joints)

    @property
    def q(self) -> np.ndarray:
        return self._q.copy()

    @q.setter
    def q(self, values: np.ndarray):
        values = np.asarray(values, dtype=float)
        if values.shape != (self.n_dof,):
            raise ValueError(f"Expected {self.n_dof} joint values, got {values.shape}")
        for i, (joint, v) in enumerate(zip(self.joints, values)):
            lo, hi = joint.limits
            values[i] = np.clip(v, lo, hi)
        self._q = values

    def set_joint(self, index: int, value: float):
        q = self._q.copy()
        q[index] = value
        self.q = q

    def transforms(self) -> list[np.ndarray]:
        """All homogeneous transforms T_0_i for i=0..n."""
        return forward_kinematics(self.joints, self._q)

    def joint_positions(self) -> np.ndarray:
        """Nx3 array of joint frame origins, starting with the base."""
        return np.array([T[:3, 3] for T in self.transforms()])

    def end_effector_pose(self) -> np.ndarray:
        """4x4 homogeneous transform of the end-effector in the base frame."""
        return self.transforms()[-1]

    def jacobian(self) -> np.ndarray:
        return jacobian(self.joints, self._q)


# ---------------------------------------------------------------------------
# Preset robot configurations
# ---------------------------------------------------------------------------

def make_3dof_planar(link_lengths=(1.0, 0.8, 0.6)) -> Robot:
    """3-DOF planar arm (all joints rotate about z)."""
    l1, l2, l3 = link_lengths
    joints = [
        Joint(d=0, a=l1, alpha=0, name="Joint 1"),
        Joint(d=0, a=l2, alpha=0, name="Joint 2"),
        Joint(d=0, a=l3, alpha=0, name="Joint 3"),
    ]
    return Robot(joints, name="3-DOF Planar Arm")


def make_6dof_arm() -> Robot:
    """Meca500 (R3) 6-DOF arm.

    DH parameters derived from the Meca500 user manual (Figure 3 / Table 1):
      d1=135 mm, a2=135 mm, a3=38 mm, d4=120 mm, d6=70 mm
    """
    d1, d4, d6 = 0.135, 0.120, 0.070
    a2, a3 = 0.135, 0.038
    joints = [
        Joint(d=d1, a=0,  alpha=-np.pi/2, name="Joint 1",
              limits=(-np.radians(175), np.radians(175))),
        Joint(d=0,  a=a2, alpha=0,        name="Joint 2",
              theta_offset=-np.pi/2,
              limits=(-np.radians(70),  np.radians(90))),
        Joint(d=0,  a=a3, alpha=np.pi/2,  name="Joint 3",
              limits=(-np.radians(135), np.radians(70))),
        Joint(d=d4, a=0,  alpha=-np.pi/2, name="Wrist 1",
              limits=(-np.radians(170), np.radians(170))),
        Joint(d=0,  a=0,  alpha=np.pi/2,  name="Wrist 2",
              limits=(-np.radians(115), np.radians(115))),
        Joint(d=d6, a=0,  alpha=0,        name="Wrist 3",
              limits=(-np.pi, np.pi)),
    ]
    return Robot(joints, name="Meca500 (R3)")
