"""
Serial Manipulator Control & Visualisation
==========================================

Usage:
    python main.py                  # default: 3-DOF planar arm
    python main.py --robot 6dof     # 6-DOF arm (UR5-style)
    python main.py --robot custom   # edit make_custom_robot() below

Controls:
    - FK mode: drag joint sliders to set angles; arm updates live
    - IK mode: click in the 3D view to set a target position;
               the robot solves IK and moves to that target
    - Reset button: zero all joint angles
"""

import argparse
import os
import numpy as np

from robot.robot import make_3dof_planar, make_6dof_arm, Robot, Joint
from visualisation.visualiser import RobotVisualiser

_STL_DIR = os.path.join(os.path.dirname(__file__), 'stl_files')

# Rotation corrections to align each STL from its CAD coordinate system
# into the DH local frame.  T_correction is applied as:
#   world_transform = T_frame @ T_correction
# so the STL vertices are first rotated into the DH frame, then placed in the world.

_I = np.eye(4)

# A0 (base): Rz(180°)
_T0 = np.array([[-1,  0,  0, 0],
                [ 0, -1,  0, 0],
                [ 0,  0,  1, 0],
                [ 0,  0,  0, 1]], dtype=float)

# A1 (shoulder): Rx(180°) applied to flip upright
_T1 = np.array([[-1,  0,  0, 0],
                [ 0,  0, -1, 0],
                [ 0, -1,  0, 0],
                [ 0,  0,  0, 1]], dtype=float)

# A2 (upper arm): Rx(180°) @ old_T2
_T2 = np.array([[ 0,  0, -1, 0],
                [-1,  0,  0, 0],
                [ 0,  1,  0, 0],
                [ 0,  0,  0, 1]], dtype=float)

# A3_4 (forearm + wrist housing): world Ry(180°) then world Rz(180°) correction
_T3 = np.array([[ 0,  0,  1, 0],
                [ 0, -1,  0, 0],
                [ 1,  0,  0, 0],
                [ 0,  0,  0, 1]], dtype=float)

# A5 (wrist 1): Rz(180°) @ Ry(90°) @ Rz(90°)
_T5 = np.array([[ 0,  0, -1, 0],
                [-1,  0,  0, 0],
                [ 0,  1,  0, 0],
                [ 0,  0,  0, 1]], dtype=float)

# A6 (wrist 2): Rx(180°) @ Ry(90°) @ prev correction
_T6 = np.array([[ 0,  0, -1, 0],
                [ 0,  1,  0, 0],
                [ 1,  0,  0, 0],
                [ 0,  0,  0, 1]], dtype=float)

# A7 (wrist 3 / end effector): world Rz(90°) correction
_T7 = np.array([[ 0,  1,  0, 0],
                [-1,  0,  0, 0],
                [ 0,  0,  1, 0],
                [ 0,  0,  0, 1]], dtype=float)

# (path, DH frame index, T_correction): frame 0 = world/base, frame i = after joint i
MECA500_STL_MAP = [
    (os.path.join(_STL_DIR, 'A0.stl'),   0, _T0),  # base
    (os.path.join(_STL_DIR, 'A1.stl'),   1, _T1),  # after joint 1
    (os.path.join(_STL_DIR, 'A2.stl'),   2, _T2),  # after joint 2
    (os.path.join(_STL_DIR, 'A3_4.stl'), 3, _T3),  # after joint 3 (forearm + wrist housing)
    (os.path.join(_STL_DIR, 'A5.stl'),   4, _T5),  # wrist 1 (after joint 4)
    (os.path.join(_STL_DIR, 'A6.stl'),   5, _T6),  # wrist 2 (after joint 5)
    (os.path.join(_STL_DIR, 'A7.stl'),   6, _T7),  # wrist 3 / end effector (after joint 6)
]


def make_custom_robot() -> Robot:
    """
    Define your own robot here.

    Each Joint takes standard DH parameters:
        d            - link offset (m)
        a            - link length (m)
        alpha        - twist angle (rad)
        theta_offset - constant angle offset (rad)
        joint_type   - "revolute" or "prismatic"
        limits       - (min, max) in rad or m
        name         - display label
    """
    joints = [
        Joint(d=0.5, a=0,   alpha=0,        name="Prismatic base", joint_type="prismatic",
              limits=(0.0, 1.0)),
        Joint(d=0,   a=1.0, alpha=0,        name="Shoulder"),
        Joint(d=0,   a=0.8, alpha=0,        name="Elbow"),
        Joint(d=0,   a=0.4, alpha=np.pi/2,  name="Wrist"),
    ]
    return Robot(joints, name="Custom Robot")


def parse_args():
    p = argparse.ArgumentParser(description="Serial manipulator visualiser")
    p.add_argument(
        "--robot",
        choices=["3dof", "6dof", "custom"],
        default="3dof",
        help="Which robot preset to load (default: 3dof)",
    )
    return p.parse_args()


def main():
    args = parse_args()

    if args.robot == "3dof":
        robot = make_3dof_planar()
        vis = RobotVisualiser(robot)
    elif args.robot == "6dof":
        robot = make_6dof_arm()
        vis = RobotVisualiser(robot, stl_map=MECA500_STL_MAP)  # tuples are (path, frame_idx, T_correction)
    else:
        robot = make_custom_robot()
        vis = RobotVisualiser(robot)

    print(f"Loaded: {robot.name}  ({robot.n_dof} DOF)")
    print("  FK mode  : use joint sliders to control the arm")
    print("  IK mode  : click in the 3D view to set a target position")
    print("  Reset    : zero all joints\n")

    vis.show()


if __name__ == "__main__":
    main()
