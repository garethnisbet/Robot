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
import numpy as np

from robot.robot import make_3dof_planar, make_6dof_arm, Robot, Joint
from visualisation.visualiser import RobotVisualiser


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
    elif args.robot == "6dof":
        robot = make_6dof_arm()
    else:
        robot = make_custom_robot()

    print(f"Loaded: {robot.name}  ({robot.n_dof} DOF)")
    print("  FK mode  : use joint sliders to control the arm")
    print("  IK mode  : click in the 3D view to set a target position")
    print("  Reset    : zero all joints\n")

    vis = RobotVisualiser(robot)
    vis.show()


if __name__ == "__main__":
    main()
