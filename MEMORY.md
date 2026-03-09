# RobotVisualisation Project

## Overview
Serial manipulator visualiser using PyQtGraph (Qt5, pyqtgraph 0.14.0).
Run: `python3 main.py --robot 6dof`

## Key Files
- `main.py` — entry point, argparse (`--robot 3dof|6dof|custom`)
- `robot/robot.py` — `Joint` (DH params), `Robot`, `make_3dof_planar()`, `make_6dof_arm()`
- `robot/kinematics.py` — FK and Jacobian
- `robot/ik.py` — numerical IK
- `robot/ur5_ik.py` — analytical IK (`best_ur5_solution`)
- `visualisation/visualiser.py` — PyQtGraph 3D GUI

## Current 6-DOF Configuration (robot/robot.py)
UR5-style DH parameters with analytical IK attached:
```python
d1, d4, d5, d6 = 0.089159, 0.0, 0.09465, 0.0823   # d4=0 (L4 zero length)
a2, a3 = -0.425, -0.39225
joints = [
    Joint(d=d1, a=0,  alpha= pi/2, name="Shoulder Pan"),   # L1 vertical
    Joint(d=0,  a=a2, alpha=0,     name="Shoulder Lift"),   # L2 along x
    Joint(d=0,  a=a3, alpha=0,     name="Elbow"),           # L3 along x
    Joint(d=d4, a=0,  alpha= pi/2, name="Wrist 1"),         # L4 zero length
    Joint(d=d5, a=0,  alpha=-pi/2, name="Wrist 2"),         # L5 spherical wrist
    Joint(d=d6, a=0,  alpha=0,     name="Wrist 3"),         # L6 spherical wrist
]
```
Wrist is a proper spherical wrist (ZYZ). d4=0 means joints 3 and 4 share the same position.

## Link Label Overlay (visualiser.py)
- `GLTextItem` doesn't work with Qt5 QOpenGLWidget (QPainter inside paintGL fails)
- Solution: `LabelledGLViewWidget` subclasses `gl.GLViewWidget`, draws labels in `paintEvent`
- Projects 3D midpoints to 2D using `projectionMatrix()` + `viewMatrix()`
- Yellow labels with dark outline: L1–L6 at midpoint of each link

## DH Convention Notes
- With links along x (a-parameter), joint z-axes are confined to the yz-plane
- Cannot achieve forearm-roll axis (along x) using DH alpha alone
- Proper spherical wrist uses d-offsets for joints 4-6 (UR5 style)
