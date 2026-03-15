# Meca500 Robot Visualisation

Interactive 3D visualisation and control of a Meca500 R3 6-DOF serial manipulator with forward and inverse kinematics.

## Features

- **Forward Kinematics** — joint angle sliders drive the robot pose in real time
- **Inverse Kinematics** — 6-DOF damped least-squares solver (position + ZYX Euler orientation)
- **Draggable IK target** — move the green sphere with the gizmo or use XYZ / alpha-beta-gamma sliders
- **Double-click to type** — double-click any slider value label to enter a number directly
- **Mesh labels toggle** — show/hide object name labels on all meshes
- **STL mesh rendering** — CAD meshes (A0–A6) attached to the FK chain via Blender armature bone transforms

## Quick Start

```bash
python3 -m http.server 8000
```

Open `http://localhost:8000/threejs_scene.html` in a browser.

Requires `robot_scene.glb` (exported from Blender with the Meca500 armature and meshes).

## Project Structure

```
threejs_scene.html       Three.js FK/IK viewer (standalone, loads robot_scene.glb)
robot_scene.glb          glTF binary exported from Blender (armature + meshes)

stl_files/
  A0.stl – A7.stl        Meca500 component meshes (source CAD assets)

Images/
  SideView.png           Reference zero-config side view (XZ plane)
  FrontView.png          Reference zero-config front view (YZ plane)
```

## Meca500 R3 DH Parameters

| Joint | d (mm) | a (mm) | alpha |
|-------|--------|--------|-------|
| 1     | 135    | 0      | -90   |
| 2     | 0      | 135    | 0     |
| 3     | 0      | 38     | -90   |
| 4     | 120    | 0      | 90    |
| 5     | 0      | 0      | -90   |
| 6     | 70     | 0      | 0     |

## IK Solver

The viewer uses a 6x6 geometric Jacobian with damped least-squares (DLS):

- **Position error**: difference between target and end-effector world position
- **Orientation error**: rotation vector from quaternion error (target * current^-1)
- **Convention**: ZYX Euler angles (alpha=Rz, beta=Ry, gamma=Rx)
- Orientation is weighted at 0.3x relative to position to prioritise reach accuracy
