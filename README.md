# Meca500 Robot Visualisation

Interactive 3D visualisation and control of a Meca500 R3 6-DOF serial manipulator with forward and inverse kinematics.

## Features

- **Forward Kinematics** — joint angle sliders drive the robot pose in real time
- **Inverse Kinematics** — 6-DOF damped least-squares solver (position + ZYX Euler orientation)
- **Draggable IK target** — move the green sphere with the gizmo or use XYZ / alpha-beta-gamma sliders
- **Double-click to type** — double-click any slider value label to enter a number directly
- **Mesh labels toggle** — show/hide object name labels on all meshes
- **STL mesh rendering** — CAD meshes (A0–A6) attached to the FK chain via Blender armature bone transforms
- **Remote control API** — two-way WebSocket API for controlling the robot from Python, scripts, or any WebSocket client

## Quick Start

**Standalone (no remote control):**
```bash
python3 -m http.server 8000
```
Open `http://localhost:8000/threejs_scene.html` in a browser.

**With remote control API:**
```bash
pip3 install aiohttp websockets
python3 server.py
```
Open `http://localhost:8000` in a browser. The green dot in the top-left corner shows WebSocket connection status.

## Remote Control

The WebSocket API at `ws://localhost:8000/ws` allows any client to control the robot and read its state in real time.

### Interactive Client

```bash
python3 remote_control.py
```

Commands:

| Command | Example | Description |
|---------|---------|-------------|
| `state` | `state` | Request current robot state |
| `home` | `home` | All joints to 0 |
| `fk` | `fk` | Switch to FK mode |
| `ik` | `ik` | Switch to IK mode |
| `joints` | `joints 0 -30 60 0 45 90` | Set joint angles (degrees) |
| `move` | `move 150 100 300 45 0 0` | Switch to IK and move to position (mm) + orientation (deg) |
| `target` | `target 190 0 308` | Set IK target without switching mode |
| `demo` | `demo` | Run an animated demo sequence |

### API Protocol (JSON over WebSocket)

**Commands (send to the robot):**
```json
{"cmd": "getState"}
{"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}
{"cmd": "home"}
{"cmd": "setMode", "mode": "IK"}
{"cmd": "setIKTarget", "position": [190, 0, 308], "orientation": [0, 0, 0]}
{"cmd": "moveTo", "position": [150, 100, 300], "orientation": [45, 0, 0]}
```

Positions are in mm, angles in degrees, using Z-up robot convention.

**State (received from the robot):**
```json
{
  "type": "state",
  "joints": [0, -30, 60, 0, 45, 90],
  "eePosition": [190.0, 0.0, 308.0],
  "eeOrientation": [0.0, 0.0, 0.0],
  "mode": "FK",
  "ikError": null
}
```

### Custom Client Example (Python)

```python
import asyncio, json, websockets

async def main():
    async with websockets.connect("ws://localhost:8000/ws") as ws:
        # Set joint angles
        await ws.send(json.dumps({"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}))
        state = json.loads(await ws.recv())
        print(state["eePosition"])  # [190.0, 0.0, 308.0]

asyncio.run(main())
```

## Project Structure

```
threejs_scene.html       Three.js FK/IK viewer (standalone, loads robot_scene.glb)
server.py                WebSocket + HTTP server for remote control API
remote_control.py        Interactive command-line remote control client
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
