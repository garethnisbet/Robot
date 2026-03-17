# Meca500 Robot Visualisation

Interactive 3D visualisation and control of a Meca500 R3 6-DOF serial manipulator with forward and inverse kinematics, STL import, and collision detection.

## Features

- **Forward Kinematics** — joint angle sliders drive the robot pose in real time
- **Inverse Kinematics** — 6-DOF damped least-squares solver (position + ZYX Euler orientation)
- **Draggable IK target** — move the green sphere with the gizmo or use XYZ / alpha-beta-gamma sliders
- **Double-click to type** — double-click any slider value label to enter a number directly
- **Mesh labels toggle** — show/hide object name labels on all meshes
- **STL mesh rendering** — CAD meshes (A0–A6) attached to the FK chain via Blender armature bone transforms
- **STL import** — load external STL files into the scene with auto-scaling, labels, and per-object colour
- **Persistent STL objects** — imported objects are automatically saved to IndexedDB and restored on page reload, including geometry, transforms, colour, and visibility
- **Object manipulation** — click imported STL objects to select, then move, rotate, or scale with transform gizmos (keyboard: T/R/S, Escape to deselect)
- **Collision detection** — BVH-accelerated triangle-level intersection testing between robot links and imported objects, with red highlight on colliding meshes
- **Remote control API** — two-way WebSocket API for controlling the robot, imported objects, and collision detection from Python, scripts, or any WebSocket client

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

## STL Import & Object Manipulation

Click **Import STL** to load `.stl` files into the scene. Imported objects appear in the panel list with:

- **Colour swatch** — click to change the object colour
- **Name** — click to select the object, double-click to rename
- **Visibility toggle** — show/hide individual objects
- **Remove button** — delete from scene

When an object is selected, a transform gizmo appears. Use the mode buttons or keyboard shortcuts:

| Key | Mode |
|-----|------|
| `T` | Move (translate) |
| `R` | Rotate |
| `S` | Scale |
| `Esc` | Deselect |

STL files in mm are auto-scaled to meters if the bounding box exceeds 1m.

### Persistence

Imported STL objects are automatically persisted in the browser's IndexedDB. All object data is saved and restored across page reloads:

- Raw STL geometry (binary data)
- Position, rotation, and scale transforms
- Colour and visibility state
- Object name

Changes are saved automatically whenever you import, move, rotate, scale, rename, recolour, hide/show, or delete an object. Objects modified via the remote control API (`setObject`) are also persisted. To clear all saved objects, remove them from the panel list or clear site data in your browser.

## Collision Detection

Toggle **Collision: ON/OFF** in the panel. When enabled, the viewer tests for triangle-level intersections between all robot link meshes and imported STL objects using a two-phase approach:

1. **Broad phase** — axis-aligned bounding box (AABB) check to quickly eliminate distant pairs
2. **Narrow phase** — BVH (Bounding Volume Hierarchy) accelerated triangle-triangle intersection via [three-mesh-bvh](https://github.com/gkjohnson/three-mesh-bvh)

Colliding meshes are highlighted red and collision pairs are listed in the panel. Meshes return to their original colour when the collision clears.

## Remote Control

The WebSocket API at `ws://localhost:8000/ws` allows any client to control the robot and read its state in real time.

### Interactive Client

```bash
python3 remote_control.py
```

**Robot commands:**

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

**Object commands:**

| Command | Example | Description |
|---------|---------|-------------|
| `objects` | `objects` | List all imported STL objects with transforms |
| `obj` | `obj MyPart` or `obj #0` | Get object details (by name or index) |
| `objpos` | `objpos MyPart 100 50 0` | Set object position (mm) |
| `objrot` | `objrot #0 0 0 45` | Set object rotation (degrees) |
| `objscale` | `objscale MyPart 2` | Set uniform scale |
| `objscale` | `objscale #0 1 1 2` | Set per-axis scale (sx sy sz) |

**Collision commands:**

| Command | Example | Description |
|---------|---------|-------------|
| `collision` | `collision on` | Enable/disable collision detection (or toggle) |
| `collisions` | `collisions` | Get current collision pairs |

### API Protocol (JSON over WebSocket)

**Robot commands:**
```json
{"cmd": "getState"}
{"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}
{"cmd": "home"}
{"cmd": "setMode", "mode": "IK"}
{"cmd": "setIKTarget", "position": [190, 0, 308], "orientation": [0, 0, 0]}
{"cmd": "moveTo", "position": [150, 100, 300], "orientation": [45, 0, 0]}
```

**Object commands:**
```json
{"cmd": "listObjects"}
{"cmd": "getObject", "object": "MyPart"}
{"cmd": "getObject", "index": 0}
{"cmd": "setObject", "object": "MyPart", "position": [100, 50, 0]}
{"cmd": "setObject", "index": 0, "rotation": [0, 0, 45], "scale": 2}
{"cmd": "setObject", "object": "MyPart", "position": [100, 50, 0], "rotation": [0, 0, 45], "scale": [1, 1, 2], "visible": true}
```

**Collision commands:**
```json
{"cmd": "setCollision", "enabled": true}
{"cmd": "getCollisions"}
```

Positions are in mm, angles in degrees, using Z-up robot convention.

**State response:**
```json
{
  "type": "state",
  "joints": [0, -30, 60, 0, 45, 90],
  "eePosition": [190.0, 0.0, 308.0],
  "eeOrientation": [0.0, 0.0, 0.0],
  "mode": "FK",
  "ikError": null,
  "collisionEnabled": true,
  "collision": true,
  "collisions": [{"link": "L2", "object": "MyPart"}]
}
```

**Object response:**
```json
{
  "type": "object",
  "index": 0,
  "name": "MyPart",
  "position": [100.0, 50.0, 0.0],
  "rotation": [0.0, 0.0, 45.0],
  "scale": [1.0, 1.0, 1.0],
  "visible": true
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

        # Move an imported object and check collisions
        await ws.send(json.dumps({"cmd": "setCollision", "enabled": True}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "setObject", "object": "MyPart", "position": [100, 0, 200]}))
        result = json.loads(await ws.recv())
        print(result)  # object transform confirmation

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
