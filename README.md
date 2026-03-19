# Meca500 Robot Visualisation

Interactive 3D visualisation and control of a Meca500 R3 6-DOF serial manipulator with forward and inverse kinematics, STL import, and collision detection.

## Features

- **Forward Kinematics** — joint angle sliders drive the robot pose in real time
- **Inverse Kinematics** — 6-DOF damped least-squares solver (position + ZYX Euler orientation)
- **Draggable IK target** — move the green sphere with the gizmo or use XYZ / alpha-beta-gamma sliders
- **Orientation gizmo** — visual end-effector orientation indicator showing the current tool frame axes
- **Double-click to type** — double-click any slider value label to enter a number directly
- **Mesh labels toggle** — show/hide object name labels on all meshes
- **STL mesh rendering** — CAD meshes (A0–A6) attached to the FK chain via Blender armature bone transforms
- **Mesh import** — load external STL, OBJ, and GLB/GLTF files into the scene with auto-scaling, labels, and per-object colour
- **Primitive objects** — add cube, sphere, and cylinder primitives directly from the toolbar
- **Persistent objects** — imported and primitive objects are automatically saved to IndexedDB and restored on page reload, including geometry, transforms, colour, and visibility
- **Object manipulation** — click objects to select, then move, rotate, or scale with transform gizmos (keyboard: T/R/S, Escape to deselect)
- **Lock aspect ratio** — toggle to constrain scaling to uniform across all axes
- **Reset transforms** — reset an object's rotation or scale back to default with one click
- **Parent-child linking** — attach objects to robot links so they follow the kinematic chain
- **Collision detection** — BVH-accelerated triangle-level intersection testing between robot links and scene objects, with red highlight on colliding meshes
- **Remote control API** — two-way WebSocket API for controlling the robot, objects, and collision detection from Python, scripts, or any WebSocket client

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

## Objects

### Mesh Import

Click **Import Mesh** to load files into the scene. Supported formats:

| Format | Extension | Notes |
|--------|-----------|-------|
| STL | `.stl` | Binary or ASCII; auto-scaled if bounding box > 1 m |
| OBJ | `.obj` | Geometry only; no MTL material files |
| GLB / GLTF | `.glb`, `.gltf` | Full scene hierarchy, materials, and textures (GLB self-contained; external GLTF loads geometry only) |

All formats are auto-scaled to meters if the bounding box exceeds 1 m, and are persisted in IndexedDB across reloads.

### Primitives

Click **Cube**, **Sphere**, or **Cylinder** to add a primitive shape to the scene. Primitives behave identically to imported STL objects — they can be moved, coloured, parented, and are persisted across reloads.

### Object Panel

All scene objects appear in the panel list with:

- **Colour swatch** — click to change the object colour
- **Name** — click to select the object, double-click to rename
- **Visibility toggle** — show/hide individual objects
- **Remove button** — delete from scene

### Transform Gizmos

When an object is selected, a transform gizmo appears. Use the mode buttons or keyboard shortcuts:

| Key | Mode |
|-----|------|
| `T` | Move (translate) |
| `R` | Rotate |
| `S` | Scale |
| `Esc` | Deselect |

Additional controls in the selection panel:

- **Lock aspect ratio** — when checked, dragging any scale axis applies the same scale factor to all three axes, preserving the object's proportions
- **Reset Rotation** — returns the object to zero rotation (Euler 0, 0, 0)
- **Reset Scale** — returns the object to unit scale (1, 1, 1)

### Parent-Child Linking

Use the **Parent** dropdown to attach an object to a robot link (L0–L5: Base through Wrist 2). Parented objects follow the kinematic chain — when the robot moves, the attached object moves with its parent link. Local transforms are preserved when reparenting.

### Persistence

All scene objects (imported STL and primitives) are automatically persisted in the browser's IndexedDB. Saved data includes:

- Raw geometry (binary buffer for STL/GLB, text for OBJ, type identifier for primitives)
- Position, rotation, and scale transforms
- Colour and visibility state
- Object name and parent link

Changes are saved automatically whenever you import, create, move, rotate, scale, rename, recolour, reparent, hide/show, or delete an object. Objects modified via the remote control API (`setObject`) are also persisted. To clear all saved objects, remove them from the panel list or clear site data in your browser.

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

## Deployment

### Docker

```bash
docker build -t robot-visualisation .
docker run -p 8080:8080 robot-visualisation
```

The image uses a multi-stage build on Python 3.12-slim, runs as a non-root user, and exposes port 8080.

### Kubernetes (Helm)

```bash
helm install robot-vis ./helm
```

Deploys with ClusterIP service, nginx ingress, and TLS. See `helm/values.yaml` for configuration.

## Project Structure

```
threejs_scene.html       Three.js FK/IK viewer (standalone, loads robot_scene.glb)
server.py                WebSocket + HTTP server for remote control API
remote_control.py        Interactive command-line remote control client
robot_scene.glb          glTF binary exported from Blender (armature + meshes)
Dockerfile               Multi-stage container build
helm/                    Kubernetes Helm chart

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
