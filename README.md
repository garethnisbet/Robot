# Robot & Device Visualisation

Interactive 3D visualisation and control for robots and scientific instruments. Config-driven — works with any device defined by a JSON config and GLB model. Includes forward and inverse kinematics, mesh import, collision detection, and a remote control API.

## Supported Devices

| Device | Config | Joints | Description |
|--------|--------|--------|-------------|
| Meca500 R3 | `robot_config.json` | 6 (serial) | 6-DOF industrial manipulator |
| i16 Diffractometer | `i16_config.json` | 10 movable (branching) | Diamond Light Source 6-circle diffractometer with merlin and crystal detectors |

New devices can be added from Blender scenes using the `/build-viewer` skill (see [Adding New Devices](#adding-new-devices)).

## Features

- **Config-driven viewer** — a single generic `threejs_scene.html` viewer loads any device via JSON config
- **Model selector** — dropdown to switch between available devices without changing URLs
- **Auto-fit camera** — camera automatically frames the loaded model on startup
- **Forward Kinematics** — joint angle sliders for all movable joints (fixed kinematic links are hidden)
- **Inverse Kinematics** — 6-DOF damped least-squares solver (position + ZYX Euler orientation)
- **Branching kinematic chains** — supports devices with multiple independent chains and sub-branches (e.g., i16 has gamma and mu chains with merlin/crystal sub-branches)
- **Draggable IK target** — move the green sphere with the gizmo or use XYZ / alpha-beta-gamma sliders
- **Orientation gizmo** — visual end-effector orientation indicator showing the current tool frame axes
- **Double-click to type** — double-click any slider value label to enter a number directly
- **Mesh labels toggle** — show/hide object name labels on all meshes
- **Mesh import** — load external STL, OBJ, PLY, and GLB/GLTF files into the scene with auto-scaling, labels, and per-object colour
- **Primitive objects** — add cube, sphere, and cylinder primitives directly from the toolbar
- **Persistent objects** — imported and primitive objects are automatically saved to IndexedDB and restored on page reload
- **Object manipulation** — click objects to select, then move, rotate, or scale with transform gizmos (keyboard: T/R/S, Escape to deselect)
- **Parent-child linking** — attach objects to device links so they follow the kinematic chain
- **Self-collision detection** — BVH-accelerated triangle-level intersection testing between device links, using kinematic adjacency to skip physically connected parts
- **Collision detection** — intersection testing between device links and imported scene objects, with red highlight on colliding meshes
- **Remote control API** — two-way WebSocket API for controlling any device from Python or any WebSocket client

## Quick Start

**Standalone (no remote control):**
```bash
python3 -m http.server 8000
```
Open `http://localhost:8000/threejs_scene.html` in a browser. Use the dropdown to switch devices, or specify a config: `?config=i16_config.json`.

**With remote control API:**
```bash
pip3 install aiohttp websockets
python3 server.py --config robot_config.json
```
Open `http://localhost:8080` in a browser. The green dot in the top-left corner shows WebSocket connection status.

## Adding New Devices

New devices can be added from Blender scenes using the `/build-viewer` Claude Code skill. The process:

1. Set up an armature in Blender with bones for each joint/kinematic link
2. Parent meshes to bones
3. Set rotation constraints via quaternion locks (WXYZ)
4. Run `/build-viewer` to extract bone transforms, determine rotation axes, export a GLB, and generate a config JSON
5. Add the new config filename to the `configFiles` array in `threejs_scene.html`

### Config File Structure

```json
{
  "name": "Device Name",
  "model": "device_scene.glb",
  "joints": [
    {
      "name": "joint_name",
      "restPos": [x, y, z],
      "restQuat": [w, x, y, z],
      "axis": [x, y, z],
      "limits": [-180, 180],
      "parent": 0,
      "fixed": false
    }
  ],
  "links": [
    { "name": "mesh_name", "label": "Display Name", "joint": 0 }
  ],
  "demoPose": [0, 45, -90]
}
```

Key fields:
- **joints**: one entry per bone — `restPos`/`restQuat` from Blender (C_gltf converted), `axis` from quaternion lock analysis, `parent` index (-1 for roots), `fixed` for non-movable kinematic links
- **links**: maps GLB mesh names (sanitized: spaces→underscores) to joint indices
- **demoPose**: joint angles in degrees for the demo button (one per joint, fixed joints = 0)

## Objects

### Mesh Import

Click **Import Mesh** to load files into the scene. Supported formats:

| Format | Extension | Notes |
|--------|-----------|-------|
| STL | `.stl` | Binary or ASCII; auto-scaled if bounding box > 1 m |
| OBJ | `.obj` | Geometry only; no MTL material files |
| PLY | `.ply` | Binary and ASCII; vertex colours preserved if present |
| GLB / GLTF | `.glb`, `.gltf` | Full scene hierarchy, materials, and textures |

### Primitives

Click **Cube**, **Sphere**, or **Cylinder** to add a primitive shape. Primitives behave identically to imported objects — they can be moved, coloured, parented, and are persisted across reloads.

### Transform Gizmos

| Key | Mode |
|-----|------|
| `T` | Move (translate) |
| `R` | Rotate |
| `S` | Scale |
| `Esc` | Deselect |

### Parent-Child Linking

Use the **Parent** dropdown to attach objects to device links. Parented objects follow the kinematic chain. Local transforms are preserved when reparenting.

## Collision Detection

Toggle **Collision: ON/OFF** in the panel. The viewer tests for triangle-level intersections using:

1. **Broad phase** — AABB check to eliminate distant pairs
2. **Narrow phase** — BVH-accelerated triangle-triangle intersection via [three-mesh-bvh](https://github.com/gkjohnson/three-mesh-bvh)

Self-collision between device links uses kinematic adjacency analysis — links sharing the same joint or connected through a parent-child relationship are skipped. Links on separate branches (e.g., gamma chain vs mu chain on the i16) are always checked.

## Remote Control

The WebSocket API at `ws://localhost:8080/ws` allows any client to control the device and read its state in real time.

### Interactive Client

```bash
python3 remote_control.py --config robot_config.json       # Meca500
python3 remote_control.py --config i16_config.json          # i16 diffractometer
python3 remote_control.py --url ws://192.168.1.100:8080/ws  # remote server
```

The client reads the device config to determine joint names and count. The prompt, help text, and tab completion adapt to the loaded device.

**Device commands:**

| Command | Example | Description |
|---------|---------|-------------|
| `state` | `state` | Request current device state |
| `home` | `home` | All joints to 0 |
| `fk` | `fk` | Switch to FK mode |
| `ik` | `ik` | Switch to IK mode |
| `joints` | `joints 45 -90 0 0 30 0` | Set all movable joint angles (degrees) |
| `joint` | `joint gamma 45` | Set a single joint by name (tab-completes) |
| `move` | `move 150 100 300 45 0 0` | Switch to IK and move to position (mm) + orientation (deg) |
| `target` | `target 190 0 308` | Set IK target without switching mode |
| `demo` | `demo` | Run the config's demo pose |

**Object commands:**

| Command | Example | Description |
|---------|---------|-------------|
| `objects` | `objects` | List all imported objects with transforms |
| `obj` | `obj MyPart` or `obj #0` | Get object details (by name or index) |
| `objpos` | `objpos MyPart 100 50 0` | Set object position (mm) |
| `objrot` | `objrot #0 0 0 45` | Set object rotation (degrees) |
| `objscale` | `objscale MyPart 2` | Set uniform scale |
| `objvis` | `objvis MyPart on` | Show/hide object |
| `objresetrot` | `objresetrot MyPart` | Reset rotation |
| `objresetscale` | `objresetscale #0` | Reset scale |

**Collision commands:**

| Command | Example | Description |
|---------|---------|-------------|
| `collision` | `collision on` | Enable/disable collision detection |
| `collisions` | `collisions` | Get current collision pairs |

### API Protocol (JSON over WebSocket)

**Device commands:**
```json
{"cmd": "getState"}
{"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}
{"cmd": "setSingleJoint", "index": 1, "angle": -30}
{"cmd": "home"}
{"cmd": "setMode", "mode": "IK"}
{"cmd": "setIKTarget", "position": [190, 0, 308], "orientation": [0, 0, 0]}
{"cmd": "moveTo", "position": [150, 100, 300], "orientation": [45, 0, 0]}
```

**Object commands:**
```json
{"cmd": "listObjects"}
{"cmd": "getObject", "object": "MyPart"}
{"cmd": "setObject", "object": "MyPart", "position": [100, 50, 0], "rotation": [0, 0, 45]}
{"cmd": "resetObjectRotation", "object": "MyPart"}
{"cmd": "resetObjectScale", "index": 0}
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
  "collision": false,
  "collisions": []
}
```

### Custom Client Example (Python)

```python
import asyncio, json, websockets

async def main():
    async with websockets.connect("ws://localhost:8080/ws") as ws:
        # Set joint angles
        await ws.send(json.dumps({"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}))
        state = json.loads(await ws.recv())
        print(state["eePosition"])

        # Set a single joint by index
        await ws.send(json.dumps({"cmd": "setSingleJoint", "index": 1, "angle": -45}))

        # Enable collision detection
        await ws.send(json.dumps({"cmd": "setCollision", "enabled": True}))

asyncio.run(main())
```

## Deployment

### Docker

```bash
docker build -t robot-visualisation .
docker run -p 8080:8080 robot-visualisation
```

### Kubernetes (Helm)

```bash
helm install robot-vis ./helm
```

See `helm/values.yaml` for configuration.

## Project Structure

```
threejs_scene.html       Generic config-driven Three.js FK/IK viewer
server.py                WebSocket + HTTP server for remote control API
remote_control.py        Interactive command-line remote control client (any device)
robot_config.json        Meca500 R3 device config
i16_config.json          i16 diffractometer device config
robot_scene.glb          Meca500 GLB model
i16_scene.glb            i16 diffractometer GLB model
.claude/commands/
  build-viewer.md        Skill for building new device viewers from Blender scenes
Dockerfile               Multi-stage container build
helm/                    Kubernetes Helm chart
```

## IK Solver

The viewer uses a 6×N geometric Jacobian with damped least-squares (DLS):

- **Position error**: difference between target and end-effector world position
- **Orientation error**: rotation vector from quaternion error (target × current⁻¹)
- **Convention**: ZYX Euler angles (alpha=Rz, beta=Ry, gamma=Rx)
- Orientation is weighted at 0.3× relative to position to prioritise reach accuracy
- For N < 6 joints: underdetermined for full 6-DOF; for N > 6: redundancy handled naturally by DLS
