# Robot & Device Visualisation

Interactive 3D visualisation and control for robots and scientific instruments. Config-driven — works with any device defined by a JSON config and GLB model. Includes forward and inverse kinematics, mesh import, collision detection, and a remote control API.

## Supported Devices

| Device | Config | Joints | Description |
|--------|--------|--------|-------------|
| Meca500 R3 | `robot_config.json` | 6 (serial) | 6-DOF compact industrial manipulator |
| i16 Diffractometer | `i16_config.json` | 10 movable (branching) | Diamond Light Source 6-circle diffractometer with merlin and crystal detectors |
| Yaskawa GP225 | `gp225_config.json` | 6 (serial) | 6-DOF heavy-payload industrial robot |
| Yaskawa GP280 | `gp280_config.json` | 6 (serial) | 6-DOF heavy-payload industrial robot |

New devices can be added from Blender scenes using the `/build-viewer` skill (see [Adding New Devices](#adding-new-devices)).

## Features

- **Config-driven viewer** — a single generic `threejs_scene.html` viewer loads any device via JSON config
- **Multi-device scene** — load multiple devices simultaneously from the add-device dropdown; click a device in the list or click its mesh to switch active device
- **Device renaming** — double-click a device name in the device list to rename it
- **Device origin transform** — move and rotate device origins with translate/rotate gizmo modes
- **Device parenting** — parent a device to a link on another device so it follows the kinematic chain
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
- **Object duplication** — duplicate any imported or primitive object with a single click
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
5. Add the new config filename to the `configFiles` array in `js/panel.js`

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

The WebSocket API at `ws://localhost:8080/ws` allows any client to control devices, manage multi-device scenes, manipulate objects, and control the camera in real time. All commands target the active device by default; include `"device": "<name>"` to target a specific device.

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
| `devices` | `devices` | List all loaded devices |
| `device` | `device i16` | Switch active device by name |
| `home` | `home` | All joints to 0 |
| `fk` | `fk` | Switch to FK mode |
| `ik` | `ik` | Switch to IK mode |
| `joints` | `joints 45 -90 0 0 30 0` | Set all movable joint angles (degrees) |
| `joint` | `joint gamma 45` | Set a single joint by name (tab-completes) |
| `move` | `move 150 100 300 45 0 0` | Switch to IK and move to position (mm) + orientation (deg) |
| `target` | `target 190 0 308` | Set IK target without switching mode |
| `demo` | `demo` | Run the config's demo pose |

**Motion planning commands:**

| Command | Example | Description |
|---------|---------|-------------|
| `plan` | `plan --start 0 0 0 --end 45 90 0` | Linear interpolation between two poses |
| `plan` | `plan --start mu=0 --end mu=45 --stepsize 1 --steptime 50` | Named axes with custom step |
| `scan` | `scan theta 0 90 5` | 1D scan of a single axis |
| `scan` | `scan theta 0 90 5 phi 0 30 2` | 2D grid scan |
| `scan` | `scan theta 0 90 5 phi 0 1` | Coupled scan (phi steps with theta) |
| `scan` | `scan DevA:theta 0 50 5 DevB:phi 0 30 5` | Multi-device scan |

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

All positions are in mm, angles in degrees, using Z-up robot convention. Most commands accept an optional `"device"` field to target a specific device by name or ID; if omitted, the active device is used.

**Device management:**
```json
{"cmd": "getState"}
{"cmd": "listDevices"}
{"cmd": "getDevice"}
{"cmd": "addDevice", "config": "i16_config.json"}
{"cmd": "removeDevice", "device": "Meca500"}
{"cmd": "renameDevice", "name": "MyRobot"}
{"cmd": "setActiveDevice", "device": "i16"}
{"cmd": "setDeviceOrigin", "position": [100, 0, 0], "rotation": [0, 0, 45]}
{"cmd": "setDeviceParent", "parent": "dev_0:L3"}
{"cmd": "listConfigs"}
```

**Joint control:**
```json
{"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}
{"cmd": "setSingleJoint", "index": 1, "angle": -30}
{"cmd": "home"}
{"cmd": "demoPose"}
```

**Kappa virtual angles** (diffractometer geometry):
```json
{"cmd": "setVirtualAngles", "chi": 45, "theta": 10, "phi": 20}
{"cmd": "getVirtualAngles"}
{"cmd": "setKappaSign", "positive": true}
```

**IK control:**
```json
{"cmd": "setMode", "mode": "IK"}
{"cmd": "setIKTarget", "position": [190, 0, 308], "orientation": [0, 0, 0]}
{"cmd": "moveTo", "position": [150, 100, 300], "orientation": [45, 0, 0]}
```

**Object commands:**
```json
{"cmd": "listObjects"}
{"cmd": "getObject", "object": "MyPart"}
{"cmd": "setObject", "object": "MyPart", "position": [100, 50, 0], "rotation": [0, 0, 45]}
{"cmd": "setObject", "object": "MyPart", "color": "#ff0000", "parent": "dev_0:L3"}
{"cmd": "addPrimitive", "type": "cube"}
{"cmd": "removeObject", "object": "MyPart"}
{"cmd": "duplicateObject", "object": "MyPart"}
{"cmd": "resetObjectRotation", "object": "MyPart"}
{"cmd": "resetObjectScale", "index": 0}
```

**Collision commands:**
```json
{"cmd": "setCollision", "enabled": true}
{"cmd": "getCollisions"}
```

**Visualization toggles:**
```json
{"cmd": "setLabels", "enabled": true}
{"cmd": "setOrigins", "enabled": true}
{"cmd": "setChain", "enabled": true}
{"cmd": "setOrtho", "enabled": true}
```

**Camera control:**
```json
{"cmd": "getCamera"}
{"cmd": "setCamera", "position": [500, 500, 500], "target": [0, 0, 150]}
{"cmd": "snapCamera", "view": "front"}
```
Snap views: `+X`, `-X`, `+Y`, `-Y`, `+Z`, `-Z`, `top`, `bottom`, `front`, `back`, `left`, `right`, `iso`.

**Scene persistence:**
```json
{"cmd": "getSceneState"}
{"cmd": "saveScene"}
{"cmd": "help"}
```

**State response:**
```json
{
  "type": "state",
  "device": "Meca500",
  "joints": [0, -30, 60, 0, 45, 90],
  "eePosition": [190.0, 0.0, 308.0],
  "eeOrientation": [0.0, 0.0, 0.0],
  "mode": "FK",
  "ikError": null,
  "collisionEnabled": true,
  "collision": false,
  "collisions": [],
  "chi": 45.0
}
```

The `chi` field is only present for kappa-geometry devices (diffractometers).

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

        # Multi-device: add a second device and position it
        await ws.send(json.dumps({"cmd": "addDevice", "config": "i16_config.json"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "setDeviceOrigin", "device": "i16", "position": [500, 0, 0]}))

        # Camera: snap to front view
        await ws.send(json.dumps({"cmd": "snapCamera", "view": "front"}))

        # Kappa virtual angles (diffractometer only)
        await ws.send(json.dumps({"cmd": "setActiveDevice", "device": "i16"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "setVirtualAngles", "chi": 45, "theta": 10}))

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
threejs_scene.html       HTML shell — loads viewer.css and js/main.js
viewer.css               All viewer styles
js/
  main.js                Entry point — animate loop, event handlers, initialisation
  state.js               Shared mutable state (scene, cameras, controls, devices)
  scene.js               Three.js scene setup, cameras, lights, ground, nav gizmo
  device.js              Device loading, GLB import, slider/IK sync
  kinematics.js          FK, IK solver, kappa geometry math
  panel.js               Control panel UI, device list, parent dropdowns
  stl.js                 Mesh import/export, primitives, duplication, IndexedDB persistence
  collision.js           BVH-accelerated collision detection
  websocket.js           WebSocket client for remote control API
server.py                WebSocket + HTTP server for remote control API
remote_control.py        Interactive command-line remote control client (any device)
robot_config.json        Meca500 R3 device config
i16_config.json          i16 diffractometer device config
gp225_config.json        Yaskawa GP225 device config
gp280_config.json        Yaskawa GP280 device config
robot_scene.glb          Meca500 GLB model
i16_scene.glb            i16 diffractometer GLB model
gp225_scene.glb          Yaskawa GP225 GLB model
gp280_scene_v2.glb       Yaskawa GP280 GLB model
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
