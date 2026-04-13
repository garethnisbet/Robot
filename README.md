# Robot & Device Visualisation

Interactive 3D visualisation and control for robots and scientific instruments. Config-driven — works with any device defined by a JSON config and GLB model. Includes forward and inverse kinematics, mesh import, collision detection, and a remote control API.

## Supported Devices

| Device | Config | Joints | Description |
|--------|--------|--------|-------------|
| Meca500 R3 | `robot_config.json` | 6 (serial) | 6-DOF compact industrial manipulator |
| i16 Diffractometer | `i16_config.json` | 10 movable (branching) | Diamond Light Source 6-circle diffractometer with merlin and crystal detectors |
| i19 Kappa Diffractometer | `i19_config.json` | branching | Diamond Light Source kappa diffractometer (2θ / θ / κ / φ chain) |
| Yaskawa GP225 | `gp225_config.json` | 6 (serial) | 6-DOF heavy-payload industrial robot |
| Yaskawa GP280 | `gp280_config.json` | 6 (serial) | 6-DOF heavy-payload industrial robot |
| Yaskawa GP180-120 | `gp180_config.json` | 6 (serial) | 6-DOF heavy-payload industrial robot |

New devices can be added from Blender scenes using the `/build-viewer` skill (see [Adding New Devices](#adding-new-devices)).

## Features

- **Config-driven viewer** — a single generic `threejs_scene.html` viewer loads any device via JSON config
- **Multi-device scene** — load multiple devices simultaneously from the add-device dropdown; click a device in the list or click its mesh to switch active device
- **Device renaming** — double-click a device name in the device list to rename it
- **Device origin transform** — move and rotate device origins with translate/rotate gizmo modes; World/Local space toggle for gizmo axis alignment
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
- **Object manipulation** — click objects to select, then move, rotate, or scale with transform gizmos (keyboard: T/R/S, Escape to deselect); World/Local space toggle for gizmo axis alignment
- **Parent-child linking** — attach objects to device links so they follow the kinematic chain
- **Self-collision detection** — BVH-accelerated triangle-level intersection testing between device links, using kinematic adjacency to skip physically connected parts
- **Collision detection** — intersection testing between device links and imported scene objects, with red highlight on colliding meshes
- **Screenshot** — one-click PNG capture of the WebGL view composited with the control panel overlay
- **Unified Euler convention** — viewer, WebSocket state, and the Python `GNKinematics` library all report end-effector orientation as the same ZYX Euler triple (α=Rx, β=Ry, γ=Rz), with matching gimbal-lock branches
- **Remote control API** — two-way WebSocket API for controlling any device from Python or any WebSocket client
- **Session routing** — each browser tab gets a unique session ID; controllers can target a specific tab or broadcast to all
- **Connection info panel** — click the API status indicator (top-left) to see the session ID, connection command, and download `RemoteAPI.zip` (IPython client + `GNKinematics` library + robot definitions)

## Quick Start

**Standalone (no remote control):**
```bash
python3 -m http.server 8000
```
Open `http://localhost:8000/threejs_scene.html` in a browser. Use the dropdown to switch devices, or specify a config: `?config=i16_config.json`.

**With remote control API:**
```bash
pip3 install aiohttp websockets ipython
python3 server.py --config robot_config.json
```
Open `http://localhost:8080` in a browser. The status indicator in the top-left shows the WebSocket connection state and the session ID for this tab. Click it to open the connection info panel, which shows the full connection command, WebSocket URL, and a link to download `RemoteAPI.zip` — a self-contained bundle with `robot_ipython.py`, the `GNKinematics` Python library, and `RobotDefinitions.py`.

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

Both device and object gizmos have a **World/Local** toggle button. In World mode the gizmo axes align with the scene axes; in Local mode they align with the object's own axes. The toggle resets to World when the gizmo is deactivated.

### Parent-Child Linking

Use the **Parent** dropdown to attach objects to device links. Parented objects follow the kinematic chain. Local transforms are preserved when reparenting.

## Collision Detection

Toggle **Collision: ON/OFF** in the panel. The viewer tests for triangle-level intersections using:

1. **Broad phase** — AABB check to eliminate distant pairs
2. **Narrow phase** — BVH-accelerated triangle-triangle intersection via [three-mesh-bvh](https://github.com/gkjohnson/three-mesh-bvh)

Self-collision between device links uses kinematic adjacency analysis — links sharing the same joint or connected through a parent-child relationship are skipped. Links on separate branches (e.g., gamma chain vs mu chain on the i16) are always checked.

## Remote Control

The WebSocket API at `ws://localhost:8080/ws` allows any client to control devices, manage multi-device scenes, manipulate objects, and control the camera in real time. All commands target the active device by default; include `"device": "<name>"` to target a specific device.

### Session Routing

Each browser tab that connects to the viewer is assigned a unique **session ID** (an 8-character hex string, e.g. `ab12cd34`). The ID is shown in the status bar (`API: connected [ab12cd34]`) and in the connection info panel.

Controllers can target a specific viewer tab or broadcast to all:

| Connection | Routes to |
|-----------|-----------|
| `ws://localhost:8080/ws` | All connected viewer tabs |
| `ws://localhost:8080/ws?session=ab12cd34` | Only the tab with that session ID |

Active sessions can be listed via the HTTP endpoint:
```
GET http://localhost:8080/sessions
→ [{"id": "ab12cd34", "viewers": 1}, ...]
```

### Interactive Client (IPython)

```bash
python3 robot_ipython.py --config robot_config.json                        # broadcast to all tabs
python3 robot_ipython.py --config robot_config.json --session ab12cd34     # target a specific tab
python3 robot_ipython.py --config i16_config.json --session ab12cd34       # i16 diffractometer
python3 robot_ipython.py --url ws://192.168.1.100:8080/ws --session ab12cd34  # remote server
```

The client launches an IPython terminal with a pre-connected `robot` object. It supports two syntaxes — **Python method calls** for full programmatic control, and **space-separated commands** (via IPython magics) for quick interactive use. The prompt, help text, and tab completion adapt to the loaded device. The `robot_ipython.py` file can also be downloaded from the connection info panel in the viewer.

**Both syntaxes work side by side:**

```python
meca500 [1]: home                                    # space-separated
meca500 [2]: robot.home()                            # Python method
meca500 [3]: joints 0 30 60 0 45 90                  # space-separated
meca500 [4]: robot.joints(0, 30, 60, 0, 45, 90)     # Python method
meca500 [5]: for a in range(0, 91, 10):              # full Python syntax
         ...:     robot.joint('J1', a)
         ...:     time.sleep(0.1)
```

**Position queries** (return values for programmatic use):

| Property / Method | Example | Description |
|---------|---------|-------------|
| `robot.pos` | `x, y, z = robot.pos` | End-effector position [x, y, z] mm |
| `robot.ori` | `robot.ori` | End-effector orientation [a, b, g] degrees |
| `robot.angles` | `robot.angles` | All joint angles (list) |
| `robot.get_joint('J1')` | `a = robot.get_joint('J1')` | Single joint angle by name |
| `robot.mode` | `robot.mode` | Current mode ('FK' or 'IK') |
| `robot.get_device_pos('GP225')` | `d = robot.get_device_pos()` | Any device: pos, rot, joints, EE |
| `robot.get_obj_pos('cube_1')` | `o = robot.get_obj_pos(0)` | Any object: pos, rot, scale, BB |

**Device commands:**

| Space-separated | Python | Description |
|---------|---------|-------------|
| `state` | `robot.state()` | Request current device state |
| `devices` | `robot.devices()` | List all loaded devices |
| `device i16` | `robot.device('i16')` | Switch active device by name |
| `sessions` | `robot.sessions()` | List viewer session IDs |
| `home` | `robot.home()` | All joints to 0 |
| `fk` | `robot.fk()` | Switch to FK mode |
| `ik` | `robot.ik()` | Switch to IK mode |
| `joints 45 -90 0 0 30 0` | `robot.joints(45, -90, 0, 0, 30, 0)` | Set all movable joint angles (degrees) |
| `joint gamma 45` | `robot.joint('gamma', 45)` | Set a single joint by name |
| `pos meca500 [0,0,0,0,0,0]` | `robot.set_pos('meca500', [0,0,0,0,0,0])` | Set joints on a named device (accepts list, numpy array, or callable) |
| `inc meca500 [0,0,0,0,0,10]` | `robot.inc_pos('meca500', [0,0,0,0,0,10])` | Increment joints on a named device relative to current |
| `move 150 100 300 45 0 0` | `robot.move(150, 100, 300, 45, 0, 0)` | IK move to position (mm) + orientation (deg) |
| `target 190 0 308` | `robot.target(190, 0, 308)` | Set IK target without switching mode |
| `demo` | `robot.demo()` | Run the config's demo pose |

**Device transform commands:**

| Python | Description |
|--------|-------------|
| `robot.devtranslate(dx, dy, dz, space='parent')` | Translate device origin by delta (mm) in parent/local/world frame |
| `robot.devrotate(rx, ry, rz, space='parent')` | Rotate device origin by delta (deg) in parent/local/world frame |

**Motion planning commands:**

| Space-separated | Python | Description |
|---------|---------|-------------|
| `plan --start 0 0 0 --end 45 90 0` | `robot.plan([0,0,0], [45,90,0])` | Path plan between two poses |
| `plan --start mu=0 --end mu=45` | `robot.plan({'mu': 0}, {'mu': 45})` | Named axes |
| `scan theta 0 90 5` | `robot.scan(('theta', 0, 90, 5))` | 1D scan |
| `scan theta 0 90 5 phi 0 30 2` | `robot.scan(('theta',0,90,5), ('phi',0,30,2))` | 2D grid scan |
| `scan theta 0 90 5 phi 0 1` | `robot.scan(('theta',0,90,5), ('phi',0,1))` | Coupled scan |
| `scan DevA:J1 0 50 5 DevB:J1 0 30 5` | `robot.scan(('DevA:J1',0,50,5), ('DevB:J1',0,30,5))` | Multi-device scan |
| — | `robot.scan(('@Cube:tx', 0, 100, 10))` | Object translation scan |
| — | `robot.scan(('@Cube:rz', 0, 360, 10))` | Object rotation scan |
| — | `robot.scan(('@Cube:tx',0,100,10), space='world')` | Object scan in world coords |
| — | `robot.scan(('J1',0,90,5), ('@Cube:tz',0,50,5))` | Mixed joint + object scan |

Object scan axes use `@ObjectName:component` syntax where component is `tx`, `ty`, `tz`, `rx`, `ry`, or `rz`. The `space` parameter (`'local'` or `'world'`) controls the coordinate frame for object transforms (default: `'local'`).

**Object commands:**

| Space-separated | Python | Description |
|---------|---------|-------------|
| `objects` | `robot.objects()` | List all imported objects |
| `obj MyPart` | `robot.obj('MyPart')` | Get object details |
| `objpos MyPart 100 50 0` | `robot.objpos('MyPart', 100, 50, 0)` | Set object position (mm) |
| `objrot #0 0 0 45` | `robot.objrot('#0', 0, 0, 45)` | Set object rotation (degrees) |
| `objscale MyPart 2` | `robot.objscale('MyPart', 2)` | Set uniform scale |
| `objvis MyPart on` | `robot.objvis('MyPart', True)` | Show/hide object |
| `collision on` | `robot.collision(True)` | Enable/disable collision detection |
| `collisions` | `robot.collisions()` | Get current collision pairs |

Object transforms accept a `space` parameter (`'parent'`, `'local'`, or `'world'`):

| Python | Description |
|--------|-------------|
| `robot.objtranslate('Cube', dx, dy, dz, space='parent')` | Translate object by delta (mm) |
| `robot.objrotate('Cube', rx, ry, rz, space='parent')` | Rotate object by delta (deg) |

**Visualization and camera:**

| Python | Description |
|--------|-------------|
| `robot.labels()` / `robot.labels(False)` | Show/hide joint labels |
| `robot.origins()` / `robot.origins(False)` | Show/hide joint origin axes |
| `robot.chain()` / `robot.chain(False)` | Show/hide kinematic chain |
| `robot.ortho()` / `robot.ortho(False)` | Orthographic/perspective camera |
| `robot.camera(position=[500,500,500])` | Set camera position/target |
| `robot.snap('iso')` | Snap to preset view |
| `robot.virtual_angles(chi=45)` | Set kappa virtual angles (diffractometers) |

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
{"cmd": "translateDevice", "delta": [10, 0, 0], "space": "parent"}
{"cmd": "rotateDevice", "delta": [0, 0, 45], "space": "local"}
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
{"cmd": "setObject", "object": "MyPart", "position": [100, 50, 0], "space": "world"}
{"cmd": "setObject", "object": "MyPart", "color": "#ff0000", "parent": "dev_0:L3"}
{"cmd": "translateObject", "name": "MyPart", "delta": [10, 0, 0], "space": "parent"}
{"cmd": "rotateObject", "name": "MyPart", "delta": [0, 0, 45], "space": "local"}
{"cmd": "addPrimitive", "type": "cube"}
{"cmd": "removeObject", "object": "MyPart"}
{"cmd": "duplicateObject", "object": "MyPart"}
{"cmd": "resetObjectRotation", "object": "MyPart"}
{"cmd": "resetObjectScale", "index": 0}
```

`setObject`, `translateObject`, and `rotateObject` accept `"space": "parent"|"local"|"world"` (default: `"parent"`). The `getObject` response includes both local (`position`, `rotation`) and world-frame (`worldPosition`, `worldRotation`) coordinates. `translateDevice` and `rotateDevice` use the same space convention.

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
    # Target a specific viewer tab by session ID (omit ?session=... to broadcast to all)
    async with websockets.connect("ws://localhost:8080/ws?session=ab12cd34") as ws:
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

        # Translate a device in world coordinates
        await ws.send(json.dumps({"cmd": "translateDevice", "delta": [100, 0, 0], "space": "world"}))

        # Move an object in world space
        await ws.send(json.dumps({"cmd": "translateObject", "name": "Cube", "delta": [50, 0, 0], "space": "world"}))

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
  collision.js           BVH-accelerated collision detection (Web Worker + main-thread fallback)
  collision-worker.js    Background thread for collision math
  websocket.js           WebSocket client for remote control API
server.py                WebSocket + HTTP server for remote control API
robot_ipython.py         IPython remote control client (any device)
GNKinematics/            Python forward/inverse kinematics library (matches viewer's ZYX Euler)
RobotDefinitions.py      Robot DH / geometry parameters for GNKinematics
RemoteAPI.zip            Bundled client (ipython client + GNKinematics + RobotDefinitions); served from viewer
robot_config.json        Meca500 R3 device config
i16_config.json          i16 diffractometer device config
i19_config.json          i19 kappa diffractometer device config
gp225_config.json        Yaskawa GP225 device config
gp280_config.json        Yaskawa GP280 device config
gp180_config.json        Yaskawa GP180-120 device config
robot_scene.glb          Meca500 GLB model
i16_scene.glb            i16 diffractometer GLB model
i19_scene.glb            i19 kappa diffractometer GLB model
gp225_scene.glb          Yaskawa GP225 GLB model
gp280_scene.glb          Yaskawa GP280 GLB model
gp180_scene.glb          Yaskawa GP180-120 GLB model
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
