#!/usr/bin/env python3
"""
Robot Visualisation — MCP Server

Exposes the viewer as a *simulation sandbox* to an MCP client (Claude Desktop,
Claude Code, any agent runtime), for orchestrating synchrotron experiments
across physical space (is this pose reachable, does it collide) and
measurement space (does this scan cover what I need).

Design notes, since they are load-bearing:

  * This is a curated surface, not a bridge. The viewer's WebSocket API has
    ~55 commands; dumping all of them as tools would eat the model's context
    and give it too many indistinguishable choices. The ~12 tools here are the
    verbs an experiment-planning task actually uses.

  * Tools return verdicts, not state dumps. check_trajectory answers
    "collides at step 47 between J3 Elbow and detector" rather than handing
    back geometry for the model to reason over.

  * Nothing here drives hardware. Every motion tool moves the digital twin.
    Keeping the simulate/commit line at the tool boundary is the whole point:
    an agent cannot move a real motor with these tools because no tool here
    can. A hardware path belongs behind its own server and its own approval.

Usage:
    pip install "mcp>=1.2" websockets
    python mcp_server.py [--url ws://localhost:8000/ws] [--config meca500_config.json]

Register with Claude Code:
    claude mcp add robot-vis -- python /path/to/mcp_server.py
"""

import argparse
import asyncio
import base64
import json
import os
import sys
from typing import Any, Optional

import websockets
from mcp.server.fastmcp import FastMCP, Image

DEFAULT_URL = os.getenv("ROBOT_VIS_URL", "ws://localhost:8000/ws")
DEFAULT_CONFIG = os.getenv("ROBOT_VIS_CONFIG", "meca500_config.json")

_url = DEFAULT_URL
_config_path = DEFAULT_CONFIG

mcp = FastMCP("robot-vis")


# ─────────────────────────────────────────────────────────────────────────────
#  Viewer connection
#
#  One connection, lazily opened and reused. The viewer streams unsolicited
#  state echoes (every slider drag, every scan step), so a request cannot just
#  read the next frame off the socket — it has to filter for the reply it
#  wants and drop the rest.
# ─────────────────────────────────────────────────────────────────────────────

class Viewer:
    def __init__(self, url: str):
        self.url = url
        self._ws = None
        self._lock = asyncio.Lock()

    async def _connect(self):
        if self._ws is not None:
            try:
                await self._ws.ping()
                return
            except Exception:
                self._ws = None
        self._ws = await websockets.connect(self.url, max_size=32 * 1024 * 1024)

    async def send(self, msg: dict):
        """Fire-and-forget — used for the steps of a swept trajectory."""
        async with self._lock:
            await self._connect()
            await self._ws.send(json.dumps(msg))

    async def _drain(self, grace: float = 0.15):
        """Discard messages already queued on the socket.

        Every setJoints echoes a state frame, so after a sweep the socket holds
        a backlog of stale poses. Without this, the next request is answered by
        one of them and reports the wrong pose. Caller holds the lock.
        """
        while True:
            try:
                await asyncio.wait_for(self._ws.recv(), timeout=grace)
            except asyncio.TimeoutError:
                return

    async def request(self, msg: dict, want: str, timeout: float = 10.0,
                      device: Optional[str] = None) -> dict:
        """Send a command and wait for the first reply of type `want`.

        `device` pins the reply to one device, so a state echo for some other
        device (or a stale one from an in-flight sweep) cannot satisfy the wait
        with the wrong pose.
        """
        async with self._lock:
            await self._connect()
            await self._drain()
            await self._ws.send(json.dumps(msg))

            async def _await_reply():
                while True:
                    raw = await self._ws.recv()
                    data = json.loads(raw)
                    if data.get("type") == "error":
                        raise RuntimeError(data.get("error", "viewer error"))
                    if data.get("type") != want:
                        continue
                    if device is not None and data.get("device") != device:
                        continue
                    return data

            try:
                return await asyncio.wait_for(_await_reply(), timeout=timeout)
            except asyncio.TimeoutError:
                raise RuntimeError(
                    f"viewer did not reply to '{msg.get('cmd')}' within {timeout:g}s. "
                    f"Is the viewer page open at {self.url}?"
                )

    async def close(self):
        if self._ws is not None:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None


_viewer: Optional[Viewer] = None


def viewer() -> Viewer:
    global _viewer
    if _viewer is None:
        _viewer = Viewer(_url)
    return _viewer


def _dev(msg: dict, device: Optional[str]) -> dict:
    if device:
        msg["device"] = device
    return msg


# ─────────────────────────────────────────────────────────────────────────────
#  Orientation
# ─────────────────────────────────────────────────────────────────────────────

@mcp.tool()
async def list_devices() -> str:
    """List the devices loaded in the viewer, with their joint names and types.

    Call this first: joint order and axis names differ per device, and every
    other tool addresses devices by the names returned here.
    """
    reply = await viewer().request({"cmd": "listDevices"}, "devices")
    out = []
    for d in reply.get("devices", []):
        out.append({
            "name": d.get("name"),
            "type": d.get("type", "serial"),
            "joints": d.get("jointNames") or d.get("joints"),
            "active": d.get("active", False),
        })
    return json.dumps(out, indent=2)


@mcp.tool()
async def get_state(device: Optional[str] = None) -> str:
    """Get a device's current pose: joint angles (deg), end-effector position
    (mm, Z-up) and orientation (deg), FK/IK mode, and any live collisions.

    Args:
        device: device name; defaults to the active device.
    """
    reply = await viewer().request(_dev({"cmd": "getState"}, device), "state",
                                   device=device)
    reply.pop("type", None)
    return json.dumps(reply, indent=2)


# ─────────────────────────────────────────────────────────────────────────────
#  Motion in the twin
# ─────────────────────────────────────────────────────────────────────────────

@mcp.tool()
async def set_joints(angles: list[float], device: Optional[str] = None) -> str:
    """Move a device in the twin by setting all its joint angles (degrees).

    Supply one value per movable joint, in the order list_devices reports.
    Angles outside a joint's limits are clamped by the viewer, so compare the
    returned pose against what you asked for.

    Args:
        angles: joint angles in degrees, one per movable joint.
        device: device name; defaults to the active device.
    """
    reply = await viewer().request(
        _dev({"cmd": "setJoints", "angles": [round(float(a), 4) for a in angles]},
             device), "state", device=device)
    return json.dumps({
        "joints": reply.get("joints"),
        "eePosition": reply.get("eePosition"),
        "eeOrientation": reply.get("eeOrientation"),
        "collision": reply.get("collision"),
        "collisions": reply.get("collisions"),
    }, indent=2)


@mcp.tool()
async def move_to(position: list[float], orientation: Optional[list[float]] = None,
                  device: Optional[str] = None) -> str:
    """Drive a device's end-effector to a Cartesian target in the twin (IK).

    Switches the device to IK mode and solves. The solver is damped least
    squares and weights orientation against position, so check `ikError_mm`
    and `reachable` rather than assuming the pose was achieved.

    Omitting `orientation` does not free the wrist: the solver keeps pulling
    towards whatever orientation target was last set, which can hold the
    end-effector millimetres off an otherwise reachable position. If the
    residual is larger than you want, pass the orientation you actually need.

    Args:
        position: [x, y, z] in mm, world frame, Z-up.
        orientation: [a, b, g] Euler angles in degrees, relative to home. Optional.
        device: device name; defaults to the active device.
    """
    v = viewer()
    msg = {"cmd": "moveTo", "position": [float(p) for p in position]}
    if orientation is not None:
        msg["orientation"] = [float(o) for o in orientation]
    reply = await v.request(_dev(msg, device), "state", device=device)

    # A current viewer settles the solver before replying, so this usually
    # returns on the first check. The polling stays for an older viewer, where
    # IK only advances in the animation loop — and note that loop is paused
    # while the tab is hidden, so there the error may never improve.
    err = reply.get("ikError")
    for _ in range(30):
        if err is not None and err < 1.0:
            break
        await asyncio.sleep(0.1)
        reply = await v.request(_dev({"cmd": "getState"}, device), "state",
                                device=device)
        new_err = reply.get("ikError")
        if new_err is None:
            break
        # Converged or stalled: either way, further waiting buys nothing.
        if err is not None and abs(err - new_err) < 1e-3:
            err = new_err
            break
        err = new_err

    return json.dumps({
        "reachable": (err is not None and err < 1.0),
        "ikError_mm": err,
        "joints": reply.get("joints"),
        "eePosition": reply.get("eePosition"),
        "eeOrientation": reply.get("eeOrientation"),
        "collision": reply.get("collision"),
        "collisions": reply.get("collisions"),
    }, indent=2)


@mcp.tool()
async def home(device: Optional[str] = None) -> str:
    """Return a device to its home pose (all joints zero) in the twin.

    Args:
        device: device name; defaults to the active device.
    """
    reply = await viewer().request(_dev({"cmd": "home"}, device), "state",
                                   device=device)
    return json.dumps({"joints": reply.get("joints"),
                       "eePosition": reply.get("eePosition")}, indent=2)


# ─────────────────────────────────────────────────────────────────────────────
#  Validation — the reason this server exists
# ─────────────────────────────────────────────────────────────────────────────

def _planner(step_deg: float = 5.0):
    from planner import RobotPlanner
    return RobotPlanner(_config_path, step_deg=step_deg)


async def _planner_with_scene(step_deg: float = 5.0):
    """A planner whose obstacles are the objects currently in the viewer."""
    p = _planner(step_deg)
    try:
        objs = await viewer().request({"cmd": "listObjects"}, "objects", timeout=5.0)
        n = p.sync_from_viewer_objects(objs.get("objects", []))
    except Exception:
        n = 0
    return p, n


@mcp.tool()
async def check_pose(angles: list[float]) -> str:
    """Check whether one joint configuration is legal and collision-free.

    Checks joint limits, self-collision, and collision with the objects
    currently in the viewer scene. Returns a verdict with the offending pair
    named, not raw geometry.

    Args:
        angles: joint angles in degrees, one per movable joint.
    """
    p, n_obs = await _planner_with_scene()
    reason = p.diagnose(angles)
    return json.dumps({
        "ok": reason is None,
        "reason": reason,
        "obstacles_considered": n_obs,
    }, indent=2)


@mcp.tool()
async def check_trajectory(waypoints: list[list[float]],
                           interpolate_deg: float = 2.0) -> str:
    """Check a whole trajectory for limit violations and collisions.

    Densifies between waypoints so a sweep cannot tunnel through an obstacle
    between two legal endpoints, then reports the FIRST failure with the step
    index and the offending pair. Run this before committing any trajectory to
    hardware.

    Args:
        waypoints: list of joint-angle vectors (degrees).
        interpolate_deg: max joint-space spacing between checked samples.
                         Smaller is safer and slower.
    """
    if not waypoints:
        return json.dumps({"ok": False, "reason": "no waypoints given"})

    p, n_obs = await _planner_with_scene()

    # Densify: check the path, not just its corners.
    samples: list[tuple[int, list[float]]] = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        span = max(abs(y - x) for x, y in zip(a, b)) if a and b else 0.0
        steps = max(1, int(span / max(0.1, interpolate_deg)))
        for s in range(steps):
            t = s / steps
            samples.append((i, [x + (y - x) * t for x, y in zip(a, b)]))
    samples.append((len(waypoints) - 1, list(waypoints[-1])))

    for idx, q in samples:
        reason = p.diagnose(q)
        if reason is not None:
            return json.dumps({
                "ok": False,
                "reason": reason,
                "failed_between_waypoints": [idx, min(idx + 1, len(waypoints) - 1)],
                "failed_at": [round(v, 3) for v in q],
                "samples_checked": len(samples),
                "obstacles_considered": n_obs,
            }, indent=2)

    return json.dumps({
        "ok": True,
        "waypoints": len(waypoints),
        "samples_checked": len(samples),
        "obstacles_considered": n_obs,
    }, indent=2)


@mcp.tool()
async def plan_path(start: list[float], goal: list[float],
                    step_deg: float = 5.0) -> str:
    """Plan a collision-free joint-space path between two configurations.

    Runs RRT-Connect against the current scene. Returns the waypoints WITHOUT
    executing them — call execute_path to watch it in the twin, or hand the
    waypoints to the beamline control system yourself.

    Args:
        start: starting joint angles (degrees).
        goal: target joint angles (degrees).
        step_deg: planner resolution in degrees. Smaller finds tighter routes, slower.
    """
    p, n_obs = await _planner_with_scene(step_deg)
    path = await asyncio.to_thread(p.plan, list(start), list(goal), False)
    if path is None:
        return json.dumps({
            "found": False,
            "reason": "no collision-free path found; check start and goal with "
                      "check_pose, or reduce step_deg",
            "obstacles_considered": n_obs,
        }, indent=2)
    return json.dumps({
        "found": True,
        "waypoints": [[round(float(v), 4) for v in q] for q in path],
        "count": len(path),
        "obstacles_considered": n_obs,
    }, indent=2)


@mcp.tool()
async def execute_path(waypoints: list[list[float]], step_ms: int = 80,
                       device: Optional[str] = None) -> str:
    """Play a trajectory through the twin so it can be watched or captured.

    This moves the simulated device only. It validates the path first and
    refuses to play one that collides.

    Args:
        waypoints: joint-angle vectors (degrees).
        step_ms: delay between steps, milliseconds.
        device: device name; defaults to the active device.
    """
    verdict = json.loads(await check_trajectory(waypoints))
    if not verdict.get("ok"):
        return json.dumps({"executed": False, "refused_because": verdict}, indent=2)

    v = viewer()
    for q in waypoints:
        await v.send(_dev({"cmd": "setJoints",
                           "angles": [round(float(a), 4) for a in q]}, device))
        await asyncio.sleep(step_ms / 1000.0)

    # Let the last step's echo land before asking where we ended up.
    await asyncio.sleep(0.2)
    state = await v.request(_dev({"cmd": "getState"}, device), "state", device=device)
    return json.dumps({
        "executed": True,
        "steps": len(waypoints),
        "finalJoints": state.get("joints"),
        "eePosition": state.get("eePosition"),
    }, indent=2)


# ─────────────────────────────────────────────────────────────────────────────
#  Measurement space
# ─────────────────────────────────────────────────────────────────────────────

@mcp.tool()
async def plan_scan(axis: str, start: float, stop: float, step: float,
                    device: Optional[str] = None) -> str:
    """Build and validate a single-axis scan over measurement space.

    Expands the axis range into joint-space waypoints from the device's current
    pose, then validates every point. Returns the points and a verdict — it
    does not move anything. Feed the result to execute_path to watch it.

    Args:
        axis: joint/axis name, as reported by list_devices (e.g. "J1 Base").
        start: first value, degrees.
        stop: last value, degrees (inclusive).
        step: increment, degrees. Sign is inferred from start/stop.
        device: device name; defaults to the active device.
    """
    state = await viewer().request(_dev({"cmd": "getState"}, device), "state",
                                   device=device)
    names = state.get("jointNames") or []
    base = list(state.get("joints") or [])
    if not names:
        return json.dumps({"error": "device reports no named joints; "
                                    "hexapods are not supported by plan_scan"})

    lower = [n.lower() for n in names]
    key = axis.strip().lower()
    if key in lower:
        idx = lower.index(key)
    else:
        matches = [i for i, n in enumerate(lower) if n.startswith(key)]
        if len(matches) != 1:
            return json.dumps({"error": f"axis '{axis}' did not match exactly one "
                                        f"joint", "available": names}, indent=2)
        idx = matches[0]

    n = int(abs(stop - start) / abs(step)) + 1 if step else 1
    sign = 1.0 if stop >= start else -1.0
    points = []
    for i in range(n):
        q = list(base)
        q[idx] = start + sign * abs(step) * i
        points.append([round(v, 4) for v in q])

    verdict = json.loads(await check_trajectory(points))
    return json.dumps({
        "axis": names[idx],
        "points": len(points),
        "range": [start, stop, step],
        "valid": verdict.get("ok"),
        "problem": None if verdict.get("ok") else verdict,
        "waypoints": points,
    }, indent=2)


# ─────────────────────────────────────────────────────────────────────────────
#  Live collision detection in the viewer
#
#  Distinct from check_pose/check_trajectory, which run the Python planner
#  against capsules and bounding boxes. These read the viewer's own mesh-level
#  checker, which is the only one that sees imported meshes and point clouds.
# ─────────────────────────────────────────────────────────────────────────────

@mcp.tool()
async def set_collision(enabled: Optional[bool] = None,
                        floor: Optional[bool] = None,
                        headless: Optional[bool] = None) -> str:
    """Turn the viewer's live collision checking on or off.

    Args:
        enabled: master switch for mesh-vs-mesh collision checking.
        floor: floor-plane checks. Turn this OFF when the scene contains a
            scanned room or terrain: its floor points lie in the plane, so the
            whole cloud reports a permanent floor contact that masks every
            real collision. Needs a viewer from 2026-09 or later.
        headless: run the checks off the render loop. Turn this ON for
            unattended use — the render loop is paused while the browser tab
            is hidden, and with it the collision checks.
    """
    v = viewer()
    out: dict[str, Any] = {}

    if enabled is not None:
        r = await v.request({"cmd": "setCollision", "enabled": bool(enabled)}, "state")
        out["collisionEnabled"] = r.get("collisionEnabled")
        if enabled:
            # The first checking pass has not run yet. Returning before it
            # lands leaves the next get_collisions reporting an empty list,
            # which reads as "all clear" when it only means "not looked yet".
            await asyncio.sleep(0.6)
    if headless is not None:
        r = await v.request({"cmd": "setCollisionHeadless", "enabled": bool(headless)},
                            "collisionHeadless")
        out["headless"] = r.get("enabled")
    if floor is not None:
        try:
            r = await v.request({"cmd": "setFloorCollision", "enabled": bool(floor)},
                                "floorCollision")
            out["floorCollision"] = r.get("enabled")
        except RuntimeError as e:
            out["floorCollision"] = (
                f"unsupported by this viewer ({e}). The tab predates the "
                f"setFloorCollision command — reload it, or use the "
                f"'Floor Collision' button in the control panel."
            )

    if not out:
        return json.dumps({"error": "nothing to set; pass enabled, floor or headless"})
    return json.dumps(out, indent=2)


@mcp.tool()
async def get_collisions() -> str:
    """Read what is currently in collision in the viewer.

    Floor-plane contacts are reported separately from everything else, because
    a scanned room sits permanently in the floor plane and would otherwise
    bury the contacts you care about.

    This reports the last completed checking pass. After moving something,
    give the checker a moment before reading, or an empty result may mean
    "not looked yet" rather than "clear".
    """
    r = await viewer().request({"cmd": "getCollisions"}, "collisions")
    pairs = r.get("pairs", [])
    floor = [p for p in pairs if p.get("link") == "floor"]
    real = [p for p in pairs if p.get("link") != "floor"]

    # 'current' is the viewer comparing the scene's fingerprint against the one
    # the standing result was computed from — an exact answer to "does this
    # still describe the scene", not a guess from elapsed time.
    current = r.get("current")
    age = r.get("ageMs")
    stale = current is False

    if not r.get("enabled"):
        note = "collision checking is off; run set_collision(enabled=True) first"
    elif "passes" not in r:
        note = ("this viewer predates freshness reporting, so there is no way to tell "
                "a current result from a frozen one — reload the tab to pick it up")
    elif r.get("passes") == 0:
        note = "the checker has not completed a pass yet; this result means nothing"
    elif stale:
        note = ("the scene has changed since this result was computed, so it is out "
                "of date — treat it as unknown, not as clear. The checker runs in "
                "the viewer's render loop, which the browser pauses while the tab is "
                "hidden; use set_collision(headless=True) for unattended runs.")
    else:
        note = None

    return json.dumps({
        "enabled": r.get("enabled"),
        "headless": r.get("headless"),
        "contact": bool(real),
        "contacts": real,
        "floorContacts": floor,
        "describesCurrentScene": current,
        "stale": stale,
        "passes": r.get("passes"),
        "ageMs": age,
        "note": note,
    }, indent=2)


# ─────────────────────────────────────────────────────────────────────────────
#  Seeing and saving
# ─────────────────────────────────────────────────────────────────────────────

@mcp.tool()
async def capture_view(view: Optional[str] = None, max_width: int = 800) -> Image:
    """Render the current scene and return it as an image.

    Use this to actually look at a pose or the end of a trajectory rather than
    inferring it from numbers.

    Args:
        view: optionally snap the camera first — one of +X, -X, +Y, -Y, +Z, -Z,
              top, bottom, front, back, left, right, iso.
        max_width: longest edge of the returned image in pixels.
    """
    v = viewer()
    if view:
        await v.request({"cmd": "snapCamera", "view": view}, "camera")
    reply = await v.request({"cmd": "captureImage", "maxWidth": int(max_width)},
                            "image", timeout=20.0)
    return Image(data=base64.b64decode(reply["data"]), format="png")


@mcp.tool()
async def get_scene() -> str:
    """Describe the whole scene: devices, imported objects and camera.

    Use this to find out what obstacles exist before planning around them.
    """
    reply = await viewer().request({"cmd": "getSceneState"}, "sceneState")
    reply.pop("type", None)
    return json.dumps(reply, indent=2)


# ─────────────────────────────────────────────────────────────────────────────

def main():
    global _url, _config_path
    ap = argparse.ArgumentParser(description="MCP server for the robot viewer")
    ap.add_argument("--url", default=DEFAULT_URL,
                    help=f"viewer WebSocket URL (default {DEFAULT_URL})")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help="device config JSON, used by the planner")
    args = ap.parse_args()

    _url = args.url
    _config_path = args.config

    # The client launches this server from whatever cwd it likes, so resolve a
    # relative config against the project directory rather than that cwd.
    if not os.path.isabs(_config_path) and not os.path.exists(_config_path):
        candidate = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 _config_path)
        if os.path.exists(candidate):
            _config_path = candidate

    if not os.path.exists(_config_path):
        print(f"warning: config '{_config_path}' not found; planning tools will "
              f"fail until --config points at a real device config",
              file=sys.stderr)

    mcp.run()


if __name__ == "__main__":
    main()
