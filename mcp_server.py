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
    pip install "mcp>=1.2,<2" websockets
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
        self._ws = await websockets.connect(self.url, max_size=64 * 1024 * 1024)

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

# Verdicts come from the viewer's own collision check, run headless
# (headless/engine.mjs via headless_client.py) on a copy of the viewer's
# scene: the exact meshes the viewer tests, with the viewer's pair rules.
# The planner's capsules only steer the search; a path is not called clear
# until the exact check agrees. If the exact check cannot run (no Node, or a
# viewer page too old for exportScene), the capsule check answers and the
# verdict says so in checked_by.

EXACT = "viewer collision engine (exact meshes, run headless)"

_engine = None
_engine_error: Optional[str] = None


def _headless():
    """The headless engine, started on first use. None if it cannot run."""
    global _engine, _engine_error
    if _engine is None and _engine_error is None:
        try:
            from headless_client import HeadlessEngine
            _engine = HeadlessEngine()
        except Exception as e:           # no node, no node_modules, failed start
            _engine_error = str(e)
    return _engine


# Collision points of point clouds and PLY splats, by object id, fetched
# from the viewer once and shared by the planner and the headless engine.
_cloud_points: dict = {}


async def _cloud(obj_id: str):
    """An object's collision points (local frame, float32 N x 3), fetched in
    chunks the first time."""
    if obj_id not in _cloud_points:
        import numpy as np
        parts, offset, total = [], 0, None
        while total is None or offset < total:
            chunk = await viewer().request(
                {"cmd": "exportObjectPoints", "id": obj_id, "offset": offset, "count": 1_000_000},
                "objectPoints", timeout=120.0)
            total = chunk["total"]
            parts.append(np.frombuffer(base64.b64decode(chunk["positions"]), dtype="<f4"))
            if chunk["count"] == 0:
                break
            offset += chunk["count"]
        _cloud_points[obj_id] = np.concatenate(parts).reshape(-1, 3) if parts else np.zeros((0, 3), "<f4")
    return _cloud_points[obj_id]


def _points_chunk(obj_id, offset, count):
    """An exportObjectPoints-shaped reply served from the shared cache."""
    pts = _cloud_points[obj_id]
    part = pts[offset:offset + count]
    return {"id": obj_id, "total": len(pts), "offset": offset, "count": len(part),
            "positions": base64.b64encode(part.astype("<f4").tobytes()).decode()}


async def _target_device(device: Optional[str]) -> tuple[int, dict]:
    """The viewer's device by name or id (default: the active one), with its
    index in the viewer's device list."""
    devs = (await viewer().request({"cmd": "listDevices"}, "devices"))["devices"]
    for i, d in enumerate(devs):
        if (device and device in (d["name"], d["id"])) or (not device and d.get("active")):
            return i, d
    if not device and devs:
        return 0, devs[0]
    raise ValueError(f"device '{device}' not found in the viewer")


async def _exact(waypoints, device: Optional[str], resolution_deg: float):
    """Exact verdict from the headless engine, or (None, why not)."""
    global _engine
    engine = _headless()
    if engine is None:
        return None, _engine_error
    try:
        meta = await viewer().request({"cmd": "exportScene", "buffers": False}, "scene", timeout=30.0)
    except RuntimeError as e:
        if "Unknown command" in str(e):
            return None, "the viewer page predates exportScene; reload it"
        raise
    try:
        r = await asyncio.to_thread(engine.sync, meta["scene"])
        if r.get("needBuffers"):
            try:
                full = await viewer().request({"cmd": "exportScene"}, "scene", timeout=120.0)
            except RuntimeError:
                # The small export arrived, so the viewer is there; the one
                # carrying geometry did not. The relay drops messages over its
                # limit (4 MB in server.py before 2026-09-23, 64 MB since).
                await viewer().close()
                return None, ("the viewer's scene export (with geometry) never arrived; "
                              "it is likely larger than the relay server's message limit. "
                              "Restart server.py from this version (64 MB limit)")
            r = await asyncio.to_thread(engine.sync, full["scene"])
        # Point clouds and PLY splats: their points travel apart, once per
        # cloud, through the cache the planner shares.
        for cloud in r.get("needPoints", []):
            await _cloud(cloud["id"])
            await asyncio.to_thread(engine.upload_points, cloud["id"], _points_chunk)
        index, _ = await _target_device(device)
        return await asyncio.to_thread(engine.check_path, index, waypoints, resolution_deg), None
    except Exception as e:
        # A dead engine is restarted on the next call rather than kept.
        from headless_client import HeadlessEngineError
        if isinstance(e, HeadlessEngineError) and "exited" in str(e):
            _engine = None
        raise


def _exact_verdict(r: dict) -> dict:
    out: dict[str, Any] = {"ok": r["ok"], "checked_by": EXACT}
    if not r["ok"]:
        out["reason"] = r["reason"]
        if r.get("pairs"):
            out["collisions"] = r["pairs"]
        if "segment" in r:
            out["failed_between_waypoints"] = r["segment"]
            out["failed_at"] = r["angles"]
    if r.get("samples") is not None:
        out["samples_checked"] = r["samples"]
    if r.get("background"):
        out["other_collisions_in_scene"] = r["background"]
    if r.get("unchecked"):
        out["not_checked_against"] = r["unchecked"]
        out["note"] = ("visible objects the headless check cannot see yet; "
                       "clearance from them is not established")
    return out


def _planner(step_deg: float = 5.0, config_path: Optional[str] = None):
    from planner import RobotPlanner
    return RobotPlanner(config_path or _config_path, step_deg=step_deg)


async def _planner_with_scene(step_deg: float = 5.0, device: Optional[str] = None):
    """A planner for the viewer's device (default: the active one), placed
    where the viewer has it, with the rest of the scene as obstacles: other
    devices, objects, and payload the device carries."""
    try:
        index, dev = await _target_device(device)
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), dev["config"])
        p = _planner(step_deg, config_path)
        devs = (await viewer().request({"cmd": "listDevices"}, "devices", timeout=5.0))["devices"]
        objs = (await viewer().request({"cmd": "listObjects"}, "objects", timeout=5.0)).get("objects", [])
        for o in objs:
            if o.get("hasCollisionPoints") and o.get("visible", True):
                await _cloud(o["id"])
        n = p.sync_from_viewer(devs, objs, index, fetch_points=lambda i: _cloud_points[i])
    except Exception:
        # Viewer unreachable or too old for the fields above: plan the
        # startup config at the origin against nothing, and say so.
        p, n = _planner(step_deg), 0
    return p, n


def _densify(waypoints, interpolate_deg):
    samples: list[tuple[int, list[float]]] = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        span = max(abs(y - x) for x, y in zip(a, b)) if a and b else 0.0
        steps = max(1, int(span / max(0.1, interpolate_deg)))
        for k in range(steps):
            t = k / steps
            samples.append((i, [x + (y - x) * t for x, y in zip(a, b)]))
    samples.append((len(waypoints) - 1, list(waypoints[-1])))
    return samples


async def _capsule_verdict(waypoints, device, interpolate_deg, why_not_exact):
    """The approximate check, used only when the exact one cannot run."""
    p, n_obs = await _planner_with_scene(device=device)
    samples = _densify(waypoints, interpolate_deg)
    base = {"checked_by": f"capsule approximation (exact check unavailable: {why_not_exact})",
            "samples_checked": len(samples), "obstacles_considered": n_obs}
    for idx, q in samples:
        reason = p.diagnose(q)
        if reason is not None:
            return {"ok": False, "reason": reason,
                    "failed_between_waypoints": [idx, min(idx + 1, len(waypoints) - 1)],
                    "failed_at": [round(v, 3) for v in q], **base}
    return {"ok": True, **base}


async def _verdict(waypoints, device, interpolate_deg) -> dict:
    r, why_not = await _exact(waypoints, device, interpolate_deg)
    if r is not None:
        return _exact_verdict(r)
    return await _capsule_verdict(waypoints, device, interpolate_deg, why_not)


@mcp.tool()
async def check_pose(angles: list[float], device: Optional[str] = None) -> str:
    """Check whether one joint configuration is legal and collision-free.

    Checks joint limits, self-collision, and collision with everything in the
    viewer scene (objects, other devices, the floor), using the viewer's own
    collision check. Returns a verdict with the offending pair named, not raw
    geometry.

    Args:
        angles: joint angles in degrees, one per movable joint.
        device: device name; defaults to the active device.
    """
    if not angles:
        return json.dumps({"ok": False, "reason": "no angles given"})
    return json.dumps(await _verdict([list(angles)], device, 1.0), indent=2)


@mcp.tool()
async def check_trajectory(waypoints: list[list[float]],
                           interpolate_deg: float = 1.0,
                           device: Optional[str] = None) -> str:
    """Check a whole trajectory for limit violations and collisions.

    Densifies between waypoints so a sweep cannot tunnel through an obstacle
    between two legal endpoints, then reports the FIRST failure with the
    waypoints it lies between and the offending pair. Uses the viewer's own
    collision check. Run this before committing any trajectory to hardware.

    Args:
        waypoints: list of joint-angle vectors (degrees).
        interpolate_deg: max joint-space spacing between checked samples.
                         Smaller is safer and slower.
        device: device name; defaults to the active device.
    """
    if not waypoints:
        return json.dumps({"ok": False, "reason": "no waypoints given"})
    verdict = await _verdict([list(w) for w in waypoints], device, interpolate_deg)
    verdict["waypoints"] = len(waypoints)
    return json.dumps(verdict, indent=2)


@mcp.tool()
async def plan_path(start: list[float], goal: list[float],
                    step_deg: float = 5.0, device: Optional[str] = None) -> str:
    """Plan a collision-free joint-space path between two configurations.

    Runs RRT-Connect against the current scene, then checks the result with
    the viewer's own collision check; a path is only returned as found once
    that check passes (a few attempts are made). Returns the waypoints WITHOUT
    executing them — call execute_path to watch it in the twin, or hand the
    waypoints to the beamline control system yourself.

    Args:
        start: starting joint angles (degrees).
        goal: target joint angles (degrees).
        step_deg: planner resolution in degrees. Smaller finds tighter routes, slower.
        device: device name; defaults to the active device.
    """
    p, n_obs = await _planner_with_scene(step_deg, device)

    attempts = []
    for _ in range(3):
        path = await asyncio.to_thread(p.plan, list(start), list(goal), False)
        if path is None:
            break
        waypoints = [[round(float(v), 4) for v in q] for q in path]
        verdict = await _verdict(waypoints, device, 1.0)
        if verdict["ok"]:
            return json.dumps({
                "found": True,
                "waypoints": waypoints,
                "count": len(waypoints),
                "validated": verdict,
                "obstacles_considered_by_planner": n_obs,
            }, indent=2)
        attempts.append(verdict)
        if verdict.get("checked_by") != EXACT:
            break          # the capsule check refused its own plan: retrying won't help

    return json.dumps({
        "found": False,
        "reason": ("no collision-free path found; check start and goal with "
                   "check_pose, or reduce step_deg") if not attempts else
                  ("the planner's paths all failed the exact check; its capsule model "
                   "is coarser than the viewer's meshes here"),
        "rejected_paths": attempts,
        "obstacles_considered_by_planner": n_obs,
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
    verdict = json.loads(await check_trajectory(waypoints, device=device))
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
        "validated": verdict,
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

    verdict = json.loads(await check_trajectory(points, device=device))
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
#  check_pose/check_trajectory run the same mesh-level check headless, on a
#  copy of the scene, without moving anything. These read the live viewer's
#  checker for the scene as it stands, which is also the only one that sees
#  point clouds and splats so far.
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
