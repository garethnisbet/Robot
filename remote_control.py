#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot/Device — Remote Control Terminal

Interactive terminal client for controlling the viewer via WebSocket API.
Works with any device config (Meca500, i16 diffractometer, etc.).
Provides a Linux terminal-like experience with command history, tab completion,
and colored output.

Usage:
    pip install websockets
    python remote_control.py [--url ws://localhost:8080/ws] [--config robot_config.json]
"""

import argparse
import asyncio
import json
import os
import readline
import select
import sys
import termios
import tty
import urllib.request

import websockets

# ── Pending async responses (used by plan_sequence to intercept listObjects) ─
_ws_pending: dict = {}   # key → {"event": asyncio.Event, "data": any}

# ── ANSI colour helpers ──────────────────────────────────────────────────────

_COLORS = os.getenv("NO_COLOR") is None and sys.stdout.isatty()

def _c(code, text):
    return f"\033[{code}m{text}\033[0m" if _COLORS else text

def _bold(t):     return _c("1", t)
def _dim(t):      return _c("2", t)
def _red(t):      return _c("31", t)
def _green(t):    return _c("32", t)
def _yellow(t):   return _c("33", t)
def _blue(t):     return _c("34", t)
def _magenta(t):  return _c("35", t)
def _cyan(t):     return _c("36", t)
def _white(t):    return _c("37", t)
def _bred(t):     return _c("1;31", t)
def _bgreen(t):   return _c("1;32", t)
def _byellow(t):  return _c("1;33", t)
def _bcyan(t):    return _c("1;36", t)

# ── History file ─────────────────────────────────────────────────────────────

HISTORY_FILE = os.path.expanduser("~/.robot_viewer_history")
HISTORY_LENGTH = 1000

def _load_history():
    try:
        readline.read_history_file(HISTORY_FILE)
    except FileNotFoundError:
        pass

def _save_history():
    readline.set_history_length(HISTORY_LENGTH)
    try:
        readline.write_history_file(HISTORY_FILE)
    except OSError:
        pass

# ── Tab completion ───────────────────────────────────────────────────────────

COMMANDS = [
    "state", "home", "fk", "ik", "joints", "joint", "pos", "move", "target",
    "collision", "collisions", "objects", "obj", "objpos", "objrot",
    "objscale", "objvis", "objresetrot", "objresetscale", "devices", "device",
    "sessions",
    "demo", "plan", "scan", "clear", "history", "help",
    "quit", "exit",
]

COLLISION_ARGS = ["on", "off"]
OBJVIS_ARGS = ["on", "off"]

class Completer:
    def __init__(self, joint_names=None):
        self.matches = []
        self.joint_names = joint_names or []
        self.device_names = []

    def complete(self, text, state):
        if state == 0:
            line = readline.get_line_buffer()
            parts = line.lstrip().split()
            if len(parts) <= 1:
                self.matches = [c + " " for c in COMMANDS if c.startswith(text)]
            elif parts[0] == "collision" and len(parts) == 2:
                self.matches = [a + " " for a in COLLISION_ARGS if a.startswith(text)]
            elif parts[0] == "objvis" and len(parts) == 3:
                self.matches = [a + " " for a in OBJVIS_ARGS if a.startswith(text)]
            elif parts[0] in ("joint", "pos") and len(parts) == 2:
                self.matches = [n + " " for n in self.joint_names if n.lower().startswith(text.lower())]
            elif parts[0] == "device" and len(parts) == 2:
                self.matches = [n + " " for n in self.device_names if n.lower().startswith(text.lower())]
            else:
                self.matches = []
        return self.matches[state] if state < len(self.matches) else None

# ── Config loading ──────────────────────────────────────────────────────────

_http_base_url = None   # set from the WS URL at startup

def _ws_url_to_http(ws_url):
    """Convert ws(s)://host:port/ws?... → http(s)://host:port"""
    from urllib.parse import urlparse
    p = urlparse(ws_url)
    scheme = "https" if p.scheme == "wss" else "http"
    return f"{scheme}://{p.hostname}:{p.port}" if p.port else f"{scheme}://{p.hostname}"

def load_config(config_path):
    """Load the robot/device config — try local file first, then fetch from server."""
    # Try local file
    try:
        with open(config_path) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        pass

    # Try fetching from server
    if _http_base_url:
        filename = os.path.basename(config_path)
        url = f"{_http_base_url}/{filename}"
        try:
            with urllib.request.urlopen(url, timeout=5) as resp:
                return json.load(resp)
        except Exception as e:
            print(f"  {_yellow('Warning')}: Could not load config {filename} from server: {e}")
            return None

    print(f"  {_yellow('Warning')}: Could not load config {config_path}")
    return None

def get_movable_joints(config):
    """Return list of (index, name) for movable (non-fixed) joints."""
    if not config:
        return []
    return [(i, j["name"]) for i, j in enumerate(config["joints"]) if not j.get("fixed")]

# ── Prompt ───────────────────────────────────────────────────────────────────

def _prompt(device_name):
    safe = device_name.lower().replace(" ", "_")
    if _COLORS:
        return f"\001\033[1;32m\002{safe}\001\033[0m\002:\001\033[1;34m\002~\001\033[0m\002$ "
    return f"{safe}:~$ "

# ── Banner ───────────────────────────────────────────────────────────────────

def _print_banner(device_name, url, movable_joints):
    if _COLORS:
        print(_cyan(f"\n  ╔══════════════════════════════════════════╗"))
        print(_cyan(f"  ║") + _bold(f"  {device_name:^38s}") + _cyan(f"  ║"))
        print(_cyan(f"  ╚══════════════════════════════════════════╝"))
    else:
        print(f"\n  {device_name}")
        print(f"  {'=' * len(device_name)}")
    print(f"\n  Remote Control Terminal v2.0")
    print(f"  Server: {_bold(url) if _COLORS else url}")
    if movable_joints:
        names = ", ".join(name for _, name in movable_joints)
        print(f"  Joints ({len(movable_joints)}): {_dim(names)}")
    print()

# ── Help (man-page style) ───────────────────────────────────────────────────

def _print_help(device_name, movable_joints):
    n_joints = len(movable_joints)
    joint_args = " ".join(f"j{i+1}" for i in range(min(n_joints, 6)))
    if n_joints > 6:
        joint_args += " ..."

    title = f"{device_name.upper()}(1)"
    header = f"{title:<30}Remote Control{title:>30}"
    sections = [
        (header, None),
        ("NAME", f"    {device_name.lower()} - interactive remote control terminal for {device_name} viewer"),
        ("SYNOPSIS", f"    python remote_control.py [--url ws://HOST:PORT/ws] [--config CONFIG.json] [--session SESSION_ID]"),
        ("SESSION COMMANDS", f"""\
    {_bold('sessions') if _COLORS else 'sessions'}
        List viewer sessions currently connected to the server, with their
        session IDs.  Use a session ID with --session to target a specific
        browser tab.  The session ID is also shown in the viewer status bar."""),
        ("DEVICE COMMANDS", f"""\
    {_bold('devices') if _COLORS else 'devices'}
        List all devices loaded in the viewer, showing which is active.

    {_bold('device') if _COLORS else 'device'} {_cyan('<name>') if _COLORS else '<name>'}
        Switch to a different device by name.  Updates the prompt, joint
        names, and tab completion to match the new device's config."""),
        ("ROBOT COMMANDS", f"""\
    {_bold('state') if _COLORS else 'state'}
        Request current device state (joints, end-effector pose, collisions).

    {_bold('home') if _COLORS else 'home'}
        Move all joints to 0 degrees (home position).

    {_bold('fk') if _COLORS else 'fk'}
        Switch to Forward Kinematics mode.

    {_bold('ik') if _COLORS else 'ik'}
        Switch to Inverse Kinematics mode.

    {_bold('joints') if _COLORS else 'joints'} {_cyan(joint_args) if _COLORS else joint_args}
        Set all {n_joints} movable joint angles in degrees.
        Values are given in config order (skipping fixed joints).

    {_bold('joint') if _COLORS else 'joint'} {_cyan('<name> <angle>') if _COLORS else '<name> <angle>'}
        Set a single joint by name. Tab-completes joint names.

    {_bold('move') if _COLORS else 'move'} {_cyan('x y z') if _COLORS else 'x y z'} [{_cyan('a b g') if _COLORS else 'a b g'}]
        Switch to IK mode and move to position (mm) with optional ZYX
        orientation (degrees). Defaults to (0, 0, 0).

    {_bold('target') if _COLORS else 'target'} {_cyan('x y z') if _COLORS else 'x y z'} [{_cyan('a b g') if _COLORS else 'a b g'}]
        Set IK target position/orientation without switching mode.

    {_bold('plan') if _COLORS else 'plan'} {_cyan('--start') if _COLORS else '--start'} {_cyan('<axes>') if _COLORS else '<axes>'} {_cyan('--end') if _COLORS else '--end'} {_cyan('<axes>') if _COLORS else '<axes>'} [{_cyan('--stepsize <deg>') if _COLORS else '--stepsize <deg>'}] [{_cyan('--steptime <ms>') if _COLORS else '--steptime <ms>'}]
        Run RRT-Connect path planner and stream the resulting waypoints to
        the viewer.  Scene mesh objects are automatically used as obstacles.
        Axes can be positional ({_dim(joint_args) if _COLORS else joint_args}) or named ({_dim('mu=30 theta=45') if _COLORS else 'mu=30 theta=45'}).
        Named mode: unspecified axes default to 0.  Names match the first
        word of the joint name (e.g. {_dim('J1') if _COLORS else 'J1'} for {_dim('J1 Base') if _COLORS else 'J1 Base'}) and are case-insensitive.
        --stepsize  RRT step size in degrees (default 5)
        --steptime  Delay between waypoints in ms (default 80)

    {_bold('scan') if _COLORS else 'scan'} {_cyan('<axis> <start> <end> <step>') if _COLORS else '<axis> <start> <end> <step>'} [{_cyan('<axis> ...')} ] [{_cyan('--steptime <ms>') if _COLORS else '--steptime <ms>'}]
        Scan one or more axes.  The first axis always takes start/end/step.
        Subsequent axes can be:
          3 values (start end step) — grid scan (cartesian product)
          2 values (start step)    — coupled scan (lockstep with primary)
        Axis is resolved by name (first word, case-insensitive) or
        1-based index.  Other axes hold their current positions.
        Use {_bold('Device:Axis') if _COLORS else 'Device:Axis'} syntax to scan axes across multiple
        devices simultaneously (e.g. scan Meca500:J1 0 90 5 GP225:J2 0 45 5).
        --steptime  Delay between steps in ms (default 80)"""),
        ("COLLISION COMMANDS", f"""\
    {_bold('collision') if _COLORS else 'collision'} [{_cyan('on') if _COLORS else 'on'}|{_cyan('off') if _COLORS else 'off'}]
        Toggle or explicitly enable/disable collision detection.

    {_bold('collisions') if _COLORS else 'collisions'}
        Get current collision pairs."""),
        ("OBJECT COMMANDS", f"""\
    {_bold('objects') if _COLORS else 'objects'}
        List all imported mesh objects.

    {_bold('obj') if _COLORS else 'obj'} {_cyan('<name|#idx>') if _COLORS else '<name|#idx>'}
        Get details for an object by name or index (e.g. #0).

    {_bold('objpos') if _COLORS else 'objpos'} {_cyan('<name|#idx> x y z') if _COLORS else '<name|#idx> x y z'}
        Set object position in mm.

    {_bold('objrot') if _COLORS else 'objrot'} {_cyan('<name|#idx> rx ry rz') if _COLORS else '<name|#idx> rx ry rz'}
        Set object rotation in degrees.

    {_bold('objscale') if _COLORS else 'objscale'} {_cyan('<name|#idx> s') if _COLORS else '<name|#idx> s'} | {_cyan('sx sy sz') if _COLORS else 'sx sy sz'}
        Set uniform or per-axis scale.

    {_bold('objvis') if _COLORS else 'objvis'} {_cyan('<name|#idx>') if _COLORS else '<name|#idx>'} {_cyan('on') if _COLORS else 'on'}|{_cyan('off') if _COLORS else 'off'}
        Show or hide an object.

    {_bold('objresetrot') if _COLORS else 'objresetrot'} {_cyan('<name|#idx>') if _COLORS else '<name|#idx>'}
        Reset object rotation to (0, 0, 0).

    {_bold('objresetscale') if _COLORS else 'objresetscale'} {_cyan('<name|#idx>') if _COLORS else '<name|#idx>'}
        Reset object scale to (1, 1, 1)."""),
        ("TERMINAL COMMANDS", f"""\
    {_bold('demo') if _COLORS else 'demo'}
        Run the demo pose from the config.

    {_bold('clear') if _COLORS else 'clear'}
        Clear the terminal screen.

    {_bold('history') if _COLORS else 'history'}
        Show command history.

    {_bold('help') if _COLORS else 'help'}
        Show this help page.

    {_bold('quit') if _COLORS else 'quit'}, {_bold('exit') if _COLORS else 'exit'}, {_bold('Ctrl+D') if _COLORS else 'Ctrl+D'}
        Disconnect and exit."""),
        ("KEYBOARD SHORTCUTS", """\
    Up/Down     Navigate command history
    Tab         Auto-complete commands and joint names
    Ctrl+A      Move cursor to start of line
    Ctrl+E      Move cursor to end of line
    Ctrl+K      Delete from cursor to end of line
    Ctrl+U      Delete entire line
    Ctrl+W      Delete previous word
    Ctrl+R      Reverse history search
    Ctrl+C      Cancel current input
    Ctrl+D      Exit terminal"""),
        ("EXAMPLES", f"""\
    $ joints {' '.join(['0'] * min(n_joints, 6))}{'...' if n_joints > 6 else ''}
    $ joint {movable_joints[0][1] if movable_joints else 'J1'} 45
    $ move 150 100 300 45 0 0
    $ scan {movable_joints[0][1].split()[0].lower() if movable_joints else 'delta'} 0 50 5
    $ scan {movable_joints[0][1].split()[0].lower() if movable_joints else 'delta'} 0 20 1 {movable_joints[1][1].split()[0].lower() if len(movable_joints) > 1 else 'gamma'} 20 2
    $ scan {movable_joints[0][1].split()[0].lower() if movable_joints else 'delta'} 0 20 1 {movable_joints[1][1].split()[0].lower() if len(movable_joints) > 1 else 'gamma'} 0 30 2
    $ collision on
    $ devices
    $ device GP225
    $ scan Meca500:J1 0 90 5 GP225:J2 0 45 5"""),
    ]
    print()
    for heading, body in sections:
        if body is None:
            print(_bold(heading) if _COLORS else heading)
        else:
            print(f"  {_byellow(heading) if _COLORS else heading}")
            print(body)
        print()

# ── Output formatting ────────────────────────────────────────────────────────

def _clear_line():
    print("\r\033[K", end="")

def _reprint_prompt(device_name):
    print(_prompt(device_name), end="", flush=True)


async def print_incoming(ws, ctx):
    """Background task to print state messages from the viewer."""
    try:
        async for message in ws:
            device_name = ctx["device_name"]
            movable_joints = ctx["movable_joints"]
            data = json.loads(message)
            _clear_line()

            if data.get("type") == "state":
                # If scan_sequence is waiting for state, hand it off silently
                pending = _ws_pending.get("state")
                if pending:
                    pending["data"] = data
                    pending["event"].set()
                    continue

                all_joints = data["joints"]
                ee = data["eePosition"]
                ori = data.get("eeOrientation", [0, 0, 0])
                mode = data.get("mode", "?")
                err = data.get("ikError")

                # Show only movable joint values with names
                if movable_joints:
                    j_parts = []
                    for idx, name in movable_joints:
                        val = all_joints[idx] if idx < len(all_joints) else 0
                        j_parts.append(f"{name}={val:.1f}")
                    j_str = ", ".join(j_parts)
                else:
                    j_str = ", ".join(f"{a:7.1f}" for a in all_joints)

                mode_c = _bgreen(mode) if mode == "FK" else _bcyan(mode)

                print(f"  {_bold('STATE')} | mode={mode_c}")
                print(f"         joints: {_white(j_str)}")
                print(f"         ee=({_cyan(f'{ee[0]:.1f}')}, {_cyan(f'{ee[1]:.1f}')}, {_cyan(f'{ee[2]:.1f}')})mm  "
                      f"ori=({_magenta(f'{ori[0]:.1f}')}, {_magenta(f'{ori[1]:.1f}')}, {_magenta(f'{ori[2]:.1f}')})deg  "
                      f"err={_yellow(f'{err:.2f}mm') if err is not None else _dim('-')}")

                coll_on = data.get("collisionEnabled", False)
                collisions = data.get("collisions", [])
                if coll_on:
                    if collisions:
                        pairs = ", ".join(f"{_bred(c['link'])}<>{_bred(c['object'])}" for c in collisions)
                        print(f"         collision: {_bred('YES')} [{pairs}]")
                    else:
                        print(f"         collision: {_green('none')}")

                _reprint_prompt(device_name)

            elif data.get("type") == "collisions":
                enabled = data.get("enabled", False)
                pairs = data.get("pairs", [])
                status = _bgreen("ON") if enabled else _dim("OFF")
                print(f"  {_bold('COLLISION')} detection: {status}")
                if enabled:
                    if pairs:
                        for p in pairs:
                            print(f"    {_red(p['link'])} <> {_red(p['object'])}")
                    else:
                        print(f"    {_green('No collisions')}")
                _reprint_prompt(device_name)

            elif data.get("type") == "objects":
                # If plan_sequence is waiting for this, hand it off silently
                pending = _ws_pending.get("objects")
                if pending:
                    pending["data"] = data.get("objects", [])
                    pending["event"].set()
                    continue

                objs = data.get("objects", [])
                print(f"  {_bold('OBJECTS')} ({len(objs)} imported)")
                for o in objs:
                    p, r, s = o["position"], o["rotation"], o["scale"]
                    vis = _green("visible") if o.get("visible", True) else _dim("hidden")
                    idx = _dim(f"[{o['index']}]")
                    print(f"    {idx} {_bold(o['name'])}  {vis}")
                    print(f"        pos=({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})mm  "
                          f"rot=({r[0]:.1f}, {r[1]:.1f}, {r[2]:.1f})deg  "
                          f"scale=({s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f})")
                _reprint_prompt(device_name)

            elif data.get("type") == "object":
                o = data
                p, r, s = o["position"], o["rotation"], o["scale"]
                vis = _green("visible") if o.get("visible", True) else _dim("hidden")
                idx_str = o["index"]
                print(f"  {_dim(f'[{idx_str}]')} {_bold(o['name'])}  {vis}")
                print(f"    pos  = ({_cyan(f'{p[0]:.1f}')}, {_cyan(f'{p[1]:.1f}')}, {_cyan(f'{p[2]:.1f}')}) mm")
                print(f"    rot  = ({_magenta(f'{r[0]:.1f}')}, {_magenta(f'{r[1]:.1f}')}, {_magenta(f'{r[2]:.1f}')}) deg")
                print(f"    scale= ({s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f})")
                _reprint_prompt(device_name)

            elif data.get("type") == "devices":
                pending = _ws_pending.get("devices")
                if pending:
                    pending["data"] = data
                    pending["event"].set()
                    continue
                devs = data.get("devices", [])
                if "completer" in ctx:
                    ctx["completer"].device_names = [d["name"] for d in devs]
                print(f"  {_bold('DEVICES')} ({len(devs)} loaded)")
                for d in devs:
                    active = d.get("active", False)
                    marker = _bgreen("*") if active else " "
                    name = _bold(d["name"]) if active else d["name"]
                    cfg = _dim(d.get("config", "?"))
                    kappa = f" {_dim('[kappa]')}" if d.get("isKappa") else ""
                    print(f"    {marker} {name}  {cfg}  "
                          f"{d.get('numJoints', '?')} joints  {d.get('mode', '?')}{kappa}")
                _reprint_prompt(device_name)

            elif data.get("type") == "device":
                pending = _ws_pending.get("device")
                if pending:
                    pending["data"] = data
                    pending["event"].set()
                    continue
                d = data
                kappa = f" {_dim('[kappa]')}" if d.get("isKappa") else ""
                print(f"  {_bold('DEVICE')} {_bold(d.get('name', '?'))}  "
                      f"{_dim(d.get('config', '?'))}  "
                      f"{d.get('numJoints', '?')} joints  mode={d.get('mode', '?')}{kappa}")
                _reprint_prompt(device_name)

            elif data.get("type") == "error" or "error" in data:
                print(f"  {_bred('ERROR')}: {_red(data.get('error', 'unknown error'))}")
                _reprint_prompt(device_name)

    except websockets.ConnectionClosed:
        pass


# ── Demo sequence ────────────────────────────────────────────────────────────

async def demo_sequence(ws, config, num_joints):
    """Run the demo pose from the config, then return home."""
    demo_pose = config.get("demoPose") if config else None
    if not demo_pose:
        print(f"  {_yellow('No demoPose defined in config')}")
        return

    poses = [
        ("Home position", {"cmd": "home"}),
        ("Demo pose", {"cmd": "setJoints", "angles": demo_pose}),
    ]

    for i, (desc, cmd) in enumerate(poses, 1):
        print(f"  {_dim(f'[{i}/{len(poses)}]')} {_cyan(desc)}")
        await ws.send(json.dumps(cmd))
        await asyncio.sleep(1.5)

    print(f"  {_bgreen('Demo complete!')}")


# ── Path planning sequence ───────────────────────────────────────────────────

def _densify_path(path, step_deg):
    """
    Interpolate between consecutive waypoints so no step exceeds step_deg
    (measured as the max joint displacement across all joints).
    """
    import math
    dense = [path[0]]
    for i in range(len(path) - 1):
        q0 = path[i]
        q1 = path[i + 1]
        max_delta = max(abs(a - b) for a, b in zip(q0, q1))
        n_steps = max(1, math.ceil(max_delta / step_deg))
        for k in range(1, n_steps + 1):
            t = k / n_steps
            dense.append([a + t * (b - a) for a, b in zip(q0, q1)])
    return dense


async def plan_sequence(ws, config_path, start_q, goal_q, step_ms=80, step_deg=5.0):
    """
    Run RRT-Connect planner from start_q to goal_q, then stream waypoints
    to the viewer.
      step_ms  — delay between waypoints in milliseconds (playback speed)
      step_deg — RRT extension step size in degrees (smaller = finer paths, slower planning)
    Automatically fetches imported mesh objects from the viewer and uses
    their world bounding boxes as collision obstacles.
    """
    try:
        from planner import RobotPlanner
    except ImportError:
        print(f"  {_bred('Error')}: planner.py not found — make sure it is in the same directory")
        return

    # ── Fetch current scene objects from viewer ───────────────────────────
    event = asyncio.Event()
    _ws_pending["objects"] = {"event": event, "data": None}
    await ws.send(json.dumps({"cmd": "listObjects"}))
    try:
        await asyncio.wait_for(event.wait(), timeout=3.0)
        objects_data = _ws_pending["objects"]["data"] or []
    except asyncio.TimeoutError:
        objects_data = []
        print(f"  {_yellow('Warning')}: timed out waiting for object list — planning without scene obstacles")
    finally:
        _ws_pending.pop("objects", None)

    # ── Build planner and sync obstacles ─────────────────────────────────
    planner = RobotPlanner(config_path, step_deg=step_deg)
    n_obs = planner.sync_from_viewer_objects(objects_data)
    if n_obs:
        print(f"  {_dim(f'Using {n_obs} scene object(s) as collision obstacles')}")
    elif objects_data:
        print(f"  {_dim('Scene objects found but none have geometry bounding boxes (point clouds?)')}")

    print(f"  {_dim(f'Planning... (step size {step_deg}°)')}")
    path = planner.plan(start_q, goal_q)

    if path is None:
        print(f"  {_bred('No path found.')}")
        return

    # Densify: interpolate between sparse planner waypoints so motion is smooth
    dense = _densify_path(path, step_deg)

    print(f"  {_bgreen('Executing')} {len(dense)} steps "
          f"({_dim(f'{step_ms} ms/step, {step_deg}° resolution')})  "
          f"{_dim('press q to stop')}")

    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    stopped = False
    try:
        tty.setcbreak(fd)   # single-keypress mode; keeps Ctrl+C working
        for q in dense:
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch.lower() == 'q':
                    stopped = True
                    break
            await ws.send(json.dumps({"cmd": "setJoints", "angles": [round(a, 4) for a in q]}))
            await asyncio.sleep(step_ms / 1000.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)

    if stopped:
        print(f"\n  {_yellow('Motion stopped.')}")
    else:
        print(f"  {_bgreen('Done.')}")


# ── Scan helpers ─────────────────────────────────────────────────────────────

def _resolve_axis_for_device(name, movable_joints):
    """Map axis name → index in a device's movable_joints list, or None."""
    nl = name.lower()
    for mi, (_, jname) in enumerate(movable_joints):
        if jname.lower() == nl:
            return mi
        if jname.lower().split()[0] == nl:
            return mi
    try:
        n = int(name)
        if 1 <= n <= len(movable_joints):
            return n - 1
    except ValueError:
        pass
    return None


# ── Scan sequence ────────────────────────────────────────────────────────────

def _build_axis_vals(start, end, step):
    """Build a list of values from start to end in increments of step."""
    import math
    step = abs(step)
    direction = 1.0 if end >= start else -1.0
    n_steps = int(math.floor(abs(end - start) / step)) + 1
    vals = []
    for i in range(n_steps):
        val = start + i * step * direction
        if direction > 0 and val > end:
            val = end
        elif direction < 0 and val < end:
            val = end
        vals.append(val)
    if not vals or vals[-1] != end:
        vals.append(end)
    return vals


async def scan_sequence(ws, movable_joints, num_joints, grid_axes,
                        coupled_axes=None, step_ms=80):
    """
    N-dimensional grid scan, or 1D coupled (lockstep) scan.

    grid_axes    — list of (axis_idx, start, end, step) for axes that form
                   the cartesian-product grid.
    coupled_axes — list of (axis_idx, start, step) for axes that move in
                   lockstep with the primary (first grid) axis.  The number
                   of points is determined by the primary axis.
    """
    import itertools

    # ── Fetch current joint state ─────────────────────────────────────────
    event = asyncio.Event()
    _ws_pending["state"] = {"event": event, "data": None}
    await ws.send(json.dumps({"cmd": "getState"}))
    try:
        await asyncio.wait_for(event.wait(), timeout=3.0)
        state_data = _ws_pending["state"]["data"]
    except asyncio.TimeoutError:
        state_data = None
        print(f"  {_yellow('Warning')}: timed out waiting for state — using zeros for other axes")
    finally:
        _ws_pending.pop("state", None)

    # Build base as full joint array (all joints including fixed)
    if state_data:
        base = list(state_data["joints"])
    else:
        base = [0.0] * num_joints

    coupled_axes = coupled_axes or []

    # ── Build per-axis value ranges for grid axes ─────────────────────────
    grid_ranges = []   # [(actual_joint_idx, [vals...])]
    for axis_idx, start, end, step in grid_axes:
        step = abs(step)
        if step < 1e-6:
            print(f"  {_bred('Error')}: step size must be > 0")
            return
        vals = _build_axis_vals(start, end, step)
        actual_joint_idx = movable_joints[axis_idx][0]
        grid_ranges.append((actual_joint_idx, vals))

    # ── Build waypoints ───────────────────────────────────────────────────
    if coupled_axes:
        # 1D coupled scan — all axes move in lockstep with primary axis
        primary_vals = grid_ranges[0][1]
        n_points = len(primary_vals)

        # Build coupled value lists (same length as primary)
        coupled_ranges = []  # [(actual_joint_idx, [vals...])]
        for axis_idx, c_start, c_step in coupled_axes:
            actual_joint_idx = movable_joints[axis_idx][0]
            vals = [c_start + i * c_step for i in range(n_points)]
            coupled_ranges.append((actual_joint_idx, vals))

        # All grid axes beyond the first also lockstep (same length)
        all_ranges = grid_ranges + coupled_ranges
        waypoints = []
        for i in range(n_points):
            angles = list(base)
            for joint_idx, vals in all_ranges:
                angles[joint_idx] = vals[i]
            waypoints.append(angles)
    else:
        # N-dimensional grid scan (cartesian product)
        all_val_lists = [vals for _, vals in grid_ranges]
        joint_indices = [idx for idx, _ in grid_ranges]
        waypoints = []
        for combo in itertools.product(*all_val_lists):
            angles = list(base)
            for joint_idx, val in zip(joint_indices, combo):
                angles[joint_idx] = val
            waypoints.append(angles)

    # ── Print scan summary ────────────────────────────────────────────────
    axis_descs = []
    for axis_idx, start, end, step in grid_axes:
        name = movable_joints[axis_idx][1]
        axis_descs.append(f"{_bold(name)}: {start} → {end} (step {abs(step)})")
    for axis_idx, c_start, c_step in coupled_axes:
        name = movable_joints[axis_idx][1]
        n_points = len(grid_ranges[0][1])
        c_end = c_start + (n_points - 1) * c_step
        axis_descs.append(f"{_bold(name)}: {c_start} → {c_end} (step {c_step})")
    mode_str = _dim("coupled") if coupled_axes else (_dim("grid") if len(grid_axes) > 1 else "")
    mode_suffix = f"  {mode_str}" if mode_str else ""
    print(f"  {_bgreen('Scanning')} {', '.join(axis_descs)}{mode_suffix}")
    print(f"  {len(waypoints)} points, {step_ms} ms/step  "
          f"{_dim('press q to stop')}")

    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    stopped = False
    try:
        tty.setcbreak(fd)
        for q in waypoints:
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch.lower() == 'q':
                    stopped = True
                    break
            await ws.send(json.dumps({"cmd": "setJoints", "angles": [round(a, 4) for a in q]}))
            await asyncio.sleep(step_ms / 1000.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)

    if stopped:
        print(f"\n  {_yellow('Scan stopped.')}")
    else:
        print(f"  {_bgreen('Scan complete.')}")


# ── Multi-device scan ───────────────────────────────────────────────────────

async def _fetch_device_configs(ws, device_names):
    """Fetch device list from the viewer and load configs for named devices."""
    event = asyncio.Event()
    _ws_pending["devices"] = {"event": event, "data": None}
    await ws.send(json.dumps({"cmd": "listDevices"}))
    try:
        await asyncio.wait_for(event.wait(), timeout=3.0)
        resp = _ws_pending["devices"]["data"]
    except asyncio.TimeoutError:
        return None
    finally:
        _ws_pending.pop("devices", None)

    if not resp:
        return None
    devs = resp.get("devices", [])
    result = {}
    for dname in device_names:
        dev = next((d for d in devs if d["name"] == dname), None)
        if not dev:
            return None
        cfg_path = dev.get("config")
        if not cfg_path:
            return None
        config = load_config(cfg_path)
        if not config:
            return None
        result[dname] = {
            "config": config,
            "movable_joints": get_movable_joints(config),
            "num_joints": len(config["joints"]),
        }
    return result


async def multi_scan_sequence(ws, device_axes, step_ms=80):
    """
    Multi-device scan: axes on different devices scanned simultaneously.

    device_axes — list of tuples:
        Grid:    (device, joint_idx, "grid",    start, end, step, label)
        Coupled: (device, joint_idx, "coupled", start, step, label)
    """
    import itertools

    # Collect unique devices in order
    device_names = list(dict.fromkeys(a[0] for a in device_axes))

    # Fetch current state for each device
    bases = {}
    for dname in device_names:
        event = asyncio.Event()
        _ws_pending["state"] = {"event": event, "data": None}
        await ws.send(json.dumps({"cmd": "getState", "device": dname}))
        try:
            await asyncio.wait_for(event.wait(), timeout=3.0)
            state_data = _ws_pending["state"]["data"]
        except asyncio.TimeoutError:
            state_data = None
            print(f"  {_yellow('Warning')}: timed out fetching state for {dname}")
        finally:
            _ws_pending.pop("state", None)
        bases[dname] = list(state_data["joints"]) if state_data else [0.0] * 20

    # Separate grid and coupled axes
    grid_ranges = []    # [(device, joint_idx, [vals], label)]
    coupled_info = []   # [(device, joint_idx, start, step, label)]
    for a in device_axes:
        dname, joint_idx, mode = a[0], a[1], a[2]
        if mode == "grid":
            start, end, step, label = a[3], a[4], a[5], a[6]
            if abs(step) < 1e-6:
                print(f"  {_bred('Error')}: step size must be > 0")
                return
            vals = _build_axis_vals(start, end, step)
            grid_ranges.append((dname, joint_idx, vals, label))
        else:
            coupled_info.append((dname, joint_idx, a[3], a[4], a[5]))

    # Build waypoints — each is {device_name: [angles...]}
    if coupled_info:
        primary_vals = grid_ranges[0][2]
        n_points = len(primary_vals)
        coupled_ranges = []
        for dname, joint_idx, c_start, c_step, _lbl in coupled_info:
            vals = [c_start + i * c_step for i in range(n_points)]
            coupled_ranges.append((dname, joint_idx, vals))
        all_ranges = [(d, j, v) for d, j, v, _ in grid_ranges] + coupled_ranges
        waypoints = []
        for i in range(n_points):
            wp = {d: list(bases[d]) for d in device_names}
            for dname, joint_idx, vals in all_ranges:
                wp[dname][joint_idx] = vals[i]
            waypoints.append(wp)
    else:
        all_val_lists = [v for _, _, v, _ in grid_ranges]
        waypoints = []
        for combo in itertools.product(*all_val_lists):
            wp = {d: list(bases[d]) for d in device_names}
            for (dname, joint_idx, _, _), val in zip(grid_ranges, combo):
                wp[dname][joint_idx] = val
            waypoints.append(wp)

    # Print summary
    axis_descs = []
    for a in device_axes:
        if a[2] == "grid":
            label, start, end, step = a[6], a[3], a[4], a[5]
            axis_descs.append(f"{_bold(label)}: {start} \u2192 {end} (step {abs(step)})")
        else:
            label, c_start, c_step = a[5], a[3], a[4]
            n_points = len(grid_ranges[0][2])
            c_end = c_start + (n_points - 1) * c_step
            axis_descs.append(f"{_bold(label)}: {c_start} \u2192 {c_end} (step {c_step})")
    mode_str = _dim("coupled") if coupled_info else (_dim("grid") if len(grid_ranges) > 1 else "")
    mode_suffix = f"  {mode_str}" if mode_str else ""
    print(f"  {_bgreen('Scanning')} {', '.join(axis_descs)}  {_dim('multi-device')}{mode_suffix}")
    print(f"  {len(waypoints)} points, {step_ms} ms/step  {_dim('press q to stop')}")

    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    stopped = False
    try:
        tty.setcbreak(fd)
        for wp in waypoints:
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch.lower() == 'q':
                    stopped = True
                    break
            for dname, angles in wp.items():
                await ws.send(json.dumps({
                    "cmd": "setJoints",
                    "device": dname,
                    "angles": [round(a, 4) for a in angles],
                }))
            await asyncio.sleep(step_ms / 1000.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)

    if stopped:
        print(f"\n  {_yellow('Scan stopped.')}")
    else:
        print(f"  {_bgreen('Scan complete.')}")


# ── Object reference parsing ─────────────────────────────────────────────────

def _parse_obj_ref(token):
    if token.startswith("#") and token[1:].isdigit():
        return {"index": int(token[1:])}
    return {"object": token}


# ── Main interactive loop ────────────────────────────────────────────────────

async def interactive(url, config_path):
    global _http_base_url
    _http_base_url = _ws_url_to_http(url)
    config = load_config(config_path)
    device_name = config["name"] if config else "Robot"
    num_joints = len(config["joints"]) if config else 6
    movable_joints = get_movable_joints(config)
    n_movable = len(movable_joints)
    joint_name_to_idx = {name.lower(): idx for idx, name in movable_joints}
    ctx = {"device_name": device_name, "movable_joints": movable_joints}

    _load_history()
    completer = Completer(joint_names=[name for _, name in movable_joints])
    readline.set_completer(completer.complete)
    readline.parse_and_bind("tab: complete")
    readline.set_completer_delims(" \t\n")
    ctx["completer"] = completer

    _print_banner(device_name, url, movable_joints)
    print(f"  Connecting to {_dim(url)} ...")

    try:
        ws = await websockets.connect(url)
    except (ConnectionRefusedError, OSError) as e:
        print(f"  {_bred('Connection failed')}: {e}")
        print(f"  Is the server running?  python server.py --config {config_path}")
        return

    print(f"  {_bgreen('Connected!')} Type {_bold('help')} for commands, {_bold('Tab')} to complete.\n")

    listener = asyncio.create_task(print_incoming(ws, ctx))

    def _update_device_state(new_config, new_config_path, new_name):
        nonlocal config, config_path, device_name, num_joints, movable_joints
        nonlocal n_movable, joint_name_to_idx, prompt
        config = new_config
        config_path = new_config_path
        device_name = new_name
        num_joints = len(config["joints"]) if config else 6
        movable_joints = get_movable_joints(config)
        n_movable = len(movable_joints)
        joint_name_to_idx = {name.lower(): idx for idx, name in movable_joints}
        prompt = _prompt(device_name)
        completer.joint_names = [name for _, name in movable_joints]
        ctx["device_name"] = device_name
        ctx["movable_joints"] = movable_joints

    def _resolve_axis_name(name):
        """Map axis name → index in movable_joints list, or None."""
        nl = name.lower()
        for mi, (_, jname) in enumerate(movable_joints):
            if jname.lower() == nl:                     # exact
                return mi
            if jname.lower().split()[0] == nl:          # first word
                return mi
        try:                                            # 1-based number
            n = int(name)
            if 1 <= n <= n_movable:
                return n - 1
        except ValueError:
            pass
        return None

    loop = asyncio.get_event_loop()
    prompt = _prompt(device_name)

    try:
        while True:
            try:
                line = await loop.run_in_executor(None, lambda: input(prompt))
            except EOFError:
                break
            except KeyboardInterrupt:
                print()
                continue

            line = line.strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            try:
                if cmd in ("quit", "exit", "q"):
                    break

                elif cmd == "help":
                    _print_help(device_name, movable_joints)

                elif cmd == "clear":
                    os.system("clear" if os.name != "nt" else "cls")

                elif cmd == "history":
                    n = readline.get_current_history_length()
                    start = max(1, n - 49)
                    for i in range(start, n + 1):
                        entry = readline.get_history_item(i)
                        if entry:
                            print(f"  {_dim(f'{i:4d}')}  {entry}")

                elif cmd == "state":
                    await ws.send(json.dumps({"cmd": "getState"}))

                elif cmd == "devices":
                    await ws.send(json.dumps({"cmd": "listDevices"}))

                elif cmd == "sessions":
                    http_url = url.replace("ws://", "http://").replace("wss://", "https://")
                    http_url = http_url.split("?")[0]          # strip query string
                    http_url = http_url.rsplit("/ws", 1)[0] + "/sessions"
                    try:
                        with urllib.request.urlopen(http_url, timeout=3) as resp:
                            data = json.loads(resp.read())
                        if not data:
                            print(f"  {_dim('No active viewer sessions.')}")
                        else:
                            print(f"  {_bold('Active sessions')}:")
                            for s in data:
                                viewers_label = _dim(f"({s['viewers']} viewer{'s' if s['viewers'] != 1 else ''})")
                                print(f"    {_bcyan(s['id'])}  {viewers_label}")
                    except Exception as e:
                        print(f"  {_red('Could not fetch sessions')}: {e}")

                elif cmd == "device":
                    if len(parts) < 2:
                        print(f"  {_yellow('Usage')}: device {_cyan('<name>')}")
                        print(f"  Type {_bold('devices')} to list available devices.")
                    else:
                        target = " ".join(parts[1:])
                        # Switch active device and wait for confirmation
                        event = asyncio.Event()
                        _ws_pending["state"] = {"event": event, "data": None}
                        await ws.send(json.dumps({"cmd": "setActiveDevice", "device": target}))
                        try:
                            await asyncio.wait_for(event.wait(), timeout=2.0)
                            state_data = _ws_pending["state"]["data"]
                        except asyncio.TimeoutError:
                            state_data = None
                        finally:
                            _ws_pending.pop("state", None)

                        if not state_data:
                            continue

                        new_name = state_data.get("device", target)

                        # Fetch device info to get config path
                        event = asyncio.Event()
                        _ws_pending["device"] = {"event": event, "data": None}
                        await ws.send(json.dumps({"cmd": "getDevice"}))
                        try:
                            await asyncio.wait_for(event.wait(), timeout=2.0)
                            dev_data = _ws_pending["device"]["data"]
                        except asyncio.TimeoutError:
                            dev_data = None
                        finally:
                            _ws_pending.pop("device", None)

                        new_config_path = dev_data.get("config") if dev_data else None
                        if new_config_path:
                            new_config = load_config(new_config_path)
                            if new_config:
                                _update_device_state(new_config, new_config_path, new_name)
                                names = ", ".join(name for _, name in movable_joints)
                                print(f"  {_bgreen('Switched to')} {_bold(new_name)}")
                                print(f"  Joints ({n_movable}): {_dim(names)}")
                            else:
                                device_name = new_name
                                prompt = _prompt(device_name)
                                ctx["device_name"] = device_name
                                print(f"  {_bgreen('Switched to')} {_bold(new_name)} "
                                      f"{_yellow('(config not available locally)')}")
                        else:
                            device_name = new_name
                            prompt = _prompt(device_name)
                            ctx["device_name"] = device_name
                            print(f"  {_bgreen('Switched to')} {_bold(new_name)}")

                elif cmd == "home":
                    await ws.send(json.dumps({"cmd": "home"}))

                elif cmd == "fk":
                    await ws.send(json.dumps({"cmd": "setMode", "mode": "FK"}))

                elif cmd == "ik":
                    await ws.send(json.dumps({"cmd": "setMode", "mode": "IK"}))

                elif cmd == "joints":
                    if len(parts) - 1 != n_movable:
                        names = " ".join(f"<{name}>" for _, name in movable_joints[:6])
                        suffix = " ..." if n_movable > 6 else ""
                        print(f"  {_yellow('Usage')}: joints {_cyan(names + suffix)}")
                        print(f"  Expected {_bold(str(n_movable))} values (movable joints only)")
                    else:
                        # Build full joint angles array (all joints, including fixed at 0)
                        movable_vals = [float(x) for x in parts[1:]]
                        all_angles = [0.0] * num_joints
                        for (joint_idx, _name), val in zip(movable_joints, movable_vals):
                            all_angles[joint_idx] = val
                        await ws.send(json.dumps({"cmd": "setJoints", "angles": all_angles}))

                elif cmd in ("joint", "pos"):
                    if len(parts) < 3:
                        print(f"  {_yellow('Usage')}: {cmd} {_cyan('<name> <angle>')}")
                        print(f"  Available: {', '.join(name for _, name in movable_joints)}")
                    else:
                        name = parts[1].lower()
                        angle = float(parts[2])
                        if name not in joint_name_to_idx:
                            # Try case-insensitive partial match
                            matches = [(n, idx) for n, idx in joint_name_to_idx.items() if n.startswith(name)]
                            if len(matches) == 1:
                                name = matches[0][0]
                            else:
                                print(f"  {_yellow('Unknown joint')}: {parts[1]}")
                                print(f"  Available: {', '.join(n for _, n in movable_joints)}")
                                continue
                        joint_idx = joint_name_to_idx[name]
                        await ws.send(json.dumps({
                            "cmd": "setSingleJoint",
                            "index": joint_idx,
                            "angle": angle,
                        }))

                elif cmd == "move":
                    if len(parts) < 4:
                        print(f"  {_yellow('Usage')}: move {_cyan('x y z')} [{_cyan('a b g')}]")
                    else:
                        pos = [float(x) for x in parts[1:4]]
                        ori = [float(x) for x in parts[4:7]] if len(parts) >= 7 else [0, 0, 0]
                        await ws.send(json.dumps({"cmd": "moveTo", "position": pos, "orientation": ori}))

                elif cmd == "target":
                    if len(parts) < 4:
                        print(f"  {_yellow('Usage')}: target {_cyan('x y z')} [{_cyan('a b g')}]")
                    else:
                        pos = [float(x) for x in parts[1:4]]
                        ori = [float(x) for x in parts[4:7]] if len(parts) >= 7 else [0, 0, 0]
                        await ws.send(json.dumps({"cmd": "setIKTarget", "position": pos, "orientation": ori}))

                elif cmd == "collision":
                    if len(parts) >= 2 and parts[1].lower() in ("on", "off"):
                        enabled = parts[1].lower() == "on"
                        await ws.send(json.dumps({"cmd": "setCollision", "enabled": enabled}))
                    else:
                        await ws.send(json.dumps({"cmd": "setCollision"}))

                elif cmd == "collisions":
                    await ws.send(json.dumps({"cmd": "getCollisions"}))

                elif cmd == "objects":
                    await ws.send(json.dumps({"cmd": "listObjects"}))

                elif cmd == "obj":
                    if len(parts) < 2:
                        print(f"  {_yellow('Usage')}: obj {_cyan('<name|#idx>')}")
                    else:
                        ref = _parse_obj_ref(parts[1])
                        await ws.send(json.dumps({"cmd": "getObject", **ref}))

                elif cmd == "objpos":
                    if len(parts) < 5:
                        print(f"  {_yellow('Usage')}: objpos {_cyan('<name|#idx> x y z')}")
                    else:
                        ref = _parse_obj_ref(parts[1])
                        pos = [float(x) for x in parts[2:5]]
                        await ws.send(json.dumps({"cmd": "setObject", **ref, "position": pos}))

                elif cmd == "objrot":
                    if len(parts) < 5:
                        print(f"  {_yellow('Usage')}: objrot {_cyan('<name|#idx> rx ry rz')}")
                    else:
                        ref = _parse_obj_ref(parts[1])
                        rot = [float(x) for x in parts[2:5]]
                        await ws.send(json.dumps({"cmd": "setObject", **ref, "rotation": rot}))

                elif cmd == "objscale":
                    if len(parts) < 3:
                        print(f"  {_yellow('Usage')}: objscale {_cyan('<name|#idx> s')}  or  {_cyan('<name|#idx> sx sy sz')}")
                    else:
                        ref = _parse_obj_ref(parts[1])
                        vals = [float(x) for x in parts[2:]]
                        if len(vals) == 1:
                            await ws.send(json.dumps({"cmd": "setObject", **ref, "scale": vals[0]}))
                        elif len(vals) >= 3:
                            await ws.send(json.dumps({"cmd": "setObject", **ref, "scale": vals[:3]}))
                        else:
                            print(f"  {_yellow('Usage')}: objscale {_cyan('<name|#idx> s')}  or  {_cyan('<name|#idx> sx sy sz')}")

                elif cmd == "objvis":
                    if len(parts) < 3 or parts[2].lower() not in ("on", "off"):
                        print(f"  {_yellow('Usage')}: objvis {_cyan('<name|#idx> on|off')}")
                    else:
                        ref = _parse_obj_ref(parts[1])
                        visible = parts[2].lower() == "on"
                        await ws.send(json.dumps({"cmd": "setObject", **ref, "visible": visible}))

                elif cmd == "objresetrot":
                    if len(parts) < 2:
                        print(f"  {_yellow('Usage')}: objresetrot {_cyan('<name|#idx>')}")
                    else:
                        ref = _parse_obj_ref(parts[1])
                        await ws.send(json.dumps({"cmd": "resetObjectRotation", **ref}))

                elif cmd == "objresetscale":
                    if len(parts) < 2:
                        print(f"  {_yellow('Usage')}: objresetscale {_cyan('<name|#idx>')}")
                    else:
                        ref = _parse_obj_ref(parts[1])
                        await ws.send(json.dumps({"cmd": "resetObjectScale", **ref}))

                elif cmd == "demo":
                    await demo_sequence(ws, config, num_joints)

                elif cmd == "plan":
                    # plan --start <axes> --end <axes> [--stepsize <deg>] [--steptime <ms>]
                    # Axes can be positional:  0 0 0 0 0 0
                    # or named:                mu=0 theta=30  (unspecified axes default to 0)
                    raw = parts[1:]

                    def _get_flag(flag, default=None):
                        if flag in raw:
                            i = raw.index(flag)
                            return raw[i + 1], raw[:i] + raw[i+2:]
                        return default, raw

                    def _get_angle_list(flag):
                        nonlocal raw
                        if flag not in raw:
                            return None
                        i = raw.index(flag)
                        tokens = []
                        j = i + 1
                        while j < len(raw) and not raw[j].startswith('--'):
                            tokens.append(raw[j])
                            j += 1
                        raw = raw[:i] + raw[j:]
                        if not tokens:
                            return None
                        if any('=' in t for t in tokens):
                            # Named axis mode: mu=30 theta=45 ...
                            vals = [0.0] * n_movable
                            for t in tokens:
                                if '=' not in t:
                                    raise ValueError(f"Mix of positional and named axes: {t!r}")
                                name, val = t.split('=', 1)
                                mi = _resolve_axis_name(name.strip())
                                if mi is None:
                                    names = ', '.join(n for _, n in movable_joints)
                                    raise ValueError(f"Unknown axis {name!r}. Available: {names}")
                                vals[mi] = float(val)
                            return vals
                        else:
                            # Positional mode
                            return [float(t) for t in tokens]

                    stepsize_str, raw = _get_flag('--stepsize')
                    steptime_str, raw = _get_flag('--steptime')
                    start_vals = _get_angle_list('--start')
                    end_vals   = _get_angle_list('--end')

                    if start_vals is None or end_vals is None:
                        names_eg = ' '.join(f'{n.split()[0].lower()}=0'
                                            for _, n in movable_joints[:3])
                        print(f"  {_yellow('Usage')}: plan "
                              f"{_cyan('--start')} {_cyan('<axes>')} "
                              f"{_cyan('--end')} {_cyan('<axes>')} "
                              f"[{_cyan('--stepsize')} {_cyan('<deg>')}] "
                              f"[{_cyan('--steptime')} {_cyan('<ms>')}]")
                        print(f"  Positional: {_dim(' '.join(['0'] * n_movable))}")
                        print(f"  Named:      {_dim(names_eg + ' ...')}  "
                              f"{_dim('(unspecified axes default to 0)')}")
                        continue

                    if len(start_vals) != n_movable or len(end_vals) != n_movable:
                        print(f"  {_yellow('Error')}: expected {n_movable} axis values, "
                              f"got start={len(start_vals)} end={len(end_vals)}")
                        continue

                    step_deg = float(stepsize_str) if stepsize_str is not None else 5.0
                    step_ms  = int(steptime_str)   if steptime_str  is not None else 80
                    await plan_sequence(ws, config_path, start_vals, end_vals,
                                        step_ms=step_ms, step_deg=step_deg)

                elif cmd == "scan":
                    # scan <axis> <start> <end> <step> [<axis> ...] [--steptime <ms>]
                    # Subsequent axes: 3 nums = grid (start end step)
                    #                  2 nums = coupled (start step)
                    raw = parts[1:]

                    # Extract optional --steptime flag
                    steptime_str = None
                    if '--steptime' in raw:
                        i = raw.index('--steptime')
                        steptime_str = raw[i + 1]
                        raw = raw[:i] + raw[i+2:]

                    if len(raw) < 4:
                        first_name = movable_joints[0][1].split()[0].lower() if movable_joints else 'J1'
                        second_name = movable_joints[1][1].split()[0].lower() if len(movable_joints) > 1 else 'J2'
                        print(f"  {_yellow('Usage')}: scan "
                              f"{_cyan('<axis> <start> <end> <step>')} "
                              f"[{_cyan('<axis> ...')}] "
                              f"[{_cyan('--steptime')} {_cyan('<ms>')}]")
                        print(f"  1D:      {_dim(f'scan {first_name} 0 50 5')}")
                        print(f"  Coupled: {_dim(f'scan {first_name} 10 20 1 {second_name} 20 2')}")
                        print(f"  Grid:    {_dim(f'scan {first_name} 0 20 1 {second_name} 0 30 2')}")
                        print(f"  Multi:   {_dim('scan DeviceA:axis1 0 50 5 DeviceB:axis1 0 30 5')}")
                        continue

                    # Parse axis groups by detecting axis names vs numbers
                    def _is_number(s):
                        try:
                            float(s)
                            return True
                        except ValueError:
                            return False

                    groups = []
                    i = 0
                    while i < len(raw):
                        axis_name = raw[i]
                        i += 1
                        nums = []
                        while i < len(raw) and _is_number(raw[i]):
                            nums.append(float(raw[i]))
                            i += 1
                        groups.append((axis_name, nums))

                    # Validate first group (must have 3 numbers: start end step)
                    valid = True
                    if not groups or len(groups[0][1]) != 3:
                        print(f"  {_yellow('Error')}: first axis needs 3 values (start end step)")
                        continue

                    step_ms = int(steptime_str) if steptime_str is not None else 80

                    # Check if any axis uses device:axis syntax
                    is_multi = any(':' in g[0] for g in groups)

                    if is_multi:
                        # ── Multi-device scan ────────────────────────
                        parsed = []
                        for axis_name, nums in groups:
                            if ':' in axis_name:
                                dp, ap = axis_name.split(':', 1)
                            else:
                                dp, ap = device_name, axis_name
                            parsed.append((dp, ap, nums))

                        dev_names_used = list(dict.fromkeys(g[0] for g in parsed))
                        dev_configs = await _fetch_device_configs(ws, dev_names_used)
                        if dev_configs is None:
                            print(f"  {_bred('Error')}: could not load config for one or more devices. "
                                  f"Are they loaded in the viewer?")
                            continue

                        device_axes = []
                        for gi, (dp, ap, nums) in enumerate(parsed):
                            dc = dev_configs[dp]
                            mi = _resolve_axis_for_device(ap, dc["movable_joints"])
                            if mi is None:
                                names = ', '.join(n for _, n in dc["movable_joints"])
                                print(f"  {_yellow('Error')}: unknown axis {ap!r} on "
                                      f"{dp}. Available: {names}")
                                valid = False
                                break
                            jidx = dc["movable_joints"][mi][0]
                            label = f"{dp}:{dc['movable_joints'][mi][1]}"
                            if gi == 0 or len(nums) == 3:
                                device_axes.append((dp, jidx, "grid",
                                                    nums[0], nums[1], nums[2], label))
                            elif len(nums) == 2:
                                device_axes.append((dp, jidx, "coupled",
                                                    nums[0], nums[1], label))
                            else:
                                print(f"  {_yellow('Error')}: axis {ap!r} needs 3 values "
                                      f"(start end step) or 2 (start step), got {len(nums)}")
                                valid = False
                                break
                        if not valid:
                            continue
                        await multi_scan_sequence(ws, device_axes, step_ms=step_ms)

                    else:
                        # ── Single-device scan (existing) ────────────
                        grid_axes = []
                        coupled_axes = []
                        for gi, (axis_name, nums) in enumerate(groups):
                            axis_idx = _resolve_axis_name(axis_name)
                            if axis_idx is None:
                                names = ', '.join(n for _, n in movable_joints)
                                print(f"  {_yellow('Error')}: unknown axis {axis_name!r}. "
                                      f"Available: {names}")
                                valid = False
                                break
                            if gi == 0:
                                grid_axes.append((axis_idx, nums[0], nums[1], nums[2]))
                            elif len(nums) == 3:
                                grid_axes.append((axis_idx, nums[0], nums[1], nums[2]))
                            elif len(nums) == 2:
                                coupled_axes.append((axis_idx, nums[0], nums[1]))
                            else:
                                print(f"  {_yellow('Error')}: axis {axis_name!r} needs 3 values "
                                      f"(start end step) or 2 values (start step), "
                                      f"got {len(nums)}")
                                valid = False
                                break
                        if not valid:
                            continue
                        await scan_sequence(ws, movable_joints, num_joints,
                                            grid_axes, coupled_axes=coupled_axes,
                                            step_ms=step_ms)

                else:
                    print(f"  {_yellow(cmd)}: command not found. Type {_bold('help')} for usage.")

            except ValueError as e:
                print(f"  {_bred('Error')}: invalid number — {e}")
            except Exception as e:
                print(f"  {_bred('Error')}: {e}")

    except KeyboardInterrupt:
        pass
    finally:
        _save_history()
        listener.cancel()
        await ws.close()
        print(f"\n  {_dim('Connection closed.')}")


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Robot/Device Remote Control Terminal",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Examples:\n"
               "  python remote_control.py\n"
               "  python remote_control.py --config i16_config.json\n"
               "  python remote_control.py --url ws://192.168.1.100:8080/ws --config robot_config.json\n"
               "  python remote_control.py --session ab12cd34\n",
    )
    parser.add_argument("--url", default="ws://localhost:8080/ws",
                        help="WebSocket URL (default: ws://localhost:8080/ws)")
    parser.add_argument("--config", default="robot_config.json",
                        help="Path to device config JSON (default: robot_config.json)")
    parser.add_argument("--session", default=None,
                        help="Session ID of the viewer instance to connect to. "
                             "Omit to broadcast to all viewers. "
                             "Run 'sessions' command to list active sessions.")
    args = parser.parse_args()
    url = args.url
    if args.session:
        sep = "&" if "?" in url else "?"
        url = f"{url}{sep}session={args.session}"
    asyncio.run(interactive(url, args.config))


if __name__ == "__main__":
    main()
