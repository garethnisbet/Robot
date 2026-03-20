#!/usr/bin/env python3
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
import sys

import websockets

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
    "state", "home", "fk", "ik", "joints", "joint", "move", "target",
    "collision", "collisions", "objects", "obj", "objpos", "objrot",
    "objscale", "objvis", "objresetrot", "objresetscale", "demo", "clear", "history", "help",
    "quit", "exit",
]

COLLISION_ARGS = ["on", "off"]
OBJVIS_ARGS = ["on", "off"]

class Completer:
    def __init__(self, joint_names=None):
        self.matches = []
        self.joint_names = joint_names or []

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
            elif parts[0] == "joint" and len(parts) == 2:
                self.matches = [n + " " for n in self.joint_names if n.lower().startswith(text.lower())]
            else:
                self.matches = []
        return self.matches[state] if state < len(self.matches) else None

# ── Config loading ──────────────────────────────────────────────────────────

def load_config(config_path):
    """Load the robot/device config to get joint info."""
    try:
        with open(config_path) as f:
            config = json.load(f)
        return config
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"  {_yellow('Warning')}: Could not load config {config_path}: {e}")
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
        ("SYNOPSIS", f"    python remote_control.py [--url ws://HOST:PORT/ws] [--config CONFIG.json]"),
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
        Set IK target position/orientation without switching mode."""),
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
    $ collision on"""),
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


async def print_incoming(ws, device_name, movable_joints):
    """Background task to print state messages from the viewer."""
    try:
        async for message in ws:
            data = json.loads(message)
            _clear_line()

            if data.get("type") == "state":
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


# ── Object reference parsing ─────────────────────────────────────────────────

def _parse_obj_ref(token):
    if token.startswith("#") and token[1:].isdigit():
        return {"index": int(token[1:])}
    return {"object": token}


# ── Main interactive loop ────────────────────────────────────────────────────

async def interactive(url, config_path):
    config = load_config(config_path)
    device_name = config["name"] if config else "Robot"
    num_joints = len(config["joints"]) if config else 6
    movable_joints = get_movable_joints(config)
    n_movable = len(movable_joints)
    joint_name_to_idx = {name.lower(): idx for idx, name in movable_joints}

    _load_history()
    completer = Completer(joint_names=[name for _, name in movable_joints])
    readline.set_completer(completer.complete)
    readline.parse_and_bind("tab: complete")
    readline.set_completer_delims(" \t\n")

    _print_banner(device_name, url, movable_joints)
    print(f"  Connecting to {_dim(url)} ...")

    try:
        ws = await websockets.connect(url)
    except (ConnectionRefusedError, OSError) as e:
        print(f"  {_bred('Connection failed')}: {e}")
        print(f"  Is the server running?  python server.py --config {config_path}")
        return

    print(f"  {_bgreen('Connected!')} Type {_bold('help')} for commands, {_bold('Tab')} to complete.\n")

    listener = asyncio.create_task(print_incoming(ws, device_name, movable_joints))

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

                elif cmd == "joint":
                    if len(parts) < 3:
                        print(f"  {_yellow('Usage')}: joint {_cyan('<name> <angle>')}")
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
                        # Get current state, modify one joint, send all
                        # For simplicity, send a getState first, but we can also just
                        # build from scratch. Send setJoints with only the target joint changed.
                        # We need current angles — request state and set in callback.
                        # Simpler: build array with 0s except the target joint
                        # Actually, the viewer keeps its own state, so we should
                        # get state, modify, and re-send. But that requires async round-trip.
                        # Alternative: use a dedicated single-joint command if viewer supports it.
                        # For now, just print a note and set via setJoints with all 0s except target.
                        # Better approach: viewer accepts partial joint updates.
                        # Let's just send a setJoint command that the viewer can handle.
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
               "  python remote_control.py --url ws://192.168.1.100:8080/ws --config robot_config.json\n",
    )
    parser.add_argument("--url", default="ws://localhost:8080/ws",
                        help="WebSocket URL (default: ws://localhost:8080/ws)")
    parser.add_argument("--config", default="robot_config.json",
                        help="Path to device config JSON (default: robot_config.json)")
    args = parser.parse_args()
    asyncio.run(interactive(args.url, args.config))


if __name__ == "__main__":
    main()
