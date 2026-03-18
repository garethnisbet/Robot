#!/usr/bin/env python3
"""
Meca500 Robot — Remote Control Terminal

Interactive terminal client for controlling the robot viewer via WebSocket API.
Provides a Linux terminal-like experience with command history, tab completion,
and colored output.

Usage:
    pip install websockets
    python remote_control.py [--url ws://localhost:8080/ws]
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

HISTORY_FILE = os.path.expanduser("~/.meca500_history")
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
    "state", "home", "fk", "ik", "joints", "move", "target",
    "collision", "collisions", "objects", "obj", "objpos", "objrot",
    "objscale", "objvis", "demo", "clear", "history", "help",
    "quit", "exit",
]

COLLISION_ARGS = ["on", "off"]
OBJVIS_ARGS = ["on", "off"]

class Completer:
    def __init__(self):
        self.matches = []

    def complete(self, text, state):
        if state == 0:
            line = readline.get_line_buffer()
            parts = line.lstrip().split()
            if len(parts) <= 1:
                # Complete command name
                self.matches = [c + " " for c in COMMANDS if c.startswith(text)]
            elif parts[0] == "collision" and len(parts) == 2:
                self.matches = [a + " " for a in COLLISION_ARGS if a.startswith(text)]
            elif parts[0] == "objvis" and len(parts) == 3:
                self.matches = [a + " " for a in OBJVIS_ARGS if a.startswith(text)]
            else:
                self.matches = []
        return self.matches[state] if state < len(self.matches) else None

def _setup_readline():
    _load_history()
    readline.set_completer(Completer().complete)
    readline.parse_and_bind("tab: complete")
    # Treat / # - as word characters for completion
    readline.set_completer_delims(" \t\n")

# ── Prompt ───────────────────────────────────────────────────────────────────

def _prompt():
    if _COLORS:
        return "\001\033[1;32m\002meca500\001\033[0m\002:\001\033[1;34m\002~\001\033[0m\002$ "
    return "meca500:~$ "

# ── Banner ───────────────────────────────────────────────────────────────────

BANNER = r"""
  __  __                 ____   ___   ___
 |  \/  | ___  ___ __ _| ___| / _ \ / _ \
 | |\/| |/ _ \/ __/ _` |___ \| | | | | | |
 | |  | |  __/ (_| (_| |___) | |_| | |_| |
 |_|  |_|\___|\___\__,_|____/ \___/ \___/

"""

def _print_banner(url):
    if _COLORS:
        print(_cyan(BANNER))
    else:
        print(BANNER)
    print(f"  Remote Control Terminal v1.0")
    print(f"  Server: {_bold(url) if _COLORS else url}")
    print()

# ── Help (man-page style) ───────────────────────────────────────────────────

def _print_help():
    title = "MECA500(1)"
    header = f"{title:<30}Robot Remote Control{title:>30}"
    sections = [
        (header, None),
        ("NAME", "    meca500 - interactive remote control terminal for Meca500 robot viewer"),
        ("SYNOPSIS", "    python remote_control.py [--url ws://HOST:PORT/ws]"),
        ("ROBOT COMMANDS", f"""\
    {_bold('state') if _COLORS else 'state'}
        Request current robot state (joints, end-effector pose, collisions).

    {_bold('home') if _COLORS else 'home'}
        Move all joints to 0 degrees (home position).

    {_bold('fk') if _COLORS else 'fk'}
        Switch to Forward Kinematics mode.

    {_bold('ik') if _COLORS else 'ik'}
        Switch to Inverse Kinematics mode.

    {_bold('joints') if _COLORS else 'joints'} {_cyan('j1 j2 j3 j4 j5 j6') if _COLORS else 'j1 j2 j3 j4 j5 j6'}
        Set all six joint angles in degrees.

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
        List all imported STL objects.

    {_bold('obj') if _COLORS else 'obj'} {_cyan('<name|#idx>') if _COLORS else '<name|#idx>'}
        Get details for an object by name or index (e.g. #0).

    {_bold('objpos') if _COLORS else 'objpos'} {_cyan('<name|#idx> x y z') if _COLORS else '<name|#idx> x y z'}
        Set object position in mm.

    {_bold('objrot') if _COLORS else 'objrot'} {_cyan('<name|#idx> rx ry rz') if _COLORS else '<name|#idx> rx ry rz'}
        Set object rotation in degrees.

    {_bold('objscale') if _COLORS else 'objscale'} {_cyan('<name|#idx> s') if _COLORS else '<name|#idx> s'} | {_cyan('sx sy sz') if _COLORS else 'sx sy sz'}
        Set uniform or per-axis scale.

    {_bold('objvis') if _COLORS else 'objvis'} {_cyan('<name|#idx>') if _COLORS else '<name|#idx>'} {_cyan('on') if _COLORS else 'on'}|{_cyan('off') if _COLORS else 'off'}
        Show or hide an object."""),
        ("TERMINAL COMMANDS", f"""\
    {_bold('demo') if _COLORS else 'demo'}
        Run an automated demo sequence through several poses.

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
    Tab         Auto-complete commands
    Ctrl+A      Move cursor to start of line
    Ctrl+E      Move cursor to end of line
    Ctrl+K      Delete from cursor to end of line
    Ctrl+U      Delete entire line
    Ctrl+W      Delete previous word
    Ctrl+R      Reverse history search
    Ctrl+C      Cancel current input
    Ctrl+D      Exit terminal"""),
        ("EXAMPLES", f"""\
    $ joints 0 -30 60 0 45 90
    $ move 150 100 300 45 0 0
    $ objpos MyPart 100 50 0
    $ collision on
    $ obj #0"""),
    ]
    print()
    for heading, body in sections:
        if body is None:
            # Title line
            print(_bold(heading) if _COLORS else heading)
        else:
            print(f"  {_byellow(heading) if _COLORS else heading}")
            print(body)
        print()

# ── Output formatting ────────────────────────────────────────────────────────

def _clear_line():
    """Clear the current line (remove partial prompt before printing output)."""
    print("\r\033[K", end="")

def _reprint_prompt():
    """Re-display the prompt after output."""
    print(_prompt(), end="", flush=True)


async def print_incoming(ws):
    """Background task to print state messages from the viewer."""
    try:
        async for message in ws:
            data = json.loads(message)
            _clear_line()

            if data.get("type") == "state":
                joints = data["joints"]
                ee = data["eePosition"]
                ori = data.get("eeOrientation", [0, 0, 0])
                mode = data.get("mode", "?")
                err = data.get("ikError")

                j_str = ", ".join(f"{a:7.1f}" for a in joints)
                mode_c = _bgreen(mode) if mode == "FK" else _bcyan(mode)

                print(f"  {_bold('STATE')} | mode={mode_c}  joints=[{_white(j_str)}]")
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

                _reprint_prompt()

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
                _reprint_prompt()

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
                _reprint_prompt()

            elif data.get("type") == "object":
                o = data
                p, r, s = o["position"], o["rotation"], o["scale"]
                vis = _green("visible") if o.get("visible", True) else _dim("hidden")
                idx_str = o["index"]
                print(f"  {_dim(f'[{idx_str}]')} {_bold(o['name'])}  {vis}")
                print(f"    pos  = ({_cyan(f'{p[0]:.1f}')}, {_cyan(f'{p[1]:.1f}')}, {_cyan(f'{p[2]:.1f}')}) mm")
                print(f"    rot  = ({_magenta(f'{r[0]:.1f}')}, {_magenta(f'{r[1]:.1f}')}, {_magenta(f'{r[2]:.1f}')}) deg")
                print(f"    scale= ({s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f})")
                _reprint_prompt()

            elif data.get("type") == "error" or "error" in data:
                print(f"  {_bred('ERROR')}: {_red(data.get('error', 'unknown error'))}")
                _reprint_prompt()

    except websockets.ConnectionClosed:
        pass


# ── Demo sequence ────────────────────────────────────────────────────────────

async def demo_sequence(ws):
    """Run a demo that moves the robot through several poses."""
    poses = [
        ("Home position", {"cmd": "home"}),
        ("FK: shoulder up", {"cmd": "setJoints", "angles": [0, -30, 0, 0, 0, 0]}),
        ("FK: elbow bend", {"cmd": "setJoints", "angles": [0, -30, 60, 0, 0, 0]}),
        ("FK: demo pose", {"cmd": "setJoints", "angles": [0, -30, 60, 0, 45, 90]}),
        ("FK: base rotate", {"cmd": "setJoints", "angles": [45, -30, 60, 0, 45, 90]}),
        ("Switch to IK", {"cmd": "setMode", "mode": "IK"}),
        ("IK: move up", {"cmd": "setIKTarget", "position": [150, 0, 350], "orientation": [0, 0, 0]}),
        ("IK: move right", {"cmd": "setIKTarget", "position": [150, 100, 350], "orientation": [0, 0, 0]}),
        ("IK: move down", {"cmd": "setIKTarget", "position": [200, 100, 200], "orientation": [0, 0, 0]}),
        ("IK: rotate wrist", {"cmd": "setIKTarget", "position": [200, 100, 200], "orientation": [45, 0, 0]}),
        ("Back to FK", {"cmd": "setMode", "mode": "FK"}),
        ("Home", {"cmd": "home"}),
    ]

    for i, (desc, cmd) in enumerate(poses, 1):
        print(f"  {_dim(f'[{i}/{len(poses)}]')} {_cyan(desc)}")
        await ws.send(json.dumps(cmd))
        await asyncio.sleep(1.5)

    print(f"  {_bgreen('Demo complete!')}")


# ── Object reference parsing ─────────────────────────────────────────────────

def _parse_obj_ref(token):
    """Parse an object reference: '#0' for index, or name string."""
    if token.startswith("#") and token[1:].isdigit():
        return {"index": int(token[1:])}
    return {"object": token}


# ── Main interactive loop ────────────────────────────────────────────────────

async def interactive(url):
    _setup_readline()
    _print_banner(url)
    print(f"  Connecting to {_dim(url)} ...")

    try:
        ws = await websockets.connect(url)
    except (ConnectionRefusedError, OSError) as e:
        print(f"  {_bred('Connection failed')}: {e}")
        print(f"  Is the server running?  python server.py")
        return

    print(f"  {_bgreen('Connected!')} Type {_bold('help')} for commands, {_bold('Tab')} to complete.\n")

    # Start background listener
    listener = asyncio.create_task(print_incoming(ws))

    loop = asyncio.get_event_loop()
    prompt = _prompt()

    try:
        while True:
            try:
                line = await loop.run_in_executor(None, lambda: input(prompt))
            except EOFError:
                break
            except KeyboardInterrupt:
                print()  # newline after ^C
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
                    _print_help()

                elif cmd == "clear":
                    os.system("clear" if os.name != "nt" else "cls")

                elif cmd == "history":
                    n = readline.get_current_history_length()
                    start = max(1, n - 49)  # last 50 entries
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
                    if len(parts) != 7:
                        print(f"  {_yellow('Usage')}: joints {_cyan('j1 j2 j3 j4 j5 j6')}")
                    else:
                        angles = [float(x) for x in parts[1:7]]
                        await ws.send(json.dumps({"cmd": "setJoints", "angles": angles}))

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

                elif cmd == "demo":
                    await demo_sequence(ws)

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
        description="Meca500 Remote Control Terminal",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Examples:\n"
               "  python remote_control.py\n"
               "  python remote_control.py --url ws://192.168.1.100:8080/ws\n",
    )
    parser.add_argument("--url", default="ws://localhost:8080/ws",
                        help="WebSocket URL (default: ws://localhost:8080/ws)")
    args = parser.parse_args()
    print(args.url)
    asyncio.run(interactive(args.url))


if __name__ == "__main__":
    main()
