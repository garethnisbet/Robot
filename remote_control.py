#!/usr/bin/env python3
"""
Meca500 Robot — Remote Control Client

Example Python client for controlling the robot viewer via the WebSocket API.

Usage:
    pip install websockets
    python remote_control.py [--url ws://localhost:8000/ws]

Commands (interactive):
    state           — Request current robot state
    home            — Move all joints to 0°
    fk              — Switch to FK mode
    ik              — Switch to IK mode
    joints j1 j2 j3 j4 j5 j6   — Set joint angles (degrees)
    move x y z [a b g]          — Move to position (mm) + optional orientation (deg)
    target x y z [a b g]        — Set IK target without switching mode
    collision [on|off]           — Toggle or set collision detection
    collisions                   — Get current collision pairs
    objects                      — List imported STL objects
    obj <name|#idx>              — Get object details
    objpos <name|#idx> x y z    — Set object position (mm)
    objrot <name|#idx> rx ry rz — Set object rotation (degrees)
    objscale <name|#idx> s      — Set uniform scale (or sx sy sz)
    demo            — Run a demo sequence
    quit / exit     — Disconnect
"""

import argparse
import asyncio
import json
import sys

import websockets


async def print_incoming(ws):
    """Background task to print state messages from the viewer."""
    try:
        async for message in ws:
            data = json.loads(message)
            if data.get("type") == "state":
                joints = data["joints"]
                ee = data["eePosition"]
                ori = data.get("eeOrientation", [0, 0, 0])
                mode = data.get("mode", "?")
                err = data.get("ikError")
                j_str = ", ".join(f"{a:7.1f}" for a in joints)
                print(f"\r  State | mode={mode}  joints=[{j_str}]")
                print(f"         ee=({ee[0]:.1f}, {ee[1]:.1f}, {ee[2]:.1f})mm  "
                      f"ori=({ori[0]:.1f}, {ori[1]:.1f}, {ori[2]:.1f})°  "
                      f"err={f'{err:.2f}mm' if err is not None else '-'}")
                coll_on = data.get("collisionEnabled", False)
                collisions = data.get("collisions", [])
                if coll_on:
                    if collisions:
                        pairs = ", ".join(f"{c['link']}<>{c['object']}" for c in collisions)
                        print(f"         collision: YES [{pairs}]")
                    else:
                        print(f"         collision: none")
                print("> ", end="", flush=True)
            elif data.get("type") == "collisions":
                enabled = data.get("enabled", False)
                pairs = data.get("pairs", [])
                print(f"\r  Collision detection: {'ON' if enabled else 'OFF'}")
                if enabled:
                    if pairs:
                        for p in pairs:
                            print(f"    {p['link']} <> {p['object']}")
                    else:
                        print(f"    No collisions")
                print("> ", end="", flush=True)
            elif data.get("type") == "objects":
                objs = data.get("objects", [])
                print(f"\r  Imported objects: {len(objs)}")
                for o in objs:
                    p = o["position"]
                    r = o["rotation"]
                    s = o["scale"]
                    vis = "visible" if o.get("visible", True) else "hidden"
                    print(f"    [{o['index']}] {o['name']}  "
                          f"pos=({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})mm  "
                          f"rot=({r[0]:.1f}, {r[1]:.1f}, {r[2]:.1f})°  "
                          f"scale=({s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f})  {vis}")
                print("> ", end="", flush=True)
            elif data.get("type") == "object":
                o = data
                p = o["position"]
                r = o["rotation"]
                s = o["scale"]
                vis = "visible" if o.get("visible", True) else "hidden"
                print(f"\r  [{o['index']}] {o['name']}  {vis}")
                print(f"    pos=({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})mm")
                print(f"    rot=({r[0]:.1f}, {r[1]:.1f}, {r[2]:.1f})°")
                print(f"    scale=({s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f})")
                print("> ", end="", flush=True)
            elif data.get("type") == "error" or "error" in data:
                print(f"\r  Error: {data['error']}")
                print("> ", end="", flush=True)
    except websockets.ConnectionClosed:
        pass


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

    for desc, cmd in poses:
        print(f"  Demo: {desc}")
        await ws.send(json.dumps(cmd))
        await asyncio.sleep(1.5)

    print("  Demo complete!")


def _parse_obj_ref(token):
    """Parse an object reference: '#0' for index, or name string."""
    if token.startswith("#") and token[1:].isdigit():
        return {"index": int(token[1:])}
    return {"object": token}


async def interactive(url):
    print(f"Connecting to {url} ...")
    async with websockets.connect(url) as ws:
        print("Connected! Type 'help' for commands.\n")

        # Start background listener
        listener = asyncio.create_task(print_incoming(ws))

        loop = asyncio.get_event_loop()
        try:
            while True:
                try:
                    line = await loop.run_in_executor(None, lambda: input("> "))
                except EOFError:
                    break

                line = line.strip()
                if not line:
                    continue

                parts = line.split()
                cmd = parts[0].lower()

                if cmd in ("quit", "exit", "q"):
                    break
                elif cmd == "help":
                    print(__doc__)
                elif cmd == "state":
                    await ws.send(json.dumps({"cmd": "getState"}))
                elif cmd == "home":
                    await ws.send(json.dumps({"cmd": "home"}))
                elif cmd == "fk":
                    await ws.send(json.dumps({"cmd": "setMode", "mode": "FK"}))
                elif cmd == "ik":
                    await ws.send(json.dumps({"cmd": "setMode", "mode": "IK"}))
                elif cmd == "joints" and len(parts) == 7:
                    angles = [float(x) for x in parts[1:7]]
                    await ws.send(json.dumps({"cmd": "setJoints", "angles": angles}))
                elif cmd == "move" and len(parts) >= 4:
                    pos = [float(x) for x in parts[1:4]]
                    ori = [float(x) for x in parts[4:7]] if len(parts) >= 7 else [0, 0, 0]
                    await ws.send(json.dumps({"cmd": "moveTo", "position": pos, "orientation": ori}))
                elif cmd == "target" and len(parts) >= 4:
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
                elif cmd == "obj" and len(parts) >= 2:
                    ref = _parse_obj_ref(parts[1])
                    await ws.send(json.dumps({"cmd": "getObject", **ref}))
                elif cmd == "objpos" and len(parts) >= 5:
                    ref = _parse_obj_ref(parts[1])
                    pos = [float(x) for x in parts[2:5]]
                    await ws.send(json.dumps({"cmd": "setObject", **ref, "position": pos}))
                elif cmd == "objrot" and len(parts) >= 5:
                    ref = _parse_obj_ref(parts[1])
                    rot = [float(x) for x in parts[2:5]]
                    await ws.send(json.dumps({"cmd": "setObject", **ref, "rotation": rot}))
                elif cmd == "objscale" and len(parts) >= 3:
                    ref = _parse_obj_ref(parts[1])
                    vals = [float(x) for x in parts[2:]]
                    if len(vals) == 1:
                        await ws.send(json.dumps({"cmd": "setObject", **ref, "scale": vals[0]}))
                    elif len(vals) >= 3:
                        await ws.send(json.dumps({"cmd": "setObject", **ref, "scale": vals[:3]}))
                    else:
                        print("  Usage: objscale <name|#idx> s  OR  objscale <name|#idx> sx sy sz")
                elif cmd == "demo":
                    await demo_sequence(ws)
                else:
                    print("  Unknown command. Type 'help' for usage.")

        except KeyboardInterrupt:
            pass
        finally:
            listener.cancel()
            print("\nDisconnected.")


def main():
    parser = argparse.ArgumentParser(description="Meca500 Remote Control Client")
    parser.add_argument("--url", default="ws://localhost:8000/ws",
                        help="WebSocket URL (default: ws://localhost:8000/ws)")
    args = parser.parse_args()
    asyncio.run(interactive(args.url))


if __name__ == "__main__":
    main()
