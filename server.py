#!/usr/bin/env python3
"""
Robot Visualisation — WebSocket + HTTP Server

Serves the Three.js viewer and provides a two-way WebSocket API
for remote robot control.

Usage:
    pip install aiohttp
    python server.py [--port 8000] [--config meca500_config.json]

WebSocket endpoint: ws://localhost:8000/ws

API Protocol (JSON over WebSocket):
────────────────────────────────────
Commands (send TO the viewer):
  {"cmd": "getState"}
  {"cmd": "setJoints", "angles": [j1, j2, ...jN]}                # degrees (all joints)
  {"cmd": "setSingleJoint", "index": i, "angle": deg}           # set one joint by index
  {"cmd": "home"}                                                 # all joints to 0
  {"cmd": "setMode", "mode": "FK"}                                # "FK" or "IK"
  {"cmd": "setIKTarget", "position": [x,y,z], "orientation": [a,b,g]}  # mm, deg (Z-up)
  {"cmd": "moveTo", "position": [x,y,z], "orientation": [a,b,g]}       # switch to IK + set target
  {"cmd": "translateDevice", "delta": [dx,dy,dz], "space": "parent"}   # mm; space: parent|local|world
  {"cmd": "rotateDevice", "delta": [rx,ry,rz], "space": "parent"}      # deg; space: parent|local|world
  {"cmd": "translateObject", "name": "Cube", "delta": [dx,dy,dz], "space": "parent"}
  {"cmd": "rotateObject", "name": "Cube", "delta": [rx,ry,rz], "space": "parent"}

State (sent FROM the viewer):
  {
    "type": "state",
    "joints": [j1..jN],          # degrees (all joints)
    "eePosition": [x, y, z],    # mm, Z-up
    "eeOrientation": [a, b, g], # degrees, Euler (a=Rx, b=Ry, g=Rz) relative to home (home = [0, 90, 0])
    "mode": "FK" | "IK",
    "ikError": null | <mm>
  }

Session routing:
  Viewers connect with ?role=viewer&session=<id>
  Controllers connect with ?role=controller&session=<id>
  If no session is given for a controller, messages are broadcast to all viewers.
  GET /sessions  →  list active session IDs
"""

import argparse
import asyncio
import json
import logging
import uuid
from pathlib import Path

from aiohttp import web

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(message)s")
log = logging.getLogger("robot-server")

ROOT = Path(__file__).parent

# sessions[session_id] = set of viewer websockets
sessions: dict = {}
# session_controllers[session_id | None] = set of controller websockets
# None key = "all sessions" (no session filter)
session_controllers: dict = {}


def _short_id() -> str:
    return uuid.uuid4().hex[:8]


def _viewer_count() -> int:
    return sum(len(v) for v in sessions.values())


def _controller_count() -> int:
    return sum(len(v) for v in session_controllers.values())


async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    role = request.query.get("role", "controller")
    peer = request.remote

    if role == "viewer":
        session_id = request.query.get("session") or _short_id()
        sessions.setdefault(session_id, set()).add(ws)
        log.info(
            f"viewer [{session_id}] connected from {peer}"
            f"  (sessions={len(sessions)}, viewers={_viewer_count()}, controllers={_controller_count()})"
        )
        # Confirm assigned session ID back to the viewer
        await ws.send_json({"type": "session", "id": session_id})
    else:
        session_id = request.query.get("session") or None
        session_controllers.setdefault(session_id, set()).add(ws)
        label = session_id if session_id else "*"
        log.info(
            f"controller [{label}] connected from {peer}"
            f"  (sessions={len(sessions)}, viewers={_viewer_count()}, controllers={_controller_count()})"
        )

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    await ws.send_json({"error": "invalid JSON"})
                    continue

                if role == "controller":
                    # Route to the targeted session's viewers, or all viewers if no session filter
                    if session_id:
                        target_viewers = list(sessions.get(session_id, set()))
                    else:
                        target_viewers = [v for vset in sessions.values() for v in vset]
                    for v in target_viewers:
                        try:
                            await v.send_json(data)
                        except Exception:
                            for s in sessions.values():
                                s.discard(v)

                elif role == "viewer":
                    # Forward state to controllers targeting this session + unfiltered controllers
                    target_controllers = list(
                        session_controllers.get(session_id, set()) |
                        session_controllers.get(None, set())
                    )
                    for c in target_controllers:
                        try:
                            await c.send_json(data)
                        except Exception:
                            for s in session_controllers.values():
                                s.discard(c)

            elif msg.type in (web.WSMsgType.ERROR, web.WSMsgType.CLOSE):
                break
    finally:
        if role == "viewer":
            s = sessions.get(session_id, set())
            s.discard(ws)
            if not s:
                sessions.pop(session_id, None)
            log.info(
                f"viewer [{session_id}] disconnected from {peer}"
                f"  (sessions={len(sessions)}, viewers={_viewer_count()}, controllers={_controller_count()})"
            )
        else:
            label = session_id if session_id else "*"
            s = session_controllers.get(session_id, set())
            s.discard(ws)
            if not s:
                session_controllers.pop(session_id, None)
            log.info(
                f"controller [{label}] disconnected from {peer}"
                f"  (sessions={len(sessions)}, viewers={_viewer_count()}, controllers={_controller_count()})"
            )

    return ws


async def healthz_handler(request):
    return web.Response(text="ok")


async def sessions_handler(request):
    """Return list of active viewer sessions."""
    data = [
        {"id": sid, "viewers": len(vws)}
        for sid, vws in sessions.items()
        if vws
    ]
    return web.json_response(data)


def create_app(config_path=None):
    app = web.Application()

    config_name = Path(config_path).name if config_path else "meca500_config.json"

    async def index_handler(request):
        if "config" not in request.query:
            raise web.HTTPFound(f"/?config={config_name}")
        return web.FileResponse(ROOT / "threejs_scene.html")

    app.router.add_get("/healthz", healthz_handler)
    app.router.add_get("/sessions", sessions_handler)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/", index_handler)

    app.router.add_static("/", ROOT, show_index=False)
    return app


def main():
    parser = argparse.ArgumentParser(description="Robot Visualisation WebSocket Server")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--config", default="meca500_config.json",
                        help="Path to robot config JSON (default: meca500_config.json)")
    args = parser.parse_args()

    app = create_app(config_path=args.config)
    log.info(f"Starting server on http://{args.host}:{args.port}")
    config_name = Path(args.config).name
    log.info(f"Open http://localhost:{args.port}/?config={config_name} in your browser")
    log.info(f"WebSocket API at ws://localhost:{args.port}/ws")
    log.info(f"Active sessions at http://localhost:{args.port}/sessions")
    log.info(f"Robot config: {args.config}")
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
