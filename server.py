#!/usr/bin/env python3
"""
Meca500 Robot Visualisation — WebSocket + HTTP Server

Serves the Three.js viewer and provides a two-way WebSocket API
for remote robot control.

Usage:
    pip install aiohttp
    python server.py [--port 8000]

WebSocket endpoint: ws://localhost:8000/ws

API Protocol (JSON over WebSocket):
────────────────────────────────────
Commands (send TO the viewer):
  {"cmd": "getState"}
  {"cmd": "setJoints", "angles": [j1, j2, j3, j4, j5, j6]}     # degrees
  {"cmd": "home"}                                                 # all joints to 0
  {"cmd": "setMode", "mode": "FK"}                                # "FK" or "IK"
  {"cmd": "setIKTarget", "position": [x,y,z], "orientation": [a,b,g]}  # mm, deg (Z-up)
  {"cmd": "moveTo", "position": [x,y,z], "orientation": [a,b,g]}       # switch to IK + set target

State (sent FROM the viewer):
  {
    "type": "state",
    "joints": [j1..j6],          # degrees
    "eePosition": [x, y, z],    # mm, Z-up
    "eeOrientation": [a, b, g], # degrees, ZYX Euler
    "mode": "FK" | "IK",
    "ikError": null | <mm>
  }
"""

import argparse
import asyncio
import json
import logging
from pathlib import Path

from aiohttp import web

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(message)s")
log = logging.getLogger("robot-server")

ROOT = Path(__file__).parent

# Track connected clients
viewers = set()      # browser viewer(s)
controllers = set()  # remote control clients


async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    # Determine role from query param: ?role=viewer (default) or ?role=controller
    role = request.query.get("role", "controller")
    pool = viewers if role == "viewer" else controllers
    pool.add(ws)
    peer = request.remote
    log.info(f"{role} connected from {peer}  (viewers={len(viewers)}, controllers={len(controllers)})")

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    await ws.send_json({"error": "invalid JSON"})
                    continue

                if role == "controller":
                    # Forward command to all viewers
                    for v in list(viewers):
                        try:
                            await v.send_json(data)
                        except Exception:
                            viewers.discard(v)

                elif role == "viewer":
                    # Forward state/responses to all controllers
                    for c in list(controllers):
                        try:
                            await c.send_json(data)
                        except Exception:
                            controllers.discard(c)

            elif msg.type in (web.WSMsgType.ERROR, web.WSMsgType.CLOSE):
                break
    finally:
        pool.discard(ws)
        log.info(f"{role} disconnected from {peer}  (viewers={len(viewers)}, controllers={len(controllers)})")

    return ws


async def index_handler(request):
    return web.FileResponse(ROOT / "threejs_scene.html")


async def healthz_handler(request):
    return web.Response(text="ok")


def create_app():
    app = web.Application()
    app.router.add_get("/healthz", healthz_handler)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/", index_handler)
    # Serve static files (GLB, images, etc.)
    app.router.add_static("/", ROOT, show_index=False)
    return app


def main():
    parser = argparse.ArgumentParser(description="Meca500 Robot WebSocket Server")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    args = parser.parse_args()

    app = create_app()
    log.info(f"Starting server on http://{args.host}:{args.port}")
    log.info(f"Open http://localhost:{args.port} in your browser")
    log.info(f"WebSocket API at ws://localhost:{args.port}/ws")
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
