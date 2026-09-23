"""
Headless engine client — the viewer's own collision check, from Python.

Runs headless/stdio.mjs (Node) as a subprocess and talks to it in the
viewer's WebSocket protocol. The engine runs the viewer's modules, so its
verdicts are the viewer's. Needs Node on PATH and this repo's node_modules,
js/ and model files next to this file.

    from headless_client import HeadlessEngine
    with HeadlessEngine() as engine:
        engine.sync_scene(lambda buffers: fetch_export_from_viewer(buffers))
        verdict = engine.check_path("Meca500", [[0]*6, [30, 20, 10, 0, 0, 0]])
"""

import json
import os
import queue
import shutil
import subprocess
import threading

ROOT = os.path.dirname(os.path.abspath(__file__))


class HeadlessEngineError(RuntimeError):
    pass


class HeadlessEngine:
    def __init__(self, root=ROOT, node=None, start_timeout=30.0):
        self._root = root
        self._node = node or shutil.which("node")
        if not self.available(root, self._node):
            raise HeadlessEngineError(
                "headless engine unavailable: needs node on PATH and this repo's "
                "node_modules (run npm ci)")
        self._proc = subprocess.Popen(
            [self._node, "--import", "./headless/register.mjs", "headless/stdio.mjs"],
            cwd=root, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1,
        )
        self._lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()
        self._lock = threading.Lock()
        self._next_id = 1
        ready = self._read(start_timeout)
        if not ready.get("ready"):
            raise HeadlessEngineError(f"headless engine did not start: {ready}")

    @staticmethod
    def available(root=ROOT, node=None):
        node = node or shutil.which("node")
        return bool(node) and all(os.path.exists(os.path.join(root, p)) for p in (
            "headless/stdio.mjs", "node_modules/three", "node_modules/three-mesh-bvh"))

    def _pump(self):
        for line in self._proc.stdout:
            self._lines.put(line)
        self._lines.put(None)

    def _read(self, timeout):
        try:
            line = self._lines.get(timeout=timeout)
        except queue.Empty:
            raise HeadlessEngineError(f"headless engine timed out after {timeout:.0f} s")
        if line is None:
            raise HeadlessEngineError("headless engine exited")
        return json.loads(line)

    def request(self, msg, timeout=120.0):
        """Send one command; return the list of replies the viewer would send."""
        with self._lock:
            rid = self._next_id
            self._next_id += 1
            self._proc.stdin.write(json.dumps({"id": rid, "msg": msg}) + "\n")
            self._proc.stdin.flush()
            resp = self._read(timeout)
        if resp.get("id") != rid:
            raise HeadlessEngineError(f"reply out of order: {resp}")
        if "error" in resp:
            raise HeadlessEngineError(resp["error"])
        return resp["replies"]

    def _one(self, msg, want, timeout=120.0):
        for r in self.request(msg, timeout):
            if r.get("type") == want:
                return r
            if r.get("type") == "error":
                raise HeadlessEngineError(r.get("error"))
        raise HeadlessEngineError(f"no '{want}' reply to {msg.get('cmd')}")

    def sync(self, payload):
        """Bring the engine's scene in line with an exportScene payload.
        Returns {"needBuffers": True} when the scene's structure changed and
        the payload carries no geometry."""
        return self._one({"cmd": "syncScene", "scene": payload}, "sceneSynced")

    # Points per exportObjectPoints chunk: 12 MB of float32, 16 MB as
    # base64, well inside the relay's 64 MB message limit.
    POINT_CHUNK = 1_000_000

    def sync_scene(self, export, fetch_points=None):
        """Mirror a viewer's scene. `export(buffers)` returns the viewer's
        exportScene payload; geometry is fetched only when the scene's
        structure changed since the last sync. `fetch_points(id, offset,
        count)` returns the viewer's exportObjectPoints reply; it is called
        for visible point clouds and splats the engine has no points for yet
        (each is fetched once and kept). Without it, those stay unchecked."""
        r = self.sync(export(False))
        if r.get("needBuffers"):
            r = self.sync(export(True))
        if fetch_points and r.get("needPoints"):
            for cloud in r["needPoints"]:
                self.upload_points(cloud["id"], fetch_points)
            r = self.sync(export(False))
        return r

    def upload_points(self, obj_id, fetch_points):
        """Copy one object's collision points from the viewer, in chunks."""
        offset, total = 0, None
        while total is None or offset < total:
            chunk = fetch_points(obj_id, offset, self.POINT_CHUNK)
            total = chunk["total"]
            self._one({"cmd": "setObjectPoints", "id": obj_id, "offset": chunk["offset"],
                       "total": total, "positions": chunk["positions"]}, "objectPointsStored")
            if chunk["count"] == 0:
                break
            offset += chunk["count"]

    def check_path(self, device, waypoints, resolution_deg=1.0):
        """Exact verdict for a joint-space path (API degrees). `device` is a
        name, or an int: the device's index in the viewer's device list."""
        msg = {"cmd": "checkPath",
               "waypoints": [[float(v) for v in w] for w in waypoints],
               "resolutionDeg": float(resolution_deg)}
        msg["deviceIndex" if isinstance(device, int) else "device"] = device
        return self._one(msg, "pathCheck")

    def close(self):
        if self._proc.poll() is None:
            self._proc.stdin.close()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
