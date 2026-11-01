"""Start the VRRobotController viewer headless, for simulation and tests.

    with HeadlessViewer() as hv:          # server + headless Chrome on a spare port
        v = Viewer(hv.ws_url, hv.session)

To watch instead, run the viewer normally (python3 server.py in VRRobotController,
open http://localhost:8080) and connect Viewer() to it directly.
"""
import json
import os
import shutil
import signal
import socket
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

from .boards import ROOT

VIEWER_DIR = Path(os.environ.get("VRROBOT_DIR", ROOT.parent / "VRRobotController"))


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def chrome_binary():
    for name in ("google-chrome", "chromium", "chromium-browser"):
        p = shutil.which(name)
        if p:
            return p
    return None


def available():
    return (VIEWER_DIR / "server.py").exists() and chrome_binary() is not None


class HeadlessViewer:
    def __init__(self, port=None, window=(1280, 800), timeout=60):
        self.port = port or _free_port()
        self.window = window
        self.timeout = timeout
        self.procs = []
        self.session = None
        self._profile = None

    @property
    def http_url(self):
        return f"http://127.0.0.1:{self.port}"

    @property
    def ws_url(self):
        return f"ws://127.0.0.1:{self.port}/ws"

    def __enter__(self):
        if not available():
            raise RuntimeError(f"viewer not found at {VIEWER_DIR} or no Chrome installed")
        self._profile = tempfile.mkdtemp(prefix="legosort-chrome-")
        self.procs.append(subprocess.Popen(
            ["python3", "server.py", "--port", str(self.port), "--host", "127.0.0.1"],
            cwd=VIEWER_DIR, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True))
        self._wait(lambda: self._sessions() is not None, "server")
        w, h = self.window
        self.procs.append(subprocess.Popen(
            [chrome_binary(), "--headless=new", "--no-first-run", "--no-default-browser-check",
             f"--user-data-dir={self._profile}", "--use-angle=swiftshader",
             "--enable-unsafe-swiftshader", f"--window-size={w},{h}", self.http_url + "/"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        self._wait(lambda: bool(self._sessions()), "viewer page")
        self.session = self._sessions()[0]["id"]
        time.sleep(1.0)      # let the robot model finish loading
        return self

    def __exit__(self, *exc):
        for p in reversed(self.procs):
            try:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        if self._profile:
            shutil.rmtree(self._profile, ignore_errors=True)
        return False

    def _sessions(self):
        try:
            with urllib.request.urlopen(self.http_url + "/sessions", timeout=1) as r:
                return json.loads(r.read())
        except OSError:
            return None

    def _wait(self, cond, what):
        end = time.monotonic() + self.timeout
        while time.monotonic() < end:
            if cond():
                return
            time.sleep(0.25)
        self.__exit__()
        raise TimeoutError(f"{what} did not start within {self.timeout}s")
