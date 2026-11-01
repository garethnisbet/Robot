"""Webcam capture with locked focus, exposure and white balance.

The intrinsics are only valid for the focus/zoom they were calibrated at, so the
settings are stored next to them and re-applied every time the camera is opened.
Controls are set with ``v4l2-ctl`` (Linux, UVC cameras such as the Logitech C920).
"""
import json
import shutil
import subprocess
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .boards import ROOT

DATA = ROOT / "data"

# Order matters: the automatic modes must be switched off before the absolute values stick.
DEFAULT_CONTROLS = {
    "focus_automatic_continuous": 0,
    "focus_absolute": 30,
    "zoom_absolute": 100,
    "auto_exposure": 1,                 # 1 = manual on UVC cameras
    "exposure_dynamic_framerate": 0,
    "exposure_time_absolute": 40,
    "gain": 0,
    "white_balance_automatic": 0,
    "white_balance_temperature": 4500,
}


@dataclass
class CameraSettings:
    name: str = "c920"
    device: str = "/dev/video0"
    width: int = 1920
    height: int = 1080
    fps: int = 30
    fourcc: str = "MJPG"
    controls: dict = field(default_factory=lambda: dict(DEFAULT_CONTROLS))

    @property
    def path(self):
        return DATA / f"camera_{self.name}.json"

    def save(self):
        DATA.mkdir(exist_ok=True)
        self.path.write_text(json.dumps(asdict(self), indent=2))
        return self.path

    @classmethod
    def load(cls, name):
        p = DATA / f"camera_{name}.json"
        if not p.exists():
            return cls(name=name)
        return cls(**json.loads(p.read_text()))


def _v4l2(device, *args):
    return subprocess.run(["v4l2-ctl", "-d", device, *args],
                          capture_output=True, text=True)


def set_controls(device, controls):
    """Apply v4l2 controls one at a time. Returns {name: error} for any that failed."""
    if not shutil.which("v4l2-ctl"):
        return {"*": "v4l2-ctl not installed; camera controls not locked"}
    errors = {}
    for k, v in controls.items():
        r = _v4l2(device, "-c", f"{k}={v}")
        if r.returncode != 0:
            errors[k] = (r.stderr or r.stdout).strip()
    return errors


def get_controls(device, names):
    if not shutil.which("v4l2-ctl"):
        return {}
    out = {}
    for n in names:
        r = _v4l2(device, "-C", n)
        if r.returncode == 0 and ":" in r.stdout:
            try:
                out[n] = int(r.stdout.split(":")[1])
            except ValueError:
                pass
    return out


class Camera:
    def __init__(self, settings: CameraSettings, lock=True):
        self.s = settings
        dev = settings.device
        index = int(dev[len("/dev/video"):]) if dev.startswith("/dev/video") else dev
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera {dev}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*settings.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.height)
        self.cap.set(cv2.CAP_PROP_FPS, settings.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.lock_errors = {}
        # Opening the device can reset controls, so lock after a first frame arrives.
        self.cap.read()
        if lock:
            self.relock()
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (w, h) != (settings.width, settings.height):
            raise RuntimeError(f"camera gave {w}x{h}, expected {settings.width}x{settings.height}")

    def relock(self):
        self.lock_errors = set_controls(self.s.device, self.s.controls)
        time.sleep(0.2)
        return self.lock_errors

    def set(self, name, value):
        self.s.controls[name] = value
        return set_controls(self.s.device, {name: value})

    def _mean_level(self, n=5):
        vals = []
        for _ in range(n):
            ok, f = self.cap.read()
            if ok:
                vals.append(float(np.median(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))))
        return float(np.mean(vals)) if vals else float("nan")

    def lock_from_auto(self, seconds=3.0, iterations=4):
        """Let auto exposure and white balance settle on this scene, then lock them.

        The C920 does not reliably report the gain/exposure its auto mode is using, so
        instead the median image brightness under auto mode is recorded, gain is locked
        at 0, and the manual exposure is adjusted until the brightness matches.
        Updates ``self.s.controls``. Focus is untouched.
        """
        set_controls(self.s.device, {"auto_exposure": 3, "white_balance_automatic": 1})
        t = time.time()
        while time.time() - t < seconds:
            self.cap.read()
        target = self._mean_level(10)
        wb = get_controls(self.s.device, ["white_balance_temperature"])
        exp = get_controls(self.s.device, ["exposure_time_absolute"]).get(
            "exposure_time_absolute", self.s.controls["exposure_time_absolute"])
        self.s.controls.update(wb)
        self.s.controls["gain"] = 0
        level = float("nan")
        for _ in range(iterations):
            self.s.controls["exposure_time_absolute"] = int(np.clip(exp, 3, 2047))
            self.relock()
            for _ in range(8):       # flush frames taken before the change
                self.cap.read()
            level = self._mean_level()
            if not level or abs(level - target) < 0.05 * target:
                break
            exp = exp * target / max(level, 1.0)
        return {"target_level": round(target, 1), "level": round(level, 1),
                **{k: self.s.controls[k] for k in
                   ("exposure_time_absolute", "gain", "white_balance_temperature")}}

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        return frame

    def check_locked(self):
        """Controls whose live value differs from the stored settings."""
        live = get_controls(self.s.device, self.s.controls.keys())
        return {k: (v, live[k]) for k, v in self.s.controls.items()
                if k in live and live[k] != v}

    def close(self):
        self.cap.release()


def sharpness(gray):
    """Variance of the Laplacian: higher is sharper. Used to choose a focus value."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())
