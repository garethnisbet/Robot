"""Stage 1 in simulation: the live view, fed by a virtual camera in the VRRobotController twin.

    python3 apps/sim_view.py                       # starts its own headless viewer
    python3 apps/sim_view.py --url ws://127.0.0.1:8080/ws [--session ID]
                                                   # use a viewer you already have open
                                                   # (start it with python3 start.py --sim)

The table and base mat are added to the viewer as objects named ls_*. The virtual
camera orbits the mat; the overlay is drawn from the tracked pose only.
    a / d   orbit left / right       w / s   higher / lower
    + / -   closer / further         j       random joint pose for the arm
    click   drop a pin (robot frame) c       clear pins       q   quit
"""
import argparse
import collections
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from legosort import overlay  # noqa: E402
from legosort.boards import BaseMat  # noqa: E402
from legosort.sim import SimCamera, Viewer, build_scene  # noqa: E402
from legosort.simserver import HeadlessViewer  # noqa: E402
from legosort.tracker import BaseTracker  # noqa: E402
from live_view import draw  # noqa: E402

CENTRE = np.array([230.0, 0.0, 0.0])
SAFE_POSE = [0, -20, 20, 0, 60, 0]


def camera_at(az, el, dist, fov=50.0):
    a, e = np.radians(az), np.radians(el)
    eye = CENTRE + dist * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    # image "up" points away from the camera along the table, so the mat stays upright-ish
    up = CENTRE - eye
    up[2] = 0
    return SimCamera(eye=tuple(eye), target=tuple(CENTRE), up=tuple(up / np.linalg.norm(up)), fov_deg=fov)


def run(viewer, args):
    mat = BaseMat.load()
    viewer.set_joints(SAFE_POSE)
    build_scene(viewer, mat)
    az, el, dist = -90.0, 50.0, 620.0
    pins, history = [], collections.deque(maxlen=30)
    state = {"pose": None, "tracker": None}
    rng = np.random.default_rng()

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and state["pose"] is not None:
            p = state["tracker"].pixel_to_robot(state["pose"], [[x / args.scale, y / args.scale]])[0]
            if np.all(np.isfinite(p)):
                pins.append(p)
                print(f"pin {len(pins)}: robot X {p[0]:.2f}  Y {p[1]:.2f}")

    cv2.namedWindow("sim")
    cv2.setMouseCallback("sim", on_mouse)
    while True:
        cam = camera_at(az, el, dist)
        img, K = viewer.capture(cam, exposure=args.exposure)
        tracker = BaseTracker(mat, K, np.zeros(5))
        pose = tracker.estimate(img)
        state.update(pose=pose, tracker=tracker)
        lines = draw(img, tracker, pose, pins, history)
        if pose is not None:
            err = np.linalg.norm(pose.camera_position - np.asarray(cam.eye))
            lines.append(f"true camera error {err:.2f} mm   (az {az:.0f}  el {el:.0f}  d {dist:.0f})")
        lines.append("a/d orbit  w/s height  +/- dist  j joints  c clear  q quit")
        overlay.text_panel(img, lines)
        overlay.show("sim", img, args.scale)
        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        az += {ord("a"): -10, ord("d"): 10}.get(k, 0)
        el = float(np.clip(el + {ord("w"): 5, ord("s"): -5}.get(k, 0), 15, 89))
        dist = float(np.clip(dist + {ord("+"): -40, ord("="): -40, ord("-"): 40}.get(k, 0), 300, 1500))
        if k == ord("c"):
            pins.clear()
        if k == ord("j"):
            viewer.set_joints(rng.uniform([-60, -30, -40, -90, 20, -90], [60, 30, 30, 90, 90, 90]))
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", help="WebSocket URL of a running viewer (default: start a headless one)")
    ap.add_argument("--session", help="viewer session id (default: broadcast)")
    ap.add_argument("--exposure", type=float, default=0.15)
    ap.add_argument("--scale", type=float, default=1.0)
    args = ap.parse_args()
    if args.url:
        v = Viewer(args.url, args.session)
        try:
            run(v, args)
        finally:
            v.close()
    else:
        with HeadlessViewer() as hv:
            v = Viewer(hv.ws_url, hv.session)
            try:
                run(v, args)
            finally:
                v.close()


if __name__ == "__main__":
    main()
