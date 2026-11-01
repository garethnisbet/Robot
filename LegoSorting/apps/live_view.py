"""Live camera pose in the robot frame, from the base mat.

    python3 apps/live_view.py [--name c920]
    python3 apps/live_view.py --image frame.png --out annotated.png   # one still image

Overlays the robot base axes (X red, Y green, Z blue, 100 mm), the mat outline and
a 50 mm grid on the table, all drawn from robot-frame coordinates.

Click on the table to drop a pin: its robot-frame XY is printed and the pin is
redrawn from robot coordinates every frame. Move the camera by hand: if the pins and
grid stay on the same physical spots, stage 1 works.
    c   clear pins      s   save snapshot to data/snapshots/      q   quit
"""
import argparse
import collections
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from legosort import overlay  # noqa: E402
from legosort.boards import BaseMat  # noqa: E402
from legosort.camera import DATA, Camera, CameraSettings  # noqa: E402
from legosort.geometry import transform_points  # noqa: E402
from legosort.intrinsics import Intrinsics  # noqa: E402
from legosort.tracker import BaseTracker  # noqa: E402


def grid_lines(mat, step=50.0):
    """Robot-frame segments for a grid over the mat interior."""
    w, h = mat.width_mm / 2 - 52, mat.height_mm / 2 - 52
    segs = []
    for x in np.arange(-np.floor(w / step) * step, w + 1e-6, step):
        segs.append(([x, -h, 0], [x, h, 0]))
    for y in np.arange(-np.floor(h / step) * step, h + 1e-6, step):
        segs.append(([-w, y, 0], [w, y, 0]))
    T = mat.T_robot_board
    return [(transform_points(T, a)[0], transform_points(T, b)[0]) for a, b in segs]


def draw(img, tracker, pose, pins, history):
    mat = tracker.mat
    lines = []
    if pose is None:
        lines.append(("NO BASE MAT - need 2+ markers", overlay.RED))
    else:
        for c in pose.corners:
            overlay.polyline(img, c.reshape(4, 2), overlay.CYAN, thickness=1)
        r2p = lambda pts: tracker.robot_to_pixel(pose, pts)  # noqa: E731
        overlay.polyline(img, r2p(transform_points(mat.T_robot_board, mat.outline())), overlay.YELLOW)
        for a, b in grid_lines(mat):
            pa, pb = r2p([a, b])
            overlay.line(img, pa, pb, (200, 200, 200), 1)
        o, x, y, z = r2p([[0, 0, 0], [100, 0, 0], [0, 100, 0], [0, 0, 100]])
        overlay.axes(img, o, x, y, z, labels=("Xr", "Yr", "Zr"))
        for i, p in enumerate(pins):
            q = r2p([p])[0]
            if np.all(np.isfinite(q)):
                cv2.drawMarker(img, (int(q[0]), int(q[1])), overlay.RED, cv2.MARKER_TILTED_CROSS, 24, 3)
                cv2.putText(img, f"{i + 1}: {p[0]:.1f}, {p[1]:.1f}", (int(q[0]) + 14, int(q[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, overlay.RED, 2, cv2.LINE_AA)
        cp = pose.camera_position
        history.append(cp)
        jit = (f"{np.linalg.norm(np.std(np.array(history), axis=0)):.2f} mm"
               if len(history) > 5 else "-")
        lines += [
            f"markers {pose.n_markers}   reproj {pose.rms_px:.2f} px",
            f"camera in robot frame  X {cp[0]:7.1f}  Y {cp[1]:7.1f}  Z {cp[2]:7.1f} mm",
            f"camera jitter (sd, last {len(history)})  {jit}",
        ]
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="c920")
    ap.add_argument("--mat", help="mat config (default config/base_mat.json)")
    ap.add_argument("--image", help="process this still image instead of the camera")
    ap.add_argument("--out", help="with --image: save the annotated result here")
    ap.add_argument("--scale", type=float, default=0.6)
    args = ap.parse_args()

    try:
        intr = Intrinsics.load(args.name)
    except FileNotFoundError:
        sys.exit(f"no intrinsics for '{args.name}': run apps/calibrate_intrinsics.py first")
    mat = BaseMat.load(args.mat) if args.mat else BaseMat.load()
    tracker = BaseTracker(mat, intr.K, intr.dist)
    pins, history = [], collections.deque(maxlen=30)

    if args.image:
        img = cv2.imread(args.image)
        pose = tracker.estimate(img)
        lines = draw(img, tracker, pose, pins, history)
        overlay.text_panel(img, lines)
        print("\n".join(l if isinstance(l, str) else l[0] for l in lines))
        if args.out:
            cv2.imwrite(args.out, img)
        return

    settings = CameraSettings.load(args.name)
    for why in intr.mismatch(settings):
        print("WARNING intrinsics may not apply:", why)
    cam = Camera(settings)
    if cam.lock_errors:
        print("could not set:", cam.lock_errors)
    state = {"pose": None}

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and state["pose"] is not None:
            p = tracker.pixel_to_robot(state["pose"], [[x / args.scale, y / args.scale]])[0]
            if np.all(np.isfinite(p)):
                pins.append(p)
                print(f"pin {len(pins)}: robot X {p[0]:.2f}  Y {p[1]:.2f}  Z {p[2]:.2f} mm")

    cv2.namedWindow("live")
    cv2.setMouseCallback("live", on_mouse)
    last_check, warn, t_prev = 0.0, "", time.time()
    try:
        while True:
            img = cam.read()
            pose = tracker.estimate(img)
            state["pose"] = pose
            lines = draw(img, tracker, pose, pins, history)
            if time.time() - last_check > 5:
                drift = cam.check_locked()
                warn = ("camera controls drifted: " + ", ".join(drift)) if drift else ""
                last_check = time.time()
            if warn:
                lines.append((warn, overlay.RED))
            t = time.time()
            lines.append(f"{1 / max(t - t_prev, 1e-3):4.1f} fps   click=pin  c clear  s save  q quit")
            t_prev = t
            overlay.text_panel(img, lines)
            overlay.show("live", img, args.scale)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("c"):
                pins.clear()
            if k == ord("s"):
                d = DATA / "snapshots"
                d.mkdir(parents=True, exist_ok=True)
                p = d / time.strftime("%Y%m%d_%H%M%S.png")
                cv2.imwrite(str(p), img)
                print("saved", p)
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
