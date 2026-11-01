"""Calibrate a webcam's intrinsics with the printed ChArUco board.

    python3 apps/calibrate_intrinsics.py --square-mm 30.0 [--name c920]
    python3 apps/calibrate_intrinsics.py --square-mm 30.0 --from-images data/calib_c920

Measure the printed square size first (across all 9 squares, divide by 9) and
pass it as --square-mm: printers often scale by a percent or two.

Hold the board still at many positions and angles: near and far, tilted up to about
45 deg, and especially in the image corners. The grid shows which parts of the image
have been covered.
    SPACE   capture this view            a   toggle auto-capture
    u       undo last capture            c   calibrate and save
    q       quit
Captured frames are kept in data/calib_<name>/ so you can recalibrate offline.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from legosort import overlay  # noqa: E402
from legosort.boards import CharucoSpec  # noqa: E402
from legosort.camera import DATA, Camera, CameraSettings  # noqa: E402
from legosort.intrinsics import calibrate, detect_charuco, view_diversity  # noqa: E402

MIN_CORNERS = 12


def spec_for(square_mm):
    base = CharucoSpec()
    return CharucoSpec(base.squares_x, base.squares_y, square_mm,
                       square_mm * base.marker_mm / base.square_mm, base.dict_name)


def run_calibration(views, size, spec, settings, name):
    if len(views) < 10:
        print(f"need at least 10 views, have {len(views)}")
        return None
    intr, kept = calibrate(views, size)
    intr.camera = {"device": settings.device, "controls": dict(settings.controls)} if settings else {}
    intr.board = spec.__dict__
    path = intr.save(name)
    K = intr.K
    print(f"\nviews used {len(kept)}/{len(views)}   RMS reprojection {intr.rms:.3f} px")
    print(f"fx {K[0,0]:.1f}  fy {K[1,1]:.1f}  cx {K[0,2]:.1f}  cy {K[1,2]:.1f}")
    print("dist", np.round(intr.dist, 5))
    print(f"coverage {view_diversity(size, [views[i] for i in kept]) * 100:.0f}% of image grid")
    if intr.rms > 0.8:
        print("RMS is high: check for motion blur, a non-flat board, or the wrong --square-mm")
    print("saved", path)
    return intr


def from_images(folder, spec, name):
    files = sorted(Path(folder).glob("*.png"))
    views, size = [], None
    for f in files:
        gray = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        size = (gray.shape[1], gray.shape[0])
        v = detect_charuco(gray, spec)
        if v is not None and len(v[0]) >= MIN_CORNERS:
            views.append(v)
    print(f"{len(views)} usable views from {len(files)} images")
    settings = CameraSettings.load(name)
    run_calibration(views, size, spec, settings, name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="c920")
    ap.add_argument("--square-mm", type=float, required=True, help="measured printed square size")
    ap.add_argument("--from-images", help="calibrate from saved frames instead of the camera")
    ap.add_argument("--scale", type=float, default=0.6)
    args = ap.parse_args()
    spec = spec_for(args.square_mm)
    if args.from_images:
        return from_images(args.from_images, spec, args.name)

    settings = CameraSettings.load(args.name)
    if not settings.path.exists():
        print(f"note: {settings.path} not found; using defaults. Run apps/camera_setup.py first "
              "to choose focus and exposure.")
    cam = Camera(settings)
    if cam.lock_errors:
        print("could not set:", cam.lock_errors)
    detector = cv2.aruco.CharucoDetector(spec.board())
    folder = DATA / f"calib_{args.name}"
    folder.mkdir(parents=True, exist_ok=True)
    for f in folder.glob("*.png"):
        f.unlink()
    views, frames = [], []
    auto, last_auto, prev_img_pts = False, 0.0, None
    size = (settings.width, settings.height)
    try:
        while True:
            img = cam.read()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            v = detect_charuco(gray, spec, detector)
            disp = img.copy()
            cells = np.zeros((4, 6), bool)
            for _, ip in views:
                gx = np.clip((ip[:, 0] / size[0] * 6).astype(int), 0, 5)
                gy = np.clip((ip[:, 1] / size[1] * 4).astype(int), 0, 3)
                cells[gy, gx] = True
            overlay.coverage(disp, cells, alpha=0.15)
            n = 0 if v is None else len(v[0])
            still = False
            if v is not None:
                for p in v[1]:
                    cv2.circle(disp, (int(p[0]), int(p[1])), 5, overlay.YELLOW, 2)
                # "still" = same corners moved < 1 px since the previous frame
                if prev_img_pts is not None and len(prev_img_pts) == n:
                    still = float(np.max(np.abs(prev_img_pts - v[1]))) < 1.0
                prev_img_pts = v[1]
            else:
                prev_img_pts = None
            good = n >= MIN_CORNERS
            capture = False
            k = cv2.waitKey(1) & 0xFF
            if k == ord(" ") and good:
                capture = True
            if auto and good and still and time.time() - last_auto > 1.0:
                # only if the board centre moved since the last capture
                c = v[1].mean(axis=0)
                if not views or np.linalg.norm(c - views[-1][1].mean(axis=0)) > 0.08 * size[0]:
                    capture = True
            if capture:
                views.append(v)
                frames.append(img)
                cv2.imwrite(str(folder / f"{len(frames):03d}.png"), img)
                last_auto = time.time()
                disp[:] = 255
            if k == ord("a"):
                auto = not auto
            if k == ord("u") and views:
                views.pop()
                frames.pop()
                (folder / f"{len(frames) + 1:03d}.png").unlink(missing_ok=True)
            if k == ord("c"):
                run_calibration(views, size, spec, settings, args.name)
            if k == ord("q"):
                break
            overlay.text_panel(disp, [
                f"views {len(views)}   coverage {cells.mean() * 100:.0f}%",
                (f"corners {n}" + ("  still" if still else ""), overlay.GREEN if good else overlay.RED),
                (f"auto-capture {'ON' if auto else 'off'}", overlay.YELLOW if auto else overlay.WHITE),
                "SPACE capture  a auto  u undo  c calibrate  q quit",
            ])
            overlay.show("intrinsics", disp, args.scale)
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
