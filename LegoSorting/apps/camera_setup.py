"""Choose and lock the webcam's focus, exposure and white balance.

    python3 apps/camera_setup.py [--name c920] [--device /dev/video0]

Point the camera at the mat from where it will normally sit, then adjust:
    a       let auto exposure + white balance settle on this scene, then lock them
    f / F   focus  -/+ 2        (watch the sharpness number; higher is sharper)
    e / E   exposure -/+ 15%
    w / W   white balance -/+ 250 K
    s       save to data/camera_<name>.json
    q       quit (unsaved changes are lost)

Re-run the intrinsics calibration after changing focus or zoom.
"""
import argparse
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from legosort import overlay  # noqa: E402
from legosort.camera import Camera, CameraSettings, sharpness  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="c920")
    ap.add_argument("--device")
    ap.add_argument("--scale", type=float, default=0.6, help="display scale")
    ap.add_argument("--snapshot", help="grab one frame, save it annotated to this path, exit")
    args = ap.parse_args()

    s = CameraSettings.load(args.name)
    first_run = not s.path.exists()
    if args.device:
        s.device = args.device
    cam = Camera(s)
    if cam.lock_errors:
        print("could not set:", cam.lock_errors)
    if first_run:
        print("no saved settings: taking exposure and white balance from auto mode:",
              cam.lock_from_auto())
    c = s.controls
    steps = {
        ord("f"): ("focus_absolute", lambda v: max(0, v - 2)),
        ord("F"): ("focus_absolute", lambda v: min(250, v + 2)),
        ord("e"): ("exposure_time_absolute", lambda v: max(3, int(v / 1.15))),
        ord("E"): ("exposure_time_absolute", lambda v: min(2047, int(v * 1.15) + 1)),
        ord("w"): ("white_balance_temperature", lambda v: max(2800, v - 250)),
        ord("W"): ("white_balance_temperature", lambda v: min(7500, v + 250)),
    }
    if args.snapshot:
        for _ in range(20):          # let the new exposure settle
            cam.read()
    try:
        while True:
            img = cam.read()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            roi = gray[h // 3: 2 * h // 3, w // 3: 2 * w // 3]
            sat = float((gray >= 250).mean() * 100)
            lines = [
                f"{args.name}  {w}x{h}",
                f"sharpness (centre) {sharpness(roi):8.1f}",
                f"focus {c['focus_absolute']}   exposure {c['exposure_time_absolute']}   "
                f"WB {c['white_balance_temperature']} K",
                (f"saturated pixels {sat:.2f}%", overlay.RED if sat > 1 else overlay.WHITE),
                "a auto-lock  f/F focus  e/E exposure  w/W WB  s save  q quit",
            ]
            cv2.rectangle(img, (w // 3, h // 3), (2 * w // 3, 2 * h // 3), overlay.YELLOW, 1)
            overlay.text_panel(img, lines)
            if args.snapshot:
                cv2.imwrite(args.snapshot, img)
                print("\n".join(l if isinstance(l, str) else l[0] for l in lines))
                break
            overlay.show("camera setup", img, args.scale)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("s"):
                print("saved", s.save())
            if k == ord("a"):
                print("locked from auto:", cam.lock_from_auto())
            if k in steps:
                name, fn = steps[k]
                err = cam.set(name, fn(c[name]))
                if err:
                    print(err)
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
