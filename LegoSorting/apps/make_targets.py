"""Generate the printable targets.

    python3 apps/make_targets.py            # writes prints/*.pdf and config/base_mat.json if missing

prints/charuco_A4.pdf   intrinsics board (A4 landscape)
prints/base_mat_A3.pdf  base mat: ArUco markers around an A3 border

Print both at 100% / "actual size" (no fit-to-page), then measure the printed
sizes with a ruler or calipers. Each sheet carries a 100 mm check bar.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from legosort.boards import DEFAULT_MAT, ROOT, BaseMat, CharucoSpec, check_bar, default_mat_layout  # noqa: E402

DPI = 300
PX_PER_MM = DPI / 25.4
A4 = (297.0, 210.0)
A3 = (420.0, 297.0)


def page(size_mm):
    w, h = (int(round(v * PX_PER_MM)) for v in size_mm)
    return np.full((h, w), 255, np.uint8)


def paste_centred(canvas, img):
    H, W = canvas.shape
    h, w = img.shape
    v0, u0 = (H - h) // 2, (W - w) // 2
    canvas[v0:v0 + h, u0:u0 + w] = img
    return canvas


def save_pdf(img, path):
    Image.fromarray(img).save(path, "PDF", resolution=DPI)
    Image.fromarray(img).save(path.with_suffix(".png"), dpi=(DPI, DPI))


def ensure_mat_config(force=False):
    if DEFAULT_MAT.exists() and not force:
        return BaseMat.load(DEFAULT_MAT)
    w, h, dict_name, markers = default_mat_layout(*A3)
    d = {
        "_note": ("Board frame: origin at sheet centre, X right, Y towards the top edge "
                  "of the print, Z up. Lay the mat with its TOP edge towards the robot. "
                  "board_in_robot is a hand measurement of the sheet centre in the "
                  "Meca500 base frame (BRF, origin at the base bottom, X forward) - MEASURE IT; "
                  "rz_deg = 90 makes board +Y point at the robot (robot -X) and board +X along robot +Y. "
                  "Stage 2 replaces this with a fitted T_robot_board."),
        "width_mm": w, "height_mm": h, "dict_name": dict_name,
        "markers": [m.__dict__ for m in markers],
        "board_in_robot": {"xyz": [230.0, 0.0, 0.0], "rz_deg": 90.0},
    }
    DEFAULT_MAT.parent.mkdir(exist_ok=True)
    DEFAULT_MAT.write_text(json.dumps(d, indent=2))
    return BaseMat.load(DEFAULT_MAT)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--force-config", action="store_true", help="overwrite config/base_mat.json")
    args = ap.parse_args()
    out = ROOT / "prints"
    out.mkdir(exist_ok=True)

    spec = CharucoSpec()
    sheet = paste_centred(page(A4), spec.render(PX_PER_MM))
    check_bar(sheet, PX_PER_MM, 15, 205, f"100 mm   ChArUco {spec.squares_x}x{spec.squares_y}, "
                              f"square {spec.square_mm} mm, marker {spec.marker_mm} mm")
    save_pdf(sheet, out / "charuco_A4.pdf")

    mat = ensure_mat_config(args.force_config)
    img = mat.render(PX_PER_MM)            # markers + check bar + centre cross
    save_pdf(img, out / "base_mat_A3.pdf")
    print(f"wrote {out}/charuco_A4.pdf, {out}/base_mat_A3.pdf; mat config {mat.source}")


if __name__ == "__main__":
    main()
