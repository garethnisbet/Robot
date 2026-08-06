#!/usr/bin/env python3
"""Panoramic image stitcher with interactive spherical viewer."""

import argparse
import sys
from pathlib import Path

from stitcher import stitch_images, prepare_editor_data
from server import serve

SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def collect_images(source):
    source = Path(source)
    if source.is_file():
        return [source]
    if source.is_dir():
        images = sorted(
            p for p in source.iterdir()
            if p.suffix.lower() in SUPPORTED_EXTENSIONS
        )
        if not images:
            print(f"No images found in {source}")
            sys.exit(1)
        return images
    print(f"Path not found: {source}")
    sys.exit(1)


def is_single_image(path):
    return Path(path).is_file() and Path(path).suffix.lower() in SUPPORTED_EXTENSIONS


def main():
    parser = argparse.ArgumentParser(description="Panoramic image stitcher and viewer")
    parser.add_argument("images", nargs="*", help="Panorama file to view, or multiple images to stitch")
    parser.add_argument("-o", "--output", default="output/panorama.jpg", help="Output panorama path")
    parser.add_argument("-s", "--stitch", action="store_true", help="Force stitching mode (multiple images)")
    parser.add_argument("-e", "--edit", action="store_true", help="Open interactive seam editor")
    parser.add_argument("-r", "--reference", help="Reference panorama to use as editor starting image")
    parser.add_argument("-p", "--port", type=int, default=8420, help="Viewer server port")
    args = parser.parse_args()

    if not args.images:
        print("Panorama Viewer")
        print("Opening viewer — import a panorama from the browser.\n")
        serve(None, port=args.port)
        return

    if len(args.images) == 1 and is_single_image(args.images[0]) and not args.stitch:
        pano_path = Path(args.images[0]).resolve()
        print(f"Viewing: {pano_path}")
        serve(str(pano_path), port=args.port)
        return

    all_images = []
    for src in args.images:
        all_images.extend(collect_images(src))

    print(f"Found {len(all_images)} images:")
    for img in all_images:
        print(f"  {img}")

    if args.edit:
        editor_dir = Path("output/editor")
        print(f"\nPreparing editor data...")
        meta = prepare_editor_data(all_images, str(editor_dir),
                                   reference_path=args.reference)
        if meta is None:
            sys.exit(1)
        print()
        serve(None, port=args.port, editor_dir=editor_dir)
        return

    output = stitch_images(all_images, args.output)
    if output is None:
        sys.exit(1)

    print()
    # Pass the sources on so the viewer's editor button can prepare editor data
    # on demand rather than dead-ending.
    serve(str(Path(output).resolve()), port=args.port, image_paths=all_images)


if __name__ == "__main__":
    main()
