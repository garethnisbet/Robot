# Panorama Stitcher & Editor

A panoramic image stitcher with an interactive 3D spherical editor for DJI drone imagery. Supports automatic stitching using DJI gimbal metadata, people-aware seam detection, and manual seam and clone painting on a 3D sphere.

## Installation

```bash
pip install -r requirements.txt
```

Requirements: Python 3.8+, OpenCV 4.8+, NumPy.

Optional: OpenCV built with CUDA support enables GPU-accelerated rendering.

## Quick Start

```bash
# Open the viewer (import images from browser)
python main.py

# View an existing panorama
python main.py panorama.jpg

# Stitch a folder of DJI images and view the result
python main.py /path/to/dji/photos/

# Open the seam editor for manual control
python main.py /path/to/dji/photos/ -e

# Use a reference panorama as the editor starting composite
python main.py /path/to/dji/photos/ -e -r reference_pano.jpg
```

## Command Line Options

| Option | Description |
|--------|-------------|
| `images` | Panorama file to view, or folder/files to stitch |
| `-e, --edit` | Open the interactive seam editor |
| `-r, --reference` | Reference panorama for the editor's initial composite |
| `-o, --output` | Output panorama path (default: `output/panorama.jpg`) |
| `-s, --stitch` | Force stitching mode for multiple images |
| `-p, --port` | Server port (default: 8420) |

## Features

### Automatic Stitching

When given a folder of DJI drone images, the stitcher:

1. Reads gimbal metadata (yaw, pitch, roll) from EXIF data
2. Refines rotations using SIFT feature matching with RANSAC
3. Computes exposure gain compensation
4. Detects people/subjects and preserves them in seam placement
5. Generates a full equirectangular (360x180) panorama

### Panorama Viewer

The viewer opens in a browser with a 3D sphere displaying the panorama. Controls:
- Left-drag: orbit around the sphere
- Scroll: zoom in/out
- Palette button (or `C`): colour and contrast panel — brightness, contrast, saturation and warmth, with a Reset button. Adjustments apply in every projection mode and are baked into exported images.

### Seam Editor

The editor provides full control over how source images are composited. It runs at `http://127.0.0.1:8420/editor.html`.

## Editor Guide

### Modes

The editor has three modes, toggled via toolbar buttons:

#### Move Mode
- **Click**: Cycle through layers at that point (selects the next image underneath)
- **Drag**: Move the selected image (adjusts its yaw/pitch position)
- **Right-drag**: Orbit the camera

#### Paint Mode
- **Left-drag**: Paint the selected image onto the sphere (reveals it as foreground)
- **Shift+drag**: Unpaint (restores the original seam labels)
- **Alt+click**: Pick which image to paint with (cycles through layers at that point)
- **Right-drag**: Orbit the camera

A yellow circle cursor shows the brush size in paint mode, scaled to the projected size on the sphere.

#### Clone Mode

Paint mode can only choose between the source images that cover a point. Clone mode copies pixels from somewhere else on the sphere, for things no seam can fix — a person who moved between frames, a tripod leg, a gap no image covers.

- **Alt+click**: Set the clone source (a cyan crosshair marks it)
- **Left-drag**: Stamp pixels from the source into the destination, with a soft-edged brush
- **Shift+drag**: Erase clone paint, putting the original pixels back
- **Ctrl+Z**: Undo the last clone stroke
- **Right-drag**: Orbit the camera

A dashed cyan ring shows where the brush is reading from. With **Aligned** ticked (the default) the source keeps its offset from the destination, so a drag copies a moving region; unticked, every stroke starts again from the source anchor, stamping the same patch repeatedly.

Clone paint lives above the seam labels, so painting seams underneath it still works. It is stored as a list of strokes rather than a painted image: the full-res render replays them at its own resolution, copying full-res pixels instead of upscaling what you painted in the editor. "Clear Clone" removes all of it.

### Thumbnail Strip

The bottom strip shows all source images:

- **Click**: Select an image (highlights it, used for painting/moving)
- **Double-click**: Show full-size preview of the image (click or Escape to dismiss)
- **Checkbox**: Check images to mark them as foreground candidates
  - In **Move mode**: checking a box immediately brings that image to the foreground everywhere it has coverage
  - In **Paint mode**: painting reveals all checked images where they have coverage (instead of just the single selected image)

### Toolbar Buttons

| Button | Action |
|--------|--------|
| Move / Paint / Clone | Switch between modes |
| Brush Size | Slider to adjust paint and clone brush radius |
| Aligned | Clone source follows the brush (on) or restarts from the anchor each stroke (off) |
| Clear Clone | Discard all clone paint |
| Opacity | Drag preview opacity when moving images |
| Overlay | Toggle coloured seam label overlay |
| Reset | Reset all image positions to their original values |
| Apply Positions | Reproject layers after moving images (required before painting) |
| Save | Save current seam state and clone strokes (auto-loads on page refresh) |
| Save Project | Save entire state as a named project |
| Load Project | Load a previously saved project |
| Preview | Generate a preview composite from current seams |
| Render Full Res | Render final panorama at full resolution (8192x4096). Button changes to "Cancel Render" while in progress. |
| View | Open the result in the panorama viewer |

### Workflow

1. **Launch the editor** with your source images:
   ```bash
   python main.py /path/to/photos/ -e
   ```

2. **Inspect the automatic result** — the stitcher uses people detection and distance-based seam placement automatically.

3. **Adjust image positions** (Move mode):
   - Click an image thumbnail or click on the sphere to select it
   - Drag to reposition if alignment is off
   - Click "Apply Positions" to reproject (this recalculates layers and seams)

4. **Refine seams** (Paint mode):
   - Select the image you want to reveal (click thumbnail or Alt+click on sphere)
   - Paint over areas where you want that image to show
   - Use Shift+paint to undo painting (restore original seams)
   - Use checkboxes for batch operations (check multiple images, paint to reveal all)

5. **Retouch what seams cannot fix** (Clone mode):
   - Alt+click a clean patch of the sphere to set the source
   - Drag over the problem area to stamp those pixels in
   - Shift+drag to take clone paint back off, Ctrl+Z to undo a whole stroke

6. **Save your work**:
   - "Save" preserves seam state and clone strokes (auto-restores on reload)
   - "Save Project" creates a named snapshot you can return to later

7. **Render**:
   - "Preview" generates a quick composite at editor resolution (Gaussian-blended)
   - "Render Full Res" produces the final 8192x4096 panorama with hard seams (no ghosting)
   - Rendering can be cancelled mid-progress via the "Cancel Render" button

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| Ctrl+S | Save seams |
| Ctrl+Z | Undo the last clone stroke (Clone mode) |
| M / P / C | Move / Paint / Clone mode |
| O | Toggle the seam label overlay |
| Escape | Deselect current image |

### Projects

Projects save the complete editor state (layers, masks, seams, clone strokes, metadata, cache) under `output/projects/<name>/`. Use "Load Project" to switch between different editing sessions. Loading a project replaces the current editor state and reloads the page.

## Render Output

"Render Full Res" produces three outputs:

1. **`panorama.jpg`** — 8192x4096 equirectangular JPEG with hard seam compositing (no blending/ghosting)
2. **GPano XMP metadata** — embedded in the JPEG so Google Photos, Facebook, and other services automatically recognize it as a 360 panorama
3. **`panorama.html`** — standalone interactive 360 viewer that can be opened directly in Chrome (image embedded as base64, works offline via file://)

The standalone HTML viewer features:
- Drag to look around (smooth damped motion)
- Scroll to zoom (smooth FOV adjustment, 20°-120°)
- Works completely offline — no server needed
- Uses THREE.js from CDN for rendering

## Performance

The full-res render uses optimized projection:
- **Hard seam compositing**: each pixel comes from exactly one source image (no blending), eliminating ghosting
- **Column-clipped projection**: only processes the ~20% of columns where each image is visible
- **CUDA acceleration** (when available): source images uploaded to GPU, remap via `cv2.cuda.remap`
- **Multi-threaded CPU fallback**: ThreadPoolExecutor with up to 8 threads
- **Cancellable**: cancel button stops both client and server-side processing

Typical render time: ~17 seconds on CPU for 8192x4096 (26 source images).

## Output Files

```
output/
  editor/           # Current editor working data
    metadata.json   # Image positions and parameters
    cache.pkl       # Cached rotations and camera intrinsics
    seams.png       # Current seam label map
    clone_strokes.json  # Clone-stamp strokes, replayed at render time
    composite.jpg   # Preview composite
    layer_XX.png    # Reprojected image layers
    mask_XX.png     # Coverage masks per layer
    source_XX.jpg   # Source image thumbnails
  projects/         # Saved project snapshots
    <name>/         # Each project is a copy of editor/
  panorama.jpg      # Final rendered panorama (with GPano XMP)
  panorama.html     # Standalone 360 viewer (open in browser)
```

## How It Works

### Stitching Pipeline

1. **Metadata extraction**: Reads DJI EXIF (GimbalYawDegree, GimbalPitchDegree, GimbalRollDegree)
2. **Rotation refinement**: SIFT feature matching between overlapping image pairs, with RANSAC and iterative bundle adjustment (clamped ±2° per iteration, ±15° in total). Each pair is first vetted against the gimbal metadata — pairs implying an implausibly large correction, or whose matches disagree with each other, are discarded, since RANSAC alone will happily fit a homography to repetitive texture such as open water. The adjustment is regularized towards the metadata angles, which also keeps the horizon level.
3. **Gain compensation**: Normalizes exposure across images
4. **Equirectangular projection**: Projects each image onto a full 360x180 sphere using camera intrinsics
5. **Seam detection**: Voronoi-based assignment using distance transforms with horizontal wrapping at ±180°
6. **People preservation**: Haar cascade detection moves seams away from detected subjects
7. **Hard seam compositing**: Each pixel assigned to exactly one source image — no blending, no ghosting

### Editor Rendering

The editor uses THREE.js to display the panorama on a sphere. Source images are projected as equirectangular layers with per-pixel seam labels determining which image is visible at each point. Painting modifies the seam labels in real-time and updates the composite texture. Clone strokes are stamped into a separate RGBA layer drawn over that composite, and replayed onto the render at full resolution.

### CUDA Detection

At import time, `stitcher.py` checks `cv2.cuda.getCudaEnabledDeviceCount()`. If CUDA is available, the render uses GPU-accelerated remap. Check with:

```bash
python3 -c "from stitcher import USE_CUDA; print('CUDA:', USE_CUDA)"
```

The server terminal also prints `(CUDA)` or `(N threads)` when rendering starts.
