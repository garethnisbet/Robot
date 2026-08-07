# Panorama Stitcher & Editor

A panoramic image stitcher with an interactive 3D spherical editor for DJI drone imagery. Supports automatic stitching using DJI gimbal metadata, people-aware seam detection, and manual seam and clone painting on a 3D sphere.

Everything runs locally: a small Python HTTP server does the stitching and rendering, and the browser provides the viewer and editor.

---

## Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Command Line Options](#command-line-options)
- [Preparing Source Images](#preparing-source-images)
- [Full Walkthrough](#full-walkthrough)
- [Viewer Guide](#viewer-guide)
- [Editor Guide](#editor-guide)
- [Render Output](#render-output)
- [Output Files](#output-files)
- [HTTP Endpoints](#http-endpoints)
- [How It Works](#how-it-works)
- [Performance](#performance)
- [Troubleshooting](#troubleshooting)
- [Tuning Constants](#tuning-constants)

---

## Installation

```bash
pip install -r requirements.txt
```

Requirements: Python 3.8+, OpenCV 4.8+, NumPy, SciPy.

A browser with WebGL is needed for the viewer and editor (Chrome is what this has been developed against).

Optional: OpenCV built with CUDA support enables GPU-accelerated rendering. Check whether it was detected:

```bash
python3 -c "from stitcher import USE_CUDA; print('CUDA:', USE_CUDA)"
```

The server terminal also prints `(CUDA)` or the band count when a render starts.

---

## Quick Start

| What you want | Command |
|---------------|---------|
| Open an empty viewer and import from the browser | `python main.py` |
| View an existing panorama | `python main.py panorama.jpg` |
| Stitch a folder of DJI photos and view the result | `python main.py /path/to/dji/photos/` |
| Stitch and go straight into the editor | `python main.py /path/to/dji/photos/ -e` |
| Edit using an existing panorama as the starting composite | `python main.py /path/to/dji/photos/ -e -r reference_pano.jpg` |
| Run on a different port | `python main.py /path/to/photos/ -p 8500` |

The server prints a URL when it starts — by default the viewer is at `http://127.0.0.1:8420/` and the editor at `http://127.0.0.1:8420/editor.html`. Press `Ctrl+C` in the terminal to stop it.

---

## Command Line Options

| Option | Description |
|--------|-------------|
| `images` | Panorama file to view, or folder/files to stitch |
| `-e, --edit` | Prepare editor data and open the interactive seam editor |
| `-r, --reference` | Reference panorama used to seed the editor's initial seams and composite |
| `-o, --output` | Output panorama path (default: `output/panorama.jpg`) |
| `-s, --stitch` | Force stitching instead of viewing when a single path is given (stitching still needs at least two images) |
| `-p, --port` | Server port (default: 8420) |

A single image argument is *viewed*; two or more (or a folder) are *stitched*.

---

## Preparing Source Images

**DJI drone panoramas (the main path).** The stitcher reads gimbal angles from each photo's XMP block — `GimbalYawDegree`, `GimbalPitchDegree` and `GimbalRollDegree`. Shoot with the drone's built-in sphere/panorama mode and keep the original JPEGs: re-encoding through an editor usually strips the XMP and the metadata path stops working.

**Other images.** If no DJI metadata is found, stitching falls back to OpenCV's generic `Stitcher`. That still produces a panorama, but the editor is not available for those images — it needs the per-image gimbal orientation to project layers onto the sphere.

**Supported extensions:** `.jpg`, `.jpeg`, `.png`, `.bmp`, `.tif`, `.tiff`.

Images without readable metadata are skipped with a warning rather than aborting the run.

---

## Full Walkthrough

This is the end-to-end path from a folder of photos to a finished 360 panorama.

### 1. Launch

```bash
python main.py /path/to/photos/ -e
```

The terminal prints each image it found, then prepares editor data: it projects every source image onto the sphere, builds coverage masks, and computes an initial seam assignment. **This takes a few minutes** for a typical 26-image sphere and only happens once per image set — afterwards the data is reused from `output/editor/`.

Without `-e` the run stitches straight to `output/panorama.jpg` and opens the viewer; you can still open the editor later from the viewer's pencil button, which builds the same data on demand.

### 2. Inspect the automatic result

The editor opens showing the composite on a sphere. Right-drag to orbit, scroll to zoom. The automatic seams already avoid cutting through detected people and follow the Voronoi boundaries between image centres, so much of the sphere usually needs no work.

Press `O` (or click **Overlay**) to tint each region by which source image owns it — the fastest way to see where the seams actually run.

### 3. Fix alignment (Move mode)

If a frame is visibly misaligned:

1. Click it on the sphere or in the bottom thumbnail strip to select it.
2. Drag it into place. While dragging, the layer is drawn semi-transparent over the rest — use the **Opacity** slider to judge the fit.
3. Click **Apply Positions**. This re-projects the layers at the new angles and recomputes seams.

Applying is required before painting: paint and clone work in the equirect pixel grid, which moves when a layer moves. The editor blocks the switch and says so if there are unapplied changes. **Reset** discards pending moves.

### 4. Refine seams (Paint mode)

Press `P`. Painting changes *which source image owns each pixel* — it cannot invent detail, only choose between the frames that cover that point.

1. Choose the image to reveal: click its thumbnail, press its number key, or `Alt+click` on the sphere to cycle through the layers stacked at that point.
2. Left-drag to paint it forward.
3. `Shift+drag` restores the original automatic seam labels.

To work with several images at once, tick their checkboxes in the strip: painting then reveals whichever checked image covers each pixel.

### 5. Retouch what seams cannot fix (Clone mode)

Press `C`. Clone mode copies pixels from elsewhere on the sphere — for a person who moved between frames, a tripod leg, or a gap no image covers.

1. `Alt+click` a clean patch to set the source. A cyan crosshair marks it.
2. Left-drag over the problem area. A dashed cyan ring shows where the brush is reading from.
3. `Shift+drag` erases clone paint; `Ctrl+Z` undoes a whole stroke.

### 6. Save

- **Save** (or `Ctrl+S`) writes the seam map and clone strokes into `output/editor/`. They reload automatically next time the page opens.
- **Save Project** copies the whole editor state to `output/projects/<name>/` as a named snapshot you can return to.

### 7. Preview and render

- **Preview** asks the server for a Gaussian-blended composite at editor resolution — a quick check of how the seams will read, several seconds.
- **Render Full Res** produces the final 8192x4096 panorama with multi-band blending, zenith fill and tone mapping. The button becomes **Cancel Render** while it runs.
- **View** opens the result in the panorama viewer.

---

## Viewer Guide

The viewer displays the panorama on a sphere with damped, momentum-based motion.

### Mouse

| Action | Result |
|--------|--------|
| Left-drag | Look around (releases into momentum) |
| Scroll | Zoom (FOV 20°–110°, widening to 10°–160° in the projection modes) |
| Drag files onto the window | Import — one image is viewed, several are stitched |

### Keyboard

| Key | Action |
|-----|--------|
| Arrow keys | Pan the view |
| `+` / `-` | Zoom in / out |
| `R` | Reset view (also resets stretch and pole fill) |
| `F` | Fullscreen |
| `T` | Tiny planet mode |
| `Y` | Tunnel mode |
| `[` / `]` | Stretch down / up (tiny planet and tunnel only) |
| `C` | Colour & contrast panel |
| `E` | Export the current view as a JPEG |
| `D` | Download the full stitched panorama |
| `Space` | Toggle auto-rotate |

Arrow keys are ignored while a slider has focus, so a focused slider can still be nudged.

### Buttons

| Button | Action |
|--------|--------|
| 📂 | Import images (file picker) |
| `+` / `−` | Zoom |
| ↺ | Reset view |
| ◎ | Tiny planet |
| ◉ | Tunnel |
| 🎨 | Colour & contrast panel |
| ⟳ | Auto-rotate |
| ⬇ | Export current view |
| 💾 | Download full panorama |
| ⛶ | Fullscreen |
| ✎ | Open the editor |

The pencil button is greyed out when this session has no source photos to edit — for example after viewing a single panorama file. Importing a folder of photos re-enables it.

### Projection modes

**Tiny planet** (`T`) and **tunnel** (`Y`) are the same stereographic projection with the vertical axis inverted. In either mode a panel appears with:

- **Stretch** (0.30–3.00): how far the projection pulls the horizon out.
- **Pole fill** (0–25%): grows the pole outward to cover the hole left when the original panorama has nothing directly below.

### Colour & contrast

The 🎨 panel adjusts **brightness** and **contrast** (0.2–2.0), **saturation** (0–2.0) and **warmth** (−1 to +1), with a **Reset** button. Adjustments apply in every projection mode and are baked into exported images.

### Export vs download

- **Export** (`E`) renders what is on screen — current mode, framing and colour — at the viewport's aspect ratio and the largest size the GPU allows, saved as `panorama_<mode>_<width>x<height>.jpg`.
- **Download** (`D`) saves the full equirectangular panorama file the server is serving, untouched.

---

## Editor Guide

The editor runs at `http://127.0.0.1:8420/editor.html`. It works at half the render resolution (4096x2048) for responsiveness; the final render is 8192x4096.

### Modes

The editor has three modes, toggled via toolbar buttons or the `M`, `P`, `C` keys.

#### Move Mode

- **Click**: Cycle through layers at that point (selects the next image underneath)
- **Drag**: Move the selected image (adjusts its yaw/pitch position)
- **Right-drag**: Orbit the camera

Moving marks the state as pending; **Apply Positions** re-projects and **Reset** discards.

#### Paint Mode

- **Left-drag**: Paint the selected image onto the sphere (reveals it as foreground)
- **Shift+drag**: Unpaint (restores the original seam labels)
- **Alt+click**: Pick which image to paint with (cycles through layers at that point)
- **Right-drag**: Orbit the camera

A yellow circle cursor shows the brush size, scaled to its projected size on the sphere.

#### Clone Mode

Paint mode can only choose between the source images that cover a point. Clone mode copies pixels from somewhere else on the sphere, for things no seam can fix — a person who moved between frames, a tripod leg, a gap no image covers.

- **Alt+click**: Set the clone source (a cyan crosshair marks it)
- **Left-drag**: Stamp pixels from the source into the destination, with a soft-edged brush
- **Shift+drag**: Erase clone paint, putting the original pixels back
- **Ctrl+Z**: Undo the last clone stroke
- **Right-drag**: Orbit the camera

A dashed cyan ring shows where the brush is reading from. With **Aligned** ticked (the default) the source keeps its offset from the destination, so a drag copies a moving region; unticked, every stroke starts again from the source anchor, stamping the same patch repeatedly.

Clone paint lives above the seam labels, so painting seams underneath it still works. It is stored as a list of strokes rather than a painted image: the full-res render replays them at its own resolution, copying full-res pixels instead of upscaling what you painted in the editor. "Clear Clone" removes all of it.

Because the editor composites hard seams while the render blends and tone maps, a cloned patch can look slightly different in the final render than in the editor — in the same way the rest of the image does.

### Thumbnail Strip

The bottom strip shows all source images:

- **Click**: Select an image (highlights it, used for painting/moving)
- **Double-click**: Show full-size preview of the image (click or Escape to dismiss)
- **Checkbox**: Check images to mark them as foreground candidates
  - In **Move mode**: checking a box immediately brings that image to the foreground everywhere it has coverage
  - In **Paint mode**: painting reveals all checked images where they have coverage (instead of just the single selected image)

Each thumbnail carries a coloured dot matching the tint that image gets in the seam overlay.

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

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `M` / `P` / `C` | Move / Paint / Clone mode |
| `O` | Toggle the seam label overlay |
| `0`–`9` | Select that source image |
| `Ctrl+S` | Save seams and clone strokes |
| `Ctrl+Z` | Undo the last clone stroke (Clone mode) |
| `Escape` | Deselect current image |

### Projects

Projects save the complete editor state (layers, masks, seams, clone strokes, metadata, cache) under `output/projects/<name>/`. Use "Load Project" to switch between different editing sessions. Loading a project replaces the current editor state and reloads the page.

### Tips

- Work coarse to fine: fix positions first, then seams, then clone. Each step invalidates less of the last.
- Turn the overlay on while painting; it is much easier to see a seam you are chasing than to infer it from the photo.
- A large brush blends better than a small one when cloning texture like foliage or gravel; a small brush is for edges.
- The zenith is synthesised, not photographed, when the source images do not reach straight up. Cloning near the top of the sphere fights that fill — retouch below it where possible.

---

## Render Output

"Render Full Res" produces three outputs:

1. **`panorama.jpg`** — 8192x4096 equirectangular JPEG, multi-band blended in linear light at each frame's own exposure and tone mapped once at the end
2. **GPano XMP metadata** — embedded in the JPEG so Google Photos, Facebook, and other services automatically recognize it as a 360 panorama
3. **`panorama.html`** — standalone interactive 360 viewer that can be opened directly in Chrome (image embedded as base64, works offline via file://)

The standalone HTML viewer features:
- Drag to look around (smooth damped motion)
- Scroll to zoom (smooth FOV adjustment, 20°-120°)
- Works completely offline — no server needed
- Uses THREE.js from CDN for rendering

---

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
  uploads/          # Photos imported through the browser
  panorama.jpg      # Final rendered panorama (with GPano XMP)
  panorama.html     # Standalone 360 viewer (open in browser)
```

Browser-imported photos are kept in `uploads/` rather than discarded, so a session started from the browser can still open the editor — which needs the original photos.

---

## HTTP Endpoints

Useful when scripting against a running server.

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/panorama-info` | Path of the panorama currently loaded |
| GET | `/panorama-image` | The panorama file itself |
| GET | `/editor/status` | Whether the editor is ready, preparable, or unavailable, and why |
| GET | `/editor/data` | Editor metadata (image angles, sizes, FOV) |
| GET | `/editor/seams` | Current seam label map (PNG, one label per pixel) |
| GET | `/editor/clone-strokes` | Saved clone strokes (JSON) |
| GET | `/editor/composite` | Current composite preview |
| GET | `/editor/{layer,mask,source}/{i}` | Per-image layer, coverage mask, or source thumbnail |
| GET | `/projects/list` | Names of saved projects |
| POST | `/upload` | Import images (multipart) — one is viewed, several are stitched |
| POST | `/editor/prepare` | Build editor data for the loaded photos |
| POST | `/editor/preview` | Blended preview from a posted seam map |
| POST | `/editor/render` | Full-res render from a posted seam map |
| POST | `/editor/cancel-render` | Cancel a render in progress |
| POST | `/editor/apply-rotations` | Re-project layers at new yaw/pitch/roll |
| POST | `/editor/save-seams` | Persist the seam map |
| POST | `/editor/save-clone` | Persist the clone strokes |
| POST | `/projects/{save,load}` | Snapshot or restore the editor state |

The server binds to localhost and has no authentication — it is a local tool, not something to expose to a network.

---

## How It Works

### Stitching Pipeline

1. **Metadata extraction**: Reads DJI EXIF (GimbalYawDegree, GimbalPitchDegree, GimbalRollDegree)
2. **Rotation refinement**: SIFT feature matching between overlapping image pairs, with RANSAC and iterative bundle adjustment (clamped ±2° per iteration, ±15° in total). Each pair is first vetted against the gimbal metadata — pairs implying an implausibly large correction, or whose matches disagree with each other, are discarded, since RANSAC alone will happily fit a homography to repetitive texture such as open water. The adjustment is regularized towards the metadata angles, which also keeps the horizon level.
3. **Gain compensation**: Normalizes exposure across images
4. **Equirectangular projection**: Projects each image onto a full 360x180 sphere using camera intrinsics
5. **Seam detection**: Voronoi-based assignment using distance transforms with horizontal wrapping at ±180°
6. **People preservation**: Haar cascade detection moves seams away from detected subjects
7. **Compositing**: Multi-band blending in linear light, with the painted seam labels deciding which frame owns each pixel — the pyramid only governs how wide the handover is at each spatial frequency, so a seam drawn around an object is respected
8. **Finishing**: Zenith fill for uncovered sky, then a single tone map with a highlight shoulder

### Editor Rendering

The editor uses THREE.js to display the panorama on a sphere. Source images are projected as equirectangular layers with per-pixel seam labels determining which image is visible at each point. Painting modifies the seam labels in real-time and updates the composite texture. Clone strokes are stamped into a separate RGBA layer drawn over that composite, and replayed onto the render at full resolution.

Both painting and cloning wrap horizontally, so a brush that crosses the ±180° seam paints columns at both edges of the equirect.

### CUDA Detection

At import time, `stitcher.py` checks `cv2.cuda.getCudaEnabledDeviceCount()`. If CUDA is available, the render uses GPU-accelerated remap.

---

## Performance

The full-res render uses optimized projection:
- **Seam-driven compositing**: each pixel's owner is decided by the seam map, so blending never mixes two frames across an object
- **Column-clipped projection**: only processes the ~20% of columns where each image is visible
- **CUDA acceleration** (when available): source images uploaded to GPU, remap via `cv2.cuda.remap`
- **Multi-threaded CPU fallback**: ThreadPoolExecutor with up to 8 threads
- **Cancellable**: cancel button stops both client and server-side processing

Typical render time: ~17 seconds on CPU for 8192x4096 (26 source images). Preparing editor data for the same set takes a few minutes, once.

---

## Troubleshooting

**The editor button is greyed out.** The session has no source photos — viewing a single panorama file gives the server nothing to edit. Relaunch with a folder (`python main.py /path/to/photos/ -e`) or import the photos through the browser.

**"The editor is not available for this session."** Same cause, or the images have no DJI gimbal metadata. The editor needs per-image orientation; generic stitched panoramas cannot be edited.

**Stitching failed / "images may not overlap enough".** The OpenCV fallback path could not find enough matches. Use images with DJI metadata, or shoot with more overlap.

**A layer is visibly rotated or offset.** Metadata was off and refinement did not fully correct it. Fix it by hand in Move mode and click Apply Positions.

**Painting is refused with "Apply position changes before painting".** Positions have been moved but not re-projected. Click Apply Positions (or Reset to discard).

**Clone paint disappeared after loading a project.** Strokes belong to the project they were saved with; loading a project that has none clears them. They are also dropped if the editor grid size changed, since the coordinates would no longer line up — the browser console says so when that happens.

**The editor page looks stale or throws after an update.** The browser is holding a cached copy of `editor.html`/`editor.js`. Hard-reload the page; the script tag carries a version query for exactly this reason.

**Port already in use.** Another instance is still running, or something else has 8420. Use `-p` to pick a different port.

**Render is slow.** Check whether CUDA was detected (see [Installation](#installation)). Without it the render is CPU-threaded and takes tens of seconds.

---

## Tuning Constants

These live at the top of the relevant section of `stitcher.py` and are worth knowing about if results need adjusting.

| Constant | Default | Effect |
|----------|---------|--------|
| `MAX_METADATA_OFFSET` | 20.0° | Reject a pair implying a larger correction than this |
| `MAX_MATCH_SCATTER` | 2.5° | Reject a pair whose matches disagree by more than this |
| `METADATA_REG_WEIGHT` | 1.0 | How strongly bundle adjustment is pulled back to the gimbal angles |
| `FOCAL_SCALE_BOUNDS` | 0.8–1.25 | Allowed focal length correction |
| `MULTIBAND_LEVELS` | 6 | Pyramid levels in the blender — higher hides wider exposure steps |
| `SHOULDER_KNEE` | 0.18 | Where the tone curve starts rolling off highlights |
| `TARGET_MIDTONE` | 0.03 | Median luminance the tone map aims for |
| `MAX_MIDTONE_LIFT` | 2.0 | Cap on the global gain that lift can apply |

Render size is set by the `out_width`/`out_height` arguments to `render_with_seams` and `stitch_dji` (8192x4096); the editor works at half that.
