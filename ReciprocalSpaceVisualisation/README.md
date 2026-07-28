# Reciprocal Space Visualisation

Interactive 3D **reciprocal-space map** viewer for **conventional crystals** and
**icosahedral quasicrystals**, with a **draggable detector panel** that maps
real-space features into reciprocal space, live. Built in the style of the
`RobotVisualisation` project: a lean [aiohttp](https://docs.aiohttp.org/) server
serving a modular [Three.js](https://threejs.org/) front end with a dark control
panel and on-demand rendering.

The crystallography is a JavaScript port of the validated routines in the sibling
`DMS` project (`DMSAnalysis/ts_quasi.py`), so the map rebuilds instantly in the
browser as you drag the controls — no server round-trip.

## Quick start

```bash
pip install aiohttp        # server dependency
npm install                # fetches node_modules/three
python server.py           # serves on http://localhost:8080
```

Then open **http://localhost:8080**.

## The idea

Everything lives in one scene, sharing the sample at the origin:

- the **reciprocal lattice** (Å⁻¹) of the crystal or quasicrystal you set up, and
- a **detector panel** you drag around that scene.

A point on the panel defines a scattered-beam *direction* from the sample, so it
maps into reciprocal space by the one line of physics behind the whole tool:

```
Q = k0 · û(point) − k_in                 (elastic; k = 1/λ, so |Q| = 2 sinθ/λ)
```

Because only the *direction* matters, how far away the panel is drawn is a display
choice, and the mapping is computed entirely in the browser — nothing is sent to
the server as you drag. See `js/optics.js`.

## What you can do

- **Set the crystal.** *Crystal* mode builds the reciprocal lattice from the six
  lattice parameters (`a, b, c, α, β, γ`) via the Busing–Levy **B-matrix**
  (`js/crystal.js`), with Bravais **systematic-absence** rules for the centring
  (P/I/F/A/B/C/R) and the seven crystal systems. *Quasicrystal* mode builds
  icosahedral peaks from the **6D cut-and-project** (`js/quasicrystal.js`): each
  rank-6 index is projected to physical space `Q∥` and phason space `Q⊥`, which
  sets the intensity (`I ∝ exp(−(|Q⊥|/σ)²)`). Hover a peak for its indices and `|Q|`.

- **Cut into the map with the visibility box.** *Place visibility box* drops a
  draggable box, fitted to half the extent of the current peak cloud, that hides
  everything **inside** or **outside** it (`js/visbox.js`, ported from the
  LidarStudio viewer). Use it to see the peaks a shell of outer reflections is
  hiding, or to isolate one slab of the DMS curve network. **Move / rotate /
  scale** it with its own gizmo — or press **T / R / S** — so it can be tilted
  onto any plane of interest; *showing: inside/outside* flips which side survives;
  *clear* brings everything back. **Nothing is deleted**: it is a view filter, and
  a peak it hides is not pickable by the hover tooltip either. *re-fit box to
  data* re-centres it on the current cloud; clearing remembers the pose, so
  placing again brings the box back where it was.

  The cut is six of Three's local **clipping planes** — the box's own inward
  faces — assigned to the materials of the reciprocal-space content only, so the
  axes, the basis arrows and the gizmos are never sliced. The detector panel is
  instrumentation and is left whole unless *cut the detector panel too* is
  ticked. "Showing outside" is the same six planes with `clipIntersection`, which
  discards only where all of them clip — i.e. only the box interior.

- **Drag the detector panel.** Turn on the panel and a gizmo appears. **Translate**
  moves it; **rotate** turns it about its **own origin** (in place) or about the
  **sample** (orbiting the origin). Panel size and the display distance are
  adjustable. Orbit controls step aside while you drag the gizmo.

  The panel is a **to-scale model of the detector the DMS engine computes
  against** — the frame size, distance, plate rotations and beam centre, taken
  from a loaded DMS state (`initial_guess` slots 10-13 plus `px`/`py`) and
  inverted out of the engine's own pixel map by `dms_compute`. **place on
  detector (as fitted)** puts it back there; recomputing DMS follows any change,
  unless you have dragged it, in which case it stays where you put it.

  Note where that is: the panel sits **opposite the incident beam** (back-
  reflection, 2θ ≈ 180°), on the far side of the Ewald sphere, where the
  diffracted Kossel beams actually go — a scattered ray `k_out = k_in + Q` leaves
  the sample and lands on it. (`dms_compute` places it on the −`k_in` side because
  the engine's own pixel map is a point reflection; the earlier +`k_in`, 2θ ≈ 0
  placement put the panel where the DMS curves' forward rays could reach it only
  by extending backwards through the sample.) The **primary reflection is not on
  it** — for `Quasi_Reference.json` its exit beam is at 2θ = 87.44°, ~93° off the
  plate centre, while the plate subtends ±6.3°. Both angles are in the readout, so
  an empty middle doesn't read as a bug. Before DMS has run there is no detector
  to model and the panel starts 30° off the beam (on-axis would collapse the
  scattering triangle).

- **Map a Gaussian blob.** Put a blob on the panel and drag it across the surface
  (or move the panel). Its footprint is pushed through `Q = k0·û − k_in` and drawn
  in reciprocal space, where it visibly **bows onto the Ewald sphere**; the centre
  `|Q|` is read out live.

- **Project DMS lines.** *Compute DMS* fetches the multiple-scattering (Kossel)
  loci from the server (`/api/dms`), drawn as **obstacle curves** in reciprocal
  space on the Ewald sphere. The engine's **own detector-pixel projection** of
  those lines — the very pattern the `DMSAnalysis.slider` paints on its detector
  image — is drawn straight onto the panel, so the panel reproduces the slider
  exactly (the panel rectangle is inverted out of that same pixel map, so its own
  projection can be used verbatim; a ray-trace of `k_out = k_in + Q` from the
  sample would *not* match, because the engine anchors each beam at the detector
  centre, not the origin). The matching Q-space loci are highlighted **back in
  Q** — the round trip.

## DMS multiple-scattering physics

A DMS line comes from **one secondary reflection** — the multiply-scattered
radiation exits along a **cone of beam directions** all satisfying that plane's
diffraction condition, so every point on the line shares the same secondary hkl.
Pushed through `Q = k_out − k_in`, each cone becomes a **curve in reciprocal
space** lying on the Ewald sphere. The server (`/api/dms` → `dms_compute.py`)
reuses the DMS project's `ts_quasi.dmscalc_ico` (one engine for both crystal
types; conventional uses a zero phason component) to build the doubly-diffracted
beam directions and map them to Q, returning the curves in the crystal (B-matrix)
frame — the same frame the lattice is drawn in.

Set the primary hkl / energy / azimuthal reference and the secondary depth (or, for
quasicrystals, the 6D index set), then **Compute DMS**. Or load the list that was
actually fitted — see below. When it runs, the shared
incident beam is re-pointed along the DMS incident wavevector, so the panel/blob
mapping and the DMS mapping agree and the recovered points land back on the curves.

The **ψ slider** rotates the crystal about the primary scattering vector (a
Renninger / azimuthal scan): the primary stays in the Bragg condition while the
secondaries sweep in and out, so the curve network turns about **Q**. Once curves
exist, dragging ψ re-runs the computation (debounced) and the beam/Ewald sphere
sweep around the fixed reciprocal lattice.

**ψ step (°)** sets the slider's increment, so arrow keys (or a click on the
track) walk the azimuthal scan by exactly that much — 0.01° to 90°, fractional
steps included. It is also the spacing of the trail slices below, so the two
controls together decide the resolution of a sweep.

**leave a trail as ψ sweeps** keeps each slice in place instead of replacing it,
so a sweep draws out the surface the DMS cones trace through reciprocal space —
the Kossel network woven over the Ewald sphere rather than one ψ at a time.
Older slices fade so the sweep direction reads and a long trail stays legible.
Turning it off stops recording but leaves what is there to inspect; **Clear
trail** discards it, and **trail length** caps how many slices are held (the
oldest is dropped past the limit, and lowering it trims immediately). The note
under the controls does the arithmetic — `360° at 9° = 40 slices` — and says so
when the sweep would outrun the cap.

One slice is recorded per ψ the recompute actually runs at, and that recompute is
debounced (200 ms) and coalesced while a request is in flight. So **arrow-keying
the slider lays down one slice per step, while a fast drag records only where you
paused** — a 12-step drag with no pauses leaves 2 slices, the same drag stepped
deliberately leaves all 12.

**Sweep ψ (record every step)** removes that trap: it walks the whole range at
the step size and *awaits* each computation, so every stop lands a slice and the
trace comes out evenly spaced. It turns the trail on if it is off, clears what is
there so the result is exactly one sweep, and reports progress (`sweeping ψ…
62/360`). Press it again to stop part-way — ψ stays where it stopped and the
partial trace is kept. Because ψ is an azimuth, +180° is the same orientation as
−180°, so a sweep stops one step short of wrapping: `ceil(360 / step)` slices,
which is exactly the count the note quotes beforehand.

A sweep still honours **trail length** — sweeping 40 steps with a cap of 20 keeps
the last 20 and says so (`swept 40 ψ steps · trail holds 20`). Raise the cap
before a fine sweep if you want the whole thing, keeping the vertex cost above in
mind.

Each slice is one merged `LineSegments` — every curve at that ψ in a single
geometry with per-vertex colour — so a slice costs *one* draw call rather than
one per curve; a scene with ~500 secondaries would otherwise cost hundreds of
draw calls per slice. That merge is what makes a long trail affordable, but the
vertices are still real, and the cost per slice scales with the **secondary
set**, not with the trail length:

| secondary depth | curves / slice | per slice | 360 slices | 1000 slices |
|---|---|---|---|---|
| 1 | 48 | 184 kB | 65 MB | 180 MB |
| 2 | 200 | 806 kB | 283 MB | 787 MB |
| 3 | 482 | 1.8 MB | 643 MB | 1.8 GB |

**trail length** caps it, up to 1000 slices. A full 1° sweep (360 slices) is
comfortable with a shallow secondary set and finished in ~11 s; the same trail at
depth 3 is over half a gigabyte, so raise the cap for fine sweeps of a *sparse*
secondary set rather than a dense one. The note under the controls reports what
is actually held (`360 slices held, ψ -180…179° · 65 MB`) so the number is in
front of you before it becomes a problem.

Requires the sibling `DMS` project importable (set `DMS_PATH` or keep it beside
this project) and `numpy`/`scipy` (`pip install -e '.[dms]'`).

### Curve method: sampled sweep, or the circle the cone actually is

Because a DMS locus is a **cone** of exit directions, and `k_in` is a fixed
translation, each locus is *exactly a circle in Q*. Measured against this engine
the fit residual is ~1e-15 Å⁻¹ — machine precision — at any ψ, in both lattice
modes, with or without phason strain. The **curve method** dropdown picks how
that is delivered:

| Method | Response | Payload (241 secondaries) |
|--------|----------|---------------------------|
| `θ-sweep (sampled)` | `qcurves`: polylines of sampled points | ~960 kB |
| `circles (analytic)` | `circles`: `{c, r, n, a0, a1}` per arc, tessellated client-side | ~130 kB |

Both run the *same* θ-scan at `θ-scan steps`, so at equal steps the two agree to
within the sweep's own output rounding — the circles are not an approximation of
the sampled curve, they are what it lies on. What changes is **what the steps
buy**: in sweep mode they set the resolution of the whole curve, so lowering them
facets it; in circle mode the curve between the ends is exact and the steps only
locate where each arc *stops* (at the θ where the Ewald intersection becomes
non-physical). So circle mode can be run far coarser — 64 steps costs ~66 ms
against ~103 ms for a 160-step sweep — and the curves stay perfectly smooth at
any zoom, because the client tessellates the arc rather than receiving it.

`θcor` is the one case where the circle is not exact: it offsets the exit polar
angle *after* the azimuth was solved at the uncorrected θ, shearing the locus
slightly off-plane — first order in θcor, ~5e-4 Å⁻¹ at θcor = 1°, against a Q
scale of ~1.3 Å⁻¹. The response carries `circleResidual` and the panel appends it
to the status line rather than hiding it.

The arc angles `a0`/`a1` are measured against an in-plane axis that is *derived*
from `n`, not transmitted — `_plane_basis` in `dms_compute.py` and `planeBasis`
in `js/dms.js` must stay identical.

## Loading a reflection list from a DMS JSON file

**Reflection list (DMS JSON)** in the panel loads a list the `DMS` project wrote —
either with the **file picker**, or by **path** (the lists live in the sibling
project; the path is read server-side by `POST /api/reflections` and remembered
for next time). Both of its JSON shapes are understood (`js/reflist.js`):

| Shape | Where the list is | Example |
|-------|-------------------|---------|
| saved state / reflection list | `ref_6d`, `ref_6d_checked`, `hkl`, `azir`, `initial_guess[24]` | `DMS/iAlPdMn_Annealed_913223.json` |
| fit / workflow config | `crystal.ref_6d`, `experiment.*`, `geometry.*`, `computation.*` | `DMS/workflow_913123_dp3.json` |

The row width picks the lattice mode: **6 wide** → icosahedral rank-6 indices,
**3 wide** → conventional `hkl`. The loaded list then *replaces the generated
secondary set* for **Compute DMS** — `ref_6d` instead of the set derived from the
6D sliders, `reflist` instead of the hkl box — and the geometry that came with it
is applied: the fitted lattice (`a₆D` or `a b c α β γ`, with the Bravais setting),
the **energy**, the **primary reflection**, the azimuthal reference, ψ and its
correction (slot 6), the θ-range and, for a quasicrystal, the **phason strain
matrix** (slots 15-23 of the 24-slot guess vector in `DMSAnalysis/fit.py`).
Everything stays editable afterwards.

`ref_6d_checked` records which reflections were ticked in the DMS slider UI; **use
only checked reflections** honours it, and **use this list for DMS** switches back
to the generated set without unloading. Path loads are confined to this project,
the sibling `DMS` project and anything in `RECIP_JSON_ROOTS`.

### The azimuthal reference: from the file when it has one, else from the scan

`azir` sets the axis the multiple-scattering condition is computed about, so
without the right one every DMS line is in the wrong place. The viewer takes it
from the file wherever the DMS project writes it:

| Where | Which files |
|-------|-------------|
| top-level `azir` | slider states from **version 3** on (`DMSAnalysis/slider.py`) |
| `experiment.azir` | fit / workflow configs |
| `geometry.azir` | tripfit configs |

A version-3 state stores it already re-indexed by the active pseudo-cubic
transform, exactly like its `hkl` and `ref_6d`, so it is used as it stands. The
status line under the loaded file reports `azir … (from file)`.

Older states (`DMS/Quasi_Reference.json`) stored only what the *sliders* held and
have no `azir` — the slider re-read it from the scan's `.dat` when it restored a
session, and for those files the viewer still does the same: on load it calls
`POST /api/scanmeta` with the file's `scan` block, which runs
`DMSAnalysis.dat2config.extract_metadata`, and reports `azir … (from scan)`. Scan
data normally lives outside this project, so point `RECIP_SCAN_ROOTS` at it:

```bash
RECIP_SCAN_ROOTS=/Barracuda/Python/Quasicrystal/Data python server.py
```

If the file has an `azir` the scan is not consulted at all — the stored value is
what that session was actually computed with, even if the scan's differs. Only
when neither source yields a usable direction (a zero vector is not one) does the
panel say so and leave `azimuthal ref` for you to fill in by hand — it does *not*
quietly fall back to a default.

Two conventions worth knowing, both matching `DMSAnalysis/slider.py`: the **θ range
is relative to the primary's Bragg angle θ_B** (the `thrange_delta` of a fit
config), and the **phason matrix is a strain** — `q∥ → q∥ + P·q⊥`, so an
unstrained quasicrystal has `P = 0`, not the identity.

## The pixel ↔ Q transformation, from first principles

`geometry.py` holds a full detector model and the pixel ↔ **Q** transformation the
way a detector is actually known on a beamline (position, orientation, pixel size,
size, origin pixel), used as the reference implementation and exercised by the
tests:

```python
from geometry import Beam, Detector, Goniometer, q_map, q_from_pixel, pixel_from_q

beam = Beam(energy_kev=15.0)                     # k = 1/λ; pass two_pi=True for 2π/λ
det  = Detector.from_beam_centre(distance=100.0, shape=(512, 512),
                                 pixel_size=0.3, origin_pixel=(255.5, 255.5),
                                 beam=beam, tilt=(6.0, -4.0, 15.0))
q    = q_map(det, beam)                          # (n_slow, n_fast, 3) in Å⁻¹
```

The physics is just `k_in = k0·ŝ0`, `k_out = k0·unit(pixel position)`,
`Q = k_out − k_in`, so every pixel of a still frame lands on the Ewald sphere.
`Goniometer` rotates that surface into the sample/crystal frame. **Lab frame:**
beam along **+y** by default, `+x` transverse, `+z` vertical.

```bash
pip install -e '.[dev]'
pytest -q -p no:anyio           # geometric identities + pixel → Q → pixel round trips
```

The tests pin the invariants directly: the beam centre maps to `Q = 0`, every pixel
satisfies `|Q + k_in| = k0` and `|Q| = 2 sinθ/λ`, `Q·(k_in + k_out) = 0`,
pixel → Q → pixel is the identity, and tilting about the origin pixel leaves it on
the beam.

## Files

| File | Role |
|------|------|
| `server.py` | aiohttp server: serves the viewer; `POST /api/dms` for DMS curves, `POST /api/reflections` to read a reflection-list JSON by path, `POST /api/scanmeta` to recover a session's beamline metadata (`azir`) from its scan `.dat` |
| `dms_compute.py` | DMS backend — reuses the DMS project's `ts_quasi` (beam cones → Q curves), as sampled polylines or as the analytic circles they lie on |
| `geometry.py` | detector geometry reference: `Beam`, `Detector`, `Goniometer`, pixel ↔ Q |
| `tests/test_geometry.py` | geometric identities and the pixel → Q → pixel round trip |
| `reciprocal_space.html` | entry page (Three.js importmap + panel container) |
| `viewer.css` | dark panel + HUD styling |
| `js/state.js` | shared params (lattice, display, detector, blob, dms) + render flag |
| `js/optics.js` | the incident beam and the point → Q mapping (client-side physics) |
| `js/scene.js` | scene, cameras, OrbitControls, TransformControls gizmo, axes/basis, nav-ball |
| `js/crystal.js` | conventional reciprocal lattice (B-matrix + absence rules) |
| `js/quasicrystal.js` | icosahedral 6D → 3D cut-and-project |
| `js/reflist.js` | reads a DMS reflection-list JSON (either shape) → secondary list + geometry |
| `js/reflections.js` | builds the `THREE.Points` peak cloud (per-point size/colour) |
| `js/detector.js` | the draggable panel, the on-panel Gaussian blob, and the DMS → panel → Q map |
| `js/dms.js` | fetches `/api/dms`; renders DMS Q-curves, Ewald sphere, incident beam |
| `js/visbox.js` | the visibility box: an oriented clip box hiding everything inside/outside it |
| `js/panel.js` | dark control panel: lattice, display, visibility box, detector, feature, reflection list, DMS |
| `js/main.js` | init, render loop, peak-hover tooltip, nav-gizmo mouse events |
