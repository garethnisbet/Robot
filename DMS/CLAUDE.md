# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Running the analysis scripts

Scripts are run directly from the repository root:

```bash
python fit_fivefold_axis_AlPdMn_Not_Annealed_2M_2ROIS_internal_hkl.py
python slider_quasi_AlPdMn_Annealed_hkl.py
```

Each script inserts its own directory into `sys.path` at startup so it can locate `calcms/`. No install or build step is required.

## Architecture

```
DMS/
├── calcms/
│   ├── ts_quasi.py     # Core library: crystallography, MS geometry, fitting, ROI builders
│   └── loader.py       # Reads Diamond Light Source .dat scan files into a dict-like object
├── fit_fivefold_axis_*.py   # Fitting script: loads data, builds ROIs, runs optimiser
├── fit_fivefold_axis_*.json # Config file for the above (same basename)
├── slider_quasi_*.py        # Interactive slider visualiser for quasicrystal MS simulation
└── Processing/              # Timestamped output snapshots (auto-created when save=1)
```

`calcms/ts_quasi.py` is the sole library module. Both analysis scripts import it as `import calcms.ts_quasi as ts` and `from calcms import loader as do`. Full API documentation is in `calcms/README.md`.

## JSON configuration

Each analysis script reads a JSON config with the same basename at startup. Key sections:

| Section | Purpose |
|---------|---------|
| `scan` | `scannum`, `scanpath`, `datapoint`, `datapoint0` — which scan file and image to load |
| `flags` | `save`, `fit`, `firstplot`, `detoptimize`, `energyopt` — boolean run controls |
| `display` | `zoomval` (1 or 2), `colourlim`, `colmap` — image display settings |
| `roi` | `width_per_zoom`, `comwidth_per_zoom` — ROI extraction widths (scaled by `zoomval`) |
| `geometry` | `hkl`, `psi`, `px_unscaled`, `py_unscaled` — primary reflection and detector origin |
| `computation` | `numsteps`, `simsigma_per_zoom`, `thrange_delta`, `bravais`, `opt_method`, `tolerance` |
| `crystal` | `lattice2`, `initial_guess_base`, `ref_6d` — starting parameters and 6D reference reflections |
| `manual_centres` | Dict of `"roi_index": pixel_position` overrides for poorly fitted ROI centres |
| `paths` | `cif_file` — path to CIF file used by `loadcif()` |

## Initial guess parameter vector (fit script)

`initial_guess_base` in the JSON is a 24-element array. Indices:

```
0        a (lattice parameter, Å)
1–2      b, c  (unused for icosahedral — cubic constraint applied)
3–5      alpha, beta, gamma  (unused for icosahedral)
6–9      psicor, hcor, kcor, lcor  (azimuthal/hkl corrections)
10       detdist (detector distance, pixels; halved and scaled by zoomval at runtime)
11–13    dxrot, dyrot, dzrot  (detector rotation angles, degrees)
14       energy offset (added to loaded energy value)
15–23    phason strain matrix elements (3×3 upper-triangular packed)
```

The `bravais` flag selects which subset of indices are passed to the optimiser. For `icosahedral`, parameters [0, 6–9, 10–13, 15–23] (with optional energy) are optimised; lattice parameters 1–5 are locked by symmetry.

## Processing output

When `save=1`, the script creates a timestamped directory under `Processing/`:

```
Processing/YYYYMMDDHHMM_<imnum>_<scannum>_<description>_<fittype>/
    fit_fivefold_axis_*.py   # snapshot of the script
    ts_quasi.py              # snapshot of the library
    IM_<scannum>.png
    _PLOT_<scannum>.svg
    Result.txt
    res.x.txt
    ROIS<scannum>.png
```

These directories are immutable run records — do not modify them.

## Physics context

This code analyses **X-ray multiple scattering (MS)** in an **icosahedral quasicrystal** (Al-Pd-Mn) measured at Diamond Light Source beamline i16. The quasicrystal is indexed in 6D using pairs `(h, k, l)` and `(h', k', l')` where the physical reciprocal vector is `h + h'·τ` (with τ = golden ratio). Phason strain is a 3×3 matrix coupling the perpendicular-space component; it is the main physically interesting quantity being refined. Bragg geometry, Ewald sphere construction, and ROI-based Gaussian peak fitting are all handled by `ts_quasi.py`.

## Dependencies

```
numpy  scipy  matplotlib  PIL(Pillow)  shapely  imageio  cctbx(optional, for loadcif)
```

`cctbx`/`iotbx` imports are commented out in `ts_quasi.py`; `loadcif()` requires them at runtime only when `autoreflist=1`.
