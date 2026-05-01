#!/usr/bin/env python
"""
Unified DMS workflow: interactive slider refinement → automated fitting.

Usage:
    python workflow.py [config.json]

Opens an interactive slider window for manual refinement of the initial guess
overlaid on the experimental detector image. Click "Fit" to launch the
optimizer using the current slider values. Click "Print" to output the
current parameter vector. Click "Reset" to restore the initial guess.
"""

import os, sys, json, time, copy

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib import rc
from scipy import ndimage
from scipy.optimize import minimize, differential_evolution, basinhopping
from joblib import Parallel, delayed
import imageio.v2 as imageio
from time import strftime

dmspath = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, dmspath)
from calcms import ts_quasi as ts
from calcms import loader as do

rc('xtick', labelsize=6)
rc('ytick', labelsize=6)

# ── Load config ──────��───────────────────────────────────────────────────────

if len(sys.argv) > 1:
    cfg_path = os.path.abspath(sys.argv[1])
else:
    cfg_path = os.path.join(
        dmspath,
        'fit_fivefold_axis_AlPdMn_Not_Annealed_2M_2ROIS_internal_hkl.json'
    )

with open(cfg_path) as f:
    cfg = json.load(f)

print(f'Loaded config: {cfg_path}')

# ── Extract config values ─────────────────���──────────────────────────────────

zoomval      = cfg["display"]["zoomval"]
width        = cfg["roi"]["width_per_zoom"] * zoomval
comwidth     = cfg["roi"]["comwidth_per_zoom"] * zoomval
scan = scannum = cfg["scan"]["scannum"]
datapoint0   = cfg["scan"]["datapoint0"]
datapoint    = cfg["scan"]["datapoint"]
scanpath     = cfg["scan"]["scanpath"]
imnum        = datapoint + 1
tolerance    = cfg["computation"]["tolerance"]
scatv        = cfg["geometry"]["scatv"]
detoptimize  = cfg["flags"]["detoptimize"]
energyopt    = cfg["flags"]["energyopt"]
colourlim    = cfg["display"]["colourlim"]
colmap       = cfg["display"]["colmap"]
bravais      = cfg["computation"]["bravais"]
autoreflist  = cfg["flags"]["autoreflist"]
OptMethod    = cfg["computation"]["opt_method"]
strat        = ts.DE_Strategy['best1exp']
intensity    = cfg["computation"]["intensity"]
threshold    = cfg["computation"]["threshold"]
numsteps     = cfg["computation"]["numsteps"]
numsteps_interactive = min(numsteps, 300)
simsigma     = cfg["computation"]["simsigma_per_zoom"] * zoomval
colourmap    = cfg["display"]["colourmap"]
lattice2     = cfg["crystal"]["lattice2"]
cif_file     = cfg["paths"]["cif_file"]
show_centres = cfg["flags"].get("show_centres", 1)
show_numbers = cfg["flags"].get("show_numbers", 1)
axis_off     = cfg["flags"].get("axis_off", 0)
datestr      = strftime("%Y%m%d%H%M")

# ── Load experimental data ────────────────��──────────────────────────────────

d = do.load(scanpath + str(scannum) + '.dat')
met = d.metadata
lattice = [met.a, met.b, met.c, met.alpha1, met.alpha2, met.alpha3]
psi = cfg["geometry"]["psi"]
hkl = np.array(cfg["geometry"]["hkl"])
hkl = hkl * d.energy2[datapoint] / d.energy2[datapoint0]
hklint = np.round(hkl)
imtemplate = str(scannum) + '-pilatus2M-files/%05d.tif'

try:
    energy = d.energy2[datapoint]
except AttributeError:
    try:
        energy = d.metadata.Energy
    except AttributeError:
        try:
            energy = d.DCMenergy
        except AttributeError:
            energy = d.metadata.en

azir = [d.metadata['azih'], d.metadata['azik'], d.metadata['azil']]

# ── Load and filter image ────────────────────���────────────────���──────────────

im_raw = imageio.imread(str(scanpath + str(imtemplate % imnum)))
im = ndimage.zoom(im_raw, zoomval, order=3)
imdata = np.copy(im)

px = cfg["geometry"]["px_unscaled"] * zoomval
py = cfg["geometry"]["py_unscaled"] * zoomval

thb = ts.bragg(lattice, hkl, energy).th()[0]
_td = cfg["computation"]["thrange_delta"]
thrange = [thb + _td[0], thb + _td[1]]
psirange = [psi - 360, psi + 360]
detvects = np.matrix([[1, 0, 0], [0, 0, 1]])
hkllist = ts.pilkhlrange(lattice, hkl, energy, thrange[0], thrange[1]).hklscan(numsteps)
hkllistrange = [thrange[0], thrange[1], numsteps]

# ── Build reflection list ─────────────────────��──────────────────────────────

if autoreflist:
    mslist = [[np.NAN] * 7]
    hkllistcorse = ts.pilkhlrange(lattice, hkl, energy, thrange[0], thrange[1]).hklscan(30)
    SF, reflist, lattice2, structure, sfc = ts.loadcif(cif_file, energy)
    for hklval in range(len(hkllistcorse[:, 0])):
        ms = ts.calcms(lattice, hkllistcorse[hklval, :], hklint, reflist, energy, azir)
        mslist = np.concatenate((mslist, ms.full()), 0)
    mslist = ts.reducebypsirange(mslist, psirange)
    reflist = np.matrix(ts.uniquearray(mslist[:, 0:3]))
    reflist2 = 0
    ref_6d = None
else:
    ref_6d = np.array(cfg["crystal"]["ref_6d"])
    p6d = ts.Projection6d(ref_6d)
    reflist0 = p6d.reflection_6d()
    reflist  = reflist0[0]
    reflist2 = reflist0[1]

# ── Build initial parameter vector ───────���───────────────────────────────────
# 24-element vector:
#   [a, b, c, alpha, beta, gamma, psicor, hcor, kcor, lcor,
#    detdist, dxrot, dyrot, dzrot, energy, a11..a33]

ig_base = np.array(cfg["crystal"]["initial_guess_base"], dtype=float)
ig_base[10] = ig_base[10] / 2 * zoomval
ig_base[14] = energy + ig_base[14]
initial_guess = ig_base.copy()

detdistancepx = initial_guess[10]
rotx  = initial_guess[11]
roty  = initial_guess[12]
rotz  = initial_guess[13]
mtrx2 = list(initial_guess[15:24])

# ── Build ROI kernels & extract centres ──────────────────────────────────────

print('Building ROI kernels...')
builderargs = (
    reflist, hkllist, hklint, intensity, psirange, threshold, hkl,
    detvects, imdata.shape, simsigma, azir, psi, px, py, scatv,
    detdistancepx, rotx, roty, rotz, energy,
    initial_guess, reflist2, mtrx2
)
kernel = ts.roibuilder_ico_hkl(builderargs)

print('Extracting ROI centres...')
imcoeffs, linedatax, linedatay, fitpoints, rois, pcov = ts.multiroifit2(
    imdata, kernel, width, 0.02, 10.0
)
centres = np.array([imcoeffs[:, 2]]).T

for _idx, _val in cfg["manual_centres"].items():
    centres[int(_idx)] = _val / 2 * zoomval

# ── Helper: extract reduced parameter vector ─────────────────────��───────────

def extract_reduced(full_ig):
    if bravais == 'icosahedral':
        if detoptimize:
            if energyopt:
                idx = [0,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
            else:
                idx = [0,6,7,8,9,10,11,12,13,15,16,17,18,19,20,21,22,23]
        else:
            if energyopt:
                idx = [0,6,7,8,9,14,15,16,17,18,19,20,21,22,23]
            else:
                idx = [0,6,7,8,9,15,16,17,18,19,20,21,22,23]
    elif bravais == 'icosahedral_fixed_a':
        if detoptimize:
            if energyopt:
                idx = [6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
            else:
                idx = [6,7,8,9,10,11,12,14,15,16,17,18,19,20,21,22,23]
        else:
            if energyopt:
                idx = [6,7,8,13,14,15,16,17,18,19,20,21,22,23]
            else:
                idx = [6,7,8,14,15,16,17,18,19,20,21,22,23]
    elif bravais == 'cubic_no_strain':
        if detoptimize:
            if energyopt:
                idx = [0,6,7,8,9,10,11,12,13]
            else:
                idx = [0,6,7,8,9,10,11,12]
        else:
            if energyopt:
                idx = [0,6,7,8,13]
            else:
                idx = [0,6,7,8]
    elif bravais == 'calibrate':
        if detoptimize:
            if energyopt:
                idx = [6,7,8,9,10,11,12,13]
            else:
                idx = [6,7,8,9,10,11,12]
        else:
            if energyopt:
                idx = [6,7,8,13]
            else:
                idx = [6,7,8]
    else:
        raise ValueError(f'Unknown bravais: {bravais}')
    return full_ig[idx]

# ── Create fitting object ────────────────────────────────────────────────────

dms = ts.dmsfit_ico_hkl(
    reflist, list(hkllistrange), hklint, psirange, width, centres, kernel,
    hkl, detvects, imdata, simsigma, azir, psi, px, py, scatv,
    bravais, detoptimize, energyopt,
    detdistancepx, rotx, roty, rotz, energy,
    reflist2, mtrx2, initial_guess[0]
)
dms.setCalLattice(initial_guess[:6].tolist())
dms.setLattice(initial_guess[:6].tolist())

# ── Initial computation ──────────────────────────────────────────────────────

ig = initial_guess.copy()
ig_reduced = extract_reduced(ig)
imdata_max = imdata.max()

dms.hkllistrange[2] = numsteps_interactive
try:
    dms.imcalc(ig_reduced)
    imoverlay = np.copy(imdata)
    imoverlay[dms.dmsindex] = imdata_max
except Exception:
    imoverlay = np.copy(imdata)

print(f'Ready. {kernel.shape[2]} ROIs, {len(centres)} centres.')

# ── Build GUI ─────────────��──────────────────────────────���───────────────────

fig = plt.figure('DMS Workflow — Slider → Fit', figsize=(15, 9), dpi=100)

ax_img = fig.add_axes([0.02, 0.05, 0.50, 0.90])
p = ax_img.imshow(imoverlay, cmap=colmap, clim=(colourlim[0], colourlim[1]))
ax_img.set_title(f'Scan {scannum}  dp={datapoint}  E={energy:.4f} keV')

# Slider definitions: (label, ig_index, half_range, format)
slider_defs = [
    ('a',      0,   0.2,   '%0.6f'),
    ('psicor', 6,   5.0,   '%0.6f'),
    ('hcor',   7,   2.0,   '%0.6f'),
    ('kcor',   8,   2.0,   '%0.6f'),
    ('lcor',   9,   2.0,   '%0.6f'),
    ('detdist',10,  300.0, '%0.3f'),
    ('dxrot',  11,  5.0,   '%0.6f'),
    ('dyrot',  12,  5.0,   '%0.6f'),
    ('dzrot',  13,  10.0,  '%0.6f'),
    ('energy', 14,  0.5,   '%0.6f'),
    ('a11',    15,  0.05,  '%0.6f'),
    ('a12',    16,  0.05,  '%0.6f'),
    ('a13',    17,  0.05,  '%0.6f'),
    ('a21',    18,  0.05,  '%0.6f'),
    ('a22',    19,  0.05,  '%0.6f'),
    ('a23',    20,  0.05,  '%0.6f'),
    ('a31',    21,  0.05,  '%0.6f'),
    ('a32',    22,  0.05,  '%0.6f'),
    ('a33',    23,  0.05,  '%0.6f'),
]

n_sliders = len(slider_defs)
spacing = np.linspace(0.94, 0.06, n_sliders + 3)
thickness = 0.022
sl = 0.60
sw = 0.35

sliders = {}
for i, (label, idx, half_range, fmt) in enumerate(slider_defs):
    ax = fig.add_axes([sl, spacing[i], sw, thickness])
    val = ig[idx]
    sldr = Slider(ax, label, val - half_range, val + half_range,
                  valinit=val, valfmt=fmt, color='gray')
    sliders[idx] = sldr

method_row_y = spacing[n_sliders]
btn_y = spacing[n_sliders + 1]
btn_h = 0.035
ax_fit   = fig.add_axes([sl,         btn_y, 0.10, btn_h])
ax_reset = fig.add_axes([sl + 0.115, btn_y, 0.10, btn_h])
ax_print = fig.add_axes([sl + 0.230, btn_y, 0.10, btn_h])
btn_fit   = Button(ax_fit,   'Fit',   color='lightgreen', hovercolor='green')
btn_reset = Button(ax_reset, 'Reset', color='lightyellow', hovercolor='yellow')
btn_print = Button(ax_print, 'Print', color='lightblue',  hovercolor='deepskyblue')

# ── Algorithm selector ────────────────────────────────────────────────────────

method_labels = [
    ('COBYLA',  'COBYLA'),
    ('N-Mead',  'Nelder-Mead'),
    ('Powell',  'Powell'),
    ('BH+Pw',   'BHPowell'),
    ('BH+Co',   'BHCOBYLA'),
    ('BH+NM',   'BHNelderMead'),
    ('GA',      'GA'),
]
active_method = [OptMethod]
n_methods = len(method_labels)
btn_w_m = sw / n_methods
method_axes_map = {}
method_btns_map = {}

for i, (lbl, mstr) in enumerate(method_labels):
    ax_m = fig.add_axes([sl + i * btn_w_m, method_row_y, btn_w_m - 0.002, 0.028])
    color = 'lightgreen' if mstr == OptMethod else 'lightgray'
    btn_m = Button(ax_m, lbl, color=color, hovercolor='palegreen')
    method_axes_map[mstr] = ax_m
    method_btns_map[mstr] = btn_m

def make_method_cb(chosen):
    def cb(event):
        active_method[0] = chosen
        for ms, ax in method_axes_map.items():
            ax.set_facecolor('lightgreen' if ms == chosen else 'lightgray')
        fig.canvas.draw_idle()
    return cb

for mstr, btn_m in method_btns_map.items():
    btn_m.on_clicked(make_method_cb(mstr))

status_y = spacing[n_sliders + 2]
ax_status = fig.add_axes([sl, status_y, sw, 0.025])
ax_status.set_axis_off()
status_text = ax_status.text(0, 0.5, 'Ready — adjust sliders then click Fit',
                             fontsize=9, va='center', family='monospace')

# ── Slider callback ──────────────��───────────────────────────────────────────

_suppress_update = False
_last_update_time = 0.0

def sync_sliders_to_ig():
    for idx, sldr in sliders.items():
        ig[idx] = sldr.val
    ig[1] = ig[2] = ig[0]
    ig[3] = ig[4] = ig[5] = 90.0

def update(val):
    global _last_update_time
    if _suppress_update:
        return
    now = time.time()
    if now - _last_update_time < 0.15:
        return
    _last_update_time = now

    sync_sliders_to_ig()
    reduced = extract_reduced(ig)
    dms.hkllistrange[2] = numsteps_interactive
    try:
        dms.imcalc(reduced)
        overlay = np.copy(imdata)
        overlay[dms.dmsindex] = imdata_max
        p.set_data(overlay)
    except Exception:
        p.set_data(imdata)
    fig.canvas.draw_idle()

for sldr in sliders.values():
    sldr.on_changed(update)

# ── Fit callback ───────────────��─────────────────────────��───────────────────

def on_fit(event):
    global _suppress_update

    sync_sliders_to_ig()
    status_text.set_text('Fitting...')
    fig.canvas.draw_idle()
    fig.canvas.flush_events()

    reduced = extract_reduced(ig)
    dms.hkllistrange[2] = numsteps

    dms.detdistancepx = ig[10]
    dms.detxrot = ig[11]
    dms.detyrot = ig[12]
    dms.detzrot = ig[13]
    dms.energy  = ig[14]
    dms.a       = ig[0]
    dms.setLattice([ig[0], ig[0], ig[0], 90, 90, 90])

    starttime = time.time()

    iglow  = reduced - 1.5
    ighigh = reduced + 1.5
    bounds = list(zip(iglow, ighigh))

    cur_method = active_method[0]
    try:
        if cur_method == 'GA':
            print('Using Differential Evolution with strategy ' + strat)
            res = differential_evolution(dms.fit, bounds, strategy=strat,
                                         polish=True, workers=-1)
        elif cur_method in ('BHPowell', 'BHCOBYLA', 'BHNelderMead'):
            bh_map = {'BHPowell': ('Powell', 150),
                      'BHCOBYLA': ('COBYLA', 400),
                      'BHNelderMead': ('Nelder-Mead', 400)}
            method, niter = bh_map[cur_method]
            print(f'Using Basinhopping ({method})')
            minimizer_kwargs = {"method": method}
            res = basinhopping(dms.fit, reduced,
                               minimizer_kwargs=minimizer_kwargs, niter=niter)
        else:
            n_starts = cfg["computation"].get("n_parallel_starts", 4)
            print(f'Using {cur_method} with {n_starts}-start parallel search')
            rng = np.random.default_rng(42)
            starts = [reduced] + [
                reduced + rng.uniform(-0.5, 0.5, reduced.shape)
                for _ in range(n_starts - 1)
            ]
            def _run_one(_ig_start):
                _dms = copy.deepcopy(dms)
                return minimize(_dms.fit, _ig_start, method=cur_method,
                                tol=tolerance,
                                options={'xtol': tolerance, 'ftol': tolerance})
            results = Parallel(n_jobs=n_starts)(
                delayed(_run_one)(s) for s in starts
            )
            res = min(results, key=lambda r: r.fun)

        elapsed = time.time() - starttime

        dms.hkllistrange[2] = numsteps
        opt, simim, dmsindex, dataim, inputarray = dms.full(res.x)

        _suppress_update = True
        for idx, sldr in sliders.items():
            if idx < len(inputarray):
                sldr.set_val(inputarray[idx])
        ig[:] = inputarray
        _suppress_update = False

        overlay = np.copy(imdata)
        try:
            overlay[dmsindex] = imdata_max
        except (IndexError, TypeError):
            pass
        p.set_data(overlay)

        status_text.set_text(
            f'Fit complete.  χ²={opt:.4f}  t={elapsed:.1f}s  [{active_method[0]}]'
        )
        fig.canvas.draw_idle()

        print(f'\nFit complete in {elapsed:.1f}s')
        print(f'χ² = {opt:.6f}')
        print('initial_guess = np.array(['
              + ','.join(f'{v:.6f}' for v in inputarray) + '])')

        show_roi_comparison(simim)

    except Exception as e:
        elapsed = time.time() - starttime
        status_text.set_text(f'Fit failed: {e}')
        fig.canvas.draw_idle()
        print(f'\nFit failed after {elapsed:.1f}s: {e}')
        import traceback
        traceback.print_exc()

btn_fit.on_clicked(on_fit)

# ── Reset callback ────────────────���──────────────────────────────────────────

def on_reset(event):
    global _suppress_update
    ig[:] = initial_guess
    _suppress_update = True
    for idx, sldr in sliders.items():
        sldr.set_val(ig[idx])
    _suppress_update = False
    update(None)
    status_text.set_text('Reset to initial guess from config')

btn_reset.on_clicked(on_reset)

# ── Print callback ────────────��──────────────────────────────���───────────────

def on_print(event):
    sync_sliders_to_ig()
    print('\n' + '=' * 72)
    print('Current 24-element parameter vector:')
    print('initial_guess = np.array(['
          + ','.join(f'{v:.6f}' for v in ig) + '])')
    print()
    reduced = extract_reduced(ig)
    print(f'Reduced ({bravais}, detopt={detoptimize}, eopt={energyopt}):')
    print('ig = np.array(['
          + ','.join(f'{v:.6f}' for v in reduced) + '])')
    print()

    labels = ['a', 'psicor', 'hcor', 'kcor', 'lcor', 'detdist',
              'dxrot', 'dyrot', 'dzrot', 'energy',
              'a11', 'a12', 'a13', 'a21', 'a22', 'a23',
              'a31', 'a32', 'a33']
    indices = [0, 6, 7, 8, 9, 10, 11, 12, 13, 14,
               15, 16, 17, 18, 19, 20, 21, 22, 23]
    for lbl, idx in zip(labels, indices):
        print(f'  {lbl:8s} = {ig[idx]:.6f}')
    print('=' * 72)

btn_print.on_clicked(on_print)

# ── ROI comparison figure ────────────────────────────────────��───────────────

def show_roi_comparison(simim):
    imcoeffs_sim, linedatasimx, linedatasimy, fitpointssim, rois2, covmat = \
        ts.multiroifit(simim, kernel, width, 10)

    subcellsx = cfg["display"].get("subcellsx", 7)
    subcellsy = cfg["display"].get("subcellsy", 4)

    fig2, axlist = plt.subplots(subcellsx, subcellsy, figsize=(6, 10))
    fig2.canvas.manager.set_window_title('ROI Comparison — Data vs Fit')

    ii = np.indices((subcellsx, subcellsy))
    irow = ii[0].flatten()
    icol = ii[1].flatten()

    refnum = 0
    roicount = 0

    for i1 in range(min(kernel.shape[2], subcellsx * subcellsy)):
        ax = axlist[irow[i1], icol[i1]]
        ax.plot(linedatax[i1], linedatay[i1], '-', c='green', linewidth=0.5)
        ax.plot(linedatax[i1], fitpoints[i1], '.', markersize=2, c='r')

        if show_centres:
            ax.plot([centres[i1], centres[i1]],
                    [fitpoints[i1].min(), linedatay[i1].max()],
                    'g', linewidth=0.5)

        title = ''
        if ref_6d is not None:
            if show_numbers:
                title = f'{i1} {ref_6d[refnum, :]}'
            else:
                title = str(ref_6d[refnum, :])
        else:
            title = str(i1)
        ax.set_title(title, fontsize=8)

        denom = linedatasimy[i1].max() - linedatasimy[i1].min()
        if abs(denom) < 1e-10:
            denom = 1.0
        yscale = (linedatay[i1].max() - linedatay[i1].min()) / denom
        yoffset = linedatay[i1].min() - (linedatasimy[i1] * yscale).min()
        ax.plot(linedatasimx[i1], (linedatasimy[i1] * yscale) + yoffset,
                '-.', c='blue', linewidth=0.5)
        ax.plot(linedatasimx[i1], (fitpointssim[i1] * yscale) + yoffset,
                '.', markersize=2, c='g')

        if axis_off:
            ax.set_axis_off()
        if roicount == 1:
            refnum += 1
            roicount = -1
        roicount += 1

    for i1 in range(kernel.shape[2], subcellsx * subcellsy):
        axlist[irow[i1], icol[i1]].set_axis_off()

    plt.tight_layout()
    plt.show(block=False)

# ── Show ─────────────���────────────────────────────��──────────────────────────

plt.show()
