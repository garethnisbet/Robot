#!/usr/bin/env python3
"""
DMS (multiple-scattering / Kossel) line computation for the reciprocal-space
viewer — Phase 2 backend.

Reuses the validated crystallography in the sibling ``DMS`` project
(``DMSAnalysis/ts_quasi.py``); nothing is re-derived here. For a chosen primary
reflection + energy we:

  1. build the secondary reflection list (conventional: ``hklgen_3d``;
     icosahedral: ``Projection6d`` of a 6D index set),
  2. sweep the primary through a θ-range (``pilkhlrange().hklscan``) — this traces
     each DMS line,
  3. run ``dmscalc_ico`` (one engine for both crystal types; conventional uses a
     zero perpendicular component + identity phason matrix), which builds the
     doubly-diffracted beam directions ``vecs`` and their detector pixels,
  4. map each beam direction to reciprocal space, ``Q = k_out − k_in``, giving one
     curve per secondary reflection — the "obstacle" in reciprocal space (every
     point on a curve shares the same secondary hkl).

``method`` picks how step 4 is delivered. ``"sweep"`` sends the sampled polylines
(``qcurves``). ``"circle"`` exploits the fact that a DMS locus is a *cone* of exit
directions, so in Q — a fixed ``k_in`` translation away — it is exactly a circle:
each run is reduced to ``{c, r, n, a0, a1}`` (``circles``) and the client
tessellates the arc at display resolution. Same geometry, ~10x less payload.

Frame: everything is returned in the ``calcms`` aligned frame (primary ∥ +z), so
the curves, the lattice points (``calcms.tv()``) and the detector share one frame.
The psith2v beam frame differs from that frame by a Y-flip
(``Yflip(-ko·psith2v(0,θ)) = calcms.kov()``), which is applied to every vector.
"""

import os
import numpy as np

# ── Locate and import the sibling DMS project's ts_quasi ────────────────────
_DMS_PATHS = [
    os.environ.get("DMS_PATH"),
    os.path.join(os.path.dirname(__file__), "..", "DMS"),
    os.path.expanduser("~/Dropbox/ClaudeCodeProjects/DMS"),
]


def _import_ts():
    import sys
    last = None
    for p in _DMS_PATHS:
        if not p:
            continue
        p = os.path.abspath(p)
        if os.path.isdir(os.path.join(p, "DMSAnalysis")):
            if p not in sys.path:
                sys.path.insert(0, p)
            from DMSAnalysis import ts_quasi as ts  # noqa: E402
            return ts
    raise ImportError(
        "Could not find the DMS project (DMSAnalysis/ts_quasi.py). "
        "Set DMS_PATH or place DMS beside this project. Tried: %s"
        % [p for p in _DMS_PATHS if p]
    )


def scan_metadata(dat_path, datapoint=None, datapoint0=None):
    """Beamline metadata for one point of a scan, via ``DMSAnalysis.dat2config``.

    A saved DMS slider state carries the refined parameters but not the
    experiment: the lattice it started from, the beam energy and — critically —
    the azimuthal reference direction ``azir`` are read back off the scan's
    ``.dat`` file when the slider restores a session. Returns the same dict the
    slider consumes: ``lattice, energy, energy0, azir, image_template``.
    """
    _import_ts()                      # puts the DMS project on sys.path
    from DMSAnalysis import dat2config
    dp  = 1 if datapoint  is None else int(datapoint)
    dp0 = 1 if datapoint0 is None else int(datapoint0)
    meta = dat2config.extract_metadata(dat_path, dp, dp0)
    return {k: (list(v) if isinstance(v, (list, tuple)) else v) for k, v in meta.items()}


def _yflip(a):
    """psith2v beam frame → calcms aligned frame (primary ∥ z)."""
    a = np.array(a, dtype=float)
    a[..., 1] = -a[..., 1]
    return a


def _break_mask(pts, idx=None, gap_factor=6.0):
    """Where an ordered point sequence stops being one continuous arc.

    A DMS line is *not* one connected sweep: the Ewald construction gives two
    branches (psi1/psi2) and each branch drops out wherever the intersection is
    non-physical. Drawing all of a reflection's points as a single polyline
    therefore paints straight connectors between the ends of unrelated arcs.
    Break wherever the sweep index skips a step, or where consecutive points
    jump much further apart than the typical sample spacing.
    """
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    nz = d[d > 0]
    brk = (d > gap_factor * np.median(nz)) if nz.size else np.zeros(len(d), bool)
    if idx is not None:
        brk = brk | (np.diff(np.asarray(idx)) != 1)
    return brk


def _split_polyline(pts, idx=None, gap_factor=6.0, min_pts=2):
    """Split an ordered point sequence into continuous polylines."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return []
    segs = np.split(pts, np.flatnonzero(_break_mask(pts, idx, gap_factor)) + 1)
    return [s for s in segs if len(s) >= min_pts]


def _split_run_rows(pts, idx, rows, gap_factor=6.0):
    """As ``_split_polyline``, but returns each run as *row indices* into Q."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return []
    return np.split(rows, np.flatnonzero(_break_mask(pts, idx, gap_factor)) + 1)


def _plane_basis(n):
    """Deterministic in-plane axis for a circle of normal ``n``.

    The arc angles are measured in this basis, so it is *not* sent — the client
    rederives it. Both sides must use exactly this rule: see `planeBasis` in
    js/dms.js. (n × x̂, falling back to n × ŷ when n is along x̂.)
    """
    u = np.array([0.0, n[2], -n[1]])
    if u @ u < 1e-18:
        u = np.array([-n[2], 0.0, n[0]])
    return u / np.sqrt(u @ u)


def _fit_arcs(Q, runs):
    """Fit every run's circle at once → ``(centres, radii, normals, a0, a1, resid)``.

    Every DMS locus is a cone of exit directions, so in Q — where ``k_in`` is a
    fixed translation — it is exactly a circle; measured against this engine the
    fit residual is at machine precision (~1e-15) for any ψ, both lattice modes,
    with or without phason strain. ``θcor`` is the one exception: it offsets the
    exit polar angle *after* the azimuth was solved at the uncorrected θ, which
    shears the curve slightly off-plane (first order in θcor, ~5e-4 Å⁻¹ at
    θcor = 1°). ``resid`` is returned so the caller can report that, not hide it.

    Done as one batched pass rather than a fit per curve: there are ~500 runs in
    a typical scene and the per-call numpy overhead of a scalar fit dominates
    everything else in the response. Every quantity below is a ``reduceat`` over
    the concatenated runs, then a batched 3x3 ``eigh``/``solve``.

    Points must be in sweep order: the arc runs a0 → a1 the way they progress,
    which is what makes the span unambiguous for an arc of any length.
    """
    lens = np.array([len(r) for r in runs])
    offs = np.concatenate([[0], np.cumsum(lens)[:-1]])
    rid  = np.repeat(np.arange(len(runs)), lens)     # run index per point
    P    = Q[np.concatenate(runs)]

    c0 = np.add.reduceat(P, offs, axis=0) / lens[:, None]
    # Scatter matrix per run: Σ p⊗p − n·c⊗c. Its least eigenvector is the plane.
    S = (np.add.reduceat(P[:, :, None] * P[:, None, :], offs, axis=0)
         - lens[:, None, None] * c0[:, :, None] * c0[:, None, :])
    n = np.linalg.eigh(S)[1][:, :, 0]

    u = np.stack([np.zeros(len(n)), n[:, 2], -n[:, 1]], axis=1)   # == _plane_basis,
    small = (u * u).sum(1) < 1e-18                                # batched
    if small.any():
        u[small] = np.stack([-n[small, 2], np.zeros(small.sum()), n[small, 0]], axis=1)
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    v = np.cross(n, u)

    X = P - c0[rid]
    x = np.einsum('ij,ij->i', X, u[rid])
    y = np.einsum('ij,ij->i', X, v[rid])
    b = x * x + y * y
    red = lambda a: np.add.reduceat(a, offs)
    # 3x3 normal equations of |p|² = 2·cx·x + 2·cy·y + (r² − |c|²), per run.
    AtA = np.empty((len(runs), 3, 3))
    AtA[:, 0, 0] = red(4 * x * x); AtA[:, 0, 1] = AtA[:, 1, 0] = red(4 * x * y)
    AtA[:, 1, 1] = red(4 * y * y); AtA[:, 0, 2] = AtA[:, 2, 0] = red(2 * x)
    AtA[:, 1, 2] = AtA[:, 2, 1] = red(2 * y); AtA[:, 2, 2] = lens
    Atb = np.stack([red(2 * x * b), red(2 * y * b), red(b)], axis=1)
    # b as a stack of column vectors: numpy >= 2 reads a bare 2-D b as one matrix.
    sol = np.linalg.solve(AtA, Atb[:, :, None])[:, :, 0]
    cx, cy = sol[:, 0], sol[:, 1]
    r = np.sqrt(np.maximum(sol[:, 2] + cx * cx + cy * cy, 0.0))

    dx, dy = x - cx[rid], y - cy[rid]
    resid = np.maximum(
        np.maximum.reduceat(np.abs(np.sqrt(dx * dx + dy * dy) - r[rid]), offs),
        np.maximum.reduceat(np.abs(np.einsum('ij,ij->i', X, n[rid])), offs))

    # Unwrap the angle within each run (not across run boundaries), so a0 → a1
    # spans the arc the way the sweep traversed it, for arcs of any length.
    ang = np.arctan2(dy, dx)
    d = np.diff(ang, prepend=ang[0])
    d = (d + np.pi) % (2 * np.pi) - np.pi
    d[offs] = 0.0                                    # no carry into a new run
    cum = np.cumsum(d)
    a0 = ang[offs]
    a1 = a0 + (cum[offs + lens - 1] - cum[offs])
    return c0 + cx[:, None] * u + cy[:, None] * v, r, n, a0, a1, resid


def _subsample(seg, max_pts):
    if len(seg) <= max_pts:
        return seg
    return seg[np.linspace(0, len(seg) - 1, max_pts).astype(int)]


def _rz(deg):
    """Rotation about z by `deg` degrees (row-vector convention: v @ _rz(a).T)."""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def compute_dms(req):
    """Compute DMS reciprocal-space curves. `req` is the parsed request dict.

    Returns a JSON-serialisable dict (see server.py / README for the schema).
    """
    ts = _import_ts()

    mode      = req.get("mode", "crystal")
    energy    = float(req.get("energy", 15.0))
    azir      = list(req.get("azir", [1, -1, 0]))
    # θ-range is *relative to the primary's Bragg angle* — the same convention as
    # the DMS project's `thrange_delta` (slider.py: thrange = [thb-27, thb+10]).
    # pilkhlrange takes absolute angles, so θ_B is added below once it is known.
    thrange   = list(req.get("thrange", [-27, 10]))
    numsteps  = int(req.get("numsteps", 160))
    psi       = float(req.get("psi", -180.0))
    # Azimuthal correction (slot 6 of the DMS 24-element guess vector). The
    # engine offsets every outgoing beam by it, so it is part of the geometry a
    # fitted state carries and must be un-done alongside ψ (see `to_crystal`).
    psicor    = float(req.get("psicor", 0.0))
    # χ / θ corrections (slots 7 / 8 of the guess vector). χcor rotates the whole
    # scan-hkl list about the chi axis (⊥ to the primary and its azimuthal
    # reference), which changes *which* secondaries diffract across the sweep, so
    # a fitted χcor of a few thousandths of a degree still moves the Kossel lines
    # visibly; θcor offsets every outgoing Bragg angle. Both are applied by the
    # slider (dmsfit_ico_hkl slots 7/8) — dropping them makes the viewer's
    # pattern disagree with the slider's for the same JSON.
    chicor    = float(req.get("chicor", 0.0))
    thetacor  = float(req.get("thetacor", 0.0))
    max_curves = int(req.get("maxCurves", 400))
    max_pts    = int(req.get("maxPtsPerCurve", 80))
    # How each DMS locus is delivered. Both methods run the same θ-scan at
    # `numsteps`; only the representation differs:
    #   "sweep"  — the sampled points, as polylines (original).
    #   "circle" — each run reduced to the circle it lies on, for the client to
    #              tessellate at display resolution. Identical geometry, ~10x
    #              less payload, and `numsteps` then only sets how precisely the
    #              *ends* of each arc are located, not the shape of the curve.
    method = "circle" if str(req.get("method", "sweep")).lower() == "circle" else "sweep"

    det = req.get("detector", {})
    detdist = float(det.get("detdist", 3000.0))
    rotx = float(det.get("rotx", 0.0)); roty = float(det.get("roty", 0.0)); rotz = float(det.get("rotz", 0.0))
    px = float(det.get("px", 600)); py = float(det.get("py", 600))
    scatv = int(det.get("scatv", 0))
    imshape = tuple(det.get("imshape", [1200, 1200]))
    detvects = np.matrix(det.get("detvects", [[1, 0, 0], [0, 0, 1]]))

    # ── Lattice, primary, secondaries ──────────────────────────────────────
    if mode == "ico":
        aq = float(req.get("aq", 6.461))
        lattice = [aq, aq, aq, 90.0, 90.0, 90.0]
        system = None
        ref_6d = np.array(req.get("ref_6d") or [], dtype=float)
        if ref_6d.ndim != 2 or ref_6d.shape[0] == 0 or ref_6d.shape[1] != 6:
            raise ValueError("quasicrystal DMS needs a non-empty ref_6d: a list of "
                             "6D secondary indices [n0..n5]")
        ref_6d = ref_6d[np.any(ref_6d != 0, axis=1)]          # drop the origin
        if ref_6d.shape[0] == 0:
            raise ValueError("ref_6d is empty once the origin is removed")
        proj = ts.Projection6d(ref_6d)
        reflist, reflist2 = proj.reflection_6d()      # parallel, perpendicular
        reflist = np.asarray(reflist); reflist2 = np.asarray(reflist2)
        # Phason *strain* matrix: the engine forms q∥ + P·q⊥, so an unstrained
        # quasicrystal is P = 0 — not the identity, which would add the whole
        # perpendicular component to every secondary reflection.
        mtrx2 = list(req.get("mtrx2", [0.0] * 9))
        primary = list(req.get("primaryHkl") or reflist[0].tolist())
    else:
        lattice = list(req.get("lattice", [5.431, 5.431, 5.431, 90, 90, 90]))
        system = req.get("system")            # conventional lattice constraint (or None)
        primary = list(req.get("primaryHkl", [1, 1, 1]))
        # An explicit secondary list (e.g. loaded from a DMS reflection-list JSON)
        # takes the place of the full hkl box.
        given = req.get("reflist")
        if given:
            reflist = np.asarray(given, dtype=float)
            if reflist.ndim != 2 or reflist.shape[1] != 3:
                raise ValueError("reflist must be a list of [h,k,l] triples")
            reflist = reflist[np.any(reflist != 0, axis=1)]   # drop the origin
            if reflist.shape[0] == 0:
                raise ValueError("reflist is empty once the origin is removed")
        else:
            depth = int(req.get("secondaryDepth", 3))
            reflist = ts.hklgen_3d(depth).astype(float)
        reflist2 = np.zeros_like(reflist)
        mtrx2 = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    hklint = np.round(np.array(primary)).astype(float)
    psirange = [psi - 180, psi + 180]

    # ── calcms alignment + crystal-frame transform ────────────────────────
    # calcms works in an aligned frame (primary ∥ z). To map DMS features into
    # the true crystal reciprocal frame — the frame a diffraction spot maps into,
    # where the main reflection is pinned at its hkl (B·hkl) independent of the
    # azimuth ψ — we undo the crystal orientation:  O⁻¹ = de-azimuth(−ψ) then
    # un-align (Rᵀ). Validated: |k_in|=k₀ and |B·primary + k_in|=k₀ for all ψ.
    ms = ts.calcms(lattice, primary, hklint, reflist, energy, azir)
    rmat = np.asarray(getattr(ms, "rmatrix", np.eye(3)), dtype=float)  # crystal·R = aligned
    thb = float(ms.bragg()[0][0])
    ko = energy / 12.398

    # The engine's constant azimuth for the outgoing beams is (ψ − psicor); it is
    # that combination, not ψ alone, that has to be undone.
    psi_eff = psi - psicor

    def to_crystal(v, deazimuth=True):
        """Aligned-frame row vector(s) → crystal (B-matrix) frame."""
        v = np.asarray(v, dtype=float)
        if deazimuth:
            v = v @ _rz(-psi_eff).T
        return v @ rmat.T

    # Lattice & primary are crystal-native (no ψ) → un-align only → B·hkl.
    latt = to_crystal(np.asarray(ms.tv()), deazimuth=False)      # == reflist·Bᵀ (Phase-1 positions)
    primary_q = to_crystal(np.asarray(ms.tvp()).ravel(), deazimuth=False).tolist()
    kin = np.asarray(ms.orig(), dtype=float).ravel()             # aligned incident (== ms.kov())

    # ── DMS engine: beam vectors + detector pixels ─────────────────────────
    thscan = [thb + thrange[0], thb + thrange[1]]
    hkllist = ts.pilkhlrange(lattice, primary, energy, thscan[0], thscan[1]).hklscan(numsteps)
    dc = ts.dmscalc_ico(reflist, hkllist, hklint, 1, psirange, 0, primary,
                        detvects, np.zeros(imshape), 0, azir, psi, px, py, scatv,
                        detdist, rotx, roty, rotz, energy, reflist2, mtrx2)
    dc.crystal_system = system
    # inputs: lattice[0:6], psicor, θcor, χcor, detdist, rotx/y/z, energy, phason
    # (dmscalc_ico.imcalc reads inputs[7]=thetacorrection, inputs[8]=chicorrection)
    dc.imcalc([*lattice, psicor, thetacor, chicor, detdist, rotx, roty, rotz, energy, *mtrx2])

    vecs = np.asarray(dc.vecs, dtype=float)          # (M, 3) beam unit vectors
    N_refs = reflist.shape[0]
    N_steps = np.asarray(hkllist).shape[0]
    half = N_steps * N_refs + 1
    # reflection index per row (sentinel at 0), duplicated for the psi1/psi2 halves
    ridx = np.full(half, -1, dtype=int)
    ridx[1:] = np.tile(np.arange(N_refs), N_steps)
    ridx_all = np.concatenate([ridx, ridx])
    finite = np.isfinite(vecs).all(axis=1)
    # Row bookkeeping so each curve can be traced along the sweep instead of
    # being sorted (sorting welds the two branches into one bogus polyline).
    rows = np.arange(vecs.shape[0])
    branch = (rows >= half).astype(int)      # psi1 / psi2 solution branch
    step_idx = (rows % half - 1) // N_refs   # sweep step within the branch

    # DMS beam vectors → Q = k_out − k_in in the aligned frame → crystal frame.
    # k_out = ko·Yflip(vecs) (psith2v beam frame → aligned); k_in = ms.orig().
    Q = to_crystal(ko * _yflip(vecs) - kin)           # de-azimuth(−ψ) + un-align

    # detector pixels per reflection (already on-detector), from the engine
    pxv2d = getattr(dc, "pxv2d_all", np.empty((0, 2)))
    pxrefidx = getattr(dc, "pxv2d_refidx", np.empty(0, dtype=int))

    # ── Assemble per-reflection curves ─────────────────────────────────────
    reflections = []
    worst_resid = 0.0
    # Group the rows by reflection once. Selecting them with `ridx_all == j` per
    # reflection (and again to rank them) walks the whole 2·N_steps·N_refs array
    # N_refs times — O(N_refs²·N_steps), which is the dominant cost well before
    # the secondary list gets large. One stable sort gives the same groups, and
    # the same `order`, in a single pass.
    keep = np.flatnonzero(finite & (ridx_all >= 0))
    counts = np.bincount(ridx_all[keep], minlength=N_refs)
    order = np.argsort(-counts)
    grouped = keep[np.argsort(ridx_all[keep], kind="stable")]   # sweep order kept
    starts = np.concatenate([[0], np.cumsum(counts)])
    def _runs_of(j):
        """The continuous runs of reflection `j`, as row indices, branch by branch."""
        jrows = grouped[starts[j]:starts[j + 1]]
        jbranch = branch[jrows]
        out = []
        for b in (0, 1):
            rb = jrows[jbranch == b]
            if rb.size:
                out.extend(_split_run_rows(Q[rb], step_idx[rb], rb))
        return out

    # Circle method: split every reflection first, fit all the arcs in one
    # batched pass, then hand each reflection its own. Fitting curve-by-curve
    # costs more in numpy call overhead than everything else in the response.
    arcs_by_ref = {}
    if method == "circle":
        run_rows, run_owner = [], []
        seen = 0
        for j in order:
            if counts[j] < 6:
                continue
            rs = [r for r in _runs_of(j) if len(r) >= 3]   # a circle needs three points
            if not rs:
                continue
            run_rows.extend(rs)
            run_owner.extend([j] * len(rs))
            seen += 1
            if seen >= max_curves:
                break
        if run_rows:
            cs, rs_, ns, a0s, a1s, res = _fit_arcs(Q, run_rows)
            ok = np.isfinite(rs_) & (rs_ > 0)
            worst_resid = float(res[ok].max()) if ok.any() else 0.0
            cs = np.round(cs, 5); ns = np.round(ns, 5)
            for i, j in enumerate(run_owner):
                if not ok[i]:
                    continue
                # `u` is not sent: the client rederives it from `n` with the same
                # rule (_plane_basis / planeBasis), which is what makes a0/a1
                # mean the same thing at both ends.
                arcs_by_ref.setdefault(j, []).append({
                    "c": cs[i].tolist(), "r": round(float(rs_[i]), 5),
                    "n": ns[i].tolist(),
                    "a0": round(float(a0s[i]), 5), "a1": round(float(a1s[i]), 5),
                })

    for j in order:
        if counts[j] < 6:
            continue
        if method == "circle":
            arcs = arcs_by_ref.get(j)
            if not arcs:
                continue
            entry = {"hkl": [int(round(x)) for x in reflist[j][:3]], "circles": arcs}
        else:
            # One polyline per (branch, continuous run) — rows are already in
            # sweep order, so no arc ends get joined together.
            qsegs = [np.round(_subsample(Q[r], max_pts), 4).tolist()
                     for r in _runs_of(j) if len(r) >= 2]
            if sum(len(s) for s in qsegs) < 6:
                continue
            entry = {"hkl": [int(round(x)) for x in reflist[j][:3]], "qcurves": qsegs}
        # detector polylines for this reflection (normalised to [-0.5, 0.5]).
        # pxv2d keeps the engine's branch-major, step-major order, so the same
        # gap test separates the arcs — only the step index is unavailable.
        dm = pxrefidx == j
        if dm.any():
            dpx = np.asarray(pxv2d)[dm].astype(float)
            duv = np.stack([dpx[:, 0] / imshape[0] - 0.5,
                            dpx[:, 1] / imshape[1] - 0.5], axis=1)
            dsegs = [np.round(_subsample(s, max_pts), 4).tolist()
                     for s in _split_polyline(duv)]
            if dsegs:
                entry["dets"] = dsegs
        reflections.append(entry)
        if len(reflections) >= max_curves:
            break

    # ── The real detector, in the crystal frame ────────────────────────────
    # Inverted straight out of the engine's own pixel map rather than re-derived,
    # so the viewer's panel is the same rectangle the DMS fit was made against.
    # imcalc does, for a scattered ray hitting the plane at `hit`:
    #     prepx = −(hit − centralv);  pv = prepx · irmat
    #     row = px + pv[0];  col = py − pv[2]
    # `irmat` takes the detector plane onto x–z, so pv[1] = 0 for any in-plane
    # point and the map inverts exactly:
    #     pv = (row − px, 0, py − col);  hit = centralv − pv · irmatᵀ
    #
    # The final **negation** places the panel on the physically-correct side of
    # the sample. The engine's `dms2px` measures its pixel offset from a beam
    # line anchored at `centralv` against a plane through the origin, which makes
    # the recovered offset a *point reflection* of where a ray actually travels:
    # a scattered ray leaving the sample (origin) along k_out = k_in + Q lands on
    # the reflection-through-origin of the engine's plane point. Without the
    # negation the panel sat on the +k_in side (2θ ≈ 0), where those forward rays
    # reach it only by extending *backwards* through the sample — so the DMS
    # curves and the detector ended up on opposite sides of the Ewald sphere.
    # Negated, the panel is on the −k_in side, where the diffracted Kossel beams
    # genuinely go: a forward ray from the sample along each curve's k_out lands
    # on it, reproducing the slider's detector pattern pixel-for-pixel.
    irmat = np.asarray(ts.rotxyz([1, 0, 0], rotx + thb).rmat()
                       @ ts.rotxyz([0, 1, 0], roty).rmat()
                       @ ts.rotxyz([0, 0, 1], rotz).rmat(), dtype=float)
    centralv = -np.asarray(ts.psith2v(0, thb), dtype=float).ravel() * detdist

    def _pixel_dir(row, col):
        """Image pixel → direction from the sample, in units of the detector
        distance (so the client scales by whatever display distance it likes).
        Negated so a forward scattered ray (k_out = k_in + Q) reaches it — see
        the note above."""
        pv = np.array([row - px, 0.0, py - col])
        hit = centralv - pv @ irmat.T
        return -to_crystal(_yflip(hit)) / detdist

    nrow, ncol = float(imshape[0]), float(imshape[1])
    corners = [_pixel_dir(0, 0), _pixel_dir(nrow, 0),
               _pixel_dir(nrow, ncol), _pixel_dir(0, ncol)]
    beam_centre = _pixel_dir(px, py)                    # panel centre (px,py pixel)
    u_axis = _pixel_dir(px + 1, py) - beam_centre       # +row, unit-ish (1/detdist long)
    v_axis = _pixel_dir(px, py + 1) - beam_centre       # +col
    unit = lambda a: (a / np.linalg.norm(a)) if np.linalg.norm(a) > 1e-12 else a

    # Half-angle to the nearest image edge, about the detector centre. If the DMS
    # cones all fall outside it the plane renders bare — report it rather than
    # leaving an empty detector looking like a failed computation.
    accept = float(np.degrees(np.arctan(
        min(px, py, nrow - px, ncol - py) / detdist))) if detdist > 0 else 0.0
    detector = {
        # Everything below is in *detector distances*: multiply by the display
        # distance to place it. The quad is the image rectangle, in the fitted
        # orientation, with the beam centre wherever px/py actually put it.
        "corners": [np.round(c, 6).tolist() for c in corners],
        "beamCentre": np.round(beam_centre, 6).tolist(),
        "u": np.round(unit(u_axis), 6).tolist(),
        "v": np.round(unit(v_axis), 6).tolist(),
        "sizePx": [nrow, ncol],
        "beamCentrePx": [px, py],
        "detdistPx": detdist,
        "rotDeg": [rotx, roty, rotz],
        "twoThetaDeg": round(float(np.degrees(np.arccos(np.clip(
            unit(beam_centre) @ unit(to_crystal(kin)), -1, 1)))), 4),
        "acceptanceDeg": round(accept, 2),
        "nCurvesOnDetector": sum(1 for r in reflections if r.get("dets")),
    }

    return {
        "mode": mode,
        "method": method,
        # Worst deviation of any run from the circle fitted to it (Å⁻¹). Machine
        # precision unless θcor is non-zero — see `_fit_arc`. Reported so the
        # viewer can say when the analytic form has stopped being exact.
        "circleResidual": (round(worst_resid, 9) if method == "circle" else None),
        "ko": round(ko, 5),
        "braggPrimary": round(thb, 4),
        "thetaScan": [round(thscan[0], 4), round(thscan[1], 4)],   # absolute θ swept
        "frame": "crystal (B-matrix)",
        "primary": {"hkl": [float(x) for x in primary], "q": [round(float(v), 4) for v in primary_q]},
        "kin": to_crystal(kin).round(4).tolist(),
        "lattice": [
            {"hkl": [int(round(x)) for x in reflist[j][:3]], "q": [round(float(v), 4) for v in latt[j]]}
            for j in range(N_refs)
        ],
        "reflections": reflections,
        "detector": detector,
    }
