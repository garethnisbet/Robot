// ============================================================
// js/quasicrystal.js — icosahedral quasicrystal reciprocal space
// ------------------------------------------------------------
// JavaScript port of ts_quasi.Projection6d (DMS project,
// DMSAnalysis/ts_quasi.py:1184) and its supporting `mmm`/`m6d`
// matrices. See 6D_projection_matrix_derivation.md in the DMS
// project for the full derivation.
//
// Every icosahedral Bragg peak is a rank-6 integer index
// n = (n0..n5). The 6x6 orthonormal matrix m6d rotates R^6 into
// E∥ ⊕ E⊥ (physical ⊕ perpendicular space):
//
//   Q6 = m6d · (mmm · n)
//   Q_par  = Q6[0:3]   physical reciprocal vector  → where the peak sits
//   Q_perp = Q6[3:6]   perpendicular (phason) part → sets the intensity
//
// Peaks with large |Q_perp| are weak: I ∝ exp(-(|Q_perp|/σ)²), the
// standard cut-and-project perpendicular-space window.
// ============================================================

const TAU = 0.5 + 0.5 * Math.sqrt(5.0);          // golden ratio
const CONST = 1.0 / Math.sqrt(2.0 * (2.0 + TAU)); // orthonormalisation

// mmm: relabels Elser 6D indices to Cahn's convention (signed permutation).
const MMM = [
  [1, 0, 0, 0, 0, 0],
  [0, 1, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 1],
  [0, 0, 0, 0, 1, 0],
  [0, 0, 1, 0, 0, 0],
  [0, 0, 0, -1, 0, 0],
];

// m6d: projection onto (parallel | perpendicular) space, pre-scaled by CONST.
const M6D = [
  [1, TAU, 0, -1, TAU, 0],
  [TAU, 0, 1, TAU, 0, -1],
  [0, 1, TAU, 0, -1, TAU],
  [-TAU, 1, 0, TAU, 1, 0],
  [1, 0, -TAU, 1, 0, TAU],
  [0, -TAU, 1, 0, TAU, 1],
].map((row) => row.map((v) => v * CONST));

// (m6d · mmm) folded into one 6x6 matrix so projection is a single mat-vec.
const M = M6D.map((row) => {
  const out = new Array(6).fill(0);
  for (let j = 0; j < 6; j++)
    for (let k = 0; k < 6; k++) out[j] += row[k] * MMM[k][j];
  return out;
});

// Project one 6D integer index → { par:[x,y,z], perp:[x,y,z] } (r.l.u).
export function project6d(n) {
  const q = new Array(6).fill(0);
  for (let i = 0; i < 6; i++)
    for (let j = 0; j < 6; j++) q[i] += M[i][j] * n[j];
  return { par: [q[0], q[1], q[2]], perp: [q[3], q[4], q[5]] };
}

// ── Generate the icosahedral reciprocal-space peak set ──────
// Same return shape as generateCrystal() in crystal.js.
export function generateQuasicrystal(params) {
  const N = Math.max(1, params.sixDRange | 0);
  const sigma = Math.max(1e-3, params.sigmaPerp);
  const iThresh = params.iThreshold;
  const qMax = params.qMax;
  // Physical scale: convert r.l.u. parallel components to Å⁻¹ via the
  // 6D hypercubic edge. (2π/aq gives a conventional Q; the exact factor
  // only sets the display scale, which qMax then bounds.)
  const scale = (2.0 * Math.PI) / params.aq;

  const pos = [], qmag = [], inten = [], meta = [];
  for (let a = -N; a <= N; a++)
  for (let b = -N; b <= N; b++)
  for (let c = -N; c <= N; c++)
  for (let d = -N; d <= N; d++)
  for (let e = -N; e <= N; e++)
  for (let f = -N; f <= N; f++) {
    if (!(a || b || c || d || e || f)) continue;
    const { par, perp } = project6d([a, b, c, d, e, f]);
    const qm = Math.hypot(par[0], par[1], par[2]) * scale;
    if (qm > qMax || qm < 1e-6) continue;
    const qp = Math.hypot(perp[0], perp[1], perp[2]);
    const I = Math.exp(-Math.pow(qp / sigma, 2));
    if (I < iThresh) continue;
    pos.push(par[0] * scale, par[1] * scale, par[2] * scale);
    qmag.push(qm);
    inten.push(I);
    meta.push({ n: [a, b, c, d, e, f], q: qm, qperp: qp });
  }
  return {
    positions: new Float32Array(pos),
    qmag: new Float32Array(qmag),
    intensity: new Float32Array(inten),
    basis: null,
    meta,
  };
}

// ── 6D reflections for the DMS secondary list ───────────────
// The strongest icosahedral reflections as rank-6 integer indices, from the same
// 6D enumeration and perpendicular-space intensity window as the lattice, so the
// DMS secondary set tracks the icosahedral sliders (6D range, σ⊥, I threshold).
// Returns the `maxRefs` strongest as ``[[n0..n5], ...]`` for ts.Projection6d.
export function icoDmsReflections(params, maxRefs = 120) {
  const N = Math.max(1, params.sixDRange | 0);
  const sigma = Math.max(1e-3, params.sigmaPerp);
  const iThresh = params.iThreshold;
  const out = [];
  for (let a = -N; a <= N; a++)
  for (let b = -N; b <= N; b++)
  for (let c = -N; c <= N; c++)
  for (let d = -N; d <= N; d++)
  for (let e = -N; e <= N; e++)
  for (let f = -N; f <= N; f++) {
    if (!(a || b || c || d || e || f)) continue;
    const { par, perp } = project6d([a, b, c, d, e, f]);
    if (Math.hypot(par[0], par[1], par[2]) < 1e-6) continue;
    const qp = Math.hypot(perp[0], perp[1], perp[2]);
    const I = Math.exp(-Math.pow(qp / sigma, 2));
    if (I < iThresh) continue;
    out.push({ n: [a, b, c, d, e, f], I });
  }
  out.sort((x, y) => y.I - x.I);
  return out.slice(0, maxRefs).map((r) => r.n);
}
