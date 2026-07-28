// ============================================================
// js/crystal.js — conventional-crystal reciprocal lattice
// ------------------------------------------------------------
// Direct JavaScript port of ts_quasi.bmatrix (DMS project,
// DMSAnalysis/ts_quasi.py:317). Builds the reciprocal-space
// B-matrix from the six lattice parameters, generates every
// hkl in a box, and applies the Bravais systematic-absence
// rule for the chosen centring.
//
//   Q(hkl) = B · [h k l]ᵀ      (Cartesian, units of Å⁻¹, 1/d convention)
//   |Q| = 1/d(hkl)
// ============================================================

const DEG = Math.PI / 180;

// ── B-matrix (reciprocal basis as columns), Å⁻¹ ─────────────
// Reproduces ts_quasi.bmatrix exactly:
//   beta* are the reciprocal-cell angles, b1/b2/b3 the reciprocal
//   axis lengths, and B is the upper-triangular Busing–Levy matrix.
export function bmatrix(lattice) {
  const [a, b, c, al, be, ga] = lattice;
  const a1 = al * DEG, a2 = be * DEG, a3 = ga * DEG;

  const beta1 = Math.acos((Math.cos(a2) * Math.cos(a3) - Math.cos(a1)) /
                          (Math.sin(a2) * Math.sin(a3)));
  const beta2 = Math.acos((Math.cos(a1) * Math.cos(a3) - Math.cos(a2)) /
                          (Math.sin(a1) * Math.sin(a3)));
  const beta3 = Math.acos((Math.cos(a1) * Math.cos(a2) - Math.cos(a3)) /
                          (Math.sin(a1) * Math.sin(a2)));

  const b1 = 1.0 / (a * Math.sin(a2) * Math.sin(beta3));
  const b2 = 1.0 / (b * Math.sin(a3) * Math.sin(beta1));
  const b3 = 1.0 / (c * Math.sin(a1) * Math.sin(beta2));

  // Column-major reciprocal basis vectors b1*, b2*, b3* (each a 3-vector).
  return [
    [b1, 0, 0],
    [b2 * Math.cos(beta3), b2 * Math.sin(beta3), 0],
    [b3 * Math.cos(beta2), -b3 * Math.sin(beta2) * Math.cos(a1), 1.0 / c],
  ];
}

// Q vector for a single reflection given the reciprocal basis columns.
export function qOf(basis, h, k, l) {
  const [B0, B1, B2] = basis;
  return [
    h * B0[0] + k * B1[0] + l * B2[0],
    h * B0[1] + k * B1[1] + l * B2[1],
    h * B0[2] + k * B1[2] + l * B2[2],
  ];
}

// ── Bravais systematic-absence rules ────────────────────────
// Returns true if the reflection is ALLOWED (present) for the centring.
const ABSENCE = {
  P: () => true,
  I: (h, k, l) => (h + k + l) % 2 === 0,
  F: (h, k, l) => (((h + k) % 2 === 0) && ((k + l) % 2 === 0)), // all same parity
  A: (h, k, l) => (k + l) % 2 === 0,
  B: (h, k, l) => (h + l) % 2 === 0,
  C: (h, k, l) => (h + k) % 2 === 0,
  R: (h, k, l) => mod(-h + k + l, 3) === 0, // obverse rhombohedral setting
};

function mod(n, m) { return ((n % m) + m) % m; }

// ── Generate the reciprocal lattice ─────────────────────────
// Returns { positions:Float32Array(n*3), qmag:Float32Array,
//           intensity:Float32Array, meta:[{hkl,q}] }.
// Intensity is a geometric proxy (a smooth |Q| falloff, standing in
// for a real form-factor/structure-factor until the CIF phase); every
// allowed reflection is present.
export function generateCrystal(params) {
  const basis = bmatrix(params.lattice);
  const N = Math.max(1, params.hklRange | 0);
  const allowed = ABSENCE[params.centring] || ABSENCE.P;
  const qMax = params.qMax;

  const pos = [], qmag = [], inten = [], meta = [];
  for (let h = -N; h <= N; h++) {
    for (let k = -N; k <= N; k++) {
      for (let l = -N; l <= N; l++) {
        if (h === 0 && k === 0 && l === 0) continue;
        if (!allowed(h, k, l)) continue;
        const q = qOf(basis, h, k, l);
        const qm = Math.hypot(q[0], q[1], q[2]);
        if (qm > qMax) continue;
        // Geometric intensity proxy: exp(-(|Q|/qMax)^2) — bright near the
        // origin, fading out to the resolution edge. Replaced by true
        // structure factors in the CIF enhancement.
        const I = Math.exp(-Math.pow(qm / (0.6 * qMax), 2));
        pos.push(q[0], q[1], q[2]);
        qmag.push(qm);
        inten.push(I);
        meta.push({ hkl: [h, k, l], q: qm });
      }
    }
  }
  return {
    positions: new Float32Array(pos),
    qmag: new Float32Array(qmag),
    intensity: new Float32Array(inten),
    basis,
    meta,
  };
}
