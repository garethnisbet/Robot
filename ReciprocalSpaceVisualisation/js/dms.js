// ============================================================
// js/dms.js — DMS (multiple-scattering) curves in reciprocal space
// ------------------------------------------------------------
// Fetches /api/dms (which reuses the DMS project's ts_quasi server-side) and
// renders, in the true crystal (B-matrix) reciprocal frame:
//   • the reciprocal lattice points in play (at their true B·hkl positions)
//   • one Q-space curve per secondary reflection — the "obstacles" (every point
//     on a curve shares the same secondary hkl)
//   • the Ewald sphere the curves lie on, and the incident beam
//
// The curves are handed to js/detector.js (getCurves) so it can project them
// onto the draggable panel; the incident wavevector is handed to js/optics.js
// (setBeam) so the panel/blob mapping shares the DMS geometry. No detector plane
// is drawn here any more — that is the real, draggable one.
// ============================================================
import * as THREE from 'three';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import * as State from './state.js';
import { scene } from './scene.js';
import * as Optics from './optics.js';
import { icoDmsReflections } from './quasicrystal.js';
import { activeRefs } from './reflist.js';

const group = new THREE.Group();
scene.add(group);

// ψ trail: the curves left behind at previous ψ. Its own group, because `group`
// is emptied on every render — see clear()/render().
const trailGroup = new THREE.Group();
scene.add(trailGroup);
let trail = [];            // [{psi, obj}] oldest first

// Arc tessellation (method: 'circle'). Each DMS locus is exactly a circle in Q,
// so the server sends {c, r, n, u, a0, a1} and the chord length is ours to pick:
// aim for ARC_CHORD Å⁻¹ per segment, bounded so a full-circle arc stays cheap.
const ARC_CHORD = 0.006;      // Å⁻¹ — well under the line width drawn on the panel
const ARC_MIN_PTS = 12, ARC_MAX_PTS = 512;

let lastResult = null;
let curves = [];           // [{hkl, pts:[[x,y,z]...]}] the Q-space obstacle curves
let dets = [];             // [{hkl, uv:[[du,dv]...]}] the engine's own detector
                           // pixel projection (normalised to [-0.5,0.5]) — the
                           // pattern the DMS slider draws on its detector image.
let primaryQ = null;       // the primary reflection's Q (on the Ewald sphere)
let detectorGeom = null;   // the engine's detector rectangle, for the panel
let inFlight = false, pending = null;   // coalesce ψ-slider driven recomputes

// Deterministic hue from an hkl triple (matches detector.js).
function hueColor(hkl) {
  const s = (hkl[0] * 73856093) ^ (hkl[1] * 19349663) ^ (hkl[2] * 83492791);
  return new THREE.Color().setHSL((((s % 360) + 360) % 360) / 360, 0.65, 0.6);
}

// The in-plane axis the server measured a0/a1 against. It is derived, not sent,
// so this rule must stay identical to `_plane_basis` in dms_compute.py:
// n × x̂, falling back to n × ŷ when n lies along x̂.
function planeBasis(nx, ny, nz) {
  let ux = 0, uy = nz, uz = -ny;
  if (ux * ux + uy * uy + uz * uz < 1e-18) { ux = -nz; uy = 0; uz = nx; }
  const L = Math.hypot(ux, uy, uz);
  return [ux / L, uy / L, uz / L];
}

// One analytic arc → points, at whatever resolution the display wants:
//   P(t) = c + r·(cos t · u + sin t · v),   v = n × u,   t from a0 to a1.
function arcPoints(a) {
  const [cx, cy, cz] = a.c, [nx, ny, nz] = a.n;
  const [ux, uy, uz] = planeBasis(nx, ny, nz);
  const vx = ny * uz - nz * uy, vy = nz * ux - nx * uz, vz = nx * uy - ny * ux;
  const span = a.a1 - a.a0;
  const n = Math.max(ARC_MIN_PTS,
    Math.min(ARC_MAX_PTS, Math.ceil(Math.abs(span) * a.r / ARC_CHORD)));
  const pts = new Array(n + 1);
  for (let i = 0; i <= n; i++) {
    const t = a.a0 + span * (i / n), C = Math.cos(t) * a.r, S = Math.sin(t) * a.r;
    pts[i] = [cx + C * ux + S * vx, cy + C * uy + S * vy, cz + C * uz + S * vz];
  }
  return pts;
}

// ── ψ trail ─────────────────────────────────────────────────
// Sweeping ψ turns the whole curve network about Q, so keeping each slice draws
// out the surface the DMS cones sweep through reciprocal space.
//
// A slice is one merged THREE.LineSegments — every curve of that ψ in a single
// geometry with per-vertex colour — not one Line per curve. A scene can hold
// ~500 curves, so a per-curve object would cost hundreds of draw calls *per
// slice*; merged, a slice is one. That is what makes a long trail affordable.
function buildTrailSlice() {
  let nseg = 0;
  for (const c of curves) nseg += Math.max(0, c.pts.length - 1);
  if (!nseg) return null;
  const pos = new Float32Array(nseg * 6);
  const col = new Float32Array(nseg * 6);
  let o = 0;
  for (const c of curves) {
    const { r, g, b } = hueColor(c.hkl);
    for (let i = 0; i + 1 < c.pts.length; i++) {
      const p = c.pts[i], q = c.pts[i + 1];
      pos[o]     = p[0]; pos[o + 1] = p[1]; pos[o + 2] = p[2];
      pos[o + 3] = q[0]; pos[o + 4] = q[1]; pos[o + 5] = q[2];
      col[o]     = r; col[o + 1] = g; col[o + 2] = b;
      col[o + 3] = r; col[o + 4] = g; col[o + 5] = b;
      o += 6;
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  // depthWrite off so overlapping slices blend instead of occluding each other.
  return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({
    vertexColors: true, transparent: true, depthWrite: false, opacity: 1 }));
}

// Older slices fade, so the sweep direction reads at a glance and a long trail
// stays legible instead of turning into a solid shell.
function restyleTrail() {
  const base = State.params.dms.curveOpacity;
  const n = trail.length;
  trail.forEach((t, i) => {
    const age = n > 1 ? i / (n - 1) : 1;        // 0 = oldest, 1 = newest
    t.obj.material.opacity = base * (0.12 + 0.88 * age);
  });
}

function dropSlice(t) {
  trailGroup.remove(t.obj);
  t.obj.geometry.dispose();
  t.obj.material.dispose();
}

/** Record the current curves as a ψ slice (no-op if that ψ is already the newest). */
function pushTrail(psi) {
  if (trail.length && trail[trail.length - 1].psi === psi) return;
  const obj = buildTrailSlice();
  if (!obj) return;
  trailGroup.add(obj);
  trail.push({ psi, obj });
  const max = Math.max(1, State.params.dms.trailMax | 0);
  while (trail.length > max) dropSlice(trail.shift());
  restyleTrail();
}

/** Drop every recorded slice. */
export function clearTrail() {
  trail.forEach(dropSlice);
  trail = [];
  State.requestRender();
}

/** How many ψ slices the trail holds, the ψ range they span, and their size.
 *  `bytes` counts the position + colour buffers (3 floats each per vertex) —
 *  the trail is the one thing here that can grow without bound, so the panel
 *  reports it rather than letting a long sweep quietly eat the tab. */
export function trailInfo() {
  if (!trail.length) return { count: 0, vertices: 0, bytes: 0 };
  const ps = trail.map((t) => t.psi);
  let vertices = 0;
  for (const t of trail) vertices += t.obj.geometry.attributes.position.count;
  return {
    count: trail.length, from: Math.min(...ps), to: Math.max(...ps),
    vertices, bytes: vertices * 6 * 4,
  };
}

/** Re-apply the length limit, the fade and the trail's visibility. */
export function refreshTrail() {
  const max = Math.max(1, State.params.dms.trailMax | 0);
  while (trail.length > max) dropSlice(trail.shift());
  trailGroup.visible = State.params.dms.showCurves;
  restyleTrail();
  State.requestRender();
}

function clear() {
  for (let i = group.children.length - 1; i >= 0; i--) {
    const c = group.children[i];
    c.geometry?.dispose?.();
    c.material?.dispose?.();
    group.remove(c);
  }
}

// ── Build the /api/dms request from the current params ──────
function buildRequest() {
  const p = State.params;
  const d = p.dms;
  const ico = p.mode === 'ico';
  const req = {
    mode: ico ? 'ico' : 'crystal',
    energy: d.energy, azir: d.azir, thrange: d.thrange, numsteps: d.numsteps,
    psi: d.psi, psicor: d.psicor, chicor: d.chicor, thetacor: d.thetacor,
    maxCurves: d.maxCurves, primaryHkl: d.primaryHkl,
    method: d.method,
  };
  // The fitted detector, when a DMS state brought one — the response then carries
  // the real image rectangle for the panel (js/detector.js setFittedDetector).
  if (p.detector.fitted) req.detector = p.detector.fitted;
  // A reflection list loaded from a DMS JSON file (js/reflist.js) replaces the
  // generated secondary set — the 6D list for a quasicrystal, the hkl list for a
  // conventional crystal. Falls back to the generated set when none is active.
  if (ico) {
    req.aq = p.aq;
    // Derived from the icosahedral sliders (6D range, σ⊥, I threshold), so the
    // DMS pattern tracks them — not a fixed list.
    req.ref_6d = activeRefs(6) || icoDmsReflections(p);
    req.mtrx2 = d.mtrx2 || [1, 0, 0, 0, 1, 0, 0, 0, 1];
  } else {
    req.lattice = p.lattice;
    req.system = p.system;
    const loaded = activeRefs(3);
    if (loaded) req.reflist = loaded;
    else req.secondaryDepth = d.secondaryDepth;
  }
  return req;
}

export async function computeDMS(onStatus) {
  // Dragging the ψ slider fires this repeatedly; keep one request in flight and
  // remember only the latest, so the curves follow the slider without a backlog.
  if (inFlight) { pending = onStatus; return; }
  const req = buildRequest();
  onStatus?.('computing…');
  inFlight = true;
  let res;
  try {
    res = await fetch('/api/dms', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(req),
    });
  } catch (e) {
    onStatus?.('request failed'); inFlight = false;
    if (pending) { const cb = pending; pending = null; computeDMS(cb); }
    return;
  }
  if (!res.ok) {
    const err = await res.json().catch(() => ({}));
    onStatus?.('error: ' + (err.detail || err.error || res.status));
    inFlight = false;
    if (pending) { const cb = pending; pending = null; computeDMS(cb); }
    return;
  }
  const data = await res.json();
  lastResult = data;
  primaryQ = data.primary?.q || null;
  detectorGeom = data.detector?.corners ? data.detector : null;

  // Flatten the Q-space curves and the engine's detector polylines. Either
  // delivery shape lands in the same `curves` array, so nothing downstream —
  // the panel projection, the recovered points — needs to know which was used.
  curves = [];
  dets = [];
  for (const r of data.reflections) {
    const segs = r.circles ? r.circles.map(arcPoints) : (r.qcurves || []);
    for (const seg of segs) {
      if (seg.length >= 2) curves.push({ hkl: r.hkl, pts: seg });
    }
    // The engine's own pixel projection of this reflection's DMS line — the
    // pattern the slider paints on its detector image. detector.js draws these
    // straight onto the panel, so the panel reproduces the slider exactly (the
    // panel rectangle is inverted from this same map; see dms_compute).
    for (const seg of r.dets || []) {
      if (seg.length >= 2) dets.push({ hkl: r.hkl, uv: seg });
    }
  }

  // Point the shared beam along the DMS incident wavevector, so the panel/blob
  // mapping lands the recovered points back on these very curves.
  Optics.setBeam(State.params.dms.energy, data.kin);

  // Leave this ψ behind before the live curves move on. Recorded from the ψ the
  // request was built with, so a slice is always labelled by the ψ that produced
  // it even if the slider has already moved.
  if (State.params.dms.trail) pushTrail(req.psi);

  render(data);
  dispatchEvent(new CustomEvent('dmsUpdated'));
  // In circle mode say how far the circles actually are from the sampled loci —
  // exact (~1e-15) unless θcor is in play, which shears them slightly off-plane.
  const dev = (data.method === 'circle' && data.circleResidual > 1e-9)
    ? ` · circles ±${data.circleResidual.toExponential(1)} Å⁻¹` : '';
  const ti = trailInfo();
  const tr = ti.count
    ? ` · trail ${ti.count} slice${ti.count > 1 ? 's' : ''} (ψ ${ti.from.toFixed(0)}…${ti.to.toFixed(0)}°)`
    : '';
  onStatus?.(`${data.reflections.length} curves · ${data.lattice.length} reflections${dev}${tr}`);

  inFlight = false;
  if (pending) { const cb = pending; pending = null; computeDMS(cb); }
}

function render(data) {
  clear();
  const p = State.params;

  // The reciprocal lattice itself is already drawn by reflections.js in this
  // same B-matrix frame, so we don't repeat it here — only the DMS-specific
  // features: the primary marker, the obstacle curves, the Ewald sphere, beam.

  // ── Primary reflection marker + label ──
  const pq = data.primary.q;
  const pgeo = new THREE.BufferGeometry();
  pgeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pq), 3));
  group.add(new THREE.Points(pgeo, new THREE.PointsMaterial({
    color: 0xff4466, size: 0.16, sizeAttenuation: true })));
  const plab = document.createElement('div');
  plab.className = 'axis-label'; plab.style.color = '#ff8899';
  plab.textContent = `primary [${data.primary.hkl.map(v => (+v).toFixed(2)).join(' ')}]`;
  const plabo = new CSS2DObject(plab); plabo.position.set(pq[0], pq[1], pq[2]); group.add(plabo);

  // ── DMS Q-curves (the obstacles) ──
  // The ideal loci — drawn thin. Line *width* (σ) belongs to the recorded image
  // on the panel and its recovered Q, not to these; see js/detector.js.
  // The trail is the same curves at earlier ψ, so it follows the same toggle.
  trailGroup.visible = p.dms.showCurves;
  restyleTrail();

  if (p.dms.showCurves) {
    // Drawn from the flattened list, so both delivery methods render identically.
    for (const c of curves) {
      const pts = c.pts.map(q => new THREE.Vector3(q[0], q[1], q[2]));
      group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: hueColor(c.hkl),
          transparent: true, opacity: p.dms.curveOpacity })));
    }
  }

  // ── Ewald sphere the curves lie on (centre = −kin, radius ko) ──
  if (p.dms.showEwald) {
    const kin = data.kin;
    const sph = new THREE.Mesh(
      new THREE.SphereGeometry(data.ko, 48, 32),
      new THREE.MeshBasicMaterial({ color: 0x335577, wireframe: true,
        transparent: true, opacity: 0.12 }));
    sph.position.set(-kin[0], -kin[1], -kin[2]);
    group.add(sph);
  }

  // ── Incident beam (through the origin, along kin) ──
  const kin = data.kin;
  group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-kin[0], -kin[1], -kin[2]), new THREE.Vector3(0, 0, 0)]),
    new THREE.LineBasicMaterial({ color: 0xffcc44 })));

  State.requestRender();
}

/** The flattened Q-space DMS obstacle curves: [{hkl, pts}]. */
export function getCurves() { return curves; }
export function hasCurves() { return curves.length > 0; }

/** The engine's detector polylines for the panel: [{hkl, uv:[[du,dv]...]}], each
 *  point normalised to the image rectangle ([-0.5,0.5]). Drawing these on the
 *  panel reproduces the DMS slider's detector pattern. */
export function getDets() { return dets; }

/** The primary reflection's Q ([x,y,z], on the Ewald sphere) or null. */
export function getPrimaryQ() { return primaryQ; }

/** The engine's detector rectangle in the crystal frame — `corners`/`beamCentre`
 *  in units of the detector distance, plus `sizePx`, `twoThetaDeg`. Null until
 *  DMS has run. See dms_compute; the panel is placed from it. */
export function getDetectorGeom() { return detectorGeom; }

/** Re-render with the last result (respects the current show* toggles). */
export function refreshRender() { if (lastResult) render(lastResult); }

/** Clear the curves entirely (and the projector's copy, and the ψ trail). */
export function clearDMS() {
  clear(); lastResult = null; curves = []; dets = []; primaryQ = null;
  clearTrail();
  dispatchEvent(new CustomEvent('dmsUpdated'));
  State.requestRender();
}
