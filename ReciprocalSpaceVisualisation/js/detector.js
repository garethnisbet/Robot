// ============================================================
// js/detector.js — the draggable detector panel + on-panel blob + DMS map
// ------------------------------------------------------------
// A flat panel you drag around the scene with a gizmo (translate / rotate,
// pivoting about its own centre or about the sample at the origin). Two things
// living on the panel map into reciprocal space, live, as it (or they) move:
//
//   • a Gaussian blob — its footprint is pushed through Q = k0·û(point) − k_in,
//     so it bows onto the Ewald sphere; drag it across the panel and watch Q
//     move;
//   • the DMS multiple-scattering pattern — the engine's own detector-pixel
//     projection of the Kossel lines (the same pattern the DMS slider paints on
//     its detector image) is drawn straight onto the panel, since the panel is
//     modelled from that engine's pixel map; the matching Q-space loci are
//     highlighted back in reciprocal space (the round trip).
//
// All of it is client-side: the map depends only on the *direction* from the
// sample, so the panel's display distance is irrelevant and no server is asked
// anything as you drag. See js/optics.js for the physics.
// ============================================================
import * as THREE from 'three';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import * as State from './state.js';
import { scene, orbit, transform, attachTransform, detachTransform, snapToDirection } from './scene.js';
import { gizmoBusy as visGizmoBusy } from './visbox.js';
import * as Optics from './optics.js';
import { getCurves, getDets, getPrimaryQ, getDetectorGeom } from './dms.js';

// ── inferno ramp (matches the lattice/Q colouring elsewhere) ──
const INFERNO = [
  [0.001, 0.000, 0.014], [0.290, 0.047, 0.418], [0.611, 0.191, 0.377],
  [0.881, 0.392, 0.187], [0.988, 0.710, 0.115], [0.988, 0.998, 0.645],
];
function inferno(t) {
  t = Math.min(1, Math.max(0, t));
  const x = t * (INFERNO.length - 1);
  const i = Math.floor(x), f = x - i;
  const a = INFERNO[i], b = INFERNO[Math.min(i + 1, INFERNO.length - 1)];
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}
function hueColor(hkl) {
  if (!hkl) return new THREE.Color(0x5ec8d8);
  const s = (hkl[0] * 73856093) ^ (hkl[1] * 19349663) ^ (hkl[2] * 83492791);
  return new THREE.Color().setHSL((((s % 360) + 360) % 360) / 360, 0.65, 0.62);
}

// ── scene graph ─────────────────────────────────────────────
// pivotGroup (at the origin) → panel → { blob disc/dot, projected DMS lines }.
// qGroup holds everything drawn in reciprocal space (the blob's mapped patch
// and the DMS points recovered from the panel).
const pivotGroup = new THREE.Group();
pivotGroup.visible = false;
// Instrumentation rather than reciprocal-space content: the visibility box
// leaves the panel whole unless explicitly asked to cut it (js/visbox.js).
pivotGroup.userData.instrument = true;
scene.add(pivotGroup);

const qGroup = new THREE.Group();
scene.add(qGroup);

// The Ewald scattering triangle for the blob (k_in, k_out, Q), in Q space.
const triGroup = new THREE.Group();
scene.add(triGroup);

let panel = null, border = null;
// Set once the gizmo has been dragged: a panel the user has placed by hand is
// theirs, so recomputing DMS must not snatch it back onto the fitted geometry.
let panelMoved = false;
// Panel half-extents in scene units. Square and slider-driven until DMS hands
// back a real detector, then the image rectangle's own aspect (a Pilatus 2M
// frame is 1679 × 1475, not square).
let halfW = 1.2, halfH = 1.2;
let blobDisc = null, blobDot = null, dmsProj = null;
let blobPatch = null, blobCentre = null, dmsRecovered = null;
let onBlobQ = null;                       // callback: (|Q| number | null) → void

// ── build the panel once ────────────────────────────────────
function ensurePanel() {
  if (panel) return;
  halfW = halfH = State.params.detector.halfSize;
  panel = new THREE.Mesh(
    new THREE.PlaneGeometry(2 * halfW, 2 * halfH),
    new THREE.MeshBasicMaterial({ color: 0x24384f, transparent: true, opacity: 0.32,
      side: THREE.DoubleSide, depthWrite: false }));
  border = new THREE.LineSegments(
    new THREE.EdgesGeometry(panel.geometry),
    new THREE.LineBasicMaterial({ color: 0x5ec8d8, transparent: true, opacity: 0.85 }));
  panel.add(border);

  // blob footprint (a disc the size of σ) + a bright centre dot, on the surface
  blobDisc = new THREE.Mesh(
    new THREE.CircleGeometry(State.params.blob.sigma, 32),
    new THREE.MeshBasicMaterial({ color: 0xffcc44, transparent: true, opacity: 0.35,
      side: THREE.DoubleSide, depthWrite: false }));
  blobDot = new THREE.Mesh(
    new THREE.SphereGeometry(0.03, 12, 12),
    new THREE.MeshBasicMaterial({ color: 0xffe08a }));
  blobDisc.add(blobDot);
  blobDisc.position.z = 0.002;
  panel.add(blobDisc);

  placePanel();
  pivotGroup.add(panel);
}

/** The scattering angle 2θ (degrees) of the panel centre, or null before DMS
 *  has run. The engine's detector sits opposite the incident beam (back-
 *  reflection, 2θ ≈ 180°), where the diffracted Kossel beams land — the primary
 *  reflection is *not* on it (its k_out misses by tens of degrees). */
export function twoThetaDeg() {
  const g = getDetectorGeom();
  return g ? g.twoThetaDeg : null;
}

/** 2θ of the primary reflection's own exit beam, k_out = k_in + Q_primary. */
export function primaryTwoThetaDeg() {
  const pq = getPrimaryQ();
  if (!pq) return null;
  const kout = new THREE.Vector3(pq[0], pq[1], pq[2]).add(Optics.kIn());
  if (kout.lengthSq() < 1e-12) return null;
  return THREE.MathUtils.radToDeg(Math.acos(
    THREE.MathUtils.clamp(kout.normalize().dot(Optics.s0()), -1, 1)));
}

// Placement: the panel *is* the engine's detector, scaled to the display
// distance — the same rectangle the DMS fit was made against, at its fitted
// plate rotations, with the beam centre wherever px/py put it (so the panel
// centre and the beam spot are not the same point).
//
// That detector sits **opposite the incident beam** (back-reflection, 2θ ≈ 180°):
// the diffracted Kossel beams leave the sample along k_out = k_in + Q and land on
// it, so a forward ray from the sample reaches it (dms_compute places it on the
// −k_in side for exactly this reason — the engine's own pixel map is a point
// reflection). The primary reflection's own k_out is far off it — placing the
// panel at 2θ_B instead would show a view the slider never shows.
//
// Before DMS has run there is no detector to model, so fall back to a generic
// 30° off the beam: sitting on the beam axis puts the panel centre at Q = 0,
// where k_out coincides with k_in and the scattering triangle collapses.
function placePanel() {
  const d = State.params.detector;
  const g = getDetectorGeom();

  if (g?.corners?.length === 4) {
    const C = g.corners.map((c) => new THREE.Vector3(c[0], c[1], c[2])
      .multiplyScalar(d.displayDist));
    const u = C[1].clone().sub(C[0]);          // +row edge
    const v = C[3].clone().sub(C[0]);          // +col edge
    halfW = u.length() / 2;
    halfH = v.length() / 2;
    u.normalize(); v.normalize();
    const n = new THREE.Vector3().crossVectors(u, v).normalize();
    panel.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(u, v, n));
    panel.position.copy(C[0]).add(C[2]).multiplyScalar(0.5);   // rectangle centre
    rebuildPanelGeometry();
    return;
  }

  halfW = halfH = d.halfSize;
  rebuildPanelGeometry();
  const s0 = Optics.s0();
  let perp = new THREE.Vector3(1, 0, 0);
  if (Math.abs(s0.dot(perp)) > 0.9) perp = new THREE.Vector3(0, 0, 1);
  const axis = new THREE.Vector3().crossVectors(s0, perp).normalize();
  const dir = s0.clone().applyAxisAngle(axis, THREE.MathUtils.degToRad(30));
  panel.position.copy(dir.multiplyScalar(d.displayDist));
  panel.lookAt(0, 0, 0);
}

function rebuildPanelGeometry() {
  panel.geometry.dispose();
  panel.geometry = new THREE.PlaneGeometry(2 * halfW, 2 * halfH);
  border.geometry.dispose();
  border.geometry = new THREE.EdgesGeometry(panel.geometry);
  clampBlob();
}

// ── public API used by the panel UI ─────────────────────────
export function setDetectorVisible(on) {
  ensurePanel();
  pivotGroup.visible = on;
  if (on) applyGizmo(); else detachTransform();
  qGroup.visible = on;
  refresh();
  State.requestRender();
}

/** Override the modelled size with a square panel (the "panel size" slider). */
export function setPanelSize(halfSize) {
  ensurePanel();
  halfW = halfH = halfSize;
  rebuildPanelGeometry();
  refresh();
}

/** Put the panel back on the detector the DMS engine models. */
export function resetPanel() {
  ensurePanel();
  pivotGroup.rotation.set(0, 0, 0);
  pivotGroup.position.set(0, 0, 0);
  panel.rotation.set(0, 0, 0);
  panelMoved = false;
  placePanel();
  applyGizmo();
  refresh();
  State.requestRender();
}

/** Attach the gizmo to the right object for the current mode + pivot. */
export function applyGizmo() {
  if (!pivotGroup.visible) return;
  const d = State.params.detector;
  if (d.gizmo === 'rotate' && d.pivot === 'sample') attachTransform(pivotGroup, 'rotate');
  else attachTransform(panel, d.gizmo);
}

export function setBlobVisible(on) {
  ensurePanel();
  blobDisc.visible = on;
  refresh();
  State.requestRender();
}
export function setBlobSigma(sigma) {
  ensurePanel();
  blobDisc.geometry.dispose();
  blobDisc.geometry = new THREE.CircleGeometry(sigma, 32);
  refresh();
}
export function onBlobQChange(cb) { onBlobQ = cb; }
export function setTriangleVisible(on) { State.params.dms.showTriangle = on; refresh(); }

/** Snap the camera to look down the exit beam k_out = k_in + Q_primary (the
 *  diffracted beam of the primary reflection). Needs a computed DMS primary;
 *  returns false if there is none yet. */
export function viewAlongKout() {
  const pq = getPrimaryQ();
  if (!pq) return false;
  const kout = new THREE.Vector3(pq[0], pq[1], pq[2]).add(Optics.kIn());
  if (kout.lengthSq() < 1e-12) return false;
  const dir = kout.normalize();
  const up = Math.abs(dir.y) > 0.9 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(0, 1, 0);
  // Look *in* the k_out direction (beam into the screen): camera on the −k_out side.
  snapToDirection(dir.clone().multiplyScalar(-1), up);
  return true;
}

// Energy sets the Ewald radius k0; keep the current beam direction (the DMS
// computation owns the direction once it has run).
export function setBeamEnergy(kev) { Optics.setBeam(kev, null); refresh(); }

export function hasPanel() { return panel !== null && pivotGroup.visible; }

// Recompute both maps (blob patch + DMS projection). rAF-throttled.
let refreshQueued = false;
export function refresh() {
  if (refreshQueued) return;
  refreshQueued = true;
  requestAnimationFrame(() => {
    refreshQueued = false;
    buildTriangle();                    // primary-reflection triangle: no panel needed
    if (!panel || !pivotGroup.visible) { clearQGroup(); clearDms(); State.requestRender(); return; }
    panel.updateWorldMatrix(true, false);
    updateBlobLocal();
    computeBlobPatch();
    computeDmsProjection();
    State.requestRender();
  });
}
// dms.js calls this when a fresh set of curves arrives.
export function refreshProjection() { refresh(); }

// ── blob: local position on the panel and its image in Q ────
function clampBlob() {
  const b = State.params.blob;
  b.u = Math.max(-halfW, Math.min(halfW, b.u));
  b.v = Math.max(-halfH, Math.min(halfH, b.v));
}
function updateBlobLocal() {
  const b = State.params.blob;
  blobDisc.position.set(b.u, b.v, 0.002);
}

function clearQGroup() {
  for (const o of [blobPatch, blobCentre]) {
    if (!o) continue;
    o.geometry?.dispose(); o.material?.dispose(); qGroup.remove(o);
  }
  blobPatch = blobCentre = null;
}

function computeBlobPatch() {
  clearQGroup();
  const b = State.params.blob;
  if (!b.show || !blobDisc.visible) { onBlobQ?.(null); return; }
  const N = 11, reach = 2 * b.sigma;
  const pos = new Float32Array(N * N * 3), col = new Float32Array(N * N * 3);
  const world = new THREE.Vector3();
  let k = 0;
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < N; j++) {
      const du = ((i / (N - 1)) - 0.5) * 2 * reach;
      const dv = ((j / (N - 1)) - 0.5) * 2 * reach;
      const lu = Math.max(-halfW, Math.min(halfW, b.u + du));
      const lv = Math.max(-halfH, Math.min(halfH, b.v + dv));
      world.set(lu, lv, 0); panel.localToWorld(world);
      const q = Optics.qFromWorld(world);
      const w = Math.exp(-(du * du + dv * dv) / (2 * b.sigma * b.sigma));
      const c = inferno(0.2 + 0.8 * w);
      pos[k * 3] = q.x; pos[k * 3 + 1] = q.y; pos[k * 3 + 2] = q.z;
      col[k * 3] = c[0]; col[k * 3 + 1] = c[1]; col[k * 3 + 2] = c[2];
      k++;
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  blobPatch = new THREE.Points(g, new THREE.PointsMaterial({
    size: 0.06, sizeAttenuation: true, vertexColors: true,
    transparent: true, opacity: 0.9, depthWrite: false }));
  qGroup.add(blobPatch);

  // centre marker + |Q| readout
  world.set(b.u, b.v, 0); panel.localToWorld(world);
  const qc = Optics.qFromWorld(world);
  const cg = new THREE.BufferGeometry();
  cg.setAttribute('position', new THREE.BufferAttribute(new Float32Array([qc.x, qc.y, qc.z]), 3));
  blobCentre = new THREE.Points(cg, new THREE.PointsMaterial({
    color: 0xffe08a, size: 0.16, sizeAttenuation: true, depthWrite: false }));
  qGroup.add(blobCentre);
  onBlobQ?.(qc.length());
}

// ── the Ewald scattering triangle for the primary reflection (k_in, k_out, Q) ──
// In this frame the Ewald sphere is centred at C = −k_in and the origin O holds
// the direct beam (Q = 0). The primary reflection sits on the sphere (it is in
// the Bragg condition), so the triangle is C→O = k_in, C→(primary) = k_out, and
// O→(primary) = Q — k_out connects the source to the primary reflection, and
// k_out = k_in + Q reads straight off the picture. Needs a computed DMS primary.
function clearTriangle() {
  for (let i = triGroup.children.length - 1; i >= 0; i--) {
    const c = triGroup.children[i];
    c.dispose?.();                                   // ArrowHelper.dispose()
    if (c.element && c.element.remove) c.element.remove();
    triGroup.remove(c);
  }
}
function makeLabel(text, color, pos) {
  const e = document.createElement('div');
  e.className = 'axis-label'; e.textContent = text; e.style.color = color;
  const o = new CSS2DObject(e); o.position.copy(pos);
  return o;
}
function buildTriangle() {
  clearTriangle();
  if (!State.params.dms.showTriangle) return;
  const pq = getPrimaryQ();
  if (!pq) return;                                   // no primary until DMS is computed

  const Q = new THREE.Vector3(pq[0], pq[1], pq[2]);  // the primary reflection
  const C = Optics.ewaldCentre();                    // −k_in
  const O = new THREE.Vector3(0, 0, 0);
  const k0 = Optics.k0();

  // k_in: C → O
  const kin = O.clone().sub(C);
  triGroup.add(new THREE.ArrowHelper(kin.clone().normalize(), C, kin.length(),
    0xffcc44, 0.1 * k0, 0.06 * k0));
  triGroup.add(makeLabel('k_in', '#ffcc44', C.clone().lerp(O, 0.5)));

  // k_out: C → the primary reflection
  const kout = Q.clone().sub(C);
  triGroup.add(new THREE.ArrowHelper(kout.clone().normalize(), C, kout.length(),
    0xff8844, 0.1 * k0, 0.06 * k0));
  triGroup.add(makeLabel('k_out', '#ff8844', C.clone().lerp(Q, 0.5)));

  // Q (the scattering vector): O → the primary reflection
  const qlen = Q.length();
  if (qlen > 1e-5) {
    triGroup.add(new THREE.ArrowHelper(Q.clone().normalize(), O, qlen,
      0x5ec8d8, Math.min(0.12, 0.25 * qlen), Math.min(0.07, 0.15 * qlen)));
    triGroup.add(makeLabel('Q', '#8fe4ef', Q.clone().multiplyScalar(0.5)));
  }
}

// ── DMS curves → panel, and the caught points highlighted in Q ──
function clearDms() {
  if (dmsProj) { dmsProj.geometry.dispose(); dmsProj.material.dispose(); panel.remove(dmsProj); dmsProj = null; }
  if (dmsRecovered) { dmsRecovered.geometry.dispose(); dmsRecovered.material.dispose(); qGroup.remove(dmsRecovered); dmsRecovered = null; }
}

function computeDmsProjection() {
  clearDms();
  const dp = State.params.dms;
  const dets = getDets();
  if (!dets.length || !(dp.projectOnPanel || dp.showRecovered)) return;

  // The panel *is* the engine's detector rectangle (its corners are inverted out
  // of the same pixel map that produced these polylines — see dms_compute), so
  // the engine's own detector projection maps straight onto the panel: a point
  // normalised to [-0.5,0.5] across the image sits at (2·halfW·du, 2·halfH·dv)
  // in panel-local coordinates. Painting them here reproduces the DMS slider's
  // detector pattern exactly, and they ride on the surface when the panel moves.
  //
  // (An earlier version ray-traced k_out = k_in + Q from the sample onto the
  // panel instead. That is a *different* geometry from the engine's — its
  // dms2px anchors each beam at the detector centre, not the sample origin — so
  // it never matched the slider; the reason the panel is modelled from the
  // engine's map is precisely so its own projection can be used verbatim.)
  const segPos = [], segCol = [];
  for (const d of dets) {
    const col = hueColor(d.hkl);
    const uv = d.uv;
    for (let i = 1; i < uv.length; i++) {
      const [du0, dv0] = uv[i - 1], [du1, dv1] = uv[i];
      segPos.push(2 * halfW * du0, 2 * halfH * dv0, 0.003,
                  2 * halfW * du1, 2 * halfH * dv1, 0.003);
      segCol.push(col.r, col.g, col.b, col.r, col.g, col.b);
    }
  }

  if (dp.projectOnPanel && segPos.length) {
    const sigma = Math.max(1e-4, dp.lineSigma);   // drawn line half-width
    const g = new LineSegmentsGeometry();
    g.setPositions(segPos); g.setColors(segCol);
    const m = new LineMaterial({ vertexColors: true, worldUnits: true,
      linewidth: 2 * sigma, transparent: true, opacity: 0.95 });
    m.resolution.set(innerWidth, innerHeight);
    dmsProj = new LineSegments2(g, m);
    panel.add(dmsProj);
  }

  // "Recovered in Q": the Q-space curves are the exact reciprocal-space loci of
  // the very lines drawn on the panel (same engine, same beams), so highlight
  // their points back in Q — the round trip between the detector and Q.
  if (dp.showRecovered) {
    const recPos = [], recCol = [];
    for (const cur of getCurves()) {
      const col = hueColor(cur.hkl);
      for (const q of cur.pts) { recPos.push(q[0], q[1], q[2]); recCol.push(col.r, col.g, col.b); }
    }
    if (recPos.length) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(recPos), 3));
      g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(recCol), 3));
      dmsRecovered = new THREE.Points(g, new THREE.PointsMaterial({
        size: 0.05, sizeAttenuation: true, vertexColors: true,
        transparent: true, opacity: 0.95, depthWrite: false }));
      qGroup.add(dmsRecovered);
    }
  }
}

// ── dragging the blob across the panel (raycast the panel plane) ──
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let dragging = false;

function setNdc(e) {
  ndc.x = (e.clientX / innerWidth) * 2 - 1;
  ndc.y = -(e.clientY / innerHeight) * 2 + 1;
}
function panelHitLocal(e) {
  setNdc(e);
  raycaster.setFromCamera(ndc, State.camera);
  const hits = raycaster.intersectObject(panel, false);
  if (!hits.length) return null;
  return panel.worldToLocal(hits[0].point.clone());
}

// Capture phase, so we can veto the camera controls before they grab the pointer.
State.renderer.domElement.addEventListener('pointerdown', (e) => {
  if (!pivotGroup.visible || !State.params.blob.show || !blobDisc.visible) return;
  if (transform.dragging || transform.axis) return;      // the gizmo owns this click
  if (visGizmoBusy()) return;                            // ditto the visibility box's
  const loc = panelHitLocal(e);
  if (!loc) return;
  const b = State.params.blob;
  b.u = loc.x; b.v = loc.y; clampBlob();
  dragging = true;
  orbit.enabled = false;
  updateBlobLocal();
  refresh();
}, true);

addEventListener('pointermove', (e) => {
  if (!dragging) return;
  const loc = panelHitLocal(e);
  if (!loc) return;
  const b = State.params.blob;
  b.u = loc.x; b.v = loc.y; clampBlob();
  updateBlobLocal();
  refresh();
});

addEventListener('pointerup', () => {
  if (!dragging) return;
  dragging = false;
  orbit.enabled = true;
});

// When the gizmo moves the panel, the maps must follow — and the placement is
// now the user's, not the primary's 2θ (see placePanel / dmsUpdated below).
transform.addEventListener('objectChange', () => { panelMoved = true; refresh(); });

// A fresh DMS computation (new curves + a re-pointed beam) re-projects — and
// brings a new primary with it, so an untouched panel follows the exit beam to
// its new 2θ. A panel the user has dragged stays where they put it.
addEventListener('dmsUpdated', () => {
  if (panel && !panelMoved) { panel.rotation.set(0, 0, 0); placePanel(); }
  refresh();
});

// The projected fat line is sized in world units but still needs the viewport
// size for correct screen rasterisation.
addEventListener('resize', () => {
  if (dmsProj?.material?.resolution) {
    dmsProj.material.resolution.set(innerWidth, innerHeight);
    State.requestRender();
  }
});
