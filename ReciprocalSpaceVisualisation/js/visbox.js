// ============================================================
// js/visbox.js — visibility clip box (non-destructive)
// ------------------------------------------------------------
// An oriented box, dragged with its own gizmo, that hides everything inside or
// outside it — so the interior of the map can be inspected (a shell of peaks
// hiding the ones behind it, a slab of the DMS network, one lobe of the Ewald
// sphere) without changing the model. Ported from the LidarStudio viewer's
// visibility box; the interaction is the same (place / Move-Rotate-Scale or
// T-R-S / inside-outside / clear), but the cut is made with Three's own local
// clipping planes rather than a shader uniform: the six inward faces of the box
// are exactly six planes, and every material here supports them (LineMaterial
// and the fat DMS lines included — the one exception, the peak cloud's custom
// ShaderMaterial, carries the clipping chunks; see js/reflections.js).
//
// "Show outside" is the same six planes with `clipIntersection`, which discards
// only where *all* of them clip — i.e. only the box interior.
//
// Nothing is deleted: this is a view filter, and clearing the box restores
// everything. (LidarStudio's box can also delete the shown points; there is no
// equivalent here — the peaks are generated from the lattice, not a file.)
// ============================================================
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import * as State from './state.js';
import { scene, orbit } from './scene.js';
import { getPoints } from './reflections.js';

const BOX_COLOR = 0x33aaff;
const MIN_EDGE = 1e-3;        // scene units — keeps the planes well-conditioned

// The six inward-facing planes, rebuilt whenever the box moves. Every clipped
// material holds *this* array, so one update cuts the whole scene at once — and
// since the identity never changes, re-assigning it to a material that already
// has it costs nothing (no shader recompile).
const planes = [];
for (let i = 0; i < 6; i++) planes.push(new THREE.Plane());

let box = null;               // unit cube ([-0.5,0.5]³) placed by its world matrix
let gizmo = null;
// Pose kept when the box is cleared, so re-placing brings it back where it was
// instead of re-fitting to the data.
let lastPose = null;
// Set when the box goes away: one more sweep is then needed to strip the planes
// off the materials that still hold them.
let needsSweep = false;

const _c = new THREE.Vector3(), _a = new THREE.Vector3();
const _p = new THREE.Vector3(), _n = new THREE.Vector3();

// ── The six planes of the current box ───────────────────────
// The cube is 1×1×1, so column i of the world matrix is the box's i-th edge
// vector: its direction is the face normal (the columns stay mutually
// orthogonal under rotate+scale) and its length is the edge length.
function updatePlanes() {
  if (!box) return;
  box.updateWorldMatrix(true, false);
  const m = box.matrixWorld;
  _c.setFromMatrixPosition(m);
  for (let i = 0; i < 3; i++) {
    _a.setFromMatrixColumn(m, i);
    const half = Math.max(_a.length(), MIN_EDGE) * 0.5;
    _a.normalize();
    // +face: keep the side the centre is on, so the normal points inward.
    planes[2 * i].setFromNormalAndCoplanarPoint(
      _n.copy(_a).negate(), _p.copy(_c).addScaledVector(_a, half));
    planes[2 * i + 1].setFromNormalAndCoplanarPoint(
      _a, _p.copy(_c).addScaledVector(_a, -half));
  }
}

// ── Applying the planes to the scene's materials ────────────
// `want` is the shared array (clip) or null (don't). The plane count and the
// union/intersection mode are compiled into the program, so a material only
// needs recompiling when one of them actually changes — hence the early-out.
function setMaterialClip(mat, want) {
  const intersect = want ? !!State.params.visBox.showOutside : false;
  if (mat.clippingPlanes === want && mat.clipIntersection === intersect) return;
  mat.clippingPlanes = want;
  mat.clipIntersection = intersect;
  mat.needsUpdate = true;
}

// Walk the graph, skipping the box, the gizmos and anything flagged noClip
// (the axes and basis arrows — they are the frame of reference, not content).
function sweep(obj, want) {
  if (obj === box || obj.isTransformControls || obj.userData.noClip) return;
  const m = obj.material;
  if (m) {
    if (Array.isArray(m)) m.forEach((x) => setMaterialClip(x, want));
    else setMaterialClip(m, want);
  }
  for (const child of obj.children) sweep(child, want);
}

/** Push the current clip state onto every material. Called once per rendered
 *  frame (main.js) because the DMS curves, the trail and the peak cloud are
 *  rebuilt constantly — a fresh material must pick the box up on its first
 *  frame. Cheap: a graph walk of same-value comparisons. */
export function syncClip() {
  if (!box) {
    if (needsSweep) { sweep(scene, null); needsSweep = false; }
    return;
  }
  sweep(scene, planes);
  // The detector panel is instrumentation rather than reciprocal-space content,
  // so it is left whole unless asked for: sweep it back off afterwards.
  if (!State.params.visBox.clipDetector) {
    for (const n of scene.children) if (n.userData.instrument) sweep(n, null);
  }
}

// ── Placing the box ─────────────────────────────────────────
function dataBounds() {
  const b = new THREE.Box3();
  const pts = getPoints();
  if (pts) b.setFromObject(pts);
  if (b.isEmpty()) {
    const q = State.params.qMax || 3;
    b.set(new THREE.Vector3(-q, -q, -q), new THREE.Vector3(q, q, q));
  }
  return b;
}

function clampScale() {
  if (!box) return;
  box.scale.set(Math.max(box.scale.x, MIN_EDGE),
                Math.max(box.scale.y, MIN_EDGE),
                Math.max(box.scale.z, MIN_EDGE));
}

function ensureGizmo() {
  if (gizmo) return gizmo;
  const g = new TransformControls(State.camera, State.renderer.domElement);
  g.setSize(0.8);
  // Its own gizmo, so dragging the box never disturbs the detector panel's.
  g.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; });
  g.addEventListener('objectChange', () => { clampScale(); updatePlanes(); State.requestRender(); });
  g.addEventListener('change', () => State.requestRender());
  scene.add(g);
  gizmo = g;
  return g;
}

/** Place the box. With no box present it returns to the pose it was cleared at
 *  (if any); `refit: true` always re-fits it to the current peak cloud. */
export function placeBox({ refit = false } = {}) {
  const restore = !refit && !box && lastPose;
  clearBox(false);

  const geo = new THREE.BoxGeometry(1, 1, 1);
  box = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
    color: BOX_COLOR, transparent: true, opacity: 0.06, depthWrite: false }));
  box.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo),
    new THREE.LineBasicMaterial({ color: BOX_COLOR })));
  box.userData.noClip = true;          // a box that clipped itself would vanish

  if (restore) {
    box.position.copy(lastPose.position);
    box.quaternion.copy(lastPose.quaternion);
    box.scale.copy(lastPose.scale);
  } else {
    const b = dataBounds();
    const c = b.getCenter(new THREE.Vector3()), s = b.getSize(new THREE.Vector3());
    box.position.copy(c);
    // Half the data's extent, so there is something to reveal from the start.
    box.scale.set(Math.max(s.x * 0.5, 0.05),
                  Math.max(s.y * 0.5, 0.05),
                  Math.max(s.z * 0.5, 0.05));
  }

  scene.add(box);
  ensureGizmo().attach(box);
  gizmo.setMode(State.params.visBox.gizmo);
  State.params.visBox.show = true;
  updatePlanes();
  State.requestRender();
  dispatchEvent(new CustomEvent('visBoxChanged'));
}

/** Remove the box and un-clip everything. */
export function clearBox(render = true) {
  if (gizmo && gizmo.object) gizmo.detach();
  if (box) {
    lastPose = {
      position: box.position.clone(),
      quaternion: box.quaternion.clone(),
      scale: box.scale.clone(),
    };
    scene.remove(box);
    box.traverse((o) => { o.geometry?.dispose?.(); o.material?.dispose?.(); });
    box = null;
    needsSweep = true;
  }
  State.params.visBox.show = false;
  if (render) {
    syncClip();
    State.requestRender();
    dispatchEvent(new CustomEvent('visBoxChanged'));
  }
}

export function hasBox() { return !!box; }

/** True while the box's gizmo has the pointer (hovering an axis or dragging) —
 *  so other click handlers, notably the blob drag, can stand aside. */
export function gizmoBusy() { return !!(gizmo && (gizmo.dragging || gizmo.axis)); }

/** Gizmo mode: 'translate' | 'rotate' | 'scale'. */
export function setGizmoMode(mode) {
  State.params.visBox.gizmo = mode;
  gizmo?.setMode(mode);
  State.requestRender();
  dispatchEvent(new CustomEvent('visBoxChanged'));
}

/** false = show what is inside the box, true = show what is outside it. */
export function setShowOutside(v) {
  State.params.visBox.showOutside = !!v;
  State.requestRender();
  dispatchEvent(new CustomEvent('visBoxChanged'));
}

/** Whether the detector panel is cut by the box as well. */
export function setClipDetector(v) {
  State.params.visBox.clipDetector = !!v;
  // Turning it off has to strip the planes the panel already holds.
  if (!v) for (const n of scene.children) if (n.userData.instrument) sweep(n, null);
  State.requestRender();
}

/** Is a world-space point currently shown? Used by the peak hover, so a hidden
 *  peak cannot be picked out of thin air. */
export function isPointVisible(p) {
  if (!box) return true;
  let inside = true;
  for (const pl of planes) { if (pl.distanceToPoint(p) < 0) { inside = false; break; } }
  return State.params.visBox.showOutside ? !inside : inside;
}

// The gizmo has to follow the camera swap (perspective ⇄ orthographic), exactly
// as scene.js does for the shared detector gizmo.
addEventListener('cameraChanged', (e) => { if (gizmo) gizmo.camera = e.detail; });

// T / R / S switch the gizmo mode, as in LidarStudio. Ignored while typing in
// the control panel, so "translate" can still be spelled into a text field.
addEventListener('keydown', (e) => {
  if (!box || e.ctrlKey || e.metaKey || e.altKey) return;
  const t = e.target;
  if (t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' ||
            t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
  const mode = { t: 'translate', r: 'rotate', s: 'scale' }[e.key.toLowerCase()];
  if (mode) setGizmoMode(mode);
});
