// ============================================================
// js/scene.js — Three.js scene, cameras, controls, nav gizmo,
//               reciprocal-space axes. On-demand rendering.
// ============================================================
import * as THREE from 'three';
import { TrackballControls } from 'three/addons/controls/TrackballControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

import * as State from './state.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e0e14);

const aspect = innerWidth / innerHeight;
const perspCamera = new THREE.PerspectiveCamera(45, aspect, 0.001, 1000);
perspCamera.position.set(4, 3, 5);

// Orthographic camera shares the same look direction; frustum set in updateOrtho.
const orthoCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.001, 1000);
orthoCamera.position.copy(perspCamera.position);

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
// Per-material clipping planes — the visibility box (js/visbox.js) cuts with
// six of them, assigned only to the materials it should affect.
renderer.localClippingEnabled = true;
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(devicePixelRatio);
document.body.appendChild(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.setSize(innerWidth, innerHeight);
labelRenderer.domElement.style.position = 'absolute';
labelRenderer.domElement.style.top = '0';
labelRenderer.domElement.style.pointerEvents = 'none';
document.body.appendChild(labelRenderer.domElement);

// TrackballControls (rather than OrbitControls) so the scene tumbles freely and
// continuously in any direction. OrbitControls keeps the horizon level and
// clamps the vertical angle at the poles, which reads as a rotation limit.
const orbit = new TrackballControls(perspCamera, renderer.domElement);
orbit.rotateSpeed = 3.2;
orbit.zoomSpeed = 1.2;
orbit.panSpeed = 0.8;
orbit.staticMoving = false;          // dynamic damping → smooth, weighty feel
orbit.dynamicDampingFactor = 0.15;
orbit.target.set(0, 0, 0);
orbit.update();
orbit.addEventListener('change', () => State.requestRender());

// Hold Shift and drag to pan (translate) instead of orbit. TrackballControls
// decides pan vs rotate from keyState on each move; we set it from the pointer's
// Shift state at the start of every drag (capture phase, before Trackball's own
// handler). _STATE.PAN = 2, _STATE.NONE = -1 (TrackballControls internals).
renderer.domElement.addEventListener('pointerdown', (e) => {
  if (orbit.enabled) orbit.keyState = (e.shiftKey && !orbit.noPan) ? 2 : -1;
}, true);

// ── Transform gizmo (drag the detector panel) ───────────────
// One shared gizmo; detector.js attaches it to the panel (or its pivot group).
// While it is being dragged the orbit controls must let go of the pointer, or
// the camera spins under the drag.
const transform = new TransformControls(perspCamera, renderer.domElement);
transform.setSize(0.9);
transform.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; });
transform.addEventListener('change', () => State.requestRender());
scene.add(transform);

/** Attach the gizmo to `object` in the given mode ('translate'|'rotate'). */
export function attachTransform(object, mode = 'translate') {
  transform.attach(object);
  transform.setMode(mode);
  State.requestRender();
}
export function setTransformMode(mode) { transform.setMode(mode); State.requestRender(); }
export function detachTransform() { transform.detach(); State.requestRender(); }
export function transformDragging() { return transform.dragging; }
export { transform };

State.initCore(scene, perspCamera, orthoCamera, renderer, labelRenderer, orbit);

// ── Lights (mostly for any solid geometry; points are unlit) ──
scene.add(new THREE.AmbientLight(0xffffff, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 0.6);
key.position.set(5, 8, 6);
scene.add(key);

// ── Reciprocal-space axes (Qx, Qy, Qz) ──────────────────────
const axesGroup = new THREE.Group();
// The frame of reference, not content: the visibility box never cuts it.
axesGroup.userData.noClip = true;
scene.add(axesGroup);
const AXIS_COLORS = [0xff5566, 0x55dd66, 0x5588ff];
const AXIS_NAMES = ['Qx', 'Qy', 'Qz'];

export function buildAxes(length = 3.2) {
  axesGroup.clear();
  const dirs = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  dirs.forEach((d, i) => {
    const v = new THREE.Vector3(...d);
    const arrow = new THREE.ArrowHelper(v, new THREE.Vector3(0, 0, 0),
      length, AXIS_COLORS[i], length * 0.06, length * 0.035);
    axesGroup.add(arrow);
    const el = document.createElement('div');
    el.className = 'axis-label';
    el.textContent = AXIS_NAMES[i];
    el.style.color = '#' + AXIS_COLORS[i].toString(16).padStart(6, '0');
    const label = new CSS2DObject(el);
    label.position.copy(v.multiplyScalar(length * 1.06));
    axesGroup.add(label);
  });
  State.requestRender();
}
export function setAxesVisible(v) { axesGroup.visible = v; State.requestRender(); }

// ── Reciprocal basis arrows (b1*, b2*, b3*) for crystal mode ──
const basisGroup = new THREE.Group();
basisGroup.userData.noClip = true;
scene.add(basisGroup);
const BASIS_COLORS = [0xffaa33, 0xffee44, 0x66ffcc];
const BASIS_NAMES = ['b1*', 'b2*', 'b3*'];

export function buildBasis(basis) {
  basisGroup.clear();
  if (!basis) return;
  basis.forEach((col, i) => {
    const v = new THREE.Vector3(col[0], col[1], col[2]);
    const len = v.length();
    if (len < 1e-9) return;
    const arrow = new THREE.ArrowHelper(v.clone().normalize(),
      new THREE.Vector3(0, 0, 0), len, BASIS_COLORS[i], len * 0.12, len * 0.07);
    basisGroup.add(arrow);
    const el = document.createElement('div');
    el.className = 'axis-label';
    el.textContent = BASIS_NAMES[i];
    el.style.color = '#' + BASIS_COLORS[i].toString(16).padStart(6, '0');
    const label = new CSS2DObject(el);
    label.position.copy(v.clone().multiplyScalar(1.08));
    basisGroup.add(label);
  });
  State.requestRender();
}
export function setBasisVisible(v) { basisGroup.visible = v; State.requestRender(); }

// ── Ortho / perspective toggle ──────────────────────────────
export function setOrtho(on) {
  State.setOrthoOn(on);
  const cam = on ? orthoCamera : perspCamera;
  cam.position.copy(orbit.object.position);
  cam.up.copy(orbit.object.up);
  orbit.object = cam;
  transform.camera = cam;
  State.setActiveCamera(cam);
  // Other gizmos (the visibility box's) have to follow the swap too.
  dispatchEvent(new CustomEvent('cameraChanged', { detail: cam }));
  updateOrtho();
  onResize();
  State.requestRender();
}
function updateOrtho() {
  const dist = orthoCamera.position.distanceTo(orbit.target);
  const h = dist * 0.6;
  const w = h * (innerWidth / innerHeight);
  orthoCamera.left = -w; orthoCamera.right = w;
  orthoCamera.top = h; orthoCamera.bottom = -h;
  orthoCamera.updateProjectionMatrix();
}

// ============================================================
// Navigation gizmo (Blender-style ball, Canvas 2D)
// ------------------------------------------------------------
// Ported from the RobotVisualisation viewer so both projects behave the same:
// six axis balls projected through the inverse camera rotation, depth-sorted so
// the near ones occlude the far, hover highlight, and click to snap the camera
// to that axis with a short eased animation. Labels/colours are this project's
// (Qx/Qy/Qz), the mechanics are identical.
// ============================================================
export const navCanvas = document.getElementById('nav-gizmo');
const NAV_SIZE = 110;
navCanvas.width = Math.round(NAV_SIZE * devicePixelRatio);
navCanvas.height = Math.round(NAV_SIZE * devicePixelRatio);
const navCtx = navCanvas.getContext('2d');
navCtx.scale(devicePixelRatio, devicePixelRatio);

// [id, world direction, colour, label, radius, camera up-vector when snapped]
export const NAV_AXES = [
  { id: 'x+', dir: new THREE.Vector3( 1, 0, 0), color: '#ff5566', label: 'Qx', r: 11, up: new THREE.Vector3(0, 1, 0) },
  { id: 'y+', dir: new THREE.Vector3( 0, 1, 0), color: '#55dd66', label: 'Qy', r: 11, up: new THREE.Vector3(0, 0, -1) },
  { id: 'z+', dir: new THREE.Vector3( 0, 0, 1), color: '#5588ff', label: 'Qz', r: 11, up: new THREE.Vector3(0, 1, 0) },
  { id: 'x-', dir: new THREE.Vector3(-1, 0, 0), color: '#8a2f3a', label: '',   r:  8, up: new THREE.Vector3(0, 1, 0) },
  { id: 'y-', dir: new THREE.Vector3( 0,-1, 0), color: '#2f7a38', label: '',   r:  8, up: new THREE.Vector3(0, 0, 1) },
  { id: 'z-', dir: new THREE.Vector3( 0, 0,-1), color: '#2f4d8c', label: '',   r:  8, up: new THREE.Vector3(0, 1, 0) },
];

const _navQ = new THREE.Quaternion();
const _navV = new THREE.Vector3();
export function navProject(dir) {
  _navQ.copy(State.camera.quaternion).invert();
  _navV.copy(dir).applyQuaternion(_navQ);
  const c = NAV_SIZE / 2;
  const scale = NAV_SIZE / 2 - 16;
  return { x: c + _navV.x * scale, y: c - _navV.y * scale, z: _navV.z };
}

export let navHovered = null;
export function setNavHovered(v) { navHovered = v; }

function hexAlpha(hex, alpha) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  return `rgba(${r},${g},${b},${alpha.toFixed(2)})`;
}

export function renderNavGizmo() {
  const ctx = navCtx;
  const S = NAV_SIZE;
  ctx.clearRect(0, 0, S, S);

  ctx.save();
  ctx.beginPath();
  ctx.arc(S / 2, S / 2, S / 2 - 1, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(20,20,40,0.55)';
  ctx.fill();
  ctx.restore();

  const items = NAV_AXES.map((ax) => ({ ax, ...navProject(ax.dir) }));
  items.sort((a, b) => a.z - b.z);          // painter's order: far first

  // spokes joining each +/- pair
  for (const item of items) {
    if (!item.ax.id.endsWith('+')) continue;
    const neg = items.find((i) => i.ax.id === item.ax.id.replace('+', '-'));
    if (!neg) continue;
    ctx.beginPath();
    ctx.moveTo(neg.x, neg.y);
    ctx.lineTo(item.x, item.y);
    ctx.strokeStyle = 'rgba(180,180,180,0.25)';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  for (const item of items) {
    const { ax, x, y, z } = item;
    const alpha = 0.45 + 0.55 * ((z + 1) / 2);   // dim the ones pointing away
    const hovered = navHovered === ax.id;
    const r = ax.r + (hovered ? 2 : 0);

    const grad = ctx.createRadialGradient(x - r * 0.3, y - r * 0.3, r * 0.1, x, y, r);
    grad.addColorStop(0, hexAlpha(ax.color, Math.min(1, alpha + 0.3)));
    grad.addColorStop(1, hexAlpha(ax.color, alpha));
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = grad;
    ctx.fill();

    if (hovered) {
      ctx.beginPath();
      ctx.arc(x, y, r + 1, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(255,255,255,0.7)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    if (ax.label) {
      ctx.fillStyle = `rgba(255,255,255,${Math.min(1, alpha + 0.1)})`;
      ctx.font = `bold ${r - 2}px sans-serif`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(ax.label, x, y + 0.5);
    }
  }
}

export function navHitTest(mx, my) {
  const items = NAV_AXES.map((ax) => ({ ax, ...navProject(ax.dir) }));
  items.sort((a, b) => b.z - a.z);          // nearest first, so it wins the hit
  for (const item of items) {
    const dx = mx - item.x, dy = my - item.y;
    if (Math.sqrt(dx * dx + dy * dy) <= item.ax.r + 3) return item.ax.id;
  }
  return null;
}

// ── Camera snap animation ───────────────────────────────────
export let snapAnim = null;
export function setSnapAnim(v) { snapAnim = v; }
export const snapClock = new THREE.Clock();
export function easeNavSnap(t) { return t * t * (3 - 2 * t); }

/** Start an eased camera move to look down `dir` with `up` as the up-vector. */
export function snapToDirection(dir, up) {
  const cam = State.camera;
  const dist = cam.position.distanceTo(orbit.target) || 8;
  setSnapAnim({
    sp: cam.position.clone(),
    ep: orbit.target.clone().addScaledVector(dir, dist),
    su: cam.up.clone(),
    eu: up.clone(),
    t: 0,
  });
  orbit.enabled = false;
  State.requestRender();
}

/** Advance the snap by one frame; returns true while it is still running. */
export function stepSnap() {
  if (!snapAnim) { snapClock.getDelta(); return false; }
  snapAnim.t = Math.min(1, snapAnim.t + snapClock.getDelta() / 0.4);
  const t = easeNavSnap(snapAnim.t);
  const cam = State.camera;
  cam.position.lerpVectors(snapAnim.sp, snapAnim.ep, t);
  cam.up.lerpVectors(snapAnim.su, snapAnim.eu, t).normalize();
  cam.lookAt(orbit.target);
  State.requestRender();
  if (snapAnim.t >= 1) {
    setSnapAnim(null);
    orbit.enabled = true;
    if (State.orthoOn) updateOrtho();
  }
  return true;
}

// ── Camera view snapping (iso / top / front / side) ─────────
// Shares the gizmo's animation so a view button and an axis ball behave alike.
const VIEWS = {
  iso:   { dir: new THREE.Vector3(0.6, 0.5, 0.7).normalize(), up: new THREE.Vector3(0, 1, 0) },
  top:   { dir: new THREE.Vector3(0, 1, 0),  up: new THREE.Vector3(0, 0, -1) },
  front: { dir: new THREE.Vector3(0, 0, 1),  up: new THREE.Vector3(0, 1, 0) },
  side:  { dir: new THREE.Vector3(1, 0, 0),  up: new THREE.Vector3(0, 1, 0) },
};
export function snapView(view) {
  const v = VIEWS[view] || VIEWS.iso;
  snapToDirection(v.dir, v.up);
}

// ── Resize ──────────────────────────────────────────────────
export function onResize() {
  const w = innerWidth, h = innerHeight;
  perspCamera.aspect = w / h;
  perspCamera.updateProjectionMatrix();
  if (State.orthoOn) updateOrtho();
  renderer.setSize(w, h);
  labelRenderer.setSize(w, h);
  orbit.handleResize();               // TrackballControls caches the viewport rect
  State.requestRender();
}
addEventListener('resize', onResize);

export { scene, orbit, updateOrtho };
