// ============================================================
// js/scene.js — Three.js scene setup, camera, renderer,
//               labelRenderer, lights, ground, transform
//               controls, nav gizmo
// ============================================================
import * as THREE from 'three';
import { OrbitControls }    from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { CSS2DRenderer }    from 'three/addons/renderers/CSS2DRenderer.js';

import * as State from './state.js';

// ============================================================
// Scene, camera, renderer, labelRenderer
// ============================================================
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x2a2a3a);

const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.001, 50);
camera.position.set(0.45, 0.30, 0.40);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(devicePixelRatio);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.4;
document.body.appendChild(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.setSize(innerWidth, innerHeight);
labelRenderer.domElement.style.position = 'absolute';
labelRenderer.domElement.style.top = '0';
labelRenderer.domElement.style.pointerEvents = 'none';
document.body.appendChild(labelRenderer.domElement);

const orbitCtrl = new OrbitControls(camera, renderer.domElement);
orbitCtrl.target.set(0, 0.15, 0);
orbitCtrl.enableDamping = true;
orbitCtrl.dampingFactor = 0.08;
orbitCtrl.update();

// Orthographic camera (created once, activated on toggle)
const orthoCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.001, 50);

// Transform controls — need references to STL state for drag handler;
// that reference is injected by main.js after both modules load.
const transformCtrl = new TransformControls(camera, renderer.domElement);
transformCtrl.setSize(0.5);
transformCtrl.setMode('translate');
transformCtrl.addEventListener('dragging-changed', (e) => {
  orbitCtrl.enabled = !e.value;
});
scene.add(transformCtrl);

const stlTransformCtrl = new TransformControls(camera, renderer.domElement);
stlTransformCtrl.setSize(0.4);
stlTransformCtrl.setMode('translate');

const _scaleOnDragStart = new THREE.Vector3();

stlTransformCtrl.addEventListener('mouseDown', () => {
  if (State.selectedSTL) _scaleOnDragStart.copy(State.selectedSTL.mesh.scale);
});

stlTransformCtrl.addEventListener('objectChange', () => {
  if (!State.lockAspect || !State.selectedSTL || stlTransformCtrl.getMode() !== 'scale') return;
  const s = State.selectedSTL.mesh.scale;
  const s0 = _scaleOnDragStart;
  const rx = s0.x !== 0 ? s.x / s0.x : 1;
  const ry = s0.y !== 0 ? s.y / s0.y : 1;
  const rz = s0.z !== 0 ? s.z / s0.z : 1;
  const dx = Math.abs(rx - 1), dy = Math.abs(ry - 1), dz = Math.abs(rz - 1);
  const ratio = dx >= dy && dx >= dz ? rx : (dy >= dz ? ry : rz);
  s.set(s0.x * ratio, s0.y * ratio, s0.z * ratio);
});

stlTransformCtrl.addEventListener('dragging-changed', (e) => {
  orbitCtrl.enabled = !e.value;
});
scene.add(stlTransformCtrl);

// Device origin TransformControls
const deviceTransformCtrl = new TransformControls(camera, renderer.domElement);
deviceTransformCtrl.setSize(0.6);
deviceTransformCtrl.setMode('translate');
deviceTransformCtrl.addEventListener('dragging-changed', (e) => {
  orbitCtrl.enabled = !e.value;
});
scene.add(deviceTransformCtrl);

// Publish all objects into shared state
State.initCoreObjects(scene, camera, renderer, labelRenderer);
State.initCameras(orthoCamera, camera);
State.initControls(orbitCtrl, transformCtrl, stlTransformCtrl, deviceTransformCtrl);

// ============================================================
// Orthographic frustum helper (synchronous, uses State bindings)
// ============================================================
export function updateOrthoFrustum() {
  const dist = State.orthoCamera.position.distanceTo(State.orbitControls.target);
  const halfH = Math.tan(State.camera.fov * Math.PI / 360) * dist;
  const aspect = innerWidth / innerHeight;
  State.orthoCamera.left = -halfH * aspect; State.orthoCamera.right = halfH * aspect;
  State.orthoCamera.top  =  halfH;          State.orthoCamera.bottom = -halfH;
  State.orthoCamera.updateProjectionMatrix();
}

// ============================================================
// setOrtho — exported so main.js can wire the button
// ============================================================
export function setOrtho(on) {
  State.setOrthoOn(on);
  document.getElementById('orthoBtn').textContent = `Ortho: ${on ? 'ON' : 'OFF'}`;
  document.getElementById('orthoBtn').classList.toggle('active', on);
  if (on) {
    State.orthoCamera.position.copy(State.camera.position);
    State.orthoCamera.quaternion.copy(State.camera.quaternion);
    State.orthoCamera.up.copy(State.camera.up);
    updateOrthoFrustum();
    State.setActiveCamera(State.orthoCamera);
  } else {
    State.camera.position.copy(State.orthoCamera.position);
    State.camera.quaternion.copy(State.orthoCamera.quaternion);
    State.camera.up.copy(State.orthoCamera.up);
    State.setActiveCamera(State.camera);
  }
  State.orbitControls.object = State.activeCamera;
  State.transformControls.camera = State.activeCamera;
  State.stlTransformControls.camera = State.activeCamera;
  State.deviceTransformControls.camera = State.activeCamera;
  State.orbitControls.update();
}

// ============================================================
// Lighting
// ============================================================
const sun = new THREE.DirectionalLight(0xffffff, 19.2);
sun.position.set(3, 5, 2);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.near = 0.01;
sun.shadow.camera.far = 8;
sun.shadow.camera.left = sun.shadow.camera.bottom = -0.5;
sun.shadow.camera.right = sun.shadow.camera.top = 0.5;
scene.add(sun);

const fill = new THREE.DirectionalLight(0xffffff, 0.8);
fill.position.set(-0.886, 1.022, 0.668);
scene.add(fill);

const spot = new THREE.SpotLight(0x0058ff, 10, 3, Math.PI / 4, 0.5);
spot.position.set(-0.134, 1.043, 1.665);
scene.add(spot);

const spot2 = new THREE.SpotLight(0x0058ff, 33.75, 3, Math.PI / 4, 0.5);
spot2.position.set(-1.665, 1.0, -0.134);
spot2.target.position.set(0, 0.5, 0);
scene.add(spot2);
scene.add(spot2.target);

const spot3 = new THREE.SpotLight(0x0058ff, 33.75, 3, Math.PI / 4, 0.5);
spot3.position.set(1.665, 1.0, 0.134);
spot3.target.position.set(0, 0.5, 0);
scene.add(spot3);
scene.add(spot3.target);

scene.add(new THREE.AmbientLight(0x404060, 0.7));
scene.add(new THREE.HemisphereLight(0x8888bb, 0x333344, 0.4));

// ============================================================
// Ground + grid
// ============================================================
const groundMat = new THREE.MeshStandardMaterial({ color: 0x1a1a2e, roughness: 0.9 });
const ground = new THREE.Mesh(new THREE.PlaneGeometry(4, 4), groundMat);
ground.rotation.x = -Math.PI / 2;
ground.position.y = 0;
ground.receiveShadow = true;
scene.add(ground);
scene.add(new THREE.GridHelper(2, 40, 0x333355, 0x222244));

export function setFloorSize(radius) {
  const size = radius * 2;
  ground.geometry.dispose();
  ground.geometry = new THREE.PlaneGeometry(size * 2, size * 2);
}

// ============================================================
// Navigation gizmo (Blender-style, Canvas 2D)
// ============================================================
export const navCanvas = document.getElementById('nav-gizmo');
const NAV_SIZE = 110;
navCanvas.width  = Math.round(NAV_SIZE * devicePixelRatio);
navCanvas.height = Math.round(NAV_SIZE * devicePixelRatio);
const navCtx = navCanvas.getContext('2d');
navCtx.scale(devicePixelRatio, devicePixelRatio);

// Axis definitions  [id, world-dir, sphere-color, label, snap-up-vector]
export const NAV_AXES = [
  { id:'x+', dir:new THREE.Vector3( 1, 0, 0), color:'#ff4455', label:'X', r:11, up:new THREE.Vector3(0,1,0) },
  { id:'y+', dir:new THREE.Vector3( 0, 1, 0), color:'#4499ff', label:'Z', r:11, up:new THREE.Vector3(0,0,-1) },
  { id:'z+', dir:new THREE.Vector3( 0, 0, 1), color:'#55dd55', label:'Y', r:11, up:new THREE.Vector3(0,1,0) },
  { id:'x-', dir:new THREE.Vector3(-1, 0, 0), color:'#882233', label:'',  r: 8, up:new THREE.Vector3(0,1,0) },
  { id:'y-', dir:new THREE.Vector3( 0,-1, 0), color:'#224488', label:'',  r: 8, up:new THREE.Vector3(0,0,1)  },
  { id:'z-', dir:new THREE.Vector3( 0, 0,-1), color:'#226622', label:'',  r: 8, up:new THREE.Vector3(0,1,0) },
];

const _navQ = new THREE.Quaternion();
const _navV = new THREE.Vector3();
export function navProject(dir) {
  _navQ.copy(State.activeCamera.quaternion).invert();
  _navV.copy(dir).applyQuaternion(_navQ);
  const cx = NAV_SIZE / 2, cy = NAV_SIZE / 2;
  const scale = NAV_SIZE / 2 - 16;
  return { x: cx + _navV.x * scale, y: cy - _navV.y * scale, z: _navV.z };
}

export let navHovered = null;
export function setNavHovered(v) { navHovered = v; }

function hexAlpha(hex, alpha) {
  const r = parseInt(hex.slice(1,3),16);
  const g = parseInt(hex.slice(3,5),16);
  const b = parseInt(hex.slice(5,7),16);
  return `rgba(${r},${g},${b},${alpha.toFixed(2)})`;
}

export function renderNavGizmo() {
  const ctx = navCtx;
  const S = NAV_SIZE;
  ctx.clearRect(0, 0, S, S);

  ctx.save();
  ctx.beginPath();
  ctx.arc(S/2, S/2, S/2 - 1, 0, Math.PI*2);
  ctx.fillStyle = 'rgba(20,20,40,0.55)';
  ctx.fill();
  ctx.restore();

  const items = NAV_AXES.map(ax => ({ ax, ...navProject(ax.dir) }));
  items.sort((a, b) => a.z - b.z);

  for (const item of items) {
    if (!item.ax.id.endsWith('+')) continue;
    const neg = items.find(i => i.ax.id === item.ax.id.replace('+','-'));
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
    const alpha = 0.45 + 0.55 * ((z + 1) / 2);
    const hovered = navHovered === ax.id;
    const r = ax.r + (hovered ? 2 : 0);

    const grad = ctx.createRadialGradient(x - r*0.3, y - r*0.3, r*0.1, x, y, r);
    const base = hexAlpha(ax.color, alpha);
    const bright = hexAlpha(ax.color, Math.min(1, alpha + 0.3));
    grad.addColorStop(0, bright);
    grad.addColorStop(1, base);
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI*2);
    ctx.fillStyle = grad;
    ctx.fill();

    if (hovered) {
      ctx.beginPath();
      ctx.arc(x, y, r + 1, 0, Math.PI*2);
      ctx.strokeStyle = `rgba(255,255,255,0.7)`;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    if (ax.label) {
      ctx.fillStyle = `rgba(255,255,255,${Math.min(1, alpha + 0.1)})`;
      ctx.font = `bold ${r - 1}px sans-serif`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(ax.label, x, y + 0.5);
    }
  }
}

export function navHitTest(mx, my) {
  const items = NAV_AXES.map(ax => ({ ax, ...navProject(ax.dir) }));
  items.sort((a, b) => b.z - a.z);
  for (const item of items) {
    const dx = mx - item.x, dy = my - item.y;
    if (Math.sqrt(dx*dx + dy*dy) <= item.ax.r + 3) return item.ax.id;
  }
  return null;
}

// Snap animation state
export let snapAnim = null;
export function setSnapAnim(v) { snapAnim = v; }
export const snapClock = new THREE.Clock();
export function easeNavSnap(t) { return t*t*(3-2*t); }
