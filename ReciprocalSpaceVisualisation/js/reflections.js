// ============================================================
// js/reflections.js — build the reciprocal-space peak cloud
// ------------------------------------------------------------
// Takes a generator result ({positions, qmag, intensity, meta})
// from crystal.js or quasicrystal.js and produces one THREE.Points
// object with per-point size and colour buffer attributes. A round
// sprite texture makes the peaks read as spots rather than squares.
// ============================================================
import * as THREE from 'three';
import * as State from './state.js';
import { generateCrystal } from './crystal.js';
import { generateQuasicrystal } from './quasicrystal.js';
import { buildBasis, buildAxes, scene } from './scene.js';

let pointsObj = null;
let spriteTex = null;

function roundSprite() {
  if (spriteTex) return spriteTex;
  const s = 64;
  const cv = document.createElement('canvas');
  cv.width = cv.height = s;
  const ctx = cv.getContext('2d');
  const g = ctx.createRadialGradient(s / 2, s / 2, 0, s / 2, s / 2, s / 2);
  g.addColorStop(0.0, 'rgba(255,255,255,1)');
  g.addColorStop(0.6, 'rgba(255,255,255,0.85)');
  g.addColorStop(1.0, 'rgba(255,255,255,0)');
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, s, s);
  spriteTex = new THREE.CanvasTexture(cv);
  return spriteTex;
}

// ── Colormaps ───────────────────────────────────────────────
// Perceptual-ish viridis approximation (t in [0,1] → [r,g,b] 0..1).
const VIRIDIS = [
  [0.267, 0.005, 0.329], [0.283, 0.141, 0.458], [0.254, 0.265, 0.530],
  [0.207, 0.372, 0.553], [0.164, 0.471, 0.558], [0.128, 0.567, 0.551],
  [0.135, 0.659, 0.518], [0.267, 0.749, 0.441], [0.478, 0.821, 0.318],
  [0.741, 0.873, 0.150], [0.993, 0.906, 0.144],
];
function viridis(t) {
  t = Math.min(1, Math.max(0, t));
  const x = t * (VIRIDIS.length - 1);
  const i = Math.floor(x), f = x - i;
  const a = VIRIDIS[i], b = VIRIDIS[Math.min(i + 1, VIRIDIS.length - 1)];
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}

function colourForPoint(colourBy, qm, I, qMaxSeen, pos, i) {
  if (colourBy === 'intensity') return viridis(I);
  if (colourBy === 'axis') {
    // Colour by dominant reciprocal axis (which |component| is largest).
    const x = Math.abs(pos[i * 3]), y = Math.abs(pos[i * 3 + 1]), z = Math.abs(pos[i * 3 + 2]);
    if (x >= y && x >= z) return [1.0, 0.35, 0.4];
    if (y >= z) return [0.35, 0.85, 0.4];
    return [0.35, 0.55, 1.0];
  }
  // default: qmag → viridis by resolution
  return viridis(qMaxSeen > 0 ? qm / qMaxSeen : 0);
}

// ── Build / rebuild the cloud from the current params ───────
export function rebuild() {
  const p = State.params;
  const data = p.mode === 'ico' ? generateQuasicrystal(p) : generateCrystal(p);

  if (pointsObj) {
    scene.remove(pointsObj);
    pointsObj.geometry.dispose();
    pointsObj.material.dispose();
    pointsObj = null;
  }

  const n = data.qmag.length;
  const colors = new Float32Array(n * 3);
  const sizes = new Float32Array(n);
  let qMaxSeen = 0;
  for (let i = 0; i < n; i++) qMaxSeen = Math.max(qMaxSeen, data.qmag[i]);

  for (let i = 0; i < n; i++) {
    const c = colourForPoint(p.colourBy, data.qmag[i], data.intensity[i], qMaxSeen, data.positions, i);
    colors[i * 3] = c[0]; colors[i * 3 + 1] = c[1]; colors[i * 3 + 2] = c[2];
    // World-space peak diameter (Å⁻¹): weak peaks stay visible, strong ones
    // dominate. Converted to pixels per-point in the vertex shader.
    sizes[i] = (0.02 + 0.055 * Math.sqrt(data.intensity[i])) * p.pointScale;
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(data.positions, 3));
  geo.setAttribute('customColor', new THREE.BufferAttribute(colors, 3));
  geo.setAttribute('size', new THREE.BufferAttribute(sizes, 1));

  // Pixels-per-world-unit-at-unit-depth: height / (2·tan(fov/2)). Points sized
  // by world diameter then divided by view depth → correct perspective scaling,
  // clamped so near peaks can't blow up to fill the screen.
  const cam = State.perspCamera;
  const uScale = State.renderer.domElement.height /
                 (2 * Math.tan((cam.fov * Math.PI) / 360));

  const mat = new THREE.ShaderMaterial({
    uniforms: { map: { value: roundSprite() }, uScale: { value: uScale } },
    // The clipping chunks (and `clipping: true`, which a ShaderMaterial needs
    // for the uniforms to be injected) let the visibility box cut the cloud the
    // same way it cuts every stock material — see js/visbox.js. The chunk reads
    // the view-space position from a variable named `mvPosition`.
    vertexShader: `
      #include <clipping_planes_pars_vertex>
      attribute float size;
      attribute vec3 customColor;
      uniform float uScale;
      varying vec3 vColor;
      void main() {
        vColor = customColor;
        vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
        gl_PointSize = clamp(size * uScale / max(-mvPosition.z, 0.05), 1.5, 90.0);
        gl_Position = projectionMatrix * mvPosition;
        #include <clipping_planes_vertex>
      }`,
    fragmentShader: `
      #include <clipping_planes_pars_fragment>
      uniform sampler2D map;
      varying vec3 vColor;
      void main() {
        #include <clipping_planes_fragment>
        vec4 tex = texture2D(map, gl_PointCoord);
        if (tex.a < 0.25) discard;
        gl_FragColor = vec4(vColor, tex.a);
      }`,
    clipping: true,
    transparent: true,
    depthWrite: true,
    depthTest: true,
  });

  pointsObj = new THREE.Points(geo, mat);
  scene.add(pointsObj);

  // Basis arrows + axes sized to the data.
  buildBasis(p.showBasis ? data.basis : null);
  const ext = Math.max(qMaxSeen, p.qMax) * 1.05 || 3;
  buildAxes(ext);

  State.setPeakMeta(data.meta);
  State.requestRender();
  dispatchEvent(new CustomEvent('peaksUpdated', { detail: n }));
  return n;
}

export function getPoints() { return pointsObj; }

// Keep world→pixel point scaling correct when the viewport resizes.
addEventListener('resize', () => {
  if (!pointsObj) return;
  const cam = State.perspCamera;
  pointsObj.material.uniforms.uScale.value =
    State.renderer.domElement.height / (2 * Math.tan((cam.fov * Math.PI) / 360));
  State.requestRender();
});
