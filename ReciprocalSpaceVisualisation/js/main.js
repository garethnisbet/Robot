// ============================================================
// js/main.js — init, animate loop, nav gizmo, peak hover
// ============================================================
import * as THREE from 'three';
import * as State from './state.js';
import { scene, orbit } from './scene.js';
import { rebuild, getPoints } from './reflections.js';
import { buildPanel, setCount } from './panel.js';
import { syncClip, isPointVisible } from './visbox.js';
import { snapView, navCanvas, NAV_AXES, navHitTest, navHovered, setNavHovered,
         renderNavGizmo, snapToDirection, stepSnap } from './scene.js';

// ── Peak count readout ──────────────────────────────────────
addEventListener('peaksUpdated', (e) => setCount(e.detail));

// ── Build UI + initial cloud ────────────────────────────────
buildPanel();
rebuild();
const query = new URLSearchParams(location.search);

// Optional camera in the URL, so a particular view can be linked or scripted:
//   ?view=iso|top|front|side   ?cam=x,y,z   ?dist=<camera distance>
if (query.has('cam')) {
  const c = query.get('cam').split(',').map(Number);
  if (c.length === 3 && c.every(isFinite) && Math.hypot(...c) > 0) {
    const d = State.camera.position.length();
    State.camera.position.set(c[0], c[1], c[2]).setLength(d);
    State.camera.lookAt(orbit.target);
  }
}
if (query.has('dist')) {
  const d = parseFloat(query.get('dist'));
  if (isFinite(d) && d > 0) State.camera.position.setLength(d);
}
if (query.has('view')) snapView(query.get('view'));
State.requestRender();
const loadingEl = document.getElementById('loading');
if (loadingEl) loadingEl.style.display = 'none';

// ── Peak hover tooltip (raycast against the Points cloud) ───
const raycaster = new THREE.Raycaster();
raycaster.params.Points.threshold = 0.04;
const mouse = new THREE.Vector2();
const tip = document.getElementById('peak-tip');
let hoverPending = false, lastEvent = null;

addEventListener('mousemove', (e) => { lastEvent = e; hoverPending = true; State.requestRender(); });

function updateHover() {
  if (!lastEvent) return;
  const pts = getPoints();
  if (!pts) { tip.style.display = 'none'; return; }
  mouse.x = (lastEvent.clientX / innerWidth) * 2 - 1;
  mouse.y = -(lastEvent.clientY / innerHeight) * 2 + 1;
  // scale threshold with view distance so picking stays usable when zoomed out
  raycaster.params.Points.threshold = 0.01 * State.camera.position.distanceTo(orbit.target);
  raycaster.setFromCamera(mouse, State.camera);
  // A peak hidden by the visibility box must not be pickable either, or the
  // tooltip reports indices for something that isn't on screen.
  const hits = raycaster.intersectObject(pts, false).filter((h) => isPointVisible(h.point));
  if (hits.length) {
    const idx = hits[0].index;
    const m = State.peakMeta[idx];
    if (m) {
      const label = m.hkl ? `hkl [${m.hkl.join(' ')}]`
                          : `n [${m.n.join(' ')}]`;
      tip.innerHTML = `${label}<br><span class="qval">|Q| = ${m.q.toFixed(3)} Å⁻¹</span>`
        + (m.qperp != null ? `<br><span class="qval">|Q⊥| = ${m.qperp.toFixed(3)}</span>` : '');
      tip.style.left = (lastEvent.clientX + 14) + 'px';
      tip.style.top = (lastEvent.clientY + 12) + 'px';
      tip.style.display = 'block';
    }
  } else {
    tip.style.display = 'none';
  }
}

// ── Navigation gizmo mouse events ───────────────────────────
// Same interaction as the RobotVisualisation viewer: hover highlights a ball,
// clicking one snaps the camera onto that axis with a short eased move.
navCanvas.addEventListener('mousemove', (e) => {
  const rect = navCanvas.getBoundingClientRect();
  const id = navHitTest(e.clientX - rect.left, e.clientY - rect.top);
  if (id !== navHovered) {
    setNavHovered(id);
    navCanvas.style.cursor = id ? 'pointer' : 'default';
    State.requestRender();
  }
});
navCanvas.addEventListener('mouseleave', () => {
  setNavHovered(null);
  navCanvas.style.cursor = 'default';
  State.requestRender();
});
navCanvas.addEventListener('click', (e) => {
  const rect = navCanvas.getBoundingClientRect();
  const id = navHitTest(e.clientX - rect.left, e.clientY - rect.top);
  if (!id) return;
  const ax = NAV_AXES.find((a) => a.id === id);
  snapToDirection(ax.dir, ax.up);
});

// ── Render loop (on-demand) ─────────────────────────────────
function animate() {
  requestAnimationFrame(animate);
  // The camera controls emit 'change' during damping inertia, which calls
  // requestRender(); update() runs every frame, but we draw only when flagged.
  // A snap animation drives the camera directly, so the controls are disabled
  // for its duration (stepSnap re-enables them) and damping resumes afterwards.
  stepSnap();
  orbit.update();
  if (hoverPending) { updateHover(); hoverPending = false; }
  if (State.needsRender) {
    // The curves, trail and peak cloud are rebuilt constantly; this hands the
    // visibility box's clip planes to any material made since the last frame.
    syncClip();
    State.renderer.render(scene, State.camera);
    State.labelRenderer.render(scene, State.camera);
    renderNavGizmo();
    State.clearNeedsRender();
  }
}
animate();
