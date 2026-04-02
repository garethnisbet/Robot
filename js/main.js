// ============================================================
// js/main.js — animate loop, event handlers, initialisation
// ============================================================
import * as THREE from 'three';
import { MeshBVH, acceleratedRaycast } from 'three-mesh-bvh';

// Patch Three.js Mesh to use BVH-accelerated raycasting
THREE.Mesh.prototype.raycast = acceleratedRaycast;

// ============================================================
// Constants
// ============================================================
const deg2rad = Math.PI / 180;
const rad2deg = 180 / Math.PI;

// ============================================================
// Modules — scene must be imported first (it writes to State)
// ============================================================
import * as State from './state.js';
import {
  setOrtho, updateOrthoFrustum,
  navCanvas, NAV_AXES, navHitTest,
  navHovered, setNavHovered, renderNavGizmo,
  snapAnim, setSnapAnim, snapClock, easeNavSnap,
  setFloorSize,
} from './scene.js';
import {
  updateFK, getEEWorldPosition, getEEWorldQuaternion,
  updateChain, solveIK,
} from './kinematics.js';
import {
  loadDevice,
  updateSliders, setIKMode, syncIKSliders,
} from './device.js';
import {
  buildControlPanel, rebuildDeviceList,
  setActiveDevice, findDeviceForObject,
  setDeviceParent, rebuildDeviceParentDropdown,
  rebuildPrimaryModelDropdown,
} from './panel.js';
import {
  buildScenePayload, exportSceneState, importSceneState, restoreSTLsFromState,
  loadSTLFile, loadOBJFile, loadPLYFile, loadGLBFile,
  addPrimitive,
  selectSTL, deselectSTL, setSTLTransformMode, setSTLParent,
} from './stl.js';
import { checkCollisions, clearCollisionHighlights, initCollisionWorker } from './collision.js';
import {
  wsConnect, initWsInfoPanel, registerSetActiveDevice, registerAvailableConfigs,
} from './websocket.js';

// Register callbacks for websocket.js
// (avoids circular dependency: websocket -> panel -> websocket)
import { configFiles } from './panel.js';
registerSetActiveDevice(setActiveDevice);
registerAvailableConfigs(configFiles);

// Start collision Web Worker (falls back to main thread if unavailable)
initCollisionWorker();

// ============================================================
// Raycaster (for click-to-select)
// ============================================================
const raycaster = new THREE.Raycaster();
raycaster.params.Points = { threshold: 0.005 };
const mouse = new THREE.Vector2();
const _originWP = new THREE.Vector3();

// ============================================================
// fmtV helper
// ============================================================
function fmtV(v) {
  return `(${(v.x*1000).toFixed(1)}, ${(v.z*1000).toFixed(1)}, ${(v.y*1000).toFixed(1)})mm`;
}

// ============================================================
// Animate loop
// ============================================================
const eePosEl = document.getElementById('eePos');
const tgtPosEl = document.getElementById('tgtPos');
const ikErrEl  = document.getElementById('ikErr');

function animate() {
  requestAnimationFrame(animate);

  if (State.activeDevice) {
    if (State.activeDevice.ikMode) {
      const err = solveIK(State.activeDevice, State.activeDevice.ikTarget.position, State.activeDevice.ikTargetQuat, 10, 0.00005);
      updateSliders(State.activeDevice);

      const eePos = getEEWorldPosition(State.activeDevice);
      const pts = State.activeDevice.ikLine.geometry.attributes.position;
      pts.setXYZ(0, eePos.x, eePos.y, eePos.z);
      pts.setXYZ(1, State.activeDevice.ikTarget.position.x, State.activeDevice.ikTarget.position.y, State.activeDevice.ikTarget.position.z);
      pts.needsUpdate = true;

      eePosEl.textContent  = fmtV(eePos);
      tgtPosEl.textContent = fmtV(State.activeDevice.ikTarget.position);
      ikErrEl.textContent  = (err * 1000).toFixed(2) + 'mm';

      syncIKSliders(State.activeDevice);
    } else {
      const eePos = getEEWorldPosition(State.activeDevice);
      eePosEl.textContent  = fmtV(eePos);
      tgtPosEl.textContent = '-';
      ikErrEl.textContent  = '-';
    }
  }

  // Chain visualization for all devices
  for (const dev of State.devices) {
    if (dev.chainVisible) updateChain(dev);
  }

  // Origin coordinate labels
  if (State.originsOn) {
    for (const dev of State.devices) {
      dev.rootGroup.getWorldPosition(_originWP);
      const x = +(_originWP.x * 1000).toFixed(1);
      const y = +(_originWP.z * 1000).toFixed(1);
      const z = +(_originWP.y * 1000).toFixed(1);
      dev.originLabels[0].element.textContent = `${dev.name} ${x}, ${y}, ${z}`;
    }
  }

  checkCollisions();

  // Camera snap animation
  const currentSnapAnim = snapAnim; // read live export
  if (currentSnapAnim) {
    currentSnapAnim.t = Math.min(1, currentSnapAnim.t + snapClock.getDelta() / 0.4);
    const t = easeNavSnap(currentSnapAnim.t);
    State.activeCamera.position.lerpVectors(currentSnapAnim.sp, currentSnapAnim.ep, t);
    State.activeCamera.up.lerpVectors(currentSnapAnim.su, currentSnapAnim.eu, t).normalize();
    if (currentSnapAnim.t >= 1) {
      setSnapAnim(null); State.orbitControls.enabled = true;
      if (State.orthoOn) updateOrthoFrustum();
    }
  } else {
    snapClock.getDelta();
  }

  State.orbitControls.update();
  State.renderer.render(State.scene, State.activeCamera);
  State.labelRenderer.render(State.scene, State.activeCamera);
  renderNavGizmo();
}

// ============================================================
// Button event handlers
// ============================================================

document.getElementById('resetBtn').addEventListener('click', () => {
  if (!State.activeDevice) return;
  for (let i = 0; i < State.activeDevice.numJoints; i++) State.activeDevice.jointAngles[i] = 0;
  updateFK(State.activeDevice);
  updateSliders(State.activeDevice);
  if (State.activeDevice.ikMode) {
    State.scene.updateMatrixWorld(true);
    State.activeDevice.ikTarget.position.copy(getEEWorldPosition(State.activeDevice));
    State.activeDevice.ikTargetQuat.copy(getEEWorldQuaternion(State.activeDevice));
    State.activeDevice.ikTargetEuler.setFromQuaternion(State.activeDevice.ikTargetQuat, 'YZX');
    State.activeDevice.ikTarget.quaternion.copy(State.activeDevice.ikTargetQuat);
    syncIKSliders(State.activeDevice);
  }
});

document.getElementById('demoBtn').addEventListener('click', () => {
  if (!State.activeDevice) return;
  if (State.activeDevice.isKappaGeometry) {
    for (let i = 0; i < State.activeDevice.numJoints; i++) State.activeDevice.jointAngles[i] = 0;
    State.activeDevice.jointAngles[State.activeDevice.kappaJointIdx] = -134.6 * deg2rad;
    State.activeDevice.jointAngles[State.activeDevice.thetaJointIdx] = -33.5 * deg2rad;
    State.activeDevice.jointAngles[State.activeDevice.phiJointIdx]   = -146.9 * deg2rad;
  } else if (State.activeDevice.config.demoPose) {
    const pose = State.activeDevice.config.demoPose;
    for (let i = 0; i < State.activeDevice.numJoints && i < pose.length; i++) {
      State.activeDevice.jointAngles[i] = pose[i] * deg2rad;
    }
  }
  updateFK(State.activeDevice);
  updateSliders(State.activeDevice);
  if (State.activeDevice.ikMode) {
    State.scene.updateMatrixWorld(true);
    State.activeDevice.ikTarget.position.copy(getEEWorldPosition(State.activeDevice));
    State.activeDevice.ikTargetQuat.copy(getEEWorldQuaternion(State.activeDevice));
    State.activeDevice.ikTargetEuler.setFromQuaternion(State.activeDevice.ikTargetQuat, 'YZX');
    State.activeDevice.ikTarget.quaternion.copy(State.activeDevice.ikTargetQuat);
    syncIKSliders(State.activeDevice);
  }
});

document.getElementById('ikBtn').addEventListener('click', () => {
  if (!State.activeDevice || State.activeDevice.isBranching) return;
  setIKMode(State.activeDevice, !State.activeDevice.ikMode);
});

document.getElementById('chainBtn').addEventListener('click', () => {
  if (!State.activeDevice) return;
  State.activeDevice.chainVisible = !State.activeDevice.chainVisible;
  document.getElementById('chainBtn').textContent = `Chain: ${State.activeDevice.chainVisible ? 'ON' : 'OFF'}`;
  document.getElementById('chainBtn').classList.toggle('active', State.activeDevice.chainVisible);
  State.activeDevice.chainLine.visible = State.activeDevice.chainVisible;
  State.activeDevice.chainSpheres.forEach(s => s.visible = State.activeDevice.chainVisible);
  if (State.activeDevice.chainVisible) updateChain(State.activeDevice);
});

document.getElementById('orthoBtn').addEventListener('click', () => setOrtho(!State.orthoOn));

document.getElementById('originsBtn').addEventListener('click', () => {
  State.setOriginsOn(!State.originsOn);
  document.getElementById('originsBtn').textContent = `Origins: ${State.originsOn ? 'ON' : 'OFF'}`;
  document.getElementById('originsBtn').classList.toggle('active', State.originsOn);
  for (const dev of State.devices) {
    dev.originHelpers.forEach(h => h.visible = State.originsOn);
    dev.originLabels.forEach(l => l.visible = State.originsOn);
  }
});

document.getElementById('labelBtn').addEventListener('click', () => {
  State.setLabelsOn(!State.labelsOn);
  document.getElementById('labelBtn').textContent = `Labels: ${State.labelsOn ? 'ON' : 'OFF'}`;
  document.getElementById('labelBtn').classList.toggle('active', State.labelsOn);
  for (const dev of State.devices) {
    dev.meshLabels.forEach(l => l.visible = State.labelsOn);
  }
});

// IK target position sliders
['ikx', 'iky', 'ikz'].forEach((id) => {
  document.getElementById(id).addEventListener('input', (e) => {
    if (!State.activeDevice) return;
    const mm = parseFloat(e.target.value);
    document.getElementById(id.replace('ik', 'ikv')).textContent = Math.round(mm);
    const sliderAxis = id.charAt(2);
    const threeAxis = sliderAxis === 'y' ? 'z' : sliderAxis === 'z' ? 'y' : 'x';
    State.activeDevice.ikTarget.position[threeAxis] = mm / 1000;
  });
});

// IK target orientation sliders
['ika', 'ikb', 'ikc'].forEach((id) => {
  document.getElementById(id).addEventListener('input', (e) => {
    if (!State.activeDevice) return;
    const deg = parseFloat(e.target.value);
    document.getElementById(id.replace('ik', 'ikv')).textContent = Math.round(deg);
    const comp = id.charAt(2);
    const axis = comp === 'a' ? 'y' : comp === 'b' ? 'z' : 'x';
    State.activeDevice.ikTargetEuler[axis] = deg * deg2rad;
    State.activeDevice.ikTargetQuat.setFromEuler(State.activeDevice.ikTargetEuler);
    State.activeDevice.ikTarget.quaternion.copy(State.activeDevice.ikTargetQuat);
  });
});

// Move device origin button
document.getElementById('moveDeviceBtn').addEventListener('click', () => {
  if (!State.activeDevice) return;
  State.setMoveDeviceActive(!State.moveDeviceActive);
  const btn = document.getElementById('moveDeviceBtn');
  btn.textContent = State.moveDeviceActive ? 'Stop Moving Origin' : 'Move Device Origin';
  btn.classList.toggle('active', State.moveDeviceActive);
  document.getElementById('device-mode').style.display = State.moveDeviceActive ? 'block' : 'none';
  if (State.moveDeviceActive) {
    State.deviceTransformControls.attach(State.activeDevice.rootGroup);
  } else {
    State.deviceTransformControls.detach();
  }
});

// Device transform mode buttons
document.getElementById('devModeT').addEventListener('click', () => {
  State.deviceTransformControls.setMode('translate');
  document.getElementById('devModeT').classList.add('active');
  document.getElementById('devModeR').classList.remove('active');
});
document.getElementById('devModeR').addEventListener('click', () => {
  State.deviceTransformControls.setMode('rotate');
  document.getElementById('devModeR').classList.add('active');
  document.getElementById('devModeT').classList.remove('active');
});

// Device parent dropdown
document.getElementById('deviceParentSelect').addEventListener('change', (e) => {
  if (!State.activeDevice) return;
  setDeviceParent(State.activeDevice, e.target.value);
});

// Primary model selector
document.getElementById('primaryModelSelect').addEventListener('change', async (e) => {
  const configFile = e.target.value;
  if (!configFile) return;

  // Remove the current primary device (first in the list)
  const primaryDev = State.devices[0];
  if (primaryDev && primaryDev.configFile === configFile) return; // same model

  const select = e.target;
  select.disabled = true;
  try {
    document.getElementById('loading').style.display = 'block';
    document.getElementById('loading').textContent = 'Loading model...';

    // Load the new device first
    const dev = await loadDevice(configFile);

    // Remove old primary (allow removal even if it's the only device since we're replacing)
    if (primaryDev) {
      // Detach any active controls
      if (primaryDev === State.activeDevice) {
        State.transformControls.detach();
        State.deviceTransformControls.detach();
      }
      // Clean up scene objects
      State.scene.remove(primaryDev.rootGroup);
      primaryDev.rootGroup.traverse(child => {
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
          if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
          else child.material.dispose();
        }
      });
      for (const label of primaryDev.meshLabels) label.removeFromParent();
      if (primaryDev.chainLine) State.scene.remove(primaryDev.chainLine);
      for (const s of primaryDev.chainSpheres) State.scene.remove(s);
      if (primaryDev.ikTarget) State.scene.remove(primaryDev.ikTarget);
      if (primaryDev.ikLine) State.scene.remove(primaryDev.ikLine);
      const idx = State.devices.indexOf(primaryDev);
      if (idx >= 0) State.devices.splice(idx, 1);
    }

    // Insert new device at front as primary
    State.devices.unshift(dev);
    State.setActiveDevice(dev);
    updateFK(dev);
    buildControlPanel(dev);
    rebuildDeviceList();

    // Auto-fit camera to new primary
    const fitBox = new THREE.Box3();
    for (const grp of dev.jointRotGroups) {
      grp.updateWorldMatrix(true, true);
      grp.traverse((child) => { if (child.isMesh) fitBox.expandByObject(child); });
    }
    for (const mesh of dev.staticMeshes) {
      mesh.updateWorldMatrix(true, false);
      fitBox.expandByObject(mesh);
    }
    if (!fitBox.isEmpty()) {
      const fitCenter = fitBox.getCenter(new THREE.Vector3());
      const fitSize   = fitBox.getSize(new THREE.Vector3());
      const maxDim    = Math.max(fitSize.x, fitSize.y, fitSize.z);
      const fov       = State.camera.fov * (Math.PI / 180);
      const fitDist   = maxDim / (2 * Math.tan(fov / 2)) * 1.2;
      const direction = new THREE.Vector3(1, 0.6, 1).normalize();
      State.camera.position.copy(fitCenter).addScaledVector(direction, fitDist);
      State.orbitControls.target.copy(fitCenter);
      State.camera.updateProjectionMatrix();
      State.orbitControls.update();
    }

    document.getElementById('loading').style.display = 'none';
  } catch (err) {
    console.error('Failed to load primary model:', err);
    document.getElementById('loading').innerHTML =
      `<span style="color:#f88">Failed to load model</span><br>` +
      `<span style="color:#aaa; font-size:0.85em">${err?.message || err}</span>`;
    setTimeout(() => { document.getElementById('loading').style.display = 'none'; }, 3000);
  }
  select.disabled = false;
});

// Add device button
document.getElementById('addDeviceBtn').addEventListener('click', async () => {
  const select = document.getElementById('addDeviceSelect');
  const configFile = select.value;
  if (!configFile) return;

  const btn = document.getElementById('addDeviceBtn');
  btn.disabled = true;
  btn.textContent = '...';
  try {
    document.getElementById('loading').style.display = 'block';
    document.getElementById('loading').textContent = `Loading device...`;
    const dev = await loadDevice(configFile);
    State.devices.push(dev);
    updateFK(dev);
    setActiveDevice(dev);
    document.getElementById('loading').style.display = 'none';
  } catch (err) {
    console.error('Failed to load device:', err);
    document.getElementById('loading').innerHTML =
      `<span style="color:#f88">Failed to load device</span><br>` +
      `<span style="color:#aaa; font-size:0.85em">${err?.message || err}</span>`;
    setTimeout(() => { document.getElementById('loading').style.display = 'none'; }, 3000);
  }
  btn.disabled = false;
  btn.textContent = '+';
});

// STL import button
document.getElementById('stlBtn').addEventListener('click', () => {
  document.getElementById('stlFile').click();
});

document.getElementById('stlFile').addEventListener('change', (e) => {
  for (const file of e.target.files) {
    const ext = file.name.split('.').pop().toLowerCase();
    if (ext === 'stl')                   loadSTLFile(file);
    else if (ext === 'obj')              loadOBJFile(file);
    else if (ext === 'ply')              loadPLYFile(file);
    else if (ext === 'glb' || ext === 'gltf') loadGLBFile(file);
  }
  e.target.value = '';
});

// Primitive buttons
document.getElementById('addCubeBtn').addEventListener('click',     () => addPrimitive('cube'));
document.getElementById('addSphereBtn').addEventListener('click',   () => addPrimitive('sphere'));
document.getElementById('addCylinderBtn').addEventListener('click', () => addPrimitive('cylinder'));

// STL transform mode buttons
document.getElementById('stlModeT').addEventListener('click',  () => setSTLTransformMode('translate'));
document.getElementById('stlModeR').addEventListener('click',  () => setSTLTransformMode('rotate'));
document.getElementById('stlModeS').addEventListener('click',  () => setSTLTransformMode('scale'));
document.getElementById('stlDeselect').addEventListener('click', deselectSTL);

document.getElementById('lockAspectCb').addEventListener('change', (e) => {
  State.setLockAspect(e.target.checked);
});

document.getElementById('stlResetRot').addEventListener('click', () => {
  if (!State.selectedSTL) return;
  State.selectedSTL.mesh.rotation.set(0, 0, 0);
});

document.getElementById('stlResetScale').addEventListener('click', () => {
  if (!State.selectedSTL) return;
  State.selectedSTL.mesh.scale.set(1, 1, 1);
});

document.getElementById('stlParentSelect').addEventListener('change', (e) => {
  if (State.selectedSTL) setSTLParent(State.selectedSTL, e.target.value, false);
});

// Collision button
const collisionBtn    = document.getElementById('collisionBtn');
const collisionInfoEl = document.getElementById('collision-info');
collisionBtn.addEventListener('click', () => {
  State.setCollisionEnabled(!State.collisionEnabled);
  collisionBtn.textContent = `Collision: ${State.collisionEnabled ? 'ON' : 'OFF'}`;
  collisionBtn.classList.toggle('active', State.collisionEnabled);
  collisionInfoEl.style.display = State.collisionEnabled ? 'block' : 'none';
  if (!State.collisionEnabled) clearCollisionHighlights();
});

// Mesh-select toggle
const stlSelectBtn = document.getElementById('stlSelectBtn');
stlSelectBtn.classList.add('active');
stlSelectBtn.addEventListener('click', () => {
  State.setStlSelectable(!State.stlSelectable);
  stlSelectBtn.textContent = `Mesh Select: ${State.stlSelectable ? 'ON' : 'OFF'}`;
  stlSelectBtn.classList.toggle('active', State.stlSelectable);
});

// Floor size slider
document.getElementById('floorSize').addEventListener('input', (e) => {
  const r = parseFloat(e.target.value);
  document.getElementById('floorSizeVal').textContent = `${r} m`;
  setFloorSize(r);
});

// ============================================================
// Click-to-select STL meshes or activate devices
// ============================================================
State.renderer.domElement.addEventListener('pointerdown', (e) => {
  if (State.stlTransformControls.dragging || State.transformControls.dragging || State.deviceTransformControls.dragging) return;
  if (e.button !== 0) return;

  mouse.x = (e.clientX / innerWidth) * 2 - 1;
  mouse.y = -(e.clientY / innerHeight) * 2 + 1;
  raycaster.setFromCamera(mouse, State.activeCamera);

  // Test against all imported STL meshes first (if selection enabled)
  if (State.stlSelectable) {
    const stlMeshes = State.importedSTLs.filter(s => s.mesh.visible).map(s => s.mesh);
    const stlHits = raycaster.intersectObjects(stlMeshes, false);

    if (stlHits.length > 0) {
      const hitMesh = stlHits[0].object;
      const entry = State.importedSTLs.find(s => s.mesh === hitMesh);
      if (entry) {
        const listItems = document.querySelectorAll('.stl-item');
        const idx = State.importedSTLs.indexOf(entry);
        selectSTL(entry, listItems[idx] || null);
      }
      return;
    }
  }

  // Test against all device meshes
  const allDeviceMeshes = [];
  for (const dev of State.devices) {
    for (const link of dev.robotLinkMeshes) {
      for (const mesh of link.meshes) {
        allDeviceMeshes.push(mesh);
      }
    }
    for (const mesh of dev.staticMeshes) {
      allDeviceMeshes.push(mesh);
    }
  }

  const deviceHits = raycaster.intersectObjects(allDeviceMeshes, false);
  if (deviceHits.length > 0) {
    const hitDev = findDeviceForObject(deviceHits[0].object);
    if (hitDev && hitDev !== State.activeDevice) {
      setActiveDevice(hitDev);
    }
  }
});

// Click on empty space to deselect
State.renderer.domElement.addEventListener('click', (e) => {
  if (State.stlTransformControls.dragging || State.transformControls.dragging || State.deviceTransformControls.dragging) return;
  if (!State.selectedSTL) return;

  mouse.x = (e.clientX / innerWidth) * 2 - 1;
  mouse.y = -(e.clientY / innerHeight) * 2 + 1;
  raycaster.setFromCamera(mouse, State.activeCamera);

  const stlMeshes = State.importedSTLs.filter(s => s.mesh.visible).map(s => s.mesh);
  const hits = raycaster.intersectObjects(stlMeshes, false);
  if (hits.length === 0) {
    const gizmoHits = raycaster.intersectObjects(State.stlTransformControls.children, true);
    if (gizmoHits.length === 0) deselectSTL();
  }
});

// Keyboard shortcuts for STL transform
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

  if (State.selectedSTL) {
    if (e.key === 't' || e.key === 'T') {
      setSTLTransformMode('translate');
    } else if (e.key === 'r' || e.key === 'R') {
      setSTLTransformMode('rotate');
    } else if (e.key === 's' || e.key === 'S') {
      setSTLTransformMode('scale');
    } else if (e.key === 'Escape') {
      deselectSTL();
    }
  } else if (State.activeDevice && State.activeDevice.ikMode) {
    const tc = State.transformControls;
    if (e.key === 't' || e.key === 'T') {
      tc.setMode('translate');
      tc.showX = true; tc.showY = true; tc.showZ = true;
    } else if (e.key === 'r' || e.key === 'R') {
      // Cycle Rx → Ry → Rz → Rx
      if (tc.mode === 'rotate') {
        if (tc.showX && !tc.showY && !tc.showZ)      { tc.showX = false; tc.showY = true; }
        else if (!tc.showX && tc.showY && !tc.showZ)  { tc.showY = false; tc.showZ = true; }
        else                                           { tc.showX = true; tc.showY = false; tc.showZ = false; }
      } else {
        tc.setMode('rotate');
        tc.showX = true; tc.showY = false; tc.showZ = false;
      }
    }
  }
});

// ============================================================
// Navigation gizmo mouse events
// ============================================================
navCanvas.addEventListener('mousemove', (e) => {
  const rect = navCanvas.getBoundingClientRect();
  const id = navHitTest(e.clientX - rect.left, e.clientY - rect.top);
  if (id !== navHovered) {
    setNavHovered(id);
    navCanvas.style.cursor = id ? 'pointer' : 'default';
  }
});
navCanvas.addEventListener('mouseleave', () => { setNavHovered(null); navCanvas.style.cursor = 'default'; });

const _snapVec = new THREE.Vector3();

navCanvas.addEventListener('click', (e) => {
  const rect = navCanvas.getBoundingClientRect();
  const id = navHitTest(e.clientX - rect.left, e.clientY - rect.top);
  if (!id) return;
  const ax = NAV_AXES.find(a => a.id === id);
  const dist = State.activeCamera.position.distanceTo(State.orbitControls.target);
  setSnapAnim({
    sp: State.activeCamera.position.clone(),
    ep: State.orbitControls.target.clone().addScaledVector(ax.dir, dist),
    su: State.activeCamera.up.clone(),
    eu: ax.up.clone(),
    t: 0,
  });
  State.orbitControls.enabled = false;
});

// ============================================================
// Resize handler
// ============================================================
window.addEventListener('resize', () => {
  State.camera.aspect = innerWidth / innerHeight;
  State.camera.updateProjectionMatrix();
  if (State.orthoOn) updateOrthoFrustum();
  State.renderer.setPixelRatio(devicePixelRatio);
  State.renderer.setSize(innerWidth, innerHeight);
  State.labelRenderer.setSize(innerWidth, innerHeight);
});

// ============================================================
// Initialization
// ============================================================
const configParam = new URLSearchParams(window.location.search).get('config') || 'robot_config.json';

// Try restoring from localStorage first
const SCENE_STORAGE_KEY = 'robotvis_scene';
let restoredFromStorage = false;

try {
  const saved = localStorage.getItem(SCENE_STORAGE_KEY);
  if (saved) {
    const data = JSON.parse(saved);
    if (data.version && Array.isArray(data.devices) && data.devices.length > 0) {
      document.getElementById('loading').textContent = 'Restoring scene...';
      await restoreScene(data);
      restoredFromStorage = true;
      console.log('[Auto-restore] Scene restored from localStorage');
    }
  }
} catch (err) {
  console.warn('[Auto-restore] Failed, falling back to default:', err);
  localStorage.removeItem(SCENE_STORAGE_KEY);
  // Clean up any partial restore
  for (const dev of [...State.devices]) {
    State.transformControls.detach();
    State.deviceTransformControls.detach();
    State.scene.remove(dev.rootGroup);
  }
  State.devices.length = 0;
  State.resetDeviceIdCounter();
  restoredFromStorage = false;
}

if (!restoredFromStorage) {
  try {
    const initialDevice = await loadDevice(configParam);
    State.devices.push(initialDevice);
    State.setActiveDevice(initialDevice);
    updateFK(initialDevice);
    buildControlPanel(initialDevice);
    rebuildDeviceList();
    rebuildPrimaryModelDropdown(configParam);

    // Auto-fit camera
    const fitBox = new THREE.Box3();
    for (const grp of initialDevice.jointRotGroups) {
      grp.updateWorldMatrix(true, true);
      grp.traverse((child) => {
        if (child.isMesh) fitBox.expandByObject(child);
      });
    }
    for (const mesh of initialDevice.staticMeshes) {
      mesh.updateWorldMatrix(true, false);
      fitBox.expandByObject(mesh);
    }
    if (!fitBox.isEmpty()) {
      const fitCenter = fitBox.getCenter(new THREE.Vector3());
      const fitSize   = fitBox.getSize(new THREE.Vector3());
      const maxDim    = Math.max(fitSize.x, fitSize.y, fitSize.z);
      const fov       = State.camera.fov * (Math.PI / 180);
      const fitDist   = maxDim / (2 * Math.tan(fov / 2)) * 1.2;
      const direction = new THREE.Vector3(1, 0.6, 1).normalize();
      State.camera.position.copy(fitCenter).addScaledVector(direction, fitDist);
      State.orbitControls.target.copy(fitCenter);
      State.camera.updateProjectionMatrix();
      State.orbitControls.update();
    }
  } catch (err) {
    console.error('Failed to load initial device:', err);
    const msg = err?.message || String(err);
    document.getElementById('loading').innerHTML =
      `<span style="color:#f88">Failed to load <b>${configParam}</b></span><br>` +
      `<span style="color:#aaa; font-size:0.85em">${msg}</span><br>` +
      `<span style="color:#aaa; font-size:0.85em">Check the browser console (F12) and server logs for details.</span>`;
  }
}

document.getElementById('loading').style.display = 'none';

// Auto-save scene to localStorage on page unload
window.addEventListener('beforeunload', () => {
  if (State.devices.length === 0) return;
  try {
    const payload = buildScenePayload();
    localStorage.setItem(SCENE_STORAGE_KEY, JSON.stringify(payload));
  } catch (e) {
    console.warn('[Auto-save] Failed to save scene to localStorage:', e);
  }
});

// Save / Load scene buttons
document.getElementById('saveSceneBtn').addEventListener('click', () => exportSceneState());

document.getElementById('loadSceneBtn').addEventListener('click', () => {
  document.getElementById('loadSceneFile').click();
});

document.getElementById('clearSceneBtn').addEventListener('click', () => {
  if (!confirm('Clear the entire scene? This cannot be undone.')) return;

  // Clear imported STLs
  for (const entry of [...State.importedSTLs]) {
    if (State.selectedSTL === entry) deselectSTL();
    entry.mesh.removeFromParent();
    entry.mesh.geometry.dispose();
    entry.mesh.material.dispose();
  }
  State.importedSTLs.length = 0;
  document.getElementById('stl-list').innerHTML = '';

  // Clear devices
  for (const dev of [...State.devices]) {
    State.transformControls.detach();
    State.deviceTransformControls.detach();
    State.scene.remove(dev.rootGroup);
    dev.rootGroup.traverse(child => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
        else child.material.dispose();
      }
    });
    for (const label of dev.meshLabels) label.removeFromParent();
    if (dev.chainLine) State.scene.remove(dev.chainLine);
    for (const s of dev.chainSpheres) State.scene.remove(s);
    if (dev.ikTarget) State.scene.remove(dev.ikTarget);
    if (dev.ikLine) State.scene.remove(dev.ikLine);
  }
  State.devices.length = 0;
  State.resetDeviceIdCounter();

  rebuildDeviceList();
  rebuildPrimaryModelDropdown('robot_config.json');
});

// ============================================================
// Core scene restore (used by file load and localStorage restore)
// ============================================================
async function restoreScene(data) {
  // Clear existing imported STLs
  for (const entry of [...State.importedSTLs]) {
    if (State.selectedSTL === entry) deselectSTL();
    entry.mesh.removeFromParent();
    entry.mesh.geometry.dispose();
    entry.mesh.material.dispose();
  }
  State.importedSTLs.length = 0;
  document.getElementById('stl-list').innerHTML = '';

  // Clear existing devices and reload from state
  for (const dev of [...State.devices]) {
    State.transformControls.detach();
    State.deviceTransformControls.detach();
    State.scene.remove(dev.rootGroup);
    dev.rootGroup.traverse(child => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
        else child.material.dispose();
      }
    });
    for (const label of dev.meshLabels) label.removeFromParent();
    if (dev.chainLine) State.scene.remove(dev.chainLine);
    for (const s of dev.chainSpheres) State.scene.remove(s);
    if (dev.ikTarget) State.scene.remove(dev.ikTarget);
    if (dev.ikLine) State.scene.remove(dev.ikLine);
  }
  State.devices.length = 0;
  State.resetDeviceIdCounter();

  // Reload devices from state
  if (data.devices && data.devices.length > 0) {
    for (const devState of data.devices) {
      const dev = await loadDevice(devState.configFile);
      State.devices.push(dev);
      if (devState.name) dev.name = devState.name;
      if (devState.jointAngles) {
        for (let i = 0; i < devState.jointAngles.length && i < dev.jointAngles.length; i++) {
          dev.jointAngles[i] = devState.jointAngles[i];
        }
      }
      if (devState.position) {
        dev.rootGroup.position.set(...devState.position);
      }
      if (devState.rotation) {
        dev.rootGroup.rotation.set(...devState.rotation);
      }
      updateFK(dev);
      console.log('[Load Scene] Device:', dev.name, 'id:', dev.id,
        'joints:', dev.jointAngles.map(a => (a * 180 / Math.PI).toFixed(1)),
        'pos:', [dev.rootGroup.position.x, dev.rootGroup.position.y, dev.rootGroup.position.z]);
    }
    // Restore device parent links (must happen after all devices are loaded)
    for (let i = 0; i < data.devices.length; i++) {
      const parentLink = data.devices[i].parentLink;
      if (parentLink && parentLink.includes(':')) {
        const [idxStr, linkName] = parentLink.split(':', 2);
        const parentIdx = parseInt(idxStr, 10);
        let runtimeLink = null;
        if (!isNaN(parentIdx) && parentIdx >= 0 && parentIdx < State.devices.length) {
          runtimeLink = State.devices[parentIdx].id + ':' + linkName;
        } else {
          // Legacy format — search by link name
          for (const dev of State.devices) {
            if (dev !== State.devices[i] && dev.linkToJoint && dev.linkToJoint[linkName] !== undefined) {
              runtimeLink = dev.id + ':' + linkName;
              break;
            }
          }
        }
        if (runtimeLink) setDeviceParent(State.devices[i], runtimeLink, true);
      }
    }
    State.setActiveDevice(State.devices[0]);
    buildControlPanel(State.devices[0]);
  }

  // Restore floor size
  if (data.floorSize != null) {
    setFloorSize(data.floorSize);
    document.getElementById('floorSize').value = data.floorSize;
    document.getElementById('floorSizeVal').textContent = `${data.floorSize} m`;
  }

  // Restore camera
  if (data.camera) {
    if (data.camera.position) State.camera.position.set(...data.camera.position);
    if (data.camera.target) State.orbitControls.target.set(...data.camera.target);
    State.camera.updateProjectionMatrix();
    State.orbitControls.update();
  }

  // Ensure full scene graph is updated before restoring STLs
  State.scene.updateMatrixWorld(true);

  // Restore STLs (two-phase: create, then apply transforms)
  if (data.stls && data.stls.length > 0) {
    await restoreSTLsFromState(data.stls);
  }

  rebuildDeviceList();
  rebuildPrimaryModelDropdown(State.devices[0]?.configFile || 'robot_config.json');
}

document.getElementById('loadSceneFile').addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  try {
    document.getElementById('loading').style.display = 'block';
    document.getElementById('loading').textContent = 'Loading scene...';

    const data = await importSceneState(file);
    await restoreScene(data);

    document.getElementById('loading').style.display = 'none';
  } catch (err) {
    console.error('Failed to load scene:', err);
    document.getElementById('loading').innerHTML =
      `<span style="color:#f88">Failed to load scene</span><br>` +
      `<span style="color:#aaa; font-size:0.85em">${err?.message || err}</span>`;
    setTimeout(() => { document.getElementById('loading').style.display = 'none'; }, 3000);
  }
  e.target.value = '';
});

// Start render loop
animate();

// Start WebSocket
wsConnect();
initWsInfoPanel();
