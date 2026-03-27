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
  setStlSaveCallback,
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
} from './panel.js';
import {
  saveSTLDebounced,
  restorePersistedSTLs,
  loadSTLFile, loadOBJFile, loadPLYFile, loadGLBFile,
  addPrimitive,
  selectSTL, deselectSTL, setSTLTransformMode, setSTLParent,
} from './stl.js';
import { checkCollisions, clearCollisionHighlights } from './collision.js';
import {
  wsConnect, registerSetActiveDevice,
} from './websocket.js';

// Register the setActiveDevice callback for websocket.js
// (avoids circular dependency: websocket -> panel -> websocket)
registerSetActiveDevice(setActiveDevice);

// Wire the STL-save callback into scene.js transform controls
setStlSaveCallback(saveSTLDebounced);

// ============================================================
// Raycaster (for click-to-select)
// ============================================================
const raycaster = new THREE.Raycaster();
raycaster.params.Points = { threshold: 0.005 };
const mouse = new THREE.Vector2();

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
  saveSTLDebounced(State.selectedSTL);
});

document.getElementById('stlResetScale').addEventListener('click', () => {
  if (!State.selectedSTL) return;
  State.selectedSTL.mesh.scale.set(1, 1, 1);
  saveSTLDebounced(State.selectedSTL);
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

// ============================================================
// Click-to-select STL meshes or activate devices
// ============================================================
State.renderer.domElement.addEventListener('pointerdown', (e) => {
  if (State.stlTransformControls.dragging || State.transformControls.dragging || State.deviceTransformControls.dragging) return;
  if (e.button !== 0) return;

  mouse.x = (e.clientX / innerWidth) * 2 - 1;
  mouse.y = -(e.clientY / innerHeight) * 2 + 1;
  raycaster.setFromCamera(mouse, State.activeCamera);

  // Test against all imported STL meshes first
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

try {
  const initialDevice = await loadDevice(configParam);
  State.devices.push(initialDevice);
  State.setActiveDevice(initialDevice);
  updateFK(initialDevice);
  buildControlPanel(initialDevice);
  rebuildDeviceList();

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

  document.getElementById('loading').style.display = 'none';
} catch (err) {
  console.error('Failed to load initial device:', err);
  const msg = err?.message || String(err);
  document.getElementById('loading').innerHTML =
    `<span style="color:#f88">Failed to load <b>${configParam}</b></span><br>` +
    `<span style="color:#aaa; font-size:0.85em">${msg}</span><br>` +
    `<span style="color:#aaa; font-size:0.85em">Check the browser console (F12) and server logs for details.</span>`;
}

// Restore persisted STLs
restorePersistedSTLs();

// Start render loop
animate();

// Start WebSocket
wsConnect();
