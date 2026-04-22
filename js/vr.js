import * as THREE from 'three';
import { VRButton } from 'three/addons/webxr/VRButton.js';
import { XRControllerModelFactory } from 'three/addons/webxr/XRControllerModelFactory.js';
import * as State from './state.js';
import { setOrtho } from './scene.js';

const _raycaster = new THREE.Raycaster();
const _tempMatrix = new THREE.Matrix4();
const _worldPos = new THREE.Vector3();
const _groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const _intersection = new THREE.Vector3();

let teleportMarker;
let savedCamPos, savedCamQuat, savedTarget;
let activeGrab = null;
const selecting = new Set();

export async function initVR() {
  const renderer = State.renderer;

  const vrSupported = navigator.xr &&
    await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);

  renderer.xr.enabled = !!vrSupported;

  const btn = VRButton.createButton(renderer);
  btn.style.zIndex = '9999';
  btn.style.transition = 'opacity 1s ease';
  document.body.appendChild(btn);

  if (!vrSupported) {
    setTimeout(() => {
      btn.style.opacity = '0';
      setTimeout(() => { btn.style.display = 'none'; }, 1000);
    }, 10000);
    return;
  }

  const rig = new THREE.Group();
  rig.name = 'VRCameraRig';
  State.scene.add(rig);
  rig.add(State.camera);
  State.setVRRig(rig);

  const factory = new XRControllerModelFactory();

  for (let i = 0; i < 2; i++) {
    const controller = renderer.xr.getController(i);
    rig.add(controller);

    const grip = renderer.xr.getControllerGrip(i);
    grip.add(factory.createControllerModel(grip));
    rig.add(grip);

    const rayGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, -5),
    ]);
    const color = i === 0 ? 0x4488ff : 0x44ff88;
    controller.add(new THREE.Line(rayGeo,
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.5 })));

    controller.addEventListener('selectstart', () => selecting.add(controller));
    controller.addEventListener('selectend', () => {
      onTriggerRelease(controller);
      selecting.delete(controller);
    });
    controller.addEventListener('squeezestart', () => onGrabStart(controller));
    controller.addEventListener('squeezeend', () => onGrabEnd(controller));
  }

  const ringGeo = new THREE.RingGeometry(0.05, 0.09, 32);
  ringGeo.rotateX(-Math.PI / 2);
  const discGeo = new THREE.CircleGeometry(0.05, 32);
  discGeo.rotateX(-Math.PI / 2);
  teleportMarker = new THREE.Group();
  teleportMarker.add(new THREE.Mesh(ringGeo,
    new THREE.MeshBasicMaterial({ color: 0x44ff88, side: THREE.DoubleSide })));
  teleportMarker.add(new THREE.Mesh(discGeo,
    new THREE.MeshBasicMaterial({ color: 0x44ff88, transparent: true, opacity: 0.3, side: THREE.DoubleSide })));
  teleportMarker.visible = false;
  State.scene.add(teleportMarker);

  renderer.xr.addEventListener('sessionstart', onSessionStart);
  renderer.xr.addEventListener('sessionend', onSessionEnd);
}

function onSessionStart() {
  State.setVRActive(true);
  if (State.orthoOn) setOrtho(false);

  savedCamPos = State.camera.position.clone();
  savedCamQuat = State.camera.quaternion.clone();
  savedTarget = State.orbitControls.target.clone();

  const rig = State.vrRig;
  rig.position.set(savedTarget.x, 0, savedTarget.z);
  State.camera.position.set(0, 0, 0);
  State.camera.quaternion.identity();
  State.orbitControls.enabled = false;
}

function onSessionEnd() {
  State.setVRActive(false);

  const rig = State.vrRig;
  State.camera.position.copy(savedCamPos);
  State.camera.quaternion.copy(savedCamQuat);
  rig.position.set(0, 0, 0);
  rig.quaternion.identity();

  State.orbitControls.target.copy(savedTarget);
  State.orbitControls.enabled = true;
  State.orbitControls.update();

  teleportMarker.visible = false;
  activeGrab = null;
}

function castRay(controller) {
  _tempMatrix.identity().extractRotation(controller.matrixWorld);
  _raycaster.ray.origin.setFromMatrixPosition(controller.matrixWorld);
  _raycaster.ray.direction.set(0, 0, -1).applyMatrix4(_tempMatrix);
}

function onTriggerRelease(controller) {
  if (!teleportMarker.visible) return;

  const rig = State.vrRig;
  State.camera.getWorldPosition(_worldPos);
  rig.position.x += teleportMarker.position.x - _worldPos.x;
  rig.position.z += teleportMarker.position.z - _worldPos.z;
  teleportMarker.visible = false;
}

function onGrabStart(controller) {
  if (activeGrab) return;
  castRay(controller);

  const candidates = State.importedSTLs.filter(s => s.mesh.visible).map(s => s.mesh);
  let hits = _raycaster.intersectObjects(candidates, false);

  if (hits.length === 0 && State.activeDevice?.ikMode && State.activeDevice.ikTarget) {
    hits = _raycaster.intersectObject(State.activeDevice.ikTarget, true);
  }

  if (hits.length > 0 && hits[0].distance < 3) {
    let target = hits[0].object;
    const stlEntry = State.importedSTLs.find(s => s.mesh === target);
    if (stlEntry) {
      target = stlEntry.mesh;
    } else if (State.activeDevice?.ikTarget) {
      let p = target;
      while (p) {
        if (p === State.activeDevice.ikTarget) { target = p; break; }
        p = p.parent;
      }
    }

    controller.getWorldPosition(_worldPos);
    const objWorld = new THREE.Vector3();
    target.getWorldPosition(objWorld);

    activeGrab = {
      controller,
      object: target,
      offset: objWorld.sub(_worldPos),
      isIKTarget: target === State.activeDevice?.ikTarget,
    };
  }
}

function onGrabEnd(controller) {
  if (activeGrab?.controller === controller) activeGrab = null;
}

export function updateVR() {
  if (!State.vrActive) return;

  teleportMarker.visible = false;
  for (const ctrl of selecting) {
    castRay(ctrl);
    if (_raycaster.ray.intersectPlane(_groundPlane, _intersection)) {
      teleportMarker.position.set(_intersection.x, 0.005, _intersection.z);
      teleportMarker.visible = true;
    }
  }

  if (activeGrab) {
    activeGrab.controller.getWorldPosition(_worldPos);
    const newWorld = _worldPos.clone().add(activeGrab.offset);
    if (activeGrab.object.parent) activeGrab.object.parent.worldToLocal(newWorld);
    activeGrab.object.position.copy(newWorld);

    if (activeGrab.isIKTarget && State.activeDevice) {
      State.activeDevice.ikTargetQuat.copy(activeGrab.object.quaternion);
      State.activeDevice.ikTargetEuler.setFromQuaternion(State.activeDevice.ikTargetQuat, 'YZX');
    }
  }
}
