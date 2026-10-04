// ============================================================
// js/ik-lock.js — lock a serial arm's IK target to an imported object
// ============================================================
// A locked target keeps its pose relative to the object, so moving or
// rotating the object drags the target with it and the IK follows. Lock two
// arms to one object (a pipe held at both ends) and they move in tandem.
// It works both ways: moving one locked arm's target (gizmo, sliders, API)
// carries the object, and with it every other arm locked to it. To change
// a grip, unlock, move the target, and lock again. The object's scale is
// left out of the pose, so scaling it does not skew the grip.
import * as THREE from 'three';
import * as State from './state.js';

const _one   = new THREE.Vector3(1, 1, 1);
const _pos   = new THREE.Vector3();
const _scale = new THREE.Vector3();
const _quat  = new THREE.Quaternion();
const _obj   = new THREE.Matrix4();
const _tgt   = new THREE.Matrix4();

function objectPose(entry, out) {
  entry.mesh.updateWorldMatrix(true, false);
  entry.mesh.matrixWorld.decompose(_pos, _quat, _scale);
  return out.compose(_pos, _quat, _one);
}

// The EE quaternion can come back a little off unit length, which compose
// would turn into a scale and decompose would then drop.
function targetPose(dev, out) {
  return out.compose(dev.ikTarget.position, _quat.copy(dev.ikTargetQuat).normalize(), _one);
}

// An object the arm itself carries would chase its own target, so only
// objects outside the device are offered.
function belongsTo(entry, dev) {
  for (let o = entry.mesh; o; o = o.parent) if (o === dev.rootGroup) return true;
  return false;
}

export function lockableObjects(dev) {
  return State.importedSTLs.filter(e => !belongsTo(e, dev));
}

// Lock dev's target to entry where the target is now; null unlocks.
export function setIKLock(dev, entry) {
  if (!entry || belongsTo(entry, dev)) { dev.ikLock = null; return; }
  const objPose = objectPose(entry, new THREE.Matrix4());
  const tgtPose = targetPose(dev, new THREE.Matrix4());
  const offset = objPose.clone().invert().multiply(tgtPose);
  dev.ikLock = { entry, objPose, tgtPose, offset };
}

// Give the object a new rigid world pose, keeping its scale and parent.
const _delta = new THREE.Matrix4();
const _local = new THREE.Matrix4();
function moveObject(entry, fromPose, toPose) {
  const m = entry.mesh;
  _delta.copy(fromPose).invert().premultiply(toPose);
  _local.multiplyMatrices(_delta, m.matrixWorld);
  if (m.parent) _local.premultiply(_tgt.copy(m.parent.matrixWorld).invert());
  _local.decompose(m.position, m.quaternion, m.scale);
  m.updateMatrixWorld(true);
}

function setTarget(dev, pose) {
  pose.decompose(dev.ikTarget.position, dev.ikTargetQuat, _scale);
  dev.ikTarget.quaternion.copy(dev.ikTargetQuat);
  dev.ikTargetEuler.setFromQuaternion(dev.ikTargetQuat, 'YZX');
}

// Called once per frame before the IK solve. An object and the arms locked
// to it move as one: whichever moved since the last frame (the object, or
// one arm's target) drives the rest. Returns the devices whose target the
// lock moved, and whether an object was moved.
export function updateIKLocks() {
  const moved = [];
  let objectsMoved = false;
  const groups = new Map();   // entry -> devices locked to it
  for (const dev of State.devices) {
    const lock = dev.ikLock;
    if (!lock) continue;
    if (!dev.ikMode || !State.importedSTLs.includes(lock.entry) || belongsTo(lock.entry, dev)) {
      dev.ikLock = null;
      if (dev === State.activeDevice) refreshIKLockSelect(dev);
      continue;
    }
    if (!groups.has(lock.entry)) groups.set(lock.entry, []);
    groups.get(lock.entry).push(dev);
  }

  for (const [entry, devs] of groups) {
    objectPose(entry, _obj);
    let driver = null;
    if (devs.every(d => _obj.equals(d.ikLock.objPose))) {
      // The object is where the locks left it; an arm whose target has
      // moved carries the object along.
      driver = devs.find(d => !targetPose(d, _tgt).equals(d.ikLock.tgtPose));
      if (!driver) continue;
      const newObj = targetPose(driver, new THREE.Matrix4())
        .multiply(_delta.copy(driver.ikLock.offset).invert());
      moveObject(entry, _obj, newObj);
      objectPose(entry, _obj);
      objectsMoved = true;
    }
    for (const d of devs) {
      const lock = d.ikLock;
      lock.objPose.copy(_obj);
      if (d !== driver) {
        setTarget(d, _tgt.multiplyMatrices(_obj, lock.offset));
        moved.push(d);
      }
      targetPose(d, lock.tgtPose);
    }
  }
  return { moved, objectsMoved };
}

// Fill the "Lock to" dropdown for dev and show its current lock.
export function refreshIKLockSelect(dev) {
  const sel = typeof document !== 'undefined' && document.getElementById('ikLockSelect');
  if (!sel || !dev) return;
  sel.innerHTML = '';
  sel.add(new Option('None', ''));
  for (const e of lockableObjects(dev)) sel.add(new Option(e.name, e.stlId));
  sel.value = dev.ikLock ? dev.ikLock.entry.stlId : '';
}
