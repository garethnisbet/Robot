// ============================================================
// js/locks.js — lock objects to each other, and arms' IK targets to objects
// ============================================================
// Locked things move as one rigid group: objects locked to each other
// (entry.lockedTo), and arms whose IK target is locked to one of those
// objects (dev.ikLock). Each frame, whichever member moved since the last
// frame (an object, by its gizmo, the API or the arm carrying it; or an
// arm's target, by its gizmo, sliders or the API) drives the rest: they all
// get the same world motion. So a clamp locked to a pipe rides with it, two
// arms locked to the pipe carry it between them, and moving either arm
// carries the pipe, the clamp and the other arm.
//
// To change a grip, unlock, move, and lock again. Poses are rigid (an
// object's scale is left out), so scaling an object does not skew a grip.
import * as THREE from 'three';
import * as State from './state.js';

const _one   = new THREE.Vector3(1, 1, 1);
const _pos   = new THREE.Vector3();
const _scale = new THREE.Vector3();
const _quat  = new THREE.Quaternion();
const _m     = new THREE.Matrix4();
const _delta = new THREE.Matrix4();

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

// Apply a world motion to an object, keeping its scale and parent.
function moveObject(entry, delta) {
  const m = entry.mesh;
  m.updateWorldMatrix(true, false);
  _m.multiplyMatrices(delta, m.matrixWorld);
  if (m.parent) _m.premultiply(new THREE.Matrix4().copy(m.parent.matrixWorld).invert());
  _m.decompose(m.position, m.quaternion, m.scale);
  m.updateMatrixWorld(true);
}

function moveTarget(dev, delta) {
  targetPose(dev, _m).premultiply(delta).decompose(dev.ikTarget.position, dev.ikTargetQuat, _scale);
  dev.ikTarget.quaternion.copy(dev.ikTargetQuat);
  dev.ikTargetEuler.setFromQuaternion(dev.ikTargetQuat, 'YZX');
}

// Is the object part of dev (on one of its links)?
function carriedBy(entry, dev) {
  for (let o = entry.mesh; o; o = o.parent) if (o === dev.rootGroup) return true;
  return false;
}

// The objects locked together with entry (itself included).
export function lockedGroup(entry) {
  const group = new Set([entry]);
  let grew = true;
  while (grew) {
    grew = false;
    for (const e of State.importedSTLs) {
      if (group.has(e)) {
        if (e.lockedTo && !group.has(e.lockedTo)) { group.add(e.lockedTo); grew = true; }
      } else if (e.lockedTo && group.has(e.lockedTo)) { group.add(e); grew = true; }
    }
  }
  return [...group].filter(e => State.importedSTLs.includes(e));
}

// An arm whose target is locked into a group that holds an object the arm
// carries would chase its own target, so such objects are not offered.
export function lockableObjects(dev) {
  return State.importedSTLs.filter(e => !lockedGroup(e).some(o => carriedBy(o, dev)));
}

// Poses are recorded when a lock is made, so a move made before the next
// frame counts as a move.
function startPose(entry) {
  entry._lockPose ??= objectPose(entry, new THREE.Matrix4());
}

// Lock dev's target to entry where the target is now; null unlocks.
export function setIKLock(dev, entry) {
  if (!entry || !lockableObjects(dev).includes(entry)) { dev.ikLock = null; return; }
  startPose(entry);
  dev.ikLock = { entry, pose: targetPose(dev, new THREE.Matrix4()) };
}

// Lock entry to other where both are now; null unlocks.
export function setObjectLock(entry, other) {
  if (!other || other === entry) { entry.lockedTo = null; return; }
  startPose(entry);
  startPose(other);
  entry.lockedTo = other;
}

// Called once per frame before the IK solve. Returns the devices whose
// target a lock moved, and whether a lock moved any object.
export function updateLocks() {
  const moved = [];
  let objectsMoved = false;

  for (const e of State.importedSTLs) {
    if (e.lockedTo && !State.importedSTLs.includes(e.lockedTo)) e.lockedTo = null;
  }
  const arms = [];
  for (const dev of State.devices) {
    const lock = dev.ikLock;
    if (!lock) continue;
    if (!dev.ikMode || !State.importedSTLs.includes(lock.entry)
        || lockedGroup(lock.entry).some(o => carriedBy(o, dev))) {
      dev.ikLock = null;
      if (dev === State.activeDevice) refreshIKLockSelect(dev);
      continue;
    }
    arms.push(dev);
  }

  const seen = new Set();
  for (const start of State.importedSTLs) {
    if (seen.has(start)) continue;
    const objects = lockedGroup(start);
    objects.forEach(o => seen.add(o));
    const devs = arms.filter(d => objects.includes(d.ikLock.entry));
    if (objects.length + devs.length < 2) {
      // Not locked to anything: forget its pose, so locking it later
      // starts from where it is then.
      for (const o of objects) o._lockPose = null;
      continue;
    }

    // Each member's pose now and at the end of the last frame (a new member
    // starts as unmoved).
    const members = [
      ...objects.map(o => ({ o, last: o._lockPose, now: objectPose(o, new THREE.Matrix4()) })),
      ...devs.map(d => ({ d, last: d.ikLock.pose, now: targetPose(d, new THREE.Matrix4()) })),
    ];
    for (const m of members) if (!m.last) m.last = m.now.clone();
    const changed = members.filter(m => !m.now.equals(m.last));
    if (changed.length) {
      // The first that moved drives the members that did not. Two that
      // moved together (on the same link, say) each keep their own motion.
      const driver = changed[0];
      _delta.copy(driver.last).invert().premultiply(driver.now);
      for (const m of members) {
        if (changed.includes(m)) continue;
        if (m.o) { moveObject(m.o, _delta); objectPose(m.o, m.now); objectsMoved = true; }
        else { moveTarget(m.d, _delta); targetPose(m.d, m.now); moved.push(m.d); }
      }
    }
    for (const m of members) {
      if (m.o) m.o._lockPose = m.now; else m.d.ikLock.pose = m.now;
    }
  }
  return { moved, objectsMoved };
}

function fillSelect(sel, options, value) {
  sel.innerHTML = '';
  sel.add(new Option('None', ''));
  for (const e of options) sel.add(new Option(e.name, e.stlId));
  sel.value = value || '';
}

// Fill the IK panel's "Lock to" dropdown for dev and show its current lock.
export function refreshIKLockSelect(dev) {
  const sel = typeof document !== 'undefined' && document.getElementById('ikLockSelect');
  if (!sel || !dev) return;
  fillSelect(sel, lockableObjects(dev), dev.ikLock?.entry.stlId);
}

// Fill the object panel's "Lock to" dropdown for entry.
export function refreshObjectLockSelect(entry) {
  const sel = typeof document !== 'undefined' && document.getElementById('stlLockSelect');
  if (!sel || !entry) return;
  fillSelect(sel, State.importedSTLs.filter(e => e !== entry), entry.lockedTo?.stlId);
}
