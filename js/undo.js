// ============================================================
// js/undo.js — undo / redo of scene edits
// ============================================================
// Each step is a snapshot of the scene as Save Scene describes it, without
// the heavy buffers, the camera or the floor: a few kB. Steps are taken
// after an action (pointer or key released, a control changed, an API
// command), not by each feature, so anything that ends up in a saved scene
// can be undone. A change nobody made (IK settling after a drag, locked
// arms catching up) is folded into the step before it rather than becoming
// a step of its own; a change after an action that took a while (a device
// or file loading) becomes the step for that action.
//
// Undo applies the difference in place: transforms, joints, parents, locks,
// origins, colours and visibility at once; deleted objects come back from
// their kept file buffers, deleted devices reload their model.
import * as State from './state.js';
import {
  buildSceneMetadataForDB, restoreSTLsFromState, applySavedObjectState,
  setSTLParent, removeSTLEntry, syncSTLListItem, syncSTLNumericInputs,
} from './stl.js';
import { loadDevice, updateSliders, setDeviceOpacity, restoreIKLock } from './device.js';
import { removeDevice, setDeviceParent, buildControlPanel,
         rebuildDeviceList, rebuildParentDropdown, rebuildDeviceParentDropdown } from './panel.js';
import { updateFK, getEEWorldPosition, getEEWorldQuaternion } from './kinematics.js';
import { updateHexapodPose } from './hexapod.js';
import { setIKLock, refreshObjectLockSelect } from './locks.js';

const MAX_STEPS = 100;
const history = [];          // snapshots (JSON strings), oldest first
let index = -1;              // history[index] is the scene as it stands
let inputSinceStep = false;  // has anyone acted since the last step?
let busy = false;            // applying a step, or the page is loading a scene
let pointersDown = 0;
let lastApiInput = 0;
let keyCounter = 0;
const buffers = new Map();   // object id -> file buffer, while a step holds it

// Devices get a key of their own: their ids restart after Clear Scene.
function deviceKey(dev) { return (dev.undoKey ??= 'u' + (++keyCounter)); }

export function sceneSnapshot() {
  const data = buildSceneMetadataForDB();
  data.devices.forEach((d, i) => { d.key = deviceKey(State.devices[i]); });
  delete data.camera;
  delete data.floorSize;
  return JSON.stringify(data);
}

function dragging() {
  return pointersDown > 0
    || !!(State.transformControls?.dragging || State.stlTransformControls?.dragging
          || State.deviceTransformControls?.dragging);
}

// Someone acted. The next change is a new step.
export function noteUndoInput() { inputSinceStep = true; }

// An API command. A script streaming commands makes one step per burst
// (commands less than a second apart), not one per command.
export function noteApiInput() {
  const now = Date.now();
  if (now - lastApiInput > 1000) inputSinceStep = true;
  lastApiInput = now;
}

// While the page loads a whole scene, its half-built states are not steps.
export function setUndoSuspended(on) { busy = on; }

// Take a step if the scene changed. Returns true if it did.
export function recordUndo() {
  if (busy || dragging()) return false;
  const snap = sceneSnapshot();
  if (snap === history[index]) return false;
  for (const e of State.importedSTLs) if (e._buffer) buffers.set(e.stlId, e._buffer);
  if (index < 0 || inputSinceStep) {
    history.length = index + 1;
    history.push(snap);
    if (history.length > MAX_STEPS) history.shift();
    index = history.length - 1;
    inputSinceStep = false;
    pruneBuffers();
  } else {
    history[index] = snap;
  }
  updateButtons();
  return true;
}

// Forget any recorded steps and start again from the scene as it is.
export function resetUndo() {
  history.length = 0;
  index = -1;
  buffers.clear();
  inputSinceStep = false;
  recordUndo();
}

function pruneBuffers() {
  const held = new Set();
  for (const snap of history) for (const r of JSON.parse(snap).stls) held.add(r.id);
  for (const e of State.importedSTLs) held.add(e.stlId);
  for (const id of buffers.keys()) if (!held.has(id)) buffers.delete(id);
}

export const canUndo = () => index > 0 && !busy;
export const canRedo = () => index < history.length - 1 && !busy;

export async function undo(opts) {
  if (!canUndo()) return false;
  // Fold anything still settling into the current step. (The key or click
  // that asked for the undo counts as input, which would make it a step.)
  inputSinceStep = false;
  recordUndo();
  if (!canUndo()) return false;
  await applyStep(index - 1, opts);
  return true;
}

export async function redo(opts) {
  if (!canRedo()) return false;
  await applyStep(index + 1, opts);
  return true;
}

async function applyStep(i, opts) {
  busy = true;
  try {
    await applySnapshot(JSON.parse(history[i]), opts);
    index = i;
  } finally {
    busy = false;
    inputSinceStep = false;
    updateButtons();
  }
}

// A saved parent link ("deviceIndex:link") as the live one ("devId:link").
// Devices are in the snapshot's order by the time this is used.
function liveParent(stable) {
  if (!stable || !stable.includes(':')) return stable || null;
  const [idx, link] = stable.split(':', 2);
  const dev = State.devices[+idx];
  return dev ? dev.id + ':' + link : null;
}

// Make the scene match a snapshot. `loadDevice` builds a device from its
// config without adding it to the scene (the headless tests pass their own).
export async function applySnapshot(data, { loadDevice: load = loadDevice } = {}) {
  // ── Devices: add what is missing, remove what is extra, keep the order ──
  const byKey = new Map(State.devices.map(d => [deviceKey(d), d]));
  const order = [];
  for (const d of data.devices) {
    let dev = byKey.get(d.key);
    if (!dev) {
      dev = await load(d.configFile);
      dev.undoKey = d.key;
      State.devices.push(dev);
    }
    order.push(dev);
  }
  for (const dev of [...State.devices]) {
    if (!order.includes(dev)) removeDevice(dev, { allowLast: true });
  }
  State.devices.splice(0, State.devices.length, ...order);
  if (!State.activeDevice && order.length) State.setActiveDevice(order[0]);

  data.devices.forEach((d, i) => {
    const dev = order[i];
    const parent = liveParent(d.parentLink);
    if ((dev.parentLink || null) !== parent) setDeviceParent(dev, parent, true);
    if (d.name) dev.name = d.name;
    if (d.jointAngles) d.jointAngles.forEach((a, j) => { if (j < dev.jointAngles.length) dev.jointAngles[j] = a; });
    if (d.position) dev.rootGroup.position.set(...d.position);
    if (d.rotation) dev.rootGroup.rotation.set(...d.rotation);
    if (d.visible !== undefined) dev.rootGroup.visible = d.visible;
    if (d.opacity !== undefined && d.opacity !== (dev.opacity ?? 1)) setDeviceOpacity(dev, d.opacity);
    if (dev.type === 'hexapod') {
      if (d.platformPose) d.platformPose.forEach((v, j) => { dev.platformPose[j] = v; });
      updateHexapodPose(dev);
    } else {
      updateFK(dev);
    }
  });
  State.scene.updateMatrixWorld(true);

  // ── Objects: remove what is extra, bring back what was deleted ──
  const recIds = new Set(data.stls.map(r => r.id));
  for (const e of [...State.importedSTLs]) if (!recIds.has(e.stlId)) removeSTLEntry(e);
  const have = new Set(State.importedSTLs.map(e => e.stlId));
  const missing = data.stls.filter(r => !have.has(r.id) && buffers.has(r.id));
  if (missing.length) await restoreSTLsFromState(missing.map(r => ({ ...r, buffer: buffers.get(r.id) })));

  const byId = new Map(State.importedSTLs.map(e => [e.stlId, e]));
  const objects = data.stls.map(r => byId.get(r.id)).filter(Boolean);
  State.importedSTLs.splice(0, State.importedSTLs.length, ...objects);
  for (const e of objects) e._ui?.item.parentNode?.appendChild(e._ui.item);   // list in the same order

  for (const rec of data.stls) {
    const entry = byId.get(rec.id);
    if (!entry) continue;
    // A restore only ever adds a parent; a step must also take one away.
    if (!rec.parentLink && entry.parentLink) setSTLParent(entry, null, true);
    applySavedObjectState(entry, rec);
    if (!entry.isSplat && rec.color !== undefined && rec.color !== entry.color) {
      entry.color = rec.color;
      entry.mesh.material.color.setHex(rec.color);
    }
    if (rec.name) entry.name = rec.name;
    entry.lockedTo = (rec.lockedTo && byId.get(rec.lockedTo)) || null;
    syncSTLListItem(entry);
  }
  State.scene.updateMatrixWorld(true);

  // ── Arm locks and IK targets ──
  data.devices.forEach((d, i) => {
    const dev = order[i];
    const want = (d.ikLock && byId.get(d.ikLock)) || null;
    if ((dev.ikLock?.entry || null) !== want) {
      if (want) restoreIKLock(dev, want); else setIKLock(dev, null);
    }
    // The solver would drag the arm back to its old target.
    if (dev.ikMode && dev.type !== 'hexapod') {
      dev.ikTarget.position.copy(getEEWorldPosition(dev));
      dev.ikTargetQuat.copy(getEEWorldQuaternion(dev));
      dev.ikTargetEuler.setFromQuaternion(dev.ikTargetQuat, 'YZX');
      dev.ikTarget.quaternion.copy(dev.ikTargetQuat);
    }
  });
  // Everything in a lock group moved at once: take the poses as they are,
  // rather than letting the first one drive the rest.
  for (const e of State.importedSTLs) e._lockPose = null;
  for (const dev of State.devices) if (dev.ikLock) dev.ikLock.pose = null;

  // ── Panels ──
  if (typeof document !== 'undefined' && document.getElementById?.('device-list')) {
    rebuildDeviceList();
    rebuildParentDropdown();
    rebuildDeviceParentDropdown();
    const dev = State.activeDevice;
    if (dev) {
      buildControlPanel(dev);
      if (dev.type !== 'hexapod') updateSliders(dev);
    }
    if (State.selectedSTL) {
      syncSTLNumericInputs(State.selectedSTL);
      refreshObjectLockSelect(State.selectedSTL);
    }
  }
  State.requestRender();
}

function updateButtons() {
  if (typeof document === 'undefined') return;
  const u = document.getElementById?.('undoBtn');
  const r = document.getElementById?.('redoBtn');
  if (u) u.disabled = !canUndo();
  if (r) r.disabled = !canRedo();
}

// Wire the page: record after actions, Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y, and
// the Undo / Redo buttons. Call once the initial scene has loaded.
export function initUndo() {
  const settle = () => requestAnimationFrame(() => recordUndo());
  window.addEventListener('pointerdown', () => { pointersDown++; noteUndoInput(); }, true);
  window.addEventListener('pointerup', () => { pointersDown = Math.max(0, pointersDown - 1); settle(); }, true);
  window.addEventListener('pointercancel', () => { pointersDown = Math.max(0, pointersDown - 1); }, true);
  window.addEventListener('blur', () => { pointersDown = 0; });
  for (const type of ['keydown', 'input', 'drop']) window.addEventListener(type, noteUndoInput, true);
  for (const type of ['keyup', 'change', 'drop']) window.addEventListener(type, settle, true);
  // Changes that finish later (a device or file loading) are caught here.
  setInterval(recordUndo, 500);

  window.addEventListener('keydown', (e) => {
    if (!(e.ctrlKey || e.metaKey) || e.altKey) return;
    const t = e.target;
    if (t && (t.tagName === 'INPUT' && t.type === 'text' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
    const k = e.key.toLowerCase();
    if (k === 'z' && !e.shiftKey) { e.preventDefault(); undo(); }
    else if ((k === 'z' && e.shiftKey) || k === 'y') { e.preventDefault(); redo(); }
  });
  document.getElementById('undoBtn')?.addEventListener('click', () => undo());
  document.getElementById('redoBtn')?.addEventListener('click', () => redo());
  resetUndo();
}
