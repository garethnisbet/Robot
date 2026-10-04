// Undo / redo. Bringing back a deleted object goes through the page's
// scene restore, which builds list rows, so this file runs with a stand-in
// DOM (each test file is its own process).
import { test } from 'node:test';
import assert from 'node:assert/strict';

const fake = () => new Proxy(function () {}, {
  get: (t, k) => (k === Symbol.toPrimitive ? () => '' : k in t ? t[k] : fake()),
  set: () => true,
  apply: () => fake(),
});
const memory = () => {
  const m = new Map();
  return { getItem: k => m.get(k) ?? null, setItem: (k, v) => m.set(k, String(v)), removeItem: k => m.delete(k) };
};
globalThis.document = { getElementById: () => fake(), createElement: () => fake(), querySelectorAll: () => [], body: fake() };
globalThis.Option = function Option() {};
globalThis.sessionStorage = memory();
globalThis.localStorage = memory();

const THREE = await import('three');
const State = await import('../../js/state.js');
const { createEngine } = await import('../../headless/engine.mjs');
const { setObjectOrigin, removeSTLEntry } = await import('../../js/stl.js');
const { setObjectLock, setIKLock, updateLocks } = await import('../../js/locks.js');
const { removeDevice } = await import('../../js/panel.js');
const { getEEWorldPosition, getEEWorldQuaternion, updateFK } = await import('../../js/kinematics.js');
const undoMod = await import('../../js/undo.js');
const { sceneSnapshot, recordUndo, noteUndoInput, resetUndo, undo, redo, canUndo, canRedo } = undoMod;

// The page-only parts the panels touch: gizmos, and a device's chain,
// labels and IK line.
const gizmo = () => ({ attach() {}, detach() {}, setMode() {}, setSpace() {}, dragging: false });
function pageParts(dev) {
  dev.chainSpheres ??= [];
  dev.meshLabels ??= [];
  dev.ikLine ??= new THREE.Object3D();
  return dev;
}

async function scene() {
  const e = await createEngine();
  State.initCoreObjects(e.scene, new THREE.PerspectiveCamera(), null, null);
  State.initControls({ target: new THREE.Vector3() }, gizmo(), gizmo(), gizmo());
  const addDevice = e.addDevice;
  e.addDevice = async (configFile) => pageParts(await addDevice(configFile));
  // Builds a device without adding it, as the page's loadDevice does.
  const load = async (configFile) => {
    const dev = await e.addDevice(configFile);
    State.devices.splice(State.devices.indexOf(dev), 1);
    return dev;
  };
  return { e, opts: { loadDevice: load } };
}

// One user action: input, the change, then the step.
function act(change) {
  noteUndoInput();
  change();
  assert.equal(recordUndo(), true, 'the action changed nothing');
}

test('each action is a step; undo and redo walk them exactly', async () => {
  const { e, opts } = await scene();
  await e.addDevice('meca500_config.json');
  const cube = await e.addPrimitive('cube');
  const ball = await e.addPrimitive('icosphere');
  resetUndo();
  const states = [sceneSnapshot()];
  act(() => cube.mesh.position.set(0.3, 0.1, 0.2));
  states.push(sceneSnapshot());
  act(() => { cube.color = 0xff0000; cube.mesh.material.color.setHex(0xff0000); });
  states.push(sceneSnapshot());
  act(() => setObjectOrigin(ball, new THREE.Vector3(10, 0, 0)));
  states.push(sceneSnapshot());
  act(() => setObjectLock(ball, cube));
  states.push(sceneSnapshot());
  act(() => e.State.devices[0].jointAngles[1] = 0.4);
  states.push(sceneSnapshot());

  for (let i = states.length - 2; i >= 0; i--) {
    assert.equal(await undo(opts), true);
    assert.equal(sceneSnapshot(), states[i], `undo to step ${i}`);
  }
  assert.equal(canUndo(), false);
  assert.equal(cube.mesh.material.color.getHex(), 0x44aaff);
  assert.ok(!ball.origin || ball.origin.length() === 0, 'origin undone');
  for (let i = 1; i < states.length; i++) {
    assert.equal(await redo(opts), true);
    assert.equal(sceneSnapshot(), states[i], `redo to step ${i}`);
  }
  assert.equal(canRedo(), false);
  assert.equal(ball.lockedTo, cube);
});

test('a change nobody made joins the step before it', async () => {
  const { e, opts } = await scene();
  const cube = await e.addPrimitive('cube');
  resetUndo();
  const start = sceneSnapshot();
  act(() => cube.mesh.position.x = 0.1);
  cube.mesh.position.x = 0.11;            // IK settling, a lock catching up …
  assert.equal(recordUndo(), true);
  await undo(opts);
  assert.equal(sceneSnapshot(), start);
  assert.equal(canUndo(), false, 'one step, not two');
  await redo(opts);
  assert.equal(cube.mesh.position.x, 0.11);
});

test('a new action after an undo drops the redo steps', async () => {
  const { e, opts } = await scene();
  const cube = await e.addPrimitive('cube');
  resetUndo();
  act(() => cube.mesh.position.x = 0.1);
  act(() => cube.mesh.position.x = 0.2);
  await undo(opts);
  act(() => cube.mesh.position.y = 0.5);
  assert.equal(canRedo(), false);
  await undo(opts);
  assert.equal(cube.mesh.position.x, 0.1);
  assert.equal(cube.mesh.position.y, 0);
});

test('a deleted object comes back as it was, locks and all', async () => {
  const { e, opts } = await scene();
  const pipe = await e.addPrimitive('cylinder');
  const clamp = await e.addPrimitive('cube');
  pipe.mesh.position.set(0.2, 0.3, 0.1);
  pipe.mesh.rotation.set(0.1, 0.2, 0.3);
  setObjectOrigin(pipe, new THREE.Vector3(0, 10, 0));
  setObjectLock(clamp, pipe);
  resetUndo();
  const before = sceneSnapshot();
  const triangles = pipe.mesh.geometry.getAttribute('position').count;

  act(() => removeSTLEntry(pipe));
  assert.equal(State.importedSTLs.length, 1);
  await undo(opts);
  assert.equal(sceneSnapshot(), before);
  const back = State.importedSTLs.find(o => o.stlId === pipe.stlId);
  assert.ok(back && back !== pipe, 'rebuilt from its buffer');
  assert.equal(back.mesh.geometry.getAttribute('position').count, triangles);
  assert.equal(clamp.lockedTo, back);
  assert.deepEqual(State.importedSTLs.map(o => o.name), ['Cylinder', 'Cube'], 'list order kept');
  await redo(opts);
  assert.equal(State.importedSTLs.length, 1);
});

test('a removed device comes back with its pose and parent', async () => {
  const { e, opts } = await scene();
  const a = await e.addDevice('meca500_config.json');
  const b = await e.addDevice('meca500_config.json');
  b.rootGroup.position.set(0.5, 0, 0.2);
  b.jointAngles[0] = 0.7;
  updateFK(b);
  const cube = await e.addPrimitive('cube');
  await e.handle({ cmd: 'setObject', index: 0, parent: `${b.id}:L2` });
  resetUndo();
  const before = sceneSnapshot();

  act(() => { removeSTLEntry(cube); removeDevice(b); });
  assert.equal(State.devices.length, 1);
  await undo(opts);
  assert.equal(sceneSnapshot(), before);
  const b2 = State.devices[1];
  assert.notEqual(b2, b);
  assert.equal(b2.jointAngles[0], 0.7);
  assert.equal(State.importedSTLs[0].parentLink, `${b2.id}:L2`, 'parent rebuilt on the new device');
  assert.equal(State.devices[0], a);
});

test('after an undo, a lock group stays still and arms are not dragged back', async () => {
  const { e, opts } = await scene();
  const a = await e.addDevice('meca500_config.json');
  [0, -20, 30, 0, 40, 0].forEach((deg, i) => { a.jointAngles[i] = deg * Math.PI / 180; });
  updateFK(a);
  const pipe = await e.addPrimitive('cylinder');
  const clamp = await e.addPrimitive('cube');
  e.State.scene.updateMatrixWorld(true);
  a.ikMode = true;
  a.ikTarget.position.copy(getEEWorldPosition(a));
  a.ikTargetQuat.copy(getEEWorldQuaternion(a));
  setIKLock(a, pipe);
  setObjectLock(clamp, pipe);
  updateLocks();
  resetUndo();
  const before = sceneSnapshot();

  act(() => { pipe.mesh.position.x += 0.02; updateLocks(); a.jointAngles[0] += 0.05; updateFK(a); });
  await undo(opts);
  assert.deepEqual(updateLocks(), { moved: [], objectsMoved: false });
  assert.equal(sceneSnapshot(), before);
  assert.ok(getEEWorldPosition(a).distanceTo(a.ikTarget.position) < 1e-9, 'target is on the restored hand');
  assert.equal(a.ikLock.entry, pipe);
});

test('undoing a lock removes it, for objects and arms', async () => {
  const { e, opts } = await scene();
  const a = await e.addDevice('meca500_config.json');
  const pipe = await e.addPrimitive('cylinder');
  const clamp = await e.addPrimitive('cube');
  resetUndo();
  act(() => { a.ikMode = true; setIKLock(a, pipe); setObjectLock(clamp, pipe); });
  await undo(opts);
  assert.equal(a.ikLock, null);
  assert.equal(clamp.lockedTo, null);
  await redo(opts);
  assert.equal(a.ikLock?.entry, pipe);
  assert.equal(clamp.lockedTo, pipe);
});
