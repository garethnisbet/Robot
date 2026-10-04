// Object origins, and IK targets locked to objects.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import * as State from '../../js/state.js';
import { createEngine } from '../../headless/engine.mjs';
import {
  setObjectOrigin, originPresetPoint, applySavedObjectState, buildScenePayloadForDB,
  createMeshEntry, parseMeshGeometry, primitiveSTLBuffer,
} from '../../js/stl.js';
import { getEEWorldPosition, getEEWorldQuaternion, solveIK, updateFK } from '../../js/kinematics.js';
import { setIKLock, updateIKLocks, lockableObjects } from '../../js/ik-lock.js';

// World positions of a mesh's vertices.
function worldVertices(entry) {
  entry.mesh.updateWorldMatrix(true, false);
  const pos = entry.mesh.geometry.getAttribute('position');
  const v = new THREE.Vector3();
  const out = [];
  for (let i = 0; i < pos.count; i++) out.push(v.fromBufferAttribute(pos, i).applyMatrix4(entry.mesh.matrixWorld).toArray());
  return out;
}

function assertClose(a, b, tol, msg) {
  assert.equal(a.length, b.length);
  a.flat().forEach((x, i) => assert.ok(Math.abs(x - b.flat()[i]) < tol, `${msg}: ${x} vs ${b.flat()[i]}`));
}

async function pairs(e) {
  const [r] = await e.handle({ cmd: 'getCollisions' });
  return r.pairs.map(p => [p.link, p.object].sort().join('↔')).sort();
}

test('moving the origin leaves the object where it is, and it now turns about the new origin', async () => {
  const e = await createEngine();
  const cube = await e.addPrimitive('cube');
  cube.mesh.position.set(0.3, 0.1, -0.2);
  cube.mesh.rotation.set(0.4, -0.7, 1.1);
  cube.mesh.scale.set(0.002, 0.001, 0.003);
  const before = worldVertices(cube);

  const corner = new THREE.Vector3(20, -10, 5);       // local units
  const cornerWorld = corner.clone().applyMatrix4(cube.mesh.matrixWorld);
  setObjectOrigin(cube, corner);
  assertClose(worldVertices(cube), before, 1e-9, 'vertex moved');
  // The object's position is now the world position of that corner.
  const p = new THREE.Vector3().setFromMatrixPosition(cube.mesh.matrixWorld);
  assert.ok(p.distanceTo(cornerWorld) < 1e-12, `${p.toArray()} vs ${cornerWorld.toArray()}`);
  const turned = cube.mesh.clone();
  turned.rotation.y += 1;
  turned.updateMatrixWorld(true);
  assert.ok(new THREE.Vector3().setFromMatrixPosition(turned.matrixWorld).distanceTo(p) < 1e-12,
    'rotating moves the origin');

  // Back to the file's origin restores the original geometry.
  setObjectOrigin(cube, originPresetPoint(cube, 'file'));
  assertClose(worldVertices(cube), before, 1e-9, 'vertex moved');
  assert.ok(cube.origin.length() < 1e-9);
});

test('the centre preset puts the origin at the middle of the box', async () => {
  const e = await createEngine();
  const cube = await e.addPrimitive('cube');
  setObjectOrigin(cube, new THREE.Vector3(30, 30, 30));
  setObjectOrigin(cube, originPresetPoint(cube, 'centre'));
  cube.mesh.geometry.computeBoundingBox();
  const c = cube.mesh.geometry.boundingBox.getCenter(new THREE.Vector3());
  assert.ok(c.length() < 1e-9, c.toArray().join());
});

test('a saved origin is restored, and applying the same record again changes nothing', async () => {
  const e = await createEngine();
  const cube = await e.addPrimitive('cube');
  cube.mesh.position.set(0.1, 0.2, 0.3);
  setObjectOrigin(cube, new THREE.Vector3(10, 0, -25));
  // Save Scene reads the camera, which the engine has none of.
  State.initCoreObjects(e.scene, new THREE.PerspectiveCamera(), null, null);
  State.initControls({ target: new THREE.Vector3() }, null, null, null);
  const rec = buildScenePayloadForDB().stls[0];
  assert.deepEqual(rec.origin, [10, 0, -25]);

  const { buffer } = primitiveSTLBuffer('cube');
  const copy = createMeshEntry(await parseMeshGeometry(buffer, 'stl'), buffer, 'stl', 'copy', 0, 'copy', null);
  applySavedObjectState(copy, rec);
  applySavedObjectState(copy, rec);
  assertClose(worldVertices(copy), worldVertices(cube), 1e-9, 'restore differs');
});

test('collisions follow the moved geometry', async () => {
  const e = await createEngine();
  await e.handle({ cmd: 'addDevice', config: 'meca500_config.json' });
  const cube = await e.addPrimitive('cube');
  await e.handle({ cmd: 'setObject', index: 0, position: [190, 0, 308] });
  assert.ok((await pairs(e)).length > 0);
  // Origin moved 0.5 m away (in local units) and the cube put back at its
  // new origin's old spot: the geometry is now half a metre off the flange.
  setObjectOrigin(cube, new THREE.Vector3(500 / cube.mesh.scale.x / 1000, 0, 0));
  assert.ok((await pairs(e)).length > 0, 'setting the origin alone must not move it');
  cube.mesh.position.set(0.19, 0.308, 0);
  assert.deepEqual(await pairs(e), []);
});

test('two arms locked to one object follow it together', async () => {
  const e = await createEngine();
  const a = await e.addDevice('meca500_config.json');
  const b = await e.addDevice('meca500_config.json');
  b.rootGroup.position.set(0.5, 0, 0);
  b.rootGroup.rotation.y = Math.PI;
  const pipe = await e.addPrimitive('cylinder');
  pipe.mesh.position.set(0.25, 0.3, 0);
  e.State.scene.updateMatrixWorld(true);

  // Start clear of the home pose's wrist singularity (J5 = 0).
  for (const d of [a, b]) {
    [0, -20, 30, 0, 40, 0].forEach((deg, i) => { d.jointAngles[i] = deg * Math.PI / 180; });
    updateFK(d);
  }
  e.State.scene.updateMatrixWorld(true);
  for (const d of [a, b]) {
    d.ikMode = true;
    d.ikTarget.position.copy(getEEWorldPosition(d));
    d.ikTargetQuat.copy(getEEWorldQuaternion(d));
    setIKLock(d, pipe);
  }
  const grip = d => new THREE.Matrix4().compose(d.ikTarget.position, d.ikTargetQuat.clone().normalize(), new THREE.Vector3(1, 1, 1));
  const pipePose = () => new THREE.Matrix4().compose(pipe.mesh.position, pipe.mesh.quaternion, new THREE.Vector3(1, 1, 1));
  const rel = d => pipePose().invert().multiply(grip(d));
  const relA = rel(a), relB = rel(b);

  assert.deepEqual(updateIKLocks(), { moved: [], objectsMoved: false }, 'nothing moved yet');
  pipe.mesh.position.y += 0.03;
  pipe.mesh.rotation.x += 0.2;
  assert.deepEqual(updateIKLocks(), { moved: [a, b], objectsMoved: false });
  for (const [d, r] of [[a, relA], [b, relB]]) {
    assertClose(rel(d).toArray(), r.toArray(), 1e-9, 'grip slipped');
    const err = solveIK(d, d.ikTarget.position, d.ikTargetQuat, 200, 0.00005);
    assert.ok(err < 1e-3, `IK error ${err}`);
  }

  // Moving one arm's target carries the object, and the other arm with it.
  a.ikTarget.position.x += 0.01;
  a.ikTargetQuat.premultiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), 0.1));
  assert.deepEqual(updateIKLocks(), { moved: [b], objectsMoved: true });
  assertClose(rel(a).toArray(), relA.toArray(), 1e-9, 'driving grip slipped');
  assertClose(rel(b).toArray(), relB.toArray(), 1e-9, 'following grip slipped');
  assert.deepEqual(updateIKLocks(), { moved: [], objectsMoved: false }, 'settled after one frame');

  // Leaving IK mode drops the lock.
  b.ikMode = false;
  updateIKLocks();
  assert.equal(b.ikLock, null);
});

test('an arm drives an object that a third arm carries', async () => {
  const e = await createEngine();
  const a = await e.addDevice('meca500_config.json');
  const c = await e.addDevice('meca500_config.json');
  c.rootGroup.position.set(0, 0, 0.6);
  const cube = await e.addPrimitive('cube');
  await e.handle({ cmd: 'setObject', index: 0, position: [250, 300, 300] });
  await e.handle({ cmd: 'setObject', index: 0, parent: `${c.id}:L5` });
  e.State.scene.updateMatrixWorld(true);
  const world = () => { cube.mesh.updateWorldMatrix(true, false); return cube.mesh.matrixWorld.clone(); };
  const before = world();

  a.ikMode = true;
  a.ikTarget.position.set(0.2, 0.25, 0.05);
  a.ikTargetQuat.identity();
  setIKLock(a, cube);
  a.ikTarget.position.y += 0.02;
  assert.equal(updateIKLocks().objectsMoved, true);
  // Moved 20 mm up in the world, scale and parent unchanged.
  const expected = before.clone().premultiply(new THREE.Matrix4().makeTranslation(0, 0.02, 0));
  assertClose(world().toArray(), expected.toArray(), 1e-9, 'object pose');
  assert.equal(cube.parentLink, `${c.id}:L5`);

  // And the carrying arm moving the object moves the locked arm.
  const target0 = a.ikTarget.position.clone();
  await e.handle({ cmd: 'setJoints', device: c.id, angles: [20, 0, 0, 0, 0, 0] });
  assert.deepEqual(updateIKLocks().moved, [a]);
  assert.ok(a.ikTarget.position.distanceTo(target0) > 0.01);
});

test('an arm cannot lock to an object it carries', async () => {
  const e = await createEngine();
  const a = await e.addDevice('meca500_config.json');
  const cube = await e.addPrimitive('cube');
  assert.deepEqual(lockableObjects(a), [cube]);
  await e.handle({ cmd: 'setObject', index: 0, parent: `${a.id}:L5` });
  assert.deepEqual(lockableObjects(a), []);
  setIKLock(a, cube);
  assert.equal(a.ikLock, null);
});
