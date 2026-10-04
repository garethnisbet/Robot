// A lock group is collision-checked as one body, the way an object
// parented to a link is: against point clouds, objects, devices and other
// groups, but not against itself or the arms that grip it.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createEngine } from '../../headless/engine.mjs';
import { createPointsEntry, primitiveSTLBuffer } from '../../js/stl.js';
import { setObjectLock, setIKLock } from '../../js/locks.js';
import { getEEWorldPosition, getEEWorldQuaternion } from '../../js/kinematics.js';

async function pairs(e) {
  const [r] = await e.handle({ cmd: 'getCollisions' });
  return r.pairs.map(p => [p.link, p.object].sort().join('↔')).sort();
}

// A block of points, 5 mm apart, 0.2 m on a side, centred on c (world, m).
function cloudAt(e, c) {
  const pts = [];
  for (let i = -20; i <= 20; i++) for (let j = -20; j <= 20; j++) for (let k = -20; k <= 20; k++) {
    pts.push(c[0] + i * 0.005, c[1] + j * 0.005, c[2] + k * 0.005);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  g.computeBoundingBox();
  return createPointsEntry(g, null, 'Scan', 0x888888, 'scan', null);
}

// Three.js metres (Y up) from API mm (Z up).
const place = (entry, mm) => entry.mesh.position.set(mm[0] / 1000, mm[2] / 1000, mm[1] / 1000);

test('a world object meets a point cloud only once it is in a lock group', async () => {
  const e = await createEngine();
  cloudAt(e, [0.5, 0.5, 0.5]);
  const pipe = await e.addPrimitive('cylinder');
  const tag = await e.addPrimitive('sphere');
  place(pipe, [500, 500, 500]);
  place(tag, [800, 800, 800]);
  assert.deepEqual(await pairs(e), [], 'world objects are not checked against clouds');
  setObjectLock(tag, pipe);
  assert.deepEqual(await pairs(e), ['Cylinder↔Scan']);
});

test('members of a group do not collide with each other, but do with others', async () => {
  const e = await createEngine();
  const pipe = await e.addPrimitive('cylinder');
  const clamp = await e.addPrimitive('cube');
  const other = await e.addPrimitive('sphere');
  place(pipe, [600, 0, 400]);
  place(clamp, [610, 0, 410]);          // overlapping the pipe
  place(other, [2000, 0, 400]);
  setObjectLock(clamp, pipe);
  assert.deepEqual(await pairs(e), []);
  place(other, [620, 20, 400]);         // now through the group
  const hits = await pairs(e);
  assert.ok(hits.length > 0 && hits.every(h => h.includes('Sphere')), hits.join(', '));
});

test('an arm does not collide with what it grips, but the rest of it does', async () => {
  const e = await createEngine();
  await e.handle({ cmd: 'addDevice', config: 'meca500_config.json' });
  const arm = e.State.devices[0];
  const part = await e.addPrimitive('cube');
  place(part, [190, 0, 308]);           // at the flange
  const touching = await pairs(e);
  assert.ok(touching.length > 0, 'the cube touches the wrist');
  e.State.scene.updateMatrixWorld(true);
  arm.ikMode = true;
  arm.ikTarget.position.copy(getEEWorldPosition(arm));
  arm.ikTargetQuat.copy(getEEWorldQuaternion(arm));
  setIKLock(arm, part);
  const held = await pairs(e);
  assert.ok(held.length < touching.length, `${held} vs ${touching}`);
  assert.ok(!held.some(h => /L5/.test(h)), `grip link reported: ${held}`);

  // And the held part meets a scan the arm carries it into.
  cloudAt(e, [0.19, 0.308, 0]);
  assert.ok((await pairs(e)).includes('Cube↔Scan'));
});

test('a cloud locked into a group is not checked against it', async () => {
  const e = await createEngine();
  const scan = cloudAt(e, [0.5, 0.5, 0.5]);
  const pipe = await e.addPrimitive('cylinder');
  const tag = await e.addPrimitive('sphere');
  place(pipe, [500, 500, 500]);
  place(tag, [800, 800, 800]);
  setObjectLock(tag, pipe);
  setObjectLock(scan, pipe);
  assert.deepEqual(await pairs(e), []);
});

test('the headless engine takes the locks from a viewer scene', async () => {
  const e = await createEngine();
  const b64 = type => Buffer.from(primitiveSTLBuffer(type).buffer).toString('base64');
  const rec = (id, name, type, pos) => ({ id, name, fileType: 'stl', buffer: b64(type),
    position: pos, rotation: [0, 0, 0], scale: [1, 1, 1], visible: true });
  const scene = {
    version: 1, devices: [],
    stls: [rec('a', 'Pipe', 'cylinder', [0.6, 0.4, 0]), rec('b', 'Clamp', 'cube', [0.61, 0.41, 0])],
  };
  await e.handle({ cmd: 'loadScene', scene });
  assert.deepEqual(await pairs(e), []);
  scene.stls.push(rec('c', 'Ball', 'sphere', [0.62, 0.4, 0]));
  scene.stls[1].lockedTo = 'a';
  await e.handle({ cmd: 'loadScene', scene });
  const hits = await pairs(e);
  assert.ok(hits.length > 0 && hits.every(h => h.includes('Ball')), hits.join(', '));
});
