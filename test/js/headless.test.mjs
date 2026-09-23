// The headless engine: the viewer's collision checking with no page.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createEngine } from '../../headless/engine.mjs';
import { primitiveSTLBuffer } from '../../js/stl.js';

async function meca() {
  const e = await createEngine();
  await e.handle({ cmd: 'addDevice', config: 'meca500_config.json' });
  return e;
}

async function pairs(e) {
  const [r] = await e.handle({ cmd: 'getCollisions' });
  assert.equal(r.type, 'collisions');
  assert.equal(r.current, true, 'result must describe the current scene');
  return r.pairs.map(p => [p.link, p.object].sort().join('↔')).sort();
}

test('the arm at home collides with nothing', async () => {
  assert.deepEqual(await pairs(await meca()), []);
});

test('a cube at the flange hits the wrist; moved away, nothing does', async () => {
  const e = await meca();
  await e.handle({ cmd: 'addPrimitive', type: 'cube' });
  await e.handle({ cmd: 'setObject', index: 0, position: [190, 0, 308] });
  const hits = await pairs(e);
  assert.ok(hits.length > 0 && hits.every(h => h.includes('Cube')), hits.join(', '));
  await e.handle({ cmd: 'setObject', index: 0, position: [500, 500, 500] });
  assert.deepEqual(await pairs(e), []);
});

test('a cube resting through the floor plane reports the floor, until floor checks are off', async () => {
  const e = await meca();
  await e.handle({ cmd: 'addPrimitive', type: 'cube' });
  await e.handle({ cmd: 'setObject', index: 0, position: [400, 0, 0] });   // half below z = 0
  assert.deepEqual(await pairs(e), ['Cube↔floor']);
  await e.handle({ cmd: 'setFloorCollision', enabled: false });
  assert.deepEqual(await pairs(e), []);
});

test('folding the arm onto its base is a self-collision', async () => {
  const e = await meca();
  await e.handle({ cmd: 'setJoints', angles: [0, 60, 60, 0, 60, 0] });
  assert.deepEqual(await pairs(e), ['L0↔L4']);
});

test('an object parented to a link rides with it', async () => {
  const e = await meca();
  await e.handle({ cmd: 'addPrimitive', type: 'cube' });
  await e.handle({ cmd: 'setObject', index: 0, position: [300, 0, 450] });   // clear of the arm
  const where = async () => (await e.handle({ cmd: 'getObject', index: 0 }))[0].worldPosition;

  // Unparented, it stays put when J1 turns.
  await e.handle({ cmd: 'setJoints', angles: [90, 0, 0, 0, 0, 0] });
  assert.deepEqual((await where()).map(Math.round), [300, 0, 450]);
  await e.handle({ cmd: 'home' });

  // Parented to L1 (downstream of J1), keeping its world pose, it swings
  // round with J1: same height and radius, a quarter turn away.
  const [dev] = (await e.handle({ cmd: 'listDevices' }))[0].devices;
  await e.handle({ cmd: 'setObject', index: 0, parent: `${dev.id}:L1` });
  assert.deepEqual((await where()).map(Math.round), [300, 0, 450]);
  await e.handle({ cmd: 'setJoints', angles: [90, 0, 0, 0, 0, 0] });
  const [x, y, z] = await where();
  assert.ok(Math.abs(x) < 0.01 && Math.abs(Math.abs(y) - 300) < 0.01 && Math.abs(z - 450) < 0.01,
    `${[x, y, z]}`);
  assert.deepEqual(await pairs(e), []);
});

test('a hidden object drops out of the check', async () => {
  const e = await meca();
  await e.handle({ cmd: 'addPrimitive', type: 'cube' });
  await e.handle({ cmd: 'setObject', index: 0, position: [190, 0, 308] });
  assert.notDeepEqual(await pairs(e), []);
  await e.handle({ cmd: 'setObject', index: 0, visible: false });
  assert.deepEqual(await pairs(e), []);
});

test('two arms facing each other collide when they reach across', async () => {
  const e = await createEngine();
  await e.handle({ cmd: 'addDevice', config: 'meca500_config.json' });
  const [b] = await e.handle({ cmd: 'addDevice', config: 'meca500_config.json' });
  await e.handle({ cmd: 'setDeviceOrigin', device: b.id, position: [420, 0, 0], rotation: [0, 0, 180] });
  assert.deepEqual(await pairs(e), []);
  await e.handle({ cmd: 'setJoints', device: 'dev_0', angles: [0, 45, -20, 0, 0, 0] });
  await e.handle({ cmd: 'setJoints', device: b.id, angles: [0, 45, -20, 0, 0, 0] });
  const hits = await pairs(e);
  assert.ok(hits.length > 0 && hits.every(h => h.includes(':')), hits.join(', '));
});

test('a saved scene loads with its devices, poses and objects', async () => {
  const { buffer } = primitiveSTLBuffer('cube');
  const e = await createEngine();
  const [r] = await e.handle({ cmd: 'loadScene', scene: {
    version: 1,
    devices: [{ configFile: 'meca500_config.json', jointAngles: [0, 0, 0, 0, 0, 0],
                position: [0, 0, 0], rotation: [0, 0, 0], visible: true }],
    stls: [{ id: 'a', name: 'Block', fileType: 'stl', buffer: Buffer.from(buffer).toString('base64'),
             position: [0.19, 0.308, 0], rotation: [0, 0, 0], scale: [1, 1, 1], visible: true }],
  } });
  assert.deepEqual([r.devices, r.objects, r.skipped], [1, 1, []]);
  assert.ok((await pairs(e)).some(p => p.includes('Block')));
});

test('commands that need the page say so', async () => {
  const [r] = await (await meca()).handle({ cmd: 'captureImage' });
  assert.equal(r.type, 'error');
  assert.match(r.error, /headless/);
});
