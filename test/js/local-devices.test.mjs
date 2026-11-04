import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
  importLocalDeviceFiles, readDeviceConfig, loadingManagerFor,
  looksLikeDeviceConfig, isLocalConfig,
} from '../../js/local-devices.js';

// GLTFLoader reaches browser image APIs through `self` (as headless/engine.mjs).
globalThis.self ??= globalThis;
const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js');

const root = new URL('../../', import.meta.url);
const file = (name) => new File([readFileSync(new URL(name, root))], name);

test('device configs are told apart from saved scenes', () => {
  assert.ok(looksLikeDeviceConfig(JSON.parse(readFileSync(new URL('meca500_config.json', root)))));
  assert.ok(looksLikeDeviceConfig(JSON.parse(readFileSync(new URL('hexapod_config.json', root)))));
  assert.ok(!looksLikeDeviceConfig({ version: 3, devices: [], stls: [] }));
});

test('a config without its model is refused, naming the missing file', async () => {
  await assert.rejects(importLocalDeviceFiles([file('meca500_config.json')]), /meca500_scene\.glb/);
});

test('an imported device reads back and its model parses through the local loader', async () => {
  const cfg = file('meca500_config.json');
  const glb = file('meca500_scene.glb');
  const other = new File(['x'], 'notes.txt');
  const { keys, used } = await importLocalDeviceFiles([cfg, glb, other]);
  assert.deepEqual(keys, ['local:meca500_config.json']);
  assert.ok(isLocalConfig(keys[0]));
  assert.ok(used.has(cfg) && used.has(glb) && !used.has(other));

  const config = await readDeviceConfig(keys[0]);
  assert.equal(config.model, 'meca500_scene.glb');

  // Resolve the model through the manager as the viewer does, then parse it.
  const manager = loadingManagerFor(keys[0]);
  const url = manager.resolveURL(config.model);
  assert.match(url, /^blob:/);
  const buf = await (await fetch(url)).arrayBuffer();
  const gltf = await new GLTFLoader(manager).parseAsync(buf, '');
  const names = [];
  gltf.scene.traverse(c => c.name && names.push(c.name));
  for (const link of config.links) assert.ok(names.includes(link.name), `${link.name} in model`);
});
