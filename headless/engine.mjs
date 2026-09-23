// ============================================================
// headless/engine.mjs — the viewer's scene and collision checking,
//                       without a page
// ------------------------------------------------------------
// Devices, objects, kinematics, commands and collision all run through the
// viewer's own modules: chain.js and model.js build devices, stl.js builds
// objects, websocket.js answers commands, collision.js finds collisions.
// Only the page (rendering, panels, labels) is left out, so a verdict here
// is the verdict the viewer reaches for the same scene.
//
// Needs the import-map hook: node --import ./headless/register.mjs …
//
//   const engine = await createEngine();
//   await engine.handle({ cmd: 'addDevice', config: 'meca500_config.json' });
//   await engine.handle({ cmd: 'setJoints', angles: [0, 30, 60, 0, 45, 90] });
//   const [reply] = await engine.handle({ cmd: 'getCollisions' });
// ============================================================
import { readFileSync, readdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const REPO_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

// A page with none of the viewer's elements on it. The viewer's modules look
// elements up by id and leave them alone when they are missing.
globalThis.document ??= { getElementById: () => null };
// GLTFLoader decodes embedded textures with browser image APIs, reached
// through `self`. Collision needs no textures; this lets the load finish
// (the texture itself fails, with a warning).
globalThis.self ??= globalThis;
globalThis.WebSocket ??= { OPEN: 1 };

const THREE = await import('three');
const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js');
const State = await import('../js/state.js');
const { updateFK } = await import('../js/kinematics.js');
const { assembleDevice } = await import('../js/model.js');
const { setDeviceParent } = await import('../js/panel.js');
const {
  createMeshEntry, parseMeshGeometry, primitiveSTLBuffer, applySavedObjectState,
} = await import('../js/stl.js');
const { checkCollisionsNow } = await import('../js/collision.js');
const {
  handleCommand, setApiEnabled, buildDeviceInfo, buildObjectInfo, registerAvailableConfigs,
} = await import('../js/websocket.js');

// Commands that need a renderer or a person at the page.
const PAGE_ONLY = new Set([
  'captureImage', 'getStats', 'saveScene', 'getCamera', 'setCamera', 'snapCamera',
  'setOrtho', 'setLabels', 'setOrigins', 'setChain', 'setDeviceTransparency',
  'demoPose', 'setCollisionHeadless',
]);

// Commands that change the scene, after which the standing collision
// result is recomputed.
const READ_ONLY = new Set([
  'getState', 'listDevices', 'getDevice', 'listConfigs', 'getVirtualAngles',
  'getLegLengths', 'getCollisions', 'listObjects', 'getObject', 'getSceneState',
  'worldToLocal', 'help', 'listCommands', 'hexapodFK', 'hexapodIK',
]);

function toArrayBuffer(buf) {
  return buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
}

export async function createEngine({ root = REPO_ROOT } = {}) {
  const scene = new THREE.Scene();
  State.initCoreObjects(scene, null, null, null);
  State.devices.length = 0;
  State.importedSTLs.length = 0;
  State.resetDeviceIdCounter();
  State.setActiveDevice(null);
  // Collision is what this engine is for, so it is always on.
  State.setCollisionEnabled(true);
  setApiEnabled(true);
  registerAvailableConfigs(readdirSync(root).filter(f => f.endsWith('_config.json')).sort());

  let replies = [];
  State.setWs({ readyState: 1, send: (s) => replies.push(JSON.parse(s)) });

  const read = (file) => readFileSync(path.resolve(root, file));

  async function addDevice(configFile) {
    const config = JSON.parse(read(configFile).toString('utf8'));
    if (config.type === 'hexapod') {
      throw new Error(`${configFile}: hexapods are not supported headless yet`);
    }
    const gltf = await new Promise((resolve, reject) =>
      new GLTFLoader().parse(toArrayBuffer(read(config.model)), '', resolve, reject));
    const dev = assembleDevice(config, {
      id: State.incrementDeviceId(), configFile, gltfScene: gltf.scene,
    });
    State.scene.add(dev.rootGroup);
    State.devices.push(dev);
    if (!State.activeDevice) State.setActiveDevice(dev);
    updateFK(dev);
    return dev;
  }

  function addMesh(geometry, buffer, fileType, name) {
    const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
    return createMeshEntry(geometry, buffer, fileType, name, 0x44aaff, stlId, null);
  }

  async function addPrimitive(type) {
    const { buffer, name } = primitiveSTLBuffer(type);
    return addMesh(await parseMeshGeometry(buffer, 'stl'), buffer, 'stl', name);
  }

  // Load a scene saved by the viewer (Save Scene, or its auto-save format).
  // Point clouds and splats are returned in `skipped`: they are not
  // supported headless yet, and a check without them is not the viewer's.
  async function loadScene(payload) {
    const skipped = [];
    for (const d of payload.devices || []) {
      const dev = await addDevice(d.configFile);
      if (d.name) dev.name = d.name;
      if (d.jointAngles) {
        for (let i = 0; i < d.jointAngles.length && i < dev.jointAngles.length; i++) {
          dev.jointAngles[i] = d.jointAngles[i];
        }
      }
      if (d.position) dev.rootGroup.position.set(...d.position);
      if (d.rotation) dev.rootGroup.rotation.set(...d.rotation);
      if (d.visible !== undefined) dev.rootGroup.visible = d.visible;
      updateFK(dev);
    }
    // Device parents, once every device exists ("index:link" in saves).
    (payload.devices || []).forEach((d, i) => {
      if (!d.parentLink || !d.parentLink.includes(':')) return;
      const [idx, linkName] = d.parentLink.split(':', 2);
      const parent = State.devices[parseInt(idx, 10)];
      if (parent) setDeviceParent(State.devices[i], parent.id + ':' + linkName, true);
    });
    State.scene.updateMatrixWorld(true);

    for (const rec of payload.stls || []) {
      if (rec.isPointCloud || rec.isSplat) { skipped.push(rec.name); continue; }
      const buffer = rec.buffer instanceof ArrayBuffer
        ? rec.buffer : toArrayBuffer(Buffer.from(rec.buffer, 'base64'));
      const fileType = rec.fileType || 'stl';
      const entry = addMesh(await parseMeshGeometry(buffer, fileType), buffer, fileType, rec.name);
      entry.stlId = rec.id ?? entry.stlId;
      applySavedObjectState(entry, rec);
    }
    checkCollisionsNow();
    return { devices: State.devices.length, objects: State.importedSTLs.length, skipped };
  }

  // Answer one command in the viewer's WebSocket protocol. Returns the
  // replies the viewer would have sent.
  async function handle(msg) {
    replies = [];
    const { cmd } = msg;
    if (PAGE_ONLY.has(cmd)) {
      return [{ type: 'error', error: `'${cmd}' needs the viewer page; not available headless`, _reqId: msg._reqId }];
    }

    // Bring the standing result up to date first, so queries (getCollisions,
    // getState) answer for the scene as it is now.
    checkCollisionsNow();

    if (cmd === 'addDevice') {
      const dev = await addDevice(msg.config);
      replies.push({ type: 'deviceAdded', ...buildDeviceInfo(dev) });
    } else if (cmd === 'addPrimitive') {
      const type = (msg.type || msg.primitive || 'cube').toLowerCase();
      if (!['cube', 'sphere', 'cylinder'].includes(type)) {
        replies.push({ type: 'error', error: 'Invalid primitive type. Use: cube, sphere, cylinder' });
      } else {
        const entry = await addPrimitive(type);
        replies.push({ type: 'objectAdded', ...buildObjectInfo(entry, State.importedSTLs.indexOf(entry)) });
      }
    } else if (cmd === 'loadScene') {
      replies.push({ type: 'sceneLoaded', ...(await loadScene(msg.scene)) });
    } else {
      handleCommand(msg);
    }

    if (!READ_ONLY.has(cmd)) checkCollisionsNow();
    return replies;
  }

  function collisions() {
    return checkCollisionsNow().map(c => ({ link: c.linkName, object: c.stlName }));
  }

  return { handle, addDevice, addPrimitive, loadScene, collisions, State, scene };
}
