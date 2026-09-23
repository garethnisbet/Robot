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
const { clampJoints } = await import('../js/kinematics.js');
const { setDeviceParent } = await import('../js/panel.js');
const {
  createMeshEntry, parseMeshGeometry, primitiveSTLBuffer, applySavedObjectState, setSTLParent,
} = await import('../js/stl.js');
const { checkCollisionsNow } = await import('../js/collision.js');
const {
  handleCommand, setApiEnabled, buildDeviceInfo, buildObjectInfo, registerAvailableConfigs,
} = await import('../js/websocket.js');

// Commands that need a renderer or a person at the page.
const PAGE_ONLY = new Set([
  'captureImage', 'getStats', 'saveScene', 'getCamera', 'setCamera', 'snapCamera',
  'setOrtho', 'setLabels', 'setOrigins', 'setChain', 'setDeviceTransparency',
  'demoPose', 'setCollisionHeadless', 'exportScene',
]);

// Commands that change the scene, after which the standing collision
// result is recomputed.
const READ_ONLY = new Set([
  'getState', 'listDevices', 'getDevice', 'listConfigs', 'getVirtualAngles',
  'getLegLengths', 'getCollisions', 'listObjects', 'getObject', 'getSceneState',
  'worldToLocal', 'help', 'listCommands', 'hexapodFK', 'hexapodIK', 'checkPath',
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

  // Empty the scene: every device and object goes.
  function reset() {
    for (const dev of State.devices) dev.rootGroup.removeFromParent();
    for (const e of State.importedSTLs) e.mesh.removeFromParent();
    State.devices.length = 0;
    State.importedSTLs.length = 0;
    State.resetDeviceIdCounter();
    State.setActiveDevice(null);
    loadedKey = null;
    skipped = [];
  }

  // What a scene is made of, as opposed to where things are: the device
  // configs and the objects. While it is unchanged, a sync only moves
  // things; otherwise the scene is rebuilt from the geometry.
  const structureKey = (payload) => JSON.stringify([
    (payload.devices || []).map(d => d.configFile),
    (payload.stls || []).map(r => [r.id, r.fileType || 'stl', !!r.isPointCloud, !!r.isSplat]),
  ]);
  let loadedKey = null;
  let skipped = [];          // [{ name, kind, visible }] not built headless
  let deviceIndexMap = new Map();   // viewer's device index -> State.devices index

  // Saves refer to devices by index ("2:linkName"). Skipped devices shift
  // the indices, so references are rewritten to the devices actually built
  // (null when the parent itself was skipped).
  function remapParent(ref, indexMap) {
    if (!ref || !ref.includes(':')) return ref;
    const [idx, link] = ref.split(':', 2);
    const to = indexMap.get(parseInt(idx, 10));
    return to === undefined ? null : `${to}:${link}`;
  }

  function applyDeviceState(dev, d) {
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

  // Load a scene saved by the viewer (Save Scene, its auto-save format, or
  // exportScene). Hexapods, point clouds and splats are not built; they are
  // listed in `skipped`, since a check without them is not the viewer's.
  async function loadScene(payload) {
    reset();
    const indexMap = new Map();
    const devices = payload.devices || [];
    for (let i = 0; i < devices.length; i++) {
      const config = JSON.parse(read(devices[i].configFile).toString('utf8'));
      if (config.type === 'hexapod') {
        skipped.push({ name: devices[i].name || config.name, kind: 'hexapod', visible: devices[i].visible !== false });
        continue;
      }
      indexMap.set(i, State.devices.length);
      applyDeviceState(await addDevice(devices[i].configFile), devices[i]);
    }
    deviceIndexMap = indexMap;
    devices.forEach((d, i) => {
      if (!indexMap.has(i)) return;
      const ref = remapParent(d.parentLink, indexMap);
      if (!ref) return;
      const [idx, link] = ref.split(':', 2);
      setDeviceParent(State.devices[indexMap.get(i)], State.devices[idx].id + ':' + link, true);
    });
    State.scene.updateMatrixWorld(true);

    for (const rec of payload.stls || []) {
      if (rec.isPointCloud || rec.isSplat) {
        skipped.push({ name: rec.name, kind: rec.isSplat ? 'splat' : 'point cloud', visible: rec.visible !== false });
        continue;
      }
      if (!rec.buffer) throw new Error(`object '${rec.name}' has no geometry in the scene payload`);
      const buffer = rec.buffer instanceof ArrayBuffer
        ? rec.buffer : toArrayBuffer(Buffer.from(rec.buffer, 'base64'));
      const fileType = rec.fileType || 'stl';
      const entry = addMesh(await parseMeshGeometry(buffer, fileType), buffer, fileType, rec.name);
      entry.stlId = rec.id ?? entry.stlId;
      applySavedObjectState(entry, { ...rec, parentLink: remapParent(rec.parentLink, indexMap) });
    }
    loadedKey = structureKey(payload);
    checkCollisionsNow();
    return { rebuilt: true, devices: State.devices.length, objects: State.importedSTLs.length, skipped };
  }

  // Bring the scene in line with a viewer's: moves only while the structure
  // is unchanged, rebuilds otherwise. A payload without geometry can only
  // move things; if a rebuild is needed it answers needBuffers.
  async function syncScene(payload) {
    if (structureKey(payload) !== loadedKey) {
      const meshes = (payload.stls || []).filter(r => !r.isPointCloud && !r.isSplat);
      if (meshes.some(r => !r.buffer)) return { needBuffers: true };
      return loadScene(payload);
    }
    const devices = payload.devices || [];
    const indexMap = new Map();
    let built = 0;
    devices.forEach((d, i) => {
      if (skipped.some(s => s.kind === 'hexapod' && s.name === d.name)) return;
      indexMap.set(i, built++);
    });
    devices.forEach((d, i) => {
      if (!indexMap.has(i)) return;
      const dev = State.devices[indexMap.get(i)];
      applyDeviceState(dev, d);
      const ref = remapParent(d.parentLink, indexMap);
      const [idx, link] = ref ? ref.split(':', 2) : [];
      setDeviceParent(dev, ref ? State.devices[idx].id + ':' + link : null, true);
    });
    State.scene.updateMatrixWorld(true);
    const meshRecs = (payload.stls || []).filter(r => !r.isPointCloud && !r.isSplat);
    meshRecs.forEach((rec, i) => {
      const entry = State.importedSTLs[i];
      const parentLink = remapParent(rec.parentLink, indexMap);
      // A restore only ever adds a parent; a sync must also take one away.
      if (!parentLink && entry.parentLink) setSTLParent(entry, null, true);
      applySavedObjectState(entry, { ...rec, parentLink });
    });
    for (const s of skipped) {
      const rec = (payload.stls || []).find(r => r.name === s.name);
      if (rec) s.visible = rec.visible !== false;
    }
    checkCollisionsNow();
    return { rebuilt: false, devices: State.devices.length, objects: State.importedSTLs.length, skipped };
  }

  // Is obj part of dev: its links, its static structure, and anything
  // parented under it (objects, other devices)?
  function belongsTo(obj, dev) {
    for (let o = obj; o; o = o.parent) if (o === dev.rootGroup) return true;
    return false;
  }

  // Check a joint-space path exactly, as the viewer would see it: joint
  // limits first, then collisions at every sample, densified so that no
  // joint moves more than resolutionDeg between samples. Only collisions
  // that involve the device (or anything attached to it) count against the
  // path; the rest are reported as background. Stops at the first failing
  // sample. The device is put back where it was afterwards.
  function checkPath({ device, deviceIndex, waypoints, resolutionDeg = 1.0 }) {
    // deviceIndex is the device's position in the viewer's list, which is
    // unambiguous where names are not (two arms of one model share a name).
    let dev;
    if (deviceIndex !== undefined) {
      const skippedHere = !deviceIndexMap.has(deviceIndex);
      dev = skippedHere ? null : State.devices[deviceIndexMap.get(deviceIndex)];
      if (!dev) return { ok: false, reason: `device #${deviceIndex} is not available headless (hexapods are not supported yet)` };
    } else {
      dev = device
        ? State.devices.find(d => d.name === device || d.id === device)
        : State.activeDevice;
    }
    if (!dev) return { ok: false, reason: `device '${device}' not found` };
    if (!Array.isArray(waypoints) || waypoints.length === 0) {
      return { ok: false, reason: 'no waypoints given' };
    }
    const n = dev.sliderJointMap.length;
    const bad = waypoints.findIndex(w => !Array.isArray(w) || w.length !== n);
    if (bad >= 0) return { ok: false, reason: `waypoint ${bad} has ${waypoints[bad]?.length} values; ${dev.name} has ${n} joints` };

    // Limits in API degrees (the config's, mirrored for apiSign −1).
    const limits = dev.sliderJointMap.map(ji => {
      const [lo, hi] = dev.config.joints[ji].limits;
      return dev.apiSign[ji] > 0 ? [lo, hi] : [-hi, -lo];
    });
    const names = dev.sliderJointMap.map(ji => dev.config.joints[ji].name);

    const samples = [];
    for (let i = 0; i < waypoints.length - 1; i++) {
      const a = waypoints[i], b = waypoints[i + 1];
      const span = Math.max(...a.map((v, k) => Math.abs(b[k] - v)));
      const steps = Math.max(1, Math.ceil(span / Math.max(0.01, resolutionDeg)));
      for (let s = 0; s < steps; s++) samples.push([i, a.map((v, k) => v + (b[k] - v) * s / steps)]);
    }
    samples.push([waypoints.length - 1, waypoints[waypoints.length - 1]]);

    const saved = [...dev.jointAngles];
    const round = (v) => Math.round(v * 1e4) / 1e4;
    let background = null;
    try {
      for (let k = 0; k < samples.length; k++) {
        const [seg, angles] = samples[k];
        for (let j = 0; j < n; j++) {
          const [lo, hi] = limits[j];
          if (angles[j] < lo - 1e-9 || angles[j] > hi + 1e-9) {
            return {
              ok: false, reason: `${names[j]} at ${round(angles[j])}° is outside its limits [${lo}, ${hi}]`,
              sample: k, segment: [seg, Math.min(seg + 1, waypoints.length - 1)],
              angles: angles.map(round), samples: samples.length, unchecked: uncheckedNames(),
            };
          }
        }
        dev.sliderJointMap.forEach((ji, j) => { dev.jointAngles[ji] = dev.apiSign[ji] * angles[j] * Math.PI / 180; });
        clampJoints(dev);
        updateFK(dev);
        const hits = checkCollisionsNow();
        const mine = hits.filter(c => [c.meshA, c.meshB].some(m => m && belongsTo(m, dev)));
        if (background === null) {
          background = hits.filter(c => !mine.includes(c)).map(c => ({ link: c.linkName, object: c.stlName }));
        }
        if (mine.length) {
          return {
            ok: false, reason: mine.map(c => `${c.linkName} ↔ ${c.stlName}`).join(', '),
            pairs: mine.map(c => ({ link: c.linkName, object: c.stlName })),
            sample: k, segment: [seg, Math.min(seg + 1, waypoints.length - 1)],
            angles: angles.map(round), samples: samples.length,
            background, unchecked: uncheckedNames(),
          };
        }
      }
      return { ok: true, samples: samples.length, background: background || [], unchecked: uncheckedNames() };
    } finally {
      saved.forEach((v, i) => { dev.jointAngles[i] = v; });
      updateFK(dev);
      checkCollisionsNow();
    }
  }

  // Visible scene content the check could not include.
  const uncheckedNames = () => skipped.filter(s => s.visible).map(s => `${s.name} (${s.kind})`);

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
    } else if (cmd === 'syncScene') {
      replies.push({ type: 'sceneSynced', ...(await syncScene(msg.scene)) });
    } else if (cmd === 'checkPath') {
      replies.push({ type: 'pathCheck', ...checkPath(msg) });
    } else {
      handleCommand(msg);
    }

    if (!READ_ONLY.has(cmd)) checkCollisionsNow();
    return replies;
  }

  function collisions() {
    return checkCollisionsNow().map(c => ({ link: c.linkName, object: c.stlName }));
  }

  return { handle, addDevice, addPrimitive, loadScene, syncScene, checkPath, collisions, State, scene };
}
