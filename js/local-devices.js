// ============================================================
// js/local-devices.js — devices imported from the user's own
//                       machine rather than served with the viewer
// ============================================================
// A local device is a *_config.json plus the model it names (a .glb, or a
// .gltf with its .bin and textures), picked or dropped in the browser. It is
// given a config key 'local:<config file name>' and from then on loads like
// any served config: loadDevice, undo, scene restore and the websocket
// addDevice command all take the key.
//
// The files are kept in IndexedDB as well as memory, so a device still loads
// after a page reload (auto-save restore) without picking the files again.
import * as THREE from 'three';
import { dbSave, dbLoad, dbListKeys, dbDelete } from './storage.js';

export const LOCAL_PREFIX = 'local:';
const DB_PREFIX = 'local_device:';

// key → { config, files: Map<lowercase basename, Blob>, urls: Map<name, blob URL> }
const registry = new Map();

export const isLocalConfig = (configFile) =>
  typeof configFile === 'string' && configFile.startsWith(LOCAL_PREFIX);

const baseName = (p) => decodeURIComponent(String(p).split(/[\\/]/).pop().split(/[?#]/)[0]).toLowerCase();

// A device config, as opposed to a saved scene or anything else.
export function looksLikeDeviceConfig(data) {
  return !!data && typeof data === 'object' && typeof data.model === 'string' &&
    (data.type === 'hexapod' || (Array.isArray(data.joints) && Array.isArray(data.links)));
}

async function _entry(configFile) {
  let entry = registry.get(configFile);
  if (entry) return entry;
  let rec = null;
  try { rec = await dbLoad(DB_PREFIX + configFile); } catch (e) { console.warn('[local-devices] read failed:', e); }
  if (!rec) {
    throw new Error(`Local device ${configFile.slice(LOCAL_PREFIX.length)} is not stored in this browser; import its files again`);
  }
  entry = { config: rec.config, files: new Map(Object.entries(rec.files).map(([n, b]) => [n, new Blob([b])])), urls: new Map() };
  registry.set(configFile, entry);
  return entry;
}

// The config for any config key: from the registry for a local device,
// fetched from the server otherwise.
export async function readDeviceConfig(configFile) {
  if (!isLocalConfig(configFile)) {
    const r = await fetch(configFile + '?_=' + Date.now());
    if (!r.ok) throw new Error(`${configFile}: HTTP ${r.status}`);
    return r.json();
  }
  return structuredClone((await _entry(configFile)).config);
}

// A LoadingManager that serves a local device's model, buffers and textures
// from its stored files, matched by file name. Undefined (the default
// manager) for a served device.
export function loadingManagerFor(configFile) {
  if (!isLocalConfig(configFile)) return undefined;
  const entry = registry.get(configFile);
  if (!entry) return undefined;
  const manager = new THREE.LoadingManager();
  manager.setURLModifier((url) => {
    if (url.startsWith('data:') || url.startsWith('blob:')) return url;
    const name = baseName(url);
    const blob = entry.files.get(name);
    if (!blob) { console.warn(`[local-devices] ${configFile}: no file named ${name}`); return url; }
    if (!entry.urls.has(name)) entry.urls.set(name, URL.createObjectURL(blob));
    return entry.urls.get(name);
  });
  return manager;
}

// Register every device config among `files`, with the model files it needs.
// Returns { keys: [config keys added], used: Set<File> of files consumed }.
// A config whose model is not among the files is an error.
export async function importLocalDeviceFiles(files) {
  const byName = new Map(files.map(f => [f.name.toLowerCase(), f]));
  const keys = [];
  const used = new Set();
  const errors = [];
  for (const f of files) {
    if (!f.name.toLowerCase().endsWith('.json')) continue;
    let data;
    try { data = JSON.parse(await f.text()); } catch { continue; }
    if (!looksLikeDeviceConfig(data)) continue;
    used.add(f);

    const model = byName.get(baseName(data.model));
    if (!model) {
      errors.push(`${f.name} needs ${baseName(data.model)}; select it together with the config`);
      continue;
    }
    // A .gltf may reference a .bin and textures; take every other non-config
    // file offered so those resolve. A .glb is self-contained.
    const deviceFiles = new Map([[baseName(data.model), model]]);
    used.add(model);
    if (model.name.toLowerCase().endsWith('.gltf')) {
      for (const other of files) {
        if (other === model || other.name.toLowerCase().endsWith('.json')) continue;
        deviceFiles.set(other.name.toLowerCase(), other);
        used.add(other);
      }
    }

    const key = LOCAL_PREFIX + f.name;
    const old = registry.get(key);
    if (old) for (const u of old.urls.values()) URL.revokeObjectURL(u);
    registry.set(key, { config: data, files: deviceFiles, urls: new Map() });

    const stored = {};
    for (const [n, b] of deviceFiles) stored[n] = await b.arrayBuffer();
    try { await dbSave({ config: data, files: stored }, DB_PREFIX + key); }
    catch (e) { console.warn(`[local-devices] ${key} not stored (it will not survive a reload):`, e); }
    keys.push(key);
  }
  if (!keys.length && errors.length) throw new Error(errors.join('; '));
  if (errors.length) console.warn('[local-devices]', errors.join('; '));
  return { keys, used, errors };
}

// Config keys of every local device known in this browser.
export async function listLocalConfigs() {
  const keys = new Set(registry.keys());
  try {
    for (const k of await dbListKeys(DB_PREFIX)) keys.add(k.slice(DB_PREFIX.length));
  } catch (e) { console.warn('[local-devices] list failed:', e); }
  return [...keys].sort();
}

// Forget a local device: its files leave memory and this browser's storage.
// Devices already built from it keep their meshes; the caller decides
// whether deleting one that is still in the scene is allowed.
export async function deleteLocalDevice(configFile) {
  if (!isLocalConfig(configFile)) throw new Error(`${configFile} is not an imported device`);
  const entry = registry.get(configFile);
  if (entry) for (const u of entry.urls.values()) URL.revokeObjectURL(u);
  registry.delete(configFile);
  try { await dbDelete(DB_PREFIX + configFile); }
  catch (e) { console.warn(`[local-devices] ${configFile} not removed from storage (it will be back after a reload):`, e); }
}

export const localConfigLabel = (key) =>
  key.slice(LOCAL_PREFIX.length).replace(/_config\.json$/i, '').replace(/\.json$/i, '').replace(/_/g, ' ') + ' (local)';
