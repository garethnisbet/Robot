// ============================================================
// js/stl.js — STL/OBJ/PLY/GLB import, IndexedDB persistence,
//             primitive creation, STL list UI,
//             selection, transform mode, parent assignment
// ============================================================
import * as THREE from 'three';
import { STLLoader }  from 'three/addons/loaders/STLLoader.js';
import { OBJLoader }  from 'three/addons/loaders/OBJLoader.js';
import { PLYLoader }  from 'three/addons/loaders/PLYLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { MeshBVH } from 'three-mesh-bvh';

import * as State from './state.js';

// ============================================================
// Loaders & colour palette
// ============================================================
const stlLoader         = new STLLoader();
const objLoader         = new OBJLoader();
const plyLoader         = new PLYLoader();
const gltfImportLoader  = new GLTFLoader();

export const stlColors = [0x44aaff, 0xff6644, 0x44ff88, 0xffaa22, 0xcc44ff, 0xff4488, 0x22ddcc, 0xaadd22];

// Helper: consume the next color slot and return the color
function nextColor() {
  const idx = State.stlColorIdx;
  State.setStlColorIdx(idx + 1);
  return stlColors[idx % stlColors.length];
}

// ============================================================
// IndexedDB helpers
// ============================================================
const STL_DB_NAME    = 'RobotViewerSTLs';
const STL_DB_VERSION = 1;
const STL_STORE      = 'stls';

function openSTLDB() {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open(STL_DB_NAME, STL_DB_VERSION);
    req.onupgradeneeded = () => {
      const db = req.result;
      if (!db.objectStoreNames.contains(STL_STORE)) {
        db.createObjectStore(STL_STORE, { keyPath: 'id' });
      }
    };
    req.onsuccess = () => resolve(req.result);
    req.onerror   = () => reject(req.error);
  });
}

async function saveSTLToDB(entry) {
  const db = await openSTLDB();
  const tx = db.transaction(STL_STORE, 'readwrite');
  const store = tx.objectStore(STL_STORE);
  const m = entry.mesh;
  store.put({
    id: entry.stlId,
    name: entry.name,
    color: entry.color,
    buffer: entry._buffer,
    fileType: entry.fileType || 'stl',
    position: [m.position.x, m.position.y, m.position.z],
    rotation:  [m.rotation.x, m.rotation.y, m.rotation.z],
    scale:    [m.scale.x, m.scale.y, m.scale.z],
    visible:  m.visible,
    parentLink: entry.parentLink || null,
  });
  return new Promise((resolve, reject) => {
    tx.oncomplete = resolve;
    tx.onerror    = () => reject(tx.error);
  });
}

async function deleteSTLFromDB(stlId) {
  const db = await openSTLDB();
  const tx = db.transaction(STL_STORE, 'readwrite');
  tx.objectStore(STL_STORE).delete(stlId);
  return new Promise((resolve, reject) => {
    tx.oncomplete = resolve;
    tx.onerror    = () => reject(tx.error);
  });
}

async function loadAllSTLsFromDB() {
  const db = await openSTLDB();
  const tx = db.transaction(STL_STORE, 'readonly');
  const store = tx.objectStore(STL_STORE);
  return new Promise((resolve, reject) => {
    const req = store.getAll();
    req.onsuccess = () => resolve(req.result);
    req.onerror   = () => reject(req.error);
  });
}

export function saveSTLDebounced(entry) {
  saveSTLToDB(entry).catch(e => console.warn('STL save failed:', e));
}

// ============================================================
// Restore persisted meshes
// ============================================================
export async function restorePersistedSTLs() {
  try {
    const records = await loadAllSTLsFromDB();
    for (const rec of records) {
      const transforms = {
        position: rec.position,
        rotation:  rec.rotation,
        scale:    rec.scale,
        visible:  rec.visible,
        parentLink: rec.parentLink || null,
      };
      const fileType = rec.fileType || 'stl';
      if (fileType === 'stl') {
        createSTLFromBuffer(rec.buffer, rec.name, rec.color, rec.id, transforms);
      } else if (fileType === 'ply') {
        const geometry = plyLoader.parse(rec.buffer);
        if (_isPLYPointCloud(rec.buffer)) {
          _addPointsToScene(geometry, rec.buffer, rec.name, rec.color, rec.id, transforms);
        } else {
          geometry.computeVertexNormals();
          _addMeshToScene(geometry, rec.buffer, 'ply', rec.name, rec.color, rec.id, transforms);
        }
      } else if (fileType === 'obj') {
        const text = new TextDecoder().decode(rec.buffer);
        const group = objLoader.parse(text);
        const geometry = _mergeObject3D(group);
        _addMeshToScene(geometry, rec.buffer, 'obj', rec.name, rec.color, rec.id, transforms);
      } else if (fileType === 'glb') {
        try {
          const gltf = await new Promise((resolve, reject) =>
            gltfImportLoader.parse(rec.buffer, '', resolve, reject));
          const geometry = _mergeObject3D(gltf.scene);
          _addMeshToScene(geometry, rec.buffer, 'glb', rec.name, rec.color, rec.id, transforms);
        } catch (e) {
          console.warn('Failed to restore GLB mesh:', rec.name, e);
        }
      }
    }
  } catch (e) {
    console.warn('Failed to restore meshes:', e);
  }
}

// ============================================================
// Internal geometry helpers
// ============================================================
export function _mergeObject3D(object3D) {
  const chunks = [];
  let totalVerts = 0;
  object3D.updateWorldMatrix(true, true);
  object3D.traverse(child => {
    if (!child.isMesh || !child.geometry) return;
    let geo = child.geometry.clone();
    child.updateWorldMatrix(true, false);
    geo.applyMatrix4(child.matrixWorld);
    if (geo.index) geo = geo.toNonIndexed();
    const pos = geo.getAttribute('position');
    if (!pos || pos.count === 0) return;
    const arr = new Float32Array(pos.count * 3);
    for (let i = 0; i < pos.count; i++) {
      arr[i * 3]     = pos.getX(i);
      arr[i * 3 + 1] = pos.getY(i);
      arr[i * 3 + 2] = pos.getZ(i);
    }
    chunks.push(arr);
    totalVerts += pos.count;
  });
  const merged = new THREE.BufferGeometry();
  if (totalVerts > 0) {
    const positions = new Float32Array(totalVerts * 3);
    let offset = 0;
    for (const chunk of chunks) {
      positions.set(chunk, offset);
      offset += chunk.length;
    }
    merged.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  }
  merged.computeVertexNormals();
  return merged;
}

export function _isPLYPointCloud(buffer) {
  const header = new TextDecoder().decode(new Uint8Array(buffer, 0, Math.min(2048, buffer.byteLength)));
  return !header.includes('element face');
}

export function _addPointsToScene(geometry, buffer, name, color, stlId, transforms) {
  const hasVertexColors = geometry.hasAttribute('color');
  const matColor = hasVertexColors ? 0xffffff : color;
  const material = new THREE.PointsMaterial({
    color: matColor, size: 0.003, sizeAttenuation: true,
    vertexColors: hasVertexColors,
    transparent: true, opacity: 0.9,
  });
  const points = new THREE.Points(geometry, material);

  if (transforms) {
    points.position.set(...transforms.position);
    points.rotation.set(...transforms.rotation);
    points.scale.set(...transforms.scale);
    points.visible = transforms.visible;
  } else {
    const box = new THREE.Box3().setFromObject(points);
    const size = box.getSize(new THREE.Vector3());
    if (size.length() > 1) points.scale.setScalar(0.001);
  }

  State.scene.add(points);
  const box = new THREE.Box3().setFromObject(points);
  const div = document.createElement('div');
  div.className = 'mesh-label';
  div.textContent = name;
  const label = new CSS2DObject(div);
  label.visible = State.labelsOn;
  const center = box.getCenter(new THREE.Vector3());
  points.worldToLocal(center);
  label.position.copy(center);
  points.add(label);

  const entry = { mesh: points, label, name, color: matColor, stlId, _buffer: buffer, fileType: 'ply', isPointCloud: true, parentLink: null };
  State.importedSTLs.push(entry);
  State.setStlColorIdx(Math.max(State.stlColorIdx, stlColors.indexOf(color) + 1));
  addSTLListItem(entry);
  return entry;
}

export function _addMeshToScene(geometry, buffer, fileType, name, color, stlId, transforms) {
  geometry.boundsTree = new MeshBVH(geometry);

  const material = new THREE.MeshStandardMaterial({
    color, metalness: 0.3, roughness: 0.6,
    transparent: true, opacity: 0.85,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;

  if (transforms) {
    mesh.position.set(...transforms.position);
    mesh.rotation.set(...transforms.rotation);
    mesh.scale.set(...transforms.scale);
    mesh.visible = transforms.visible;
  } else {
    const box = new THREE.Box3().setFromObject(mesh);
    const size = box.getSize(new THREE.Vector3());
    if (size.length() > 1) mesh.scale.setScalar(0.001);
  }

  State.scene.add(mesh);
  const box = new THREE.Box3().setFromObject(mesh);
  const div = document.createElement('div');
  div.className = 'mesh-label';
  div.textContent = name;
  const label = new CSS2DObject(div);
  label.visible = State.labelsOn;
  const center = box.getCenter(new THREE.Vector3());
  mesh.worldToLocal(center);
  label.position.copy(center);
  mesh.add(label);

  const entry = { mesh, label, name, color, stlId, _buffer: buffer, fileType, parentLink: null };
  State.importedSTLs.push(entry);
  State.setStlColorIdx(Math.max(State.stlColorIdx, stlColors.indexOf(color) + 1));
  addSTLListItem(entry);

  if (transforms && transforms.parentLink) {
    setSTLParent(entry, transforms.parentLink, true);
  }
  return entry;
}

export function createSTLFromBuffer(buffer, name, color, stlId, transforms) {
  const geometry = stlLoader.parse(buffer);
  geometry.computeVertexNormals();
  return _addMeshToScene(geometry, buffer, 'stl', name, color, stlId, transforms);
}

// ============================================================
// File loaders (wired to input events in main.js)
// ============================================================
export function loadSTLFile(file) {
  const reader = new FileReader();
  reader.onload = (e) => {
    const buffer = e.target.result;
    const baseName = file.name.replace(/\.stl$/i, '');
    const color = nextColor();
    const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
    const entry = createSTLFromBuffer(buffer, baseName, color, stlId, null);
    saveSTLDebounced(entry);
  };
  reader.readAsArrayBuffer(file);
}

export function loadOBJFile(file) {
  const reader = new FileReader();
  reader.onload = (e) => {
    const text = e.target.result;
    const group = objLoader.parse(text);
    const geometry = _mergeObject3D(group);
    const buffer = new TextEncoder().encode(text).buffer;
    const baseName = file.name.replace(/\.obj$/i, '');
    const color = nextColor();
    const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
    const entry = _addMeshToScene(geometry, buffer, 'obj', baseName, color, stlId, null);
    saveSTLDebounced(entry);
  };
  reader.readAsText(file);
}

export function loadPLYFile(file) {
  const reader = new FileReader();
  reader.onload = (e) => {
    const buffer = e.target.result;
    const geometry = plyLoader.parse(buffer);
    const baseName = file.name.replace(/\.ply$/i, '');
    const color = nextColor();
    const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
    let entry;
    if (_isPLYPointCloud(buffer)) {
      entry = _addPointsToScene(geometry, buffer, baseName, color, stlId, null);
    } else {
      geometry.computeVertexNormals();
      entry = _addMeshToScene(geometry, buffer, 'ply', baseName, color, stlId, null);
    }
    saveSTLDebounced(entry);
  };
  reader.readAsArrayBuffer(file);
}

export async function loadGLBFile(file) {
  const buffer = await file.arrayBuffer();
  const baseName = file.name.replace(/\.(glb|gltf)$/i, '');
  const color = nextColor();
  const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
  try {
    const gltf = await new Promise((resolve, reject) =>
      gltfImportLoader.parse(buffer, '', resolve, reject));
    const geometry = _mergeObject3D(gltf.scene);
    const entry = _addMeshToScene(geometry, buffer, 'glb', baseName, color, stlId, null);
    saveSTLDebounced(entry);
  } catch (err) {
    console.error('GLB load error:', err);
    alert(`Failed to load GLB/GLTF file: ${err.message}`);
  }
}

// ============================================================
// Primitive creation
// ============================================================
function geometryToSTLBuffer(geometry) {
  const pos = geometry.getAttribute('position');
  const idx = geometry.getIndex();
  const triCount = idx ? idx.count / 3 : pos.count / 3;
  const bufLen = 84 + triCount * 50;
  const buf = new ArrayBuffer(bufLen);
  const view = new DataView(buf);
  view.setUint32(80, triCount, true);
  let offset = 84;
  const vA = new THREE.Vector3(), vB = new THREE.Vector3(), vC = new THREE.Vector3();
  const cb = new THREE.Vector3(), ab = new THREE.Vector3();
  for (let i = 0; i < triCount; i++) {
    const a = idx ? idx.getX(i * 3)     : i * 3;
    const b = idx ? idx.getX(i * 3 + 1) : i * 3 + 1;
    const c = idx ? idx.getX(i * 3 + 2) : i * 3 + 2;
    vA.fromBufferAttribute(pos, a);
    vB.fromBufferAttribute(pos, b);
    vC.fromBufferAttribute(pos, c);
    cb.subVectors(vC, vB); ab.subVectors(vA, vB); cb.cross(ab).normalize();
    view.setFloat32(offset, cb.x, true); offset += 4;
    view.setFloat32(offset, cb.y, true); offset += 4;
    view.setFloat32(offset, cb.z, true); offset += 4;
    for (const v of [vA, vB, vC]) {
      view.setFloat32(offset, v.x, true); offset += 4;
      view.setFloat32(offset, v.y, true); offset += 4;
      view.setFloat32(offset, v.z, true); offset += 4;
    }
    view.setUint16(offset, 0, true); offset += 2;
  }
  return buf;
}

export function addPrimitive(type) {
  const size = 0.05;
  let geometry;
  let name;
  if (type === 'cube') {
    geometry = new THREE.BoxGeometry(size, size, size);
    name = 'Cube';
  } else if (type === 'sphere') {
    geometry = new THREE.SphereGeometry(size / 2, 24, 16);
    name = 'Sphere';
  } else {
    geometry = new THREE.CylinderGeometry(size / 2, size / 2, size, 24);
    name = 'Cylinder';
  }
  const nonIndexed = geometry.index ? geometry.toNonIndexed() : geometry;
  const buffer = geometryToSTLBuffer(nonIndexed);
  geometry.dispose();
  nonIndexed.dispose();

  const color = nextColor();
  const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
  const entry = createSTLFromBuffer(buffer, name, color, stlId, null);
  saveSTLDebounced(entry);
}

// ============================================================
// Duplicate an imported object
// ============================================================
export async function duplicateSTL(srcEntry) {
  const m = srcEntry.mesh;
  const stlId = Date.now() + '_' + Math.random().toString(36).slice(2, 8);
  const name = srcEntry.name + ' copy';
  const color = srcEntry.color;
  const transforms = {
    position: [m.position.x + 0.02, m.position.y, m.position.z],
    rotation: [m.rotation.x, m.rotation.y, m.rotation.z],
    scale: [m.scale.x, m.scale.y, m.scale.z],
    visible: m.visible,
    parentLink: srcEntry.parentLink || null,
  };

  let newEntry;
  const ft = srcEntry.fileType || 'stl';
  if (ft === 'stl') {
    newEntry = createSTLFromBuffer(srcEntry._buffer, name, color, stlId, transforms);
  } else if (ft === 'ply' && srcEntry.isPointCloud) {
    const geometry = plyLoader.parse(srcEntry._buffer);
    newEntry = _addPointsToScene(geometry, srcEntry._buffer, name, color, stlId, transforms);
  } else if (ft === 'ply') {
    const geometry = plyLoader.parse(srcEntry._buffer);
    geometry.computeVertexNormals();
    newEntry = _addMeshToScene(geometry, srcEntry._buffer, 'ply', name, color, stlId, transforms);
  } else if (ft === 'obj') {
    const text = new TextDecoder().decode(srcEntry._buffer);
    const group = objLoader.parse(text);
    const geometry = _mergeObject3D(group);
    newEntry = _addMeshToScene(geometry, srcEntry._buffer, 'obj', name, color, stlId, transforms);
  } else if (ft === 'glb') {
    try {
      const gltf = await new Promise((resolve, reject) =>
        gltfImportLoader.parse(srcEntry._buffer, '', resolve, reject));
      const geometry = _mergeObject3D(gltf.scene);
      newEntry = _addMeshToScene(geometry, srcEntry._buffer, 'glb', name, color, stlId, transforms);
    } catch (e) {
      console.warn('Failed to duplicate GLB mesh:', name, e);
      return;
    }
  }
  if (newEntry) saveSTLDebounced(newEntry);
}

// ============================================================
// STL list UI
// ============================================================
export function addSTLListItem(entry) {
  const list = document.getElementById('stl-list');
  const item = document.createElement('div');
  item.className = 'stl-item';

  const colorSwatch = document.createElement('input');
  colorSwatch.type = 'color';
  colorSwatch.className = 'stl-color';
  colorSwatch.value = '#' + new THREE.Color(entry.color).getHexString();
  colorSwatch.title = 'Change color';
  colorSwatch.addEventListener('input', () => {
    entry.mesh.material.color.set(colorSwatch.value);
    entry.color = entry.mesh.material.color.getHex();
    saveSTLDebounced(entry);
  });

  const nameSpan = document.createElement('span');
  nameSpan.className = 'stl-name';
  nameSpan.textContent = entry.name;
  nameSpan.title = 'Click to select, double-click to rename';
  nameSpan.addEventListener('click', (e) => {
    e.stopPropagation();
    selectSTL(entry, item);
  });
  nameSpan.addEventListener('dblclick', () => {
    const input = document.createElement('input');
    input.type = 'text';
    input.className = 'val-input';
    input.style.width = '100px';
    input.value = entry.name;
    nameSpan.style.display = 'none';
    nameSpan.parentNode.insertBefore(input, nameSpan.nextSibling);
    input.focus();
    input.select();
    const finish = (apply) => {
      if (apply && input.value.trim()) {
        entry.name = input.value.trim();
        nameSpan.textContent = entry.name;
        entry.label.element.textContent = entry.name;
        saveSTLDebounced(entry);
      }
      input.remove();
      nameSpan.style.display = '';
    };
    input.addEventListener('blur', () => finish(true));
    input.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); finish(true); }
      if (e.key === 'Escape') finish(false);
    });
  });

  const visBtn = document.createElement('button');
  visBtn.className = 'stl-vis';
  visBtn.textContent = '\uD83D\uDC41';
  visBtn.title = 'Toggle visibility';
  visBtn.style.opacity = entry.mesh.visible ? 1 : 0.3;
  visBtn.addEventListener('click', () => {
    entry.mesh.visible = !entry.mesh.visible;
    visBtn.style.opacity = entry.mesh.visible ? 1 : 0.3;
    saveSTLDebounced(entry);
  });

  const rmBtn = document.createElement('button');
  rmBtn.className = 'stl-rm';
  rmBtn.textContent = '\u2715';
  rmBtn.title = 'Remove';
  rmBtn.addEventListener('click', () => {
    if (State.selectedSTL === entry) deselectSTL();
    entry.mesh.removeFromParent();
    entry.mesh.geometry.dispose();
    entry.mesh.material.dispose();
    const si = State.importedSTLs.indexOf(entry);
    if (si >= 0) State.importedSTLs.splice(si, 1);
    item.remove();
    deleteSTLFromDB(entry.stlId).catch(e => console.warn('STL delete failed:', e));
  });

  const dupBtn = document.createElement('button');
  dupBtn.className = 'stl-vis';
  dupBtn.textContent = '\u2398';
  dupBtn.title = 'Duplicate';
  dupBtn.addEventListener('click', (e) => {
    e.stopPropagation();
    duplicateSTL(entry);
  });

  item.appendChild(colorSwatch);
  item.appendChild(nameSpan);
  item.appendChild(dupBtn);
  item.appendChild(visBtn);
  item.appendChild(rmBtn);
  list.appendChild(item);
}

// ============================================================
// STL Selection & Transform
// ============================================================
const stlModePanel = document.getElementById('stl-mode');
const stlSelName   = document.getElementById('stl-sel-name');

// STL parent-link assignment (multi-device aware)
const _reparentMat = new THREE.Matrix4();

export function resolveParentLink(parentValue) {
  // parentValue is either 'devId:linkName' (new format) or just 'linkName' (legacy)
  if (!parentValue) return { dev: null, linkName: null };
  if (parentValue.includes(':')) {
    const [devId, linkName] = parentValue.split(':', 2);
    const dev = State.devices.find(d => d.id === devId);
    return { dev: dev || null, linkName };
  }
  // Legacy: search all devices for this link name
  for (const dev of State.devices) {
    if (dev.linkToJoint[parentValue] !== undefined) {
      return { dev, linkName: parentValue };
    }
  }
  return { dev: null, linkName: parentValue };
}

export function setSTLParent(entry, parentValue, preserveLocal = false) {
  const mesh = entry.mesh;
  const { dev, linkName } = resolveParentLink(parentValue);

  if (!preserveLocal) {
    mesh.updateWorldMatrix(true, false);
    _reparentMat.copy(mesh.matrixWorld);
  }

  mesh.removeFromParent();

  if (dev && linkName && dev.linkToJoint[linkName] !== undefined) {
    const jointIdx  = dev.linkToJoint[linkName];
    const linkGroup = dev.jointRotGroups[jointIdx];
    if (!preserveLocal) {
      linkGroup.updateWorldMatrix(true, false);
      const localMat = linkGroup.matrixWorld.clone().invert().multiply(_reparentMat);
      localMat.decompose(mesh.position, mesh.quaternion, mesh.scale);
      mesh.rotation.setFromQuaternion(mesh.quaternion);
    }
    linkGroup.add(mesh);
    entry.parentLink = dev.id + ':' + linkName;
  } else {
    if (!preserveLocal) {
      _reparentMat.decompose(mesh.position, mesh.quaternion, mesh.scale);
      mesh.rotation.setFromQuaternion(mesh.quaternion);
    }
    State.scene.add(mesh);
    entry.parentLink = null;
  }

  saveSTLDebounced(entry);
}

export function selectSTL(entry, listItem) {
  deselectSTL();
  State.setSelectedSTL(entry);
  State.setSelectedListItem(listItem || null);

  State.stlTransformControls.attach(entry.mesh);
  stlModePanel.style.display = 'block';
  stlSelName.textContent = entry.name;
  document.getElementById('stlParentSelect').value = entry.parentLink || '';

  if (listItem) listItem.classList.add('selected');
}

export function deselectSTL() {
  if (State.selectedSTL) {
    State.stlTransformControls.detach();
    if (State.selectedListItem) State.selectedListItem.classList.remove('selected');
    State.setSelectedSTL(null);
    State.setSelectedListItem(null);
    stlModePanel.style.display = 'none';
    stlSelName.textContent = '';
  }
}

export function setSTLTransformMode(mode) {
  State.stlTransformControls.setMode(mode);
  document.getElementById('stlModeT').classList.toggle('active', mode === 'translate');
  document.getElementById('stlModeR').classList.toggle('active', mode === 'rotate');
  document.getElementById('stlModeS').classList.toggle('active', mode === 'scale');
}
