// ============================================================
// js/collision-worker.js — offloaded collision detection
// Runs BVH intersection tests in a background thread.
// Imports Three.js + three-mesh-bvh from esm.sh (self-contained); keep the
// pinned versions in step with package.json.
// ============================================================

import { buildPointGrid, pointCloudIntersectsMesh,
         POINT_CLOUD_COLLISION_THRESHOLD } from './point-grid.js';

let THREE;
let initialized = false;

// Geometry & mesh registries
const geometryStore = new Map(); // geomId -> { geometry, refCount, grid? }
const meshStore     = new Map(); // meshId -> { geomId, isPointCloud }

// Reusable math objects (allocated after init)
let _box1, _box2, _matrix, _matA, _matB, _point;
let _corners;

const POINT_CLOUD_THRESHOLD = POINT_CLOUD_COLLISION_THRESHOLD;

// ============================================================
// Initialisation — dynamic import from esm.sh
// ============================================================
async function init() {
  try {
    THREE = await import('https://esm.sh/three@0.168.0');
    const bvh = await import('https://esm.sh/three-mesh-bvh@0.8.0?deps=three@0.168.0');

    THREE.BufferGeometry.prototype.computeBoundsTree  = bvh.computeBoundsTree;
    THREE.BufferGeometry.prototype.disposeBoundsTree   = bvh.disposeBoundsTree;

    _box1    = new THREE.Box3();
    _box2    = new THREE.Box3();
    _matrix  = new THREE.Matrix4();
    _matA    = new THREE.Matrix4();
    _matB    = new THREE.Matrix4();
    _point   = new THREE.Vector3();
    _corners = Array.from({ length: 8 }, () => new THREE.Vector3());

    initialized = true;
    self.postMessage({ type: 'ready' });
  } catch (err) {
    self.postMessage({ type: 'error', message: err.message });
  }
}

// ============================================================
// Geometry management
// ============================================================
function addMesh(meshId, geomId, positions, index, isPointCloud) {
  if (!geometryStore.has(geomId)) {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    if (index) geom.setIndex(new THREE.BufferAttribute(index, 1));
    geom.computeBoundingBox();

    const entry = { geometry: geom, refCount: 0 };

    if (!isPointCloud && index) {
      geom.computeBoundsTree();
    }

    // Every point is indexed — no subsampling, or sparse structure in
    // the cloud would never be tested against (see js/point-grid.js).
    if (isPointCloud) {
      entry.grid = buildPointGrid(positions, POINT_CLOUD_THRESHOLD);
    }

    geometryStore.set(geomId, entry);
  }

  geometryStore.get(geomId).refCount++;
  meshStore.set(meshId, { geomId, isPointCloud });
}

function removeMesh(meshId) {
  const info = meshStore.get(meshId);
  if (!info) return;
  meshStore.delete(meshId);

  const gEntry = geometryStore.get(info.geomId);
  if (gEntry) {
    gEntry.refCount--;
    if (gEntry.refCount <= 0) {
      if (gEntry.geometry.boundsTree) gEntry.geometry.disposeBoundsTree();
      gEntry.geometry.dispose();
      geometryStore.delete(info.geomId);
    }
  }
}

// ============================================================
// AABB helpers
// ============================================================
function fastWorldAABB(bb, mat, target) {
  const m = mat;
  let i = 0;
  for (let x = 0; x <= 1; x++)
    for (let y = 0; y <= 1; y++)
      for (let z = 0; z <= 1; z++) {
        const px = x ? bb.max.x : bb.min.x;
        const py = y ? bb.max.y : bb.min.y;
        const pz = z ? bb.max.z : bb.min.z;
        _corners[i++].set(
          m[0]*px + m[4]*py + m[8]*pz  + m[12],
          m[1]*px + m[5]*py + m[9]*pz  + m[13],
          m[2]*px + m[6]*py + m[10]*pz + m[14],
        );
      }
  target.makeEmpty();
  for (let j = 0; j < 8; j++) target.expandByPoint(_corners[j]);
  return target;
}

// ============================================================
// Mesh-pair test (AABB + BVH narrow phase)
// ============================================================
function testMeshPair(geomA, matA, geomB, matB) {
  fastWorldAABB(geomA.boundingBox, matA, _box1);
  fastWorldAABB(geomB.boundingBox, matB, _box2);
  if (!_box1.intersectsBox(_box2)) return false;

  const bvhA = geomA.boundsTree;
  const bvhB = geomB.boundsTree;
  if (bvhA && bvhB) {
    _matA.fromArray(matA);
    _matB.fromArray(matB);
    _matrix.copy(_matA).invert().multiply(_matB);
    if (!bvhA.intersectsGeometry(geomB, _matrix)) return false;
  }
  return true;
}

// ============================================================
// Point-cloud vs mesh test
// ============================================================
function testPointCloudVsMesh(pcGeomEntry, pcMat, meshGeomEntry, meshMat) {
  fastWorldAABB(pcGeomEntry.geometry.boundingBox, pcMat, _box1);
  fastWorldAABB(meshGeomEntry.geometry.boundingBox, meshMat, _box2);
  if (!_box1.intersectsBox(_box2)) return false;

  return pointCloudIntersectsMesh(
    pcGeomEntry.grid, pcMat, meshMat,
    meshGeomEntry.geometry.boundingBox,
    meshGeomEntry.geometry.boundsTree,
    _point, POINT_CLOUD_THRESHOLD,
  );
}

// ============================================================
// Check pairs — main entry point per frame
// ============================================================
function checkPairs(matrices, meshPairs, pointCloudPairs) {
  const collisions = [];

  for (const [meshIdA, meshIdB, nameA, nameB] of meshPairs) {
    const infoA = meshStore.get(meshIdA);
    const infoB = meshStore.get(meshIdB);
    if (!infoA || !infoB) continue;
    const gA = geometryStore.get(infoA.geomId);
    const gB = geometryStore.get(infoB.geomId);
    if (!gA || !gB) continue;
    const matA = matrices[meshIdA];
    const matB = matrices[meshIdB];
    if (!matA || !matB) continue;

    if (testMeshPair(gA.geometry, matA, gB.geometry, matB)) {
      collisions.push([meshIdA, meshIdB, nameA, nameB]);
    }
  }

  if (pointCloudPairs) {
    for (const [pcId, meshId, nameA, nameB] of pointCloudPairs) {
      const pcInfo   = meshStore.get(pcId);
      const meshInfo = meshStore.get(meshId);
      if (!pcInfo || !meshInfo) continue;
      const pcGeom   = geometryStore.get(pcInfo.geomId);
      const meshGeom = geometryStore.get(meshInfo.geomId);
      if (!pcGeom || !meshGeom) continue;
      const pcMat   = matrices[pcId];
      const meshMat = matrices[meshId];
      if (!pcMat || !meshMat) continue;

      if (testPointCloudVsMesh(pcGeom, pcMat, meshGeom, meshMat)) {
        collisions.push([pcId, meshId, nameA, nameB]);
      }
    }
  }

  self.postMessage({ type: 'results', collisions });
}

// ============================================================
// Message handler
// ============================================================
self.onmessage = function (e) {
  const msg = e.data;
  switch (msg.type) {
    case 'addMesh':
      if (!initialized) return;
      addMesh(msg.meshId, msg.geomId, msg.positions, msg.index, msg.isPointCloud);
      break;
    case 'removeMesh':
      removeMesh(msg.meshId);
      break;
    case 'checkPairs':
      if (!initialized) return;
      checkPairs(msg.matrices, msg.meshPairs, msg.pointCloudPairs || []);
      break;
  }
};

init();
