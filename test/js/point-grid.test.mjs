import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import {
  buildPointGrid, pointCloudIntersectsMesh, POINT_CLOUD_COLLISION_THRESHOLD as THR,
} from '../../js/point-grid.js';

// A 0.1 m cube at the origin, as collision.js hands it over.
const geom = new THREE.BoxGeometry(0.1, 0.1, 0.1);
geom.computeBoundingBox();
const bvh = new MeshBVH(geom);
const identity = new THREE.Matrix4().elements;

function hits(points, pcMatrix = identity, meshMatrix = identity) {
  const grid = buildPointGrid(new Float32Array(points.flat()), THR);
  return pointCloudIntersectsMesh(grid, pcMatrix, meshMatrix, geom.boundingBox, bvh, new THREE.Vector3());
}

test('a lone point just inside the threshold of a face collides', () => {
  assert.equal(hits([[0.05 + THR * 0.9, 0, 0]]), true);
});

test('a lone point just outside the threshold does not', () => {
  // Guards the single-leaf BVH case where closestPointToPoint ignores
  // maxThreshold, so the distance has to be measured.
  assert.equal(hits([[0.05 + THR * 1.1, 0, 0]]), false);
});

test('one near point among a sparse far cloud still collides', () => {
  const cloud = [];
  for (let i = 0; i < 5000; i++) cloud.push([2 + (i % 50) * 0.01, 2, Math.floor(i / 50) * 0.01]);
  cloud.push([0, 0.06, 0]);
  assert.equal(hits(cloud), true);
});

test('transforms are honoured: the same cloud moved away stops colliding', () => {
  const moved = new THREE.Matrix4().makeTranslation(1, 0, 0).elements;
  assert.equal(hits([[0, 0.06, 0]], moved), false);
  assert.equal(hits([[0, 0.06, 0]], moved, moved), true);
});

test('scale is honoured: the threshold is a world distance', () => {
  // Mesh authored in mm and scaled to metres, like an imported part.
  const mmGeom = new THREE.BoxGeometry(100, 100, 100);
  mmGeom.computeBoundingBox();
  const mmBVH = new MeshBVH(mmGeom);
  const mm = new THREE.Matrix4().makeScale(0.001, 0.001, 0.001).elements;
  const grid = buildPointGrid(new Float32Array([0.05 + THR * 0.9, 0, 0]), THR);
  assert.equal(pointCloudIntersectsMesh(grid, identity, mm, mmGeom.boundingBox, mmBVH, new THREE.Vector3()), true);
});
