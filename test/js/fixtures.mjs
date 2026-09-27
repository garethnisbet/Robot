// Shared test fixtures: devices built from the real configs, without a
// GLB, a renderer or a page.
import { readFileSync } from 'node:fs';
import * as THREE from 'three';
import * as State from '../../js/state.js';
import { buildChain } from '../../js/chain.js';

const root = new URL('../../', import.meta.url);

export function loadConfig(name) {
  return JSON.parse(readFileSync(new URL(`${name}_config.json`, root), 'utf8'));
}

// A fresh scene in State, as scene.js would leave it.
export function freshScene() {
  const scene = new THREE.Scene();
  State.initCoreObjects(scene, null, null, null);
  State.devices.length = 0;
  State.setActiveDevice(null);
  return scene;
}

// A serial device with the fields the kinematics and API code read.
export function makeDevice(name, scene = State.scene) {
  const config = loadConfig(name);
  const dev = {
    ...buildChain(config),
    id: `dev_${name}`,
    config,
    name: config.name,
    ikMode: false,
    ikTarget: new THREE.Object3D(),
    ikTargetQuat: new THREE.Quaternion(),
    ikTargetEuler: new THREE.Euler(0, 0, 0, 'YZX'),
    robotLinkMeshes: [],
    staticMeshes: [],
  };
  scene.add(dev.rootGroup);
  return dev;
}

// Deterministic PRNG so failures reproduce.
export function rng(seed = 1) {
  let s = seed >>> 0;
  return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 2 ** 32);
}

export const deg2rad = Math.PI / 180;
