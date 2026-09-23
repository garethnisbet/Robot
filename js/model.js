// ============================================================
// js/model.js — a serial device's model attached to its chain:
//               everything collision and kinematics need, and
//               nothing that touches the page (no labels, helpers
//               or scene-wide state)
// ============================================================
// Shared by loadDevice in the viewer and by the headless engine, so the
// two cannot disagree about which mesh belongs to which link.
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { buildChain } from './chain.js';
import { kappaToEuler } from './kinematics.js';

const rad2deg = 180 / Math.PI;

// Nodes Blender exports as helpers; they are not part of the machine.
export const HIDDEN_NODE_NAMES = ['Icosphere', 'Cross'];

// Link pairs that touch by construction (same joint, or parent and child)
// and so are never reported as a self-collision.
export function buildAdjacencyPairs(config) {
  const adjPairs = new Set();
  function movableAncestor(jointIdx) {
    let idx = config.joints[jointIdx].parent;
    while (idx >= 0) {
      if (!config.joints[idx].fixed) return idx;
      idx = config.joints[idx].parent;
    }
    return -1;
  }
  const linksByJoint = {};
  for (const link of config.links) {
    if (!linksByJoint[link.joint]) linksByJoint[link.joint] = [];
    linksByJoint[link.joint].push(link.name);
  }
  for (const names of Object.values(linksByJoint)) {
    for (let a = 0; a < names.length; a++)
      for (let b = a + 1; b < names.length; b++)
        adjPairs.add([names[a], names[b]].sort().join('|'));
  }
  for (const linkA of config.links) {
    const ancA = movableAncestor(linkA.joint);
    for (const linkB of config.links) {
      if (linkA.name >= linkB.name) continue;
      const ancB = movableAncestor(linkB.joint);
      const jA = config.joints[linkA.joint].fixed ? movableAncestor(linkA.joint) : linkA.joint;
      const jB = config.joints[linkB.joint].fixed ? movableAncestor(linkB.joint) : linkB.joint;
      if (jA === jB || jA === ancB || jB === ancA) {
        adjPairs.add([linkA.name, linkB.name].sort().join('|'));
      }
    }
  }
  return adjPairs;
}

// Move each link's node from the glTF scene onto its joint group, keeping
// its world pose, and collect every other visible mesh as static structure
// on the root. Link meshes get a BVH for collision. Returns the glTF nodes
// by name, for callers that decorate them further.
export function attachModel(dev, gltfScene) {
  const { linkToJoint, jointRotGroups, rootGroup } = dev;

  const allNodes = {};
  gltfScene.traverse((child) => {
    if (child.name) allNodes[child.name] = child;
  });

  const reparented = new Set();
  for (const [linkName, jointIdx] of Object.entries(linkToJoint)) {
    const node = allNodes[linkName];
    if (!node) {
      console.warn(`${linkName} not found in glTF`);
      continue;
    }
    node.updateWorldMatrix(true, false);
    const worldMat = node.matrixWorld.clone();
    node.removeFromParent();

    const target = jointRotGroups[jointIdx];
    target.updateWorldMatrix(true, false);
    const localMat = target.matrixWorld.clone().invert().multiply(worldMat);

    node.matrix.copy(localMat);
    node.matrix.decompose(node.position, node.quaternion, node.scale);
    target.add(node);

    reparented.add(linkName);
    node.traverse((c) => { if (c.name) reparented.add(c.name); });
  }

  const hiddenNodes = new Set();
  gltfScene.traverse((child) => {
    if (child.name && HIDDEN_NODE_NAMES.includes(child.name)) {
      child.traverse((c) => hiddenNodes.add(c));
    }
  });
  const statics = [];
  gltfScene.traverse((child) => {
    if (child.isMesh && !reparented.has(child.name) && !hiddenNodes.has(child)) {
      statics.push(child);
    }
  });
  for (const mesh of statics) {
    mesh.updateWorldMatrix(true, false);
    const wm = mesh.matrixWorld.clone();
    mesh.removeFromParent();
    rootGroup.add(mesh);
    mesh.matrix.copy(wm);
    mesh.matrix.decompose(mesh.position, mesh.quaternion, mesh.scale);
    mesh.userData.deviceId = dev.id;
    dev.staticMeshes.push(mesh);
  }

  for (const [linkName, jointIdx] of Object.entries(linkToJoint)) {
    const node = allNodes[linkName];
    if (!node) continue;
    const meshes = [];
    node.traverse((c) => {
      if (c.isMesh) {
        meshes.push(c);
        c.geometry.boundsTree = new MeshBVH(c.geometry);
        c.userData.deviceId = dev.id;
      }
    });
    if (meshes.length > 0) dev.robotLinkMeshes.push({ name: linkName, meshes, jointIdx });
  }

  // Chi range the kappa geometry can reach within the kappa limits.
  if (dev.isKappaGeometry) {
    const lim = dev.jointLimits[dev.kappaJointIdx];
    const chiAtMin = -kappaToEuler(dev, lim[0] * rad2deg).chi;
    const chiAtMax = -kappaToEuler(dev, lim[1] * rad2deg).chi;
    dev._chiLimits = [Math.min(chiAtMin, chiAtMax), Math.max(chiAtMin, chiAtMax)];
  }

  dev.loaded = true;
  return allNodes;
}

// The part of a serial device that does not depend on the page: chain,
// collision data, and the state the API reads. The viewer adds its
// visuals (EE axes, chain overlay, IK target, labels) on top.
export function assembleDevice(config, { id, configFile, gltfScene }) {
  const dev = {
    ...buildChain(config),
    id,
    config,
    configFile,
    name: config.name,
    meshLabels: [],
    robotLinkMeshes: [],
    staticMeshes: [],
    opacity: 1,
    adjPairs: buildAdjacencyPairs(config),
    ikMode: false,
    ikTarget: new THREE.Object3D(),
    ikTargetQuat: new THREE.Quaternion(),
    ikTargetEuler: new THREE.Euler(0, 0, 0, 'YZX'),
    kappaSignPositive: true,
    parentLink: null,
    loaded: false,
  };
  if (gltfScene) attachModel(dev, gltfScene);
  return dev;
}
