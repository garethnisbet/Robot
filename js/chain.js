// ============================================================
// js/chain.js — the kinematic skeleton of a serial device, built
//               from its config alone (no GLB, no scene, no DOM)
// ============================================================
import * as THREE from 'three';
import { getJointWorldAxis, getEEWorldQuaternion } from './kinematics.js';

const deg2rad = Math.PI / 180;

// Build the joint groups, axes, limits and end-effector marker for a
// serial device config. Everything hangs off the returned rootGroup, which
// the caller adds to the scene. The result carries the fields the
// kinematics functions read, so it can be spread into a device object.
export function buildChain(config) {
  const numJoints = config.joints.length;

  // Root group for the entire device (movable origin)
  const rootGroup = new THREE.Group();
  rootGroup.name = config.name + '_root';

  const jointLimits = config.joints.map(j => [j.limits[0] * deg2rad, j.limits[1] * deg2rad]);
  const jointFixed = config.joints.map(j => !!j.fixed);
  const apiSign = config.joints.map(j => (j.apiSign !== undefined) ? j.apiSign : 1);

  const linkToJoint = {};
  for (const link of config.links) linkToJoint[link.name] = link.joint;

  const jointRestGroups = [];
  const jointRotGroups = [];
  const jointAxes = [];

  const isBranching = config.joints.some((j, i) => {
    const p = j.parent !== undefined ? j.parent : i - 1;
    return i > 0 && p === -1;
  });

  for (let i = 0; i < numJoints; i++) {
    const d = config.joints[i];
    const parentIdx = d.parent !== undefined ? d.parent : i - 1;
    const parentGroup = parentIdx < 0 ? rootGroup : jointRotGroups[parentIdx];

    const restGrp = new THREE.Group();
    restGrp.name = `J${i+1}_rest`;
    restGrp.position.set(d.restPos[0], d.restPos[1], d.restPos[2]);
    restGrp.quaternion.set(d.restQuat[1], d.restQuat[2], d.restQuat[3], d.restQuat[0]);
    parentGroup.add(restGrp);
    jointRestGroups.push(restGrp);

    const rotGrp = new THREE.Group();
    rotGrp.name = `J${i+1}_rot`;
    restGrp.add(rotGrp);
    jointRotGroups.push(rotGrp);

    if (d.name && d.name.startsWith('virtual_axis')) {
      rotGrp.add(new THREE.AxesHelper(0.1));
    }

    jointAxes.push(new THREE.Vector3(d.axis[0], d.axis[1], d.axis[2]).normalize());
  }

  // End-effector marker
  const eeMarker = new THREE.Group();
  const eeParentGroup = isBranching ? jointRotGroups[jointRotGroups.length - 1] : jointRotGroups[numJoints - 1];
  eeParentGroup.add(eeMarker);
  if (config.eeOffset) eeMarker.position.set(...config.eeOffset);

  // Slider mapping (skip fixed joints)
  const sliderJointMap = [];
  const kappaSliderNames = {};
  {
    const ki = config.joints.findIndex(j => j.name === 'kappa');
    const ti = config.joints.findIndex(j => j.name === 'theta');
    const pi = config.joints.findIndex(j => j.name === 'phi');
    if (ki >= 0 && ti >= 0 && pi >= 0) {
      kappaSliderNames[ti] = 'ktheta';
      kappaSliderNames[pi] = 'kphi';
    }
  }
  for (let i = 0; i < numJoints; i++) {
    if (config.joints[i].fixed) continue;
    sliderJointMap.push(i);
  }

  // Kappa geometry detection
  const kappaJointIdx = config.joints.findIndex(j => j.name === 'kappa');
  const thetaJointIdx = config.joints.findIndex(j => j.name === 'theta');
  const phiJointIdx   = config.joints.findIndex(j => j.name === 'phi');
  const isKappaGeometry = kappaJointIdx >= 0 && thetaJointIdx >= 0 && phiJointIdx >= 0;

  const chain = {
    numJoints,
    rootGroup,
    jointLimits,
    jointFixed,
    jointAngles: Array(numJoints).fill(0),
    jointRestGroups,
    jointRotGroups,
    jointAxes,
    apiSign,
    linkToJoint,
    sliderJointMap,
    kappaSliderNames,
    isBranching,
    eeMarker,
    isKappaGeometry,
    kappaAlpha: 0,
    kappaJointIdx,
    thetaJointIdx,
    phiJointIdx,
    kappaPhiSign: 1,
    kappaThetaSign: 1,
  };

  // Home EE orientation (all joints at 0, root at identity). Reported
  // orientations are relative to this.
  chain.homeQuaternion = getEEWorldQuaternion(chain);
  chain.homeQuaternionInv = chain.homeQuaternion.clone().invert();

  // Kappa geometry parameters, measured on the chain at rest. Only the
  // relative directions of the axes matter, so the root pose is irrelevant.
  if (isKappaGeometry) {
    rootGroup.updateMatrixWorld(true);
    const thetaWorldAxis = getJointWorldAxis(chain, thetaJointIdx);
    const kappaWorldAxis = getJointWorldAxis(chain, kappaJointIdx);
    const phiWorldAxis   = getJointWorldAxis(chain, phiJointIdx);
    chain.kappaAlpha      = Math.acos(Math.min(1, Math.abs(thetaWorldAxis.dot(kappaWorldAxis))));
    chain.kappaPhiSign    = thetaWorldAxis.dot(phiWorldAxis) >= 0 ? 1 : -1;
    chain.kappaThetaSign  = thetaWorldAxis.dot(kappaWorldAxis) >= 0 ? 1 : -1;
  }

  return chain;
}
