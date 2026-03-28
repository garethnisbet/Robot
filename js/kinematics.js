// ============================================================
// js/kinematics.js — pure math: FK, IK, kappa geometry
// ============================================================
import * as THREE from 'three';
import * as State from './state.js';

const deg2rad = Math.PI / 180;
const rad2deg = 180 / Math.PI;

// ============================================================
// Per-device FK/IK functions
// ============================================================

export function updateFK(dev) {
  for (let i = 0; i < dev.numJoints; i++) {
    dev.jointRotGroups[i].quaternion.setFromAxisAngle(dev.jointAxes[i], dev.jointAngles[i]);
  }
  updateVirtualAngles(dev);
}

export function getEEWorldPosition(dev) {
  dev.eeMarker.updateWorldMatrix(true, false);
  return new THREE.Vector3().setFromMatrixPosition(dev.eeMarker.matrixWorld);
}

export function getEEWorldQuaternion(dev) {
  dev.eeMarker.updateWorldMatrix(true, false);
  return new THREE.Quaternion().setFromRotationMatrix(dev.eeMarker.matrixWorld);
}

export function getJointWorldAxis(dev, i) {
  const worldQuat = new THREE.Quaternion();
  dev.jointRestGroups[i].getWorldQuaternion(worldQuat);
  return dev.jointAxes[i].clone().applyQuaternion(worldQuat).normalize();
}

export function getJointWorldPosition(dev, i) {
  dev.jointRestGroups[i].updateWorldMatrix(true, false);
  return new THREE.Vector3().setFromMatrixPosition(dev.jointRestGroups[i].matrixWorld);
}

export function clampJoints(dev) {
  for (let i = 0; i < dev.numJoints; i++) {
    dev.jointAngles[i] = Math.max(dev.jointLimits[i][0], Math.min(dev.jointLimits[i][1], dev.jointAngles[i]));
  }
}

// ============================================================
// Kappa geometry functions
// ============================================================

export function kappaToEuler(dev, kappaDeg) {
  const kappaRad = kappaDeg * deg2rad;
  const chi = 2 * Math.asin(Math.sin(kappaRad / 2) * Math.sin(dev.kappaAlpha));
  return { chi: chi * rad2deg, kappa: kappaDeg };
}

export function eulerToKappa(dev, chiDeg) {
  const chiRad = chiDeg * deg2rad;
  const sinHalfChi = Math.sin(chiRad / 2);
  const sinAlpha = Math.sin(dev.kappaAlpha);
  if (Math.abs(sinHalfChi / sinAlpha) > 1) return null;
  const kappaRad = 2 * Math.asin(sinHalfChi / sinAlpha);
  return { kappa: kappaRad * rad2deg, chi: chiDeg };
}

export function getCompensation(dev, kappaDeg) {
  const halfK = kappaDeg * deg2rad / 2;
  const delta = Math.atan2(Math.cos(dev.kappaAlpha) * Math.sin(halfK), Math.cos(halfK)) * rad2deg;
  return { theta: -delta, phi: -delta * dev.kappaPhiSign };
}

export function updateVirtualAngles(dev) {
  if (!dev.isKappaGeometry || !dev.kappaAlpha) return;
  if (dev !== State.activeDevice) return;
  const sign = dev.kappaSignPositive ? 1 : -1;
  const kappaDeg = sign * dev.jointAngles[dev.kappaJointIdx] * rad2deg;
  const chiDeg = kappaToEuler(dev, kappaDeg).chi;
  const comp = getCompensation(dev, kappaDeg);
  let thetaDeg = sign * dev.kappaThetaSign * dev.jointAngles[dev.thetaJointIdx] * rad2deg - comp.theta + 90;
  let phiDeg = sign * dev.kappaThetaSign * dev.jointAngles[dev.phiJointIdx] * rad2deg - comp.phi + 90;

  const snap = v => Math.abs(v) < 0.05 ? 0 : v;
  const chiSlider = document.getElementById('chiSlider');
  if (chiSlider) {
    chiSlider.value = -chiDeg;
    document.getElementById('vchi').textContent = snap(-chiDeg).toFixed(1);
    document.getElementById('vthetaSlider').value = thetaDeg;
    document.getElementById('vvtheta').textContent = snap(thetaDeg).toFixed(1);
    document.getElementById('vphiSlider').value = phiDeg;
    document.getElementById('vvphi').textContent = snap(phiDeg).toFixed(1);
  }
}

export function updateChain(dev) {
  State.scene.updateMatrixWorld(true);
  for (let i = 0; i < dev.numJoints; i++) {
    const p = getJointWorldPosition(dev, i);
    dev.chainPts[i * 3]     = p.x;
    dev.chainPts[i * 3 + 1] = p.y;
    dev.chainPts[i * 3 + 2] = p.z;
    dev.chainSpheres[i].position.copy(p);
  }
  const ee = getEEWorldPosition(dev);
  const eeIdx = dev.numJoints * 3;
  dev.chainPts[eeIdx] = ee.x; dev.chainPts[eeIdx + 1] = ee.y; dev.chainPts[eeIdx + 2] = ee.z;
  dev.chainSpheres[dev.numJoints].position.copy(ee);
  dev.chainLineGeo.attributes.position.needsUpdate = true;
}

// ============================================================
// IK solver
// ============================================================

export function orientationError(qTarget, qCurrent) {
  const qErr = qTarget.clone().multiply(qCurrent.clone().conjugate());
  if (qErr.w < 0) { qErr.x = -qErr.x; qErr.y = -qErr.y; qErr.z = -qErr.z; qErr.w = -qErr.w; }
  const sinHalf = Math.sqrt(qErr.x*qErr.x + qErr.y*qErr.y + qErr.z*qErr.z);
  if (sinHalf < 1e-8) return new THREE.Vector3(0, 0, 0);
  const angle = 2 * Math.atan2(sinHalf, qErr.w);
  return new THREE.Vector3(qErr.x/sinHalf * angle, qErr.y/sinHalf * angle, qErr.z/sinHalf * angle);
}

export function solveIK(dev, targetPos, targetQuat, maxIter = 20, tolerance = 0.0005) {
  const lambda = 0.5;
  const oriWeight = 0.3;
  const numJoints = dev.numJoints;

  for (let iter = 0; iter < maxIter; iter++) {
    updateFK(dev);
    State.scene.updateMatrixWorld(true);

    const eePos = getEEWorldPosition(dev);
    const eeQuat = getEEWorldQuaternion(dev);

    const posErr = new THREE.Vector3().subVectors(targetPos, eePos);
    const oriErr = orientationError(targetQuat, eeQuat);
    const posNorm = posErr.length();

    if (posNorm < tolerance && oriErr.length() < 0.01) return posNorm;

    const J = Array.from({length: 6}, () => Array(numJoints).fill(0));
    for (let j = 0; j < numJoints; j++) {
      const axis = getJointWorldAxis(dev, j);
      const jPos = getJointWorldPosition(dev, j);
      const r = new THREE.Vector3().subVectors(eePos, jPos);
      const lin = new THREE.Vector3().crossVectors(axis, r);
      J[0][j] = lin.x;  J[1][j] = lin.y;  J[2][j] = lin.z;
      J[3][j] = axis.x * oriWeight;
      J[4][j] = axis.y * oriWeight;
      J[5][j] = axis.z * oriWeight;
    }

    const e = [
      posErr.x, posErr.y, posErr.z,
      oriErr.x * oriWeight, oriErr.y * oriWeight, oriErr.z * oriWeight
    ];

    const n = 6;
    const JJT = Array.from({length: n}, () => Array(n).fill(0));
    for (let r = 0; r < n; r++) {
      for (let c = 0; c < n; c++) {
        let sum = 0;
        for (let k = 0; k < numJoints; k++) sum += J[r][k] * J[c][k];
        JJT[r][c] = sum;
      }
    }

    const lam2 = lambda * lambda;
    for (let i = 0; i < n; i++) JJT[i][i] += lam2;

    const inv = invertNxN(JJT, n);
    if (!inv) continue;

    const v = Array(n).fill(0);
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++)
        v[i] += inv[i][j] * e[j];

    for (let i = 0; i < numJoints; i++) {
      let dq = 0;
      for (let j = 0; j < n; j++) dq += J[j][i] * v[j];
      dev.jointAngles[i] += dq;
    }

    clampJoints(dev);
  }

  updateFK(dev);
  State.scene.updateMatrixWorld(true);
  return getEEWorldPosition(dev).distanceTo(targetPos);
}

export function invertNxN(m, n) {
  const aug = Array.from({length: n}, (_, i) => {
    const row = Array(2 * n).fill(0);
    for (let j = 0; j < n; j++) row[j] = m[i][j];
    row[n + i] = 1;
    return row;
  });
  for (let col = 0; col < n; col++) {
    let maxRow = col, maxVal = Math.abs(aug[col][col]);
    for (let row = col + 1; row < n; row++) {
      const v = Math.abs(aug[row][col]);
      if (v > maxVal) { maxVal = v; maxRow = row; }
    }
    if (maxVal < 1e-12) return null;
    [aug[col], aug[maxRow]] = [aug[maxRow], aug[col]];
    const pivot = aug[col][col];
    for (let j = 0; j < 2 * n; j++) aug[col][j] /= pivot;
    for (let row = 0; row < n; row++) {
      if (row === col) continue;
      const f = aug[row][col];
      for (let j = 0; j < 2 * n; j++) aug[row][j] -= f * aug[col][j];
    }
  }
  return aug.map(row => row.slice(n));
}
