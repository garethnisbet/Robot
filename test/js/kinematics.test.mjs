import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import * as State from '../../js/state.js';
import {
  updateFK, getEEWorldPosition, getEEWorldQuaternion, solveIK, invertNxN,
  effectiveLimitsDeg, clampJoints, pyEulerFromRelQuat, relQuatFromPyEuler,
  kappaToEuler, eulerToKappa,
} from '../../js/kinematics.js';
import { freshScene, makeDevice, rng, deg2rad } from './fixtures.mjs';

const close = (a, b, tol, msg) =>
  assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} expected ${b}, got ${a} (tol ${tol})`);

test('pyEuler ↔ relative quaternion round-trips away from gimbal lock', () => {
  const r = rng(7);
  for (let k = 0; k < 200; k++) {
    const a = (r() * 2 - 1) * 179, b = (r() * 2 - 1) * 85, g = (r() * 2 - 1) * 179;
    const [a2, b2, g2] = pyEulerFromRelQuat(relQuatFromPyEuler(a, b, g));
    close(a2, a, 1e-6, 'alpha'); close(b2, b, 1e-6, 'beta'); close(g2, g, 1e-6, 'gamma');
  }
});

test('pyEuler at gimbal lock is deterministic: gamma folds into alpha', () => {
  const [a, b, g] = pyEulerFromRelQuat(relQuatFromPyEuler(30, 90, 0));
  close(b, 90, 1e-6); close(g, 0, 1e-9);
  // Same rotation as the input, whatever alpha it lands on.
  const q1 = relQuatFromPyEuler(30, 90, 0), q2 = relQuatFromPyEuler(a, b, g);
  close(Math.abs(q1.dot(q2)), 1, 1e-9);
});

test('invertNxN inverts, and reports singular matrices', () => {
  const m = [[4, 7, 2], [3, 6, 1], [2, 5, 3]];
  const inv = invertNxN(m, 3);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    let s = 0; for (let k = 0; k < 3; k++) s += m[i][k] * inv[k][j];
    close(s, i === j ? 1 : 0, 1e-12);
  }
  assert.equal(invertNxN([[1, 2], [2, 4]], 2), null);
});

test('limits: disabling them frees moving joints but never fixed ones', (t) => {
  t.after(() => State.setLimitsEnabled(true));
  State.setLimitsEnabled(true);
  assert.deepEqual(effectiveLimitsDeg([-90, 90]), [-90, 90]);
  State.setLimitsEnabled(false);
  assert.deepEqual(effectiveLimitsDeg([-90, 90]), [-360, 360]);
  assert.deepEqual(effectiveLimitsDeg([5, 5]), [5, 5]);
});

test('clampJoints holds joints inside their config limits', () => {
  freshScene();
  const dev = makeDevice('meca500');
  dev.jointAngles.fill(10);                      // radians — far out of range
  clampJoints(dev);
  dev.jointAngles.forEach((a, i) => assert.ok(a <= dev.jointLimits[i][1] + 1e-12));
});

for (const name of ['meca500', 'gp225', 'gp280']) {
  test(`${name}: IK recovers an EE pose reached by FK`, () => {
    const scene = freshScene();
    const dev = makeDevice(name, scene);
    const r = rng(3);
    for (let trial = 0; trial < 5; trial++) {
      // A reachable target: FK from a random in-limit pose.
      const goal = dev.jointAngles.map((_, i) => {
        if (dev.jointFixed[i]) return 0;
        const [lo, hi] = dev.jointLimits[i];
        const span = Math.min(hi - lo, 120 * deg2rad) * 0.5;
        return (lo + hi) / 2 + (r() * 2 - 1) * span * 0.5;
      });
      dev.jointAngles.splice(0, dev.numJoints, ...goal);
      updateFK(dev); scene.updateMatrixWorld(true);
      const pos = getEEWorldPosition(dev), quat = getEEWorldQuaternion(dev);

      // Start nearby so the local solver has a fair chance.
      dev.jointAngles.forEach((a, i) => { if (!dev.jointFixed[i]) dev.jointAngles[i] = a + 0.1; });
      const err = solveIK(dev, pos, quat, 400, 0.00005);
      assert.ok(err < 1e-4, `position error ${err} m`);
      const q = getEEWorldQuaternion(dev);
      close(Math.abs(q.dot(quat)), 1, 1e-4, 'orientation');
    }
  });
}

for (const name of ['i16', 'i19']) {
  test(`${name}: kappa ↔ euler chi round-trips`, () => {
    freshScene();
    const dev = makeDevice(name);
    assert.ok(dev.isKappaGeometry);
    for (const chi of [-60, -20, 0, 15, 45, 80]) {
      const sol = eulerToKappa(dev, chi);
      assert.ok(sol, `no kappa for chi=${chi}`);
      close(kappaToEuler(dev, sol.kappa).chi, chi, 1e-6, `chi=${chi}`);
    }
  });
}

// A target dragged at constant speed, solved once a frame as the viewer does
// (10 iterations). With damping fixed in metres the small arms trailed by
// 9–20 mm and crept on after the drag stopped; it is now scaled to reach.
// The tool orientation must hold tight throughout, including a rotation-only
// drag, which a loose 0.57 deg convergence test used to leave stepping behind.
const deg = Math.PI / 180;
for (const name of ['meca500', 'motomini', 'gp180', 'gp225']) {
  for (const [kind, moveFrac, rotDeg] of [['move', 0.25, 0], ['move+rotate', 0.15, 20], ['rotate', 0, 15]]) {
    test(`${name}: IK keeps up with a dragged target (${kind})`, () => {
      const scene = freshScene();
      const dev = makeDevice(name);
      const start = [0.3, 0.2, 0.4, 0.2, -0.6, 0.1];
      let k = 0;
      for (let i = 0; i < dev.numJoints; i++) if (!dev.jointFixed?.[i]) dev.jointAngles[i] = start[k++ % 6];
      updateFK(dev); scene.updateMatrixWorld(true);
      const p0 = getEEWorldPosition(dev), q0 = getEEWorldQuaternion(dev);
      const dir = new THREE.Vector3(0.6, -0.3, 0.74).normalize().multiplyScalar(moveFrac * p0.length());
      const axis = new THREE.Vector3(0.3, 1, 0.2).normalize();
      const target = new THREE.Vector3(), tq = new THREE.Quaternion();
      let maxLag = 0, maxOri = 0;
      for (let f = 1; f <= 90; f++) {
        target.copy(p0).addScaledVector(dir, f / 90);
        tq.setFromAxisAngle(axis, rotDeg * deg * f / 90).multiply(q0);
        solveIK(dev, target, tq, 10, 0.00005);
        maxLag = Math.max(maxLag, getEEWorldPosition(dev).distanceTo(target) / p0.length());
        maxOri = Math.max(maxOri, getEEWorldQuaternion(dev).angleTo(tq));
      }
      for (let f = 0; f < 5; f++) solveIK(dev, target, tq, 10, 0.00005);
      assert.ok(maxLag < 0.005, `lag ${(maxLag * 100).toFixed(2)}% of reach`);
      assert.ok(maxOri < 0.01 * deg, `orientation slipped ${(maxOri / deg).toFixed(4)} deg during the drag`);
      assert.ok(getEEWorldPosition(dev).distanceTo(target) < 1e-4, 'settles within 5 frames');
      assert.ok(getEEWorldQuaternion(dev).angleTo(tq) < 0.002 * deg, 'orientation exact once settled');
    });
  }
}
