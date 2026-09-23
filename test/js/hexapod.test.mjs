import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import * as State from '../../js/state.js';
import {
  computeLegLengthsFromPose, solveHexapodFK, clampPlatformPose, effectivePoseLimits,
} from '../../js/hexapod.js';

// A symmetric Stewart platform: base pivots on a 0.2 m circle, platform
// pivots on a 0.12 m circle, platform resting 0.25 m up.
function makeHexapod() {
  const legData = [];
  for (let i = 0; i < 6; i++) {
    const b = (Math.floor(i / 2) * 120 + (i % 2 ? 15 : -15)) * Math.PI / 180;
    const p = (Math.floor(i / 2) * 120 + (i % 2 ? 45 : -45)) * Math.PI / 180;
    legData.push({
      basePivot: new THREE.Vector3(0.2 * Math.cos(b), 0, 0.2 * Math.sin(b)),
      platformPivotLocal: new THREE.Vector3(0.12 * Math.cos(p), 0, 0.12 * Math.sin(p)),
    });
  }
  return {
    legData,
    platformRestPos: new THREE.Vector3(0, 0.25, 0),
    platformPose: [0, 0, 0, 0, 0, 0],
    config: { limits: { x: [-50, 50], y: [-50, 50], z: [-30, 30], rx: [-15, 15], ry: [-15, 15], rz: [-20, 20] } },
  };
}

test('hexapod FK inverts leg lengths back to the pose', () => {
  const dev = makeHexapod();
  const pose = [12, -8, 5, 3, -4, 7];
  const lengths = computeLegLengthsFromPose(dev, pose);
  const solved = solveHexapodFK(dev, lengths);
  solved.forEach((v, i) => assert.ok(Math.abs(v - pose[i]) < 1e-5, `axis ${i}: ${v} vs ${pose[i]}`));
});

test('hexapod: at rest every leg has the same length', () => {
  const lengths = computeLegLengthsFromPose(makeHexapod(), [0, 0, 0, 0, 0, 0]);
  for (const l of lengths) assert.ok(Math.abs(l - lengths[0]) < 1e-12);
});

test('hexapod pose clamps to limits, and widens when limits are off', (t) => {
  t.after(() => State.setLimitsEnabled(true));
  const dev = makeHexapod();
  assert.deepEqual(clampPlatformPose(dev, [99, -99, 0, 0, 0, 99]), [50, -50, 0, 0, 0, 20]);
  State.setLimitsEnabled(false);
  const [lo, hi] = effectivePoseLimits(dev).x;
  assert.ok(lo < -50 && hi > 50);
});
