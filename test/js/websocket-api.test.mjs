// The remote API contract: what handleCommand replies with for a given
// command. The replies go out through State.ws, so a fake socket
// captures them.
import { test, beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import * as State from '../../js/state.js';
import { handleCommand, setApiEnabled } from '../../js/websocket.js';
import { freshScene, makeDevice } from './fixtures.mjs';

globalThis.WebSocket ??= { OPEN: 1 };
// A page with none of the viewer's elements on it.
globalThis.document ??= { getElementById: () => null };

let sent;
let dev;

beforeEach(() => {
  freshScene();
  sent = [];
  State.setWs({ readyState: 1, send: (s) => sent.push(JSON.parse(s)) });
  setApiEnabled(true);
  // Registered but not active, so the UI-sync paths stay dormant.
  dev = makeDevice('meca500');
  State.devices.push(dev);
});

function call(msg) {
  sent.length = 0;
  handleCommand(msg);
  assert.equal(sent.length, 1, `expected one reply to ${msg.cmd}, got ${sent.length}`);
  return sent[0];
}

test('getState at home matches the Meca500 manual home pose', () => {
  const s = call({ cmd: 'getState', device: dev.name });
  assert.equal(s.type, 'state');
  assert.equal(s.device, dev.name);
  assert.deepEqual(s.joints, [0, 0, 0, 0, 0, 0]);
  assert.equal(s.jointNames.length, 6);
  // Manual: home is x=190, y=0, z=308 mm with the flange at (0, 90, 0)°.
  s.eePosition.forEach((v, i) => assert.ok(Math.abs(v - [190, 0, 308][i]) < 0.01, `eePosition ${s.eePosition}`));
  s.eeOrientation.forEach((v, i) => assert.ok(Math.abs(v - [0, 90, 0][i]) < 1e-3, `eeOrientation ${s.eeOrientation}`));
  assert.equal(s.mode, 'FK');
});

test('setJoints echoes the angles back in API sign convention', () => {
  const angles = [10, -20, 30, -40, 50, -60];
  const s = call({ cmd: 'setJoints', device: dev.id, angles });
  assert.deepEqual(s.joints, angles);
});

test('setJoints clamps to joint limits', () => {
  const s = call({ cmd: 'setJoints', device: dev.id, angles: [999, 0, 0, 0, 0, 0] });
  assert.equal(s.joints[0], dev.config.joints[0].limits[1]);
});

test('setSingleJoint moves only that joint', () => {
  const s = call({ cmd: 'setSingleJoint', device: dev.id, index: 2, angle: 15 });
  assert.deepEqual(s.joints, [0, 0, 15, 0, 0, 0]);
});

test('home returns every joint to zero', () => {
  call({ cmd: 'setJoints', device: dev.id, angles: [10, 10, 10, 10, 10, 10] });
  const s = call({ cmd: 'home', device: dev.id });
  assert.deepEqual(s.joints.map(Math.abs), [0, 0, 0, 0, 0, 0]);
});

test('listDevices describes each registered device', () => {
  const r = call({ cmd: 'listDevices' });
  assert.equal(r.type, 'devices');
  assert.equal(r.devices.length, 1);
  assert.equal(r.devices[0].id, dev.id);
  assert.equal(r.devices[0].numJoints, 6);
});

test('unknown device yields an error reply', () => {
  const r = call({ cmd: 'getDevice', device: 'no-such-device' });
  assert.equal(r.type, 'error');
});

test('worldToLocal undoes the device origin offset', () => {
  dev.rootGroup.position.set(0.1, 0, 0.2);          // Three: x, y-up, z
  const r = call({ cmd: 'worldToLocal', device: dev.id, position: [150, 250, 30], _reqId: 7 });
  assert.equal(r._reqId, 7);
  assert.deepEqual(r.position, [50, 50, 30]);
});

test('with the API switched off, commands are refused and nothing moves', () => {
  setApiEnabled(false);
  const r = call({ cmd: 'setJoints', device: dev.id, angles: [10, 0, 0, 0, 0, 0] });
  assert.equal(r.type, 'error');
  assert.equal(r.apiEnabled, false);
  assert.equal(dev.jointAngles[0], 0);
});

// The Python client solves IK with GNKinematics and streams the result
// as setJoints, so the viewer must put the EE where GNKinematics says it
// is for the same API joints. Reference poses come from RobotDefinitions.py
// (kin.f_kinematics(joints): [-2] position in mm, [-1] ZYX Euler, here in
// degrees); the viewer's Y axis is the kinematic −Y. A wrong apiSign moves
// the EE by hundreds of mm (or, for a wrist joint, turns it); the
// tolerances absorb GP225's ~0.65 mm / 0.2° model-vs-config mismatch.
const GNKINEMATICS_FK = {
  meca500: [[10, -20, 30, -40, 50, -60], [120.008, -13.839, 230.176], [150.338, 37.485, 102.172]],
  gp225:   [[15, 10, -20, 30, 40, -50],  [1805.152, 566.007, 2188.076], [-128.037, 47.637, 102.8]],
  gp280:   [[-25, 5, 10, -30, 20, 45],   [1466.556, -731.039, 1642.075], [136.279, 42.912, -115.148]],
};

for (const [name, [joints, [x, y, z], ori]] of Object.entries(GNKINEMATICS_FK)) {
  test(`${name}: viewer EE agrees with the Python kinematics`, () => {
    const arm = makeDevice(name);
    State.devices.push(arm);
    const s = call({ cmd: 'setJoints', device: arm.id, angles: joints });
    const want = [x, -y, z];
    s.eePosition.forEach((v, i) =>
      assert.ok(Math.abs(v - want[i]) < 1, `eePosition ${s.eePosition} vs ${want}`));
    s.eeOrientation.forEach((v, i) =>
      assert.ok(Math.abs(v - ori[i]) < 0.5, `eeOrientation ${s.eeOrientation} vs ${ori}`));
  });
}
