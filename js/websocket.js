// ============================================================
// js/websocket.js — WebSocket API
// ============================================================
import * as THREE from 'three';
import * as State from './state.js';
import {
  updateFK, getEEWorldPosition, getEEWorldQuaternion, clampJoints,
  kappaToEuler,
} from './kinematics.js';
import { loadDevice } from './device.js';
import { updateSliders, setIKMode, syncIKSliders } from './device.js';
import { rebuildDeviceList, rebuildParentDropdown } from './panel.js';
import { setSTLParent, saveSTLDebounced } from './stl.js';
import { clearCollisionHighlights } from './collision.js';

const deg2rad = Math.PI / 180;
const rad2deg = 180 / Math.PI;

// ============================================================
// Status indicator
// ============================================================
export function wsSetStatus(state) {
  const wsDot  = document.getElementById('ws-dot');
  const wsText = document.getElementById('ws-text');
  wsDot.className = 'dot ' + (state === 'on' ? 'on' : state === 'err' ? 'err' : 'off');
  wsText.textContent = state === 'on'  ? 'API: connected'     :
                        state === 'err' ? 'API: error' : 'API: not connected';
}

// ============================================================
// buildState
// ============================================================
export function buildState(dev) {
  dev = dev || State.activeDevice;
  if (!dev) return { type: 'state' };
  State.scene.updateMatrixWorld(true);
  const eePos  = getEEWorldPosition(dev);
  const eeQuat = getEEWorldQuaternion(dev);
  const euler  = new THREE.Euler().setFromQuaternion(eeQuat, 'YZX');
  return {
    type: 'state',
    device: dev.name,
    joints: dev.jointAngles.map((a, i) => +(dev.apiSign[i] * a * rad2deg).toFixed(2)),
    eePosition:    [+(eePos.x * 1000).toFixed(2), +(eePos.z * 1000).toFixed(2), +(eePos.y * 1000).toFixed(2)],
    eeOrientation: [+(euler.y * rad2deg).toFixed(2), +(euler.z * rad2deg).toFixed(2), +(euler.x * rad2deg).toFixed(2)],
    mode: dev.ikMode ? 'IK' : 'FK',
    ikError: dev.ikMode ? +((getEEWorldPosition(dev).distanceTo(dev.ikTarget.position)) * 1000).toFixed(3) : null,
    collisionEnabled: State.collisionEnabled,
    collision: State.collisionEnabled && State.lastCollisions.length > 0,
    collisions: State.collisionEnabled ? State.lastCollisions.map(c => ({ link: c.linkName, object: c.stlName })) : [],
    ...(dev.isKappaGeometry ? { chi: +((dev.kappaSignPositive ? 1 : -1) * kappaToEuler(dev, dev.jointAngles[dev.kappaJointIdx] * rad2deg).chi).toFixed(2) } : {}),
  };
}

// ============================================================
// wsSend
// ============================================================
export function wsSend(data) {
  if (State.ws && State.ws.readyState === WebSocket.OPEN) {
    State.ws.send(JSON.stringify(data));
  }
}

// ============================================================
// resolveTargetDevice
// ============================================================
export function resolveTargetDevice(data) {
  if (data.device) {
    return State.devices.find(d => d.name === data.device || d.id === data.device) || null;
  }
  return State.activeDevice;
}

// ============================================================
// buildObjectInfo
// ============================================================
const _buildInfoBB = new THREE.Box3();

// fastWorldAABB helper (local copy to avoid circular import with collision.js)
const _bbCorners = new Array(8).fill(null).map(() => new THREE.Vector3());
function _fastWorldAABB(mesh, target) {
  if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox();
  const bb = mesh.geometry.boundingBox;
  const m = mesh.matrixWorld;
  let i = 0;
  for (let x = 0; x <= 1; x++)
    for (let y = 0; y <= 1; y++)
      for (let z = 0; z <= 1; z++)
        _bbCorners[i++].set(
          x ? bb.max.x : bb.min.x,
          y ? bb.max.y : bb.min.y,
          z ? bb.max.z : bb.min.z
        ).applyMatrix4(m);
  target.makeEmpty();
  for (let j = 0; j < 8; j++) target.expandByPoint(_bbCorners[j]);
  return target;
}

export function buildObjectInfo(entry, index) {
  const m = entry.mesh;
  const p = m.position;
  const r = m.rotation;
  const s = m.scale;

  let worldBB = null;
  if (!entry.isPointCloud && m.geometry) {
    m.updateMatrixWorld(false);
    _fastWorldAABB(m, _buildInfoBB);
    worldBB = {
      min: [+_buildInfoBB.min.x.toFixed(4), +_buildInfoBB.min.z.toFixed(4), +_buildInfoBB.min.y.toFixed(4)],
      max: [+_buildInfoBB.max.x.toFixed(4), +_buildInfoBB.max.z.toFixed(4), +_buildInfoBB.max.y.toFixed(4)],
    };
  }

  return {
    index,
    name: entry.name,
    position: [+(p.x * 1000).toFixed(2), +(p.z * 1000).toFixed(2), +(p.y * 1000).toFixed(2)],
    rotation:  [+(r.x * rad2deg).toFixed(2), +(r.z * rad2deg).toFixed(2), +(r.y * rad2deg).toFixed(2)],
    scale:    [+s.x.toFixed(4), +s.y.toFixed(4), +s.z.toFixed(4)],
    visible:  m.visible,
    worldBB,
  };
}

// ============================================================
// findSTLEntry
// ============================================================
export function findSTLEntry(data) {
  if (data.index !== undefined) return State.importedSTLs[data.index] || null;
  if (data.name)                return State.importedSTLs.find(e => e.name === data.name) || null;
  if (data.object)              return State.importedSTLs.find(e => e.name === data.object) || null;
  return null;
}

// ============================================================
// applyIKTarget
// ============================================================
export function applyIKTarget(dev, data) {
  if (Array.isArray(data.position) && data.position.length === 3) {
    dev.ikTarget.position.set(data.position[0] / 1000, data.position[2] / 1000, data.position[1] / 1000);
  }
  if (Array.isArray(data.orientation) && data.orientation.length === 3) {
    dev.ikTargetEuler.set(data.orientation[2] * deg2rad, data.orientation[0] * deg2rad, data.orientation[1] * deg2rad, 'YZX');
    dev.ikTargetQuat.setFromEuler(dev.ikTargetEuler);
  }
  syncIKSliders(dev);
}

// ============================================================
// handleCommand
// ============================================================
export function handleCommand(data) {
  const cmd = data.cmd;
  if (!cmd) return;

  const collisionBtn    = document.getElementById('collisionBtn');
  const collisionInfoEl = document.getElementById('collision-info');
  const dev = resolveTargetDevice(data);

  if (cmd === 'getState') {
    wsSend(buildState(dev));

  } else if (cmd === 'listDevices') {
    wsSend({
      type: 'devices',
      devices: State.devices.map(d => ({
        id: d.id,
        name: d.name,
        config: d.configFile,
        active: d === State.activeDevice,
        position: [d.rootGroup.position.x, d.rootGroup.position.z, d.rootGroup.position.y],
      })),
    });

  } else if (cmd === 'addDevice') {
    if (data.config) {
      loadDevice(data.config).then(newDev => {
        State.devices.push(newDev);
        updateFK(newDev);
        rebuildDeviceList();
        rebuildParentDropdown();
        wsSend({ type: 'deviceAdded', device: newDev.name, id: newDev.id });
      }).catch(err => {
        wsSend({ type: 'error', error: `Failed to load device: ${err.message}` });
      });
    }

  } else if (cmd === 'setActiveDevice') {
    // Import setActiveDevice from panel to avoid circular; use dynamic import workaround
    // We call the panel function via a registered callback
    const target = State.devices.find(d => d.name === data.device || d.id === data.device);
    if (target) {
      _setActiveDeviceFn(target);
      wsSend(buildState(target));
    }

  } else if (cmd === 'setDeviceOrigin') {
    if (dev && Array.isArray(data.position) && data.position.length === 3) {
      dev.rootGroup.position.set(data.position[0] / 1000, data.position[2] / 1000, data.position[1] / 1000);
    }
    if (dev && Array.isArray(data.rotation) && data.rotation.length === 3) {
      dev.rootGroup.rotation.set(data.rotation[0] * deg2rad, data.rotation[2] * deg2rad, data.rotation[1] * deg2rad);
    }
    wsSend(buildState(dev));

  } else if (cmd === 'setJoints') {
    if (!dev) return;
    const angles = data.angles;
    if (Array.isArray(angles) && angles.length === dev.numJoints) {
      for (let i = 0; i < dev.numJoints; i++) dev.jointAngles[i] = dev.apiSign[i] * angles[i] * deg2rad;
      clampJoints(dev);
      updateFK(dev);
      updateSliders(dev);
      if (dev.ikMode) {
        State.scene.updateMatrixWorld(true);
        dev.ikTarget.position.copy(getEEWorldPosition(dev));
        dev.ikTargetQuat.copy(getEEWorldQuaternion(dev));
        dev.ikTargetEuler.setFromQuaternion(dev.ikTargetQuat, 'YZX');
        syncIKSliders(dev);
      }
      wsSend(buildState(dev));
    }

  } else if (cmd === 'home') {
    if (!dev) return;
    for (let i = 0; i < dev.numJoints; i++) dev.jointAngles[i] = 0;
    updateFK(dev);
    updateSliders(dev);
    if (dev.ikMode) {
      State.scene.updateMatrixWorld(true);
      dev.ikTarget.position.copy(getEEWorldPosition(dev));
      dev.ikTargetQuat.copy(getEEWorldQuaternion(dev));
      dev.ikTargetEuler.setFromQuaternion(dev.ikTargetQuat, 'YZX');
      syncIKSliders(dev);
    }
    wsSend(buildState(dev));

  } else if (cmd === 'setSingleJoint') {
    if (!dev) return;
    const idx   = data.index;
    const angle = data.angle;
    if (idx >= 0 && idx < dev.numJoints && typeof angle === 'number') {
      dev.jointAngles[idx] = dev.apiSign[idx] * angle * deg2rad;
      clampJoints(dev);
      updateFK(dev);
      updateSliders(dev);
      if (dev.ikMode) {
        State.scene.updateMatrixWorld(true);
        dev.ikTarget.position.copy(getEEWorldPosition(dev));
        dev.ikTargetQuat.copy(getEEWorldQuaternion(dev));
        dev.ikTargetEuler.setFromQuaternion(dev.ikTargetQuat, 'YZX');
        syncIKSliders(dev);
      }
      wsSend(buildState(dev));
    }

  } else if (cmd === 'setMode') {
    if (!dev) return;
    if (data.mode === 'FK' || data.mode === 'IK') {
      setIKMode(dev, data.mode === 'IK');
      wsSend(buildState(dev));
    }

  } else if (cmd === 'setIKTarget') {
    if (!dev) return;
    applyIKTarget(dev, data);
    wsSend(buildState(dev));

  } else if (cmd === 'moveTo') {
    if (!dev) return;
    if (!dev.ikMode) setIKMode(dev, true);
    applyIKTarget(dev, data);
    wsSend(buildState(dev));

  } else if (cmd === 'setCollision') {
    const on = data.enabled !== undefined ? !!data.enabled : !State.collisionEnabled;
    if (on !== State.collisionEnabled) {
      State.setCollisionEnabled(on);
      collisionBtn.textContent = `Collision: ${on ? 'ON' : 'OFF'}`;
      collisionBtn.classList.toggle('active', on);
      collisionInfoEl.style.display = on ? 'block' : 'none';
      if (!on) clearCollisionHighlights();
    }
    wsSend(buildState(dev));

  } else if (cmd === 'getCollisions') {
    wsSend({
      type: 'collisions',
      enabled: State.collisionEnabled,
      collision: State.collisionEnabled && State.lastCollisions.length > 0,
      pairs: State.collisionEnabled ? State.lastCollisions.map(c => ({ link: c.linkName, object: c.stlName })) : [],
    });

  } else if (cmd === 'listObjects') {
    wsSend({
      type: 'objects',
      objects: State.importedSTLs.map((e, i) => buildObjectInfo(e, i)),
    });

  } else if (cmd === 'setObject') {
    const entry = findSTLEntry(data);
    if (!entry) {
      wsSend({ type: 'error', error: 'Object not found' });
      return;
    }
    if (data.visible !== undefined) entry.mesh.visible = !!data.visible;
    if (Array.isArray(data.position) && data.position.length === 3) {
      entry.mesh.position.set(data.position[0] / 1000, data.position[2] / 1000, data.position[1] / 1000);
    }
    if (Array.isArray(data.rotation) && data.rotation.length === 3) {
      entry.mesh.rotation.set(data.rotation[2] * deg2rad, data.rotation[0] * deg2rad, data.rotation[1] * deg2rad);
    }
    if (Array.isArray(data.scale) && data.scale.length === 3) {
      entry.mesh.scale.set(data.scale[0], data.scale[1], data.scale[2]);
    }
    if (data.parent !== undefined) {
      setSTLParent(entry, data.parent, false);
    }
    saveSTLDebounced(entry);
    const idx = State.importedSTLs.indexOf(entry);
    wsSend({ type: 'object', ...buildObjectInfo(entry, idx) });

  } else if (cmd === 'getObject') {
    const entry = findSTLEntry(data);
    if (!entry) {
      wsSend({ type: 'error', error: 'Object not found' });
      return;
    }
    const idx = State.importedSTLs.indexOf(entry);
    wsSend({ type: 'object', ...buildObjectInfo(entry, idx) });
  }
}

// ============================================================
// Callback injection for setActiveDevice (avoids circular import)
// ============================================================
let _setActiveDeviceFn = () => {};
export function registerSetActiveDevice(fn) { _setActiveDeviceFn = fn; }

// ============================================================
// wsConnect
// ============================================================
export function wsConnect() {
  if (location.protocol === 'file:') return;

  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const url   = `${proto}//${location.host}/ws?role=viewer`;

  const socket = new WebSocket(url);
  State.setWs(socket);

  socket.onopen = () => {
    wsSetStatus('on');
    console.log('WebSocket connected');
  };

  socket.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      handleCommand(data);
    } catch (e) {
      console.warn('WS bad message:', e);
    }
  };

  socket.onclose = () => {
    wsSetStatus('off');
    State.setWs(null);
    State.setWsReconnectTimer(setTimeout(wsConnect, 2000));
  };

  socket.onerror = () => {
    wsSetStatus('err');
  };
}
