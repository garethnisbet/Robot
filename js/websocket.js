// ============================================================
// js/websocket.js — WebSocket API
// ============================================================
import * as THREE from 'three';
import * as State from './state.js';
import {
  updateFK, getEEWorldPosition, getEEWorldQuaternion, clampJoints,
  kappaToEuler, eulerToKappa, getCompensation, updateVirtualAngles,
  updateChain,
} from './kinematics.js';
import { loadDevice } from './device.js';
import { updateSliders, setIKMode, syncIKSliders } from './device.js';
import {
  rebuildDeviceList, rebuildParentDropdown, removeDevice,
  setDeviceParent, rebuildDeviceParentDropdown, buildControlPanel,
  rebuildPrimaryModelDropdown,
} from './panel.js';
import {
  setSTLParent, addPrimitive, duplicateSTL, deselectSTL,
  exportSceneState,
} from './stl.js';
import { clearCollisionHighlights } from './collision.js';
import { setOrtho } from './scene.js';

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
    ...(dev.isKappaGeometry ? { chi: +-((dev.kappaSignPositive ? 1 : -1) * kappaToEuler(dev, dev.jointAngles[dev.kappaJointIdx] * rad2deg).chi).toFixed(2) } : {}),
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
    parent:   entry.parentLink || null,
    worldBB,
  };
}

// ============================================================
// buildDeviceInfo
// ============================================================
function buildDeviceInfo(dev) {
  const rg = dev.rootGroup;
  return {
    id: dev.id,
    name: dev.name,
    config: dev.configFile,
    active: dev === State.activeDevice,
    numJoints: dev.numJoints,
    joints: dev.jointAngles.map((a, i) => +(dev.apiSign[i] * a * rad2deg).toFixed(2)),
    position: [+(rg.position.x * 1000).toFixed(2), +(rg.position.z * 1000).toFixed(2), +(rg.position.y * 1000).toFixed(2)],
    rotation: [+(rg.rotation.x * rad2deg).toFixed(2), +(rg.rotation.z * rad2deg).toFixed(2), +(rg.rotation.y * rad2deg).toFixed(2)],
    parent: dev.parentLink || null,
    isKappa: dev.isKappaGeometry || false,
    mode: dev.ikMode ? 'IK' : 'FK',
    links: Object.keys(dev.linkToJoint || {}),
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
// syncIKAfterFK — helper to update IK target after FK changes
// ============================================================
function syncIKAfterFK(dev) {
  if (dev.ikMode) {
    State.scene.updateMatrixWorld(true);
    dev.ikTarget.position.copy(getEEWorldPosition(dev));
    dev.ikTargetQuat.copy(getEEWorldQuaternion(dev));
    dev.ikTargetEuler.setFromQuaternion(dev.ikTargetQuat, 'YZX');
    syncIKSliders(dev);
  }
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

  // ── Device queries ──────────────────────────────────────────

  if (cmd === 'getState') {
    wsSend(buildState(dev));

  } else if (cmd === 'listDevices') {
    wsSend({
      type: 'devices',
      devices: State.devices.map(d => buildDeviceInfo(d)),
    });

  } else if (cmd === 'getDevice') {
    if (dev) {
      wsSend({ type: 'device', ...buildDeviceInfo(dev) });
    } else {
      wsSend({ type: 'error', error: 'Device not found' });
    }

  // ── Device management ───────────────────────────────────────

  } else if (cmd === 'addDevice') {
    if (data.config) {
      loadDevice(data.config).then(newDev => {
        State.devices.push(newDev);
        updateFK(newDev);
        rebuildDeviceList();
        rebuildParentDropdown();
        rebuildDeviceParentDropdown();
        wsSend({ type: 'deviceAdded', ...buildDeviceInfo(newDev) });
      }).catch(err => {
        wsSend({ type: 'error', error: `Failed to load device: ${err.message}` });
      });
    }

  } else if (cmd === 'removeDevice') {
    if (!dev) { wsSend({ type: 'error', error: 'Device not found' }); return; }
    if (State.devices.length <= 1) { wsSend({ type: 'error', error: 'Cannot remove last device' }); return; }
    const info = buildDeviceInfo(dev);
    removeDevice(dev);
    wsSend({ type: 'deviceRemoved', device: info.name, id: info.id });

  } else if (cmd === 'renameDevice') {
    if (!dev) { wsSend({ type: 'error', error: 'Device not found' }); return; }
    if (data.name && typeof data.name === 'string') {
      dev.name = data.name.trim();
      rebuildDeviceList();
    }
    wsSend({ type: 'device', ...buildDeviceInfo(dev) });

  } else if (cmd === 'setActiveDevice') {
    const target = State.devices.find(d => d.name === data.device || d.id === data.device);
    if (target) {
      _setActiveDeviceFn(target);
      wsSend(buildState(target));
    } else {
      wsSend({ type: 'error', error: 'Device not found' });
    }

  } else if (cmd === 'setDeviceOrigin') {
    if (!dev) { wsSend({ type: 'error', error: 'Device not found' }); return; }
    if (Array.isArray(data.position) && data.position.length === 3) {
      dev.rootGroup.position.set(data.position[0] / 1000, data.position[2] / 1000, data.position[1] / 1000);
    }
    if (Array.isArray(data.rotation) && data.rotation.length === 3) {
      dev.rootGroup.rotation.set(data.rotation[0] * deg2rad, data.rotation[2] * deg2rad, data.rotation[1] * deg2rad);
    }
    wsSend(buildState(dev));

  } else if (cmd === 'setDeviceParent') {
    if (!dev) { wsSend({ type: 'error', error: 'Device not found' }); return; }
    const parentVal = data.parent !== undefined ? data.parent : null;
    setDeviceParent(dev, parentVal);
    rebuildDeviceParentDropdown();
    wsSend({ type: 'device', ...buildDeviceInfo(dev) });

  } else if (cmd === 'listConfigs') {
    // Return available config files
    wsSend({
      type: 'configs',
      configs: _availableConfigs,
    });

  // ── Joint control ───────────────────────────────────────────

  } else if (cmd === 'setJoints') {
    if (!dev) return;
    const angles = data.angles;
    if (Array.isArray(angles) && angles.length === dev.numJoints) {
      for (let i = 0; i < dev.numJoints; i++) dev.jointAngles[i] = dev.apiSign[i] * angles[i] * deg2rad;
      clampJoints(dev);
      updateFK(dev);
      updateSliders(dev);
      syncIKAfterFK(dev);
      wsSend(buildState(dev));
    }

  } else if (cmd === 'home') {
    if (!dev) return;
    for (let i = 0; i < dev.numJoints; i++) dev.jointAngles[i] = 0;
    updateFK(dev);
    updateSliders(dev);
    syncIKAfterFK(dev);
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
      syncIKAfterFK(dev);
      wsSend(buildState(dev));
    }

  } else if (cmd === 'demoPose') {
    if (!dev) return;
    if (dev.isKappaGeometry) {
      for (let i = 0; i < dev.numJoints; i++) dev.jointAngles[i] = 0;
      dev.jointAngles[dev.kappaJointIdx] = -134.6 * deg2rad;
      dev.jointAngles[dev.thetaJointIdx] = -33.5 * deg2rad;
      dev.jointAngles[dev.phiJointIdx]   = -146.9 * deg2rad;
    } else if (dev.config.demoPose) {
      const pose = dev.config.demoPose;
      for (let i = 0; i < dev.numJoints && i < pose.length; i++) {
        dev.jointAngles[i] = pose[i] * deg2rad;
      }
    }
    updateFK(dev);
    updateSliders(dev);
    syncIKAfterFK(dev);
    wsSend(buildState(dev));

  // ── Kappa virtual angles ────────────────────────────────────

  } else if (cmd === 'setVirtualAngles') {
    if (!dev || !dev.isKappaGeometry) {
      wsSend({ type: 'error', error: 'Device is not kappa geometry' }); return;
    }
    const chi   = typeof data.chi === 'number'   ? data.chi   : null;
    const theta = typeof data.theta === 'number' ? data.theta : null;
    const phi   = typeof data.phi === 'number'   ? data.phi   : null;
    // Read current virtual angles
    const sign = dev.kappaSignPositive ? 1 : -1;
    const curKappaDeg = sign * dev.jointAngles[dev.kappaJointIdx] * rad2deg;
    const curChi = -kappaToEuler(dev, curKappaDeg).chi;
    const curComp = getCompensation(dev, curKappaDeg);
    const curTheta = sign * dev.kappaThetaSign * dev.jointAngles[dev.thetaJointIdx] * rad2deg - curComp.theta + 90;
    const curPhi   = sign * dev.kappaThetaSign * dev.jointAngles[dev.phiJointIdx]   * rad2deg - curComp.phi + 90;
    // Use provided values or fall back to current
    const newChi   = chi   !== null ? chi   : curChi;
    const newTheta = theta !== null ? theta : curTheta;
    const newPhi   = phi   !== null ? phi   : curPhi;
    const result = eulerToKappa(dev, -newChi);
    if (!result) { wsSend({ type: 'error', error: 'Chi value out of range' }); return; }
    const comp = getCompensation(dev, result.kappa);
    dev.jointAngles[dev.kappaJointIdx] = sign * result.kappa * deg2rad;
    dev.jointAngles[dev.thetaJointIdx] = sign * dev.kappaThetaSign * (newTheta - 90 + comp.theta) * deg2rad;
    dev.jointAngles[dev.phiJointIdx]   = sign * dev.kappaThetaSign * (newPhi - 90 + comp.phi) * deg2rad;
    clampJoints(dev);
    updateFK(dev);
    updateSliders(dev);
    updateVirtualAngles(dev);
    syncIKAfterFK(dev);
    wsSend(buildState(dev));

  } else if (cmd === 'getVirtualAngles') {
    if (!dev || !dev.isKappaGeometry) {
      wsSend({ type: 'error', error: 'Device is not kappa geometry' }); return;
    }
    const sign = dev.kappaSignPositive ? 1 : -1;
    const kappaDeg = sign * dev.jointAngles[dev.kappaJointIdx] * rad2deg;
    const chiDeg = -kappaToEuler(dev, kappaDeg).chi;
    const comp = getCompensation(dev, kappaDeg);
    const thetaDeg = sign * dev.kappaThetaSign * dev.jointAngles[dev.thetaJointIdx] * rad2deg - comp.theta + 90;
    const phiDeg   = sign * dev.kappaThetaSign * dev.jointAngles[dev.phiJointIdx]   * rad2deg - comp.phi + 90;
    wsSend({
      type: 'virtualAngles',
      device: dev.name,
      chi: +chiDeg.toFixed(2),
      theta: +thetaDeg.toFixed(2),
      phi: +phiDeg.toFixed(2),
      kappaSign: dev.kappaSignPositive ? '+' : '-',
    });

  } else if (cmd === 'setKappaSign') {
    if (!dev || !dev.isKappaGeometry) {
      wsSend({ type: 'error', error: 'Device is not kappa geometry' }); return;
    }
    if (data.positive !== undefined) {
      dev.kappaSignPositive = !!data.positive;
    } else {
      dev.kappaSignPositive = !dev.kappaSignPositive;
    }
    const btn = document.getElementById('kappaSignBtn');
    if (btn) {
      btn.textContent = dev.kappaSignPositive ? '\u03BA Sign: +' : '\u03BA Sign: \u2212';
      btn.classList.toggle('active', dev.kappaSignPositive);
    }
    updateVirtualAngles(dev);
    wsSend(buildState(dev));

  // ── IK control ──────────────────────────────────────────────

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

  // ── Collision ───────────────────────────────────────────────

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

  // ── Object queries ──────────────────────────────────────────

  } else if (cmd === 'listObjects') {
    wsSend({
      type: 'objects',
      objects: State.importedSTLs.map((e, i) => buildObjectInfo(e, i)),
    });

  } else if (cmd === 'getObject') {
    const entry = findSTLEntry(data);
    if (!entry) { wsSend({ type: 'error', error: 'Object not found' }); return; }
    const idx = State.importedSTLs.indexOf(entry);
    wsSend({ type: 'object', ...buildObjectInfo(entry, idx) });

  // ── Object manipulation ─────────────────────────────────────

  } else if (cmd === 'setObject') {
    const entry = findSTLEntry(data);
    if (!entry) { wsSend({ type: 'error', error: 'Object not found' }); return; }
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
    if (data.color !== undefined) {
      const c = new THREE.Color(data.color);
      entry.mesh.material.color.copy(c);
      entry.color = c.getHex();
    }
    if (data.name !== undefined && typeof data.name === 'string') {
      entry.name = data.name.trim();
      entry.label.element.textContent = entry.name;
    }
    const idx = State.importedSTLs.indexOf(entry);
    wsSend({ type: 'object', ...buildObjectInfo(entry, idx) });

  } else if (cmd === 'addPrimitive') {
    const ptype = (data.type || data.primitive || 'cube').toLowerCase();
    if (!['cube', 'sphere', 'cylinder'].includes(ptype)) {
      wsSend({ type: 'error', error: 'Invalid primitive type. Use: cube, sphere, cylinder' }); return;
    }
    addPrimitive(ptype);
    const entry = State.importedSTLs[State.importedSTLs.length - 1];
    const idx = State.importedSTLs.length - 1;
    wsSend({ type: 'objectAdded', ...buildObjectInfo(entry, idx) });

  } else if (cmd === 'removeObject') {
    const entry = findSTLEntry(data);
    if (!entry) { wsSend({ type: 'error', error: 'Object not found' }); return; }
    const info = buildObjectInfo(entry, State.importedSTLs.indexOf(entry));
    // Deselect if selected
    if (State.selectedSTL === entry) deselectSTL();
    // Remove mesh from scene
    entry.mesh.removeFromParent();
    if (entry.mesh.geometry) entry.mesh.geometry.dispose();
    if (entry.mesh.material) entry.mesh.material.dispose();
    // Remove from registry
    const si = State.importedSTLs.indexOf(entry);
    if (si >= 0) State.importedSTLs.splice(si, 1);
    // Remove list item DOM
    const listItems = document.querySelectorAll('#stl-list .stl-item');
    if (listItems[si]) listItems[si].remove();
    wsSend({ type: 'objectRemoved', name: info.name, index: info.index });

  } else if (cmd === 'duplicateObject') {
    const entry = findSTLEntry(data);
    if (!entry) { wsSend({ type: 'error', error: 'Object not found' }); return; }
    duplicateSTL(entry).then(() => {
      const newEntry = State.importedSTLs[State.importedSTLs.length - 1];
      const idx = State.importedSTLs.length - 1;
      wsSend({ type: 'objectAdded', ...buildObjectInfo(newEntry, idx) });
    });

  } else if (cmd === 'resetObjectRotation') {
    const entry = findSTLEntry(data);
    if (!entry) { wsSend({ type: 'error', error: 'Object not found' }); return; }
    entry.mesh.rotation.set(0, 0, 0);
    const idx = State.importedSTLs.indexOf(entry);
    wsSend({ type: 'object', ...buildObjectInfo(entry, idx) });

  } else if (cmd === 'resetObjectScale') {
    const entry = findSTLEntry(data);
    if (!entry) { wsSend({ type: 'error', error: 'Object not found' }); return; }
    entry.mesh.scale.set(1, 1, 1);
    const idx = State.importedSTLs.indexOf(entry);
    wsSend({ type: 'object', ...buildObjectInfo(entry, idx) });

  // ── Visualization toggles ──────────────────────────────────

  } else if (cmd === 'setLabels') {
    const on = data.enabled !== undefined ? !!data.enabled : !State.labelsOn;
    State.setLabelsOn(on);
    document.getElementById('labelBtn').textContent = `Labels: ${on ? 'ON' : 'OFF'}`;
    document.getElementById('labelBtn').classList.toggle('active', on);
    for (const d of State.devices) {
      d.meshLabels.forEach(l => l.visible = on);
    }
    for (const e of State.importedSTLs) {
      if (e.label) e.label.visible = on;
    }
    wsSend({ type: 'setting', setting: 'labels', enabled: on });

  } else if (cmd === 'setOrigins') {
    const on = data.enabled !== undefined ? !!data.enabled : !State.originsOn;
    State.setOriginsOn(on);
    document.getElementById('originsBtn').textContent = `Origins: ${on ? 'ON' : 'OFF'}`;
    document.getElementById('originsBtn').classList.toggle('active', on);
    for (const d of State.devices) {
      d.originHelpers.forEach(h => h.visible = on);
      d.originLabels.forEach(l => l.visible = on);
    }
    wsSend({ type: 'setting', setting: 'origins', enabled: on });

  } else if (cmd === 'setChain') {
    if (!dev) return;
    const on = data.enabled !== undefined ? !!data.enabled : !dev.chainVisible;
    dev.chainVisible = on;
    document.getElementById('chainBtn').textContent = `Chain: ${on ? 'ON' : 'OFF'}`;
    document.getElementById('chainBtn').classList.toggle('active', on);
    dev.chainLine.visible = on;
    dev.chainSpheres.forEach(s => s.visible = on);
    if (on) updateChain(dev);
    wsSend({ type: 'setting', setting: 'chain', enabled: on, device: dev.name });

  } else if (cmd === 'setOrtho') {
    const on = data.enabled !== undefined ? !!data.enabled : !State.orthoOn;
    setOrtho(on);
    wsSend({ type: 'setting', setting: 'ortho', enabled: on });

  // ── Camera control ──────────────────────────────────────────

  } else if (cmd === 'getCamera') {
    const cam = State.activeCamera;
    const tgt = State.orbitControls.target;
    wsSend({
      type: 'camera',
      position: [+(cam.position.x * 1000).toFixed(2), +(cam.position.z * 1000).toFixed(2), +(cam.position.y * 1000).toFixed(2)],
      target:   [+(tgt.x * 1000).toFixed(2), +(tgt.z * 1000).toFixed(2), +(tgt.y * 1000).toFixed(2)],
      ortho: State.orthoOn,
      fov: State.camera.fov,
    });

  } else if (cmd === 'setCamera') {
    const cam = State.activeCamera;
    if (Array.isArray(data.position) && data.position.length === 3) {
      cam.position.set(data.position[0] / 1000, data.position[2] / 1000, data.position[1] / 1000);
    }
    if (Array.isArray(data.target) && data.target.length === 3) {
      State.orbitControls.target.set(data.target[0] / 1000, data.target[2] / 1000, data.target[1] / 1000);
    }
    State.orbitControls.update();
    const tgt = State.orbitControls.target;
    wsSend({
      type: 'camera',
      position: [+(cam.position.x * 1000).toFixed(2), +(cam.position.z * 1000).toFixed(2), +(cam.position.y * 1000).toFixed(2)],
      target:   [+(tgt.x * 1000).toFixed(2), +(tgt.z * 1000).toFixed(2), +(tgt.y * 1000).toFixed(2)],
      ortho: State.orthoOn,
    });

  } else if (cmd === 'snapCamera') {
    // Snap camera to an axis view: +X, -X, +Y, -Y, +Z, -Z, or iso
    const view = (data.view || '').toLowerCase();
    const dist = State.activeCamera.position.distanceTo(State.orbitControls.target);
    const tgt = State.orbitControls.target;
    const viewMap = {
      '+x': { pos: [dist, 0, 0], up: [0, 1, 0] },
      '-x': { pos: [-dist, 0, 0], up: [0, 1, 0] },
      '+y': { pos: [0, 0, dist], up: [0, 1, 0] },
      '-y': { pos: [0, 0, -dist], up: [0, 1, 0] },
      '+z': { pos: [0, dist, 0], up: [0, 0, -1] },
      '-z': { pos: [0, -dist, 0], up: [0, 0, 1] },
      'top':    { pos: [0, dist, 0], up: [0, 0, -1] },
      'bottom': { pos: [0, -dist, 0], up: [0, 0, 1] },
      'front':  { pos: [0, 0, dist], up: [0, 1, 0] },
      'back':   { pos: [0, 0, -dist], up: [0, 1, 0] },
      'left':   { pos: [-dist, 0, 0], up: [0, 1, 0] },
      'right':  { pos: [dist, 0, 0], up: [0, 1, 0] },
      'iso':    { pos: [dist * 0.577, dist * 0.577, dist * 0.577], up: [0, 1, 0] },
    };
    const v = viewMap[view];
    if (!v) { wsSend({ type: 'error', error: `Unknown view: ${view}. Use: +X,-X,+Y,-Y,+Z,-Z,top,bottom,front,back,left,right,iso` }); return; }
    State.activeCamera.position.set(tgt.x + v.pos[0], tgt.y + v.pos[1], tgt.z + v.pos[2]);
    State.activeCamera.up.set(v.up[0], v.up[1], v.up[2]);
    State.orbitControls.update();
    wsSend({ type: 'camera', position: [+(State.activeCamera.position.x * 1000).toFixed(2), +(State.activeCamera.position.z * 1000).toFixed(2), +(State.activeCamera.position.y * 1000).toFixed(2)], target: [+(tgt.x * 1000).toFixed(2), +(tgt.z * 1000).toFixed(2), +(tgt.y * 1000).toFixed(2)] });

  // ── Scene persistence ───────────────────────────────────────

  } else if (cmd === 'getSceneState') {
    // Return the full scene state (same data as Save Scene, but via WS)
    const stls = State.importedSTLs.map((e, i) => buildObjectInfo(e, i));
    const devices = State.devices.map(d => buildDeviceInfo(d));
    const cam = State.activeCamera;
    const tgt = State.orbitControls.target;
    wsSend({
      type: 'sceneState',
      devices,
      objects: stls,
      camera: {
        position: [+(cam.position.x * 1000).toFixed(2), +(cam.position.z * 1000).toFixed(2), +(cam.position.y * 1000).toFixed(2)],
        target:   [+(tgt.x * 1000).toFixed(2), +(tgt.z * 1000).toFixed(2), +(tgt.y * 1000).toFixed(2)],
      },
      collisionEnabled: State.collisionEnabled,
      labels: State.labelsOn,
      ortho: State.orthoOn,
      floorSize: State.floorSize,
    });

  } else if (cmd === 'saveScene') {
    // Trigger browser download of scene JSON file
    exportSceneState();
    wsSend({ type: 'sceneSaved' });

  // ── Help / command listing ──────────────────────────────────

  } else if (cmd === 'help' || cmd === 'listCommands') {
    wsSend({
      type: 'help',
      commands: {
        // Device
        getState:         { params: 'device?', description: 'Get device state (joints, EE, mode, collisions)' },
        listDevices:      { params: '', description: 'List all loaded devices' },
        getDevice:        { params: 'device?', description: 'Get detailed info for a single device' },
        addDevice:        { params: 'config', description: 'Load a new device from config file' },
        removeDevice:     { params: 'device?', description: 'Remove a device from scene' },
        renameDevice:     { params: 'device?, name', description: 'Rename a device' },
        setActiveDevice:  { params: 'device', description: 'Set the active device' },
        setDeviceOrigin:  { params: 'device?, position?, rotation?', description: 'Set device world position/rotation (mm, deg)' },
        setDeviceParent:  { params: 'device?, parent', description: 'Parent device to a link (e.g. "dev_0:L3") or null for world' },
        listConfigs:      { params: '', description: 'List available config files' },
        // Joints
        setJoints:        { params: 'device?, angles[]', description: 'Set all joint angles (degrees)' },
        setSingleJoint:   { params: 'device?, index, angle', description: 'Set one joint angle (degrees)' },
        home:             { params: 'device?', description: 'Reset all joints to 0' },
        demoPose:         { params: 'device?', description: 'Apply demo pose from config' },
        // Kappa
        setVirtualAngles: { params: 'device?, chi?, theta?, phi?', description: 'Set kappa virtual angles (degrees)' },
        getVirtualAngles: { params: 'device?', description: 'Get current kappa virtual angles' },
        setKappaSign:     { params: 'device?, positive?', description: 'Toggle or set kappa sign' },
        // IK
        setMode:          { params: 'device?, mode', description: 'Set FK or IK mode' },
        setIKTarget:      { params: 'device?, position?, orientation?', description: 'Set IK target (mm, deg)' },
        moveTo:           { params: 'device?, position?, orientation?', description: 'Switch to IK and set target' },
        // Collision
        setCollision:     { params: 'enabled?', description: 'Toggle or set collision detection' },
        getCollisions:    { params: '', description: 'Get current collision pairs' },
        // Objects
        listObjects:      { params: '', description: 'List all imported objects' },
        getObject:        { params: 'index|name|object', description: 'Get info for one object' },
        setObject:        { params: 'index|name, position?, rotation?, scale?, visible?, parent?, color?, name?', description: 'Modify an object' },
        addPrimitive:     { params: 'type', description: 'Add cube, sphere, or cylinder' },
        removeObject:     { params: 'index|name|object', description: 'Remove an object' },
        duplicateObject:  { params: 'index|name|object', description: 'Duplicate an object' },
        resetObjectRotation: { params: 'index|name|object', description: 'Reset object rotation to identity' },
        resetObjectScale:    { params: 'index|name|object', description: 'Reset object scale to 1' },
        // Visualization
        setLabels:        { params: 'enabled?', description: 'Toggle or set label visibility' },
        setOrigins:       { params: 'enabled?', description: 'Toggle or set joint origin axes for all devices' },
        setChain:         { params: 'device?, enabled?', description: 'Toggle or set chain visualization' },
        setOrtho:         { params: 'enabled?', description: 'Toggle or set orthographic camera' },
        // Camera
        getCamera:        { params: '', description: 'Get camera position and target (mm)' },
        setCamera:        { params: 'position?, target?', description: 'Set camera position and/or target (mm)' },
        snapCamera:       { params: 'view', description: 'Snap to axis view (+X,-X,+Y,-Y,+Z,-Z,top,bottom,front,back,left,right,iso)' },
        // Scene
        getSceneState:    { params: '', description: 'Get full scene state (devices, objects, camera)' },
        saveScene:        { params: '', description: 'Trigger scene file download in viewer' },
        help:             { params: '', description: 'List all available commands' },
      },
    });

  } else {
    wsSend({ type: 'error', error: `Unknown command: ${cmd}` });
  }
}

// ============================================================
// Callback injection for setActiveDevice (avoids circular import)
// ============================================================
let _setActiveDeviceFn = () => {};
export function registerSetActiveDevice(fn) { _setActiveDeviceFn = fn; }

// ============================================================
// Available configs (registered by main.js)
// ============================================================
let _availableConfigs = [];
export function registerAvailableConfigs(configs) { _availableConfigs = configs; }

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
