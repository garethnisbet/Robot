// ============================================================
// headless/compare-live.mjs — does the headless engine agree with a live
//                             viewer? Runs one scenario against both and
//                             compares their collision reports.
// ------------------------------------------------------------
//   node --experimental-websocket --import ./headless/register.mjs \
//        headless/compare-live.mjs --url 'ws://127.0.0.1:8080/ws?session=ab12cd34' [--trials 40] [--near 80]
//
// The viewer must hold exactly one serial device and no objects, so the
// scenario cannot disturb real work. It adds one cube, drives random joint
// poses with the cube placed near the end effector, then removes the cube, restores the joints and
// the collision settings. The live check is async (a worker, paced by the
// page), so each comparison waits until getCollisions reports current.
// ============================================================
import { parseArgs } from 'node:util';
import { createEngine } from './engine.mjs';

const { values: args } = parseArgs({ options: {
  url:    { type: 'string' },
  trials: { type: 'string', default: '40' },
  seed:   { type: 'string', default: '1' },
  near:   { type: 'string', default: '80' },   // cube within this many mm of the EE
} });
if (!args.url || !args.url.includes('session=')) {
  console.error("--url must name a viewer session, e.g. 'ws://127.0.0.1:8080/ws?session=ab12cd34'");
  process.exit(2);
}

// ── live viewer connection ───────────────────────────────────
const ws = new WebSocket(args.url);
await new Promise((res, rej) => { ws.onopen = res; ws.onerror = () => rej(new Error(`cannot connect to ${args.url}`)); });
const inbox = [];
let wake = null;
ws.onmessage = (ev) => { inbox.push(JSON.parse(ev.data)); if (wake) wake(); };

async function live(msg, type, timeoutMs = 10000) {
  inbox.length = 0;
  ws.send(JSON.stringify(msg));
  const deadline = Date.now() + timeoutMs;
  for (;;) {
    const i = inbox.findIndex(m => m.type === type || m.type === 'error');
    if (i >= 0) {
      const m = inbox.splice(i, 1)[0];
      if (m.type === 'error') throw new Error(`viewer: ${m.error} (${msg.cmd})`);
      return m;
    }
    if (Date.now() > deadline) throw new Error(`viewer: no '${type}' reply to ${msg.cmd}`);
    await new Promise(r => { wake = r; setTimeout(r, 50); });
  }
}

// The live result for the scene as it stands now: poll until the viewer
// says its standing result was computed from the current scene.
async function liveCollisions() {
  const deadline = Date.now() + 15000;
  for (;;) {
    const r = await live({ cmd: 'getCollisions' }, 'collisions');
    if (r.current === true) return r.pairs;
    if (Date.now() > deadline) throw new Error('viewer never produced a current collision result');
    await new Promise(r => setTimeout(r, 30));
  }
}

const key = (pairs) => pairs.map(p => [p.link, p.object].sort().join('↔')).sort();

// ── preflight: only a clean single-device viewer ─────────────
const { devices } = await live({ cmd: 'listDevices' }, 'devices');
const { objects } = await live({ cmd: 'listObjects' }, 'objects');
if (devices.length !== 1 || devices[0].deviceType !== 'serial' || objects.length !== 0) {
  console.error(`The viewer holds ${devices.length} device(s) and ${objects.length} object(s). ` +
    'Open a viewer with one serial device and no objects, so this check cannot disturb a real scene.');
  ws.close();
  process.exit(2);
}
const dev = devices[0];
const before = await live({ cmd: 'getState', device: dev.name }, 'state');
const hadCollision = before.collisionEnabled;
const hadHeadless  = (await live({ cmd: 'getCollisions' }, 'collisions')).headless;

// ── headless twin of the same scene ──────────────────────────
const engine = await createEngine();
await engine.handle({ cmd: 'addDevice', config: dev.config });
await engine.handle({ cmd: 'setDeviceOrigin', position: dev.position, rotation: dev.rotation });

const both = async (msg, liveType) => {
  await live({ ...msg, device: dev.name }, liveType);
  await engine.handle(msg);
};

let mismatches = 0, collided = 0, trials = 0;
async function compare(label) {
  const a = key(await liveCollisions());
  const [h] = await engine.handle({ cmd: 'getCollisions' });
  const b = key(h.pairs);
  trials++;
  if (a.length) collided++;
  const same = a.join('|') === b.join('|');
  if (!same) mismatches++;
  console.log(`${same ? 'same' : 'DIFF'}  ${label}\n      live:     ${a.join(', ') || '—'}` +
              (same ? '' : `\n      headless: ${b.join(', ') || '—'}`));
}

// Deterministic seeded sampling, so a disagreement can be replayed.
let s = Number(args.seed) >>> 0;
const rnd = () => ((s = (s * 1664525 + 1013904223) >>> 0) / 2 ** 32);
const round = (v) => Math.round(v * 100) / 100;

try {
  await live({ cmd: 'setCollision', enabled: true }, 'state');
  await live({ cmd: 'setCollisionHeadless', enabled: true }, 'collisionHeadless').catch(() => {});
  await live({ cmd: 'setFloorCollision', enabled: true }, 'floorCollision');
  await engine.handle({ cmd: 'setFloorCollision', enabled: true });

  await both({ cmd: 'home' }, 'state');
  await compare('home, no objects');

  await live({ cmd: 'addPrimitive', type: 'cube' }, 'objectAdded');
  await engine.handle({ cmd: 'addPrimitive', type: 'cube' });

  const limits = engine.State.devices[0].sliderJointMap.map(ji => {
    const d = engine.State.devices[0];
    const [lo, hi] = d.config.joints[ji].limits;
    return d.apiSign[ji] > 0 ? [lo, hi] : [-hi, -lo];
  });
  // The cube goes near the end effector, where a few millimetres decide
  // between a hit and a miss: that boundary is what has to agree.
  const near = Number(args.near);

  for (let t = 0; t < Number(args.trials); t++) {
    const angles = limits.map(([lo, hi]) => round(lo + rnd() * (hi - lo)));
    const state = await live({ cmd: 'setJoints', device: dev.name, angles }, 'state');
    await engine.handle({ cmd: 'setJoints', angles });
    const dir = [rnd() * 2 - 1, rnd() * 2 - 1, rnd() * 2 - 1];
    const len = Math.hypot(...dir) || 1;
    const dist = rnd() * near;
    const pos = state.eePosition.map((v, i) => round(v + dir[i] / len * dist));
    await live({ cmd: 'setObject', index: 0, position: pos }, 'object');
    await engine.handle({ cmd: 'setObject', index: 0, position: pos });
    await compare(`joints [${angles}]  cube ${round(dist)} mm from EE at [${pos}]`);
  }
} finally {
  // Leave the viewer as it was found.
  await live({ cmd: 'removeObject', index: 0 }, 'objectRemoved').catch(() => {});
  await live({ cmd: 'setJoints', device: dev.name, angles: before.joints }, 'state').catch(() => {});
  await live({ cmd: 'setCollisionHeadless', enabled: hadHeadless }, 'collisionHeadless').catch(() => {});
  await live({ cmd: 'setCollision', enabled: hadCollision }, 'state').catch(() => {});
  ws.close();
}

console.log(`\n${trials} comparisons, ${collided} with collisions in the viewer, ${mismatches} disagreements.`);
process.exit(mismatches ? 1 : 0);
