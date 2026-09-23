// ============================================================
// headless/fit-capsules.mjs — one capsule per link that encloses the
//                             link's mesh, for the planner's fast check
// ------------------------------------------------------------
//   node --import ./headless/register.mjs headless/fit-capsules.mjs [config ...]
//
// Writes <name>_capsules.json next to each <name>_config.json (all serial
// configs when none are named). Each capsule is given in the frame of the
// joint its link hangs from (the viewer's jointRotGroups[joint], which is
// the frame planner.fk computes), in metres:
//
//   { "model": "meca500_scene.glb", "links": [
//       { "name": "L4", "joint": 4, "p0": [x, y, z], "p1": [x, y, z], "radius": r,
//         "parts": [ { "p0": …, "p1": …, "radius": r }, … ] } ] }
//
// p0/p1/radius is one capsule around the whole link; `parts` is a tighter
// set whose union also encloses it (see fitParts).
//
// The meshes are the ones the viewer attaches to each link (js/model.js),
// so every vertex the viewer tests lies inside its link's capsule. That
// makes the planner's check stricter than the viewer's, never looser.
// test/js/capsules.test.mjs holds this for every config.
// ============================================================
import { readdirSync, readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createEngine } from './engine.mjs';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

// Every vertex of a link's meshes, in the frame of the joint it hangs from.
export function linkVertices(dev, link) {
  const THREE_M = dev.jointRotGroups[link.jointIdx].matrixWorld.clone().invert();
  const pts = [];
  for (const mesh of link.meshes) {
    const m = THREE_M.clone().multiply(mesh.matrixWorld).elements;
    const pos = mesh.geometry.getAttribute('position');
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
      pts.push([
        m[0] * x + m[4] * y + m[8] * z + m[12],
        m[1] * x + m[5] * y + m[9] * z + m[13],
        m[2] * x + m[6] * y + m[10] * z + m[14],
      ]);
    }
  }
  return pts;
}

// Distance from p to segment a-b.
export function segmentDistance(p, a, b) {
  const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
  const len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
  let t = len2 > 0 ? (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const d = [ap[0] - t * ab[0], ap[1] - t * ab[1], ap[2] - t * ab[2]];
  return Math.hypot(d[0], d[1], d[2]);
}

// Principal axes of a point set (eigenvectors of the covariance, by Jacobi).
function principalAxes(pts) {
  const n = pts.length;
  const c = [0, 0, 0];
  for (const p of pts) { c[0] += p[0] / n; c[1] += p[1] / n; c[2] += p[2] / n; }
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (const p of pts) {
    const d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) A[i][j] += d[i] * d[j];
  }
  const V = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let sweep = 0; sweep < 50; sweep++) {
    let off = 0;
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) off += A[p][q] ** 2;
    if (off < 1e-30) break;
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) {
      if (Math.abs(A[p][q]) < 1e-30) continue;
      const th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const cs = 1 / Math.sqrt(t * t + 1), sn = t * cs;
      for (let k = 0; k < 3; k++) {
        const akp = A[k][p], akq = A[k][q];
        A[k][p] = cs * akp - sn * akq; A[k][q] = sn * akp + cs * akq;
      }
      for (let k = 0; k < 3; k++) {
        const apk = A[p][k], aqk = A[q][k];
        A[p][k] = cs * apk - sn * aqk; A[q][k] = sn * apk + cs * aqk;
      }
      for (let k = 0; k < 3; k++) {
        const vkp = V[k][p], vkq = V[k][q];
        V[k][p] = cs * vkp - sn * vkq; V[k][q] = sn * vkp + cs * vkq;
      }
    }
  }
  return { centre: c, axes: [0, 1, 2].map(i => [V[0][i], V[1][i], V[2][i]]) };
}

// The smallest-volume enclosing capsule along one axis: the segment spans
// the projections, pulled in from both ends by s; the radius is then the
// farthest point from the segment. s is scanned.
function capsuleAlong(pts, centre, axis) {
  let tmin = Infinity, tmax = -Infinity;
  const ts = pts.map(p => {
    const t = (p[0] - centre[0]) * axis[0] + (p[1] - centre[1]) * axis[1] + (p[2] - centre[2]) * axis[2];
    if (t < tmin) tmin = t;
    if (t > tmax) tmax = t;
    return t;
  });
  const perp2 = pts.map((p, i) => {
    const d = [p[0] - centre[0] - ts[i] * axis[0], p[1] - centre[1] - ts[i] * axis[1], p[2] - centre[2] - ts[i] * axis[2]];
    return d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
  });
  let best = null;
  const half = (tmax - tmin) / 2;
  for (let k = 0; k <= 40; k++) {
    const s = half * k / 40;
    const a = tmin + s, b = tmax - s;
    let r2 = 0;
    for (let i = 0; i < ts.length; i++) {
      const over = ts[i] < a ? a - ts[i] : ts[i] > b ? ts[i] - b : 0;
      const d2 = perp2[i] + over * over;
      if (d2 > r2) r2 = d2;
    }
    const r = Math.sqrt(r2), L = b - a;
    const volume = Math.PI * r * r * L + 4 / 3 * Math.PI * r * r * r;
    if (!best || volume < best.volume) {
      best = {
        volume, radius: r,
        p0: [0, 1, 2].map(j => centre[j] + a * axis[j]),
        p1: [0, 1, 2].map(j => centre[j] + b * axis[j]),
      };
    }
  }
  return best;
}

// Rounded outward: endpoints to 0.1 mm, radius up to the next 0.1 mm after
// adding back what rounding the endpoints can cost, then re-measured.
function roundCapsule(cap, pts) {
  const q = (v) => Math.round(v * 1e4) / 1e4;
  const p0 = cap.p0.map(q), p1 = cap.p1.map(q);
  let r = 0;
  for (const p of pts) r = Math.max(r, segmentDistance(p, p0, p1));
  return { p0, p1, radius: Math.ceil((r + 1e-6) * 1e4) / 1e4 };
}

export function fitCapsule(pts, jointAxis) {
  const { centre, axes } = principalAxes(pts);
  const candidates = [...axes];
  if (jointAxis) candidates.push(jointAxis);
  let best = null;
  for (const axis of candidates) {
    const n = Math.hypot(...axis);
    const cap = capsuleAlong(pts, centre, axis.map(v => v / n));
    if (!best || cap.volume < best.volume) best = cap;
  }
  return roundCapsule(best, pts);
}

const capsuleVolume = (c) => {
  const L = Math.hypot(c.p1[0] - c.p0[0], c.p1[1] - c.p0[1], c.p1[2] - c.p0[2]);
  return Math.PI * c.radius * c.radius * L + 4 / 3 * Math.PI * c.radius ** 3;
};

// Several capsules whose union encloses the points. The largest part is
// split at the median along whichever principal axis leaves the smaller
// total, repeatedly, up to maxParts; then the fewest parts whose total
// volume is within ENOUGH of the best seen are kept. A single split need
// not pay off on its own: halving a ring (a diffractometer circle) barely
// shrinks it, quartering it does. Every point stays inside the capsule
// fitted to its own part, so the union encloses them all.
const ENOUGH = 1.10;
const MIN_POINTS = 32;

export function fitParts(pts, jointAxis, maxParts = 12) {
  let parts = [{ pts, cap: fitCapsule(pts, jointAxis) }];
  parts[0].volume = capsuleVolume(parts[0].cap);
  const history = [parts];
  while (parts.length < maxParts) {
    const open = parts.filter(p => p.pts.length >= 2 * MIN_POINTS);
    if (!open.length) break;
    const part = open.reduce((a, b) => (b.volume > a.volume ? b : a));
    const { centre, axes } = principalAxes(part.pts);
    let best = null;
    for (const axis of axes) {
      const t = part.pts.map(p => (p[0] - centre[0]) * axis[0] + (p[1] - centre[1]) * axis[1] + (p[2] - centre[2]) * axis[2]);
      const median = Float64Array.from(t).sort()[t.length >> 1];
      const a = [], b = [];
      part.pts.forEach((p, i) => (t[i] < median ? a : b).push(p));
      if (a.length < MIN_POINTS || b.length < MIN_POINTS) continue;
      const ca = fitCapsule(a, jointAxis), cb = fitCapsule(b, jointAxis);
      const volume = capsuleVolume(ca) + capsuleVolume(cb);
      if (!best || volume < best.volume) best = { volume, halves: [[a, ca], [b, cb]] };
    }
    if (!best) break;
    parts = parts.filter(p => p !== part).concat(
      best.halves.map(([pp, cap]) => ({ pts: pp, cap, volume: capsuleVolume(cap) })));
    history.push(parts);
  }
  const total = (ps) => ps.reduce((v, p) => v + p.volume, 0);
  const least = Math.min(...history.map(total));
  return history.find(ps => total(ps) <= ENOUGH * least).map(p => p.cap);
}

export async function fitConfig(configFile) {
  const engine = await createEngine();
  const dev = await engine.addDevice(configFile);
  engine.scene.updateMatrixWorld(true);
  const links = dev.robotLinkMeshes.map(link => {
    const pts = linkVertices(dev, link);
    const axis = dev.jointAxes[link.jointIdx].toArray();
    return { name: link.name, joint: link.jointIdx, vertices: pts.length, ...fitCapsule(pts, axis),
             parts: fitParts(pts, axis) };
  });
  return { model: dev.config.model, links };
}

// ── CLI ──────────────────────────────────────────────────────
if (process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])) {
  let configs = process.argv.slice(2);
  if (configs.length === 0) {
    configs = readdirSync(ROOT).filter(f => f.endsWith('_config.json')).sort()
      .filter(f => JSON.parse(readFileSync(path.join(ROOT, f), 'utf8')).type !== 'hexapod');
  }
  for (const configFile of configs) {
    const fit = await fitConfig(configFile);
    const out = {
      comment: 'Generated by headless/fit-capsules.mjs from the model; do not edit. ' +
               'Per link, in its joint frame, metres: one enclosing capsule, and `parts`, ' +
               'a tighter set of capsules whose union encloses it.',
      model: fit.model,
      links: fit.links.map(({ vertices, ...l }) => l),
    };
    const file = configFile.replace(/_config\.json$/, '_capsules.json');
    writeFileSync(path.join(ROOT, file), JSON.stringify(out, null, 2) + '\n');
    console.error(`${file}: ${fit.links.map(l =>
      `${l.name} r=${(l.radius * 1000).toFixed(0)}mm → ${l.parts.length} parts, ` +
      `volume ${(100 * l.parts.reduce((v, c) => v + capsuleVolume(c), 0) / capsuleVolume(l)).toFixed(0)}%`).join('; ')}`);
  }
}
