// ============================================================
// js/point-grid.js — uniform spatial grid over a point cloud
// ------------------------------------------------------------
// Point-cloud collision used to test an index-strided sample of the
// cloud (every Nth point, capped at 20k). On a 3M-point lidar scan that
// is 1 point in 150: sparse structure — a cable, a pipe, a lone
// obstacle, the thin edge of a scanned surface — almost never survives
// the stride, so the arm drives straight through it.
//
// Instead every point is kept and indexed into a uniform grid. A test
// visits only the cells the mesh's bounding box actually overlaps, which
// is both exact and far cheaper than blindly querying the BVH with
// thousands of samples that are nowhere near the mesh.
//
// Shared by the main thread (js/collision.js) and the collision worker,
// which pull three from different places — so this module imports
// nothing. Matrices arrive as plain 16-element column-major arrays (as
// in Matrix4.elements) and the only object touched is a scratch vector
// with .set(), supplied by the caller.
// ============================================================

export const POINT_CLOUD_COLLISION_THRESHOLD = 0.04;   // scene units (m)

// Cell budget. The grid is dense, so this bounds its memory at 4 bytes
// per cell; the cell size grows until the cloud's extent fits.
const MAX_CELLS = 4_000_000;

// ============================================================
// Build
// ============================================================
export function buildPointGrid(positions, cellSize = POINT_CLOUD_COLLISION_THRESHOLD) {
  const count = (positions.length / 3) | 0;
  if (count === 0) return null;

  let minX =  Infinity, minY =  Infinity, minZ =  Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < count; i++) {
    const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  if (!(minX <= maxX)) return null;            // all-NaN cloud

  let cell = cellSize, nx = 1, ny = 1, nz = 1;
  for (;;) {
    nx = Math.max(1, Math.ceil((maxX - minX) / cell) + 1);
    ny = Math.max(1, Math.ceil((maxY - minY) / cell) + 1);
    nz = Math.max(1, Math.ceil((maxZ - minZ) / cell) + 1);
    if (nx * ny * nz <= MAX_CELLS) break;
    cell *= 1.5;
  }
  const nCells = nx * ny * nz;
  const inv = 1 / cell;

  // Counting sort of the point indices by cell.
  const starts  = new Int32Array(nCells + 1);
  const cellOf  = new Int32Array(count);
  for (let i = 0; i < count; i++) {
    const cx = clampInt((positions[i * 3]     - minX) * inv, nx);
    const cy = clampInt((positions[i * 3 + 1] - minY) * inv, ny);
    const cz = clampInt((positions[i * 3 + 2] - minZ) * inv, nz);
    const c  = (cz * ny + cy) * nx + cx;
    cellOf[i] = c;
    starts[c + 1]++;
  }
  for (let c = 0; c < nCells; c++) starts[c + 1] += starts[c];

  const order  = new Int32Array(count);
  const cursor = starts.slice(0, nCells);
  for (let i = 0; i < count; i++) order[cursor[cellOf[i]]++] = i;

  return { positions, count, cell, inv, minX, minY, minZ, nx, ny, nz, starts, order };
}

function clampInt(v, n) {
  const i = Math.floor(v);
  return i < 0 ? 0 : (i >= n ? n - 1 : i);
}

// ============================================================
// Point cloud vs mesh
// ------------------------------------------------------------
// grid      — from buildPointGrid, in the cloud's local space
// pcMatrix  — cloud local -> world (16 numbers, column-major)
// meshMatrix— mesh local  -> world
// meshBox   — mesh geometry bounding box ({min,max} with .x/.y/.z)
// bvh       — the mesh geometry's boundsTree
// point     — scratch vector with .set(x,y,z)
// threshold — contact distance in world units
// ============================================================
const _m  = new Float64Array(16);   // pc local -> mesh local
const _b  = new Float64Array(16);   // mesh local -> pc local
const _t  = new Float64Array(16);   // scratch

export function pointCloudIntersectsMesh(grid, pcMatrix, meshMatrix, meshBox, bvh, point,
                                         threshold = POINT_CLOUD_COLLISION_THRESHOLD) {
  if (!grid || !bvh || !meshBox) return false;

  // Thresholds are world-space distances; both spaces may be scaled
  // (imported parts often carry a 0.001 mm->m scale), so convert.
  const meshScale = maxScale(meshMatrix) || 1;
  const pcScale   = maxScale(pcMatrix)   || 1;
  const meshThr   = threshold / meshScale;   // in mesh local units
  const pcPad     = threshold / pcScale;     // in cloud local units

  // Mesh box -> cloud local space, so the grid can be walked directly.
  if (!invert(pcMatrix, _t)) return false;
  multiply(_t, meshMatrix, _b);
  if (!invert(meshMatrix, _t)) return false;
  multiply(_t, pcMatrix, _m);

  const bx0 = meshBox.min.x, by0 = meshBox.min.y, bz0 = meshBox.min.z;
  const bx1 = meshBox.max.x, by1 = meshBox.max.y, bz1 = meshBox.max.z;

  let lo0 = Infinity, lo1 = Infinity, lo2 = Infinity;
  let hi0 = -Infinity, hi1 = -Infinity, hi2 = -Infinity;
  for (let k = 0; k < 8; k++) {
    const px = (k & 1) ? bx1 : bx0;
    const py = (k & 2) ? by1 : by0;
    const pz = (k & 4) ? bz1 : bz0;
    const x = _b[0]*px + _b[4]*py + _b[8]*pz  + _b[12];
    const y = _b[1]*px + _b[5]*py + _b[9]*pz  + _b[13];
    const z = _b[2]*px + _b[6]*py + _b[10]*pz + _b[14];
    if (x < lo0) lo0 = x; if (x > hi0) hi0 = x;
    if (y < lo1) lo1 = y; if (y > hi1) hi1 = y;
    if (z < lo2) lo2 = z; if (z > hi2) hi2 = z;
  }
  lo0 -= pcPad; lo1 -= pcPad; lo2 -= pcPad;
  hi0 += pcPad; hi1 += pcPad; hi2 += pcPad;

  const { positions, cell, inv, minX, minY, minZ, nx, ny, nz, starts, order } = grid;

  // Cell range, rejecting a box that misses the cloud entirely.
  const cx0 = Math.floor((lo0 - minX) * inv), cx1 = Math.floor((hi0 - minX) * inv);
  const cy0 = Math.floor((lo1 - minY) * inv), cy1 = Math.floor((hi1 - minY) * inv);
  const cz0 = Math.floor((lo2 - minZ) * inv), cz1 = Math.floor((hi2 - minZ) * inv);
  if (cx1 < 0 || cy1 < 0 || cz1 < 0 || cx0 >= nx || cy0 >= ny || cz0 >= nz) return false;
  const ix0 = cx0 < 0 ? 0 : cx0, ix1 = cx1 >= nx ? nx - 1 : cx1;
  const iy0 = cy0 < 0 ? 0 : cy0, iy1 = cy1 >= ny ? ny - 1 : cy1;
  const iz0 = cz0 < 0 ? 0 : cz0, iz1 = cz1 >= nz ? nz - 1 : cz1;

  // Mesh-local box padded by the contact distance: a cheap exact reject
  // before the BVH query, since the cell range is an over-estimate of a
  // rotated box.
  const px0 = bx0 - meshThr, py0 = by0 - meshThr, pz0 = bz0 - meshThr;
  const px1 = bx1 + meshThr, py1 = by1 + meshThr, pz1 = bz1 + meshThr;

  for (let cz = iz0; cz <= iz1; cz++) {
    for (let cy = iy0; cy <= iy1; cy++) {
      const rowBase = (cz * ny + cy) * nx;
      const from = starts[rowBase + ix0];
      const to   = starts[rowBase + ix1 + 1];
      for (let s = from; s < to; s++) {
        const i = order[s] * 3;
        const sx = positions[i], sy = positions[i + 1], sz = positions[i + 2];
        const lx = _m[0]*sx + _m[4]*sy + _m[8]*sz  + _m[12];
        if (lx < px0 || lx > px1) continue;
        const ly = _m[1]*sx + _m[5]*sy + _m[9]*sz  + _m[13];
        if (ly < py0 || ly > py1) continue;
        const lz = _m[2]*sx + _m[6]*sy + _m[10]*sz + _m[14];
        if (lz < pz0 || lz > pz1) continue;
        point.set(lx, ly, lz);
        if (bvh.closestPointToPoint(point, {}, 0, meshThr)) return true;
      }
    }
  }
  return false;
}

// ============================================================
// Plain column-major 4x4 helpers
// ============================================================
function multiply(a, b, out) {
  for (let c = 0; c < 4; c++) {
    const b0 = b[c*4], b1 = b[c*4+1], b2 = b[c*4+2], b3 = b[c*4+3];
    out[c*4]   = a[0]*b0 + a[4]*b1 + a[8]*b2  + a[12]*b3;
    out[c*4+1] = a[1]*b0 + a[5]*b1 + a[9]*b2  + a[13]*b3;
    out[c*4+2] = a[2]*b0 + a[6]*b1 + a[10]*b2 + a[14]*b3;
    out[c*4+3] = a[3]*b0 + a[7]*b1 + a[11]*b2 + a[15]*b3;
  }
  return out;
}

function maxScale(m) {
  const a = Math.hypot(m[0], m[1], m[2]);
  const b = Math.hypot(m[4], m[5], m[6]);
  const c = Math.hypot(m[8], m[9], m[10]);
  return Math.max(a, b, c);
}

function invert(m, out) {
  const n11=m[0],n21=m[1],n31=m[2],n41=m[3],
        n12=m[4],n22=m[5],n32=m[6],n42=m[7],
        n13=m[8],n23=m[9],n33=m[10],n43=m[11],
        n14=m[12],n24=m[13],n34=m[14],n44=m[15];
  const t11 = n23*n34*n42 - n24*n33*n42 + n24*n32*n43 - n22*n34*n43 - n23*n32*n44 + n22*n33*n44;
  const t12 = n14*n33*n42 - n13*n34*n42 - n14*n32*n43 + n12*n34*n43 + n13*n32*n44 - n12*n33*n44;
  const t13 = n13*n24*n42 - n14*n23*n42 + n14*n22*n43 - n12*n24*n43 - n13*n22*n44 + n12*n23*n44;
  const t14 = n14*n23*n32 - n13*n24*n32 - n14*n22*n33 + n12*n24*n33 + n13*n22*n34 - n12*n23*n34;
  const det = n11*t11 + n21*t12 + n31*t13 + n41*t14;
  if (det === 0) return null;
  const d = 1 / det;
  out[0]=t11*d;
  out[1]=(n24*n33*n41 - n23*n34*n41 - n24*n31*n43 + n21*n34*n43 + n23*n31*n44 - n21*n33*n44)*d;
  out[2]=(n22*n34*n41 - n24*n32*n41 + n24*n31*n42 - n21*n34*n42 - n22*n31*n44 + n21*n32*n44)*d;
  out[3]=(n23*n32*n41 - n22*n33*n41 - n23*n31*n42 + n21*n33*n42 + n22*n31*n43 - n21*n32*n43)*d;
  out[4]=t12*d;
  out[5]=(n13*n34*n41 - n14*n33*n41 + n14*n31*n43 - n11*n34*n43 - n13*n31*n44 + n11*n33*n44)*d;
  out[6]=(n14*n32*n41 - n12*n34*n41 - n14*n31*n42 + n11*n34*n42 + n12*n31*n44 - n11*n32*n44)*d;
  out[7]=(n12*n33*n41 - n13*n32*n41 + n13*n31*n42 - n11*n33*n42 - n12*n31*n43 + n11*n32*n43)*d;
  out[8]=t13*d;
  out[9]=(n14*n23*n41 - n13*n24*n41 - n14*n21*n43 + n11*n24*n43 + n13*n21*n44 - n11*n23*n44)*d;
  out[10]=(n12*n24*n41 - n14*n22*n41 + n14*n21*n42 - n11*n24*n42 - n12*n21*n44 + n11*n22*n44)*d;
  out[11]=(n13*n22*n41 - n12*n23*n41 - n13*n21*n42 + n11*n23*n42 + n12*n21*n43 - n11*n22*n43)*d;
  out[12]=t14*d;
  out[13]=(n13*n24*n31 - n14*n23*n31 + n14*n21*n33 - n11*n24*n33 - n13*n21*n34 + n11*n23*n34)*d;
  out[14]=(n14*n22*n31 - n12*n24*n31 - n14*n21*n32 + n11*n24*n32 + n12*n21*n34 - n11*n22*n34)*d;
  out[15]=(n12*n23*n31 - n13*n22*n31 + n13*n21*n32 - n11*n23*n32 - n12*n21*n33 + n11*n22*n33)*d;
  return out;
}
