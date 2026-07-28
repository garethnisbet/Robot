// ============================================================
// js/optics.js — the incident beam and the point → Q mapping
// ------------------------------------------------------------
// The whole "map a feature to reciprocal space" story is one line of physics:
// a point in the scene defines a scattered-beam *direction* from the sample at
// the origin, and
//
//     Q = k0 · û(point) − k_in                       (elastic, |k_out| = k0)
//
// so every point maps onto the Ewald sphere (centre −k_in, radius k0). Only the
// direction matters — how far away the detector panel is drawn is irrelevant to
// Q — which is why the panel can be dragged around the scene and its features
// mapped live, entirely client-side, with no server round trip.
//
// Convention: k = 1/λ, so |Q| = 2 sinθ/λ = 1/d, in Å⁻¹. k0 = E[keV] / 12.398,
// matching dms_compute (`ko = energy / 12.398`) so the DMS curves and the blob
// share one Ewald sphere. Lab default: beam along +y (the ts_quasi / geometry.py
// frame); the DMS computation re-points it along its own incident wavevector.
// ============================================================
import * as THREE from 'three';

const HC_KEV_ANGSTROM = 12.398;

let _k0 = 15.0 / HC_KEV_ANGSTROM;                 // Å⁻¹
let _s0 = new THREE.Vector3(0, 1, 0);             // unit beam direction
let _kIn = _s0.clone().multiplyScalar(_k0);       // incident wavevector

/** Point the beam. `energyKev` sets k0; `dir` (need not be unit) sets s0.
 *  Pass a falsy `dir` to keep the current direction. */
export function setBeam(energyKev, dir) {
  if (energyKev != null && isFinite(energyKev) && energyKev > 0) {
    _k0 = energyKev / HC_KEV_ANGSTROM;
  }
  if (dir) {
    const d = new THREE.Vector3(dir.x ?? dir[0], dir.y ?? dir[1], dir.z ?? dir[2]);
    if (d.lengthSq() > 1e-20) _s0 = d.normalize();
  }
  _kIn = _s0.clone().multiplyScalar(_k0);
}

export function k0()  { return _k0; }
export function s0()  { return _s0.clone(); }
export function kIn() { return _kIn.clone(); }

/** Centre of the Ewald sphere in Q space (−k_in); radius is k0(). */
export function ewaldCentre() { return _kIn.clone().multiplyScalar(-1); }

/** Scattering vector Q for a scene point (a direction from the sample). */
export function qFromWorld(pos) {
  const dir = new THREE.Vector3(pos.x, pos.y, pos.z);
  if (dir.lengthSq() < 1e-20) return _kIn.clone().multiplyScalar(-1);   // straight through
  return dir.normalize().multiplyScalar(_k0).sub(_kIn);
}

/** Where a ray from the sample (origin) along `dir` meets a plane.
 *  Returns the hit point (THREE.Vector3) or null if it is behind or parallel. */
export function rayPlaneHit(dir, planePoint, planeNormal) {
  const denom = dir.dot(planeNormal);
  if (Math.abs(denom) < 1e-9) return null;
  const t = planePoint.dot(planeNormal) / denom;
  if (!(t > 0)) return null;
  return dir.clone().multiplyScalar(t);
}
