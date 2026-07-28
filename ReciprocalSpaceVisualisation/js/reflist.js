// ============================================================
// js/reflist.js — load a reflection list from a DMS JSON file
// ------------------------------------------------------------
// The sibling DMS project writes its reflection lists into JSON in two
// shapes, and this module normalises either into one record:
//
//   • a saved state / reflection list (flat keys):
//       { version, bravais, hkl, azir, initial_guess[24], ref_6d,
//         ref_6d_checked, ... }
//     e.g. DMS/iAlPdMn_Annealed_913223.json, DMS/PMN_PT_ReflectionList.json
//
//   • a fit / workflow config (sectioned):
//       { experiment:{lattice,energy,azir}, geometry:{hkl,psi,azir},
//         computation:{bravais,numsteps,thrange_delta},
//         crystal:{ref_6d, initial_guess_base[24]} }
//     e.g. DMS/workflow_913123_dp3.json, DMSAnalysis/configs/*.json
//
// `azir`, the azimuthal reference, only appears in a slider state from version 3
// on (DMSAnalysis/slider.py) — before that the viewer had to re-read it from the
// scan's .dat, which it still does for older files. See `azirFrom`.
//
// `ref_6d` rows are 6 wide for an icosahedral quasicrystal (rank-6 indices)
// and 3 wide for a conventional crystal (hkl) — which is what tells us the
// lattice mode. The 24-slot guess vector is laid out as in DMSAnalysis/fit.py:
//
//   0-5 lattice · 6-9 psi/h/k/l corrections · 10-13 detector · 14 energy
//   15-23 phason (strain) matrix, row-major
//
// In a *config* slot 14 is an offset added to the scan energy; in a *saved
// state* it is the absolute energy. Everything else is read the same way.
//
// The loaded list replaces the DMS secondary set: for a quasicrystal it is
// sent as `ref_6d` instead of the slider-derived set (quasicrystal.js →
// icoDmsReflections), for a crystal as `reflist` instead of the hkl box.
// ============================================================

// DMS bravais name → this viewer's crystal-system name (see panel.js SYSTEMS).
const BRAVAIS_SYSTEM = {
  cubic: 'cubic',
  tetragonal: 'tetragonal', tetragonal_a: 'tetragonal', tetragonal_b: 'tetragonal',
  hexagonal: 'hexagonal',
  orthorhombic: 'orthorhombic',
  monoclinic: 'monoclinic', monoclinic_a: 'monoclinic', monoclinic_c: 'monoclinic',
  rhombohedral: 'trigonal', trigonal: 'trigonal',
  triclinic: 'triclinic',
};

// Not every file records its bravais name (older reflection lists don't), so
// fall back to reading the constraints straight off the lattice parameters.
function inferSystem(l) {
  if (!l) return null;
  const [a, b, c, al, be, ga] = l;
  const eq = (x, y) => Math.abs(x - y) < 1e-4;
  const ninety = eq(al, 90) && eq(be, 90) && eq(ga, 90);
  if (ninety) {
    if (eq(a, b) && eq(b, c)) return 'cubic';
    if (eq(a, b) || eq(b, c) || eq(a, c)) return 'tetragonal';
    return 'orthorhombic';
  }
  if (eq(a, b) && eq(al, 90) && eq(be, 90) && eq(ga, 120)) return 'hexagonal';
  if (eq(a, b) && eq(b, c) && eq(al, be) && eq(be, ga)) return 'trigonal';   // rhombohedral
  if (eq(al, 90) && eq(ga, 90)) return 'monoclinic';
  return 'triclinic';
}

const num = (v) => (Number.isFinite(Number(v)) ? Number(v) : null);
function vec(v, n) {
  if (!Array.isArray(v) || v.length < n) return null;
  const out = v.slice(0, n).map(Number);
  return out.every(Number.isFinite) ? out : null;
}

// A direction, not just three numbers: `azir` is normalised to an angle about
// the primary (arctan2 of its components), so a zero vector has no direction at
// all and would silently pin every DMS line to an arbitrary azimuth. Reject it
// here so the caller falls back to reading the scan instead.
function dirVec(v) {
  const out = vec(v, 3);
  return (out && out.some((x) => x !== 0)) ? out : null;
}

/**
 * Normalise a parsed DMS JSON document into a reflection-list record.
 * Throws Error with a human-readable reason if it holds no usable list.
 */
export function parseReflectionJson(json, name = 'reflections.json') {
  if (Array.isArray(json)) json = { ref_6d: json };      // a bare list of indices
  if (!json || typeof json !== 'object') throw new Error('not a JSON object');

  const crystal = json.crystal || {};
  const exp     = json.experiment || {};
  const geom    = json.geometry || {};
  const comp    = json.computation || {};

  // ── the list itself ──
  const raw = crystal.ref_6d || crystal.reflist || json.ref_6d || json.reflist || json.reflections;
  if (!Array.isArray(raw) || raw.length === 0)
    throw new Error('no reflection list found (expected "ref_6d")');
  const rank = Array.isArray(raw[0]) ? raw[0].length : 0;
  if (rank !== 3 && rank !== 6)
    throw new Error('reflections must be 3 (hkl) or 6 (icosahedral) indices wide');
  const refs = raw.map((r, i) => {
    const v = vec(r, rank);
    if (!v || r.length !== rank) throw new Error(`reflection ${i} is not ${rank} numbers`);
    return v;
  });

  // Per-reflection checkboxes from the DMS slider UI (which ones were used).
  const rawChecked = json.ref_6d_checked || crystal.ref_6d_checked;
  const checked = (Array.isArray(rawChecked) && rawChecked.length === refs.length)
    ? rawChecked.map(Boolean) : null;

  // ── geometry that came with the list ──
  const igFlat = vec(json.initial_guess, 24);        // saved state: absolute values
  const igBase = vec(crystal.initial_guess_base, 24); // config: energy slot is an offset
  const ig = igFlat || igBase;

  let energy = null;
  if (igFlat) energy = igFlat[14];
  else if (igBase && exp.energy != null) energy = Number(exp.energy) + igBase[14];
  else if (exp.energy != null) energy = Number(exp.energy);
  if (!(energy > 0)) energy = null;

  const bravaisRaw = (comp.bravais || json.bravais || '').toString().toLowerCase() || null;
  const mode = rank === 6 ? 'ico' : 'crystal';
  const lattice = (ig ? ig.slice(0, 6) : null) || vec(exp.lattice, 6) || vec(json.lattice, 6);
  const scanNum = num(json.scannum) ?? num(json.scan?.scannum);
  const datapoint = num(json.datapoint) ?? num(json.scan?.datapoint);

  // The azimuthal reference, in every place the DMS project writes it: a
  // version-3 slider state records it at the top level (already in the active
  // pseudo-cubic indexing, like `hkl` and `ref_6d`), a fit config under
  // `experiment`, a tripfit config under `geometry`. Older states have none —
  // then it is recovered from the scan (js/panel.js → /api/scanmeta).
  const azir = dirVec(json.azir) || dirVec(exp.azir) || dirVec(geom.azir);

  return {
    name,
    version: num(json.version),
    mode,
    rank,
    refs,
    checked,
    bravais: bravaisRaw,
    system: mode === 'crystal'
      ? (BRAVAIS_SYSTEM[bravaisRaw] || inferSystem(lattice)) : null,
    lattice,
    energy,
    primary: vec(geom.hkl, 3) || vec(json.hkl, 3),
    azir,
    azirFrom: azir ? 'file' : null,
    psi: num(geom.psi) ?? num(json.psi),
    // Slot 6: the azimuthal correction the fit refined. The DMS engine offsets
    // every outgoing beam by it, so dropping it rotates the whole pattern.
    psicor: ig ? ig[6] : null,
    // Slots 7/8: the χ / θ corrections the fit refined (in the DMS 24-vector,
    // slot 7 = χ, slot 8 = θ). χcor rotates the scan-hkl list about the chi
    // axis, so it changes which secondaries diffract across the sweep — a fitted
    // χcor of a few thousandths of a degree still moves the Kossel lines. The
    // engine applies both (slider.py dmsfit_ico_hkl slots 7/8); dropping them
    // makes the viewer's on-panel pattern disagree with the slider's.
    chicor: ig ? ig[7] : null,
    thetacor: ig ? ig[8] : null,
    // θ sweep, *relative to the primary's Bragg angle* (hence "delta").
    thrange: vec(comp.thrange_delta, 2) || vec(json.thrange, 2),
    numsteps: num(comp.numsteps) ?? num(json.numsteps),
    // Phason strain matrix (row-major). Only meaningful for a quasicrystal — a
    // conventional fit never refines slots 15-23, so whatever sits there is stale.
    mtrx2: (mode === 'ico' && ig) ? ig.slice(15, 24) : null,
    // The fitted detector: distance (px), the three plate rotations, and the
    // beam centre. Only taken from a *saved state*, where slots 10-13 hold the
    // values the engine was actually run with — a fit config stores a scaled
    // detdist (fit.py divides it by 2 or 3 depending on the mode), and guessing
    // which is worse than leaving the viewer's default in place.
    // The image size is in neither: it is the Pilatus 2M frame the DMS project
    // assumes (slider.py BLANK_IMAGE_SHAPE).
    detector: igFlat ? {
      detdist: igFlat[10], rotx: igFlat[11], roty: igFlat[12], rotz: igFlat[13],
      px: num(json.px) ?? num(geom.px_unscaled),
      py: num(json.py) ?? num(geom.py_unscaled),
      imshape: [1679, 1475],
    } : null,
    scanNum,
    datapoint,
    // Where the experiment itself is recorded. Only needed when the file has no
    // `azir` of its own (a slider state before version 3): the slider re-reads
    // it from this scan's .dat on restore, and so does the viewer
    // (js/panel.js → /api/scanmeta).
    scan: json.scan?.scanpath ? {
      scanpath:   String(json.scan.scanpath),
      scannum:    num(json.scan.scannum) ?? scanNum,
      datapoint:  num(json.scan.datapoint) ?? datapoint,
      datapoint0: num(json.scan.datapoint0),
    } : null,
  };
}

/** Ask the server for a session's beamline metadata (lattice/energy/azir). */
export async function fetchScanMeta(scan) {
  const res = await fetch('/api/scanmeta', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(scan),
  });
  const body = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(body.detail || body.error || `HTTP ${res.status}`);
  return body;   // { lattice, energy, energy0, azir, image_template }
}

// ── Module state: the currently loaded list ─────────────────
let loaded = null;
let enabled = true;         // use it for DMS (vs the generated secondary set)
let checkedOnly = true;     // honour ref_6d_checked

export function setLoaded(rec) { loaded = rec; enabled = !!rec; }
export function clearLoaded()  { loaded = null; }
export function getLoaded()    { return loaded; }
export function isEnabled()    { return !!loaded && enabled; }
export function setEnabled(v)  { enabled = !!v; }
export function getCheckedOnly() { return checkedOnly; }
export function setCheckedOnly(v) { checkedOnly = !!v; }

/** The reflections to send to /api/dms, or null if the list can't serve `rank`. */
export function activeRefs(rank) {
  if (!isEnabled() || loaded.rank !== rank) return null;
  const out = (loaded.checked && checkedOnly)
    ? loaded.refs.filter((_, i) => loaded.checked[i])
    : loaded.refs;
  return out.length ? out : null;
}

/** How many reflections the list would contribute right now. */
export function activeCount() {
  if (!loaded) return 0;
  return (loaded.checked && checkedOnly)
    ? loaded.checked.filter(Boolean).length
    : loaded.refs.length;
}

/** Fetch a JSON file by server-side path (see server.py /api/reflections). */
export async function fetchByPath(path) {
  const res = await fetch('/api/reflections', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ path }),
  });
  const body = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(body.detail || body.error || `HTTP ${res.status}`);
  return body;   // { name, path, data }
}
