// ============================================================
// js/state.js — shared mutable globals + on-demand render flag
// ============================================================
// Mirrors the RobotVisualisation state pattern: ES modules cannot
// reassign a foreign binding, so `let` exports are mutated through
// the setter helpers below.

// Three.js core (initialised by scene.js)
export let scene         = null;
export let camera        = null;   // active camera
export let perspCamera   = null;
export let orthoCamera   = null;
export let renderer      = null;
export let labelRenderer = null;
export let orbitControls = null;
export let orthoOn       = false;

export function initCore(s, persp, ortho, r, lr, oc) {
  scene = s; perspCamera = persp; orthoCamera = ortho;
  camera = persp; renderer = r; labelRenderer = lr; orbitControls = oc;
}
export function setActiveCamera(cam) { camera = cam; }
export function setOrthoOn(v)        { orthoOn = v; }

// ── Quasicrystal DMS primary (i-AlPdMn) ─────────────────────
// DMS on a quasicrystal needs a parallel-space primary reflection (the secondary
// list is derived from the icosahedral sliders — see quasicrystal.icoDmsReflections).
// This is the i-AlPdMn primary used by the sibling DMS project.
export const ICO_PRIMARY = [2.279680642791141, 3.703079697081845, 1.2960038712280701];

// ── Viewer parameters (the current reciprocal-space model) ──
// A single object holding every control-panel value. panel.js mutates
// it and calls rebuild(); reflections.js reads it to build the peaks.
export const params = {
  mode:      'crystal',            // 'crystal' | 'ico'
  // Conventional crystal
  system:    'cubic',              // cubic|tetragonal|orthorhombic|hexagonal|trigonal|monoclinic|triclinic
  lattice:   [4.0, 4.0, 4.0, 90, 90, 90],  // a,b,c (Å), alpha,beta,gamma (deg)
  centring:  'P',                  // P|I|F|A|B|C|R
  hklRange:  6,
  // Icosahedral quasicrystal
  aq:        6.461,                // 6D hypercubic edge (Å)
  sixDRange: 3,                    // 6D index box half-width
  sigmaPerp: 1.2,                  // perpendicular-space damping width
  iThreshold: 0.02,               // min relative intensity to render
  // Shared
  qMax:      3.0,                  // max |Q_par| (Å^-1) to render
  colourBy:  'qmag',              // 'qmag' | 'intensity' | 'axis'
  pointScale: 1.0,
  showBasis: true,
  showAxes:  true,
  // ── Visibility box (js/visbox.js) ──
  // A draggable box that hides everything inside or outside it — a view filter,
  // nothing is deleted. Its pose lives on the box itself, not here.
  visBox: {
    show:         false,
    showOutside:  false,      // false = show what's inside the box
    gizmo:        'translate',// 'translate' | 'rotate' | 'scale'
    clipDetector: false,      // also cut the detector panel, not just the map
  },
  // ── Detector panel (drawn in the scene, dragged with a gizmo) ──
  // The panel is a plane at `displayDist` scene units from the sample; only the
  // *direction* to each panel point matters for Q, so this distance is a display
  // choice, not a physical one. See js/optics.js and js/detector.js.
  detector: {
    show:        false,
    energy:      15.0,            // keV — sets k0 / the Ewald sphere for the blob
    displayDist: 2.4,            // scene units, sample → panel centre (initial)
    halfSize:    1.2,            // scene units, panel half-width
    // The fitted detector from a loaded DMS state (distance in px, plate
    // rotations, beam centre, frame size). Sent to /api/dms, which hands back
    // the matching rectangle for the panel. Null → the server's own defaults.
    fitted:      null,
    gizmo:       'translate',    // 'translate' | 'rotate'
    pivot:       'own',          // 'own' (rotate in place) | 'sample' (orbit origin)
  },
  // A Gaussian feature living on the panel; u,v are its local offset (scene
  // units) from the panel centre, sigma its width. Its image in Q bows onto the
  // Ewald sphere and tracks live as the blob or the panel moves.
  blob: {
    show:  false,
    sigma: 0.18,                 // scene units, on the panel
    u:     0.0, v: 0.0,          // local position on the panel
  },
  // ── DMS multiple-scattering curves (server: /api/dms) ──
  dms: {
    primaryHkl:  [1, 1, 1],
    energy:      15.0,            // keV
    azir:        [1, -1, 0],
    secondaryDepth: 3,           // conventional: hkl box half-width
    // Phason strain P (q∥ → q∥ + P·q⊥): zero for an unstrained quasicrystal.
    mtrx2:       [0, 0, 0, 0, 0, 0, 0, 0, 0],
    thrange:     [-27, 10],      // θ sweep *relative to the primary's θ_B*
    numsteps:    160,
    // How the curves come back from /api/dms: 'sweep' sends the sampled points as
    // polylines, 'circle' sends the analytic circle each DMS cone traces in Q for
    // the client to tessellate. Same θ-scan either way — see js/dms.js.
    method:      'sweep',        // 'sweep' | 'circle'
    psi:         -180,
    // ψ slider increment (°). Also the spacing of the trail slices below, since
    // one slice is laid down per ψ the slider stops on.
    psiStep:     1,
    psicor:      0,              // azimuthal correction (DMS guess slot 6)
    chicor:      0,              // χ correction (DMS guess slot 7)
    thetacor:    0,              // θ correction (DMS guess slot 8)
    maxCurves:   400,
    curveOpacity: 0.7,
    lineSigma:   0.012,          // DMS line half-width (world units: Å⁻¹ / scene)
    // Leave each ψ slice of the curves in place as ψ sweeps, so the network
    // traces the surface it sweeps through reciprocal space. Bounded, because a
    // slice is a full curve set — see pushTrail in js/dms.js.
    trail:       false,
    trailMax:    24,             // ψ slices kept; the oldest is dropped past this
    showCurves:  true,           // the Q-space obstacle curves
    showEwald:   true,           // the Ewald sphere they lie on
    showTriangle: false,         // the k_in / k_out / Q triangle for the primary
    projectOnPanel: true,        // draw the curves where they hit the panel
    showRecovered: true,         // highlight the caught points back in Q
  },
};

export function setParam(key, value) { params[key] = value; }

// ── On-demand rendering ─────────────────────────────────────
export let needsRender = true;
export function requestRender()    { needsRender = true; }
export function clearNeedsRender() { needsRender = false; }

// ── Peak metadata for hover/pick (populated by reflections.js) ──
// Parallel arrays indexed by point id: hkl (or 6D n), |Q|, intensity.
export let peakMeta = [];
export function setPeakMeta(m) { peakMeta = m; }
