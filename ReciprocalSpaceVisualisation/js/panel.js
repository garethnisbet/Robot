// ============================================================
// js/panel.js — dark control panel (built dynamically)
// ------------------------------------------------------------
// One flat panel, no tabs. Reads/writes State.params and rebuilds the
// reciprocal-space map live. Sections:
//   Lattice   — conventional crystal or icosahedral quasicrystal
//   Display   — how the peak cloud is drawn
//   Visibility box — a draggable box that hides everything inside/outside it
//   Detector  — the draggable panel (gizmo, pivot, size) + beam energy
//   Feature   — a Gaussian blob on the panel, mapped to Q live
//   Reflections — a reflection list loaded from a DMS JSON file
//   DMS       — multiple-scattering curves, projected onto the panel
// ============================================================
import * as State from './state.js';
import * as RefList from './reflist.js';
import * as VisBox from './visbox.js';
import { rebuild, getPoints } from './reflections.js';
import { setOrtho, setAxesVisible, setBasisVisible, snapView } from './scene.js';
import { computeDMS, refreshRender as refreshDms, hasCurves,
         clearTrail, refreshTrail, trailInfo } from './dms.js';
import { setDetectorVisible, setPanelSize, resetPanel, applyGizmo,
         setBlobVisible, setBlobSigma, onBlobQChange, refreshProjection,
         setBeamEnergy, setTriangleVisible, viewAlongKout,
         twoThetaDeg, primaryTwoThetaDeg } from './detector.js';

// system → { free:[indices editable], fill(l) normalises dependents }
const SYSTEMS = {
  cubic:        { free: [0],        fill: (l) => { l[1] = l[2] = l[0]; l[3] = l[4] = l[5] = 90; } },
  tetragonal:   { free: [0, 2],     fill: (l) => { l[1] = l[0]; l[3] = l[4] = l[5] = 90; } },
  orthorhombic: { free: [0, 1, 2],  fill: (l) => { l[3] = l[4] = l[5] = 90; } },
  hexagonal:    { free: [0, 2],     fill: (l) => { l[1] = l[0]; l[3] = l[4] = 90; l[5] = 120; } },
  trigonal:     { free: [0, 3],     fill: (l) => { l[1] = l[2] = l[0]; l[4] = l[5] = l[3]; } },
  monoclinic:   { free: [0, 1, 2, 4], fill: (l) => { l[3] = l[5] = 90; } },
  triclinic:    { free: [0, 1, 2, 3, 4, 5], fill: () => {} },
};
const LAT_NAMES = ['a', 'b', 'c', 'α', 'β', 'γ'];

let els = {};       // named references to live inputs
const panel = document.getElementById('panel');

// ── small DOM helpers ───────────────────────────────────────
function h2(text) { const e = document.createElement('h2'); e.textContent = text; return e; }
function el(tag, cls, txt) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (txt != null) e.textContent = txt;
  return e;
}
function numberInput(value, step, onChange) {
  const inp = el('input'); inp.type = 'number'; inp.step = step; inp.value = value;
  inp.addEventListener('input', () => onChange(parseFloat(inp.value)));
  return inp;
}
function textInput(value, placeholder) {
  const inp = el('input'); inp.type = 'text'; inp.value = value || '';
  if (placeholder) inp.placeholder = placeholder;
  return inp;
}
function select(options, value, onChange) {
  const s = el('select');
  options.forEach(([v, label]) => {
    const o = el('option', null, label); o.value = v;
    if (v === value) o.selected = true;
    s.appendChild(o);
  });
  s.addEventListener('change', () => onChange(s.value));
  return s;
}
function button(txt, onClick) { const b = el('button', null, txt); b.addEventListener('click', onClick); return b; }

// slider + numeric readout bound to a params key on `obj`
function slider(label, obj, key, min, max, step, onInput, fmtFn) {
  const row = el('div', 'slider-row');
  const lab = el('div', 'slabel');
  const name = el('span', null, label); name.style.color = '#99a'; name.style.fontWeight = 'normal';
  const val = el('span', null, (fmtFn || fmt)(obj[key]));
  lab.append(name, val);
  const rng = el('input'); rng.type = 'range'; rng.min = min; rng.max = max; rng.step = step; rng.value = obj[key];
  rng.addEventListener('input', () => {
    // Read the step off the element rather than the closure: ψ's is settable
    // after the row is built, and a stale integer step would truncate a
    // fractional value to nothing.
    const st = parseFloat(rng.step);
    const v = parseFloat(rng.value);
    obj[key] = (st % 1 === 0) ? Math.round(v) : v;
    val.textContent = (fmtFn || fmt)(obj[key]);
    onInput?.();
  });
  row.append(lab, rng);
  row.__range = rng; row.__val = val; row.__fmt = fmtFn || fmt;
  return row;
}
function fieldRow(label, control) {
  const row = el('div', 'field-row');
  row.appendChild(el('label', null, label));
  row.appendChild(control);
  return row;
}
function checkbox(label, checked, onChange) {
  const row = el('label', 'checkbox-row');
  const c = el('input'); c.type = 'checkbox'; c.checked = checked;
  c.addEventListener('change', () => onChange(c.checked));
  row.append(c, document.createTextNode(label));
  return row;
}
function fmt(v) { return (Math.abs(v) >= 100 || v % 1 === 0) ? String(v) : v.toFixed(3).replace(/0+$/, '').replace(/\.$/, ''); }

// ── build the panel ─────────────────────────────────────────
export function buildPanel() {
  const p = State.params;
  panel.innerHTML = '';
  els = {};

  panel.appendChild(el('h1', null, 'Reciprocal Space Map'));

  // ── Lattice source toggle ──
  const modeWrap = el('div', 'mode-toggle');
  els.crystalBtn = button('Crystal', () => setLatticeMode('crystal'));
  els.icoBtn = button('Quasicrystal', () => setLatticeMode('ico'));
  modeWrap.append(els.crystalBtn, els.icoBtn);
  panel.appendChild(modeWrap);

  // ── Crystal section ──
  els.crystalSec = el('div');
  els.crystalSec.appendChild(h2('Crystal system'));
  els.systemSel = select(
    Object.keys(SYSTEMS).map((s) => [s, s]), p.system,
    (v) => { p.system = v; SYSTEMS[v].fill(p.lattice); refreshLatticeInputs(); rebuild(); });
  els.crystalSec.appendChild(els.systemSel);

  els.crystalSec.appendChild(h2('Lattice (Å, °)'));
  const grid = el('div', 'lattice-grid');
  els.latInputs = [];
  for (let i = 0; i < 6; i++) {
    grid.appendChild(el('span', 'axis', LAT_NAMES[i]));
    const step = i < 3 ? '0.05' : '0.5';
    const inp = numberInput(p.lattice[i], step, (val) => {
      if (isNaN(val)) return;
      p.lattice[i] = val; SYSTEMS[p.system].fill(p.lattice);
      refreshLatticeInputs(); rebuild();
    });
    els.latInputs.push(inp);
    grid.appendChild(inp);
  }
  els.crystalSec.appendChild(grid);

  els.crystalSec.appendChild(h2('Centring'));
  els.centringSel = select(
    [['P', 'P — primitive'], ['I', 'I — body'], ['F', 'F — face'],
     ['A', 'A'], ['B', 'B'], ['C', 'C'], ['R', 'R — rhombohedral']],
    p.centring, (v) => { p.centring = v; rebuild(); });
  els.crystalSec.appendChild(els.centringSel);
  els.crystalSec.appendChild(sliderRow('hkl range ±', 'hklRange', p.hklRange, 1, 12, 1));
  panel.appendChild(els.crystalSec);

  // ── Quasicrystal section ──
  els.icoSec = el('div');
  els.icoSec.appendChild(h2('Icosahedral (6D)'));
  els.icoSec.appendChild(fieldRow('a₆D (Å)', numberInput(p.aq, '0.01',
    (v) => { if (!isNaN(v)) { p.aq = v; icoChanged(); } })));
  els.icoSec.appendChild(slider('6D range ±', p, 'sixDRange', 1, 5, 1, icoChanged));
  els.icoSec.appendChild(slider('σ⊥ (phason)', p, 'sigmaPerp', 0.2, 4, 0.1, icoChanged));
  els.icoSec.appendChild(slider('I threshold', p, 'iThreshold', 0, 0.5, 0.005, icoChanged));
  panel.appendChild(els.icoSec);

  // ── Display section ──
  const disp = el('div');
  disp.appendChild(h2('Display'));
  disp.appendChild(sliderRow('|Q|max (Å⁻¹)', 'qMax', p.qMax, 0.5, 10, 0.1));
  disp.appendChild(sliderRow('point scale', 'pointScale', p.pointScale, 0.2, 3, 0.1));
  disp.appendChild(fieldRow('colour by', select(
    [['qmag', '|Q| (resolution)'], ['intensity', 'intensity'], ['axis', 'axis']],
    p.colourBy, (v) => { p.colourBy = v; rebuild(); })));
  disp.appendChild(checkbox('show basis vectors', p.showBasis,
    (v) => { p.showBasis = v; setBasisVisible(v); rebuild(); }));
  disp.appendChild(checkbox('show axes', p.showAxes,
    (v) => { p.showAxes = v; setAxesVisible(v); }));
  disp.appendChild(checkbox('orthographic', State.orthoOn, (v) => setOrtho(v)));
  panel.appendChild(disp);

  // ── Visibility box section ──
  panel.appendChild(buildVisBoxSection(p));

  // ── Detector section ──
  panel.appendChild(buildDetectorSection(p));

  // ── Feature (blob) section ──
  panel.appendChild(buildBlobSection(p));

  // ── Reflection list (loaded from a DMS JSON) ──
  panel.appendChild(buildRefListSection());

  // ── DMS section ──
  panel.appendChild(buildDmsSection(p));

  // ── Views ──
  panel.appendChild(h2('View'));
  const views = el('div', 'button-grid four');
  ['iso', 'top', 'front', 'side'].forEach((v) =>
    views.appendChild(button(v, () => snapView(v))));
  panel.appendChild(views);
  // Look down the primary's diffracted beam (needs a computed DMS primary).
  els.koutView = button('look along k_out', () => {
    els.koutView.textContent = viewAlongKout() ? 'look along k_out' : 'compute DMS first';
    setTimeout(() => { els.koutView.textContent = 'look along k_out'; }, 1400);
  });
  panel.appendChild(els.koutView);

  // ── Count note ──
  els.count = el('div', 'count-note', '');
  panel.appendChild(els.count);

  applyModeUI();
  refreshLatticeInputs();
}

// slider bound to a params top-level key (crystal/quasi/display sliders)
function sliderRow(label, key, value, min, max, step) {
  return slider(label, State.params, key, min, max, step, () => rebuild());
}

// Icosahedral controls rebuild the lattice and, once DMS has been computed, re-run
// it (debounced) so the pattern tracks the sliders — the DMS secondaries are
// derived from these same parameters (quasicrystal.icoDmsReflections).
function icoChanged() { rebuild(); scheduleDmsRecompute(); }

// ── lattice mode / refresh ──────────────────────────────────
function setLatticeMode(mode) {
  const p = State.params;
  if (mode !== p.mode) {
    p.mode = mode;
    // The DMS primary lives in a different index space per lattice type, and the
    // quasicrystal needs its 6D secondary set (there is no hkl-box generator for
    // it) — swap to sensible defaults so Compute DMS works in either mode.
    if (mode === 'ico') {
      p.dms.primaryHkl = State.ICO_PRIMARY.slice();
    } else {
      p.dms.primaryHkl = [1, 1, 1];
    }
    buildPanel();          // reflect the new primary + section visibility
  }
  rebuild();
}
function applyModeUI() {
  const p = State.params;
  const ico = p.mode === 'ico';
  els.crystalSec.style.display = ico ? 'none' : '';
  els.icoSec.style.display = ico ? '' : 'none';
  els.crystalBtn.classList.toggle('active', !ico);
  els.icoBtn.classList.toggle('active', ico);
  updateDepthRow();
  const pts = getPoints(); if (pts) pts.visible = true;
}
function refreshLatticeInputs() {
  const p = State.params;
  const free = SYSTEMS[p.system].free;
  els.latInputs.forEach((inp, i) => {
    inp.value = fmt(p.lattice[i]);
    const editable = free.includes(i);
    inp.disabled = !editable;
    inp.style.opacity = editable ? '1' : '0.45';
  });
  if (els.systemSel) els.systemSel.value = p.system;
  if (els.centringSel) els.centringSel.value = p.centring;
}

// Update the "N peaks" readout after each rebuild.
export function setCount(n) {
  if (els.count) els.count.textContent = n.toLocaleString() + ' peaks';
}

// ── Visibility box section ──────────────────────────────────
// The LidarStudio viewer's box, in reciprocal space: place it, drag/rotate/
// scale it, and everything inside (or outside) is hidden. A view filter only —
// nothing is deleted, so Clear always brings the whole map back.
function buildVisBoxSection(p) {
  const v = p.visBox;
  const sec = el('div');
  sec.appendChild(h2('Visibility box'));
  sec.appendChild(el('div', 'count-note',
    'hides everything inside or outside the box — nothing is deleted'));

  els.visPlace = button('place visibility box', () => {
    // A box already on screen re-fits to the current peaks; placing after a
    // Clear brings the cleared box back where it was.
    VisBox.placeBox({ refit: VisBox.hasBox() });
  });
  sec.appendChild(els.visPlace);

  const modeRow = el('div', 'button-grid three');
  els.visMove = button('move', () => VisBox.setGizmoMode('translate'));
  els.visRotate = button('rotate', () => VisBox.setGizmoMode('rotate'));
  els.visScale = button('scale', () => VisBox.setGizmoMode('scale'));
  modeRow.append(els.visMove, els.visRotate, els.visScale);
  sec.appendChild(modeRow);
  sec.appendChild(el('div', 'count-note', 'or press T / R / S'));

  const actRow = el('div', 'button-grid');
  els.visSide = button('showing: inside', () => VisBox.setShowOutside(!v.showOutside));
  actRow.append(els.visSide, button('clear', () => VisBox.clearBox()));
  sec.appendChild(actRow);

  sec.appendChild(checkbox('cut the detector panel too', v.clipDetector,
    (on) => VisBox.setClipDetector(on)));

  markVisBox();
  return sec;
}

// Keep the section in step with the box, however it was changed (the T/R/S
// keys and Clear both bypass the buttons).
function markVisBox() {
  const v = State.params.visBox;
  const on = VisBox.hasBox();
  if (els.visPlace) els.visPlace.textContent = on ? 're-fit box to data' : 'place visibility box';
  els.visMove?.classList.toggle('active', on && v.gizmo === 'translate');
  els.visRotate?.classList.toggle('active', on && v.gizmo === 'rotate');
  els.visScale?.classList.toggle('active', on && v.gizmo === 'scale');
  if (els.visSide) {
    els.visSide.textContent = `showing: ${v.showOutside ? 'outside' : 'inside'}`;
    els.visSide.disabled = !on;
    els.visSide.style.opacity = on ? '1' : '0.5';
  }
}
addEventListener('visBoxChanged', markVisBox);

// ── Detector control section ────────────────────────────────
function buildDetectorSection(p) {
  const d = p.detector;
  const sec = el('div');
  sec.appendChild(h2('Detector panel'));

  sec.appendChild(checkbox('show detector panel', d.show, (on) => {
    d.show = on; setDetectorVisible(on);
  }));

  sec.appendChild(fieldRow('energy (keV)', numberInput(d.energy, '0.1', (v) => {
    if (isNaN(v)) return; d.energy = v; setBeamEnergy(v);
  })));

  sec.appendChild(slider('panel size', d, 'halfSize', 0.4, 3, 0.1,
    () => setPanelSize(d.halfSize)));
  // How far out the modelled detector is drawn. Only the *direction* from the
  // sample matters to Q, so this is pure display — but it rescales the panel,
  // which is now a to-scale model of the real plate.
  sec.appendChild(slider('display distance', d, 'displayDist', 1, 6, 0.1,
    () => resetPanel()));

  // gizmo mode
  sec.appendChild(el('label', null, 'drag mode'));
  const modeRow = el('div', 'button-grid'); modeRow.style.gridTemplateColumns = '1fr 1fr';
  els.gizmoTranslate = button('translate', () => { d.gizmo = 'translate'; applyGizmo(); markGizmo(); });
  els.gizmoRotate = button('rotate', () => { d.gizmo = 'rotate'; applyGizmo(); markGizmo(); });
  modeRow.append(els.gizmoTranslate, els.gizmoRotate);
  sec.appendChild(modeRow);

  // rotation pivot
  sec.appendChild(el('label', null, 'rotate about'));
  const pivotRow = el('div', 'button-grid'); pivotRow.style.gridTemplateColumns = '1fr 1fr';
  els.pivotOwn = button('own origin', () => { d.pivot = 'own'; applyGizmo(); markGizmo(); });
  els.pivotSample = button('sample', () => { d.pivot = 'sample'; applyGizmo(); markGizmo(); });
  pivotRow.append(els.pivotOwn, els.pivotSample);
  sec.appendChild(pivotRow);
  markGizmo();

  // The panel models the detector the DMS engine computes against — the fitted
  // distance, plate rotations and beam centre when a DMS state supplied them.
  sec.appendChild(button('place on detector (as fitted)',
    () => { resetPanel(); refreshTwoTheta(); }));
  els.twoTheta = el('div', 'count-note', '');
  sec.appendChild(els.twoTheta);
  refreshTwoTheta();
  return sec;
}

// Readout under the placement button. The engine's detector sits opposite the
// incident beam (back-reflection, 2θ ≈ 180°) where the diffracted Kossel beams
// land; the primary reflection (its own 2θ) is *off* it — worth saying, because
// "the primary isn't on the panel" otherwise looks like a bug.
function refreshTwoTheta() {
  if (!els.twoTheta) return;
  const tt = twoThetaDeg(), pt = primaryTwoThetaDeg();
  const f = State.params.detector.fitted;
  if (tt == null) {
    els.twoTheta.textContent = 'compute DMS to place the panel on the detector';
    return;
  }
  const bits = [`detector 2θ = ${tt.toFixed(3)}°`];
  if (pt != null) bits.push(`primary 2θ = ${pt.toFixed(3)}°`);
  if (f) bits.push(`${f.imshape[0]}×${f.imshape[1]} px at ${f.detdist.toFixed(0)} px`
    + ` · beam centre (${f.px}, ${f.py})`);
  els.twoTheta.textContent = bits.join(' · ');
}
addEventListener('dmsUpdated', refreshTwoTheta);
function markGizmo() {
  const d = State.params.detector;
  els.gizmoTranslate?.classList.toggle('active', d.gizmo === 'translate');
  els.gizmoRotate?.classList.toggle('active', d.gizmo === 'rotate');
  els.pivotOwn?.classList.toggle('active', d.pivot === 'own');
  els.pivotSample?.classList.toggle('active', d.pivot === 'sample');
}

// ── Feature (blob) section ──────────────────────────────────
function buildBlobSection(p) {
  const b = p.blob;
  const sec = el('div');
  sec.appendChild(h2('Feature on the panel'));
  sec.appendChild(checkbox('show Gaussian blob', b.show, (on) => {
    b.show = on; setBlobVisible(on);
  }));
  sec.appendChild(slider('blob σ', b, 'sigma', 0.03, 0.6, 0.01, () => setBlobSigma(b.sigma)));
  els.blobQ = el('div', 'count-note', 'click the panel to place · drag to move');
  sec.appendChild(els.blobQ);
  onBlobQChange((q) => {
    els.blobQ.textContent = (q == null)
      ? 'click the panel to place · drag to move'
      : `|Q| = ${q.toFixed(3)} Å⁻¹`;
  });
  return sec;
}

// ── Reflection list from a DMS JSON file ────────────────────
// The DMS project's saved states / fit configs carry the secondary reflection
// list that was actually used (`ref_6d`, 6 wide for a quasicrystal, 3 for a
// crystal) together with the geometry it was fitted with. Loading one replaces
// the generated secondary set and applies that geometry, so the viewer
// reproduces the same DMS pattern. See js/reflist.js.
const REF_PATH_KEY = 'recipvis.refListPath';
let refError = '';         // transient load error, survives one buildPanel()

function buildRefListSection() {
  const sec = el('div');
  sec.appendChild(h2('Reflection list (DMS JSON)'));

  const file = el('input');
  file.type = 'file'; file.accept = '.json,application/json'; file.style.display = 'none';
  file.addEventListener('change', () => {
    const f = file.files?.[0];
    if (!f) return;
    const rd = new FileReader();
    rd.onload = () => { file.value = ''; loadReflectionText(rd.result, f.name); };
    rd.onerror = () => { file.value = ''; showRefError(`could not read ${f.name}`); };
    rd.readAsText(f);
  });
  sec.appendChild(file);
  sec.appendChild(button('Load JSON file…', () => file.click()));

  // Load by path (the lists live in the sibling DMS project) — server-side read,
  // confined to the allowed roots; see server.py /api/reflections.
  els.refPath = textInput(localStorage.getItem(REF_PATH_KEY) || '', '/path/to/reflections.json');
  els.refPath.addEventListener('keydown', (e) => { if (e.key === 'Enter') loadReflectionPath(); });
  sec.appendChild(els.refPath);
  sec.appendChild(button('Load from path', loadReflectionPath));

  els.refStatus = el('div', 'count-note', '');
  sec.appendChild(els.refStatus);

  els.refUse = checkbox('use this list for DMS', RefList.isEnabled(), (v) => {
    RefList.setEnabled(v); refreshRefList(); scheduleDmsRecompute();
  });
  els.refChecked = checkbox('use only checked reflections', RefList.getCheckedOnly(), (v) => {
    RefList.setCheckedOnly(v); refreshRefList(); scheduleDmsRecompute();
  });
  els.refClear = button('Clear list', () => {
    RefList.clearLoaded(); refError = ''; refreshRefList(); scheduleDmsRecompute();
  });
  sec.append(els.refUse, els.refChecked, els.refClear);

  refreshRefList();
  return sec;
}

// Status line + which controls make sense for the currently loaded list.
function refreshRefList() {
  const l = RefList.getLoaded();
  const on = !!l;
  els.refUse.style.display = on ? '' : 'none';
  els.refChecked.style.display = (on && l.checked) ? '' : 'none';
  els.refClear.style.display = on ? '' : 'none';

  if (!on) {
    els.refStatus.innerHTML = refError
      ? `<span style="color:#f88">${esc(refError)}</span>`
      : 'none loaded — DMS secondaries are generated from the controls above';
  } else {
    const kind = l.mode === 'ico' ? `icosahedral (6D)` : `${l.bravais || 'crystal'} (hkl)`;
    const nAct = RefList.activeCount();
    const bits = [`${l.refs.length} reflections${l.checked ? ` · ${nAct} checked` : ''}`, kind];
    if (l.scanNum) bits.push(`scan ${l.scanNum}${l.datapoint != null ? ` dp ${l.datapoint}` : ''}`);
    const applied = [];
    if (l.lattice) applied.push(l.mode === 'ico' ? `a₆D ${l.lattice[0].toFixed(4)} Å`
                                                 : `a ${l.lattice[0].toFixed(4)} Å`);
    if (l.energy) applied.push(`${l.energy.toFixed(4)} keV`);
    if (l.primary) applied.push(`primary ${l.primary.map((v) => (+v).toFixed(3)).join(' ')}`);
    if (l.psicor) applied.push(`ψcor ${(+l.psicor).toFixed(3)}°`);
    if (l.chicor) applied.push(`χcor ${(+l.chicor).toFixed(3)}°`);
    if (l.thetacor) applied.push(`θcor ${(+l.thetacor).toFixed(3)}°`);
    if (l.azir) applied.push(`azir ${l.azir.map((v) => (+v).toFixed(4)).join(' ')}`
      + (l.azirFrom ? ` (from ${l.azirFrom})` : ''));
    if (l.mtrx2 && l.mtrx2.some((v) => v !== 0)) applied.push('phason matrix');
    const mismatch = (l.mode !== State.params.mode)
      ? `<br><span style="color:#fc8">list is ${l.mode === 'ico' ? '6D' : 'hkl'} — not used in ${State.params.mode === 'ico' ? 'quasicrystal' : 'crystal'} mode</span>` : '';
    els.refStatus.innerHTML =
      `<b style="color:#8cf">${esc(l.name)}</b><br>${esc(bits.join(' · '))}` +
      (applied.length ? `<br>applied: ${esc(applied.join(' · '))}` : '') + mismatch +
      (refError ? `<br><span style="color:#f88">${esc(refError)}</span>` : '');
  }
  updateDepthRow();
}

// The secondary-depth box only applies when the hkl box generates the
// secondaries: not in icosahedral mode, and not while a loaded hkl list is in use.
function updateDepthRow() {
  if (!els.dmsDepthRow) return;
  els.dmsDepthRow.style.display =
    (State.params.mode === 'ico' || RefList.activeRefs(3)) ? 'none' : '';
}
// ── ψ sweep ─────────────────────────────────────────────────
// Drive ψ across its whole range at the step size, awaiting each computation so
// every stop lands a trail slice. `computeDMS` coalesces overlapping calls, so
// the await is what makes the trace complete rather than sampled — see the
// debounce note in scheduleDmsRecompute.
let sweeping = false;

/** Distinct ψ orientations a full sweep visits at `step` (+180 ≡ −180, so the
 *  wrap-around slice is not counted). Shared with the trail note, so the number
 *  quoted before a sweep is the number it produces. */
function sweepSlices(step) {
  return Math.ceil(360 / Math.max(0.01, step));
}

function setPsiSlider(v) {
  const row = els.dmsPsiSlider;
  if (!row) return;
  row.__range.value = v;
  row.__val.textContent = row.__fmt(v);
}

async function sweepPsi() {
  if (sweeping) { sweeping = false; return; }     // a second press stops it
  const d = State.params.dms;
  clearTimeout(dmsTimer);                          // no stale debounced run mid-sweep
  sweeping = true;
  if (!d.trail) {                                  // the sweep exists to record
    d.trail = true;
    const cb = els.dmsTrailCb?.querySelector('input');
    if (cb) cb.checked = true;
  }
  clearTrail();                                    // so the trail is exactly this sweep
  updateTrailRows();

  const step = Math.max(0.01, d.psiStep);
  // ψ is an azimuth: +180 is the same orientation as −180, so the sweep stops
  // one step short of wrapping rather than recording that slice twice.
  const n = sweepSlices(step);
  const psi0 = d.psi;
  let done = 0, failed = '';
  for (let i = 0; i < n && sweeping; i++) {
    // Built from i rather than accumulated, so the step never drifts.
    const psi = Math.round((-180 + i * step) * 1e6) / 1e6;
    if (psi >= 180) break;
    d.psi = psi;
    setPsiSlider(d.psi);
    let msg = '';
    await computeDMS((s) => { msg = s; });
    if (/^(error|request failed)/.test(msg)) { failed = msg; break; }
    done++;
    if (els.dmsStatus) {
      els.dmsStatus.textContent = `sweeping ψ… ${done}/${n} (ψ ${d.psi.toFixed(2)}°)`;
    }
    updateTrailRows();
  }
  const stopped = !sweeping;
  sweeping = false;
  if (!done) { d.psi = psi0; setPsiSlider(psi0); }
  const info = trailInfo();
  if (els.dmsStatus) {
    els.dmsStatus.textContent = failed
      ? `sweep stopped — ${failed}`
      : `swept ${done} ψ step${done === 1 ? '' : 's'}${stopped ? ' (stopped)' : ''} · `
        + `trail holds ${info.count}` + (info.count ? ` (ψ ${info.from.toFixed(0)}…${info.to.toFixed(0)}°)` : '');
  }
  updateTrailRows();
}

// The trail's length control and its clear button are only meaningful once
// something is recording or recorded.
function updateTrailRows() {
  const d = State.params.dms;
  const info = trailInfo();
  const on = d.trail || info.count > 0;
  if (els.dmsTrailMaxRow) els.dmsTrailMaxRow.style.display = d.trail ? '' : 'none';
  if (els.dmsTrailClear) els.dmsTrailClear.style.display = info.count ? '' : 'none';
  if (els.dmsSweepBtn) {
    els.dmsSweepBtn.textContent = sweeping ? 'Stop sweep' : 'Sweep ψ (record every step)';
  }
  if (els.dmsTrailNote) {
    els.dmsTrailNote.style.display = on ? '' : 'none';
    const mb = info.bytes / 1048576;
    const bits = [info.count
      ? `${info.count} slice${info.count > 1 ? 's' : ''} held, ψ ${info.from.toFixed(0)}…${info.to.toFixed(0)}°`
        + ` · ${mb < 10 ? mb.toFixed(1) : mb.toFixed(0)} MB`
      : 'drag ψ to lay down slices'];
    // The ψ step is what decides how many slices a sweep produces, so spell the
    // arithmetic out rather than letting the trail silently drop most of them.
    if (d.trail) {
      const n = sweepSlices(d.psiStep);
      bits.push(`360° at ${fmt(d.psiStep)}° = ${n} slice${n > 1 ? 's' : ''}`
        + (n > d.trailMax ? `, last ${d.trailMax} kept` : ''));
    }
    els.dmsTrailNote.textContent = bits.join(' · ');
  }
}

// What the θ-scan steps actually buy differs by method, and that is the whole
// reason to pick one — so say it next to the field rather than in the docs.
function updateMethodNote() {
  if (!els.dmsMethodNote) return;
  els.dmsMethodNote.textContent = State.params.dms.method === 'circle'
    ? 'steps set only where each arc ends — the curve between is exact, so fewer '
      + 'steps stay smooth and cost less'
    : 'steps set the resolution of the whole curve — fewer steps visibly facet it';
}
function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}
function showRefError(msg) { refError = msg; refreshRefList(); }

// Parse + install a reflection-list document (file picker or path load).
function loadReflectionText(text, name) {
  let doc;
  try { doc = JSON.parse(text); }
  catch (e) { return showRefError(`${name}: not valid JSON — ${e.message}`); }
  installReflections(doc, name);
}
function installReflections(doc, name) {
  let rec;
  try { rec = RefList.parseReflectionJson(doc, name); }
  catch (e) { return showRefError(`${name}: ${e.message}`); }
  refError = '';
  RefList.setLoaded(rec);
  applyLoadedGeometry(rec);
  buildPanel();          // every field the file touched needs to redraw
  rebuild();
  if (!rec.azir && rec.scan) recoverScanMeta(rec);
  else scheduleDmsRecompute();
}

// Fallback for a slider state written before version 3, which recorded the
// refined sliders but not the experiment: `azir`, the azimuthal reference
// direction, lived only in the scan's .dat file and the slider re-read it on
// restore. It sets the axis the multiple-scattering condition is computed
// about, so guessing it puts every DMS line in the wrong place — recover it the
// same way, and say so plainly when we can't. Newer files carry their own
// `azir` and never reach here.
async function recoverScanMeta(rec) {
  let meta;
  try { meta = await RefList.fetchScanMeta(rec.scan); }
  catch (e) {
    showRefError(`azimuthal reference not recovered from scan ${rec.scan.scannum} `
      + `— the DMS lines will not match the slider until "azimuthal ref" is set `
      + `by hand (${e.message})`);
    scheduleDmsRecompute();
    return;
  }
  if (RefList.getLoaded() !== rec) return;      // superseded while we waited
  const az = Array.isArray(meta.azir) && meta.azir.length === 3
    ? meta.azir.map(Number) : null;
  if (az && az.every(Number.isFinite) && az.some((v) => v !== 0)) {
    rec.azir = az;
    rec.azirFrom = 'scan';
    State.params.dms.azir = rec.azir.slice();
    buildPanel();
  }
  scheduleDmsRecompute();
}
async function loadReflectionPath() {
  const path = els.refPath.value.trim();
  if (!path) return showRefError('enter a path to a .json file');
  localStorage.setItem(REF_PATH_KEY, path);
  els.refStatus.textContent = 'loading…';
  try {
    const body = await RefList.fetchByPath(path);
    installReflections(body.data, body.name || path);
  } catch (e) {
    showRefError(String(e.message || e));
  }
}

// Put the file's geometry into params: the lattice it was fitted with, the beam
// energy, the primary reflection and the DMS scan settings. Everything stays
// editable afterwards — this is a starting point, not a lock.
function applyLoadedGeometry(l) {
  const p = State.params;
  p.mode = l.mode;
  if (l.lattice) {
    if (l.mode === 'ico') {
      p.aq = l.lattice[0];
    } else {
      // Keep the fitted cell exactly as it was refined: if the system's
      // constraints would move it (an axis convention this viewer doesn't
      // have, say a unique-a tetragonal), drop to triclinic rather than
      // silently changing the lattice.
      if (l.system) p.system = l.system;
      const want = l.lattice.slice();
      const constrained = want.slice();
      SYSTEMS[p.system].fill(constrained);
      if (!constrained.every((v, i) => Math.abs(v - want[i]) < 1e-4)) p.system = 'triclinic';
      p.lattice = want;
    }
  }
  if (l.primary) p.dms.primaryHkl = l.primary.slice();
  if (l.energy) {
    p.dms.energy = l.energy;
    p.detector.energy = l.energy;      // the panel/blob mapping shares the beam
    setBeamEnergy(l.energy);
  }
  if (l.azir) p.dms.azir = l.azir.slice();
  if (l.psi != null) p.dms.psi = Math.max(-180, Math.min(180, l.psi));
  p.dms.psicor = l.psicor != null ? l.psicor : 0;
  p.dms.chicor = l.chicor != null ? l.chicor : 0;
  p.dms.thetacor = l.thetacor != null ? l.thetacor : 0;
  if (l.thrange) p.dms.thrange = l.thrange.slice();
  // Fit runs use a fine θ-scan; the viewer redraws it live, so keep it sane.
  if (l.numsteps) p.dms.numsteps = Math.max(10, Math.min(400, l.numsteps | 0));
  if (l.mtrx2) p.dms.mtrx2 = l.mtrx2.slice();
  // The fitted detector goes to /api/dms, which returns the matching rectangle
  // for the panel. Needs a beam centre to be usable at all.
  p.detector.fitted = (l.detector && l.detector.px != null && l.detector.py != null)
    ? l.detector : null;
}

// Re-run DMS a moment after a live control (ψ, θ-range) settles, so dragging it
// sweeps the pattern without firing a request per keystroke. Only recomputes
// once curves already exist — otherwise it just stages the value for Compute DMS.
let dmsTimer = null;
// Every DMS run reports through here, so the trail readout keeps up with a ψ drag.
function dmsStatusCb(s) {
  if (els.dmsStatus) els.dmsStatus.textContent = s;
  updateTrailRows();
}
function scheduleDmsRecompute() {
  clearTimeout(dmsTimer);
  dmsTimer = setTimeout(() => {
    if (hasCurves()) computeDMS(dmsStatusCb);
  }, 200);
}

// ── DMS control section ─────────────────────────────────────
function buildDmsSection(p) {
  const d = p.dms;
  const sec = el('div');
  sec.appendChild(h2('DMS multiple scattering'));

  // primary hkl
  sec.appendChild(el('label', null, 'primary hkl'));
  const hklGrid = el('div', 'lattice-grid');
  ['h', 'k', 'l'].forEach((nm, i) => {
    hklGrid.appendChild(el('span', 'axis', nm));
    hklGrid.appendChild(numberInput(d.primaryHkl[i], '1',
      (v) => { if (!isNaN(v)) d.primaryHkl[i] = v; }));
  });
  hklGrid.style.gridTemplateColumns = '14px 1fr 14px 1fr 14px 1fr';
  sec.appendChild(hklGrid);

  sec.appendChild(fieldRow('energy (keV)', numberInput(d.energy, '0.1',
    (v) => { if (!isNaN(v)) d.energy = v; })));

  // azimuthal reference
  sec.appendChild(el('label', null, 'azimuthal ref'));
  const azGrid = el('div', 'lattice-grid');
  ['x', 'y', 'z'].forEach((nm, i) => {
    azGrid.appendChild(el('span', 'axis', nm));
    azGrid.appendChild(numberInput(d.azir[i], '0.1',
      (v) => { if (!isNaN(v)) d.azir[i] = v; }));
  });
  azGrid.style.gridTemplateColumns = '14px 1fr 14px 1fr 14px 1fr';
  sec.appendChild(azGrid);

  els.dmsDepthRow = fieldRow('secondary depth', numberInput(d.secondaryDepth, '1',
    (v) => { if (!isNaN(v)) d.secondaryDepth = Math.max(1, v | 0); }));
  sec.appendChild(els.dmsDepthRow);

  // How the curves are delivered. A DMS locus is a cone of exit directions, so
  // in Q it is exactly a circle — "circles" sends that (centre/radius/normal/arc)
  // and tessellates it here at display resolution instead of shipping ~34k
  // sampled points. Identical geometry at the same θ-scan steps; what changes is
  // what those steps buy you — see the note under the steps field.
  sec.appendChild(fieldRow('curve method', select([
    ['sweep', 'θ-sweep (sampled)'],
    ['circle', 'circles (analytic)'],
  ], d.method, (v) => { d.method = v; updateMethodNote(); scheduleDmsRecompute(); })));

  sec.appendChild(fieldRow('θ-scan steps', numberInput(d.numsteps, '10',
    (v) => { if (!isNaN(v)) d.numsteps = Math.max(10, v | 0); })));
  els.dmsMethodNote = el('div', 'count-note', '');
  sec.appendChild(els.dmsMethodNote);
  updateMethodNote();

  // θ range (°): how far the primary is swept in θ to trace each DMS line — sets
  // the extent of the curves. Relative to the primary's own Bragg angle, the
  // same convention as the DMS project's `thrange_delta`.
  // Live-recomputes (debounced) once curves exist.
  sec.appendChild(el('label', null, 'θ range (° from θ_B — from, to)'));
  const trGrid = el('div', 'button-grid');
  trGrid.style.cssText = 'grid-template-columns: 1fr 1fr; margin-top: 4px;';
  trGrid.appendChild(numberInput(d.thrange[0], '1',
    (v) => { if (!isNaN(v)) { d.thrange[0] = v; scheduleDmsRecompute(); } }));
  trGrid.appendChild(numberInput(d.thrange[1], '1',
    (v) => { if (!isNaN(v)) { d.thrange[1] = v; scheduleDmsRecompute(); } }));
  sec.appendChild(trGrid);

  // ψ: azimuth about the primary scattering vector (a Renninger scan). It changes
  // which secondaries are in the multiple-scattering condition, so it re-runs the
  // computation — debounced, and only once curves already exist.
  els.dmsPsiSlider = slider('ψ (°) about Q', d, 'psi', -180, 180, d.psiStep,
    scheduleDmsRecompute);
  sec.appendChild(els.dmsPsiSlider);
  // How far one nudge of ψ moves — arrow keys on the slider step by exactly this,
  // which is how you walk a Renninger scan at a chosen resolution (and what sets
  // the spacing of the trail slices).
  sec.appendChild(fieldRow('ψ step (°)', numberInput(d.psiStep, '0.1', (v) => {
    if (isNaN(v) || v <= 0) return;
    d.psiStep = Math.max(0.01, Math.min(90, v));
    if (els.dmsPsiSlider) els.dmsPsiSlider.__range.step = d.psiStep;
    updateTrailRows();     // the slice count a full sweep implies just changed
  })));

  // ψ / χ / θ corrections: the refined offsets carried by a loaded DMS state
  // (guess slots 6 / 7 / 8). ψ shifts the whole pattern; χ rotates the scan-hkl
  // list about the chi axis, changing which secondaries diffract across the
  // sweep (so even a few thousandths of a degree moves the Kossel lines); θ
  // offsets every outgoing Bragg angle. All three go to the engine.
  sec.appendChild(fieldRow('ψ correction (°)', numberInput(d.psicor, '0.01',
    (v) => { if (!isNaN(v)) { d.psicor = v; scheduleDmsRecompute(); } })));
  sec.appendChild(fieldRow('χ correction (°)', numberInput(d.chicor, '0.001',
    (v) => { if (!isNaN(v)) { d.chicor = v; scheduleDmsRecompute(); } })));
  sec.appendChild(fieldRow('θ correction (°)', numberInput(d.thetacor, '0.001',
    (v) => { if (!isNaN(v)) { d.thetacor = v; scheduleDmsRecompute(); } })));

  sec.appendChild(slider('curve opacity', d, 'curveOpacity', 0.1, 1, 0.05, () => refreshDms()));

  // ψ trail: keep each slice as ψ sweeps, tracing the swept surface in 3D.
  // Turning it off stops recording but leaves what is already there to inspect —
  // "clear trail" is the one that discards it.
  els.dmsTrailCb = checkbox('leave a trail as ψ sweeps', d.trail, (v) => {
    d.trail = v;
    if (v && hasCurves()) scheduleDmsRecompute();   // start from where ψ is now
    updateTrailRows();
  });
  sec.appendChild(els.dmsTrailCb);
  // Walks the whole ψ range at the step size, computing at every stop, so the
  // trace comes out evenly spaced. Dragging the slider can't do that: the
  // recompute is debounced and coalesced, so a fast drag only records where it
  // paused. Press again to stop part-way.
  els.dmsSweepBtn = button('Sweep ψ (record every step)', sweepPsi);
  sec.appendChild(els.dmsSweepBtn);
  els.dmsTrailMaxRow = fieldRow('trail length (ψ slices)', numberInput(d.trailMax, '4',
    (v) => {
      if (isNaN(v)) return;
      d.trailMax = Math.max(1, Math.min(1000, v | 0));
      refreshTrail();          // trims immediately if the limit came down
      updateTrailRows();
    }));
  sec.appendChild(els.dmsTrailMaxRow);
  els.dmsTrailClear = button('Clear trail', () => { clearTrail(); updateTrailRows(); });
  sec.appendChild(els.dmsTrailClear);
  els.dmsTrailNote = el('div', 'count-note', '');
  sec.appendChild(els.dmsTrailNote);
  updateTrailRows();

  sec.appendChild(checkbox('show Q-curves', d.showCurves,
    (v) => { d.showCurves = v; refreshDms(); }));
  sec.appendChild(checkbox('show Ewald sphere', d.showEwald,
    (v) => { d.showEwald = v; refreshDms(); }));
  sec.appendChild(checkbox('show k_in, k_out & Q (to primary)', d.showTriangle,
    (v) => { d.showTriangle = v; setTriangleVisible(v); }));
  sec.appendChild(checkbox('project onto panel', d.projectOnPanel,
    (v) => { d.projectOnPanel = v; refreshProjection(); }));
  sec.appendChild(checkbox('show Q recovered from panel', d.showRecovered,
    (v) => { d.showRecovered = v; refreshProjection(); }));
  // Width of the DMS line as the panel records it; carried through to the
  // recovered Q as a band. Display only — no recompute.
  sec.appendChild(slider('line σ (recorded width)', d, 'lineSigma', 0.002, 0.06, 0.002,
    () => refreshProjection()));

  els.dmsStatus = el('div', 'count-note', 'not computed');
  sec.appendChild(button('Compute DMS', () => computeDMS(dmsStatusCb)));
  sec.appendChild(els.dmsStatus);

  // The icosahedral path draws its secondaries from ref_6d, not an hkl box.
  updateDepthRow();
  return sec;
}
