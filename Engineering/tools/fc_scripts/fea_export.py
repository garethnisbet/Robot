"""FEA export: per structural link, the fused solid (STEP + BREP, link frame), the interface
shapes it is clamped / loaded through (BREP, link frame; actuator parts + support-bearing races),
and the loads for each pose case with a payload on the flange face.
Writes <OUT>/link<j>.{step,brep}, link<j>_fix.brep, link<j>_load.brep and fea_<case>.json.
Globals: OUT, PAYLOAD (kg), CASES {name: q (deg)}."""
import FreeCAD as App, Part, re, json, os
import actuator_fit as af, klik3_freecad as k3
OUT = globals().get('OUT', '/tmp/fea')
PAYLOAD = globals().get('PAYLOAD', 0.5)
CASES = globals().get('CASES', {'stretch': [0, 90, -90, 0, 0, 0], 'home': [0, 0, 0, 0, 0, 0]})
os.makedirs(OUT, exist_ok=True)
d = App.getDocument('KLiK3'); sh = d.getObject('Joints')
old = [sh.get('B%d' % i) for i in range(2, 8)]

def link_of(o):
    g = o.getParentGeoFeatureGroup()
    while g is not None and not re.match(r'Link\d$|KLiK3$', g.Name): g = g.getParentGeoFeatureGroup()
    return None if g is None else (0 if g.Name == 'KLiK3' else int(g.Name[4:]))

def have(*names): return [n for n in names if d.getObject(n)]
STRUCT = {0: ['Base', 'Base_plate'], 1: ['Turret', 'Turret_cheeks'], 2: ['UpperArm_L', 'UpperArm_R'],
          3: ['Elbow', 'Elbow_lid'], 4: ['Forearm', 'Forearm_collar'], 5: ['Wrist', 'Wrist_nose']}
# interfaces: actuator parts + support-bearing races; J3/J5 support stubs are on caps bolted to the elbow / wrist
FIX = {0: [], 1: have('J1_output', 'J1_support_outer'), 2: have('J2_actuator', 'J2_support_inner'), 3: have('J3_actuator', 'J3_cap'),
       4: ['J4_output'], 5: have('J5_actuator', 'J5_cap')}
LOAD = {0: have('J1_actuator', 'J1_support_inner'), 1: have('J2_output', 'J2_support_outer'), 2: have('J3_output', 'J3_support_outer'),
        3: ['J4_actuator'], 4: have('J5_output', 'J5_support_outer'), 5: ['J6_actuator']}
AXIS = {1: (0, 0, 1), 2: (0, 1, 0), 3: (0, 1, 0), 4: (1, 0, 0), 5: (0, 1, 0), 6: (1, 0, 0)}
ACT = {'J%d_actuator' % j: k3.ACTUATORS[j - 1][0] for j in range(1, 7)}
STEEL = re.compile('screw|bolt|dowel|support', re.I)

def lframe(j):
    return App.Placement(App.Vector(0, 0, k3.WORLD_Z), App.Rotation()) if j == 0 else d.getObject('Link%d' % j).getGlobalPlacement()

def setq(q):
    for i, v in enumerate(q, 2): sh.set('B%d' % i, str(v))
    d.recompute()

def mass_com(o):
    s = o.Shape.copy(); s.Placement = o.getGlobalPlacement()
    if o.Name in ACT: m = af.MASS[ACT[o.Name]]
    elif o.Name.endswith('_output'): return 0, s.BoundBox.Center      # rotor: in the catalogue mass
    else: m = s.Volume * (7.85 if STEEL.search(o.Name) else 2.7) * 1e-6
    sols = s.Solids                                       # Shape.CenterOfMass of a multi-solid compound is only its first solid's
    c = sum((so.CenterOfMass * so.Volume for so in sols), App.Vector()) * (1.0 / sum(so.Volume for so in sols)) if sols else s.BoundBox.Center
    return m, c

def local(o, j):
    s = o.Shape.copy(); s.Placement = lframe(j).inverse().multiply(o.getGlobalPlacement()); return s

# geometry (pose independent, link frames)
setq(CASES[list(CASES)[0]])
for j in range(6):
    sol = [local(d.getObject(n), j) for n in STRUCT[j]]
    fused = sol[0].fuse(sol[1:]).removeSplitter() if len(sol) > 1 else sol[0]
    fused.exportStep(os.path.join(OUT, 'link%d.step' % j)); fused.exportBrep(os.path.join(OUT, 'link%d.brep' % j))
    for key, nms in (('fix', FIX[j]), ('load', LOAD[j])):
        if nms: Part.makeCompound([local(d.getObject(n), j) for n in nms]).exportBrep(os.path.join(OUT, 'link%d_%s.brep' % (j, key)))

fl = d.getObject('Flange'); FACE_X = fl.Shape.BoundBox.XMax                 # flange face along Link6 x
for case, Q in CASES.items():
    setq(Q)
    items = []
    for o in d.Objects:
        if not o.TypeId.startswith('Part::') or o.Shape.isNull() or o.Name.startswith('Meca'): continue
        l = link_of(o)
        if l is None: continue
        m, c = mass_com(o)
        if m > 0: items.append((l, o.Name, m, c))
    tip = d.getObject('Link6').getGlobalPlacement().multVec(App.Vector(FACE_X, 0, 0))
    info = {'q': Q, 'payload_kg': PAYLOAD, 'tip_global': list(tip), 'links': {}, 'joints': {}}
    for j in range(6):
        inv = lframe(j).inverse(); R = lframe(j).Rotation.inverted()
        fused_vol = sum(d.getObject(n).Shape.Volume for n in STRUCT[j])
        struct_kg = fused_vol * 2.7e-6
        own = [(n, m, c) for l, n, m, c in items if l == j]
        skip = set(STRUCT[j]) | set(FIX[j]) | ({'J%d_actuator' % j} if j else set())
        out = [(n, m, c) for l, n, m, c in items if l > j]
        moved = [(n, m, c) for n, m, c in own if n in LOAD[j]]           # parts this link carries at the next joint
        out += moved
        own_extra = sum(m for n, m, c in own if n not in skip and n not in LOAD[j])
        out.append(('payload', PAYLOAD, tip))
        M = sum(m for n, m, c in out)
        com = sum((c * m for n, m, c in out), App.Vector()) * (1.0 / M)
        ref = lframe(j + 1).Base if j < 5 else d.getObject('Link6').getGlobalPlacement().Base
        F = App.Vector(0, 0, -9.81 * M)
        Mom = (com - ref).cross(F)
        info['links'][j] = {
            'parts': STRUCT[j], 'struct_kg': struct_kg, 'extra_kg': own_extra,
            'density_factor': (struct_kg + own_extra) / struct_kg,
            'gravity_local': list(R.multVec(App.Vector(0, 0, -9810.0))),
            'ref_local': list(inv.multVec(ref)), 'F_local_N': list(R.multVec(F)), 'M_local_Nmm': list(R.multVec(Mom)),
            'outboard_kg': M, 'outboard': [(n, round(m, 3)) for n, m, c in out],
            'tip_local': list(inv.multVec(tip)), 'ref_global': list(ref),
            'link_placement': [list(lframe(j).Base), list(lframe(j).Rotation.Q)],
        }
    # joint loads at each actuator's output face (the bearing): everything beyond the joint
    for k in range(1, 7):
        # everything in links >= k passes through joint k's output bearing (a stator housed in the
        # outer link - J2, J3, J5 - included; J1/J4/J6 stators sit in the inner link, so l < k drops them)
        out = [(n, m, c) for l, n, m, c in items if l >= k] + [('payload', PAYLOAD, tip)]
        o = d.getObject('J%d_output' % k); s = o.Shape.copy(); s.Placement = o.getGlobalPlacement()
        P = s.CenterOfMass
        M = App.Vector()
        for n, m, c in out: M = M + (c - P).cross(App.Vector(0, 0, -9.81 * m))
        ax = lframe(k).Rotation.multVec(App.Vector(*AXIS[k]))
        info['joints'][k] = {'actuator': k3.ACTUATORS[k - 1][0], 'bearing_point': list(P), 'axis': list(ax),
                             'M_Nmm': list(M), 'torque_Nm': M.dot(ax) / 1000, 'tilt_Nm': (M - ax * M.dot(ax)).Length / 1000}
        sup = d.getObject('J%d_support_inner' % k)
        if k in (2, 3, 5) and sup:
            # far-side crossed-roller bearing: idealised as two radial supports a span s apart, the
            # tilt moment goes into a radial couple (actuator bearing + support), axial on the actuator
            ss = sup.Shape.copy(); ss.Placement = sup.getGlobalPlacement(); Ps = ss.BoundBox.Center
            s = (Ps - P).dot(ax); Fg = App.Vector(0, 0, -9.81 * sum(m for n, m, c in out))
            Mt = M - ax * M.dot(ax); Fp = Fg - ax * Fg.dot(ax)
            Rs = ax.cross(Mt) * (1.0 / s); Ra = Fp * -1 - Rs            # reactions: s ax x Rs = -Mt, Ra + Rs = -Fp
            info['joints'][k].update(span_mm=abs(s), support_radial_N=Rs.Length, actuator_radial_N=Ra.Length,
                                     axial_N=abs(Fg.dot(ax)), radial_no_support_N=Fp.Length)
    json.dump(info, open(os.path.join(OUT, 'fea_%s.json' % case), 'w'), indent=1, default=float)
    print(case, 'tip', [round(v, 1) for v in tip])
    for k, J in info['joints'].items(): print('  J%d %-8s torque %6.2f  tilt %5.2f N.m' % (k, J['actuator'], J['torque_Nm'], J['tilt_Nm']),
                                              ('  span %.1f: radial actuator %.0f N, support %.0f N (no support: %.0f N + tilt)' % (J['span_mm'], J['actuator_radial_N'], J['support_radial_N'], J['radial_no_support_N'])) if 'span_mm' in J else '')
    for j, v in info['links'].items(): print('  link%d struct %.3f extra %.3f outboard %.3f' % (j, v['struct_kg'], v['extra_kg'], v['outboard_kg']))
setq(old)
