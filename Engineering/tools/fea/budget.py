"""Flange deflection budget per case: link bending (FEA, this folder's run<j>_<case>_res.json)
+ joint compliance from klik3_ik.SagModel (gear wind-up + output-bearing tilt, support bearings)."""
import json, sys, math, numpy as np
sys.path.insert(0, '/FastDrive/Dropbox/ClaudeCodeProjects/Engineering')
import klik3_ik as K
D = __import__('os').environ.get('FEA_D', __import__('os').path.dirname(__import__('os').path.abspath(__file__))).rstrip('/') + '/'
NAMES = ['Base column', 'Turret', 'Upper arm', 'Elbow', 'Forearm', 'Wrist']
_k = json.load(open(D + 'kin.json'))          # written from FreeCAD with the export (k3 constants)
lims = [tuple(l) for l in _k['LIMITS']]
kin = K.KLiK3Kinematics(_k['J2Z'], _k['L2'], _k['J4X'], _k['J4Z'], _k['J5X'], _k['WRIST'], lims, J2X=_k['J2X'], TOOL=_k['TOOL'])
sag = K.SagModel(kin)
def qrot(q, v):
    x, y, z, w = q; u = np.array([x, y, z]); return v + 2 * np.cross(u, np.cross(u, v) + w * v)
res = {}
for case in sys.argv[1:] or ['stretch', 'home']:
    I = json.load(open(D + 'fea_%s.json' % case)); q = I['q']
    rows = []
    for j in range(6):
        r = json.load(open(D + 'run%d_%s_res.json' % (j, case)))
        dg = qrot(I['links'][str(j)]['link_placement'][1], np.array(r['tip_local_um']))
        rows.append(dict(item=NAMES[j] + ' bending (FEA)', kind='link', um=dg.tolist(), vm=r['vm_far'], vm_at=r['vm_far_at'], nodes=r['nodes']))
    pf = np.array(I['tip_global'])                       # flange face (rev B used the J6 frame origin, 11 mm short)
    for k, (T, Mt, ax, p) in enumerate(sag.joint_loads(q)):
        act = sag.acts[k]
        wu = np.array(ax) * K.windup(T, K.GEAR_SIZE[act]); kt = K.TILT_STIFFNESS[act] + K.SUPPORT_TILT.get(k + 1, 0.0)
        wt = np.array(Mt) / kt
        for kind, w, lab in (('gear', wu, 'J%d gear wind-up (%s)' % (k + 1, act)), ('bearing', wt, 'J%d bearing tilt (%s)' % (k + 1, act))):
            d = np.cross(w, pf - np.array(p)) * 1000
            rows.append(dict(item=lab, kind=kind, um=d.tolist(), torque=T, tilt=float(np.linalg.norm(Mt))))
    tot = np.sum([r['um'] for r in rows], 0)
    res[case] = dict(q=q, rows=rows, total_um=tot.tolist(),
                     joints=[dict(k=k + 1, act=sag.acts[k], torque=T, tilt=float(np.linalg.norm(Mt))) for k, (T, Mt, ax, p) in enumerate(sag.joint_loads(q))])
    print(case, 'total um', np.round(tot, 1), '|%.1f|' % np.linalg.norm(tot))
    for r in sorted(rows, key=lambda r: -np.linalg.norm(r['um'])):
        if np.linalg.norm(r['um']) > 0.5: print('   %-32s %s  |%.1f|' % (r['item'], np.round(r['um'], 1), np.linalg.norm(r['um'])), ('vm %.2f MPa' % r['vm']) if 'vm' in r else '')
json.dump(res, open(D + 'budget.json', 'w'), indent=1)
