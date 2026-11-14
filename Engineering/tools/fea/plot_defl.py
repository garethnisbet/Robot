"""Whole-robot deflection picture: each link's FEA displacement chained with everything
upstream of it (rigid motion of the links below, from their load-patch fit, plus the gear
wind-up and bearing tilt of every joint below, from klik3_ik.SagModel). Left: total, right:
link bending only. Deformation exaggerated; ghost = undeformed.
Usage: python3 plot_defl.py [case]"""
import json, sys, numpy as np, pyvista as pv
sys.path.insert(0, '/FastDrive/Dropbox/ClaudeCodeProjects/Engineering')
import klik3_ik as K
D = __import__('os').environ.get('FEA_D', __import__('os').path.dirname(__import__('os').path.abspath(__file__))).rstrip('/') + '/'
case = sys.argv[1] if len(sys.argv) > 1 else 'stretch'
I = json.load(open(D + 'fea_%s.json' % case)); B = json.load(open(D + 'budget.json'))[case]
_k = json.load(open(D + 'kin.json'))          # written from FreeCAD with the export (k3 constants)
lims = [tuple(l) for l in _k['LIMITS']]
sag = K.SagModel(K.KLiK3Kinematics(_k['J2Z'], _k['L2'], _k['J4X'], _k['J4Z'], _k['J5X'], _k['WRIST'], lims, J2X=_k['J2X'], TOOL=_k['TOOL']))

def qmat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

# joint compliance rotations (global), applied to everything outboard of the joint
W = []
for k, (T, Mt, ax, p) in enumerate(sag.joint_loads(I['q'])):
    act = sag.acts[k]
    w = np.array(ax) * K.windup(T, K.GEAR_SIZE[act]) + np.array(Mt) / (K.TILT_STIFFNESS[act] + K.SUPPORT_TILT.get(k + 1, 0.0))
    W.append((w, np.array(p)))
links = []
for j in range(6):
    z = np.load(D + 'run%d_%s_res.npz' % (j, case)); r = json.load(open(D + 'run%d_%s_res.json' % (j, case)))
    base, q = I['links'][str(j)]['link_placement']; R = qmat(q); base = np.array(base)
    links.append(dict(xyz=z['xyz'] @ R.T + base, U=z['U'] @ R.T, tris=z['tris'], R=R,
                      t=R @ np.array(r['ref_t_um']) / 1000, th=R @ np.array(r['ref_rot_urad']) * 1e-6,
                      ref=R @ np.array(I['links'][str(j)]['ref_local']) + base))
tip = np.array(I['tip_global'])

def field(j, joints=True):
    L = links[j]; x = L['xyz']; u = L['U'].copy()
    for i in range(j):                                   # rigid motion of the links below
        u += links[i]['t'] + np.cross(links[i]['th'], x - links[i]['ref'])
    if joints:
        for k in range(j):                               # joints 1..j sit below link j
            w, p = W[k]; u += np.cross(w, x - p)
    return u

def tipdefl(joints):
    u = sum(links[i]['t'] + np.cross(links[i]['th'], tip - links[i]['ref']) for i in range(6))
    if joints: u = u + sum(np.cross(w, tip - p) for w, p in W)
    return u

pl = pv.Plotter(off_screen=True, shape=(1, 2), window_size=(2600, 1250), border=False)
pl.set_background('white')
allp = np.concatenate([L['xyz'] for L in links]); c = (allp.min(0) + allp.max(0)) / 2 + np.array([35, 0, 0])
for col, (joints, ttl) in enumerate(((True, 'Total: link bending (FEA) + joint gear wind-up and bearing tilt'),
                                     (False, 'Link bending only (FEA)'))):
    pl.subplot(0, col)
    fields = [field(j, joints) for j in range(6)]
    umax = max(np.linalg.norm(f, axis=1).max() for f in fields) * 1000
    dt = tipdefl(joints) * 1000
    raw = 60 / (np.linalg.norm(dt) / 1000); scale = int(min((10, 20, 30, 50, 100, 200, 300, 500, 1000, 2000, 3000, 5000), key=lambda v: abs(np.log(v / raw))))
    for j, L in enumerate(links):
        faces = np.hstack([np.full((len(L['tris']), 1), 3), L['tris']]).ravel()
        pl.add_mesh(pv.PolyData(L['xyz'], faces), color='#bbbbbb', opacity=0.18)
        m = pv.PolyData(L['xyz'] + scale * fields[j], faces); m['|u| (um)'] = np.linalg.norm(fields[j], axis=1) * 1000
        pl.add_mesh(m, scalars='|u| (um)', cmap='turbo', clim=(0, umax), smooth_shading=True,
                    scalar_bar_args=dict(title='|u| (um)' + ('' if col == 0 else ' '), vertical=True, height=0.6, position_x=0.88, position_y=0.2,
                                         title_font_size=18, label_font_size=16, color='black', fmt='%.0f'))
    pl.add_points(np.array([tip, tip + scale * dt / 1000]), color='magenta', point_size=14, render_points_as_spheres=True)
    pl.add_text('%s\nx%d exaggeration; flange moves %.0f um down, %.0f um along x (|%.0f| um)' % (
        ttl, scale, -dt[2], dt[0], np.linalg.norm(dt)), font_size=11, color='black', position='upper_left')
    pl.camera_position = [(c[0] + 60, c[1] - 900, c[2] + 40), c, (0, 0, 1)]
    pl.camera.parallel_projection = True
    pl.camera.parallel_scale = 275
pl.screenshot(D + 'fea_defl_%s.png' % case)
print('ok', D + 'fea_defl_%s.png' % case, 'tip total um', np.round(tipdefl(True) * 1000, 1), 'links um', np.round(tipdefl(False) * 1000, 1),
      'budget um', np.round(B['total_um'], 1))
