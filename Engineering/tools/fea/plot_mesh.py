"""FEA mesh picture: every link's C3D10 surface mesh placed in the load-case pose, with the
clamp (blue) and load (red) patches; right panel zooms on the elbow (J3 bearings).
Usage: python3 plot_mesh.py [case]"""
import json, sys, numpy as np, pyvista as pv
D = __import__('os').environ.get('FEA_D', __import__('os').path.dirname(__import__('os').path.abspath(__file__))).rstrip('/') + '/'
case = sys.argv[1] if len(sys.argv) > 1 else 'stretch'
I = json.load(open(D + 'fea_%s.json' % case))
NAMES = ['Base column', 'Turret', 'Upper arm', 'Elbow', 'Forearm', 'Wrist']
COL = ['#9aa3ad', '#c9a46a', '#7fa7c9', '#a8c98a', '#c99ab8', '#d9c37a']

def qrot(q, v):
    x, y, z, w = q; u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)

def link(j):
    z = np.load(D + 'run%d_%s_res.npz' % (j, case)); r = json.load(open(D + 'run%d_%s_res.json' % (j, case)))
    base, q = I['links'][str(j)]['link_placement']
    xyz = np.array([qrot(q, p) for p in z['xyz']]) + np.array(base)
    faces = np.hstack([np.full((len(z['tris']), 1), 3), z['tris']]).ravel()
    return pv.PolyData(xyz, faces), xyz[z['fix']], xyz[z['load']], r

pv.global_theme.font.family = 'arial'
pl = pv.Plotter(off_screen=True, shape=(1, 2), window_size=(2600, 1300), border=False)
pl.set_background('white')
meshes = [link(j) for j in range(6)]
lines = []
for col, zoom in ((0, False), (1, True)):
    pl.subplot(0, col)
    for j, (m, fx, ld, r) in enumerate(meshes):
        if zoom and j != 3: continue
        pl.add_mesh(m, color=COL[j], show_edges=True, edge_color='#202020', line_width=0.4 if not zoom else 0.6,
                    lighting=True, smooth_shading=False)
        if len(fx): pl.add_points(fx, color='#1f5fd6', point_size=3 if not zoom else 5, render_points_as_spheres=True)
        if len(ld): pl.add_points(ld, color='#e0302a', point_size=3 if not zoom else 5, render_points_as_spheres=True)
        if col == 0: lines.append('%-12s %7d nodes %7d tets' % (NAMES[j], r['nodes'], r['tets']))
    if zoom:
        e = np.array(meshes[3][0].center)                # elbow alone, from -Y below
        pl.camera_position = [(e[0] - 120, e[1] - 230, e[2] - 90), e, (0, 0, 1)]
        pl.camera.zoom(1.1)
        pl.add_text('Elbow (link 3) alone, from -Y: clamped (blue) on the J3 actuator bore\nand the J3 support-bearing cap face; loaded (red) at the J4 actuator seat', font_size=11, color='black', position='upper_left')
    else:
        b = np.array([m.bounds for m, *_ in meshes]); c = np.array([(b[:, 0].min() + b[:, 1].max()) / 2, 0, (b[:, 4].min() + b[:, 5].max()) / 2])
        pl.camera_position = [(c[0] + 420, c[1] - 820, c[2] + 380), c, (0, 0, 1)]
        pl.camera.zoom(1.05)
        pl.add_text('KLiK3 rev D FEA mesh, %s pose %s\nC3D10 (gmsh, clmax 3 / clmin 0.6 mm)\n' % (case, tuple(int(v) for v in I['q'])) + '\n'.join(lines),
                    font_size=10, color='black', position='upper_left', font='courier')
        pl.add_text('blue: clamp nodes   red: load nodes', font_size=10, color='black', position='lower_left')
pl.screenshot(D + 'fea_mesh_%s.png' % case)
print('ok', D + 'fea_mesh_%s.png' % case)
