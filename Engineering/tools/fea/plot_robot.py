"""Whole-robot stress picture for one case: every link's FEA surface placed in its pose,
one shared von Mises scale; deformation exaggerated."""
import json, sys, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import cm, colors
D = __import__('os').environ.get('FEA_D', __import__('os').path.dirname(__import__('os').path.abspath(__file__))).rstrip('/') + '/'
case = sys.argv[1] if len(sys.argv) > 1 else 'home'
I = json.load(open(D + 'fea_%s.json' % case)); B = json.load(open(D + 'budget.json'))[case]
def qrot(q, v):
    x, y, z, w = q; u = np.array([x, y, z]); return v + 2 * np.cross(u, np.cross(u, v) + w * v)
P, C, allv = [], [], []
for j in range(6):
    z = np.load(D + 'run%d_%s_res.npz' % (j, case))
    base, q = I['links'][str(j)]['link_placement']
    xyz = np.array([qrot(q, p) for p in z['xyz']]) + np.array(base)
    P.append(xyz[z['tris']]); C.append(z['vm'][z['tris']].mean(1)); allv.append(z['vm'])
vmax = np.percentile(np.concatenate(allv), 99.7)
norm = colors.Normalize(0, vmax)
fig = plt.figure(figsize=(12, 6.6), dpi=140)
for k, (el, az, ttl) in enumerate(((18, -62, 'Iso view'), (0, -90, 'Side view (from -Y)'))):
    ax = fig.add_subplot(1, 2, k + 1, projection='3d')
    for tri, c in zip(P, C):
        ax.add_collection3d(Poly3DCollection(tri, facecolors=cm.turbo(norm(c)), edgecolor='none'))
    allp = np.concatenate([t.reshape(-1, 3) for t in P]); lo, hi = allp.min(0), allp.max(0); c0 = (lo + hi) / 2; s = (hi - lo).max() / 2
    ax.set_xlim(c0[0] - s, c0[0] + s); ax.set_ylim(c0[1] - s, c0[1] + s); ax.set_zlim(c0[2] - s, c0[2] + s)
    ax.set_box_aspect((1, 1, 1), zoom=1.45); ax.view_init(el, az); ax.set_axis_off(); ax.set_title(ttl, fontsize=11, y=0.97)
    tip = np.array(I['tip_global']); ax.scatter(*tip, s=60, c='magenta', depthshade=False)
peak = max(B['rows'], key=lambda r: r.get('vm', 0))
sm = cm.ScalarMappable(norm=norm, cmap='turbo'); sm.set_array([])
fig.subplots_adjust(left=0, right=0.88, top=0.9, bottom=0, wspace=0); cax = fig.add_axes([0.9, 0.2, 0.015, 0.6]); fig.colorbar(sm, cax=cax, label='von Mises (MPa), 1 g')
fig.suptitle('Whole robot, %s pose %s, 0.5 kg on the flange (magenta): peak %.1f MPa in the %s; flange sags %.2f mm' % (
    case, tuple(int(v) for v in I['q']), peak['vm'], peak['item'].split(' bending')[0].lower(), np.linalg.norm(B['total_um']) / 1000), fontsize=11, y=0.97)
fig.savefig(D + 'robot_%s.png' % case, bbox_inches='tight'); print('ok')
