import numpy as np, json, sys, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import cm, colors
D = __import__('os').environ.get('FEA_D', __import__('os').path.dirname(__import__('os').path.abspath(__file__))).rstrip('/') + '/'
NAMES = {0: 'Base column', 1: 'Turret', 2: 'Upper arm', 3: 'Elbow', 4: 'Forearm', 5: 'Wrist'}
VIEW = {0: (25, -60), 1: (15, 55), 2: (12, 60), 3: (25, 55), 4: (25, -60), 5: (25, -60)}
fig = plt.figure(figsize=(15, 9.6), dpi=130)
for k, j in enumerate(range(6)):
    z = np.load(D + 'run%d_stretch_res.npz' % j); r = json.load(open(D + 'run%d_stretch_res.json' % j))
    xyz, vm, tris, U = z['xyz'], z['vm'], z['tris'], z['U']
    ax = fig.add_subplot(2, 3, k + 1, projection='3d')
    vmax = max(np.percentile(vm, 99.5), 1e-3)
    tv = vm[tris].mean(1)
    norm = colors.Normalize(0, vmax)
    pc = Poly3DCollection(xyz[tris], facecolors=cm.turbo(norm(tv)), edgecolor='none', linewidth=0)
    ax.add_collection3d(pc)
    lo, hi = xyz.min(0), xyz.max(0); c = (lo + hi) / 2; s = (hi - lo).max() / 2
    ax.set_xlim(c[0] - s, c[0] + s); ax.set_ylim(c[1] - s, c[1] + s); ax.set_zlim(c[2] - s, c[2] + s)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(*VIEW[j]); ax.set_axis_off()
    m = np.array(r['vm_far_at']); ax.scatter(*m, s=160, facecolors='none', edgecolors='magenta', linewidths=2.2, depthshade=False, zorder=10)
    ax.set_title('%s: peak %.1f MPa (circled)\nLC2 peak %.0f MPa, yield 276 MPa' % (NAMES[j], r['vm_far'], 3 * r['vm_far']), fontsize=10.5)
    sm = cm.ScalarMappable(norm=norm, cmap='turbo'); sm.set_array([])
    fig.colorbar(sm, ax=ax, shrink=0.55, pad=0.0, label='MPa')
fig.suptitle('Von Mises stress, rev D (2026-11-14 design), LC1: arm horizontal, 0.5 kg payload, 1 g (colour scale to the 99.5th percentile of each link)', fontsize=12)
fig.tight_layout()
fig.savefig(D + 'fea_stress_stretch.png')
print('ok')
