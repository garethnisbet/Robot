"""Sections through the J2, J3 and J5 axes (horizontal plane at home), from crb_sec.json
(scratchpad crb_sec.py in FreeCAD). Support crossed-roller bearings in orange."""
import json, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch
S = '/tmp/claude-1000/-FastDrive-Dropbox-ClaudeCodeProjects-Engineering/4db86799-062d-4db7-8266-a6b187658b9c/scratchpad/'
D = json.load(open(S + 'crb_sec.json'))
TITLE = {'J2': 'J2: FHA-11C output on the +Y cheek,\nCRBT205A (20×31×5) in the −Y cheek',
         'J3': 'J3: RSA-8A output on the +Y fork,\nCRBT205A on the elbow cap in the −Y fork',
         'J5': 'J5: RSA-8A output on the −Y wall,\nCRBT155A (15×26×5) on the J5 cap hub, +Y wall'}
fig, axs = plt.subplots(1, 3, figsize=(16, 6.2), dpi=130)
for ax, (k, v) in zip(axs, D.items()):
    cx = v['c'][0]
    for p in v['polys']:
        verts, codes = [], []
        for L in p['loops']:
            pts = [(y, x - cx) for x, y in L]
            verts += pts + [pts[0]]; codes += [Path.MOVETO] + [Path.LINETO] * (len(pts) - 1) + [Path.CLOSEPOLY]
        sup = 'support' in p['name']
        crb = 'CRB' in p['name'] or p['name'].endswith(('_output',))
        col = '#f28c28' if sup else ('#3a3f47' if 'actuator' in p['name'] else p['col'])
        ax.add_patch(PathPatch(Path(verts, codes), facecolor=col, edgecolor='#222', lw=0.4, alpha=1.0 if sup else 0.85))
    ax.axhline(0, color='#c00', lw=0.6, ls='-.')
    ax.set_xlim(-50, 50); ax.set_ylim(-45, 45); ax.set_aspect('equal'); ax.set_xticks([-45, -30, 0, 30, 45]); ax.tick_params(labelsize=8)
    ax.set_xlabel('y (mm), joint axis along y', fontsize=9); ax.set_title(TITLE[k], fontsize=10.5)
axs[0].set_ylabel('x from the joint axis (mm)', fontsize=9)
fig.suptitle('Rev C: crossed-roller support bearings (orange) opposite each parallel-axis actuator (dark grey); horizontal sections through the axes at home', fontsize=11.5)
fig.tight_layout(); fig.savefig(S + 'fea3/crb_sections.png'); print('ok')
