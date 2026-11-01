"""Convert the Mecademic MEGP 25E + M500 STAP01 STEP files to meshes (run inside FreeCAD).

The FreeCAD snap cannot read /FastDrive or /tmp, so copy the inputs to its home first:

    W=~/snap/freecad/common/megp_work; mkdir -p $W
    cp "/FastDrive/MEGP_25E_electric_gripper/MEGP 25E.stp" \
       "/FastDrive/MEGP_25E_electric_gripper/M500 STAP01 adapter.stp" tools/megp25e_export.py $W/
    (cd $W && freecad.cmd -c "exec(open('megp25e_export.py').read())")
    cp $W/megp25e_parts.npz models/megp25e/

Output: one entry per solid, in STEP coordinates (mm): '<file>/<name>_v' (N, 3) float32,
'<file>/<name>_f' (M, 3) int32, and '<file>/<name>_label'.
legosort/megp25e.py turns these into the tool frame.
"""
import os

import FreeCAD
import Import
import numpy as np

here = os.getcwd()
out = {}
for f, key in [("MEGP 25E.stp", "gripper"), ("M500 STAP01 adapter.stp", "adapter")]:
    doc = FreeCAD.newDocument()
    Import.insert(os.path.join(here, f), doc.Name)
    doc.recompute()
    for o in doc.Objects:
        if o.TypeId == "Part::Feature" and o.Shape.Solids:
            s = o.Shape.copy()
            s.Placement = o.getGlobalPlacement()
            v, t = s.tessellate(0.05)
            k = f"{key}/{o.Name}"
            out[k + "_v"] = np.array([[p.x, p.y, p.z] for p in v], np.float32)
            out[k + "_f"] = np.array(t, np.int32)
            out[k + "_label"] = np.array(o.Label)
np.savez_compressed(os.path.join(here, "megp25e_parts.npz"), **out)
print("parts", len(out) // 3)
