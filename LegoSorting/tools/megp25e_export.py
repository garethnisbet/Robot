"""Mesh the MEGP 25E gripper (and the M500 STAP01 adapter) for legosort (run in FreeCAD).

    exec(open("/FastDrive/Dropbox/ClaudeCodeProjects/LegoSorting/tools/megp25e_export.py").read())

The gripper's solids come from the open document made by tools/megp25e_fingers.py
(MEGP_fingers2, or any open document holding the STEP's Part__Feature solids), else from
the vendor STEP file. Each solid is meshed with FreeCAD's mesher at a fixed deflection
(LINEAR mm, ANGULAR rad): about 55k triangles for the whole gripper, clean enough to draw
as is (no decimation, which distorted the shapes).

Output models/megp25e/megp25e_parts.npz: one entry per solid, in STEP coordinates (mm):
'<file>/<name>_v' (N, 3) float32, '<file>/<name>_f' (M, 3) int32, '<file>/<name>_label'.
legosort/megp25e.py turns these into the tool frame.
"""
import os

import FreeCAD
import Import
import MeshPart
import numpy as np

PROJECT = "/FastDrive/Dropbox/ClaudeCodeProjects/LegoSorting"
SRC = "/FastDrive/MEGP_25E_electric_gripper"
DOC = "MEGP_fingers2"
LINEAR, ANGULAR = 0.02, 0.26


def solids(doc):
    for o in doc.Objects:
        if o.TypeId == "Part::Feature" and o.Name.startswith("Part__Feature") and o.Shape.Solids:
            s = o.Shape.copy()
            s.Placement = o.getGlobalPlacement()
            yield o, s


def from_step(path):
    doc = FreeCAD.newDocument()
    Import.insert(path, doc.Name)
    doc.recompute()
    return doc


out = {}
docs = FreeCAD.listDocuments()
sources = [("gripper", docs[DOC] if DOC in docs else from_step(os.path.join(SRC, "MEGP 25E.stp"))),
           ("adapter", from_step(os.path.join(SRC, "M500 STAP01 adapter.stp")))]
for key, doc in sources:
    for o, s in solids(doc):
        m = MeshPart.meshFromShape(Shape=s, LinearDeflection=LINEAR, AngularDeflection=ANGULAR, Relative=False)
        k = f"{key}/{o.Name}"
        out[k + "_v"] = np.array([[p.x, p.y, p.z] for p in m.Points], np.float32)
        out[k + "_f"] = np.array([f.PointIndices for f in m.Facets], np.int32)
        out[k + "_label"] = np.array(o.Label)
    if key == "adapter":
        FreeCAD.closeDocument(doc.Name)
path = os.path.join(PROJECT, "models", "megp25e", "megp25e_parts.npz")
np.savez_compressed(path, **out)
print("parts", len(out) // 3, "triangles", sum(len(v) for k, v in out.items() if k.endswith("_f")), "->", path)
