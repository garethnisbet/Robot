# KLiK3 link FEA

Report, revision D (2026-11-14, current design): https://claude.ai/artifact/Pz6eCY6LxhhkDECQq2WCtK - results/figures in revD/.
Revision C (2026-10-25, crossed-roller supports at J2/J3/J5): https://claude.ai/artifact/1r3Q3As3XEME6Ek3Z9rjSn.
Revision A and B scripts/results in revA/ and revB/.

Working dir: every script reads the env var FEA_D (fea.py, budget.py, plot_*.py) or the global P (sample.py, run in
FreeCAD). Meshes/decks/frd are big: keep them in a scratch dir and copy only *_res.json, fea_*.json, kin.json,
budget.json and the PNGs into revX/. kin.json (k3 constants + LIMITS + TOOL, written from FreeCAD with k3.kinematics())
replaces the kinematics budget.py/plot_defl.py used to hard-code. Re-run tools/fc_scripts/mass_props.py first so
klik3_mass.json (sag model) matches the model.
Turret mesh (rev D): the exported link1.brep was invalid; Part shape.fix(1e-4,1e-5,1e-3) -> link1_fixed.brep meshed
(gmsh healing flags/sewing either failed or dropped the volume).

Run order (set FEA_D first):
1. FreeCAD: `tools/fc_scripts/fea_export.py` (globals OUT, PAYLOAD, CASES) -> link<j>.step/.brep, link<j>_fix/load.brep
   (actuator parts + support-bearing races), fea_<case>.json (link loads, joint torque/tilt at each output bearing).
   Mesh from BREP: the upper arm STEP re-reads broken.
2. FreeCAD: `sample.py` -> point clouds of the interface shapes.
3. gmsh (FreeCAD AppImage, LD_LIBRARY_PATH=<mount>/usr/lib):
   `gmsh link<j>.brep -3 -order 2 -clmax 3 -clmin 0.6 -optimize_netgen -setnumber Mesh.SecondOrderLinear 1 -format inp -o link<j>.inp`
   (do not fall back to STEP for the turret: gmsh segfaults on it).
4. `python3 fea.py <case> 0 1 2 3 4 5` (CalculiX; pass ccx a relative deck name - long paths >127 chars fail).
5. `python3 budget.py stretch home` (deflection at the flange face; rev B used the J6 origin) (link FEA + klik3_ik.SagModel joint compliance), `plot_s.py`, `plot_robot.py home`.
Masses for the sag model: `tools/fc_scripts/mass_props.py` -> klik3_mass.json.
6. Pictures: `plot_mesh.py stretch` (mesh + clamp/load nodes), `plot_defl.py stretch` (chained whole-robot deflection,
   total and links only; needs budget.json), `plot_crb_sections.py` (needs crb_sec.json from the scratch FreeCAD
   slicer), all PyVista/matplotlib off-screen.
Support bearings at J2/J3/J5: fea_export splits each joint's tilt into a radial couple (span_mm, *_radial_N in
fea_<case>.json); klik3_ik.SUPPORT_TILT adds the couple stiffness (assumed 100 N/um radial per bearing).
