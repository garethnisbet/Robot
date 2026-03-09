# RobotVisualisation – Development Notes

## Status
- Kinematics confirmed correct by user
- STL files load and display, but **positions/orientations are wrong** at zero config
- Reference images in `Images/FrontView.png` and `Images/SideView.png` show the correct assembly

## Goal
Position STL files to match the correct Meca500 R3 zero-config shape (L-shaped arm):
- Column goes up (z=0 → 135mm in DH world)
- Arm continues up (z=135 → 308mm)
- Then extends horizontally in -x direction to x=-190mm

## STL Map (main.py)
```python
MECA500_STL_MAP = [
    ('A0.stl',   0),  # base/column
    ('A1.stl',   1),  # shoulder arm (after joint 1)
    ('A2.stl',   2),  # upper arm (after joint 2)
    ('A3_4.stl', 3),  # forearm+wrist housing (after joint 3)
    ('A5.stl',   5),  # wrist (after joint 5)
    ('A6.stl',   6),  # end flange
]
```

## DH Frames at Zero Config (in mm)
- F0: (0, 0, 0)       R=I
- F1: (0, 0, 135)     R=[[1,0,0],[0,0,1],[0,-1,0]]
- F2: (0, 0, 270)     R=[[0,1,0],[0,0,1],[1,0,0]]
- F3: (0, 0, 308)     R=[[0,0,-1],[0,1,0],[1,0,0]]
- F4: (-120, 0, 308)  R=[[0,1,0],[0,0,1],[1,0,0]]
- F5: (-120, 0, 308)  R=[[0,0,-1],[0,1,0],[1,0,0]]
- F6: (-190, 0, 308)  R=[[0,0,-1],[0,1,0],[1,0,0]]

## STL Bounding Boxes (in mm, raw CAD coordinates)
- A0: x[-45,45]  y[-90,3]   z[-86,45]   center=(0,-43.5,-20.5)
- A1: x[-5,80]   y[170,271] z[93,169]   center=(37.8,220.7,131.1)
- A2: x[-244,-53] y[-149,-59] z[119,220] center=(-148.1,-104.1,169.2)
- A3_4: x[-186,-86] y[-36,54] z[2,70]   center=(-135.8,9.3,35.8)
- A5: x[-147,-66] y[-60,-15] z[-89,-19]  center=(-106.7,-37.9,-54.1)
- A6: x[41,133]  y[9,54]    z[133,199]  center=(87.3,31.7,165.9)

## Key Findings from STL Analysis
- **STL files are in different CAD coordinate frames** (not a single global frame)
- **A0**: column axis = CAD z-axis (height 131mm in z ≈ d1=135mm ✓)
  - Column body top centroid (z>35, y>-70): (-0.22, 0.26, 38.34)mm
  - This should map to DH frame 1 at (0, 0, 135mm) → z-offset = 96.7mm
- **A1**: arm direction = CAD y-axis (span 101mm in y, from y=170 to y=271)
  - Joint 2 face center (y>265): (37.8, 270.4, 131.4)mm → DH (0, 0, 270mm)
  - Suggests: CAD-y → DH-z for A1 (rotation Rx(90°) needed)
  - Joint 2 bearing center in x-z: (37.8, 131.4) → needs offset to (0, 0) in DH
- **A2**: largest dimension = x (191mm), likely CAD-x maps to something
- **A3_4, A5**: x-axis dominant
- **A6**: x-axis dominant

## Current Code: visualiser.py
`_build_gui` (line 310-326):
```python
T_correction = np.linalg.inv(T_zero[frame_idx])
self._mesh_items.append((item, frame_idx, T_correction))
```
`_draw_meshes` (line 538-547):
```python
T = transforms[frame_idx] @ T_correction
```
At zero config: `T = I` (STL displayed at raw CAD position). This is wrong.

## Proposed Fix
Modify STL map to support 3-tuples `(path, frame_idx, T_local)`:
- `T_display = T_dh_frame(q) @ T_local`
- `T_local = inv(T_zero[frame_idx]) @ T_world_stl_zero`

## Pending Work
Need to determine correct T_local for each STL. Two approaches tried:
1. **Analytical**: Identify joint bearing centers in STL geometry, map to DH world positions
   - Difficult because STL parts overlap at joints and bearing centers are inside geometry
   - A0 and A1+ appear to be in different CAD coordinate frames
2. **Empirical**: Run code, compare visually to reference images, iterate

**Next step**: Implement T_local support in visualiser.py and main.py, then tune values
visually by comparing to Images/FrontView.png and Images/SideView.png.

## Computed Feature Points (mm, raw CAD)
- A1 joint-2 face center (y>265): (37.8, 270.4, 131.4) → DH (0, 0, 270)
- A1 joint-1 face center (y<180, -y normal): (37.8, 171.4, 127.5)
- A1 joint-1 bearing center (x-z, vertices y<178): (37.9, 127.3)
- A1 joint-2 bearing center (x-z, vertices y>265): (37.8, 131.4)
