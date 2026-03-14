# Build Three.js Robot Viewer from Blender Scene

Build an interactive Three.js FK/IK viewer from the robot armature in the current Blender scene. The viewer includes joint angle sliders, 6-DOF inverse kinematics with position and ZYX orientation control, draggable IK target, mesh labels toggle, and double-click-to-edit value labels.

User arguments (optional): $ARGUMENTS
- If the user specifies an armature name, use that. Otherwise auto-detect the first armature in the scene.
- If the user specifies an output filename, use that. Otherwise default to `threejs_scene.html`.
- If the user specifies mesh names to exclude, hide those in the viewer.

## Step 1: Inspect the Blender Scene

Use `mcp__blender__get_scene_info` to get the scene overview. Identify:
- The armature object (type: ARMATURE)
- All mesh objects and their names
- Any objects to exclude (cameras, lights, curves, empties)

## Step 2: Extract Armature Bone Data

Use `mcp__blender__execute_blender_code` to run this Python code in Blender, adapted to the armature name found in Step 1:

```python
import bpy, json, mathutils

arm_obj = bpy.data.objects['ARMATURE_NAME']  # Replace with actual name
arm = arm_obj.data

# Coordinate conversion: Blender Z-up to Three.js Y-up
C = mathutils.Matrix((
    (1, 0, 0, 0),
    (0, 0, -1, 0),
    (0, 1, 0, 0),
    (0, 0, 0, 1),
))
C_inv = C.inverted()

bones_data = []
for bone in arm.bones:
    # Get parent-relative rest transform in Three.js Y-up coords
    if bone.parent:
        local_mat = C @ bone.parent.matrix_local.inverted() @ bone.matrix_local @ C_inv
    else:
        local_mat = C @ bone.matrix_local @ C_inv

    pos = local_mat.to_translation()
    quat = local_mat.to_quaternion()  # Blender quats are (w,x,y,z)

    bones_data.append({
        'name': bone.name,
        'parent': bone.parent.name if bone.parent else None,
        'pos': [round(pos.x, 5), round(pos.y, 5), round(pos.z, 5)],
        'quat_wxyz': [round(quat.w, 5), round(quat.x, 5), round(quat.y, 5), round(quat.z, 5)],
        'length': round(bone.length, 5),
    })

print(json.dumps(bones_data, indent=2))
```

## Step 3: Determine Joint Rotation Axes

Use `mcp__blender__execute_blender_code` to determine the rotation axis for each bone. For serial robot arms, common conventions are:
- Revolute joints about the bone's local Y or Z axis in Blender (which becomes local Z or -Y in Three.js Y-up)
- The axis can often be inferred from bone constraints or custom properties

Run this code to check for bone constraints and determine axes:

```python
import bpy, json

arm_obj = bpy.data.objects['ARMATURE_NAME']
pose = arm_obj.pose

for pb in pose.bones:
    constraints = []
    for c in pb.constraints:
        constraints.append({'type': c.type, 'name': c.name})

    # Check bone's principal axis (the direction it points)
    bone = pb.bone
    axis_vec = bone.matrix_local.to_3x3() @ mathutils.Vector((0, 1, 0))  # bone Y = along bone

    print(f"{pb.name}: constraints={constraints}, bone_y_world={[round(v,3) for v in axis_vec]}")
```

If constraints don't reveal the axes, ask the user which axis each joint rotates about. Common patterns for serial manipulators:
- Base rotation (J1): about world Z (Blender) = local Z in Three.js Y-up after rest transform
- Shoulder/elbow pitch joints: about bone's local Z (Blender) = local -Y in Three.js Y-up
- Wrist roll joints: about bone's local Y (Blender) = local Z in Three.js Y-up
- Flange roll: may be about bone's local X = local X in Three.js Y-up

The converted axes in Three.js local frame are typically one of:
- `[0, 0, 1]` — local Z (for yaw/roll joints that rotate about the bone direction after conversion)
- `[0, -1, 0]` — local -Y (for pitch joints)
- `[1, 0, 0]` — local X (for roll joints aligned with the approach axis)
- `[0, 1, 0]` — local Y

Ask the user to confirm the rotation axis for each joint if you cannot determine it automatically.

## Step 4: Determine Mesh-to-Bone Parenting

Use `mcp__blender__execute_blender_code` to find which meshes are parented to which bones:

```python
import bpy, json

arm_obj = bpy.data.objects['ARMATURE_NAME']
link_map = {}

for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.parent == arm_obj and obj.parent_bone:
        link_map[obj.name] = obj.parent_bone
    elif obj.type == 'MESH' and obj.parent == arm_obj:
        # Check for armature modifier vertex group parenting
        for mod in obj.modifiers:
            if mod.type == 'ARMATURE':
                link_map[obj.name] = 'armature_deform'
                break

print(json.dumps(link_map, indent=2))
```

The mesh names and their bone assignments will be used to build the `linkToJoint` mapping in the viewer. Each bone in the chain corresponds to a joint index (0-based).

## Step 5: Determine Joint Limits

Ask the user for joint limits (in degrees) for each joint, or use defaults:
- Revolute joints: typically -180 to 180 degrees
- Check if the Blender bones have rotation limits set:

```python
import bpy, json

arm_obj = bpy.data.objects['ARMATURE_NAME']
for pb in arm_obj.pose.bones:
    if pb.lock_ik_x or pb.lock_ik_y or pb.lock_ik_z:
        print(f"{pb.name}: IK locks x={pb.lock_ik_x} y={pb.lock_ik_y} z={pb.lock_ik_z}")
    if hasattr(pb, 'use_ik_limit_x') and pb.use_ik_limit_x:
        print(f"{pb.name}: IK X limits [{pb.ik_min_x:.3f}, {pb.ik_max_x:.3f}] rad")
    if hasattr(pb, 'use_ik_limit_y') and pb.use_ik_limit_y:
        print(f"{pb.name}: IK Y limits [{pb.ik_min_y:.3f}, {pb.ik_max_y:.3f}] rad")
    if hasattr(pb, 'use_ik_limit_z') and pb.use_ik_limit_z:
        print(f"{pb.name}: IK Z limits [{pb.ik_min_z:.3f}, {pb.ik_max_z:.3f}] rad")
```

## Step 6: Export glTF from Blender

Use `mcp__blender__execute_blender_code` to export the scene:

```python
import bpy, os

output_dir = 'OUTPUT_DIR'  # Replace with the working directory path
output_path = os.path.join(output_dir, 'robot_scene.glb')

bpy.ops.export_scene.gltf(
    filepath=output_path,
    export_format='GLB',
    export_apply=True,
    use_visible=True,
    export_yup=True,
    export_animations=False,
)
print(f"Exported to {output_path}")
```

If the export fails due to context issues, try:
```python
import bpy
# Override context for export
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        with bpy.context.temp_override(area=area):
            bpy.ops.export_scene.gltf(...)
        break
```

## Step 7: Generate the Three.js HTML Viewer

Create the HTML file using the data gathered in Steps 1-5. Use the existing `threejs_scene.html` in this project as a reference template. Read it with the Read tool to get the exact structure.

Key data to substitute into the template:
- `jointData` array: one entry per bone in the chain, with `restPos`, `restQuat` (w,x,y,z), and `axis` from Steps 2-3
- `jointLimits` array: from Step 5
- `linkToJoint` mapping: mesh name to joint index from Step 4
- Joint slider labels: use bone names or descriptive names
- Title: use the armature/robot name
- `hideNames` array: any meshes the user wants hidden
- Number of joints: adapt the slider HTML and all loops from 6 to N (the actual joint count)
- glTF filename: match the export filename from Step 6

### Important adaptations for N-DOF robots:

The template is built for 6-DOF. For robots with a different number of joints:
- Generate N slider rows (J1 through JN) in the HTML
- Set `jointData` array length to N
- Set `jointLimits` array length to N
- Set `jointAngles` array length to N
- All `for` loops iterating joints should use `jointData.length` or N instead of hardcoded 6
- The Jacobian in `solveIK` becomes a 6xN matrix (always 6 task DOFs, N joint DOFs)
- For N < 6: the system is underdetermined for orientation — consider using position-only IK (3xN Jacobian) or reducing orientation weight
- For N > 6: the system is redundant — DLS handles this naturally
- JJT is always 6x6 (or 3x3 for position-only), so `invertNxN` still works

### Coordinate convention:

The viewer uses Z-up display convention with Y-up Three.js internals:
- Position sliders: X, Y (depth), Z (vertical/up)
- Slider to Three.js mapping: slider Y ↔ Three.js Z, slider Z ↔ Three.js Y
- Orientation: ZYX Euler (α=Rz vertical, β=Ry depth, γ=Rx lateral)
- Three.js Euler order: 'YZX' (because user Z maps to Three.js Y)

## Step 8: Start HTTP Server and Test

```bash
python3 -m http.server 8000
```

Tell the user to open `http://localhost:8000/FILENAME.html` and verify:
1. All meshes load and appear correctly
2. FK sliders move the correct joints
3. Each joint rotates about the correct axis
4. IK mode works — dragging the target moves the robot
5. Labels toggle shows correct mesh names

If something is wrong, iterate on the joint axes or mesh parenting.
