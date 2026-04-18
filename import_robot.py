"""
Import a robot/device from the current Blender scene into the Three.js viewer.

Extracts bone transforms, rotation axes, mesh parenting, and joint limits
from a Blender armature, then exports a config JSON and skin-free GLB.

Usage from Blender Script Editor:
    exec(open('/FastDrive/Dropbox/ClaudeCodeProjects/RobotVisualisation/import_robot.py').read())

To override defaults, set variables before exec():
    ARMATURE_NAME = 'MyArmature'
    CONFIG_FILE = 'my_robot_config.json'
    GLB_FILE = 'my_robot_scene.glb'
    DEVICE_NAME = 'My Robot'
"""

import bpy
import json
import os
import re
import math
import mathutils

# ── Overridable settings ─────────────────────────────────────────────────────

PROJECT_DIR = '/FastDrive/Dropbox/ClaudeCodeProjects/RobotVisualisation'

try:
    armature_name = ARMATURE_NAME
except NameError:
    armature_name = None  # auto-detect

try:
    config_file = CONFIG_FILE
except NameError:
    config_file = None  # derived from device name

try:
    glb_file = GLB_FILE
except NameError:
    glb_file = None  # derived from config name

try:
    device_name = DEVICE_NAME
except NameError:
    device_name = None  # derived from armature name

# ── Coordinate conversion ────────────────────────────────────────────────────
# Blender Z-up → glTF/Three.js Y-up: X→X, Y→-Z, Z→Y

C = mathutils.Matrix((
    (1, 0, 0, 0),
    (0, 0, 1, 0),
    (0, -1, 0, 0),
    (0, 0, 0, 1),
))
C_inv = C.inverted()

# Blender quaternion free component → Three.js axis
# X and Z axes are negated to match robot joint rotation convention
AXIS_MAP = {
    'X': [1, 0, 0],
    'Y': [0, 0, -1],
    'Z': [0, -1, 0],
}


def sanitize_gltf_name(name):
    """Match Three.js GLTFLoader name sanitization: spaces→underscores, dots removed."""
    return name.replace(' ', '_').replace('.', '')


def compact_json(obj, indent=2):
    """JSON with compact single-line arrays for short numeric lists."""
    raw = json.dumps(obj, indent=indent)

    def collapse_array(m):
        inner = m.group(0)
        collapsed = re.sub(r'\s+', ' ', inner).strip()
        if len(collapsed) < 80 and '{' not in collapsed:
            return collapsed
        return inner

    return re.sub(r'\[[\s\d.,eE+\-"]+?\]', collapse_array, raw)


# ── Step 1: Find the armature ────────────────────────────────────────────────

if armature_name:
    arm_obj = bpy.data.objects[armature_name]
else:
    arm_obj = None
    for obj in bpy.data.objects:
        if obj.type == 'ARMATURE':
            arm_obj = obj
            break
    if arm_obj is None:
        raise RuntimeError("No armature found in scene. Set ARMATURE_NAME explicitly.")
    armature_name = arm_obj.name

arm = arm_obj.data

if device_name is None:
    device_name = armature_name

if config_file is None:
    slug = re.sub(r'[^a-zA-Z0-9]+', '_', device_name).strip('_').lower()
    config_file = f'{slug}_config.json'

if glb_file is None:
    slug = re.sub(r'[^a-zA-Z0-9]+', '_', device_name).strip('_').lower()
    glb_file = f'{slug}_scene.glb'

print(f"\n{'='*60}")
print(f"Robot Importer")
print(f"{'='*60}")
print(f"  Armature:  {armature_name}")
print(f"  Device:    {device_name}")
print(f"  Config:    {config_file}")
print(f"  GLB:       {glb_file}")
print(f"{'='*60}\n")


# ── Step 2: Extract bone transforms ──────────────────────────────────────────

print("Step 2: Extracting bone transforms...")

bones_ordered = []
bone_index = {}

for bone in arm.bones:
    bones_ordered.append(bone)
    bone_index[bone.name] = len(bones_ordered) - 1

bone_data = []
for bone in bones_ordered:
    if bone.parent:
        local_mat = C @ bone.parent.matrix_local.inverted() @ bone.matrix_local @ C_inv
    else:
        local_mat = C @ bone.matrix_local @ C_inv

    pos = local_mat.to_translation()
    quat = local_mat.to_quaternion()

    bone_data.append({
        'blender_name': bone.name,
        'parent_name': bone.parent.name if bone.parent else None,
        'pos': [round(pos.x, 5), round(pos.y, 5), round(pos.z, 5)],
        'quat': [round(quat.w, 5), round(quat.x, 5), round(quat.y, 5), round(quat.z, 5)],
    })

print(f"  Found {len(bone_data)} bones")


# ── Step 3: Determine rotation axes and fixed joints ─────────────────────────

print("Step 3: Determining rotation axes...")

pose = arm_obj.pose

for i, pb in enumerate(pose.bones):
    bd = bone_data[bone_index[pb.name]]
    lock_w = pb.lock_rotation_w
    lock_xyz = list(pb.lock_rotation)

    free = []
    if not lock_w:
        free.append('W')
    if not lock_xyz[0]:
        free.append('X')
    if not lock_xyz[1]:
        free.append('Y')
    if not lock_xyz[2]:
        free.append('Z')

    # All locked or all free → fixed joint
    if len(free) == 0 or len(free) == 4:
        bd['fixed'] = True
        bd['axis'] = [0, 0, 1]  # placeholder
        status = "FIXED (all locked)" if len(free) == 0 else "FIXED (all free)"
    else:
        # The free component (ignoring W) determines the axis
        free_axes = [f for f in free if f != 'W']
        if len(free_axes) == 1:
            bd['fixed'] = False
            bd['axis'] = AXIS_MAP[free_axes[0]]
            status = f"axis={free_axes[0]} → {bd['axis']}"
        else:
            # Multiple free axes — treat as fixed (unusual, user should review)
            bd['fixed'] = True
            bd['axis'] = [0, 0, 1]
            status = f"FIXED (multiple free: {free_axes}, review manually)"

    print(f"  {pb.name}: free={free}  → {status}")


# ── Step 4: Determine mesh-to-bone parenting ─────────────────────────────────

print("\nStep 4: Mapping meshes to bones...")

def find_bone_ancestor(obj):
    """Walk up the parent chain to find the bone this mesh ultimately belongs to."""
    current = obj
    while current:
        if current.parent == arm_obj and current.parent_bone:
            return current.parent_bone
        if current.parent and current.parent.type == 'MESH':
            current = current.parent
        else:
            break
    return None

mesh_to_bone = {}
unparented_meshes = []

for obj in bpy.data.objects:
    if obj.type != 'MESH':
        continue
    if obj.hide_render:
        continue

    bone_name = find_bone_ancestor(obj)
    if bone_name:
        mesh_to_bone[obj.name] = bone_name
        print(f"  {obj.name} → bone: {bone_name}")
    else:
        unparented_meshes.append(obj.name)
        print(f"  {obj.name} → (unparented, will be static)")


# ── Step 5: Extract joint limits ─────────────────────────────────────────────

print("\nStep 5: Extracting joint limits...")

for i, pb in enumerate(pose.bones):
    bd = bone_data[bone_index[pb.name]]

    if bd['fixed']:
        bd['limits'] = [0, 0]
        continue

    # Find which IK limit axis corresponds to the free rotation axis
    free_axes = []
    if not pb.lock_rotation_w:
        free_axes.append('W')
    if not pb.lock_rotation[0]:
        free_axes.append('X')
    if not pb.lock_rotation[1]:
        free_axes.append('Y')
    if not pb.lock_rotation[2]:
        free_axes.append('Z')

    rot_axis = [f for f in free_axes if f != 'W']
    if not rot_axis:
        bd['limits'] = [0, 0]
        continue

    axis_letter = rot_axis[0]
    has_limit = False

    if axis_letter == 'X' and pb.use_ik_limit_x:
        bd['limits'] = [round(math.degrees(pb.ik_min_x)), round(math.degrees(pb.ik_max_x))]
        has_limit = True
    elif axis_letter == 'Y' and pb.use_ik_limit_y:
        bd['limits'] = [round(math.degrees(pb.ik_min_y)), round(math.degrees(pb.ik_max_y))]
        has_limit = True
    elif axis_letter == 'Z' and pb.use_ik_limit_z:
        bd['limits'] = [round(math.degrees(pb.ik_min_z)), round(math.degrees(pb.ik_max_z))]
        has_limit = True

    if has_limit:
        print(f"  {pb.name}: {bd['limits']} deg")
    else:
        bd['limits'] = [-180, 180]
        print(f"  {pb.name}: no IK limits set, defaulting to [-180, 180]")


# ── Step 6: Export GLB (skin-free) ───────────────────────────────────────────

print("\nStep 6: Exporting GLB...")

glb_path = os.path.join(PROJECT_DIR, glb_file)

# Hide render-hidden meshes from viewport
visibility_backup = []
for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.hide_render and not obj.hide_viewport:
        visibility_backup.append(obj)
        obj.hide_viewport = True

# Unparent bone-parented meshes to prevent skins in GLB
parenting_backup = []
for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.parent == arm_obj and obj.parent_bone:
        parenting_backup.append({
            'obj': obj,
            'parent_bone': obj.parent_bone,
            'parent_type': obj.parent_type,
            'matrix_world': obj.matrix_world.copy()
        })
        mat = obj.matrix_world.copy()
        obj.parent = None
        obj.matrix_world = mat

bpy.ops.export_scene.gltf(
    filepath=glb_path,
    export_format='GLB',
    export_apply=True,
    use_visible=True,
    export_yup=True,
    export_animations=False,
    export_skins=False,
)

# Restore parenting
for info in parenting_backup:
    obj = info['obj']
    obj.parent = arm_obj
    obj.parent_bone = info['parent_bone']
    obj.parent_type = info['parent_type']
    obj.matrix_world = info['matrix_world']

# Restore visibility
for obj in visibility_backup:
    obj.hide_viewport = False

print(f"  Exported to {glb_path}")
print(f"  Unparented/restored {len(parenting_backup)} meshes")
print(f"  Hidden/restored {len(visibility_backup)} render-hidden meshes")


# ── Step 7: Generate config JSON ─────────────────────────────────────────────

print("\nStep 7: Generating config JSON...")

joints = []
for bd in bone_data:
    parent_idx = -1
    if bd['parent_name']:
        parent_idx = bone_index[bd['parent_name']]

    joint = {
        'name': bd['blender_name'],
        'bone': bd['blender_name'],
        'restPos': bd['pos'],
        'restQuat': bd['quat'],
        'axis': bd['axis'],
        'limits': bd['limits'],
        'parent': parent_idx,
    }
    if bd['fixed']:
        joint['fixed'] = True

    joints.append(joint)

# Build links array from mesh-to-bone mapping
links = []
for mesh_name, bone_name in sorted(mesh_to_bone.items()):
    joint_idx = bone_index.get(bone_name)
    if joint_idx is None:
        print(f"  WARNING: bone '{bone_name}' not found in armature for mesh '{mesh_name}'")
        continue

    sanitized = sanitize_gltf_name(mesh_name)
    links.append({
        'name': sanitized,
        'label': mesh_name.replace('_', ' '),
        'joint': joint_idx,
    })

# Build demo pose: all zeros
demo_pose = [0] * len(joints)

# Standard eeAxes for consistent crosshair orientation
ee_axes = [[0, 0, -1], [0, -1, 0], [1, 0, 0]]

# Compute eeOffset from the last bone's length.
# In Blender, a bone's tail is at (0, length, 0) in bone-local space.
# After C matrix conversion (Blender Y → Three.js -Z), this becomes [0, 0, -length].
last_bone = bones_ordered[-1]
ee_offset = [0, 0, round(-last_bone.length, 5)]
print(f"  eeOffset: {ee_offset} (from bone '{last_bone.name}', length={last_bone.length:.5f})")

config = {
    'name': device_name,
    'armature': armature_name,
    'model': glb_file,
    'joints': joints,
    'links': links,
    'eeOffset': ee_offset,
    'eeAxes': ee_axes,
    'demoPose': demo_pose,
}

config_path = os.path.join(PROJECT_DIR, config_file)
with open(config_path, 'w') as f:
    f.write(compact_json(config))
    f.write('\n')

print(f"  Wrote {config_path}")
print(f"  {len(joints)} joints ({sum(1 for j in joints if not j.get('fixed'))} movable, {sum(1 for j in joints if j.get('fixed'))} fixed)")
print(f"  {len(links)} links")


# ── Step 8: Register in panel.js ─────────────────────────────────────────────

print("\nStep 8: Checking panel.js registration...")

panel_path = os.path.join(PROJECT_DIR, 'js', 'panel.js')
with open(panel_path, 'r') as f:
    panel_src = f.read()

if config_file in panel_src:
    print(f"  '{config_file}' already registered in panel.js")
else:
    # Insert before the closing bracket of the configFiles array
    pattern = r"(export\s+const\s+configFiles\s*=\s*\[.*?)(])"
    match = re.search(pattern, panel_src, re.DOTALL)
    if match:
        before_bracket = match.group(1).rstrip()
        # Add trailing comma if needed
        if not before_bracket.endswith(','):
            before_bracket += ','
        new_src = panel_src[:match.start()] + before_bracket + f"\n    '{config_file}'\n" + match.group(2) + panel_src[match.end():]
        with open(panel_path, 'w') as f:
            f.write(new_src)
        print(f"  Added '{config_file}' to configFiles in panel.js")
    else:
        print(f"  WARNING: Could not find configFiles array in panel.js — add '{config_file}' manually")


# ── Summary ──────────────────────────────────────────────────────────────────

movable_joints = [j for j in joints if not j.get('fixed')]
print(f"\n{'='*60}")
print(f"Import complete!")
print(f"{'='*60}")
print(f"  Config:   {config_file}")
print(f"  GLB:      {glb_file}")
print(f"  Joints:   {len(movable_joints)} movable, {len(joints) - len(movable_joints)} fixed")
print(f"  Links:    {len(links)} meshes")
print(f"  eeOffset: {ee_offset} (IK target at flange surface)")
print(f"\nMovable joints:")
for j in movable_joints:
    print(f"  {j['name']}: axis={j['axis']}, limits={j['limits']}")

print(f"\nTo test, open:")
print(f"  http://localhost:8000/threejs_scene.html?config={config_file}")
print(f"\nReview checklist:")
print(f"  1. All meshes load and appear correctly")
print(f"  2. FK sliders move the correct joints")
print(f"  3. Each joint rotates about the correct axis")
print(f"  4. Labels toggle shows correct mesh names")
print(f"\nIf rotation directions are wrong, flip the axis sign in the config.")
print(f"If meshes are missing, check GLTFLoader name sanitization in browser console.")
print(f"{'='*60}")
