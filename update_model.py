"""
Update model script — runs inside Blender via MCP or Script Editor.

Reads a config JSON with 'bone' fields on each joint, extracts updated
transforms from the armature, increments the GLB version, exports skin-free,
and writes the updated config.

Usage from Blender Script Editor or MCP:
    exec(open('/FastDrive/Dropbox/ClaudeCodeProjects/RobotVisualisation/update_model.py').read())

To override the config file, set CONFIG_FILE before running:
    CONFIG_FILE = 'robot_config.json'
    exec(open('update_model.py').read())
"""

import bpy
import json
import re
import os
import mathutils

# Config file to update — override by setting CONFIG_FILE before exec()
try:
    config_file = CONFIG_FILE
except NameError:
    config_file = 'i16_config.json'

PROJECT_DIR = '/FastDrive/Dropbox/ClaudeCodeProjects/RobotVisualisation'
config_path = os.path.join(PROJECT_DIR, config_file)

# Blender Z-up to glTF/Three.js Y-up
C = mathutils.Matrix((
    (1, 0, 0, 0),
    (0, 0, 1, 0),
    (0, -1, 0, 0),
    (0, 0, 0, 1),
))
C_inv = C.inverted()

# --- Load config ---
with open(config_path, 'r') as f:
    config = json.load(f)

armature_name = config.get('armature', 'Armature')
arm_obj = bpy.data.objects[armature_name]
arm = arm_obj.data

# --- Build bone transform lookup ---
bone_transforms = {}
for bone in arm.bones:
    if bone.parent:
        local_mat = C @ bone.parent.matrix_local.inverted() @ bone.matrix_local @ C_inv
    else:
        local_mat = C @ bone.matrix_local @ C_inv

    pos = local_mat.to_translation()
    quat = local_mat.to_quaternion()

    bone_transforms[bone.name] = {
        'pos': [round(pos.x, 5), round(pos.y, 5), round(pos.z, 5)],
        'quat': [round(quat.w, 5), round(quat.x, 5), round(quat.y, 5), round(quat.z, 5)],
    }

# --- Update joint transforms ---
changes = []
for joint in config['joints']:
    bone_name = joint.get('bone')
    if not bone_name or bone_name not in bone_transforms:
        continue

    bt = bone_transforms[bone_name]
    old_pos = joint['restPos']
    old_quat = joint['restQuat']

    if old_pos != bt['pos'] or old_quat != bt['quat']:
        changes.append(f"  {joint['name']}: pos {old_pos} -> {bt['pos']}, quat {old_quat} -> {bt['quat']}")
        joint['restPos'] = bt['pos']
        joint['restQuat'] = bt['quat']

# --- Increment GLB version ---
old_model = config['model']
match = re.search(r'_v(\d+)\.glb$', old_model)
if match:
    version = int(match.group(1)) + 1
    new_model = re.sub(r'_v\d+\.glb$', f'_v{version}.glb', old_model)
else:
    base = old_model.replace('.glb', '')
    version = 2
    new_model = f'{base}_v{version}.glb'

config['model'] = new_model
glb_path = os.path.join(PROJECT_DIR, new_model)

# --- Export GLB (skin-free) ---
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

for info in parenting_backup:
    obj = info['obj']
    obj.parent = arm_obj
    obj.parent_bone = info['parent_bone']
    obj.parent_type = info['parent_type']
    obj.matrix_world = info['matrix_world']

# --- Write updated config (compact arrays) ---
def compact_json(obj, indent=2):
    """JSON with compact single-line arrays for short lists."""
    raw = json.dumps(obj, indent=indent)
    # Collapse short arrays (numbers only) onto single lines
    import re
    def collapse_array(m):
        inner = m.group(0)
        # Only collapse arrays of numbers (no nested objects)
        collapsed = re.sub(r'\s+', ' ', inner).strip()
        if len(collapsed) < 80 and '{' not in collapsed:
            return collapsed
        return inner
    raw = re.sub(r'\[[\s\d.,eE+\-"]+?\]', collapse_array, raw)
    return raw

with open(config_path, 'w') as f:
    f.write(compact_json(config))
    f.write('\n')

# --- Report ---
print(f"\n=== Model Update Complete ===")
print(f"Config: {config_file}")
print(f"GLB: {old_model} -> {new_model}")
print(f"Unparented/restored {len(parenting_backup)} meshes")
if changes:
    print(f"\n{len(changes)} joint(s) updated:")
    for c in changes:
        print(c)
else:
    print("\nNo transform changes detected.")
