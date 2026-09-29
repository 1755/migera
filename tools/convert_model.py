"""Headless FBX (or any Blender-importable format) -> glTF/GLB converter.

Run inside the `model-convert` Nix dev shell (`nix develop .#model-convert`):

    blender --background --python tools/convert_model.py -- <input> <output.glb>

Exists because Mixamo.com only ever exports FBX (no glTF option), but
migera's own character-loading path (`examples/character_gallery.rs`'s
`spawn_real_mesh`, via Bevy's `bevy_gltf` importer) only reads glTF/GLB.
Blender 2.8+ ships both an FBX importer and a glTF 2.0 exporter as
built-in add-ons, so no external converter binary is needed -- this
script just drives them from a clean scene, headlessly.

Input format is auto-detected from the input file's own extension (.fbx,
.obj, .dae, ... anything Blender's own importers handle); output is
always glTF Binary (.glb) if the output path ends in .glb, or glTF
Separate (.gltf + .bin + textures) otherwise, matching whichever
extension is given.
"""

import sys
import os

import bpy


def main():
    argv = sys.argv
    if "--" not in argv:
        print("usage: blender --background --python tools/convert_model.py -- <input> <output.glb>")
        sys.exit(1)
    args = argv[argv.index("--") + 1:]
    if len(args) != 2:
        print("usage: blender --background --python tools/convert_model.py -- <input> <output.glb>")
        sys.exit(1)
    input_path, output_path = args
    input_path = os.path.abspath(input_path)
    output_path = os.path.abspath(output_path)

    if not os.path.isfile(input_path):
        print(f"error: input file not found: {input_path}")
        sys.exit(1)

    # Start from a clean scene -- Blender's default startup file has a
    # camera/light/cube that would otherwise end up exported alongside
    # the real character.
    bpy.ops.wm.read_factory_settings(use_empty=True)

    ext = os.path.splitext(input_path)[1].lower()
    if ext == ".fbx":
        bpy.ops.import_scene.fbx(filepath=input_path)
    elif ext == ".obj":
        bpy.ops.wm.obj_import(filepath=input_path)
    elif ext in (".dae",):
        bpy.ops.wm.collada_import(filepath=input_path)
    elif ext in (".gltf", ".glb"):
        bpy.ops.import_scene.gltf(filepath=input_path)
    else:
        print(f"error: unsupported input extension '{ext}' -- add an importer call in this script")
        sys.exit(1)

    export_format = "GLB" if output_path.lower().endswith(".glb") else "GLTF_SEPARATE"
    bpy.ops.export_scene.gltf(
        filepath=output_path,
        export_format=export_format,
        # Export the skin/armature and its bind pose -- the whole point
        # of this conversion is a retargetable, skinned rig, not a static
        # mesh.
        export_skins=True,
        export_apply=False,
        # Preserve the source file's own bone names exactly (no Blender-
        # side renaming/prefix stripping) so migera's own naming-
        # convention resolver (`resolve_bone_node_name`) can still match
        # against Mixamo/UE-Mannequin conventions afterward.
        export_rest_position_armature=True,
    )
    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()
