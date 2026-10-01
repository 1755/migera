#!/usr/bin/env python3
"""Dump a glTF skin's BIND pose as world joint positions, in the frame
`.positions.ron` clip dumps use (+Y up, facing -Z), for
`examples/import_reference_pose --bind`.

    python3 tools/dump_bind_positions.py assets/models/idle.glb \
        assets/anim/idle_bind.positions.ron

Why it exists: a clip's world positions describe the actor's shape, which
includes the shape the source rig was BOUND in. Converted against this
crate's straight synthetic T-pose, Mixamo's bind curvature (its spine leans
back 14 and 12 degrees in its upper segments) was stored as a bend, and the
target rig's own bind curvature was then bent again: an over-arched back.
Converting against the source's bind instead keeps only what the actor did.

The bind is each joint's inverse bind matrix, inverted. The rig is turned
half round about +Y if its feet point +Z, so it faces -Z as the clip dumps
do; Mixamo's `mixamorig:` prefix is dropped.
"""
import sys

import numpy as np
import pygltflib


def main() -> None:
    if len(sys.argv) != 3:
        sys.exit("usage: dump_bind_positions.py <model.glb> <out.positions.ron>")
    source, out = sys.argv[1], sys.argv[2]
    gltf = pygltflib.GLTF2().load(source)
    blob = gltf.binary_blob()
    skin = gltf.skins[0]
    accessor = gltf.accessors[skin.inverseBindMatrices]
    view = gltf.bufferViews[accessor.bufferView]
    start = view.byteOffset + (accessor.byteOffset or 0)
    inverse_binds = np.frombuffer(blob[start : start + accessor.count * 64], dtype=np.float32).reshape(accessor.count, 4, 4)

    positions = {}
    for slot, joint in enumerate(skin.joints):
        name = gltf.nodes[joint].name.removeprefix("mixamorig:")
        # glTF matrices are column-major.
        positions[name] = np.linalg.inv(inverse_binds[slot].T)[:3, 3]

    # Face -Z: turned half round about +Y if the feet point +Z.
    if positions["LeftToeBase"][2] - positions["LeftFoot"][2] > 0.0:
        positions = {name: np.array([-p[0], p[1], -p[2]]) for name, p in positions.items()}

    lines = ["(", "    positions: {"]
    for name, p in positions.items():
        lines.append(f'        "{name}": ({p[0]:.6g}, {p[1]:.6g}, {p[2]:.6g}),')
    lines += ["    },", ")", ""]
    with open(out, "w") as file:
        file.write("\n".join(lines))
    print(f"{len(positions)} joints -> {out}")


if __name__ == "__main__":
    main()
