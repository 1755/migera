#!/usr/bin/env python3
"""How much a skinned mesh's triangles stretch when one segment is lengthened.

    python3 tools/skin_segment_stretch.py MODEL.glb UPPER LOWER [FACTOR]

e.g. `tools/skin_segment_stretch.py assets/models/character.glb
mixamorig:LeftUpLeg mixamorig:LeftLeg 1.1`. UPPER is the segment's joint and
LOWER its child (the joint that moves). Two ways to lengthen UPPER by FACTOR:

- `move`: move LOWER along its translation. UPPER's vertices stay put, so the
  blend region between the two joints is stretched across the gap.
- `proxy`: also skin UPPER through a scale of FACTOR along its own +Y (a
  Mixamo bone's axis), applied only to the skinning matrix. This is what
  `character_gallery --proportion-spike proxy` does live.

Each is measured with LOWER straight and bent 90° about X. It prints the
ratio of skinned edge length to the unmodified rig's, over every edge with
any weight on either joint, and over the knee-blend edges (an end weighted
> 0.1 on both). 1.0 is unchanged; a rigid lengthening would be 1.0 except
along the segment, at most FACTOR.
"""
import sys

import numpy as np
import pygltflib
from scipy.spatial.transform import Rotation

model, upper_name, lower_name = sys.argv[1:4]
factor = float(sys.argv[4]) if len(sys.argv) > 4 else 1.1

gltf = pygltflib.GLTF2().load(model)
blob = gltf.binary_blob()
WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
DTYPE = {5126: np.float32, 5123: np.uint16, 5121: np.uint8, 5125: np.uint32}
NORMALIZED_MAX = {np.uint8: 255.0, np.uint16: 65535.0}


def accessor(index):
    acc = gltf.accessors[index]
    view = gltf.bufferViews[acc.bufferView]
    width, dtype = WIDTH[acc.type], DTYPE[acc.componentType]
    item = width * np.dtype(dtype).itemsize
    stride = view.byteStride or item
    offset = (view.byteOffset or 0) + (acc.byteOffset or 0)
    raw = np.frombuffer(blob, np.uint8, count=stride * (acc.count - 1) + item, offset=offset)
    rows = np.lib.stride_tricks.as_strided(raw, (acc.count, item), (stride, 1)).copy()
    return rows.view(dtype).reshape(acc.count, width)


parent = {child: i for i, node in enumerate(gltf.nodes) for child in (node.children or [])}
index_of = {node.name: i for i, node in enumerate(gltf.nodes)}
upper, lower = index_of[upper_name], index_of[lower_name]


def local_matrix(i, moved, bend):
    node = gltf.nodes[i]
    t = np.array(node.translation or [0, 0, 0], float)
    q = Rotation.from_quat(node.rotation or [0, 0, 0, 1])
    s = np.array(node.scale or [1, 1, 1], float)
    if i == lower:
        if moved:
            t = t * factor
        q = q * bend
    m = np.eye(4)
    m[:3, :3] = q.as_matrix() * s
    m[:3, 3] = t
    return m


def world_matrix(i, moved, bend):
    m = local_matrix(i, moved, bend)
    while i in parent:
        i = parent[i]
        m = local_matrix(i, moved, bend) @ m
    return m


skin = gltf.skins[0]
inverse_binds = accessor(skin.inverseBindMatrices).reshape(-1, 4, 4).transpose(0, 2, 1)
slot_upper, slot_lower = skin.joints.index(upper), skin.joints.index(lower)


def skinned(moved, proxy, bend):
    joints = []
    for slot, node in enumerate(skin.joints):
        w = world_matrix(node, moved, bend)
        if proxy and node == upper:
            w = w @ np.diag([1, factor, 1, 1])
        joints.append(w @ inverse_binds[slot])
    joints = np.array(joints)
    out = []
    for prim in gltf.meshes[0].primitives:
        positions = accessor(prim.attributes.POSITION)
        slots = accessor(prim.attributes.JOINTS_0).astype(int)
        weights = accessor(prim.attributes.WEIGHTS_0).astype(float)
        weights /= NORMALIZED_MAX.get(accessor(prim.attributes.WEIGHTS_0).dtype.type, 1.0)
        homogeneous = np.c_[positions, np.ones(len(positions))]
        vertices = sum(
            weights[:, c : c + 1] * np.einsum("nij,nj->ni", joints[slots[:, c]], homogeneous)[:, :3]
            for c in range(4)
        )
        on_upper = (weights * (slots == slot_upper)).sum(1)
        on_lower = (weights * (slots == slot_lower)).sum(1)
        triangles = accessor(prim.indices).ravel().reshape(-1, 3)
        out.append((vertices, triangles, on_upper, on_lower))
    return out


for label, bend in (("straight", Rotation.identity()), ("bent 90", Rotation.from_rotvec([np.pi / 2, 0, 0]))):
    base = skinned(False, False, bend)
    for mode, proxy in (("move", False), ("proxy", True)):
        modified = skinned(True, proxy, bend)
        every, blend = [], []
        for (v0, tris, wu, wl), (v1, _, _, _) in zip(base, modified):
            edges = np.r_[tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]]
            touched = ((wu + wl)[edges] > 0.01).any(1)
            l0 = np.linalg.norm(v0[edges[:, 0]] - v0[edges[:, 1]], axis=1)
            l1 = np.linalg.norm(v1[edges[:, 0]] - v1[edges[:, 1]], axis=1)
            valid = touched & (l0 > 1e-9)
            mixed = (np.minimum(wu, wl)[edges] > 0.1).any(1)
            every.append(l1[valid] / l0[valid])
            blend.append(l1[valid & mixed] / l0[valid & mixed])
        every, blend = np.concatenate(every), np.concatenate(blend)
        print(
            f"{label:8s} {mode:5s} all {len(every):5d} edges: p50 {np.median(every):.3f} "
            f"p99 {np.percentile(every, 99):.3f} max {every.max():.3f} | "
            f"blend {len(blend):4d}: p50 {np.median(blend):.3f} max {blend.max():.3f}"
        )
