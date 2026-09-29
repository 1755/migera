---
title: Mesh Conversion
description: Both directions between meshes and SDFs — unsigned distance and sign determination for baking, glTF-specific import, GPU baking and storage, glitch-free QA, adding a baked leaf to migera's raymarcher, and isosurface extraction back to triangles. Read when baking mesh assets into SDFs or extracting meshes from SDFs.
type: index
status: current
tags:
  - sdf
  - mesh-conversion
  - baking
updated: 2026-09-28
---

# Mesh Conversion

Real content usually starts as a polygonal mesh, and production rendering usually
needs polygons at the end too, so converting *into* and *out of* an SDF are both
practical skills. Everything here is general research except
[hybrid-baked-and-procedural-scenes](./hybrid-baked-and-procedural-scenes.md), which is
scoped to migera's own `src/raymarch` evaluator (and describes a leaf kind that is not
built as of 2026-09-28).

## Start here

Baking a mesh: [exact-point-to-mesh-distance](./exact-point-to-mesh-distance.md) →
[sign-determination-methods](./sign-determination-methods.md) (sign is the harder half)
→ [efficient-grid-baking](./efficient-grid-baking.md) →
[glitch-free-baked-sdfs](./glitch-free-baked-sdfs.md) (read before shipping, not only
when something looks wrong).

## Key facts

- Unsigned mesh distance is solved given a BVH; inside/outside on real-world meshes is where bakes fail — GWN is the robust default — see [sign-determination-methods](./sign-determination-methods.md).
- A trilinear-interpolated bake is only a bound; step damping is load-bearing, and features under ~1 voxel vanish — see [glitch-free-baked-sdfs](./glitch-free-baked-sdfs.md).
- Negative-scale glTF nodes flip winding and cause partial inside-out bakes — see [gltf-import-pipeline](./gltf-import-pipeline.md).
- A baked leaf would be a small additive `eval_leaf` case in migera, but smooth-blending it against exact leaves degrades subtly and has no production precedent — see [hybrid-baked-and-procedural-scenes](./hybrid-baked-and-procedural-scenes.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Computing exact point-to-mesh distance](./exact-point-to-mesh-distance.md) | Point-to-triangle distance with BVH/octree (CPU) or jump flooding (GPU). | Baking a mesh for the first time, or a bake is slow. |
| [Sign determination](./sign-determination-methods.md) | Ray parity, pseudonormals, generalized winding number, depth rasterization; robustness tradeoffs. | Baking untrusted meshes, or a bake is inside-out. |
| [Importing glTF/GLB models](./gltf-import-pipeline.md) | glTF structure and transforms, winding/skinning gotchas, mesh defects and repair, Rust crates, decimation, bake-at-import caching. | Writing a glTF→SDF importer. |
| [Efficient baking](./efficient-grid-baking.md) | GPU bake algorithms, resolution/narrow band, Unreal clipmaps, compression, gradient storage. | Building a baker or budgeting baked-SDF memory. |
| [Producing glitch-free baked SDFs](./glitch-free-baked-sdfs.md) | Thin-feature loss, interpolation safety, sign-flip appearance, chunk seams, staircasing, QA checks. | Before shipping a bake, or when a baked surface pits/cracks. |
| [Integrating a baked mesh SDF into this project's raymarcher](./hybrid-baked-and-procedural-scenes.md) | What a baked leaf needs in `raymarch.wgsl`, smin degradation, no non-uniform scale, bounding-box culling. | Before adding any texture-sampled leaf to `src/raymarch`. |
| [Extracting a mesh from an SDF](./sdf-to-mesh-extraction.md) | Marching Cubes vs. Dual Contouring vs. hybrids, and differentiable variants. | Converting SDF content to triangles for rasterization, export or physics. |

## See also

- [Converting GLB scenes to SDF BSN scenes](../glb-to-bsn-conversion.md) — archived primitive-fitting alternative to baking.
- [SDF ↔ 3DGS Bevy integration](../../sdf-3dgs-bevy-integration/INDEX.md) — archived record of converting SDFs into Gaussian splats instead of meshes.
