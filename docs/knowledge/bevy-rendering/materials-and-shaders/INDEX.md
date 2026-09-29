---
title: Materials and Shaders
description: Bevy 0.19.1's shared plumbing crates bevy_mesh, bevy_shader and bevy_material that both 3D and 2D materials build on; the Material trait and StandardMaterial are in bevy_pbr and AsBindGroup in bevy_render, not here. Read for custom vertex attributes, WGSL imports and shader defs, or material specialization keys.
type: index
status: current
tags:
  - bevy
  - materials
  - wgsl
  - render-pipeline
updated: 2026-09-28
---

# Materials and Shaders

`bevy_mesh`, `bevy_shader` and `bevy_material` are recent standalone crates split out of
`bevy_render`/`bevy_pbr`. They hold the shared data/shader/specialization plumbing under
both the 3D (`bevy_pbr`) and 2D (`bevy_sprite_render`) material systems. The split matters
because searching here for `Material` or `AsBindGroup` finds nothing:
`Material`/`StandardMaterial` are in [PBR and Lighting](../pbr-and-lighting/INDEX.md), and
`AsBindGroup` is in [gpu-resources-and-device](../resources-and-assets/gpu-resources-and-device.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [bevy_mesh: the mesh data model](./mesh-data-model.md) | `Mesh`, `MeshVertexAttribute`, `VertexAttributeValues`, `MeshVertexBufferLayout` driving specialization, mesh builders, skinning/morph data. | Adding a custom vertex attribute, building procedural geometry, or "missing vertex attribute" errors. |
| [bevy_shader: WGSL, preprocessing, and the shader cache](./shader-system.md) | `Shader` formats, naga_oil `#import`/`#ifdef`/`ShaderDefVal`, `load_shader_library!`, import resolution, `ShaderCache`. | A shader import fails, hot reload misses a change, or adding shader defs. |
| [bevy_material: shared material plumbing](./material-system.md) | `MaterialProperties`, `AlphaMode`, type-erased keys, specializer function pointers — and what is NOT in the crate. | Reading `bevy_pbr` material code, or specialization ignores a material change. |

## See also

- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: WGSL loading and pipelines for migera's compute passes.
