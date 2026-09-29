---
title: "bevy_mesh: the mesh data model"
description: Bevy 0.19.1's bevy_mesh crate owns Mesh (topology, attributes, indices), MeshVertexAttribute for custom vertex channels, the MeshVertexBufferLayout that drives pipeline specialization, mesh builders, and skinning/morph data. Read before custom vertex attributes, procedural meshes, or missing-attribute errors.
type: reference
status: current
tags:
  - bevy
  - assets
  - render-pipeline
  - character-animation
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_mesh-0.19.1/src
aliases:
  - Mesh asset
  - MeshVertexAttribute
  - custom vertex attribute
  - skinned mesh
  - morph targets
---

# bevy_mesh: the mesh data model

## Provenance

`bevy_mesh` is a standalone crate split out of `bevy_render`, home to `Mesh` and its
supporting types (`Cargo.toml` features `morph`, `serialize`; depends on
`bevy_mikktspace`). Its `lib.rs` doesn't narrate the split, but the crate absorbed the
low-level mesh data model wholesale, decoupled from rendering.

## `Mesh`

`Mesh` (`mesh.rs`) is a `bevy_asset::Asset` holding a `PrimitiveTopology` (re-exported
from `wgpu_types` — no reinvention), a `BTreeMap<MeshVertexAttributeId,
MeshAttributeData>` of vertex data, an optional `Indices`, morph target data (feature-
gated), and bookkeeping like `asset_usage: RenderAssetUsages` and a precomputed
`final_aabb`. The `BTreeMap` is deliberate: it guarantees deterministic iteration order so
GPU vertex buffers are laid out consistently across identical meshes.

```rust
Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U32(indices))
```

Mesh data can be *extracted to the render world* (`MeshExtractableData::
ExtractedToRenderWorld`), after which CPU-side accessors return `MeshAccessError::
ExtractedToRenderWorld` rather than silently panicking — this happens when `asset_usage`
doesn't include `MAIN_WORLD`, freeing CPU memory once GPU buffers are built. A tool/editor
needing to keep reading mesh data after upload must opt into `RenderAssetUsages::
MAIN_WORLD | RenderAssetUsages::RENDER_WORLD` (see
[render-assets](../resources-and-assets/render-assets.md)).

## `MeshVertexAttribute` and custom vertex attributes

A named, typed vertex channel schema entry (not the data itself):

```rust
pub struct MeshVertexAttribute {
    pub name: &'static str,
    pub id: MeshVertexAttributeId,   // u64, must be globally unique
    pub format: VertexFormat,
}
```

Built-ins use small ids (`ATTRIBUTE_POSITION` = `0`, plus `ATTRIBUTE_NORMAL`, `ATTRIBUTE_
UV_0`, `ATTRIBUTE_UV_1`, `ATTRIBUTE_TANGENT`, `ATTRIBUTE_COLOR`, `ATTRIBUTE_JOINT_WEIGHT`,
`ATTRIBUTE_JOINT_INDEX`). A custom attribute is a `const` you define:

```rust
pub const ATTRIBUTE_BARYCENTRIC: MeshVertexAttribute =
    MeshVertexAttribute::new("Vertex_Barycentric", 988540917, VertexFormat::Float32x3);
```

There is **no central registry** — the doc comment explicitly warns to "use a random /
very large u64 to avoid conflicts." This is the extension point for a custom vertex
format: define the const, `insert_attribute` data into a `Mesh`, wire it to a shader
location via `at_shader_location(n)`.

## `VertexAttributeValues` and `Indices`

`VertexAttributeValues` is a large enum mirroring every `wgpu_types::VertexFormat`
variant (`Float32x3(Vec<[f32;3]>)`, `Uint16x4`, `Unorm8x4`, ...), plus Bevy-only
convenience unpacked-scalar variants. Each `MeshVertexAttribute` must pair with matching-
format `VertexAttributeValues` — mismatches panic at insertion time (attributes are type-
erased at the `Mesh` boundary, so this can't be caught statically). `get_bytes()`
flattens any variant to `&[u8]` via `bytemuck::cast_slice`, which is how the renderer
uploads uniformly regardless of concrete type.

`Indices` is `U16(Vec<u16>)` / `U32(Vec<u32>)`. `push`/`extend` auto-promote `U16` to
`U32` the moment a value exceeds `u16::MAX`, so incremental builders don't need to
pre-decide width.

## `MeshVertexBufferLayout` and pipeline specialization

This is the crux of how custom vertex formats plug into rendering.
`Mesh::get_mesh_vertex_buffer_layout` walks the attribute `BTreeMap` in id order,
computes byte offsets, and produces a `MeshVertexBufferLayout` (attribute ids + a
`VertexBufferLayout`). Because the same logical layout recurs across many mesh instances,
layouts are interned in `MeshVertexBufferLayouts` (`HashSet<Arc<MeshVertexBufferLayout>>`)
yielding a `MeshVertexBufferLayoutRef` whose `PartialEq`/`Hash` compare **by pointer**, not
structurally — intentional for performance, since this is used as part of a specialization
cache key every frame (see
[pipeline-cache-and-specialization](../resources-and-assets/pipeline-cache-and-specialization.md)).

A pipeline consumes this via `get_layout(&[VertexAttributeDescriptor])`: given the shader
locations a pipeline variant wants, it looks up each requested attribute id in the mesh's
actual layout and remaps offsets/formats, returning `MissingVertexAttributeError` if the
mesh lacks something the shader needs. `SpecializedMeshPipeline` implementations call this
during specialization — a mesh with a nonstandard vertex format naturally produces a
different `MeshVertexBufferLayoutRef`, which becomes a different specialization key,
yielding a distinct compiled pipeline.

`BaseMeshPipelineKey` packs `PrimitiveTopology`, strip index format, and a morph-targets
flag into high bits of a `u64`, deliberately leaving low bits free so `bevy_pbr`'s own key
bits can coexist without shifting.

## Primitive topology and mesh builders

`PrimitiveTopology` is a plain re-export from `wgpu_types`. The `primitives` module
implements `Meshable`/`MeshBuilder` for `bevy_math` shape primitives (`Cuboid`, `Sphere`,
`Cylinder`, `Cone`, `Torus`, `Capsule`, `Plane`, 2D shapes, `Extrusion`, ...):

```rust
pub trait Meshable { type Output: MeshBuilder; fn mesh(&self) -> Self::Output; }
pub trait MeshBuilder { fn build(&self) -> Mesh; }
```

Idiomatic usage: `meshes.add(Circle { radius: 25.0 }.mesh().resolution(64))`.

## Skinning and morph targets live here, not in bevy_pbr

`skinning.rs` defines `SkinnedMesh` (handle to `SkinnedMeshInverseBindposes` + joint
`Entity` list), `SkinnedMeshInverseBindposes` (a boxed `[Mat4]` asset), and CPU-side AABB
computation for culling skinned meshes correctly. `morph.rs` defines `MorphWeights`/
`MeshMorphWeights` and packing constants (`MAX_MORPH_WEIGHTS = 256`, `MAX_TEXTURE_WIDTH =
2048`) governing the morph-target texture packing scheme. `bevy_pbr` consumes these for
the actual GPU-side skinning/morphing math (compute shaders, buffer layout), but the
*data representation and component types* are owned by `bevy_mesh` — sensible, since 2D
or non-PBR renderers could reuse the same skeleton without depending on `bevy_pbr`.

## When to dive in

- Adding a custom vertex attribute (e.g. barycentric coords for wireframe shading, per-
  vertex custom data) → the `MeshVertexAttribute`/`MeshVertexBufferLayout` sections are
  the whole story; pair with
  [shader-system](./shader-system.md) for wiring it into WGSL.
- Building procedural geometry → use `Meshable`/`MeshBuilder` for standard shapes, or
  construct a `Mesh` directly with `with_inserted_attribute`.
- Debugging "missing vertex attribute" pipeline errors → the mesh's actual
  `MeshVertexBufferLayoutRef` doesn't contain what your shader/pipeline requested; check
  what attributes the mesh was actually built with.

## Related
- [bevy_shader: WGSL, preprocessing, and the shader cache](./shader-system.md) — deeper: wiring a custom vertex attribute into WGSL.
- [PipelineCache and pipeline specialization](../resources-and-assets/pipeline-cache-and-specialization.md) — deeper: how `MeshVertexBufferLayout` becomes part of a specialization key.
- [RenderAsset lifecycle](../resources-and-assets/render-assets.md) — deeper: how a `Mesh` becomes GPU buffers.
- [bevy_material: shared material plumbing](./material-system.md) — contrast: the material half of the pipeline key.
- [migera pivots to Bevy PBR for characters](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — applies: migera's characters are skinned `Mesh` assets on this data model.
