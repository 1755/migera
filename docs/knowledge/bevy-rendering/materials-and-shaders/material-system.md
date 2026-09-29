---
title: "bevy_material: shared material plumbing"
description: Bevy 0.19.1's bevy_material crate holds shared plumbing only - MaterialProperties, AlphaMode, type-erased specialization keys, specializer function pointers - not the Material trait (bevy_pbr) nor AsBindGroup (bevy_render). Read when bevy_pbr material code confuses you or specialization ignores a material change.
type: reference
status: current
tags:
  - bevy
  - materials
  - render-pipeline
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_material-0.19.1/src
  - Bevy 0.19 migration guide, PR 22426
aliases:
  - MaterialProperties
  - AlphaMode
  - ErasedMaterialKey
  - DirtySpecializations
  - material specialization
---

# bevy_material: shared material plumbing

## Provenance and what it does NOT contain

`bevy_material`'s migration guide entry (PR 22426) is explicit: "Various material-related
machinery was extracted from `bevy_pbr` and `bevy_render` into a new crate called
`bevy_material`," moving `AlphaMode`/`SpecializedMeshPipelineError` from `bevy_render`, and
`OpaqueRendererMethod`, the `Erased*Key` family, `RenderPhaseType`, and
`MaterialProperties` from `bevy_pbr`.

**Neither `Material`/`Material2d` nor `AsBindGroup` live in `bevy_material`.** `Material`/
`Material2d` remain in `bevy_pbr::material` / `bevy_sprite_render::mesh2d::material`
(confirmed by direct grep — see [pbr-and-lighting](../pbr-and-lighting/INDEX.md) for the
`Material` trait itself); `AsBindGroup` remains in `bevy_render` (see
[gpu-resources-and-device](../resources-and-assets/gpu-resources-and-device.md)). So
`bevy_material` is best understood as **shared plumbing** — keys, properties,
specialization function pointers, descriptors — that both the 3D (`bevy_pbr`) and 2D
(`bevy_sprite_render`) material systems build on, rather than the material trait itself.

## `MaterialProperties`

A per-material-instance bag computed once and cached: `render_method:
OpaqueRendererMethod`, `alpha_mode`, precomputed `mesh_pipeline_key_bits:
ErasedMeshPipelineKey` ("precalculated so that we can just 'or' them together in
`queue_material_meshes`"), `depth_bias`, `reads_view_transmission_texture`,
`render_phase_type`, an optional `material_layout: BindGroupLayoutDescriptor`, up to a
`SmallVec<[..;4]>` of `(InternedDrawFunctionLabel, DrawFunctionId)` pairs, up to
`SmallVec<[..;3]>` `(InternedShaderLabel, Handle<Shader>)` pairs, a `bindless` flag, and
three type-erased specialize function pointers (`base_specialize`, `prepass_specialize`,
`user_specialize`). This is the object a custom `Material` impl's derive/setup code
populates once so per-frame queueing doesn't need to re-derive it.

## `AlphaMode`

`Opaque`, `Mask(f32)`, `Blend`, `Premultiplied`, `AlphaToCoverage`, `Add`, `Multiply` —
moved verbatim from `bevy_render`. Set on a material struct (e.g.
`StandardMaterial::alpha_mode`) to control blend-state selection during specialization.

## Type-erased keys

Because `bevy_material` can't know concrete material key types (defined per-user-material
in downstream crates), it type-erases them:

- **`ErasedMeshPipelineKey`** — stores a `u64` + `TypeId`, requires `downcast::<T:
  From<u64>>()`, asserting the `TypeId` matches.
- **`ErasedMaterialKey`** — boxes an `Any` value plus a hand-rolled vtable (`clone_fn`,
  `partial_eq_fn`) so it can be cloned/compared without generic bounds propagating
  everywhere.
- **`ErasedMaterialPipelineKey`** — bundles `mesh_key + material_key + type_id`; this
  triple is the actual hash-map key used to look up/insert specialized pipelines.

## `SpecializedMeshPipelineError` and specializer function pointers

```rust
pub enum SpecializedMeshPipelineError {
    MissingVertexAttribute(#[from] MissingVertexAttributeError),
}
```

Deliberately thin, just wrapping the `bevy_mesh` layout-lookup failure (see
[mesh-data-model](./mesh-data-model.md)). The three `*SpecializeFn` type aliases
(`BaseSpecializeFn`, `PrepassSpecializeFn`, `UserSpecializeFn`) are plain `fn` pointers
(not trait objects) taking `&mut World` + the erased key + `&MeshVertexBufferLayoutRef` +
`&Arc<MaterialProperties>`, documented to: look up the right specializer resource from
`World`, downcast the erased key, and call `SpecializedMeshPipelines::specialize`. This is
how `bevy_material` invokes generic, per-`M: Material` specialization logic without itself
being generic over `M`.

## `DrawFunctionId`, `ShaderLabel`, `DrawFunctionLabel`

`DrawFunctionId(pub u32)` is a lightweight index into `DrawFunctions` (defined in
`bevy_render`, see
[render-phases-and-batching](../architecture/render-phases-and-batching.md)).
`ShaderLabel`/`DrawFunctionLabel` are `bevy_ecs`-style interned label traits (`define_label!`),
letting a material register multiple named shader variants (`"frag"`, `"prepass_frag"`,
`"deferred_frag"`) and draw functions, retrievable by label rather than positional index.

## `DirtySpecializations`: NOT in bevy_material either

Grepping the crate confirms `DirtySpecializations` doesn't live here — it's in
`bevy_render::camera` (see
[camera-and-view-system](../scene-and-views/camera-and-view-system.md)) and is
**consumed** by `bevy_pbr::material`. Rather than iterating every visible entity every
frame to check whether its pipeline needs (re)specializing, `bevy_pbr` (via
`extract_entities_needs_specialization::<M>` / `extract_entities_that_need_
specializations_removed::<M>`) drains a main-world `EntitiesNeedingSpecialization<M>`
resource — populated when a material asset, mesh, or relevant component actually changed
— into the render-world change-list each frame. Downstream queueing systems only walk
`changed_renderables`/`removed_renderables` instead of the full visible-entity set. A
companion `PendingQueues`/`ViewPendingQueues` resource tracks entities that *couldn't* be
specialized/queued yet (e.g. assets still loading) so they're retried without falling back
to full re-iteration either.

## Practical recipe: custom Material + custom vertex format

1. Define custom vertex attributes (see [mesh-data-model](./mesh-data-model.md)); insert
   data via `Mesh::insert_attribute`.
2. In a custom pipeline's `specialize()`, call `layout.0.get_layout(&[Mesh::
   ATTRIBUTE_POSITION.at_shader_location(0), MyAttr.at_shader_location(3), ...])` to build
   the `VertexBufferLayout` for a given mesh.
3. Author shaders as `.wgsl` with `#define_import_path your_crate::your_module`; register
   library files with `load_shader_library!`; use `ShaderDefVal`-driven `#ifdef` blocks
   for optional features, matching them in your specialization key (see
   [shader-system](./shader-system.md)).
4. Implement `AsBindGroup` (from `bevy_render`) and `Material`/`Material2d` (from
   `bevy_pbr`/`bevy_sprite_render`) — `bevy_material` types (`MaterialProperties`,
   `AlphaMode`, `ErasedMaterialKey`) are what those trait impls populate under the hood,
   largely via macros; hand-authoring `bevy_material` types directly is rare, mostly
   reserved for fully custom render pipelines (see `examples/shader_advanced/
   specialized_mesh_pipeline.rs`, `manual_material.rs` in the vendored Bevy source).

## When to dive in

- Implementing `Material`/`Material2d` (the common path) → you mostly interact with
  `bevy_material` types indirectly, through macros and trait defaults; read
  [pbr-and-lighting](../pbr-and-lighting/INDEX.md) for where `Material` itself and
  `StandardMaterial` live.
- Debugging pipeline specialization not updating when a material asset changes → check the
  `DirtySpecializations`/`EntitiesNeedingSpecialization<M>` flow rather than assuming a
  cache bug.
- Building a fully custom, non-`Material`-trait render pipeline that still wants to share
  Bevy's key/specialization infrastructure → this is the rare case where hand-authoring
  `bevy_material` types directly makes sense.

## Related
- [StandardMaterial and the Material trait](../pbr-and-lighting/standard-material-and-pbr.md) — contrast: where the `Material` trait itself lives and how it uses this plumbing.
- [PipelineCache and pipeline specialization](../resources-and-assets/pipeline-cache-and-specialization.md) — deeper: what the specialization keys here feed into.
- [Camera and the view system](../scene-and-views/camera-and-view-system.md) — deeper: the `DirtySpecializations` change-list mechanism.
- [bevy_mesh: the mesh data model](./mesh-data-model.md) — prerequisite: the vertex layout half of a material pipeline key.
