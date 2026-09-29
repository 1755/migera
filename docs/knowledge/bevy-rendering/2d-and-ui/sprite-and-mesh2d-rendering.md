---
title: "2D rendering: Sprite, Mesh2d, and the two batching strategies"
description: Bevy 0.19.1 renders 2D through two paths - the classic CPU-batched Sprite pipeline (batches only consecutive same-texture items, never writes depth) and the GPU-driven Mesh2d/Material2d pipeline that shares 3D's batching code. Read when 2D draw calls explode, layering is wrong, or you write a 2D material.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - rasterization
  - performance
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_sprite-0.19.1/src, bevy_sprite_render-0.19.1/src
aliases:
  - Sprite batching
  - Material2d
  - Transparent2d
  - SpriteMesh
---

# 2D rendering: Sprite, Mesh2d, and the two batching strategies

## `bevy_sprite` vs `bevy_sprite_render`: the data/logic vs GPU crate split

Same pattern as `bevy_camera`/`bevy_render` elsewhere: `bevy_sprite` owns "what a thing
is" (ECS components, layout math), `bevy_sprite_render` owns "how it gets onto the GPU."

`bevy_sprite-0.19.1/src/` is render-agnostic: `sprite.rs` (the `Sprite` component,
`Anchor`, `SpriteImageMode`/`SpriteScalingMode`), `sprite_mesh.rs` (a newer `SpriteMesh`
component, see below), `texture_slice/` (9-slice/tiled slicing math, pure CPU geometry),
`text2d.rs` (`Text2d` layout), `picking_backend.rs`. `SpritePlugin` only runs `PostUpdate`
visibility/AABB systems — no `RenderApp` touch at all.

`bevy_sprite_render-0.19.1/src/` is the GPU counterpart: `render/` (classic sprite
batching pipeline), `mesh2d/` (Mesh2d/Material2d GPU pipeline), `sprite_mesh/` (bridges
`SpriteMesh` onto Mesh2d), `text2d/` (extracts `Text2d` into sprite draw data),
`texture_slice/computed_slices.rs`, `tilemap_chunk/`.

```rust
pub struct Sprite {
    pub image: Handle<Image>,
    pub texture_atlas: Option<TextureAtlas>,
    pub color: Color,
    pub flip_x: bool, pub flip_y: bool,
    pub custom_size: Option<Vec2>,
    pub rect: Option<Rect>,
    pub image_mode: SpriteImageMode, // Auto | Scale(mode) | Sliced(slicer) | Tiled{..}
}
```

Texture atlas handling is minimal at the data layer: `Sprite.texture_atlas` just stores a
`Handle<TextureAtlasLayout>` + `index`; `TextureAtlas::texture_rect()` (in `bevy_image`)
looks up a `URect`. Atlas-vs-custom-rect reconciliation happens independently in both
`bevy_sprite`'s `calculate_bounds_2d` (for AABB/culling) and `bevy_sprite_render`'s
`extract_sprites` (for draw data) — duplicated logic, a natural consequence of the crate
split.

## Two sprite representations, mid-migration

There are now **two** parallel sprite representations:

1. **Legacy `Sprite`** — rendered via the bespoke `SpritePipeline` (below).
2. **`SpriteMesh`** (`bevy_sprite-0.19.1/src/sprite_mesh.rs`) — a unit `Mesh2d` quad plus
   a `SpriteMaterial` (`Material2d`), going through the generic Mesh2d GPU-batched
   pipeline instead. The doc comment on `calculate_bounds_2d_sprite_mesh` explicitly says
   this is transitional: "Will eventually be merged with Sprite in the system above."

## The classic Sprite batching path: Extract → Queue → Prepare → Render

**Extract** (`extract_sprites`): for every visible `(Sprite, GlobalTransform, Anchor,
Option<ComputedTextureSlices>)`, push an `ExtractedSprite`. Sprites using 9-slice/tiling
instead push `ExtractedSpriteKind::Slices{ indices }` referencing entries in a flat
`ExtractedSlices` buffer — how one `Sprite` entity produces many quads (nine-patch
borders, text glyphs) without separate ECS entities.

**Queue** (`queue_sprites`, `RenderSystems::Queue`): for each camera, computes a
`SpritePipelineKey` (MSAA, HDR/tonemapping, compositing space), specializes
`SpritePipeline`, pushes one `Transparent2d` phase item per extracted sprite with
`sort_key = FloatOrd(transform.translation().z)`. `batch_range: 0..0` initially — actual
ranges filled in during Prepare.

**Prepare** (`prepare_sprite_image_bind_groups`, `RenderSystems::PrepareBindGroups`): the
real batching decision, fundamentally CPU-side and order-dependent, not GPU-indirect:

```rust
for item_index in 0..transparent_phase.items.len() {
    if batch_image_handle != Some(extracted_sprite.image_handle_id) {
        // texture changed -> start a new batch, create/reuse a bind group for that image
        current_batch = Some(batches.entry(...).insert(SpriteBatch { image_handle_id, range: index..index }));
    }
    // append this sprite's instance data to sprite_meta.sprite_instance_buffer,
    // extend the current batch's range
}
```

Because the phase is already sorted by depth, consecutive same-texture sprites at similar
depths naturally coalesce into one `SpriteBatch` — a single `draw_indexed` over an
instance range. Each sprite's world transform, color, and UV rect bake into one 80-byte
`SpriteInstance` (3×vec4 transposed affine + vec4 color + vec4 uv_offset_scale) appended to
a single dynamic `RawBufferVec<SpriteInstance>`, rewritten from scratch every frame. A
single static 6-index quad index buffer is shared by everything — no per-sprite CPU
vertex buffer exists; the vertex shader reconstructs quad corners from `vertex_index &
0b11` plus the instance's transform/size.

**Render**: `DrawSpriteBatch` sets the shared index/instance buffers once, issues
`pass.draw_indexed(0..6, 0, batch.range.clone())` — one draw call per batch (contiguous
same-texture run).

This is the classic description: 2D sprite batching minimizes draw calls by **coalescing
consecutive same-texture sprites into one instanced draw**, using a **CPU-rebuilt dynamic
instance buffer** each frame — quite different from 3D's indirect/GPU-driven approach.

## The Mesh2d path: convergent with 3D's GPU-driven batching

`Mesh2dRenderPlugin` is the 2D analog of `bevy_pbr`'s mesh pipeline, minus lighting. It
has `Mesh2dPipeline` (specialized per-view: MSAA, target format, compositing space,
tonemapping), `RenderMesh2dInstances`, and batches via the **same shared**
`batching::{no_gpu_preprocessing, gpu_preprocessing}` module 3D meshes use:
`batch_and_prepare_binned_render_phase::<Opaque2d/AlphaMask2d, Mesh2dPipeline>` and
`batch_and_prepare_sorted_render_phase::<Transparent2d, Mesh2dPipeline>` — the literal
generic mesh-batching entry points `bevy_pbr` also uses for 3D (see
[render-phases-and-batching](../architecture/render-phases-and-batching.md)).
`ColorMaterialPlugin` and the generic `Material2dPlugin` let arbitrary user 2D materials
plug into this batched pipeline — what `SpriteMaterial` (used by `SpriteMesh`) and the
tilemap-chunk material both build on. It deliberately lacks 3D's lighting/shadows/
clustering — "the mesh instancing/batching/material-binding machinery of 3D, without
lighting," as expected for a flat, unlit-by-default 2D renderer.

**Honest answer to "has 2D batching converged with 3D's GPU-driven approach in 0.19?"**:
partially, via the transitional `SpriteMesh` component, while the default `Sprite`
component still uses the legacy dynamic-vertex-buffer batcher. The two are kept
`ambiguous_with` each other explicitly in scheduling, confirming they're independent,
coexisting systems writing into the same phases.

## Registering into Transparent2d

`Transparent2d` is a `SortedPhaseItem`, sorted by `FloatOrd(z)` — alpha-blended sprites
must draw back-to-front for correct compositing (Bevy's 2D convention: Z is the layering
axis). `Opaque2d`/`AlphaMask2d` are binned instead, since opaque 2D draws don't need
per-pixel order. `SpritePipeline`'s depth-stencil state (`depth_write_enabled: false,
depth_compare: GreaterEqual`) confirms sprites read-but-never-write depth: they blend
against whatever opaque `Mesh2d` geometry already wrote, but never occlude each other via
depth — ordering is purely the sort key.

## Summary comparison

| | **Sprite** (classic) | **Mesh2d/Material2d** (incl. `SpriteMesh`) |
|---|---|---|
| Data source | `ExtractedSprites` rebuilt every frame | Persistent `Mesh`/`Material2d` assets + per-entity uniform |
| Batching | Bespoke CPU loop coalescing consecutive same-texture items | Generic `bevy_render::batching` — shared with 3D |
| Geometry | Implicit quad, reconstructed in shader | Real `Mesh` vertex/index buffers via `MeshAllocator` |
| Phase | `Transparent2d`, sorted by Z | `Opaque2d`/`AlphaMask2d` (binned) or `Transparent2d` (sorted), by material's `AlphaMode` |

## When to dive in

- Rendering many sprites and hitting draw-call limits → check whether sprites are
  actually batching (consecutive same-texture, similar-depth) before assuming a hard
  limit.
- Writing a custom 2D material → use `Material2d`/`Mesh2d` (the convergent path), not the
  legacy `Sprite` pipeline.
- Debugging 2D depth/layering issues → remember sprites never write depth; layering is
  purely the Z-sort key, not a depth test.

## Related
- [Render phases and batching](../architecture/render-phases-and-batching.md) — deeper: the GPU-driven batching machinery the Mesh2d path shares with 3D.
- [Gizmo rendering](./gizmo-rendering.md) — contrast: the unbatched immediate-mode path that also queues into `Transparent2d`, always sorted last.
- [UI rendering](./ui-rendering.md) — contrast: UI's adjacency-only atlas batching, the third answer to "many small draws".
- [Camera-driven scheduling](../core-pipeline/camera-driven-scheduling.md) — prerequisite: how the `Core2d` schedule that drains `Transparent2d` is chosen per camera.
