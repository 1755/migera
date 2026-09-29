---
title: "Gizmo rendering: immediate-mode debug drawing"
description: Bevy 0.19.1 draws gizmos as one per-frame CPU buffer per config group, uploaded as a RenderAsset and expanded to quads by vertex pulling. 2D gizmos sort on top; 3D gizmos are depth-tested against scene depth (depth_bias). Read when a gizmo is hidden, mis-sized, or you are writing debug drawing.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - rasterization
  - debugging
  - verification
updated: 2026-09-28
verified: 2026-09-28
sources:
  - bevy_gizmos-0.19.1/src/config.rs (GizmoConfig::depth_bias)
  - bevy_gizmos_render-0.19.1/src/pipeline_2d.rs, pipeline_3d.rs, lines.wgsl
aliases:
  - Gizmos system param
  - immediate-mode debug drawing
  - vertex pulling
  - GizmoConfig depth_bias
  - gizmos hidden behind mesh
---

# Gizmo rendering: immediate-mode debug drawing

## Why a wholly separate pipeline

Gizmo geometry (`gizmos.line(...)`, `gizmos.circle(...)`) is generated fresh every frame
from arbitrary user code — no persistent `Mesh` asset, no stable entity per line, no
meaningful way to apply mesh-instance batching or GPU culling (see
[sprite-and-mesh2d-rendering](./sprite-and-mesh2d-rendering.md) for how those work). The
natural model is "one big CPU-populated vertex buffer per gizmo *group* per frame,"
uploaded wholesale and drawn with a vertex-pulling technique to get thick lines (real
`line-list` topology can't do line width). Architecturally the opposite of mesh/sprite
batching: instead of *combining many small draws into one*, gizmos start as *one
already-large buffer* and need no batching step at all.

## GizmoAsset → GpuLineGizmo → draw

**Immediate-mode buffer building** (`bevy_gizmos`): `GizmoBuffer<Config, Clear>`
(accessed as the `Gizmos` system param) holds plain `Vec<Vec3>`/`Vec<LinearRgba>` for two
topologies — `list_positions`/`list_colors` (independent segments, pairs of points) and
`strip_positions`/`strip_colors` (connected polylines, using `Vec3::NAN` as a strip-break
sentinel). The GPU side lives in the separate `bevy_gizmos_render` crate.

Each frame this flushes into a `GizmoAsset` (a real `Asset`), treated as a `RenderAsset`
(see [render-assets](../resources-and-assets/render-assets.md)):

```rust
impl RenderAsset for GpuLineGizmo {
    fn prepare_asset(gizmo: GizmoAsset, ...) -> Result<Self, ...> {
        // 4 GPU buffers, uploaded wholesale from the CPU vecs each time the asset changes
        list_position_buffer, list_color_buffer, strip_position_buffer, strip_color_buffer
    }
}
```

This reuses Bevy's generic `RenderAssetPlugin<GpuLineGizmo>` extract/prepare machinery
(the same mechanism `Mesh`/`Image` use) rather than a bespoke per-frame extraction system
— but conceptually it's still "rebuild everything," since gizmo content changes every
frame in practice.

## Lines as quads via vertex pulling, not real line primitives

Despite the name "line list/strip," `lines.wgsl`'s pipelines actually issue `pass.draw(0..
6, 0..instances)` — six vertices (two triangles = a quad) per line segment, *instanced*
over segment count, not `PrimitiveTopology::LineList`. Each instance reads two consecutive
position-buffer offsets (`position_a` from `buffer[..len-1]`, `position_b` from
`buffer[item_size..]` — an overlapping-slice trick to get adjacent pairs without
duplicating data), and the vertex shader expands each segment into a screen-space quad of
`line_width` — how Bevy gets variable-width, anti-aliased lines that a native 1px
`LineList` couldn't provide. `DrawLineJointGizmo` similarly instances small fan/miter/
bevel geometry at each interior strip vertex to cover corner gaps, configured via
`GizmoLineJoint::{Miter, Round(resolution), Bevel, None}`.

## Queue: 2D gizmos always on top

`queue_line_and_joint_gizmos_2d` iterates all `(Entity, GizmoMeshConfig)` — one entity per
gizmo *config group*, spawned as `TemporaryRenderEntity`s created purely for the render
world (immediate-mode gizmos have no persistent main-world entity). It emits up to three
`Transparent2d` items per group (list-lines, strip-lines, strip-joints):

```rust
sort_key: FloatOrd(f32::INFINITY),   // always drawn last / on top
batch_range: 0..1,                   // no batching — single instanced draw
extracted_index: usize::MAX,         // sentinel: not a "real" ExtractedSprite
```

`sort_key = INFINITY` means 2D gizmos always sort to the end of `Transparent2d` — drawn on
top of all sprites/2D meshes, matching their role as a debug overlay.

## 2D vs 3D gizmo rendering: 3D gizmos are depth-tested

Two near-mirror pipelines (`pipeline_2d.rs`, `pipeline_3d.rs` in `bevy_gizmos_render`),
both built on shared `DrawLineGizmo`/`DrawLineJointGizmo`/`SetLineGizmoBindGroup` render
commands and the same `lines.wgsl`/`line_joints.wgsl` shaders. They differ in phase, view
bind group and **depth policy**:

- `pipeline_2d.rs` reuses `Mesh2dPipeline`'s view layout and queues into `Transparent2d`
  (gated on `SpriteRenderPlugin` being present). Layering is draw order plus the
  `INFINITY` sort key; `GizmoConfig::depth_bias` "has no effect" in 2D.
- `pipeline_3d.rs` queues into `Transparent3d` (with `distance: 0.`) against `bevy_pbr`'s
  mesh pipeline, and its `DepthStencilState` is `depth_write_enabled: true`,
  `depth_compare: Greater` on `CORE_3D_DEPTH_FORMAT` (reverse-Z). **3D gizmos are
  depth-tested against the scene**: any opaque mesh in front of a line hides it. The only
  knob is `GizmoConfig::depth_bias` in `[-1, 1]`: `0` = true depth, `-1` = always in
  front, small negatives fix z-fighting with a wireframe. `lines.wgsl` rescales
  `clip.z` by it. It also handles perspective-correct width
  (`GizmoMeshConfig::line_perspective`), so 3D lines can shrink with distance whereas 2D
  lines stay screen-space-constant.

Until 2026-09-28 this note claimed both pipelines share sprites' `depth_write_enabled:
false` policy; the 0.19.1 source shows that is true only of 2D.

## When to dive in

- A 3D gizmo is missing or only partly visible → it is depth-occluded by scene geometry,
  not broken. Hide the occluding mesh or set a negative `depth_bias` (see the
  character-animation lesson in Related).
- Debugging why a 2D gizmo doesn't render on top of geometry as expected → check that
  `SpriteRenderPlugin`/`bevy_pbr`'s mesh pipeline actually loaded before
  `GizmoRenderPlugin` (it warns loudly if not).
- Writing custom immediate-mode debug visualization → follow the `GizmoBuffer`/
  `GizmoAsset` pattern rather than spawning real entities per frame.
- Wondering why gizmo lines have consistent screen-space width in 2D but shrink with
  distance in 3D → that's `GizmoMeshConfig::line_perspective`, intentional per-dimension
  behavior, not a bug.

## Related
- [Gizmos need --show-real-mesh off](../../character-animation/animation-core/gizmos-need-show-real-mesh-off.md) — applies: the migera trap caused by 3D gizmo depth testing; read before using `character_gallery --gizmos on` as a skeleton view.
- [RenderAsset lifecycle](../resources-and-assets/render-assets.md) — prerequisite: the extract/prepare machinery `GizmoAsset` → `GpuLineGizmo` reuses.
- [2D rendering: Sprite and Mesh2d](./sprite-and-mesh2d-rendering.md) — contrast: the batching paths gizmos deliberately skip, and the `Transparent2d` phase they share.
- [Render phases and batching](../architecture/render-phases-and-batching.md) — deeper: how `Transparent2d`/`Transparent3d` items become draw calls.
- [bevy_light: light components, atmosphere, gizmos](../pbr-and-lighting/light-components-and-atmosphere.md) — example: built-in light debug gizmos (`ShowLightGizmo`).
