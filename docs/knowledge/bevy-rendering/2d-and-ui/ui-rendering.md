---
title: "UI rendering: from Node to pixels"
description: Bevy 0.19.1 UI (bevy_ui layout + bevy_ui_render) turns Node components into SDF-shaded quads for rounded rects and borders, queues them in TransparentUi sorted by stack_z_offsets, batches only adjacent same-texture items, and shapes text via Parley. Read before custom UI rendering or when UI layers or clips wrong.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - rasterization
  - sdf
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_ui-0.19.1/src, bevy_ui_render-0.19.1/src
aliases:
  - bevy_ui_render
  - TransparentUi
  - box shadow
  - UI gradient
  - stack_z_offsets
---

# UI rendering: from Node to pixels

Contents: [Architecture](#architecture-and-crate-split) · [Extraction](#extraction-node--extracteduinode) · [Geometry via SDF](#geometry-rectangles-rounded-corners-borders-via-sdf) · [Batching](#batching-queueing-and-transparentui) · [Camera targeting](#camera-targeting) · [Text](#text-rendering) · [Shadows and gradients](#box-shadows-and-gradients) · [When to dive in](#when-to-dive-in) · [Related](#related)

## Architecture and crate split

`bevy_ui` owns layout only — it computes `ComputedNode` (resolved size, border widths,
border radii via Taffy) and world-space transforms (`UiGlobalTransform`). It has zero GPU
knowledge. `bevy_ui_render` is a plugin (`UiRenderPlugin`) living entirely inside
`RenderApp`, turning computed layout components into vertex buffers and draw calls —
mirroring the sprite/mesh2d split (see
[sprite-and-mesh2d-rendering](./sprite-and-mesh2d-rendering.md)).

Consistent with 0.19's schedule-based renderer (see
[camera-driven-scheduling](../core-pipeline/camera-driven-scheduling.md)), UI has no
node-based graph — `UiRenderPlugin` inserts a plain ECS system, `ui_pass`, directly into
`Core2d` and `Core3d`:

```rust
.add_systems(Core2d, ui_pass.after(Core2dSystems::PostProcess).before(upscaling))
.add_systems(Core3d, ui_pass.after(Core3dSystems::PostProcess).before(upscaling))
```

UI is **not** a separate schedule — it's an overlay pass ordered after the main scene's
post-processing and before the final upscaling blit, for both 2D and 3D cameras.

## Extraction: Node → ExtractedUiNode

Each visual concern (background, image, border, text, gradient, box shadow) has its own
`ExtractSchedule` system reading main-world components and pushing `ExtractedUiNode`/
`ExtractedGlyph` records into `ExtractedUiNodes`. Splitting by concern lets each feature
be self-contained with its own extraction/queue/prepare systems.

- `extract_uinode_background_colors` reads `ComputedNode`, `UiGlobalTransform`,
  `BackgroundColor`, pushes `ExtractedUiItem::Node { rect, border, border_radius,
  node_type: NodeType::Rect, .. }`, skipping fully-transparent/empty nodes early.
- `extract_uinode_borders` groups border edges sharing the same color into one draw via
  bitflags (`BORDER_LEFT | BORDER_TOP | ...`) so contiguous same-color edges don't
  needlessly explode into separate quads.
- Extraction systems are ordered via `RenderUiSystems` (backgrounds → images → texture-
  slices → borders → text-backgrounds → text-shadows → text → cursor → debug), which
  indirectly defines default paint order *within* a stack index.
- Each extraction spawns a `TemporaryRenderEntity` per visual, stamped with `z_order =
  stack_index + stack_z_offsets::X`:
  ```rust
  pub mod stack_z_offsets {
      pub const BOX_SHADOW: f32 = -0.1;
      pub const BACKGROUND_COLOR: f32 = 0.0;
      pub const BORDER: f32 = 0.01;
      pub const GRADIENT: f32 = 0.02;
      pub const IMAGE: f32 = 0.04;
      pub const TEXT: f32 = 0.06;
      ...
  }
  ```
  This is what lets a background, its border, and its text — all belonging to the same UI
  stacking index — still paint in correct order, since the fractional offsets never cross
  into a sibling/parent's integer stack index.

Every extraction query carries `&ComputedUiTargetCamera`, resolved via `UiCameraMap` into
the render-world camera entity, stored as `extracted_camera_entity` — this lets multiple
cameras/windows have independent, non-interleaved UI phases.

## Geometry: rectangles, rounded corners, borders via SDF

Rounded corners and variable-width borders are notoriously fiddly to tessellate exactly.
Bevy sidesteps geometry generation entirely: every UI visual is a single **axis-aligned
quad** (still transformable via an arbitrary `Affine2`), and the rounded-rect/border shape
is computed analytically in the fragment shader using a signed-distance-field (SDF).

`prepare_uinodes` builds vertex/index buffers: takes the unit-quad corners, transforms by
the node's `Affine2` × `rect_size`, applies CPU-side rectangular clipping
(`CalculatedClip`, only correct for unrotated/unscaled clip rects — a documented
limitation), culls nodes fully outside their clip rect (when not rotated), and emits 4
vertices per quad with:

```rust
struct UiVertex {
    position: [f32; 3], uv: [f32; 2], color: [f32; 4],
    flags: u32,               // TEXTURED, BORDER_LEFT/TOP/..., INVERT, etc.
    radius: [f32; 4],         // per-corner
    border: [f32; 4],         // per-edge thickness
    size: [f32; 2],
    point: [f32; 2],          // position relative to node center
}
```

In `ui.wgsl`, `sd_rounded_box` computes the signed distance to a rounded rectangle
boundary (negative inside); `sd_inset_rounded_box` computes the same for the border's
*inner* edge. `draw_uinode_border` takes `max(external_distance, -internal_distance)` —
inside the outer rect AND outside the inner rect = border color — while
`draw_uinode_background` uses only the internal distance, so backgrounds never bleed
under the border. `nearest_border_active` resolves which edge "owns" a corner pixel when
adjacent borders differ in color/width. Anti-aliasing (`UiAntiAlias`, default on) uses
`saturate(0.5 - distance)` rather than MSAA. `NodeType::Inverted` (for drawing outside a
shape) flips the sign of the internal distance.

## Batching, queueing, and TransparentUi

UI needs strict back-to-front alpha-blended painter's-algorithm ordering by UI stacking
context — it cannot be depth-tested/reordered like opaque 3D geometry — so it gets its own
`SortedRenderPhase`, `TransparentUi`, separate from `Transparent2d`/`Transparent3d`.

- `queue_uinodes` (Queue stage) resolves each node's render-world camera/view, specializes
  `UiPipeline`, pushes a `TransparentUi` item with `sort_key = FloatOrd(z_order)`.
- `sort_phase_system::<TransparentUi>` sorts strictly by that float key — enforcing
  correct paint order across an entire node tree via the global `ComputedStackIndex` +
  sub-1.0 `stack_z_offsets`.
- `prepare_uinodes` (PrepareBindGroups) does **texture-atlas batching**: walks the sorted
  phase items in order and merges *consecutive* items sharing the same `image:
  AssetId<Image>` into a `UiBatch { range, image }`, reusing/growing one shared
  `RawBufferVec<UiVertex>`. A batch breaks only when the image changes (the untextured
  "default" image is compatible with any texture, so solid-color rects sit in a textured
  batch without forcing a break). Batching is **order-preserving and adjacency-only** —
  correctness of layering wins over batch count, unlike the classic sprite batcher's
  freedom to reorder by depth (see
  [sprite-and-mesh2d-rendering](./sprite-and-mesh2d-rendering.md)).
- Text glyphs are pre-grouped into contiguous runs sharing the same atlas texture at
  extraction time, so a text run naturally becomes one or more image-batched quads.
- **Box shadows and gradients are separate render commands/pipelines** enqueuing into the
  *same* `TransparentUi` phase (so everything interleaves correctly by `sort_key`), but
  with their own vertex formats/shaders/buffers — not batched into the base UI buffer.

`ui_pass` opens one `wgpu` render pass per camera with `depth_stencil_attachment: None`
(UI never depth-tests), writing into the same color target the main pass just rendered to
— the "overlay after the main scene" behavior.

## Camera targeting

Root nodes may carry `UiTargetCamera(Entity)` explicitly; if absent,
`DefaultUiCamera` resolves one (prefer a camera tagged `IsDefaultUiCamera`, else the
highest-`order` camera targeting the primary window), written as
`ComputedUiTargetCamera` and propagated down the UI tree. On the render side,
`extract_ui_camera_view` creates a **second `ExtractedView`** per active camera — a
dedicated orthographic UI subview (`UI_CAMERA_SUBVIEW = 1`, avoiding collision with the
main view's subview 0 in `RetainedViewEntity` — see
[camera-and-view-system](../scene-and-views/camera-and-view-system.md)).

## Text rendering

`bevy_text` uses **Parley** (Linebender's layout engine) for shaping/line-breaking and
**Swash** for glyph rasterization/hinting — a departure from the historically common
`cosmic-text`/`ab_glyph` stack. `TextPipeline::update_buffer` builds a Parley
`RangedBuilder` per text block; `update_text_layout_info` walks the resulting glyph runs,
rasterizing new glyphs with Swash and packing them into a shared atlas texture
(`FontAtlas`'s `DynamicTextureAtlasBuilder`, a shelf/guillotine-style packer). Each glyph
becomes a `PositionedGlyph` on `TextLayoutInfo`. `bevy_ui_render::extract_text_sections`
walks these glyphs, pushing `ExtractedGlyph`s and closing an `ExtractedUiItem::Glyphs {
range }` node whenever consecutive glyphs' atlas texture differs — the atlas-batching
boundary. Text decorations (highlight, strikethrough, underline, cursor) extract as
ordinary untextured rects positioned via `stack_z_offsets::TEXT_*`.

## Box shadows and gradients

Both are separate plugins (`BoxShadowPlugin`, `GradientPlugin`), following the identical
extract → queue → prepare → draw-command pattern, with own WGSL/vertex layouts. Box
shadows expand the quad bounds by `6 * blur_radius` and render an SDF-based approximation
(no real blur pass — cheap, resolution-independent). Gradients support Linear/Radial/
Conic with multiple interpolation color spaces (sRGB, linear, Oklab, Oklch, HSL, HSV),
rendering multi-stop gradients as several thin segment quads.

## When to dive in

- Building a custom UI rendering feature (shadow/gradient plugins are the template): (1)
  extraction system reading your component alongside `ComputedNode`/`UiGlobalTransform`/
  `ComputedUiTargetCamera`; (2) a `queue_*` system pushing `TransparentUi` items with a
  `stack_z_offsets`-style sort key; (3) a `prepare_*` system building vertex/index
  buffers; (4) a `RenderCommand` tuple registered via `add_render_command::<
  TransparentUi, _>()`. No render-graph node needed — `ui_pass` already drains the phase.
- Debugging UI layering bugs (wrong element on top) → check `stack_z_offsets` and
  `ComputedStackIndex`, not draw order in code.
- Debugging UI clipping glitches on rotated/scaled elements → `CalculatedClip` is
  documented to only work correctly for unrotated/unscaled clip rects.

## Related
- [2D rendering: Sprite and Mesh2d](./sprite-and-mesh2d-rendering.md) — contrast: the same data-crate/render-crate split, with different batching strategies.
- [Camera-driven scheduling](../core-pipeline/camera-driven-scheduling.md) — prerequisite: where `ui_pass` sits in `Core2d`/`Core3d` relative to post-processing and upscaling.
- [Render phases and batching](../architecture/render-phases-and-batching.md) — deeper: `SortedPhaseItem` and `RenderCommand` composition that `TransparentUi` items use.
- [Entity sync and extraction patterns](../architecture/entity-sync-and-extraction-patterns.md) — prerequisite: the extraction step that turns `Node` into `ExtractedUiNode`.
