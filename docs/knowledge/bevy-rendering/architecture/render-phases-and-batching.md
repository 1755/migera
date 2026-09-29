---
title: Render phases, PhaseItems, and GPU-driven batching
description: Bevy 0.19.1 separates what to draw (render phases of binned or sorted PhaseItems, drawn through composed RenderCommands) from when (pass systems), and 0.19 moved batching GPU-side with bin unpacking, sparse uploads and change-list DirtySpecializations. Read before a custom phase item or when draw-call counts regress.
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
  - bevy_render-0.19.1/src/render_phase/
  - bevy-0.19.1/examples/shader_advanced/custom_phase_item.rs, custom_render_phase.rs
aliases:
  - PhaseItem
  - BinnedPhaseItem
  - SortedPhaseItem
  - RenderCommand
  - draw call batching
  - GPU-driven rendering
---

# Render phases, PhaseItems, and GPU-driven batching

## Why this exists

A pass system (e.g. `main_opaque_pass_3d`, see
[render-graph-as-systems](./render-graph-as-systems.md)) needs to know *what* to draw and
in *what order*, without hardcoding knowledge of every possible drawable thing (meshes,
sprites, gizmos, UI). Bevy solves this with **render phases**: a phase collects abstract
"phase items" during the `Queue`/`Prepare` steps, then a pass simply calls
`phase.render(&mut render_pass, world, view_entity)` and the phase handles walking its
items and issuing draw calls, regardless of what they represent.

Splitting *what to draw* (phases, populated over many systems from many crates) from
*when to draw it* (passes, one per logical stage of the pipeline) is what lets 3D meshes,
2D sprites, gizmos, and UI all plug into the same pass infrastructure independently.

## PhaseItem and its two flavors

`PhaseItem` (`bevy_render::render_phase`) is the core trait: every item knows its
`entity()`, `main_entity()`, `draw_function()` (a `DrawFunctionId`, see below),
`batch_range()`, and `extra_index()` (dynamic uniform offset or indirect-draw index).

Two sub-traits describe how items are ordered:

- **`BinnedPhaseItem`** — items are grouped into bins keyed by `BinKey` (ideally ordered
  to minimize state changes: pipeline id, then draw function id, then mesh id, then bind
  group ids...). Bins are *not* internally sorted — used when draw order doesn't matter,
  e.g. `Opaque3d`, `AlphaMask3d`. This is the common case for opaque geometry, since
  depth testing makes draw order irrelevant for correctness, only for GPU efficiency.
- **`SortedPhaseItem`** — items go into a single list sorted by `SortKey`, used when order
  *is* semantically required, e.g. `Transparent3d` (back-to-front for correct alpha
  blending via the painter's algorithm).

A third trait, `CachedRenderPipelinePhaseItem`, marks items that carry a
`CachedRenderPipelineId` so the `SetItemPipeline` render command can bind the right
pipeline automatically.

`ViewBinnedRenderPhases<BPI>` / `ViewSortedRenderPhases<SPI>` are resources mapping
`RetainedViewEntity → BinnedRenderPhase<BPI>` (or sorted equivalent) — one phase instance
per view, since each camera (or shadow map) needs its own list of visible, orderable
items. `RetainedViewEntity` (not a raw `Entity`) is the key precisely because it's stable
across frames — see
[camera-and-view-system](../scene-and-views/camera-and-view-system.md) — which is what
lets bins/sorted lists be **persisted and incrementally updated** across frames instead of
rebuilt from scratch every frame (see "change lists," below).

## Draw functions and RenderCommand composition

A `Draw<P: PhaseItem>` impl is "the thing that actually issues wgpu draw calls for one
phase item." Writing one from scratch means manually fetching ECS data and calling
`TrackedRenderPass` methods. In practice, almost nothing does this directly — instead,
`RenderCommand<P>` provides small, composable, stateless units of pipeline setup:

```rust
pub type DrawMaterial<M> = (
    SetItemPipeline,
    SetMeshViewBindGroup<0>,
    SetMeshBindGroup<2>,
    SetMaterialBindGroup<M, 3>,
    DrawMesh,
);
```

A tuple of `RenderCommand`s automatically implements `Draw<P>` (via
`AddRenderCommand::add_render_command`), running each command in sequence and
short-circuiting on `RenderCommandResult::Skip`/`Failure`. Each command declares its own
`Param` (`SystemParam`, typically resources), `ViewQuery` (data from the view entity), and
`ItemQuery` (data from the item's render-world entity, which may be `None` — many
drawable entities, e.g. simple meshes, are never given a full render-world mirror; see
[entity-sync-and-extraction-patterns](./entity-sync-and-extraction-patterns.md)).

This is the standard extension point: adding a custom material or custom phase item
usually means writing a handful of small `RenderCommand`s and composing them into a tuple,
rather than implementing `Draw` by hand.

## GPU-driven batching (the 0.19 headline performance work)

Historically, batching (merging multiple draw calls for objects sharing a pipeline/mesh
into fewer, larger draw calls) required significant CPU-side bookkeeping every frame. In
0.19, Bevy moved much of this work to the GPU:

- **Bin unpacking on the GPU** — the CPU still decides *which* entities go in which bin,
  but the work of turning that into per-instance GPU data (`PreprocessWorkItem`s) that
  feeds indirect draw calls happens in a compute shader, not on the CPU.
  `RenderMultidrawableBatchSet<BPI>` (`render_phase/mod.rs`) documents this data
  structure with an ASCII diagram: bins map to indirect-parameter offsets, and a "binned
  mesh instance buffer" on the GPU is what the bin-unpacking shader consumes and produces.
- **Sparse mesh uniform uploads** — only mesh uniforms that actually changed since last
  frame are re-uploaded, a large win for mostly-static scenes.
- **GPU light clustering** — clustered forward light assignment (see
  [pbr-and-lighting](../pbr-and-lighting/INDEX.md)) moved from CPU to GPU, reported ~20x
  faster on the relevant benchmark.
- **Indirect drawing / multi-draw-indirect** — when the backend supports it
  (`multi_draw_indirect_count`), whole batch sets can be issued as a single indirect draw
  call, with the GPU itself deciding (via occlusion/frustum culling compute passes) how
  many of the batch's instances actually need rasterizing. `PhaseItemExtraIndex::
  IndirectParametersIndex { range, batch_set_index }` is the plumbing that connects a
  `PhaseItem` back to its slot in the indirect parameters buffer.

`bevy_render::batching::gpu_preprocessing` is the entry point for this machinery;
`bevy_render::batching::no_gpu_preprocessing` is the CPU-side fallback path used on
backends without compute/storage-buffer support (e.g. some WebGL2 configurations).

## Change lists: avoiding full re-iteration

Before 0.19, render phases and pipeline specialization had to scan **every visible
entity** each frame to detect what changed (new entity, removed entity, needs
re-specialization). 0.19 replaced this with explicit "change lists" so only entities that
actually changed are touched:

- `SortedRenderPhase` moved its backing storage from `Vec` to `IndexMap`, so entities can
  be incrementally added (`add_retained`) or removed without rebuilding the whole list
  each frame (`add_transient` remains for genuinely one-off/temporary items).
- `DirtySpecializations` (`bevy_render::camera`) tracks exactly which main-world entities
  need (re-)specialization or removal this frame, keyed by `RetainedViewEntity`, with
  iterator methods (`iter_to_queue`, `iter_to_dequeue`, `iter_to_despecialize`) that a
  phase's `queue_*`/`specialize_*` systems call instead of iterating all visible entities.
  See [camera-and-view-system](../scene-and-views/camera-and-view-system.md) for the full
  mechanics, including the "wipe this view's specializations entirely" escape hatch used
  when something view-wide changes (e.g. MSAA sample count).

If you are implementing a **custom render phase or specialized pipeline**, this is the
pattern to follow — do not write a system that iterates `VisibleEntities`/
`RenderVisibleEntities` from scratch every frame; hook into the dirty/change-list
machinery instead, or you will reintroduce the exact bottleneck this rework removed.

## When to dive in

- Writing a custom `Material` or mesh-like renderable → you'll compose `RenderCommand`s
  into a `Draw` impl; you rarely touch `PhaseItem`/batching internals directly (the
  `Material`/`Mesh` machinery in
  [materials-and-shaders](../materials-and-shaders/INDEX.md) does it for you).
- Writing a fully custom phase item (not mesh- or material-based) → read
  `bevy-0.19.1/examples/shader_advanced/custom_phase_item.rs` and
  `custom_render_phase.rs` (vendored alongside the crate sources) as worked examples, and
  implement the change-list pattern from day one.
- Debugging draw-call counts or batching regressions → check whether GPU preprocessing is
  active for your backend (`no_gpu_preprocessing` is the fallback, with different
  performance characteristics) before assuming a batching bug.

## Related
- [The render graph is gone](./render-graph-as-systems.md) — prerequisite: the pass systems that call `phase.render(...)`.
- [Camera and the view system](../scene-and-views/camera-and-view-system.md) — deeper: the `DirtySpecializations` change-list mechanism in detail.
- [GPU buffer types and allocation](../resources-and-assets/gpu-buffers-and-allocation.md) — deeper: the buffers GPU-driven batching writes into.
- [PipelineCache and pipeline specialization](../resources-and-assets/pipeline-cache-and-specialization.md) — deeper: how phase items get their specialized pipelines.
- [2D rendering: Sprite and Mesh2d](../2d-and-ui/sprite-and-mesh2d-rendering.md) — example: CPU batching versus the GPU-driven path side by side.
- [Native integration with Bevy's render pipeline](../../sdf-3dgs-bevy-integration/architecture/bevy-pipeline-integration.md) — applies: a custom `Splat3d` phase item built on this machinery.
