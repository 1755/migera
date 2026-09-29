---
title: Camera and the view system
description: Covers Bevy 0.19.1's render-agnostic bevy_camera (Camera, Projection, RenderTarget), camera extraction, the ViewTarget post-process ping-pong, ViewUniform, RetainedViewEntity for non-camera views, frustum culling, and DirtySpecializations change lists. Read before multi-camera, post-process or custom-phase work.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - culling
  - post-processing
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_camera-0.19.1/src
  - bevy_render-0.19.1/src/view/
aliases:
  - ViewTarget
  - post_process_write
  - ViewUniform
  - RetainedViewEntity
  - DirtySpecializations
  - frustum culling
---

# Camera and the view system

Contents: [bevy_camera](#bevy_camera-a-standalone-render-api-agnostic-crate) · [DirtySpecializations](#dirtyspecializations-change-lists-instead-of-full-re-iteration) · [Extraction](#extraction-into-the-render-world) · [ViewTarget ping-pong](#viewtarget-the-post-process-ping-pong-mechanism) · [ViewUniform](#viewuniform--the-per-view-gpu-uniform-block) · [RetainedViewEntity](#retainedviewentity-stable-identity-for-non-camera-views) · [Frustum culling](#frustum-culling-visibility-determination) · [Key chain](#summary-of-the-key-chain) · [When to dive in](#when-to-dive-in) · [Related](#related)

## `bevy_camera`: a standalone, render-API-agnostic crate

`bevy_camera` depends only on ECS/math/transform/window/image/mesh crates — no
`bevy_render`, no `wgpu`. This is why it exists: the *logical* concept of a camera (where
it is, what it projects, what's visible to it) is decoupled from the GPU backend, letting
headless tools, custom renderers, or crates that need camera math reuse it without wgpu.
`bevy_render`/`bevy_pbr` build the actual rendering machinery on top of it.

### `Camera` and required components

`Camera` (`camera.rs`) is plain data: `viewport`, `order`, `is_active`, `computed:
ComputedCameraValues` (cached projection matrix + target info), `output_mode`,
`clear_color`, `invert_culling`, `sub_camera_view`. It uses `#[require(Frustum,
CameraMainTextureUsages, VisibleEntities, Transform, Visibility, RenderTarget)]` — Bevy's
required-components mechanism auto-inserts these when a bare `Camera` is spawned. `Camera`
does **not** require a render-graph component; an `on_add` hook warns if you spawn one
without `Camera2d`/`Camera3d` (or a manual schedule), since without one nothing renders.

### `Projection`: Perspective vs Orthographic

The `CameraProjection` trait (`get_clip_from_view`, `update(width, height)`, `far`,
`compute_frustum`) is implemented by `PerspectiveProjection` and `OrthographicProjection`,
unified behind the `Projection` enum component (plus a `CustomProjection` escape hatch for
user-defined projections). `Camera3d` requires `Projection` (default perspective);
`Camera2d` requires `Projection::Orthographic(OrthographicProjection::default_2d())`.
`CameraProjectionPlugin` recomputes `Frustum` (used for culling) in `PostUpdate` whenever
`GlobalTransform`/`Projection` changes.

### `CameraRenderGraph`: it moved, and it changed shape

**Key finding**: `CameraRenderGraph` is no longer defined in `bevy_camera` — it lives in
`bevy_render::camera`:

```rust
#[derive(Component, Debug, Deref, DerefMut, Reflect, Clone)]
pub struct CameraRenderGraph(pub InternedScheduleLabel);
```

With 0.19's [render-graph-as-systems](../architecture/render-graph-as-systems.md) change,
the old string/`RenderLabelId`-keyed render graph node no longer exists.
`CameraRenderGraph` now wraps an `InternedScheduleLabel` — it names a **Bevy schedule**
(like `Core3d`) run for that camera, not a graph-node identifier. `Camera2d`/`Camera3d`
(in `bevy_core_pipeline`) attach the appropriate schedule label. This is why the type
still lives conceptually "near" cameras but had to move into `bevy_render`: schedules are
an ECS/app concept `bevy_render` owns, while `bevy_camera` intentionally has no rendering-
schedule knowledge.

### `RenderTarget` and viewport handling

`RenderTarget` is an enum: `Window(WindowRef)`, `Image(ImageRenderTarget)`, `TextureView(
ManualTextureViewHandle)`, or `None { size }` (useful for prepass-only cameras).
`Camera::normalize()` resolves ambiguous references (`WindowRef::Primary`) into
`NormalizedRenderTarget` (`Eq + Hash + Ord`, safe as a map key for grouping cameras by
target). `Viewport` (physical position/size/depth range) lets multiple cameras share a
target for split-screen/minimaps.

### `visibility/` module

Three visibility components form a pipeline:

- **`Visibility`** (`Inherited | Hidden | Visible`) — user-authored, propagates down the
  hierarchy.
- **`InheritedVisibility`** — computed in `PostUpdate` by `visibility_propagate_system`,
  walking `ChildOf`/`Children`.
- **`ViewVisibility`** — "is this entity visible to *any* view (camera or shadow-casting
  light) this frame." Bit-packed to track current *and previous* frame state so
  `set_visible()` only triggers change detection on a hidden→visible transition, not every
  frame the object stays visible; `mark_newly_hidden_entities_invisible` catches the
  reverse transition.

`VisibleEntities` is per-camera: a `TypeIdMap<Vec<Entity>>` keyed by "visibility class"
(e.g. `Mesh3d`, `Sprite`), populated by frustum tests against each entity's
`Aabb`/`Sphere`. `RenderLayers` is a growable bitset letting cameras and entities opt into
layer masks via `intersects()`.

## `DirtySpecializations`: change lists instead of full re-iteration

**Correction to a common assumption**: `DirtySpecializations` is defined in
`bevy_render::camera`, not in `bevy_camera`. It's a render-world type.

**The problem it solved**: before 0.19, render phases had to iterate *every visible
entity* each frame to detect which needed pipeline re-specialization — an O(visible
entities) scan even when nothing changed. This was a real bottleneck at scale (PR #22966).

```rust
pub struct DirtySpecializations {
    pub changed_renderables: MainEntityHashSet,   // need re-specialization
    pub removed_renderables: MainEntityHashSet,    // need despecialization
    pub views: HashSet<RetainedViewEntity>,        // views needing a full wipe
}
```

Instead of scanning everything, purpose-built iterators (`iter_to_dequeue`,
`iter_to_queue`/`iter_to_specialize`, `iter_to_despecialize`) let a phase's specialize/
queue systems look only at what changed since last frame:

```rust
for &main_entity in dirty_specializations
    .iter_to_dequeue(view.retained_view_entity, render_visible_mesh_entities)
{
    opaque_phase.remove(main_entity);
}
```

If a *view itself* changes in a way invalidating all its cached specializations (e.g. MSAA
sample count changed), its `RetainedViewEntity` goes into `views`, and
`must_wipe_specializations_for_view()` falls back to full re-specialization for just that
view — a controlled escape hatch, not a global re-scan. Cleared every frame; a companion
`DirtyWireframeSpecializations` exists since a mesh's normal and wireframe-overlay
pipelines specialize independently. `SortedRenderPhase` moved from `Vec` to `IndexMap` in
the same rework, so entities can be incrementally added (`add_retained`)/removed instead
of rebuilt every frame.

**When you need this**: only if writing a custom render phase or specialized pipeline
(see `bevy-0.19.1/examples/shader_advanced/specialized_mesh_pipeline.rs`). Ordinary
`Mesh3d`/material users never touch it.

## Extraction into the render world

`extract_cameras` (`ExtractSchedule`) copies active `Camera` data into render-world
components on the corresponding `RenderEntity` each frame:

```rust
pub struct ExtractedCamera {
    pub target: Option<NormalizedRenderTarget>,
    pub physical_viewport_size: Option<UVec2>,
    pub schedule: InternedScheduleLabel,   // from CameraRenderGraph
    pub order: isize,
    pub hdr: bool,
    ...
}
```

Alongside `ExtractedCamera`, it inserts `ExtractedView` (matrices, viewport, target
format) and `RenderVisibleEntities` (remapped from main-world `Entity` to render-world
`RenderEntity`). Cameras with `is_active == false` or a zero-sized target have their
extracted components removed instead — this is how disabling a camera actually stops
downstream rendering work. The extracted camera's `RetainedViewEntity` is built here too:
`RetainedViewEntity::new(main_entity.into(), None, 0)` — subview index 0, no auxiliary
entity, since a camera is a single, non-cascaded view (contrast with shadow-casting
lights, below).

`sort_cameras` (`RenderSystems::CreateViews`) sorts all extracted cameras by `(order,
target)` into the `SortedCameras` resource, assigning each a `sorted_camera_index_for_
target` — its position among cameras sharing that target. This is what lets multiple
cameras layer correctly onto one window and lets later passes know "am I first to write
this target" (clear vs. load ops). It also detects/warns on `(order, target)` ambiguities.

## `ViewTarget`: the post-process ping-pong mechanism

`ViewTarget` doesn't hold one texture — it holds **two** ("main_texture_a"/"b") plus a
shared atomic flip flag:

```rust
pub struct ViewTarget {
    main_textures: MainTargetTextures,       // { a, b }
    main_texture: Arc<AtomicUsize>,          // 0 = a is current, 1 = b is current
    out_texture: Option<OutputColorAttachment>,
    ...
}
```

`prepare_view_targets` allocates both textures once per unique `(target, usage, format,
msaa)` key, reusing the same `Arc<AtomicUsize>` flip-flag across frames for a given render
target "to ensure post process writes persist through msaa writeback."

```rust
pub fn post_process_write(&self) -> PostProcessWrite<'_> {
    let old_is_a = self.main_texture.fetch_xor(1, Ordering::SeqCst);
    // whichever was "current" becomes `source`; the other becomes `destination`
}
```

Calling `post_process_write()` atomically flips which texture is "current" and returns
`{ source, destination }`. The contract: render `source → destination`; after the call,
`main_texture_view()` transparently points at what was just written. This is what lets an
arbitrary sequence of post-process effects (bloom, tonemapping, FXAA, etc.) chain without
each one knowing about the others — see
[post-processing](../post-processing/INDEX.md) for the full worked pattern.
`out_texture()` is the actual swapchain/target texture (distinct from the ping-pong pair),
written at the end of the pipeline.

`Msaa` (`Off=1, Sample2=2, Sample4=4 (default), Sample8=8`) both configures the MSAA
intermediate texture (allocated when `samples() > 1`) and participates in the key used to
bucket/reuse ping-pong textures.

## `ViewUniform` — the per-view GPU uniform block

`ShaderType`-derived, uploadable per-view struct: `clip_from_world`, `world_from_view`,
`view_from_world`, `clip_from_view`, `view_from_clip`, `world_position`, `exposure`,
`viewport`, `main_pass_viewport`, the 6-plane `frustum` (for GPU-side culling in shaders/
compute), `lod_view_world_position`, `color_grading`, `mip_bias`, `frame_count`.
`ViewUniforms` wraps a `DynamicUniformBuffer<ViewUniform>`; `prepare_view_uniforms` writes
one entry per view per frame and stores the byte offset in `ViewUniformOffset` — this
offset is what a render pass binds to select "this view's" uniform block from the shared
buffer. A custom shader reading `view.clip_from_world` in WGSL is reading this struct's
mirror in `view.wgsl`.

## `RetainedViewEntity`: stable identity for non-camera views

```rust
pub struct RetainedViewEntity {
    pub main_entity: MainEntity,
    pub auxiliary_entity: MainEntity,  // placeholder if unused
    pub subview_index: u32,
}
```

**Why it exists**: render-world entities are recreated/rearranged each frame (extraction
doesn't guarantee stable `Entity` IDs across frames), so a plain `Entity` can't serve as a
durable cache key. But many render-side caches — dirty-specialization bookkeeping, shadow
map allocation, per-view pipeline caches — need to say "this is the *same* view as last
frame." A `Camera` maps 1:1 to a single view (subview 0, no auxiliary). A shadow-casting
light is not a camera: a point light needs 6 views (cubemap faces), a directional light
needs N views (cascades) — each sub-view needs its own stable identity via
`subview_index`. `auxiliary_entity` additionally disambiguates directional-light cascades
*per camera* in multi-camera scenes, since each camera can have its own cascade set for
the same light.

This mechanism is what `DirtySpecializations.views` and shadow-map/pipeline caches key on
— see [pbr-and-lighting](../pbr-and-lighting/INDEX.md) for how shadow views use it.

## Frustum culling (visibility determination)

Two systems drive `ViewVisibility` (`VisibilitySystems::CheckVisibility`):
`check_visibility_cpu_culling` (sphere test, then OBB-vs-frustum per entity per view,
using each view's `Frustum`) and `check_visibility_gpu_culling` (entities with
`NoCpuCulling` mirror `InheritedVisibility` and defer to a GPU compute pass). On the
render side, `collect_visible_cpu_culled_entities` builds `RenderVisibleEntities` from the
main-world `VisibleEntities`, keeping lists sorted — sortedness is what makes
`DirtySpecializations`'s binary-search-based lookups valid.

## Summary of the key chain

`Camera` → requires `Camera2d`/`Camera3d` which attach `CameraRenderGraph(
InternedScheduleLabel)` → `extract_cameras` builds `ExtractedCamera` + `ExtractedView`
with a fresh `RetainedViewEntity` → `sort_cameras` assigns per-target ordering into
`SortedCameras` → `prepare_view_targets` allocates the `ViewTarget` ping-pong pair →
post-process systems call `post_process_write()` repeatedly to chain effects →
`prepare_view_uniforms` uploads the per-view `ViewUniform` block → throughout,
`DirtySpecializations` keyed by `RetainedViewEntity` lets phases update incrementally.

## When to dive in

- Building split-screen/render-to-texture setups → `RenderTarget`/`Viewport`/
  `NormalizedRenderTarget`.
- Debugging why a hidden object still costs render time → check `ViewVisibility` vs
  `InheritedVisibility` distinction; hidden ≠ culled ≠ invisible-to-this-view.
- Writing a custom post-process pass → read the `ViewTarget`/`post_process_write()`
  section closely; see [post-processing](../post-processing/INDEX.md) for the worked
  example.
- Writing a custom render phase or specialized pipeline → read the
  `DirtySpecializations` section; do not iterate all visible entities from scratch.
- Implementing shadow-map-like auxiliary views → use `RetainedViewEntity` for stable
  identity, following the pattern in
  [lighting-and-shadows](../pbr-and-lighting/lighting-and-shadows.md).

## Related
- [Camera-driven scheduling](../core-pipeline/camera-driven-scheduling.md) — prerequisite: how a camera's `CameraRenderGraph` schedule is run.
- [The effect stack and writing a custom effect](../post-processing/effect-stack-and-custom-effects.md) — example: `ViewTarget` ping-pong used end to end.
- [Render phases and batching](../architecture/render-phases-and-batching.md) — deeper: where `DirtySpecializations` feeds phase queueing.
- [Shadow rendering](../pbr-and-lighting/lighting-and-shadows.md) — example: shadow views keyed by `RetainedViewEntity`.
- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — applies: `src/hybrid` binds Bevy's own `ViewUniform` instead of a custom camera block.
