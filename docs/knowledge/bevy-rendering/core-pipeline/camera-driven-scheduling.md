---
title: "Camera-driven scheduling: Core3d and Core2d"
description: In Bevy 0.19.1 camera_driver runs, per camera, the schedule named in its CameraRenderGraph (Core3d/Core2d); Core3dPlugin/Core2dPlugin wire Camera3d/Camera2d to them, and a pass finds its render phase via RetainedViewEntity. Read before inserting a custom pass or giving a camera its own pipeline.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - ecs
  - integration
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_core_pipeline-0.19.1/src/core_3d/mod.rs, core_2d/mod.rs
  - bevy_render-0.19.1/src/camera.rs
aliases:
  - camera_driver
  - CameraRenderGraph
  - Core3dSystems
  - Core2dSystems
  - custom pass ordering
---

# Camera-driven scheduling: Core3d and Core2d

## The foundation

Bevy 0.19 removed the old `RenderGraph`/`Node` trait indirection (see
[render-graph-as-systems](../architecture/render-graph-as-systems.md)) because it forced
pass authors into a bespoke trait instead of plain ECS systems. The replacement reuses the
ECS scheduler itself as "the graph": a schedule *is* the render graph, its system sets are
the graph's stages, and system ordering (`.before`/`.after`/`.in_set`) replaces node-edge
wiring.

`bevy_render::renderer::RenderGraph` is a root `ScheduleLabel` with sets
`Begin/Render/Submit/Finish`. Inside its `Render` set, `bevy_core_pipeline` registers
`camera_driver`:

```rust
render_app.add_systems(RenderGraph, (
    camera_driver.in_set(RenderGraphSystems::Render),
    (submit_pending_command_buffers, handle_uncovered_swap_chains)
        .chain().in_set(RenderGraphSystems::Submit),
));
```

`camera_driver` is the bridge between "one root schedule" and "per-camera sub-schedules."
For every live camera (from `SortedCameras`) and every `RootNonCameraView` (shadow-
casting lights that need a view but have no `Camera`), it reads that view's
`ExtractedCamera::schedule` (an `InternedScheduleLabel`), inserts a `CurrentView(
view_entity)` resource, and calls `world.run_schedule(schedule)`. Because each camera
carries its *own* schedule label, different cameras can run entirely different render
graphs — a 3D camera runs `Core3d`, a 2D camera runs `Core2d`, a custom camera could run a
user-defined schedule.

The critical glue for pass authors is `ViewQuery<D, F>` — a `SystemParam` that reads the
`CurrentView` resource `camera_driver` just set and performs the equivalent of
`query.get(current_view.entity())`. This is why pass systems can simply declare `view:
ViewQuery<(&ExtractedCamera, &ExtractedView, ...)>` and automatically operate on whatever
camera the driver is currently iterating, with no manual entity plumbing.

## `Core3dPlugin`: wiring `Camera3d` to `Core3d`

Three things happen in `Core3dPlugin::build`:

1. **Wires `Camera3d` to `Core3d`** via required components:
   ```rust
   app.register_required_components_with::<Camera3d, CameraRenderGraph>(|| {
       CameraRenderGraph::new(Core3d)
   });
   ```
   So spawning `Camera3d` auto-attaches `CameraRenderGraph(Core3d)`. During extraction,
   this becomes `ExtractedCamera::schedule` — the exact field `camera_driver` reads. Full
   chain: `Camera3d` → required `CameraRenderGraph(Core3d)` → extracted into
   `ExtractedCamera::schedule` → `camera_driver` calls `world.run_schedule(Core3d)`.

2. **Registers the schedule and default systems**:
   ```rust
   render_app.add_schedule(Core3d::base_schedule())
       .add_systems(Core3d, (
           (early_prepass, early_deferred_prepass, late_prepass,
            late_deferred_prepass, copy_deferred_lighting_id)
               .chain().in_set(Core3dSystems::Prepass),
           (main_opaque_pass_3d, main_transparent_pass_3d)
               .chain().in_set(Core3dSystems::MainPass),
           tonemapping.in_set(Core3dSystems::PostProcess),
           upscaling.after(Core3dSystems::PostProcess),
       ));
   ```
   Note `upscaling` runs *after* the whole `PostProcess` set, not inside it — deliberately
   the very last thing in the schedule.

3. **Initializes phase resources and extraction/queue systems** for `Opaque3d`,
   `AlphaMask3d`, `Transparent3d` (and prepass/deferred counterparts) as
   `ViewBinnedRenderPhases`/`ViewSortedRenderPhases`.

`Core2dPlugin` mirrors this structurally but simpler: no prepass, no deferred, no OIT by
default. Phase types are `Opaque2d`/`AlphaMask2d` (binned) and `Transparent2d` (sorted).
`Core2dSystems` has the same four-set shape even though nothing populates `Prepass` by
default — it exists so third-party plugins can add 2D prepasses without redesigning the
schedule.

## RenderPhase → Pass relationship

Phases and passes are deliberately decoupled: *queueing* (deciding what's drawable,
sorting/binning it, picking pipelines) happens during the `Render` schedule's `Queue`/
`PrepareResources`/`PhaseSort` sets (see
[render-app-and-extraction](../architecture/render-app-and-extraction.md)), while
*consuming* (recording GPU draw calls) happens later, inside each camera's own schedule.
This lets extraction/queue work be shared and ordered independently of per-view execution,
and lets multiple cameras redraw the same queued data without redoing culling/sorting/
binning.

Pass systems look the phase up by the current view's retained identity:

```rust
let (Some(opaque_phase), Some(alpha_mask_phase)) = (
    opaque_phases.get(&extracted_view.retained_view_entity),
    alpha_mask_phases.get(&extracted_view.retained_view_entity),
) else { return; };
opaque_phase.render(&mut render_pass, world, view_entity)?;
```

Full path: `camera_driver` picks a camera → sets `CurrentView` → `ViewQuery` in the pass
resolves that entity's components, including `ExtractedView::retained_view_entity` → the
pass indexes the phase `HashMap` with that key → `phase.render(...)` iterates the phase's
batched/sorted items and issues draw calls via `RenderContext`.

## Practical takeaway: adding a custom pass

```rust
render_app.add_systems(Core3d,
    my_post_process.in_set(Core3dSystems::PostProcess).before(tonemapping));

fn my_post_process(
    view: ViewQuery<(&ExtractedCamera, &ViewTarget, &MyComponent)>,
    mut ctx: RenderContext,
) {
    let (camera, target, settings) = view.into_inner();
    let post_process = target.post_process_write(); // ping-pong source/destination
    // begin a fullscreen-triangle RenderPass writing post_process.destination,
    // sampling post_process.source
}
```

If the pass needs to draw scene geometry (not just full-screen), it needs its own
`PhaseItem`/`ViewBinnedRenderPhases<MyPhase>` populated by a `Queue`-set system, following
the `Opaque3d`/`Transparent3d` pattern (see
[render-phases-and-batching](../architecture/render-phases-and-batching.md)). For the
common "single fragment shader over the whole screen" case,
`FullscreenMaterialPlugin<T: FullscreenMaterial>` (`fullscreen_material.rs`) does all of
this automatically — see [passes-and-fullscreen-effects](./passes-and-fullscreen-effects.md).

## When to dive in

- Porting a custom render feature from an older Bevy version's `Node` → rewrite as a
  system with `ViewQuery`/`RenderContext`, register into the right `Core3dSystems`/
  `Core2dSystems` set.
- Wondering "which sub-graph does my custom camera use" → not a sub-graph anymore, it's
  whichever `ScheduleLabel` sits in that camera's `CameraRenderGraph`.
- Adding a pass that needs to run relative to existing passes → check
  [passes-and-fullscreen-effects](./passes-and-fullscreen-effects.md) for the concrete
  system-ordering map (prepass → main pass → tonemapping → upscaling).

## Related
- [The render graph is gone](../architecture/render-graph-as-systems.md) — prerequisite: why schedules replaced graph nodes in 0.19.
- [Prepass, tonemapping, upscaling, deferred, and OIT](./passes-and-fullscreen-effects.md) — deeper: the concrete passes these schedules run, in order.
- [Camera and the view system](../scene-and-views/camera-and-view-system.md) — deeper: `CameraRenderGraph`, camera extraction, and `RetainedViewEntity`.
- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: placing compute work before `camera_driver` or inside `Core3d` sets.
