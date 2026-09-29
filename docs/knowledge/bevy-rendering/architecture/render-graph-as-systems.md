---
title: "The render graph is gone: render-graph-as-systems (Bevy 0.19)"
description: Bevy 0.19 deleted the Node-trait RenderGraph; RenderGraph is now a ScheduleLabel, camera_driver runs a per-camera schedule (Core3d/Core2d), and passes are plain systems using ViewQuery/RenderContext. Most tutorials and AI knowledge predate this. Read before trusting any render-graph explanation or porting a Node.
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
  - bevy_render-0.19.1/src/renderer/mod.rs
  - Bevy 0.18 to 0.19 migration guide, "Render Graph as Systems"
aliases:
  - RenderGraph removed
  - Node trait
  - add_render_graph_node
  - render sub-graph
  - RenderGraphApp
---

# The render graph is gone: render-graph-as-systems (Bevy 0.19)

## Why this matters

If you've read older Bevy documentation, blog posts, or the Unofficial Bevy Cheat Book,
you'll see rendering described as a `RenderGraph` of `Node`s connected by typed
input/output slots, executed by a graph-runner that resolves dependencies and threads
sub-graphs together (e.g. a "3d" sub-graph run once per camera). **That system was
completely removed in Bevy 0.19.** Any mental model built on `Node`, `RenderGraphApp`,
`add_render_graph_node`, or sub-graph slots is now outdated for 0.19.1. This is the single
biggest architectural change to Bevy's renderer in this version, and most third-party
docs/tutorials/AI training data predate it — verify against source before trusting a
"how the Bevy render graph works" explanation found elsewhere.

## What replaced it

`bevy_render::renderer::RenderGraph` still exists as a name, but it is now nothing more
than a `ScheduleLabel`:

```rust
#[derive(ScheduleLabel, Debug, Clone, PartialEq, Eq, Hash, Default)]
pub struct RenderGraph;

impl RenderGraph {
    pub fn base_schedule() -> Schedule {
        let mut schedule = Schedule::new(Self);
        schedule.configure_sets(
            (RenderGraphSystems::Begin, RenderGraphSystems::Render,
             RenderGraphSystems::Submit, RenderGraphSystems::Finish).chain(),
        );
        schedule
    }
}
```

`render_system` (the system that used to invoke the graph runner) now just does
`world.run_schedule(RenderGraph)` — an ordinary ECS schedule run, with four chained
system sets (`Begin → Render → Submit → Finish`) instead of a graph traversal.

There is no `graph.rs`/`graph_runner.rs`/`Node` trait anywhere in `bevy_render-0.19.1`
anymore — it has been deleted outright, not deprecated.

## Camera-driven scheduling

The former "run the 3d sub-graph once per camera" behavior is now implemented directly as
an ordinary system, `camera_driver` (`bevy_core_pipeline::schedule`), registered into
`RenderGraphSystems::Render`. Each frame it:

1. Collects every active camera (via the `SortedCameras` resource — see
   [camera-and-view-system](../scene-and-views/camera-and-view-system.md)) plus any
   `RootNonCameraView` entities (views with no associated camera, such as point/spot
   light shadow maps).
2. For each one, reads `ExtractedCamera::schedule` — an `InternedScheduleLabel` — and
   calls `world.run_schedule(schedule)`.

```rust
pub fn camera_driver(world: &mut World) {
    let root_views: Vec<_> = /* sorted cameras + auxiliary views */;
    for root_view in root_views {
        let schedule = /* camera.schedule or RootNonCameraView.0 */;
        world.insert_resource(CurrentView(view_entity));
        world.run_schedule(schedule);
    }
    world.remove_resource::<CurrentView>();
}
```

Each camera's `schedule` is set via `CameraRenderGraph(InternedScheduleLabel)`
(`bevy_render::camera`), which despite its name no longer stores a graph identity — it
stores which **schedule** to run for that camera. `Camera3d`/`Camera2d` (in
`bevy_core_pipeline`) point this at the `Core3d`/`Core2d` schedules respectively. This is
what "camera-driven rendering" means concretely in 0.19: different cameras can literally
run different schedules, giving each one an independently customizable pipeline, while
sharing the same underlying mechanism (`world.run_schedule`) as everything else.

See [core-pipeline](../core-pipeline/INDEX.md) for how `Core3d`/`Core2d` are built.

## Passes are systems, not Node impls

A pass that used to be a `Node::run(&self, graph_context, render_context, world)` impl is
now a plain system with ordinary Bevy system params, e.g. (abbreviated):

```rust
pub fn main_opaque_pass_3d(
    world: &World,
    view: ViewQuery<(&ExtractedCamera, &ExtractedView, &ViewTarget, &ViewDepthTexture, ...)>,
    opaque_phases: Res<ViewBinnedRenderPhases<Opaque3d>>,
    pipeline_cache: Res<PipelineCache>,
    mut ctx: RenderContext,
) {
    let (camera, extracted_view, target, depth, ...) = view.into_inner();
    let opaque_phase = opaque_phases.get(&extracted_view.retained_view_entity).unwrap();
    let mut render_pass = ctx.begin_tracked_render_pass(RenderPassDescriptor { .. });
    opaque_phase.render(&mut render_pass, world, view.entity())?;
}
```

Two params replace what the old `Node` trait provided:

- **`ViewQuery<D>`** — resolves to exactly the `CurrentView` entity's data (the view
  `camera_driver` set as a resource before running this schedule), instead of the old
  graph context's `view_entity()`.
- **`RenderContext`** (a `SystemParam`, `bevy_render::renderer::render_context`) —
  wraps command-encoder management (`begin_tracked_render_pass`,
  `command_encoder`), replacing the old `RenderContext` parameter that graph nodes
  received directly.

Because passes are ordinary systems, they compose with the *entire* rest of Bevy's system
API for free: `.before()`/`.after()`, run conditions (`.run_if(...)`), system sets, and
(crucially) the ordinary Bevy scheduler's automatic multithreading — the old graph runner
had to solve parallelism itself; now it's just what the ECS scheduler already does.

## Sub-schedules as the new "sub-graph"

`Core3d`/`Core2d` (`bevy_core_pipeline::schedule`) are themselves `ScheduleLabel`s with
their own ordered system sets:

```rust
pub enum Core3dSystems { Prepass, MainPass, EarlyPostProcess, PostProcess }
```

configured to run `(Prepass, MainPass, EarlyPostProcess, PostProcess).chain()`. Adding a
custom pass or post-process effect to the default 3D pipeline means adding a system to
one of these sets — see [core-pipeline](../core-pipeline/INDEX.md) and
[post-processing](../post-processing/INDEX.md).

## When to dive in

- Porting a custom `Node`-based render feature from an older Bevy version → rewrite it as
  a system with `ViewQuery`/`RenderContext`, and register it into the appropriate
  `Core3dSystems`/`Core2dSystems` set (or a custom schedule if you want a fully separate
  camera pipeline).
- Wondering "which sub-graph does my custom camera use" → it's not a sub-graph anymore,
  it's whatever `ScheduleLabel` you put in that camera's `CameraRenderGraph` component.
- Reading older Bevy blog posts/tutorials/Cheat Book pages about the render graph →
  treat them as historical background only; verify any claim about *current* behavior
  against the source in this knowledge base or the vendored crate source.

## Related
- [The RenderApp split and the Extract step](./render-app-and-extraction.md) — prerequisite: the two worlds and the `Render` schedule that hosts `RenderGraph`.
- [Camera-driven scheduling](../core-pipeline/camera-driven-scheduling.md) — deeper: how `camera_driver` and `Core3dPlugin` wire cameras to schedules.
- [Prepass, tonemapping, upscaling, deferred, and OIT](../core-pipeline/passes-and-fullscreen-effects.md) — deeper: the concrete pass systems in the default pipeline.
- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: where migera-style compute passes go (`RenderGraph` before `camera_driver`, or `Core3d` sets).
- [Native integration with Bevy's render pipeline](../../sdf-3dgs-bevy-integration/architecture/bevy-pipeline-integration.md) — example: a real custom pass registered into `Core3dSystems::MainPass`.
