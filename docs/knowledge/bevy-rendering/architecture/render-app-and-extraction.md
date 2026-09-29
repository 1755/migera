---
title: The RenderApp split and the Extract step
description: Bevy 0.19.1 renders from a second ECS World (the RenderApp SubApp); data crosses once per frame in ExtractSchedule via a world swap, then the Render schedule runs RenderSystems sets in order (ExtractCommands ... PostCleanup). Read first when new to Bevy rendering or choosing where a render system goes.
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
  - bevy_render-0.19.1/src/lib.rs, extract_plugin.rs, extract_param.rs
aliases:
  - RenderApp
  - SubApp
  - ExtractSchedule
  - RenderSystems
  - main world vs render world
---

# The RenderApp split and the Extract step

## Why this exists

Bevy runs simulation (gameplay logic) and rendering (GPU command generation) as two
mostly-independent ECS `World`s so that:

- Rendering code can use render-specific components (GPU handles, cached bind groups,
  batching data) without polluting the gameplay world or its archetypes.
- The two worlds *can* be pipelined across threads — frame N's rendering overlaps frame
  N+1's simulation (see [pipelined-rendering](./pipelined-rendering.md)) — without data
  races, because only one narrow, well-defined step ever touches both worlds at once.

## How it works

`RenderPlugin::build` (`bevy_render::lib`) creates a second `World` wrapped as a
`SubApp` and inserts it into the main `App` under the `RenderApp` label
(`bevy_app::AppLabel`). From this point on there are two worlds living side by side:

- **Main world** — gameplay/simulation, owned by the ordinary `App`.
- **Render world** — GPU-facing state, owned by the `RenderApp` sub-app.

Every frame, a narrow **extract** step copies the subset of main-world data the renderer
needs into the render world. This is implemented with a deliberately cheap trick in
`extract_plugin.rs`: the *entire* main `World` is temporarily moved (not cloned) into the
render world as a resource called `MainWorld`, a schedule called `ExtractSchedule` runs
with access to both worlds, and then the main world is moved back out:

```rust
pub fn extract(main_world: &mut World, render_world: &mut World) {
    let scratch_world = main_world.remove_resource::<ScratchMainWorld>().unwrap();
    let inserted_world = core::mem::replace(main_world, scratch_world.0);
    render_world.insert_resource(MainWorld(inserted_world));
    render_world.run_schedule(ExtractSchedule);

    let inserted_world = render_world.remove_resource::<MainWorld>().unwrap();
    let scratch_world = core::mem::replace(main_world, inserted_world.0);
    main_world.insert_resource(ScratchMainWorld(scratch_world));
}
```

A `ScratchMainWorld` placeholder avoids allocating a fresh `World` every frame just to
have something to leave behind in the main `App` slot during the swap.

Systems registered on `ExtractSchedule` run *in the render world* (so `Commands`,
`ResMut`, etc. target render-world state) but can read main-world data through the
`Extract<P>` system param (`extract_param.rs`), which resolves any read-only
`SystemParam` (typically a `Query`) against the `MainWorld` resource instead:

```rust
fn extract_clouds(mut commands: Commands, clouds: Extract<Query<RenderEntity, With<Cloud>>>) {
    for cloud in &clouds {
        commands.entity(cloud).insert(Cloud);
    }
}
```

`Extract` is restricted to `ReadOnlySystemParam` — writing to main-world data from
`ExtractSchedule` is not possible through this API, by design: extraction is meant to be
one-directional and as short as possible, since it's the one part of the frame where
main-world and render-world are not usable independently.

`ExtractSchedule` deliberately disables automatic `apply_deferred` insertion
(`auto_insert_apply_deferred: false`, `set_apply_final_deferred(false)`); any `Commands`
queued during extraction are instead applied later by `apply_extract_commands`, running
inside the `Render` schedule's `RenderSystems::ExtractCommands` set. This lets command
application overlap with other render-world work instead of blocking extraction itself.

## The Render schedule and RenderSystems sets

Once extraction has populated the render world, the `Render` schedule
(`bevy_render::Render`, a `ScheduleLabel`) runs the rest of the frame's rendering work
through an ordered chain of `RenderSystems` sets:

```
ExtractCommands → PrepareMeshes → CreateViews → Specialize → PrepareViews
  → Queue → PhaseSort → Prepare → Render → Cleanup → PostCleanup
```

This is the modern equivalent of the classic "extract / prepare / queue / render"
mental model, just with more granular sets for GPU-driven batching:

- **Queue** has sub-sets `QueueMeshes` / `QueueSweep` — entities are queued as phase
  items into [render phases](./render-phases-and-batching.md), and stale/invisible
  entries are swept out of persistent bins.
- **Prepare** has sub-sets `PrepareResources` → `PrepareResourcesBatchPhases` →
  `PrepareResourcesWritePhaseBuffers` → `PrepareResourcesCollectPhaseBuffers` →
  `PrepareResourcesFlush` → `PrepareBindGroups` — this is where CPU-side data gets
  uploaded to GPU buffers and bind groups get built.
- **Render** is where `render_system` actually runs the
  [render-graph-as-systems](./render-graph-as-systems.md) schedule and submits GPU
  command buffers. In most cases user code should not add systems directly to this set —
  it's reserved for the render backend itself.

`RenderPlugin::build` wires `(PipelineCache::process_pipeline_queue_system,
render_system).chain().in_set(RenderSystems::Render)`, so pipeline compilation results
are absorbed just before the frame's draw commands are recorded.

## When to dive in

- Writing a system that needs to move main-world data into the render world → read
  [entity-sync-and-extraction-patterns](./entity-sync-and-extraction-patterns.md) for the
  four extraction plugin flavors (`ExtractComponentPlugin`, `ExtractInstancesPlugin`,
  `ExtractResourcePlugin`, manual `Extract<...>` systems).
- Ordering a custom system relative to Bevy's own prepare/queue/render work → check which
  `RenderSystems` set it belongs in before reaching for ad-hoc `.before()`/`.after()`.
- Debugging "why does my render-world change show up one frame late" → see
  [pipelined-rendering](./pipelined-rendering.md), which explains exactly when extraction
  happens relative to the render thread.

## Related
- [Entity sync and extraction patterns](./entity-sync-and-extraction-patterns.md) — deeper: stable render-world entities and the four ways to extract data.
- [Pipelined rendering](./pipelined-rendering.md) — deeper: how the render SubApp runs on its own thread and why data can appear a frame late.
- [The render graph is gone](./render-graph-as-systems.md) — deeper: what runs inside the `Render` set now that passes are systems.
- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — applies: how `src/hybrid` extracts camera and light data across this boundary.
