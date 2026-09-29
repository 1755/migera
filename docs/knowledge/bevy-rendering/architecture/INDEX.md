---
title: Architecture
description: Bevy 0.19.1 renderer foundations - the main-world/render-world split, extraction and entity sync, pipelined threading, render phases and GPU-driven batching, and the replacement of the node render graph with ECS schedules. Read first when new to Bevy rendering or when a doc mentions RenderGraph nodes.
type: index
status: current
tags:
  - bevy
  - render-pipeline
  - ecs
updated: 2026-09-28
---

# Architecture

The conceptual foundation the rest of [Bevy Rendering](../INDEX.md) builds on: how the
main world and render world are split, how data crosses that boundary, how the frame is
scheduled, and — critically for 0.19 — how the node-based render graph was replaced with
plain ECS systems. Revisit whenever another note says "extraction," "the Render schedule,"
"RenderSystems sets," or "camera-driven scheduling" without explaining it.

## Start here

1. [The RenderApp split and the Extract step](./render-app-and-extraction.md)
2. [The render graph is gone](./render-graph-as-systems.md) — mandatory if you have any
   pre-0.19 Bevy knowledge.
3. The rest in any order.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [The RenderApp split and the Extract step](./render-app-and-extraction.md) | The `RenderApp` SubApp, the `ExtractSchedule` world swap, and `RenderSystems` set order (`ExtractCommands` … `PostCleanup`). | Deciding where a render system goes, or onboarding to the renderer. |
| [The render graph is gone](./render-graph-as-systems.md) | 0.19 deleted `Node`/`RenderGraphApp`; `RenderGraph` is a `ScheduleLabel`, `camera_driver` runs per-camera schedules, passes use `ViewQuery`/`RenderContext`. | Before trusting any render-graph explanation, or porting a `Node`. |
| [Render phases, PhaseItems, and GPU-driven batching](./render-phases-and-batching.md) | Binned vs sorted phase items, `RenderCommand` composition, GPU bin unpacking, sparse uploads, change-list `DirtySpecializations`. | Writing a custom phase item or debugging draw-call counts. |
| [Entity sync and the four extraction patterns](./entity-sync-and-extraction-patterns.md) | `RenderEntity`/`MainEntity` mirrors, and `ExtractComponentPlugin`, `ExtractInstancesPlugin`, `ExtractResourcePlugin`, manual `Extract<>`. | Adding render support for a component, or when extracted data never appears. |
| [Pipelined rendering](./pipelined-rendering.md) | `PipelinedRenderingPlugin` ping-pongs the render SubApp to a render thread; extraction stays on the main thread. | Data shows up a frame late, frame pacing, or thread-context assumptions. |

## See also

- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: compute passes in the 0.19 schedule model.
- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — applies: what `src/hybrid` extracts from real Bevy components.
