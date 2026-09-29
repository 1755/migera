---
title: Bevy Rendering (v0.19.1)
description: Source-verified map of how Bevy 0.19.1 (the version in migera's Cargo.lock) renders a frame - two-world extraction, render-graph-as-systems, camera-driven schedules, phases and batching, GPU resources, materials, PBR/lighting, post-processing, 2D/UI and gizmos. Read before writing or debugging any Bevy render code.
type: index
status: current
tags:
  - bevy
  - render-pipeline
  - ecs
  - integration
updated: 2026-09-28
---

# Bevy Rendering (v0.19.1)

How Bevy actually renders a frame in **version 0.19.1** — the version migera pins
(`bevy = "0.19.1"` in `Cargo.toml`, confirmed in `Cargo.lock` on 2026-09-28). 0.19 made
the largest architectural change to Bevy's renderer since camera-driven rendering: it
**deleted the node-based `RenderGraph`/`Node` trait system** and replaced it with plain
ECS systems running in per-camera schedules. Most outside documentation (the Unofficial
Bevy Cheat Book, DeepWiki, older blog posts, most AI training data) still describes the
pre-0.19 model and will mislead you. Every note here was written on 2026-08-15 by reading
the vendored 0.19.1 source (`~/.cargo/registry/src/.../bevy_*-0.19.1/`), cross-checked
against the 0.19 release notes and migration guide. If migera upgrades Bevy, treat these
notes as `stale` until re-verified.

## Start here

1. [Architecture](./architecture/INDEX.md) if you are new to Bevy's renderer, or if anything
   you read elsewhere mentions a `RenderGraph`/`Node` trait — check
   [the render graph is gone](./architecture/render-graph-as-systems.md) before trusting it.
2. Otherwise jump to the topic matching your task below and follow its INDEX.

## Key facts

- **There is no classic render graph.** `RenderGraph` is a `ScheduleLabel`; passes are
  ordinary systems ([render-graph-as-systems](./architecture/render-graph-as-systems.md)).
- **Two ECS worlds, one narrow bridge.** Rendering runs in the `RenderApp` SubApp; data
  crosses once per frame in `ExtractSchedule` ([render-app-and-extraction](./architecture/render-app-and-extraction.md)).
- **Cameras drive scheduling.** `CameraRenderGraph` stores a schedule label, not a graph
  ([camera-driven-scheduling](./core-pipeline/camera-driven-scheduling.md)).
- **Types live in recently split crates** (`bevy_camera`, `bevy_light`, `bevy_material`,
  `bevy_mesh`, `bevy_shader`, split out since 0.17); the `Material` trait is in `bevy_pbr`,
  not `bevy_material` ([material-system](./materials-and-shaders/material-system.md)).
- **0.19 batching is GPU-driven with change lists** — `DirtySpecializations` avoids
  re-iterating every visible entity each frame ([render-phases-and-batching](./architecture/render-phases-and-batching.md)).
- **Light clustering moved to the GPU** (~20x faster per the 0.19 release notes)
  ([clustered-forward-rendering](./pbr-and-lighting/clustered-forward-rendering.md)).
- **Post-process effects chain through `ViewTarget::post_process_write()`** ping-pong, with
  no graph wiring ([camera-and-view-system](./scene-and-views/camera-and-view-system.md)).
- **3D gizmos are depth-tested** against scene depth (`GizmoConfig::depth_bias`); 2D gizmos
  always draw on top ([gizmo-rendering](./2d-and-ui/gizmo-rendering.md)).
- **Pipelines compile asynchronously** — a new pipeline can be missing for a few frames
  ([pipeline-cache-and-specialization](./resources-and-assets/pipeline-cache-and-specialization.md)).

## Topics

| Note | What it establishes | Read when |
|---|---|---|
| [Architecture](./architecture/INDEX.md) | Main/render world split, extraction, entity sync, pipelined threads, render phases, and the 0.19 render-graph-as-systems change. | New to Bevy rendering, or placing a system relative to extract/prepare/queue/render. |
| [Resources and Assets](./resources-and-assets/INDEX.md) | Async pipeline compilation and specialization, `RenderAsset` lifecycle, `RenderDevice`/`RenderContext`/`AsBindGroup`, GPU buffer primitives. | Writing a custom material, GPU-backed asset, compute or GPU-driven feature. |
| [Scene and Views](./scene-and-views/INDEX.md) | Cameras and camera-like views: extraction, `ViewTarget` ping-pong, `ViewUniform`, `RetainedViewEntity`, culling, `DirtySpecializations`. | Multi-camera setups, a post-process pass, a custom phase, or visibility bugs. |
| [Materials and Shaders](./materials-and-shaders/INDEX.md) | `bevy_mesh`, `bevy_shader`, `bevy_material` — the shared mesh/shader/key plumbing under both 3D and 2D materials. | Custom vertex attributes, WGSL `#import`/shader defs, or material specialization keys. |
| [PBR and Lighting](./pbr-and-lighting/INDEX.md) | Light components, GPU clustering, shadow maps and contact shadows, `StandardMaterial`/`Material`, SSR. | Configuring lights/shadows/sky, writing a PBR material, or light-count performance. |
| [Core Pipeline](./core-pipeline/INDEX.md) | `Core3d`/`Core2d` schedules and their concrete passes: prepass, tonemapping, upscaling, deferred, OIT. | Inserting a custom pass or needing exact pass order. |
| [Post-Processing](./post-processing/INDEX.md) | AA, bloom, DOF, motion blur, auto exposure, the fused vignette/lens/chromatic stack, and a custom-effect template. | Adding, ordering, or writing a screen-space effect. |
| [2D and UI](./2d-and-ui/INDEX.md) | Sprites/Mesh2d, immediate-mode gizmos, and UI — three batching strategies side by side. | 2D draw calls, debug drawing, or custom UI rendering. |

## See also

- [Compute shaders in Bevy 0.19](../compute-shaders/bevy-integration.md) — applies: where migera's compute passes sit in the 0.19 schedule model.
- [Bevy-native integration](../hybrid-architecture/bevy-native-integration.md) — applies: migera's rule for reusing Bevy camera/light components in `src/hybrid`.
- [SDF + 3DGS Bevy integration](../sdf-3dgs-bevy-integration/INDEX.md) — applies: a custom phase item, `RenderAsset` and `Core3d` pass built on this material.
- [migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — applies: why characters render through the stock PBR pipeline described here.
