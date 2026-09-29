---
title: Core Pipeline
description: Bevy 0.19.1's bevy_core_pipeline - how camera_driver runs Core3d/Core2d per camera and the concrete default passes in order (prepass, main, tonemapping, upscaling, deferred, OIT, FullscreenMaterialPlugin). Read before inserting a custom pass or when an effect lacks depth/normal data.
type: index
status: current
tags:
  - bevy
  - render-pipeline
  - post-processing
updated: 2026-09-28
---

# Core Pipeline

`bevy_core_pipeline` builds the concrete default 3D and 2D pipelines (`Core3d`/`Core2d`)
on top of the [schedule-based architecture](../architecture/render-graph-as-systems.md)
introduced in 0.19. Read it to insert, reorder or understand a stage of the default
pipeline, to find out why a screen-space effect is missing prepass data, or why MSAA was
switched off.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Camera-driven scheduling: Core3d and Core2d](./camera-driven-scheduling.md) | `camera_driver` runs the schedule in each camera's `CameraRenderGraph`; `Core3dPlugin`/`Core2dPlugin` wiring; phase-to-pass lookup via `RetainedViewEntity`. | Adding a custom pass, or giving a camera its own pipeline. |
| [Prepass, tonemapping, upscaling, deferred, and OIT](./passes-and-fullscreen-effects.md) | The default passes in order; two-phase prepass for occlusion culling; deferred as "a fancier prepass" that disables MSAA; OIT; `FullscreenMaterialPlugin`. | An effect needs depth/normals/motion vectors, MSAA turned off, or pass order matters. |

## See also

- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: running compute before `camera_driver` or inside `Core3d` sets.
- [Self-shading vs. G-buffer-writer decision](../../hybrid-architecture/self-shading-vs-gbuffer-decision.md) — contrast: why `src/hybrid` bypasses Bevy's deferred path.
