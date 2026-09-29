---
title: Scene and Views
description: Bevy 0.19.1 views (cameras plus camera-like shadow and probe views) - bevy_camera components, camera extraction, ViewTarget ping-pong, ViewUniform, RetainedViewEntity, frustum culling and DirtySpecializations. Read for multi-camera setups, a custom post-process pass or phase, or visibility bugs.
type: index
status: current
tags:
  - bevy
  - render-pipeline
  - culling
updated: 2026-09-28
---

# Scene and Views

"Views" — cameras and camera-like things such as shadow-casting lights and reflection
probes — are the unit most of the renderer organizes around: render phases are per view,
post-processing runs per view, and camera-driven scheduling runs a schedule per view (see
[Architecture](../architecture/INDEX.md)). This topic bridges the abstract camera
(`bevy_camera`) and per-frame GPU view state (`bevy_render::view`).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Camera and the view system](./camera-and-view-system.md) | `Camera`/`Projection`/`RenderTarget`; `CameraRenderGraph` names a schedule; camera extraction and sorting; `ViewTarget` ping-pong; `ViewUniform`; `RetainedViewEntity`; frustum culling; `DirtySpecializations`. | Split-screen or render-to-texture, a custom post-process pass or phase, shadow-like auxiliary views, or an object that renders when it shouldn't (or vice versa). |

## See also

- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — applies: `src/hybrid` binds Bevy's own `ViewUniform`/`ExtractedView`.
