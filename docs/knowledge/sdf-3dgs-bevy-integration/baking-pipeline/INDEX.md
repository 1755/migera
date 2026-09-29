---
title: Baking Pipeline
description: Archived design and implementation record for turning an SDF into a splat cloud: the forward four-step bake with its sampling traps, a never-built GPU-compute bake, and briefly-built chunked streaming/LOD/invalidation. Read before sampling points or splats off an SDF or designing chunked SDF streaming.
type: index
status: current
tags:
  - sdf
  - 3dgs
  - baking
  - streaming
updated: 2026-09-28
---

# Baking Pipeline

Converting an SDF into a renderable splat cloud is a forward sampling problem: the SDF
gives exact surface position and normal everywhere, so nothing has to be discovered by
gradient descent as in photographic 3DGS training. All three notes are archived — the
CPU bake and chunk streaming were deleted in commit 22d3b91 (2026-08-16), the later bake
in `src/hybrid` in 684490c (2026-09-06), and the GPU bake was never built.

## Archived / superseded

| Note | What it establishes | Read when |
|---|---|---|
| [Baking a Gaussian splat cloud from an SDF](./sdf-to-splat-baking.md) | Sample → orient/scale from gradient and curvature → colour → pack, plus real sampling bugs (tangent-plane jumps, degenerate seed normals, hard-edge gaps, no baked specular) | Writing any SDF surface sampler |
| [GPU compute-shader baking and Bevy asset-pipeline integration](./gpu-compute-baking.md) | The bake is embarrassingly parallel and maps onto Bevy's `UninitBufferVec`/`PrepareResources`; share one WGSL SDF module between preview and bake | Writing a GPU pass that samples an SDF surface |
| [Streaming, LOD, and cache invalidation for large baked worlds](./streaming-and-invalidation.md) | Chunked bake regions, multi-density LOD bakes, and a three-trigger re-bake taxonomy | Designing chunked streaming or cache invalidation for SDF-derived data |
