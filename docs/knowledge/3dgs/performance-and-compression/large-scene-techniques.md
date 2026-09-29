---
title: Level-of-detail, streaming, and large-scene techniques
description: Explains (as of 2025-2026) why city-scale 3DGS scenes need explicit LOD such as Octree-GS's anchor hierarchy, how streaming keeps only view-relevant Gaussians resident, and layered LapisGS-style streaming for dynamic content. Read when splat render speed varies with camera distance or a scene exceeds memory.
type: research
status: current
tags:
  - 3dgs
  - lod
  - streaming
  - performance
  - state-of-the-art
updated: 2026-08-15
sources:
  - Ren et al., "Octree-GS", 2024
aliases:
  - Octree-GS
  - LapisGS
  - level of detail
  - large-scale scenes
---

# Level-of-detail, streaming, and large-scene techniques

## The problem large scenes create

A single 3DGS scene's rendering cost scales with the number of Gaussians that must be
projected, sorted, and blended for a given view (see
[tile-based-rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md)). This
is fine for object-scale or room-scale captures with a bounded, moderate Gaussian count,
but breaks down for large-scale scenes (city blocks, expansive outdoor environments, or
any capture where scene extent and detail both grow substantially): a naive approach
that keeps every Gaussian resident and candidate for rendering regardless of camera
distance produces a rendering cost that doesn't scale — a zoomed-out view of a large
scene still has to consider the same enormous Gaussian count as a zoomed-in view, even
though most of that detail is imperceptible at a distance. This specifically manifests
as inconsistent rendering speed (fast up close, slow zoomed out, or vice versa depending
on implementation details) rather than a uniform cost — a signal, when observed, that
LOD/streaming techniques are needed rather than simply "more optimization."

## Octree-GS: explicit LOD structure

Octree-GS integrates an explicit octree spatial structure with 3D Gaussian Splatting
specifically to provide **level-of-detail (LOD) rendering** for large scenes. Each octree
level corresponds to a set of **anchor Gaussians** defining that LOD tier — coarser
levels use fewer, larger anchor Gaussians (appropriate for distant viewing, where fine
detail wouldn't be resolvable anyway), while finer levels add progressively more detailed
anchors as the camera gets closer. Some variants incorporate learned "neural Gaussians"
and small MLPs to predict anchor-wise features, producing a more compact representation
than storing full independent Gaussian parameters at every LOD level redundantly. The
practical effect: rendering cost for a given view becomes proportional to the detail
actually *needed* at the current camera distance, rather than the scene's total Gaussian
count regardless of viewing distance — directly solving the inconsistent-rendering-speed
problem described above.

## Streaming and progressive loading

For scenes too large to load entirely into GPU (or even system) memory at once —
particularly relevant for web/browser deployment (see
[rasterizer-implementations](../rendering-and-rasterization/rasterizer-implementations.md)
for browser memory cap discussion) or mobile devices with constrained VRAM — LOD
structures naturally support **streaming**: only the LOD tiers and spatial regions
actually needed for the current view need to be resident in memory, with additional
detail loaded progressively as the camera moves closer to a region, and evicted as it
moves away. This mirrors, at the level of scene content rather than texture mipmaps, the
same "load only what's needed at the current resolution/distance" principle that
mipmapping and clipmaps embody in other rendering contexts.

## Progressive/layered streaming for dynamic content

For dynamic (4D/temporal) scenes specifically, layered/progressive streaming approaches
(e.g. LapisGS) apply an analogous idea across time as well as space: rather than
requiring an entire dynamic sequence's full-detail data to be available before playback
can begin, a base layer provides immediately-available lower-fidelity playback, with
additional detail layers streamed in progressively — directly relevant to any
application needing to start playback quickly (streaming video-like consumption of a
captured dynamic scene) rather than requiring a large upfront download.

## Resource-constrained device targets

Recent work specifically targets rendering very large (city-scale) Gaussian scenes on
genuinely resource-constrained devices (mobile/embedded hardware, not just "a somewhat
less powerful desktop GPU") — this is an active, distinct research thread from general
LOD techniques, since mobile-class hardware imposes both stricter memory limits and
substantially lower compute throughput than the desktop/workstation GPUs most 3DGS
research and tooling still primarily targets by default.

## When to dive in

- A scene's rendering speed is inconsistent (fast up close, slow far away, or the
  reverse) rather than uniformly slow → this is the specific symptom LOD techniques
  address; profile Gaussian count vs. viewing distance before assuming a general
  performance problem needing compression (see
  [compression-techniques](./compression-techniques.md)) rather than LOD.
- Deploying a large-scale (city block or bigger) capture, especially to web or mobile →
  Octree-GS-style explicit LOD structuring, combined with streaming, is close to a
  requirement rather than an optional optimization at this scale.
- Building a dynamic-scene streaming application (video-like playback of a captured 4D
  scene) → the layered/progressive streaming approach (LapisGS-style) is the relevant
  pattern, distinct from static-scene LOD.

## Related

- [Compression techniques](./compression-techniques.md) — contrast: reduces size, not distance-dependent cost.
- [The tile-based rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md) — prerequisite: why cost scales with the number of Gaussians considered per view.
- [Streaming, LOD, and cache invalidation for large baked worlds](../../sdf-3dgs-bevy-integration/baking-pipeline/streaming-and-invalidation.md) — applies: migera's (archived) SDF-baked equivalent of this LOD problem.
- [Game engine integration and production status](../state-of-the-art/game-engine-integration.md) — applies: scale limits of production plugins.
