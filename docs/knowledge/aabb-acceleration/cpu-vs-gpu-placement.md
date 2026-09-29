---
title: "Where AABB work belongs: CPU or compute shader"
description: Decision framework for placing AABB work - few rays, entity culling and static builds on the CPU; per-pixel/per-step tests and culling feeding other GPU passes on the GPU - with data residency as the tiebreaker, plus worst practices. Read before deciding where a bounds test or structure build runs.
type: guide
status: current
tags:
  - bounding-volumes
  - gpu-compute
  - culling
  - performance
updated: 2026-08-23
aliases:
  - CPU vs GPU
  - data residency
---

# Where AABB work belongs: CPU or compute shader

## Decision framework

| Workload | Place | Why |
|---|---|---|
| Few rays (picking, gameplay queries) | **CPU** (`RayCast3d`) | GPU dispatch + readback latency dwarfs a handful of slab tests. |
| Frustum/visibility culling of entities | **CPU** (already built into Bevy: `Aabb` component + visibility systems) | Runs once per entity per frame; needs CPU-side visibility decisions anyway. |
| Structure build for static scenes | **CPU** at scene-assembly time (our `flatten.rs` stage) or one-time compute; upload result | Build once; amortize forever. |
| Per-pixel / per-march-step tests against many primitives | **Compute** (WGSL slab test) | Millions of coherent lanes; data already lives in GPU buffers; zero readback needed when results feed another pass. |
| Occlusion-style culling driving indirect draws | **GPU** with async readback one frame late | Bevy's own occlusion_culling module is the template. |

The tiebreaker is almost always **data residency**: if the primitives are already in a
storage buffer and the consumers are other GPU passes, keep the tests on the GPU - a
CPU round trip costs a full frame of latency plus bandwidth.

## Coherence notes for the GPU side

Rays within a warp should traverse similar structures: sort/tile by screen space
(primary rays), by light direction bucket (shadow rays). Random-access pointer chasing
(child indices, cell lists) kills coalescing - prefer compact per-cell primitive lists
in flat arrays over linked structures.

## Worst practices (both sides)

- CPU: testing every primitive per ray because "it's only 15" - then scaling that code
  path to 10k; rebuilding acceleration data per frame for static content; using
  `BoundingSphere` where an AABB is both cheaper *and* tighter (thin slabs!).
- GPU: divergent early-outs inside warps on data-dependent trees (BVH traversal with
  stack in registers); per-thread `malloc`-style dynamic indexing into hash tables
  without considering coalescing; forgetting bounds checks when dispatch rounds up;
  NaN-sensitive min/max without the clamped-update form.
- Both: bool-only APIs where intervals are needed; world-space box refits for rotated
  content instead of local boxes + transformed rays; mixing conventions (half-open vs
  closed cells) across producer/consumer passes.

## Related
- [CPU<->GPU data flow](../compute-shaders/cpu-gpu-data-flow.md) — prerequisite: why a readback costs a frame of latency, which drives this table.
- [Bevy 0.19 AABB primitives](./bevy-aabb-primitives.md) — applies: `RayCast3d` for the CPU rows, Bevy's occlusion-culling module for the GPU rows.
- [Acceleration structures around AABBs](./acceleration-structures.md) — prerequisite: the structures being placed.
- [Compute shader performance](../compute-shaders/performance-best-practices.md) — deeper: coalescing and divergence behind the GPU-side coherence notes.
- [Bevy 0.19.1 primitives & frustum culling](../hierarchical-volumes/bevy-frustum-culling.md) — applies: the CPU frustum-culling row in detail.
