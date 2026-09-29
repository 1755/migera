---
title: "Occupancy first-pass: design blueprint for migera"
description: Blueprint (never built) for a 3-level hash-rooted, DAG-deduplicated occupancy hierarchy gating render/sim work via frustum, Hi-Z and indirect dispatch. Two scaled-down single-level grid gates were built on src/hybrid's BVH and reverted with no measured win. Read before proposing any empty-space gate.
type: design
status: archived
tags:
  - spatial-acceleration
  - culling
  - lod
  - gpu-compute
  - hybrid-renderer
updated: 2026-09-28
code:
  - src/hybrid/bvh.rs
sources:
  - PROGRESS.md "Stage C: occupancy-grid shadow-ray accelerator"
  - PROGRESS.md "Cone tracing's own BVH optimization" (3c)
  - commit f4885a2
  - commit 263b6e1
aliases:
  - occupancy grid
  - occupancy gate
  - empty-space skipping
  - first-pass gating
  - indirect dispatch work manifest
---

# Occupancy first-pass: design blueprint for migera

> **Archived:** never built at this scale. `src/hybrid` has no unbounded/tiled world;
> two single-level occupancy-grid gates layered on its SAH BVH (Stage C shadow-ray gate,
> commit f4885a2; cone-trace gate 3c, commit 263b6e1) were correct but measured no
> speedup and were reverted — the BVH already prunes empty space. Kept as the design
> record; see [Status](#status).

Goal: the theoretically fastest possible "is this volume empty, and at what level is
there something?" gate that every consumer (render passes, marchers, future simulation)
consults before spending work.

## Structure (concretized)

```
Level 2  world cells   64 m   hash-rooted top grid (unbounded world)
Level 1  sub-cells      8 m   fixed 8x8x8 per level-2 cell
Level 0  leaf bitfield  1 m   8x8x8 = one u64 occupancy word per leaf
(+ optional level -1 from the SDF itself when a 1m cell says MIXED)
```

Node record (WGSL, scalars-only convention):

```wgsl
struct HNode {
    child_base: u32,     // index of first child in next level's array (compacted)
    mask_words_x4: u32,  // or store mask inline for small levels
    min_max_e3: vec4<u32>, // packed bounds/flags; e.g. empty|solid tile flags
};
// Level 1 mask: 512 bits -> 8 u64s. Level 2 root: hash map cell_coord -> node.
```

Three states per cell: EMPTY / FULL(tile) / MIXED(has children). A FULL tile at level L
is the "answer at this level" consumers want.

## Build pipeline

Static scene => build once on CPU during flatten (scene already assembled there):
1. Rasterize each leaf's local AABB into level-0 bitfields (Morton order).
2. Reduce upward with bitwise OR into masks; compact children per level.
3. DAG-dedup identical subtrees across tiles (guaranteed hits due to TILE_PERIOD).
4. Upload: two storage buffers per level (nodes, child lists) + a tiny root table.
Dynamic content later: rebuild dirty branches only; Morton order keeps edits local
(bottom-up SVO construction maps directly to a compaction compute pass if needed).

## Query paths

**Ray queries** (render/march): hierarchical DDA as in the grids doc - branchless DDA
at current level, descend on active bit via popcount, short-stack of exit t's. Empty
cells are skipped in O(1) DDA steps; FULL tiles let callers jump straight past (or to,
for shadow occlusion) whole regions.

**Region queries** (simulation/spawn/AI): point-to-cell walk = 2-3 masked loads;
range queries = frustum/AABB test at level L then iterate active bits.

**First-pass gating for rendering**: per view -
1. Frustum-test level-2 cell boxes (plane tests; Bevy `Frustum::intersects_obb_identity`
   equivalent math in WGSL) -> keep list of visible coarse cells.
2. Optional Hi-Z occlusion against last frame's depth (Bevy's occlusion_culling is the
   template): drop cells fully behind opaque geometry.
3. Emit per-consumer work manifests via atomics: e.g. `visible_cells[]` +
   `count`, consumed by indirect dispatches so downstream passes never launch threads
   for empty space ("what should run and at what level" becomes data-driven dispatch).

## CPU<->GPU split

Build/edits: CPU (or one-shot compute) at scene-change time. Queries: GPU-only within
the frame - no readback on the hot path. The only CPU-visible outputs are debug stats
via the established double-buffered async readback pattern. Indirect dispatch counts
come from GPU-side atomic appends, never CPU guesses.

## Best/worst practices specific to this pass

Best: answer at the COARSEST sufficient level; cache last query's node path per ray
(accessor trick); keep leaves as raw u64 words; sort everything Morton; make EMPTY the
implicit default (absence of allocation = empty).
Worst: querying level-0 first; per-pixel tree walks starting at the hash root every
time without path caching; eager subdivision around thin geometry (SparseLeap's
fragmentation warning); mixing conventions (closed/half-open cells) between build and
query; CPU round-trips inside the frame loop.

## Status

Not built. What was built instead, both measured and reverted (details in
`PROGRESS.md`):

- **Stage C (2026-09-07)** — a single-level bounded grid over the BVH root AABB, gating
  shadow rays. A first point-neighborhood gate answered the wrong question (a shading
  point's local emptiness says nothing about its shadow ray's path); the ray-aware
  Amanatides-Woo DDA gate that replaced it was correct but gave no measurable speedup at
  `--stress 10000`. Investigation also found an inverted fail-safe fallback (see the
  last worst practice in [BVH deep dive](./bvh-deep-dive.md)).
- **Cone-trace gate, step 3c (commit 263b6e1)** — `ConeOccupancyGrid` with a
  cone-width-aware DDA. `hybrid_trace` time with the gate off vs on on RADV Renoir:
  `--stress 100` ~54-59 ms vs ~54-58 ms; `--stress 10000` ~102-109 ms vs ~99-110 ms —
  indistinguishable.

Why both lost: the BVH's padded descent already rejects empty space at the node level,
so the grid's DDA paid real per-ray cost to answer the same question, while the
dominant cost is exact candidate marching for rays that *do* hit geometry.

Revisit when: the scene becomes an unbounded or tiled world with many more objects than
a BVH handles cheaply, or a non-rendering consumer (simulation, spawning) needs region
queries.

## Related
- [Hierarchical grids & VDB-style trees](./hierarchical-grids-and-trees.md) — prerequisite: the VDB/GVDB/SparseLeap research this blueprint is built from.
- [BVH deep dive](./bvh-deep-dive.md) — contrast: the structure that made both built gates redundant; also holds the fail-safe-fallback lesson from Stage C.
- [Bevy 0.19.1 primitives & frustum culling](./bevy-frustum-culling.md) — prerequisite: the frustum and Hi-Z occlusion templates step 1-2 of the gating plan copy.
- [Module and stage skeleton](../hybrid-architecture/module-and-stage-skeleton.md) — applies: where the gate would have plugged into `src/hybrid`.
- [Trace-pass bottleneck is not march steps](../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — same-trap: another plausible trace-pass optimization that measured zero win.
- [CPU<->GPU data flow](../compute-shaders/cpu-gpu-data-flow.md) — prerequisite: the async readback and indirect-dispatch rules the CPU<->GPU split relies on.
