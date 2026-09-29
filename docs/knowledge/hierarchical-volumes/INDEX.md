---
title: Hierarchical volume structures — Knowledge Base
description: BVHs and VDB-style hierarchical grids for migera - construction, traversal, GPU layouts, Bevy's frustum/occlusion culling, and the archived occupancy first-pass design with its measured no-win history. Read before changing src/hybrid/bvh.rs, writing hierarchy traversal in WGSL, or proposing an empty-space gate.
type: index
status: current
tags:
  - spatial-acceleration
  - bounding-volumes
  - culling
  - gpu-compute
updated: 2026-09-28
---

# Hierarchical volume structures — Knowledge Base

Research-grounded notes on Bounding Volume Hierarchies and the hierarchical-grid family
(VDB/OpenVDB-style shallow N^3-trees, plus SVO/SVDAGs), designed for CPU build +
compute-shader query, answering *"is this region empty, and at what granularity?"*.
The tree was started to design a first-pass occupancy hierarchy for the legacy marcher.
That hierarchy was never built: `src/hybrid` uses a SAH object BVH
(`src/hybrid/bvh.rs`), and two single-level grid gates layered on it were measured and
reverted with no win. The BVH research is what the live renderer follows.

## Key facts

1. **`src/hybrid` uses bucketed SAH + refit + near-first descent**, all taken from the BVH research ([BVH deep dive](./bvh-deep-dive.md)).
2. **A padded query must pad every node, internal and leaf** — padding only leaves silently prunes subtrees ([BVH deep dive](./bvh-deep-dive.md)).
3. **A fail-safe fallback needs its own test that triggers it** — an inverted boolean once turned a correctness bug into a fake speedup ([BVH deep dive](./bvh-deep-dive.md)).
4. **Occupancy grids on top of the BVH measured no speedup, twice** (Stage C shadow gate, cone-trace gate) — the BVH already prunes empty space ([occupancy design](./occupancy-first-pass-design.md)).
5. **Shallow fixed-depth trees beat deep octrees on GPU**: OpenVDB 5-4-3; GVDB measured octrees ~30-40% slower builds ([hierarchical grids](./hierarchical-grids-and-trees.md)).
6. **Bitmasks are the core primitive**: child lookup = popcount over masked words; leaves as raw u64 bitfields ([hierarchical grids](./hierarchical-grids-and-trees.md)).
7. **Traversal = DDA + short stack**, re-using one set of DDA variables across levels ([hierarchical grids](./hierarchical-grids-and-trees.md)).
8. **Three occupancy states, not two** (SparseLeap empty/non-empty/mixed) avoid fragmentation around fine geometry ([hierarchical grids](./hierarchical-grids-and-trees.md)).
9. **Cull before you descend**: frustum plane tests (`relative_radius`), then Hi-Z occlusion ([Bevy frustum culling](./bevy-frustum-culling.md)).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [BVH deep dive](./bvh-deep-dive.md) | SAH/LBVH builds, ordered/short-stack/stackless traversal, layouts, padded-query and fail-safe-fallback traps, what `src/hybrid` adopted | before changing `src/hybrid/bvh.rs` or BVH traversal in WGSL |
| [Hierarchical grids & VDB-style trees](./hierarchical-grids-and-trees.md) | OpenVDB/GVDB topology and GPU layout, hierarchical DDA, SVO/SVDAG, SparseLeap occupancy | before designing any grid hierarchy |
| [Bevy 0.19.1 primitives & frustum culling](./bevy-frustum-culling.md) | `Aabb`, `Frustum::intersects_obb`, visibility pipeline, GPU occlusion-culling module | before touching visibility/culling or view-gating a custom pass |

## See also

- [AABBs & spatial acceleration](../aabb-acceleration/INDEX.md) — slab test, grid vs BVH landscape, CPU vs GPU placement.
- [Sparse and hierarchical acceleration structures for grid SDFs](../sdf-3d/performance-and-production/sparse-and-hierarchical-structures.md) — the same layouts storing SDF samples.
- [Module and stage skeleton](../hybrid-architecture/module-and-stage-skeleton.md) — where the BVH and the (unbuilt) occupancy seam sit in `src/hybrid`.

## Archived / superseded

| Note | What it establishes | Read when |
|---|---|---|
| [Occupancy first-pass: design blueprint for migera](./occupancy-first-pass-design.md) | Never-built 3-level occupancy hierarchy; two scaled-down gates built and reverted with measured no-win numbers | before proposing any empty-space gate, to avoid repeating it |
