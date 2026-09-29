---
title: Acceleration structures around AABBs
description: Compares uniform/infinite grids, octrees, kd-trees and BVHs (memory, build, traversal, density adaptivity), covers SAH/LBVH build quality and grid hierarchies, and gives a structure-per-workload table plus worst practices. Read before choosing a grid vs BVH for any new subsystem.
type: research
status: current
tags:
  - spatial-acceleration
  - bounding-volumes
  - ray-tracing
  - performance
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/hybrid/bvh.rs
sources:
  - pbr-book.org (PBRT v2 ch.4, v3 ch.4.3)
  - Hapala & Havran, When It Makes Sense to Use Uniform Grids (WSCG 2011)
  - Popov et al., Object Partitioning Considered Harmful
  - Amanatides & Woo 1987 (3D-DDA)
aliases:
  - uniform grid
  - infinite grid
  - spatial hashing
  - kd-tree
  - SAH
  - 3D-DDA
  - teapot in a stadium
---

# Acceleration structures around AABBs

Sources: pbr-book.org (PBRT v2 ch.4 / v3 ch.4.3), CTU Prague "Rendering: Spatial
Acceleration Structures" course notes, Hapala & Havran survey + "When It Makes Sense to
Use Uniform Grids" (WSCG 2011), Boulos "Notes on efficient ray tracing" (Stanford),
Popov et al. "Object Partitioning Considered Harmful", Amanatides & Woo 1987 (DDA).

## The landscape

| Structure | Memory | Build | Traversal | Adapts to density |
|---|---|---|---|---|
| none | - | - | O(N) per ray | - |
| Uniform grid | low-high (res) | **O(N)** | good if uniform; poor otherwise | no |
| Octree/quadtree | low-high (overlap) | low-med | good | yes |
| kd-tree | low-high (splitting) | med-high | excellent | yes |
| BVH | low | med (SAH) / low (LBVH) | very good | yes (object partition) |

- **Uniform grid**: slice scene bounds into equal voxels; each primitive is referenced
  by every voxel it overlaps. Rays traverse cells near-to-far via **3D-DDA**
  (Amanatides-Woo: incremental tMax per axis - never recompute from scratch).
  Resolution rule of thumb: ~cbrt(N) cells on the longest axis (pbrt scales by ~3).
  Strengths: linear build, constant-time *entry* for rays starting inside the volume
  (hierarchical structures pay log N just to get started), trivially parallel build,
  early exit once a hit closer than the next cell boundary is found.
  Weakness: **teapot-in-stadium** - non-uniform density collapses it ("almost all
  triangles in one cell").
- **Infinite grid** (unbounded/hashed): don't allocate the full lattice; hash cell
  coordinates into a table of non-empty cells (spatial hashing) or chunk the world
  (Minecraft-style). Gives O(1) lookup for rays starting anywhere and unbounded worlds
  at the cost of hash indirection. Natural companion to migera's `Node::Repeat`
  tiling: geometry repeats with period P so *cell contents are identical modulo P* -
  one tile's cell list serves every copy (the same insight `repeat_xz` exploits).
- **BVH**: object partition - each node's children hold disjoint *primitive subsets*
  whose boxes may overlap in space. No primitive duplication or splitting ever needed;
  memory stays low; boxes can be arbitrarily loose without correctness loss.
  Traversal: near child first, far child on a stack, prune with "closest hit so far";
  non-overlapping siblings give free early exits.

## BVH construction quality

- **SAH (surface area heuristic)**: probability a random ray hits a node is
  proportional to its surface area. Greedy per-node split minimizing
  cost = C_trav + SA_left/N * N_left + SA_right/N * N_right; evaluate candidate splits
  at bucket boundaries after sorting centroids along each axis (pbrt: 12 buckets).
  Up to **2x traversal speed** vs naive midpoint splits.
- Cheap variant used by pbrt once few primitives remain: median/equal-count split.
- **LBVH/HLBVH**: Morton-code partitioning - linear time, parallel/GPU-friendly, lower
  quality than full SAH for mixed-size primitives.
- Space subdivision enforced inside BVH builds ("object partitioning considered
  harmful") measurably helps overlapping geometry.
- Modern practice: wide nodes (4-8 children), sometimes splitting primitives for
  maximal quality.

## Hierarchies of grids / hybrid layouts

- Two-level: coarse top grid whose cells contain fine grids (or per-object grids)
  placed in a top-level BVH - "teapot gets its own grid, stadium stays coarse".
- Adaptive-resolution/hierarchical grids avoid the empty-cell cost of uniform grids
  while keeping O(1) entry.
- Decision literature (Hapala et al.): build the grid first (it's cheap); if measured
  ray throughput is adequate, stop - only escalate to a hierarchy when scene density is
  provably non-uniform or ray counts are huge.

## Choosing for a given workload

| Situation | Pick |
|---|---|
| Dense, uniform geometry (voxels/meshes/SDF tiles) | uniform/infinite grid |
| Sparse scene, big empty regions | BVH (SAH) |
| Rays starting inside the structure constantly (shadow/AO bounces) | grid (O(1) entry) |
| Static scene, millions of rays/frame | SAH BVH or kd-tree |
| Dynamic/rebuilt-per-frame data | grid or LBVH (build cost dominates) |
| Repeating/tiled world | one tile's grid, reused modulo period |

## Worst practices

- Hand-rolling a BVH before profiling shows a grid is insufficient.
- Building any structure per frame on CPU for mostly-static content.
- Full-lattice allocation for sparse worlds (hash it instead).
- DDA loops recomputing floor/mod per cell instead of incremental tMax.
- Grid resolution chosen without cbrt(N) reasoning - either empty-cell death or
  everything-in-one-cell death.
- Ignoring primitive-overlap duplication costs in grids/octrees (a long diagonal
  triangle referenced by hundreds of cells); consider reference-splitting policies.

## Relevance to migera

`src/hybrid` chose a SAH-bucketed object BVH with refit (`src/hybrid/bvh.rs`), not a
grid. Two single-level occupancy grids layered on top of that BVH were built and
reverted with no measured win (see [Occupancy first-pass design](../hierarchical-volumes/occupancy-first-pass-design.md)):
the BVH already prunes empty space cheaply at the scene sizes the renderer runs.

## Related
- [Ray–AABB intersection: the slab method](./ray-aabb-slab-test.md) — prerequisite: the node/cell test every structure here relies on.
- [BVH deep dive](../hierarchical-volumes/bvh-deep-dive.md) — deeper: BVH build, traversal and memory layout in full.
- [Hierarchical grids & VDB-style trees](../hierarchical-volumes/hierarchical-grids-and-trees.md) — deeper: the grid-hierarchy branch of this landscape.
- [Sparse and hierarchical acceleration structures for grid SDFs](../sdf-3d/performance-and-production/sparse-and-hierarchical-structures.md) — contrast: the same structures applied to baked SDF grids rather than object bounds.
- [Where AABB work belongs: CPU or compute shader](./cpu-vs-gpu-placement.md) — applies: where to build and where to query the chosen structure.
- [Domain operations: repetition, symmetry, and infinite instancing](../sdf-3d/primitives-and-operators/domain-operations.md) — applies: the repetition that makes "one tile's grid, reused modulo period" work.
