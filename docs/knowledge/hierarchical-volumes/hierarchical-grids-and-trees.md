---
title: Hierarchical grids & VDB-style trees
description: Surveys shallow fixed-depth N^3 trees (OpenVDB 5-4-3, GVDB on GPU), bitmask+popcount child lookup, hierarchical DDA with a short exit stack, SVO PUSH/ADVANCE/POP, SVDAG deduplication and SparseLeap's three-state occupancy. Read before designing any grid hierarchy or empty-space gate.
type: research
status: current
tags:
  - spatial-acceleration
  - culling
  - compression
  - performance
  - state-of-the-art
updated: 2026-08-23
sources:
  - Museth, VDB - High-Resolution Sparse Volumes with Dynamic Topology (ToG 2013)
  - Hoetzlein, GVDB - Raytracing Sparse Voxel Databases on the GPU (HPG 2016)
  - Laine & Karras, Efficient Sparse Voxel Octrees (I3D 2010)
  - Kampe/Sjostrom/Ulfasson, Sparse Voxel DAGs
  - Hadwiger et al., SparseLeap (VIS 2017)
  - arXiv 2410.14128 (Hybrid Voxel Formats)
aliases:
  - OpenVDB
  - GVDB
  - sparse voxel octree
  - SVO
  - SVDAG
  - SparseLeap
  - hierarchical DDA
  - occupancy grid
---

# Hierarchical grids & VDB-style trees

The family matching the target design: a coarse cube grid (e.g. 64 m cells) whose cells
contain finer grids, recursively, at a fixed small depth.

Sources: Museth "VDB: High-Resolution Sparse Volumes with Dynamic Topology" (ToG 2013)
+ OpenVDB docs (openvdb.org overview/FAQ), "VDB a deep dive" (jangafx.com), Hoetzlein
"GVDB: Raytracing Sparse Voxel Databases on the GPU" (HPG 2016),
Laine & Karras "Efficient Sparse Voxel Octrees" (I3D 2010 + tech report),
Kampe/Sjostrom/Ulfasson "Sparse Voxel DAGs" (+ Chalmers thesis),
Hadwiger et al. SparseLeap (VIS 2017), "Hybrid Voxel Formats" (arXiv 2410.14128).

## Topology: shallow N^3-trees, not octrees

- **OpenVDB production layout**: RootNode (unbounded hash of top-level nodes) ->
  InternalNode 32^3 -> InternalNode 16^3 -> LeafNode 8^3 ("5-4-3" = log2 dims read
  leaf-up). Fixed compile-time branching per level; effectively infinite index space;
  all leaves at one depth.
- Each node: **child mask** (one bit per slot; 32^3=32768 bits=512 u64 words for the
  top level) + compacted child-pointer list. Values/tiles may live at any level
  (a fully-homogeneous subtree collapses to a tile = our "non-empty at this level"
  answer).
- **Why not deep octrees**: measured ~30-40% slower builds (more interior nodes to
  maintain); more levels = more pointer hops per query. **Why not giant two-level
  index maps** (e.g. 128^3 bitmask): multi-MB masks make popcount/insertion worst-case
  and waste memory on empty space. GVDB's empirical sweet spot: `<3,3,3,4>`-ish N-ary
  configs with 16^3 or 32^3 bricks - i.e., 2-4 levels total. For a 64 m cell that is:
  64m -> 8x8x8 of 8m -> 8x8x8 of 1m -> leaf bitfield of 12.5 cm voxels.
- **Cascaded grids** (clipmap-style): fixed-resolution grids re-centered on the camera
  per LOD ring - the alternative for unbounded worlds when content near the camera is
  all that matters; simpler than hashing but moves data every recentring step.

## GPU data layout (GVDB recipe)

- Per level: memory pool of uniform nodes (same size within a level!) + separate pool
  for child-index lists. Node = header + level-sized bitmask + child-list offset.
- Child lookup: `popcount(mask & below_bit)` over 64-bit words -> index into compacted
  child list.
- Iteration of active children: classic bit tricks -
  `for (w = mask; w != 0; w &= w-1) { idx = tzcnt(w); }`.
- Brick payload data lives out-of-band in an atlas texture; topology stays in pools.

## Traversal: hierarchical DDA / octree walk

Two proven shapes:

1. **Hierarchical 3D-DDA (grid-shaped trees)**: branchless Amanatides-Woo stepping at
   current level; when entering an active child cell, descend by *re-initializing the
   same single set of DDA variables* at the finer level (GVDB found unrolling per-level
   DDA state kills occupancy via register pressure); keep a tiny stack of saved exit
   t-values to climb back up (short-stack method). Never revisit the root until done.
2. **SVO PUSH/ADVANCE/POP (octree)**: maintain current cube {pos, scale, child-slot};
   per iteration choose PUSH (descend into first-entered child), ADVANCE (next sibling,
   flipping crossed-axis bits, validated against ray direction signs), or POP (climb to
   highest exited ancestor using the scale-indexed parent stack). Contours optionally
   bound subtrees' t-ranges. Beam/coarse-distance pre-pass can skip whole empty spans
   before per-pixel work (4x4 or 8x8 pixel blocks).

## Compression upgrades

- **Occupancy bitfield leaves**: encode the last level as raw bits inside the node
  (4^3=64 voxels = one u64; Laine-Karras use child-descriptor masks instead).
- **SVDAG**: deduplicate identical subtrees into a DAG (bottom-up hash/sort per level).
  Down to 0.08 bits/non-empty voxel; traversal identical to SVO but children may be
  shared (up to 8 explicit pointers/node after merging).
- **For migera specifically**: `TILE_PERIOD` repetition guarantees identical subtrees
  across tiles - a DAG built over ONE tile is automatically the DAG for all tiles.
  This is the single biggest structural gift available to us.
- Hybrid-format study findings: formats shaped `R(N^3) G(M)` (dense-grid top, SVDAG
  below) consistently perform well; too-granular dense levels waste both memory and DDA
  steps; DF (distance-field) levels buy big march acceleration at storage cost.

## Occupancy classification (SparseLeap)

Track THREE states per region: empty / non-empty / **unknown(mixed)**. Mixed regions do
NOT subdivide eagerly (fragmentation death around fine geometry); they resolve lazily
at the next level down or via the consumer. Their occupancy histogram tree emits
bounding boxes only where class changes down the tree, then rasterizes per-pixel ray
segment lists so ray marching becomes linear list skipping. Lesson for us: the
hierarchy answers "empty here?" fast, and otherwise says "go finer", never "subdivide
everything".

## Best/worst practices summary

Best: fixed shallow depth; power-of-two shifts between levels; bitmask+compacted
children; single reused DDA state; Morton ordering everywhere (build, sort, leaves);
tiles/homogeneous-collapse at every level; DAG dedup when repetition exists.
Worst: deep recursion-heavy octrees on GPU; per-level register copies of DDA state;
full-lattice indirection tables; eager subdivision of mixed regions; storing payloads
inline in nodes; forgetting half-open cell conventions across levels.

## Related
- [Occupancy first-pass design](./occupancy-first-pass-design.md) — applies: the migera blueprint built from this research (archived; never built at full scale).
- [BVH deep dive](./bvh-deep-dive.md) — contrast: the object-partition hierarchy `src/hybrid` actually uses.
- [Acceleration structures around AABBs](../aabb-acceleration/acceleration-structures.md) — prerequisite: uniform grids, 3D-DDA and the teapot-in-stadium problem.
- [Sparse and hierarchical acceleration structures for grid SDFs](../sdf-3d/performance-and-production/sparse-and-hierarchical-structures.md) — deeper: the same brick/VDB layouts storing SDF samples rather than occupancy.
- [Domain operations: repetition, symmetry, and infinite instancing](../sdf-3d/primitives-and-operators/domain-operations.md) — applies: the tiling that makes DAG dedup across tiles free (legacy `TILE_PERIOD` world; `src/hybrid` has no tiling).
