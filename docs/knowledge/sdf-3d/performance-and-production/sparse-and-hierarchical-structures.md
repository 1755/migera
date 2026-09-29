---
title: Sparse and hierarchical acceleration structures for grid SDFs
description: Surveys structures that store grid SDF detail only near the surface and skip empty space — sparse voxel octrees, SVDAGs, voxel hashing for unbounded/streamed scenes, clipmaps, and nested distance fields — with a how-to-choose table. Read before building any grid SDF larger than one asset.
type: concept
status: current
tags:
  - sdf
  - spatial-acceleration
  - streaming
  - lod
  - compression
  - performance
updated: 2026-08-15
aliases:
  - sparse voxel octree
  - SVO
  - SVDAG
  - voxel hashing
  - empty-space skipping
---

# Sparse and hierarchical acceleration structures for grid SDFs

## Why a dense grid alone isn't enough at scale

A dense volume texture (see
[performance-characteristics](./performance-characteristics.md)) stores a value at every
grid point, including the vast majority of 3D space that is either deep interior or far
exterior — regions where accurate per-point distance rarely matters, since a raymarcher
mainly needs "roughly how much empty space can I skip" out there, not precise distance.
Sparse and hierarchical structures exploit this to cut both memory and (via empty-space
skipping) raymarching cost.

## Sparse Voxel Octrees (SVO)

An SVO recursively subdivides a volume into 8 child octants, but — critically — **prunes
subtrees that contain no relevant surface detail** (entirely empty or entirely solid
regions collapse to a single leaf rather than recursing further). This:

- **Saves memory**: a sparse octree can use a fraction of the memory of an equivalent
  dense grid at the same maximum resolution, since detail-free regions cost almost
  nothing to represent.
- **Accelerates raymarching directly**: a ray traversing the octree can skip an entire
  large empty subtree in one step (jumping past its bounding box) rather than sampling it
  point by point, which is a qualitatively different and more powerful form of empty-space
  skipping than relying on local SDF magnitude alone.

## Directed Acyclic Graphs (SVDAG)

A further compression step exploits the fact that many SVO subtrees are **structurally
identical** (a common occurrence in scenes with repetitive or symmetric geometry) —
de-duplicating identical subtrees turns the tree into a directed acyclic graph, since
multiple parent nodes can now point at the same shared child. This can compress storage
substantially beyond a plain SVO for scenes with redundant geometry, at the cost of losing
the simple one-path-per-voxel traversal guarantee a pure tree provides.

## Voxel hashing

For scenes too large to fit any single fixed-size grid or octree in memory at once
(city-scale open worlds, live 3D-scanning/SLAM), **voxel hashing** allocates fixed-size
voxel *blocks* only where surface data actually exists, addressed via a spatial hash table
rather than a dense array index. Blocks outside the currently-relevant region (e.g. far
from the camera) can be streamed out to host/disk memory and streamed back in as needed —
a direct architectural answer to "the world is bigger than GPU memory," commonly paired
with [narrow-band/truncated SDF](../fundamentals/narrow-band-and-truncated-sdfs.md)
representations in real-time fusion/SLAM systems.

## Clipmaps

A clipmap represents a large volume at **variable resolution depending on distance from a
focal point** (typically the camera) — high resolution near the camera, progressively
coarser resolution farther away, each resolution band ("clip level") stored as a
fixed-size grid that shifts/wraps as the focal point moves, rather than reallocating.
This is the structure Unreal's Global Distance Field uses (see
[unreal-lumen-distance-fields](../state-of-the-art/unreal-lumen-distance-fields.md)):
combining many per-object baked Mesh Distance Fields into a single scene-wide clipmap
gives an approximate, distance-appropriate-resolution global geometry representation
without needing uniform high resolution across an entire large scene.

## Nested/hierarchical distance fields for empty-space skipping specifically

A related but distinct technique — used, for example, in Claybook's engine — maintains a
**hierarchy of SDF grids at multiple resolutions simultaneously**, using the coarser
levels specifically to safely maximize sphere-tracing step size during traversal (query
the coarse level first to take a large confident step, refining to finer levels only as
the ray approaches a surface), rather than primarily as a memory-compression technique.
This is the same core idea as image-pyramid mipmapping applied to accelerate raymarching
itself, not just to save storage.

## How to choose

| Constraint | Structure |
|---|---|
| Memory is the primary concern, scene fits in a bounded volume | Sparse Voxel Octree |
| Scene has significant repeated/symmetric structure | SVDAG (SVO + subtree deduplication) |
| Scene is larger than available memory, or built incrementally (scanning/SLAM) | Voxel hashing with streaming |
| Large scene needing distance-appropriate resolution falloff from a camera/focal point | Clipmap |
| Raymarching step-size acceleration is the primary goal (not just storage) | Multi-resolution SDF hierarchy with coarse-to-fine traversal |

These are not mutually exclusive — production systems frequently combine several (e.g. a
clipmap of sparse per-region grids, each internally using octree pruning).

## When to dive in

- Building any grid-based SDF system for a scene larger than a single small object →
  start here before assuming a dense grid is sufficient; dense grids only remain
  practical for individual asset-scale volumes.
- Streaming/open-world scale scenes, or live sensor-fusion pipelines → voxel hashing is
  almost certainly the right starting point.
- Wanting to understand Unreal's Global Distance Field architecture specifically → the
  clipmap concept here is the prerequisite; see
  [unreal-lumen-distance-fields](../state-of-the-art/unreal-lumen-distance-fields.md) for
  the full system built on top of it.

## Related
- [Narrow-band and truncated SDFs](../fundamentals/narrow-band-and-truncated-sdfs.md) — prerequisite: why only the surface band needs storage.
- [Hierarchical grids and trees](../../hierarchical-volumes/hierarchical-grids-and-trees.md) — deeper: the same structures from the traversal side.
- [Occupancy-first pass design](../../hierarchical-volumes/occupancy-first-pass-design.md) — applies: migera's occupancy-based empty-space skipping design.
- [Efficient grid baking](../mesh-conversion/efficient-grid-baking.md) — applies: baking into these structures.
- [Unreal Lumen distance fields](../state-of-the-art/unreal-lumen-distance-fields.md) — example: clipmaps in production.
