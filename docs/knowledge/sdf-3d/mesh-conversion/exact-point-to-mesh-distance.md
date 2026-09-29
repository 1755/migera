---
title: Computing exact point-to-mesh distance
description: Covers the unsigned half of mesh-to-SDF conversion — point-to-triangle distance, BVH/octree nearest-triangle queries on CPU, jump flooding on GPU, and the tool landscape. Read when baking a mesh into an SDF or when a bake is too slow.
type: concept
status: current
tags:
  - sdf
  - mesh-conversion
  - spatial-acceleration
  - baking
  - performance
updated: 2026-08-15
aliases:
  - unsigned distance
  - point-to-triangle distance
  - jump flooding
---

# Computing exact point-to-mesh distance

## The unsigned distance sub-problem

Before addressing sign (inside/outside — see
[sign-determination-methods](./sign-determination-methods.md)), converting a polygonal
mesh into an SDF requires solving the *unsigned* distance problem: given a query point,
find the shortest distance to the closest point anywhere on the mesh's surface (a
triangle soup).

The core primitive is point-to-triangle distance in 3D — closed-form and cheap to
evaluate for a single triangle (this is the same "Triangle" primitive referenced in
[primitive-shapes](../primitives-and-operators/primitive-shapes.md)). The naive approach
computes this against *every* triangle in the mesh and takes the minimum — correct, but
`O(n)` per query, prohibitively slow for meshes with more than a few thousand triangles
evaluated at many grid points or query positions.

## Acceleration: spatial structures

Production mesh-to-SDF tools accelerate the nearest-triangle search with a spatial
index built once per mesh:

- **BVH (bounding volume hierarchy)** — a tree of nested bounding boxes over the
  triangles; a query descends the tree, pruning entire subtrees whose bounding box is
  already farther than the current best candidate distance. This is the most common
  choice for CPU-side mesh-to-SDF baking.
- **Octree** — spatially partitions the mesh's bounding volume (rather than the
  triangles themselves) into nested cubes, useful when combined with a regular sampling
  grid since query points and octree cells share the same coordinate structure.

Both bring per-query cost down to roughly `O(log n)` for well-distributed meshes, making
baking a `256^3`-or-larger grid against a moderately detailed mesh tractable.

## GPU-side acceleration: jump flooding

For GPU-driven baking (producing a full 3D grid rather than isolated point queries), the
**jump flooding algorithm (JFA)** is a common technique: rather than each grid cell
independently searching for its nearest triangle, JFA propagates nearest-feature
information across the grid in `O(log(resolution))` parallel passes, each pass having
cells exchange candidate nearest points with neighbors at exponentially decreasing
offsets. This produces an approximate (not always exactly optimal) but very fast and
highly parallel nearest-surface-point field, well suited to real-time or near-real-time
baking on GPU compute shaders — the same broad algorithmic family used for GPU Voronoi
diagrams.

## Practical tool landscape

Several open-source libraries implement this pipeline (BVH/octree-accelerated exact
distance + a chosen sign method, see
[sign-determination-methods](./sign-determination-methods.md)) for common languages and
runtimes: CPU-parallelized triangle-mesh-to-SDF tools exist for Python and C++, generally
built around the combination of a spatial acceleration structure plus one of the sign
methods discussed next. The specific tool choice usually matters less than understanding
which sign-determination method it uses, since that determines robustness to the input
mesh's quality (watertightness, self-intersection, consistent winding).

## When to dive in

- Baking a mesh into a sampled-grid SDF for the first time → this document plus
  [sign-determination-methods](./sign-determination-methods.md) together cover the full
  pipeline; implement or select a tool that pairs BVH/octree distance queries with a sign
  method appropriate to your input mesh quality.
- Baking is too slow for interactive iteration → consider GPU jump-flooding for the
  distance pass, or reduce target grid resolution and rely on
  [sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md)
  for detail near the surface only.
- Baking many meshes at asset-import time in a game engine pipeline → this is exactly
  what Unreal's offline Mesh Distance Field generation does; see
  [unreal-lumen-distance-fields](../state-of-the-art/unreal-lumen-distance-fields.md).

## Related
- [Sign determination](./sign-determination-methods.md) — deeper: the harder inside/outside half of the same bake.
- [Efficient grid baking](./efficient-grid-baking.md) — applies: running these queries at grid scale on the GPU.
- [BVH deep dive](../../hierarchical-volumes/bvh-deep-dive.md) — deeper: the BVH used for nearest-triangle queries.
- [Primitive shapes](../primitives-and-operators/primitive-shapes.md) — prerequisite: the triangle SDF is the core per-triangle primitive.
