---
title: Narrow-band and truncated SDFs (TSDF)
description: Explains truncated SDFs that store accurate distance only within ±tau of the surface — why this makes storage and live depth-fusion updates scale with surface area, and why sphere tracing against one is not globally safe without an occupancy structure. Read before storing a grid SDF or marching a TSDF.
type: concept
status: current
tags:
  - sdf
  - baking
  - spatial-acceleration
  - raymarching
updated: 2026-08-15
aliases:
  - TSDF
  - KinectFusion
  - narrow band
---

# Narrow-band and truncated SDFs (TSDF)

## The insight

For most rendering and reconstruction uses, the *exact* distance value far from a
surface is not useful — a raymarcher only needs "is it safe to take a big step," not the
precise Euclidean distance to a surface hundreds of units away. This observation drives
one of the most common practical optimizations for sampled/grid SDFs: only store accurate
values in a **narrow band** around the surface, and truncate/clamp values beyond it.

A **truncated signed distance field (TSDF)** clamps stored distance values to a fixed
range `[-tau, +tau]` around the surface. Points farther than `tau` from the surface all
store the same saturated value (`+tau` outside, `-tau` inside), rather than their true
(larger) distance.

## Why this matters for storage and update cost

- **Storage**: only voxels within the narrow band need high-precision distance data;
  interior/exterior "bulk" voxels can use a single saturated value or be omitted entirely
  in sparse structures (see
  [sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md)).
  This is what makes real-time volumetric fusion (KinectFusion-style live 3D scanning,
  and its many descendants) tractable — the surface band is a small fraction of the total
  volume.
- **Incremental updates**: TSDFs are the standard representation for real-time depth-
  sensor fusion because each new depth frame only needs to update voxels near its
  observed surface, using a simple running-weighted-average update rule per voxel. This
  is far cheaper than re-fusing a whole scene's worth of mesh geometry per frame.
- **Robustness to noisy/partial data**: because a TSDF fuses many noisy observations by
  averaging, it naturally denoises small measurement errors better than directly meshing
  each individual depth frame would.

## The tradeoff versus a full SDF

The obvious cost: **raymarching/sphere-tracing against a TSDF is not globally safe** —
once a ray is farther than `tau` from any surface, the field no longer tells you how far
it actually is (it's saturated), so you cannot safely take a large confident step; you
must either fall back to a fixed small step size in that region, or maintain a separate
coarse "occupancy"/bounding acceleration structure to skip empty space quickly (see
[sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md)).
Truncation trades "sphere tracing works everywhere for free" for "storage and update cost
scale with surface area, not volume."

## Relationship to the four representation families

TSDFs are a special case of the [sampled grid representation](./sdf-representations.md)
(#2), specifically optimized for the case where either (a) the SDF is being incrementally
built/fused from live sensor data rather than authored offline, or (b) memory is the
binding constraint and a genuinely global exact distance field is unaffordable. Baked
mesh distance fields in game engines (see
[unreal-lumen-distance-fields](../state-of-the-art/unreal-lumen-distance-fields.md)) use
a related but distinct optimization: they store exact-ish distance out to a bounded
volume around the mesh's bounding box (not literally globally), since object-scale
distance fields don't need to represent distance to infinity either.

## When to dive in

- Building a real-time 3D scanning/SLAM/fusion pipeline → TSDF is almost certainly the
  right representation; look at KinectFusion-lineage papers and voxel-hashing extensions
  for handling scenes larger than a fixed grid.
- Raymarching against a TSDF and seeing rays "give up" or step incorrectly far from
  surfaces → this is expected; combine with a coarse occupancy/bounding structure for
  empty-space skipping rather than trusting the SDF value at long range.
- Deciding TSDF truncation width `tau` → too small risks holes/noise near thin features
  or fast-moving surfaces between frames; too large increases the per-frame update cost
  and defeats the storage savings. This is scene- and sensor-noise-dependent, tune
  empirically.

## Related
- [The four ways an SDF can be stored/evaluated](./sdf-representations.md) — prerequisite: TSDF is a special case of the sampled-grid family.
- [Sparse and hierarchical structures](../performance-and-production/sparse-and-hierarchical-structures.md) — deeper: the octree/hash/clipmap structures that store only the band and skip empty space.
- [Efficient grid baking](../mesh-conversion/efficient-grid-baking.md) — applies: narrow-band baking of mesh SDFs.
- [Unreal Lumen distance fields](../state-of-the-art/unreal-lumen-distance-fields.md) — contrast: bounded-volume per-mesh fields instead of truncation.
- [Hierarchical grids and trees](../../hierarchical-volumes/hierarchical-grids-and-trees.md) — deeper: occupancy hierarchies for the empty-space skipping a TSDF needs.
