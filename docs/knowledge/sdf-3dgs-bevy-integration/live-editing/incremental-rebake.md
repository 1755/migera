---
title: Making a local SDF edit re-bake fast enough to feel live
description: Designs sub-chunk dirty-region re-bake for SDF edits (dirty-cell grid plus octree, per-edit dirty bounds grown by blend radius, Newton reprojection for small edits); the sub-16 ms budget is extrapolated, not measured. Never built; archived. Read before building SDF sculpting or terraforming.
type: design
status: archived
tags:
  - sdf
  - baking
  - spatial-acceleration
  - gpu-compute
  - performance
updated: 2026-08-15
sources:
  - https://reindernijhoff.net/2026/01/webgpu-sdf-editor-real-time-signed-distance-field-modeling/
  - https://medium.com/@jasonbooth_86226/optimizing-spline-operations-d48b5f8fede4
aliases:
  - dirty region
  - terraforming
  - sculpting
  - incremental re-bake
  - spatial hashing
---

# Making a local SDF edit re-bake fast enough to feel live

> **Archived (2026-09-28):** never implemented; the chunk streaming, `animate_rings`/`animate_pillar` and splat bake it extends were deleted in commit 22d3b91 (2026-08-16) and splats were dropped in 684490c (2026-09-06). The dirty-region ideas apply equally to re-baking any SDF-derived cache.

Contents: [The gap](#the-gap-this-fills) · [WebGPU SDF editor prior art](#the-closest-concrete-prior-art-a-real-time-webgpu-sdf-sculpting-editor) · [What "dirty" means](#what-dirty-means-for-this-projects-specific-edit-types) · [Reprojection](#reprojection-instead-of-resampling-for-small-edits) · [Performance expectations](#performance-expectations--whats-measured-vs-extrapolated) · [When to dive in](#when-to-dive-in)

## The gap this fills

[streaming-and-invalidation](../baking-pipeline/streaming-and-invalidation.md) already
establishes *that* a re-bake should be scoped to "the affected region(s), bounded by the
edited primitive's spatial influence" rather than the whole world — but this project's
current region granularity is a **streaming chunk** (see `streaming.rs`'s `ChunkCoord`/
`Lod` machinery), sized for camera-distance LOD tiers, not for a single brush stroke or
a dragged primitive. A brush stroke's influence is typically a small fraction of a
chunk's volume; re-baking the entire containing chunk on every stroke would work
correctly but waste most of the re-sampling work on unaffected surface far from the
edit. This document is about the next level of granularity down: **dirty-region
tracking within a chunk**, so re-bake cost scales with edited surface area, not chunk
volume.

## The closest concrete prior art: a real-time WebGPU SDF sculpting editor

The most directly applicable project found in research is a real-time SDF sculpting
editor built in WebGPU
([Reinder Nijhoff, January 2026](https://reindernijhoff.net/2026/01/webgpu-sdf-editor-real-time-signed-distance-field-modeling/)) —
notably close to this project's own domain (a hierarchical scene graph of spheres,
boxes, tori, capsules, and cylinders combined via smooth union/subtract/intersect
blends). It has no Gaussian-splatting component (confirmed: it renders via Marching
Cubes / Surface Nets to a mesh, not splats), but its **editing-side data structures are
the reusable part**, independent of what the final rendering primitive is:

1. **Frame-to-frame dirty-cell diffing.** The world is divided into a 3D grid (up to
   `2^14` cells). Each frame, the current primitive buffer is compared against a cached
   copy from the previous frame; a cell is marked dirty only if the set of primitives
   overlapping it actually changed. Unchanged cells reuse their prior geometry/samples
   entirely — no re-evaluation at all.
2. **GPU spatial binning of primitives via counting sort.** Each primitive is assigned
   to every grid cell its bounding box overlaps, computed with a 3-pass GPU counting
   sort (count occurrences per cell → prefix-sum to get per-cell offsets → scatter
   primitive indices into place). This is the direct GPU analogue of a spatial hash
   grid over SDF primitives, and it is what makes "which primitives affect this cell"
   a cheap lookup instead of a linear scan of the whole scene graph per cell.
3. **Hierarchical refinement inside dirty cells.** Each dirty cell undergoes 4-6 octree
   splits to localize where the surface actually crosses the zero level set before
   running full extraction — so the expensive step (Marching Cubes for edit-mode
   preview, Surface Nets for final view) only runs at the resolution the local surface
   actually needs, not uniformly across the whole dirty cell.
4. **Compact, edit-friendly primitive layout.** All primitives live in a single GPU
   buffer, 28 floats (112 bytes) each — rotation quaternion, position, bounding box,
   blend parameters — a layout chosen specifically so a single primitive edit (move,
   resize, re-parent) is a small, fast partial buffer update rather than requiring a
   full scene re-upload.

**Direct translation to this project's architecture**: steps 1-2 map onto a sub-chunk
dirty-cell grid layered *inside* the existing chunk system (chunk = coarse streaming/LOD
unit, as now; dirty cell = fine re-bake unit, new) — the ECS-authored `Shape`/`Blend`/
`Transform` hierarchy this project already assembles per-bake (see
[assembly.rs](../architecture/bevy-pipeline-integration.md) and
`sdf::assembly::assemble_scene`) is a natural place to also compute each primitive's
world-space bounding volume for the counting-sort binning step, since `GlobalTransform`
is already being read there for exactly this kind of spatial query. Step 3's octree
refinement corresponds to running this project's existing Poisson-disk surface sampling
(see [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) Step 1) scoped to
just the dirty cells rather than the whole chunk.

## What "dirty" means for this project's specific edit types

Not every kind of scene change needs the same invalidation response — extending
[streaming-and-invalidation](../baking-pipeline/streaming-and-invalidation.md)'s
three-trigger taxonomy down to sub-chunk granularity:

- **A primitive's `Shape` parameters change** (e.g. a torus's `minor_radius` grows under
  a brush) — dirty region is that primitive's own bounding volume, expanded by the
  containing smooth-union's blend radius (since a shape change can shift the blended
  surface beyond the primitive's own raw bounds — see
  [combination-operators](../../sdf-3d/primitives-and-operators/combination-operators.md)
  for why a smooth blend's influence extends past the two input primitives' hard
  bounds by roughly the blend radius `k`).
- **A primitive's `Transform` changes** (moved, rotated — this project already animates
  `Transform` for the pillar and rings via `GlobalTransform` propagation, see
  `main.rs`'s `animate_pillar`/`animate_rings`) — dirty region is the **union** of the
  primitive's bounding volume before and after the move, not just the new position,
  since surface that existed at the old position must also be re-evaluated (it may no
  longer be covered by this primitive and could revert to whatever's underneath, or
  disappear entirely if nothing else fills that space).
- **A new primitive is added, or a primitive is removed** — dirty region is that
  primitive's bounding volume (expanded by blend radius, as above); removal needs the
  same expanded-bounds treatment as a transform change, since neighboring geometry that
  was being smooth-unioned against the removed primitive changes shape too.
- **Only a `Blend` radius changes with shapes/transforms fixed** — dirty region is
  still the same expanded bounds (a larger blend radius extends a smooth union's
  influence further from the hard primitive bounds, exactly like a shape-parameter
  change), even though no primitive's own geometry moved.

In every case, the dirty region computed here is a strict *subset* of the containing
chunk (or, worst case for an edit whose blend radius happens to be unusually large,
occasionally spans a chunk boundary — the existing
[streaming-and-invalidation](../baking-pipeline/streaming-and-invalidation.md) chunk
system already has to handle cross-chunk influence for this reason, since the pillar's
smooth-subtract bite is already documented as a "scene-relative, not pillar-relative"
limitation for large translations in `sdf::world::assemble_infinite_scene` — the same
class of problem recurring at a finer granularity).

## Reprojection instead of resampling for small edits

For edits small enough that the *existing* splat sample points are still close to the
(now slightly moved) surface, re-projecting each existing dirty-region splat via one
Newton step — `p_new = p − f(p)·∇f(p)`, from the
[sdf-to-gaussian-math](./sdf-to-gaussian-math.md) discussion of the zero-level-set
pulling operator — is cheaper than discarding and re-running full Poisson-disk sampling
from scratch, since it reuses the existing sample distribution (already well-spaced,
already curvature-adapted) rather than re-solving the blue-noise placement problem.
This degrades gracefully: if a splat's projected distance moves it far enough that its
local curvature/orientation assumptions are no longer valid (a cheap check: does the
new `∇f(p)` differ substantially from the splat's stored orientation?), fall back to
full resampling for that splat specifically rather than accepting a stale orientation —
a per-splat decision, not an all-or-nothing choice for the whole dirty region.

## Performance expectations — what's measured vs. extrapolated

Be explicit about what research actually found here, since this is the topic's weakest
citation coverage:

- **A cautionary, not-dirty-region-limited number**: recomputing a full 512×512 SDF
  field for 4 spline-driven terrain edits costs roughly 70ms of GPU time (Jason Booth,
  ["Optimizing Spline Operations"](https://medium.com/@jasonbooth_86226/optimizing-spline-operations-d48b5f8fede4),
  MicroVerse devlog) — this is brute-force full-grid recompute, presented here
  specifically as the cost dirty-region tracking exists to avoid, not as a target.
- **A reference point for cheap per-sample SDF work**: normal computation via the
  standard 4-tap ("tetrahedron trick") or 6-tap central-difference gradient
  ([Inigo Quílez](https://iquilezles.org/articles/normalsSDF/)) costs a handful of
  extra `distance()` evaluations per sample — sub-microsecond per sample on desktop GPU
  compute, though the real cost is proportional to how expensive the *scene's* SDF
  evaluation is per tap (CSG tree depth), which is scene-dependent and not something
  a generic number can capture.
- **No paper or project found gives a direct, verified "milliseconds per changed
  region, combining SDF resample and splat-parameter derivation" number** — this is
  exactly the metric this project needs and it does not appear to exist in published
  form yet. The defensible extrapolation from the numbers above: a well-localized,
  brush-sized dirty region (orders of magnitude smaller than a 512×512 full-field
  recompute) should comfortably fit a sub-16ms "feels live" budget *if* dirty-region
  scoping is actually working (i.e., cost genuinely scales with edited area, not
  chunk size) — but this is inference from adjacent numbers, not a measurement. Treat
  it as a hypothesis to validate empirically (the same way this project already
  validates bake-time regressions via the
  `chunk_bake_produces_fewer_splats_at_coarser_lod` test's timing) rather than a
  performance guarantee to design around.

## When to dive in

- Implementing brush-based terraforming with live visual feedback → the four-part
  WebGPU SDF editor pattern (frame-diff dirty cells, GPU counting-sort spatial binning,
  octree refinement inside dirty cells, compact edit-friendly primitive buffer) is the
  concrete structure to build; layer it *inside* the existing chunk system rather than
  replacing chunk-level streaming.
- Deciding how large a dirty region should be for a specific edit type → the per-edit-type
  breakdown above (shape change / transform change / add-remove / blend-only change) is
  the concrete rule set, all keyed off "primitive bounds expanded by blend radius,
  unioned across before/after state where relevant."
- Optimizing re-bake further once basic dirty-region tracking works → the
  reprojection-instead-of-resampling technique is the next lever, with a cheap
  per-splat validity check (orientation drift) to decide when to fall back to full
  resampling instead of accepting a stale splat.
- Needing a real performance number to plan around → there isn't one in the literature;
  budget time to measure this project's own dirty-region re-bake cost directly rather
  than trusting an extrapolated estimate.

## Related

- [Streaming, LOD, and cache invalidation for large baked worlds](../baking-pipeline/streaming-and-invalidation.md) — prerequisite: the chunk-level invalidation this subdivides.
- [When to transform already-baked splats vs. re-bake from the SDF](./deformation-vs-rebake.md) — contrast: changes that need no re-bake at all.
- [GPU compute-shader baking and Bevy asset-pipeline integration](../baking-pipeline/gpu-compute-baking.md) — applies: live re-bake latency needs a GPU bake.
- [Where this project's SDF-to-Gaussian math sits in the published literature](./sdf-to-gaussian-math.md) — deeper: the zero-level-set projection used for reprojection.
- [Sparse and hierarchical acceleration structures for grid SDFs](../../sdf-3d/performance-and-production/sparse-and-hierarchical-structures.md) — deeper: octree and sparse-grid layouts for dirty cells.
- [Efficient baking: GPU algorithms, resolution, and storage](../../sdf-3d/mesh-conversion/efficient-grid-baking.md) — contrast: baking an SDF grid rather than splats.
