---
title: Streaming, LOD, and cache invalidation for large baked worlds
description: Records the design for chunked bake/stream regions, multi-density LOD bakes, and a three-trigger re-bake taxonomy (geometry edit, appearance-only change, LOD-density change) for large SDF-authored splat worlds. Briefly built (commit 1fe5e40), deleted in 22d3b91; archived. Read before designing chunked SDF streaming.
type: design
status: archived
tags:
  - sdf
  - 3dgs
  - streaming
  - lod
  - baking
updated: 2026-08-15
sources:
  - commit 1fe5e40 (chunked LOD streaming added)
  - commit 22d3b91 (streaming.rs removed)
aliases:
  - cache invalidation
  - chunk streaming
  - clipmap
  - ChunkCoord
---

# Streaming, LOD, and cache invalidation for large baked worlds

> **Archived (2026-09-28):** the chunked streaming this describes (`streaming.rs`, `ChunkCoord`/`Lod`, `REANIMATE_INTERVAL_SECS`) was added in commit 1fe5e40 (2026-08-15) and deleted with the whole splat pipeline in 22d3b91 (2026-08-16).

## The problem this solves

A world description authored as an SDF can be effectively unbounded in extent (domain
repetition alone gives free infinite tiling, see
[domain-operations](../../sdf-3d/primitives-and-operators/domain-operations.md)), but a
baked splat cloud for the *entire* world at once is not something any GPU can hold in
memory or usefully cull per-frame — this is the same tension the 3DGS knowledge base
documents for city-scale captured scenes (see
[large-scene-techniques](../../3dgs/performance-and-compression/large-scene-techniques.md)),
here arising from procedural rather than captured content, but requiring the same class
of solution.

## Region-based baking keyed to the SDF's own spatial structure

Because the SDF world description is composed of primitives with well-defined local
support (each primitive occupies a bounded region of space, or in the case of infinite
domain-repeated content, a well-defined repeating cell — see
[domain-operations](../../sdf-3d/primitives-and-operators/domain-operations.md)), the
natural unit of bake/stream granularity is a spatial region — a chunk, following the same
logic a voxel-terrain or streaming-world engine would use, sized to balance bake-call
overhead against streaming granularity. Each region:

- Has its own independently baked `SplatAsset` (see
  [bevy-pipeline-integration](../architecture/bevy-pipeline-integration.md)), loaded/
  unloaded as an ordinary Bevy asset via `AssetServer`, participating in the same
  `RenderAsset` extract/prepare lifecycle every other splat asset uses — no bespoke
  streaming system needed at the asset layer.
- Can be re-baked independently when the SDF description for that region changes (see
  the "when to re-bake" discussion in
  [sdf-to-splat-baking](./sdf-to-splat-baking.md)) — region-bounded invalidation is a
  direct consequence of choosing region-sized bake granularity in the first place.

## LOD: baking multiple detail levels per region

Following [Octree-GS](../../3dgs/performance-and-compression/large-scene-techniques.md)'s
anchor-based approach (using anchor Gaussians per octree level rather than a fixed,
distance-independent splat density), each region can be baked at **multiple sampling
densities** — a direct consequence of Step 1 in
[sdf-to-splat-baking](./sdf-to-splat-baking.md) being parameterized by target sample
density in the first place, so producing a coarse-LOD bake of the same region is simply
re-running the same bake algorithm with a lower density parameter, not a structurally
different process. This is a meaningfully easier LOD story than Octree-GS's original
setting (LOD-structuring a *captured*, already-fixed-density 3DGS scene after the fact) —
here, because geometry is analytically known, arbitrary sampling density is available for
free at bake time rather than needing to be derived from an already-baked fixed
representation.

Runtime LOD selection then follows the same pattern documented for large captured
scenes: load/render the region's low-density bake at far camera distances, progressively
loading/swapping to higher-density bakes as the camera approaches — see
[large-scene-techniques](../../3dgs/performance-and-compression/large-scene-techniques.md)
for the general streaming-swap mechanics this reuses, and note that "rendering speed
inconsistent across viewing distance" (that document's key symptom for "you need LOD, not
just compression") applies identically here if region LOD is skipped.

## Cache invalidation: what triggers a re-bake

Three distinct triggers, each with a different appropriate response:

1. **SDF scene edit within a region** (moving/adding/removing a primitive, changing a
   blend operator's parameters) → re-bake only the affected region(s), bounded by the
   edited primitive's spatial influence (its local support, or — for a smooth-blend
   operator, per
   [combination-operators](../../sdf-3d/primitives-and-operators/combination-operators.md)
   — the blend radius `k` around the affected primitives) plus any region boundary the
   edit's influence crosses.
2. **Appearance/material change with geometry unchanged** (e.g. a new lighting bake, or
   updated procedural material function) → only Step 3 of the bake (appearance) needs
   re-running (see
   [sdf-to-splat-baking](./sdf-to-splat-baking.md)) — geometry (position/orientation/
   scale, Steps 1-2) is unaffected and can be reused directly, another concrete
   consequence of the SDF-driven pipeline's step separation that a photographically-
   trained scene has no equivalent shortcut for.
3. **LOD-density-only change** (e.g. tuning the far-distance sampling density for
   performance) → re-run Step 1 (and consequently 2-4) at the new density, but only for
   the specific LOD tier being adjusted, leaving other LOD tiers of the same region
   untouched.

Distinguishing these triggers explicitly (rather than treating any SDF-adjacent change as
"invalidate everything, re-bake the whole world") is what makes interactive editing and
live-tunable performance/quality settings practical rather than requiring a full
world-rebake on every adjustment.

## When to dive in

- Building a large or effectively-open-world SDF-authored scene → region-based bake/
  stream granularity is close to a requirement; decide chunk size early, since it's the
  single parameter most other decisions in this document (LOD tier count, re-bake
  invalidation radius) are relative to.
- Building an interactive SDF editor → the three-trigger invalidation taxonomy is the
  concrete mechanism that keeps edit-to-visual-feedback latency low; implement the
  narrowest applicable trigger response, not a blanket full-rebake, from the start.
- Tuning runtime performance for a large baked world → this and
  [large-scene-techniques](../../3dgs/performance-and-compression/large-scene-techniques.md)
  together are the reference; check whether inconsistent rendering speed across distance
  is present before assuming compression (rather than LOD) is the needed fix.

## Related

- [Baking a Gaussian splat cloud from an SDF](./sdf-to-splat-baking.md) — prerequisite: the per-region bake being scheduled.
- [Making a local SDF edit re-bake fast enough to feel live](../live-editing/incremental-rebake.md) — deeper: dirty regions finer than a chunk.
- [When to transform already-baked splats vs. re-bake from the SDF](../live-editing/deformation-vs-rebake.md) — deeper: which changes need a re-bake at all.
- [Level-of-detail, streaming, and large-scene techniques](../../3dgs/performance-and-compression/large-scene-techniques.md) — contrast: LOD for captured splat scenes.
- [Domain operations: repetition, symmetry, and infinite instancing](../../sdf-3d/primitives-and-operators/domain-operations.md) — prerequisite: domain repetition, the source of unbounded SDF worlds.
