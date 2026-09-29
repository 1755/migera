---
title: "Live Editing: Terraforming, Physics, and Lighting on a Shared SDF"
description: Archived, never-built research (2023-2026 papers) on making an SDF a live world model behind splats: bake-vs-raymarch efficiency, SDF-to-Gaussian math, dirty-region re-bake, deformation vs. re-bake, and SDF-driven physics and lighting. Read before planning SDF sculpting, SDF collision, or reopening bake vs. raymarch.
type: index
status: current
tags:
  - sdf
  - 3dgs
  - physics
  - baking
  - prior-art
updated: 2026-09-28
---

# Live Editing: Terraforming, Physics, and Lighting on a Shared SDF

This topic went one level deeper than the [baking pipeline](../baking-pipeline/INDEX.md):
how to make a local SDF edit re-bake fast enough to feel live, and how the same SDF could
drive physics and lighting instead of three separately synced systems. **None of it was
ever built.** It was researched on 2026-08-15 against the splat renderer, which was
deleted in commits 22d3b91 (2026-08-16) and 684490c (2026-09-06). Research found strong
prior art for each piece but no project combining an editable SDF source of truth with a
live-updated splat renderer; the closest precedent,
[Dreams](../../sdf-3d/performance-and-production/production-case-studies.md), predates
Gaussian splatting and used point clouds.

## Start here

If reconsidering the idea, read [bake vs. raymarch efficiency](./bake-vs-direct-raymarch-efficiency.md)
first (is the architecture a win at all?), then [SDF-to-Gaussian math](./sdf-to-gaussian-math.md),
[incremental re-bake](./incremental-rebake.md), [deformation vs. re-bake](./deformation-vs-rebake.md)
and [unified physics and lighting](./unified-physics-and-lighting.md).

## Archived / superseded

| Note | What it establishes | Read when |
|---|---|---|
| [Is baking to splats actually more efficient than raymarching the SDF directly?](./bake-vs-direct-raymarch-efficiency.md) | Baking amortizes CSG evaluation qualitatively, but no same-scene benchmark or crossover number exists | Reopening bake vs. raymarch for SDF content |
| [Where this project's SDF-to-Gaussian math sits in the published literature](./sdf-to-gaussian-math.md) | SDF+Gaussian papers solve the inverse problem; reusable pieces are SDF-to-opacity and zero-level-set projection; curvature-to-covariance is unpublished | Deriving Gaussians or surfels from a known SDF |
| [Making a local SDF edit re-bake fast enough to feel live](./incremental-rebake.md) | Sub-chunk dirty-cell tracking, per-edit dirty bounds expanded by blend radius, Newton reprojection; latency budget extrapolated, not measured | Building SDF sculpting or terraforming with live feedback |
| [When to transform already-baked splats vs. re-bake from the SDF](./deformation-vs-rebake.md) | Rigid moves transform splats, shape or blend changes re-bake; PhysGaussian/VR-GS/SC-GS for non-rigid motion | Animating any baked SDF-derived representation |
| [One SDF driving collision, physics, and lighting — not three separate systems](./unified-physics-and-lighting.md) | SDF collision per Claybook, getting physics results onto splats, sphere-traced shadows/AO baked vs. per-frame | Wiring SDF collision or SDF lighting into a derived representation |
