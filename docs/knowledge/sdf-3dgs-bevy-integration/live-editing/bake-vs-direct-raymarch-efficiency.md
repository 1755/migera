---
title: Is baking to splats actually more efficient than raymarching the SDF directly?
description: Argues that baking amortizes CSG-tree evaluation that raymarching repays per pixel per frame (Dreams tried and rejected raymarching), states that no same-scene benchmark or crossover number exists, and lists where raymarching wins. Archived; migera chose raymarching. Read before reopening bake vs. raymarch.
type: research
status: archived
tags:
  - sdf
  - 3dgs
  - raymarching
  - performance
  - baking
updated: 2026-08-15
sources:
  - commit 1fe5e40 (note written)
  - commit 684490c (3DGS dropped; SDF raymarcher kept)
aliases:
  - amortization
  - bake vs raymarch
  - crossover point
  - Dreams
  - Claybook
---

# Is baking to splats actually more efficient than raymarching the SDF directly?

> **Archived (2026-09-28):** migera went the other way — the splat bake was deleted (commits 22d3b91, 684490c) and the SDF is raymarched every frame in `src/hybrid`; the crossover this note asks for was never measured, so its "bake wins" conclusion is unproven for migera.

Contents: [Short answer](#the-honest-short-answer) · [Structural asymmetry](#the-core-structural-asymmetry) · [Where baking wins](#where-baking-clearly-wins-well-supported) · [Where raymarching wins](#where-direct-raymarching-wins-also-real-narrower) · [Unmeasured](#whats-genuinely-unmeasured--do-not-cite-invented-numbers-here) · [This project's bake](#what-this-means-for-this-projects-own-bake) · [When to dive in](#when-to-dive-in)

## The honest short answer

**Qualitatively yes, and the reasoning is well-supported — but no source found gives a
quantitative crossover number** (the splat density / CSG depth / step count at which one
approach overtakes the other). This document lays out the argument precisely and flags
exactly where it's citable fact versus where it's this project's own extrapolation, so
future work doesn't mistake a plausible argument for a measured one.

## The core structural asymmetry

Both approaches ultimately pay for the same expensive operation — **evaluating the
combined CSG-tree SDF** (every primitive, every smooth-blend operator, in the whole
scene's composed distance function). What differs is *how many times that operation
gets paid for*, per
[performance-characteristics](../../sdf-3d/performance-and-production/performance-characteristics.md)'s
existing cost model:

- **Direct raymarching** pays the full tree-evaluation cost at **every march step, for
  every pixel, every single frame** — `pixel_count × average_step_count ×
  tree_evaluation_cost`, repeated 60+ times per second even for completely static
  content. [performance-characteristics](../../sdf-3d/performance-and-production/performance-characteristics.md)
  already establishes this scales at minimum linearly (often worse) with scene
  complexity, "very different from rasterization, where geometry outside a pixel's
  coverage costs nothing for that pixel."
- **Baking** pays that same tree-evaluation cost **once per sample point, once per bake**
  — `sample_count × tree_evaluation_cost` — and then every subsequent frame pays only
  the tile-based Gaussian rasterizer's cost, which (per
  [tile-based-rasterizer](../../3dgs/rendering-and-rasterization/tile-based-rasterizer.md))
  is a function of **splat count and screen-space overlap density**, decoupled entirely
  from how deep or expensive the originating CSG tree was. A scene baked from a
  200-primitive smooth-blended tree and one baked from a single sphere cost the same to
  render afterward, if they produce similar splat counts — the CSG complexity is
  "burned off" at bake time and never paid again until the next bake.

This is exactly the same amortization logic that already justifies this project's
existing baked-vs-raymarched architecture (see
[world-representation](../architecture/world-representation.md)'s "why not raymarch the
SDF directly and skip 3DGS entirely?" section) — this document exists to make the
*efficiency* argument for that choice explicit and sourced, rather than assumed.

## Where baking clearly wins (well-supported)

- **Static or slowly-changing content relative to frame rate.** At 60fps, one second of
  a static raymarched region has already paid for ~60× more tree evaluations (per
  affected pixel) than a single bake pass touching that same region once. This project's
  own chunk-streaming design (see
  [streaming-and-invalidation](../baking-pipeline/streaming-and-invalidation.md)) already
  assumes exactly this — most chunks are static between re-bakes, and even animated
  chunks only re-bake every `REANIMATE_INTERVAL_SECS`, not every frame.
- **Deep or expensive CSG trees.** The more primitives/blend operators in the scene
  description, the more each raymarch step costs — but bake cost only grows with
  *sample count*, not with how many times each sample's tree evaluation must run beyond
  once. A deeper tree makes raymarching worse and leaves baking's per-sample cost
  essentially unchanged in kind (still one evaluation per sample, just a pricier one).
- **High output resolution.** Raymarch cost is linear in pixel count; bake cost is tied
  to surface sample density, not screen pixels — baking a chunk once and rendering it at
  4K costs the same bake as rendering it at 720p, where direct raymarching would pay
  proportionally more per frame at the higher resolution.
- **Production precedent, not just theory**: Media Molecule's *Dreams* is the strongest
  real evidence available, and it is directly on point because the team **explicitly
  tried direct SDF raytracing early in development and rejected it** — already
  documented in
  [production-case-studies](../../sdf-3d/performance-and-production/production-case-studies.md).
  Their own account (Alex Evans, SIGGRAPH 2015, "Learning from Failure") describes
  directly rendering the distance field as leaving "very little room for imagination"
  and making good-looking content difficult, with results that "just looked like
  untextured models, but slower" — a combined *performance and quality* rejection, not
  a pure benchmark number, but a real team choosing the bake-to-points architecture
  this project's bake-to-splats design is a direct descendant of, after concretely
  trying the alternative first.

## Where direct raymarching wins (also real, narrower)

- **Content changing every frame faster than any re-bake cycle can track** — e.g. a
  continuously-running physical simulation (fluid, clay) where there is no "static
  interval" to amortize a bake against at all. This is precisely Claybook's situation
  (see [production-case-studies](../../sdf-3d/performance-and-production/production-case-studies.md)):
  its clay/fluid SDF is a physically simulated field updated continuously, so baking
  would mean re-baking every frame anyway — at which point direct raymarching's
  per-frame cost and a hypothetical "re-bake + render splats" per-frame cost are
  competing on the same footing, and Claybook's shipped 60Hz raymarch-direct result
  shows that footing can clearly favor raymarching when the content genuinely never
  holds still.
- **Shallow/cheap CSG trees**, where the per-step evaluation cost is small enough that
  the step-count multiplier never gets expensive regardless of frame rate — the
  asymmetry above only matters once tree evaluation cost is non-trivial.
- **Very low screen coverage or heavily occluded scenes**, where raymarching's
  empty-space-skipping (large safe steps through open space, per
  [performance-characteristics](../../sdf-3d/performance-and-production/performance-characteristics.md))
  keeps the pixel-count side of the cost equation small regardless of CSG depth.
- **A splat density high enough to cause its own overdraw blowup.** Tile-based splat
  rendering cost isn't free of a density ceiling — per
  [tile-based-rasterizer](../../3dgs/rendering-and-rasterization/tile-based-rasterizer.md),
  heavy overlap/overdraw among many splats in one tile is itself a real cost driver,
  structurally analogous to raymarching's silhouette-cost blowup but triggered by splat
  density/LOD choice rather than geometry silhouettes. If achieving artifact-free
  coverage of a very detailed surface needs enough splats to saturate tile overdraw,
  the "splats are cheap" side of the argument weakens — **the point at which this
  crossover happens is not quantified in any source found** (see below).

## What's genuinely unmeasured — do not cite invented numbers here

Research explicitly could not find:

- A controlled, apples-to-apples benchmark rendering the *same* CSG scene both ways
  (direct raymarch vs. bake-then-splat-render) and reporting frame times for both.
- Any quantitative "crossover point" — e.g. "above N primitives/blend operators,
  baking wins" or "below M splats-per-unit-area, raymarching wins."
- Any 2023-2026 paper or talk that revisits Dreams' points-vs-raymarch decision
  specifically in a Gaussian-splatting context — the 3DGS-and-SDF literature runs
  almost entirely the *other* direction (deriving/regularizing an SDF from *trained*
  Gaussians to improve surface extraction, e.g. GS-SDF, SDF-Splats — see
  [sdf-to-gaussian-math](./sdf-to-gaussian-math.md)), not "should procedurally-generated
  SDF content be raymarched or baked to Gaussians."

This is a real, currently-open gap: the closest adjacent numbers are a raymarching
benchmark showing 5,760+ SDF evaluations per pixel for a moderately complex scene
(72 steps × 80 objects) at default settings, and 3DGS tile-rasterizer numbers showing
sustained 60+ fps for millions of *captured* splats on mid-range GPUs (RTX 3070-class) —
but these come from unrelated benchmarks (a demoscene raymarcher; photogrammetric 3DGS
captures) and are not a same-scene comparison. Treat any specific ms/fps crossover
figure as **something this project would need to measure itself**, not something to
assert from the literature.

## What this means for this project's own bake

Given the qualitative argument holds but the crossover is unmeasured, the actionable
takeaway is to **instrument, not assume**: when
[incremental-rebake](./incremental-rebake.md)'s dirty-region re-bake is implemented,
the natural validation step is to measure this project's own actual bake time per
dirty-region-sample and actual splat-render time per resulting splat count, and compare
that against what a direct raymarch of the same dirty region would have cost at the
same resolution — the same empirical, screenshot/timing-driven discipline this
project's bake pipeline has already used for other decisions (e.g. the
`chunk_bake_produces_fewer_splats_at_coarser_lod` test's timing-based verification of
the `Isometry` rotation fast path, see
[baking-pipeline](../baking-pipeline/INDEX.md)). Until that measurement exists, the
correct position is "the architecture this project already has (bake, don't raymarch)
is well-motivated by real precedent and cost-model reasoning, not yet by a project-specific
number" — which is a stronger position than either overclaiming a citable benchmark that
doesn't exist, or discarding a sound qualitative argument for lack of one.

## When to dive in

- Second-guessing whether baking to splats is worth the added architectural complexity
  versus just raymarching the SDF every frame → the amortization argument above is the
  reasoning; it favors baking for this project's actual content (streamed/LOD'd,
  mostly-static-between-re-bakes chunks), matching Dreams' precedent rather than
  Claybook's continuously-simulated-field precedent.
- Planning to add content that changes every single frame (continuous physical
  simulation, not periodic re-bake) → re-read Claybook's case specifically; that
  content profile is the one case where this project's own reasoning suggests direct
  raymarching (or a hybrid: raymarch just that region, bake everything else) may
  outperform trying to re-bake every frame — see
  [deformation-vs-rebake](./deformation-vs-rebake.md) for the related but distinct
  question of transform-update vs. re-bake for *moving-but-not-deforming* content.
- Needing a real number to justify a performance claim → there isn't one in the
  literature for this specific comparison; measure this project's own bake/render
  timings directly rather than citing an adjacent, non-equivalent benchmark.

## Related

- [SDF as world description, 3DGS as world rendering](../architecture/world-representation.md) — prerequisite: the architecture whose efficiency this questions.
- [Performance characteristics of SDF rendering](../../sdf-3d/performance-and-production/performance-characteristics.md) — prerequisite: the per-step tree-evaluation cost model.
- [Production case studies: Dreams and Claybook](../../sdf-3d/performance-and-production/production-case-studies.md) — deeper: Dreams rejected raymarching, Claybook shipped it.
- [Raymarching via compute: what it buys migera (and what it doesn't)](../../compute-shaders/raymarching-via-compute.md) — contrast: the path migera actually took.
- [Trace-pass bottleneck is not march steps](../../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — applies: measured raymarch cost in migera's renderer.
- [Where this project's SDF-to-Gaussian math sits in the published literature](./sdf-to-gaussian-math.md) — deeper: why the SDF+3DGS literature does not answer this question.
