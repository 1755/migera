---
title: Where this project's SDF-to-Gaussian math sits in the published literature
description: Finds 2023-2026 SDF+Gaussian papers (3DGSR, NeuSG, GSurf, PINGS) solve the inverse, photo-driven problem; SDF-to-opacity and zero-level-set projection are reusable, the bake's primitive is a classic surfel/2DGS, and curvature-to-covariance is unpublished. Read before deriving Gaussians from an SDF.
type: research
status: archived
tags:
  - sdf
  - 3dgs
  - math
  - prior-art
  - state-of-the-art
updated: 2026-08-15
sources:
  - 3DGSR, ACM TOG 2024, arXiv 2404.00409
  - Zwicker et al., "Surface Splatting", SIGGRAPH 2001
aliases:
  - SDF-to-opacity
  - surfels
  - zero level set projection
  - curvature to covariance
  - NeuSG
  - GSurf
---

# Where this project's SDF-to-Gaussian math sits in the published literature

> **Archived (2026-09-28):** the baker this positions (`migera::splat`) was deleted in commit 684490c (2026-09-06); the literature survey itself is still accurate as of 2026-08-15 if splats are ever reconsidered.

Contents: [One-sentence finding](#the-one-sentence-finding) · [Photogrammetric papers](#what-the-photogrammetric-sdfgaussian-papers-actually-do) · [Surfels and 2DGS](#what-already-is-this-projects-technique-named-correctly-surfels-and-2dgs) · [Curvature to covariance](#curvature-to-covariance-a-genuine-gap-not-a-missed-citation) · [Grid-indexed Gaussians](#a-related-structurally-useful-idea-grid-indexed-gaussians) · [When to dive in](#when-to-dive-in)

## The one-sentence finding

Deep research across the 2023-2026 literature found **no paper that does what this
project's baker does**: analytically derive Gaussian position, orientation, scale, and
opacity from an already-known, authored SDF, with no optimization loop at all. Nearly
every "SDF + Gaussian Splatting" paper solves the *opposite* problem — using a
jointly-*learned* neural SDF as a geometric regularizer while *training* Gaussians from
photographs, because standard 3DGS training doesn't otherwise know where surfaces are
(see [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) for why that
inverse-problem framing doesn't apply here at all). That means this project isn't
missing a documented technique it should adopt wholesale — but several *formulas* from
that literature are directly reusable primitives worth stealing individually, and it's
worth knowing precisely which ones and why the rest doesn't transfer.

## What the photogrammetric SDF+Gaussian papers actually do

All of the following train Gaussians from multi-view images while using an SDF as an
auxiliary geometric signal — read them for their *formulas*, not their *pipelines*,
since the pipeline (differentiable rendering, backprop into both the Gaussians and a
neural SDF simultaneously) has no counterpart in this project's forward bake at all.

- **3DGSR** (Implicit Surface Reconstruction with 3D Gaussian Splatting, ACM TOG 2024,
  [arXiv 2404.00409](https://arxiv.org/abs/2404.00409)) defines a differentiable
  **SDF-to-opacity transformation**:

  ```
  Φ_β(f(x)) = e^(−β·f(x)) / (1 + e^(−β·f(x)))²
  ```

  — a bell-shaped function of signed distance `f(x)`, peaking at `f(x) = 0` (on the
  surface) and falling off smoothly to either side, with a learnable sharpness `β`
  controlling how tightly opacity concentrates around the zero level set. This is
  directly reusable as-is: this project's bake already computes `f(x)` for every
  candidate sample as part of the Poisson-disk projection step (see
  [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) Step 1), so opacity
  could be assigned by this formula instead of (or as a soft refinement of) the current
  "near-opaque for anything accepted onto the surface" initialization — useful
  specifically for splats accepted slightly off the exact zero level set (e.g. under a
  loosened acceptance tolerance for performance), where a hard binary opaque/transparent
  split currently either includes or excludes a near-surface sample with no graceful
  falloff in between.
- The same paper's **point-to-surface loss**, `ℒ_pt = ‖f(x_g)‖₁` (pulling a Gaussian's
  center `x_g` toward the zero level set by penalizing its signed-distance magnitude),
  corresponds to a **projection step** this project's bake already performs
  procedurally rather than via a loss gradient: converge a candidate onto the surface
  via one or a few sphere-tracing-style steps along `−f(x)·∇f(x)` (see
  [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) Step 1's "project
  along the gradient direction" description). The **Neural SDF Inference through
  Splatting 3D Gaussians Pulled on Zero-Level Set** paper
  ([NeurIPS 2024](https://openreview.net/forum?id=r6tnDXIkNS)) makes this projection
  step explicit as a standalone operator: `new_position = position − f(x)·∇f(x)`, a
  single Newton-style step toward the zero set. This is worth adopting verbatim as the
  named form of the projection this project's baker already does informally — useful
  specifically for the incremental re-bake case (see
  [incremental-rebake](./incremental-rebake.md)), where re-projecting an *existing*
  splat position onto a *slightly changed* surface (rather than resampling from
  scratch) is exactly this one-step Newton update.
- **NeuSG** ([arXiv 2312.00846](https://arxiv.org/abs/2312.00846)) forces Gaussians to
  become "extremely thin" via a scale regularizer specifically so their centers
  approximate true surface points, then uses SDF-derived normals to refine point-cloud
  orientation. The "force thin, orient from SDF normal" *goal* is exactly what this
  project's Step 2 already achieves directly and exactly (via `∇f(p) = n̂(p)`, no
  regularizer needed — see below) — cited here mainly to confirm that the *target*
  shape (thin, surface-normal-aligned Gaussians) this project already produces
  analytically is the same target multiple photogrammetric papers spend a training loss
  term trying to approximate indirectly.
- **GSurf** ([arXiv 2411.15723](https://arxiv.org/abs/2411.15723)) and **Gaussian
  Splatting with Discretized SDF for Relightable Assets**
  ([arXiv 2507.15629](https://arxiv.org/html/2507.15629)) both store a discretized SDF
  sample per-Gaussian specifically to improve *relighting* quality (better normals from
  the SDF → better shading) — see
  [unified-physics-and-lighting](./unified-physics-and-lighting.md) for why this
  matters directly to this project (this project already has exact, not discretized,
  per-splat SDF/normal data at bake time, for free — the relighting motivation these
  papers work hard to earn is a baseline property here, not a research contribution
  still to unlock).
- **PINGS** (RSS 2025, [arXiv 2502.05752](https://arxiv.org/abs/2502.05752),
  [code](https://github.com/PRBonn/PINGS)) unifies a continuous SDF and a 3DGS radiance
  field in one point-based implicit neural map for LiDAR SLAM, with an explicit
  *geometric consistency* loss keeping the two fields mutually agreeing. Not directly
  reusable (it's neural and jointly optimized, built for incremental sensor-driven
  mapping, not artist-authored CSG), but it is the closest published validation that
  "one shared point-based structure serving both an SDF and a Gaussian field" is a
  coherent, working architecture rather than a naive idea — worth citing as
  corroborating precedent for this project's overall shape even though the mechanism
  differs completely.

## What already IS this project's technique, named correctly: surfels and 2DGS

The actual mathematical ancestry of "flat, surface-normal-oriented splat, tangent-plane
scale from local sampling density" is **not** the Gaussian-splatting literature at
all — it's classical **surface splatting** (Zwicker et al., SIGGRAPH 2001,
["Surface Splatting"](https://www.cs.umd.edu/~zwicker/publications/SurfaceSplatting-SIG01.pdf)),
which defined exactly this primitive (an oriented elliptical splat, or "surfel," aligned
to a surface's local tangent frame) decades before 3DGS existed. **2D Gaussian
Splatting** ([2d-gaussian-splatting](../../3dgs/state-of-the-art/2d-gaussian-splatting.md))
is, per that document, the modern re-derivation of the same primitive inside the 3DGS
formalism — a flat disk defined by a center, two tangent vectors, and a scale, with the
surfel normal recoverable as the cross product of the tangents (equivalently, the
eigenvector of the splat's covariance matrix with the smallest eigenvalue).

The practical implication: **this project's baker is not doing a novel or unusual thing
by orienting splats to the SDF gradient and flattening the normal-axis scale — it is
correctly implementing the surfel/2DGS primitive, just deriving its parameters
analytically from a known SDF instead of learning them via 2DGS's training-time
constraint or SuGaR's regularization loss** (both already documented in
[sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md)'s Step 2 as the
photogrammetric routes to the same target shape).

## Curvature to covariance: a genuine gap, not a missed citation

No paper found makes the curvature→anisotropic-scale/orientation connection **explicit
and analytic** the way this project's bake does (see
[gaussian-primitive-parameters](../../3dgs/fundamentals/gaussian-primitive-parameters.md)
for the `Σ = R S Sᵀ Rᵀ` decomposition this maps onto). The general differential-geometry
fact this project's implementation already exploits — principal curvatures are the
eigenvalues of the surface's second fundamental form (the shape operator), with
eigenvectors giving the principal curvature *directions* — is textbook differential
geometry, not a Gaussian-splatting-specific result, and the 3DGS/SDF literature searched
does not connect it explicitly to per-splat covariance the way
[sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) Step 2 already does
(scale the tangent axes from local sampling density modulated by curvature — thinner
splats in high-curvature regions). Every photogrammetric paper found instead learns an
equivalent effect indirectly via adaptive density control's split/clone heuristics (see
[adaptive-density-control](../../3dgs/optimization-and-training/adaptive-density-control.md)).
**Practical conclusion: this specific piece of this project's design is closer to novel
synthesis than to an implementation of a documented technique** — worth stating
explicitly rather than assuming a citation exists to lean on, and worth treating any
future refinement here (e.g. deriving splat aspect ratio directly from the ratio of
principal curvatures, rather than from isotropic curvature magnitude alone) as original
work to validate empirically against screenshots, the same way the anisotropic
Poisson-disk acceptance fix already was.

## A related, structurally useful idea: grid-indexed Gaussians

**GaussianCube** ([NeurIPS 2024](https://gaussiancube.github.io/),
[arXiv 2403.19655](https://arxiv.org/abs/2403.19655)) is not SDF-derived and not
directly reusable for this project's math, but its core idea — rearranging a scene's
Gaussians onto a structured voxel grid via Optimal Transport, so that Gaussians become
addressable by grid index rather than an unordered point soup — is a useful structural
precedent for [incremental-rebake](./incremental-rebake.md)'s dirty-region problem:
indexing splats by the SDF chunk/cell they were sampled from (which this project's
chunk-based streaming already does at the chunk granularity, see
[streaming-and-invalidation](../baking-pipeline/streaming-and-invalidation.md)) is the
same idea applied one level coarser, and GaussianCube is evidence that finer-grained
grid-indexing of splats is a tractable, precedented structure if sub-chunk dirty regions
are needed.

## When to dive in

- Deciding how to assign opacity to splats accepted slightly off the exact zero level
  set (e.g. under a relaxed sampling tolerance) → 3DGSR's `Φ_β(f(x))` formula above is a
  directly reusable, principled alternative to a hard accept/reject cutoff.
- Implementing incremental re-projection of an existing splat after a small local SDF
  edit, instead of resampling from scratch → the Newton-step projection
  `p − f(p)·∇f(p)` from the NeurIPS 2024 zero-level-set-pulling paper is the exact
  primitive needed; see [incremental-rebake](./incremental-rebake.md) for the
  surrounding dirty-region mechanism this projection step would run inside.
- Second-guessing whether this project's curvature-to-covariance derivation is
  reinventing a known technique → it is not; treat it as this project's own
  contribution, subject to the same empirical-verification discipline as any other
  original piece of the bake algorithm (screenshot/artifact-driven, not
  citation-driven).

## Related

- [Baking a Gaussian splat cloud from an SDF](../baking-pipeline/sdf-to-splat-baking.md) — prerequisite: the bake being positioned against the literature.
- [2D Gaussian Splatting and mesh extraction](../../3dgs/state-of-the-art/2d-gaussian-splatting.md) — deeper: the 2DGS/SuGaR primitive this bake reproduces analytically.
- [The Gaussian primitive: parameters and math](../../3dgs/fundamentals/gaussian-primitive-parameters.md) — prerequisite: the `Σ = R S Sᵀ Rᵀ` decomposition curvature maps onto.
- [Neural and learned SDFs](../../sdf-3d/state-of-the-art/neural-and-learned-sdfs.md) — contrast: learned SDFs, which most of these papers co-train.
- [Making a local SDF edit re-bake fast enough to feel live](./incremental-rebake.md) — applies: reuses the zero-level-set projection for reprojection.
- [Is baking to splats actually more efficient than raymarching the SDF directly?](./bake-vs-direct-raymarch-efficiency.md) — deeper: why this literature does not answer bake vs. raymarch.
