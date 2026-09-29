---
title: Baking a Gaussian splat cloud from an SDF
description: Records migera's deleted forward SDF-to-splat bake (Poisson-disk sampling, gradient/curvature orientation, colour, packing), why it is a forward problem unlike 3DGS training, and its traps (tangent-plane jumps, degenerate seed normals, hard-edge gaps, baked specular). Read before sampling points or splats off an SDF.
type: design
status: archived
tags:
  - sdf
  - 3dgs
  - baking
  - mesh-conversion
  - correctness
updated: 2026-08-30
sources:
  - commit ad0b7fe (curvature-adaptive sizing + baked shading)
  - commit 684490c (src/splat removed)
aliases:
  - forward_bake_splats_poisson_seeded
  - Poisson-disk sampling
  - splat bake
  - surface sampling
---

# Baking a Gaussian splat cloud from an SDF

> **Archived (2026-09-28):** `migera::splat` (including `forward_bake_splats_poisson_seeded`, `create_splat`, `bake_light_response`) was deleted in commit 684490c (2026-09-06) when 3DGS was dropped; the sampling traps below still apply to any SDF surface sampler.

Contents: [Forward vs. inverse](#why-this-is-a-fundamentally-different-problem-than-standard-3dgs-training) · [Step 1 sampling](#step-1-surface-sampling) · [Step 2 orientation](#step-2-orientation-and-shape-from-the-sdf-gradient-and-curvature) · [Step 3 appearance](#step-3-appearance-color--spherical-harmonics) · [Step 4 packing](#step-4-packing-into-the-runtime-asset) · [When to re-bake](#when-to-re-bake) · [When to dive in](#when-to-dive-in)

> **Proven by a working implementation** (`migera::splat::forward_bake_splats_poisson_seeded`
> and `create_splat` in this repository, exercised by `examples/gallery.rs`): Step 1's
> curvature-adaptive spacing and Step 2's gradient-derived anisotropic orientation are
> both implemented as designed below, as of this correction — an earlier version of this
> callout claimed this was already proven via a `principal_curvature_frame` function and
> a gradient-projection-convergence mechanism that, on inspection while implementing this
> for real, **did not exist anywhere in the codebase** (only stale comments in
> `src/sdf/world.rs`/`assembly.rs`/`primitives.rs` referenced a `bake::principal_curvature_frame`
> that was never written). Treat "proven by a working implementation" as meaning exactly
> that build, not a general property of the design — re-verify against current source
> before trusting either claim in the future.
>
> Two real, non-obvious bugs surfaced building the adaptive version, both worth knowing
> before touching this code again:
> - **A Poisson-disk candidate's jump direction must be sampled from the local tangent
>   plane, not isotropically in 3D.** The original (uniform-density) sampler picked a
>   fully random 3D direction for each candidate and got away with it only because the
>   jump distance (`~1-7x min_distance`) was small enough that most directions still
>   landed near a reasonably-thick surface by chance. Once radius became adaptive (up to
>   3x larger on flat regions, per this doc's Step 1), that same 3D-random jump missed a
>   thin flat surface (an 11m-wide, 0.4m-thick ground slab, in the concrete case that
>   surfaced this) almost every attempt, silently capping the whole surface at 1 splat.
>   The fix: build an orthonormal tangent basis from the parent splat's normal, sample
>   the jump direction within that 2D plane, then snap the candidate back onto the true
>   isosurface with a handful of gradient-descent steps (`p -= sdf(p) * normal(p)`,
>   converging fast since an SDF's gradient magnitude is 1 by construction). This is
>   strictly better sampling efficiency at *any* radius, not just a large-radius
>   workaround.
> - **A hand-picked "seed" point must be verified to sit on the true isosurface, not
>   merely close enough to pass a loose acceptance tolerance.** A seed placed at a flat
>   box's exact geometric center (rather than on its surface) has a genuinely degenerate,
>   zero-length finite-difference normal at that exact symmetric point — Gram-Schmidt
>   against a zero vector produces a zero tangent basis, so every candidate spawned from
>   it degenerates too, silently stalling growth at 1 splat again. The loose seed-distance
>   check (`sdf(seed).abs() < min_distance * 0.5`, meant to allow slightly-off-surface
>   seeds) let this pass undetected; only a `println!` of the seed's own normal caught it.
>   The general lesson: verify a seed's *normal* is non-degenerate, not just that its
>   *distance* is small.
>
> A separate, still-live finding from an earlier pass at this: **a genuinely hard SDF
> edge (e.g. a sharp box corner) is a real gradient discontinuity that starves sampling
> right at the crease** — finite-difference curvature estimates are unreliable exactly at
> the discontinuity, and the gradient-descent surface-snap above can fail for candidates
> whose nearest surface point sits in the crease itself. The fix that worked was rounding
> the edge in the SDF itself (a standard `box_distance - corner_radius` trick, radius much
> smaller than the sampling spacing so it reads as visually sharp) rather than trying to
> compensate on the sampling or rendering side — see
> [primitive-shapes](../../sdf-3d/primitives-and-operators/primitive-shapes.md) for the
> general rounded-primitive technique. Widening splat overlap to paper over the gap was
> tried first and made things worse (reintroduced z-fighting at grazing angles) — the
> lesson generalizes: a sampling gap caused by a genuine geometric discontinuity needs a
> geometric fix, not a sampling-density or rendering-side patch.
>
> Step 3's "baked lighting response" is also now implemented (`bake_light_response`) —
> diffuse-only Lambertian with a Fresnel-Schlick weight, deliberately **no specular
> term**: an initial attempt used the surface normal itself as a stand-in view direction
> (baked lighting has no single camera to reference), which made the half-vector
> specular's `n·h` term track `n·l` almost exactly and fired a near-maximal, unclamped
> specular spike (color channel values over 1.0, visible as blown-out/hue-shifted
> blotches) on every lit splat regardless of the light's true reflection geometry — worse
> than omitting specular entirely. There is no single correct view direction to bake a
> specular highlight against for a splat meant to look right from every angle; dropping
> the term is the correct choice here, not a missing feature.

## Why this is a fundamentally different problem than standard 3DGS training

Standard 3DGS training (see
[training-loop-and-loss](../../3dgs/optimization-and-training/training-loop-and-loss.md))
solves an **inverse problem**: given only photographs and camera poses, gradient descent
must *discover* where geometry is, what shape it has, and what it looks like, entirely
from indirect photometric evidence — which is precisely why
[adaptive density control](../../3dgs/optimization-and-training/adaptive-density-control.md)
exists (the optimizer doesn't know a priori how many primitives are needed or where), why
[SfM initialization quality](../../3dgs/optimization-and-training/sfm-initialization.md)
matters so much (bad initial guesses are hard to correct via gradient descent alone), and
why [floaters](../../3dgs/quality-and-artifacts/common-artifacts.md) are a persistent
failure mode (under-constrained regions have no unique correct answer).

Baking from an SDF is a **forward problem**: the exact geometry — surface position,
surface normal (via the SDF gradient, `∇f(p) = n̂(p)`, see
[what-is-an-sdf](../../sdf-3d/fundamentals/what-is-an-sdf.md)), and local curvature — is
known *analytically and exactly* everywhere, for free, with no photographic evidence or
gradient descent required to discover it at all. This changes the entire character of
the pipeline: geometry placement becomes a **sampling and packing problem**, not an
**optimization problem**. This single fact is the reason SDF→splat baking can be an
order of magnitude cheaper than standard 3DGS training for equivalent visual output —
there is no 30,000-iteration loop, no gradient collision problem, no floater-trap risk
from unreliable initialization (see
[adaptive-density-control](../../3dgs/optimization-and-training/adaptive-density-control.md)
and
[capture-best-practices](../../3dgs/quality-and-artifacts/capture-best-practices.md) for
why those problems exist in the standard pipeline specifically because geometry is
*unknown* — a premise that simply does not hold here).

## Step 1: surface sampling

Sample candidate splat center positions on (or very near) the SDF's zero level set. Two
practical approaches, both well-precedented by existing techniques in the source
knowledge bases even though neither was originally framed as "for Gaussian splats":

- **Grid/isosurface-derived sampling** — run
  [marching cubes](../../sdf-3d/mesh-conversion/sdf-to-mesh-extraction.md) (or a cheaper
  partial variant that only needs vertex positions, not full triangulation) over the SDF
  at the target sampling density, and use the resulting vertex positions directly as
  candidate splat centers — reusing the exact machinery the SDF knowledge base already
  documents for mesh extraction, repurposed here for point sampling rather than
  triangulation.
- **Poisson-disk / blue-noise surface sampling** — directly sample the implicit surface
  using a Poisson-disk-style rejection process (project a candidate point along the
  gradient direction toward the zero level set — a single or few-iteration
  [sphere-tracing](../../sdf-3d/rendering/sphere-tracing.md) step converges essentially
  immediately since the starting point is already close to the surface by construction),
  producing an evenly-spaced point distribution without grid-aliasing artifacts. This is
  directly analogous to how SuGaR's mesh extraction (see
  [2d-gaussian-splatting](../../3dgs/state-of-the-art/2d-gaussian-splatting.md)) exploits
  Poisson-reconstruction-style techniques once Gaussians are known to be surface-aligned
  — here the surface alignment is given from the start rather than earned via
  regularization during training.

Sampling density should vary with local surface curvature (computable from the second
derivative / Hessian of the SDF, or approximated via finite differences of the gradient
at nearby points) — flat regions need few, large splats; highly curved or detailed
regions need many, small splats. This is the SDF-native equivalent of what
[adaptive density control](../../3dgs/optimization-and-training/adaptive-density-control.md)'s
split/clone mechanism discovers *empirically* through many training iterations; here it
can be computed directly from the field's local geometry in a single pass.

## Step 2: orientation and shape from the SDF gradient and curvature

This is the step that most clearly differentiates SDF-sourced splats from
photograph-sourced ones, and it directly imports the core insight of
[2D Gaussian Splatting](../../3dgs/state-of-the-art/2d-gaussian-splatting.md): standard
3DGS has no training signal rewarding a Gaussian for actually lying flush against the
true surface or orienting correctly to it, which is exactly why 2DGS had to
*structurally* constrain the primitive to a flat oriented disk and why SuGaR had to *add
a regularization term* to approximate the same effect. An SDF-sourced splat gets this
alignment for free and exactly, with no regularization needed at all:

- **Orientation**: the Gaussian's covariance (via the rotation component of the `Σ = R S
  Sᵀ Rᵀ` decomposition, see
  [gaussian-primitive-parameters](../../3dgs/fundamentals/gaussian-primitive-parameters.md))
  is set so one axis aligns with the SDF's surface normal at the sample point
  (`∇f(p)`), directly analogous to a 2DGS surfel's orientation — in effect, this baking
  pipeline is naturally producing **2DGS-flavored** (or near-2DGS, flattened-3D) splats
  by construction, not because 2DGS's specific loss/regularization was applied, but
  because the SDF gradient supplies the exact same information 2DGS's training has to
  discover indirectly. The remaining two axes lie in the local tangent plane.
- **Scale**: the in-plane (tangent) scale axes are set from the local sampling density
  (adjacent splat spacing, from Step 1); the normal-axis scale is kept small (a thin
  "flattened" Gaussian, again echoing 2DGS's structural constraint), optionally
  modulated by local surface curvature (higher curvature → thinner/smaller splat, to
  avoid a flat primitive poorly approximating a sharply curved patch — the same
  underlying tradeoff Marching-Cubes-vs-curvature-adaptive-resolution reasoning in
  [sdf-to-mesh-extraction](../../sdf-3d/mesh-conversion/sdf-to-mesh-extraction.md)
  reflects, applied to splat sizing instead of triangle sizing).
- **Opacity**: initialized near-opaque for splats sampled directly on a solid SDF
  surface (unlike standard 3DGS, there's no need to start conservative and let training
  discover which primitives should be visible at all — the surface *is* visible, by
  definition of being sampled from the zero level set).

## Step 3: appearance (color / spherical harmonics)

This is the one step baking genuinely cannot resolve purely analytically from the SDF
alone, because **an SDF encodes geometry, not appearance** — nothing in a distance field
says what color a surface is. Three practical sourcing strategies, usable independently
or combined per-region:

1. **Procedural material function** — the SDF scene description is commonly built from a
   tree of primitives and operators (see
   [primitives-and-operators](../../sdf-3d/primitives-and-operators/INDEX.md)); pairing
   each primitive/operator-tree node with a procedural material/color function (noise,
   triplanar texture lookup, gradient-based tinting) lets appearance be assigned
   deterministically at bake time, with **no optimization step required at all** — this
   is the fastest and most "native-to-procedural-world-generation" path, since the color
   is just another function evaluated at the same sample point geometry was derived from.
   Since a Gaussian's view-dependent appearance is optional (a flat, non-view-dependent
   color is simply SH degree 0 — see
   [gaussian-primitive-parameters](../../3dgs/fundamentals/gaussian-primitive-parameters.md)),
   purely procedural appearance can skip spherical harmonics almost entirely, which is
   also a direct storage win (see
   [compression-techniques](../../3dgs/performance-and-compression/compression-techniques.md)
   — SH is the dominant per-Gaussian storage cost).
2. **Texture/material-map projection** — if the SDF scene (or a region of it) has an
   associated conventional material definition (e.g. a triplanar-mapped PBR texture set,
   common for procedural terrain), sample it at each splat's surface position/normal
   directly, again with no optimization needed.
3. **Baked lighting response** — since normal and position are known exactly, a
   simplified BRDF/lighting evaluation (using the scene's static or baked lighting) can
   be evaluated once per splat at bake time and stored directly as the splat's base
   color/low-order SH — pre-baking direct lighting response the same way lightmaps
   pre-bake static lighting for conventional meshes, letting the *rendered* splats look
   correctly lit without needing dynamic per-frame lighting computation in the splat
   shader itself (a genuine performance win, at the cost of losing dynamic relighting —
   the same static/dynamic lighting tradeoff ordinary baked-lightmap mesh rendering
   makes, applied here to splats instead).
3. **Photographic fitting (optional, short optimization)** — if photographic reference
   exists for a region (e.g. a scanned real-world area blended into an otherwise
   procedural SDF world, per the mixed hand-placed/baked-region pattern in
   [sdf-representations](../../sdf-3d/fundamentals/sdf-representations.md)), a **short**
   gradient-descent pass *over appearance parameters only* (SH coefficients, held
   opacity/geometry fixed) can fit color to those photographs — this reuses the standard
   3DGS [training loop's loss function](../../3dgs/optimization-and-training/training-loop-and-loss.md)
   (L1 + D-SSIM against real photos) but only needs to converge appearance, not discover
   geometry from scratch, so it converges dramatically faster and does not need
   [adaptive density control](../../3dgs/optimization-and-training/adaptive-density-control.md)
   at all (geometry/primitive-count is already fixed from Steps 1-2). This is the
   pipeline's designated escape hatch for scenes needing photographic ground truth
   without abandoning the SDF-driven geometry pipeline entirely.

## Step 4: packing into the runtime asset

The sampled, oriented, colored splats are packed directly into the `SplatAsset` layout
described in
[bevy-pipeline-integration](../architecture/bevy-pipeline-integration.md) — this step is
purely mechanical (flatten position/rotation/scale/opacity/SH arrays into the buffer
layout the `RenderAsset` expects) and is where
[compression techniques](../../3dgs/performance-and-compression/compression-techniques.md)
(quantization, pruning of near-zero-opacity or redundant splats) apply directly, exactly
as they would to a photographically-trained scene — compression is agnostic to *how* the
splats were produced.

## When to re-bake

Because geometry is derived analytically from the SDF rather than discovered via
training, re-baking after an SDF scene edit (moving a primitive, changing a boolean
operation's blend radius — see
[combination-operators](../../sdf-3d/primitives-and-operators/combination-operators.md))
is comparatively cheap and requires no re-training loop — Steps 1-2 can be re-run for
just the affected spatial region (bounded by the edited primitive's influence radius,
easily computable since SDF primitives have well-defined local support), and only Step 3
needs re-evaluation for that region if the material/lighting function depends on the
change. This is a genuinely different operational profile from re-training a
photographically-captured 3DGS scene after any edit, which has no equivalent
"just re-run one step for the changed region" shortcut, since geometry itself was never
analytically known in the first place.

## When to dive in

- Implementing the actual baking pipeline → the four-step breakdown above (sample →
  orient/scale → color → pack) is the concrete algorithm; each step links to the exact
  prior-art technique (Marching Cubes sampling, 2DGS-style orientation, SuGaR-style
  Poisson surface sampling) it borrows from.
- Deciding how to source splat appearance for a specific scene → the three strategies in
  Step 3 are not mutually exclusive; mix procedural/baked-lighting for most of a
  procedurally-generated world and reserve photographic fitting for specifically
  scanned/captured regions.
- Designing an editor workflow (live SDF editing with fast visual feedback) → the
  "when to re-bake" section's region-bounded re-bake is the mechanism that makes
  interactive editing tractable, distinct from a full scene re-bake on every edit.

## Related

- [GPU compute-shader baking and Bevy asset-pipeline integration](./gpu-compute-baking.md) — deeper: running this algorithm on the GPU.
- [Streaming, LOD, and cache invalidation for large baked worlds](./streaming-and-invalidation.md) — deeper: region-scoped re-bakes.
- [Where this project's SDF-to-Gaussian math sits in the published literature](../live-editing/sdf-to-gaussian-math.md) — deeper: how this bake relates to published SDF+Gaussian papers.
- [The training loop and loss function](../../3dgs/optimization-and-training/training-loop-and-loss.md) — contrast: the inverse problem photographic training solves.
- [2D Gaussian Splatting and mesh extraction](../../3dgs/state-of-the-art/2d-gaussian-splatting.md) — contrast: 2DGS needs structural constraints to get the surface alignment this bake gets for free.
- [Estimating surface normals from an SDF](../../sdf-3d/rendering/normal-estimation.md) — prerequisite: the gradient that orients each splat.
- [Rendering grid-based and hybrid SDF representations](../../sdf-3d/rendering/hybrid-and-grid-rendering.md) — contrast: other ways of converting an SDF for faster rendering.
