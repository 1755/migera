---
title: Common quality artifacts and their causes
description: Maps visible 3DGS defects to root causes (floaters from bad SfM init, popping from sort-based compositing, needles from extreme anisotropic scale, colour outliers from SH overfitting, zoom aliasing) with mitigations. Read first when a trained or baked splat scene looks wrong, before adding training iterations.
type: guide
status: current
tags:
  - 3dgs
  - troubleshooting
  - correctness
  - debugging
updated: 2026-08-15
aliases:
  - floaters
  - popping
  - needle artifacts
  - splat artifacts
---

# Common quality artifacts and their causes

A troubleshooting reference mapping visible 3DGS problems back to their underlying cause
elsewhere in the pipeline.

## Floaters

**Symptom**: small, disconnected blobs of geometry floating in space, unrelated to any
real surface — often only visible from certain viewing angles, appearing/disappearing or
shifting as the camera moves.

**Root causes** (multiple, often compounding):
- **Unreliable SfM initialization** — spurious or mis-triangulated Structure-from-Motion
  points seed Gaussians in genuinely wrong locations from the very start of training; see
  [sfm-initialization](../optimization-and-training/sfm-initialization.md). Research has
  found these "floater traps" resist correction once formed during initialization —
  post-hoc interventions during or after training are often insufficient, making this
  fundamentally a capture/initialization-quality problem rather than something to fix
  purely through better training hyperparameters.
- **Sparse or gappy view coverage** — regions seen from too few angles are
  under-constrained; the optimizer has too much freedom to place geometry in ways that
  happen to satisfy the few available training views without corresponding to real
  surface structure (this is the same failure mode that dominates sparse-view
  reconstruction more generally, an active, distinct research area — see
  [state-of-the-art](../state-of-the-art/INDEX.md)).
- **Frequency bias in training** combined with low-quality initialization compounds this
  further, producing over-shrunk (rather than correctly-positioned) Gaussians that read
  visually as floaters or fine speckling.

**Mitigation**: prioritize capture quality and coverage (see
[capture-best-practices](./capture-best-practices.md)) over post-hoc fixes; some
research directions specifically target floater suppression via regularization or
generative restoration, but robust initialization remains the most reliable lever.

## Popping artifacts

**Symptom**: sudden, discrete visual "jumps" in rendered appearance during smooth camera
motion — a region's color or detail changes abruptly rather than continuously as the
viewpoint rotates.

**Root cause**: the [tile-based rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md)
computes depth order via a **global sort of primitives per tile for the current view**.
As the camera moves, the relative depth ordering of two overlapping (or near-overlapping)
Gaussians can flip — and because this is a discrete sort, not a continuous quantity, the
change in composited color at that flip point is discontinuous rather than gradual. This
is a structural property of sort-based alpha compositing, not a bug in any specific
implementation, though its visibility can be reduced by regularization that discourages
degenerate, nearly-coplanar or heavily-overlapping Gaussian configurations in the first
place.

## Needle-like artifacts

**Symptom**: thin, elongated, spike-like Gaussians visible as visual noise, especially
around fine detail or high-frequency regions.

**Root cause**: extreme anisotropic scaling — a Gaussian whose covariance decomposition
(see
[gaussian-primitive-parameters](../fundamentals/gaussian-primitive-parameters.md))
produces one very large scale axis and two very small ones, effectively making the
primitive needle-shaped rather than a reasonably compact ellipsoid. This can emerge from
[adaptive density control](../optimization-and-training/adaptive-density-control.md)
splitting/optimization dynamics attempting to represent thin or high-frequency structures
with unconstrained scale ratios, and is one of the specific failure modes some
regularization-focused follow-up work targets directly (constraining scale-ratio extremes
during optimization).

## Color outliers from spherical harmonics overfitting

**Symptom**: isolated Gaussians with visually implausible, saturated, or otherwise wrong
colors — most noticeable from viewing angles far from those well-represented in the
training set.

**Root cause**: [spherical harmonics](../fundamentals/gaussian-primitive-parameters.md)
coefficients are optimized to fit the *training views specifically* — a Gaussian visible
from only a narrow range of training angles has an under-constrained SH function outside
that range, and the optimizer has no signal to keep the extrapolated color plausible for
unseen viewing directions. This is a direct consequence of the per-scene, non-
generalizable nature of standard 3DGS training (see
[nerf-comparison](../fundamentals/nerf-comparison.md)) — there's no prior pushing SH
coefficients toward physically plausible behavior outside the observed view cone unless
one is explicitly added via regularization.

## Aliasing when changing sampling rate (zoom/distance)

**Symptom**: strong visual artifacts (shimmering, moiré-like patterns, or excessive
blur) when the effective sampling rate changes relative to what training views used —
most commonly, zooming in or moving the camera much closer than any training photo was
captured from, or zooming/moving far out.

**Root cause**: a distinct, well-studied failure mode with its own dedicated fix — see
[mip-splatting-and-anti-aliasing](./mip-splatting-and-anti-aliasing.md) for the full
technical explanation and solution (a 3D smoothing filter plus a 2D Mip filter replacing
the original method's simpler 2D dilation approach).

## When to dive in

- Any visible 3DGS quality problem → this page is the first stop to map symptom to root
  cause before assuming a training bug or reaching for more training iterations, which
  frequently does not fix artifacts whose root cause is initialization or capture
  quality rather than under-training.
- Aliasing/zoom-related artifacts specifically → skip straight to
  [mip-splatting-and-anti-aliasing](./mip-splatting-and-anti-aliasing.md), a distinct and
  well-solved problem separate from the artifacts on this page.
- Planning a new capture and wanting to avoid these problems proactively rather than
  fixing them after the fact → [capture-best-practices](./capture-best-practices.md).

## Related

- [Structure-from-Motion initialization](../optimization-and-training/sfm-initialization.md) — deeper: the usual origin of floaters.
- [Mip-Splatting: fixing aliasing artifacts](./mip-splatting-and-anti-aliasing.md) — deeper: the zoom-aliasing failure mode and its fix.
- [Capture best practices](./capture-best-practices.md) — applies: preventing these artifacts before training.
- [The tile-based rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md) — deeper: the per-tile sort that causes popping.
- [Compositing: Weighted Sum Rendering vs. real depth-tested opaque rendering](../../sdf-3dgs-bevy-integration/render-integration/sort-free-compositing.md) — example: migera's (archived) splat renderer fought popping and sort-order artifacts.
