---
title: "Adaptive density control: splitting, cloning, and pruning"
description: Explains how 3DGS training clones small under-covering Gaussians, splits large blurry ones and prunes transparent ones from view-space positional gradients, the gradient-collision weakness of the original heuristic (fixed by AbsGS) and periodic opacity reset. Read when training leaves large regions persistently blurry.
type: research
status: current
tags:
  - 3dgs
  - correctness
  - prior-art
updated: 2026-08-15
sources:
  - Kerbl et al., "3D Gaussian Splatting", SIGGRAPH 2023
  - Ye et al., "AbsGS: Recovering Fine Details for 3D Gaussian Splatting", 2024
aliases:
  - densification
  - gradient collision
  - AbsGS
  - opacity reset
---

# Adaptive density control: splitting, cloning, and pruning

## Why this exists

Structure-from-Motion initialization (see
[sfm-initialization](./sfm-initialization.md)) seeds a scene with a *sparse* point cloud
— far too few primitives to represent fine detail, and with an initial per-Gaussian size
that's frequently wrong for the local geometry. Simply optimizing the fixed initial set
of Gaussians via gradient descent (see
[training-loop-and-loss](./training-loop-and-loss.md)) cannot fix this — no amount of
moving existing Gaussians around adds the *additional* primitives needed to represent
fine detail, and no amount of shrinking a too-large Gaussian removes the fundamental
mismatch between "one primitive" and "detail that actually needs several." Adaptive
density control is the mechanism that periodically restructures the *number* and *size*
of Gaussians during training, not just their continuous parameters — interleaved with
the ordinary gradient-descent loop at fixed intervals (e.g. every few hundred
iterations).

It has two complementary operations: **densification** (split + clone, adding
Gaussians) and **pruning** (removing Gaussians).

## The core signal: view-space positional gradients

The original method's criterion for "does this region need more Gaussians" is the
**average magnitude of the view-space positional gradient** accumulated for each
Gaussian across recent training iterations. Intuitively: if the optimizer keeps wanting
to *move* a Gaussian's projected screen-space position significantly to reduce the loss,
that's a signal the current primitive (or set of primitives) isn't adequately
representing whatever detail is there — either there isn't enough geometric capacity in
that region (under-reconstruction) or a single large Gaussian is trying to cover a
region with actual internal structure it can't represent (over-reconstruction). A
Gaussian's positional gradient magnitude exceeding a threshold, checked periodically,
triggers one of the two densification operations below.

## Densification: cloning vs. splitting

The method distinguishes two distinct failure modes with the same gradient symptom but
different fixes:

- **Cloning — for under-reconstruction.** When a *small* Gaussian has a high positional
  gradient, the interpretation is "this region is under-covered — there just isn't enough
  geometry here yet." The fix: duplicate the Gaussian into two copies at (initially)
  identical position and covariance, and adjust so total contributed intensity/opacity is
  preserved (rather than doubling it) — the two copies then diverge from each other
  during subsequent gradient descent, effectively adding capacity precisely where it was
  needed.
- **Splitting — for over-reconstruction.** When a *large* Gaussian has a high positional
  gradient, the interpretation is "this single primitive is trying to cover a region with
  more internal variation than one Gaussian can represent." The fix: replace the single
  large Gaussian with two smaller ones, generally along the Gaussian's longest axis
  (long-axis splitting), which — per more recent refinements to the original heuristic —
  minimizes disruption to the local shape/density distribution and avoids excessive
  overlap or unwanted opacity reduction that naive splitting strategies can introduce.

## The gradient collision problem (a known limitation of the original heuristic)

A specific, well-documented weakness of the original view-space-positional-gradient
criterion: for a single large Gaussian covering many pixels, the *individual pixel-wise
sub-gradients* contributing to that Gaussian's overall positional gradient can point in
different (even opposing) directions — and because the original method simply **sums**
these sub-gradients to get the Gaussian's overall gradient magnitude, opposing
sub-gradients can partially or fully **cancel out**, understating how badly that
Gaussian actually needs to be split even when it clearly should be (visible as blurry,
under-detailed large regions the naive criterion fails to flag for densification). This
is termed **gradient collision** in the literature that identified and addressed it.

The proposed fix (AbsGS and related work) replaces the naive summed gradient with a
**homodirectional view-space positional gradient**: the sum of the *absolute values* of
the pixel-wise sub-gradients, rather than the sum of the (possibly sign-canceling)
sub-gradients themselves. This directly recovers the "should be split" signal that
cancellation was masking, and is one of the more impactful refinements to the original
density-control heuristic to have emerged in the post-2023 literature — a concrete
illustration of how even a core, widely-adopted piece of the original method has
continued to see meaningful correctness improvements.

## Pruning: removing low-contribution Gaussians

Periodically (at the same or a related cadence to densification checks), Gaussians whose
opacity has been optimized down below a small threshold are **pruned** — removed from
the scene entirely. Since a near-zero-opacity Gaussian contributes negligibly to any
rendered pixel regardless of its position or shape, removing it costs essentially no
rendering-quality loss while directly reducing the total Gaussian count (and therefore
memory and rendering cost). More sophisticated pruning criteria beyond the simple
opacity threshold have since been developed — accounting for a Gaussian's actual
rendering contribution across training views, its volume, learned importance/saliency
scores, or combinations of gradient/volume/opacity signals — generally aiming to prune
more aggressively without the quality loss that naive opacity-only thresholding can
sometimes cause (see
[compression-techniques](../performance-and-compression/compression-techniques.md) for
pruning as a *post-training* compression tool, a related but distinct use case from the
*during-training* pruning described here).

## Opacity reset

A related technique used periodically during training: resetting all (or many)
Gaussians' opacity to a small fixed value at fixed intervals. This counteracts a
tendency for the optimizer to increase opacity as a "cheap" way to improve the loss
locally (since higher opacity for an already-reasonably-placed Gaussian can reduce
photometric error without fixing an underlying representation problem), forcing the
optimizer to periodically "prove" that a Gaussian's opacity should be high by having it
survive the reset and grow back through legitimate gradient signal — a regularization
technique against a specific local-optimum failure mode rather than a core densification
operation per se.

## When to dive in

- Implementing or debugging a 3DGS training pipeline's density control → the split-vs-
  clone distinction (large-and-blurry vs. small-and-undercovered) and the threshold-based
  triggering described here are the core mechanics to replicate correctly.
- Seeing persistently blurry large regions that don't improve with more training
  iterations → this is the classic gradient-collision symptom; consider whether the
  density-control implementation uses summed or absolute-value-summed sub-gradients.
- Wanting to reduce Gaussian count / memory footprint after training completes (as
  opposed to during training) → that's a related but distinct problem, covered in
  [compression-techniques](../performance-and-compression/compression-techniques.md).

## Related

- [The training loop and loss function](./training-loop-and-loss.md) — prerequisite: the gradients this mechanism consumes.
- [Structure-from-Motion initialization](./sfm-initialization.md) — prerequisite: the sparse starting set that density control grows.
- [Compression techniques](../performance-and-compression/compression-techniques.md) — contrast: post-training pruning, as opposed to pruning during training.
- [Common quality artifacts and their causes](../quality-and-artifacts/common-artifacts.md) — example: needle and blur artifacts linked to density decisions.
- [Baking a Gaussian splat cloud from an SDF](../../sdf-3dgs-bevy-integration/baking-pipeline/sdf-to-splat-baking.md) — contrast: an SDF bake picks density from curvature directly, with no densification loop.
