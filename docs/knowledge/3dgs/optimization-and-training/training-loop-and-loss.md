---
title: The training loop and loss function
description: Describes the 3DGS optimization loop (render a training view through the differentiable rasterizer, L1 + D-SSIM loss, Adam, ~30k iterations), per-parameter learning rates with position decay, and SH degree annealing. Read before implementing or debugging 3DGS training that diverges or converges poorly.
type: research
status: current
tags:
  - 3dgs
  - numerics
  - prior-art
updated: 2026-08-15
sources:
  - Kerbl et al., "3D Gaussian Splatting", SIGGRAPH 2023
aliases:
  - D-SSIM loss
  - Adam optimizer
  - spherical harmonics annealing
  - backpropagation
---

# The training loop and loss function

## The core optimization loop

3DGS training is, at its heart, ordinary gradient-based optimization: for each training
iteration, one of the input photos (with a known camera pose from
[sfm-initialization](./sfm-initialization.md)) is chosen, the current set of Gaussians is
rendered from that exact camera pose via the differentiable rasterizer (see
[tile-based-rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md)), the
rendered image is compared against the real photo via a loss function, and gradients are
backpropagated through the entire rendering pipeline — projection, sorting, alpha
blending — all the way back to every Gaussian's position, covariance, opacity, and
spherical harmonics parameters. An optimizer (Adam, in the original implementation and
most descendants) then updates all parameters based on these gradients.

This loop repeats for many thousands of iterations (commonly around 30,000 in the
original paper's standard configuration, cycling through the training photo set many
times), interleaved with periodic
[adaptive-density-control](./adaptive-density-control.md) steps that add or remove
Gaussians.

## Loss function

The training loss combines two components:

1. **An L1 (or L2) photometric loss** — direct per-pixel color difference between the
   rendered image and the ground-truth photo, the straightforward pixel-accuracy term.
2. **A D-SSIM (structural dissimilarity) term** — a perceptual/structural-similarity-
   based loss component that better captures perceived visual quality than a purely
   per-pixel color loss alone, since SSIM-family metrics are sensitive to structural
   patterns (edges, local contrast) rather than only raw color values.

The combined loss is a weighted sum of these two terms. This L1 + SSIM combination is a
standard choice in image-restoration and novel-view-synthesis literature more broadly,
not unique to 3DGS, but its specific weighting and application here is part of what the
original paper tuned for good convergence behavior.

## What gets optimized, and how

Every parameter described in
[gaussian-primitive-parameters](../fundamentals/gaussian-primitive-parameters.md) is a
free variable under gradient descent, but different parameters benefit from different
learning-rate schedules because they have very different sensitivities and scales:

- **Position** typically uses a learning rate that decays over the course of training
  (starting higher to allow large corrective moves early, decaying to allow fine
  refinement later) — this exponential learning-rate decay for position specifically is
  called out in the original paper as important for convergence stability, since position
  errors have an outsized effect on rendered image quality compared to small errors in,
  say, an individual SH coefficient.
- **Covariance (scale + rotation)**, **opacity**, and **spherical harmonics
  coefficients** are each optimized with their own learning rates, generally held more
  constant across training than position's decaying schedule.

## Spherical harmonics degree annealing

Rather than optimizing all SH coefficients (up to degree 3, 48 total per Gaussian — see
[gaussian-primitive-parameters](../fundamentals/gaussian-primitive-parameters.md)) from
the very first iteration, the original training procedure introduces higher SH degrees
**progressively**: training starts with only the zeroth-order (DC) term active — i.e.
every Gaussian starts as a simple, non-view-dependent flat-colored primitive — and
higher-order terms are unlocked incrementally as training progresses. This staged
introduction avoids the optimization difficulty of trying to fit high-frequency
view-dependent detail before the underlying geometry (position, shape) has even roughly
converged, and is a specific, deliberate design choice rather than an incidental
implementation detail.

## Interaction with adaptive density control

The loss/gradient machinery described here doesn't operate in isolation — it directly
feeds [adaptive-density-control](./adaptive-density-control.md): the same view-space
positional gradients computed during ordinary backpropagation are the signal used to
decide which Gaussians need to be split, cloned, or left alone. This tight coupling
between "what the loss gradient says needs improvement" and "where new geometric
capacity gets allocated" is a core part of why 3DGS training converges to detailed,
accurate scenes without requiring a human to manually specify where more resolution is
needed.

## Practical training characteristics

- **Iteration count**: the original paper's standard configuration runs roughly 30,000
  iterations, with quality continuing to improve (though with diminishing returns) if
  training is extended further — the paper specifically notes about an additional hour
  of training pushes quality toward state-of-the-art beyond the faster baseline
  configuration.
- **Per-scene training, not a generalizable model**: unlike some machine learning systems
  that train once and generalize to new inputs, a standard 3DGS training run produces a
  set of Gaussians specific to *one* captured scene — there is no cross-scene
  generalization in the base method (though feed-forward/generalizable variants exist as
  an active research direction, see
  [state-of-the-art](../state-of-the-art/INDEX.md)).
- **GPU memory during training** exceeds the memory needed for inference/rendering alone,
  since gradients, optimizer state (Adam maintains running moment estimates per
  parameter), and intermediate rasterization buffers must all be held simultaneously —
  a real practical constraint when training high-Gaussian-count scenes, and part of why
  alternative implementations like `gsplat` specifically target reduced training memory
  footprint (see
  [rendering-and-rasterization/rasterizer-implementations](../rendering-and-rasterization/rasterizer-implementations.md)).

## When to dive in

- Implementing or debugging a 3DGS training loop from scratch → the loss function
  (L1 + D-SSIM) and the per-parameter learning-rate scheme (especially position's decay
  and SH degree annealing) are the specific, non-obvious choices worth replicating
  faithfully rather than guessing at defaults.
- Training is unstable, diverging, or converging to poor quality → check whether SH
  degree annealing and position learning-rate decay are actually implemented/enabled
  before suspecting a deeper issue; these are common corners cut in simplified
  reimplementations.
- Wanting to understand where the "add/remove Gaussians" decisions come from → this
  page's gradient discussion is the prerequisite for
  [adaptive-density-control](./adaptive-density-control.md), which consumes these same
  gradients as its core signal.

## Related

- [Structure-from-Motion initialization](./sfm-initialization.md) — prerequisite: where the camera poses and initial Gaussians come from.
- [Adaptive density control](./adaptive-density-control.md) — deeper: consumes this loop's positional gradients to add and remove Gaussians.
- [The tile-based rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md) — prerequisite: the differentiable renderer the loop backpropagates through.
- [The Gaussian primitive: parameters and math](../fundamentals/gaussian-primitive-parameters.md) — prerequisite: the parameters being optimized.
