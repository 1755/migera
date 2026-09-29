---
title: Optimization and Training
description: The photographic 3DGS training pipeline end to end: COLMAP/SfM initialization, the L1 + D-SSIM gradient-descent loop, and adaptive density control that changes the primitive count. Read when setting up training or debugging poor convergence, floaters or blur.
type: index
status: current
tags:
  - 3dgs
  - correctness
  - prior-art
updated: 2026-09-28
---

# Optimization and Training

3DGS scenes are the output of a pipeline that starts from photos with known or
estimated camera poses and ends with a trained set of Gaussians. This topic covers
where the initial geometry comes from, how gradient descent refines it, and the part
most distinctive to 3DGS: how the *number* of primitives changes adaptively during
training. None of it applies to baking splats from an analytic SDF, which is a forward
problem (see [Baking a Gaussian splat cloud from an SDF](../../sdf-3dgs-bevy-integration/baking-pipeline/sdf-to-splat-baking.md)).

## Start here

Read in pipeline order: [SfM initialization](./sfm-initialization.md) →
[training loop](./training-loop-and-loss.md) →
[adaptive density control](./adaptive-density-control.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Structure-from-Motion initialization](./sfm-initialization.md) | Training needs COLMAP poses and a sparse point cloud; poor SfM coverage is a leading cause of floaters | Building a capture-to-training pipeline, or a region shows floaters or missing geometry |
| [The training loop and loss function](./training-loop-and-loss.md) | L1 + D-SSIM loss, Adam, per-parameter learning rates with position decay, SH degree annealing | Implementing training, or training diverges or converges poorly |
| [Adaptive density control: splitting, cloning, and pruning](./adaptive-density-control.md) | Gradient-driven clone/split/prune, the gradient-collision weakness and its AbsGS fix, opacity reset | Large regions stay blurry however long you train |
