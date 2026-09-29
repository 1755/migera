---
title: Structure-from-Motion initialization
description: Explains why photographic 3DGS training starts from COLMAP camera poses and a sparse SfM point cloud, how those points seed the initial Gaussians, and why poor SfM coverage is a leading root cause of floaters. Read before building a capture-to-training pipeline or when a region shows floaters or missing geometry.
type: research
status: current
tags:
  - 3dgs
  - correctness
  - prior-art
updated: 2026-08-15
sources:
  - Schönberger & Frahm, "Structure-from-Motion Revisited" (COLMAP), CVPR 2016
  - Kerbl et al., "3D Gaussian Splatting", SIGGRAPH 2023
aliases:
  - SfM
  - COLMAP
  - point cloud initialization
---

# Structure-from-Motion initialization

## Why 3DGS needs an initial point cloud

Training a 3DGS scene from scratch (randomly initialized Gaussians with no prior
structure) is possible in principle but converges poorly and slowly in practice — the
optimizer has no signal about where in 3D space geometry actually exists, so gradient
descent has to discover scene structure essentially from nothing. The original method
sidesteps this by initializing Gaussians directly from a **sparse 3D point cloud**
produced during camera calibration via Structure-from-Motion (SfM), most commonly
**COLMAP**, the standard open-source SfM/multi-view-stereo pipeline used throughout the
novel-view-synthesis research community.

## The role of COLMAP in the pipeline

Before any Gaussian training begins, a capture (a set of photos or video frames of a
scene) is processed through COLMAP (or an equivalent SfM tool) to solve two problems
simultaneously:

1. **Camera pose estimation** — recovering the position and orientation of each input
   photo's camera in a shared 3D coordinate system, by finding and matching feature
   points across overlapping images. This is *required*, not optional — 3DGS training
   needs known camera poses for every training image, since the differentiable rendering
   loss (see [training-loop-and-loss](./training-loop-and-loss.md)) compares a rendered
   image from a specific known viewpoint against the corresponding real photo.
2. **Sparse point cloud reconstruction** — as a byproduct of pose estimation, COLMAP also
   triangulates the matched feature points into a sparse 3D point cloud (typically tens
   of thousands of points, far sparser than the final Gaussian count will be). This point
   cloud is what seeds the initial Gaussian positions.

## From sparse points to initial Gaussians

Each SfM point becomes one initial Gaussian: its position is copied directly from the SfM
point, and its initial scale/covariance is typically set based on the local point density
(e.g. distance to nearest neighboring points), giving a reasonable starting size before
any gradient-based refinement happens. Initial opacity and spherical harmonics
coefficients are set to simple defaults (a moderate opacity, and the SH DC/zeroth-order
term set from the point's color if available, with higher-order terms starting at zero)
— training then refines every one of these parameters via gradient descent (see
[training-loop-and-loss](./training-loop-and-loss.md)), while
[adaptive-density-control](./adaptive-density-control.md) adds and removes Gaussians well
beyond the initial sparse-point count as training progresses.

## Why initialization quality matters disproportionately

Initialization is not merely a minor implementation detail — poor or unreliable SfM
points are a directly documented root cause of persistent quality problems. Specifically,
regions where SfM produces spurious, incorrectly-triangulated, or missing points can seed
"floater traps": Gaussians that get placed in genuinely wrong locations (often
floating in empty space in front of or behind the true surface) and, once training has
converged around them, prove difficult or impossible for later optimization or pruning to
fully correct — research has found post-hoc interventions insufficient once a floater
trap has formed during initialization, which is why capture quality and coverage (see
[capture-best-practices](../quality-and-artifacts/capture-best-practices.md)) has an
outsized effect on final result quality compared to what one might expect from "just an
initialization step."

## Failure modes and mitigations

- **Textureless or reflective surfaces** — SfM feature matching fundamentally relies on
  finding consistent visual features across images; large flat, textureless walls,
  reflective glass, or repetitive patterns (tiled floors, brick walls) are classic SfM
  failure cases, producing sparse or absent points in exactly those regions, which then
  become likely floater/quality-loss regions in the trained 3DGS scene.
- **Insufficient view overlap** — SfM (and, downstream, 3DGS itself) needs enough
  overlapping viewpoints of every part of the scene to triangulate points reliably;
  sparse or gappy capture coverage is a direct cause of poor initialization in the
  under-covered regions.
- **Dynamic/moving content** — SfM assumes a static scene; moving objects, people, or
  foliage during capture violate this assumption and can corrupt pose estimation and
  point triangulation, not just local reconstruction quality.

## Alternatives to SfM initialization

While COLMAP-derived SfM points remain the standard starting point, alternative or
supplementary initialization sources exist and are increasingly used in more recent
pipelines: dense depth priors from monocular depth estimation networks, LiDAR or other
active-sensor point clouds when available (common in robotics/autonomous-driving capture
setups), or random/uniform initialization for synthetic scenes where no photographic
capture pipeline exists at all. These are generally used to *supplement* or *replace*
weak SfM regions rather than replace the pipeline outright, since SfM's camera pose
estimation remains necessary regardless of point-cloud source.

## When to dive in

- Setting up a 3DGS capture-to-training pipeline for the first time → COLMAP (or an
  equivalent SfM tool) is a required upstream step, not optional; budget time for it and
  verify its pose-estimation and point-cloud quality before starting Gaussian training.
- Seeing persistent floaters or missing geometry in a specific region of a trained scene
  → check that region's SfM point coverage and feature-matching quality first, before
  assuming the problem is in the Gaussian training/density-control stage itself.
- Capturing a new scene and wanting to avoid initialization problems proactively → see
  [capture-best-practices](../quality-and-artifacts/capture-best-practices.md) for
  concrete guidance on view coverage, overlap, and avoiding known SfM failure modes.

## Related

- [The training loop and loss function](./training-loop-and-loss.md) — deeper: what optimizes the Gaussians this step seeds.
- [Adaptive density control](./adaptive-density-control.md) — deeper: how the sparse initial set grows during training.
- [Common quality artifacts and their causes](../quality-and-artifacts/common-artifacts.md) — example: floaters traced back to bad SfM points.
- [Capture best practices](../quality-and-artifacts/capture-best-practices.md) — applies: preventing SfM failures at capture time.
- [Baking a Gaussian splat cloud from an SDF](../../sdf-3dgs-bevy-integration/baking-pipeline/sdf-to-splat-baking.md) — contrast: with an analytic SDF there is no SfM step at all.
