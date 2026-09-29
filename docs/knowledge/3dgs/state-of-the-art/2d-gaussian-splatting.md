---
title: 2D Gaussian Splatting and mesh extraction
description: Explains, as of 2025-2026, why standard 3DGS has no geometric grounding for mesh extraction, how 2DGS (SIGGRAPH 2024) flattens primitives into oriented surface disks, SuGaR's (CVPR 2024) surface-alignment regularizer plus Poisson meshing, and 2D-SuGaR. Read when a mesh or accurate surface must come out of splats.
type: research
status: current
tags:
  - 3dgs
  - mesh-conversion
  - state-of-the-art
updated: 2026-08-15
sources:
  - Huang et al., "2D Gaussian Splatting for Geometrically Accurate Radiance Fields", SIGGRAPH 2024
  - Guédon & Lepetit, "SuGaR", CVPR 2024
aliases:
  - 2DGS
  - SuGaR
  - surfels
  - surface reconstruction
---

# 2D Gaussian Splatting and mesh extraction

## Why 3D Gaussians are a poor fit for surface reconstruction

Standard 3D Gaussian Splatting is optimized purely for novel-view-synthesis image
quality, not for producing geometrically accurate surfaces. This has a direct
consequence: nothing in the base training objective (an image-space photometric loss,
see [training-loop-and-loss](../optimization-and-training/training-loop-and-loss.md))
rewards a Gaussian for actually lying flush against the true surface, or for its
orientation to correctly match a surface normal — a 3D ellipsoid can contribute
correctly to rendered pixel colors from the training viewpoints while being positioned
or oriented in a way that has little to do with genuine underlying geometry.
Consequently, extracting a clean, geometrically accurate mesh directly from a standard
3DGS scene tends to produce poor results — noisy, inconsistent surfaces reflecting the
representation's genuine lack of geometric grounding, not implementation bugs.

## 2D Gaussian Splatting: collapsing to oriented planar disks

2D Gaussian Splatting (2DGS, SIGGRAPH 2024, Huang et al.) addresses this directly by
changing the primitive itself: rather than a full 3D ellipsoid, each primitive is a **2D
oriented planar disk** (a "surfel" — surface element) embedded in 3D space. This is a
structural constraint, not just a training regularizer: collapsing the third dimension
means every primitive is, by construction, a flat, oriented patch rather than a
volumetric blob — directly encoding a much stronger geometric prior toward "this
represents a piece of surface" rather than "this represents some volumetric density."

The rendering process uses **perspective-correct differentiable rasterization** via
ray-splat intersection (finding where a camera ray intersects each oriented disk,
correctly accounting for perspective rather than relying purely on the same
locally-affine projection approximation 3DGS uses — see
[gaussian-primitive-parameters](../fundamentals/gaussian-primitive-parameters.md) for the
3DGS projection math this refines upon). Training additionally incorporates **depth
distortion** and **normal consistency** regularization terms specifically to encourage
stable optimization and genuinely surface-aligned results, addressing failure modes that
would otherwise arise from the more constrained (and therefore, in one sense, harder to
optimize well) 2D primitive representation.

## The payoff: view-consistent geometry

The direct result: 2DGS produces surfaces with **significantly improved geometric
accuracy and multi-view consistency** compared to 3DGS — the same surface, viewed from
different training or novel viewpoints, is represented consistently rather than
potentially differing 3D-ellipsoid interpretations happening to each satisfy their
respective local photometric constraints independently. This makes 2DGS the preferred
starting point specifically when downstream mesh extraction, geometric measurement, or
any application requiring a genuinely accurate 3D surface (not just plausible-looking
renders) is the goal, at some cost relative to plain 3DGS in raw novel-view-synthesis
image quality for view directions far from any oriented disk's plane.

## SuGaR: fast mesh extraction from Gaussian splats

SuGaR (Surface-Aligned Gaussian Splatting, CVPR 2024, Guédon & Lepetit) takes a related
but distinct approach: rather than changing the base primitive to 2D, it adds a
**regularization term during standard 3DGS training** that encourages Gaussians to align
well with the scene's actual surface, then exploits that alignment to extract a mesh
efficiently via **Poisson surface reconstruction** — a well-established, fast, scalable
classical geometry-processing technique that reconstructs a watertight surface from an
oriented point set. Because the regularization has already pushed Gaussians toward
surface alignment during training (rather than attempting mesh extraction as a purely
post-hoc step against an unconstrained scene), the resulting Poisson reconstruction is
both fast and detail-preserving, compared to what plain 3DGS mesh extraction would
produce.

## 2D-SuGaR: combining both approaches

2D-SuGaR (a more recent extension) combines ideas from both lines of work: it
incorporates monocular depth and normal priors (external, learned single-image geometric
cues, not derived from the multi-view capture itself) to improve initialization
robustness and geometric accuracy specifically for 2DGS — since 2DGS, despite its
stronger geometric prior relative to 3DGS, remains sensitive to Gaussian initialization
quality (echoing the general 3DGS initialization sensitivity discussed in
[sfm-initialization](../optimization-and-training/sfm-initialization.md) and
[capture-best-practices](../quality-and-artifacts/capture-best-practices.md), but for the
2D-disk primitive specifically). It also introduces a clustering-based pruning strategy
to remove degenerate Gaussians, and a joint mesh-Gaussian refinement step that partially
relaxes the strict 2D constraint back toward 3D primitives during a later refinement
stage — providing a stronger training signal than a purely 2D-constrained optimization
alone, while still benefiting from 2DGS's initial geometric grounding.

## Choosing an approach

| Goal | Approach |
|---|---|
| Best possible novel-view-synthesis image quality, geometry accuracy not needed | Standard 3D Gaussian Splatting |
| Accurate, view-consistent surface geometry as the primary goal (measurement, mesh extraction as the actual deliverable, downstream physics/collision use) | 2D Gaussian Splatting |
| Need a mesh from an existing/standard 3DGS-style training pipeline with minimal representation change | SuGaR-style surface-alignment regularization + Poisson extraction |
| Maximum geometric accuracy and robustness, willing to accept a more complex pipeline | 2D-SuGaR (2DGS initialization/priors + joint mesh-Gaussian refinement) |

## When to dive in

- Any application needing a genuine 3D mesh output (not just rendered images) from a
  Gaussian-splat capture — game asset creation, 3D printing, CAD/measurement, physics
  simulation input — → this page is the starting point; plain 3DGS mesh extraction
  without one of these techniques will generally disappoint.
- Choosing between 2DGS and 3DGS for a new capture pipeline where the end goal is unclear
  or dual-purpose (both good renders and usable geometry) → the tradeoff table above is
  the quick reference; note the accuracy-vs-raw-image-quality tension is real, not just a
  theoretical concern.
- Wanting to understand why a mesh extracted directly from a standard 3DGS scene looks
  noisy or geometrically implausible → the root-cause explanation at the top of this page
  (no geometric grounding in the base training objective) is the answer, not an
  implementation bug in whatever extraction tool was used.

## Related

- [The training loop and loss function](../optimization-and-training/training-loop-and-loss.md) — prerequisite: the image-only loss that leaves 3DGS without geometry.
- [Extracting a mesh from an SDF (isosurface extraction)](../../sdf-3d/mesh-conversion/sdf-to-mesh-extraction.md) — contrast: meshing from a true distance field is well-posed.
- [Baking a Gaussian splat cloud from an SDF](../../sdf-3dgs-bevy-integration/baking-pipeline/sdf-to-splat-baking.md) — applies: SDF-baked splats are surface-aligned (2DGS-like) with no regularizer.
- [Where this project's SDF-to-Gaussian math sits in the published literature](../../sdf-3dgs-bevy-integration/live-editing/sdf-to-gaussian-math.md) — deeper: surfels and 2DGS as the named form of an SDF bake.
