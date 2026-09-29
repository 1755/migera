---
title: Dynamic and 4D Gaussian Splatting
description: Surveys dynamic-scene splatting as of 2025-2026: deformation fields over canonical Gaussians versus native 4D space-time primitives (better for abrupt motion), anchor-based and consistency-focused variants, and compression for dynamic scenes. Read before reconstructing video-like captures or animating splats.
type: research
status: current
tags:
  - 3dgs
  - temporal
  - state-of-the-art
updated: 2026-08-15
aliases:
  - 4DGS
  - deformation field
  - dynamic scenes
  - video splatting
---

# Dynamic and 4D Gaussian Splatting

## The problem: extending a static representation across time

Standard 3DGS represents one static scene, trained from photos assumed to depict an
unmoving world. Representing a **dynamic** scene (video-like capture with real motion —
people, objects, deformable content) requires extending the representation across time,
and several architecturally distinct approaches have emerged, differing primarily in
whether time is treated as a native dimension of the primitive itself or as a separate
deformation applied to an otherwise-static set of Gaussians.

## Deformation-field-based approaches

The more direct extension of standard 3DGS: maintain a set of 3D Gaussians representing
a canonical/reference state of the scene, and learn a separate **deformation field** — a
function of time (and often per-Gaussian or per-point latent embeddings) that predicts
how each Gaussian's position, rotation, and/or scale should change at a given moment.
Rendering at a specific time means first applying the deformation field to warp the
canonical Gaussians into that moment's configuration, then rendering normally via the
standard rasterizer. This general pattern (4DGaussians, D3DGS, and related work)
inherits most of static 3DGS's machinery largely unchanged, adding only the deformation-
prediction component — a comparatively lower-risk extension of the base method. More
refined variants (e.g. E-D3DGS) define deformations as functions of both per-Gaussian and
temporal embeddings jointly, aiming for more expressive and temporally consistent motion
modeling than a simpler time-only deformation function would provide.

## Native 4D primitives

A more structurally different approach treats time as a genuine fourth dimension of the
primitive itself, rather than a separate warp applied after the fact: **4D-Rotor Gaussian
Splatting** represents dynamics using anisotropic Gaussians defined directly in 4D
space-time (position + time, XYZT), with a given moment's 3D scene obtained by
**temporally slicing** the 4D Gaussian at that specific time — naturally composing into
ordinary projectable 3D Gaussians for rendering. This native-4D formulation is
specifically noted as powerful for representing complicated dynamics and fine detail,
particularly scenes with **abrupt motions** — cases where a smoothly-parameterized
deformation field (the approach above) may struggle to represent sudden, discontinuous
changes cleanly, since a native 4D Gaussian's temporal extent is itself a learnable,
localized parameter rather than requiring a globally smooth deformation function to
capture a sudden event.

## Anchor-based and hybrid approaches

**Anchored 4D Gaussian Splatting** uses 4D anchor points, each storing a latent feature
used to generate the actual rendering attributes of associated neural Gaussians — echoing
the anchor-based LOD structuring seen in Octree-GS (see
[large-scene-techniques](../performance-and-compression/large-scene-techniques.md)) but
applied along the temporal dimension rather than (or in addition to) spatial scale,
suggesting these two problems (spatial LOD, temporal compactness) are converging toward
related anchor/latent-feature architectural solutions rather than being addressed with
entirely separate technique families.

**Spatial-temporal consistency approaches** (e.g. ST-4DGS) specifically target a known
weakness of naive deformation-field approaches: rendering quality that's inconsistent
across time (some moments render well, others poorly) rather than uniformly good
throughout a sequence — addressed via temporal shape regularization and temporal-aware
density control (extending the static-scene
[adaptive density control](../optimization-and-training/adaptive-density-control.md)
concept to account for how a Gaussian's needed detail level might itself vary over time,
not just across static viewpoints).

## The core architectural choice

| Approach family | Representative work | Best suited for |
|---|---|---|
| Deformation field over canonical Gaussians | 4DGaussians, D3DGS, E-D3DGS | Smooth, continuous motion; simpler extension of existing static pipelines |
| Native 4D primitives (space-time Gaussians) | 4D-Rotor Gaussian Splatting | Abrupt/discontinuous motion, fine dynamic detail |
| Anchor/latent-feature based | Anchored 4D Gaussian Splatting | Compact representation, potential synergy with spatial LOD techniques |
| Consistency-focused regularization | ST-4DGS | Fixing uneven quality across a sequence's timeline |

There is not yet a single dominant "best" architecture across this space as of
2025-2026 — the field is actively exploring these distinct formulations, each with
different strengths, rather than having converged on one clearly superior approach the
way static 3DGS itself has converged around the core tile-based-rasterizer/adaptive-
density-control formula.

## Compression for dynamic scenes

Storage cost is a compounding concern for dynamic scenes specifically, since a naive
per-frame storage approach multiplies static-scene storage costs by frame count. See
[compression-techniques](../performance-and-compression/compression-techniques.md) for
TC3DGS's specific approach (mask pruning + mixed-precision quantization + trajectory
keypoint interpolation, achieving up to 67x compression on dynamic sequences) as a
concrete illustration of how dynamic-scene-specific compression strategies differ from
static-scene compression.

## When to dive in

- Capturing or reconstructing genuinely dynamic content (video-like captures with real
  motion, not just multiple static photos) → this page's architectural comparison is the
  starting point for choosing an approach family before committing to an implementation.
- Motion in a captured scene is abrupt or discontinuous (fast movement, sudden
  appearance/disappearance) rather than smooth → deformation-field approaches may
  struggle; consider native 4D primitive approaches specifically for this case.
- Rendering quality is inconsistent across a played-back dynamic sequence (some frames
  look noticeably worse than others) → this is the specific failure mode consistency-
  focused approaches (ST-4DGS-style) address; consider whether temporal-aware density
  control is present in the pipeline being used.
- Animating or physically simulating Gaussians that were *derived from an SDF* rather
  than captured from video — a related but distinct problem, since there's no
  photographic ground truth to learn a deformation field from and geometry is already
  analytically known → see
  [deformation-vs-rebake](../../sdf-3dgs-bevy-integration/live-editing/deformation-vs-rebake.md),
  which surveys physics-driven (PhysGaussian, VR-GS) and control-point-based (SC-GS)
  realtime deformation techniques as an alternative to this page's capture-focused
  deformation-field approaches.

## Related

- [When to transform already-baked splats vs. re-bake from the SDF](../../sdf-3dgs-bevy-integration/live-editing/deformation-vs-rebake.md) — contrast: physics- and control-point-driven deformation of SDF-derived splats.
- [Compression techniques](../performance-and-compression/compression-techniques.md) — applies: TC3DGS-style compression for per-frame storage.
- [Level-of-detail, streaming, and large-scene techniques](../performance-and-compression/large-scene-techniques.md) — applies: layered streaming for dynamic playback.
