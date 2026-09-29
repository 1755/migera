---
title: Neural and learned SDFs
description: Surveys SDFs represented by trained MLPs — DeepSDF auto-decoders, SIREN against spectral bias, the Eikonal loss as a soft (not hard) constraint, NeuS-style reconstruction, unsigned/orthogonal variants, real-time status. Read before considering neural SDFs or when a learned SDF has bad normals.
type: research
status: current
tags:
  - sdf
  - state-of-the-art
  - numerics
  - performance
updated: 2026-08-15
aliases:
  - DeepSDF
  - SIREN
  - NeuS
  - implicit neural representation
  - Eikonal loss
---

# Neural and learned SDFs

## The core idea

Instead of storing an SDF procedurally or in a sampled grid (see
[sdf-representations](../fundamentals/sdf-representations.md)), a **neural SDF**
represents the distance field as the output of a trained neural network — typically a
multilayer perceptron (MLP) — mapping a 3D coordinate to a signed distance value,
`f_θ(p) -> R`, where `θ` are learned network weights.

## DeepSDF: the foundational approach

DeepSDF pioneered representing SDFs with deep networks for 3D shape modeling, using an
**auto-decoder** architecture: rather than a separate encoder network mapping input data
to a latent code, a per-shape latent vector is directly optimized (jointly with network
weights) during training, then held fixed and used to condition the decoder MLP at
inference to reconstruct or interpolate that specific shape. This lets a single trained
network represent an entire *family* of related shapes (e.g. all chairs in a dataset),
with different latent codes producing different specific chairs — a genuinely different
capability from procedural or grid SDFs, which each represent exactly one fixed shape.

## SIREN: fixing the spectral bias problem

A plain MLP with standard activations (ReLU, etc.) has a well-documented **spectral
bias** — it struggles to represent high-frequency detail, tending to produce
overly-smooth, low-frequency approximations of fine geometric detail no matter how it's
trained. SIREN ("Implicit Neural Representations with Periodic Activation Functions")
addresses this by using **sine activations** throughout the network instead, which
dramatically improves the network's ability to represent fine detail and — notably — to
represent *derivatives* of the signal accurately, since sine functions are smooth and
differentiable to arbitrary order in a way ReLU networks are not. This derivative
accuracy matters specifically for SDFs, where the gradient must approximate the true
surface normal (see [what-is-an-sdf](../fundamentals/what-is-an-sdf.md)) — a network that
represents the field's value well but has poor gradients produces bad normals and
therefore bad shading, even if the reconstructed surface looks acceptable in isolation.

## The Eikonal loss

Training a neural SDF to actually behave like a genuine distance field (not just a scalar
function with the right zero level set) requires explicitly encouraging the Eikonal
property `|∇f| = 1` (see [what-is-an-sdf](../fundamentals/what-is-an-sdf.md)) during
training, typically via a loss term penalizing deviation of the network's gradient
magnitude from 1 at sampled points. This is a **soft constraint** enforced via training
loss, not a hard architectural guarantee — meaning a trained neural SDF's gradient can
still deviate from true distance-field behavior in regions poorly covered by training
data, or arbitrarily far from the surface where few/no training samples exist. This is
the single most important caveat when using a neural SDF's raw output for something like
raymarching step sizes, versus using it only for surface classification/reconstruction
where gradient-magnitude accuracy matters less.

## Volume rendering with SDFs: NeuS and successors

A major line of work (NeuS and its many descendants) combines SDF-based geometry
representation with **NeRF-style volume rendering** for multi-view 3D reconstruction: a
neural SDF represents geometry (with a well-defined, extractable surface via its zero
level set — a property plain NeRF density fields lack, since NeRF has no notion of a
crisp surface boundary), while a separate or coupled radiance/color field handles
appearance, and the two are jointly optimized by rendering novel views and comparing
against real photographs. This hybrid (SDF geometry + NeRF-style differentiable volume
rendering) is significant because it gets the best of both: NeRF's photorealistic
appearance quality and gradient-based multi-view optimization, plus SDF's clean, directly
mesh-extractable geometry (see
[sdf-to-mesh-extraction](../mesh-conversion/sdf-to-mesh-extraction.md)) — a plain NeRF
density field requires an extra, lossier step (thresholding density) to extract a surface
at all.

## Unsigned and orthogonal distance field variants

Not all shapes are cleanly "inside vs. outside" — open surfaces (a single sheet of cloth,
a mesh with genuine holes rather than reconstruction artifacts) don't have a well-defined
sign at all. This motivates **unsigned distance field (UDF)** variants and further
refinements like orthogonal distance fields, which drop the sign requirement to represent
open, non-watertight, or otherwise topologically ambiguous geometry that a signed
representation fundamentally cannot express correctly. This remains an active research
area precisely because the loss of the sign also removes some of the useful structure
(inside/outside classification, the CSG boolean operations from
[combination-operators](../primitives-and-operators/combination-operators.md)) that make
signed fields so convenient — UDF research is generally trying to recover as much of that
convenience as possible without requiring watertightness.

## Real-time neural SDF evaluation

Historically, a per-query MLP forward pass was far more expensive than a procedural or
grid-sampled SDF evaluation, which kept neural SDFs largely in the offline
reconstruction/generation domain rather than real-time rendering. Current (2024-2026)
research directions pushing toward real-time viability include: small/distilled networks
specialized per-shape rather than large generalist networks, hardware tensor-core
acceleration of the MLP forward pass (mirroring how neural materials/shading research is
also leaning on tensor cores — see
[recent-research-directions](./recent-research-directions.md)), and hybrid approaches
that bake a trained neural SDF back down into a sampled grid for fast runtime evaluation
while keeping the network only for offline training/generation — trading away some of the
neural representation's compactness and continuous-resolution advantages in exchange for
grid-speed runtime queries.

## When to dive in

- Doing 3D reconstruction from images or point clouds where a clean, watertight,
  mesh-extractable surface is needed → NeuS-family SDF+volume-rendering approaches are
  the current standard starting point, ahead of plain NeRF if surface extraction matters.
- Building a shape-generation or shape-completion system → DeepSDF-style auto-decoder
  architectures are the foundational pattern to understand before evaluating newer
  variants.
- Seeing bad normals/shading from a neural SDF despite a visually plausible reconstructed
  surface → check whether an Eikonal loss was used during training, and whether the
  query region has adequate training-data coverage; this is expected behavior, not
  necessarily a bug.
- Considering neural SDFs for a real-time renderer today → understand this remains
  substantially more expensive per-query than grid or procedural alternatives; hybrid
  bake-to-grid approaches are the more production-realistic near-term path.

## Related
- [The four ways an SDF can be stored/evaluated](../fundamentals/sdf-representations.md) — prerequisite: where the neural family sits.
- [What is a signed distance field?](../fundamentals/what-is-an-sdf.md) — prerequisite: the Eikonal property the loss approximates.
- [Extracting a mesh from an SDF](../mesh-conversion/sdf-to-mesh-extraction.md) — applies: differentiable extraction for training pipelines.
- [3DGS vs. NeRF](../../3dgs/fundamentals/nerf-comparison.md) — contrast: the explicit splat representation vs. implicit neural fields.
- [Recent research directions](./recent-research-directions.md) — deeper: real-time neural fusion and other moving fronts.
