---
title: "3DGS vs. NeRF: explicit rasterization vs. implicit ray marching"
description: Contrasts NeRF (MLP queried at many ray-march samples per pixel) with 3DGS (explicit primitives rasterized once): why 3DGS renders far faster at comparable training cost, and the quality/editability trade-offs each way. Read when choosing between radiance-field techniques or explaining the speed gap.
type: research
status: current
tags:
  - 3dgs
  - raymarching
  - rasterization
  - performance
  - prior-art
updated: 2026-08-15
sources:
  - Mildenhall et al., "NeRF", ECCV 2020
  - Kerbl et al., "3D Gaussian Splatting", SIGGRAPH 2023
aliases:
  - NeRF
  - neural radiance fields
  - implicit vs explicit
---

# 3DGS vs. NeRF: explicit rasterization vs. implicit ray marching

Understanding 3DGS in contrast to Neural Radiance Fields (NeRF) — the family of methods
it displaced as the dominant real-time novel-view-synthesis technique — clarifies why
3DGS made the specific design choices it did.

## NeRF's representation: implicit, continuous, network-encoded

A NeRF represents a scene as a neural network (typically an MLP) that maps a 3D position
plus a viewing direction to a color and density value: `F_θ(x, d) -> (c, σ)`. There is no
explicit geometry stored anywhere — the entire scene is encoded in the network's learned
weights, a genuinely continuous and implicit representation.

**Rendering a NeRF** requires **volume rendering via ray marching**: for each pixel, a
ray is cast into the scene, and the network is queried at many sample points along that
ray (commonly tens to over a hundred samples per ray, sometimes with a coarse-to-fine
two-pass hierarchical sampling strategy), with the resulting colors and densities
composited via the standard volume rendering integral. This is the direct cause of NeRF's
historical rendering-speed problem: producing a single final pixel color requires dozens
of separate neural network evaluations, and a full-resolution image requires this for
every pixel — computationally expensive even on modern GPUs, and the reason original NeRF
rendering took seconds per frame rather than achieving real-time rates.

## 3DGS's representation: explicit, primitive-based, directly rasterizable

3DGS instead stores an explicit list of primitives (see
[gaussian-primitive-parameters](./gaussian-primitive-parameters.md)) with directly
addressable, directly editable parameters — no network weights encode the scene; the
Gaussians *are* the scene. Rendering requires **rasterization**, not ray marching: each
Gaussian is projected once to screen space and composited via alpha blending in a
single, GPU-friendly pass (see
[tile-based-rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md)) — no
per-pixel network evaluation, and no per-pixel iterative sampling along a ray. This is
the direct, mechanical reason 3DGS achieves real-time frame rates where NeRF historically
could not without substantial additional engineering (baking, distillation, specialized
hardware-friendly network architectures like Instant-NGP's hash grids).

## Training cost: broadly comparable, sometimes faster for 3DGS

Despite the large rendering-speed gap, the *training* time for 3DGS and fast NeRF variants
(Instant-NGP-style hash-grid NeRFs, in particular) is more comparable than the rendering
gap would suggest — the original 3DGS paper reports training times competitive with the
fastest prior NeRF methods for equivalent quality, with additional training time (up to
roughly an hour on typical scenes) pushing quality further toward state-of-the-art. Both
approaches ultimately rely on gradient-based optimization against a photometric
(image-space) loss computed from known camera poses — see
[training-loop-and-loss](../optimization-and-training/training-loop-and-loss.md) — so
neither has an inherent training-speed advantage purely from being implicit vs. explicit;
the speed differences that do exist come from implementation-specific factors (network
architecture, adaptive density control efficiency, etc.), not from the implicit/explicit
distinction itself.

## Quality tradeoffs

Neither representation strictly dominates the other on quality:

- 3DGS's explicit primitives naturally represent hard, well-defined geometric boundaries
  and can produce very sharp detail in well-captured, well-covered regions, but can
  struggle with genuinely volumetric/translucent phenomena (smoke, fog, glass) that don't
  decompose cleanly into discrete opaque-ish blobs, and its density-control heuristics
  can produce characteristic artifacts (floaters, needle-like Gaussians) discussed in
  [quality-and-artifacts](../quality-and-artifacts/INDEX.md).
- NeRF's continuous implicit function has no inherent primitive-count limit or
  discretization artifact, and can represent smoothly-varying volumetric density more
  naturally, but is harder to edit/compose (there's no addressable "part" of a neural
  network the way there's an addressable Gaussian primitive) and historically required
  more careful regularization to avoid its own characteristic failure modes (blur in
  under-observed regions, "floaters" of a different origin than 3DGS's).

## Editability and composability

This is one of the most practically significant differences for production use, beyond
raw rendering speed: because a 3DGS scene is a literal list of discrete, individually
addressable primitives, operations like selecting a region, deleting unwanted geometry,
merging two captured scenes, or applying a spatial transform to part of a scene are
comparatively natural (though not without their own challenges — see
[quality-and-artifacts](../quality-and-artifacts/INDEX.md) for editing-adjacent issues
like popping artifacts from re-sorting). A NeRF's weights have no such natural
decomposition into "parts of the scene," making this class of editing operation
fundamentally harder without additional specialized machinery.

## Why this comparison matters for choosing a technique today

As of 2025-2026, 3DGS has become the dominant choice for real-time interactive novel-view
synthesis specifically because of the rendering-speed gap described above — NeRF-family
methods remain relevant primarily where their continuous implicit representation offers a
specific advantage (certain volumetric-heavy content, some inverse-rendering/relighting
pipelines that benefit from a smooth, differentiable-everywhere density field, or
research contexts building on NeRF's specific mathematical properties) rather than as a
general real-time rendering competitor to 3DGS. See
[state-of-the-art](../state-of-the-art/INDEX.md) for where the current research frontier
sits, including hybrid approaches that borrow ideas from both families.

## When to dive in

- Deciding between a NeRF-based and 3DGS-based pipeline for a new project → real-time
  interactive viewing needs almost always favor 3DGS today; offline, quality-maximizing,
  or specifically volumetric/translucent-heavy content may still favor NeRF variants or a
  hybrid.
- Explaining to a stakeholder why 3DGS renders so much faster than earlier radiance-field
  techniques → the ray-marching-vs-rasterization distinction on this page is the concrete,
  mechanical answer, not just "it's a newer/better technique."

## Related

- [What is 3D Gaussian Splatting?](./what-is-3dgs.md) — prerequisite: the explicit representation being compared.
- [The tile-based rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md) — deeper: the mechanism behind 3DGS's rendering-speed advantage.
- [Sphere tracing](../../sdf-3d/rendering/sphere-tracing.md) — contrast: ray marching a distance field, which migera's renderer does, has a different cost profile from NeRF's volume sampling.
- [SDF as world description, 3DGS as world rendering](../../sdf-3dgs-bevy-integration/architecture/world-representation.md) — applies: cites this note for 3DGS's lack of geometry.
