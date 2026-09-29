---
title: Compression techniques
description: Surveys (as of 2025-2026) the two complementary 3DGS compression levers, pruning Gaussian count and quantizing per-Gaussian storage, with calibration points (LightGaussian ~15x with faster rendering, TC3DGS ~67x for dynamic scenes). Read before shipping or streaming a trained scene or cutting splat VRAM.
type: research
status: current
tags:
  - 3dgs
  - compression
  - performance
  - state-of-the-art
updated: 2026-08-15
sources:
  - Fan et al., "LightGaussian", NeurIPS 2024
aliases:
  - LightGaussian
  - vector quantization
  - codebook
  - TC3DGS
---

# Compression techniques

## Why compression matters

A full-quality trained 3DGS scene stores roughly 59 scalar parameters per Gaussian
(see [gaussian-primitive-parameters](../fundamentals/gaussian-primitive-parameters.md)),
of which spherical harmonics coefficients alone account for 48 — multiplied across
potentially millions of Gaussians in a single scene, this produces file sizes commonly in
the hundreds of MB to multiple GB range, and correspondingly large GPU VRAM requirements
for rendering. This is a genuine practical constraint for deployment (streaming a scene
over a network, fitting a scene in a mobile device's memory budget, loading multiple
scenes simultaneously) distinct from raw rendering speed — a scene can render fast per-
Gaussian and still be impractical to distribute or load due to its raw size.

Compression research targets two independent, complementary levers: **reducing the
number of Gaussians** (pruning) and **reducing the storage cost per Gaussian**
(quantization).

## Pruning: reducing Gaussian count

Beyond the basic opacity-threshold pruning that happens *during* training as part of
[adaptive density control](../optimization-and-training/adaptive-density-control.md),
post-training compression pipelines apply more sophisticated pruning criteria to more
aggressively cut Gaussian count without proportional quality loss:

- **Importance-score-based pruning** — rather than opacity alone, score each Gaussian's
  actual measured contribution to rendered output across training views (how much it
  actually affects final pixel colors, not just its raw opacity parameter), and prune
  low-scoring Gaussians.
- **Multi-signal criteria** — combining gradient magnitude, per-pixel saliency, Gaussian
  volume, and other attribute values into a composite pruning score, generally
  outperforming any single simple threshold.
- **Principled/uncertainty-based approaches** — more recent work frames pruning as
  removing Gaussians whose contribution can be shown, with some statistical rigor
  (rather than heuristically), to be safely removable without meaningfully affecting
  rendered quality across the range of views the scene needs to support.

## Quantization: reducing per-Gaussian storage cost

- **Vector quantization** — rather than storing each Gaussian's continuous parameters at
  full precision, discretize them against a learned/optimized **codebook** of
  representative values; each Gaussian then stores only a compact codebook index rather
  than the full continuous parameter values, with the codebook itself (shared across all
  Gaussians in the scene) contributing only a small fixed overhead.
- **Sensitivity-based / mixed-precision quantization** — not all parameters are equally
  sensitive to quantization error; frameworks that use gradient information to determine
  how aggressively each parameter (or parameter group) can be quantized without harming
  quality achieve better compression-vs-quality tradeoffs than uniform quantization
  applied identically to every parameter.
- **Spherical harmonics-specific compression** — since SH coefficients are both the
  largest storage cost and, for many Gaussians, represent relatively subtle view-
  dependent variation around a dominant base color, they're a natural target for more
  aggressive quantization or reduced-order storage relative to position/opacity, which
  more directly determine basic scene structure and are correspondingly more sensitive
  to precision loss.

## Concrete reported results

Specific published compression systems illustrate the achievable range, useful as
calibration points when evaluating a compression strategy for a project:

- **LightGaussian** reports an average compression ratio of over **15x**, while
  simultaneously *increasing* rendering speed (139 FPS → 215 FPS in reported
  benchmarks) — illustrating that compression and rendering-speed improvements are
  often aligned, not opposed: fewer, more compact Gaussians generally means less data to
  move and process per frame, not just less to store on disk.
- A separate reported approach achieves a **27x** reduction in on-disk size alongside a
  **1.7x** rendering speedup.
- **TC3DGS** (targeting dynamic/temporal sequences specifically, see
  [large-scene-techniques](./large-scene-techniques.md) for related dynamic-scene
  considerations) combines per-frame mask pruning, mixed-precision quantization, and
  trajectory keypoint interpolation to shrink dynamic sequence storage by up to **67x**
  with minimal quality drop (reported under 0.4 dB PSNR difference — a small, generally
  visually negligible quality change for that level of compression).
- Scene-adaptive lattice vector quantization variants specifically target peak GPU VRAM
  reduction during rendering, not just on-disk file size — a distinct and sometimes more
  operationally important metric than disk size alone, since VRAM headroom directly gates
  what hardware a scene can be viewed on at all.

## The general compression-vs-quality relationship

Across nearly all reported compression techniques, there is a genuine but favorable
tradeoff curve: aggressive compression (very high ratios) does eventually cost visible
quality, but the reported results above consistently show substantial compression
(10-25x+) achievable with quality loss small enough to be near-imperceptible in most
practical viewing conditions — meaning that shipping an uncompressed, full-precision
trained scene is very rarely the right default for any deployment scenario beyond
research/archival purposes, given how much can be recovered essentially "for free."

## When to dive in

- Preparing a trained 3DGS scene for any kind of distribution (web deployment, mobile,
  streaming, or just reducing local storage/VRAM footprint) → compression should be a
  standard post-training step, not an afterthought reserved for scenes that are
  specifically "too big" — the quality cost is typically small enough to be worth taking
  by default.
- Choosing between pruning-focused and quantization-focused approaches → these are
  complementary, not competing — most high-ratio results (LightGaussian, the 27x
  example) combine both rather than relying on either alone.
- Working with dynamic/temporal (4D) scenes specifically, where storage cost multiplies
  across frames → see TC3DGS's approach above and
  [state-of-the-art](../state-of-the-art/INDEX.md) for the broader dynamic-scene
  landscape.

## Related

- [The Gaussian primitive: parameters and math](../fundamentals/gaussian-primitive-parameters.md) — prerequisite: the 59-float layout whose 48 SH floats dominate size.
- [Level-of-detail, streaming, and large-scene techniques](./large-scene-techniques.md) — contrast: fixes distance-dependent render cost rather than raw size.
- [Adaptive density control](../optimization-and-training/adaptive-density-control.md) — contrast: pruning during training rather than after it.
- [Dynamic and 4D Gaussian Splatting](../state-of-the-art/dynamic-4d-gaussian-splatting.md) — applies: storage multiplies per frame for dynamic scenes.
