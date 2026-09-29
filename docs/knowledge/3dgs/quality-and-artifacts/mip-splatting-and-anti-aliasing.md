---
title: "Mip-Splatting: fixing aliasing artifacts"
description: Explains why original 3DGS aliases when zooming in or out of the training sampling rate (no minimum Gaussian size plus an ad hoc screen-space dilation) and Mip-Splatting's fix (CVPR 2024): a 3D smoothing filter plus a 2D Mip filter. Read when splats shimmer, alias or over-sharpen on zoom.
type: research
status: current
tags:
  - 3dgs
  - rasterization
  - correctness
  - lod
updated: 2026-08-15
sources:
  - Yu et al., "Mip-Splatting: Alias-free 3D Gaussian Splatting", CVPR 2024
aliases:
  - anti-aliasing
  - aliasing
  - low-pass filter
  - 3D smoothing filter
---

# Mip-Splatting: fixing aliasing artifacts

## The problem

The original 3DGS method exhibits strong visual artifacts when the effective **sampling
rate** changes relative to what the training views used — concretely, zooming in
(effectively increasing sampling rate/magnification beyond what training captured) or
zooming out / moving farther away (decreasing effective sampling rate) both produce
visible degradation: high-frequency noise and aliasing-like artifacts when zooming in
past the training views' effective resolution, and different but related quality loss
when viewing from farther away than training views covered.

## Root cause

Mip-Splatting's analysis (CVPR 2024, Yu et al.) traces this to two compounding issues in
the original method:

1. **No 3D frequency constraint on Gaussian size.** Nothing in the original training
   procedure prevents a Gaussian from becoming arbitrarily small relative to the sampling
   density the training views actually support — training views taken from a certain
   range of distances only constrain scene frequency content up to a corresponding
   maximum frequency (per ordinary signal-processing sampling-theorem reasoning), but the
   original method has no mechanism enforcing that Gaussians stay consistent with this
   limit. The result: Gaussians can end up smaller/sharper than the training data
   actually justified, and this excess high-frequency content becomes visible as
   artifacts specifically when the viewing sampling rate changes from what training used
   (e.g. zooming in reveals detail that was never actually reliably captured, just
   spuriously present in an overly-sharp Gaussian).
2. **The original 2D screen-space dilation approach.** The original method's approach to
   keeping small projected Gaussians from becoming sub-pixel and causing rendering
   instability was a simple 2D dilation heuristic — inflating a Gaussian's projected
   screen-space footprint slightly when it would otherwise be too small. This heuristic,
   while functional, doesn't correctly model the actual physical image-formation process
   (a camera sensor integrates light over a pixel's physical area, which is more
   accurately modeled as a box filter, not an ad hoc dilation), contributing to the
   aliasing/dilation artifacts observed.

## The fix: two complementary filters

Mip-Splatting introduces two changes, addressing each root cause directly:

1. **A 3D smoothing filter** — constrains the minimum size of each 3D Gaussian primitive
   based on the maximal sampling frequency actually induced by the input training views.
   Concretely, this applies a Gaussian low-pass filter to each primitive *before*
   projection to screen space, ensuring no primitive can represent frequency content
   finer than what the training views' sampling density could have reliably determined.
   This directly eliminates the high-frequency zoom-in artifacts, since Gaussians are no
   longer able to become spuriously sharper than the training data supports.
2. **A 2D Mip filter, replacing the original 2D dilation approach** — rather than an ad
   hoc screen-space inflation, this applies a 2D Gaussian filter that approximates a true
   physical box filter, modeling how a camera sensor actually integrates light over a
   pixel's area. This more accurately mitigates aliasing and dilation-related artifacts
   than the original heuristic, particularly visible when zoomed out (viewing from
   farther away than training views covered).

## Why "Mip" in the name

The naming references classic **mipmapping** from texture filtering — the general
principle of pre-filtering content to match the actual sampling/display rate rather than
relying on point-sampling and post-hoc anti-aliasing, applied here to Gaussian primitive
size/frequency content rather than to a 2D texture's mip chain. The conceptual parallel
is deliberate: both techniques solve the same class of problem (representing content at a
resolution appropriate to the current sampling rate) in their respective domains.

## Practical impact

This is one of the most widely adopted post-2023 refinements to the base 3DGS method —
"Mip-Splatting" or equivalent filtering is commonly enabled by default or offered as a
standard option in most current production and research 3DGS implementations, since the
aliasing problem it fixes is visible in essentially any scene where a viewer zooms or
navigates to a distance meaningfully different from the original capture's viewing
distances — a near-universal use case for interactive viewing, not a corner case.

## When to dive in

- Seeing shimmering, moiré, or excessive sharpening/blur when zooming in/out of a trained
  scene → this is almost certainly the aliasing problem Mip-Splatting addresses; check
  whether the training/rendering pipeline in use has this fix enabled before assuming a
  different root cause.
- Building or evaluating a new 3DGS implementation or pipeline → treat Mip-Splatting-
  style filtering as a near-mandatory quality baseline for any interactive-viewing use
  case, not an optional enhancement, given how universally the aliasing problem manifests
  without it.
- Distinguishing this from other artifacts (floaters, popping, needle artifacts) → see
  [common-artifacts](./common-artifacts.md) for the broader troubleshooting map; this
  page covers specifically the zoom/sampling-rate-dependent failure mode.

## Related

- [The Gaussian primitive: parameters and math](../fundamentals/gaussian-primitive-parameters.md) — prerequisite: the affine projection whose approximation this filters.
- [Common quality artifacts and their causes](./common-artifacts.md) — contrast: the other, non-zoom artifact classes.
- [The tile-based rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md) — prerequisite: the projection-and-blend stage the 2D Mip filter modifies.
- [Anti-aliasing techniques](../../bevy-rendering/post-processing/anti-aliasing.md) — contrast: screen-space AA, which cannot fix per-primitive undersampling.
