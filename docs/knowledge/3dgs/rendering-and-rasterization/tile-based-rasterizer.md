---
title: The tile-based rasterizer
description: Explains the 3DGS rasterizer's five steps (16×16 tiles, per-tile culling, one entry per Gaussian-tile overlap, one depth sort per tile instead of per pixel, front-to-back blending with early termination), its alpha-blend formula and differentiability. Read before writing or debugging a splat rasterizer.
type: concept
status: current
tags:
  - 3dgs
  - rasterization
  - gpu-compute
  - performance
updated: 2026-08-15
sources:
  - Kerbl et al., "3D Gaussian Splatting", SIGGRAPH 2023
aliases:
  - tile sort
  - alpha blending
  - early termination
  - splat rasterization
---

# The tile-based rasterizer

## Why sorting was the historical bottleneck

Rendering a set of overlapping, semi-transparent primitives correctly requires
compositing them in back-to-front (or front-to-back with appropriate math) depth order
per pixel — get the order wrong and alpha blending produces visibly incorrect results
(a nearer, more opaque primitive should occlude what's behind it, not blend as if order
didn't matter). Prior point-based/splat alpha-blending approaches typically needed to
sort primitives **per pixel**, which is expensive — a genuinely different primitive-to-
pixel assignment (and therefore potentially different sort order) can exist at every
pixel, since different Gaussians cover different, overlapping screen regions.

## The tile-based solution: sort once per tile, not per pixel

3DGS's rasterizer avoids per-pixel sorting by instead sorting **once per screen tile**
for the entire image, borrowing an approach from prior GPU software-rasterization
techniques. Concretely, the algorithm:

1. **Divides the screen into fixed-size tiles** — 16×16 pixels in the original
   implementation.
2. **Culls Gaussians against the view frustum and against each tile** — for every
   Gaussian, determine which tiles its projected 2D footprint actually overlaps (most
   Gaussians only touch a small number of tiles, since their projected extent is
   typically much smaller than the full screen).
3. **Instantiates one (Gaussian, tile) entry per overlap** — a Gaussian that overlaps 4
   tiles produces 4 separate sortable entries, one per tile it touches, each tagged with
   that Gaussian's depth (view-space distance from the camera).
4. **Sorts all entries within each tile by depth** — a single, efficient GPU sort
   (rather than millions of tiny per-pixel sorts), producing one shared depth-ordered
   list of Gaussians per tile.
5. **Rasterizes each tile in parallel** — pixels within a tile all iterate through that
   tile's shared sorted Gaussian list, alpha-blending each Gaussian's contribution to
   every pixel it covers within the tile, walking front-to-back and terminating early
   once accumulated opacity reaches near-total saturation (further contributions behind
   an opaque enough surface are visually negligible, so the compositing loop can stop
   early for a given pixel once this happens — a meaningful practical speedup, not just a
   theoretical one, since many pixels saturate well before reaching the end of a long
   sorted list).

This "sort per tile, not per pixel" restructuring is the single biggest algorithmic
reason 3DGS's rasterizer achieves real-time frame rates: it reduces what would be an
enormous number of tiny, redundant per-pixel sorts into a comparatively small number of
tile-level sorts, each amortized across every pixel within that tile.

## Alpha blending formula

Within a tile, the final color at a pixel is computed via standard front-to-back alpha
compositing over the tile's depth-sorted Gaussian list:

```
C = Σᵢ cᵢ αᵢ Tᵢ ,   where Tᵢ = Πⱼ<ᵢ (1 - αⱼ)
```

Each Gaussian `i` contributes its (view-dependent, SH-derived — see
[gaussian-primitive-parameters](../fundamentals/gaussian-primitive-parameters.md)) color
`cᵢ`, weighted by its effective opacity at that pixel `αᵢ` (the primitive's opacity
parameter multiplied by the Gaussian falloff `G(x)` at that pixel's position within the
projected footprint), further weighted by the accumulated transmittance `Tᵢ` — the
fraction of light that survived passing through all *nearer* Gaussians already
composited at that pixel. This is mathematically the same alpha-compositing formula
used throughout point-based and volume-rendering graphics generally; what's specific to
3DGS is the tile-based sorting infrastructure that makes evaluating it efficient at
real-time frame rates for millions of overlapping primitives.

## Differentiability

Critically, every step of this pipeline — projection (the `Σ' = JWΣWᵀJᵀ` covariance
transform, see
[gaussian-primitive-parameters](../fundamentals/gaussian-primitive-parameters.md)),
tile assignment, and alpha blending — is implemented to be fully **differentiable**, so
the exact same rasterizer used for real-time rendering (inference) is also used during
**training** to compute the forward pass whose loss then gets backpropagated (see
[training-loop-and-loss](../optimization-and-training/training-loop-and-loss.md)). This
is a meaningful engineering property, not incidental: it means there's no separate,
possibly-inconsistent "training renderer" vs. "deployment renderer" — the same code path
serves both roles. The original implementation reports this differentiable
backpropagation requires only a **constant per-pixel memory overhead** regardless of how
many Gaussians are blended at that pixel, an important efficiency property for training
scenes with millions of primitives without memory cost scaling with blend depth.

## Why this beats general-purpose ray marching for this specific workload

Contrasted with NeRF-style ray marching (see
[nerf-comparison](../fundamentals/nerf-comparison.md)), the tile-based rasterizer's
efficiency comes from exploiting properties specific to the Gaussian-splat workload:
primitives have small, mostly-local screen-space footprints (unlike a ray marching a full
scene's implicit density field, which conceptually must consider the *entire* scene along
every ray), and the sort-once-per-tile restructuring turns an otherwise per-pixel
O(overlapping primitives) sorting cost into a shared, amortized cost. This is fundamentally
a rasterization-family optimization (closely related to how conventional triangle
rasterizers use tile/bin-based approaches for efficient GPU parallelism), applied here to
semi-transparent, depth-sorted Gaussian primitives rather than opaque, depth-tested
triangles.

## When to dive in

- Implementing a custom 3DGS rasterizer or understanding an existing implementation's
  performance characteristics → this page's five-step pipeline is the algorithmic core to
  replicate or optimize against.
- Debugging incorrect blending/occlusion in a custom renderer → verify the per-tile sort
  is actually happening correctly and that the early-termination-on-saturation logic
  isn't cutting off contributions prematurely (or, conversely, not terminating early
  enough, wasting performance).
- Comparing rasterization-based 3DGS rendering against emerging ray-tracing-based
  alternatives → see
  [ray-tracing-gaussians](./ray-tracing-gaussians.md) for the tradeoffs of an
  alternative rendering approach built for a different set of use cases (shadows,
  reflections, non-pinhole cameras) that rasterization handles poorly or not at all.

## Related

- [The Gaussian primitive: parameters and math](../fundamentals/gaussian-primitive-parameters.md) — prerequisite: the projected covariance and opacity each tile blends.
- [Ray tracing Gaussians](./ray-tracing-gaussians.md) — contrast: per-ray sorting for shadows, reflections and non-pinhole cameras.
- [Rasterizer implementations across platforms](./rasterizer-implementations.md) — applies: gsplat and WebGPU implementations of this algorithm.
- [Compositing: Weighted Sum Rendering vs. real depth-tested opaque rendering](../../sdf-3dgs-bevy-integration/render-integration/sort-free-compositing.md) — contrast: why migera's (archived) Bevy splat pass did not use this global sort.
- [Compute shader performance: best practices & anti-patterns](../../compute-shaders/performance-best-practices.md) — applies: GPU sorting, divergence and workgroup sizing for a compute rasterizer.
