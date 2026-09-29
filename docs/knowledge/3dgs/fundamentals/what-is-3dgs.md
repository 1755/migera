---
title: What is 3D Gaussian Splatting?
description: Defines 3DGS (Kerbl et al., SIGGRAPH 2023) as an explicit, differentiable set of anisotropic 3D Gaussians that is rasterized rather than ray-marched, and explains why that one choice gives its speed and editability. Read first when new to Gaussian splatting.
type: concept
status: current
tags:
  - 3dgs
  - rasterization
  - prior-art
updated: 2026-08-15
sources:
  - Kerbl, Kopanas, Leimkühler, Drettakis, "3D Gaussian Splatting for Real-Time Radiance Field Rendering", ACM TOG / SIGGRAPH 2023
aliases:
  - gaussian splatting
  - novel view synthesis
  - radiance field
---

# What is 3D Gaussian Splatting?

## Definition and origin

3D Gaussian Splatting (3DGS) is a scene representation and real-time rendering technique
introduced by Kerbl, Kopanas, Leimkühler, and Drettakis in "3D Gaussian Splatting for
Real-Time Radiance Field Rendering" (SIGGRAPH / ACM TOG, July 2023). It represents a
captured 3D scene as a large collection (typically hundreds of thousands to millions) of
**anisotropic 3D Gaussian primitives** — soft, fuzzy, ellipsoidal "blobs" of color and
opacity — that, when rendered together, reproduce photorealistic novel views of the
original scene.

The paper's central claim, which drove its rapid adoption: real-time rendering (≥30 fps
at 1080p) with visual quality matching or exceeding the best NeRF-based methods of the
time, while also training faster than most of them. This combination — quality
competitive with the best implicit neural methods, but with the rendering speed of an
explicit, rasterizable representation — is what made 3DGS the dominant novel-view-
synthesis technique within roughly a year of publication.

## The core idea: explicit, differentiable, splattable primitives

Prior state-of-the-art radiance field methods (NeRF and its many variants) represented a
scene *implicitly*, as a neural network queried by ray marching — expensive per-pixel,
since many network evaluations are needed along each camera ray to render a single pixel
(see [nerf-comparison](./nerf-comparison.md) for the detailed contrast). 3DGS instead uses
an **explicit** representation: a point cloud of Gaussian primitives, each with its own
learned parameters, that can be directly **rasterized** — projected to screen space and
alpha-blended — using GPU techniques closely related to conventional point/splat
rendering, requiring no per-pixel neural network evaluation at all.

This explicit-and-rasterizable property is the single fact that explains nearly
everything else distinctive about 3DGS:

- It's fast to render because rasterization is fast and GPU-friendly (see
  [tile-based-rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md)).
- It's editable/composable in ways implicit neural fields are not, because individual
  Gaussians are addressable primitives, not weights buried inside a network.
- It can be optimized (trained) via ordinary gradient descent because the entire
  rendering pipeline — projection, sorting, alpha blending — is differentiable, so
  standard backpropagation flows from a pixel-space image loss all the way back to each
  Gaussian's parameters.

## What a scene "is" in 3DGS

A trained 3DGS scene is literally just a list of Gaussian primitives, each carrying (see
[gaussian-primitive-parameters](./gaussian-primitive-parameters.md) for full detail):

- a 3D **position** (mean),
- a 3D **covariance** (shape/size/orientation — how the ellipsoid is stretched and
  rotated),
- an **opacity** scalar,
- **view-dependent color**, typically encoded as spherical harmonics coefficients.

There is no mesh, no explicit surface, no texture map, and no scene graph — just a large,
unordered set of these primitives. Rendering a novel view means projecting every relevant
Gaussian to screen space and compositing them together (see
[rendering-and-rasterization](../rendering-and-rasterization/INDEX.md)); nothing about the
representation privileges any particular "front" surface the way a mesh's polygons do.

## Why Gaussians specifically

The choice of a Gaussian (rather than, say, a disk, a sphere, or a general ellipsoid with
hard edges) as the base primitive is deliberate and load-bearing for several reasons
covered in more depth elsewhere in this knowledge base:

- Gaussians have closed-form, well-understood behavior under affine projection — a 3D
  Gaussian projected (approximately) to screen space remains a 2D Gaussian, which is what
  makes the EWA-splatting-based projection step tractable and differentiable (see
  [gaussian-primitive-parameters](./gaussian-primitive-parameters.md)).
- Gaussians have smooth, infinite (but rapidly decaying) support, which gives them
  naturally soft edges — useful for representing genuinely fuzzy real-world detail (hair,
  foliage, translucency) without special-casing, unlike hard-edged primitives.
- The exponential falloff of a Gaussian's density means its *gradient* with respect to
  its parameters (position, shape, opacity) is well-behaved and useful for gradient-based
  optimization — a genuinely different mathematical object from, say, a hard-edged
  primitive whose boundary produces a discontinuous (non-differentiable) gradient
  everywhere except in special formulations.

## Relationship to point clouds and splatting

3DGS is often described as "differentiable point-based rendering" or "splatting," and
this framing is accurate: the technique builds on decades of prior point-cloud/splat
rendering research (EWA splatting, point-based graphics), but the specific combination of
(a) making every splat parameter differentiable and (b) coupling that with an adaptive
density-control training loop that adds and removes primitives on the fly (see
[adaptive-density-control](../optimization-and-training/adaptive-density-control.md)) is
what makes 3DGS a genuinely new capability rather than simply an application of known
splat-rendering techniques to a new dataset.

## When to dive in

- New to 3DGS entirely → read this page, then
  [gaussian-primitive-parameters](./gaussian-primitive-parameters.md), then
  [nerf-comparison](./nerf-comparison.md) for the full conceptual picture before touching
  training or rendering code.
- Deciding whether 3DGS is the right technique for a project → see
  [nerf-comparison](./nerf-comparison.md) for the tradeoffs against implicit neural
  fields, and
  [performance-and-compression](../performance-and-compression/INDEX.md) for concrete
  cost numbers.

## Related

- [The Gaussian primitive: parameters and math](./gaussian-primitive-parameters.md) — deeper: the exact per-Gaussian parameter set and projection math.
- [3DGS vs. NeRF](./nerf-comparison.md) — contrast: why explicit rasterization beats implicit ray marching for real-time viewing.
- [The tile-based rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md) — deeper: the algorithm that makes the explicit representation real-time.
- [SDF as world description, 3DGS as world rendering](../../sdf-3dgs-bevy-integration/architecture/world-representation.md) — applies: migera's (now archived) plan to bake SDFs into splats.
