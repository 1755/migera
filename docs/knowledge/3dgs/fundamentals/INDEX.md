---
title: Fundamentals
description: The 3DGS vocabulary every other topic assumes: what the representation is, the exact per-Gaussian parameters and projection math, and how it differs mechanically from NeRF. Read first when new to Gaussian splatting.
type: index
status: current
tags:
  - 3dgs
  - math
  - rasterization
updated: 2026-09-28
---

# Fundamentals

Before touching training, rendering or optimization, you need the vocabulary: what a
Gaussian primitive stores, why the representation is shaped the way it is, and how it
differs mechanically from the NeRF-family techniques it displaced.

## Start here

Read in order: [What is 3D Gaussian Splatting?](./what-is-3dgs.md) →
[The Gaussian primitive](./gaussian-primitive-parameters.md) →
[3DGS vs. NeRF](./nerf-comparison.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [What is 3D Gaussian Splatting?](./what-is-3dgs.md) | 3DGS is an explicit, differentiable, rasterizable set of Gaussians; that single fact explains its speed and editability | First contact with 3DGS |
| [The Gaussian primitive: parameters and math](./gaussian-primitive-parameters.md) | Mean, rotation+scale covariance (7 params), opacity, degree-3 SH colour (48 params), 59 floats total, and the `Σ' = JWΣWᵀJᵀ` projection | Implementing splat storage/projection, sizing memory, or debugging close-range projection artifacts |
| [3DGS vs. NeRF: explicit rasterization vs. implicit ray marching](./nerf-comparison.md) | Why rasterized primitives render far faster than per-pixel MLP ray marching at comparable training cost, and the quality/editability trade-offs | Choosing between radiance-field techniques or explaining the speed gap |
