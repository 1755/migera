---
title: "The Gaussian primitive: parameters and math"
description: Gives the per-Gaussian parameter set (mean, rotation+scale covariance, opacity, degree-3 SH colour: 59 floats, 48 of them SH) and the Σ' = JWΣWᵀJᵀ screen projection. Read before implementing splat storage, projection or compression, or debugging close-range projection artifacts.
type: concept
status: current
tags:
  - 3dgs
  - math
  - rasterization
updated: 2026-08-15
sources:
  - Kerbl et al., "3D Gaussian Splatting for Real-Time Radiance Field Rendering", SIGGRAPH 2023
  - Zwicker et al., "EWA Splatting", 2002
aliases:
  - covariance
  - spherical harmonics
  - opacity
  - EWA projection
---

# The Gaussian primitive: parameters and math

Each of the (typically) hundreds of thousands to millions of Gaussians making up a 3DGS
scene carries a fixed set of learnable parameters. This page details what each one means
and the math connecting them to the rendered image.

## The 3D Gaussian density function

A single Gaussian primitive's spatial extent is defined by a standard multivariate
Gaussian probability density function, centered at a mean position and shaped by a
covariance matrix:

```
G(x) = exp( -1/2 (x - μ)ᵀ Σ⁻¹ (x - μ) )
```

- **μ ∈ R³** — the mean: the Gaussian's 3D position in world space.
- **Σ ∈ R³ˣ³** — the covariance matrix: how the Gaussian's density falls off in each
  direction, i.e. its size, aspect ratio, and orientation as an ellipsoid.

`G(x)` is not itself an opacity or color — it's a *falloff weight* that multiplies a
separate opacity value when compositing (see below).

## Covariance via rotation and scale decomposition

A raw covariance matrix has 6 independent degrees of freedom (it's symmetric), but
optimizing it directly via unconstrained gradient descent is numerically unstable — a
gradient step can easily produce a matrix that is no longer positive semi-definite,
which is physically meaningless (negative variance). The paper's solution is to
parameterize covariance through a factorization that's automatically valid by
construction:

```
Σ = R S Sᵀ Rᵀ
```

- **S ∈ R³ˣ³** — a diagonal scaling matrix (3 scalars: the ellipsoid's extent along each
  local axis).
- **R ∈ R³ˣ³** — a rotation matrix, in practice stored and optimized as a **quaternion**
  (4 parameters) rather than a full 3x3 matrix, both for compactness and because
  quaternions are easier to keep normalized/valid under gradient updates than raw
  rotation matrices.

This gives each Gaussian **3 scale parameters + 4 quaternion parameters = 7 learnable
shape parameters**, and any combination the optimizer produces is guaranteed to
correspond to a valid covariance matrix (positive semi-definite by construction, since
`R S Sᵀ Rᵀ` is always PSD regardless of what values `R` and `S` take) — this is the key
engineering trick that makes gradient-based covariance optimization tractable at all.

## Projecting 3D covariance to 2D screen space

Rendering requires projecting each 3D Gaussian into 2D screen space, since rasterization
happens on a pixel grid. Given the camera's viewing transformation `W` and the Jacobian
`J` of the (locally-linearized) perspective projection, the projected 2D covariance is:

```
Σ' = J W Σ Wᵀ Jᵀ
```

This is the core mathematical step of "splatting": approximating the true (nonlinear)
perspective projection of a 3D Gaussian by its local affine (linear) approximation via
the Jacobian `J` — a technique inherited directly from the EWA (Elliptical Weighted
Average) splatting literature that predates 3DGS by decades, applied here to a fully
differentiable, GPU-rasterized pipeline for the first time at this scale and quality.
Because this is an *approximation* (true perspective projection of an ellipsoid is not
exactly an ellipse), it introduces small errors that become more visible at extreme
projection angles or very close camera distances — a detail directly relevant to some
[quality-and-artifacts](../quality-and-artifacts/INDEX.md) failure modes and to
[mip-splatting-and-anti-aliasing](../quality-and-artifacts/mip-splatting-and-anti-aliasing.md).

## Opacity

A single scalar **α ∈ [0, 1]** per Gaussian, controlling how much that primitive
contributes to the final composited color at any pixel it covers (combined with the
Gaussian falloff `G(x)` — a pixel near the center of a Gaussian's projected footprint
receives close to the full `α`, while a pixel near the edge receives much less, since
`G(x)` decays toward zero away from the projected center). Opacity is a free, directly
optimized parameter, not derived from anything else, and is one of the primary levers
[adaptive density control](../optimization-and-training/adaptive-density-control.md) uses
during training — Gaussians whose opacity is optimized down near zero contribute nothing
to any rendered image and are periodically pruned away entirely.

## View-dependent color via spherical harmonics

Real surfaces are rarely perfectly diffuse (Lambertian) — specular highlights,
reflections, and other view-dependent appearance effects are extremely common in
real-world captures. Rather than storing a single fixed RGB color per Gaussian, 3DGS
stores a small set of **spherical harmonics (SH) coefficients** per Gaussian, which
define a function over viewing direction: querying the SH function from a specific camera
direction produces that Gaussian's color as seen from that direction.

The number of coefficients scales with SH order `n` as `3 × (n+1)²` (the factor of 3 for
RGB channels). Common practice uses up to degree 3 (`n=3`), giving `3 × 16 = 48`
coefficients per Gaussian — a real, non-trivial storage cost multiplied across millions of
Gaussians, and a major target of compression techniques (see
[performance-and-compression](../performance-and-compression/INDEX.md)). Higher SH degree
captures sharper, more localized view-dependent effects (tight specular highlights) at the
cost of more parameters and, per the original paper, a more gradual introduction during
training tends to improve stability (see
[training-loop-and-loss](../optimization-and-training/training-loop-and-loss.md)).

## Summary: total parameter count per Gaussian

| Parameter | Dimensionality | Purpose |
|---|---|---|
| Position (mean μ) | 3 | Where the Gaussian is centered |
| Rotation (quaternion) | 4 | Orientation of the ellipsoid |
| Scale | 3 | Size along each local axis |
| Opacity (α) | 1 | Blending contribution strength |
| Spherical harmonics (degree 3) | 48 (16 per channel × 3) | View-dependent color |

Total: **59 learnable scalar parameters per Gaussian** at full SH degree 3 — multiplied by
potentially millions of Gaussians in a scene, this is the concrete origin of 3DGS's
substantial memory footprint (commonly hundreds of MB to several GB per scene at full
quality), which motivates the entire compression literature covered in
[performance-and-compression](../performance-and-compression/INDEX.md).

## When to dive in

- Implementing or modifying a 3DGS training/rendering pipeline → this page's parameter
  breakdown is the concrete reference for what each Gaussian actually stores.
- Investigating memory footprint or designing a compression scheme → the SH coefficient
  cost (48 of the 59 total parameters) is the single largest target; see
  [compression-techniques](../performance-and-compression/compression-techniques.md).
- Debugging projection-related visual artifacts at extreme angles or close range → the
  `Σ' = JWΣWᵀJᵀ` affine approximation is the root mathematical cause; see
  [mip-splatting-and-anti-aliasing](../quality-and-artifacts/mip-splatting-and-anti-aliasing.md).

## Related

- [What is 3D Gaussian Splatting?](./what-is-3dgs.md) — prerequisite: the representation these parameters belong to.
- [Compression techniques](../performance-and-compression/compression-techniques.md) — applies: SH coefficients are the main compression target.
- [Mip-Splatting: fixing aliasing artifacts](../quality-and-artifacts/mip-splatting-and-anti-aliasing.md) — deeper: how the affine projection approximation causes aliasing.
- [The tile-based rasterizer](../rendering-and-rasterization/tile-based-rasterizer.md) — applies: where the projected 2D covariance is consumed.
- [When to transform already-baked splats vs. re-bake from the SDF](../../sdf-3dgs-bevy-integration/live-editing/deformation-vs-rebake.md) — applies: rigidly rotating a covariance (Σ' = RΣRᵀ) instead of re-baking.
