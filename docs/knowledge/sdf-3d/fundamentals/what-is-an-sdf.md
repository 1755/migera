---
title: What is a signed distance field?
description: Defines an SDF (sign = inside/outside, zero level set = surface), the Eikonal equation |∇f| = 1 that makes the value a true distance, gradient = surface normal, and the three properties behind SDF popularity. Read first when new to SDFs or when another note assumes these terms.
type: concept
status: current
tags:
  - sdf
  - math
updated: 2026-08-15
aliases:
  - Eikonal equation
  - implicit surface
  - zero level set
---

# What is a signed distance field?

## Definition

A signed distance field (SDF) is a scalar field `f: R^3 -> R` that maps every point in
space to the shortest distance from that point to the boundary of a shape (a surface),
with the **sign** encoding which side of the surface the point is on:

- `f(p) > 0` — point `p` is outside the shape; the value is the distance to the nearest
  surface point.
- `f(p) < 0` — point `p` is inside the shape; the value is the negative distance to the
  nearest surface point.
- `f(p) = 0` — point `p` is exactly on the surface. The set `{p : f(p) = 0}` is the
  **zero level set**, which implicitly defines the surface/mesh the SDF represents.

This is a form of **implicit surface** representation: instead of storing the surface
explicitly (vertices/triangles for a mesh, or points for a point cloud), the surface is
defined as *wherever a function evaluates to zero*. Everything about the shape's geometry
is recoverable by evaluating `f` at query points, rather than by looking up stored
geometry.

## The Eikonal equation

A true (exact) signed distance field satisfies the **Eikonal equation**:

```
|∇f(p)| = 1   (almost everywhere)
```

Intuitively: moving a small distance `δ` in the direction of steepest ascent of `f`
increases the distance-to-surface value by exactly `δ`. This is what makes the field a
genuine *distance* function rather than an arbitrary implicit function that merely has
the correct sign — many scalar fields can have the right zero level set without their
magnitude meaning "distance" at all (see [exact vs bound
SDFs](./exact-vs-bound-sdfs.md) for why this distinction matters practically).

The gradient of the field has a second, highly useful property: **on the surface, the
gradient equals the surface normal**:

```
∇f(p) = n̂(p)    for p on the zero level set
```

More generally, off the surface, `-∇f(p)` points toward the nearest surface point. This
is why SDFs are so convenient for rendering: the surface normal — needed for lighting —
can be computed directly from the SDF itself via numerical (or in some cases analytic)
differentiation, with no separate normal data required (see
[normal-estimation](../rendering/normal-estimation.md)).

The field is differentiable almost everywhere, but **not** at points equidistant from
multiple closest surface points (e.g. along the medial axis of a shape, such as the
center line of a torus tube, or at sharp interior/exterior corners). At those points the
gradient is discontinuous, which is a source of visual artifacts and instability in some
uses of SDFs (see [neural-and-learned-sdfs](../state-of-the-art/neural-and-learned-sdfs.md)
for why this motivates smoothed/regularized variants in learned representations).

## Why this representation is useful

Three properties, together, explain nearly all of SDFs' popularity in graphics and
robotics:

1. **Constant-size distance queries with directional information.** A single scalar
   evaluation at any point tells you both "how far to the nearest surface" and (via the
   gradient) "which direction." This is exactly what's needed to safely advance a ray
   toward a surface without overshooting — the basis of
   [sphere tracing](../rendering/sphere-tracing.md).

2. **Trivial combination.** Because SDFs are just scalar functions, boolean/CSG
   operations and smooth blends reduce to simple `min`/`max`/interpolation of two scalar
   values evaluated at the same point — no explicit geometric intersection/clipping
   algorithms are needed at all. See
   [combination-operators](../primitives-and-operators/combination-operators.md).

3. **Resolution-independence and compactness for simple shapes.** A primitive like a
   sphere or box has an exact, closed-form SDF expressible in a few lines of code,
   representing a perfectly smooth surface at *any* zoom level with no polygon budget,
   no LOD popping, and no storage cost beyond the formula itself. This breaks down for
   complex shapes captured from meshes/scans, which instead require sampled/gridded or
   learned SDF representations — see [mesh-conversion](../mesh-conversion/INDEX.md).

## Where SDFs show up beyond rendering

Because the Eikonal property gives a genuine, direction-aware distance-to-obstacle value,
SDFs (or unsigned variants) are used well outside graphics:

- **Robotics/motion planning** — collision checking and safe path planning against an
  environment SDF (e.g. iSDF, ESDF-based planners), where the distance value directly
  bounds how far a robot can move without collision.
- **Physics simulation** — Claybook (see
  [production-case-studies](../performance-and-production/production-case-studies.md))
  used SDFs as the primary physics representation because negative interior distances
  elegantly prevent tunneling in a way thin triangle shells cannot.
- **Font/vector-graphics rendering** — 2D SDFs are the standard technique for
  resolution-independent text rendering (this knowledge base focuses on 3D, but the same
  underlying math applies in 2D).
- **Geometry processing** — offsetting, morphological operations (erosion/dilation via
  SDF thresholding), and shape interpolation.

## When to dive in

- Implementing a raymarcher or SDF-based renderer → understand this page fully first,
  then go to [rendering](../rendering/INDEX.md).
- Deciding whether a shape needs an exact or bound SDF → see
  [exact-vs-bound-sdfs](./exact-vs-bound-sdfs.md).
- Wondering why a learned/neural SDF looks "bumpy" or has bad gradients away from the
  surface → the Eikonal constraint is usually only loosely enforced during training; see
  [neural-and-learned-sdfs](../state-of-the-art/neural-and-learned-sdfs.md).

## Related
- [Exact vs. bound distance fields](./exact-vs-bound-sdfs.md) — deeper: what happens when the Eikonal property only holds approximately.
- [The four ways an SDF can be stored/evaluated](./sdf-representations.md) — deeper: how the function is actually stored.
- [Sphere tracing](../rendering/sphere-tracing.md) — applies: the renderer built on the distance guarantee.
- [Normal estimation](../rendering/normal-estimation.md) — applies: using gradient = normal for shading.
