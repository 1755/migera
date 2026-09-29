---
title: Estimating surface normals from an SDF
description: Establishes that the shading normal is the SDF's gradient, computed with the 4-tap tetrahedron finite difference; covers the offset-h tradeoff, per-hit cost, instability at sharp edges, and when analytic gradients are worth it. Read when shading a raymarched hit or chasing normal noise on sharp geometry.
type: concept
status: current
tags:
  - sdf
  - raymarching
  - lighting
  - numerics
updated: 2026-08-15
aliases:
  - tetrahedron normal
  - finite-difference gradient
  - SDF gradient
---

# Estimating surface normals from an SDF

## The core idea

Per [what-is-an-sdf](../fundamentals/what-is-an-sdf.md), the gradient of an SDF equals
the surface normal on the zero level set: `∇f(p) = n̂(p)`. This means shading a
raymarched surface never needs separately stored/authored normal data — the same SDF
used to find the hit point also produces its normal, for free (up to a numerical
differentiation cost).

## Numerical (finite-difference) gradient

Since most SDFs used in practice (compound procedural scenes, sampled grids) don't have a
convenient analytic gradient, the standard approach is central-difference numerical
differentiation, sampling the SDF at several nearby offset points:

```glsl
vec3 estimateNormal(vec3 p) {
    const float h = 0.0001;
    const vec2 k = vec2(1, -1);
    return normalize(
        k.xyy * map(p + k.xyy*h) +
        k.yyx * map(p + k.yyx*h) +
        k.yxy * map(p + k.yxy*h) +
        k.xxx * map(p + k.xxx*h)
    );
}
```

This "tetrahedron" sampling pattern (4 taps) is the common optimized form, cheaper than
the naive 6-tap central-difference version (`+x/-x, +y/-y, +z/-z`) while giving equivalent
gradient accuracy for a smooth field.

## Cost and the epsilon tradeoff

Normal estimation costs 4-6 extra SDF evaluations *per shaded pixel* — for a scene with
an expensive combined SDF (many primitives, smooth blends, displacement), this can
meaningfully add to per-frame cost since it's paid once per final hit, not once per
raymarch step. The sample offset `h` trades noise/faceting (too large — the gradient
estimate blurs over real surface detail) against numerical precision loss (too small —
floating point subtraction of nearly-equal SDF values loses precision), and generally
needs separate tuning from the raymarch hit epsilon.

## Where the gradient is undefined

As noted in [what-is-an-sdf](../fundamentals/what-is-an-sdf.md), the true SDF gradient is
discontinuous at points equidistant from multiple closest surface points — sharp edges,
corners, and medial-axis-adjacent interior regions. Numerical finite-difference estimation
doesn't "fail" outright at these points the way an analytic gradient would be undefined,
but it does produce results sensitive to the exact sample offset and orientation, visible
as slight shading instability/aliasing along sharp edges — an inherent limitation of the
representation rather than an implementation bug, and one reason very sharp SDF edges can
look subtly different from meshed sharp edges under moving lights.

## Analytic gradients

For simple procedural primitives, an analytic gradient formula can be derived by hand
(differentiating the closed-form distance expression), avoiding the extra SDF evaluations
entirely. This is common in performance-critical shader-art and demoscene work for
individual primitives, but becomes impractical to maintain by hand once primitives are
combined through many CSG/smooth-blend/domain operators — most production raymarchers use
numerical differentiation on the full combined scene SDF rather than trying to propagate
analytic derivatives through an arbitrary operator tree.

## When to dive in

- Implementing shading for a raymarcher → the 4-tap tetrahedron pattern above is the
  standard starting point; tune `h` against your scene's typical feature size.
- Seeing shading noise/flicker on fine or sharp geometry → check whether it's a normal-
  estimation epsilon issue before suspecting the SDF construction itself.
- Chasing maximum performance in a scene with expensive combined SDFs → consider whether
  a simpler primitive-local analytic gradient is feasible, or whether reducing the tap
  count (at some quality cost) is an acceptable tradeoff.

## Related
- [What is a signed distance field?](../fundamentals/what-is-an-sdf.md) — prerequisite: why gradient = normal.
- [Sphere tracing](./sphere-tracing.md) — prerequisite: produces the hit point this note shades.
- [Analytical SDF gradients](../../hybrid-architecture/plans/analytical-sdf-gradients.md) — applies: migera's plan to replace finite-difference taps with analytic gradients in `src/hybrid`.
- [PBR shading model](../materials-and-texturing/pbr-shading-model.md) — applies: consumes the normal.
- [Raymarching artifacts and fixes](./raymarching-artifacts-and-fixes.md) — deeper: normal-noise symptom and fix.
