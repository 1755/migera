---
title: "Combining SDFs: boolean CSG and smooth blending"
description: Establishes that SDF booleans are min/max with no clipping, compares polynomial, exponential and root smooth-minimum variants, explains the C² issue and the smooth-union "bulge", and how to pick blend radius k. Read before blending primitives or when a blend seam bulges or looks wrong.
type: concept
status: current
tags:
  - sdf
  - csg
  - math
  - correctness
updated: 2026-08-15
aliases:
  - smooth minimum
  - smin
  - smooth union
---

# Combining SDFs: boolean CSG and smooth blending

## Boolean CSG operations are just min/max

Because two SDFs `a` and `b` are simply scalar functions evaluated at the same point,
constructive solid geometry (CSG) boolean operations reduce to elementary scalar math —
no explicit polygon clipping, no BSP trees, no geometric intersection algorithms:

```
union(a, b)        = min(a, b)
intersection(a, b)  = max(a, b)
subtraction(a, b)   = max(a, -b)     // "a minus b": a with b's volume carved out
```

This is arguably the single biggest reason SDFs are attractive for procedural modeling:
booleans that are notoriously fiddly and failure-prone on explicit meshes (self-
intersections, non-manifold results, numerical degeneracies at coplanar faces) are
numerically trivial and always well-defined on SDFs.

The catch: `min`/`max` are **not smooth** (their derivative is discontinuous at the
crossover point), which produces a visible sharp seam/crease at the boolean boundary. For
hard mechanical shapes this is often exactly what you want. For organic shapes, it's
usually not — which motivates smooth blending.

## Smooth minimum: the core organic-blending tool

A **smooth minimum** family of functions behaves like `min` far from the crossover point,
but interpolates smoothly near it, controlled by a blend-radius parameter `k`. As `k -> 0`
it converges to the hard `min`.

### Polynomial smooth minimum (cubic, most common in practice)

```glsl
float smin(float a, float b, float k) {
    float h = clamp(0.5 + 0.5*(b-a)/k, 0.0, 1.0);
    return mix(b, a, h) - k*h*(1.0-h);
}
```

This is the most widely used variant in shader/demoscene code because it's cheap (no
transcendental functions) and gives an intuitive, controllable blend radius `k`.

### Exponential smooth minimum

```glsl
float smin(float a, float b, float k) {
    float r = exp2(-a/k) + exp2(-b/k);
    return -k*log2(r);
}
```

Smoother falloff shape than the polynomial version but costs an `exp2`/`log2` pair per
evaluation — a real consideration when a scene evaluates many smooth-blended primitives
per pixel per raymarch step (see
[performance-characteristics](../performance-and-production/performance-characteristics.md)).

### Root/quadratic smooth minimum

A third family (root-based) exists trading a different smoothness profile and cost
against the two above; the practical choice among the three families is usually driven by
desired blend silhouette shape and per-scene profiling rather than a universal "best"
answer.

### C² continuity and the "bulge" artifact

The naive polynomial smin above is only C¹ continuous (continuous first derivative but
not second), which can be visible as faceting under certain lighting. A C² polynomial
variant exists using a cubic falloff:

```
d(f0, f1, k) = (k/6) * max(1 - |f0 - f1|/k, 0)^3
smooth_union(f0, f1, k) = min(f0, f1) - d(f0, f1, k)
```

A separate, commonly-noted issue with smooth blending in general is that the blended
result can locally **bulge outward** beyond either input surface's silhouette near the
join — because the smooth minimum is not itself an exact distance field (see
[exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md)), the blended region's
"distance" values are approximate, and the resulting geometric bulge is a known,
often-discussed visual side effect rather than a bug — deliberate use of it is common (it
reads as organic "fillet" material accumulating at the joint, much like a real weld or
wax join).

## Smooth intersection and smooth subtraction

The same interpolation trick extends directly to the other two booleans by substituting
into the max-based formulas:

```glsl
float smax(float a, float b, float k) { return -smin(-a, -b, k); }
// smooth intersection: smax(a, b, k)
// smooth subtraction (a minus b): smax(a, -b, k)
```

## Choosing a blend radius `k`

`k` should be scaled relative to the size of the features being joined — too small and
the blend is imperceptible from a hard boolean; too large and small details near the join
get swallowed into a single blob, and (per the bulge note above) the outward bulge becomes
more pronounced. There's no universal default; `k` is typically exposed as a per-join
tunable parameter in procedural modeling tools (see
[sdf-modeling-tools](../state-of-the-art/sdf-modeling-tools.md)).

## When to dive in

- Building an organic/character shape from primitives → smooth union with a modest `k`
  is almost always the starting point; tune per-joint.
- Building hard-surface/mechanical shapes → prefer hard `min`/`max`/subtraction; smooth
  blending usually looks wrong for machined edges.
- Seeing unwanted bulging at a blend seam → this is the expected behavior of smooth
  minimum, not a bug; reduce `k` or reconsider whether smooth blending is appropriate for
  that particular joint.
- Profiling a scene with many blended primitives and finding the exponential smin too
  costly → switch to the polynomial variant.

## Related
- [Exact vs. bound distance fields](../fundamentals/exact-vs-bound-sdfs.md) — prerequisite: smooth blends only produce bounds.
- [Material blending](../materials-and-texturing/material-blending.md) — applies: reuses the same smin weight to blend materials across a seam.
- [Integrating a baked mesh SDF into this project's raymarcher](../mesh-conversion/hybrid-baked-and-procedural-scenes.md) — applies: migera's `smin` in `assets/shaders/raymarch.wgsl` blending a baked leaf.
- [Raymarching artifacts and fixes](../rendering/raymarching-artifacts-and-fixes.md) — deeper: pitting/punch-through at blend regions.
- [Primitive shapes](./primitive-shapes.md) — prerequisite: the shapes being combined.
