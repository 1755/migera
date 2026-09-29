---
title: "Single-shape modifiers: rounding, elongation, onion, displacement"
description: Covers operators that turn one SDF into a related shape — rounding and onion (exact), elongation (exact via query clamping), displacement, twist and bend (approximate, need step damping). Read before softening, hollowing, stretching or warping a primitive, or when a displaced/twisted surface shows holes.
type: concept
status: current
tags:
  - sdf
  - primitives
  - math
  - correctness
updated: 2026-08-15
aliases:
  - rounding
  - onion shell
  - elongation
  - twist
  - bend
---

# Single-shape modifiers: rounding, elongation, onion, displacement

These operators take one SDF and transform it into a related but distinct shape, as
opposed to [combination-operators](./combination-operators.md) which combine two or more
SDFs together.

## Rounding (inflate/bevel)

Subtracting a constant from any SDF grows the shape outward by that amount uniformly in
every direction, which has the visual effect of rounding off sharp edges and corners:

```
round(p) = sdf(p) - r
```

This is distance-preserving and exact if the input SDF was exact — a genuinely powerful
result of the representation: an operation that would require nontrivial mesh
re-topology (adding bevel geometry along every edge) on an explicit mesh is a single
subtraction here. This is exactly how "Round Box" is derived from "Box" rather than being
a separate primitive formula (see
[primitive-shapes](./primitive-shapes.md)).

Adding a constant instead (rather than subtracting) shrinks the shape and sharpens
concave regions — less commonly useful but the same principle.

## Onion / shell

Taking the absolute value of an SDF and subtracting a thickness turns a solid shape into
a hollow shell of that thickness:

```
onion(p) = abs(sdf(p)) - thickness
```

Because `abs()` reflects both the inside and outside of the original surface onto each
other, applying `onion` repeatedly produces nested concentric shells — a cheap way to get
layered/laminated structures (tree rings, geode-like layers) from a single base
primitive.

## Elongation

Elongation "splits a primitive in two (or four, or eight)" along one, two, or three axes,
moves the pieces apart by an offset, and reconnects them — stretching a shape along
straight sections while preserving its rounded caps/ends exactly, unlike naive non-
uniform scaling (which, per
[exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md), turns most primitives into
bounds rather than exact fields). The standard implementation clamps the query point's
coordinates into a smaller range before evaluating the base primitive:

```glsl
float elongate(vec3 p, vec3 h) {
    return sdf(p - clamp(p, -h, h));
}
```

This is how, for example, a capsule-like "stadium" shape with flat parallel sides but
rounded caps is built from a sphere: elongate the sphere along one axis, and the caps
remain perfectly round while the middle section becomes a straight cylinder — all while
remaining an *exact* distance field, because clamping the query point is itself
distance-preserving for convex "core" regions.

## Displacement (surface detail via noise)

Adding a secondary function of position (often noise, or a sine/cosine pattern) to a base
SDF perturbs the surface with fine detail:

```
displaced(p) = sdf(p) + displacement(p)
```

This is almost never an exact SDF once displacement is added — the displacement function
generally does not satisfy the Eikonal equation itself — so it is only a bound, and only a
loose one at that (see [exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md)).
Raymarching against a displaced SDF typically needs a conservative step-damping factor
scaled to the maximum possible displacement amplitude, or raymarching can miss/skip past
fine surface detail entirely if steps remain too large. This is one of the most common
sources of visible artifacts in shader-art raymarching scenes with procedural surface
texture.

## Twist and bend

Twisting rotates the query point progressively as a function of one coordinate (e.g.
height) before evaluating the base SDF; bending curves the query point's coordinate
frame along an arc. Both are domain-warping operations:

```glsl
vec3 twist(vec3 p, float k) {
    float c = cos(k*p.y), s = sin(k*p.y);
    mat2 m = mat2(c, -s, s, c);
    return vec3(m * p.xz, p.y);
}
```

These are generally **not** distance-preserving — the amount of local stretch/compression
introduced by the warp varies with the twist/bend rate and distance from the axis, so the
result is at best an approximate bound (and can, in extreme cases, violate even the
Lipschitz-1 safety bound needed for safe raymarching if the warp rate is too aggressive
relative to the query point's distance from the twist axis). Conservative step-damping is
essential when using these operators.

## When to dive in

- Softening a hard-edged primitive → rounding (subtract a constant) is almost always the
  simplest, cheapest, exactness-preserving option — reach for it before reaching for a
  smooth-blend CSG operation if you only have one shape.
- Building a stretched/stadium shape → elongation, not non-uniform scaling, if you need
  to preserve exact distance and rounded caps.
- Adding fine surface detail (bumps, cracks, organic texture) → displacement, but budget
  for a conservative step-damping factor and expect raymarching cost to rise near
  displaced surfaces (see
  [raymarching-artifacts-and-fixes](../rendering/raymarching-artifacts-and-fixes.md)).
- Building twisted/curved organic forms (tentacles, vines, drill bits) → twist/bend, with
  the same step-damping caution as displacement.

## Related
- [Primitive shapes](./primitive-shapes.md) — prerequisite: the shapes these modifiers act on.
- [Combining SDFs](./combination-operators.md) — contrast: multi-shape operators.
- [Domain operations](./domain-operations.md) — contrast: query-point transforms for repetition/symmetry.
- [Exact vs. bound distance fields](../fundamentals/exact-vs-bound-sdfs.md) — prerequisite: which modifiers keep exactness.
- [Raymarching artifacts and fixes](../rendering/raymarching-artifacts-and-fixes.md) — applies: holes near displaced/warped surfaces.
