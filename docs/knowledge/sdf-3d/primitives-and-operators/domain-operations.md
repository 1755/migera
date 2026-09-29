---
title: "Domain operations: repetition, symmetry, and infinite instancing"
description: Explains transforming the query point — abs() mirroring and mod() repetition, finite (clamped) repetition — which gives effectively free instancing, and the seam caveat when a shape is large relative to its period. Read before tiling/mirroring SDF content or when repeated geometry shows seams.
type: concept
status: current
tags:
  - sdf
  - csg
  - performance
  - correctness
updated: 2026-08-15
aliases:
  - domain repetition
  - symmetry
  - instancing
  - repeat_xz
---

# Domain operations: repetition, symmetry, and infinite instancing

Domain operations transform the *query point* before it's handed to the underlying SDF,
rather than transforming the SDF's output value (contrast with
[modifier-operators](./modifier-operators.md)). This category is what makes SDFs
particularly well-suited to procedural patterns that would require enormous explicit
geometry counts otherwise.

## Symmetry via absolute value

Taking the absolute value of one or more coordinates before evaluating the SDF mirrors
the shape across the corresponding plane(s), essentially for free:

```glsl
float mirrored(vec3 p) {
    p.x = abs(p.x);   // mirror across the YZ plane
    return sdf(p);
}
```

This is exact when the underlying shape doesn't straddle the mirror plane in a way that
creates new closest-point ambiguity, and a bound in the general case (the query point can
end up on the "wrong side" relative to which mirrored copy is actually closest, near the
seam). In practice it is used constantly and the artifacts are rarely visible unless
geometry is deliberately placed very close to the mirror plane.

## Domain repetition (infinite tiling)

Applying `mod()` (or a floor-based equivalent) to a coordinate before evaluating the SDF
repeats the shape infinitely along that axis, in **constant time** — the cost of
evaluating one primitive instance, not the cost of the (infinite) count of copies:

```glsl
float repeated(vec3 p, float period) {
    p.x = mod(p.x + 0.5*period, period) - 0.5*period;
    return sdf(p);
}
```

This is the single most important domain operation for building large procedural scenes
(city blocks, forests, crystal lattices, brick walls) without any per-instance storage or
draw-call cost — the entire infinite field of copies is described by one primitive
evaluated in a remapped coordinate space.

**Correctness caveat**: naive `mod()`-based repetition is only exact/safe when
neighboring cells' shapes cannot reach into an adjacent cell (i.e. the shape fits
entirely within its cell). If a shape is large or irregular relative to the repetition
period, the naive remap can return an incorrect (too-large) distance near cell boundaries,
because it only ever considers the copy in the *current* cell, not neighboring copies that
might actually be closer. Fixing this generally requires checking a small neighborhood of
cells (evaluating multiple candidate copies and taking the minimum) rather than assuming a
single remapped evaluation is always correct — a real cost/correctness tradeoff to be
aware of before repeating irregularly-shaped or large primitives.

## Limited (finite) repetition

Clamping the cell index to a fixed range after computing it (rather than repeating
infinitely) produces a finite grid of copies — combining the constant-time evaluation
benefit of domain repetition with a bounded instance count, useful for finite arrays
(windows on a building facade, teeth, fence posts) rather than truly infinite fields.

## Why this matters relative to explicit-geometry instancing

Mesh-based renderers achieve a similar effect via GPU instancing (drawing the same mesh
many times with different transforms), which still costs one draw-call-equivalent unit of
work *per instance* even if vertex data is shared. Domain repetition in SDF space instead
collapses an unbounded field of copies into a single evaluation with remapped
coordinates — there is no concept of "per-instance" cost at all until a ray actually
enters a region requiring evaluation. This is one of the clearest cases where the SDF
representation is not just "a different way to draw the same thing" but genuinely changes
the asymptotic cost structure of a scene.

## When to dive in

- Building large repetitive procedural scenes (architecture, forests, crystalline/organic
  patterns) → domain repetition is almost always dramatically cheaper than explicit
  instancing at this scale; start here.
- Repeating a shape and seeing "seams" or missing geometry at cell boundaries → check
  whether the shape is small enough relative to the period for naive single-cell `mod()`
  remapping to be valid; if not, evaluate neighboring cells too.
- Wanting a finite (not infinite) count of repeated copies → clamp the cell index rather
  than using unbounded `mod()`.

## Related
- [Exact vs. bound distance fields](../fundamentals/exact-vs-bound-sdfs.md) — prerequisite: repetition seams produce bounds, not exact fields.
- [Single-shape modifiers](./modifier-operators.md) — contrast: operators on the output value rather than the query point.
- [Glitch-free baked SDFs](../mesh-conversion/glitch-free-baked-sdfs.md) — contrast: chunked bakes must solve tile seams without the single-canonical-cell trick migera's `Node::Repeat`/`repeat_xz` (in `src/sdf/scene.rs`, `assets/shaders/raymarch.wgsl`) relies on.
