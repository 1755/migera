---
title: The canonical 3D SDF primitive set
description: Catalogs Inigo Quilez's standard 3D SDF primitives (sphere, box, torus, capsule, cones, prisms, ~28 total), marking which are exact and which are bounds (ellipsoid, triangular prism, cheap octahedron). Read when adding a primitive to src/sdf/primitives.rs or choosing building blocks for a scene.
type: reference
status: current
tags:
  - sdf
  - primitives
  - math
  - correctness
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/sdf/primitives.rs
sources:
  - https://iquilezles.org/articles/distfunctions/
aliases:
  - iquilezles distfunctions
  - capsule SDF
  - round cone SDF
---

# The canonical 3D SDF primitive set

Inigo Quilez's ["3D Signed Distance Functions"](https://iquilezles.org/articles/distfunctions/)
article is the field's de facto reference — nearly every SDF library, shader-art scene,
and modeling tool traces its primitive formulas back to it. This page catalogs the
primitive set and which are exact vs. approximate.

## Exact primitives

These have closed-form formulas that satisfy the Eikonal equation exactly (see
[what-is-an-sdf](../fundamentals/what-is-an-sdf.md)) — the returned value is the true
Euclidean distance everywhere:

- **Sphere** — `length(p) - r`. The simplest possible SDF; the archetype every
  explanation of the concept starts with.
- **Box** / **Round Box** — exact distance to an axis-aligned box, with a rounding-radius
  variant for beveled edges (built from the general
  [rounding operator](./modifier-operators.md), not a separate formula).
- **Box Frame** — the hollow wireframe/edges of a box (exact).
- **Torus** — donut shape, exact.
- **Capped Torus** — a torus segment cut short of a full revolution, with flat end caps.
- **Link** — a chain-link/stadium-torus shape (two half-circles joined by straight
  segments, revolved).
- **Infinite Cylinder** / **Capped Cylinder** / **Rounded Cylinder** — cylinder variants
  with unbounded, flat-capped, and beveled-edge termination respectively.
- **Cone** / **Infinite Cone** / **Capped Cone** — pointed shapes with finite, unbounded,
  and truncated-tip variants.
- **Plane** — an infinite flat half-space, `dot(p, n) + h`; foundational for ground
  planes and as a clipping primitive in boolean combinations.
- **Hexagonal Prism** / **Triangular Prism** — extruded polygon cross-sections (the
  triangular prism variant is listed as a *bound*, not exact — see below).
- **Capsule / Line** — a sphere swept along a line segment; extremely common as a
  building block for organic/character shapes (limbs, tentacles) since it's cheap and
  exact.
- **Solid Angle** — a conical wedge/sector shape, exact.
- **Cut Sphere** / **Cut Hollow Sphere** — a sphere with a planar slice removed, solid or
  shell variants.
- **Death Star** — the boolean difference of two spheres, given its own dedicated exact
  formula (faster than generic CSG subtraction of two sphere SDFs at grazing angles).
- **Round Cone** — a cone-like shape with continuously varying radius and rounded caps
  (distinct from capping a cone with rounding as a post-process).
- **Vesica Segment** — a lens/eye shape (intersection of two spheres/circles).
- **Rhombus** — diamond-cross-section prism.
- **Octahedron** — eight-faced solid, exact formula exists (a separate *bound* variant
  also exists for cheaper evaluation, see below).
- **Pyramid** — four-sided pointed shape, exact.
- **Triangle** / **Quad** — distance to a single 2D triangle or quad embedded in 3D
  space, used as a low-level building block (this is also the same core primitive needed
  for [triangle-mesh distance queries](../mesh-conversion/exact-point-to-mesh-distance.md)).

## Approximate / bound primitives

A handful of shapes are commonly implemented as *bounds* rather than exact distance
functions, because an exact closed form is impractical or nonexistent — see
[exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md) for why this is still safe
to raymarch against:

- **Ellipsoid** — a non-uniformly scaled sphere. There is no simple exact closed-form
  distance to a general ellipsoid; the standard formula is a lower-bound approximation
  that is exact only in the degenerate spherical case.
- **Triangular Prism** — listed by Quilez as a bound rather than exact.
- **Octahedron (bound variant)** — a cheaper-to-evaluate approximate version exists
  alongside the exact one, useful when the extra raymarching steps a bound induces cost
  less than the exact formula's arithmetic (see the performance tradeoff note in
  [exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md)).

## Practical guidance

- Prefer exact primitives when composing complex scenes with many nested boolean/smooth-
  blend operations — errors from bound primitives can compound when blended, whereas
  exact primitives keep blend formulas behaving predictably.
- Use capsules and rounded boxes as the default building blocks for organic/soft shapes
  — they're both exact and directly support the
  [rounding modifier](./modifier-operators.md) for controlling silhouette softness
  without extra CSG operations.
- When a primitive doesn't exist in closed form for your target shape, consider whether a
  bound is acceptable (usually yes, with appropriate step-damping) before reaching for a
  numerically-sampled or neural representation.

## When to dive in

- Building a scene from scratch → start here, then read
  [combination-operators](./combination-operators.md) to compose primitives into
  complex shapes.
- Need a shape not on this list → check
  [modifier-operators](./modifier-operators.md) and
  [domain-operations](./domain-operations.md) first (elongation, rounding, repetition,
  and symmetry can synthesize many shapes from the base set above) before writing a new
  primitive formula from scratch.

## Relevance to migera

`src/sdf/primitives.rs` implements its formulas from this page. As of 2026-09-28 it
has Sphere, Box3/RoundedBox3, Plane, Cylinder/RoundedCylinder, Capsule, RoundedCone,
Ellipsoid, BoxFrame and HexPrism (GPU kinds in `GpuShapeKind`); Torus was removed in
commit 3d5d570. `RoundedCone::distance` has a known bug (reports every point as
exterior) — see the RoundedCone note below.

## Related
- [Exact vs. bound distance fields](../fundamentals/exact-vs-bound-sdfs.md) — prerequisite: why bound primitives are still safe to march.
- [Single-shape modifiers](./modifier-operators.md) — deeper: rounding/elongation that derive more shapes from this set.
- [Combining SDFs](./combination-operators.md) — deeper: composing primitives into scenes.
- [RoundedCone SDF reports everything exterior](../../hybrid-architecture/roundedcone-sdf-reports-everything-exterior.md) — same-trap: a migera implementation of a primitive from this list that is currently broken.
- [Analytic ray–primitive intersections](../../analytic-intersections/INDEX.md) — contrast: closed-form ray hits instead of distance fields for the same shapes.
