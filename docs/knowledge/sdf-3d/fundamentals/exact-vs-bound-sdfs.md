---
title: Exact vs. bound (conservative) distance fields
description: Establishes that sphere tracing only needs a field that never overestimates distance (Lipschitz ≤ 1), why scaling, smooth blends, repetition and warps yield bounds rather than exact SDFs, and the step-damping fix. Read before combining/warping primitives or when a raymarch punches through surfaces.
type: concept
status: current
tags:
  - sdf
  - raymarching
  - correctness
  - numerics
updated: 2026-08-15
aliases:
  - Lipschitz bound
  - conservative SDF
  - step damping
---

# Exact vs. bound (conservative) distance fields

## The distinction

An **exact** SDF returns the true Euclidean shortest distance to the surface everywhere —
it satisfies the Eikonal equation `|∇f| = 1` (see
[what-is-an-sdf](./what-is-an-sdf.md)) exactly, for every point in space.

A **bound** (also called "conservative" or "Lipschitz bound") distance field returns a
value that is *guaranteed not to overestimate* the true distance, but may
*underestimate* it. Formally, a bound SDF `g` satisfies `|g(p)| <= |f(p)|` for the true
distance `f`, and more precisely is often constructed to be a Lipschitz-continuous
function with Lipschitz constant `<= 1` — meaning `g` never changes faster than
real distance would, so it never claims a surface is farther away than it actually is.

## Why the distinction matters in practice

This matters critically for [sphere tracing](../rendering/sphere-tracing.md): the
algorithm's safety guarantee — that stepping by the SDF value never passes through a
surface — depends *only* on the field never overestimating distance. An exact SDF trivially
satisfies this. A bound SDF also satisfies it (by construction) *and remains safe to
raymarch*, even though its values are not literally "distance" everywhere.

This means many useful and common SDF operations deliberately produce bound rather than
exact fields:

- **Non-uniform scaling** (stretching a primitive along one axis, as with an ellipsoid)
  distorts distance in a way that cannot generally be computed exactly with a simple
  closed form — the standard ellipsoid SDF is a widely-used *bound*, not exact.
- **Domain repetition/mirroring** using `abs()` or `mod()` on coordinates can produce
  bounds rather than exact distances near the seams between repeated cells.
- **Smooth minimum blending** (see
  [combination-operators](../primitives-and-operators/combination-operators.md)) does not
  preserve exactness — the blended field is a smooth approximation, not the literal
  distance to the blended surface, though it remains safe for raymarching if implemented
  correctly.
- **Twist, bend, and other domain-warping deformations** generally break exactness,
  sometimes even breaking the safety bound if the warp is not distance-preserving (see
  [domain-operations](../primitives-and-operators/domain-operations.md) for which
  operations are safe).

## Practical consequence: step damping

Because many practically useful SDF constructions are bounds rather than exact fields,
production raymarchers commonly multiply the SDF value by a damping/safety factor less
than 1 (e.g. `0.8`–`0.95`) before stepping, trading a small amount of extra iteration cost
for robustness against small violations of the Lipschitz-1 guarantee that can otherwise
cause visible surface "punch-through" artifacts, especially at grazing angles or near
highly warped domains.

## Bound fields are still useful even where exact ones exist

Even for shapes with a known *exact* closed form, a cheaper-to-evaluate bound is
sometimes preferred for performance — fewer arithmetic operations per query at the cost of
slightly more raymarching steps overall can be a net win, particularly in scenes with many
overlapping primitives evaluated per pixel.

## When to dive in

- Writing or combining custom SDF primitives → check whether your construction remains a
  valid bound; if unsure, add a conservative safety-scale factor before raymarching
  against it.
- Seeing raymarching artifacts (surface holes, banding, "swiss cheese" punch-through) →
  suspect a non-exact/non-conservative combination first, especially after adding
  scaling, twisting, or aggressive smooth blending; see
  [raymarching-artifacts-and-fixes](../rendering/raymarching-artifacts-and-fixes.md).
- Deciding between a cheap bound and an exact-but-expensive formula for a primitive →
  profile; the extra raymarching steps a bound induces are often cheaper than a costly
  exact evaluation, but this is scene-dependent.

## Related
- [What is a signed distance field?](./what-is-an-sdf.md) — prerequisite: the Eikonal equation this note relaxes.
- [Sphere tracing](../rendering/sphere-tracing.md) — applies: the algorithm whose safety depends on the no-overestimate guarantee.
- [Raymarching artifacts and fixes](../rendering/raymarching-artifacts-and-fixes.md) — applies: symptom → cause map when a bound is violated.
- [Combination operators](../primitives-and-operators/combination-operators.md) — example: smooth-min blends that only produce bounds.
- [Glitch-free baked SDFs](../mesh-conversion/glitch-free-baked-sdfs.md) — example: why a trilinear-interpolated baked grid is only ever a bound.
