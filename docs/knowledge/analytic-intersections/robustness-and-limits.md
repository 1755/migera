---
title: Robustness, best/worst practices & limitations
description: Gives the three precision fixes for ray quadratics (q-formula, geometric reformulation, Kahan/FMA discriminant), best/worst-practice lists, the retro on why migera's f32 GPU torus intersector was dropped, and the fundamental limits of analytic intersection. Read before writing any root solver in f32/WGSL.
type: research
status: current
tags:
  - numerics
  - ray-tracing
  - primitives
  - correctness
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/prim/mod.rs
sources:
  - Haines et al., Precision Improvements for Ray/Sphere Intersection (2019)
  - Marc Reynolds, FMA quadratic discriminant
  - Boldo/Daumas/Kahan/Melquiond, certified discriminant
  - commit ba3844e
aliases:
  - catastrophic cancellation
  - q-formula
  - Kahan discriminant
  - quartic torus
  - beaded ring
---

# Robustness, best/worst practices & limitations

Sources: Scratchapixel (q-formulation), NVIDIA "Precision Improvements for Ray/Sphere
Intersection" (Haines et al. 2019), Marc Reynolds FMA quadratic post (Kahan discriminant
branchless with fma), Boldo/Daumas/Kahan/Melquiond certified-discriminant work.

## The three precision fixes for quadratics

1. **Sign-stabilized roots (q-formula)**:
   `q = -(b + sign(b)*sqrt(b*b-4ac))/2; t1 = q/a; t2 = c/q;`
   avoids subtracting nearly-equal quantities; `c/q` path is exact-ish by construction.
2. **Geometric reformulation for far/small spheres** (NVIDIA 2019): the schoolbook form
   loses all bits of r once |o-c| > ~4096r because c = f.f - r^2 squares BEFORE
   subtracting. Compute the perpendicular distance l of the center from the ray as
   `l = length(f - dot(f,d)*d)` (subtract BEFORE squaring) so
   discriminant = r^2 - l^2 keeps r's bits. Fixes ground-plane-sized spheres and tiny
   distant objects.
3. **Kahan/FMA discriminant**: compute b^2-4ac in extended precision via two fmams
   (`e=fmaf(c,d,-cd)` trick); branchless; error <= 2 ulp (formally proven). WGSL has
   `fma()` - use it where precision matters (shadow acne at grazing angles).

## Best-practice checklist

- Precompute per-ray: rd_inv, sign(rd); per-primitive: canonical-space constants.
- Early-outs before sqrt: negative discriminant; c>0 && b>0 (sphere behind); slab
  tN>tF; y-range tests for capped solids.
- Return intervals [tN,tF]; callers compose CSG/shadows without re-solving.
- Always pair expensive intersectors with cheap bounds (AABB/sphere/coaxial cylinders).
- Distance-bound clipping threaded through every intersector.
- Shadow rays: any-hit mode - skip normal computation and root selection.
- Transform rays to canonical space; transform normals back by transpose-inverse
  (rotation-only isometries make this just the rotation).
- Test suite must include: origin-on-surface, tangent rays, axis-parallel rays,
  inside-origin queries, huge-distance spheres.

## Worst practices

- Schoolbook quadratic in production (both cancellation modes above).
- Normalizing rd when the formula tolerates unnormalized (wasted rsqrt; also changes
  t semantics - pick one convention and keep it).
- Computing BOTH roots then discarding one (select via branch/min instead).
- Generalizing primitive formulas to arbitrary placement instead of transforming rays.
- Storing full span lists on GPU per ray (atomics+memory) when single-hit tables suffice.
- Using exact intersectors against smooth-blended SDFs and wondering why edges look
  wrong (see csg-intervals limitation).
- f64 "to be safe" on GPU: doubles are 1/16-rate or emulated; use the reformulations.
- f32 quartic resolvents near an anti-tangent ray: the D/E radicands cancel three
  O(coeff²) terms; expand them in ray geometry rather than evaluating the raw
  difference (see the torus retro below for why this alone wasn't enough).

## Retro: why migera has no torus primitive

An f32 quartic torus intersector was prototyped for the GPU analytic tier
(resolvent-cubic split, `n4` expansion for the D/E cancellation above, a
near-even biquadratic fast path, oracle-gated sign-scan/bisection fallback,
Newton polish) and it DID work — but the fallback path fired on the majority
of adversarial conformance rays (grazing/near-tangent cases the closed form
can't resolve in f32), each fallback costing ~1024-2048 dense polynomial
samples per ray. That's not a GPU-viable "fast" tier; it was slower than
plain SDF marching for the one primitive it was supposed to speed up. Torus
support was removed wholesale rather than shipped as a slow special case. Rings were
first modeled as a circle of 10 smooth-unioned spheres ("beaded ring"); that made
analytic mode slower than SDF mode, so rings were removed from all scenes (commit
ba3844e, 2026-08-30), and the whole analytic tier followed (see
[the production finding](./production-performance-finding.md)). `src/prim/mod.rs`'s
module doc now only records the tier's removal.

## Limitations of the whole analytic approach

1. Degree ceiling: no closed forms past quartics (Abel-Ruffini); organic shapes need
   marching regardless.
2. Smooth blending incompatible (the migera-critical one).
3. Numerical epsilon tuning moves from "marching convergence" to "root robustness" -
   still present, just different.
4. Scene complexity shifts into acceleration structure management: exact hit-tests
   without a hierarchy degenerate to O(N) per ray exactly like naive marching.
5. Non-closed primitives break solid-CSG; self-intersections break single-hit tables.
6. Curved-normal interpolation is free (exact gradients!) but texturing needs explicit
   UV parameterization per primitive type - not uniform like triplanar-on-SDF.

## Related
- [Quadric surfaces & algebraic surfaces](./quadrics-algebraic-surfaces.md) — prerequisite: the polynomials whose roots these fixes stabilize.
- [Exact ray-primitive intersectors - catalog with costs](./primitive-intersection-catalog.md) — applies: the intersectors these practices govern.
- [The analytic tier cost 5-6x SDF marching and was removed](./production-performance-finding.md) — example: the production outcome that followed the torus retro.
- [CSG on exact intersections](./csg-intervals.md) — deeper: the smooth-blend limitation (limit 2) in detail.
- [Quat::angle_between precision floor](../engineering-practice/measurement/quat-angle-between-precision-floor.md) — same-trap: another f32 cancellation near a degenerate configuration.
