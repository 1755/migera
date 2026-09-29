---
title: CSG on exact intersections
description: Explains exact CSG on ray spans (Roth 1982) - union/intersection/difference as span-list operations, flipped normals on cut faces, a low-memory single-hit state table, interval-arithmetic bisection - and why smooth blends (smin) fall outside exact CSG. Read before combining exact intersectors with booleans.
type: concept
status: current
tags:
  - csg
  - ray-tracing
  - math
  - sdf
updated: 2026-09-28
sources:
  - Roth, Ray Casting for Modeling Solids (1982)
  - Ray Tracing CSG Objects Using Single Hit Intersections (xrt project)
  - Knoll et al., interval-arithmetic CSG (Dagstuhl 2010)
aliases:
  - span CSG
  - Roth classification
  - single-hit CSG
  - interval arithmetic
---

# CSG on exact intersections

Sources: Roth "Ray Casting for Modeling Solids" (1982); Cambridge AdvGraph CSG notes
(sorted-t state machine); "Ray Tracing CSG Objects Using Single Hit Intersections"
(xrt project doc - state tables); Rochester CSG ray-tracing project (span merge rules);
Knoll et al. interval-arithmetic CSG (Dagstuhl 2010).

## The core idea

A closed solid A intersects a ray in a set of disjoint t-intervals ("spans"):
`[t_in_1,t_out_1], [t_in_2,t_out_2], ...` with in/out parity alternating. Every CSG
operator is then an operation on these span lists, applied bottom-up on the tree:

- **Union**: merge overlapping spans; keep min-enter/max-exit.
- **Intersection**: intersect spans pairwise; result spans are [max(enters),min(exits)]
  where nonempty. Empty results vanish.
- **Difference A-B**: subtract B's spans from A's (clip; B fully inside A splits A's
  span in TWO).
- Parity/classification alternative: merge all hits into one sorted t-list carrying
  (inA,inB) flags; the Roth table classifies each transition into/out of the combined
  state (in-union/in-intersection/in-A-only/...). Equivalent to span ops.

## Normals & materials at cut faces

At a boundary created by subtraction, the visible surface belongs to **B** with
FLIPPED normal (`n_B = -grad F_B`). Span-merging implementations must "paste" B's hit
(normal+material) onto the new span boundary and negate the normal - forgetting this is
the classic inverted-shading bug.

## Single-hit variant (low memory, GPU-friendly)

Instead of materializing full span lists, walk the ray keeping at most two candidate
hits and classify Enter/Exit per sub-object by sign(dot(rd,n)); a small state table
(union/difference/intersection x Enter/Exit/Miss for A,B) decides: return A/B,
flip normal, or advance one subobject past its current t and re-intersect.
Requires only closed, consistently-oriented, non-self-intersecting subobjects.
Cost: a few re-intersections per node instead of list storage - usually the right
tradeoff on GPUs where lists need atomics/buffers.

## Interval-arithmetic bisection (general implicits)

For primitives without closed roots (or arbitrary implicit trees): evaluate the tree's
interval extension over a t-span of the ray; if 0 not in the resulting interval the
whole span is provably surface-free; else bisect. Robust to any composition, but costs
iterations - use only where closed forms do not exist. Precision near CSG joints needs
~1e-5 epsilons (wider interval bounds make naive 1e-3 insufficient there).

## The smooth-blend limitation (critical for migera)

All of the above composes HARD boolean operations exactly. Smooth operators (smin/smax
blends) produce surfaces that are NOT the zero set of any composition of primitive
polynomials - they are new, non-algebraic surfaces. Consequences:
1. Exact intersectors cannot render blended unions; marching must take over near blends.
2. Hybrid architecture: leaves/subtrees flagged `exact` vs `blended`; rays run exact
   span CSG through exact regions and switch to sphere tracing only within blended
   subtrees' bounds (blend radius k gives the region size).
3. Subtract-with-smooth-radius similarly breaks exactness in its annulus.

In migera's legacy hybrid renderer, span composition cost scaled with leaf/op count: a
10-sphere smooth-union "beaded ring" made analytic mode slower than SDF mode (mean frame
time SDF 14.06 ms vs analytic 16.15 ms, commit ba3844e), one of the findings that led to the tier's
removal.

## Related
- [Combining SDFs: boolean CSG and smooth blending](../sdf-3d/primitives-and-operators/combination-operators.md) — contrast: CSG on distance fields, where smooth blends are cheap.
- [Exact ray-primitive intersectors - catalog with costs](./primitive-intersection-catalog.md) — prerequisite: where the per-primitive spans come from.
- [Quadric surfaces & algebraic surfaces](./quadrics-algebraic-surfaces.md) — prerequisite: capped solids are already quadric ∩ slab intervals.
- [The analytic tier cost 5-6x SDF marching and was removed](./production-performance-finding.md) — example: what span composition cost in practice on the GPU.
- [Robustness, best/worst practices & limitations](./robustness-and-limits.md) — deeper: why single-hit tables beat per-ray span lists on the GPU.
