---
title: "Ray–AABB intersection: the slab method"
description: Establishes the slab method as the ray/AABB test - branchless with per-ray reciprocals, NaN-safe clamped updates, boundary-inclusive variants - and why it should return [t_near, t_far] intervals; includes reference WGSL. Read before writing or changing any ray/box test in WGSL or Rust.
type: concept
status: current
tags:
  - bounding-volumes
  - ray-tracing
  - numerics
  - wgsl
  - spatial-acceleration
updated: 2026-09-28
verified: 2026-09-28
code:
  - assets/shaders/hybrid_trace.wgsl
  - src/hybrid/cpu_ref.rs
  - src/hybrid/conetrace_ref.rs
sources:
  - Kay & Kajiya, Ray Tracing Complex Scenes (SIGGRAPH 1986)
  - Williams et al., JGT 2005
  - Majercik et al., JCGT 2018
  - Shirley/Wald/Marrs, Ray Axis-Aligned Bounding Box Intersection (RT Gems II, 2021)
  - https://tavianator.com/2022/ray_box_boundary.html
aliases:
  - ray-box intersection
  - slab test
  - Kay-Kajiya
  - slab_hit
---

# Ray–AABB intersection: the slab method

Sources: Kay & Kajiya "Ray Tracing Complex Scenes" (SIGGRAPH 1986) - the original;
Williams et al. JGT 2005 (robust IEEE variant); Majercik et al. JCGT 2018 (Ray-Tracing
Gems); Shirley/Wald/Marrs "Ray Axis-Aligned Bounding Box Intersection" (RT Gems II
2021, the reference formulation); Tavianator's three-part series
(tavianator.com/2011/ray_box.html, /2015/ray_box_nan.html, /2022/ray_box_boundary.html);
Wikipedia "Slab method"; Bevy 0.19.1's own implementation in
`bevy_math::bounding::RayCast3d::aabb_intersection_at` (verified from source).

## The algorithm

Treat the box as the intersection of 3 slabs (regions between parallel planes,
x=xmin..xmax etc.). Parametrize the ray p(t)=o+t*d and clip by each slab pair:

```
t1 = (box.min[d] - o[d]) * d_inv[d]      // d_inv precomputed ONCE per ray
t2 = (box.max[d] - o[d]) * d_inv[d]
t_enter = max over d of min(t1,t2)
t_exit  = min over d of max(t1,t2)
hit     <=> t_enter <= t_exit   (and t_exit >= ray.t_min, t_enter <= ray.t_max)
```

Why it's the dominant method: no square roots, no dot products, ~6 multiplies +
6 min/max ops per test, naturally returns the **t-interval**, and composes with the
caller's own [t_start, t_max] window as a 4th interval.

## Evolution of implementations

1. Naive: per-axis zero-direction branches. Correct but slow.
2. Branchless (Williams et al.): rely on IEEE 754 - `d_inv = inf` when `d=0`;
   opposite-sign infinities inside a slab cancel through min/max; same-sign infinities
   outside fail the test. No divisions at test time (precomputed reciprocals), no data
   branches; compiles to branch-free minss/maxss.
3. NaN-safe (part 2): rays lying exactly *on* a face produce 0*inf=NaN which pollutes
   SSE-style min/max inconsistently. Fix: clamp inner results against running
   tmin/tmax so one argument is always non-NaN:
   `tmin = max(tmin, min(min(t1,t2), tmax)); tmax = min(tmax, max(max(t1,t2), tmin));`
   (~30% cheaper than IEEE minNum/maxNum emulation).
4. Boundary-inclusive + fastest (part 3, supersedes all):
   `float tmin = 0.0, tmax = INFINITY;` seeded with the caller's window, then the
   clamped update above; final test `tmin < tmax` (or `<=` to include grazing edges -
   use `>=` semantics for zero-thickness boxes like ground planes!).
5. Sign trick: replace per-axis min/max pairs by selecting near/far corners via the
   sign bit of d_inv (`corners[signs[d]]`) - fewer ops, same result.
6. Batch/vectorized: test N boxes per iteration with SIMD (SoA layout); measured 3.3x.
   On GPU this is free - every lane does one box.

## Return intervals, not booleans

For traversal you almost always want `[t_near, t_far]`: ordered BVH child descent needs
near-first; interval-skipping marches need both endpoints; shadow rays need "any hit
before distance X" = `t_near < X`. A boolean forces recomputation later.

## Pitfalls / worst practices

- Recomputing `1/d` per test instead of per ray (the single most common mistake).
- Using `<` vs `<=` inconsistently: exclusive boundaries silently drop coplanar rays;
  inclusive boundaries double-count shared faces between adjacent boxes (grid cells).
  Pick deliberately per structure (cells: half-open convention).
- Forgetting the `>= 0` / t-window clamp for shadow or offset secondary rays whose
  origin sits on/inside geometry.
- Trusting NaN behavior of your language's min/max without testing the exact-on-face
  case (WGSL min/max: documented as returning the non-NaN operand when one is NaN -
  matches Bevy's reliance on it).
- Storing boxes as two vec3s in WGSL structs without checking alignment/padding - use
  six scalar f32s (this repo's established convention) so Rust repr(C) == WGSL layout.
- Fitting world-space AABBs per frame for rotated objects via min/max of transformed
  points (conservative but grows with rotation); prefer local-space boxes + transformed
  ray when rotation is static per primitive.

## Reference WGSL (boundary-inclusive, interval-returning)

```wgsl
// r_dir_inv precomputed per ray; returns true and writes [t0,t1] on hit.
fn ray_aabb(o: vec3<f32>, dir_inv: vec3<f32>, b_min: vec3<f32>, b_max: vec3<f32>,
            t_max_win: f32, out_ti: ptr<function, vec2<f32>>) -> bool {
    let t1 = (b_min - o) * dir_inv;
    let t2 = (b_max - o) * dir_inv;
    let tsmn = min(t1, t2);          // component-wise (NaN -> other operand, per spec)
    let tsmx = max(t1, t2);
    var t_enter = max(tsmn.x, max(tsmn.y, tsmn.z));
    let t_exit  = min(tsmx.x, min(tsmx.y, tsmx.z));
    t_enter = max(t_enter, 0.0);
    if (t_enter > t_exit || t_enter > t_max_win) { return false; }
    (*out_ti) = vec2<f32>(t_enter, t_exit);
    return true;
}
```

## Relevance to migera

`src/hybrid` implements this as `slab_hit` in `assets/shaders/hybrid_trace.wgsl`,
mirrored on the CPU in `src/hybrid/cpu_ref.rs` (and `conetrace_ref.rs`, whose
`cone_slab_hit` pads the box by the cone radius). It returns an interval, as advised
above, and is used for every BVH node test.

## Related
- [Acceleration structures around AABBs](./acceleration-structures.md) — applies: the grids and BVHs whose traversal is built from this test.
- [BVH deep dive](../hierarchical-volumes/bvh-deep-dive.md) — applies: ordered descent needs the `t_near` this test returns.
- [Bevy 0.19 AABB primitives](./bevy-aabb-primitives.md) — example: Bevy's own branchless `RayCast3d::aabb_intersection_at`.
- [Exact ray-primitive intersectors](../analytic-intersections/primitive-intersection-catalog.md) — deeper: the box intersector with face normals, and why every exact intersector gets a slab pre-test.
- [Verify camera rays from the shader's own matrices](./verify-camera-rays-from-the-shaders-own-matrices.md) — same-trap: how a per-object interval bug was nearly misdiagnosed.
- [Common raymarching artifacts and their causes](../sdf-3d/rendering/raymarching-artifacts-and-fixes.md#flat-cut-or-chord-bitten-out-of-a-round-silhouette-near-where-two-objects-touch) — example: marching a per-object interval through a merged interval list skips real geometry.
