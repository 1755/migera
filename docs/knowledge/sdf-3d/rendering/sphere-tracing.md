---
title: Sphere tracing (raymarching against an SDF)
description: Explains the sphere-tracing loop (step by the SDF value), why it is safe for any non-overestimating field, why a fixed MAX_STEPS cap causes grazing-angle misses, epsilon scaling, overrelaxation (Enhanced Sphere Tracing) and the link to cone tracing. Read before writing or tuning a raymarch loop.
type: concept
status: current
tags:
  - sdf
  - raymarching
  - performance
  - correctness
updated: 2026-08-15
aliases:
  - raymarching
  - enhanced sphere tracing
  - overrelaxation
  - march epsilon
---

# Sphere tracing (raymarching against an SDF)

## The core algorithm

Sphere tracing is the standard method for finding where a ray intersects a surface
defined implicitly by an SDF. It's called "sphere tracing" because at each step, the SDF
value defines a sphere around the current ray point guaranteed to contain no surface —
the ray can safely advance by that sphere's radius without risk of stepping through
geometry:

```glsl
float raymarch(vec3 ro, vec3 rd, float maxDist) {
    float t = 0.0;
    for (int i = 0; i < MAX_STEPS; i++) {
        vec3 p = ro + rd * t;
        float d = map(p);          // evaluate the scene SDF
        if (d < EPSILON) return t;  // close enough — hit
        t += d;                     // step exactly as far as we know is safe
        if (t > maxDist) break;     // missed everything
    }
    return -1.0;  // no hit
}
```

This differs fundamentally from traditional ray *marching* with a fixed step size: the
step length here is adaptive, taken from the SDF's own guarantee, so rays traveling
through open empty space (far from any surface) take large steps automatically, while
rays approaching a surface automatically slow down and refine — no manual tuning of a
step size against scene scale is needed for correctness, only for *iteration budget*.

## Why this is safe

The safety guarantee depends entirely on the SDF property discussed in
[exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md): as long as the field never
*overestimates* distance (whether it's an exact field or merely a conservative Lipschitz-1
bound), a step of that length cannot pass through any surface, because by definition no
surface exists within that radius of the current point.

## Convergence is not guaranteed in a fixed number of steps

A well-known limitation: sphere tracing is only guaranteed to converge given an
*unbounded* number of steps. In practice every implementation caps iterations
(`MAX_STEPS`), which means rays that graze a surface at a shallow angle, or that pass
very close to — but not through — geometry, can require many small steps to resolve and
may hit the iteration cap before converging, producing either a false miss or a visibly
"blocky"/incorrect result at silhouette edges. This is the primary practical failure mode
practitioners tune around (see
[raymarching-artifacts-and-fixes](./raymarching-artifacts-and-fixes.md)).

## Termination epsilon and its tradeoffs

The `EPSILON` hit threshold trades accuracy against iteration count: too large and the
surface position error becomes visible (geometry appears to "float" slightly above/below
its true position, most noticeable in normal-dependent shading); too small and rays need
more steps to converge, especially at grazing angles where the SDF value shrinks slowly
per step. Production raymarchers commonly scale epsilon relative to distance traveled (a
larger threshold for far-away hits, since a fixed absolute epsilon becomes a
proportionally tighter constraint the farther the camera is from a surface — this
interacts with floating-point precision loss at scale, an important consideration for
[large-scene handling](../performance-and-production/performance-characteristics.md)).

## Acceleration: overrelaxation

The "Enhanced Sphere Tracing" technique (Keinert et al.) accelerates convergence via
**overrelaxation**: instead of stepping exactly by the SDF value, take a slightly larger
step (multiplied by a relaxation factor `> 1`), which converges faster for locally
near-flat surfaces — the common case — while requiring a fallback correction (reducing the
relaxation factor or backtracking) when the larger step turns out to have been unsafe.
This can meaningfully cut the average iteration count for typical scenes, at the cost of
extra bookkeeping per step.

A related technique addresses acceleration specifically for **convex primitives enclosed
in convex bounding volumes**, exploiting the bound's simpler geometry to skip ahead faster
than the general per-step SDF evaluation would allow.

## Relationship to cone tracing

Sphere tracing generalizes naturally to **cone tracing** against an implicit surface: a
cone (rather than a single ray) can use the SDF-derived safe-step spheres as bounds on
where the cone's growing radius might first touch geometry, useful for cheap approximate
soft shadows, depth-of-field, and antialiasing without shooting many individual rays per
pixel (see [soft-shadows-and-ao](./soft-shadows-and-ao.md) for the most common
production use of this idea).

## When to dive in

- Implementing a raymarcher from scratch → the loop above is the whole algorithm; the
  real engineering effort goes into artifact mitigation and acceleration (see the two
  linked documents below).
- Rays that should hit thin/grazing geometry are missing or showing holes → read
  [raymarching-artifacts-and-fixes](./raymarching-artifacts-and-fixes.md) — this is
  almost always a convergence/iteration-cap issue, not a bug in the SDF itself.
- Performance-tuning a raymarcher → overrelaxation and epsilon scaling are the first two
  levers to pull before reaching for spatial acceleration structures; see
  [performance-characteristics](../performance-and-production/performance-characteristics.md).

## Related
- [Exact vs. bound distance fields](../fundamentals/exact-vs-bound-sdfs.md) — prerequisite: the no-overestimate guarantee this algorithm relies on.
- [Normal estimation](./normal-estimation.md) — deeper: shading the hit this loop finds.
- [Raymarching artifacts and fixes](./raymarching-artifacts-and-fixes.md) — deeper: what goes wrong at the iteration cap and epsilon.
- [Hybrid renderer hardening](../../hybrid-architecture/plans/hybrid-renderer-hardening.md) — deeper: Enhanced Sphere Tracing (ω = 1.6 overrelaxation) and other step accelerators surveyed for migera.
- [Raymarching via compute](../../compute-shaders/raymarching-via-compute.md) — applies: running this loop in a WGSL compute shader.
- [Trace-pass bottleneck is not march steps](../../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — contrast: in migera's `src/hybrid`, distance-scaled epsilon and a 4x step-cap cut measured zero win.
