---
title: The analytic tier cost 5-6x SDF marching and was removed
description: RenderDoc on the legacy hybrid renderer measured exact intersection at ~5.5-6x the GPU dispatch time of SDF marching even for a best-case 2-object scene, because every shadow/reflection ray repeats a full solve; the tier was removed (commit dcf8a70). Read before reintroducing exact intersection anywhere per-frame.
type: decision
status: current
tags:
  - performance
  - ray-tracing
  - raymarching
  - hybrid-renderer
  - primitives
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/prim/mod.rs
sources:
  - commit dcf8a70 (Remove analytic ray-intersection tier; SDF+splat only)
  - commit ba3844e (Remove rings from all scenes)
  - RenderDoc capture on AMD RADV RENOIR, Vulkan/Mesa 26.1.5
aliases:
  - analytic tier
  - eval_spans
  - exact intersection performance
  - analytic vs SDF
---

# The analytic tier cost 5-6x SDF marching and was removed

Exact analytic ray intersection cost ~5.5-6x the GPU dispatch time of SDF sphere
tracing, even in its best case, because every shadow and reflection ray repeats a full
solve that cannot stop early. The analytic tier was removed from the renderer on
2026-08-30 (commit dcf8a70). The research in this tree stays valid; migera does not use
it for per-frame shading.

## Context

The legacy hybrid renderer (`src/hybrid_legacy` era, `hybrid_trace.wgsl`) had three
tiers: exact analytic intersection for hard/unblended shapes, SDF marching for blended
ones, and Gaussian splatting for the far band. The analytic tier promised zero
iteration, zero epsilon tuning and exact normals.

**Measurement.** RenderDoc on a minimal 2-object scene (one sphere, one flat ground
plate, both single-leaf, zero smooth-CSG ops — the analytic path's best case, since the
single-leaf fast path skips the RPN span stack), AMD RDNA2 (RADV RENOIR, Vulkan/Mesa
26.1.5):

| Mode | `vkCmdDispatch(160, 90, 1)` GPU time |
|------|---------------------------------------|
| SDF marching | 24014.00 (RenderDoc duration units) |
| Analytic intersection | 133628.72 - 139317.48 |

Wall-clock `--bench` frame times showed a smaller gap (SDF p50 ~13.6 ms vs analytic p50
~16.7 ms, ~23%) because fixed CPU and present/vsync costs dilute the compute
difference. The dispatch cost, which scales with scene complexity, was the real signal.

**Root cause.** The cost is not one ray's intersection but how many rays per pixel
need a full solve. For one lit, shadowed, reflective analytic object, `trace()` called
`eval_spans` (quadratic solve + span composition) once per ray:

- 1x primary ray;
- +1x per light's shadow ray (`trace_shadow` re-derived the BVH candidate set and
  called `eval_spans` from scratch, sharing nothing with the primary hit);
- +1x reflection ray, which repeats the shadow fan-out at its own hit point.

With 2 lights that is up to 5-7 `eval_spans` calls per pixel, each costlier than one
SDF sample: a `solve_quadratic` plus `RootSet`/`SpanSet` bookkeeping versus a
`length(p) - r`-class evaluation. An analytic ray also cannot stop at "close enough",
while marching takes ~3-15 near-free steps (Keinert over-relaxation) with early exit.
This is the structural trade-off of exact intersection, not a missed optimization.

## Decision

Remove the analytic tier entirely rather than patch it (commit dcf8a70). Its cost
scales with light count and reflection depth in a way marching's does not, and exact
normals were not worth a 5-6x per-dispatch cost in any lit/shadowed scene.

## Alternatives considered

- **Single-leaf fast path in `eval_spans`** (skip the general RPN stack and its
  16-slot `array<SpanSet, 16u>`). Helped ~2% mean frame time; the same quadratic solve
  per ray remained.
- **Fix multi-leaf CSG cost.** A separate problem: a 10-sphere smooth-union "beaded
  ring" (the torus stand-in) made analytic mode slower than SDF mode (SDF 14.06 ms vs
  analytic 16.15 ms mean frame time). Removing rings (commit ba3844e) closed that gap,
  but the one-solve-per-ray cost above persists even for the simplest object.

## Consequences

- The renderer that made this decision became SDF near band + splat far band
  (`HybridTierConfig`). That renderer was itself later replaced: `src/hybrid_legacy`
  was deleted (commit 0f04320), and the from-scratch `src/hybrid` rewrite is SDF
  marching only, with no analytic or splat tier (checked 2026-09-28).
- `src/prim/mod.rs`'s module doc records the removal and points to this tree.
- Torus is gone as a primitive (see
  [Robustness, best/worst practices & limitations](./robustness-and-limits.md#retro-why-migera-has-no-torus-primitive)).

## Revisit when

A use pays the per-ray solve once rather than 5-7 times per pixel — picking, collision,
or editor queries on the CPU. The catalog, interval-CSG rules and robustness fixes in
this tree apply directly there.

## Related
- [Exact ray-primitive intersectors - catalog with costs](./primitive-intersection-catalog.md) — deeper: the intersectors whose per-ray cost is measured here.
- [CSG on exact intersections](./csg-intervals.md) — deeper: the span-composition model behind `eval_spans` and the beaded-ring cost.
- [Robustness, best/worst practices & limitations](./robustness-and-limits.md) — example: the f32 torus that was dropped before the tier itself.
- [Sphere tracing](../sdf-3d/rendering/sphere-tracing.md) — contrast: the technique that won.
- [Estimating surface normals from an SDF](../sdf-3d/rendering/normal-estimation.md) — contrast: how a marcher gets normals without exact intersection, and when analytic gradients are worth it.
- [Trace-pass bottleneck is not march steps](../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — same-trap: secondary-ray fan-out, not per-sample cost, dominates trace time.
- [Skybox/far-object bake is not worth it](../hybrid-architecture/performance-findings/skybox-far-object-bake-not-worth-it.md) — contrast: the "third far band" idea mentioned when this tier went, measured and not built.
