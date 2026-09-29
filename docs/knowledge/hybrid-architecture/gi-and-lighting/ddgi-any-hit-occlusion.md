---
title: DDGI any-hit occlusion fixed a leak but gave no measured speed-up
description: DDGI's per-probe occlusion ray is ~48–52% of gpu trace at --stress 10000; replacing nearest-hit trace() with an early-out any_hit fixed a real self-graze leak but measured zero speed-up, likely from 64-lane wavefront divergence (unprofiled). Read before optimizing DDGI occlusion or adding early-out queries.
type: decision
status: current
tags:
  - global-illumination
  - performance
  - gpu-compute
  - correctness
  - hybrid-renderer
updated: 2026-09-19
verified: 2026-09-28
code:
  - src/hybrid/cpu_ref.rs
  - src/hybrid/ddgi_ref.rs
  - assets/shaders/hybrid_trace.wgsl
  - assets/shaders/hybrid_ddgi_relight.wgsl
sources:
  - Claude memory ddgi_any_hit_occlusion_correctness_fix_perf_null (2026-09-19)
  - commit 503ec26
  - PROGRESS.md "Performance investigation: skybox/far-object baking ... + DDGI any-hit occlusion" entry
aliases:
  - any_hit
  - early-out occlusion query
  - wavefront divergence
  - ddgi_sample_probe_grid
---

# DDGI any-hit occlusion fixed a leak but gave no measured speed-up

DDGI's per-pixel hard-occlusion rays are the single largest cost in the trace
pass. Replacing the nearest-hit `trace()` used for them with an early-out
`any_hit` **fixed a real correctness gap and delivered zero measured speed-up**.
`any_hit` was kept for the fix; the performance angle is closed as a measured
null result.

## Context

- A single-toggle-off cost breakdown at `--stress 10000` found DDGI occlusion
  at ~65–72 ms of a ~139 ms frame (~48–52%), larger than shadows (~28 ms) or
  reflections (~28–31 ms).
- In `ddgi_sample_probe_grid` (`hybrid_trace.wgsl` / `ddgi_ref.rs`), each of the
  8 trilinear-neighbour probes fired a full BVH `trace()` (nearest hit, with
  normal) just to answer "is anything in the way".

## Decision

Implement `any_hit` (CPU reference `cpu_ref.rs::any_hit`; WGSL in
`hybrid_trace.wgsl` and its duplicate in `hybrid_ddgi_relight.wgsl`): the same
BVH descent, but return `true` on the first converged march against a
non-excluded object.

**Correctness bug fixed along the way.** The old check was
`trace(...).is_some_and(|hit| hit.entity != origin_entity)`, which inspects only
the nearest hit. If the shaded object's own self-graze was nearest, it stopped
there and never saw a real occluder farther along the ray: under-occlusion, a
light leak. `any_hit` skips the excluded entity's leaf during traversal and
keeps searching.

## Alternatives considered

- **Skip the occlusion call entirely** (diagnostic only). `gpu trace` fell from
  ~139 ms to ~70 ms, which shows the cost is real, but it removes correctness.
- **Keep nearest-hit `trace()`.** Lost on correctness.

## Consequences

- **Measured:** `any_hit` left `gpu trace` at ~138–146 ms across several
  uncontended runs at `--stress 10000`. At `--stress 100` there was never a gap
  (~66–72 ms both ways; a shallow BVH makes nearest-hit and any-hit cost about
  the same).
- **Likely explanation, not confirmed by profiling:** this AMD GPU reports
  `subgroup_min_size = subgroup_max_size = 64`. An early return helps only when
  all 64 lanes exit together. One lane with a clear line of sight must traverse
  the whole tree, and the wavefront waits for it. That is different from
  skipping the call, which removes the shared descent work altogether.
- Same pattern as the distance-scaled march epsilon: a real correctness gain,
  zero performance gain, kept anyway.

## Revisit when

- Occlusion cost is attacked for performance again. First measure wavefront
  divergence directly (`renderdoc` is in `flake.nix`).
- The remaining unmeasured, low-risk lever: skip only the reflection bounce's
  own re-run shadow loop (`shade_for_reflection_bounce`, about half the total
  shadow cost) with a constant or single-light approximation.

## Evidence

- Regression test `excluding_the_origin_entity_does_not_hide_a_real_occluder_farther_along_the_same_ray`
  (`src/hybrid/ddgi_ref.rs`), confirmed to fail against the old logic and pass
  after. Six CPU parity tests in `cpu_ref.rs` check that `any_hit` agrees with
  `trace()` on hit/miss. All four sealed-room regression tests still pass;
  `gi_room --at-frame 200` (roof sealed) renders black.

## Related
- [Trace-pass bottleneck is not march steps](../performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — prerequisite: the earlier finding that ruled out march iteration count.
- [Skybox / far-object bake not worth it](../performance-findings/skybox-far-object-bake-not-worth-it.md) — contrast: the same investigation's other lead, rejected because misses are already cheap.
- [Render scale subsumes half-res reflections](../performance-findings/render-scale-subsumes-half-res-reflections.md) — deeper: the earlier perf pass this investigation continued.
- [Same function on both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — example: the regression test was proven able to fail before it was trusted.
- [Fundamentals and execution model](../../compute-shaders/fundamentals-and-execution-model.md) — deeper: wavefronts, lockstep execution and divergence.
