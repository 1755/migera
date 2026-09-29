---
title: The trace pass's cost is not sphere-march iteration count
description: On the dev machine's integrated AMD Renoir GPU, distance-scaled march epsilon and even cutting MAX_MARCH_STEPS 128 to 32 left gpu trace unchanged (~137–143 ms at --stress 10000); march steps are not the bottleneck. Read before any optimization that targets march step count.
type: lesson
status: current
tags:
  - performance
  - raymarching
  - hybrid-renderer
  - gpu-compute
updated: 2026-09-19
verified: 2026-09-28
code:
  - src/hybrid/cpu_ref.rs
  - assets/shaders/hybrid_trace.wgsl
sources:
  - Claude memory hybrid_trace_pass_bottleneck_not_march_steps (2026-09-19)
  - commit a3be2c4
  - PROGRESS.md "Performance: distance-scaled march-convergence epsilon" entry
aliases:
  - MAX_MARCH_STEPS
  - distance-scaled epsilon
  - pixel_eps
  - march step count
---

# The trace pass's cost is not sphere-march iteration count

On this project's dev GPU (integrated AMD Radeon Renoir), **march-loop
iteration count does not drive the trace pass's cost**. Two changes that cut
march work, including a 4x cut to the worst-case step count, produced no
measurable change in `gpu trace`.

## What happened

- On 2026-09-19, item 1 of a three-item perf pass: distance-scaled convergence
  epsilon in `march_object`, reusing the existing `pixel_eps(t)` growth curve
  (already used for `shadow_bias`), applied to all five CPU/WGSL copies of
  `march_object`.
- Result: no measurable change on two very different scenes, `--stress 10000`
  (BVH-heavy grid) and `--stress 1` with a far orbit camera (a single long
  march).
- Sanity check: temporarily cutting `MAX_MARCH_STEPS` from 128 to 32 on the
  `--stress 10000` scene left `gpu trace` at ~137–143 ms either way.
- The epsilon change was kept anyway (the user's explicit call): a harmless,
  tested precision relaxation that may pay off on other hardware.

## Why it matters

March iteration is the textbook bottleneck for SDF ray marching, so it is the
first thing anyone optimizes. On this renderer and GPU it is not what costs
time. The same investigation later found the real dominant cost: DDGI's
per-probe occlusion rays (~48–52% of the frame).

## How to apply

- Before any optimization that targets step count (over-relaxation, better
  steppers, epsilon tuning), repeat the `MAX_MARCH_STEPS` cut as a sanity check.
  If slashing the cap does not move `gpu trace`, the optimization cannot either.
- Profile the trace pass (RenderDoc capture, or per-BVH-node-visit counters)
  before guessing.

## Evidence

- Commit `a3be2c4` (perf pass items 1–2).
- `gpu trace` ~137–143 ms at `--stress 10000` with `MAX_MARCH_STEPS` 128 or 32.

## Related
- [DDGI any-hit occlusion](../gi-and-lighting/ddgi-any-hit-occlusion.md) — deeper: where the trace-pass time actually goes.
- [Render scale subsumes half-res reflections](./render-scale-subsumes-half-res-reflections.md) — deeper: items 2 and 3 of the same perf pass.
- [Skybox / far-object bake not worth it](./skybox-far-object-bake-not-worth-it.md) — example: another cost premise that failed a measurement.
- [Hardening research for the tiered hybrid renderer](../plans/hybrid-renderer-hardening.md) — contrast: its step accelerators assume marching dominates.
- [Sphere tracing](../../sdf-3d/rendering/sphere-tracing.md) — prerequisite: the algorithm and its step-count theory.
- [A measurement of a broken system](../../engineering-practice/measurement/a-measurement-of-a-broken-system.md) — deeper: why a premise must be measured on this system, not assumed from theory.
