---
title: RenderScaleConfig already gives reflections the half-res win
description: A half-res reflection/refraction tier with bilateral upsample was not built, because RenderScaleConfig already scales those passes (render_scale 1.0 to 0.5 gave ~3.2–3.5x on reflect/transmit temporal and denoise) and rough surfaces are already cone-blurred. Read before adding a per-pass resolution knob.
type: decision
status: current
tags:
  - performance
  - lod
  - post-processing
  - hybrid-renderer
updated: 2026-09-19
verified: 2026-09-28
code:
  - src/hybrid/pipeline.rs
  - src/hybrid/extract.rs
  - assets/shaders/hybrid_trace.wgsl
  - src/hybrid/reflect_ref.rs
sources:
  - Claude memory render_scale_subsumes_reflection_halfres_idea (2026-09-19)
  - commit a3be2c4
  - PROGRESS.md "Performance: item 3 (half-res reflections/refraction) -- measured, not implemented" entry
aliases:
  - half-res reflections
  - render_scale
  - trace resolution scale
  - bilateral upsample
---

# RenderScaleConfig already gives reflections the half-res win

A dedicated half-resolution tier for reflections and refraction (with a
bilateral upsample) was researched, measured and **not built**. The existing
`RenderScaleConfig`, which scales the trace resolution, already shrinks those
passes by about the same factor.

## Context

A three-item perf pass on 2026-09-19: distance-scaled march epsilon, primary-ray
jitter plus render-scale upsampling (both shipped in `a3be2c4`), and half-res
reflections (this note).

## Decision

Not implemented, because:
- Reflection and refraction are written by the **same** `trace_main` dispatch at
  the same `scene.trace_size` resolution as everything else, so
  `RenderScaleConfig` already scales them.
- The classic argument ("rough reflections are blurry, so downsample them") is
  already handled another way: rough surfaces get a spatial self-blur from their
  roughness-widened reflection cone (`reflect_trace_ray`) and skip temporal
  accumulation entirely above `REFLECT_ROUGHNESS_GATE`/`TRANSMIT_ROUGHNESS_GATE`
  (0.3).

## Alternatives considered

- **Separate half-res reflection tier + bilateral upsample.** Real SOTA practice
  in general, but not additive here. PROGRESS.md keeps the re-implementation
  map (a new `SceneUniform` field, new storage textures, new dispatch locals,
  bilateral upsample reusing `hybrid_denoise.wgsl`'s `BLUR_NORMAL_SIGMA`/
  `BLUR_DEPTH_SIGMA`).

## Consequences

Measured on a mirror cube (roughness 0.0, metallic 1.0: the below-the-gate case,
with no cone blur or temporal skip hiding the cost), `render_scale` 1.0 → 0.5:
- reflect-temporal ~4.3 ms → 1.35 ms (~3.2x)
- transmit-temporal ~4.3 ms → 1.3 ms (~3.3x)
- denoise ~2.25 ms → 0.65 ms (~3.5x)
- Reflection/refraction passes kept about the same share of the frame (~27% →
  23%), so they are not a growing bottleneck left behind.

## Revisit when

- A scene shows reflections dominating frame cost far more than this test case:
  many large mirrors, or reflection `max_bounces` well above the default of 1.

## Related
- [Trace-pass bottleneck is not march steps](./trace-pass-bottleneck-is-not-march-steps.md) — deeper: item 1 of the same perf pass.
- [Skybox / far-object bake not worth it](./skybox-far-object-bake-not-worth-it.md) — same-trap: another SOTA technique whose premise did not hold here.
- [DDGI any-hit occlusion](../gi-and-lighting/ddgi-any-hit-occlusion.md) — deeper: the follow-up investigation that found the real dominant cost.
- [Module and stage skeleton](../module-and-stage-skeleton.md) — contrast: its hit/shade split seam was waiting for exactly this feature.
- [Screen-space reflections](../../bevy-rendering/pbr-and-lighting/screen-space-reflections.md) — contrast: how Bevy's own reflections trade resolution for cost.
