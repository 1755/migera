---
title: Temporal accumulation, not a wider blur, fixes indirect-diffuse banding
description: The hybrid renderer's indirect-diffuse banding was estimator variance, so it shipped temporal accumulation (per-object motion vectors, ping-ponged history, reprojection, commit 0b2c92b) instead of more samples or a wider blur; about +3 ms at --stress 10000. Read before touching hybrid_temporal.wgsl or denoising GI.
type: decision
status: current
tags:
  - temporal
  - global-illumination
  - hybrid-renderer
  - post-processing
updated: 2026-09-07
verified: 2026-09-28
code:
  - src/hybrid/motion.rs
  - src/hybrid/temporal_ref.rs
  - assets/shaders/hybrid_temporal.wgsl
  - assets/shaders/hybrid_denoise.wgsl
  - src/hybrid/pipeline.rs
sources:
  - Claude memory hybrid_gi_temporal_accumulation_decision (2026-09-07)
  - commit 0b2c92b
  - PROGRESS.md "Temporal accumulation for indirect diffuse" entry
aliases:
  - GI banding
  - reprojection
  - history buffer
  - SVGF
  - PreviousShapeTransform
---

# Temporal accumulation, not a wider blur, fixes indirect-diffuse banding

The banding in `src/hybrid`'s indirect diffuse was estimator variance, not a
bug in any pass. The fix shipped was **real temporal accumulation**: motion
vectors (including per-object motion), a ping-ponged history buffer and
reprojection, in commit `0b2c92b`.

## Context

- Stage B's single-bounce indirect diffuse used 5 fixed hemisphere sample
  directions per pixel and showed visible banding on curved surfaces such as a
  sphere.
- It had been hidden under raw-hash jitter noise. After switching to
  interleaved gradient noise (IGN) and adding an edge-aware spatial blur, the
  banding became clearly visible. A screenshot A/B showed the same faint bands
  with denoise off, just hidden under grain.

## Decision

The user chose full temporal accumulation over a cheaper blur patch, and
rejected "ship a patch now, plan temporal later". Built:
- `src/hybrid/motion.rs`: `PreviousShapeTransform` and a `PreUpdate` system.
  Bevy's `bevy_pbr::prepass::PreviousGlobalTransform` is filtered to
  `With<Mesh3d>`, so it is inert for this renderer's `Shape` entities.
- `src/hybrid/temporal_ref.rs`: CPU reference for reprojection, disocclusion
  and the EMA blend (17 tests).
- `assets/shaders/hybrid_temporal.wgsl`: a compute pass between trace and the
  existing spatial-blur denoise pass.
- A ping-ponged GPU history buffer (`HybridHistoryRes` in `pipeline.rs`), the
  first ping-pong pattern in this codebase.
- `TemporalConfig` with `--no-temporal` and `--temporal-max-history` flags.

The spatial blur (`hybrid_denoise.wgsl`, `blur_indirect_at` in `cpu_ref.rs`)
was kept and runs on top of the accumulated signal. That is the standard
SVGF shape: pixels with no history (disocclusion, first frame) still need a
spatial clean-up.

## Alternatives considered

- **More samples per pixel.** Linear GPU cost, does not scale.
- **A wider/smarter spatial blur.** Cheap, but it hides the artifact instead of
  reducing it, and over-smooths real detail at a large radius.

## Consequences

- Verified by screenshot A/B at the original repro angle
  (`--shape sphere --camera-pos 0,1.5,1.9`): banding gone with temporal on,
  back exactly with `--no-temporal`.
- Cost: about +3 ms over the pass's own ~4.5 ms copy-through baseline at
  `--stress 10000`. Resolution-bound, not scene-bound. The GPU was heavily
  contended during that measurement (see PROGRESS.md).
- Follow-up done later: the per-frame sample count became a tunable dial
  (PROGRESS.md "reduce per-frame indirect-diffuse sample count").
- Stage B itself was later removed; DDGI is now the only indirect-diffuse
  source, and this pass accumulates DDGI's indirect term. Reflections and
  transmission later got their own dedicated temporal histories on the same
  pattern.

## Revisit when

- A new GI source has its own temporal filter (as DDGI's probe relight does),
  making screen-space accumulation of the same signal redundant.

## Related
- [Radiance Cascades experiment](./radiance-cascades-experiment.md) — contrast: a GI technique without cross-frame accumulation, and what that cost it.
- [DDGI any-hit occlusion](./ddgi-any-hit-occlusion.md) — applies: DDGI, the indirect source this pass now accumulates.
- [Tier-transition popping](../plans/tier-transition-popping.md) — prerequisite: its "G0" plan called for exactly this pass.
- [Anti-aliasing](../../bevy-rendering/post-processing/anti-aliasing.md) — deeper: Bevy's TAA and the prepass motion vectors this renderer could not reuse for `Shape` entities.
