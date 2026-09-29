---
title: "Raymarching via compute: what it buys migera (and what it doesn't)"
description: Argues compute beats a fragment raymarcher only through structure (internal resolution, hit/shade split, work queues, temporal reuse), not by switching stage, and lays out a 3-phase plan; Status records which phases src/hybrid built. Read before restructuring the hybrid trace dispatch.
type: design
status: current
tags:
  - raymarching
  - gpu-compute
  - performance
  - hybrid-renderer
  - lod
updated: 2026-09-28
verified: 2026-09-28
code:
  - assets/shaders/hybrid_trace.wgsl
  - assets/shaders/hybrid_blit.wgsl
  - src/hybrid/pass.rs
  - src/hybrid/extract.rs
aliases:
  - compute raymarcher
  - dynamic resolution
  - render scale
  - hit/shade split
  - tile work queue
---

# Raymarching via compute: what it buys migera (and what it doesn't)

Written 2026-08-23 against the original fullscreen-fragment marcher
(`assets/shaders/raymarch.wgsl`); see [Status](#status) for what the `src/hybrid`
rewrite has since built.

## The honest baseline

A fullscreen fragment raymarcher is already "one invocation per pixel" - switching to
compute with one invocation per pixel is **not automatically faster**. A controlled
deferred-shading study measured naive compute tiling ~2x *slower* than the same shading
in a fragment pass (vendor pixel-ordering heuristics are that good). Compute pays off
only when its structural features change the algorithm: shared memory, work queues,
scatter writes, multi-resolution passes, and CPU-free GPU-driven scheduling.

## What limited the original fragment marcher

- Sphere-tracing divergence: ~80% of steps go to ~20% of pixels (silhouettes); a wave
  runs at its slowest lane's pace. Fragment gives zero control over wave composition.
- Every effect (shadows, AO, reflection) is bolted inside the same per-pixel program at
  full resolution; no way to run them at lower resolution or reuse neighbors' results.
- No temporal state: every frame re-marches everything from scratch.
- Resolution is welded to the swapchain; can't scale workload under load.

These are exactly the things compute + intermediate textures unlock.

## Proposed architecture (phased)

Phase 1 - move the marcher into a compute pass writing an RGBA16F storage texture
(color) at *internal* resolution, plus a tiny fragment blit into `ViewTarget` before
tonemapping (Bevy integration doc, option 2). Immediate wins:
  - Dynamic resolution scaling: dispatch grid sized by measured frame time.
  - Workgroup size becomes a tunable (start 8x8; try single-wave 64-thread groups -
    Claybook finding - for march-heavy shaders).
  - Thread-group swizzling for L2 locality once the scene grows.

Phase 2 - split primary hit from shading:
  Pass A computes per-pixel hit data (depth/t, normal oct-encoded, material id) into
  buffers/textures. Pass B shades using it. Enables half-res secondary effects:
  shadow/AO/reflection traced at quarter sample count in their own dispatches with
  bilateral upsample. This is the standard deferred-hybrid shape.

Phase 3 - divergence & reuse:
  - Tile-based work queues (atomic counter + indirect dispatch): pixels sorted into
    cost buckets; long rays stop pinning short-ray waves. The study above shows this
    form beating fragment shading outright.
  - Temporal accumulation of marched distance/first-hit (ping-pong textures keyed on
    reprojection): sky/background rays reuse last frame's miss evidence; silhouettes
    still re-march. Requires motion vectors - start with static-camera assumption used
    by this demo's orbit camera only when idle.

## Sphere-tracing-specific guidance

- Single-wave groups for the marching pass (AMD GPUOpen's own recommendation for
  fluctuating-loop kernels).
- Keep map() branch-uniform per wave where possible; the RPN eval_stack is already
  data-uniform across lanes at a given t only if all lanes step together - they don't.
  Wave-divergent stepping is inherent; mitigate via cost-bucketing rather than trying
  to lockstep.
- Watchdogs: cap total steps conservatively under software renderers (llvmpipe) and
  integrated parts.

## Measurement plan

Use RenderDiagnosticsPlugin (timestamp queries) to get true pass times; add a debug
heatmap mode keyed off computed internal resolution; validate each phase against the
8fps baseline on the Renoir iGPU before accepting complexity. Decision rule: if Phase 1
compute blit alone regresses vs fragment (possible per the study), jump straight to
Phase 2 structure where the real wins live, or stay fragment-only with resolution
scaling implemented via viewport tricks.

## Status

Checked against the code on 2026-09-28:

- **Phase 1 — built.** `src/hybrid` traces in a compute pass
  (`assets/shaders/hybrid_trace.wgsl`, `@workgroup_size(8, 8, 1)` = 64 invocations)
  and blits into the view (`hybrid_blit.wgsl`), first landed in commit 53b3f4a.
  Internal resolution is `RenderScaleConfig` (`src/hybrid/extract.rs`); its measured
  effect is in [render_scale subsumes half-res reflections](../hybrid-architecture/performance-findings/render-scale-subsumes-half-res-reflections.md).
- **Measurement plan — built.** Per-pass timestamp spans exist (see
  [CPU<->GPU data flow](./cpu-gpu-data-flow.md)); the Renoir "8fps baseline" above
  refers to the legacy marcher, not to current numbers (those live in `PROGRESS.md`).
- **Phase 2 hit/shade split — not built.** Documented as an extension seam in
  [Module and stage skeleton](../hybrid-architecture/module-and-stage-skeleton.md).
  Temporal accumulation exists for GI, reflections and transmission
  (`hybrid_temporal.wgsl`), not for primary-hit reuse.
- **Phase 3 tile work queues — not built.** The measured trace-pass bottleneck is
  secondary-ray work, not primary march steps
  ([trace-pass bottleneck](../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md)),
  so cost-bucketing primary rays is not currently the lever.

## Related
- [Compute shader performance: best practices & anti-patterns](./performance-best-practices.md) — prerequisite: the divergence and occupancy facts this plan is built on.
- [Compute shaders in Bevy 0.19](./bevy-integration.md) — prerequisite: the storage-texture + blit wiring Phase 1 uses.
- [Sphere tracing](../sdf-3d/rendering/sphere-tracing.md) — prerequisite: the march algorithm being moved into compute.
- [Rendering grid-based and hybrid SDF representations](../sdf-3d/rendering/hybrid-and-grid-rendering.md) — contrast: other ways to cut per-pixel march cost.
- [Module and stage skeleton](../hybrid-architecture/module-and-stage-skeleton.md) — applies: where the unbuilt Phase 2 split would land.
- [Trace-pass bottleneck is not march steps](../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — contrast: measured evidence against spending effort on primary-ray step counts.
- [render_scale subsumes half-res reflections](../hybrid-architecture/performance-findings/render-scale-subsumes-half-res-reflections.md) — example: the built resolution-scaling lever, measured.
