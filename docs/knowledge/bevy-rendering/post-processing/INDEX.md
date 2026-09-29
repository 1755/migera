---
title: Post-Processing
description: Bevy 0.19.1 screen-space effects - anti-aliasing, bloom, DOF, motion blur, auto exposure, and the fused vignette/lens-distortion/chromatic-aberration pass - all systems in Core3d post-process sets chained via ViewTarget::post_process_write(). Read when adding, ordering or writing an effect.
type: index
status: current
tags:
  - bevy
  - post-processing
  - render-pipeline
updated: 2026-09-28
---

# Post-Processing

Screen-space effects applied after the main scene is rendered. They share one pattern: a
system in `Core3dSystems::EarlyPostProcess`/`PostProcess` that uses
`ViewTarget::post_process_write()` to chain with its neighbours, ordered by
`.before()`/`.after()` around `tonemapping`. So this topic is also the reference for writing
your own effect. The conceptual `ViewTarget` ping-pong lives in
[camera-and-view-system](../scene-and-views/camera-and-view-system.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Anti-aliasing techniques](./anti-aliasing.md) | FXAA, TAA, SMAA, CAS, DLSS: what each targets, pre- vs post-tonemap placement; MSAA is a camera setting, not in this crate. | Choosing AA, or ghosting, shimmer, or post-upscale blur. |
| [Bloom, depth of field, motion blur, auto exposure](./effects-bloom-dof-motion-blur.md) | Mip-chain dual-filter bloom, prepass-driven DOF, motion-vector blur, compute-histogram auto exposure; SSAO is in `bevy_pbr`. | Adding cinematic effects or reasoning about which effect sees which intermediate. |
| [The effect stack and writing a custom effect](./effect-stack-and-custom-effects.md) | Vignette/lens distortion/chromatic aberration fused into one pass; `Vignette` traced end to end; recipe for a custom effect. | Before writing any custom post-process pass. |

## See also

- [Screen-space effect compatibility for splats](../../sdf-3dgs-bevy-integration/render-integration/screen-space-effect-compatibility.md) — applies: which built-in effects work on custom-rendered pixels without extra code.
- [Hybrid GI temporal accumulation](../../hybrid-architecture/gi-and-lighting/hybrid-gi-temporal-accumulation.md) — contrast: migera's own temporal reprojection in `src/hybrid`.
