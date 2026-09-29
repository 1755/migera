---
title: Anti-aliasing techniques
description: Covers Bevy 0.19.1's bevy_anti_alias (FXAA, TAA, SMAA, CAS, DLSS) as ordinary Core3d systems chained via ViewTarget ping-pong, which run pre- vs post-tonemap, and why MSAA is a camera sample-count setting rather than an effect. Read when choosing AA or diagnosing ghosting, shimmer, or upscale blur.
type: reference
status: current
tags:
  - bevy
  - post-processing
  - temporal
  - render-pipeline
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_anti_alias-0.19.1/src
aliases:
  - FXAA
  - TAA
  - SMAA
  - CAS
  - DLSS
  - MSAA
---

# Anti-aliasing techniques

## The ECS-schedule model

Each camera runs a per-camera `Schedule` (`Core3d`/`Core2d`) containing four chained
system sets: `Prepass → MainPass → EarlyPostProcess → PostProcess` (see
[core-pipeline](../core-pipeline/INDEX.md)). AA effects are ordinary render-world systems
placed into one of these sets, ordered relative to named systems like `tonemapping` via
`.after()`/`.before()`. `ViewTarget` (see
[camera-and-view-system](../scene-and-views/camera-and-view-system.md)) owns the two
ping-pong textures; `post_process_write()` flips the active one and hands the caller a
`{ source, destination }` pair, so a chain of independently-registered systems composes
correctly without any graph wiring — ordering is entirely `bevy_ecs` schedule ordering.

**MSAA itself is not an anti-alias "effect" in this crate** — the `Msaa` enum lives in
`bevy_render::view`, a camera-driver-level sample-count setting consumed when render
targets/pipelines are created (see
[camera-and-view-system](../scene-and-views/camera-and-view-system.md)). A related but
distinct utility, `msaa_writeback` (in `bevy_post_process`), blits the resolved MSAA
target back into a sampled attachment so downstream draws (e.g. transparent UI) can still
write into an MSAA-enabled target — it is not an AA algorithm itself.

## Techniques, by file (`bevy_anti_alias/src/`)

- **FXAA** (`fxaa/`) — targets jagged geometric edges cheaply, no temporal or depth data
  needed. Single fragment-shader pass: luminance-edge detection + local blending along the
  edge direction, with a `Sensitivity` enum (Low..Extreme) via shader defines. Component:
  `Fxaa { enabled, edges }`. Registered `.after(tonemapping).in_set(Core3dSystems::
  PostProcess)` (and `Core2d` equivalent) — runs on the *tonemapped LDR* image, like most
  classic screen-space AA.

- **TAA** (`taa/`) — targets edge aliasing *and* shader/specular aliasing by accumulating
  jittered samples over time; trades ghosting risk for much higher quality. Requires
  `DepthPrepass` + `MotionVectorPrepass`, uses `TemporalJitter` (subpixel camera
  projection jitter) plus history textures reprojected via motion vectors and blended with
  the current frame (neighborhood clamping to fight ghosting). Component:
  `TemporalAntiAliasing`. Registered `.in_set(Core3dSystems::EarlyPostProcess)` —
  *before* tonemapping/other post effects, since it operates on the working HDR buffer.

- **SMAA** (`smaa/`) — same target as FXAA (jaggies) with subpixel morphological pattern
  matching for higher quality at higher cost; no temporal accumulation (the module doc
  explicitly notes the temporal SMAA variant isn't implemented). A **three-pass**
  pipeline: edge detection → blending-weight calculation (samples precomputed area/search
  LUT textures) → neighborhood blending. Component: `Smaa { preset }`. Registered
  `.after(tonemapping).in_set(Core3dSystems::PostProcess)`; docs recommend `Msaa::Off`
  when using it.

- **CAS** (`contrast_adaptive_sharpening/`) — not an anti-aliaser targeting jaggies but a
  post-upscale/post-AA **sharpening** pass (AMD's "robust CAS") restoring detail softened
  by other AA/upscaling; single fragment-shader pass adaptively sharpening based on local
  contrast. Component: `ContrastAdaptiveSharpening`. Registered
  `.in_set(Core3dSystems::PostProcess)` (Core2d too), typically last in that set.

- **DLSS** (`dlss/`, feature-gated behind `dlss` + not `force_disable_dlss`) — NVIDIA's
  neural super-resolution/ray-reconstruction AA-and-upscale, via the `dlss_wgpu` crate/
  SDK; needs a `DlssProjectId`, hardware support markers, and a `DlssSdk` resource.
  Component: `Dlss<F: DlssFeature>` generic over `DlssSuperResolutionFeature`/
  `DlssRayReconstructionFeature`. Registered `.in_set(Core3dSystems::EarlyPostProcess)`,
  alongside TAA conceptually, since it also consumes/produces the working HDR buffer and
  motion vectors before tonemapping.

## Set-placement summary

TAA and DLSS operate pre-tonemap in `EarlyPostProcess` (they need linear HDR + motion
vectors); FXAA, SMAA, CAS operate post-tonemap in `PostProcess` (classic screen-space
techniques operating on the final LDR image).

## When to dive in

- Choosing an AA technique for a project → FXAA/CAS for cheap+simple, TAA for highest
  quality (at the cost of ghosting risk and requiring prepasses), SMAA as a non-temporal
  middle ground, DLSS where hardware support and licensing allow.
- Debugging AA artifacts → ghosting/smearing points to TAA history/motion-vector issues;
  shimmering on thin geometry points to insufficient sample count or missing AA entirely;
  blurriness after upscaling → consider adding CAS.
- Writing a custom AA-like effect → decide pre- or post-tonemap placement based on whether
  it needs linear HDR data, following the set-placement pattern above.

## Related
- [Camera and the view system](../scene-and-views/camera-and-view-system.md) — prerequisite: the `ViewTarget` ping-pong and `Msaa` setting.
- [Prepass, tonemapping, upscaling, deferred, and OIT](../core-pipeline/passes-and-fullscreen-effects.md) — prerequisite: the motion-vector and depth prepasses TAA needs.
- [Bloom, depth of field, motion blur, auto exposure](./effects-bloom-dof-motion-blur.md) — contrast: the other built-in effects and their ordering around tonemapping.
- [The effect stack and writing a custom effect](./effect-stack-and-custom-effects.md) — deeper: the template for writing an AA-like pass.
- [Hybrid GI temporal accumulation](../../hybrid-architecture/gi-and-lighting/hybrid-gi-temporal-accumulation.md) — contrast: migera's own reprojection-plus-history approach in `src/hybrid`.
