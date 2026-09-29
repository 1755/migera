---
title: Prepass, tonemapping, upscaling, deferred, and OIT
description: Maps Bevy 0.19.1's default Core3d/Core2d passes in order - depth/normal/motion prepass (two-phase for occlusion culling), tonemapping, upscaling blit, deferred as a fancier prepass (disables MSAA), OIT - plus FullscreenMaterialPlugin. Read when an effect lacks depth/normals or pass order matters.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - post-processing
  - rasterization
  - culling
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_core_pipeline-0.19.1/src/prepass/, tonemapping/, upscaling/, deferred/, oit/
aliases:
  - depth prepass
  - DepthPrepass
  - deferred rendering
  - order-independent transparency
  - FullscreenMaterialPlugin
  - MSAA silently disabled
---

# Prepass, tonemapping, upscaling, deferred, and OIT

Contents: [Prepass](#prepass-depth-normal-motion-vector) · [Tonemapping](#tonemapping) · [Upscaling](#upscaling-the-final-blit) · [Deferred](#deferred-rendering) · [OIT](#order-independent-transparency-oit) · [Utility modules](#supporting-utility-modules) · [When to dive in](#when-to-dive-in) · [Related](#related)

This document maps the concrete default passes registered by `Core3dPlugin`/`Core2dPlugin`
(see [camera-driven-scheduling](./camera-driven-scheduling.md) for the scheduling
mechanism these plug into), in pipeline order.

## Prepass: depth, normal, motion-vector

A prepass renders opaque/alpha-masked geometry to depth (and optionally normals/motion
vectors) *before* the main color pass, producing a "thin G-buffer." Used for: SSAO and
other screen-space effects needing scene depth/normals without waiting for full shading;
TAA and motion blur (motion vectors); occlusion culling (a `late_prepass` Hi-Z pass can
cull geometry against depth already written by `early_prepass`); reduced overdraw via
early-Z rejection in the main pass.

Components `DepthPrepass`, `NormalPrepass`, `MotionVectorPrepass`, `DeferredPrepass` (and
`*DoubleBuffer` variants for temporal reprojection) are opt-in on `Camera3d`.
`extract_camera_prepass_phase` mirrors these flags onto the render-world camera entity and
populates `ViewBinnedRenderPhases<Opaque3dPrepass>`/`<AlphaMask3dPrepass>`.
`prepare_prepass_textures` allocates the GPU textures (`NORMAL_PREPASS_FORMAT =
Rgb10a2Unorm`, `MOTION_VECTOR_PREPASS_FORMAT = Rg16Float`) sized to the camera's target.

The actual passes, `early_prepass` and `late_prepass`, are chained in
`Core3dSystems::Prepass`:

```rust
(early_prepass, early_deferred_prepass, late_prepass,
 late_deferred_prepass, copy_deferred_lighting_id)
    .chain().in_set(Core3dSystems::Prepass)
```

`early_prepass` always runs, writing depth (copying into the `DepthPrepass` texture if
enabled). `late_prepass` only runs `if occlusion_culling && !no_indirect_drawing`,
re-rendering after a depth-pyramid (Hi-Z) pass has culled more objects — this two-phase
split is what enables GPU-driven occlusion culling. Both call a shared `run_prepass_
system` helper that opens one render pass, renders `opaque_prepass_phase` then
`alpha_mask_prepass_phase`, and optionally draws a background-motion-vectors fullscreen
triangle.

Strictly before `MainPass` in the chain — its outputs (depth/normal/motion textures, and
the primary depth buffer) are available as bind-group inputs to opaque/transparent draws
and to later post-process passes (SSAO, TAA) a user plugin might insert.

## Tonemapping

Bevy's lighting math is linear HDR; tonemapping compresses the unbounded signal into a
displayable range while controlling hue/contrast. It must run *after* all HDR-lit geometry
(main pass) but *before* the image reaches a non-HDR swapchain.

`Tonemapping` is an enum component (required-component default on both `Camera3d`/
`Camera2d`) with variants `None, Reinhard, ReinhardLuminance, AcesFitted, AgX,
SomewhatBoringDisplayTransform, TonyMcMapface (default), BlenderFilmic, ...`. AgX and
TonyMcMapface use 3D LUT textures (gated by the `tonemapping_luts` feature). The
`tonemapping` system early-returns unless the camera is actually rendering in HDR
(`camera.hdr`) — non-HDR cameras skip it entirely. It reads/writes via `target.
post_process_write()` (see
[camera-and-view-system](../scene-and-views/camera-and-view-system.md)) — the pattern any
custom post-process pass should reuse.

Registered as `tonemapping.in_set(Core3dSystems::PostProcess)` (and `Core2dSystems::
PostProcess`) — i.e. in the **last** logical set, not `EarlyPostProcess`. `EarlyPostProcess`
exists as a placeholder extension point for effects that must run on the raw HDR image
*before* tonemapping (bloom, depth of field — see
[post-processing](../post-processing/INDEX.md)); nothing in `bevy_core_pipeline` itself
populates it.

## Upscaling: the final blit

Bevy typically renders off-screen into a `ViewTarget` (which may differ in size/format
from the window under MSAA, custom render scale, or HDR-to-SDR conversion) and must copy/
blend that into the actual swapchain at the end. `upscaling` blits `target.
main_texture_view()` to `target.out_texture_color_attachment(clear_color)` using the
shared `BlitPipeline`, also handling clearing/scissor-rect setup per camera and honoring
`CameraOutputMode` (`Skip` vs `Write { blend_state, clear_color }`) so multiple cameras
targeting the same window composite correctly.

`upscaling.after(Core3dSystems::PostProcess)` / `.after(Core2dSystems::PostProcess)` —
registered *inside* `Core3d`/`Core2d` (part of each camera's own schedule) but strictly
after the entire `PostProcess` set, so it is the last system in the camera's schedule,
guaranteeing every post-process effect has already written its final result before the
upscale/blit.

## Deferred rendering

Deferred shading writes geometric/material data (normals, motion vectors, a packed
G-buffer, and a lighting-pass-id) in one pass, deferring lighting computation to a full-
screen pass later — useful for scenes with many lights. Built as an extension of the
prepass infrastructure: `Opaque3dDeferred`/`AlphaMask3dDeferred` are additional binned
phases populated by the same `extract_camera_prepass_phase` when `DeferredPrepass` is
present. `early_deferred_prepass`/`late_deferred_prepass` run interleaved with the plain
prepass in the same chained group, writing `DEFERRED_PREPASS_FORMAT = Rgba32Uint` (the
G-buffer) and `DEFERRED_LIGHTING_PASS_ID_FORMAT = R8Uint` (which material/lighting shader
owns each pixel), then copying depth into the shared prepass depth texture.
`copy_deferred_lighting_id` runs last in the chain, copying the lighting-pass-id into a
dedicated texture the deferred lighting shader (in `bevy_pbr`) uses to dispatch per-
material lighting logic in a stencil-masked full-screen pass. MSAA is forcibly disabled
when `DeferredPrepass` is present, since deferred G-buffers are incompatible with
multisampling in this implementation.

Entirely within `Core3dSystems::Prepass`, chained after depth/normal/motion — deferred
rendering is modeled as "a fancier prepass"; the actual lighting resolve happens later (in
`bevy_pbr`, not `bevy_core_pipeline`), typically inside `MainPass`.

## Order Independent Transparency (OIT)

Standard back-to-front sorted alpha blending (`Transparent3d`) produces artifacts for
interpenetrating or unsortable transparent geometry. OIT captures all transparent
fragments per pixel into a linked list, then sorts and blends them correctly in a resolve
pass. `OrderIndependentTransparencySettings` is opt-in. Pass 1 is effectively a forward
pass writing depth/color per-fragment into a linked-list buffer (via `oit_draw(position,
color)` calls from user shaders, inside `main_transparent_pass_3d`); pass 2, `oit_resolve`,
is a single fullscreen-triangle pass that sorts and composites:

```rust
render_app.add_systems(Core3d,
    oit_resolve.after(main_transparent_pass_3d).in_set(Core3dSystems::MainPass));
```

A clean illustration of extending the pipeline: `oit_resolve` is inserted into the
*existing* `MainPass` set with an explicit `.after()` constraint, rather than needing a
new set.

## Supporting utility modules

- **`skybox/`**: renders a cubemap background using a dedicated pipeline drawn directly
  inside `main_opaque_pass_3d` (not a separate schedule system) — a fullscreen triangle
  drawn opportunistically at the end of the opaque pass, functioning as a cheap "clear to
  sky."
- **`mip_generation/`**: AMD FidelityFX single-pass downsampling (SPD) for generating
  mipmaps and for building a hierarchical Z-buffer used by occlusion culling.
- **`blit/`**: `BlitPipeline`, a minimal "sample one texture, write it to another"
  fullscreen-triangle pipeline shared by `upscaling` and tonemapping/post-process code.
- **`fullscreen_vertex_shader/`**: `FullscreenShader` resource, providing the shared
  vertex stage (`draw(0..3, 0..1)`, one oversized triangle covering the viewport — the
  standard technique avoiding a quad's diagonal seam) used by virtually every post-
  process/blit pipeline in the crate.
- **`fullscreen_material.rs`**: `FullscreenMaterialPlugin<T: FullscreenMaterial>` — the
  officially blessed extension point for "add a custom post-process pass": implement
  `FullscreenMaterial` (`fragment_shader()`, optional `schedule()`/`schedule_configs()`
  overrides), and the plugin auto-registers a system in `Core3dSystems::PostProcess`
  before `tonemapping`. Start here for simple, single-fragment-shader effects rather than
  hand-rolling the pipeline/bind-group/system boilerplate.

## When to dive in

- Adding a screen-space effect needing depth/normals/motion vectors → require the
  relevant prepass component (`DepthPrepass`, etc.) on your effect's marker component.
- Writing the simplest possible custom post-process effect → use
  `FullscreenMaterialPlugin` before hand-rolling a pipeline.
- Debugging why MSAA silently turned off → check for `DeferredPrepass` on the camera.
- Wanting correct rendering of many overlapping transparent objects → consider
  `OrderIndependentTransparencySettings` instead of fighting sort-order artifacts.

## Related
- [Camera-driven scheduling](./camera-driven-scheduling.md) — prerequisite: the per-camera schedule mechanism these passes plug into.
- [The effect stack and writing a custom effect](../post-processing/effect-stack-and-custom-effects.md) — deeper: a full hand-rolled post-process pass, the alternative to `FullscreenMaterialPlugin`.
- [Screen-space reflections](../pbr-and-lighting/screen-space-reflections.md) — example: an effect that requires `DeferredPrepass` + `DepthPrepass`.
- [Screen-space effect compatibility for splats](../../sdf-3dgs-bevy-integration/render-integration/screen-space-effect-compatibility.md) — applies: which effects need a custom pass to write real prepass data.
- [Self-shading vs. G-buffer-writer decision](../../hybrid-architecture/self-shading-vs-gbuffer-decision.md) — contrast: why `src/hybrid` does not feed Bevy's deferred G-buffer.
