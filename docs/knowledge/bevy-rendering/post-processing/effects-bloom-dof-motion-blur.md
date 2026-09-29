---
title: Bloom, depth of field, motion blur, auto exposure
description: Bevy 0.19.1's bevy_post_process implements bloom (mip-chain dual filter, pre-tonemap), depth of field (Gaussian or bokeh from the depth prepass), motion blur (motion vectors) and auto exposure (compute histogram) as ordered Core3d systems; SSAO lives in bevy_pbr instead. Read when adding or ordering camera effects.
type: reference
status: current
tags:
  - bevy
  - post-processing
  - render-pipeline
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_post_process-0.19.1/src/bloom/, dof/, motion_blur/, auto_exposure/
aliases:
  - bloom
  - depth of field
  - DoF
  - motion blur
  - auto exposure
  - SSAO location
---

# Bloom, depth of field, motion blur, auto exposure

All effects here live in `bevy_post_process/src/` and are, like anti-aliasing (see
[anti-aliasing](./anti-aliasing.md)), ordinary systems inserted into a camera's `Core3d`/
`Core2d` schedule, chained relative to each other and to `tonemapping`.

## Bloom (`bloom/`)

Simulates light bleeding/glow around bright areas. A mip-chain of fragment-shader passes:
repeated downsample-with-threshold/prefilter, then an upsample-and-additively-composite
chain back up (classic "dual filtering" bloom), each mip level its own render
pass/texture. Component: `Bloom` (with nested `BloomPrefilter` for threshold/knee).
Registered `bloom.before(tonemapping).in_set(Core3dSystems::PostProcess)` — runs on HDR
data before tonemapping so blooming maintains correct exposure.

## Depth of Field (`dof/`)

Simulates camera focus falloff (bokeh blur outside the focal plane). Fragment-shader
passes using a `DepthOfFieldMode` (Gaussian vs. a more physical bokeh mode) and a depth
prepass to determine circle-of-confusion per pixel. Component: `DepthOfField { mode,
focal_distance, aperture_f_stops, ... }`. Registered explicitly
`depth_of_field.after(bloom).before(tonemapping)` in `Core3d` only — no `Core2d` support,
since DOF needs 3D depth.

## Motion Blur (`motion_blur/`)

Simulates blur from camera/object motion within the frame's shutter time. Single
fragment-shader pass sampling along the per-pixel motion-vector direction (from the
motion-vector prepass) a configurable number of times, weighted by `shutter_angle`.
Component: `MotionBlur { shutter_angle, samples }`. Registered
`motion_blur.before(bloom).in_set(Core3dSystems::PostProcess)`.

## Auto Exposure (`auto_exposure/`)

Simulates eye/camera exposure adaptation: builds a luminance histogram from the scene and
adjusts exposure over time toward a target. This one is **compute-shader** based
(histogram build + reduce) rather than a fragment pass, feeding an exposure value consumed
by tonemapping. Component: `AutoExposure`, plus an optional
`AutoExposureCompensationCurve` asset. Registered `.in_set(Core3dSystems::PostProcess)`,
ordered so its output is ready before tonemapping consumes exposure.

## SSAO is not in `bevy_post_process`

Screen-space ambient occlusion lives in `bevy_pbr`, not here — it needs deferred/prepass
G-buffer data tightly coupled to the PBR lighting pipeline, so it's implemented alongside
that pipeline rather than as a generic post-process crate effect. See
[standard-material-and-pbr](../pbr-and-lighting/standard-material-and-pbr.md) for the
`bevy_pbr` module map.

## When to dive in

- Adding cinematic camera effects (bloom, DOF, motion blur) → these are the components to
  add to a `Camera3d`; check the `.before()`/`.after()` ordering above if layering
  multiple effects to understand which sees which intermediate result.
- Tuning exposure/auto-exposure behavior → `AutoExposure`/`AutoExposureCompensationCurve`.
- Looking for SSAO → it's in `bevy_pbr`, not this crate.

## Related
- [Anti-aliasing techniques](./anti-aliasing.md) — contrast: the AA systems placed in the same sets.
- [The effect stack and writing a custom effect](./effect-stack-and-custom-effects.md) — deeper: the fused effect pass that runs after DOF, and the custom-effect recipe.
- [Prepass, tonemapping, upscaling, deferred, and OIT](../core-pipeline/passes-and-fullscreen-effects.md) — prerequisite: the tonemapping pass these effects are ordered around and the prepasses DOF/motion blur read.
- [Screen-space effect compatibility for splats](../../sdf-3dgs-bevy-integration/render-integration/screen-space-effect-compatibility.md) — applies: which of these effects work on custom-rendered pixels for free.
