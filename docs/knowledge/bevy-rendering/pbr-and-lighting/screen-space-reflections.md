---
title: Screen-space reflections (SSR)
description: Bevy 0.19.1's SSR raymarches the prepass depth buffer hierarchically and resolves hits with roughness-aware importance sampling from the G-buffer; it works only with deferred rendering plus DepthPrepass and shares its raymarch helper with contact shadows. Read before adding reflective surfaces.
type: reference
status: current
tags:
  - bevy
  - post-processing
  - raymarching
  - lighting
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_pbr-0.19.1/src/ssr/
aliases:
  - SSR
  - ScreenSpaceReflections
  - depth_ray_march_to_ws
---

# Screen-space reflections (SSR)

## What it is

`bevy_pbr::ssr` implements reflections via screen-space raymarching against the depth
prepass. The `ScreenSpaceReflections` component doc states it's "currently only supported
with deferred rendering" and requires `DepthPrepass` + `DeferredPrepass`.

`ssr/raymarch.wgsl` performs hierarchical depth raymarching against the prepass depth
buffer; `ssr.wgsl` resolves hits using the material's roughness/PBR data from the
G-buffer — i.e. it's PBR-aware (importance-sampling the reflection direction by
roughness) rather than a naive mirror reflection. This roughness-aware importance
sampling is the "physically-based" distinction referenced in Bevy's 0.19 release notes,
compared to simpler SSR implementations that only handle perfect mirror reflections.

The same raymarching helper (`raymarch::depth_ray_march_to_ws`) is shared with
[contact shadows](./lighting-and-shadows.md#contact-shadows-new-in-019) — both are
screen-space depth-buffer raymarching techniques, just consuming the result differently
(reflection color vs. shadow occlusion factor).

## When to dive in

- Adding reflective surfaces to a scene → requires deferred rendering (`DeferredPrepass`)
  plus `DepthPrepass`; won't work in a purely forward-rendered scene.
- Debugging reflection artifacts (missing reflections at screen edges, incorrect
  roughness response) → screen-space techniques inherently can't reflect off-screen
  geometry; check whether the artifact is a fundamental SSR limitation before assuming a
  bug.

## Related
- [Shadow rendering](./lighting-and-shadows.md) — contrast: contact shadows use the same depth raymarch for occlusion instead of color.
- [Prepass, tonemapping, upscaling, deferred, and OIT](../core-pipeline/passes-and-fullscreen-effects.md) — prerequisite: the deferred and depth prepasses SSR requires.
- [render_scale subsumes half-res reflections](../../hybrid-architecture/performance-findings/render-scale-subsumes-half-res-reflections.md) — contrast: how migera's own hybrid renderer prices its reflection pass.
