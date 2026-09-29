---
title: PBR and Lighting
description: Bevy 0.19.1's largest subsystem across bevy_light (GPU-free data) and bevy_pbr (GPU side) - light components and atmosphere, GPU light clustering, shadow maps and contact shadows, StandardMaterial and the Material trait, and SSR. Read for lights, shadows, sky, PBR materials, or light-count cost.
type: index
status: current
tags:
  - bevy
  - lighting
  - shadows
  - materials
updated: 2026-09-28
---

# PBR and Lighting

Physically-based materials, clustered light assignment, shadow mapping and screen-space
lighting. It spans `bevy_light` (pure data/CPU logic, no GPU dependency) and `bevy_pbr`
(extraction, buffer layout, WGSL). 0.19 brought GPU-driven light clustering (~20x faster
per the release notes), contact shadows, physically-based SSR, and `Atmosphere` as a
standalone world entity instead of a camera attachment.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [bevy_light: light components, atmosphere, gizmos](./light-components-and-atmosphere.md) | Light components and their shadow-bias knobs; why `bevy_light` has no `bevy_render` dependency; `Atmosphere` as its own entity; `ScatteringMedium`; light gizmos. | Configuring lights, shadow bias, or sky/atmosphere. |
| [Clustered forward rendering — now on the GPU](./clustered-forward-rendering.md) | Froxel grid and `ClusterConfig` modes; 0.19 GPU clustering via rasterizer-as-intersection-test plus prefix sum. | Many dynamic lights cost too much, or a light misses an object in range. |
| [Shadow rendering: shadow maps, cascades, and contact shadows](./lighting-and-shadows.md) | Shadow maps as per-light `RetainedViewEntity` views; cascade distribution; bias tuning; contact shadows (need `DepthPrepass`). | Shadows alias, acne, detach, or look soft at contact. |
| [StandardMaterial and the Material trait](./standard-material-and-pbr.md) | `bevy_pbr` module map, the `Material` trait, `MaterialExtension`, `StandardMaterial` texture slots/bindless/shader defs. | Writing or extending a PBR material, or a material forces forward rendering. |
| [Screen-space reflections (SSR)](./screen-space-reflections.md) | Roughness-aware depth-buffer raymarching; deferred + `DepthPrepass` only; shared helper with contact shadows. | Adding reflective surfaces or debugging missing reflections. |

## See also

- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — applies: how `src/hybrid` consumes Bevy's `DirectionalLight` and the light gap it documents.
- [migera pivots to Bevy PBR for characters](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — applies: characters render through this stock PBR path.
