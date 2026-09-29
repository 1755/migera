---
title: Materials and Texturing
description: Putting colour and PBR material properties on an SDF surface that has no UVs — triplanar/biplanar projection and procedural noise, the unchanged Cook-Torrance BRDF plus traced reflections, smin-weighted material blending, and procedural vs. scanned authoring. Read when texturing or shading any SDF surface.
type: index
status: current
tags:
  - sdf
  - materials
  - lighting
updated: 2026-09-28
---

# Materials and Texturing

An SDF has no mesh, no vertices and no UV parameterization, so "how do you put
colour and physically based material properties on it" is a distinct problem from
shaping the geometry ([primitives-and-operators](../primitives-and-operators/INDEX.md))
or getting pixels out of the field ([rendering](../rendering/INDEX.md)). All notes
here are research-grounded, cited to IQ's articles, GDC/SIGGRAPH talks, papers, and
the Claybook/Dreams case studies.

## Start here

[uv-less-texturing](./uv-less-texturing.md) first — everything else depends on
triplanar/biplanar projection or procedural noise. Then
[pbr-shading-model](./pbr-shading-model.md), [material-blending](./material-blending.md),
and [procedural-vs-asset-authoring](./procedural-vs-asset-authoring.md) for the
pipeline decision.

## Key facts

- The BRDF is not SDF-specific: metallic-roughness Cook-Torrance applies unchanged; what's SDF-specific is that shadows, AO, reflections and translucency become cheap side-marches — see [pbr-shading-model](./pbr-shading-model.md).
- Triplanar/biplanar projection is the mandatory bridge for any 2D texture on an SDF — see [uv-less-texturing](./uv-less-texturing.md).
- smin's interpolation weight `h` can `mix()` materials at the same seam, keeping geometric and material transitions coincident — see [material-blending](./material-blending.md).
- Adding detail to the field itself (hypertexture, domain warp) degrades the Lipschitz bound and causes pitting — see [procedural-vs-asset-authoring](./procedural-vs-asset-authoring.md).
- Procedural is the native fit but not strictly better; production pipelines layer procedural macro-variation over scanned micro-detail — see [procedural-vs-asset-authoring](./procedural-vs-asset-authoring.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Texturing without UVs](./uv-less-texturing.md) | Triplanar/biplanar projection, world vs. object space, RNM normal-map blending, procedural noise and domain warping, field-derived cavity/curvature/material ID. | Texturing an SDF surface for the first time, or a triplanar seam/flattened normal map. |
| [Physically based shading for raymarched hits](./pbr-shading-model.md) | Unmodified Cook-Torrance, traced reflections vs. SSR/cubemaps, split-sum fallback, SDF-native SSS/translucency, cost budget. | Choosing/implementing a shading model, adding reflections or translucency. |
| [Blending and mixing multiple materials](./material-blending.md) | Material-ID propagation, smin-weight material blending, height/splat blending, triplanar × N layers, noise masks. | Assigning per-primitive materials, or a blended seam shows a colour jump. |
| [Procedural vs. asset-based material authoring](./procedural-vs-asset-authoring.md) | Shader math vs. scanned texture sets, hybrid pipelines, resolution independence, AI texture tools, cost crossover. | Deciding how to author a material, or fighting visible tiling. |

## See also

- [Lighting and shadows in Bevy](../../bevy-rendering/pbr-and-lighting/lighting-and-shadows.md) — the rasterized PBR path migera now uses for characters.
