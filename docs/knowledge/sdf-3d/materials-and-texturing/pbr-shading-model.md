---
title: Physically based shading for raymarched hits
description: Establishes that the metallic-roughness Cook-Torrance BRDF applies to a raymarched hit unchanged; what is SDF-specific is traced reflections instead of SSR/cubemaps, split-sum as fallback, cheap SDF-native translucency/SSS, and that march cost, not the BRDF, dominates. Read when choosing or implementing SDF shading.
type: concept
status: current
tags:
  - sdf
  - lighting
  - materials
  - raymarching
  - performance
updated: 2026-08-16
verified: 2026-09-28
code:
  - assets/shaders/raymarch.wgsl
aliases:
  - Cook-Torrance
  - GGX
  - image-based lighting
  - subsurface scattering
  - metallic-roughness
---

# Physically based shading for raymarched hits

Contents: [BRDF isn't SDF-specific](#the-central-claim-the-brdf-isnt-sdf-specific-at-all) ·
[Metallic-roughness](#the-metallic-roughness-model) · [Cook-Torrance](#cook-torrance-brdf-math) ·
[Cost](#cost-in-a-per-pixel-sphere-tracing-budget) ·
[IBL and traced reflections](#image-based-lighting-where-sdf-raymarchers-diverge-from-rasterizers) ·
[SSS/translucency](#subsurface-scattering-and-translucency) · [Related](#related)

## The central claim: the BRDF isn't SDF-specific at all

Metallic-roughness Cook-Torrance (the glTF/Disney-derived model used by virtually every
modern real-time renderer) transfers to a raymarched hit point completely unchanged from
its use in a rasterizer or a conventional ray tracer — it's evaluated at a single point
given a normal and view/light vectors, and doesn't care how that point was found. What
*is* SDF-specific is (1) how material parameters get **stored/authored** with no vertices
or texels to hold them, (2) how the **lighting queries feeding the BRDF** (shadows, AO,
reflections, translucency) become nearly free side effects of distance-field marches the
renderer is already paying for, and (3) the raymarcher's own tight per-pixel cost budget,
which shapes which BRDF approximations are worth affording. This document covers the
model itself and the SDF-specific slotting-in points; see
[uv-less-texturing](./uv-less-texturing.md) and
[material-blending](./material-blending.md) for the authoring/storage half.

## The metallic-roughness model

The glTF 2.0 `pbrMetallicRoughness` model
([spec](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html),
[primer](https://www.khronos.org/gltf/pbr/)) — inherited from Disney's "principled"
shading approach — defines a minimal, artist-friendly channel set:

- **baseColor** — linear RGB; means different things by metalness (diffuse albedo for
  dielectrics, specular reflectance tint for metals).
- **metallic** — 0–1 blend dielectric↔conductor; physically near-binary, since almost no
  real material is genuinely "half metal."
- **roughness** — microfacet distribution width (perceptual roughness; usually squared to
  get the GGX `alpha` parameter internally).
- **normal** — perturbation of the shading normal (see
  [uv-less-texturing](./uv-less-texturing.md)'s RNM section for how triplanar-projected
  normal maps combine correctly).
- **ambient occlusion** — attenuates indirect/ambient light only, never direct light.
- **emissive** — self-illumination, added after lighting.

**Three storage/authoring strategies for an SDF scene**, since there are no vertices or
texels to hold these channels natively:
1. **Per-primitive material ID/struct** carried through the CSG tree — see
   [material-blending](./material-blending.md).
2. **Procedural functions of world-space position** — the same technique
   [uv-less-texturing](./uv-less-texturing.md) covers for color, applied to any/all of the
   PBR channels (e.g. noise-driven roughness variation).
3. **Hybrid** — a per-primitive ID selects a base material, then position-driven noise
   perturbs its parameters within that zone.

## Cook-Torrance BRDF math

```
f_r = k_d · (albedo/π) + k_s · (D·G·F) / (4·(N·V)·(N·L))
```
with `k_d + k_s ≈ 1` for energy conservation, `k_s` typically the Fresnel term itself.
Full derivation: [LearnOpenGL — PBR Theory](https://learnopengl.com/PBR/Theory).

- **D (normal distribution) — GGX/Trowbridge-Reitz**:
  `D = α² / (π·((N·H)²·(α²−1)+1)²)`. Originally Trowbridge & Reitz (1975), reintroduced
  to graphics as "GGX" by
  [Walter, Marschner, Li & Torrance, EGSR 2007](https://www.researchgate.net/publication/220853074_Microfacet_Models_for_Refraction_through_Rough_Surfaces) —
  favored over Beckmann/Blinn-Phong because its longer tails match measured real-world
  highlights.
- **G (geometry/shadowing-masking) — Smith, separable**:
  `G(N,V,L) = G1(V)·G1(L)`, with Schlick-GGX approximating each `G1`:
  `G_sub = (N·V) / ((N·V)(1−k)+k)` — using **different** `k` remappings for direct
  lighting (`k=(α+1)²/8`) vs. IBL (`k=α²/2`); an easy detail to miss.
- **F (Fresnel) — Schlick's approximation**:
  `F = F0 + (1−F0)(1−(H·V))⁵`
  ([Schlick 1994](https://onlinelibrary.wiley.com/doi/10.1111/1467-8659.1330233)). `F0`
  is ~0.04 for common dielectrics, or `baseColor` itself when `metallic=1` — this is how
  metallic-roughness materials fold into Cook-Torrance without a separate specular-color
  channel.
- **Diffuse — Lambertian**: `albedo/π`.

The full microfacet framework combining these three terms is
[Cook & Torrance, 1982](https://www.researchgate.net/publication/220184024_A_Reflectance_Model_for_Computer_Graphics);
Disney's "principled" extension
([Burley, SIGGRAPH 2012 course notes](https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf))
adds grazing-angle retro-reflective diffuse and extra artist parameters (sheen,
clearcoat, subsurface, anisotropic) on the same GGX/Smith/Schlick core — the lineage
glTF/UE4/Unity later simplified to five channels for real-time use.

## Cost in a per-pixel sphere-tracing budget

This is the real adaptation point versus a rasterizer. A raymarcher already pays a
heavy, variable per-pixel cost just to *find* the surface (often 30-150+ distance
evaluations per primary ray, more for shadow/AO/reflection rays — see
[performance-characteristics](../performance-and-production/performance-characteristics.md)),
so every added BRDF term competes directly against march budget. In practice:

- Schlick-Fresnel (one `pow5`) is essentially free and universal.
- GGX-Smith is used, but sometimes reformulated in terms of `dot(L,H)` to eliminate
  redundant dot products
  ([Filmic Worlds, "Optimizing GGX Shaders with dot(L,H)"](https://filmicworlds.com/blog/optimizing-ggx-shaders-with-dotlh/)) —
  worth doing since the shader runs at full resolution with no deferred amortization.
- Because the raymarcher already has an exact analytic-gradient normal and exact hit
  distance (see [normal-estimation](../rendering/normal-estimation.md)), it *skips*
  rasterizer-specific reconstruction hacks (screen-space normal reconstruction,
  depth-buffer AO) — but *inherits* the raymarching-specific cost of soft shadows and AO,
  which are usually the dominant per-pixel expense, not the BRDF math itself. Both are
  covered in [soft-shadows-and-ao](../rendering/soft-shadows-and-ao.md) and are
  deliberately reused there as "near-free" side effects of marches the shadow/AO code
  already performs — the same principle this document's IBL section below extends to
  reflections.

## Image-based lighting: where SDF raymarchers diverge from rasterizers

A rasterizer has no cheap way to know what's around a pixel — hence screen-space
reflections (which fail outside the visible frustum) or static cubemaps. An SDF
raymarcher can trace a genuine second ray from the hit point in the reflection direction
and sphere-trace it through the *actual* scene, producing correct reflections (including
off-screen and self-reflections) at the cost of one more full march — a widely cited
selling point of raymarched rendering. (This project already implements exactly this —
see the raymarch shader's reflection pass, added for the ground/ring materials.)

**Split-sum, adapted to "trace when you can afford it."** The standard real-time IBL
split ([Karis, "Real Shading in Unreal Engine 4," SIGGRAPH 2013](https://blog.selfshadow.com/publications/s2013-shading-course/karis/s2013_pbs_epic_slides.pdf))
separates specular IBL into a prefiltered environment mip-chain (roughness → mip level)
plus a `(NoV, roughness)` environment-BRDF lookup texture. A raymarcher can use this
exactly as a rasterizer would, but gets to **upgrade** the prefiltered-cubemap term to a
real trace opportunistically — e.g. trace real reflections for near/important
mirror-like geometry, fall back to a prefiltered cubemap sample for distant or very rough
surfaces where a blurred lookup is visually indistinguishable from a jittered
multi-sample trace. This "trace what you can afford, sample the rest" split is the
central way IBL practice in SDF renderers differs from pure-rasterizer practice.

**Diffuse/irradiance term** is almost never worth ray-tracing (needs a full hemisphere
integral) — handled the same way in both worlds, via a precomputed irradiance map or a
compact spherical-harmonics representation of the environment, sampled once per shading
point using just the surface normal
([LearnOpenGL — Diffuse Irradiance](https://learnopengl.com/PBR/IBL/Diffuse-irradiance),
worked example:
[Shadertoy — Reflection→Irradiance](https://www.shadertoy.com/view/WddXDS)).

Foundational shipped-engine treatment of prefiltered-cubemap ambient specular:
[Lagarde, "Adopting a Physically Based Shading Model"](https://seblagarde.wordpress.com/2011/08/17/hello-world/)
and companion
["Feeding a Physically Based Shading Model"](https://seblagarde.wordpress.com/2011/08/17/feeding-a-physical-based-lighting-mode/).

## Subsurface scattering and translucency

**Wrap lighting** — a non-physical bias of the `N·L` term around the terminator, faking
shallow light transport with zero extra cost (just a remapped dot product). Traced to
Barré-Brisebois & Bouchard, GDC 2011; modern write-ups at
[therealmjp, "An Introduction to Real-Time Subsurface Scattering"](https://therealmjp.github.io/posts/sss-intro/)
and
[Alan Zucconi, "Fast Subsurface Scattering in Unity"](https://www.alanzucconi.com/2017/08/30/fast-subsurface-scattering-1/).
The default choice when nothing march-based is affordable.

**Thickness-via-second-march — the genuinely SDF-native technique.** Since the renderer
already has a distance field and can march from any point in any direction, fire a short
secondary ray from the shading point toward the light (or through the surface) and use
distance-to-exit as a local thickness estimate, attenuating a transmission term by a
Beer-Lambert-style falloff. This directly reuses the sphere-tracing infrastructure
already present for shadow rays (see
[soft-shadows-and-ao](../rendering/soft-shadows-and-ao.md)) — the idiomatic SDF answer to
translucency, needing zero new infrastructure beyond a shadow-ray marcher that already
exists. The rigorous academic treatment connecting sphere tracing specifically to
multiple-scattering SSS is
[Rovinski et al., "Learning Multiple-Scattering Solutions for Sphere-Tracing of Volumetric Subsurface Effects" (2020)](https://arxiv.org/pdf/2011.03082),
which learns a neural correction to a marched single-scattering estimate.

**Spherical Gaussian SSS** — a higher-quality real-time-feasible middle ground fitting
SSS diffusion profiles with sums of spherical Gaussians, evaluable analytically per
sample with no precomputed screen-space blur pass (unlike classic screen-space SSS
approaches) — a natural fit for a per-pixel raymarcher:
[therealmjp, "Approximating Subsurface Scattering With Spherical Gaussians"](https://therealmjp.github.io/posts/sss-sg/).

## Where current research effort actually goes

No 2024-2026 paper proposing a *new* per-pixel BRDF specifically for sphere-tracing
raymarchers turned up in research for this document — the field treats "how do you shade
a raymarched hit point" as solved (apply standard Cook-Torrance/glTF metallic-roughness,
identical to any other ray hit). Current research energy concentrates instead on SDF
*geometry/GI* representation (neural SDFs, radiance caching, hardware-accelerated sphere
tracing — see [state-of-the-art](../state-of-the-art/INDEX.md)) rather than SDF-specific
material math. This is itself a useful fact: don't go looking for an "SDF BRDF" — the
raymarching-specific engineering effort is entirely in the cheap *lighting queries*
around a completely ordinary BRDF.

## When to dive in

- Choosing a shading model for a raymarched surface → metallic-roughness Cook-Torrance,
  unmodified, is the correct default; there is no raymarcher-specific BRDF to look for.
- Budgeting per-pixel cost → the BRDF itself is rarely the bottleneck; shadow/AO/
  reflection march cost dominates (see
  [performance-characteristics](../performance-and-production/performance-characteristics.md)).
- Adding reflections → trace a real secondary ray when affordable (this project already
  does, see the raymarch shader), fall back to a prefiltered-cubemap/split-sum lookup for
  rough or distant surfaces.
- Adding translucency/SSS to an SDF material → start with wrap lighting (free); add a
  second short march toward/through the light for a genuine thickness estimate if budget
  allows, reusing existing shadow-ray infrastructure.

## Related
- [Normal estimation](../rendering/normal-estimation.md) — prerequisite: the normal the BRDF consumes.
- [Soft shadows and AO](../rendering/soft-shadows-and-ao.md) — deeper: the lighting queries that are the real SDF-specific cost.
- [Material blending](./material-blending.md) — prerequisite: where per-point material parameters come from.
- [Lighting and shadows in Bevy](../../bevy-rendering/pbr-and-lighting/lighting-and-shadows.md) — contrast: the same BRDF in Bevy's rasterizer, which migera now uses for characters.
- [Render-scale subsumes half-res reflections](../../hybrid-architecture/performance-findings/render-scale-subsumes-half-res-reflections.md) — applies: measured cost of traced reflection passes in `src/hybrid`.
