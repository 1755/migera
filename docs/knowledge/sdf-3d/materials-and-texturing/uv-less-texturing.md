---
title: "Texturing without UVs: triplanar, biplanar, and procedural noise"
description: Establishes that an SDF surface is textured only from hit point and normal — triplanar and cheaper biplanar projection, world vs. object space, RNM normal-map blending, fBM noise and domain warping, field-derived cavity detail. Read before texturing an SDF surface or when a triplanar map seams or flattens.
type: concept
status: current
tags:
  - sdf
  - materials
  - performance
  - correctness
updated: 2026-08-16
aliases:
  - triplanar mapping
  - biplanar mapping
  - fBM
  - domain warping
  - reoriented normal mapping
---

# Texturing without UVs: triplanar, biplanar, and procedural noise

Contents: [Why distinct](#why-this-is-a-distinct-problem-for-sdfs) · [Triplanar](#triplanar-mapping) ·
[World vs. object space](#world-space-vs-object-space-projection) ·
[Biplanar](#biplanar-mapping-the-cheaper-alternative) ·
[Normal mapping](#triplanar-normal-mapping-why-linear-blending-breaks) ·
[Procedural noise](#procedural-noise-based-texturing) ·
[Field-derived detail](#distance-field-derived-detail-cavity-curvature-and-material-id) · [Related](#related)

## Why this is a distinct problem for SDFs

An SDF has no vertices, no UV channel, and no mesh to unwrap — a raymarcher's only
per-pixel data is the hit point `p` (world- or object-space) and the surface normal `n̂`
(see [normal-estimation](../rendering/normal-estimation.md)), both computed fresh every
frame with no persistent per-texel storage. Every technique below is a way to turn just
`(p, n̂)` into a texture lookup or procedural color/material value with no UV
parameterization at all.

## Triplanar mapping

Project a 2D texture three times from world- or object-space position — once each onto
the YZ, XZ, and XY planes — then blend the three samples by surface-normal alignment.
Originated in Ryan Geiss's GPU Gems 3 chapter on procedural terrain
([Generating Complex Procedural Terrains Using the GPU](https://developer.nvidia.com/gpugems/gpugems3/part-i-geometry/chapter-1-generating-complex-procedural-terrains-using-gpu)),
motivated by the severe stretching a single planar projection produces wherever the
surface faces away from that projection's axis.

**Blend weights** start from `abs(n̂)` and are sharpened before normalizing to sum to 1 —
Geiss's original linear form was `max(abs(n̂) - 0.2, 0) * 7.0`; the now-more-common form
raises to a power instead: `w = pow(abs(n̂), k) / dot(pow(abs(n̂), k), vec3(1))`, with `k`
in the 4–8 range directly controlling transition sharpness
([Ryan DowlingSoka, "Triplanar, Dithered Triplanar, and Biplanar Mapping in Unreal"](https://ryandowlingsoka.com/unreal/triplanar-dither-biplanar/)
— note `pow()` returns NaN for a negative base in HLSL, so `abs()` must precede it).

**Where the seam shows up.** Near a cube-corner normal direction (`(±0.577, ±0.577,
±0.577)`) all three weights are roughly equal, producing a visibly "muddy" overlap of
three differently-oriented projections. A sharper power narrows this zone at the cost of
a harder transition — there's no seam-free free lunch, only a tunable tradeoff between
blend-region width and blend-region muddiness.

## World-space vs. object-space projection

The projection axes can be fixed to the world, or attached to each primitive's own local
frame (computed by transforming `p` through the primitive's inverse transform — the same
inverse transform an SDF primitive already applies to evaluate its own distance function,
see [primitives-and-operators](../primitives-and-operators/INDEX.md), so this is close to
free to add).

- **Object-space**: the texture stays glued to the primitive under translation/rotation/
  animation — required for anything that moves, since world-space projection causes
  visible "texture swimming" as a moving surface's world position changes under a fixed
  projection.
- **World-space**: simpler (no inverse transform needed) and guarantees visual continuity
  *across* multiple static primitives/objects sharing the same region of world space (a
  ground made of several blended primitives should look like one continuous material, not
  per-primitive-varying) — but is wrong for anything that moves.

Rule of thumb: object-space for anything animated/instanced, world-space only for static,
single-instance environment geometry. Inigo Quilez solved this exact problem for a
deforming implicit-surface character (his 2014 "Fish" Shadertoy piece) by computing, for
each surface point, where it originated in the character's *rest pose* via an invertible
deformation function, then texturing in that rest-pose space — a custom object-space
parameterization built specifically to prevent swimming under animation
([iquilezles.org/articles/raymarchingdf](https://iquilezles.org/articles/raymarchingdf/)).

## Biplanar mapping: the cheaper alternative

Inigo Quilez's biplanar mapping cuts the fetch count from 3 to 2: rather than blending
all three projections, pick the **major** axis (largest `|n̂|` component) and the
**median** axis (`3 - major - minor`, since axis indices 0/1/2 sum to 3), sampling only
those two projections
([iquilezles.org/articles/biplanar](https://iquilezles.org/articles/biplanar/)). Because
axis selection is a discontinuous branch on normal direction, naive `texture()` calls at
the selection boundary break the GPU's automatic derivative-based mip selection — the
technique instead computes screen-space derivatives *before* branching and passes them
explicitly via `textureGrad()`. Quilez frames the tradeoff plainly: biplanar trades
"three texture fetches for additional arithmetic," a good deal specifically when texture
bandwidth (not ALU) is the bottleneck, which is the common case for a raymarcher already
spending its ALU budget on sphere tracing rather than texture sampling. Biplanar has its
own singularities near axis-pair-flip boundaries and is not a strict improvement in every
scene.

A third, TAA-dependent variant — **dithered triplanar** — picks a single axis
stochastically per pixel per frame (weighted by the triplanar blend weights) and relies
on temporal accumulation to resolve the noise into a smooth blend over several frames,
cutting to 1 fetch per layer at the cost of depending on TAA already running
([DowlingSoka, op. cit.](https://ryandowlingsoka.com/unreal/triplanar-dither-biplanar/)).
Not applicable to a renderer without temporal accumulation.

## Triplanar normal mapping: why linear blending breaks

A tangent-space normal map encodes a perturbation relative to a *specific* local tangent
frame — triplanar has three different tangent frames (one per projection axis), and
linearly blending the three sampled tangent-space normals with the same weights used for
albedo produces visibly wrong, flattened results: averaging vector data like color
shrinks the combined vector toward zero-length
([Barré-Brisebois & Hill, "Blending in Detail"](https://blog.selfshadow.com/publications/blending-in-detail/)).
Three fixes, in order of quality and (surprisingly) *decreasing* cost relative to a naive
per-axis-tangent-basis implementation
([bgolus, "Normal Mapping for a Triplanar Shader"](https://bgolus.medium.com/normal-mapping-for-a-triplanar-shader-10bf39dca05a)):

1. **UDN blending** — add each tangent-space sample's XY directly onto the corresponding
   world/geometric normal's components, sidestepping per-axis tangent bases entirely.
   Cheapest; visibly flattens detail at grazing angles beyond ~45° from an axis.
2. **Whiteout blending** — UDN plus taking `abs()` of the Z component before combining,
   fixing the grazing-angle flattening at negligible extra cost. Recommended default.
3. **Reoriented Normal Mapping (RNM)** — rotates the detail normal onto the frame implied
   by the base normal (shortest-arc rotation), retaining the most detail under strong
   blending:
   ```glsl
   float3 rnmBlendUnpacked(float3 n1, float3 n2) {
       n1 += float3(0, 0, 1);
       n2 *= float3(-1, -1, 1);
       return n1 * dot(n1, n2) / n1.z - n2;
   }
   ```
   Highest quality, still cheap relative to a texture fetch.

The same power-based weight sharpening from the albedo blend (above) applies to normal
blending too, and for the same reason: it narrows the artifact-prone overlap zone near
cube-corner normal directions.

## Procedural noise-based texturing

Because a raymarcher already has the exact world-space hit point `p` as a free byproduct
of sphere tracing, evaluating a 3D noise function at `p` costs zero texture memory and
needs no seam handling — 3D noise is defined everywhere in space, with no projection or
wrapping to get wrong. This is the natural, idiomatic SDF-renderer answer to "texture
without a texture," and is the basis of two decades of Inigo Quilez's raymarched
Shadertoy work
([iquilezles.org/articles/raymarchingdf](https://iquilezles.org/articles/raymarchingdf/)).

**Domain warping** is the signature technique for turning plain fractal Brownian motion
(fBM — layered/summed noise octaves at increasing frequency, decreasing amplitude) into
organic, swirling, marbled patterns: instead of evaluating `f(p)`, evaluate
`f(p + h(p))` where `h` is itself noise-derived, and nest the warp:
```
q = fbm(p)
r = fbm(p + 4*q)
result = fbm(p + 4*r)
```
([iquilezles.org/articles/warp](https://iquilezles.org/articles/warp/), tracing the idea
back to Ken Perlin's original 1985 procedural marble). Each nested layer roughly triples
the number of noise evaluations, so real-time budgets typically cap warp depth at 2–3
layers and octave count at 2–4 for anything evaluated inside a shadow/AO loop (see
[soft-shadows-and-ao](../rendering/soft-shadows-and-ao.md)), reserving deeper
domain-warped fBM for the one-time final-hit material evaluation.

The same noise field can drive more than base color: roughness/metalness variation,
normal perturbation (via the noise field's own analytic or finite-difference gradient,
added to the geometric normal), and — with real caveats, covered in
[material-blending](./material-blending.md) — even the distance field itself for surface
roughening.

**Analytic color palettes.** Inigo Quilez's cosine-based palette function
`color(t) = a + b·cos(2π(c·t + d))` generates a smooth, continuous color gradient from
four small vector constants — four lines of shader code replacing a gradient texture
lookup entirely ([iquilezles.org/articles/palettes](https://iquilezles.org/articles/palettes/)).

**Anisotropic noise (2025).** Steerable Perlin Noise (Rice & Jushchyshyn, SIGGRAPH 2025,
Walt Disney Animation Studios) adds controllable directional bias to classic Perlin noise
at little extra evaluation cost over standard Perlin
([disneyanimation.com/publications/steerable-perlin-noise](https://disneyanimation.com/publications/steerable-perlin-noise/),
open-source multi-engine implementation including a Shadertoy demo at
[github.com/jakericedesigns/SteerablePerlinNoise](https://github.com/jakericedesigns/SteerablePerlinNoise)) —
directly usable at an SDF hit point exactly like ordinary Perlin/fBM, opening directional
patterns (wood grain, brushed metal, wind-aligned terrain) that previously needed a
costlier noise basis or a separate post-hoc directional warp.

## Distance-field-derived detail: cavity, curvature, and material ID

Because normal estimation already extends the SDF evaluation with several extra nearby
samples (see [normal-estimation](../rendering/normal-estimation.md)), the same
finite-difference machinery generalizes to approximate local **curvature** — how much the
gradient/normal field bends — from additional nearby samples. This is the SDF-native
analog of a baked mesh curvature or cavity map used to drive edge-wear/crevice-dirt masks
in traditional PBR authoring (see
[Polycount: Cavity map](https://wiki.polycount.com/wiki/Cavity_map) /
[Curvature map](https://wiki.polycount.com/wiki/Curvature_map)) — except computed on the
fly with no bake step, a genuine advantage of SDF-native pipelines over mesh-based cavity
baking.

**Ambient occlusion is the most widely used instance of this pattern.** Sampling the
distance field along the surface normal at a few increasing offsets and comparing the
actual value against the offset distance approximates local occlusion — see
[soft-shadows-and-ao](../rendering/soft-shadows-and-ao.md) for the concrete technique;
it's included here because it's the clearest existing proof that "sample the field near
the surface" is a cheap, reusable pattern that generalizes past lighting into
cavity/crevice-style material detail.

**Material ID for procedural CSG scenes.** The standard way to assign different materials
to different primitives in a CSG-composed scene is to carry a material ID (or full
material struct) alongside the distance value through every composition operation — see
[material-blending](./material-blending.md) for the concrete mechanism, which reuses the
same smooth-minimum blend weight already computed for geometric blending.

## When to dive in

- Texturing any SDF surface with a scanned/asset-based texture set → triplanar is the
  required bridge technique; start there, move to biplanar only if fetch count is a
  measured bottleneck.
- The surface is animated, instanced, or otherwise moving → use object-space projection,
  not world-space, or expect visible texture swimming.
- Adding organic surface color/pattern with zero texture memory → procedural noise +
  domain warping is the idiomatic SDF-renderer technique; start with 2-4 fBM octaves and
  add warp layers only as needed.
- A triplanar normal map looks flattened or washed out → check whether blending is linear
  (wrong) vs. whiteout/RNM (correct); see the normal-mapping section above.
- Wanting per-primitive material variation in a CSG scene → see
  [material-blending](./material-blending.md) for material-ID propagation through the
  scene graph.

## Related
- [Material blending](./material-blending.md) — deeper: combining several textured materials on one surface.
- [Procedural vs. asset-based authoring](./procedural-vs-asset-authoring.md) — deeper: when to use noise vs. a scanned set through triplanar.
- [Soft shadows and AO](../rendering/soft-shadows-and-ao.md) — example: the same "sample the field near the surface" pattern behind cavity detail.
- [Integrating a baked mesh SDF into this project's raymarcher](../mesh-conversion/hybrid-baked-and-procedural-scenes.md) — applies: re-applying a baked asset's textures via triplanar.
