---
title: Blending and mixing multiple materials
description: Establishes how several materials share one CSG-composed SDF surface — material ID through the tree, reusing smin's blend weight h to mix materials at the seam, height/splat-map blending, triplanar × N-layer cost, noise masks. Read when assigning per-primitive materials or a blended seam shows a colour jump.
type: concept
status: current
tags:
  - sdf
  - materials
  - csg
  - performance
updated: 2026-08-16
aliases:
  - material ID
  - height blend
  - splat map
  - smin material weight
---

# Blending and mixing multiple materials

Contents: [Material ID](#material-id-propagated-through-the-csg-tree) ·
[Reusing smin's weight](#reusing-smins-own-blend-weight-for-smooth-material-transitions) ·
[Height blending](#height-based--depth-based-blending) ·
[Triplanar × N layers](#combining-triplanar-with-n-material-layering) ·
[Noise masks](#noise-driven-procedural-material-masks) ·
[Cost ceiling](#cost-and-complexity-ceiling) · [Related](#related)

## Material ID propagated through the CSG tree

The standard way to assign different materials to different primitives in a
CSG-composed SDF scene is to carry a material ID (or a small material struct) *alongside*
the distance value through every composition operation — the scene function returns
`(distance, materialID)` rather than a bare float, and every boolean/blend operator is
overloaded to propagate both fields, not just the winning distance. A union keeps the
material of whichever branch produced the smaller distance; a hard boolean is a one-line
extension of the same `min`/`max` operators already documented in
[combination-operators](../primitives-and-operators/combination-operators.md):
```glsl
vec2 opU(vec2 d1, vec2 d2) { return (d1.x < d2.x) ? d1 : d2; }
```
This "material rides alongside distance" pattern is how procedural CSG scenes (shadertoy
demos, and the production hybrid engines discussed below) assign per-primitive materials
with no texture/UV system at all — the resolved ID then selects which triplanar/biplanar
texture set or which procedural-noise material function to evaluate at the final hit
point (see [uv-less-texturing](./uv-less-texturing.md)).

## Reusing smin's own blend weight for smooth material transitions

This is the technique most specific to — and most naturally available in — an SDF scene,
and it's a direct extension of machinery this project's own
[combination-operators](../primitives-and-operators/combination-operators.md) doc already
covers. The polynomial smooth-minimum already computes an interpolation weight `h`
(0 = "fully shape A," 1 = "fully shape B") as an intermediate value on the way to the
blended distance:
```glsl
float smin(float a, float b, float k) {
    float h = clamp(0.5 + 0.5*(b-a)/k, 0.0, 1.0);
    return mix(b, a, h) - k*h*(1.0-h);
}
```
That same `h` is directly reusable to `mix()` the two shapes' material properties
(albedo, roughness, metallic — anything the material struct carries) across the same
seam, guaranteeing the color/material transition and the geometric blend transition stay
spatially coincident — no "smooth geometry but hard-edged paint" seam
([Inigo Quilez, "Smooth Minimum"](https://iquilezles.org/articles/smin/), whose own
article gives this exact use — "mixing the red and a blue materials based on this
blending factor" — as the motivating example for returning `h` alongside the distance).

**Order-independence problem with 3+ primitives.** Chaining pairwise `smin` calls across
more than two primitives makes the *color* blend visibly order-dependent (whichever
primitive folds in first biases the result), even though the resulting *distance field*
stays close to order-independent for reasonable `k`. Quilez's fix uses the exponential
smin family (already in
[combination-operators](../primitives-and-operators/combination-operators.md) as the
costlier alternative to the polynomial form) to get a genuinely order-independent
softmin-style weight:
```glsl
vec2 smin(in float a, in float b, in float k) {
    float f1 = exp2(-k*a);
    float f2 = exp2(-k*b);
    return vec2(-log2(f1+f2)/k, f2);
}
```
The second component is a normalized weight stable regardless of combination order,
unlike the polynomial `h` above. Only reach for this when actually blending 3+
primitives at a shared seam with runtime-varying order — otherwise the cheaper
polynomial `h` suffices for material blending exactly as it does for geometry, and the
extra `exp2`/`log2` pair per evaluation
(see [combination-operators](../primitives-and-operators/combination-operators.md)'s own
cost note) is wasted.

**Decoupling geometry blend radius from material blend radius.** Once distance and
material are computed as a pair, nothing forces them to share the same `k`. Using a
small `k_geo` for the geometric fillet (keeping the silhouette tight) and a larger
`k_mat` for the material transition is a deliberate, common divergence — it mimics how
real-world material transitions (rust creeping past a metal/organic seam, moss spreading
beyond a rock's exact edge) extend visually further than the geometric boundary itself.

**C¹ vs. C² continuity matters less here than for geometry.** The C¹-vs-C² distinction
already documented in
[combination-operators](../primitives-and-operators/combination-operators.md) (faceting
under lighting from a discontinuous second derivative) is far less perceptible for
material *color* blending than for geometric normals — a reasonable simplification is to
use the cheap C¹ polynomial weight for material mixing even in a scene using the costlier
C² variant for geometry, since the two consumers have different sensitivity to derivative
discontinuities.

## Height-based / depth-based blending

Where smin-weight reuse blends materials *at a CSG seam*, height blending blends
materials *across a broader region* driven by a per-material height/noise field — the
standard terrain-shader technique for making one material (rock) visibly "poke through"
another (dirt) at an irregular, interlocking boundary instead of a flat, uniform
cross-fade. Core form:
```
lowerLimit = threshold - blendWidth/2
upperLimit = threshold + blendWidth/2
blend = smoothstep(lowerLimit, upperLimit, heightA - heightB)
```
([Cédric Van Huffelen, "Heightmap Blending"](https://www.shaderic.com/tutorials/HeightmapBlending.html)).
Unlike a flat linear cross-fade (constant blend rate everywhere), height blending treats
each material's height/noise texture as a pseudo-elevation field and lets whichever
material's height happens to peak locally "win" first — producing the irregular,
erosion-like boundary that reads as physically plausible rather than painted. Unreal
Engine exposes this directly as the `LB Height Blend` mode of `Landscape Layer Blend`
([Epic Dev Docs](https://dev.epicgames.com/documentation/unreal-engine/landscape-material-expressions-in-unreal-engine)).

For an SDF scene with no terrain heightmap, the "height" input is naturally a procedural
noise field evaluated at the hit point (see
[uv-less-texturing](./uv-less-texturing.md)'s noise section) rather than an authored
texture — the blend math is identical either way.

## Splat-map / weight-map blending, generalized to procedural fields

Classic texture splatting stores a per-material weight (commonly packed one-per-RGBA-
channel, weights summing to 1) and computes `color = Σ weight_i · materialColor_i`
([Wikipedia — Texture splatting](https://en.wikipedia.org/wiki/Texture_splatting)). For a
procedural/SDF scene with no mesh to paint a splatmap onto, the same weight-vector idea
generalizes directly: the "weight map" becomes a **procedural field** evaluated at the
query point instead of a baked texture —
`weights = f(worldPos, noise(worldPos), heightAboveSurface, slope, distanceToNearestSeam)`,
L1-normalized so weights sum to 1 before being consumed by shading. **Four simultaneous
materials per point** is a recurring practical ceiling across both the classic (RGBA
channel budget) and procedural/voxel case — a Surface Nets voxel-SDF texturing case study
uses the identical 4-material convention, deriving weights by summing/normalizing the
surrounding voxel cell corners' material weights
([DreamCat Games, "Smooth Voxel Mapping"](https://bonsairobo.medium.com/smooth-voxel-mapping-a-technical-deep-dive-on-real-time-surface-nets-and-texturing-ef06d0f8ca14)).
Beyond 4 materials, production systems don't blend more per pixel — they decouple
material *count* from per-pixel cost via virtual/streamed texturing instead, keeping only
a bounded working set resident regardless of total authored layer count.

## Combining triplanar with N-material layering

Triplanar's "which world axis" blend (see
[uv-less-texturing](./uv-less-texturing.md)) and layering's "which material" blend are
independent weight computations that compose without excessive complexity if kept
orthogonal: compute `layerColor_i = triplanar(pos, normal, materialTextures_i)` for each
active layer, *then* combine layers with `finalColor = Σ weight_i · layerColor_i`. The
naive cost multiplies out fast — 4 materials × 3 triplanar axes × 5 texture maps per
material (albedo/normal/roughness/AO/height) = 60 fetches per fragment in the Surface
Nets case study above, reported as "readily handled" by modern GPUs but with mipmapping/
reduced-filter fallbacks recommended for lower-end targets. Biplanar mapping (see
[uv-less-texturing](./uv-less-texturing.md)) cuts this by a third when fetch count is a
measured bottleneck.

## Noise-driven procedural material masks

Domain-warped noise (see [uv-less-texturing](./uv-less-texturing.md)) generalizes past
color-ramp generation into a blend mask between two or more *different* materials — since
the mask has no relationship to the underlying geometry's silhouette or CSG topology,
it avoids the "painted-on," UV-seam-following look that geometry-derived masks (height
above surface, distance to CSG seam) can have. This is the standard choice for material
pairs whose real-world boundary is driven by a process uncorrelated with the solid's
shape — rock/moss, sand/rock, metal/rust. Production case study: Hello Games' No Man's
Sky uses domain-warped noise fields to drive planetary terrain material blending and
cave/overhang shaping
([GDC 2017, "Continuous World Generation in No Man's Sky"](https://www.gdcvault.com/play/1024265/Continuous-World-Generation-in-No)),
and has continued refining the technique post-launch (later patches moved terrain/object
fade transitions to blue noise for smoother spatial results).

**A rigorous alternative for avoiding "muddy average" artifacts.** Naively blending
several noise-offset copies of a texture (a common way to defeat visible tiling) tends to
wash blended regions toward the average/gray color — a known failure mode of plain linear
blending. Deliot & Heitz's histogram-preserving blend operator (building on
[Heitz & Neyret, HPG 2018](https://eheitzresearch.wordpress.com/738-2/), shipped in
Unity's Shader Graph) synthesizes large, non-repeating textures from a small example tile
by splatting randomly offset/rotated copies and blending with an operator specifically
designed to preserve the color histogram rather than average it out. Worth reaching for
specifically when several procedural or noise-masked material layers stacked together
start reading as flat/gray rather than richly varied.

## Cost and complexity ceiling

- **4 simultaneous materials per point** is the recurring practical ceiling (see splat-
  map section above) — independent of representation (painted texture, voxel weight
  vector, or procedural field).
- **Biplanar over triplanar** when fetch count dominates (1/3 fewer fetches, see
  [uv-less-texturing](./uv-less-texturing.md)).
- **Polynomial over exponential smin** for material blending unless genuine 3+-primitive
  order-independence is required (see above) — the same guidance
  [combination-operators](../primitives-and-operators/combination-operators.md) already
  gives for geometry blending applies identically to material blending, since it's the
  same cost tradeoff paid a second time.
- **Spatial culling of candidate materials** — the same acceleration structures that
  cluster/cull SDF primitives for faster distance queries at large scene scale (see
  [sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md))
  generalize to culling which materials are even candidates for blending at a query
  point, avoiding evaluating (e.g. triplanar-sampling) materials with effectively zero
  weight there.

## When to dive in

- Assigning different materials to different CSG primitives → material ID propagated
  through the scene graph (top section) is the baseline mechanism everything else builds
  on.
- A smooth-unioned seam looks geometrically blended but the color/material jumps abruptly
  at the same seam → reuse the smin blend weight `h` for the material mix, not just the
  distance.
- Blending 3+ primitives at a shared seam and seeing order-dependent color artifacts →
  switch to the exponential smin's normalized weight for material blending specifically.
- Texturing a large ground/terrain-like SDF surface with multiple materials → height
  blending (irregular, physically-plausible boundaries) over flat linear cross-fade.
- Combining triplanar texturing with multi-material layering → keep the two blend-weight
  computations orthogonal (axis blend inside each layer, material blend across layers'
  already-resolved colors) to keep the fetch-count multiplication tractable.
- Several stacked procedural/noise-masked materials look flat or washed-out gray →
  consider a histogram-preserving blend operator instead of plain linear mixing.

## Related
- [Combining SDFs](../primitives-and-operators/combination-operators.md) — prerequisite: the smin whose weight `h` is reused here.
- [Texturing without UVs](./uv-less-texturing.md) — prerequisite: the triplanar layer each blended material samples.
- [PBR shading model](./pbr-shading-model.md) — applies: shades the blended material parameters.
- [Sparse and hierarchical structures](../performance-and-production/sparse-and-hierarchical-structures.md) — deeper: culling candidate materials at scale.
