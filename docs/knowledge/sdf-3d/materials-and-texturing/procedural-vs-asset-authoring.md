---
title: Procedural vs. asset-based material authoring
description: Weighs procedural shader-math materials (resolution-independent, no texture memory, harder to art-direct) against scanned texture sets (match references, need triplanar, cost bandwidth), hybrid pipelines, hypertexture risk to the Lipschitz bound, and AI texture tools. Read when deciding how to author a material.
type: research
status: current
tags:
  - sdf
  - materials
  - assets
  - performance
  - state-of-the-art
updated: 2026-08-16
aliases:
  - photogrammetry
  - hypertexture
  - generative AI textures
  - texture tiling
---

# Procedural vs. asset-based material authoring

Contents: [Two traditions](#the-two-traditions-and-why-sdfs-favor-one-of-them-by-default) ·
[Procedural](#procedural-material-generation) ·
[Asset-based](#asset-based--photogrammetry-scanned-texture-sets) ·
[Hybrid pipelines](#hybrid-pipelines-procedural-macro-variation-over-asset-based-micro-detail) ·
[Resolution independence](#resolution-independence-the-direct-textual-analog-of-what-sdfs-already-give-geometrically) ·
[AI tools](#current-2024-2026-generative-ai-texture-tools) ·
[Cost crossover](#costperformance-crossover) · [Related](#related)

## The two traditions, and why SDFs favor one of them by default

**Asset-based** materials are baked texture sets — typically base color, normal,
roughness/metallic, ambient occlusion, height — authored in tools like Substance Painter/
Designer or captured via photogrammetry and distributed by libraries like
[Poly Haven](https://polyhaven.com/textures) and [ambientCG](https://ambientcg.com/).
**Procedural** materials are generated as shader math (noise, domain warping, analytic
patterns) evaluated per-pixel with no stored texture at all.

SDF scenes have no UV space to bake asset-based textures into — every asset-based texture
applied to an SDF surface must go through a UV-less bridge technique (triplanar/biplanar
mapping, see [uv-less-texturing](./uv-less-texturing.md)) to be usable at all. Procedural
materials, being pure functions of the already-available world-space hit point, need no
such bridge — this is the structural reason procedural texturing reads as the more
"native" fit for SDF rendering, independent of any quality argument.

## Procedural material generation

**Substance Designer's node-graph model** builds materials from a graph of noise
generators, transforms (warp, slope blur, directional blur), and combinators (blend,
height-to-normal), producing a full PBR set non-destructively and resolution-independent
at authoring time
([Adobe Substance 3D Designer docs](https://experienceleague.adobe.com/en/docs/substance-3d-designer/using/home)).
It does not export the graph directly as shader source, though — the standard portable
bridge for procedural node graphs is
[MaterialX](https://materialx.org/) (an Academy Software Foundation open standard,
including pattern-generation nodes, consumed by multiple DCC tools/renderers). What
actually translates to a hand-written raymarcher shader is the *vocabulary* of
operations (noise, warp, blend, slope-blur, height-to-normal) — the same operation set
Shadertoy-style procedural shaders use — not a literal graph export.

**Pure-shader procedural technique** is the direct SDF-native analog, covered in depth in
[uv-less-texturing](./uv-less-texturing.md): fBM, domain warping, analytic color
palettes, and Voronoi/cellular variants, nearly all traceable to Inigo Quilez's
[iquilezles.org/articles](https://iquilezles.org/articles/) and, further back, to Ken
Perlin's 1985 solid-texture concept
([Perlin, "An Image Synthesizer," SIGGRAPH 1985](https://www.cs.drexel.edu/~deb39/Classes/Papers/p287-perlin.pdf)) —
a 3D procedural function sampled at surface points, which is exactly the model an SDF
raymarcher wants since there's no UV space to author into in the first place.

**What makes a pattern genuinely "free."** A pattern is free only if evaluated as pure
math from the sample point with zero memory access — a checkerboard, dot grid, or
fBM-based marble computed in-shader. Some such patterns (checkers, grids, XOR fractals)
even admit *analytic* antialiasing via screen-space derivatives (`dpdx`/`dpdy`) instead
of supersampling, giving perfect antialiasing at zero extra memory or texture-unit cost
([iquilezles.org/articles/filterableprocedurals](https://iquilezles.org/articles/filterableprocedurals/)).
The moment a technique needs a stored asset as an ingredient (a low-frequency index
texture for variation, a hypertexture volume for cache-friendliness), it's a hybrid, not
pure-procedural — trading ALU for bandwidth.

## Asset-based / photogrammetry-scanned texture sets

The standard convention (Poly Haven, ambientCG, Quixel Megascans) is base color, normal
(OpenGL convention), roughness, metallic (or specular/glossiness), AO, and height/
displacement. Poly Haven's technical standard requires the diffuse/albedo map to be
**color-calibrated against a Macbeth color chart** during capture — a detail that only
matters because these are measured from a physical sample, not authored freehand
([Poly Haven technical standards](https://docs.polyhaven.com/en/technical-standards/textures)).

Poly Haven states its philosophy explicitly, and it's worth citing directly as the
principled counterpoint to procedural generation: *"we spend weeks on every photoscanned
texture instead of pumping out dozens of procedural textures"* and describes its mission
as providing content *"based on photographic data and real life"*
([Poly Haven, "AI and Poly Haven"](https://blog.polyhaven.com/ai-and-poly-haven/)). The
two major CC0 libraries (Poly Haven ~1,700+ assets, ambientCG 2,000+ materials) exist
specifically because procedural generation doesn't yet reliably match the fine-grained
irregularity of real scanned surfaces — a point their own maintainers treat as a
principle, not just a current technical limitation.

**Triplanar mapping is the required bridge** for applying any of these scanned sets to a
UV-less SDF surface — see [uv-less-texturing](./uv-less-texturing.md) for the mechanism.
Without it, scanned PBR texture sets are simply inapplicable to implicit geometry.

## Hybrid pipelines: procedural macro-variation over asset-based micro-detail

The dominant real production pattern mixes both: a tiled/scanned texture supplies
high-frequency micro-detail, and procedural noise supplies low-frequency macro-variation
layered on top specifically to break up the tiling that would otherwise be obvious on a
large triplanar-mapped surface.

- **UE4/5 "macro variation"**: sample the same tileable scanned texture again at a much
  larger tiling scale, blend that low-frequency layer over the base to disrupt the
  visible repeat pattern at distance
  ([World of Level Design](https://www.worldofleveldesign.com/categories/ue4/landscape-macro-tiling-variation.php)).
- **IQ's texture-repetition-breakup techniques** — three escalating approaches, all
  hybrids of procedural randomization driving selection from a stored/baked tile: (1)
  per-tile random offset/rotation via a hash function; (2) Voronoi-blended random copies;
  (3) a low-frequency procedural index texture selecting between pre-offset tile variants
  ([iquilezles.org/articles/texturerepetition](https://iquilezles.org/articles/texturerepetition/)).
- **Hypertexturing/domain-warping the SDF itself** — the SDF-specific hybrid case: rather
  than combining noise with a *texture*, combine noise directly with the *distance
  field* to add surface roughness that shows up in silhouette, not just shading. Three
  documented approaches trade off differently: adding a 3D noise field directly to the
  base SDF (cheap, but produces floating "floater" detail since it applies everywhere in
  3D space, not just at the surface); perturbing a primitive's *radius* as a function of
  a surface-projected noise coordinate (avoids floaters, can't represent overhangs); and
  triplanar-warping the SDF's domain before evaluation (displacement-mapping-like
  results). **All three degrade the field's Lipschitz/exact-distance guarantee** and
  require a more conservative (smaller) sphere-tracing step size to avoid pitting —
  directly relevant given this project's own prior fix for exactly this failure mode
  (raymarch pitting on curved CSG blends). See
  [Adding Details to Implicit Surfaces (aparis69, 2024)](https://aparis69.github.io/public_html/posts/2024_implicit_details.html)
  and cross-reference
  [exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md) and
  [raymarching-artifacts-and-fixes](../rendering/raymarching-artifacts-and-fixes.md).
- **Production precedent for art-directed procedural hybrids**: Hello Games' No Man's Sky
  mixes algorithmic material/color generation with artist-authored constraints/palettes
  to keep procedurally-varied planetary materials art-directable rather than
  uncontrollably random
  ([GDC, "Art Direction Bootcamp: How I Learned to Love Procedural Art"](https://www.gdcvault.com/play/1021805/Art-Direction-Bootcamp-How-I)).
- **Claybook** (Second Order, GDC 2018) is the strongest production precedent for a
  shipped game whose entire world — geometry *and* implied surface detail — is
  represented and ray-traced as SDFs built from CSG-composed primitives on GPU compute,
  with no traditional mesh/texture pipeline at all
  ([GDC Vault](https://www.gdcvault.com/play/1025316/Advanced-Graphics-Techniques-Tutorial-GPU)).
- **Media Molecule's Dreams** is architecturally distinct and worth flagging precisely:
  scenes are authored as SDF/CSG trees, but are **not** rendered by direct per-pixel
  sphere tracing — the SDF is converted to a dense, multi-resolution point cloud that's
  splatted into a G-buffer, with materials effectively painted per-point rather than
  computed procedurally per-pixel at render time
  ([Alex Evans, "Learning from Failure," SIGGRAPH 2015](https://www.mediamolecule.com/blog/article/siggraph_2015)).
  Dreams is the canonical example of SDF-*authored*, point-splat-*rendered* — a different
  architecture from a true raymarcher, and its material pipeline is closer to a deferred
  G-buffer renderer's than to a Shadertoy-style procedural shader.

## Resolution independence: the direct textual analog of what SDFs already give geometrically

An SDF has no fixed polygon budget — it resolves exactly at whatever ray-hit distance the
camera reaches, at any zoom. A pure procedural material has the identical property in
texture space: as a continuous function of the 3D hit point rather than a discretized
image, it never "runs out of resolution" the way a baked texture does once the camera
gets close enough to see individual texels. This is the direct textural-domain parallel
to what SDF geometry already provides for free, and it's why standard mipmap-based
texture antialiasing doesn't apply the same way to procedural materials — instead, either
supersample or use analytic/derivative-based filtering computed from the ray's footprint
at the hit point (see [uv-less-texturing](./uv-less-texturing.md)'s filtering notes,
and specifically
[iquilezles.org/articles/filteringrm](https://iquilezles.org/articles/filteringrm/) for
the raymarching-specific ray-differentials treatment).

**Practical downsides of leaning fully procedural**, documented rather than assumed:
- **Shader compile time / complexity.** Heavily-branched or node-graph-derived procedural
  shaders compound into many compiled permutations, each separately compiled/optimized —
  a real cost distinct from runtime performance, hitting iteration time and build size
  ([therealmjp, "The Shader Permutation Problem"](https://therealmjp.github.io/posts/shader-permutations-part1/)).
- **Art-directability.** Procedural noise stacks are good at *plausible* organic
  variation but hard to steer precisely toward a specific reference look — exactly Poly
  Haven's stated reason for favoring photogrammetry over procedural/hand-painted/
  estimated alternatives (op. cit.). Hybrid pipelines (above) exist specifically to
  reclaim reference-matching fidelity while keeping the memory/tiling benefits of
  procedural macro-variation.
- **SDF-specific downside**: any noise/detail added to the field itself (not just its
  shading) degrades the geometric correctness guarantee raymarching depends on for safe
  step sizing — "resolution-independent procedural surface detail" isn't free even
  geometrically, only in the texture-color sense (see the hypertexturing note above).

## Current (2024-2026) generative AI texture tools

Two distinct categories, and they answer "does AI seed procedural parameters, or just
produce more baked textures" directly: **almost all shipped/commercial tools produce
baked texture maps**, while only research-stage work targets actual procedural/node-graph
output.

**Baked-output, commercial:**
- **Adobe Substance 3D Sampler** — Text-to-texture, Text-to-pattern, Image-to-texture
  (Firefly-powered, debuted GDC 2024, shipped in Sampler 4.4, May 2024). Generates
  tileable texture *images*, which then feed Sampler's ordinary (non-AI) procedural
  filter stack for further editing
  ([Adobe Blog](https://blog.adobe.com/en/publish/2024/03/28/iterative-creativity-with-generative-ai-substance-3d)).
- **NVIDIA Edify (materials)** — multimodal generative architecture producing full PBR
  texture maps (base color, normal, roughness, AO) from text/image prompts, up to 4K via
  upscaling, integrated into Omniverse USD Composer — explicitly conventional texture
  maps, not a parametric representation
  ([NVIDIA Blog](https://blogs.nvidia.com/blog/siggraph-research-generative-ai-materials-3d-scenes/)).

**Procedural/parametric-output, research-stage:**
- **VLMaterial** (ICLR 2025) — fine-tunes a vision-language model to predict *editable
  Blender procedural node-graph materials as Python code* from a single input image,
  explicitly preserving resolution-independence and editability versus a static map
  ([arXiv:2501.18623](https://arxiv.org/html/2501.18623v2)).
- **MatFuse** (CVPR 2024) — diffusion-based generation conditioned on palette/sketch/
  text/reference image, with a multi-encoder model learning a disentangled latent per
  PBR map for post-generation, map-level editing — still texture-map output, but with
  more structured/editable latents than a flat baked image
  ([arXiv:2308.11408](https://arxiv.org/abs/2308.11408)).

**Bottom line**: no mainstream tool as of 2026 goes straight from a text/image prompt to
raymarcher-ready procedural shader code. Bridging the gap still requires a human either
triplanar-applying a baked AI-generated texture (asset-based path) or manually porting a
VLM-generated node graph via MaterialX (procedural path).

## Cost/performance crossover

**Texture sampling cost** depends on filtering mode and format — trilinear filtering
carries roughly a 2x cost multiplier over point sampling, FP32 formats roughly 2x over
8-bit, on typical GPU texture units
([Arm GPU Best Practices](https://developer.arm.com/documentation/101897/v2-2/Buffers-and-textures/Texture-sampling-performance)).
Whether a shader is bandwidth-bound or compute-bound determines whether fetch latency can
be hidden under existing ALU work.

**Procedural evaluation cost** scales directly with octave/warp-layer count (see
[uv-less-texturing](./uv-less-texturing.md)) — every sample must be computed, not looked
up. Standard mitigation: LOD-style octave reduction for distant/minified surfaces, or a
hash-based lookup as a middle ground between texture fetch and full noise evaluation.

**Where the crossover lies, in concrete terms:**
- **Memory** clearly favors procedural: a 4K PBR texture set (BC7-compressed) costs
  roughly 32MB VRAM versus ~2MB for 1K, while a procedural shader's footprint is just its
  instruction count — independent of apparent resolution or camera zoom.
- **Per-fragment bandwidth** runs the other way at scale: a full triplanar PBR material
  (albedo+normal+roughness+AO, each ×3 axes) can mean 12+ texture fetches per shaded
  fragment before any material layering is even added (see
  [material-blending](./material-blending.md)'s 60-fetch worked example) — cheap per
  fetch, but adds up across a full-screen raymarch paying for multiple bounces/shadow
  rays.
- There's no universal crossover point — it depends on whether the specific renderer is
  bandwidth-bound (favors procedural) or ALU-bound (favors baked/triplanar). The general
  shape: **procedural wins on memory and scales better with camera proximity** (no
  minification/aliasing budget to manage); **triplanar/baked wins on raw per-fragment GPU
  cycles** and matches hardware texture-cache design. The practical decision in shipped
  raymarchers is usually made per-material based on how many noise octaves are actually
  needed to look convincing, not by a fixed rule.

## When to dive in

- Deciding whether a new material should be procedural or asset-based → default to
  procedural for anything that benefits from the SDF's own resolution-independence
  (organic, close-up-viewable surfaces); reach for asset-based + triplanar when matching
  a specific real-world reference material matters more than avoiding texture memory.
- A large triplanar-mapped surface shows obvious tiling → add procedural macro-variation
  layered over the base tile (see hybrid section above) before trying anything more
  invasive.
- Considering adding surface roughness/detail to the SDF geometry itself, not just its
  shading → read the hypertexturing note above and
  [raymarching-artifacts-and-fixes](../rendering/raymarching-artifacts-and-fixes.md)
  first; this is a common source of pitting/floater artifacts.
- Evaluating an AI texture-generation tool for this pipeline → check whether it outputs a
  baked map (→ triplanar path) or a procedural graph (→ manual porting via MaterialX or
  by hand); no current tool bridges directly to raymarcher shader code.

## Related
- [Texturing without UVs](./uv-less-texturing.md) — prerequisite: the triplanar bridge every asset-based texture needs.
- [Material blending](./material-blending.md) — applies: layering procedural macro-variation over scanned detail.
- [Exact vs. bound distance fields](../fundamentals/exact-vs-bound-sdfs.md) — prerequisite: why hypertexturing the field itself risks pitting.
- [SDF modeling tools](../state-of-the-art/sdf-modeling-tools.md) — contrast: geometry authoring tools rather than material authoring.
