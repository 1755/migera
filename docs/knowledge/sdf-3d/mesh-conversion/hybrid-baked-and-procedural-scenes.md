---
title: Integrating a baked mesh SDF into this project's raymarcher
description: Researches adding a 3D-texture baked-mesh leaf to migera's src/raymarch — a small eval_leaf case with an analytic out-of-bounds fallback, subtly degraded smin blends against exact leaves, no non-uniform scale, bounding-box culling, no production precedent. Not built. Read before adding any texture-sampled leaf.
type: research
status: current
tags:
  - sdf
  - mesh-conversion
  - csg
  - raymarching
  - performance
  - correctness
updated: 2026-09-28
verified: 2026-09-28
code:
  - assets/shaders/raymarch.wgsl
  - src/raymarch/flatten.rs
  - src/sdf/scene.rs
sources:
  - https://iquilezles.org/articles/smin/
  - https://iquilezles.org/articles/sdfbounding/
  - https://onlinelibrary.wiley.com/doi/10.1111/cgf.70057
aliases:
  - baked mesh leaf
  - TAG_LEAF_BAKED_MESH
  - 3D texture SDF leaf
---

# Integrating a baked mesh SDF into this project's raymarcher

**Status (verified 2026-09-28):** not built — `assets/shaders/raymarch.wgsl` has no
baked-mesh leaf; every leaf is still analytic. The GLB world this was motivated by was
dropped (see [glb-to-bsn-conversion](../glb-to-bsn-conversion.md)), but the analysis
applies to any future baked asset.

This document is scoped specifically to **this project's own architecture** (as opposed
to the other documents in this topic, which are general) — it covers what adding "an
imported glTF, baked to a grid SDF" as a new primitive kind would actually require in
`assets/shaders/raymarch.wgsl` and its supporting Rust code, and what to expect from
mixing it with the existing exact analytic primitives.

## The current architecture, as the integration surface

The raymarcher evaluates the whole scene in **one fragment-shader invocation per pixel**:
a flat GPU array of `PrimitiveRecord`s (tagged with one of eight analytic leaf kinds,
`TAG_LEAF_SPHERE` … `TAG_LEAF_HEX_PRISM`, or `TAG_OP_UNION`/`SMOOTH_UNION`/`SUBTRACT`/
`SMOOTH_SUBTRACT`/`INTERSECT`/`SMOOTH_INTERSECT`; the torus leaf this note originally
listed was removed in commit 3d5d570),
combined via a non-recursive post-order (RPN) walk in `eval_stack` inside `map()`. Every
leaf already goes through a uniform world→local transform (translation + quaternion
rotation, via each record's `translation_*`/`rotation_*` fields) before its own
closed-form distance function runs — see `eval_leaf`. `map()` is called on the order of
~90 times per pixel (primary march steps + normal taps + shadow/AO steps, per the
shader's own tuning comments).

This structure is the reason a baked-mesh leaf is a comparatively *small* addition: the
transform pipeline, the RPN evaluation loop, and the CSG combinators (`smin`/`smax`) are
all leaf-kind-agnostic already. What's new is one leaf **evaluation function** (a
`TAG_LEAF_BAKED_MESH` case in the `eval_leaf` switch) that samples a 3D texture instead
of computing a closed form — everything else in `map()`/`eval_stack` is unchanged.

## The leaf evaluation function

The standard pattern, confirmed across several independent implementations
([Terrius, "Mesh Distance Field + Shadow"](https://andreasterrius.github.io/posts/sdf-shadow-guide/);
[kosmonaut, "SDF Rendering Journey pt.2"](https://kosmonautblog.wordpress.com/2017/05/09/signed-distance-field-rendering-journey-pt-2/)):

1. The query point arrives **already in local space** — this project's `eval_leaf`
   already does the world→local transform uniformly for every leaf, so a baked-mesh leaf
   needs no new transform-pipeline code, just its own remap from local space to the
   bake's own bounding box.
2. Remap the local-space point from the bake's bounding box `[bbMin, bbMax]` into
   `[0,1]³` texture coordinates, and `textureSample` the 3D texture with a linear
   sampler — hardware trilinear filtering combines up to 8 texel reads into one
   instruction, measured at roughly **1.4x the cost of a nearest-neighbor fetch**, not
   8x, since the filtering hardware absorbs most of the cost
   ([Csébfalvi, CGF 2023](https://onlinelibrary.wiley.com/doi/full/10.1111/cgf.14753)).
3. **Out-of-bounds fallback, required for sphere-tracing safety everywhere in space, not
   just inside the voxel grid**: use a padded *outer* bounding box (e.g. the bake bounds
   scaled ~1.1x) distinct from the *inner* (actual data) box. While a query point is
   outside the outer box, return the exact analytic distance to that box (the same
   `sdf_rounded_box`-style formula this project already has) as a safe, correct lower
   bound; only once inside the outer box, sample the texture. This guarantees the
   returned value is always a safe step size, matching every other leaf's contract.

This is a genuinely small, additive change to `eval_leaf` — no change to `eval_stack`,
`map()`, or the CSG combinators is needed at all.

## CSG-combining a baked leaf with exact primitives: works mechanically, degrades subtly

`smin(a, b, k)` — this project's polynomial smooth-minimum — only ever consumes two
already-evaluated scalars; it has no idea whether `a` came from `sdf_sphere` or a texture
fetch, so a `TAG_OP_SMOOTH_UNION` with one baked child and one analytic child evaluates
without any special-casing.

**But blend quality depends on both operands' gradients, and that's not formally
guaranteed once one side is baked.** Inigo Quilez's own derivation of the polynomial smin
assumes both inputs are true, exact 1-Lipschitz SDFs — under that assumption, the
blended gradient is the interpolation of the input gradients, which stays magnitude ≤1
and keeps the guarantee intact. His article does not cover the case where an input isn't
a true SDF (i.e. a trilinearly-interpolated grid field, which is only *piecewise-linear*
between texel centers and generally doesn't have exactly unit gradient magnitude even
where the bake itself was accurate) — see
[Quilez, "smin"](https://iquilezles.org/articles/smin/) and
[glitch-free-baked-sdfs §2](./glitch-free-baked-sdfs.md#2-interpolation-artifacts-and-sphere-tracing-safety)
for why that gap matters. A 2025 paper working specifically with conservative (not exact)
SDFs combined by smooth CSG makes the general point formally: "when operands are exact
SDFs, the resulting field is only a conservative SDF" after smin — even blending two
*exact* fields already only conservatively bounds distance, and deriving proper Lipschitz
bounds for such trees is itself active research, not a solved formula
([Barbier et al., "Lipschitz Pruning," CGF 2025](https://onlinelibrary.wiley.com/doi/10.1111/cgf.70057)).

**Practical consequence, concretely tied to this project's own history**: expect the
smooth-union seam against a baked leaf to need either a larger `k` fudge factor tuned
empirically per baked asset, or the same kind of step-size conservatism this project
already had to apply for purely-analytic curved blends (see the raymarch shader's own
`sphere_trace` comments on the pitting fix for `smin`'s non-exact blend region, and
[raymarching-artifacts-and-fixes](../rendering/raymarching-artifacts-and-fixes.md)) —
likely needing to be *more* conservative specifically near baked leaves, since their
gradient error compounds on top of (not instead of) the existing analytic-blend
approximation. **Hard union/subtract are unaffected** — they only take a `min`/`max` of
two scalars with no gradient-shape dependency, so a baked leaf hard-combined with
anything else has no extra blend-quality risk at all.

No public shadertoy/demoscene example was found that specifically smooth-blends a
texture-sampled mesh SDF against an analytic primitive — despite each ingredient (3D
texture sampling, IQ-style smin) being individually extremely common on Shadertoy, this
specific combination appears genuinely under-documented even in the community that
otherwise pioneers most SDF CSG techniques.

## Transform support: translation/rotation free, uniform scale needs care, avoid non-uniform scale

**Translation and rotation need no new code** — a baked-mesh leaf consumes the same
already-localized `p` every other leaf tag consumes (see above); it only needs its own
bbox-relative UVW remap inside its own leaf function.

**Uniform scale**: the standard trick is to scale the query point down before sampling
(sample the *unscaled* bake-space field) and scale the returned distance back up by the
same factor — `distance(p / scale, ...) * scale` — exact for a single uniform scalar
([kosmonaut](https://kosmonautblog.wordpress.com/2017/05/09/signed-distance-field-rendering-journey-pt-2/)).

**Non-uniform scale is worse for a baked field than for an exact analytic primitive, and
should be avoided** — this is independently confirmed by two production engines' own
documentation, not just theory. Unreal: **"Non-uniform scaling cannot be handled
correctly (although, mirroring is ok)... Scaling the mesh by two times or less is not
generally noticeable"**
([Unreal Engine docs](https://dev.epicgames.com/documentation/en-us/unreal-engine/mesh-distance-fields-in-unreal-engine)).
Flax: **"avoid non-uniform scale... distance fields are inaccurate if a model has scale
e.g. (10, 1, 1)"**, recommending scale be baked in at import/build time rather than
applied at runtime
([Flax Engine SDF docs](https://docs.flaxengine.com/manual/graphics/models/sdf.html)).
The reason: non-uniform scaling isn't a distance-preserving transform for *any* SDF —
"scale the point, scale the output" is only a conservative bound even for an exact
analytic primitive (this project's own `Isometry` type deliberately excludes scale for
exactly this reason, per its doc comment). For an *already-interpolated* baked field, the
pre-existing voxel-resolution/interpolation error and the anisotropic distortion from
non-uniform scale compound rather than either dominating — a measurably worse case than
the analytic one. **Recommendation matching this project's own `Isometry` convention**:
don't add a scale field to the baked-mesh leaf at all initially; if a differently-scaled
instance of the same source mesh is needed, re-bake it, matching what every production
engine surveyed actually recommends.

## Performance: bounding-volume culling is the lever that already exists in this codebase

A trilinear 3D texture fetch is memory-bound where every other leaf in this scene is
pure-ALU — not free, but not the "8 separate fetches" its filtering description might
suggest (§ above: ~1.4x a single fetch's cost, hardware-absorbed). The real multiplier is
**how many times `map()` runs per pixel** (~90, per this shader's own comments) — a baked
leaf anywhere in an unculled CSG subtree pays its fetch cost that many times, not once.

**This project already has the exact mitigation pattern proven and in production use**:
`map()`'s `AnimGroup` handling skips a whole subtree's `eval_stack` call entirely when
the query point is farther than that group's `bounding_radius` from its pivot — described
in the shader's own comments as "the single highest-leverage performance win available."
A baked-mesh leaf's own bounding box (already needed for the out-of-bounds fallback,
above) doubles as this same cull volume for free: skip the texture sample entirely
whenever the query point is outside the padded outer box, exactly mirroring the
`AnimGroup` early-out already in `map()`. This is also Inigo Quilez's own general
recommendation for expensive sub-SDFs — wrap the costly primitive in a cheap bounding
shape and skip evaluation whenever that bound already exceeds the running minimum
distance ([Quilez, "Bounding Volumes for Distance Fields"](https://iquilezles.org/articles/sdfbounding/)).

Secondary mitigations, if the primary cull isn't enough:
- **Reduced-fidelity sampling for shadow/AO passes specifically** — since those already
  run at reduced step counts in this shader (see `soft_shadow`/`ambient_occlusion`'s own
  tuning), sampling a coarser mip of the same bake texture for those passes (full detail
  reserved for primary visibility) is a natural, low-effort extension using the same
  texture resource.
- **Distance-based mip swap**, matching the same principle Unreal's Global Distance Field
  clipmap applies at the whole-scene level (see
  [efficient-grid-baking](./efficient-grid-baking.md#multi-resolution--clipmap-architecture-unreals-global-distance-field)) —
  cheaper for a single baked leaf than swapping to a wholly separate proxy primitive,
  since it reuses the same texture resource's built-in mip chain.

## No known precedent does exactly this — worth knowing going in

Unreal Lumen, Flax's Global SDF, and Godot's SDFGI are the closest production analogs,
but they all solve the *adjacent* problem: compositing many baked fields together into a
shared grid representation for GI/occlusion, not smooth-CSG-blending a baked field
against a live, exact analytic primitive in a single per-pixel expression tree the way
this project's `eval_stack` does. The one hobbyist raymarcher found with a closely similar
architecture — a Godot 4 node-based analytic-CSG raymarcher with per-primitive color
blending matching this project's own smooth-union material-blending approach — explicitly
states baked-mesh support is out of scope: "Adding a new primitive still means adding its
distance function and evaluation branch to the shader," and every primitive in that
system is analytic-only
([Vav Labs, "SDF Raymarching in Godot 4"](https://vav-labs.com/case-studies/sdf-raymarcher-godot/)).
Blender's 5.1+ SDF/Volume geometry nodes take the opposite approach — converting *every*
operand, including primitive shapes, to the grid representation before combining, rather
than keeping exact analytic primitives exact
([StraySpark, "Blender 5.1 SDF and Volume Nodes"](https://www.strayspark.studio/blog/blender-51-sdf-volume-nodes-game-assets)).
This project's own goal — baked mesh leaf, smooth-CSG-blended against exact analytic
leaves, in one live per-pixel evaluator — is a genuinely under-explored combination, not
a solved, off-the-shelf pattern to copy.

## When to dive in

- Actually implementing baked-mesh import → start with hard union/subtract against
  existing primitives (no blend-quality risk, per above) before attempting smooth-union
  blending against a baked leaf; validate the out-of-bounds fallback and bounding-volume
  cull first, since both reuse the bake's bounding box and are needed regardless of
  whether blending is ever used.
- Smooth-union blending looks visibly different (falloff shape, bulge) on the baked side
  of a seam versus two analytic primitives blended at the same `k` → expected, not a bug;
  tune `k` empirically for that seam rather than assuming the analytic-primitive default
  transfers directly.
- A scene with baked-mesh leaves runs slower than expected → confirm the bounding-volume
  early-out is actually wired up (mirroring the existing `AnimGroup` pattern in `map()`)
  before reaching for mip-based or reduced-fidelity mitigations.
- Considering non-uniform scale on an imported/baked asset → don't; re-bake at the target
  scale instead, per every production engine's own documented recommendation.

## Related
- [Glitch-free baked SDFs](./glitch-free-baked-sdfs.md) — prerequisite: why a trilinear bake is only a bound.
- [Efficient grid baking](./efficient-grid-baking.md) — prerequisite: producing the 3D texture this leaf samples.
- [Combining SDFs](../primitives-and-operators/combination-operators.md) — deeper: the smin whose guarantees degrade against a baked operand.
- [Converting GLB scenes to SDF BSN scenes](../glb-to-bsn-conversion.md) — contrast: the dropped primitive-fitting route for the same imported content.
- [Raymarching via compute](../../compute-shaders/raymarching-via-compute.md) — contrast: the compute-shader alternative to this fragment-shader evaluator.
- [Unreal Lumen distance fields](../state-of-the-art/unreal-lumen-distance-fields.md) — contrast: the closest production analog (baked fields composited for GI).
