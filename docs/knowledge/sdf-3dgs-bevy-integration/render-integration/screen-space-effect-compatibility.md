---
title: Why built-in Bevy screen-space effects work on splat-rendered pixels for free
description: Audits which Bevy 0.19 post effects worked on migera's deleted splat pass for free (tonemapping, bloom, vignette, FXAA, upscaling read only ViewTarget colour) and which needed a splat prepass (TAA, SSAO, DOF). Read before adding any custom Core3d geometry pass that must support post-processing.
type: design
status: archived
tags:
  - 3dgs
  - bevy
  - post-processing
  - temporal
  - integration
updated: 2026-08-15
sources:
  - commit 1fe5e40 (splat prepass added)
  - commit 22d3b91 (splat pipeline removed)
aliases:
  - screen-space effects
  - prepass
  - motion vectors
  - TAA
  - SSAO
---

# Why built-in Bevy screen-space effects work on splat-rendered pixels for free

> **Archived (2026-09-28):** the splat pass and its prepass (`src/splat/prepass.rs`, added in commit 1fe5e40) were deleted in 22d3b91 (2026-08-16) and 684490c (2026-09-06); the ViewTarget-vs-prepass split still holds for any custom Core3d pass.

## The general principle

Every Bevy post-process/screen-space effect (see
[post-processing](../../bevy-rendering/post-processing/INDEX.md)) is written against
`ViewTarget` — it reads whatever is currently in `target.post_process_write().source` and
writes to `.destination`, with **no knowledge of what produced the pixels it's reading**
(see
[camera-and-view-system](../../bevy-rendering/scene-and-views/camera-and-view-system.md)
for the `post_process_write()` ping-pong mechanism). Because
[bevy-pipeline-integration](../architecture/bevy-pipeline-integration.md) registers
`main_splat_pass_3d` as an ordinary `Core3dSystems::MainPass` system writing into that
same `ViewTarget`, every downstream effect's "I don't know or care what produced these
pixels" assumption holds exactly as true for splat-rendered pixels as for mesh-rendered
ones. This is not something that had to be specially arranged per-effect — it is the
default consequence of following Bevy's own pass-authoring convention rather than
deviating from it. The rest of this document walks through specific built-in effects
concretely, both to make the general claim verifiable and to flag the handful of
effects that need an explicit prepass contribution from the splat pass rather than
working with literally zero extra code.

## Effects that work with zero splat-specific code

These operate purely on the `ViewTarget`'s color contents (and, for tonemapping,
whether the camera is in HDR mode) — no scene-specific data required at all:

- **Tonemapping** (see
  [passes-and-fullscreen-effects](../../bevy-rendering/core-pipeline/passes-and-fullscreen-effects.md)) —
  reads/writes `ViewTarget` unconditionally for any HDR camera; splat-contributed HDR
  color values tonemap identically to mesh-contributed ones, since tonemapping is a pure
  function of pixel color.
- **Bloom** (see [post-processing](../../bevy-rendering/post-processing/INDEX.md)) —
  operates on bright regions of the HDR `ViewTarget` via its mip-chain downsample/
  upsample passes; a bright, emissive-appearing splat (e.g. a procedurally SDF-baked
  light source or highly-lit surface, see
  [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) Step 3's baked-
  lighting-response option) blooms exactly like a bright mesh surface would.
- **Vignette, lens distortion, chromatic aberration** (the effect-stack fusion pass, see
  [post-processing](../../bevy-rendering/post-processing/INDEX.md)) — pure per-pixel
  UV/color transforms with zero scene awareness; trivially agnostic to what produced the
  underlying color.
- **FXAA / SMAA / CAS** (post-tonemap screen-space anti-aliasing/sharpening, see
  [post-processing](../../bevy-rendering/post-processing/INDEX.md)) — operate on the
  final LDR image via edge-detection/local-contrast analysis of color values; splat
  silhouette edges get anti-aliased by these passes exactly as mesh silhouette edges do.
  Note this is a *distinct* anti-aliasing concern from
  [Mip-Splatting-style anti-aliasing](../../3dgs/quality-and-artifacts/mip-splatting-and-anti-aliasing.md),
  which addresses aliasing *within* the splat rasterization pass itself (zoom-dependent
  sampling-rate mismatches) — both should be applied; they solve different problems at
  different pipeline stages and are complementary, not redundant.
- **Upscaling / final blit** (see
  [passes-and-fullscreen-effects](../../bevy-rendering/core-pipeline/passes-and-fullscreen-effects.md)) —
  the final `ViewTarget → out_texture` blit is agnostic to content by construction.

## Effects that need the splat pass to write a real prepass contribution

A smaller set of effects consume more than just final composited color — they read
**depth**, **normals**, or **motion vectors** written during `Core3dSystems::Prepass`,
*before* `MainPass` runs (see
[passes-and-fullscreen-effects](../../bevy-rendering/core-pipeline/passes-and-fullscreen-effects.md)).
For these, splats must contribute to the relevant prepass texture(s), not just the final
color `ViewTarget`, or the effect will either ignore splat geometry entirely (best case)
or produce visibly wrong results (worse case — e.g. SSAO computing occlusion using only
mesh depth, making splat-rendered surfaces incorrectly appear un-occluded/unshadowed by
splat geometry near them):

- **TAA** (temporal anti-aliasing, requires `DepthPrepass` + `MotionVectorPrepass`, see
  [post-processing](../../bevy-rendering/post-processing/INDEX.md)) — needs splats to
  write per-pixel motion vectors (camera-relative, and object-relative if splats are ever
  attached to moving entities) into the `MotionVectorPrepass` texture during
  `Core3dSystems::Prepass`, analogous to how ordinary mesh geometry's prepass pass
  writes motion vectors from its own vertex transforms. Without this, TAA's temporal
  reprojection has no motion data for splat-covered pixels and will either fall back to
  treating them as static (causing ghosting under camera motion) or exclude them from
  temporal accumulation entirely (causing them to look comparatively noisier/less
  anti-aliased than surrounding mesh geometry).
- **SSAO / screen-space effects needing depth+normals** — need splats to write depth
  (straightforward — splats already produce real per-pixel depth for the standard mesh
  depth-test interaction described in
  [bevy-pipeline-integration](../architecture/bevy-pipeline-integration.md)) and,
  for normal-dependent effects, a representative per-splat surface normal into the
  `NormalPrepass` texture — directly available for splats baked via
  [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md), since splat
  orientation there is derived from the SDF gradient/surface normal in the first place
  (a genuine advantage over photographically-trained splats, which have no inherent
  normal concept beyond what a technique like
  [2D Gaussian Splatting](../../3dgs/state-of-the-art/2d-gaussian-splatting.md) adds).
- **Depth of field** (needs a depth prepass to compute circle-of-confusion, see
  [post-processing](../../bevy-rendering/post-processing/INDEX.md)) — same requirement as
  SSAO's depth dependency; splats already contribute real depth via the main-pass depth
  test, so this specifically needs that depth to also be visible to/consistent with the
  separate `DepthPrepass` texture DOF reads (implementation detail: whether splats write
  depth during `Prepass` directly, or the `DepthPrepass` texture is derived from the same
  depth buffer `MainPass` writes to, is an implementation choice — either is workable, but
  must be decided consistently, since DOF specifically runs *before* `MainPass` completes
  in schedule order and needs depth data available by then).

## The concrete engineering implication

This splits `main_splat_pass_3d`'s implementation into two logically separate concerns
that should be designed for explicitly rather than discovered as an afterthought once a
specific effect "doesn't look right":

1. A **prepass contribution** (`main_splat_prepass_3d`, registered in
   `Core3dSystems::Prepass`, chained appropriately relative to the existing mesh prepass
   systems per
   [passes-and-fullscreen-effects](../../bevy-rendering/core-pipeline/passes-and-fullscreen-effects.md))
   writing depth/normal/motion-vector data for any camera that has the corresponding
   `DepthPrepass`/`NormalPrepass`/`MotionVectorPrepass` component present — following
   the same opt-in-component pattern ordinary mesh prepass contribution already uses, so
   splats only pay this cost when a camera actually requests it.
2. The **main color pass** (`main_splat_pass_3d`, the one designed in
   [bevy-pipeline-integration](../architecture/bevy-pipeline-integration.md)), which is
   what actually needs the [sort-free WSR compositing](./sort-free-compositing.md)
   discussed separately.

Skipping (1) does not break basic rendering or tonemapping/bloom/vignette-class effects —
it specifically and only degrades the smaller set of effects listed above, which is a
reasonable, explicit scope decision for a first implementation (start with (2) alone,
verify the "zero-code" effect list above works as claimed, then add (1) incrementally per
effect as needed) rather than a correctness requirement that must be solved before
anything renders at all.

## When to dive in

- Verifying a specific built-in Bevy post-process effect will work with splat-rendered
  content before implementing → the two categorized lists above are the direct answer;
  cross-reference against
  [post-processing](../../bevy-rendering/post-processing/INDEX.md) for that effect's
  exact prepass requirements if it's not explicitly named here.
- Scoping an initial implementation → start with the main color pass only (verifying
  tonemapping/bloom/vignette/AA work), and treat prepass contribution (TAA/SSAO/DOF
  support) as an explicit, separately-scoped follow-up rather than a blocking
  requirement.
- Debugging a specific effect looking wrong specifically around splat-rendered regions
  (ghosting under motion, missing ambient occlusion, absent depth-of-field blur) → check
  whether that effect is in the prepass-dependent list above and whether the
  corresponding prepass contribution has actually been implemented.

## Related

- [Native integration with Bevy's render pipeline](../architecture/bevy-pipeline-integration.md) — prerequisite: the shared-`ViewTarget` pass registration this relies on.
- [Compositing: Weighted Sum Rendering vs. real depth-tested opaque rendering](./sort-free-compositing.md) — deeper: how the splat pass wrote colour and depth.
- [Post-Processing](../../bevy-rendering/post-processing/INDEX.md) — prerequisite: each effect's exact inputs.
- [Prepass, tonemapping, upscaling, deferred, and OIT](../../bevy-rendering/core-pipeline/passes-and-fullscreen-effects.md) — deeper: what the prepass must contain.
- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — applies: the same pre/post-process compatibility question for migera's current SDF renderer.
