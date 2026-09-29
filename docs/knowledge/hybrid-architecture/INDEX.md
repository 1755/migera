---
title: Hybrid renderer architecture
description: migera's own src/hybrid SDF renderer — the decision that characters moved to Bevy's PBR pipeline, the self-shading architecture and Bevy integration, GI/lighting leak fixes, measured perf findings, and archived plans. Read before changing src/hybrid, adding a render stage, or chasing a light leak or trace-pass cost.
type: index
status: current
tags:
  - hybrid-renderer
  - sdf
  - global-illumination
  - performance
  - render-pipeline
updated: 2026-09-28
---

# Hybrid renderer architecture

Project-specific knowledge about `src/hybrid`, the ground-up rewrite of
migera's SDF renderer: its architecture decisions, the bugs it taught us, what
was measured, and the plans it outgrew. General Bevy, SDF and acceleration
theory lives in other domains (see "See also"); this tree links to them rather
than repeating them. Chronological progress is in `PROGRESS.md`.

> **Scope since 2026-09-21: the hybrid renderer is no longer the
> character-rendering target.** Characters use Bevy's standard mesh/PBR/glTF/
> skinning pipeline (`src/character`, `examples/character_gallery.rs`). The SDF
> work (`src/sdf`, `src/raymarch`, `src/hybrid`) is kept for effects and
> procedural world generation. See
> [migera renders characters with Bevy's PBR pipeline](./migera-pivot-to-bevy-pbr-for-characters.md).

**Start here:** the pivot decision above, then
[Self-shading vs. G-buffer-writer](./self-shading-vs-gbuffer-decision.md).

## Key facts
- **Characters are not rendered by `src/hybrid`;** it is kept for SDF effects and world generation. — [Pivot decision](./migera-pivot-to-bevy-pbr-for-characters.md)
- **`src/hybrid` shades itself in one WGSL pipeline and blits lit colour;** it never writes Bevy's G-buffer. The premise is an SDF-only scene. — [Self-shading decision](./self-shading-vs-gbuffer-decision.md)
- **Every non-trivial piece of math is a CPU reference with tests before it is ported to WGSL.** Most bugs found were CPU-vs-WGSL or test-input divergences. — [DDGI grid lookup bug](./gi-and-lighting/ddgi-sealed-room-light-leak.md)
- **DDGI is the shipping indirect-diffuse method;** Radiance Cascades got close but stays experimental. — [Radiance Cascades experiment](./gi-and-lighting/radiance-cascades-experiment.md)
- **Four independent sealed-room leaks were fixed in 646a388;** after them `--gi-method ddgi` and `none` are bit-for-bit identical in a sealed `gi_room`. — [Probe-grid bounds leak](./gi-and-lighting/ddgi-probe-grid-bounds-wall-embedding-leak.md)
- **DDGI's per-probe occlusion rays are ~48–52% of the frame at `--stress 10000`,** and an early-out `any_hit` did not reduce that. — [DDGI any-hit occlusion](./gi-and-lighting/ddgi-any-hit-occlusion.md)
- **March step count is not the bottleneck,** and scene misses are already ~O(1). — [Trace-pass bottleneck](./performance-findings/trace-pass-bottleneck-is-not-march-steps.md), [Skybox bake](./performance-findings/skybox-far-object-bake-not-worth-it.md)
- **`RoundedCone`'s SDF is broken** and deliberately unported. — [RoundedCone bug](./roundedcone-sdf-reports-everything-exterior.md)

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [migera renders characters with Bevy's PBR pipeline](./migera-pivot-to-bevy-pbr-for-characters.md) | Since 2026-09-21 characters use Bevy meshes/PBR/skinning; `src/hybrid` is for SDF effects and world generation. | Before any character or rendering work, or choosing a pipeline for a feature. |
| [Self-shading vs. G-buffer-writer decision](./self-shading-vs-gbuffer-decision.md) | Why `src/hybrid` shades end-to-end instead of feeding Bevy's deferred G-buffer, and when to revisit. | Before reconsidering G-buffer output or compositing SDF content with meshes. |
| [Bevy-native integration](./bevy-native-integration.md) | Extract from real Bevy camera and light components with Bevy's own conversions; hand-roll only what Bevy lacks. | Before adding extraction or a post-process pass. |
| [Module and stage skeleton](./module-and-stage-skeleton.md) | **Stale** module map; still-valid MainPass ordering contract, extension seams, and the declined inner AABB. | Before adding a stage to `src/hybrid`. |
| [RoundedCone's SDF reports every point as exterior](./roundedcone-sdf-reports-everything-exterior.md) | The shipped `RoundedCone` formula is wrong everywhere; `src/hybrid` pins it as `unimplemented!`. | Before using, porting or fixing `RoundedCone`. |

## Topics

| Note | What it establishes | Read when |
|---|---|---|
| [GI and lighting](./gi-and-lighting/INDEX.md) | Temporal accumulation, Radiance Cascades vs DDGI, four sealed-room leaks, DDGI any-hit occlusion. | When light leaks or GI looks wrong, or before changing DDGI, shadows or bounces. |
| [Performance findings](./performance-findings/INDEX.md) | Measured null results: march steps, half-res reflections, skybox bake. | Before optimizing the trace pass. |
| [Plans](./plans/INDEX.md) | Pre-rewrite design docs: three-tier hardening survey (stale), analytic gradients and tier transitions (archived). | Before re-proposing tiers, LOD transitions or analytic normals. |

## See also
- [3D signed distance fields](../sdf-3d/INDEX.md) — SDF theory, sphere tracing, soft shadows, normals.
- [Hierarchical volumes](../hierarchical-volumes/INDEX.md) and [AABBs & spatial acceleration](../aabb-acceleration/INDEX.md) — the BVH and slab-test theory `src/hybrid` uses.
- [Analytic intersections](../analytic-intersections/INDEX.md) — the retired analytic tier and why it was removed.
- [Compute shaders](../compute-shaders/INDEX.md) — execution model and performance practice for the trace dispatch.
- [Character animation](../character-animation/INDEX.md) — where character work lives since the pivot.
- [Engineering practice](../engineering-practice/INDEX.md) — the testing, debugging and measurement lessons several of these bugs illustrate.
