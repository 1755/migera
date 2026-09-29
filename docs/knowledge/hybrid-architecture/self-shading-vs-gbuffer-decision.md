---
title: Self-shading vs. G-buffer-writer decision
description: src/hybrid traces and shades itself in one hand-rolled WGSL pipeline and blits lit color, instead of packing hits into Bevy's deferred G-buffer — chosen for shadow cost, material-format freedom and ordering, on an SDF-only scene. Read before reconsidering G-buffer output or compositing SDF content with Bevy meshes.
type: decision
status: current
tags:
  - hybrid-renderer
  - render-pipeline
  - lighting
  - shadows
  - integration
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/hybrid/mod.rs
  - src/hybrid/pass.rs
  - src/prepass_probe
sources:
  - commits 89c683c, eabe83e, 8925162 (Option A spike), 684490c (revert)
aliases:
  - G-buffer writer
  - deferred lighting
  - Option A
  - Option B
  - self-shading pipeline
---

# Self-shading vs. G-buffer-writer decision

**Decision: `src/hybrid` shades itself end-to-end in hand-rolled WGSL —
one connected pipeline it owns from primary ray to final color — rather
than packing hit data into Bevy's real deferred G-buffer and letting
Bevy's own deferred-lighting pass shade it.** This page exists so that
decision isn't re-derived, contradicted, or re-litigated by accident
later without the reasoning that produced it.

## The two options that were weighed

**Option A — G-buffer-writer.** A compute pass writes hit-t/normal/
material to scratch textures; a separate fragment blit packs them into
Bevy's real `ViewPrepassTextures.deferred` (exact `bevy_pbr` `Rgba32Uint`
packing) and real depth buffer; Bevy's own deferred-lighting pass shades
everything — real shadow maps, real clustered light list, all "for free."
Proven feasible for one hardcoded sphere by the `src/prepass_probe` spike
(kept as a frozen reference, not extended).

**Option B — self-shading monolithic (chosen).** A compute dispatch
traces *and* shades (today fused into one dispatch), then a fullscreen
blit writes already-lit color straight into Bevy's `ViewTarget`/depth
attachment — no `ViewPrepassTextures` involvement, no `bevy_pbr` G-buffer
packing, no dependency on Bevy's deferred-lighting or shadow-map
machinery. This is `src/hybrid_legacy::pass::hybrid_pass`'s proven shape.

## Why B, given this project's actual constraints

The scene is **SDF-only, forever** — no `Mesh3d`/`StandardMaterial`
content will ever exist in it. That single fact removes most of Option
A's original appeal, since almost all of its value was mesh interop:
shared depth-buffer occlusion between meshes and SDF content, mesh-
reflects-SDF via stock SSR reading a shared G-buffer, and cross-casting
shadows between meshes and SDF content — all irrelevant with zero mesh
content ever in the scene.

What's left of Option A's case, weighed against Option B's real costs:

- **Shadows.** Under A, every shadow-casting light needs the whole SDF
  scene traced into its own real shadow-map view(s) — 6 fragment passes
  per point light (cubemap faces), 1 per spot light, N per directional
  cascade — and *each pass re-traces the entire BVH + march for every
  shadow-map texel* (confirmed against `src/prepass_probe`'s actual
  per-view dispatch structure). That's `Σ_lights (6|1|N) ×
  shadow_resolution²` independent full SDF re-traces per frame, growing
  multiplicatively with light count. Under B, a shadow ray is one extra
  march per light *per already-shaded pixel*, in the same dispatch:
  `O(lights × screen_pixels)` — cheaper, and doesn't scale worse as
  lights are added.
- **Material representation.** Option A's G-buffer format is a hard,
  narrow constraint: one `Rgba32Uint` word with fixed bit budgets (4-bit
  clearcoat, unorm base-color+roughness, rgb9e5 emissive, octahedral
  normal, a handful of flag bits) — a mesh-PBR-shaped model. Anything an
  SDF material might eventually want beyond that (multi-layer blending,
  procedural pattern weights, volumetric density, ML-derived fields)
  either doesn't fit or has to be smuggled into bits that aren't there.
  Option B has zero such constraint — any custom intermediate format,
  sized exactly for what the SDF material model actually needs, including
  future custom or ML-derived fields that don't exist in `bevy_pbr`'s
  vocabulary at all.
- **Ordering.** Option A must run its G-buffer writer in
  `Core3dSystems::Prepass`, before `main_opaque_pass_3d` and before
  Bevy's own mesh prepass chain (SSAO's own dispatch runs strictly
  between Prepass and MainPass). With zero mesh content ever in the
  scene, that ordering constraint buys nothing — Prepass exists partly so
  opaque mesh content front-loads depth/normal for other systems to
  reuse, and there's no "other opaque content" here to share work with.
  It becomes a pure plumbing tax with no payoff.
- **Non-shadow multi-light performance** is the one place Option A might
  genuinely have won — Bevy's clustered lighting is GPU-driven and scales
  as `O(lights-per-cluster)` per pixel, which could beat a naive
  hand-rolled per-pixel light loop unless Option B deliberately builds its
  own light-clustering. This is a real, honest cost of choosing B for
  scenes with many non-shadow-casting lights; it's outweighed by the
  shadow-cost asymmetry above once shadows are involved at all, which is
  the common case.
- **Extensibility to future custom/ML work.** Bevy's post-process
  extension points (`EarlyPostProcess`/`PostProcess`,
  `FullscreenMaterialPlugin` as "the officially blessed extension point")
  are available to *either* option — they're downstream of whichever pass
  shades the scene, not gated behind the G-buffer choice. Where the
  options genuinely diverge is upstream: if a future custom/ML technique
  wants to read *pre-shading* geometric/material buffers shaped for its
  own needs (not `bevy_pbr`'s), Option B's freedom to define arbitrary
  intermediate buffer layouts is the better fit.

## Complexity tradeoff, stated honestly

Option A does remove hand-rolled shading math from the render path — the
actual bug source (`hybrid_legacy`'s `shade()`) that motivated this
project's fresh-start rewrite in the first place. But it doesn't remove
risk, it relocates it: G-buffer packing-format compliance has its own
proven gotcha (deferred-path emissive is never exposure-scaled by
`bevy_pbr`, has to be multiplied by hand — hit once already in
`src/prepass_probe`'s development) and the shadow-write subsystem's
per-shadow-view fragment-pass plumbing (ordering against the last shadow
pass, `LoadOp::Load` compositing correctness) is a new mechanism, not
fewer mechanisms. Option B keeps the same *shape* of risk that burned the
legacy renderer (self-authored WGSL shading math) — but that risk is more
manageable now than it was during `hybrid_legacy`'s own history, because
the CPU-reference-first discipline this project established (see
`src/hybrid/cpu_ref.rs`) didn't exist back then and applies just as well
to hand-rolled shading as it does to marching.

## Prior art and history

This project already tried Option A once, on the pre-freeze renderer
(commits `89c683c` write-and-throw spike, `eabe83e` Stage 1/5, `8925162`
Stage 2/5 shadow-write subsystem), and reverted it (`684490c`) before the
fresh-start freeze that produced `src/hybrid_legacy`. That revert bundled
in an unrelated 3D-Gaussian-splatting/far-LOD experiment that also got
dropped in the same commit, so it wasn't necessarily a clean verdict on
the G-buffer idea in isolation — this decision was re-examined from first
principles against the project's actual current constraints (SDF-only
scene, confirmed forever) rather than assumed settled by that prior
revert. It reached the same conclusion by a more complete path.

`src/prepass_probe` remains in the codebase as the proven reference for
what Option A's mechanics look like (exact G-buffer packing functions,
the Prepass-ordering constraint, the shadow-write pattern) — kept as a
frozen spike for comparison, should this decision ever be revisited with
new evidence, not deleted and not extended.

## Revisit when

The load-bearing premise is "SDF-only scene, forever — no mesh interop".
Since 2026-09-21 characters are Bevy PBR skinned meshes, and the hybrid
renderer is kept for effects and world generation (see
[the pivot decision](./migera-pivot-to-bevy-pbr-for-characters.md)). That is
still true *inside the hybrid renderer's own scenes* today; `character_gallery`
does not use `HybridRenderPlugin`. If SDF effects or SDF worlds ever have to
share a frame with those meshes (depth occlusion, shadows cast between them),
Option A's mesh-interop advantages come back and this decision must be
re-examined, not assumed.

## Related
- [migera moves characters to Bevy's PBR pipeline](./migera-pivot-to-bevy-pbr-for-characters.md) — contrast: the decision that puts meshes into migera and challenges this note's premise.
- [Module and stage skeleton](./module-and-stage-skeleton.md) — applies: the MainPass ordering contract that follows from Option B.
- [Bevy-native integration](./bevy-native-integration.md) — deeper: what Option B still reuses from Bevy.
- [Lighting and shadows](../bevy-rendering/pbr-and-lighting/lighting-and-shadows.md) — deeper: Bevy's shadow-map machinery whose per-view cost drove the decision.
- [Screen-space effect compatibility](../sdf-3dgs-bevy-integration/render-integration/screen-space-effect-compatibility.md) — contrast: what writing into Bevy's buffers would unlock.
