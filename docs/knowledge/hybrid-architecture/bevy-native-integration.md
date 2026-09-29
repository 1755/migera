---
title: Bevy-native integration
description: Convention for src/hybrid's relation to Bevy — extract from real Bevy components (camera, DirectionalLight/PointLight/SpotLight) using bevy_pbr's own conversion formulas, use Bevy's post-process extension points downstream, and hand-roll only what Bevy has no component for. Read before adding extraction or a post pass.
type: decision
status: current
tags:
  - hybrid-renderer
  - bevy
  - integration
  - lighting
  - render-pipeline
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/hybrid/extract.rs
  - src/hybrid/pass.rs
  - src/hybrid/post.rs
  - src/prepass_probe
aliases:
  - extract lights
  - ExtractedView
  - FullscreenMaterialPlugin
---

# Bevy-native integration

Convention for how `src/hybrid` should relate to Bevy's own components
and systems: default to reusing real Bevy components as extraction input
wherever one exists for the data needed, and default to Bevy's own
extension points for anything downstream of `hybrid`'s own output. Reserve
fully custom/hand-rolled representations for data Bevy genuinely has no
component for (BVH nodes, SDF primitive records, material fields), or for
cases where reuse would concretely cost performance or quality — no such
case has been identified anywhere in this project so far.

Status as of 2026-09-28: `src/hybrid_legacy`, which the sections below cite
as precedent, has been deleted. Both concrete points were carried into the
rewrite: `src/hybrid/pass.rs` reads Bevy's `ExtractedView`/`ViewUniformOffset`,
and `src/hybrid/extract.rs` now queries `DirectionalLight`, `PointLight` and
`SpotLight` with Bevy's own intensity conversions, which closes the light gap
described below.

## Camera: already correct, no gap

View/projection data needs no custom extraction. `hybrid_legacy::pipeline
::prepare_hybrid_scene` derives `view_proj`/`tan_half_y` entirely from
Bevy's real `ExtractedView` (`clip_from_view`, `world_from_view`), and its
group-0 bind group is literally Bevy's own `ViewUniform`/`ViewUniforms`,
bound the same way Bevy's stock passes bind it. This is already the
correct pattern and should carry forward unchanged into `src/hybrid`'s
own `pipeline.rs` when it gets built.

## Lights: a real, documented gap — not just non-native

`hybrid_legacy::extract::extract_light` queries Bevy's real
`&DirectionalLight` + `&GlobalTransform` — so its *input* is already
Bevy-native (`.color`, `.illuminance`, `transform.forward()`) — but the
query is `DirectionalLight`-only. `PointLight`/`SpotLight` entities are
invisible to it entirely, not partially supported: the hand-rolled
`LightGpu` output's `kind` field exists for point/spot lights but is
never populated by anything.

There is **no publicly-reusable shortcut** to close this gap by reading
Bevy's own light-list data structure directly: `bevy_pbr`'s clustered/
GPU-driven light list is assembled deep inside its own render-world
systems, in a layout private to its own shaders, with no stable public
bind group a custom pipeline could read from (confirmed against the
bevy-rendering knowledge base's clustered-forward-rendering
documentation, which documents the algorithm in detail but never exposes
or claims an externally-consumable buffer). That channel simply doesn't
exist for a custom pipeline to tap into.

`src/prepass_probe::extract_probe_lights` already demonstrates the right
pattern to use instead — match Bevy's *conventions*, not its internal
buffers: query Bevy's real `PointLight` component directly (a legitimate
"aggregate many entities into one resource" extraction pattern, per the
bevy-rendering knowledge base's own extraction-patterns documentation),
and convert its values using the *same formulas* `bevy_pbr`'s own
extraction uses internally — e.g. `PointLight::intensity`'s raw-lumens ->
luminous-intensity conversion, `intensity / 4π` — so results stay
physically consistent with what Bevy's own lighting would produce, even
though the extraction code itself is separately hand-written.

**The concrete fix for `src/hybrid/extract.rs`, when it gets real
content:** query all three real Bevy light component types
(`DirectionalLight`, `PointLight`, `SpotLight`), not directional-only,
and populate `kind`/`range`/`inner_angle`/`outer_angle` from each light
type's real fields using Bevy's own conversion formulas as the reference.
This is a completeness fix to an already-legitimate extraction pattern,
not a new mechanism or an architecture change.

## General principle for future features

- If Bevy has a real component carrying the data a feature needs (camera,
  lights, transforms), extract from that component directly — don't
  invent a parallel hand-rolled representation for data Bevy already
  models.
- If Bevy has a post-process extension point that fits (
  `FullscreenMaterialPlugin` registering into `EarlyPostProcess`/
  `PostProcess`), use it for anything downstream of `hybrid`'s own blit —
  this is available regardless of which shading architecture `hybrid`
  uses internally (see the
  [self-shading vs. G-buffer-writer decision](./self-shading-vs-gbuffer-decision.md),
  whose Option A vs. B choice does *not* gate access to Bevy's
  post-process pipeline either way).
- If Bevy has no component and no extension point for something
  (BVH nodes, SDF primitive/CSG records, per-material procedural pattern
  parameters, future ML-derived intermediate buffers), a fully custom
  representation is the right call — there's nothing to reuse, and
  Option B's architecture is chosen partly *because* it keeps this door
  open without a `bevy_pbr`-shaped constraint in the way.

## Related
- [Self-shading vs. G-buffer-writer decision](./self-shading-vs-gbuffer-decision.md) — prerequisite: the architecture choice this convention sits inside.
- [Module and stage skeleton](./module-and-stage-skeleton.md) — applies: where extraction and post-processing sit in `src/hybrid`.
- [Light components and atmosphere](../bevy-rendering/pbr-and-lighting/light-components-and-atmosphere.md) — deeper: Bevy's light components and units that the extraction converts.
- [Clustered forward rendering](../bevy-rendering/pbr-and-lighting/clustered-forward-rendering.md) — contrast: why Bevy's light list is not reusable from a custom pipeline.
- [Entity sync and extraction patterns](../bevy-rendering/architecture/entity-sync-and-extraction-patterns.md) — deeper: the "aggregate many entities into one resource" extraction pattern.
- [migera moves characters to Bevy's PBR pipeline](./migera-pivot-to-bevy-pbr-for-characters.md) — contrast: for characters, migera now uses Bevy's pipeline wholesale instead of integrating with it.
