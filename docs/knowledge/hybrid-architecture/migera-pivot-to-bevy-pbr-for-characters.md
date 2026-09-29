---
title: migera renders characters with Bevy's PBR pipeline, not the hybrid renderer
description: Since 2026-09-21 characters use standard Bevy meshes, PBR, glTF and skinning; the from-scratch src/hybrid SDF renderer is no longer the character-rendering target and is kept for effects and procedural world generation. Read before starting any character or rendering work, or choosing which pipeline a feature targets.
type: decision
status: current
tags:
  - hybrid-renderer
  - character-animation
  - bevy
  - sdf
  - render-pipeline
updated: 2026-09-21
verified: 2026-09-28
code:
  - src/character
  - examples/character_gallery.rs
  - src/hybrid
  - src/sdf
  - src/raymarch
sources:
  - Claude memory migera_pivot_to_bevy_pbr_pipeline (2026-09-21)
  - commit 567f5f7
aliases:
  - character pivot
  - Bevy PBR for characters
  - HybridRenderPlugin scope
---

# migera renders characters with Bevy's PBR pipeline, not the hybrid renderer

On 2026-09-21 the user decided to stop building the from-scratch GPU renderer
(`src/hybrid`) as the target for character rendering. **Characters use the
standard Bevy pipeline**: `Mesh3d`, `StandardMaterial`, glTF, skinned meshes,
`Transform` hierarchies, `DefaultPlugins`-based examples. The SDF/raymarching
work (`src/sdf`, `src/raymarch`, `src/hybrid`) is kept and will be reused for
specific **effects and procedural world generation**, not as the primary
renderer.

## Context

- `src/hybrid` is a self-shading SDF renderer: SDF-only scenes, hand-rolled
  WGSL shading, BVH, DDGI, reflections. Skinned, animated characters would
  have required reinventing skinning and mesh rendering inside it.
- The decision followed a review of `botica`, a Bevy game with a character
  animation system. The user wants procedural character animation built on the
  ordinary Bevy pipeline.

## Decision

- New character and animation work targets plain `bevy_pbr`/`bevy_gltf`/
  `bevy_animation` primitives and plain `DefaultPlugins` examples.
- `HybridRenderPlugin`, `SdfSceneRoot` and the SDF `Shape`/`Material`
  components stay on the SDF effects / world-generation track. They are not
  used for characters.
- `examples/character_gallery.rs` is the character counterpart of
  `examples/gallery.rs`: CLI-parsed resource configs, an egui Controls panel,
  an on-screen HUD, and the same `--shot`/`--at-frame` headless-screenshot
  convention. It grows one CLI flag and resource per feature.

## Alternatives considered

- **Keep rendering characters in `src/hybrid`.** Lost: it would mean building
  skinning, mesh interop and an animation pipeline from scratch in a renderer
  whose core premise is an SDF-only scene.
- **Hybrid rendering for everything, with meshes composited in.** Not pursued:
  it reopens the G-buffer question the renderer settled for SDF-only scenes
  (see [Self-shading vs. G-buffer-writer](./self-shading-vs-gbuffer-decision.md)).

## Consequences

- The first milestone was `src/character/skeleton.rs`: a Mixamo-compatible
  humanoid bone hierarchy (following botica's naming) spawned as a plain
  `Transform`/`ChildOf` hierarchy with debug capsules and per-bone markers.
- The same day, a physics-driven "muscle" rig went through several
  architectures: a hand-rolled spring solver, then avian3d `RevoluteJoint`
  motors (commit 567f5f7), then a Lugaru-style position-space mass-spring
  solver with a keyframe-driving layer. That whole module was later replaced by
  the rotation-space `src/character/anim` stack (commit 9981e16). Its history
  is in [The muscle module was deleted](../character-animation/animation-core/muscle-deleted-anim-is-the-only-stack.md)
  and [Lugaru joint/muscle system](../character-animation/lugaru-joint-muscle-system.md).
- Two debugging lessons came out of this milestone: an invisible egui panel
  caused by Bevy's shadow-view camera capturing the primary egui context, and
  a wrong "it was visible in my screenshots" claim. See Related.
- The hybrid renderer keeps its own progress log (`PROGRESS.md`) and remains
  valid for SDF scenes. Its "SDF-only forever" premise now holds only inside
  its own scenes.

## Revisit when

- An SDF effect or SDF world has to share a frame with character meshes
  (depth, shadows, GI between them). That is the point at which the
  self-shading decision must be re-examined.

## Related
- [botica character animation system](../character-animation/botica-character-animation-system.md) — prerequisite: the reference system that prompted the pivot and whose skeleton standard migera follows.
- [Character animation](../character-animation/INDEX.md) — applies: where character work lives now.
- [Self-shading vs. G-buffer-writer decision](./self-shading-vs-gbuffer-decision.md) — contrast: its SDF-only premise is what this decision narrows.
- [Bisect your own code before grepping dependency source](../engineering-practice/debugging/bisect-before-grepping-dependency-source.md) — example: the egui-panel bug found during this milestone.
- [Verify, don't assert from memory](../engineering-practice/debugging/verify-dont-assert-from-memory.md) — example: the wrong "I saw it earlier" claim from the same investigation.
- [SDF + 3DGS + Bevy integration](../sdf-3dgs-bevy-integration/INDEX.md) — applies: research on using SDFs as a world model inside Bevy, the track the SDF work continues on.
