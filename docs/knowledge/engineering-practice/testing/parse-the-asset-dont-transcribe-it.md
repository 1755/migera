---
title: Parse the asset, don't transcribe it
description: Tests should build the rig by parsing assets/models/puppet_base.gltf (plain JSON) through gltf_rig.rs instead of hand-copying its numbers; transcription drifts silently and cannot model what it was not told, such as the foot bound at -69.8 degrees. Read before writing a test that needs real rig or asset data.
type: lesson
status: current
tags:
  - testing
  - rig
  - assets
  - retargeting
updated: 2026-09-26
verified: 2026-09-28
code:
  - src/character/anim/gltf_rig.rs
  - assets/models/puppet_base.gltf
sources:
  - Claude memory parse_the_asset_dont_transcribe_it (2026-09-26)
  - commit 02ca1bd
aliases:
  - gltf_rig
  - puppet_base
  - hand-transcribed test rig
  - real_skeleton
---

# Parse the asset, don't transcribe it

When a test needs to model real data, check whether the real data is
parseable before copying numbers out of it. `src/character/anim/gltf_rig.rs`
builds a `RigGeometry` by parsing `assets/models/puppet_base.gltf` at test
time. Use `puppet_base()` for the whole bind pose, `real_leg_lengths()` for
lengths on an upright synthetic rig, and `real_skeleton` for a real
`HumanoidSkeleton`.

## What happened

- The previous test helper hand-transcribed the rig's numbers. That closes a
  gap the day it is written and drifts silently afterwards.
- It also could not model what it had not been told about. The rig binds its
  foot at **-69.8°**, and the transcribed rig left it at identity. A foot that
  sat flat in every test was visibly pitched in the game. The tilt was
  reported from a screenshot while the suite was green.

**Why parsing is cheap:** a `.gltf` keeps its node hierarchy in plain JSON and
only mesh data in the companion `.bin`. `puppet_base.gltf` is 31 KB with 69
nodes, each with rotation and translation, which is the whole bind pose. It is
readable with `serde_json` (already in the tree through `bevy_gltf`) without
Bevy, the asset server or a GPU. `include_str!` it the way `poses.rs` does.

**Faithfulness check:** the parsed rest-pose foot pitch is **26.6°**, against
**26.1°** measured on the live rig over BRP. Flattening `foot_l`'s rotation in
the asset fails two tests with exact messages, so a change to the asset is
caught.

**The retargeting path is covered too.** `gltf_rig::real_skeleton` builds a
real `HumanoidSkeleton` from the parsed data. An earlier belief that this path
needed the live app was wrong: of the five `HumanoidSkeleton` methods it uses,
only `entity` touches the ECS. "Needs a Bevy `Query`" is not "needs the running
game". A `World` with `MinimalPlugins` + `TransformPlugin` is cheap.

## Why it matters

Transcription is a silent-drift generator. The parse is often a dozen lines.

## How to apply

- Before copying numbers from an asset into a test, check whether the asset
  can be parsed in the test.
- **Pick a delta axis the bind rotation can move.** A conjugation test first
  used a delta about X, which the leg chain's bind rotation is also about.
  Parallel rotations commute, so it passed vacuously. About Y it fails when it
  should.
- **Put the rig's root correction on the root entity.** Offsets and bind
  rotations are both glTF-local. Without the -90° node above them the rig
  renders lying down (head y 0.017, foot y 0.088: correct for Z-up, wrong for
  a Y-up world).
- Remember what still needs the live app: the glTF **loader**. These tests
  verify the maths against the bind pose the file declares, not that
  `bevy_gltf` reproduces it on load.

## Evidence

- Commit 02ca1bd ("parse the real rig in tests instead of transcribing it").
- 26.6° parsed vs 26.1° live foot pitch; two tests fail when `foot_l` is
  flattened.

## Related
- [Synthetic rig's leg segments are shifted a joint](../../character-animation/rig-and-retargeting/synthetic-rig-leg-segments-are-shifted-a-joint.md) — example: what the hand-built rig got wrong about the legs.
- [Synthetic-rig tests are blind to retargeting](../../character-animation/rig-and-retargeting/synthetic-rig-tests-are-blind-to-retargeting.md) — example: the retargeting gap `real_skeleton` closes.
- [Same function on both sides is a vacuous test](./same-function-both-sides-is-a-vacuous-test.md) — same-trap: the commuting-axis conjugation test was vacuous.
- [Prefer BRP over prints for live ECS state](../debugging/prefer-brp-over-prints-for-live-ecs-state.md) — applies: how the parsed values were checked against the live rig.
- [DDGI probe-grid bounds wall-embedding leak](../../hybrid-architecture/gi-and-lighting/ddgi-probe-grid-bounds-wall-embedding-leak.md) — same-trap: CPU tests used hand-approximated bounds where the real pipeline used the real ones.
