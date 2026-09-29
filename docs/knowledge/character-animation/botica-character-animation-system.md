---
title: botica's character animation system
description: "Describes botica, a separate Bevy 0.18 project with an Overgrowth-style keyframe+IK+procedural animation stack, and the Mixamo-compatible humanoid skeleton standard migera's rig follows, plus mirror/axis bugs caught porting it. Read before changing bone names, axes or rest rotations, or when looking for prior art."
type: reference
status: current
tags:
  - character-animation
  - prior-art
  - rig
  - bevy
  - correctness
updated: 2026-09-21
code:
  - src/character/skeleton.rs
sources:
  - /home/sergey/Projects/Codennel/botica/branches/master/docs/ANIM_PLUGINS.md
  - botica research/ANIMATION_SYSTEM.md
  - botica research/MODEL_STANDARD.md
aliases:
  - Mixamo skeleton standard
  - MODEL_STANDARD.md
  - Overgrowth-style animation
  - humanoid bone names
---

# botica's character animation system

`/home/sergey/Projects/Codennel/botica` is a separate Bevy 0.18 game monorepo
(a bare repo plus a worktree at `branches/master`). Its
`crates/botica_shared/src/animation/` holds a mature Overgrowth-style hybrid
(keyframe + IK + procedural) character animation system. Its
Mixamo-compatible humanoid skeleton standard is the one migera's own rig
follows.

## What botica has

- Pose core: `HashMap<String, BoneTransform>` poses and RON assets.
- Skeleton extraction from glTF `SkinnedMesh`.
- IK solvers: two-bone, FABRIK, aim/look-at, soft/spring.
- Distance-based locomotion, active-ragdoll blending, hit reactions,
  ledge-grab and LOD.
- All of it as a 15-plugin, 6-stage Bevy plugin pipeline. Docs are in
  `branches/master/docs/ANIM_PLUGINS.md`; design rationale in
  `research/ANIMATION_SYSTEM.md`.

It does **no GPU skinning of its own**. It only writes CPU-side `Transform`s
on bone entities, and Bevy's standard `bevy_pbr` vertex-shader skinning does
the rest. So there is no GPU-skinning code to port, only architecture and
data-structure ideas.

The only real known gap as of 2026-09-21: walk/run/sprint blending is not
smooth yet (TODO at `animation/locomotion/systems.rs:439`). Everything else
is built and tested, with unit tests per module and demos under
`games/botica/examples/animation/`.

botica's ragdoll uses discrete `RagdollMode::{Animated, Blending, Active,
Passive}` states and a separate directional-flinch `hit_reaction` module.
Lugaru, by contrast, uses a continuous per-constraint strength dial (see
[Lugaru's joint/muscle system](./lugaru-joint-muscle-system.md)).

## The standard humanoid skeleton

From `research/MODEL_STANDARD.md`:

- Mixamo-compatible glTF 2.0 binary, Y-up, -Z forward, T-pose rest.
- PascalCase bone names with the `mixamorig:` prefix stripped.
- 15 required bones: Hips, Spine, Spine1, Neck, Head,
  {Left,Right}Arm/ForeArm/Hand, {Left,Right}UpLeg/Leg/Foot. Commonly
  included optional bones: Spine2, {Left,Right}Shoulder, {Left,Right}ToeBase.
- Bone forward is local +Y (parent → child). The head's face looks along
  local +Z.

This convention also matches Unity Humanoid and the UE Mannequin. migera's
procedural skeleton (`src/character/skeleton.rs`) follows it, so a real
skinned glTF character can be swapped in without changing animation code.

## Bugs caught while porting (2026-09-21)

**Left and right were mirrored.** The standard defines `+X` as the
character's right (`MODEL_STANDARD.md` line 61). migera's first
implementation put `LeftArm`, `LeftUpLeg` and so on at `+X`. It was caught by
deriving character-right from the forward marker with the right-hand rule,
`forward × up = (0,0,-1) × (0,1,0) = (+1,0,0)`, and checking that against
where the Left/Right-tinted bones rendered. Fixed by swapping every Left/Right
X sign. The mirrored rig still looked like a valid symmetric T-pose, so a
static screenshot alone does not catch this. **Whenever a rig defines both a
forward marker and a left/right convention, cross-check them against each
other and against the standard's explicit axes.**

**Checked against real Mixamo data** (local rig files and a web search, not
only `MODEL_STANDARD.md`):

- migera's 22 bone names and hierarchy match Mixamo's core chain exactly.
- Toes pointed backward: `LeftToeBase`/`RightToeBase` were offset toward
  `+Z` on a `-Z`-forward rig. Fixed to `-Z`.
- Proportions summed to only about 1.48 m, although the code comment and the
  standard both claimed about 1.8 m. Rebalanced to 1.78 m: Hips 0.94 (thigh
  0.45 + shin 0.42 + ankle-to-sole 0.07) + torso 0.50 + neck 0.10 +
  head 0.24.
- Intentionally not fixed: migera's bone set is a subset of full Mixamo
  (about 65 bones), with no `Toe_End`, finger or eye bones.
- migera's procedural rig has identity rest rotations (translation-only
  offsets). Real Mixamo and DCC exports bake non-trivial per-joint
  pre-rotations (confirmed in `botica/references/bevy_mod_inverse_kinematics/assets/skin.gltf`
  and `botica/references/ozz-animation/media/collada/pab/skeleton.dae`). The
  real `puppet_base.gltf` rig migera now renders has such binds, which is why
  retargeting needs bind-frame conjugation.

## Relevance to migera

The skeleton standard is live: migera's rig follows it. botica's plugin
architecture is reference material only; migera's `src/character/anim`
is its own rotation-space design.

## Related

- [Lugaru's joint/muscle system](./lugaru-joint-muscle-system.md) — contrast: continuous strength dial versus botica's discrete ragdoll modes.
- [migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — prerequisite: why migera adopted a standard skinned rig.
- [Conjugate pose deltas by the bind rotation](./rig-and-retargeting/conjugate-pose-deltas-by-the-bind-rotation.md) — applies: what the non-identity binds of a real rig require.
- [Synthetic rig's leg segments are shifted a joint](./rig-and-retargeting/synthetic-rig-leg-segments-are-shifted-a-joint.md) — deeper: the 0.45/0.42/0.07 leg split above, measured against the real rig.
- [The muscle module is deleted](./animation-core/muscle-deleted-anim-is-the-only-stack.md) — applies: the stack that now drives this skeleton.
