---
title: Rig and retargeting
description: "How rig-independent pose rotations map onto the real glTF rig: bind-frame conjugation, world-axis deltas, the synthetic rig's blind spots and leg-segment shift, and facing/sign conventions. Read before touching rig.rs, retarget.rs, stance.rs, or anything that composes rotations or assumes a facing."
type: index
status: current
tags:
  - rig
  - retargeting
  - math
  - correctness
updated: 2026-10-09
---

# Rig and retargeting

migera authors poses against a synthetic T-pose rig whose bind rotations are
all identity, then renders them on real glTF rigs (`puppet_base.gltf`) with
non-trivial binds, different segment lengths and a different facing. Every
note here is a way that gap produced a bug the test suite could not see.

## Start here

1. [Synthetic-rig tests are blind to retargeting](./synthetic-rig-tests-are-blind-to-retargeting.md) — why the gap is invisible to tests.
2. [Conjugate pose deltas by the bind rotation](./conjugate-pose-deltas-by-the-bind-rotation.md), then [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) — the maths.
3. The two leg notes for geometry and sign.

| Note | What it establishes | Read when |
|---|---|---|
| [The live rig geometry must match the rendered rig](./live-rig-geometry-must-match-the-rendered-rig.md) | Bone offsets in metres (× the armature's 0.01 on Mixamo rigs) and the hips at the rig's own rest; `character.glb` never walked without both, and `puppet_base` hid them | before building rig geometry, adding a rig, or trusting one rig's results |
| [Synthetic-rig tests are blind to retargeting](./synthetic-rig-tests-are-blind-to-retargeting.md) | `rig::forward_kinematics` never reads bind rotations, so it cannot catch retargeting bugs | before trusting a synthetic-rig test for real-rig output |
| [Conjugate pose deltas by the bind rotation](./conjugate-pose-deltas-by-the-bind-rotation.md) | Write `bind⁻¹·delta·bind`, or the angle lands on the wrong axis; sabotage your regression test | before writing pose rotations to a glTF rig |
| [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) | FK conjugates by the accumulated bind; world corrections use `P = W(parent)·bind_local·B⁻¹` (026d9e8) | before composing rotations in `rig.rs`, `legik`, `armik` or `lookat` |
| [Rotations turned over and over in one frame need renormalizing](./rotations-turned-over-and-over-in-a-frame-need-renormalizing.md) | `delta_after_world_turn` was unnormalized; re-posed through passes, a foot's norm compounded and FK grew it 0.7 % in a frame. Now normalized at the source; that first flipped a ladder whose choice hung on float noise (fixed) | before composing rotations many times a frame, or when a held point drifts while its joint reads exact |
| [A clip's world positions carry its rig's bind shape](./a-clips-positions-carry-its-rigs-bind-shape.md) | Converted from the straight T-pose, a clip stores its source rig's bind curvature as a bend, and the target bends its own bind by it again: `relaxed_stand`'s back was over-arched. Convert against the source's bind; stances are balanced over the feet | before importing, rebasing or judging a reference pose |
| [A pose delta's world is the character's frame](./a-pose-deltas-world-is-the-characters-frame.md) | Deltas turn with the character; convert in the bind-rooted rig and apply the turn at the scene boundary; the rest pose can't catch it | before converting pose deltas to or from scene rotations (ragdoll targets, read-back) |
| [Bind a rig at its own facing, not its spawn heading](./bind-a-rig-at-its-own-facing-not-its-spawn-heading.md) | Binding reads the hips' parent world rotation; spawned turned, it captured the heading and `relaxed_stand` held its hands overhead; bind with the facing correction, without the heading | before binding a rig spawned at any yaw, or adding a spawn path |
| [Synthetic rig's leg segments are shifted a joint](./synthetic-rig-leg-segments-are-shifted-a-joint.md) | Synthetic `LeftLeg` is a 0.07 m stub vs a 0.459 m real shin; knee bend 6x less visible | before judging leg shape on a preview, or naming segments by bone name |
| [Knee axis positive swings forward](./knee-axis-positive-swings-forward.md) | `+KNEE_AXIS` swings toward `-Z`; `puppet_base` faces `+Z`; never hardcode a facing | before code that assumes a facing or rotation sign, or any leg-direction test |
| [Lengthen a segment by moving its joint and scaling only its skinning](./lengthen-a-segment-by-its-joint-and-a-skinning-only-scale.md) | Move the child joint and skin the segment through a helper joint scaled along the bone; moving the joint alone stretches knee triangles 2.7×, and a scaled parent shears its children | before building body proportions or rescaling a bone |
| [The puppet_base fixture faces away from the rendered character](./puppet-base-fixture-faces-away-from-the-rendered-character.md) | World-axis-authored poses measure wrong on `puppet_base()` (hands a metre up); use `puppet_base_as_rendered()` | before measuring an authored pose's shape, posture or centre of mass on the real rig |

## See also

- [botica's character animation system](../botica-character-animation-system.md) — the Mixamo skeleton standard both rigs follow.
- [Parse the asset, don't transcribe it](../../engineering-practice/testing/parse-the-asset-dont-transcribe-it.md) — how tests read real-rig numbers.
- [Symptom is far from cause in the rig chain](../../engineering-practice/debugging/symptom-is-far-from-cause-in-the-rig-chain.md) — debugging method for frame bugs.
