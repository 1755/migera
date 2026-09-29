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
updated: 2026-09-29
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
| [Synthetic-rig tests are blind to retargeting](./synthetic-rig-tests-are-blind-to-retargeting.md) | `rig::forward_kinematics` never reads bind rotations, so it cannot catch retargeting bugs | before trusting a synthetic-rig test for real-rig output |
| [Conjugate pose deltas by the bind rotation](./conjugate-pose-deltas-by-the-bind-rotation.md) | Write `bind⁻¹·delta·bind`, or the angle lands on the wrong axis; sabotage your regression test | before writing pose rotations to a glTF rig |
| [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) | FK conjugates by the accumulated bind; world corrections use `P = W(parent)·bind_local·B⁻¹` (026d9e8) | before composing rotations in `rig.rs`, `legik`, `armik` or `lookat` |
| [Synthetic rig's leg segments are shifted a joint](./synthetic-rig-leg-segments-are-shifted-a-joint.md) | Synthetic `LeftLeg` is a 0.07 m stub vs a 0.459 m real shin; knee bend 6x less visible | before judging leg shape on a preview, or naming segments by bone name |
| [Knee axis positive swings forward](./knee-axis-positive-swings-forward.md) | `+KNEE_AXIS` swings toward `-Z`; `puppet_base` faces `+Z`; never hardcode a facing | before code that assumes a facing or rotation sign, or any leg-direction test |
| [The puppet_base fixture faces away from the rendered character](./puppet-base-fixture-faces-away-from-the-rendered-character.md) | World-axis-authored poses measure wrong on `puppet_base()` (hands a metre up); use `puppet_base_as_rendered()` | before measuring an authored pose's shape, posture or centre of mass on the real rig |

## See also

- [botica's character animation system](../botica-character-animation-system.md) — the Mixamo skeleton standard both rigs follow.
- [Parse the asset, don't transcribe it](../../engineering-practice/testing/parse-the-asset-dont-transcribe-it.md) — how tests read real-rig numbers.
- [Symptom is far from cause in the rig chain](../../engineering-practice/debugging/symptom-is-far-from-cause-in-the-rig-chain.md) — debugging method for frame bugs.
