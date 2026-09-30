---
title: Synthetic rig's leg segments are shifted a joint
description: "The synthetic T-pose rig puts a 0.45 m segment in Hips→LeftUpLeg and a 0.07 m stub under LeftLeg, while puppet_base.gltf has a 0.459 m shin there, so 42° of knee flexion moves the foot 6x less. Read before judging leg shape on any preview or naming leg segments by bone name."
type: lesson
status: current
tags:
  - rig
  - locomotion
  - verification
  - correctness
updated: 2026-09-26
code:
  - src/character/anim/rig.rs
  - src/character/anim/stance.rs
  - src/character/anim/legik.rs
  - assets/models/puppet_base.gltf
aliases:
  - leg bone names off by one joint
  - LeftLeg stub
---

# Synthetic rig's leg segments are shifted a joint

The synthetic T-pose rig and the real glTF rig disagree about where the leg
joints are, and only the real rig is anatomically right. On the synthetic
rig the knee bone carries a 0.07 m stub instead of a shin, so knee flexion
is almost invisible there.

## What happened

| Segment | Synthetic (`t_pose_offset`) | Real (`puppet_base.gltf`) |
|---|---|---|
| `Hips`/`pelvis` → `LeftUpLeg`/`thigh_l` | **0.45** | 0.114 |
| `LeftUpLeg` → `LeftLeg`/`calf_l` | 0.42 | **0.429** (femur) |
| `LeftLeg` → `LeftFoot`/`foot_l` | **0.07** | **0.459** (shin) |
| `LeftFoot` → `LeftToeBase`/`ball_l` | 0.14 | 0.159 |

On the **real** rig the names mean what they say: `LeftUpLeg` is the hip,
`LeftLeg` is the knee, with a 0.459 m shin below it. The "hip, knee, ankle"
reading in `stance.rs` and `legik.rs` is correct there.

On the **synthetic** rig the 0.45 goes into `Hips → LeftUpLeg`, which is far
too long for a hip-socket offset, and `LeftLeg → LeftFoot` is a 0.07 m stub.
Rotating `LeftUpLeg` still swings the whole leg correctly on both rigs. But
rotating `LeftLeg` bends 0.459 m of shin on the real rig and 0.07 m of stub
on the synthetic one.

Measured: 42° of knee flexion moves the ankle 0.329 m on the real rig and the
sole only 0.050 m on the synthetic one, a **6x** difference. A correct walk
cycle renders as a straight, unbent leg in any preview built on the
synthetic rig. That is what happened while building `gait.rs`, and it cost
several wrong diagnoses.

## Why it matters

Bone names are labels, not geometry. Two rigs using the same names can put
the length in different segments.

## How to apply

- Verify leg poses on a REAL rig (`--real-mesh` in `character_gallery`, or
  `puppet_base.gltf`), never on the synthetic T-pose.
- The synthetic rig is fine for rig-independent properties (rotations,
  symmetry, bone-length invariance). It is actively misleading for visible
  leg shape.
- Read segment lengths from the data, never from bone names.
- The same goes for leg IK. Folding to ground raised 25 cm under a body
  held still, the synthetic leg's two-bone chain (its `LeftUpLeg` at the
  knee, a 0.07 m ankle stub as the "shin") flipped the stub 180° and
  pitched the foot, heel ~13 cm under the floor. On `puppet_base` the
  whole sole rests on the plane (`plugin::tests::a_real_foot_stands_whole_on_raised_ground`,
  on the real-rig fixture `app_with_real_rig`).

## Related

- [Bind-pose zero leg slack is normal](../ik-and-locomotion/bind-pose-zero-leg-slack-is-normal.md) — same-trap: the "LeftUpLeg sits at the knee" naming trap from the entity side.
- [Rig authored at critical extension](../ik-and-locomotion/rig-authored-at-critical-extension.md) — applies: the synthetic femur/shin numbers this table explains.
- [Synthetic-rig tests are blind to retargeting](./synthetic-rig-tests-are-blind-to-retargeting.md) — same-trap: another way the synthetic proxy misleads.
- [Parse the asset, don't transcribe it](../../engineering-practice/testing/parse-the-asset-dont-transcribe-it.md) — applies: read real-rig segment lengths from `puppet_base.gltf` in tests.
- [Knee axis positive swings forward](./knee-axis-positive-swings-forward.md) — example: the walk-cycle direction bug found in the same work.
