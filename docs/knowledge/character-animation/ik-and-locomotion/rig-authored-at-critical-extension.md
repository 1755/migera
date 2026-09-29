---
title: Rig authored at critical extension
description: "The synthetic rig's legs are authored at exactly full extension, so on flat ground every reach calculation carries a permanent 0.0108 m shortfall; PelvisConfig::reach_margin must be a deadband (0.02), never a scale. Read before adding any 'limb is straining' check or pelvis adaptation."
type: lesson
status: current
tags:
  - ik
  - rig
  - locomotion
  - correctness
updated: 2026-09-25
code:
  - src/character/anim/pelvis.rs
  - src/character/anim/legik.rs
sources:
  - test pelvis::tests::an_ordinary_stance_sits_just_inside_full_extension
aliases:
  - reach_margin
  - reach deadband
  - LegChain offsets
---

# Rig authored at critical extension

The synthetic rig's legs are authored at **exactly** critical extension, so
every reach calculation carries a permanent **0.0108 m** shortfall on flat
ground. Any "is this leg straining" correction must be a deadband that
clears that baseline.

## What happened

Femur 0.42 + shin 0.07 = 0.49 m. Standing on level ground, the hip socket
sits 0.49 m above an ankle target that the foot's own thickness puts at
y = -0.01. The chain is therefore asked to span **0.5008** m with 0.49 m of
reach, a shortfall present on every surface.

## Why it matters

A ground-adaptation correction phrased as "treat the leg as slightly shorter
to reserve a knee bend" fires on this baseline and lowers the hips forever.
That is the "dinosaur" crouch the foot-locking literature warns about,
reached from the opposite direction.

## How to apply

- `PelvisConfig::reach_margin` is a **deadband**: gate the correction, never
  scale it. It is set at 0.02 to clear the 0.0108 baseline.
- Before adding any strain check, measure the rest-pose shortfall. It is not
  zero.
- **The naming trap:** in `LegChain`, `socket == thigh == LeftUpLeg`, and a
  bone's offset is measured from its PARENT. So the femur is
  `offsets[chain.shin]` and the shin is `offsets[chain.ankle]`.
  `offsets[chain.thigh]` is the hips-to-socket step and is not part of the
  chain.

## Evidence

Pinned by `pelvis::tests::an_ordinary_stance_sits_just_inside_full_extension`.
It fails if a future stance gains real knee bend and the deadband becomes
unnecessary.

## Related

- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — prerequisite: why a bind pose has no slack.
- [Two-bone IK pivots at the upper joint](./two-bone-ik-pivots-at-upper-not-root.md) — same-trap: which offset belongs to the chain.
- [Foot IK on uneven ground has two feedback loops](./foot-ik-feedback-loops.md) — applies: the grounding that pelvis adaptation works with.
- [Synthetic rig's leg segments are shifted a joint](../rig-and-retargeting/synthetic-rig-leg-segments-are-shifted-a-joint.md) — deeper: why the synthetic femur/shin split is 0.42/0.07.
