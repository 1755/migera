---
title: Ragdoll body and anchor frames
description: "A ragdoll body's rotation must BE its bone's world rotation, and joint anchors must be expressed in the BODY's midpoint frame, not the bone's; violating either gave 150-180° tracking error with correct PD targets. Read before changing ragdoll spawning, body placement or joint anchors."
type: lesson
status: current
tags:
  - ragdoll
  - physics
  - correctness
  - debugging
updated: 2026-09-25
code:
  - src/character/anim/ragdoll_plugin.rs
aliases:
  - body_world_position
  - joint anchors
  - capsule_endpoints
---

# Ragdoll body and anchor frames

`character::anim::ragdoll_plugin` has two frame invariants. A body's rotation
IS its bone's world rotation. Joint anchors are in the BODY's frame, whose
origin is the segment midpoint. Both were violated at first, and together
they produced 150–180° of tracking error while the published PD targets were
provably correct to 0.0°.

## What happened

**1. Body rotation.** `publish_joint_targets` drives each body toward its
bone's world rotation. Any other body frame means "tracking the target"
holds a pose the bone does not have. Orienting the body along its own
*segment* is wrong for any bone whose bind rotation differs from its
parent's: measured 93° on knees, 46° on ankles, about 40° on shoulders.
Spine and arms happened to agree, which made it hard to spot. The fix builds
the capsule from explicit `Collider::capsule_endpoints`, so the *collider*
carries the alignment.

**2. Anchor frame.** A body sits at its segment's **midpoint**, half a bone
from its bone's origin. Computing anchors with
`bone_global.affine().inverse()` put every joint half a bone off. Both passes
of `spawn_ragdoll` now derive the centre from one shared
`body_world_position` helper, so they agree by construction.

## Why it matters

The diagnosis was quick because of BRP on the live ECS. The kinematic,
pinned `Hips` matched its target *exactly* while `Spine`, one joint down,
did not. That cleared `publish_joint_targets` at once and moved the search
downstream. `JointTarget` and `Bone` are `Reflect` specifically to keep this
possible.

Measurement killed three plausible hypotheses first: gravity (the error
persisted with it off), torque ceiling (64× more torque reached only 93°),
and joint limits (154° error without any limits).

## How to apply

- Keep body rotation equal to bone world rotation; put geometric alignment
  in the collider.
- Compute every anchor in the body's own frame, from the same helper that
  places the body.
- When a controller "can't track", check the frames before the gains.

## Related

- [Prefer BRP over prints for live ECS state](../../engineering-practice/debugging/prefer-brp-over-prints-for-live-ecs-state.md) — applies: the method that localized this in minutes.
- [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) — contrast: a real gain problem, unlike this frame problem.
- [Conjugate pose deltas by the bind rotation](../rig-and-retargeting/conjugate-pose-deltas-by-the-bind-rotation.md) — same-trap: right maths in the wrong frame, in retargeting.
- [Two-bone IK pivots at the upper joint](../ik-and-locomotion/two-bone-ik-pivots-at-upper-not-root.md) — same-trap: right computation against the wrong reference point.
- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — deeper: frame defects that were still hidden after this fix.
