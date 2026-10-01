---
title: Ragdoll body and anchor frames
description: "A ragdoll body's rotation must BE its bone's world rotation, joint anchors live in the BODY's midpoint frame, and an anchor spanning a bodiless bone (arm from chest) must follow it each frame; violations gave 150-180° errors and arms 7-9 cm off. Read before changing ragdoll spawning, body placement or joint anchors."
type: lesson
status: current
tags:
  - ragdoll
  - physics
  - correctness
  - debugging
updated: 2026-10-01
verified: 2026-10-01
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

**3. An anchor across a bone with no body goes stale** (2026-10-01). The
arm hangs from the chest body, because the collarbone has none
(`nearest_simulated_ancestor`). Its anchor in the chest's frame was
computed once at spawn, from the collarbone's spawn pose. When the
collarbone moved, the joint held the arm where it had been: the arm's
bodies stood 7-9 cm off the drawn arm, and a fallen hand rested 7 cm
inside a slope. Symptoms far from the cause hid it:
- any second joint on the arm held a still arm 6° off;
- hand bodies sent character.glb's worst body to 100°.

`publish_joint_targets` now re-anchors every joint that spans a bodiless
bone each pinned frame, limit-only joints included. Bodies sit 1.0-1.5 cm
from their segments (`every_body_stands_on_its_drawn_segment`).

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
- A joint whose bodies are not parent and child spans bones that move
  without physics. Re-derive its anchor from the live bones while the
  body is pinned.
- Test where bodies stand against the drawn skeleton, not only their
  rotations: a 7 cm offset read as a few degrees of rotation error.

## Related

- [Prefer BRP over prints for live ECS state](../../engineering-practice/debugging/prefer-brp-over-prints-for-live-ecs-state.md) — applies: the method that localized this in minutes.
- [PD damping has an explicit-integration bound](./pd-damping-explicit-integration-bound.md) — contrast: a real gain problem, unlike this frame problem.
- [Conjugate pose deltas by the bind rotation](../rig-and-retargeting/conjugate-pose-deltas-by-the-bind-rotation.md) — same-trap: right maths in the wrong frame, in retargeting.
- [Two-bone IK pivots at the upper joint](../ik-and-locomotion/two-bone-ik-pivots-at-upper-not-root.md) — same-trap: right computation against the wrong reference point.
- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — deeper: frame defects that were still hidden after this fix.
- [A falling body is hinged and fleshed](./a-falling-body-is-hinged-and-fleshed.md) — applies: the shoulder cone and hand bodies the stale anchor had blocked.
