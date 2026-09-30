---
title: Full-strength read-back hides the physics
description: "The ragdoll read-back shows slerp(animated, simulated, 1-strength), so at strength 1.0 the screen is the animation whatever the bodies do; Stage 4 was declared verified this way while every body was 35-178° off target. Verify body-vs-target error and spin over BRP. Read before calling a ragdoll change verified."
type: lesson
status: current
tags:
  - ragdoll
  - verification
  - testing
  - debugging
updated: 2026-09-28
code:
  - src/character/anim/ragdoll_plugin.rs
  - src/character/anim/ragdoll.rs
sources:
  - commit db8dce5
  - commit 33cd7e3
  - CHARACTER_PROGRESS.md "Hits, and a ragdoll that actually tracks on the real rig"
aliases:
  - ragdoll read-back
  - ragdoll tracking error
  - Stage 4 verification
---

# Full-strength read-back hides the physics

The ragdoll read-back displays `slerp(animated, simulated, 1 - strength)`. At
full strength the screen shows the animation whatever the bodies do, so a
correct-looking character proves nothing about the physics.

## What happened

A progress entry declared Stage 4 verified because "a fully driven one stands
correctly". Measured later over BRP on `puppet_base`, every body was
**35–178°** off its target, with and without gravity.

Commit db8dce5 fixed seven defects, each bisected by measurement: a private
copy of the old rotation convention, self-collision between jointed bodies,
limit cones in a stale frame, shipped poses outside the limits, a PD that
could not see load, a read-back that wrote physics into the spring state, and
the ragdoll switching foot/arm IK off on screen. Afterwards the worst body
was **0.1°, about 0 rad/s**, in three independent runs. Commit 33cd7e3 then
made the pinned root follow a walking, turning character.

## Why it matters

A check that cannot fail is not a check. Here the display path made the
physics invisible by construction, the same family as comparing a function
with itself.

## How to apply

- Verify the ragdoll by querying `JointTarget.target` against avian
  `Rotation` over BRP, and `AngularVelocity` too: a standoff shows as
  constant spin at constant error.
- Measure across several runs. The broken system was chaotic from run to
  run.
- In headless tests assert error AND spin. The real-rig fixture
  `spawn_real_rig_ragdoll` exists for this.
- Draw the cyan gizmo from the collider's real segment.

## Related

- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: a comparison that cannot fail.
- [avian joint limits are not cone and twist](./avian-joint-limits-are-not-cone-and-twist.md) — example: a defect found once the physics was measured.
- [A pose delta's world is the character's frame](../rig-and-retargeting/a-pose-deltas-world-is-the-characters-frame.md) — example: after a turn, the pinned bodies held T-pose arms under a correct picture.
- [Kill stale processes before trusting BRP](../../engineering-practice/debugging/kill-stale-processes-before-trusting-brp.md) — prerequisite: make sure BRP answers from the current build.
- [Ragdoll body and anchor frames](./ragdoll-body-and-anchor-frames.md) — prerequisite: the earlier frame fixes.
- [Gizmos need --show-real-mesh off](../animation-core/gizmos-need-show-real-mesh-off.md) — same-trap: another view that cannot show what is being verified.
- [Lugaru's joint/muscle animation system](../lugaru-joint-muscle-system.md) — deeper: the prior art where the continuous strength dial comes from.
- [Foot IK on uneven ground has two feedback loops](../ik-and-locomotion/foot-ik-feedback-loops.md) — applies: the ragdoll now blends from `AnimFootIk::corrected`.
