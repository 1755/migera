---
title: Foot IK on uneven ground has two feedback loops
description: "Ground-adaptive foot IK has a within-frame and a cross-frame positive-feedback loop, plus a proxy-rig bug that swung legs 72° on slopes; sample ground from the animated pose, keep the correction out of spring state, solve on the real rig, preserve the toe-joint offset. Read before changing foot IK or grounding."
type: lesson
status: current
tags:
  - ik
  - locomotion
  - correctness
  - debugging
updated: 2026-09-24
code:
  - src/character/anim/plugin.rs
  - src/character/anim/legik.rs
  - src/character/anim/rig.rs
  - src/character/anim/ground.rs
aliases:
  - solve_foot_ik
  - AnimFootIk
  - foot grounding
  - slope lunge
---

# Foot IK on uneven ground has two feedback loops

Ground-adaptive foot IK has two positive-feedback loops, and both pass every
unit test while looking visibly broken. Fixing both was not enough: the
decisive bug was solving on a proxy rig instead of the real one.

## What happened

1. **Within-frame loop.** Sampling the ground from the in-progress solve
   means raising a target moves the foot forward, which samples higher
   ground, which raises the target again. Fix: sample once, from the
   *animated* pose, before any IK runs.
2. **Cross-frame loop.** Writing the IK correction into the spring state
   (`AnimPose::state`) makes the next frame's "animated" pose already
   corrected, so the same loop runs one frame at a time. Fix: keep the
   corrected pose in a separate field, `AnimFootIk::corrected`, that only
   the write-back reads.
3. **Solving on a proxy rig (the decisive one).** The IK ran in the
   synthetic T-pose space (`Bone::t_pose_offset`), but ground height belongs
   to the real world. The synthetic rig puts a toe at `z = -0.168`, the real
   `puppet_base.gltf` at `z ≈ 0.088`. On flat ground both sample the same
   height and the error cancels. On a slope height depends on `z`, so the
   solver lifted the foot 0.06 m it never needed and, with the hip fixed,
   could only get there by swinging the leg 72° forward. Fixed with
   `rig::RigGeometry`, `forward_kinematics_on` and `legik::solve_leg_on`.

Both loops are fixed in `anim::plugin::solve_foot_ik`.

**The toe JOINT is not the contact point.** On this rig `LeftToeBase` sits at
y = -0.02 in the bind pose, inside the foot. Forcing it to the surface lifted
the whole leg by that offset every frame and produced a visible forward
lunge. Read the offset from the bind pose (`rest_toe_height`) and keep it.

## Why it matters

Every unit test passed throughout, because they all measured the synthetic
rig, which was the thing that was wrong. Diagnosis needed BRP measurements of
the live rig.

## How to apply

- Anything solved against the WORLD uses the real rig's offsets and bind
  rotations. Poses stay rig-independent; only world-space computations need
  real geometry.
- Sample the environment from the animated input, never from the solve's own
  output.
- Never write a correction back into the state the next frame reads as
  input.
- Find the real contact point from the bind pose instead of assuming a joint
  is on the sole.

## Evidence

Slopes and flat ground were both verified as of Phase 5.

## Related

- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — prerequisite: the stance that gives foot IK room to work.
- [Synthetic-rig tests are blind to retargeting](../rig-and-retargeting/synthetic-rig-tests-are-blind-to-retargeting.md) — same-trap: tests on the synthetic proxy could not see the proxy was wrong.
- [Rig authored at critical extension](./rig-authored-at-critical-extension.md) — deeper: the reach shortfall pelvis adaptation must allow for.
- [Prefer BRP over prints for live ECS state](../../engineering-practice/debugging/prefer-brp-over-prints-for-live-ecs-state.md) — applies: how the proxy-rig bug was found.
- [Full-strength read-back hides the physics](../ragdoll-and-physics/full-strength-readback-hides-the-physics.md) — applies: the ragdoll blends from `AnimFootIk::corrected` for the same reason.
