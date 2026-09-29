---
title: Synthetic-rig tests are blind to retargeting
description: "rig::forward_kinematics walks the synthetic T-pose's t_pose_offset chain and never reads HumanoidSkeleton rest rotations, so it cannot catch a retargeting bug; relaxed_stand passed every test while raising both arms on puppet_base.gltf. Read before trusting a pose test for real-rig output."
type: lesson
status: current
tags:
  - retargeting
  - rig
  - testing
  - correctness
updated: 2026-09-24
code:
  - src/character/anim/rig.rs
  - src/character/anim/retarget.rs
  - src/character/skeleton.rs
aliases:
  - synthetic T-pose rig
  - t_pose_offset
  - rest_rotation
---

# Synthetic-rig tests are blind to retargeting

`character::anim::rig::forward_kinematics` walks `Bone::t_pose_offset`
chains and never reads the skeleton's rest (bind) rotations. So it **cannot
detect a retargeting bug**. A pose can be perfect there and render
completely wrong on the real glTF rig.

## What happened

In Phase 1, `relaxed_stand` passed every unit test: the hand sat 0.561 m
below the shoulder on the synthetic rig, with 0.03° direction-conversion
error. On `puppet_base.gltf` it rendered with both arms raised overhead.
Only a Front-view screenshot caught it. The cause is in
[Conjugate pose deltas by the bind rotation](./conjugate-pose-deltas-by-the-bind-rotation.md).

A tempting fix was to compose transforms in a unit test using the
`real_puppet_base_gltf_skeleton` fixture. That fixture attached the glTF's
bind ROTATIONS to entities spawned with synthetic T-pose TRANSLATIONS, which
builds a hybrid rig that exists nowhere. The tell was that two different
poses composed to byte-identical geometry. The fixture was sound for
asserting relationships between rotations, which is all
`write_pose_to_skeleton` computes, and unsound for positions. (The fixture
no longer exists under that name as of 2026-09-28.)

## Why it matters

A test that measures a proxy for the real system cannot see a bug that
lives in the difference between the proxy and the real system. Here the
difference is exactly the bind rotations, which are what retargeting is
about.

`HumanoidSkeleton::rest_direction` is a trap in the same family. It is a
bone's local translation direction in its PARENT's space (glTF bones run
along local +Y), not a world-space direction. Conjugating a world-space
T-pose direction against it compares two different spaces.

## How to apply

- For anything that depends on the real rig's geometry or binds, measure the
  real rig. Today that means `RigGeometry` with `forward_kinematics_on`, or
  parsing `puppet_base.gltf` in tests (see
  [Parse the asset, don't transcribe it](../../engineering-practice/testing/parse-the-asset-dont-transcribe-it.md)).
- For rendered-position questions, query the live example over BRP.
- Keep the synthetic rig for rig-independent properties: rotations,
  symmetry, bone-length invariance.

## Related

- [Conjugate pose deltas by the bind rotation](./conjugate-pose-deltas-by-the-bind-rotation.md) — deeper: the retargeting bug this blindness hid.
- [Synthetic rig's leg segments are shifted a joint](./synthetic-rig-leg-segments-are-shifted-a-joint.md) — same-trap: the synthetic rig also misrepresents leg shape.
- [Foot IK feedback loops](../ik-and-locomotion/foot-ik-feedback-loops.md) — example: foot IK solved on the synthetic proxy and lunged on slopes.
- [Prefer BRP over prints for live ECS state](../../engineering-practice/debugging/prefer-brp-over-prints-for-live-ecs-state.md) — applies: how to measure the real rendered rig.
