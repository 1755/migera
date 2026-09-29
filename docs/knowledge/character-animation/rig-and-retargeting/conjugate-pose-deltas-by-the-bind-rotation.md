---
title: Conjugate pose deltas by the bind rotation
description: "A pose delta authored on the identity-bind synthetic T-pose must be written to a real rig as bind⁻¹·delta·bind, or it applies the right angle about the wrong axis; also, verify the regression test fails with the fix disabled. Read before writing pose rotations to a glTF rig."
type: lesson
status: current
tags:
  - retargeting
  - math
  - correctness
  - testing
updated: 2026-09-24
code:
  - src/character/anim/retarget.rs
  - src/character/anim/rig.rs
sources:
  - fn retarget::delta_in_bone_frame
aliases:
  - delta_in_bone_frame
  - bind-frame conjugation
  - arms raised overhead bug
---

# Conjugate pose deltas by the bind rotation

Poses in `character::anim` are authored against the synthetic T-pose, where
every bind rotation is identity, so a local delta there *is* a world-axis
rotation. On a real rig it must be conjugated into the bone's accumulated
bind frame before it is written:

```
delta_local = bind.inverse() * delta * bind
```

Here `bind` is the product of every rest rotation from the root down, seeded
with `hips_root_rotation()`.

## What happened

On a real rig `Transform.rotation` is read in the bone's own local frame. A
rendered orientation is `accumulated_bind(parent) * rest_rotation(bone) *
delta`. Writing `rest_rotation * delta` therefore applied the right **angle**
about the wrong **axis**. Symptom: `relaxed_stand` rendered with both arms
raised overhead on `puppet_base.gltf` while all 600 unit tests passed.

Measurement ruled out two wrong hypotheses first. Both assumed the two rigs'
bone DIRECTIONS disagreed. A BRP query of the live rig showed the bind-pose
left arm runs `(-1.000, 0.000, 0.030)`, against the synthetic
`(-1.000, 0.000, 0.000)`. They match. The mismatch was never where the bone
points, only the frame the delta is applied in.

The first regression test passed with the fix disabled.
`to_axis_angle`'s sign and range conventions made its axis comparison
insensitive. Comparing the resulting quaternions directly with `1 - |dot|`
is sensitive.

## Why it matters

A rotation delta is meaningless without the frame it is expressed in. Two
rigs can agree on every bone direction and still disagree on every bone
frame.

## How to apply

- Use `retarget::delta_in_bone_frame` when writing a rig-independent delta to
  a real rig.
- Before trusting a regression test, disable the fix and watch it fail.
- Compare quaternions with `1 - |dot|`, not by decomposed axis and angle.

## Evidence

Fix in `retarget::delta_in_bone_frame` (still present in
`src/character/anim/retarget.rs` and `rig.rs` as of 2026-09-28). Commit
026d9e8 later extended the same principle to forward kinematics; see
[A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md).

## Related

- [Synthetic-rig tests are blind to retargeting](./synthetic-rig-tests-are-blind-to-retargeting.md) — prerequisite: why no unit test saw this bug.
- [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) — deeper: the same conjugation applied inside FK and to world-space corrections.
- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: a regression test that could not fail.
- [Quat::angle_between precision floor](../../engineering-practice/measurement/quat-angle-between-precision-floor.md) — deeper: why `1 - |dot|` is the reliable quaternion comparison.
- [Ragdoll body and anchor frames](../ragdoll-and-physics/ragdoll-body-and-anchor-frames.md) — same-trap: correct maths applied in the wrong frame, in the ragdoll.
