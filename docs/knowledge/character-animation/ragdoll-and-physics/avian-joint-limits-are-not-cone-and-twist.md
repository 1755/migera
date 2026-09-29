---
title: avian joint limits are not cone and twist
description: "avian 0.7 SphericalJoint swing/twist limits are measured against twist_axis.any_orthonormal_vector(); with the default twist_axis=+Y along the bone they are two unrelated bend stops. Set twist_axis=+X with the bone along +Y. Read before configuring ragdoll joint limits or upgrading avian."
type: lesson
status: current
tags:
  - physics
  - ragdoll
  - correctness
  - testing
updated: 2026-09-28
code:
  - src/character/anim/ragdoll_plugin.rs
sources:
  - avian PR #803 (0.4 joint rework)
  - test a_twist_axis_across_the_bone_makes_avians_limits_a_true_cone_and_twist
  - test every_limited_ragdoll_joint_uses_the_cone_and_twist_configuration
aliases:
  - SphericalJoint
  - swing_limit
  - twist_limit
  - twist_axis
---

# avian joint limits are not cone and twist

avian 0.7's `SphericalJoint` limits are measured against a REFERENCE axis,
`twist_axis.any_orthonormal_vector()`. `swing_limit` bounds how far that
reference tilts; `twist_limit` bounds the twist axes' roll about it. With the
default `twist_axis = +Y` running down the bone, the two limits become
unrelated bend stops, one of which also catches twist.

## What happened

Measured on the live rig: the arm's 85° bend was clamped at about 68.5° by
its 70° "twist" range.

The cause is avian's 0.4 joint rework (PR #803), which derived the reference
from `twist_axis` and inverted the 0.3 meaning. It is still so on 0.8-dev.

## Why it matters

The names suggest a swing cone plus a twist range. The implementation does
something else, and a mis-set limit looks like an unexplained stiff joint
rather than a configuration error.

## How to apply

- Set `joint.twist_axis = Vec3::X`, with the bone along the joint frame's
  `+Y`. glam's `X.any_orthonormal_vector()` is `+Y`, so swing becomes a true
  cone and twist a true roll.
- Centre limit frames on the bind pose, and keep shipped poses inside the
  anatomical ranges (pinned by a pose-vs-limit invariant, per
  `CHARACTER_PROGRESS.md`).
- Re-check on any avian upgrade.

## Evidence

Pinned by `a_twist_axis_across_the_bone_makes_avians_limits_a_true_cone_and_twist`
and `every_limited_ragdoll_joint_uses_the_cone_and_twist_configuration` in
`ragdoll_plugin.rs`.

## Related

- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — applies: this was one of the defects found once the ragdoll was actually measured.
- [Use apply_angular_acceleration, not apply_torque](./avian-apply-angular-acceleration-not-torque.md) — same-trap: another avian API that means something other than its name.
- [Ragdoll body and anchor frames](./ragdoll-body-and-anchor-frames.md) — prerequisite: the body frames these limits are expressed in.
