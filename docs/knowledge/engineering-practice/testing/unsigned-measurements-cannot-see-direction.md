---
title: Unsigned measurements cannot see direction
description: An unsigned angle reads the same for a shape and its mirror, so 30+ leg tests measuring angle_between(thigh, shin) missed a backward-bending knee for weeks. Assert a signed quantity in the rig's own frame, and test the rig as the game assembles it. Read before testing knees, elbows, facing or winding.
type: lesson
status: current
tags:
  - testing
  - ik
  - rig
  - math
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/character/anim/rig.rs
  - examples/character_gallery.rs
sources:
  - Claude memory unsigned_measurements_cannot_see_direction (2026-09-28)
  - commit 4c7fbc9
aliases:
  - backward knee
  - knee_fold_direction
  - knee_forward_offset
  - handedness
  - signed angle
---

# Unsigned measurements cannot see direction

`angle_between(thigh, shin)` is 141° whether the knee folds forward or
backward. A test that cannot tell a shape from its mirror is not testing the
shape. For anything with a handedness, assert a **signed** quantity in the
rig's own frame.

## What happened

- Every leg test in `character::anim` (more than 30) measured the unsigned
  thigh–shin angle. A solver that inverted every knee passed all of them. The
  backward knee was reported from screenshots three times before any test
  could see it.
- Picking the wrong signed measurement produced a *second* false diagnosis.
  Near full extension the knee sits on the hip-to-ankle line by definition.
  On the synthetic rig at 99.2% extension, `knee_forward_offset` read
  `-0.017` (apparently backward) while `knee_fold_direction` read `-0.34`
  (solidly human).
- An earlier version of the fixed invariant used the parsed asset, went green,
  and the character still walked on backward knees. `build_real_mesh_skeleton`
  (`examples/character_gallery.rs`) folds a 180° yaw into
  `hips_root_rotation`, so the parsed rig and the rendered rig disagree about
  facing, which was the one quantity under test.

## Why it matters

Magnitude and direction are different questions, and the intuitive
measurement usually answers only the first. Near a singularity, a signed
measurement can also lose its meaning while still printing a confident sign.

## How to apply

- For knee bend, elbow bend, facing or winding, assert a signed quantity. In
  this crate (`src/character/anim/rig.rs`):
  - `RigGeometry::knee_forward_offset`: which side of the hip-to-ankle line
    the knee is on. Good on a clearly bent leg.
  - `RigGeometry::knee_fold_direction`: how the shin turns relative to the
    thigh. **Use this one near full extension**, where the offset's residual
    is dominated by the hip's lateral placement.
- Near a singularity, ask which measurement still has meaning before you
  believe its sign.
- Test the rig **as the game assembles it**, including root corrections such
  as the yaw in `hips_root_rotation`, not only the parsed asset.

## Evidence

- Commit 4c7fbc9 ("measure the knee by its fold, not by its offset from the
  line").
- Numbers: 141° for both knee directions; `-0.017` offset vs `-0.34` fold at
  99.2% extension.

## Related
- [Knee axis: positive swings forward](../../character-animation/rig-and-retargeting/knee-axis-positive-swings-forward.md) — example: the knee-direction convention this measurement protects.
- [Kill stale processes before trusting BRP](../debugging/kill-stale-processes-before-trusting-brp.md) — same-trap: the other false knee diagnosis, made against a stale process.
- [Quat::angle_between precision floor](../measurement/quat-angle-between-precision-floor.md) — deeper: `angle_between` is also unsigned and has a precision floor near identity.
- [Measuring curve continuity at a seam](../measurement/measuring-curve-continuity-at-a-seam.md) — same-trap: unsigned `angle_between` cannot tell slowing down from reversing.
- [A measurement of a broken system](../measurement/a-measurement-of-a-broken-system.md) — same-trap: a careful measurement that still answers the wrong question.
- [Same function on both sides is a vacuous test](./same-function-both-sides-is-a-vacuous-test.md) — same-trap: another way a whole suite can pass without testing its subject.
