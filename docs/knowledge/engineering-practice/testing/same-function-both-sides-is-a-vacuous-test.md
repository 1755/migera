---
title: Same function on both sides is a vacuous test
description: A test comparing two values that both flow through the same function cannot fail, and perturbing the input data does not expose it. Prove non-vacuity by sabotaging the shared function. Read before trusting any "A matches B" test, or when a regression test passes suspiciously easily.
type: lesson
status: current
tags:
  - testing
  - verification
  - poses
updated: 2026-09-25
verified: 2026-09-28
code:
  - src/character/anim/poses.rs
  - src/character/anim/asset.rs
sources:
  - Claude memory same_function_both_sides_is_a_vacuous_test (2026-09-25)
aliases:
  - vacuous test
  - test that cannot fail
  - non-vacuity check
  - sabotage test
---

# Same function on both sides is a vacuous test

If both sides of a comparison pass through the function under test, the
test asserts nothing about that function. Perturbing the *data* does not
reveal this, because the perturbation reaches both sides. The fix is to
**sabotage the shared function itself** and check that the test fails.

## What happened

A test asserted that "the compiled-in poses match the pose files on disk".
It passed, and it could never have failed:
- Both sides parsed the same bytes through the same
  `PoseAsset::to_local_pose` (`src/character/anim/asset.rs`).
- `include_str!` makes cargo rebuild when the bytes change, so the two sides
  always moved in lockstep.

The usual non-vacuity check was to perturb the input data and confirm the
test fails. It *also* passed, because the perturbation propagated to both
sides.

## Why it matters

A green test that measures nothing is worse than no test. It is cited as
proof, and it stops anyone looking further. "I perturbed the input and it
failed" is only proof when the perturbation reaches **one** side of the
comparison.

## How to apply

1. For every comparison test, write down the code path each side flows
   through. If they share the function being tested, the test is vacuous for
   that function.
2. Prove non-vacuity by breaking the shared function (for example, call
   `.inverse()` on the rotation it returns) and confirm the test goes red.
   If it stays green, delete or rewrite the test.
3. Compare against something genuinely independent. Here that was the real
   Mixamo world positions the pose was derived from, checked through forward
   kinematics. That comparison is
   `poses::tests::relaxed_stand_still_matches_its_reference_data`.
4. For a regression test, run it with the fix disabled and confirm it fails.

## Evidence

- `src/character/anim/poses.rs`: `relaxed_stand_still_matches_its_reference_data`
  is the independent replacement.
- Related failure found the same way: a retargeting conjugation test with a
  delta about X passed vacuously because the leg's bind rotation is also about
  X (parallel rotations commute). See
  [Parse the asset, don't transcribe it](./parse-the-asset-dont-transcribe-it.md).

## Related
- [A/B test on the same input](../measurement/ab-test-on-the-same-input.md) — same-trap: the mirror image, a test that fails for the wrong reason; both are fixed by making the comparison measure one thing.
- [Restoring-force constraints need perturbed input](./restoring-force-constraints-need-perturbed-input.md) — same-trap: disabling each constraint in turn is this rule applied to a solver.
- [Cohesion tests miss free fall](./cohesion-tests-miss-free-fall.md) — same-trap: the other way a green test can measure nothing.
- [Synthetic-rig tests are blind to retargeting](../../character-animation/rig-and-retargeting/synthetic-rig-tests-are-blind-to-retargeting.md) — example: a whole suite that could not see the code path it claimed to cover.
- [Conjugate pose deltas by the bind rotation](../../character-animation/rig-and-retargeting/conjugate-pose-deltas-by-the-bind-rotation.md) — example: a regression test verified by disabling the fix.
- [DDGI any-hit occlusion](../../hybrid-architecture/gi-and-lighting/ddgi-any-hit-occlusion.md) — example: the renderer's regression test was confirmed to fail against the old logic before it was trusted.
