---
title: Measuring curve continuity at a seam
description: Finite-difference continuity tests fail in three independent ways that each look like a real discontinuity (step below the f32 floor, step too wide to be local, unsigned angle_between); compare extrapolated one-sided limits, and print raw values first. Read before testing that piecewise curves join smoothly.
type: lesson
status: current
tags:
  - testing
  - numerics
  - locomotion
  - math
updated: 2026-09-26
verified: 2026-09-28
code:
  - src/character/anim/gait.rs
sources:
  - Claude memory measuring_curve_continuity_at_a_seam (2026-09-26)
aliases:
  - C1 continuity test
  - finite difference seam
  - one-sided limit
  - gait curve join
---

# Measuring curve continuity at a seam

Testing that two piecewise curves join smoothly (in `src/character/anim/gait.rs`)
took many wrong iterations, because a naive finite-difference test can fail in
three independent ways that all look like a real discontinuity. **Print the
raw values either side of the seam first**; then compare extrapolated one-sided
limits, not raw chords.

## What happened

1. **Step below the precision floor.** `Quat::angle_between` has a ~9.8e-4 rad
   f32 floor near identity. A 1e-4 phase step produced ~1.2e-4 rad of real
   rotation, an order of magnitude below the floor. The test compared
   quantisation noise, so its verdicts meant nothing in either direction. It
   had been "passing" by luck.
2. **Step too wide to be local.** Widening to 0.01 put the "before" samples
   deep inside the previous segment, where the curve was still moving fast, and
   compared that with a genuinely flat region. It reported a "0.96 vs 0 jump"
   across a join that was provably C1. Both bounds are real; here a step of
   0.002 satisfied both.
3. **`angle_between` is unsigned.** It cannot tell slowing down from reversing,
   so a curve passing through its minimum reads as exactly zero rate on both
   chords. Use the signed angle (`2 * asin(q.x)` for a single-axis rotation).
   `to_axis_angle` does **not** fix this: it flips the axis to keep the angle
   positive, so it reports equal-magnitude, opposite-sign rates across a smooth
   join.
4. **Sample order.** Sampling the "before" side in *decreasing* phase order
   negates its differences. Take chords in increasing order on both sides.

## Why it matters

Each trap produces a confident, wrong verdict. Real curves also legitimately
have different curvature on each side of a join, and a raw chord comparison
cannot separate curvature from discontinuity.

## How to apply

1. **First**, print the raw values either side of the seam and look at the
   successive differences. That alone showed the join was smooth and would have
   saved most of the iterations.
2. Choose a step above the precision floor of your measurement and small enough
   to stay local.
3. Use a signed rate.
4. Compare **extrapolated one-sided limits**: two chords per side, linearly
   extrapolated to the seam, which removes the curvature term.

## Evidence

- The continuity tests in `src/character/anim/gait.rs`.

## Related
- [Quat::angle_between precision floor](./quat-angle-between-precision-floor.md) — prerequisite: the ~9.8e-4 rad floor behind trap 1.
- [Unsigned measurements cannot see direction](../testing/unsigned-measurements-cannot-see-direction.md) — same-trap: trap 3 in a different place.
- [A/B test on the same input](./ab-test-on-the-same-input.md) — same-trap: a comparison that measures more than one thing.
- [Verify, don't assert from memory](../debugging/verify-dont-assert-from-memory.md) — applies: look at the raw numbers before reasoning about them.
