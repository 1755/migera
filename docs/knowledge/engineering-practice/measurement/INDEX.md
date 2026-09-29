---
title: Measurement lessons
description: Lessons on numbers that misled — comparisons across different scenarios, measurements taken through a broken system, probe targets on a symmetry plane, finite-difference seam tests, and the Quat::angle_between precision floor. Read before trusting a measurement or choosing what to measure.
type: index
status: current
tags:
  - verification
  - numerics
  - testing
updated: 2026-09-28
---

# Measurement lessons

"Measure, don't assume" is this project's rule, and it is right. These notes
are the cases where a real, careful measurement still gave the wrong answer,
and why.

| Note | What it establishes | Read when |
|---|---|---|
| [A/B test a feature on the same input](./ab-test-on-the-same-input.md) | Disable the feature on identical input; a different scenario measured 0.740 rad where the feature contributed 0.304 rad. | Before claiming "feature X contributes Y". |
| [A measurement of a broken system records the breakage](./a-measurement-of-a-broken-system.md) | Constants and tables measured against a bug become wrong when it is fixed; armik's axes were exactly reversed. | After fixing anything load-bearing. |
| [False progress near the mirror axis](./false-progress-near-the-mirror-axis.md) | A metric fell from 0.36 m to 0.34 mm while the wrong arm moved; the probe sat on the symmetry plane. | Before picking a probe target, or when a number improves dramatically. |
| [Measuring curve continuity at a seam](./measuring-curve-continuity-at-a-seam.md) | Three independent finite-difference traps; print raw values first, then compare extrapolated one-sided limits. | Before testing that piecewise curves join smoothly. |
| [Quat::angle_between has a precision floor near identity](./quat-angle-between-precision-floor.md) | ~9.8e-4 rad f32 floor; assert on one minus the absolute dot product (`rotation_mismatch`). | Before a tight rotation-equality assertion. |

## See also
- [Testing lessons](../testing/INDEX.md) — when the test itself cannot fail.
- [Hybrid renderer performance findings](../../hybrid-architecture/performance-findings/INDEX.md) — measured null results in the renderer, each checking a premise before building.
