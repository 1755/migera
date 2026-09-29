---
title: A measurement of a broken system records the breakage
description: "Measured, not assumed" is necessary but not sufficient; a careful measurement taken through a buggy substrate records the bug as fact (armik's measured twist/hinge axes were exactly reversed after the pose-space fix). Read after fixing anything load-bearing, to find the constants and tables measured against it.
type: lesson
status: current
tags:
  - verification
  - ik
  - rig
updated: 2026-09-27
verified: 2026-09-28
code:
  - src/character/anim/armik.rs
sources:
  - Claude memory a_measurement_of_a_broken_system (2026-09-27)
  - commit 026d9e8
aliases:
  - stale calibration
  - constants tuned against a bug
  - arms_bent
---

# A measurement of a broken system records the breakage

A measurement inherits the validity of everything beneath it. Constants,
defaults and doc tables tuned against a buggy substrate become wrong the
moment the substrate is fixed, and they do not fail loudly. After fixing
something load-bearing, **re-measure what was measured against it**.

## What happened

- `armik`'s module doc (`src/character/anim/armik.rs`) carried a measured
  table naming `+Y` as the arm's degenerate twist axis and `+X` as the elbow
  hinge.
- Re-measured after the pose-space convention was fixed (commit 026d9e8), it is
  the exact opposite: `+X` moves the wrist 0.0058 m (nowhere) and `+Y`/`+Z`
  move it 0.186 m.
- The original measurement was performed correctly. It measured a broken
  system, so it faithfully recorded the breakage.
- The same fix invalidated the `arms_bent` test helper. It bent about the
  newly degenerate axis, so several tests silently ran inside the singularity
  they existed to escape.

## Why it matters

This project leans hard on "measure, don't derive", correctly, because
derivation has repeatedly gone wrong here. The rule has a blind spot: a
measurement is only as true as the system it was taken on.

## How to apply

- When fixing something load-bearing, grep for constants, defaults, doc tables
  and test helpers that were *measured against it*, and re-measure them.
- Treat a comparison against a system that was itself broken as void, and
  re-run it after the fix.

## Evidence

- `src/character/anim/armik.rs` module doc and `arms_bent` helper, after
  commit 026d9e8. Numbers: 0.0058 m vs 0.186 m wrist travel.
- Renderer case: Radiance Cascades was judged against DDGI while DDGI still had
  a sealed-room leak; the leak was found only when the user said DDGI looked
  worse. See Related.

## Related
- [A pose delta names a world axis](../../character-animation/rig-and-retargeting/a-pose-delta-names-a-world-axis.md) — example: the pose-space fix that reversed the measured axes.
- [False progress near the mirror axis](./false-progress-near-the-mirror-axis.md) — same-trap: a metric that improved for a reason unrelated to the claim.
- [Kill stale processes before trusting BRP](../debugging/kill-stale-processes-before-trusting-brp.md) — same-trap: a careful measurement of the wrong binary.
- [Unsigned measurements cannot see direction](../testing/unsigned-measurements-cannot-see-direction.md) — same-trap: a careful measurement that cannot answer the question asked.
- [Grep other consumers before declaring a fix done](../debugging/grep-other-consumers-before-declaring-a-fix-done.md) — applies: the same grep, aimed at measured constants.
- [Radiance Cascades experiment](../../hybrid-architecture/gi-and-lighting/radiance-cascades-experiment.md) — example: an A/B whose baseline (DDGI) was itself leaking.
