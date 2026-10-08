---
title: A reach compared against what its own lift clamps it to is noise
description: "The ladder chose a hand's rung by reach <= HAND_STRETCH·arm from the lifted shoulder, but the lift brings any liftable reach to exactly that share: float noise (±1e-7 m) decided, and a 1e-6 change flipped a ladder's pattern. Read before comparing a solved value against a threshold."
type: lesson
status: current
tags:
  - numerics
  - correctness
  - testing
  - ik
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/ladder.rs
  - src/character/anim/armik.rs
sources:
  - "test ladder::tests::a_ladders_pattern_does_not_hang_on_float_noise"
  - "test ladder::tests::the_way_it_climbs_follows_the_rungs_spacing"
aliases:
  - knife edge
  - float noise decides
  - HAND_STRETCH LIFT_FROM
  - STRETCH_SLACK
  - PASSING_LOWER
---

# A reach compared against what its own lift clamps it to is noise

`ladder::Climbing::stretched` picks the highest hand rung whose wrist is
within `HAND_STRETCH` (0.85) of the arm from the shoulder, lifted toward it.
`armik::shoulder_lift` raises the shoulder just far enough to bring a wrist
within `LIFT_FROM` (also 0.85) of it, up to the lift's limit. So every rung
the lift can bring in comes out at exactly the threshold, and `<=` against
it was decided by rounding. Such a rung now counts within `STRETCH_SLACK`
(0.1 mm).

## What happened

Normalizing `rig::delta_after_world_turn`'s result changed it by under 1e-6,
through the stance's foot IK into the standing pose the ladder measures the
body from. Five ladder tests then failed, all on rungs 0.36 m apart: the
feet passing each other instead of both to each rung, the trunk leant
0.49 rad in, a wrist 0.29 m through the rungs' plane.

Printing each choice found the cause. At several hips distances the third
rung's reach, minus the stretch, was +3e-8 to +9e-8 m before the change and
-9e-8 to 0 after. Across all seven test ladders hundreds of choices sat
within 1e-5 of the threshold, falling either way, within the same ladder.
The baseline the tests had been tuned on was itself a mix of noise.

Counting a lifted rung as within (the rule the doc meant: from the lifted
shoulder) exposed a second fault. On 0.36 m rungs the hands now reached for
passing feet, but only at the four nearest of twenty hips distances. The
choice took passing feet whenever any distance allowed it, so it took them
there, the hands 4.9 cm lower than the best both-feet climb. Passing feet
are now taken only if they hold the hands within `PASSING_LOWER` (2 cm) of
the best of all. On every ladder where the feet should pass, they hold the
hands highest at all twenty distances; on 0.36 m rungs they don't.

## Why it matters

A threshold test on a value some other step has solved to that threshold
can never be decided by geometry. It holds by accident until any unrelated
change shifts the noise. The symptom shows far away, as a different
discrete choice, and looks like the unrelated change broke it.

## How to apply

- **Before comparing a solved quantity against a threshold**, check whether
  the solve targets that same value. If it does, compare with an explicit
  slack in the direction you mean (here: lifted to the stretch counts).
- **When a tiny, correct change flips a discrete choice**, print the
  compared values on both sides. Residuals of 1e-7 to 1e-8 mean a tie by
  construction, not a close call.
- **A preference that takes a pattern whenever any candidate allows it**
  jumps when one candidate crosses. Weigh the preferred option's best
  against the best overall (a margin), not its mere existence.
- **Pin it with a test that nudges the inputs** by about 1e-6 and requires
  the same choice; then prove the test fails with the slack and margin
  removed.

## Evidence

- `a_ladders_pattern_does_not_hang_on_float_noise`: the standing pose's
  turns nudged by ±1e-6 and 3e-6 rad, all seven ladders keep their pattern
  and hips distance. With the slack at 0 and the margin unbounded, the
  standard ladder's hips moved from 0.328 to 0.346 m out under a 1e-6
  nudge.
- With both in, the ladder tests pass with and without the normalization in
  `delta_after_world_turn`, which is now in.

## Related

- [A ladder is climbed limb by limb between holds](./a-ladder-is-climbed-limb-by-limb-between-holds.md) — applies: the climb whose hand rungs and pattern this decides.
- [Rotations turned over and over in one frame need renormalizing](../rig-and-retargeting/rotations-turned-over-and-over-in-a-frame-need-renormalizing.md) — context: the correct change that exposed it.
- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: a check that holds by construction, not by the world.
