---
title: Restoring-force constraints need perturbed input
description: A constraint that pulls back toward an original relationship does nothing on input that already satisfies it, so testing it alone proves nothing. Test it where a stronger constraint displaces the positions first, at the seam between regimes. Read before testing any restoring or preservation term in a solver.
type: lesson
status: current
tags:
  - testing
  - locomotion
  - ik
updated: 2026-09-25
verified: 2026-09-28
code:
  - src/character/anim/slide.rs
sources:
  - Claude memory restoring_force_constraints_need_perturbed_input (2026-09-25)
  - commit 3e701bd
aliases:
  - motion preservation constraint
  - foot sliding removal
  - seam frame
---

# Restoring-force constraints need perturbed input

A constraint phrased as "move toward the position that reproduces the
**original** relationship" is a restoring force. When the input already
satisfies that relationship, its target *is* the current position, and the
whole term does nothing. A test on unperturbed input therefore passes with
the term deleted.

## What happened

The case was the motion-preservation constraint in `slide.rs`
(`src/character/anim/slide.rs`, offline foot-sliding removal):
- A test on a free-swinging foot in isolation **passed with the entire term
  deleted**. The positions were never perturbed, so there was nothing to
  restore.
- The term only does work where the contact constraint pulls against it. Its
  effect concentrates in the **seam frame** between contact and swing.
  Measured: a 0.140 m jump with the term disabled, against an authored
  0.03 m step.

Disabling each constraint in turn showed which tests could fail. Contact
coherence failed 3 tests, motion preservation 1, limb length 1. Two of those
only failed after the tests were rewritten. A tolerance can also hide a
constraint: the limb-length test allowed 0.12 m against a 0.0427 m effect,
about 28 times too loose to notice.

## Why it matters

The quiet region, where the constraint is already satisfied, is the easiest
place to write a test and the one place the constraint cannot be seen.

## How to apply

1. Build a scenario where a *different, stronger* constraint displaces the
   positions first.
2. Assert on the boundary between the two regimes, not on the quiet region.
3. Disable each constraint in turn and record which tests fail. A constraint
   whose removal fails no test is untested.
4. Size tolerances against the effect you measured with the term disabled,
   not against a round number.

## Evidence

- `src/character/anim/slide.rs` and its tests (landed in commit 3e701bd).
- Numbers above: 0.140 m seam jump without the term, 0.03 m authored step,
  0.12 m tolerance against a 0.0427 m effect.

## Related
- [Same function on both sides is a vacuous test](./same-function-both-sides-is-a-vacuous-test.md) — prerequisite: the general "prove the test can fail" check this applies to a solver.
- [A/B test on the same input](../measurement/ab-test-on-the-same-input.md) — deeper: disabling one term on identical input is the right control.
- [Verify, don't assert from memory](../debugging/verify-dont-assert-from-memory.md) — same-trap: re-run the check instead of trusting that it once passed.
