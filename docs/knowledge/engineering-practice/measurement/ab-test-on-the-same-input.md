---
title: A/B test a feature on the same input
description: To measure one feature's contribution, disable that feature on identical input; comparing two different scenarios measures everything else that differs too (0.740 rad measured vs 0.304 rad real for slope foot alignment). Read before asserting "feature X contributes Y" in a test or an investigation.
type: lesson
status: current
tags:
  - verification
  - testing
  - ik
updated: 2026-09-25
verified: 2026-09-28
code:
  - src/character/anim/legik.rs
sources:
  - Claude memory ab_test_on_same_input_not_different_scenario (2026-09-25)
aliases:
  - controlled comparison
  - normal_alignment
  - A/B on identical input
  - different scenario is not a control
---

# A/B test a feature on the same input

A control must differ from the experiment in exactly one thing. To measure
what a feature contributes, **turn that feature off on identical input**.
Comparing two different scenarios conflates the feature with everything else
that differs between them.

## What happened

To check that foot alignment adds the slope's tilt, the sole direction on a
slope was compared with the sole direction on flat ground.
- The comparison reported **0.740 rad** where **0.304 rad** was expected.
- Three wrong diagnoses followed (double rotation, cross-frame accumulation,
  rig feedback) before instrumenting the solver showed it received
  *identical, correct* input every frame.
- Flat ground also changes where the leg solve puts the ankle, so the
  difference measured alignment **plus** a different leg solve.
- Setting `normal_alignment: 0.0` on the **same** slope gave exactly the
  intended 0.304 rad.

## Why it matters

A different scenario is not a control. Its result is a real number with a
wrong meaning, and it sends the investigation after bugs that do not exist.

## How to apply

- Build the comparison by disabling the feature on identical input: a config
  field set to zero, a parameter passed as `None`, a CLI toggle.
- If that is awkward to arrange, the feature is not cleanly separable. That is
  worth knowing too.
- The same rule applies to whole builds: A/B the parent commit on the same
  scene, not a different scene.

## Evidence

- `normal_alignment` in `src/character/anim/legik.rs`; 0.740 rad (different
  scenario) vs 0.304 rad (same slope, feature off).
- In the renderer, the sealed-room leak hunts toggled `--no-reflect`,
  `--no-transmission` and `--gi-method none` on the same frame of the same
  scene, which isolated each leak cleanly (see Related).

## Related
- [Same function on both sides is a vacuous test](../testing/same-function-both-sides-is-a-vacuous-test.md) — contrast: that one is a test that cannot fail; this is a test that fails for the wrong reason.
- [Replicate the real frame loop in a unit test](../testing/replicate-the-real-frame-loop-in-a-unit-test.md) — same-trap: a test setup that differs from the real system in more than one way.
- [Verify, don't assert from memory](../debugging/verify-dont-assert-from-memory.md) — applies: re-run the controlled comparison instead of reasoning from the uncontrolled one.
- [Foot IK feedback loops](../../character-animation/ik-and-locomotion/foot-ik-feedback-loops.md) — example: the slope foot-alignment investigation this came from.
- [Bounce GI unconditional leak](../../hybrid-architecture/gi-and-lighting/bounce-gi-unconditional-leak.md) — example: per-feature toggles on the same frame pointed straight at the glass cube's refraction bounce.
- [DDGI probe-grid bounds wall-embedding leak](../../hybrid-architecture/gi-and-lighting/ddgi-probe-grid-bounds-wall-embedding-leak.md) — example: `--gi-method none` vs `ddgi` on the same frame, bit-for-bit identical after the fix.
