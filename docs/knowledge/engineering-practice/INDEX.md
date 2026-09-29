---
title: Engineering practice
description: Cross-cutting lessons on testing, debugging and measurement, each paid for with a real bug in migera's animation or renderer code — tests that could not fail, misleading numbers, stale live state, symptoms far from causes. Read before writing a test a fix relies on, trusting a measurement, or when a diagnosis is stuck.
type: index
status: current
tags:
  - testing
  - debugging
  - verification
  - numerics
updated: 2026-09-28
---

# Engineering practice

Lessons about *how* to verify and debug, as opposed to facts about one
subsystem. Every note records a concrete failure (with numbers and commits)
and the rule that would have prevented it. The concrete cases live in
[character animation](../character-animation/INDEX.md) and the
[hybrid renderer](../hybrid-architecture/INDEX.md); each lesson links to them
as `example`.

## Key facts
- **Prove a test can fail by sabotaging the shared function, not the data.** Data perturbation reaches both sides of a comparison. — [Same function on both sides](./testing/same-function-both-sides-is-a-vacuous-test.md)
- **Pair every relative assertion with an absolute one.** A cohesion test passed with the ragdoll at y = −1930 m. — [Cohesion tests miss free fall](./testing/cohesion-tests-miss-free-fall.md)
- **Assert signed quantities for anything with a handedness.** An unsigned angle cannot tell a knee from its mirror. — [Unsigned measurements](./testing/unsigned-measurements-cannot-see-direction.md)
- **Parse real assets in tests; never transcribe them.** — [Parse the asset](./testing/parse-the-asset-dont-transcribe-it.md)
- **A control differs from the experiment in exactly one thing.** Toggle the feature on identical input. — [A/B on the same input](./measurement/ab-test-on-the-same-input.md)
- **After fixing something load-bearing, re-measure what was calibrated against it.** — [A measurement of a broken system](./measurement/a-measurement-of-a-broken-system.md)
- **Probe off every symmetry plane,** and confirm a big improvement happened for the reason claimed. — [False progress near the mirror axis](./measurement/false-progress-near-the-mirror-axis.md)
- **Kill stale processes before any BRP read,** and A/B the parent commit in a worktree when live data and tests disagree. — [Kill stale processes](./debugging/kill-stale-processes-before-trusting-brp.md)
- **"Fixed" means every consumer checked,** including every duplicated CPU/WGSL copy. — [Grep other consumers](./debugging/grep-other-consumers-before-declaring-a-fix-done.md)
- **Measure link by link; don't guess the culprit,** and re-check rather than recall. — [Symptom far from cause](./debugging/symptom-is-far-from-cause-in-the-rig-chain.md), [Verify, don't assert from memory](./debugging/verify-dont-assert-from-memory.md)

## Topics

| Note | What it establishes | Read when |
|---|---|---|
| [Testing](./testing/INDEX.md) | Six ways a green test measured nothing: vacuous comparisons, relative measures, no-op constraints, layer-isolated tests, unsigned measures, transcribed data. | Before writing a test that a fix or claim relies on. |
| [Debugging](./debugging/INDEX.md) | Bisecting, re-verifying, checking every consumer, BRP live inspection, stale processes, long causal chains. | When a diagnosis is stuck, or before declaring a fix verified. |
| [Measurement](./measurement/INDEX.md) | Uncontrolled comparisons, measurements of broken systems, symmetric probes, seam continuity, the `angle_between` precision floor. | Before trusting a number or choosing what to measure. |

## See also
- [Character animation](../character-animation/INDEX.md) — where most of these lessons were learned; its root pose-verification protocol applies them.
- [Hybrid renderer GI and lighting](../hybrid-architecture/gi-and-lighting/INDEX.md) — sealed-room leak hunts that are worked examples of A/B toggles and checking every copy.
- [Hybrid renderer performance findings](../hybrid-architecture/performance-findings/INDEX.md) — measured null results: check the premise before building the optimization.
