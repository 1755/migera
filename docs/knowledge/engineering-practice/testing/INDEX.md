---
title: Testing lessons
description: Lessons on writing tests that can actually fail — vacuous comparisons, relative measures blind to absolute failure, constraints that are no-ops on clean input, layer-isolated tests, unsigned measures, and transcribed test data. Read before writing a test a fix or a claim will rely on.
type: index
status: current
tags:
  - testing
  - verification
  - correctness
updated: 2026-09-28
---

# Testing lessons

Each note here is a test that passed while the thing it claimed to check was
broken, and the rule that would have caught it. They came from
`src/character/anim` and the hybrid renderer, but the rules are general.

**Start here:** [Same function on both sides is a vacuous test](./same-function-both-sides-is-a-vacuous-test.md).
Its check (break the code under test and confirm the test goes red) is the
basis of the rest.

| Note | What it establishes | Read when |
|---|---|---|
| [Same function on both sides is a vacuous test](./same-function-both-sides-is-a-vacuous-test.md) | A comparison whose two sides share the tested function cannot fail; data perturbation does not reveal it; sabotage the function instead. | Before trusting any "A matches B" or regression test. |
| [Cohesion tests miss free fall](./cohesion-tests-miss-free-fall.md) | A relative measure (spread, shape) passed with the ragdoll at y = −1930 m; assert the absolute quantity too. | When a test measures parts against each other. |
| [Restoring-force constraints need perturbed input](./restoring-force-constraints-need-perturbed-input.md) | A "restore the original relationship" term is a no-op on clean input; test it at the seam where a stronger constraint displaces things. | Before testing a solver term, or when a constraint's removal fails no test. |
| [Replicate the real frame loop in a unit test](./replicate-the-real-frame-loop-in-a-unit-test.md) | Layer-isolated tests miss layer-interaction bugs; replicate the real per-frame system body in a test. | When a bug is "right in tests, wrong live". |
| [Unsigned measurements cannot see direction](./unsigned-measurements-cannot-see-direction.md) | An unsigned angle reads the same for a shape and its mirror; 30+ tests missed backward knees. Use signed measures, and test the rig as the game builds it. | Before testing knees, elbows, facing or winding. |
| [Parse the asset, don't transcribe it](./parse-the-asset-dont-transcribe-it.md) | Build test rigs by parsing `puppet_base.gltf` via `gltf_rig.rs`; transcribed numbers drift and miss what they weren't told. | Before putting real asset or rig numbers into a test. |

## See also
- [Measurement lessons](../measurement/INDEX.md) — when the test is fine but the number it reads is misleading.
- [Verifying character poses](../../character-animation/INDEX.md) — the character domain's verification protocol, where most of these lessons were learned.
