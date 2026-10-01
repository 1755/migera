---
title: A fall test samples one chaotic landing
description: "A ragdoll fall is chaotic: torques of 1e-7 rad/s before it changed how three tests' bodies landed, and they failed on the new landings (a rested body creeping 4 mm, one never resting on a slope, a hand 1 mm over a bound). A single-landing test pins one outcome, not the behaviour. Read before writing or 'fixing' a fall/rise test."
type: lesson
status: current
tags:
  - ragdoll
  - testing
  - physics
  - numerics
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/ragdoll_plugin.rs
sources:
  - "ragdoll_plugin::tests::a_fallen_ragdoll_gets_up_and_is_pinned_again, a_ragdoll_rising_on_a_slope_keeps_clear_of_the_ground_under_it, a_rise_moves_no_limb_far_above_where_its_keys_put_it (failed with velocity feedforward of rounding-level target spin; passed with it zeroed)"
aliases:
  - chaotic landing
  - flaky fall test
  - JointTargetVelocity deadband
---

# A fall test samples one chaotic landing

Each fall/rise test runs one fall and checks how that body landed, lay and
rose. A fall is chaotic: a limb's contact order decides which way the body
rolls. A perturbation far below anything physical sends it to a different
landing, and the test then judges that one.

## What happened

Adding target-velocity feedforward to the ragdoll's controller (see
[a pinned ragdoll tracks its targets' velocity](./a-pinned-ragdoll-tracks-its-targets-velocity.md))
failed three get-up tests, deterministically. The feedforward never
exceeded 0.01 rad/s anywhere they ran. It was rounding: composing the same
standing pose frame after frame left target spins near 1e-7 rad/s. With
those zeroed, all three passed again. The new landings had found:

- a body put to rest that still crept 4.1 mm through the get-up delay,
  against the test's 1 mm (rest allows anything under 0.05 m/s for 1 s);
- a body on a 0.2 slope that never came to rest within 10 s;
- a hand rising 61 mm against a 60 mm bound.

## Why it matters

- A test of one landing pins that landing, not the behaviour. It passes
  or fails on numerics anywhere in the physics or the controller,
  unrelated to what the change is about.
- The landings it does find are real: those three are things a game will
  hit on some fall.

## How to apply

- When a fall test fails after a change that shouldn't touch falls, check
  whether the landing changed (how it lies, where, `probe_how_falls_lie`)
  before suspecting the change.
- Don't feed numerical noise into torques: the controller zeroes target
  spins under 1e-3 rad/s.
- A fall test that matters should run several launches, or assert on what
  every landing must satisfy. Its bounds should match the system's own
  definitions (rest: under 0.05 m/s for 1 s), not stricter ones.

## Related

- [A fall hands the body to physics](./a-fall-hands-the-body-to-physics.md) — context: rest detection and its thresholds.
- [Getting up goes through key poses chosen by how the body lies](./getting-up-is-a-timed-blend-then-a-re-pin.md) — applies: the rise tests that tripped.
