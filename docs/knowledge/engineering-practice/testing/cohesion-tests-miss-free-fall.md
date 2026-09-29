---
title: Cohesion tests miss free fall
description: A test of a relative quantity (spread, cohesion, shape, error between parts) passes while the whole assembly drops out of the world; the ragdoll passed at y = -1930 m. Pair every "stays together" with a "stays where it should be". Read before writing a test that measures parts against each other.
type: lesson
status: current
tags:
  - testing
  - ragdoll
  - physics
updated: 2026-09-25
verified: 2026-09-28
code:
  - src/character/anim/ragdoll_plugin.rs
sources:
  - Claude memory cohesion_tests_miss_free_fall (2026-09-25)
  - commit 5f77022
aliases:
  - relative measure test
  - absolute position assertion
  - pin_root
---

# Cohesion tests miss free fall

A test that measures a **relative** quantity is blind to everything that
moves all the parts together. Assert the absolute quantity separately.

## What happened

`a_full_ragdoll_holds_itself_together_under_gravity`
(`src/character/anim/ragdoll_plugin.rs`) measured each body's distance from
the assembly's own centre and asserted that the spread did not grow. It
passed, correctly, while the ragdoll was at **y = −1930 m**. Every joint was
perfectly oriented and the whole character was in free fall.

A body falling as one connected unit keeps its spread constant. The test was
right about what it claimed. It never claimed that the character stays up.

The cause: the PD controller drives *rotation only*. Nothing in the module
applied a linear force, so there was no code path anyone thought to test.
The fix was `RagdollSpawnConfig::pin_root`, which makes the root body
kinematic.

## Why it matters

Relative metrics (spread, shape, ratio, error between parts) are chosen
because they ignore global motion. That is exactly why they cannot see a
global failure.

## How to apply

- When a test measures a relative quantity, ask which absolute quantity it is
  blind to, and assert that too.
- Pair every "stays together" with a "stays where it should be".
- A screenshot catches this class in seconds, and so does a BRP query against
  the live ECS. Both are cheaper than reasoning about it.

## Evidence

- Test `a_full_ragdoll_holds_itself_together_under_gravity` and
  `RagdollSpawnConfig::pin_root` in `src/character/anim/ragdoll_plugin.rs`.

## Related
- [Same function on both sides is a vacuous test](./same-function-both-sides-is-a-vacuous-test.md) — same-trap: the other way a green test can be measuring nothing.
- [Prefer BRP over prints for live ECS state](../debugging/prefer-brp-over-prints-for-live-ecs-state.md) — applies: the cheapest way to check an absolute position live.
- [Full-strength read-back hides the physics](../../character-animation/ragdoll-and-physics/full-strength-readback-hides-the-physics.md) — example: another ragdoll check that looked green while the physics was not being tested.
- [Gizmos need --show-real-mesh off](../../character-animation/animation-core/gizmos-need-show-real-mesh-off.md) — applies: the screenshot setup that shows where the skeleton actually is.
