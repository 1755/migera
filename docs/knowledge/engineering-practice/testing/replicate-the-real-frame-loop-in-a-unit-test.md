---
title: Replicate the real frame loop in a unit test
description: When a bug shows only in the running example, write a unit test that runs the real per-frame system body in its real order instead of relaunching the GPU example to read HUD logs. Layer-isolated tests miss bugs from layer interaction. Read when a bug is "right in tests, wrong live".
type: lesson
status: current
tags:
  - testing
  - debugging
  - character-animation
updated: 2026-09-23
verified: 2026-09-28
sources:
  - Claude memory replicate_real_frame_loop_in_unit_test (2026-09-23)
aliases:
  - end-to-end unit test
  - step_muscle_sim
  - layer-isolated tests
---

# Replicate the real frame loop in a unit test

When a bug appears only in the running example, the fastest diagnostic is a
**new unit test that runs the real per-frame system's exact body**: the same
operations in the same order, driven through the real config API. It
reproduces the live numbers in milliseconds and can sample every frame.

## What happened

The case was the old position-space muscle simulation. Its per-frame system
`step_muscle_sim` ran gravity, then `solve_step`, then
`blend_toward_target_positions`, then `apply_ground_lock`. The existing tests
each exercised **one** layer (for example only the target generation through
`advance_pose_transition` and `target_world_position`). None ran the whole
loop together.

A bug that exists only when the layers interact, such as ground-lock fighting
a keyframe target, is invisible to layer-isolated tests, however many there
are. The alternative was to rebuild and launch the GPU example and read a HUD
log line once a second.

A test that replicated `step_muscle_sim`'s body, driven by the real
`MuscleConfig::play_walk_cycle()`, found three real bugs in one session.

The muscle module was deleted in commit 9981e16 (2026-09-25). The lesson
applies unchanged to `src/character/anim`'s systems.

## Why it matters

- Unit tests are usually written per layer, and interaction bugs live between
  layers.
- A live GPU example is a slow, low-rate probe: one rebuild and launch per
  hypothesis, one sample per second.

## How to apply

1. When a report says "looks right in tests, wrong live", first check whether
   any test drives the **real** system function end to end.
2. If none does, write one: same order of operations, real config API, real
   inputs. Sample every frame.
3. Only go back to the live example to confirm the fix visually.

## Evidence

- Found the three bugs recorded in
  [Walk-cycle IK and ground-lock bugs](../../character-animation/ik-and-locomotion/walk-cycle-ik-and-ground-lock-bugs.md).

## Related
- [Walk-cycle IK and ground-lock bugs](../../character-animation/ik-and-locomotion/walk-cycle-ik-and-ground-lock-bugs.md) — example: the three bugs this technique found.
- [The muscle module was deleted](../../character-animation/animation-core/muscle-deleted-anim-is-the-only-stack.md) — prerequisite: why the code named here no longer exists.
- [DDGI probe-grid bounds wall-embedding leak](../../hybrid-architecture/gi-and-lighting/ddgi-probe-grid-bounds-wall-embedding-leak.md) — example: CPU tests fed hand-shrunk inputs while the real pipeline did not; testing the real inputs found the leak.
- [A/B test on the same input](../measurement/ab-test-on-the-same-input.md) — same-trap: comparisons that differ in more than the one thing under test.
- [Prefer BRP over prints for live ECS state](../debugging/prefer-brp-over-prints-for-live-ecs-state.md) — contrast: when the question really is about live state, query it instead of printing it.
