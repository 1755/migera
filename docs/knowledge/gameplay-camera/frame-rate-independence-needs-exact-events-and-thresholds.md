---
title: Frame-rate independence needs exact events and thresholds, not just exact dampers
description: "A camera built only from closed-form springs still drifted 3-5 cm between 30 and 144 Hz: staircase goals, mode pushes credited a frame of blend, input sampled at frame ends, delays crossed mid-frame, Euler-integrated stick rate. Tests missed two of these. Read before writing time-dependent camera logic or its tests."
type: lesson
status: current
tags:
  - camera
  - numerics
  - testing
  - correctness
updated: 2026-10-10
verified: 2026-10-10
code:
  - src/camera/pipeline.rs
  - src/camera/orbit.rs
  - src/camera/anchor.rs
  - src/camera/harness.rs
  - src/math/spring.rs
sources:
  - "test camera::harness::tests::the_camera_lands_in_the_same_place_at_30_60_and_144_hz"
  - "test camera::orbit::tests::recentring_starts_at_the_same_moment_at_any_frame_rate"
  - "test camera::orbit::tests::a_held_stick_turns_the_same_at_any_frame_rate"
  - "test math::spring::tests::the_staircase_spring_trails_a_moving_target_by_frame_rate"
aliases:
  - frame-rate dependence
  - framerate independent camera
  - staircase goal
---

# Frame-rate independence needs exact events and thresholds, not just exact dampers

Making every damper closed-form is necessary, but it is not enough. The first
version of migera's camera pipeline (2026-10-10) used only exact springs and
exponential decays. Its eye still landed **2.9 cm apart at 30 Hz and at
144 Hz**. With individual fixes undone, the gaps reached 3.5–4.9 cm. Every
leftover drift came from *when* something happened inside a frame, not from
how it was damped.

## What happened

These were measured with a scenario sampled at 30, 60 and 144 Hz and compared
at common timestamps. Each row is that fix removed alone.

| Cause | What it did | Gap at 30 vs 144 Hz | Fix |
|---|---|---|---|
| **Staircase goal** | The pivot spring held the moving target still for the whole frame | 3.95 cm | Solve exactly for a goal moving linearly over the frame (`spring_vec3_tracking`) |
| **An event credited with a frame it never had** | A mode push before `stack.step(dt)` gave the new mode one frame of blend progress: 1/30 s at 30 Hz, 1/144 s at 144 Hz | 4.88 cm mid-blend | Step existing blends first, then apply the frame's requests |
| **Input sampled at the frame end** | A stick change exactly on a boundary was attributed to the frame *ending* there, shifting it by one frame | part of the 2.9 cm | The harness samples input held during a frame at its midpoint |
| **A threshold crossed mid-frame** | Auto-recentre started on the first frame with `idle ≥ delay`, so the whole frame counted | 0.007 rad of yaw | Damp only over `(idle − delay).clamp(0, dt)` |
| **Euler-integrated rate** | Stick look added `rate·dt`, with the rate itself easing toward the stick | 3.54 cm; 0.017 rad held yaw | Integrate the exponential approach in closed form: `target·dt + (r₀ − target)(1 − e^{−λdt})/λ` |

## Why it matters

Each of these is invisible at one frame rate. Together they make a camera feel
different on a slow machine, and they make a recorded trace replay to a different
place when its frame rate differs.

The **test** had two blind spots of its own, and each let a real drift through:
- **Sample times that missed the effect.** Comparing at whole seconds put no sample
  inside the 0.5 s mode blends, so the push-order bug passed. Comparing every 1/6 s,
  the common frame boundary of 30, 60 and 144 Hz, caught it.
- **Errors that cancel.** The orbit test compared yaw only after the stick was released.
  An Euler integral errs one way on the ramp up and the other way on the ramp down, so the
  final yaw agreed and the test passed under sabotage. Comparing while the stick is held
  caught it.
- **Delays that end on a shared boundary.** The scenario's recentre delay ended exactly on
  a frame boundary at every rate, so whole-frame crediting happened to agree. A delay
  that ends mid-frame (1.45 s) exposes it.

## How to apply

- **Goals that move:** use the tracking spring, or sub-step. Never hold a moving goal
  still for a frame.
- **Discrete events** (mode pushes, cuts, button presses): define when they take effect,
  either at the frame's start or its end, and apply them after advancing existing state
  over the frame.
- **Thresholds on accumulated time:** act only on the part of the frame past the
  threshold.
- **Rates that change:** integrate the rate's closed form, not `rate·dt`.
- **Testing:** compare at common timestamps that land inside every transient. Sample
  while things are moving, not only at rest. Put thresholds off the shared frame grid.
  Undo each fix and confirm its test fails before trusting it.

## Evidence

- `the_camera_lands_in_the_same_place_at_30_60_and_144_hz`, after the fixes: every 1/6 s
  for 6 s within 2 cm (measured: sub-millimetre).
- Undoing each fix, in order: staircase goal 3.95 cm; push order 4.88 cm at t = 1.17 s;
  Euler stick 3.54 cm, and held yaw 0.668 vs 0.651 rad.
- Whole-frame recentre: 0.787 vs 0.780 rad in
  `recentring_starts_at_the_same_moment_at_any_frame_rate`.

## Related

- [Camera damping is exponential, not a per-frame lerp](./camera-damping-is-exponential-not-a-per-frame-lerp.md) — prerequisite: the damper forms this lesson assumes, and the tracking spring.
- [Same function both sides is a vacuous test](../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: a passing comparison proves nothing until sabotage makes it fail.
- [Measuring curve continuity at a seam](../engineering-practice/measurement/measuring-curve-continuity-at-a-seam.md) — same-trap: a too-coarse sampling step hides what you are measuring.
- [Third-person camera design](./third-person-camera-design.md) — applies: the clock rules and stage order these fixes established.
