---
title: Root motion is the rendered contact's displacement, not a velocity times dt
description: "Root velocity × dt slid a walk's planted foot up to 2 mm a frame once speed varied, and moving the body by the TARGET pose slid it 39 mm a stance through the springs; move it by the planted contact's displacement between RENDERED poses. Read before changing how a gait moves its character."
type: lesson
status: current
tags:
  - locomotion
  - springs
  - numerics
  - correctness
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/locomotion.rs
  - examples/character_gallery.rs
sources:
  - "test locomotion::tests::the_default_springs_keep_a_walking_foot_planted"
aliases:
  - root_displacement_between
  - ride_rendered_feet
  - foot slide
  - root motion integration
---

# Root motion is the rendered contact's displacement, not a velocity times dt

Move a walking body by **how far its planted contact moved under the hips
between the two poses actually rendered**:
`locomotion::root_displacement_between(before, after, …)`, negated and
horizontal. Do not move it by the published root velocity times the frame's
`dt`. Do not measure it on the gait's target pose when the rendered pose is
spring-filtered.

## What happened

The walk's speed stopped being constant when its legs started replaying a
recorded stride. The body slows over each foot and speeds up in double
support, about 6 m/s² either way. Two sources of foot slide appeared:

- **Explicit integration.** `position += velocity(phase_end) * dt` errs by
  about `½·a·dt²`. At 60 Hz the planted foot slid 0.1-2 mm a frame,
  about a centimetre a stance. The hand-shaped walk had hidden this because
  its speed was held constant.
- **Spring lag.** The rendered pose is the target filtered through the
  per-bone springs. With leg springs at a 0.015 s half-life, the lag still
  changes with the foot's speed. Moved by the target's contact motion, the
  body out-walked its rendered foot by 39 mm a stance, headless.

## Why it matters

Root motion cancels the planted foot's motion. The cancellation is exact
only if it measures the same pose the renderer draws, over exactly the
interval the frame covers. Any velocity sampled at one instant, or measured
on a different pose, leaves a residual. That residual is visible as foot
sliding and grows with everything that makes a walk look alive: speed
rhythm, springs, blends.

## How to apply

- `locomotion::advance_turning_with` in `Authoritative` mode now advances by
  the contact displacement between `pose_at(phase − cadence·dt)` and
  `pose_at(phase)`. It falls back to the velocity only through a flight
  phase, where no foot is down.
- `examples/character_gallery.rs` runs `ride_rendered_feet` after
  `AnimSet::Spring` and before `AnimSet::Ik`. It moves the entity by the
  displacement between the last two rendered poses (`AnimPose::pose()`), so
  the body is exact through the springs. A game's character controller
  should consume the same displacement.
- A test of planted feet must integrate the body the same way; see
  `the_default_springs_keep_a_walking_foot_planted`.

## Evidence

- Headless replay at 60 Hz, 1 stride/s, measured walk on `puppet_base`:
  target-velocity integration slid 0.1-2 mm a frame; through default springs,
  39 mm a stance; with the rendered displacement, under 5 mm (the test's
  bound).
- Live over BRP (`ball_l`/`ball_r`, 0.7-1.6 m/s): at most 1.2 mm per planted
  run.

## Related

- [A walking foot touches the ground at its heel, ball and toe](./walking-foot-rocker-contact-model.md) — prerequisite: what "the planted contact" is.
- [Replicate the real per-frame loop in a unit test](../../engineering-practice/testing/replicate-the-real-frame-loop-in-a-unit-test.md) — same-trap: why the planted-foot test replays the gallery's frame loop.
