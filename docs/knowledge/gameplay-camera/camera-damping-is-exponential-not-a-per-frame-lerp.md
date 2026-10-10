---
title: Camera damping is exponential, not a per-frame lerp
description: "Camera smoothing must be frame-rate independent: closed-form exponential decay or a critically damped spring, parameterised by half-life. A per-frame lerp or min(1, k·dt) (Gothic, UE VInterpTo) changes feel with frame rate. Covers unit conversions and moving goals. Read before writing any camera smoothing."
type: concept
status: current
tags:
  - camera
  - springs
  - numerics
  - correctness
updated: 2026-10-10
code:
  - src/math/spring.rs
  - src/math/angle.rs
  - src/math/inertialize.rs
sources:
  - "Cinemachine Predictor.cs (StandardDamp, StableDamp): https://github.com/Unity-Technologies/com.unity.cinemachine/blob/main/com.unity.cinemachine/Runtime/Core/Predictor.cs"
  - "Daniel Holden, Spring-It-On / spring-roll-call: https://theorangeduck.com/page/spring-roll-call"
  - "dolly smoothing: https://github.com/h3r2tic/dolly/blob/main/src/util.rs"
  - "OpenGothic camera.cpp (min(1, 0.25*veloTrans*dt)): https://raw.githubusercontent.com/Try/OpenGothic/master/common/camera.cpp"
  - "Freya Holmér, 'Lerp smoothing is broken' (talk; page not fetched)"
aliases:
  - lerp smoothing
  - SmoothDamp
  - VInterpTo
  - exp decay smoothing
  - framerate independent damping
---

# Camera damping is exponential, not a per-frame lerp

A camera that "feels heavier at 30 fps and twitchier at 144 fps" is using a
per-frame lerp. Every camera stage that smooths must use one of two forms, and
nothing else:
- **closed-form exponential decay**, `x = g + (x − g)·exp(−λ·dt)`, or
- **a closed-form critically damped spring**.

Parameterise both by **half-life**. migera has both forms, stable at any `dt`:
- `SpringParams` + `spring_scalar` / `spring_vec3` in `src/math/spring.rs`;
- `spring_*_tracking` there, for a goal that moves during the frame;
- `damp` / `damp_angle` in `src/math/angle.rs`;
- `decay_exponential` in `src/math/inertialize.rs`.

## The broken forms

| Form | Where it appears | Problem |
|---|---|---|
| `x = lerp(x, g, c)` per frame | countless tutorials; smooth-bevy-cameras' lag weight | The decay per second is `(1−c)^fps`, so it changes completely with frame rate |
| `x = lerp(x, g, min(1, k·dt))` | Gothic `veloTrans`; UE `VInterpTo`/`RInterpTo` | First-order correct only while `k·dt ≪ 1`; a frame spike lands closer to the goal than the exact solution, and the clamp to 1 makes it snap |
| Cubic rational `exp` approximation | copied spring code | 74× wrong at x = 10; see [the exp approximation diverges](../character-animation/animation-core/spring-exp-approximation-diverges.md) |

## Unit conversions

All of these are the same exponential with a different name for its rate:

- **half-life** *h*: `λ = ln 2 / h` (Holden's convention; migera's `SpringParams::halflife`).
- **Cinemachine damping** *T*, "1% left after *T* s": `λ = ln(100) / T ≈ 4.605 / T`,
  so *h* = 0.1505 · *T*.
- **dolly smoothness** *s*: `λ = 8 / s`, so *h* = 0.0866 · *s*.
- **Gothic `veloTrans`** *v*: approximately `λ ≈ 0.25 · v` while `k·dt` stays small.
  `veloTrans` 40 gives ≈ 10/s, so *h* ≈ 0.07 s.

## Exponential decay vs spring

- **Exponential decay has a velocity kink.** When the goal starts moving, the follower's
  velocity jumps. That is fine for a boom length easing back out, and visible as a jolt
  in the pivot when the character starts running.
- **A critically damped spring keeps velocity continuous.** It overshoots never and is
  closed-form at any `dt`. Use it for the pivot and for anything the eye tracks directly.
  `SmoothDamp` (Game Programming Gems 4) is the same family with a speed clamp.

## Moving goals still drift with frame rate

Even an exact spring step is frame-rate dependent when the *goal* moves during the
frame, because the step holds the goal still: a staircase. At 5 m/s with a 0.05 s
half-life, the staircase follower ends more than 5 cm apart at 30 Hz and at 144 Hz
(`the_staircase_spring_trails_a_moving_target_by_frame_rate`). Cinemachine's `StableDamp`
sub-steps at 1/1024 s to hide this.

migera solves it exactly instead:
- With damping on absolute velocity, the error to a goal moving at `v` settles at a
  constant lag `−2ζv/ω`. The deviation from that lag is a plain homogeneous spring.
- `spring_scalar_tracking` / `spring_vec3_tracking` advance that deviation in closed form
  over the interval the goal moved in. The goal moves from last frame's sample to this
  frame's, so this is interpolation, never extrapolation.
- The result is identical at any frame rate for a goal that moves linearly between frames.

The other ways a camera built from exact dampers still drifts with frame rate are
covered in [frame-rate independence needs exact events and thresholds, not just exact
dampers](./frame-rate-independence-needs-exact-events-and-thresholds.md).

## Angles

- **Damp yaw by the shortest signed angular difference**, never by raw subtraction.
  Otherwise a yaw crossing ±π swings the long way around.
- **Keep pitch and yaw as scalars.** Smoothing them separately and rebuilding the rotation
  as yaw·pitch keeps roll at exactly zero.
- **Slerping whole quaternions** with an exp weight works, but loses the per-axis control
  a camera wants.

## Relevance to migera

- Reuse the forms above. Don't write new damping code.
- Every camera test of smoothing should include a 30/60/144 Hz comparison at common
  timestamps: every 1/6 s is a frame boundary at all three rates. To show the test can
  fail, swap in a staircase or `lerp(k·dt)` variant: it must fail.

## Related

- [Frame-rate independence needs exact events and thresholds](./frame-rate-independence-needs-exact-events-and-thresholds.md) — deeper: the drifts left once every damper is exact, and the tests that missed them.
- [The rational exp approximation in spring code diverges](../character-animation/animation-core/spring-exp-approximation-diverges.md) — same-trap: why migera's springs call `f32::exp`.
- [A pinned ragdoll tracks its target's velocity](../character-animation/ragdoll-and-physics/a-pinned-ragdoll-tracks-its-targets-velocity.md) — same-trap: the same `2ζv/ω` lag behind a moving target, in the ragdoll's PD.
- [Measuring curve continuity at a seam](../engineering-practice/measurement/measuring-curve-continuity-at-a-seam.md) — deeper: how to measure the velocity kink without fooling yourself.
- [Third-person camera design](./third-person-camera-design.md) — applies: which stages use decay and which use springs.
- [Engine camera architectures compared](./engine-camera-architectures-compared.md) — example: how Cinemachine, Unreal and dolly each damp.
