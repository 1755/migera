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
  - src/character/anim/math/spring.rs
  - src/character/anim/math/inertialize.rs
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

Parameterise both by **half-life**. migera already has both forms, stable at any
`dt`: `SpringParams` + `spring_scalar` / `spring_vec3` in `math/spring.rs`, and
`decay_exponential` in `math/inertialize.rs`.

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

Even an exact exponential step is frame-rate dependent when the *goal* moves during the
frame, because the step assumes the goal held still. Cinemachine's `StableDamp`
sub-steps at 1/1024 s for exactly this reason.

For migera:
- A critically damped spring tracking a target that moves at constant velocity converges
  to the same lag at any frame rate. That is the property to test.
- Compare positions at **common timestamps** while sampling a *time-parameterised* target
  trajectory at 30, 60 and 144 Hz. Comparing frame N against frame N across rates tests
  nothing.

## Angles

- **Damp yaw by the shortest signed angular difference**, never by raw subtraction.
  Otherwise a yaw crossing ±π swings the long way around.
- **Keep pitch and yaw as scalars.** Smoothing them separately and rebuilding the rotation
  as yaw·pitch keeps roll at exactly zero.
- **Slerping whole quaternions** with an exp weight works, but loses the per-axis control
  a camera wants.

## Relevance to migera

- Reuse `SpringParams`, `spring_scalar` / `spring_vec3` and `decay_exponential`. Don't
  write new damping code.
- Every camera test of smoothing should include a 30/60/144 Hz common-timestamp
  comparison. To show the test can fail, swap in a `lerp(k·dt)` variant: it must fail.

## Related

- [The rational exp approximation in spring code diverges](../character-animation/animation-core/spring-exp-approximation-diverges.md) — same-trap: why migera's springs call `f32::exp`.
- [Measuring curve continuity at a seam](../engineering-practice/measurement/measuring-curve-continuity-at-a-seam.md) — deeper: how to measure the velocity kink without fooling yourself.
- [Third-person camera design](./third-person-camera-design.md) — applies: which stages use decay and which use springs.
- [Engine camera architectures compared](./engine-camera-architectures-compared.md) — example: how Cinemachine, Unreal and dolly each damp.
