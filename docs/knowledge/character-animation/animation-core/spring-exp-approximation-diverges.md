---
title: The rational exp approximation in spring code diverges
description: "The cubic rational exp(-x) approximation copied into many damped-spring implementations is 2.1x wrong at x=5 and 74x at x=10, and stiff springs reach that range; migera uses f32::exp. Read before writing or 'optimizing' spring, damper or inertialization math."
type: lesson
status: current
tags:
  - springs
  - numerics
  - correctness
updated: 2026-09-24
code:
  - src/character/anim/math/spring.rs
sources:
  - test a_spring_converges_to_its_target_from_any_start
aliases:
  - fast_negexp
  - exp approximation
---

# The rational exp approximation in spring code diverges

The rational approximation `1/(1 + x + 0.48x² + 0.235x³)` for `exp(-x)` is
widely copied into damped-spring code, and it is accurate only while `x < 1`.
Use `f32::exp`, which is one hardware-backed instruction.

## What happened

Measured error of the approximation: **2.1x at x = 5, 74x at x = 10**.

For a spring, `x = decay_rate * dt`, so large `x` is not an edge case. A
0.02 s half-life at 60 fps already gives x = 0.58. One dropped frame pushes
it past 2. Any stiff spring at a low frame rate lands in the divergent
region.

The error compounds every step. A spring converging from -100 stalled at
0.9989, where the exact solution reaches 0.999998.

## Why it matters

The approximation looks like a harmless speed trick, but it bought nothing
measurable, and it breaks exactly where springs are stiff or frames are
slow. Walk springs in migera have since become very stiff (0.015 s legs,
0.03 s arms, per `CHARACTER_PROGRESS.md`), which puts `x` even further from
the accurate range.

## How to apply

- Use `f32::exp` in spring, damper and inertialization code.
- Test convergence from far-away starts and at large `dt`, not only near
  the target at 60 fps.

## Evidence

Fixed in `src/character/anim/math/spring.rs` on 2026-09-24. Caught by
`a_spring_converges_to_its_target_from_any_start`.

## Related

- [Quat::angle_between precision floor](../../engineering-practice/measurement/quat-angle-between-precision-floor.md) — same-trap: another f32 numeric limit that makes spring tests pass or fail wrongly.
- [PD damping has an explicit-integration bound](../ragdoll-and-physics/pd-damping-explicit-integration-bound.md) — same-trap: another integration that goes unstable when a gain times `dt` gets large.
