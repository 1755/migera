---
title: Quat::angle_between has a precision floor near identity
description: Quat::angle_between is 2*acos(|dot|), and acos amplifies f32 rounding into a ~9.8e-4 rad apparent angle between bit-identical rotations. Assert on 1 - |dot| instead (rotation_mismatch in quat_ext.rs). Read before writing any tight rotation-equality assertion.
type: lesson
status: current
tags:
  - numerics
  - testing
  - math
updated: 2026-09-24
verified: 2026-09-28
code:
  - src/character/anim/math/quat_ext.rs
sources:
  - Claude memory quat_angle_between_precision_floor (2026-09-24)
aliases:
  - rotation_mismatch
  - acos precision
  - quaternion equality tolerance
---

# Quat::angle_between has a precision floor near identity

`Quat::angle_between` computes `2 * acos(|dot|)`, and `acos` has an infinite
derivative at 1. The ~1.2e-7 rounding error in an f32 `dot` is amplified into
an apparent-angle floor of **~9.8e-4 rad**: two bit-identical quaternions can
report that much "drift". Any test asserting a tighter bound with
`angle_between` is measuring the measurement. **Use `1 - |a.dot(b)|` instead.**

## What happened

Rotation round-trip tests asserted tight `angle_between` bounds and reported
drift between rotations that were in fact identical.

## Why it matters

Near identity, which is where equality tests live, `angle_between` stops being
a measurement of the rotations and becomes a measurement of f32 rounding.

## How to apply

- Assert on `1 - |a.dot(b)|`. There is no amplification.
- Since `1 - |dot| ≈ angle² / 8`, a bound of `1e-6` still pins rotations to
  about 0.003 rad (0.16°), far tighter than any animation tolerance, while
  staying safely above the f32 floor (a perfect round trip still lands 1 ULP,
  about 1.19e-7, away).
- In this crate, use `rotation_mismatch` with the `EXACT` bound from
  `src/character/anim/math/quat_ext.rs` (added 2026-09-24).
- `angle_between` is also unsigned. For rates or directions, use a signed
  angle.

## Evidence

- `rotation_mismatch` and `EXACT` in `src/character/anim/math/quat_ext.rs`,
  used by `dho.rs` and `inertialize.rs`.

## Related
- [Measuring curve continuity at a seam](./measuring-curve-continuity-at-a-seam.md) — example: a continuity test whose step fell below this floor.
- [Unsigned measurements cannot see direction](../testing/unsigned-measurements-cannot-see-direction.md) — deeper: the other limitation of `angle_between`.
- [Spring exp() approximation diverges](../../character-animation/animation-core/spring-exp-approximation-diverges.md) — same-trap: another numerics shortcut that is wrong outside the range it was checked in.
