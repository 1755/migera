---
title: An antipodal guard must still land on the target
description: "math::ik::look_rotation treated anything within 1.8° of opposite as exactly opposite and returned a bare half-turn, landing on -from, not on `to`: a 0.25 m bone aimed past its hanging direction came out up to 8 mm wide. Read before writing a from→to rotation or when an aim misses by a few mm."
type: lesson
status: current
tags:
  - ik
  - numerics
  - correctness
updated: 2026-10-07
verified: 2026-10-07
code:
  - src/math/ik.rs
  - src/character/anim/legik.rs
  - src/character/anim/armik.rs
sources:
  - "test math::ik::tests::look_rotation_lands_on_a_nearly_opposite_target"
  - "test ladder::tests::a_far_reach_lifts_the_shoulder_and_the_arm_lands_on_it"
aliases:
  - look_rotation
  - antipodal
  - from_rotation_arc near opposite
  - near-opposite aim
---

# An antipodal guard must still land on the target

`math::ik::look_rotation(from, to)` guards `Quat::from_rotation_arc`
against near-opposite inputs, which it is unstable for. The guard fired
for `dot < -0.9995`, within 1.8° of opposite, and returned a plain half
turn about a stable perpendicular. That carries `from` to `-from`, not to
`to`: up to 1.8° off. Every aim in `legik::aim_bone` and the arm solve
goes through it.

## What happened

Climbing a ladder, a hand reaching overhead lifts its shoulder
(`ladder::shoulder_lift`). Lifted, the hanging upper arm and the overhead
target came within 1.8° of opposite, and the arm solve's first aim landed
the elbow 3-8 mm off. The solve reported the wrist exactly on target; the
pose had it 5-10 mm off. With the shoulder at rest, the same pose solved
exactly, so the lift looked like the culprit; it had only moved the arm
into the band. Bisected in the pose: the positions fed to the solve were
fresh, a copy solved again missed the same way, and the first aim (the
elbow) was already off. The miss was in the aim, and only near opposite.

## Why it matters

An antipodal guard has to choose *an* axis, because at exactly 180° any
perpendicular is right. But the guard's band is wider than the point. Inside
it, `to` is not `-from`, and returning the half turn alone silently trades a
numerical instability for a deterministic error of up to the band's width.
The existing test (`look_rotation_handles_the_antipodal_case`) only tried
`to = -from` exactly, where the half turn is right, so it could not fail.

## How to apply

- Inside the band, half-turn first, then take the short, well-conditioned
  arc from `-from` onto `to`. That is `look_rotation`'s form now.
- Test a guard *inside* its band, not only at its centre: 179°, 179.5°,
  179.9° as well as 180°. The new test failed on the old code
  (`(1,0,0)` at 179° landed on `(-1,0,0)`, wanted 1° from it) before the
  fix.
- When an IK solve reports its target reached but the posed joint is off by
  a few millimetres, re-solve a copy with fresh positions and with the
  suspect change undone, to find which step misses.

## Evidence

- `math::ik::tests::look_rotation_lands_on_a_nearly_opposite_target`:
  179°, 179.5°, 179.9° and 180° from three directions, within 0.1 mm;
  failed before the fix.
- The ladder's held hands, which strayed 5-10 mm from their grips with the
  shoulder lifted, hold within 0.1 mm after it; all 833 animation tests,
  every leg and arm aim included, pass unchanged (2026-10-07).

## Related

- [A ladder is climbed limb by limb between holds](./a-ladder-is-climbed-limb-by-limb-between-holds.md) — example: where the shoulder lift exposed it.
- [Two-bone IK pivots at the upper joint, not the root](./two-bone-ik-pivots-at-upper-not-root.md) — same-trap: another IK miss whose test made the same mistake as the code.
- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: a test that only probes the one input where the code is right.
