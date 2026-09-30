---
title: Normalize what you read back from your own output
description: "The ragdoll read-back inverted the Transform it wrote last frame with glam's inverse(), which assumes unit length; the norm error fed back each frame and never decayed, reaching 1.03 lying down: skeleton scaled 6%, drawn 8-21° off its bodies. Read before closing a loop through your own writes."
type: lesson
status: current
tags:
  - numerics
  - ragdoll
  - correctness
  - debugging
updated: 2026-09-30
verified: 2026-09-30
code:
  - src/character/anim/ragdoll_plugin.rs
sources:
  - "test ragdoll_plugin::tests::a_released_ragdoll_falls_with_its_momentum_and_is_drawn_where_it_lies (unit-norm check fails at 1.0001 with the fix removed)"
aliases:
  - quaternion drift
  - non-unit quaternion
  - live_hips_parent_rotation
  - angle_between clamps
---

# Normalize what you read back from your own output

When a system reads back a value it wrote itself and builds the next write
from it, rounding error has a loop to live in. Quaternion `inverse()` in
glam is the conjugate, which is the inverse **only for unit length**. A
non-unit rotation passed through it keeps its norm error, and nothing ever
pulls it back.

## What happened

`live_hips_parent_rotation` computed the hips' parent world rotation as
`world(hips) · local(hips).inverse()`. Here `local(hips)` is the Transform
the ragdoll read-back had itself written the frame before. The read-back
then built the next hips delta from that root and wrote it. The norm
carried through each lap unchanged, so fp error random-walked with no
restoring force.

Standing, it stayed invisible. After a fall, with large rotations and
thousands of frames, the written hips rotation reached **norm 1.0315**. A
non-unit rotation in a `Transform` also *scales* whatever hangs under it
(by |q|² ≈ 1.064 here), and every simulated bone was drawn 5–21° off
its body. The one bone that looked right, `LeftArm` at 0.0°, happened to
sit close to its animated target.

It took a while to see because `Quat::angle_between` clamps its dot
product. With a non-unit quaternion involved, two readings came out 0.0°
that were really several degrees apart, contradicting the 8.2° measured
between the same rotations. Normalizing both sides before comparing
(`(a.normalize().inverse() * b.normalize()).to_axis_angle()`) exposed it.

## Why it matters

- A loop through your own output has no ground truth in it. Error doesn't
  cancel; it accumulates, and it's worst in exactly the states tests rarely
  reach (a fallen body, a long run).
- Any API that assumes normalized input (`inverse`, `angle_between`,
  `slerp`'s shortcuts) turns the drift into a wrong answer rather than an
  error.

## How to apply

- Normalize at the loop's entry: `rotation.normalize()` on anything read
  back from a component you write, and normalize what you write.
- Compare rotations with normalized inputs, or check `length()` first
  when a measurement contradicts another.
- A regression test for it must run the loop long enough and assert the
  norm (here, every frame of a 5 s fall stays within 1e-4 of 1). Without
  the fix it fails at 1.0001.

## Related

- [Full-strength read-back hides the physics](../../character-animation/ragdoll-and-physics/full-strength-readback-hides-the-physics.md) — context: the same read-back, and why it must be checked against the bodies.
- [A fall hands the body to physics](../../character-animation/ragdoll-and-physics/a-fall-hands-the-body-to-physics.md) — applies: where it surfaced.
- [Quat::angle_between precision floor](../measurement/quat-angle-between-precision-floor.md) — same-trap: angle_between is not a safe measurement near its edges.
