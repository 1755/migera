---
title: A pinned root's velocity is not its pace
description: "A kinematic body driven by `(target − position)/dt` per physics step reads twice its pace on one step and zero on the next whenever a frame runs two steps, so a fall released from it launched at 0 m/s in 6 of 10 walking falls. Track the target's own per-frame velocity (KinematicRoot::velocity). Read before reading any kinematic body's velocity."
type: lesson
status: current
tags:
  - ragdoll
  - physics
  - correctness
  - testing
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/ragdoll_plugin.rs
sources:
  - "test ragdoll_plugin::tests::a_fall_while_moving_leaves_at_the_bodys_pace (fails at −2.4 m/s with the fix disabled)"
  - "live BRP LinearVelocity of ragdoll:Hips at release, character_gallery --anim-speed-schedule 0:1.2 --ragdoll on --fall-at-frame 300..360, both rigs, before and after"
aliases:
  - KinematicRoot::velocity
  - follow_kinematic_roots
  - fall launch velocity
---

# A pinned root's velocity is not its pace

`follow_kinematic_roots` moves the pinned hips body by setting, each
physics step, the velocity that closes that step's gap to its target. When
a render frame runs two physics steps, the first closes the whole frame's
motion at twice the pace and the second finds nothing left: velocity zero.
The body's position is right. Its velocity is noise.

## What happened

A fall while walking released the hips with whatever the last step left.
Live at 1.13–1.16 m/s on both rigs, the hips body's forward velocity at
release was **0.00 in 6 of 10 falls**, 0.50–1.88 in the rest. The limbs,
driven by their joints, were at the walk's pace, so the mean over all 14
bodies looked fine (0.88–1.32) and hid it. Pelvis paths after release
varied from 1.69 m/s forward to backward.

A first fix added the root's error to every body. That launched the limbs
at up to 2.32 m/s, because the limbs never had the error.

## Why it matters

Any reader of a velocity-driven kinematic body's `LinearVelocity` gets
this alternation: a launch, a momentum estimate, a speed readout. A
per-body mean or a pelvis path can average or rotate it away.

## How to apply

- `KinematicRoot::velocity` is the target's own motion per frame
  (`publish_joint_targets`). `release_falling_roots` sets the hips body to
  it and leaves the limbs as they are. Live after: 1.02–1.59 m/s, following
  the pelvis's own ±20% through the stride.
- Measure a body's velocity body by body, never by a mean over a jointed
  assembly.
- A headless test needs frames of one AND two physics steps to see it:
  `a_fall_while_moving_leaves_at_the_bodys_pace` alternates them.

## Related

- [A fall hands the body to physics](./a-fall-hands-the-body-to-physics.md) — applies: the release this launch belongs to.
- [A push while walking moves the next footfalls](../ik-and-locomotion/a-push-while-walking-moves-the-next-footfalls.md) — context: walking falls, where it showed.
- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — same-trap: a measurement that looks right because something else carries it.
