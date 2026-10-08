---
title: Rotations turned over and over in one frame need renormalizing
description: "delta_after_world_turn now normalizes its result: unnormalized, a foot re-posed through a reshape's passes compounded its norm and FK grew it 0.7 % in a frame. Normalizing it first broke the ladder, which hung on float noise (fixed). Read before composing rotations many times a frame."
type: lesson
status: current
tags:
  - numerics
  - correctness
  - debugging
  - testing
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/rig.rs
  - src/character/anim/parkour/along.rs
sources:
  - "test parkour::along::tests::it_runs_along_a_wall_and_runs_on"
  - "test ladder::tests::a_ladders_pattern_does_not_hang_on_float_noise"
aliases:
  - quaternion drift
  - bone length drift
  - non-unit quaternion
  - normalize delta_after_world_turn
---

# Rotations turned over and over in one frame need renormalizing

`rig::delta_after_world_turn` computes `(frame⁻¹·turn·frame)·rotation`.
glam's `Quat::inverse` is the conjugate, which is only an inverse for a unit
quaternion. So a bone turned many times in one frame let its norm compound,
and FK scaled every offset below it by roughly |q|². The function now
normalizes its result, for every caller.

## What happened

A run along a wall poses each stepping foot onto its hold
(`parkour::along`):
- three passes of moving the root onto the planned COM and re-placing the
  feet;
- in each, up to four passes of `place_ankle`, `knee_toward` and a world
  turn of the foot.

With the ankle exactly on its hold and the foot's attitude exactly as
planned, the ball still sank into the face. It went 0.07, 0.44, then
1.65 mm in, over the reshape's passes. The ball-to-ankle vector had grown
from 0.1561 to 0.1572 m: the foot bone lengthened 0.7 % within one frame.
The keep-off pushed the ankle out to compensate, the foot left its hold by
2.7 mm, and the plan's reach check refused every run along the wall.
Normalizing fixed it: the ball sat exactly on the face and the foot kept its
length.

## Normalizing at the source first broke the ladder

The change to each result was under 1e-6, checked on every call through the
ladder tests. Yet five ladder tests failed, all on rungs 0.36 m apart: a
different climbing pattern, the trunk leant 0.49 rad in, a wrist 0.29 m
through the rungs' plane. The fault was the ladder's, not the norm's: its
pattern choice was decided by float noise
([a reach compared against what its own lift clamps it to is
noise](../ik-and-locomotion/a-reach-compared-against-what-its-lift-clamps-it-to-is-noise.md)).
With that fixed, the normalization went in at the source and the callers'
own copies came out.

## How to apply

- **A held point drifting by millimetres while its joint and attitude read
  exact**: measure the bone's length (child joint minus parent joint) frame
  over frame. A growing length is a norm, not a geometry bug.
- **Composing quaternions in a loop with glam**: `inverse` assumes unit
  length; renormalize whatever is written back.
- **A tiny, correct change that breaks a distant test badly** points at a
  discrete choice there sitting on a tie. Find and fix that tie; do not keep
  the correct change out to dodge it.

## Evidence

- Along a wall, before normalizing: the ball at -0.07 / -0.44 / -1.65 mm
  through the passes, the foot 0.1561 → 0.1572 m, every plan refused.
  After: the ball at 0.00000, the length constant, 18 of 18 run along.
- With the ladder's tie fixed, the whole library passes with the
  normalization at the source.

## Related

- [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) — prerequisite: what `delta_after_world_turn`'s frame is.
- [A reach compared against what its lift clamps it to is noise](../ik-and-locomotion/a-reach-compared-against-what-its-lift-clamps-it-to-is-noise.md) — deeper: the ladder fault this change exposed.
- [Symptom is far from cause in the rig chain](../../engineering-practice/debugging/symptom-is-far-from-cause-in-the-rig-chain.md) — same-trap: a foot off its hold, caused by a bone's length.
- [A wall is run along on two steps of a lifted leap](../parkour/a-wall-is-run-along-on-two-steps-of-a-lifted-leap.md) — example: the move that found it.
