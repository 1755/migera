---
title: Rotations turned over and over in one frame need renormalizing
description: "delta_after_world_turn is unnormalized: a foot re-posed through a reshape's passes compounded its norm and FK grew it 0.7 % in a frame, its ball 1.7 mm into a wall. Normalize in the caller; at the source it flips the ladder's rungs-apart climb. Read before re-posing a bone many times a frame."
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
  - src/character/anim/ladder.rs
sources:
  - "test parkour::along::tests::it_runs_along_a_wall_and_runs_on"
  - "tests ladder::tests::{a_slide_keeps_to_the_rails_and_lands_where_it_got_on, every_hold_is_held_and_every_limb_clears_the_ladder, it_climbs_upright_its_hands_above_its_shoulders, the_way_it_climbs_follows_the_rungs_spacing, led_no_hand_passes_through_the_ladder}"
aliases:
  - quaternion drift
  - bone length drift
  - non-unit quaternion
  - normalize delta_after_world_turn
---

# Rotations turned over and over in one frame need renormalizing

`rig::delta_after_world_turn` returns `(frame⁻¹·turn·frame)·rotation` without
normalizing. glam's `Quat::inverse` is the conjugate, which is only an inverse
for a unit quaternion. So a bone turned many times in one frame lets its
norm compound, and FK scales every offset below it by roughly |q|². Normalize
the bones you re-pose, in the caller.

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
Normalizing the leg's rotations after each pass fixed it: the ball sat
exactly on the face and the foot kept its length.

## Normalizing at the source loses

Normalizing inside `delta_after_world_turn` changed its results by under
1e-6 (checked on every call through the ladder tests). Yet five ladder
tests failed, all on the "rungs apart" ladder, by a lot:
- a different climbing pattern (`Pattern { gap: 1, hand: 3 }`, moving
  [2, 2] rungs);
- the trunk leant 0.49 rad in;
- a wrist 0.29 m through the rungs' plane.

So the ladder's pattern choice for that spacing sits on a tie that a 1e-6
change flips. The cause is not found yet: the climb is the same either way
for a person, so this is a fragility in the ladder, not a norm bug. Until
that is understood, the global fix stays out and callers normalize.

## How to apply

- **Re-posing a bone several times in one frame** (IK inside passes, a held
  limb re-placed after a root move): normalize the rotations you write,
  after each pass.
- **When a held point drifts by millimetres while its joint and attitude
  read exact**, measure the bone's length (child joint minus parent joint)
  frame over frame. A growing length is a norm, not a geometry bug.
- **A tiny, correct change that breaks a distant test badly** points at a
  tie in a discrete choice there, not at the change. Record it; do not
  retune the change to dodge it.

## Evidence

- Along a wall, before normalizing: the ball at -0.07 / -0.44 / -1.65 mm
  through the passes, the foot 0.1561 → 0.1572 m, every plan refused.
  After: the ball at 0.00000, the length constant, 18 of 18 run along.
- The ladder's five failures appeared with `.normalize()` in
  `delta_after_world_turn` and went away without it, deterministically. The
  largest change it made was below 1e-6.

## Related

- [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) — prerequisite: what `delta_after_world_turn`'s frame is.
- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — applies: the climb whose rungs-apart pattern sits on a tie.
- [Symptom is far from cause in the rig chain](../../engineering-practice/debugging/symptom-is-far-from-cause-in-the-rig-chain.md) — same-trap: a foot off its hold, caused by a bone's length.
- [A wall is run along on two steps of a lifted leap](../parkour/a-wall-is-run-along-on-two-steps-of-a-lifted-leap.md) — example: the move that found it.
