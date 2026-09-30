---
title: A pose delta names a world axis
description: "A LocalPose rotation names a WORLD axis, so FK must conjugate it by the accumulated bind, and a world-space correction (aim, grounding, look-at, grip) uses frame P = W(parent)·bind_local·B⁻¹; fixed in 026d9e8. Read before composing rotations in rig.rs, legik, armik or lookat."
type: lesson
status: current
tags:
  - retargeting
  - math
  - correctness
  - rig
updated: 2026-09-26
code:
  - src/character/anim/rig.rs
  - src/character/anim/legik.rs
  - src/character/anim/armik.rs
  - src/character/anim/lookat.rs
sources:
  - commit 026d9e8
  - CHARACTER_PROGRESS.md "The pose-space convention: forward kinematics disagreed with the renderer"
aliases:
  - pose-space convention
  - world_correction_frame
  - accumulate_world_rotations
---

# A pose delta names a world axis

A `LocalPose` rotation is authored against the synthetic T-pose, where every
bind is identity, so it names a **world** axis, not a local one. That world
is the character's frame as bound, which turns with the character, not the
scene's (see [a pose delta's world is the character's frame](./a-pose-deltas-world-is-the-characters-frame.md)). Forward
kinematics must conjugate it into the bone's accumulated bind frame, and
world-space corrections need their own frame.

## What happened

Two consequences were both wrong, and both were fixed in commit 026d9e8:

1. `rig::accumulate_world_rotations` must conjugate the delta into the
   bone's accumulated bind frame: `parent * bind_local(b) * [B(b)⁻¹ d B(b)]`.
   It used to compose `parent * bind * delta`. That disagreed with the
   renderer by up to 44°, while a comment claimed they matched.
2. A world-space correction (`aim_bone`, foot grounding, look-at, grip) is
   **not** pre-multiplied raw and **not** conjugated by the ancestors. Its
   frame is `P = W(parent) * bind_local(b) * B(b)⁻¹`. That is identity only
   when the ancestors are at rest, which is why errors grew wherever a
   solver moved a parent and then aimed its child.

## Why it matters

The bug hid for months because conjugating by a bind rotation is a no-op
when the delta's axis is parallel to the bind's: parallel rotations commute.
The leg chain is bound about X, and every walk-cycle leg delta is about X
too. The legs were never immune. They were only ever asked the one question
the bug cannot get wrong.

A test suite that only exercises commuting cases cannot tell a correct
composition order from a wrong one.

## How to apply

- Use `legik::world_correction_frame` for any world-space correction.
  `rotation_frame_of` and `parent_frame_of` no longer exist.
- Use `rig::delta_after_world_turn` to compose a delta after a world-space
  turn. The walk's arm-swing fix used it.
- Test compositions with deltas whose axis is NOT parallel to the bind.

## Evidence

Commit 026d9e8 (2026-09-27). `world_correction_frame` is used by
`legik.rs`, `armik.rs` and `lookat.rs` as of 2026-09-28.

## Related

- [A pose delta's world is the character's frame](./a-pose-deltas-world-is-the-characters-frame.md) — deeper: "world" here is the character as bound; it turns with the character, so scene conversions apply the turn at the boundary.
- [Conjugate pose deltas by the bind rotation](./conjugate-pose-deltas-by-the-bind-rotation.md) — prerequisite: the same conjugation at write-back time.
- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: tests that could not distinguish right from wrong.
- [Symptom is far from cause in the rig chain](../../engineering-practice/debugging/symptom-is-far-from-cause-in-the-rig-chain.md) — example: the frame bugs found right after this one.
- [Knee axis positive swings forward](./knee-axis-positive-swings-forward.md) — same-trap: another leg-only coincidence that hid a general error.
