---
title: Walk-cycle IK and ground-lock bugs in the muscle solver
description: "Records four bugs in the deleted position-space muscle walk cycle (IK reach undercount, unreachable stride, knee-bend sign, ground-lock that never unlocked at full strength) that stretched legs up to 7.3x, and the frame-loop test that found them. Read for history or when a leg overstretches."
type: lesson
status: archived
tags:
  - ik
  - locomotion
  - correctness
  - testing
updated: 2026-09-23
sources:
  - test walk_cycle_never_overstretches_a_leg_segment_under_the_real_physics_loop (deleted with src/character/muscle)
aliases:
  - straight_leg_chain_to
  - apply_ground_lock
  - leg overstretch
  - STRIDE_LENGTH
---

# Walk-cycle IK and ground-lock bugs in the muscle solver

> **Archived:** describes `src/character/muscle`, deleted in commit 9981e16 (2026-09-25); see [The muscle module is deleted](../animation-core/muscle-deleted-anim-is-the-only-stack.md).

The muscle solver's walk cycle stretched leg segments to 2.24–7.3x their rest
length. The retargeting maths was fine. Four bugs in `walk_step`'s leg IK
and in `apply_ground_lock` caused it, each hiding the next. A test that
replayed the real per-frame loop found them.

## What happened

The user reported "almost all poses/animations broken, especially walk —
white line skeleton moves to opposite direction". The code lived in
`src/character/muscle/{pose,solve_muscle}.rs`.

1. **Reach undercount.** `straight_leg_chain_to` computed max reach as
   femur + shin (Hips → UpLeg → Leg). But `Foot` and `ToeBase` ride along
   past `Leg` at the same angle, so the true reach is femur + shin + foot.
   It undercounted by about 0.07 m (0.881 m against the T-pose's actual
   0.945 m hip-to-ankle). Every replant was clamped short, and `Foot` was
   then forced to the farther target anyway, stretching `Leg → Foot` up to
   2.25x. Fix: include the foot's rest length in the reach.
2. **Unreachable stride.** Even with the full 0.951 m reach, a 0.65 m stride
   needed about 1.0 m of reach at replant, since hip height alone is 0.94 m.
   Fixed by cutting `STRIDE_LENGTH` to 0.45 m and adding
   `REPLANT_HIP_DIP = 0.07` (the hip drops during replant, as in real gait).
   A geometry-only fix (about 0.28 m stride, no dip) would have brought back
   an earlier look already rejected as too stiff.
3. **Knee-bend sign.** Exposed only once bug 2 stopped clamping reach. The
   target angle used `Vec3::angle_between`, which is unsigned `[0, π]`, and
   the knee bend was subtracted from the femur angle. The fix needed a
   SIGNED angle about the fixed axis (`atan2` of in-plane components) and
   `shin_angle = femur_angle + knee_bend`. When wrong, the stretch was 7.3x,
   worse than the original bug.
4. **Ground lock never unlocked.** At strength 1.0,
   `blend_toward_target_positions` deliberately zeroes velocity
   (`joint.velocity *= 1.0 - strength`). The unlock check was a velocity
   threshold, so a locked foot could never unlock and fought every keyframe
   lift forever. Fixed by also unlocking when this frame's keyframe TARGET
   height exceeds `LOCK_HEIGHT + TARGET_UNLOCK_MARGIN` (0.002 m).

## Why it matters

Existing tests exercised only the pure target generation
(`advance_pose_transition`, `target_world_position`), never the real loop of
solve, blend and ground-lock together. Each bug masked the next: a
permanently clamped reach kept the knee bend near zero, so its sign error
never showed.

## How to apply

- Replicate the real per-frame body in a unit test instead of launching the
  GPU example. Here that reproduced the live 2.24–7.3x numbers in about
  0.05 s per run.
- Use signed angles about a known axis for anything with a bend direction.
- Check a limb's reach budget against the rig's real segment sum before
  tuning stride.

## Evidence

After the fixes: 487/487 tests passed. Live-verified with Left and Front
screenshots, gizmos on, mid-walk and idle. `LeftLeg → LeftFoot` and
`RightLeg → RightFoot` stayed within a few percent of rest length through a
full walk cycle.

## Related

- [Replicate the real frame loop in a unit test](../../engineering-practice/testing/replicate-the-real-frame-loop-in-a-unit-test.md) — deeper: the method that found these bugs.
- [Bisect before grepping dependency source](../../engineering-practice/debugging/bisect-before-grepping-dependency-source.md) — applies: measure your own systems first.
- [Unsigned measurements cannot see direction](../../engineering-practice/testing/unsigned-measurements-cannot-see-direction.md) — same-trap: bug 3's unsigned angle returned later in the knee-direction bug.
- [Two-bone IK pivots at the upper joint](./two-bone-ik-pivots-at-upper-not-root.md) — same-trap: another IK reach measured against the wrong segments.
- [The muscle module is deleted](../animation-core/muscle-deleted-anim-is-the-only-stack.md) — superseded-by: the stack that replaced this code.
- [Lugaru's joint/muscle animation system](../lugaru-joint-muscle-system.md) — prerequisite: the design the muscle solver ported, including its ground-locked walk.
- [migera pivots to Bevy PBR for characters](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — prerequisite: the architecture this walk cycle sat inside.
