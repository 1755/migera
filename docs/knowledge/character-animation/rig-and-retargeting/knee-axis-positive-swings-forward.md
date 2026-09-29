---
title: Knee axis positive swings forward
description: "A positive rotation about stance::KNEE_AXIS (+X) swings a hanging leg toward -Z, which is forward only on a -Z-facing rig; puppet_base.gltf faces +Z. A walk once ran backward and knees bent backward for weeks. Read before any code that assumes a rig's facing or a rotation's sign."
type: lesson
status: current
tags:
  - rig
  - locomotion
  - math
  - testing
  - correctness
updated: 2026-09-28
code:
  - src/character/anim/stance.rs
  - src/character/anim/gait.rs
  - src/character/anim/rig.rs
  - src/character/anim/legik.rs
sources:
  - commit 2128c5e
  - commit 4c7fbc9
  - test a_planted_foot_travels_backward_relative_to_the_body
aliases:
  - KNEE_AXIS
  - backward walk
  - backward-bending knee
  - facing_sign
---

# Knee axis positive swings forward

A positive rotation about `stance::KNEE_AXIS` (`+X`) swings a hanging leg
toward `-Z`. That is forward **only on a rig that faces `-Z`**, like this
crate's synthetic rig. `puppet_base.gltf` faces `+Z`, so never hardcode a
facing.

## What happened

**The sign.** `stance.rs`'s doc comment claimed the opposite ("positive
swings a bone backward") until it was measured. `gait.rs` was written against
that comment, and the walk cycle ran **backward**. During stance the planted
foot travelled toward `-Z` relative to the hips: a body reversing over its
own feet.

Derivation, so it can be checked rather than recalled. A leg hangs along
`-Y`, and

```
Rx(t) * (0, -L, 0) = (0, -L*cos t, -L*sin t)
```

so positive `t` drives `z` negative. Measured on the real rig: `+0.3` rad
about `KNEE_AXIS` moves `LeftFoot` from `z = +0.002` to `z = -0.191`.

**The facing (commits 2128c5e, 4c7fbc9).** The derivation holds, but "`-Z` is
forward" is a property of the synthetic rig only. On `puppet_base.gltf`,
which faces `+Z`, the same rotation swings the leg BACKWARD. That shipped and
rendered as a backward-bending knee for weeks. `solve_leg_grounded`
hardcoded the IK branch reasoned out for a target ahead at `-Z`.

Knee rotations are applied NEGATED (`-flex`) because a knee only flexes
backward. That part was always right.

## Why it matters

Thirty-odd tests passed against a backward walk. They asserted angles,
symmetry, continuity and ordering, and a reversed gait satisfies all of
those. Nothing asserted a **direction**. Later, 30+ leg tests missed the
backward knee because they measured the *unsigned* thigh–shin angle, which
is the same whichever way the knee folds.

## How to apply

- Never hardcode a facing. `RigGeometry::forward()` measures it from
  `ankle → toe`. `stance::facing_sign()` reports `+1` or `-1` against the
  authored convention, and `stance_on_rig` / `walk_pose_on` apply it.
- Leg IK now builds both two-bone branches and keeps the one whose knee
  lands forward.
- For anything directional, assert the direction explicitly, against
  forward kinematics, not against the angle meant to produce it.
- Measure the knee with `RigGeometry::knee_fold_direction` (how the shin
  turns relative to the thigh). `knee_forward_offset` is unreliable near
  full extension, where the knee lies on the hip–ankle line by definition.

## Evidence

- `a_planted_foot_travels_backward_relative_to_the_body` in `gait.rs`
  compares the foot's hip-relative `z` early and late in stance.
- Commits 2128c5e (knee picks its IK branch by measurement) and 4c7fbc9
  (knee measured by its fold). The fold-based invariants cover both rigs,
  every phase, walk and run. Sabotage check: inverting the gait's facing
  sign fails at phase 0.000.

## Related

- [Unsigned measurements cannot see direction](../../engineering-practice/testing/unsigned-measurements-cannot-see-direction.md) — deeper: the general testing lesson from the backward knee.
- [Synthetic rig's leg segments are shifted a joint](./synthetic-rig-leg-segments-are-shifted-a-joint.md) — same-trap: another synthetic-vs-real rig difference in the legs.
- [A/B test on the same input](../../engineering-practice/measurement/ab-test-on-the-same-input.md) — applies: how the direction bug was isolated.
- [Kill stale processes before trusting BRP](../../engineering-practice/debugging/kill-stale-processes-before-trusting-brp.md) — contrast: an earlier "backward knee" reading came from a stale process, not this bug.
- [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) — same-trap: another leg-only coincidence that hid a general error.
