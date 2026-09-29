---
title: A lagging pelvis rotation slides planted feet
description: "Both legs hang from Hips, so a pelvis roll springing slower than the legs swings the feet: a 4° roll at 0.16 s moved the rendered toe 4.2 cm, hidden by the foot lock until a first step. Hips springs with the legs (0.015 s). Read before tuning springs or posing the pelvis over planted feet."
type: lesson
status: current
tags:
  - springs
  - locomotion
  - ik
  - correctness
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/dho.rs
  - src/character/anim/stance.rs
  - src/character/anim/transition.rs
sources:
  - "live BRP + temporary trace of solve_foot_ik's animated toe vs lock target, character_gallery --anim-speed-schedule 0:0,4:1.2"
aliases:
  - hips spring
  - pelvis roll lag
  - weight shift foot slide
---

# A lagging pelvis rotation slides planted feet

Any pose that moves the pelvis over planted feet is only foot-preserving
if the whole leg chain arrives with it. That chain starts at `Hips`:
the root translation (not sprung at all), the hips rotation, then the leg
bones. The legs were already near-instant (0.015 s half-life, see the
comment on `dho::default_springs`) because they carry contact geometry.
The hips rotation was still on the spine's weighty 0.16 s.

## What happened

`stance::shift_weight` (the idle's weight shift, and the release before a
first step) rolls the pelvis ~0.07 rad about forward and re-solves both
legs so the toes stay put. In the TARGET pose they did, to 0.01 mm. In
the RENDERED pose, the roll lagged the leg rotations and swung both legs
about the hip joints: the left toe drifted 4.2 cm toward the midline
(0.07 rad × ~0.95 m of leg bounds it at 6.6 cm).

Nothing looked wrong while standing: the foot lock (`footlock.rs`) pins a
still toe, so the IK hid the error. When the first step began, the lock
let go and the hidden 4 cm came out as a sideways slide of the planted
foot, with the whole walk ending up 5 cm off to one side.

## The rule

- Everything between the root and the feet springs as fast as the legs.
  `Bone::Hips` is on `SpringParams::critical(0.015)`. So is the first spine
  bone, `Bone::Spine`, which carries the trunk's counter-roll: on 0.16 s it
  let the rendered trunk roll 8.1° with the walking pelvis where the target
  held it upright
  ([pelvic obliquity](./walking-pelvic-obliquity-from-hip-abductor-power.md)).
  `Spine1` and `Spine2` keep 0.16 s, so the trunk still has weight.
- A pose check on the TARGET is not a check on the screen. A stance or
  gait change that holds the feet in the target must also be checked on
  the sprung pose (what `solve_foot_ik` calls `animated`), with the foot
  lock out of the way: a lock makes a sliding animation look planted.

## Measured

Left toe, rendered vs. locked target, through a 0.5 s release:
42 mm off with Hips at 0.16 s, ≤ 5 mm at 0.015 s. The full suite (996
tests) and the steady-walk slide tests are unchanged.

## Related

- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — prerequisite: why the legs are already on a 0.015 s spring, and the same "rendered, not target" rule for root motion.
- [Foot IK feedback loops](./foot-ik-feedback-loops.md) — context: how the IK stage samples the animated pose it corrects.
- [11.3.2 Gait initiation](../../biomechanics-winter/ch11-biomechanical-movement-synergies/11.3-dynamic-balance-during-walking/11.3.2-gait-initiation.md) — context: the release phase that exposed it.
- [The walking pelvis's roll](./walking-pelvic-obliquity-from-hip-abductor-power.md) — same-trap: the Spine spring lagging the trunk's counter-roll.
