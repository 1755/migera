---
title: The walking pelvis turns ±4° with the stepping leg, and the chest against it
description: "Winter's transverse hip data times the pelvis's turn but cannot size it (the hip angle includes femoral rotation), so ±4° is Perry's: each side furthest forward at its heel contact. The chest twist is retimed to turn against it. Read before changing the walk's pelvic yaw or chest twist."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - springs
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/phase.rs
  - src/character/anim/stance.rs
  - src/character/anim/dho.rs
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §7.4.5, Figs 7.4–7.5 (PDF pp. 209–210)"
  - "Perry, Gait Analysis: Normal and Pathological Function (1992): pelvic rotation 4° forward and back"
  - "test locomotion::tests::the_pelvis_turns_with_the_stepping_leg_and_the_chest_against_it"
  - "test locomotion::tests::the_rendered_chest_turns_against_the_pelvis_on_time"
aliases:
  - pelvic rotation
  - pelvic yaw
  - PELVIC_ROTATION
  - thorax counter-rotation
---

# The walking pelvis turns ±4° with the stepping leg, and the chest against it

The walk's pelvis turns about the vertical, `phase::pelvic_rotation_at`
= −4°·cos(2π·cycle). Each side is furthest forward at its own heel
contact. The chest twist (`Spine1`) turns the other way, at its extremes
at the same moments, with the arm swing.

## Context

- **Before:** the pelvis did not turn at all. The authored `Spine1` twist
  (±5.1°) peaked at midstance, 0.19 of a stride behind the arms. Measured
  headless on `puppet_base`.
- **Winter cannot give the size.** The transverse hip data (§7.4.5) is
  small: rotator moments ±0.2 N·m/kg, H1-T absorption −0.15 W/kg at ~10 %.
  The frontal trick used for the roll (power / moment = angular velocity)
  gives the pelvis against the stance *femur*, and about the vertical the
  femur itself rotates in the world. That trick works for the roll because
  the stance thigh barely tilts sideways.
- **Winter does give the timing.** The stance hip's external rotators brake
  the pelvis turning over the stance limb just after heel contact, so the
  turn reverses there.

## Decision

- **Size ±4° (`PELVIC_ROTATION`), labelled outside Winter.** This is Perry's
  commonly cited figure for a normal walk.
- **Same pass as the roll and sway.** The turn is composed with the roll
  and applied in `stance::move_pelvis_over_feet`, about the loaded hip
  socket, with `Spine` turned back. The feet stay exact, the knees turn a
  little with the pelvis, and root motion is untouched.
- **Chest twist retimed a quarter cycle** (offset π/2). With `Spine`
  cancelling the pelvis's turn, `Spine1`'s twist *is* the chest's world
  turn.
- **`Spine1` on the arms' 0.03 s spring.** On 0.16 s the rendered chest
  kept ±1.8° of its ±5° and was uncorrelated with the pelvis; see
  [a gait-timed motion cannot ride a weighty spring](./a-gait-timed-motion-cannot-ride-a-weighty-spring.md).

## Measured

- **Live, 1.2 m/s (BRP):** the pelvis spans ±3.9°. At left foot-down it is
  +3.3° (left side ahead), at right foot-down −3.3°. The chest is −4.9° /
  +4.1° at the same moments. Pelvis–chest correlation is −0.97 (+0.03
  before the spring change).
- **Unchanged:** planted slide at the start 2.1 / 2.5 mm, root weave 15 mm.
- **Cost:** the turn rides the existing re-solve; `anim_bench` stays about
  4.3 µs per character per frame.

## Revisit when

- A source with measured pelvic and thoracic rotation joins
  `docs/books`: the ±4° and the chest's ±5° are the two unsourced sizes in
  the walk's frontal and transverse motion.
- Speed scaling matters: both are fixed across walking speeds.

## Related

- [The walking pelvis's roll](./walking-pelvic-obliquity-from-hip-abductor-power.md) — context: the roll it is composed with, and why that one could come from Winter.
- [A gait-timed motion cannot ride a weighty spring](./a-gait-timed-motion-cannot-ride-a-weighty-spring.md) — prerequisite: why `Spine1` moved to 0.03 s.
- [7.4.5 Sample moment and power curves](../../biomechanics-winter/ch07-three-dimensional-kinematics-and-kinetics/7.4-kinetic-analysis-reactions-and-moments/7.4.5-sample-moment-and-power-curves.md) — source: H1-T.
