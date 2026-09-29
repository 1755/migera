---
title: A gait-timed motion cannot ride a weighty spring
description: "A critical spring passes a stride-rate motion at |H| = λ²/(λ²+ω²), lagging 2·atan(ω/λ). At the spine's 0.16 s and a 0.9 Hz stride: 0.36 of it, 107° late. Arms, the trunk's counter-roll and the chest twist each broke this way. Read before giving a gait-timed bone a slow spring, or judging a timed motion by its target."
type: lesson
status: current
tags:
  - springs
  - locomotion
  - correctness
  - verification
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/dho.rs
  - src/character/anim/phase.rs
sources:
  - "test locomotion::tests::the_rendered_chest_turns_against_the_pelvis_on_time"
  - "test locomotion::tests::the_rendered_trunk_stays_upright_while_the_pelvis_rolls"
  - "live BRP, character_gallery --anim-speed-schedule 0:0,4:1.2,10:0"
aliases:
  - spring lag
  - spring attenuation
  - halflife and cadence
---

# A gait-timed motion cannot ride a weighty spring

Every bone's target passes through a critical spring (`dho.rs`) before it
is rendered. A spring with half-life `h` has rate `λ = ln 2 / h`. Driven at
the stride's angular frequency `ω`, it passes:

- amplitude `|H| = λ² / (λ² + ω²)`;
- a phase lag of `2·atan(ω / λ)`.

A motion timed by the gait (arm swing, a chest twist, a counter-rotation
that must cancel the pelvis) keeps its timing on screen only if `λ ≫ ω`.

| half-life | amplitude at 0.93 Hz | lag |
|---|---|---|
| 0.16 s (the trunk's "weight") | 0.36 | 107° (~0.3 stride) |
| 0.03 s (arms, Spine1) | ~0.9 | ~28° |
| 0.015 s (legs, Hips, Spine) | ~0.97 | ~15° |

## What happened, three times

- **Arms.** On 0.12 s the arm swing lagged a quarter cycle, and the
  opposite hand led the forward foot only half the time. Moved to 0.03 s,
  with `gait::ARM_LAG` for the rest.
- **The trunk's counter-roll.** The walk rolls the Hips and rolls `Spine`
  back so the trunk stays upright. On 0.16 s the counter-roll arrived late,
  and the rendered trunk rolled 8.1° peak to peak live (8.5° headless) where
  the target held it within 1°. `Spine` moved to 0.015 s, with the Hips.
- **The chest twist.** `Spine1` twists the chest against the pelvis,
  timed to the heel contacts. On 0.16 s the rendered chest swung ±1.8°
  where the target asked ±5°, and was uncorrelated with the pelvis (+0.03).
  `Spine1` moved to 0.03 s: live ±4.7°, correlation −0.97.

## Why it matters

A test on the target pose cannot see any of these, because the target is
right. The retimed chest twist passed its target test while the screen was
still wrong. Worse, the old mistimed target had looked roughly right after
the spring's lag: the bug and the lag partly cancelled.

## How to apply

- A bone whose motion is timed to the stride goes on a fast spring. Put
  weight where nothing is timed: `Spine2`, Neck and Head keep slow springs
  and carry only breathing and looks.
- Check a timed motion on the **sprung** pose: run `DhoState` over the walk
  in a test, as `the_rendered_chest_turns_against_the_pelvis_on_time` does.
- A counter-rotation (one bone cancelling another's turn) must be on the
  same spring as the bone it cancels.

## Related

- [A lagging pelvis rotation slides planted feet](./a-lagging-pelvis-rotation-slides-planted-feet.md) — same-trap: the Hips lagging the legs.
- [The walking pelvis's roll](./walking-pelvic-obliquity-from-hip-abductor-power.md) — example: the counter-roll this broke.
- [The walking pelvis's turn](./walking-pelvic-turn-and-chest-counter-twist.md) — example: the chest twist this broke.
