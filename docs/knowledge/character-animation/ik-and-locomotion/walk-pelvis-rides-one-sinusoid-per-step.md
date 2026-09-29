---
title: The walk's pelvis rides one sinusoid per step
description: "The pelvis height the planted legs ask for is lumpy on puppet_base (44 m/s² live): the body 'drops onto each leg'. The walk rides one sinusoid per step fitted under it in single support; feet press ≤15 mm, lifted by the foot IK. Read before changing walk.rs's pelvis height or judging the bob."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - springs
updated: 2026-09-29
verified: 2026-09-29
code:
  - src/character/anim/walk.rs
  - src/character/anim/locomotion.rs
sources:
  - "test locomotion::tests::a_walking_body_rises_and_falls_smoothly"
  - "test locomotion::tests::a_planted_foot_stays_on_the_ground"
aliases:
  - pelvis bob
  - BOB_HARMONICS
  - walk jerk
---

# The walk's pelvis rides one sinusoid per step

`WalkCycle::pose` used to set the pelvis to the soft maximum of what the
planted feet ask for. On `puppet_base`, Winter's recorded leg angles make
that path lumpy: it falls 14 mm through late single support, is caught at
the next heel contact, and bumps again in the weight hand-over. Headless the
peak vertical acceleration was 9.5 m per cycle². Live it was far worse
(31.8 mm range, 44 m/s² peak), because where the path rode above what a
foot could reach, the foot IK dropped the pelvis for it. Viewers saw the
body dropping onto each leg.

## Decision

Fit the mean plus one sinusoid per step (stride harmonic 2) to that path,
kept at or below it through single support (`WalkCycle::smoothed_bob`).
Above it a planted foot floats, and this rig has no leg to spare at
midstance ([rig authored at critical extension](./rig-authored-at-critical-extension.md)).
Below it a foot presses into the floor and the foot IK lifts it by bending
the stance knee.

The result has Winter's centre-of-mass shape: highest at midstance (0.22 of
the stride), lowest just before heel contact (0.47). Planted feet press
15 / 11 / 7 mm at 0.7 / 1.2 / 1.6 m/s. At the slow walk the stance knee goes
to ~22°, where Winter's midstance knee is 15–20°.

## Alternatives measured

| Path | Peak accel (m/cycle²) | Worst press | Lowest point |
|---|---|---|---|
| Raw (before) | 9.5 | 0 | 0.42, late single support |
| Harmonics 2 + 4 | 2.6 | ~10 mm | still late single support |
| Harmonics 2 + 4 + 6 | 6.0 | ~1 mm | 0.42 |
| **Harmonic 2 (chosen)** | **0.98** | 15 mm | 0.47, before heel contact |

- **Re-solving both legs in the gait so the feet stay exactly on the
  floor: rejected.** The leg IK re-planes the leg about its fixed hinge. It
  moved the thigh 2.8° off Winter's (the fidelity test allows 2°), broke the
  left/right mirror, and jumped the root velocity 0.11 m/s.
- **A plain least-squares fit: rejected.** It floats a planted foot 3.3 mm
  at 1.2 m/s.
- **Holding the fit under the raw path in double support as well:
  rejected.** It chased the hand-over's narrow dips far down; on the
  synthetic rig the pelvis went 31 mm below standing.

## Live result (1.2 m/s)

Pelvis range 31.8 → 11.2 mm. Fastest fall 0.525 → 0.070 m/s. Vertical
acceleration p95 21 → 0.86 m/s², max 44 → 1.3 m/s².

## Related

- [Recorded pelvis path and leg angles conflict](./recorded-pelvis-path-and-leg-angles-conflict.md) — context: why the pelvis follows the legs rather than Winter's recorded path.
- [Rig authored at critical extension](./rig-authored-at-critical-extension.md) — prerequisite: why the fit may press, never float.
- [Walking foot rocker contact model](./walking-foot-rocker-contact-model.md) — context: the contacts the raw path is derived from.
- [The walk's step width and sideways sway](./walk-step-width-and-sideways-sway.md) — contrast: the pelvis's side-to-side motion, from the pendulum rather than the legs.
