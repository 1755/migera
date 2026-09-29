---
title: 11.2 Medial/lateral and anterior/posterior balance in standing
description: "Standing balance: COP − COM = −K·ẍ (inverted pendulum, <8° sway); A/P via ankle plantar/dorsiflexors, M/L via a reciprocal hip abductor load/unload of the two legs; stiffness (not reactive) control; co-contracting abductors signal inefficiency. Read before idle sway, weight shift or ragdoll standing balance."
type: index
status: current
tags:
  - biomechanics
  - balance
  - ragdoll
  - character-animation
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §11.2, pp. 286–289 (PDF pp. 299–302)"
aliases:
  - standing balance
  - quiet stance
---

# 11.2 Medial/lateral and anterior/posterior balance in standing

> **Source:** Winter (2009) §11.2, pp. 286–289 ·
> [open PDF at p. 286](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=299) ·
> Up: [Chapter 11 — Biomechanical movement synergies](../INDEX.md)

Standing balance splits cleanly by plane. Fore–aft, the body pivots about the
ankles, and the plantar/dorsiflexors place the COP. Side to side, with feet
apart, the ankles hardly matter: the hip abductors/adductors of both legs act
as one bilateral synergy. They load one leg while unloading the other, which
moves the COP. In both planes the inverted-pendulum relation
COP − COM = −K·ẍ governs the COM, and the control is a stiffness in phase
with the sway, not a reactive correction. The section has no intro text
before its first subsection.

## Key facts

- COP − COM = −(I/Wd)·ẍ holds in A/P and M/L for sway < 8°; COP − COM is the balance error signal ([11.2.1](./11.2.1-quiet-standing.md)).
- A/P COP is set by the ankle plantar/dorsiflexors, M/L COP by the hip abductors/adductors ([11.2.1](./11.2.1-quiet-standing.md)).
- Left and right vertical forces oscillate about 50% BW, equal in size and 180° out of phase; quiet M/L COP stays within ~±1 cm ([11.2.1](./11.2.1-quiet-standing.md), Fig. 11.4).
- The COP is in phase with the COM at slightly larger amplitude, so balance is stiffness control, not reaction, and sensors stay on standby ([11.2.1](./11.2.1-quiet-standing.md)).
- In a 2-hour standing task, reciprocal gluteus medius activity (R_xy −0.677) is efficient, while co-contraction (R_xy +0.766) went with pain rising 4→32 of 40 ([11.2.2](./11.2.2-ml-balance-in-workplace-tasks.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [11.2.1 Quiet standing](./11.2.1-quiet-standing.md) | Eq. 11.3, ankle (A/P) vs hip load/unload (M/L) control, stiffness control, Fig. 11.4 magnitudes | before authoring idle sway or weight shift, or designing a standing balance controller for the ragdoll |
| [11.2.2 Medial lateral balance control during workplace tasks](./11.2.2-ml-balance-in-workplace-tasks.md) | Reciprocal vs co-contracting hip abductors, measured by cross-correlation; pain outcome | when choosing between symmetric and alternating hip-abductor control, or testing actuator phase relations |

## Relevance to migera

- **Weight shift = pelvis translation driven at the hips.** A lateral weight
  shift should be authored as pelvis translation plus frontal tilt, with leg
  IK absorbing it, not as a lean about the ankles. The fore–aft sway is the
  opposite: the whole body tilts about the ankles. This informs the
  `phase.rs` standing idle and the pelvis/ground layer (`pelvis.rs`).
- **Scale.** Quiet sway is centimetres of COP and a few % of body weight
  per foot. Anything larger is a deliberate posture change.
- **Balance controller template** for an unpinned active ragdoll: a PD on COM
  state that outputs a desired COM acceleration, turned into a COP target
  through K. The COP is then realized by ankle torque (A/P) and by opposite
  hip abduction torques (M/L), never by co-contracting both hips.

## Where to read in the book

- pp. 286–287 (PDF 299–300): Eq. 11.3 and **Fig. 11.4** (two-force-plate
  load/unload traces).
- pp. 288–290 (PDF 301–303): workplace study, **Figs. 11.5–11.6** (EMG with
  cross-correlation insets).

## Related

- [5.2.9 Kinematics and kinetics of the inverted pendulum model](../../ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.9-inverted-pendulum-model.md) — prerequisite: where Eq. 11.3 is derived for both planes.
- [11.3 Dynamic balance during walking](../11.3-dynamic-balance-during-walking/INDEX.md) — contrast: balance once the COM leaves the base of support.
