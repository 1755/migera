---
title: Appendices
description: Winter's two appendices — A, a complete 2D walking-trial data set (56.7 kg, 106 frames at 69.9 Hz, raw to joint power) that all the problems use, and B, SI units and definitions. Read when you need reference gait numbers, a known-answer fixture, or a unit definition.
type: index
status: current
tags:
  - biomechanics
  - locomotion
  - inverse-dynamics
  - verification
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Appendices A–B, pp. 296–366 (PDF pp. 309–379)"
---

# Appendices

> **Source:** Winter (2009) Appendices A–B, pp. 296–366 ·
> [open PDF at p. 296](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=309) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

The back matter is almost entirely data. **Appendix A** (65 pages) is one
right-side walking stride of a 56.7 kg subject. It is carried through every
stage of the book's 2D pipeline: raw marker coordinates, filtered
coordinates with derivatives, segment kinematics, joint angles, joint
reaction forces and moments, segment energies, and joint powers. The
problem sets of Chapters 3–6 are exercises on it. The tables can also be
downloaded from the publisher (`http://www.wiley.com/go/biomechanics`, per
the Preface). **Appendix B** (6 pages) lists the SI units and the book's
definitions of the mechanical and electrical quantities.

## Key facts

- The trial is 106 frames at 69.9 Hz. Heel contact (right) is at frames 28 and 97, toe-off at 1 and 70, so the stride is 0.987 s with 61 % stance, at about 1.43 m/s ([Appendix A](./a-walking-trial-kinematic-kinetic-energy-data.md)).
- The knee flexes 16° in loading response and peaks at 66.6° in swing; the hip ranges −6.2° to 22.6°; the ankle reaches −20.5° plantarflexion just after toe-off ([Appendix A](./a-walking-trial-kinematic-kinetic-energy-data.md)).
- Vertical GRF is double-humped (604.5 N and 612.1 N, about 1.1 body weight, with a 362 N trough). The ankle plantarflexor moment peaks at −89.8 N·m and push-off power at 272 W ([Appendix A](./a-walking-trial-kinematic-kinetic-energy-data.md)).
- A.4's ankle angle is positive in **dorsiflexion**, opposite to the §3.5.2 formula. A.5 moments act on the named segment and are counterclockwise-positive ([Appendix A](./a-walking-trial-kinematic-kinetic-energy-data.md)).
- Power is P = M·ω for a moment and P = F·V for a force. Planar angular momentum is I·ω about the centroid ([Appendix B](./b-si-units-and-definitions.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [Appendix A — Kinematic, kinetic, and energy data (walking trial)](./a-walking-trial-kinematic-kinetic-energy-data.md) | Trial parameters, sign conventions, a page/column map for Tables A.1–A.7, key gait ranges, a reference curve every third frame | validating `gait.rs` / pelvis bob / foot timing against a real stride, sizing ragdoll torque ceilings, or writing an inverse-dynamics test |
| [Appendix B — Units and definitions](./b-si-units-and-definitions.md) | SI base units, derived-quantity definitions, notation rules | unsure of a unit, or of how the book defines momentum, power or energy |

## Relevance to migera

Appendix A is the most directly usable part of the book for
`src/character/anim`. It is a real, internally consistent stride that can
serve as the oracle for walk-cycle tests: joint-angle landmarks against
phase, stance/swing timing, pelvis vertical excursion (4.8 cm), toe
clearance (1.5 cm), COP travel, and the moment magnitudes a ragdoll's PD
controller must be able to produce at walking speed. Appendix B is
reference only.

## Where to read in the book

- p. 296 (PDF 309): Fig. A.1, marker set, body mass, frame rate.
- pp. 341–345 (PDF 354–358): Table A.4, joint angles, the most useful single table.
- pp. 346–352 (PDF 359–365): Tables A.5(a)–(b), GRF, COP, joint moments.
- pp. 361–366 (PDF 374–379): Appendix B.
