---
title: 11.3 Dynamic balance during walking
description: "Walking balance as a moving inverted pendulum: the COM never passes over a foot; each swing-foot placement restores balance. Initiation moves the COP back and toward the swing leg first; termination brakes over two steps with a half-length final step. Read before start/stop transitions or foot placement."
type: index
status: current
tags:
  - biomechanics
  - balance
  - locomotion
  - character-animation
  - ragdoll
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §11.3, pp. 289–294 (PDF pp. 302–307)"
aliases:
  - dynamic balance
  - gait initiation and termination
---

# 11.3 Dynamic balance during walking

> **Source:** Winter (2009) §11.3, pp. 289–294 ·
> [open PDF at p. 289](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=302) ·
> Up: [Chapter 11 — Biomechanical movement synergies](../INDEX.md)

The standing balance law COP − COM = −K·ẍ still applies in walking, but the
COM is now outside the base of support for most of the cycle, so balance
cannot be *held*. It is re-established every step by where the swing foot
lands. The section follows one COP/COM story through three regimes: steady
walking (the target pattern), initiation (how to leave quiet standing and
reach that pattern within about one step) and termination (how to bleed off
momentum over the last two steps and park the COP ahead of the COM). All
three were measured with force plates plus a whole-body COM model, and the
inverted-pendulum model correlates at about −0.94 during initiation and
termination. The section has no intro text before its first subsection.

## Key facts

- The COM passes just medial of each stance foot and never over it; each single support (~40% of the cycle) is a fall toward the next foot ([11.3.1](./11.3.1-inverted-pendulum-in-steady-walking.md)).
- The swing foot's landing decides the next step's stability, and a walker is never more than ~400 ms from falling ([11.3.1](./11.3.1-inverted-pendulum-in-steady-walking.md)).
- The COM slows while behind the stance COP (first half of stance) and speeds up once ahead of it ([11.3.1](./11.3.1-inverted-pendulum-in-steady-walking.md)).
- Initiation: the COP first moves back (plantarflexors relax) and toward the swing leg (swing leg loaded), accelerating the COM forward and toward the stance leg ([11.3.2](./11.3.2-gait-initiation.md)).
- Steady-state COM motion is reached by the end of the first step; the pattern scales with final walking speed across age and disease ([11.3.2](./11.3.2-gait-initiation.md)).
- Termination: the penultimate stance leg's plantarflexors cut COM speed ~70% and do 85% of the braking; the final step is ~half length ([11.3.3](./11.3.3-gait-termination.md)).
- Hip abductors proactively park the final COP directly ahead of the COM path; without peripheral sensation the COP overshoots ([11.3.3](./11.3.3-gait-termination.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [11.3.1 The human inverted pendulum in steady state walking](./11.3.1-inverted-pendulum-in-steady-walking.md) | COM outside the base of support, foot placement as the balance strategy, COM speed modulation within stance, heel-strike foot angle | before changing pelvis sway, foot placement, root-motion speed profile, or designing a walking ragdoll |
| [11.3.2 Initiation of gait](./11.3.2-gait-initiation.md) | The release phase (anticipatory COP shift), event timeline −69% → 35%, speed scaling | before changing how `transition.rs` starts a walk or choosing the first swing leg |
| [11.3.3 Gait termination](./11.3.3-gait-termination.md) | Two-step braking, 70%/85% numbers, half-length final step, COP parked ahead of the COM | before changing how `transition.rs` stops a walk |

## Relevance to migera

- **Start/stop transitions (`transition.rs`)** already solve the rate problem
  (a cadence spring, a footfall-synchronized fade). The book adds the
  *postural* content they lack. A start needs a preparatory weight shift onto
  the stance leg and a slight forward tip before the first foot lifts. A stop
  needs braking one step early, a half-length final step, and a short settle.
- **Pelvis sway (`phase.rs` locomotion layer)** should carry the COM toward
  but not over the stance foot, and it must fade out within the final step of
  a stop.
- **Active ragdoll:** once its root is unpinned, walking balance needs
  COM-state-driven foot placement. Both transitions also need COP moves that
  look "wrong" (backward to start, forward to stop), which a naive
  "keep COM over feet" controller would forbid.

## Where to read in the book

- pp. 289–291 (PDF 302–304): steady walking and **Fig. 11.7** (COM vs COP,
  two steps, top view).
- pp. 291–293 (PDF 304–306): initiation and **Fig. 11.8**.
- pp. 293–294 (PDF 306–307): termination and **Fig. 11.9**.

## Related

- [5.2.9 Kinematics and kinetics of the inverted pendulum model](../../ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.9-inverted-pendulum-model.md) — prerequisite: the equation every note here reads its arrows with.
- [11.2 M/L and A/P balance in standing](../11.2-standing-balance-ml-and-ap/INDEX.md) — prerequisite: the ankle and hip load/unload mechanisms reused in initiation and termination.
- [IK and locomotion](../../../character-animation/ik-and-locomotion/INDEX.md) — applies: migera's gait, foot IK and pelvis notes.
