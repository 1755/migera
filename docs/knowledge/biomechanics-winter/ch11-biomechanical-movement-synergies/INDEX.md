---
title: Chapter 11 — Biomechanical movement synergies
description: "Whole-body synergies: the support moment Ms = Mh+Mk+Ma (steady while hip/knee trade off; tracks vertical GRF), quiet-standing balance (COP−COM = −K·ẍ; ankle A/P, hip load/unload M/L, stiffness control), and walking balance incl. gait initiation and termination. Read before balance, start/stop or ragdoll support work."
type: index
status: current
tags:
  - biomechanics
  - balance
  - locomotion
  - ragdoll
  - character-animation
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), ch. 11, pp. 281–295 (PDF pp. 294–308)"
aliases:
  - synergies
  - balance control
---

# Chapter 11 — Biomechanical movement synergies

> **Source:** Winter (2009) ch. 11, pp. 281–295 ·
> [open PDF at p. 281](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=294) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** Single-joint moments are noisy and hard to
interpret. The CNS appears to control *task-level* quantities built from
several joints, and those are steady. Three such synergies are worked out.

(1) The **support moment** $M_s = M_h + M_k + M_a$ (extensors positive).
It stays consistent across days even though the hip and knee moments vary
60–68% and compensate for each other, and its double hump mirrors the
vertical ground reaction force (r = 0.97).

(2) **Standing balance.** The body is an inverted pendulum in which the COP's
offset from the COM sets COM acceleration:
$\mathrm{COP}-\mathrm{COM} = -(I/Wd)\,\ddot{x}$. The ankles place the COP
fore–aft; the hip abductors place it side to side by loading one leg and
unloading the other. The control is an in-phase stiffness, not a reactive
correction.

(3) **Walking balance.** The COM never passes over a foot, so balance is
re-won by each swing-foot placement. Starting to walk begins with the COP
moving *backward and toward the swing leg* (the release phase). Stopping
brakes over two steps with a half-length last step, and the COP is parked
just ahead of the COM.

**Start here:** for animation work read
[11.3.2 Initiation](./11.3-dynamic-balance-during-walking/11.3.2-gait-initiation.md) and
[11.3.3 Termination](./11.3-dynamic-balance-during-walking/11.3.3-gait-termination.md);
for a physics balance controller read
[11.2.1 Quiet standing](./11.2-standing-balance-ml-and-ap/11.2.1-quiet-standing.md) and
[11.3.1 Steady walking](./11.3-dynamic-balance-during-walking/11.3.1-inverted-pendulum-in-steady-walking.md).
All four lean on §5.2.8–5.2.9 (COM vs COP, inverted pendulum).

The chapter's intro (§11.0) defines a synergy as muscles collaborating
toward one goal over a stated time window. It notes that gait's subtasks
(propulsion, balance, preventing collapse) run at once and can compete.

## Key facts

- A synergy only makes sense once its goal and time window are stated; one muscle group can serve several subtasks at once ([11.0](./11.0-introduction.md)).
- Shoulder flexor moments trigger anticipatory posterior leg responses, extensor moments anterior ones ([11.0](./11.0-introduction.md)).
- $M_s = M_h + M_k + M_a$ (extensor +ve), which is $M_k - M_a - M_h$ in ch. 5 signs; stance CV is 20% vs 60–68% for knee and hip alone ([11.1](./11.1-support-moment-synergy/INDEX.md)).
- Ms tracks vertical GRF, r = 0.90–0.97 across cadences and knee-replacement patients ([11.1.1](./11.1-support-moment-synergy/11.1.1-support-moment-vs-vertical-grf.md)).
- COP − COM = −K·ẍ with K = I/(Wd), valid below 8° sway, in both A/P and M/L ([11.2.1](./11.2-standing-balance-ml-and-ap/11.2.1-quiet-standing.md)).
- Ankles control A/P and hips control M/L (load/unload, 180° out of phase), by stiffness control rather than reaction ([11.2.1](./11.2-standing-balance-ml-and-ap/11.2.1-quiet-standing.md)).
- Co-contracting hip abductors (R_xy +0.77) instead of alternating ones (−0.68) went with low-back pain in 2-hour standing work ([11.2.2](./11.2-standing-balance-ml-and-ap/11.2.2-ml-balance-in-workplace-tasks.md)).
- In walking the COM passes medial of each foot, and foot placement is the balance control; a walker is ≤ ~400 ms from falling ([11.3.1](./11.3-dynamic-balance-during-walking/11.3.1-inverted-pendulum-in-steady-walking.md)).
- Gait initiation: the COP moves back and toward the swing leg first, and steady state is reached within one step ([11.3.2](./11.3-dynamic-balance-during-walking/11.3.2-gait-initiation.md)).
- Gait termination: the penultimate stance leg's plantarflexors remove ~70% of speed (85% of the braking), the final step is ~½ length, and the COP ends ahead of the COM ([11.3.3](./11.3-dynamic-balance-during-walking/11.3.3-gait-termination.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [11.0 Introduction](./11.0-introduction.md) | Definition of a synergy; examples, including anticipatory postural responses to arm moves | before interpreting any single joint's torque, or adding secondary motion to a gesture |
| [11.1 The support moment synergy](./11.1-support-moment-synergy/INDEX.md) | Ms definition and sign conventions, hip–knee trade-off (covariance 89%), Ms ≈ vertical GRF | before judging stance-leg torques, knee-bend style, or a ragdoll support check |
| [11.2 M/L and A/P balance in standing](./11.2-standing-balance-ml-and-ap/INDEX.md) | Eq. 11.3, ankle vs hip load/unload control, stiffness control, reciprocal vs co-contraction | before authoring idle sway or weight shift, or designing a standing balance controller |
| [11.3 Dynamic balance during walking](./11.3-dynamic-balance-during-walking/INDEX.md) | COM outside the base of support, foot placement, gait initiation and termination sequences | before changing `transition.rs` start/stop, pelvis sway, or foot placement |
| [11.4 References](./11.4-references.md) | Annotated citations (Winter 1996/1998, Jian 1993, Gage 2003, …) | when you need a primary source for balance or gait transitions |

## Relevance to migera

This is among the most directly applicable chapters for `src/character/anim`.

- **Start/stop (`transition.rs`).** The rate handling (cadence spring,
  footfall-timed fade) is sound. What is missing is posture. A start needs an
  anticipatory weight shift onto the stance leg with a slight forward tip,
  and the unloaded leg steps first. A stop brakes one step early, shortens the
  final step to about half, and settles. See 11.3.2 and 11.3.3.
- **Pelvis and sway (`phase.rs`, `pelvis.rs`).** Side-to-side weight shift is
  pelvis translation driven by the hips, not an ankle lean. Quiet sway is
  centimetre-scale. Walking sway carries the COM toward but not over the
  stance foot, and must fade out during a stop. See 11.2.1 and 11.3.1.
- **Knee-bend stance (`stance.rs`, `gait.rs`).** People split the same
  support between hip and knee differently from day to day, so hip/knee
  flexion can be varied as a style parameter without changing support. See
  11.1.
- **Active ragdoll (`ragdoll.rs`, `ragdoll_plugin.rs`).** The root is pinned
  and gravity scaled today, so nothing balances. Unpinning it requires the
  chapter's controller structure:
  - COM PD → desired $\ddot{x}$ → COP target via K.
  - COP realized by ankle torque (A/P) and opposite hip abduction torques
    (M/L), never co-contraction.
  - Stepping when the COP would leave the foot.
  - Transitions that permit COP moves of the "wrong" sign.

  Verify leg support with the extensor-summed torque (Ms), not per-joint
  torques.

## Where to read in the book

- pp. 281–282 (PDF 294–295): introduction and synergy definition.
- pp. 282–286 (PDF 295–299): support moment, **Figs. 11.1–11.3**, Eqs. 11.1–11.2.
- pp. 286–290 (PDF 299–303): standing balance, **Eq. 11.3**, **Figs. 11.4–11.6**.
- pp. 289–294 (PDF 302–307): walking balance, initiation and termination, **Figs. 11.7–11.9**.
- p. 295 (PDF 308): references.

## Related

- [5.2.8 Differences between center of mass and center of pressure](../ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.8-center-of-mass-vs-center-of-pressure.md) — prerequisite: COP vs COM, read before the balance sections.
- [5.2.9 Kinematics and kinetics of the inverted pendulum model](../ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.9-inverted-pendulum-model.md) — prerequisite: derivation of the equation this chapter applies throughout.
- [5.2.6 Interpreting moment-of-force curves](../ch05-kinetics-forces-and-moments/5.2-force-transducers-and-force-plates/5.2.6-interpreting-moment-of-force-curves.md) — prerequisite: where the support moment is first introduced.
- [Character animation](../../character-animation/INDEX.md) — applies: the stack these findings inform.
- [Ragdoll and physics](../../character-animation/ragdoll-and-physics/INDEX.md) — applies: the ragdoll a balance controller would extend.
