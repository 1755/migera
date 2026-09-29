---
title: Chapter 5 — Kinetics: forces and moments of force
description: "Winter's planar inverse dynamics: link-segment model, per-segment Newton–Euler and the distal-to-proximal recursion, force plates and COP, gait moment curves and support moment, COP vs COM and the inverted-pendulum law, muscle forces. Read before joint-torque, balance or gait-plausibility work."
type: index
status: current
tags:
  - biomechanics
  - inverse-dynamics
  - balance
  - locomotion
  - muscle
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Ch. 5, pp. 107–138 (PDF pp. 120–151)"
aliases:
  - kinetics
  - inverse dynamics
---

# Chapter 5 — Kinetics: forces and moments of force

> **Source:** Winter (2009) Ch. 5, pp. 107–138 ·
> [open PDF at p. 107](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=120) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** Forces inside the body cannot be measured, so they
are computed: model the body as rigid point-mass links on pin joints (5.0),
write $\sum F = ma$ and $\sum M = I\alpha$ for each segment, and solve from the
foot upward, each joint's reaction force and net muscle moment becoming the
next segment's known load (5.1). The ground reaction and its centre of
pressure (COP) from a force plate close the chain in stance; the GRF itself
is just $\sum m_i a_i$ plus weight (5.2.2). The resulting moment curves are read
as a limb synergy — the support moment stays positive in single support
(5.2.6) — and must not be shortcut by GRF × lever arm (5.2.7). The COP is the
body's balance control signal: it must overshoot the COM to reverse it, and
$COP - COM = -(I/Wh)\,\ddot{COM}$ (5.2.8–5.2.9). Finally, muscles add large
compressive loads beyond the reaction force, and splitting a net moment into
muscles is indeterminate without assumptions (5.3).

**Start here:** [5.1](./5.1-link-segment-equations-free-body-diagram.md) for
the equations, then [5.2.9](./5.2-force-transducers-and-force-plates/5.2.9-inverted-pendulum-model.md)
for balance. Read [5.0](./5.0-biomechanical-models/INDEX.md) first if the
model's assumptions are unfamiliar.

## Key facts

- The model: fixed point masses at fixed COMs, pin joints, constant inertia and length — [5.0.1](./5.0-biomechanical-models/5.0.1-link-segment-model-development.md).
- Per segment: $R_{xp} - R_{xd} = ma_x$, $R_{yp} - R_{yd} - mg = ma_y$, $\sum M_{COM} = I_0\alpha$; solve forces before the moment, distal to proximal — [5.1](./5.1-link-segment-equations-free-body-diagram.md).
- Net moments hide co-contraction, friction and ligament loads — [5.0.2](./5.0-biomechanical-models/5.0.2-forces-on-the-link-segment-model.md).
- GRF = Σ mᵢ(aᵢ + g); walking Fy double-humps above body weight — [5.2.2](./5.2-force-transducers-and-force-plates/5.2.2-force-plates.md).
- Pushoff ankle moment ≈ −128.5 N·m vs ≈ 1.3 N·m in swing — [5.2.5](./5.2-force-transducers-and-force-plates/5.2.5-combined-force-plate-and-kinematics.md).
- Support moment $M_s = -M_a + M_k - M_h$ is positive through single support — [5.2.6](./5.2-force-transducers-and-force-plates/5.2.6-interpreting-moment-of-force-curves.md).
- GRF × lever-arm (FRFV) moments are wrong beyond the ankle and absent in swing — [5.2.7](./5.2-force-transducers-and-force-plates/5.2.7-wrong-way-to-analyze-moments.md).
- COP must lead/overshoot the COM; near the toes a step becomes mandatory — [5.2.8](./5.2-force-transducers-and-force-plates/5.2.8-center-of-mass-vs-center-of-pressure.md).
- $COP - COM = -(I/Wh)\ddot{COM}$; ankles control A/P, hip ab/adductors M/L — [5.2.9](./5.2-force-transducers-and-force-plates/5.2.9-inverted-pendulum-model.md).
- Running ankle compression > 5500 N (11 × BW), mostly muscle — [5.3.2](./5.3-dynamic-bone-on-bone-forces/5.3.2-scott-winter-1990-example.md).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [5.0 Biomechanical models](./5.0-biomechanical-models/INDEX.md) | Inverse solution; link-segment assumptions; load types; reaction vs bone-on-bone force | Before building or trusting a rigid-body body model |
| [5.1 Basic link-segment equations—the free-body diagram](./5.1-link-segment-equations-free-body-diagram.md) | Planar Newton–Euler per segment, recursion, sign conventions, three worked examples | Implementing inverse dynamics or a joint-torque test oracle |
| [5.2 Force transducers and force plates](./5.2-force-transducers-and-force-plates/INDEX.md) | GRF and COP, stance inverse dynamics, moment curves, FRFV error, COP vs COM, inverted pendulum | Gait kinetics, balance controllers, pelvis sway |
| [5.3 Bone-on-bone forces during dynamic conditions](./5.3-dynamic-bone-on-bone-forces/INDEX.md) | Muscle-force indeterminacy; equal-stress resolution; joint contact loads | Mapping torques to muscles or estimating joint loads |
| [5.4 Problems based on kinetic and kinematic data](./5.4-problems-kinetic-kinematic-data.md) | Exercises on the Appendix A walking trial with tabulated answers | Needing known-answer fixtures for an inverse-dynamics test |
| [5.5 References](./5.5-references.md) | The chapter's most useful primary sources | Tracing a claim to its paper |

## Relevance to migera

Chapter 5 is the physics that migera's procedural stack approximates and its
active ragdoll executes.

- **Test oracle for procedural motion.** The 5.1 recursion (with Winter's
  segment parameters from Ch. 4) turns any animated pose sequence into joint
  moments and an implied ground force (5.2.2). Checks: implied vertical GRF
  double-humped and averaging body weight; zero net horizontal impulse per
  stride at constant speed (5.4); support moment positive in single support
  (5.2.6); joint moments within human ranges. The worked examples (5.1, 5.2.5)
  and Appendix A tables are independent fixtures.
- **Ragdoll PD ceilings.** Stance torques are large and graded (ankle up to
  ~60–130 N·m at pushoff; knee and hip tens of N·m) while swing torques are
  ~1–3 N·m. That sets the *proportions* for per-joint `max_torque` in
  `src/character/anim/ragdoll.rs` (which, being accelerations, need dividing by
  the distal chain's inertia about the joint), and suggests stance/swing
  dependence.
- **Balance and pelvis sway.** The inverted-pendulum law (5.2.9) gives a
  per-frame feasibility check (implied COP inside the stance foot) for pelvis
  sway in `pelvis.rs`/`gait.rs`, a step-trigger criterion (COP demand leaves
  the foot), and the control split: A/P through ankles, M/L through hip
  abduction/adduction.
- **What not to do:** don't estimate knee/hip torque from contact force ×
  lever arm (5.2.7); don't treat the COP as a contact point (5.2.3).

## Where to read in the book

- pp. 107–112 (PDF 120–125): models and assumptions (Figs 5.1–5.4).
- pp. 112–117 (PDF 125–130): Eqs 5.1–5.3 and Examples 5.1–5.3 (Figs 5.5–5.8).
- pp. 117–124 (PDF 130–137): force plates, Eqs 5.4–5.8, Example 5.4 (Figs 5.9–5.13).
- pp. 124–127 (PDF 137–140): moment curves (Fig. 5.14), FRFV (Fig. 5.15).
- pp. 127–131 (PDF 140–144): COM vs COP, Eqs 5.9–5.10 (Figs 5.16–5.17).
- pp. 131–136 (PDF 144–149): muscle and bone-on-bone forces (Figs 5.18–5.21).

## See also

- [Chapter 4 — Anthropometry](../ch04-anthropometry/INDEX.md) — segment masses, COMs and inertias every equation here needs.
- [Chapter 6 — Mechanical work, energy, and power](../ch06-work-energy-and-power/INDEX.md) — joint power = moment × angular velocity, the next analysis step.
- [Chapter 7 — Three-dimensional kinematics and kinetics](../ch07-three-dimensional-kinematics-and-kinetics/INDEX.md) — the 3D version of the recursion.
- [Chapter 11 — Biomechanical movement synergies](../ch11-biomechanical-movement-synergies/INDEX.md) — support moment and balance in depth.
- [Ragdoll and physics](../../character-animation/ragdoll-and-physics/INDEX.md) — migera's active ragdoll that these moments inform.
- [IK and locomotion](../../character-animation/ik-and-locomotion/INDEX.md) — gait, pelvis and foot grounding that the balance law constrains.
