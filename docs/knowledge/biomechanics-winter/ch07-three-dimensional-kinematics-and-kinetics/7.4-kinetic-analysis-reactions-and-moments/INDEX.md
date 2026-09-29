---
title: 7.4 Kinetic analysis of reaction forces and moments
description: "3D inverse dynamics of one segment: Newton in the global frame for joint forces (Eq. 7.8), Euler's principal-axis equations with (I_a−I_b)ω_aω_b coupling for moments (Eq. 7.9), a worked knee, 3D joint power, averaged gait curves. Read before ragdoll torque/gyroscopic reasoning or gait-timing design."
type: index
status: current
tags:
  - biomechanics
  - inverse-dynamics
  - physics
  - ragdoll
  - energetics
  - locomotion
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §7.4, pp. 188–198 (PDF pp. 201–211)"
---

# 7.4 Kinetic analysis of reaction forces and moments

> **Source:** Winter (2009) §7.4, pp. 188–198 ·
> [open PDF at p. 188](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=201) ·
> Up: [Chapter 7 — Three-dimensional kinematics and kinetics](../INDEX.md)

With [G to A] matrices and body-frame ω, α in hand, the segment-by-segment
inverse dynamics of Chapter 5 goes 3D. The section's intro sets the rule that
organises everything: **joint reaction forces are computed in the global
frame** (gravity, force-plate data and COM accelerations are global), and
**joint moments in the anatomical frame** (where the inertia tensor is
diagonal). Newton's three equations give the proximal force; the forces and
the distal moment are rotated into anatomical axes; Euler's three equations
give the proximal moment; the result is rotated back to global to become the
next segment's distal load. A worked stance-phase knee, the 3D joint-power
formula, and averaged stride curves (Eng & Winter 1995) close the section.

## Key facts

- Forces in world frame, moments in the body's principal frame — [7.4.1](./7.4.1-newton-3d-equations-of-motion.md).
- Euler: I_xα_x + (I_z − I_y)ω_yω_z = ΣM_x (and cyclic); reaction forces have no moment about the long axis — [7.4.2](./7.4.2-euler-3d-equations-of-motion.md).
- The gyroscopic coupling is ~0.01 % of the knee moment in walking; stance moments are > 99 % reaction-force statics — [7.4.2](./7.4.2-euler-3d-equations-of-motion.md), [7.4.3](./7.4.3-kinetic-data-set-example.md).
- Worked frame-6 knee moments: abductor 42.35, axial 20.78, flexor −26.11 N·m — [7.4.3](./7.4.3-kinetic-data-set-example.md).
- 3D joint power = Σ over axes of moment × joint angular velocity — [7.4.4](./7.4.4-joint-mechanical-powers.md).
- Ankle push-off (A2-S, ~50 % stride) is the largest power burst; hip abductors carry the pelvis in the frontal plane — [7.4.5](./7.4.5-sample-moment-and-power-curves.md).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [7.4.1 Newtonian three-dimensional equations of motion for a segment](./7.4.1-newton-3d-equations-of-motion.md) | Eqs 7.8a–c and the three-step procedure | When computing or checking per-body joint forces |
| [7.4.2 Euler's three-dimensional equations of motion for a segment](./7.4.2-euler-3d-equations-of-motion.md) | Eqs 7.9a–c, principal axes, size of the gyroscopic term | Before reasoning about ragdoll spin/precession or PD-as-acceleration trade-offs |
| [7.4.3 Example of a kinetic data set](./7.4.3-kinetic-data-set-example.md) | Tables 7.3–7.4 → knee moments, re-verified; book typos | When a known-answer 3D dynamics fixture is needed |
| [7.4.4 Joint mechanical powers](./7.4.4-joint-mechanical-powers.md) | Eqs 7.10a–b and the generation/absorption sign | When diagnosing energy injection by a joint controller |
| [7.4.5 Sample moment and power curves](./7.4.5-sample-moment-and-power-curves.md) | 3D stride moment/power patterns and approximate per-kg peaks | When tuning gait phase timing, pelvis motion or ragdoll torque budgets |

## Relevance to migera

migera's ragdoll is a forward simulation on avian, so this inverse method is
a verification tool, not runtime code: its equations close per body in any
correct physics state, its worked example is a fixture, and its magnitudes
bound what human-like joint torques should be. Two design lessons carry over
directly: keep forces in world and moments in the body frame, and — because
the ωω coupling is tiny at walking speeds — the PD controller's choice to
prescribe angular acceleration rather than torque
([avian: apply_angular_acceleration](../../../character-animation/ragdoll-and-physics/avian-apply-angular-acceleration-not-torque.md))
loses nothing that matters for walking, only for fast tumbling.

## Where to read in the book

- pp. 188–189 (PDF 201–202): frame rule, Eq. 7.8.
- pp. 190–191 (PDF 203–204): Fig. 7.3, Eq. 7.9.
- pp. 191–194 (PDF 204–207): worked example, Tables 7.3–7.4.
- pp. 194–198 (PDF 207–211): power, Figs 7.4–7.5 and their interpretation.
