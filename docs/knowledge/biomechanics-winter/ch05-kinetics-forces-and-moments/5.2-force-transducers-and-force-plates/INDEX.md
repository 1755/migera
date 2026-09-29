---
title: 5.2 Force transducers and force plates
description: "Covers measuring the ground reaction (transducers, force plates, COP formulas, GRF = Σ m·a), combining it with kinematics for stance-phase joint moments, reading gait moment curves and the support moment, the wrong FRFV shortcut, and COP vs COM balance via the inverted pendulum. Read for gait kinetics or balance work."
type: index
status: current
tags:
  - biomechanics
  - inverse-dynamics
  - balance
  - locomotion
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §5.2, pp. 117–131 (PDF pp. 130–144)"
aliases:
  - force plate
  - ground reaction force
---

# 5.2 Force transducers and force plates

> **Source:** Winter (2009) §5.2, pp. 117–131 ·
> [open PDF at p. 117](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=130) ·
> Up: [Chapter 5 — Kinetics: forces and moments of force](../INDEX.md)

The external force needed to close an inverse solution is almost always the
ground reaction, measured by a force plate along with its centre of pressure
(COP). This section goes from the hardware to what the measurements mean: the
ground reaction is the sum of every segment's mass × acceleration; combined
with foot kinematics it gives the stance ankle moment and starts the
recursion; the resulting moment curves are read as a total-limb synergy (the
support moment); shortcuts that skip the recursion are wrong; and the COP —
not to be confused with the COM — is the control signal of balance, tied to
the COM by the inverted-pendulum equation.

Intro content (before 5.2.1): a force transducer turns a tiny strain into an
electrical signal — strain gauge bridges, piezoelectric crystals (quartz), or
piezoresistive crystals.

Children fall into three groups: measurement (5.2.1–5.2.4), inverse dynamics
in stance and its interpretation (5.2.5–5.2.7), and balance (5.2.8–5.2.9).

## Key facts

- COP from a four-corner plate is the vertical-force-weighted average position (Eqs 5.4–5.5); unreliable below ~2% body weight — [5.2.2](./5.2.2-force-plates.md).
- Ground reaction = Σ mᵢaᵢ (+ weight); walking Fy is double-humped above body weight, Fx brakes then propels — [5.2.2](./5.2.2-force-plates.md).
- The COP can sit under the arch where there is no pressure — [5.2.3](./5.2.3-pressure-measuring-systems.md).
- Pushoff ankle moment ≈ −128.5 N·m (plantarflexor) vs 1.34 N·m in swing — [5.2.5](./5.2.5-combined-force-plate-and-kinematics.md).
- Support moment Ms = −Ma + Mk − Mh stays positive in single support despite joint-level variability — [5.2.6](./5.2.6-interpreting-moment-of-force-curves.md).
- GRF × lever arm (FRFV) is fine at the ankle, badly wrong at the hip, blind in swing — [5.2.7](./5.2.7-wrong-way-to-analyze-moments.md).
- The COP must overshoot the COM to reverse it; near the toes no COP shift suffices and a step is forced — [5.2.8](./5.2.8-center-of-mass-vs-center-of-pressure.md).
- COP − COM = −(I/Wh)·CÖM; A/P via ankles, M/L via hip ab/adductors; r ≈ −0.95 A/P — [5.2.9](./5.2.9-inverted-pendulum-model.md).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [5.2.1 Multidirectional force transducers](./5.2.1-multidirectional-force-transducers.md) | Transducer principles; multi-axis = orthogonal single-axis units | Only when simulating a force sensor |
| [5.2.2 Force plates](./5.2.2-force-plates.md) | COP formulas (5.4–5.6), gait GRF shapes, GRF = Σ m·a (5.7–5.8) | Computing a COP, or checking a gait's implied ground force |
| [5.2.3 Special pressure-measuring sensory systems](./5.2.3-pressure-measuring-systems.md) | COP is an average, not a contact point; pressure insoles | Before snapping anything to "the COP" in foot grounding |
| [5.2.4 Synchronization of force plate and kinematic data](./5.2.4-force-plate-kinematic-synchronization.md) | Separate systems need sync pulses | Combining two independently clocked data streams |
| [5.2.5 Combined force plate and kinematic data](./5.2.5-combined-force-plate-and-kinematics.md) | Stance-foot free-body solution, Example 5.4 fixture | Writing the stance foot step of an inverse-dynamics oracle |
| [5.2.6 Interpretation of moment-of-force curves](./5.2.6-interpreting-moment-of-force-curves.md) | Stance ankle/knee/hip moment roles; support moment | Sizing ragdoll joint strength or judging a gait's torques |
| [5.2.7 A note about the wrong way to analyze moments of force](./5.2.7-wrong-way-to-analyze-moments.md) | Three errors of the FRFV shortcut | Before estimating joint torque from contact force × lever arm |
| [5.2.8 Differences between center of mass and center of pressure](./5.2.8-center-of-mass-vs-center-of-pressure.md) | COP as controller; five-phase sway cycle; step criterion | Designing balance, idle sway or a step trigger |
| [5.2.9 Kinematics and kinetics of the inverted pendulum model](./5.2.9-inverted-pendulum-model.md) | COP − COM ∝ −CÖM (5.9–5.10); ankle vs hip control; validation | Building a balance controller, pelvis sway or foot placement |

## Relevance to migera

The section supplies three oracles and one controller law for
`src/character/anim`: implied ground force (Eqs 5.7–5.8) to check a gait's
pelvis bob and speed changes; stance-phase inverse dynamics (5.2.5) plus the
support moment (5.2.6) to check that an animated leg actually holds the body
up and to size the ragdoll's per-joint ceilings in proportion; and the
inverted-pendulum equation (5.2.9) as both a per-frame "COP inside the foot"
test for pelvis sway and the core of any future ragdoll balance controller —
with M/L balance driven from the hips and a step triggered when the COP
demand leaves the foot.

## Where to read in the book

- pp. 117–121 (PDF 130–134): transducers, force plates, Figs 5.9–5.11, Eqs 5.4–5.8.
- pp. 121–123 (PDF 134–136): pressure insoles (Fig. 5.12), synchronization.
- pp. 123–126 (PDF 136–139): Example 5.4 (Fig. 5.13), moment curves (Fig. 5.14).
- pp. 126–127 (PDF 139–140): FRFV (Fig. 5.15).
- pp. 127–131 (PDF 140–144): COM vs COP (Figs 5.16–5.17), Eqs 5.9–5.10.
