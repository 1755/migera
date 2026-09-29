---
title: 5.3 Bone-on-bone forces during dynamic conditions
description: "Explains why muscles (linear, not torque, motors) add large joint compression and shear beyond the reaction force, why splitting a net moment into muscles is indeterminate, and the equal-stress resolution giving >5500 N ankle compression in running. Read before estimating muscle or joint loads."
type: index
status: current
tags:
  - biomechanics
  - muscle
  - inverse-dynamics
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §5.3, pp. 131–136 (PDF pp. 144–149)"
---

# 5.3 Bone-on-bone forces during dynamic conditions

> **Source:** Winter (2009) §5.3, pp. 131–136 ·
> [open PDF at p. 131](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=144) ·
> Up: [Chapter 5 — Kinetics: forces and moments of force](../INDEX.md)

A link-segment model drives each hinge with a torque motor, so its joint
reaction force would equal the force across the articular surface. Muscles
are linear motors: they pull along tendons and press the joint surfaces
together, so the true bone-on-bone force needs the muscle forces overlaid on
the free-body diagram (ligaments would matter too at end of range, but are
left out). Getting those muscle forces from a net moment is indeterminate
(5.3.1); the runner example (5.3.2) resolves it with equal-stress and
no-co-contraction assumptions and shows muscles dominate joint loading.

## Key facts

- A net joint moment is the sum over many muscles with time-varying moment arms (Eq. 5.11); nine muscles act at the knee — [5.3.1](./5.3.1-muscle-force-indeterminacy.md).
- Equal stress (force ∝ PCA) and no co-contraction make the ankle plantarflexor split solvable (Eq. 5.12) — [5.3.2](./5.3.2-scott-winter-1990-example.md).
- Running ankle compression exceeds 5500 N (> 11 × BW); the reaction supplies < 20% of it — [5.3.2](./5.3.2-scott-winter-1990-example.md).
- Plantarflexor pull cuts ankle shear from ~800 N to ~300 N (antishear) — [5.3.2](./5.3.2-scott-winter-1990-example.md).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [5.3.1 Indeterminacy in muscle force estimates](./5.3.1-muscle-force-indeterminacy.md) | Net moment ↔ many muscles is underdetermined (Eq. 5.11) | Before mapping a joint torque onto muscles or muscle-like actuators |
| [5.3.2 Example problem (Scott and Winter, 1990)](./5.3.2-scott-winter-1990-example.md) | Equal-stress resolution, attachment geometry (5.12–5.14), 11 × BW ankle load | Estimating muscle or joint contact loads from a net moment |

## Relevance to migera

Mostly background. migera's ragdoll is a torque-motor link-segment model, so
it has no indeterminacy and its joint constraint forces are reaction forces,
far below real contact loads. The ideas become relevant only if muscle-like
actuators return; then Eq. 5.11 (co-contraction raises stiffness at zero net
torque) and Eqs 5.13–5.14 (attachment geometry, effective line of pull) are
the starting point.

## Where to read in the book

- p. 131 (PDF 144): torque motors vs linear motors; Eq. 5.11.
- p. 132 (PDF 145): Fig. 5.18, fifteen lower-limb muscles.
- pp. 133–136 (PDF 146–149): Figs 5.19–5.21, Eqs 5.12–5.14, results.
