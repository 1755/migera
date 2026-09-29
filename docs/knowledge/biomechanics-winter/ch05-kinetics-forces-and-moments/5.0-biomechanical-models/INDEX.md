---
title: 5.0 Biomechanical models
description: "Introduces the inverse solution and the link-segment model: its five rigid-body assumptions, the three load types (gravity, external force at a COP, net muscle moment), and joint reaction vs bone-on-bone force. Read before building or trusting any rigid-body model of the body."
type: index
status: current
tags:
  - biomechanics
  - inverse-dynamics
  - anthropometry
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §5.0, pp. 107–112 (PDF pp. 120–125)"
aliases:
  - inverse solution
---

# 5.0 Biomechanical models

> **Source:** Winter (2009) §5.0, pp. 107–112 ·
> [open PDF at p. 107](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=120) ·
> Up: [Chapter 5 — Kinetics: forces and moments of force](../INDEX.md)

Muscle and joint forces cannot practically be measured in people, so they are
computed indirectly: given full kinematics, accurate anthropometrics and the
measured external forces, an **inverse solution** of a **link-segment model**
yields every joint's reaction force and net muscle moment (Fig. 5.1). That
net moment is the most informative summary of all muscle activity at a joint,
and effects of training, therapy or surgery that are obscured in raw
kinematics show up clearly at this level.

The section fixes the model (rigid point-mass links on pin joints), the loads
on it (gravity, measured external force at a COP, net muscle moments), and
what cutting it into free bodies means (reaction forces vs actual
bone-on-bone contact forces).

## Key facts

- The model assumes fixed point masses at fixed COMs, pin/ball joints, constant inertia and constant segment length — [5.0.1](./5.0.1-link-segment-model-development.md).
- A distributed contact load is represented by one force vector at the centre of pressure — [5.0.2](./5.0.2-forces-on-the-link-segment-model.md).
- Net muscle moments hide co-contraction, joint friction (a few percent at moderate speed) and end-of-range ligament loads — [5.0.2](./5.0.2-forces-on-the-link-segment-model.md).
- A joint reaction force is not a bone-on-bone force; active muscle adds compression (0 N vs 70 N in Fig. 5.4) — [5.0.3](./5.0.3-joint-reaction-and-bone-on-bone-forces.md).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [5.0.1 Link-segment model development](./5.0.1-link-segment-model-development.md) | The five rigid-body assumptions every inverse solution inherits | Before modelling the body as rigid links or trusting a computed moment |
| [5.0.2 Forces acting on the link-segment model](./5.0.2-forces-on-the-link-segment-model.md) | Gravity, external force at a COP, net muscle moment and what the net hides | When interpreting a joint moment or a ragdoll joint torque |
| [5.0.3 Joint reaction forces and bone-on-bone forces](./5.0.3-joint-reaction-and-bone-on-bone-forces.md) | Free-body cut, Newton's third law, reaction vs contact force | Before calling a constraint/reaction force a joint load |

## Relevance to migera

The link-segment model *is* migera's skeleton and avian ragdoll: rigid
constant-length bones, one body per bone, ball joints, torque-motor
actuation. So Winter's kinetics transfers one-to-one, and his caveats
(net moments hide co-contraction; reaction ≠ contact force) are also the
limits of what the ragdoll's joint torques mean.

## Where to read in the book

- pp. 107–108 (PDF 120–121): inverse solution, Fig. 5.1, the five assumptions.
- p. 109 (PDF 122): Fig. 5.2 link-segment lower limb; the three force types.
- pp. 110–112 (PDF 123–125): Fig. 5.3 free-body diagram, Fig. 5.4 reaction vs bone-on-bone.
