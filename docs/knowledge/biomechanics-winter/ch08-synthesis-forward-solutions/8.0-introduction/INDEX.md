---
title: 8.0 Introduction
description: Contrasts forward solutions (neural drive → moments → accelerations → motion) with inverse dynamics, explains why interlimb force coupling forces a whole-body model and makes errors accumulate, then lists the requirements and the internal-validity test. Read before building any torque-driven character.
type: index
status: current
tags:
  - biomechanics
  - physics
  - ragdoll
  - inverse-dynamics
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §8.0, pp. 200–202 (PDF pp. 213–215)"
aliases:
  - forward solution
  - internal validity
---

# 8.0 Introduction

> **Source:** Winter (2009) §8.0, pp. 200–202 ·
> [open PDF at p. 200](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=213) ·
> Up: [Chapter 8 — Synthesis of human movement, forward solutions](../INDEX.md)

A forward solution simulates the real causal chain: neural drive recruits
agonists and antagonists, their net effect is a moment at each joint, the
moments accelerate the segments, and integration gives the displacements a
camera would record. It is far harder than the inverse solution because it
cannot be done locally. Every movable segment must be modelled, and any
flaw in the model shows up as trajectory error that grows with time.

## Key ideas

- **Inverse is local, forward is global.** Inverse dynamics for the ankle
  and knee needs only foot and leg data. A forward model needs *every*
  segment that can move. If part of the body is fixed in space, every
  segment that is still free must be modelled.
- **Interlimb coupling.** A joint moment acts on both adjacent segments,
  and the resulting reaction forces pass the effect on to segments further
  away. Winter's example: the push-off plantarflexor moment in gait
  changes the knee reaction forces. Those change the thigh's acceleration,
  and in turn the hip, the contralateral hip and the trunk. So the ankle
  muscles affect the acceleration of *all* segments at that instant.
- **Errors accumulate.** A wrong segment mass, or a joint with a missing or
  unrealistic constraint, produces displacement errors that grow over time.
  This happens even when the input moment histories are exactly right.
- The inputs are the net muscle moments at each joint plus the initial
  position and velocity of every segment.

The two children state the formal requirements
([8.0.1](./8.0.1-forward-model-assumptions-and-constraints.md)) and what a
valid model would be good for, gated by the internal-validity test
([8.0.2](./8.0.2-forward-simulation-potential.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [8.0.1 Assumptions and constraints of forward solution models](./8.0.1-forward-model-assumptions-and-constraints.md) | The six requirements: ch. 5 link-segment assumptions, no kinematic constraints, full initial state, forces/moments as the only inputs, all DOF with passive range limits, computed ground reactions | when deciding what a ragdoll must model and which shortcuts break "simulation" |
| [8.0.2 Potential of forward solution simulations](./8.0.2-forward-simulation-potential.md) | "What would happen if...?" uses; the internal-validity test: inverse moments fed forward must reproduce the measured motion | before using a physics model to answer anything, or when designing a ragdoll round-trip test |

## Relevance to migera

The ragdoll is exactly this causal chain, cut short: PD torques stand in
for the net muscle moments. The coupling argument explains an effect seen
live in migera. A saturated forearm controller made the head and feet flail
while the torso stayed on target (see the `CEILING_SCALE` comment in
`src/character/anim/ragdoll.rs`). One joint's torque deficit moves every
body in the chain. The "errors accumulate" warning is why the ragdoll has
to be a feedback tracker, not a replay of torques.

## Where to read in the book
- p. 200 (PDF 213): forward vs. inverse, and why the whole body must be modelled.
- p. 201 (PDF 214): the plantarflexor coupling example and the six requirements.
- pp. 201–202 (PDF 214–215): uses and the internal-validity test.
