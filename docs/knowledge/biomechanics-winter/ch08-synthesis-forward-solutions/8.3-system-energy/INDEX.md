---
title: 8.3 System energy
description: Sorts system energy into segment kinetic/potential energy and spring elastic energy (both inside the Lagrangian) and friction/damper dissipation (kept out of L, entered as a generalized force via Rayleigh's function). Read when writing an energy test or a spring-damper joint for a simulated chain.
type: index
status: current
tags:
  - biomechanics
  - physics
  - springs
  - energetics
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §8.3, pp. 214–216 (PDF pp. 227–229)"
---

# 8.3 System energy

> **Source:** Winter (2009) §8.3, pp. 214–216 ·
> [open PDF at p. 214](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=227) ·
> Up: [Chapter 8 — Synthesis of human movement, forward solutions](../INDEX.md)

At any instant a model's energy has three parts: segment energy from
motion and height, elastic energy stored in deformed springs, and energy
dissipated by friction. The first two go into the Lagrangian $L$.
Dissipation is not a state energy. It is applied as an external force at
the right points. For viscous dampers there is a convenient shortcut:
Rayleigh's dissipation function, whose velocity derivative gives the
generalized force directly.

## Key ideas (intro)

- Segment KE and PE, plus spring PE, belong in $L = \mathrm{KE} - \mathrm{PE}$.
- Friction dissipation is treated as an external force (covered in §8.4).
- Dampers get an energy-like expression (Eq. 8.30) that can be used with
  the Lagrange equations.

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [8.3.1 Segment energy](./8.3.1-segment-energy.md) | KE with the LRS origin off the COM (Eq. 8.23), inertia tensor (8.24), PE = MgZ_c (8.25), the seg list (8.26) | when computing a body's energy or inertia in a test |
| [8.3.2 Spring potential energy and dissipative energy](./8.3.2-spring-potential-and-dissipative-energy.md) | Linear and torsional spring PE (8.27–8.29), Rayleigh dissipation DE and $Q_i = -\partial DE/\partial\dot q_i$ (8.30–8.31) | when designing spring/damper joint limits, contact or passive joint stiffness |

## Relevance to migera

This section gives the continuous-time energy vocabulary behind migera's
springs (`src/character/anim/math/spring.rs`, `dho.rs`) and the ragdoll's
PD terms. A PD controller is a torsional spring
$\tfrac12 k_t(\theta - \theta_s)^2$ plus a Rayleigh damper
$\tfrac12 c\,\dot\theta^2$, with the target angle as $\theta_s$. What the
book leaves out is the discretisation. A continuous damper always removes
energy, but an explicitly integrated one can inject it
([PD damping has an explicit-integration bound](../../../character-animation/ragdoll-and-physics/pd-damping-explicit-integration-bound.md)).

## Where to read in the book
- p. 214 (PDF 227): the three energy types.
- pp. 215–216 (PDF 228–229): Eqs. 8.23–8.31.
