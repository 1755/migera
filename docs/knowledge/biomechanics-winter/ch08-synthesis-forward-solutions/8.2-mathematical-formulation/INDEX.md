---
title: 8.2 Mathematical formulation
description: A Lagrangian, list-driven way to generate equations of motion for linked segments - generalized coordinates, DOF = 6S+3P−C, L = KE−PE, generalized forces and Lagrange multipliers, point/frame lists, 2D and 3D (zxz Euler) kinematics. Printed 3D matrices have typos (corrected here). Read when deriving EOMs by hand.
type: index
status: current
tags:
  - biomechanics
  - math
  - physics
  - numerics
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §8.2, pp. 203–214 (PDF pp. 216–227)"
aliases:
  - Lagrangian dynamics
  - equations of motion generation
---

# 8.2 Mathematical formulation

> **Source:** Winter (2009) §8.2, pp. 203–214 ·
> [open PDF at p. 203](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=216) ·
> Up: [Chapter 8 — Synthesis of human movement, forward solutions](../INDEX.md)

This section describes a systematic way to *generate* the equations of
motion of any linked-segment model with Lagrange's method. The model is
written as linked lists: points, local reference systems, segments, springs,
forces and torques. Deriving the equations then becomes a mechanical
procedure that suits a symbolic program, or hand derivation by "filling in
tables".

## Key ideas (intro, pp. 203–204)

- Models are built from mass elements, springs, dampers and actuators.
  Motion is either prescribed as trajectories or produced by motor forces
  under the laws of physics. Nonlinearity used to block general solutions,
  but computers now handle it.
- **Three ways to formulate the equations:**
  1. **Newton's laws on each segment.** This is direct, and the joint
     reactions come out as a by-product, but it is cumbersome for a general
     program. Adding graph theory makes it systematic: Waterloo's VECENT
     (3D particles), PLANET (planar) and ADVNET (3D).
  2. **Lagrangian dynamics** (Wells 1967). Virtual displacements, energy
     and work are expressed in generalized coordinates, and the result is
     second-order ODEs. It is the same procedure whatever the segment
     count, coordinate choice or number of (moving) constraints.
  3. **Virtual work with D'Alembert's principle** (DYMAC, Paul 1978).
- Commercial automation existed already: ADAMS (Mechanical Dynamics Inc.,
  Chace 1984), the package later used for the 1997 gait model.
- Winter picks the Lagrangian method for its simplicity and its freedom in
  choosing coordinates. Each list has a name, an index and a stack of
  parameters. The first element of each list is an integer that links it
  to another list.

## How the children fit together

[8.2.1](./8.2.1-lagrange-equations-of-motion.md) introduces the method.
[8.2.2](./8.2.2-generalized-coordinates-and-dof.md) chooses the coordinates
and counts DOF. [8.2.3](./8.2.3-lagrangian-function.md) forms $L$, and
[8.2.4](./8.2.4-generalized-forces.md) projects nonconservative and
constraint forces onto the coordinates. [8.2.5](./8.2.5-lagrange-equations.md)
assembles the equations. [8.2.6](./8.2.6-points-reference-systems.md) and
[8.2.7](./8.2.7-displacement-and-velocity-vectors.md) supply the kinematics
(positions and velocities of every point) that $L$ needs. The sliding-block
example runs through 8.2.6–8.2.7. Energy terms continue in
[8.3](../8.3-system-energy/INDEX.md).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [8.2.1 Lagrange's equations of motion](./8.2.1-lagrange-equations-of-motion.md) | Pointer: Lagrange's equations "of the second type"; see Greenwood 1977 or Wells 1967 for the derivation | when you want the textbook derivation |
| [8.2.2 The generalized coordinates and degrees of freedom](./8.2.2-generalized-coordinates-and-dof.md) | Generalized coordinates, DOF = 6S+3P−C, constraints and Lagrange multipliers, holonomic systems, transformation equations | when counting a ragdoll's DOF or choosing reduced vs. maximal coordinates |
| [8.2.3 The Lagrangian function L](./8.2.3-lagrangian-function.md) | L = KE − PE; KE relative to an inertial frame; gravity and spring PE | when writing an energy check |
| [8.2.4 Generalized forces [Q]](./8.2.4-generalized-forces.md) | Projecting forces (Eq. 8.6) and constraint multipliers (Eq. 8.7) onto $q_i$ | when converting joint torques or contact forces to generalized forces |
| [8.2.5 Lagrange's equations](./8.2.5-lagrange-equations.md) | $\frac{d}{dt}\partial L/\partial\dot q_i - \partial L/\partial q_i = Q_i$ plus constraints: n + m unknowns | when assembling a model's EOMs |
| [8.2.6 Points and reference systems](./8.2.6-points-reference-systems.md) | pt/LRS/seg list encoding; sliding-block setup (Fig. 8.2) | when encoding a model for automatic EOM generation |
| [8.2.7 Displacement and velocity vectors](./8.2.7-displacement-and-velocity-vectors.md) | $R_a = R_i + [\phi]r_{ia}$, $V_a = V_i + [\phi]v_{ia} + [\phi][\tilde\omega]r_{ia}$; zxz DCM, Euler-rate map, generalized torques (corrected); sliding-block EOMs | when you need a correct zxz Euler DCM or Euler-rate → body ω map |

## Relevance to migera

migera never derives equations of motion symbolically, because avian does
the dynamics. This section is still useful in three ways:
- **Coordinates.** The reduced (Lagrangian) vs. maximal
  (bodies + constraints) choice explains avian's model. See
  [8.2.2](./8.2.2-generalized-coordinates-and-dof.md) and
  [8.5](../8.5-designation-joints.md).
- **Verified 3D formulas.** [8.2.7](./8.2.7-displacement-and-velocity-vectors.md)
  gives a checked zxz DCM and the Euler-rate → body-angular-velocity map.
- **Oracles.** The method yields analytic test oracles for small chains
  ([8.6](../8.6-illustrative-example.md)).

## Where to read in the book
- pp. 203–204 (PDF 216–217): formulation methods and software.
- pp. 205–208 (PDF 218–221): coordinates, DOF, $L$, $Q$, Lagrange's equations (Eqs. 8.1–8.9).
- pp. 208–214 (PDF 221–227): lists, Figs. 8.2–8.4, 2D/3D kinematics (Eqs. 8.10–8.22).
