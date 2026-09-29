---
title: 3.5 Calculation of other kinematic variables
description: "Formulas from smoothed marker data: absolute segment angles (atan of marker differences, CCW from horizontal), signed knee/ankle joint angles, central-difference velocities and three-point accelerations, linear and angular. Read before computing a signed angle, angular velocity or acceleration from sampled poses."
type: index
status: current
tags:
  - biomechanics
  - math
  - numerics
  - correctness
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §3.5, pp. 75–79 (PDF pp. 88–92)"
---

# 3.5 Calculation of other kinematic variables

> **Source:** Winter (2009) §3.5, pp. 75–79 ·
> [open PDF at p. 75](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=88) ·
> Up: [Chapter 3 — Kinematics](../INDEX.md)

Once coordinates are smoothed, everything else is simple arithmetic: segment
angles from pairs of markers, joint angles as differences of segment angles
with stated polarity, and velocities/accelerations as finite differences
centred on the sample so they line up in time with the positions. The
section has no introduction; its four subsections go angle → joint angle →
velocity → acceleration.

## Key facts
- Segment angle $\theta_{ij} = \arctan\frac{y_j-y_i}{x_j-x_i}$, counterclockwise from horizontal, distal→proximal; use atan2 in code ([3.5.1](./3.5.1-limb-segment-angles.md)).
- Knee $\theta_k = \theta_{21} - \theta_{43}$ (+ = flexion); ankle $\theta_a = \theta_{43} - \theta_{65} + 90°$ (+ = plantarflexion) ([3.5.2](./3.5.2-joint-angles.md)).
- Joint-angle conventions vary between researchers and must be stated ([3.5.2](./3.5.2-joint-angles.md)).
- Central difference $(x_{i+1}-x_{i-1})/2\Delta t$ puts velocity at sample i; the forward difference is half a step late ([3.5.3](./3.5.3-linear-and-angular-velocities.md)).
- Three-point $(x_{i+1}-2x_i+x_{i-1})/\Delta t^2$ is the preferred acceleration ([3.5.4](./3.5.4-linear-and-angular-accelerations.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [3.5.1 Limb-segment angles](./3.5.1-limb-segment-angles.md) | Absolute signed segment angle from two markers; Fig. 3.23 marker set | before computing a segment angle or plotting gait angle curves |
| [3.5.2 Joint angles](./3.5.2-joint-angles.md) | Knee, ankle, MT-PH formulas with polarity | before defining or asserting a joint angle's sign |
| [3.5.3 Velocities — linear and angular](./3.5.3-linear-and-angular-velocities.md) | Central-difference velocity and angular velocity; book typos | before estimating a velocity from samples |
| [3.5.4 Accelerations — linear and angular](./3.5.4-linear-and-angular-accelerations.md) | Five-point vs three-point accelerations | before computing an acceleration or writing a second-difference test |

## Relevance to migera
These are the formulas for direction-aware gait diagnostics and tests: signed
segment and joint angles in the rig's own sagittal plane (the check that
would have caught the backward knee), and centred finite differences for
velocity/acceleration estimates — already used for toe speed in
`src/character/anim/footlock.rs`. The derived accuracy and noise gains in
the velocity and acceleration notes tell you how tight a tolerance can be.

## Where to read in the book
- pp. 75–76 (PDF 88–89): segment angles, Fig. 3.23.
- p. 77 (PDF 90): joint angles, velocity Eq. 3.15.
- p. 78 (PDF 91): Fig. 3.24, Eqs. 3.16–3.18c.
