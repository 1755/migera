---
title: 4.2 Direct experimental measures
description: In-vivo alternatives to Table 4.1 — balance-board whole-body COM and distal-segment mass, quick-release moment of inertia, and instantaneous joint axes from marker velocities (with the 0.5 rad/s reliability floor). Read when measuring a COM/inertia/pivot in a test rather than assuming it, or placing rig joint centers.
type: index
status: current
tags:
  - biomechanics
  - anthropometry
  - testing
  - rig
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §4.2, pp. 96–100 (PDF pp. 109–113)"
---

# 4.2 Direct experimental measures

> **Source:** Winter (2009) §4.2, pp. 96–100 ·
> [open PDF at p. 96](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=109) ·
> Up: [Chapter 4 — Anthropometry](../INDEX.md)

Directly measured values are preferable to tables for exact work, but the
section intro warns that the techniques are limited and sometimes barely
better than the tables. Two static moment-balance tricks on a balance
board find the whole-body COM (4.2.1) and a distal segment's mass (4.2.2).
A dynamic quick-release test finds a limb's moment of inertia about its
joint (4.2.3). A kinematic method finds the true joint axis that skin
markers only approximate (4.2.4).

## Key facts
- Whole-body COM from a board on a pivot and scale: x₂ = (S·x₃ − w₁x₁)/w₂ ([4.2.1](./4.2.1-whole-body-center-of-mass-location.md)).
- A limb's weight from the scale change when it is raised vertical; the error comes from the table-derived COM ([4.2.2](./4.2.2-distal-segment-mass.md)).
- I about a joint = F·y₁·y₂/a from a quick release ([4.2.3](./4.2.3-distal-segment-moment-of-inertia.md)).
- The ankle axis is a few cm distal to the lateral malleolus, and the trochanter marker is lateral to the hip center ([4.2.4](./4.2.4-joint-axes-rotation.md)).
- Instantaneous-axis estimates fail when |ω| < 0.5 rad/s ([4.2.4](./4.2.4-joint-axes-rotation.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [4.2.1 Location of the anatomical center of mass of the body](./4.2.1-whole-body-center-of-mass-location.md) | Balance board, Eq. 4.13 | locating a COM by static moment balance |
| [4.2.2 Calculation of the mass of a distal segment](./4.2.2-distal-segment-mass.md) | Raise-the-limb weighing, Eq. 4.14 | estimating one segment's mass by moment balance |
| [4.2.3 Moment of inertia of a distal segment](./4.2.3-distal-segment-moment-of-inertia.md) | Quick release, I = Fy₁y₂/a (Eq. 4.15) | measuring a simulated body's real inertia in a test |
| [4.2.4 Joint axes of rotation](./4.2.4-joint-axes-rotation.md) | Marker-vs-true-axis errors; V = ω×R (Eq. 4.16) | placing rig joints or fitting pivots from motion |

## Relevance to migera

Mostly lab technique. Two ideas carry over. The quick-release experiment
is a template for a unit test that measures a ragdoll body's *effective*
inertia after avian has computed it. The joint-axis section warns that
rig joints placed at surface landmarks are centimetres off the
anatomical pivots, which matters for foot roll and hip IK.

## Where to read in the book
- pp. 96–97 (PDF 109–110): balance board, Fig. 4.7.
- pp. 97–98 (PDF 110–111): quick release, Fig. 4.8.
- pp. 98–100 (PDF 111–113): joint axes, Fig. 4.9.
