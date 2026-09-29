---
title: 3.1 Kinematic conventions
description: "Winter's absolute frame (X forward, Y up, Z lateral), counterclockwise-positive angles with derivatives signed the same way, and the 15-variables-per-segment description. Read before choosing sign conventions for angles, angular velocities or test assertions."
type: index
status: current
tags:
  - biomechanics
  - math
  - rig
  - correctness
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §3.1, pp. 46–48 (PDF pp. 59–61)"
---

# 3.1 Kinematic conventions

> **Source:** Winter (2009) §3.1, pp. 46–48 ·
> [open PDF at p. 46](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=59) ·
> Up: [Chapter 3 — Kinematics](../INDEX.md)

Kinematic bookkeeping needs a convention. Anatomical terms (proximal,
flexion, anterior) are relative: they place one limb against another but not
in space. To analyse movement against gravity and the ground, Winter fixes an
absolute spatial reference system with signed axes and signed angles. Imaging
devices record in it directly; body-mounted instruments give only relative
data and lose gravity and direction of travel. §3.1.1 defines the frame and
sign rules; §3.1.2 counts the variables needed to describe a segment in it.

## Key facts
- Axes: X = progression (anterior-posterior), Y = vertical, Z = medial-lateral ([3.1.1](./3.1.1-absolute-spatial-reference-system.md)).
- Sagittal angles start at 0° along +X and are positive counterclockwise; ω and α inherit that sign ([3.1.1](./3.1.1-absolute-spatial-reference-system.md)).
- Opposite signs of ω and α mean decelerating rotation (leg example ω = −2.34 rad/s, α = 14.29 rad/s²) ([3.1.1](./3.1.1-absolute-spatial-reference-system.md)).
- A segment needs 15 variables; 12 segments need 180; sagittal one-leg gait with HAT needs 36 ([3.1.2](./3.1.2-total-description-of-a-segment.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [3.1.1 Absolute spatial reference system](./3.1.1-absolute-spatial-reference-system.md) | The X/Y/Z frame, CCW-positive angle rule, derivative signs, worked sign-reading example | before fixing a sign convention or writing a signed angle/velocity assertion |
| [3.1.2 Total description of a body segment in space](./3.1.2-total-description-of-a-segment.md) | 15 variables per segment, 180 per body, 36 for sagittal HAT gait | when scoping what state a segment model, dump or test must cover |

## Relevance to migera
The chapter's most transferable discipline: every angle has a zero, a
positive sense and a viewing axis, and derivatives inherit the sign. migera's
knee-direction and facing bugs came from exactly the missing parts (unsigned
angles, a sign assumed for one facing). Translate Winter's "+X forward"
through each rig's own forward vector rather than hardcoding it.

## Where to read in the book
- p. 46 (PDF 59): axes and sign rules.
- p. 47 (PDF 60): Fig. 3.1 and the worked sign example.
- pp. 47–48 (PDF 60–61): the 15-variable description.
