---
title: 3.2 Direct measurement techniques
description: "Body-mounted kinematic sensors: goniometers (relative joint angle, v = k1θ), fiber-optic finger gloves, and accelerometers (normal component only, in the limb's rotating frame). Read when weighing relative vs absolute kinematic data; hardware detail is low relevance for migera."
type: index
status: current
tags:
  - biomechanics
  - signal-processing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §3.2, pp. 48–53 (PDF pp. 61–66)"
---

# 3.2 Direct measurement techniques

> **Source:** Winter (2009) §3.2, pp. 48–53 ·
> [open PDF at p. 48](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=61) ·
> Up: [Chapter 3 — Kinematics](../INDEX.md)

Instruments attached to the body measure kinematics directly and give an
immediate electrical signal, but what they measure is *relative*: a joint
angle between two segments, or acceleration along an axis fixed to a moving
limb. They lose gravity and the direction of travel, which imaging (§3.3)
keeps. The section has no introductory text; its three subsections cover
goniometers, a special finger-angle glove, and accelerometers.

## Key facts
- A goniometer's output is linear in the relative joint angle, $v = k_1\theta$ ([3.2.1](./3.2.1-goniometers.md)).
- Relative angles "severely limit" assessment value; alignment drifts over soft tissue ([3.2.1](./3.2.1-goniometers.md)).
- Finger flexion can be sensed by light loss in bent, etched optical fibers ([3.2.2](./3.2.2-special-joint-angle-systems.md)).
- An accelerometer reads only the component normal to its face, in the limb's frame; the same world acceleration reads differently as the limb rotates ([3.2.3](./3.2.3-accelerometers.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [3.2.1 Goniometers](./3.2.1-goniometers.md) | Potentiometer joint-angle sensor, pros/cons of relative data | when contrasting joint (relative) with segment (absolute) angles |
| [3.2.2 Special joint angle measuring systems](./3.2.2-special-joint-angle-systems.md) | Fiber-optic glove for finger flexion | only for finger-capture hardware background |
| [3.2.3 Accelerometers](./3.2.3-accelerometers.md) | F = ma transducers, normal component only, rotation artefact, bridge circuit | when a quantity measured in a rotating local frame is compared with a world one |

## Relevance to migera
Low as hardware. The lesson worth keeping is frame awareness: relative or
body-local measurements look fine in isolation and mislead when compared with
world quantities. The accelerometer also reappears in §3.4.5 as the
independent ground truth for finite-difference accelerations.

## Where to read in the book
- pp. 48–50 (PDF 61–63): goniometers, Figs. 3.2–3.3.
- pp. 50–51 (PDF 63–64): finger glove, Fig. 3.4.
- pp. 50–53 (PDF 63–66): accelerometers, Figs. 3.5–3.7.
