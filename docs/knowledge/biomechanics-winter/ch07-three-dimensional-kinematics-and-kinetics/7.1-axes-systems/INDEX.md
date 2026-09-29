---
title: 7.1 Axes systems
description: "The frames of 3D motion analysis — lab GRS (X fwd, Y up, Z lateral), per-segment marker and anatomical frames — the x–y′–z″ Cardan matrix (Eq. 7.5), the 12 sequences, and dot/cross products. Read when converting between world and bone frames or decomposing a rotation into angles."
type: index
status: current
tags:
  - biomechanics
  - math
  - rig
  - retargeting
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §7.1, pp. 176–180 (PDF pp. 189–193)"
---

# 7.1 Axes systems

> **Source:** Winter (2009) §7.1, pp. 176–180 ·
> [open PDF at p. 176](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=189) ·
> Up: [Chapter 7 — Three-dimensional kinematics and kinetics](../INDEX.md)

3D analysis juggles three kinds of frame: the lab-fixed global reference
system (GRS), a *marker* frame per segment built from its surface markers, and
an *anatomical* frame per segment built from bony landmarks with its origin at
the COM and y along the long axis (the segment's principal axes). The section
defines the GRS, shows how a frame is reached by three ordered rotations about
moving axes (the Cardan x–y′–z″ sequence), lists the other 11 sequences, and
reviews the dot and cross products used to construct axes.

The intro text (p. 176) only names the frames: marker frame = a local
reference system (LRS); anatomical frame = a second LRS, principal axes,
defined from skeletal landmarks.

## Key facts

- Lab frame: X forward, Y vertical, Z mediolateral (to the subject's right) — [7.1.1](./7.1.1-global-reference-system.md).
- Global→anatomical is [Φ₃][Φ₂][Φ₁], Eq. 7.5; it is the transpose of scipy `from_euler('XYZ')` / glam `EulerRot::XYZ` — [7.1.2](./7.1.2-local-reference-systems-and-rotation.md).
- The decomposition is singular at θ₂ = ±90° (middle, long-axis angle) — [7.1.2](./7.1.2-local-reference-systems-and-rotation.md).
- Book → Bevy rig frame is a 90° yaw; the book's sequence becomes Z–Y′–X″ with the flexion angle negated — [7.1.2](./7.1.2-local-reference-systems-and-rotation.md).
- 12 valid sequences; the same orientation gives very different angles per sequence — [7.1.3](./7.1.3-other-rotation-sequences.md).
- In the book's sequence θ₁ = frontal, θ₂ = axial, θ₃ = sagittal (flexion last) — [7.1.3](./7.1.3-other-rotation-sequences.md).
- Axes are built with cross products; the operand order fixes the sign — [7.1.4](./7.1.4-dot-cross-products.md).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [7.1.1 Global reference system](./7.1.1-global-reference-system.md) | The lab axes and their alignment with the force plate | Before using any Chapter 7 number in Bevy's frame |
| [7.1.2 Local reference systems and rotation of axes](./7.1.2-local-reference-systems-and-rotation.md) | Eqs 7.1–7.5, passive vs active, glam/scipy equivalents, gimbal lock, book→rig mapping | Before decomposing or composing bone rotations as angles |
| [7.1.3 Other possible rotation sequences](./7.1.3-other-rotation-sequences.md) | The 12 sequences; numeric proof that angles depend on sequence | When choosing a sequence for limits/sliders or reading published joint angles |
| [7.1.4 Dot and cross products](./7.1.4-dot-cross-products.md) | Eq. 7.6; 3D power as a dot product | When building a frame from three points |

## Relevance to migera

migera animates with one quaternion per bone and so never needs Euler angles
to *run*; this section matters wherever a rotation is turned into numbers a
human or a limit reads (studio sliders, joint limits, logs, comparisons with
gait-lab data). The two traps it exposes are the ones migera's rig lessons
record: *which frame* a rotation is in (passive/active, world vs bone —
cf. [a pose delta names a world axis](../../../character-animation/rig-and-retargeting/a-pose-delta-names-a-world-axis.md))
and *which sign/axis* a convention uses (book lab frame vs the +Z-facing rig).

## Where to read in the book

- pp. 176–177 (PDF 189–190): frame definitions, GRS.
- pp. 178–179 (PDF 191–192): Fig. 7.1 and Eqs 7.1–7.5 — render these pages, the extracted text mangles the primes.
- pp. 179–180 (PDF 192–193): sequence table, Eq. 7.6.
