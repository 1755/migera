---
title: 7.2 Marker and anatomical axes systems
description: "How markers become segment anatomical frames: ≥3 non-collinear markers per segment, a standing calibration fixes a constant [M to A], each frame's markers give [G to M], and [G to A] = [M to A][G to M] yields θ₁–θ₃ plus the COM. Read when building a bone frame from joint positions or needing a worked fixture."
type: index
status: current
tags:
  - biomechanics
  - math
  - rig
  - testing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §7.2, pp. 180–187 (PDF pp. 193–200)"
---

# 7.2 Marker and anatomical axes systems

> **Source:** Winter (2009) §7.2, pp. 180–187 ·
> [open PDF at p. 180](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=193) ·
> Up: [Chapter 7 — Three-dimensional kinematics and kinetics](../INDEX.md)

The pipeline from raw marker coordinates to a segment's anatomical frame and
COM trajectory, then a fully worked numeric example for the shank.

**The method (pp. 180–183).** Each segment needs **at least three
non-collinear tracking markers, none shared with a neighbour**. They define a
tracking plane; one marker is the marker-frame origin, the line to another is
+z_m, y_m is normal to the plane, x_m completes a right-handed set (Fig. 7.2).
In a one-second standing **anatomical calibration**, temporary markers on
landmarks (for the shank: medial malleolus, medial tibial condyle) locate the
joint centres (ankle = mid-malleoli, knee = mid fibular head/medial condyle),
the long y axis (ankle → knee), x normal to the plane of y and the
malleolar line, and z completing a dextral set; the COM sits a known fraction
along y. The calibration markers are then removed: the anatomical frame is
assumed rigidly fixed to the tracking markers. Clinical labs whose patients
cannot hold the anatomical pose instead calibrate in a comfortable stance plus
anthropometric offsets (ankle/knee widths; Davis et al. 1991, Õunpuu et al.
1996).

**The matrices.** [G to M] (global → marker) varies per frame; [M to A]
(marker → anatomical) is constant from calibration; [G to A] = [M to A]·[G to M]
is solved for θ₁, θ₂, θ₃ each frame via Eq. 7.5. Position is a separate
translation: R_c = R_m + c, where R_m is the marker origin in global and c the
constant marker-origin→COM vector rotated into global.

## Key facts

- ≥3 non-collinear markers per segment, none shared between segments — this page's intro (p. 180).
- Calibration yields a *constant* marker→anatomical matrix; tracking needs only the markers afterwards — [7.2.1](./7.2.1-kinematic-data-set-example.md).
- Joint centres from the calibration land on the anatomical long axis with ≈0 off-axis components — a built-in correctness check — [7.2.1](./7.2.1-kinematic-data-set-example.md).
- Frame-6 swing leg: θ = (−8.92°, −2.71°, −53.27°); sagittal θ₃ dominates — [7.2.1](./7.2.1-kinematic-data-set-example.md).
- The printed example has four sign/digit typos; results are nonetheless right — [7.2.1](./7.2.1-kinematic-data-set-example.md).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [7.2.1 Example of a kinematic data set](./7.2.1-kinematic-data-set-example.md) | Tables 7.1–7.2, every intermediate matrix, angles and COM for frames 5–7, typos, scipy re-check | When writing a frame-construction or Euler-decomposition test and needing known answers |

## Relevance to migera

migera does not track markers, but the construction is how to measure a
*signed* anatomical frame from the real rig's joint positions (BRP or FK) and
how to validate a bone frame structurally (joints on the long axis). The
constant "[M to A]" is the same idea as a bind-pose offset: a fixed rotation
between the frame you can observe and the frame you want to reason in —
cf. [conjugate pose deltas by the bind rotation](../../../character-animation/rig-and-retargeting/conjugate-pose-deltas-by-the-bind-rotation.md).

## Where to read in the book

- pp. 180–182 (PDF 193–195): method, Fig. 7.2 (frames and the matrix flow chart).
- pp. 183–187 (PDF 196–200): the worked example (Tables 7.1, 7.2).
