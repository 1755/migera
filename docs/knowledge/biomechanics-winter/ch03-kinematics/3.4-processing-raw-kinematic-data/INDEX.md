---
title: 3.4 Processing of raw kinematic data
description: "Why sampled positions must be smoothed before differentiating (harmonic n gains n in velocity, n² in acceleration), and how: a 2nd-order Butterworth run forward and backward (zero lag, C = 0.802), cutoff by residual analysis (~6 Hz for gait). Read before deriving velocity or acceleration from samples."
type: index
status: current
tags:
  - biomechanics
  - signal-processing
  - numerics
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §3.4, pp. 64–75 (PDF pp. 77–88)"
---

# 3.4 Processing of raw kinematic data

> **Source:** Winter (2009) §3.4, pp. 64–75 ·
> [open PDF at p. 64](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=77) ·
> Up: [Chapter 3 — Kinematics](../INDEX.md)

The chapter's core. Digitized coordinates are the true trajectory plus random
high-frequency noise (§3.4.1–3.4.2). Differentiation amplifies each harmonic
by its frequency, so noise invisible in position swamps acceleration
(§3.4.3). The fix is a low-pass Butterworth filter run forward and backward
in time — zero phase lag, 4th-order roll-off, cutoff corrected for the double
pass — with the cutoff chosen by residual analysis (§3.4.4). A validation
experiment shows filtered finite differences beat polynomial fits and raw
differences (§3.4.5). The section has no introduction of its own.

## Key facts
- Raw image coordinates are sampled and carry random additive noise; smooth before use ([3.4.1](./3.4.1-unprocessed-image-data.md)).
- 99.7% of gait marker signal power lies below the 7th harmonic (~6 Hz) ([3.4.2](./3.4.2-signal-noise-kinematic-data.md)).
- Velocity multiplies harmonic n by n, acceleration by n²: a 1% 20th harmonic becomes 4x the fundamental in acceleration ([3.4.3](./3.4.3-velocity-and-acceleration-problems.md)).
- The 2nd-order Butterworth coefficients (Eq. 3.8 and following) equal scipy's `butter(2, …)` ([3.4.4](./3.4.4-smoothing-and-curve-fitting.md)).
- Forward + reverse filtering gives zero phase lag and a 4th-order roll-off; set the single-pass cutoff with C = 0.802 (Butterworth) or 0.435 (critically damped) ([3.4.4](./3.4.4-smoothing-and-curve-fitting.md)).
- Residual analysis: noise rms = intercept of the high-frequency residual line (1.8 mm for film at 5 m); choose fc where the residual meets that level ([3.4.4](./3.4.4-smoothing-and-curve-fitting.md)).
- For accelerations the optimal cutoff is higher and depends on fs: $f_{c,2} = 0.06f_s - 0.000022f_s^2 + 5.95/\varepsilon$ ([3.4.4](./3.4.4-smoothing-and-curve-fitting.md)).
- Against an accelerometer, filtered finite differences win; a 9th-order polynomial and raw differences fail ([3.4.5](./3.4.5-smoothing-techniques-compared.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [3.4.1 Nature of unprocessed image data](./3.4.1-unprocessed-image-data.md) | Sampling and additive random noise in raw coordinates | before treating any sampled trajectory as exact |
| [3.4.2 Signal versus noise in kinematic data](./3.4.2-signal-noise-kinematic-data.md) | Gait signal is the first ~7 stride harmonics; above is noise | when choosing a bandwidth or cutoff for gait signals |
| [3.4.3 Problems of calculating velocities and accelerations](./3.4.3-velocity-and-acceleration-problems.md) | n and n² noise gain, worked example, derived finite-difference noise gains | before differentiating positions/angles or setting tolerances on derived values |
| [3.4.4 Smoothing and curve fitting of data](./3.4.4-smoothing-and-curve-fitting.md) | Curve fits, Butterworth coefficients, dual-pass zero lag, residual analysis, optimal cutoff | before implementing a smoothing filter or choosing a cutoff |
| [3.4.5 Comparison of some smoothing techniques](./3.4.5-smoothing-techniques-compared.md) | Pezzack validation of filtered differences vs polynomial vs raw | when choosing how to get accelerations, or designing an independent-oracle test |

## Relevance to migera
Any offline derivation of motion quantities from sampled poses (contact
annotation from toe speed, root/COM velocity, joint angular velocity for PD
targets, clip cleanup) should filter with a zero-lag low-pass before
differencing, and set tolerances on derived quantities from the $1/\Delta t$
and $1/\Delta t^2$ noise gains. Runtime smoothing is necessarily causal;
migera's damped springs play that role, with the same overshoot-vs-rise-time
trade-off as Butterworth vs critically damped.

## Where to read in the book
- pp. 64–66 (PDF 77–79): raw data and signal vs noise, Fig. 3.16.
- pp. 66–67 (PDF 79–80): differentiation amplifies noise, Fig. 3.17.
- pp. 68–73 (PDF 81–86): filtering, Figs. 3.18–3.21, Eqs. 3.8–3.10.
- pp. 74–75 (PDF 87–88): Pezzack comparison, Fig. 3.22.
