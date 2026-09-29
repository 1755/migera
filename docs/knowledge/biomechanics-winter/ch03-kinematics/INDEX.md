---
title: Chapter 3 — Kinematics
description: "Winter's kinematics chapter: signed conventions (CCW-positive angles), capture hardware, why differentiation amplifies noise (n, n²), the dual-pass zero-lag Butterworth with residual-analysis cutoff, and angle and finite-difference formulas. Read before smoothing, differentiating or sign-checking motion."
type: index
status: current
tags:
  - biomechanics
  - signal-processing
  - numerics
  - math
  - correctness
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Chapter 3, pp. 45–81 (PDF pp. 58–94)"
---

# Chapter 3 — Kinematics

> **Source:** Winter (2009) Chapter 3, pp. 45–81 ·
> [open PDF at p. 45](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=58) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** Kinematics describes movement without its causes.
Winter fixes one absolute frame (X progression, Y vertical, Z lateral) where
every angle has a zero and a counterclockwise-positive sense and every
derivative inherits that sign. Body-mounted sensors give only relative data;
imaging gives absolute coordinates plus ~1 mm of random noise. Because
differentiation multiplies harmonic n by n (velocity) and n² (acceleration),
that noise swamps accelerations unless removed first — with a 2nd-order
Butterworth low-pass run forward and backward (zero lag, 4th order,
cutoff-corrected by 0.802), its cutoff chosen by residual analysis (~6 Hz
for gait). Then segment angles, signed joint angles and centred finite
differences give everything else.

**Start here:** for animation work read §3.1.1 (conventions), §3.4.3 (noise
amplification), §3.4.4 (the filter), §3.5 (formulas). §3.2–3.3 are capture
hardware and can be skipped.

The chapter opens (§3.0) by sizing the problem — up to 50 variables per limb
per stride, 15 per segment — and noting that an analysis should use only the
variables its question needs.

## Key facts
- Angles: 0° along +X, counterclockwise positive; ω and α inherit the sign, and opposite signs mean deceleration ([3.1.1](./3.1-kinematic-conventions/3.1.1-absolute-spatial-reference-system.md)).
- A segment needs 15 kinematic variables; sagittal one-leg gait with HAT needs 36 ([3.1.2](./3.1-kinematic-conventions/3.1.2-total-description-of-a-segment.md)).
- Body-mounted sensors (goniometers, accelerometers) give relative/local-frame data only ([3.2](./3.2-direct-measurement-techniques/INDEX.md)).
- 99.7% of gait signal power is below the 7th stride harmonic (~6 Hz) ([3.4.2](./3.4-processing-raw-kinematic-data/3.4.2-signal-noise-kinematic-data.md)).
- A 20th-harmonic noise at 1% of the fundamental becomes 4x the fundamental in acceleration ([3.4.3](./3.4-processing-raw-kinematic-data/3.4.3-velocity-and-acceleration-problems.md)).
- Zero-lag filtering: 2nd-order Butterworth forward then backward, single-pass cutoff scaled by C = 0.802 ([3.4.4](./3.4-processing-raw-kinematic-data/3.4.4-smoothing-and-curve-fitting.md)).
- Residual analysis finds the noise rms (1.8 mm for film at 5 m) and the cutoff where distortion equals passed noise ([3.4.4](./3.4-processing-raw-kinematic-data/3.4.4-smoothing-and-curve-fitting.md)).
- Filtered finite differences match an accelerometer; polynomial fits and raw differences do not ([3.4.5](./3.4-processing-raw-kinematic-data/3.4.5-smoothing-techniques-compared.md)).
- Knee $\theta_{21}-\theta_{43}$ (+ flexion), ankle $\theta_{43}-\theta_{65}+90°$ (+ plantarflexion) ([3.5.2](./3.5-other-kinematic-variables/3.5.2-joint-angles.md)).
- Velocity $(x_{i+1}-x_{i-1})/2\Delta t$; acceleration $(x_{i+1}-2x_i+x_{i-1})/\Delta t^2$ ([3.5](./3.5-other-kinematic-variables/INDEX.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [3.0 Historical development and complexity of problem](./3.0-history-and-complexity.md) | Definition of kinematics; size of the description problem | for context only |
| [3.1 Kinematic conventions](./3.1-kinematic-conventions/INDEX.md) | Absolute frame, signed angle and derivative conventions, 15-variable segment description | before choosing any sign convention or writing a signed assertion |
| [3.2 Direct measurement techniques](./3.2-direct-measurement-techniques/INDEX.md) | Goniometers, finger glove, accelerometers; relative vs absolute data | when comparing relative/local-frame with world-frame quantities |
| [3.3 Imaging measurement techniques](./3.3-imaging-measurement-techniques/INDEX.md) | Optics, film, video, optoelectric capture and their noise levels | for mocap hardware background or the book's noise figures |
| [3.4 Processing of raw kinematic data](./3.4-processing-raw-kinematic-data/INDEX.md) | Noise amplification by differentiation; zero-lag Butterworth; residual analysis; validation | before smoothing or differentiating any sampled motion signal |
| [3.5 Calculation of other kinematic variables](./3.5-other-kinematic-variables/INDEX.md) | Segment and joint angle formulas; finite-difference velocity and acceleration | before computing a signed angle, velocity or acceleration from poses |
| [3.6 Problems based on kinematic data](./3.6-problems-kinematic-data.md) | Exercises on Appendix A with checkable answers (toe clearance 1.52 cm) | when building known-answer test fixtures |
| [3.7 References](./3.7-references.md) | Primary sources for the filtering and derivative methods | when a primary source behind §3.4 is needed |

## Relevance to migera
Three direct uses. **Sign conventions:** every angle with a zero, a positive
sense and a viewing axis is the discipline whose absence let a backward knee
and a backward walk pass 30+ tests; Winter's signed knee/ankle formulas,
computed in each rig's own sagittal plane (puppet_base faces +Z, the
synthetic rig −Z), are an independent check. **Differentiation:** any
velocity or acceleration derived from sampled poses (toe speed in
`src/character/anim/footlock.rs`, root/COM velocity, joint angular velocity
for PD work, smoothness tests) has noise gain $1/\Delta t$ and $1/\Delta t^2$;
filter with a zero-lag low-pass offline, use centred differences, and set
test tolerances from those gains. **Fixtures:** Appendix A plus §3.6 give
real inputs with tabulated answers. Runtime smoothing must be causal;
migera's damped springs are that filter, facing the same overshoot vs
rise-time trade-off as Butterworth vs critically damped.

## Where to read in the book
- pp. 45–48 (PDF 58–61): history, conventions (Fig. 3.1).
- pp. 48–64 (PDF 61–77): measurement hardware.
- pp. 64–75 (PDF 77–88): smoothing and differentiation — Figs. 3.16–3.22 and Eqs. 3.3–3.12 are the core.
- pp. 75–79 (PDF 88–92): angle and finite-difference formulas (Fig. 3.23, Eqs. 3.13–3.18c).
- pp. 79–81 (PDF 92–94): problems and references.

## See also
- [Chapter 2 — frequency analysis](../ch02-signal-processing/2.2-frequency-analysis/INDEX.md) — prerequisite: harmonic analysis, sampling theorem and the filter's first introduction (§2.2.4.4).
- [Chapter 7 — axes systems](../ch07-three-dimensional-kinematics-and-kinetics/7.1-axes-systems/INDEX.md) — deeper: the 3D generalization of these planar conventions.
- [Unsigned measurements cannot see direction](../../engineering-practice/testing/unsigned-measurements-cannot-see-direction.md) — same-trap: the migera bug signed conventions prevent.
