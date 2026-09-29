---
title: 2.3 Ensemble averaging of repetitive waveforms
description: Averaging cycles of a repetitive movement into a mean ± SD waveform over 0–100% stride — normative bands (29 adults, moments ÷ body mass), time normalization by interpolation, waveform CV (Eq. 2.20). Read before comparing a procedural walk with reference gait curves or scoring its variability.
type: index
status: current
tags:
  - biomechanics
  - signal-processing
  - locomotion
  - testing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §2.3, pp. 41–43 (PDF pp. 54–56)"
aliases:
  - ensemble average
---

# 2.3 Ensemble averaging of repetitive waveforms

> **Source:** Winter (2009) §2.3, pp. 41–43 ·
> [open PDF at p. 41](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=54) ·
> Up: [Chapter 2 — Signal processing](../INDEX.md)

Cyclic movements — walking, running, cycling, rowing, lifting — can be
averaged cycle by cycle into a mean waveform with a variability band. The
average is more reliable than any single cycle, and the spread itself is
information: how random a variable is from cycle to cycle.

The intro makes two points worth keeping. Within one subject, lower-limb
joint *angles* barely vary between strides while joint *moments* vary a
lot; computing the variances and covariances of these ensembles revealed a
total-limb motor synergy (Chapter 11). And ensemble bands are the basis of
clinical assessment: overlay a patient's curve on a healthy group's band.
The children show an example (2.3.1), the resampling step that makes
averaging possible (2.3.2), and a one-number variability score (2.3.3).

## Key facts
- A normative band is mean ± 1 SD over % stride; overlaying a single curve shows abnormality at a glance — [2.3.1](./2.3.1-ensemble-averaged-profiles.md).
- Dividing moments by body mass cut inter-subject variability ≈ 50% — [2.3.1](./2.3.1-ensemble-averaged-profiles.md).
- Stride resampling to N points is linear interpolation at index k·n/N (107 → 100 example) — [2.3.2](./2.3.2-time-base-normalization-to-100-percent.md).
- Waveform CV = √(mean σᵢ²) / mean|Xᵢ| (Eq. 2.20) — [2.3.3](./2.3.3-variability-about-mean-waveform.md).
- Joint angles vary little stride-to-stride, joint moments a lot — [2.3.3](./2.3.3-variability-about-mean-waveform.md).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [2.3.1 Examples of ensemble-averaged profiles](./2.3.1-ensemble-averaged-profiles.md) | Normative moment bands (29 adults), amputee overlay, mass normalization | building a reference band for a gait-plausibility test |
| [2.3.2 Normalization of time bases to 100%](./2.3.2-time-base-normalization-to-100-percent.md) | Linear-interpolation resampling of a stride onto % stride | comparing a procedural stride with reference data of different duration |
| [2.3.3 Measure of average variability about the mean waveform](./2.3.3-variability-about-mean-waveform.md) | Waveform CV (Eq. 2.20) | scoring stride-to-stride variability or a normalized curve error |

## Relevance to migera

This is the comparison pipeline for "does the procedural walk move like a
person": time-normalize both curves to % stride from the same event,
overlay the procedural curve on a reference band, and score the difference
with a CV-style normalized RMS. It also gives a principled target for how
much stride-to-stride variation the `phase.rs` layers should add: joint
angles in real walking are highly repeatable, so visible angle variation
between strides would be *less* human, not more.

## Where to read in the book
- pp. 41–42 (PDF 54–55): intro and Fig. 2.20 (moment bands with amputee overlay).
- pp. 42–43 (PDF 55–56): time normalization and Eq. 2.20.
