---
title: 2.1 Auto- and cross-correlation analyses
description: Auto- and cross-correlation as Pearson's r over time shifts — formulae (with an Eq. 2.9 erratum), properties, the mean-bias trap, circular correlation for gait, and applications (hum detection, top-down paraspinal lag in walking). Read before measuring similarity or lag between gait curves.
type: index
status: current
tags:
  - biomechanics
  - signal-processing
  - math
  - testing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §2.1, pp. 14–26 (PDF pp. 27–39)"
---

# 2.1 Auto- and cross-correlation analyses

> **Source:** Winter (2009) §2.1, pp. 14–26 ·
> [open PDF at p. 14](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=27) ·
> Up: [Chapter 2 — Signal processing](../INDEX.md)

Correlation functions answer two questions about time series: how similar
is a signal to itself at another time (autocorrelation — periodicity,
frequency content, hidden interference), and how similar is it to another
signal at some delay (cross-correlation — the lag between them). Both are
the Pearson coefficient computed at every shift τ, on zero-mean signals,
normalized to ±1.

The section's intro frames it with Pearson's r (strength and sign of a
relationship, dimensionless in [−1, +1]). The children then go from
definitions (2.1.1–2.1.2) through mathematical properties (2.1.3–2.1.4) and
implementation pitfalls (2.1.5–2.1.6) to worked applications (2.1.7–2.1.8).

## Key facts
- Correlation over time is Pearson's r at every shift; it is shape-only and blind to amplitude and offset — [2.1.1](./2.1.1-similarity-pearson-correlation.md).
- $R_{xx}(\tau)$ peaks at 0, is even, and for $E\sin\omega t$ equals $(E^2/2)\cos\omega\tau$ (phase lost) — [2.1.3](./2.1.3-autocorrelation-properties.md).
- White noise autocorrelates to an impulse; signal + uncorrelated noise gives $R_{ss}+R_{nn}$ — [2.1.3](./2.1.3-autocorrelation-properties.md).
- $R_{xy}(\tau)$ is not even and peaks at the delay between signals; the sign of the lag says which leads — [2.1.4](./2.1.4-cross-correlation-properties.md).
- Forgetting to remove means adds a constant $m_1 m_2$ term that inflates the peak — [2.1.5](./2.1.5-removing-mean-bias.md).
- For periodic data (gait) use circular correlation; the printed Eq. 2.9 denominator should be $\sqrt{\mathrm{var}_x \mathrm{var}_y}$ — [2.1.6](./2.1.6-digital-correlation-implementation.md).
- First zero crossing of $R_{xx}$ is a quarter period: $f \approx 1/(4\tau_0)$ — [2.1.7](./2.1.7-autocorrelation-applications.md).
- In walking, neck paraspinals lead lumbar by ~70 ms; head A/P acceleration is ~25% of the hip's (0.48 vs 1.91 m/s²) — [2.1.8](./2.1.8-cross-correlation-applications.md).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [2.1.1 Similarity to the Pearson correlation](./2.1.1-similarity-pearson-correlation.md) | Eq. 2.1 and why each normalization exists | before using r as a gait-shape similarity score |
| [2.1.2 Formulae for auto- and cross-correlation coefficients](./2.1.2-correlation-coefficient-formulae.md) | Continuous $R_{xx}(\tau)$, $R_{xy}(\tau)$ (Eqs. 2.2–2.3) | implementing a lag or similarity measure |
| [2.1.3 Four properties of the autocorrelation function](./2.1.3-autocorrelation-properties.md) | Max at 0, even, sine → cosine, noise additivity (Eqs. 2.4–2.7) | finding a period or checking that a layered curve never repeats |
| [2.1.4 Three properties of the cross-correlation function](./2.1.4-cross-correlation-properties.md) | Asymmetry, peak = delay, coherence | measuring which limb leads and by how much |
| [2.1.5 Importance in removing the mean bias](./2.1.5-removing-mean-bias.md) | The $m_1 m_2$ bias term | hand-coding any correlation on joint angles |
| [2.1.6 Digital implementation](./2.1.6-digital-correlation-implementation.md) | Discrete Eqs. 2.8–2.9, shift range vs N, circular correlation | writing the Rust/Python helper for a gait-lag test |
| [2.1.7 Application of autocorrelations](./2.1.7-autocorrelation-applications.md) | Hum detection; quarter-period frequency estimate | estimating a dominant frequency without an FFT |
| [2.1.8 Applications of cross-correlations](./2.1.8-cross-correlation-applications.md) | Cross-talk, top-down paraspinal lag, coactivation | designing head/spine stabilization in a walk |

## Relevance to migera

This is the measurement kit for *phase relationships* in a procedural gait:
left/right anti-phase, knee-leads-thigh, head-bob at 2× stride, and "the
idle never visibly loops" can all become numeric assertions instead of
screenshot judgments. Implement once as a circular, mean-removed,
variance-normalized correlation over curves time-normalized to one stride
(see [2.3.2](../2.3-ensemble-averaging/2.3.2-time-base-normalization-to-100-percent.md)).
The one gait-behaviour number here — head fore-aft acceleration ≈ ¼ of the
pelvis's — is a target for head stabilization.

## Where to read in the book
- pp. 14–17 (PDF 27–30): Pearson and the correlation formulae.
- pp. 17–21 (PDF 30–34): properties and proofs; Figs. 2.3–2.5.
- pp. 21–23 (PDF 34–36): mean bias, discrete formulas (render p. 22 for the equations).
- pp. 23–26 (PDF 36–39): applications; Figs. 2.6–2.8.
