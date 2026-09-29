---
title: 2.2 Frequency analysis
description: Fourier analysis of movement signals — series and coefficients (with errata), FFT, and what the spectrum decides: 25 Hz sampling and a 6 Hz cutoff for walking (99.7% of toe power in 7 harmonics), Butterworth filter, why 9 harmonics misfit a foot path. Read before representing or testing a gait curve in harmonics.
type: index
status: current
tags:
  - biomechanics
  - signal-processing
  - locomotion
  - math
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §2.2, pp. 26–41 (PDF pp. 39–54)"
---

# 2.2 Frequency analysis

> **Source:** Winter (2009) §2.2, pp. 26–41 ·
> [open PDF at p. 26](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=39) ·
> Up: [Chapter 2 — Signal processing](../INDEX.md)

Every movement signal has a spectrum, and that spectrum drives every
processing decision: sampling rate, record length, and filter cutoff. A
periodic signal such as a stride is a dc term plus harmonics of the stride
frequency; for walking, about seven harmonics (below ~6 Hz) carry
essentially all of the real signal and the rest is measurement noise. The
Fourier coefficients are averages over the window, so a signal whose
character changes within the cycle (a foot: flat in stance, fast in swing)
is poorly rebuilt from harmonics even with nine terms.

The section has no intro text of its own beyond 2.2.1. 2.2.2 gives the
math, 2.2.3 the fast algorithm, and 2.2.4 (the longest part, six
sub-subsections) the applications with the numbers: ADC, sampling theorem,
record length, filtering with coefficient formulas, Fourier reconstitution,
and white noise.

## Key facts
- A periodic signal = dc + Σ $V_n\sin(n\omega_0 t + \theta_n)$; $c_n=\sqrt{a_n^2+b_n^2}$, $\theta_n = \mathrm{atan2}(a_n, b_n)$ — [2.2.2](./2.2.2-discrete-fourier-harmonic-analysis.md).
- The printed discrete Eqs. 2.16–2.17 need the angle $2\pi n i/N$ and $i = 0\ldots N-1$ — [2.2.2](./2.2.2-discrete-fourier-harmonic-analysis.md).
- Harmonic coefficients are window averages; non-stationary signals rebuild badly — [2.2.2](./2.2.2-discrete-fourier-harmonic-analysis.md), [2.2.4](./2.2.4-spectrum-analysis-applications.md).
- FFT: N log₂N vs N², power-of-two records; the book's "49 ms" is really ~4.9 ms — [2.2.3](./2.2.3-fast-fourier-transform-fft.md).
- Sample at ≥ 2× the highest frequency or alias; 24–25 fps suffices for walking kinematics and kinetics — [2.2.4](./2.2.4-spectrum-analysis-applications.md).
- 99.7% of a walking toe marker's power is in harmonics 1–7 (< 6 Hz) → 6 Hz cutoff — [2.2.4](./2.2.4-spectrum-analysis-applications.md).
- Butterworth 2nd-order at $f_s/f_c = 10$: $a_0=0.067455$, $b_1=1.14298$, $b_2=-0.41280$ (verified from the formulas) — [2.2.4](./2.2.4-spectrum-analysis-applications.md).
- Quiet standing needs ≥ 1 min records; COP/COM power sits below 0.2 Hz — [2.2.4](./2.2.4-spectrum-analysis-applications.md).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [2.2.1 Introduction — time domain vs. frequency domain](./2.2.1-time-domain-vs-frequency-domain.md) | Spectrum → sampling rate, record length, cutoff | choosing how densely/long to sample a curve |
| [2.2.2 Discrete Fourier (harmonic) analysis](./2.2.2-discrete-fourier-harmonic-analysis.md) | Fourier series, coefficient formulas (Eqs. 2.10–2.17, corrected), averaging caveat | fitting or representing a gait curve as harmonics; extending `phase.rs` beyond one sinusoid |
| [2.2.3 Fast Fourier Transform (FFT)](./2.2.3-fast-fourier-transform-fft.md) | N log₂N cost, power-of-two records | analysing long signals (minutes of sway) |
| [2.2.4 Applications of spectrum analyses](./2.2.4-spectrum-analysis-applications.md) | Sampling theorem, record length, gait harmonic content, IIR filter coefficients, reconstitution failure, white noise | deciding harmonic count, sample rate or smoothing cutoff for gait work |

## Relevance to migera

The section answers "how many harmonics does a gait curve need?" (≈ 7 of
stride frequency for the worst-case toe; fewer for joint angles) and
"where does it break?" (foot contact — non-stationary, keep it
phase-segmented as `gait.rs`/`footlock.rs` do). It supplies an exact filter
oracle (Butterworth/critically damped coefficients) related to the
critically damped springs of `dho.rs`, and harmonic-power assertions that
make periodic-signal tests robust against small wobble.

## Where to read in the book
- pp. 27–30 (PDF 40–43): Fourier math — render pp. 28–29 (PDF 41–42) for equations.
- p. 37 (PDF 50): **Fig. 2.17**, the gait harmonic spectrum.
- p. 38 (PDF 51): filter coefficient equations and worked example.
- p. 39 (PDF 52): Fig. 2.18, 9-harmonic reconstruction of a toe trajectory.
