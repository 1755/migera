---
title: Chapter 2 — Signal processing
description: Winter's toolkit for movement signals — auto/cross-correlation (lag between signals), Fourier analysis (walking needs ~7 stride harmonics, <6 Hz), sampling and Butterworth filtering, ensemble averaging over 0–100% stride with a waveform CV. Read before comparing, testing or compactly representing gait curves.
type: index
status: current
tags:
  - biomechanics
  - signal-processing
  - locomotion
  - math
  - testing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Ch. 2, pp. 14–44 (PDF pp. 27–57)"
---

# Chapter 2 — Signal processing

> **Source:** Winter (2009) Ch. 2, pp. 14–44 ·
> [open PDF at p. 14](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=27) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** Every biomechanical variable is a time series,
and three tools cover most of what you do with one. *Correlation* (§2.1)
measures similarity of a signal with itself or another signal at every time
shift: the autocorrelation reveals periods and hidden interference, the
cross-correlation's peak gives the delay between two signals (e.g. neck
muscles lead lumbar by ~70 ms in walking). *Frequency analysis* (§2.2)
decomposes a periodic signal into stride harmonics: for walking about seven
harmonics (< 6 Hz) carry 99.7% of the power of even the fastest-moving
marker, which fixes a 25 Hz sampling rate and a 6 Hz low-pass as adequate,
but harmonic reconstruction still misfits a foot path because its content
differs between stance and swing. *Ensemble averaging* (§2.3) resamples each
cycle to 0–100% stride, averages into a mean ± SD band, and scores
variability with a waveform coefficient of variation.

**Start here:** for gait-comparison tests read
[2.3.2](./2.3-ensemble-averaging/2.3.2-time-base-normalization-to-100-percent.md) →
[2.1.6](./2.1-auto-and-cross-correlation/2.1.6-digital-correlation-implementation.md) →
[2.2.4](./2.2-frequency-analysis/2.2.4-spectrum-analysis-applications.md).

## Key facts
- Correlation over time = Pearson r at every shift; zero-mean signals, bounded ±1 — [2.1](./2.1-auto-and-cross-correlation/INDEX.md).
- The cross-correlation peak lag is the delay; its sign says who leads; forgetting mean removal inflates it — [2.1.4](./2.1-auto-and-cross-correlation/2.1.4-cross-correlation-properties.md), [2.1.5](./2.1-auto-and-cross-correlation/2.1.5-removing-mean-bias.md).
- Use circular correlation on periodic data; the printed Eq. 2.9 denominator is wrong — [2.1.6](./2.1-auto-and-cross-correlation/2.1.6-digital-correlation-implementation.md).
- Walking head A/P acceleration ≈ ¼ of the hip's (0.48 vs 1.91 m/s²), via a top-down paraspinal strategy — [2.1.8](./2.1-auto-and-cross-correlation/2.1.8-cross-correlation-applications.md).
- Fourier coefficients are window averages; printed discrete Eqs. 2.16–2.17 need angle 2πni/N — [2.2.2](./2.2-frequency-analysis/2.2.2-discrete-fourier-harmonic-analysis.md).
- 99.7% of gait toe-marker power in harmonics 1–7 (< 6 Hz); 24–25 fps suffices for walking — [2.2.4](./2.2-frequency-analysis/2.2.4-spectrum-analysis-applications.md).
- Nine harmonics still misrepresent a foot trajectory (non-stationary stance/swing) — [2.2.4](./2.2-frequency-analysis/2.2.4-spectrum-analysis-applications.md).
- 2nd-order Butterworth at fs/fc = 10: a₀ = 0.067455, b₁ = 1.14298, b₂ = −0.41280 — [2.2.4](./2.2-frequency-analysis/2.2.4-spectrum-analysis-applications.md).
- Resample strides to % stride by linear interpolation; score spread with CV = √(mean σᵢ²)/mean|Xᵢ| — [2.3](./2.3-ensemble-averaging/INDEX.md).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [2.0 Introduction](./2.0-introduction.md) | All biomechanical variables are signals; chapter roadmap | orienting in the chapter |
| [2.1 Auto- and cross-correlation analyses](./2.1-auto-and-cross-correlation/INDEX.md) | Correlation formulae, properties, mean bias, circular correlation, gait lag applications | measuring lag or similarity between limb curves |
| [2.2 Frequency analysis](./2.2-frequency-analysis/INDEX.md) | Fourier series, FFT, sampling/record length/filtering, gait harmonic content | choosing harmonic count, sample rate or smoothing for gait |
| [2.3 Ensemble averaging of repetitive waveforms](./2.3-ensemble-averaging/INDEX.md) | Mean ± SD bands over % stride, time normalization, waveform CV | comparing a procedural walk with reference data |
| [2.4 References](./2.4-references.md) | Cited works, with the gait-relevant ones picked out | chasing a number to its source |

## Relevance to migera

Chapter 2 is the measurement toolkit for `src/character/anim`'s periodic
motion:

- **Representation.** `phase.rs`'s `PhaseOscillator` is a one-term Fourier
  series; `gait.rs` rejects a single sinusoid for legs. Winter's spectrum
  (≈ 7 harmonics for the worst-case toe marker) says a short per-joint
  Fourier series is sufficient for joint angles, while foot contact should
  stay phase-segmented (as `gait.rs` and `footlock.rs` do) because harmonic
  fits fail on stance/swing non-stationarity.
- **Tests on periodic signals.** Time-normalize to % stride, then assert
  with circular, mean-removed cross-correlation (limb phase relationships,
  knee-leads-thigh, left/right anti-phase), harmonic power fractions (head
  bob at 2× stride), and CV-style normalized RMS against a reference band.
  These replace screenshot judgments and zero-crossing counts with numbers
  that can fail.
- **Reference data.** Appendix A's single walking stride, resampled per
  §2.3.2, is the immediate comparison target.
- **Filtering and springs.** The critically damped 2nd-order filter of
  Eq. 2.18 is the discrete twin of the critically damped springs in
  `dho.rs`; its coefficients are a ready oracle.

## Where to read in the book
- pp. 14–26 (PDF 27–39): correlation; Figs. 2.3–2.8.
- pp. 26–41 (PDF 39–54): frequency analysis; **Fig. 2.17** (p. 37) gait harmonics; filter coefficients p. 38.
- pp. 41–43 (PDF 54–56): ensemble averaging; Fig. 2.20; Eq. 2.20.
- pp. 43–44 (PDF 56–57): references.

## See also
- [3.4 Processing of raw kinematic data](../ch03-kinematics/3.4-processing-raw-kinematic-data/INDEX.md) — the book applies §2.2's spectrum and filter to marker data, including multi-pass phase cancellation.
- [Appendix A — walking trial data](../appendices/a-walking-trial-kinematic-kinetic-energy-data.md) — one measured stride to compare a procedural walk against.
- [Character animation — IK and locomotion](../../character-animation/ik-and-locomotion/INDEX.md) — the gait code these tools would test.
- [Character animation — animation core](../../character-animation/animation-core/INDEX.md) — springs and the phase layer.
