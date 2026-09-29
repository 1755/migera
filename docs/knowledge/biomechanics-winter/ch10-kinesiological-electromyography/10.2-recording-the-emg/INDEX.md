---
title: 10.2 Recording of the electromyogram
description: Summarizes EMG amplifier specification (gain, input impedance, bandwidth, CMRR) and surface-EMG contamination (hum, movement artifact, cross-talk) with SENIAM reporting rules. Read when specifying EMG hardware or judging EMG data quality; no direct animation relevance.
type: index
status: current
tags:
  - biomechanics
  - muscle
  - signal-processing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §10.2, pp. 257–269 (PDF pp. 270–282)"
aliases:
  - bioamplifier
---

# 10.2 Recording of the electromyogram

> **Source:** Winter (2009) §10.2, pp. 257–269 ·
> [open PDF at p. 257](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=270) ·
> Up: [Chapter 10 — Kinesiological electromyography](../INDEX.md)

A "clean" EMG is an undistorted, noise- and artifact-free sum of m.u.a.p.'s.
The section's intro defines the three enemies: **distortion** (non-linear
amplification, most often clipping when the amplifier is overdriven — large
5 mV signals must be amplified as faithfully as 100 µV ones), **noise**
(biological, e.g. ECG on thoracic muscles, or man-made: mains hum,
machinery, amplifier internals), and **artifacts** (false signals from
electrodes and cables, notably low-frequency baseline jumps from movement).
It then specifies the amplifier by four properties — gain and dynamic range
(10.2.1), input impedance (10.2.2), frequency response (10.2.3) and
common-mode rejection (10.2.4) — and closes with the electrode-side problems
of cross-talk (10.2.5) and standard reporting (10.2.6).

## Key facts
- Surface EMG peaks at ~5 mV p-p at MVC; amplifier noise should stay ≤ 20–50 µV; gains 100–10,000 ([10.2.1](./10.2.1-amplifier-gain.md)).
- Input impedance ≥ 1 MΩ with skin ≤ 1 kΩ keeps attenuation negligible ([10.2.2](./10.2.2-input-impedance.md)).
- Surface EMG band 10–1000 Hz, most power 20–200 Hz; hum at 50/60 Hz cannot be filtered out ([10.2.3](./10.2.3-frequency-response.md)).
- A differential amplifier cancels common-mode hum; CMRR should be ≥ 80 dB ([10.2.4](./10.2.4-common-mode-rejection.md)).
- Shared signal between channels is $R_{xy}^2$: 36% at 2 cm electrode separation, 2% at 7.5 cm ([10.2.5](./10.2.5-surface-emg-cross-talk.md)).
- SENIAM specifies what to report about electrodes and placement ([10.2.6](./10.2.6-surface-emg-reporting-and-electrode-placement.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [10.2.1 Amplifier gain](./10.2.1-amplifier-gain.md) | EMG amplitude ranges, noise limits, gain range | Choosing gain/dynamic range |
| [10.2.2 Input impedance](./10.2.2-input-impedance.md) | Electrode/amplifier voltage divider and recommended impedances | Signals look attenuated |
| [10.2.3 Frequency response](./10.2.3-frequency-response.md) | −3 dB bandwidth, dB gain, EMG spectrum and recommended bands | Setting EMG filter cutoffs |
| [10.2.4 Common-mode rejection](./10.2.4-common-mode-rejection.md) | Differential amplification, CMRR, Example 10.1 | Hum on the record |
| [10.2.5 Cross-talk in surface electromyograms](./10.2.5-surface-emg-cross-talk.md) | Overlapping pick-up; manual and cross-correlation tests with a separation table | Quantifying shared signal between two channels |
| [10.2.6 Recommendations for surface electromyogram reporting and electrode placement procedures](./10.2.6-surface-emg-reporting-and-electrode-placement.md) | SENIAM reporting checklist | Reading or designing an EMG study |

## Relevance to migera

Essentially none: this is instrumentation. The only transferable method is in
10.2.5 — squared normalized cross-correlation as the "fraction of signal in
common" between two channels, usable when comparing animation curves.

## Where to read in the book
- pp. 257–269 (PDF 270–282). Key figures: Fig. 10.5 amplifier equivalent
  circuit (p. 259), Fig. 10.7 EMG spectrum (p. 261), Fig. 10.8 cutoff
  effects (p. 262), Fig. 10.10 differential amplifier (p. 264), Fig. 10.14
  cross-correlation (p. 268).
