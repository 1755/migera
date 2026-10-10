---
title: 10.3 Processing of the electromyogram
description: Summarizes the five standard EMG processing schemes — rectification, linear envelope (critically damped low-pass matched to twitch time, fc 1.5-4 Hz) and three integrators. Read before smoothing a commanded activation/strength signal or building an effort accumulator for fatigue.
type: index
status: current
tags:
  - biomechanics
  - muscle
  - signal-processing
  - springs
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §10.3, pp. 269–273 (PDF pp. 282–286)"
aliases:
  - EMG processing
---

# 10.3 Processing of the electromyogram

> **Source:** Winter (2009) §10.3, pp. 269–273 ·
> [open PDF at p. 269](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=282) ·
> Up: [Chapter 10 — Kinesiological electromyography](../INDEX.md)

Raw EMG is too high-frequency to record on 0–60 Hz pen recorders or to
correlate with force, so it is converted into a slower form. The intro lists
five common online schemes: (1) half- or full-wave rectification, (2) the
linear envelope (rectifier + low-pass), (3) integration over the whole
contraction, (4) integration for a fixed time then reset, (5) integration to
a preset level then reset — all shown on one record in Fig. 10.15.
Rectification (10.3.1) is the common first step; the linear envelope (10.3.2)
is the one with a biomechanical justification; the integrators (10.3.3)
measure accumulated activity.

## Key facts
- Raw EMG has zero mean; rectify before any averaging ([10.3.1](./10.3.1-full-wave-rectification.md)).
- The linear envelope is not "integrated EMG"; it is a low-passed rectified signal in mV ([10.3.2](./10.3.2-linear-envelope.md)).
- The right envelope filter is critically damped 2nd order with impulse response = the twitch ([10.3.2](./10.3.2-linear-envelope.md)).
- $f_c = 1/(2\pi T)$: twitch time 40–106 ms ↔ 4–1.5 Hz ([10.3.2](./10.3.2-linear-envelope.md)).
- In migera's spring vocabulary that filter is `SpringParams::critical(T·ln 2)`, halflife ≈ 0.028–0.073 s ([10.3.2](./10.3.2-linear-envelope.md)).
- A true integral (mV·s) divided by duration is the only true average EMG ([10.3.3](./10.3.3-true-mathematical-integrators.md)).
- Time-reset (40–200 ms) and level-reset integrators follow the trend like an envelope ([10.3.3](./10.3.3-true-mathematical-integrators.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [10.3.1 Full-wave rectification](./10.3.1-full-wave-rectification.md) | Absolute value exposes a bias level tracking contraction | Deriving an activity level from an oscillating signal |
| [10.3.2 Linear envelope](./10.3.2-linear-envelope.md) | Critically damped low-pass matched to twitch time; $f_c$–$T$ table; migera spring mapping | Before smoothing/delaying ragdoll strength or any activation-to-force signal |
| [10.3.3 True mathematical integrators](./10.3.3-true-mathematical-integrators.md) | Whole-contraction, time-reset and level-reset integration | Choosing an accumulator (fatigue, total effort) over a moving average |

## Relevance to migera

The section supplies a ready, physiologically justified model of **activation
dynamics**: command → rectify → critically damped low-pass at the twitch time
→ force. migera already owns that filter (critical `SpringParams` in
`src/math/spring.rs`), so applying it to the ragdoll's
strength dial in `src/character/anim/ragdoll.rs` would give muscle-like
40–100 ms force build-up and release instead of instant strength changes.
The integrators suggest the complementary signal — accumulated |effort| — as
the driver of a fatigue parameter. Neither is built; both are design leads.

## Where to read in the book
- pp. 269–273 (PDF 282–286). Key: Fig. 10.15 (p. 270) all schemes on one
  record; Eq. 10.8 and Fig. 10.16 with the $f_c$–$T$ table (pp. 271–272).
