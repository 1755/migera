---
title: Chapter 10 — Kinesiological electromyography
description: Digest of Winter ch. 10 on EMG: mostly physiology and instrumentation (low relevance), plus the core that transfers - activation-to-force as a critically damped low-pass at twitch time (1.5-4 Hz), lagged nonlinear EMG-force, and fatigue. Read before designing ragdoll activation dynamics or fatigue.
type: index
status: current
tags:
  - biomechanics
  - muscle
  - signal-processing
  - ragdoll
  - springs
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), ch. 10, pp. 250–280 (PDF pp. 263–293)"
aliases:
  - EMG
  - electromyography
---

# Chapter 10 — Kinesiological electromyography

> **Source:** Winter (2009) ch. 10, pp. 250–280 ·
> [open PDF at p. 250](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=263) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** The EMG is the summed electrical activity of the
motor units near an electrode. About two thirds of the chapter is how that
signal arises (10.1) and how to record it cleanly (10.2: gain, impedance,
bandwidth, CMRR, cross-talk, SENIAM) — accurate background, but irrelevant to
procedural animation. The transferable third is 10.3–10.4: the **linear
envelope** (rectify, then a critically damped 2nd-order low-pass whose impulse
response is the muscle twitch, $f_c = 1/(2\pi T)$, 1.5–4 Hz) is a model of
how a muscle turns an activation command into force; force lags the command by
40–100 ms; the activation→force curve is muscle- and angle-dependent and
often nonlinear; the same activation gives less force when shortening and more
when lengthening; and fatigue makes muscle weaker, slower and tremulous
(8–10 Hz).

**Start here:** [10.3.2 Linear envelope](./10.3-processing-the-emg/10.3.2-linear-envelope.md),
then [10.4](./10.4-emg-vs-biomechanical-variables/INDEX.md). Skip 10.1–10.2
unless you are reading EMG data.

## Key facts
- A muscle low-passes its activation: critically damped 2nd order, impulse response = twitch, $T$ = 40–106 ms ([10.3.2](./10.3-processing-the-emg/10.3.2-linear-envelope.md)).
- That filter is migera's `SpringParams::critical(T·ln 2)`, halflife ≈ 0.028–0.073 s ([10.3.2](./10.3-processing-the-emg/10.3.2-linear-envelope.md)).
- Tension lags EMG by 40–100 ms on onset and persists after EMG stops ([10.4.1](./10.4-emg-vs-biomechanical-variables/10.4.1-emg-vs-isometric-tension.md)).
- EMG-tension is linear for calf muscles but concave-up and joint-angle dependent for elbow flexors ([10.4.1](./10.4-emg-vs-biomechanical-variables/10.4.1-emg-vs-isometric-tension.md)).
- EMG measures activation; tension follows force-velocity, so eccentric work needs much less EMG ([10.4.2](./10.4-emg-vs-biomechanical-variables/10.4.2-emg-during-shortening-and-lengthening.md)).
- Fatigue: less force at constant activation, slower twitches, 8–10 Hz tremor; EMG median frequency −45%, rms +250% ([10.4.3](./10.4-emg-vs-biomechanical-variables/10.4.3-emg-changes-with-fatigue.md)).
- Rectify before averaging: raw EMG has zero mean ([10.3.1](./10.3-processing-the-emg/10.3.1-full-wave-rectification.md)); integrated EMG (mV·s) is not the envelope ([10.3.3](./10.3-processing-the-emg/10.3.3-true-mathematical-integrators.md)).
- Shared signal between two channels is the squared peak cross-correlation, $R_{xy}^2$ ([10.2.5](./10.2-recording-the-emg/10.2.5-surface-emg-cross-talk.md)).
- m.u.a.p. duration 3–20 ms (needle), conduction ≈ 4 m/s; slower conduction lowers the spectrum ([10.1.4](./10.1-electrophysiology-of-contraction/10.1.4-motor-unit-action-potential-duration.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [10.0 Introduction](./10.0-introduction.md) | EMG definition; tension is not its only driver | Deciding whether EMG can stand in for force |
| [10.1 Electrophysiology of muscle contraction](./10.1-electrophysiology-of-contraction/INDEX.md) | End plate, Ca²⁺ cascade, dipole model, m.u.a.p. duration and spectrum | Reading raw EMG; background only for animation |
| [10.2 Recording of the electromyogram](./10.2-recording-the-emg/INDEX.md) | Amplifier specs, hum, cross-talk, SENIAM | Specifying or auditing EMG recordings; not for animation |
| [10.3 Processing of the electromyogram](./10.3-processing-the-emg/INDEX.md) | Rectification, the twitch-matched linear envelope, integrators | Smoothing a commanded ragdoll strength or building an effort accumulator |
| [10.4 Relationship between electromyogram and biomechanical variables](./10.4-emg-vs-biomechanical-variables/INDEX.md) | EMG vs isometric tension, vs velocity, and in fatigue | Mapping activation to torque, adding force-velocity or fatigue |
| [10.5 References](./10.5-references.md) | The primary sources worth chasing | Verifying a ch. 10 number at its source |

## Relevance to migera

migera's active ragdoll (`src/character/anim/ragdoll.rs`) treats per-joint
**strength** as a torque ceiling that changes the instant it is set, with
linear stun recovery. This chapter supplies the physiology for three
refinements that sit between a command and the torque, each a design lead
rather than a built feature:

1. **Activation dynamics** — pass the strength command through the
   linear-envelope filter; migera already has it as a critically damped
   `SpringParams` in `src/character/anim/math/spring.rs` (halflife
   ≈ 0.03–0.07 s). Testable: impulse peaks at $T$, step never overshoots.
2. **Force-velocity gain** — more torque when resisting motion, less when
   assisting it (10.4.2, grounded in ch. 9).
3. **Fatigue** — a gameplay-driven scalar that lowers strength, lengthens
   $T$, and adds 8–10 Hz tremor to targets (10.4.3), possibly accumulated from
   integrated effort (10.3.3).

Everything in 10.1–10.2 (electrodes, amplifiers, CMRR, cross-talk) has no
counterpart in migera.

## Where to read in the book
- pp. 250–257 (PDF 263–270): electrophysiology; pp. 257–269 (PDF 270–282):
  recording; pp. 269–273 (PDF 282–286): processing; pp. 273–277
  (PDF 286–290): EMG vs biomechanics; pp. 277–280 (PDF 290–293): references.
- Most important figures: Fig. 10.15 processing schemes (p. 270), Fig. 10.16
  linear envelope and $f_c$–$T$ table (p. 272), Fig. 10.17 EMG-tension curves
  (p. 274), Fig. 10.18 EMG/tension lag (p. 275).

## See also
- [Chapter 9 — Muscle mechanics](../ch09-muscle-mechanics/INDEX.md) — prerequisite: twitch, force-length, force-velocity and the EMG-driven muscle model this chapter feeds.
- [Chapter 2 — Signal processing](../ch02-signal-processing/INDEX.md) — prerequisite: correlation and spectra used for cross-talk and fatigue measures.
- [Ragdoll and physics](../../character-animation/ragdoll-and-physics/INDEX.md) — applies: where activation dynamics, force-velocity and fatigue would plug in.
- [Animation core](../../character-animation/animation-core/INDEX.md) — applies: the spring numerics that implement the activation filter.
- [Lugaru joint/muscle system](../../character-animation/lugaru-joint-muscle-system.md) — prior art: a game's continuous animated-vs-ragdoll strength dial, the kind of command signal this chapter's activation filter would smooth.
