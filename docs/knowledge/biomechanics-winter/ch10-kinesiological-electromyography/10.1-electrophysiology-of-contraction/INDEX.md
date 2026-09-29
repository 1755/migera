---
title: 10.1 Electrophysiology of muscle contraction
description: Summarizes how a motor unit action potential arises (end plate, Ca2+ cascade, propagating dipole), what an electrode records (biphasic/triphasic waves, 0.5-1.5 cm pick-up), and what sets its duration and spectrum. Read for EMG physiology background; animation relevance is low.
type: index
status: current
tags:
  - biomechanics
  - muscle
  - signal-processing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §10.1, pp. 250–257 (PDF pp. 263–270)"
aliases:
  - m.u.a.p.
  - MUAP
---

# 10.1 Electrophysiology of muscle contraction

> **Source:** Winter (2009) §10.1, pp. 250–257 ·
> [open PDF at p. 250](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=263) ·
> Up: [Chapter 10 — Kinesiological electromyography](../INDEX.md)

Muscle fibres conduct action potentials much like axons. When a motor unit is
recruited, each of its fibres carries a **motor unit action potential
(m.u.a.p.)**; an electrode on or in the muscle records the algebraic sum of
all m.u.a.p.'s passing near it at that instant, with distant units
contributing smaller potentials. The section follows the signal from the
synapse (10.1.1) through the chemistry that makes force (10.1.2), to the
propagating wave an electrode sees (10.1.3), what sets its duration and
frequency content (10.1.4), and how individual units can be decomposed from
the sum (10.1.5).

## Key facts
- A healthy motor end plate converts each nerve impulse into exactly one fibre action potential ([10.1.1](./10.1.1-motor-end-plate.md)).
- Force comes from a Ca²⁺-triggered impulsive cross-bridge force, so it lags the electrical event ([10.1.2](./10.1.2-chemical-events-leading-to-twitch.md)).
- A propagating fibre potential is a current dipole: biphasic at one electrode, triphasic as a bipolar difference ([10.1.3](./10.1.3-muscle-action-potential-generation.md)).
- Surface pick-up reaches ~0.5 cm for small units and ~1.5 cm for the largest ([10.1.3](./10.1.3-muscle-action-potential-generation.md)).
- m.u.a.p. duration: 3–20 ms by needle, ~2× by surface; conduction ≈ 4 m/s ([10.1.4](./10.1.4-motor-unit-action-potential-duration.md)).
- Slower conduction lengthens potentials, lowers the spectrum and raises rectified amplitude — the fatigue signature ([10.1.4](./10.1.4-motor-unit-action-potential-duration.md)).
- Decomposition tracks only a few units near the electrodes ([10.1.5](./10.1.5-detecting-muaps-in-graded-contractions.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [10.1.1 Motor end plate](./10.1.1-motor-end-plate.md) | Synaptic trigger of the fibre potential; end-plate block | You need the neural-command origin of EMG |
| [10.1.2 Sequence of chemical events leading to a twitch](./10.1.2-chemical-events-leading-to-twitch.md) | Ca²⁺/ATP cascade from electrical event to force | You need why force lags EMG |
| [10.1.3 Generation of a muscle action potential](./10.1.3-muscle-action-potential-generation.md) | Dipole model (Eqs. 10.1–10.2), electrode geometry, pick-up zone | Interpreting raw EMG waveforms or electrode choice |
| [10.1.4 Duration of the motor unit action potential](./10.1.4-motor-unit-action-potential-duration.md) | Duration vs area, velocity, depth; spectral consequences | Before reading the fatigue spectral shift |
| [10.1.5 Detection of motor unit action potentials from electromyogram during graded contractions](./10.1.5-detecting-muaps-in-graded-contractions.md) | Decomposition of single units, 0–100% MVC | Curious how firing rates are measured |

## Relevance to migera

Low. Nothing here maps onto code; migera has no electrical layer. The only
ideas that carry over are (1) a command precedes force through a slow
chemical/mechanical process, which motivates low-passing a commanded
activation (see [10.3.2](../10.3-processing-the-emg/10.3.2-linear-envelope.md)),
and (2) fatigue slows the muscle as well as weakening it.

## Where to read in the book
- pp. 250–257 (PDF 263–270). Key figures: Fig. 10.1 dipole (p. 252), Fig. 10.2
  electrode geometry (p. 254), Fig. 10.3 bipolar triphasic wave (p. 255),
  Fig. 10.4 amplitude/frequency vs distance (p. 257).
