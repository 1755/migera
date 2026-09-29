---
title: 10.4 Relationship between electromyogram and biomechanical variables
description: Summarizes what EMG can say about force - a coarse, muscle- and length-dependent, lagged isometric tension predictor; an activation (not force) measure under shortening/lengthening; and a fatigue indicator via rising amplitude and falling spectrum. Read before mapping activation to ragdoll torque or designing fatigue.
type: index
status: current
tags:
  - biomechanics
  - muscle
  - ragdoll
  - signal-processing
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §10.4, pp. 273–277 (PDF pp. 286–290)"
---

# 10.4 Relationship between electromyogram and biomechanical variables

> **Source:** Winter (2009) §10.4, pp. 273–277 ·
> [open PDF at p. 273](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=286) ·
> Up: [Chapter 10 — Kinesiological electromyography](../INDEX.md)

The reason to process EMG is to relate it to muscle function. A reliable
EMG → tension mapping would be a cheap, non-invasive force meter, and the EMG
may also carry information on metabolism, power, fatigue and recruitment. The
section tests that hope in three regimes: isometric (10.4.1: works coarsely,
with a twitch-sized lag and a muscle-specific, often nonlinear curve),
changing length (10.4.2: EMG tracks activation while force follows the
force-velocity curve), and fatigue (10.4.3: force drops while EMG amplitude
rises and its spectrum falls). Together they separate three quantities a
simulation should keep apart — **command/activation**, **force**, and
**muscle state** (velocity, fatigue).

## Key facts
- EMG-tension is linear for some muscles (calf) and concave-up and angle-dependent for others (elbow flexors) ([10.4.1](./10.4.1-emg-vs-isometric-tension.md)).
- Tension lags EMG by the twitch delay, 40–100 ms, on both rise and fall ([10.4.1](./10.4.1-emg-vs-isometric-tension.md)).
- A calibrated envelope is only a coarse tension predictor, and only when length changes slowly ([10.4.1](./10.4.1-emg-vs-isometric-tension.md)).
- At maximal effort EMG stays roughly constant while tension falls in shortening and rises in lengthening ([10.4.2](./10.4.2-emg-during-shortening-and-lengthening.md)).
- Negative work needs considerably less EMG than equal positive work ([10.4.2](./10.4.2-emg-during-shortening-and-lengthening.md)).
- Fatigue: less tension at constant activation, slower twitches, 8–10 Hz tremor ([10.4.3](./10.4.3-emg-changes-with-fatigue.md)).
- Fatigue EMG: median frequency −45%, rms +250%, conduction velocity −10% ([10.4.3](./10.4.3-emg-changes-with-fatigue.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [10.4.1 Electromyogram versus isometric tension](./10.4.1-emg-vs-isometric-tension.md) | Linear vs nonlinear EMG-tension curves; dynamic lag (Fig. 10.18); open multi-muscle questions | Before mapping activation linearly to torque, or choosing a strength filter delay |
| [10.4.2 Electromyogram during muscle shortening and lengthening](./10.4.2-emg-during-shortening-and-lengthening.md) | EMG = activation; force-velocity sets tension; eccentric is cheaper | Before making ragdoll torque velocity-dependent or costing effort |
| [10.4.3 Electromyogram changes during fatigue](./10.4.3-emg-changes-with-fatigue.md) | Mechanical and spectral fatigue signatures; Eqs. 10.9–10.10 | Designing a fatigue parameter (weaker, slower, shakier) |

## Relevance to migera

migera's ragdoll already has the "command" half of this picture: a per-joint
strength dial and a stun/recover mechanism in `src/character/anim/ragdoll.rs`
that set a torque ceiling directly. The section argues for three
refinements between command and force, each separately testable: a
twitch-time low-pass (lag), a force-velocity gain (more force when braking),
and a fatigue state (weaker, slower, tremor). All are design leads, not
built features.

## Where to read in the book
- pp. 273–277 (PDF 286–290). Key: Fig. 10.17 EMG-tension curves (p. 274),
  Fig. 10.18 EMG/tension lag traces (p. 275), Eqs. 10.9–10.10 (p. 277).
