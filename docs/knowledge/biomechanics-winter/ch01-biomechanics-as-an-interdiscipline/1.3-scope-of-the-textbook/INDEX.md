---
title: 1.3 Scope of the textbook
description: Map of the whole book — biomechanics as the mechanics and biophysics of the musculoskeletal system, with variables grouped as signal processing, kinematics, kinetics, anthropometry, muscle mechanics, EMG, synthesis and synergies, each routed to its chapter. Read to pick which chapter you need.
type: index
status: current
tags:
  - biomechanics
  - inverse-dynamics
  - anthropometry
  - muscle
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §1.3, pp. 9–12 (PDF pp. 22–25)"
---

# 1.3 Scope of the textbook

> **Source:** Winter (2009) §1.3, pp. 9–12 ·
> [open PDF at p. 9](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=22) ·
> Up: [Chapter 1 — Biomechanics as an interdiscipline](../INDEX.md)

Winter defines the book's subject as the mechanics and biophysics of the
musculoskeletal system as they bear on any movement skill. The neural
system is included only through EMG and its link to muscle mechanics. The
variables fall into kinematics, kinetics, anthropometry, muscle mechanics
and EMG. The 4th edition adds a signal-processing chapter, keeps synthesis
(forward dynamics), and adds a synergies chapter. Each subsection is a
one-paragraph preview of a chapter, so this section is the book's table of
contents with reasons.

## Key facts

- Every biomechanical variable is a time-domain signal. Its spectrum sets the sampling rate, record length and filter cutoff ([1.3.1](./1.3.1-signal-processing.md)).
- 2D angles: 0° = horizontal right, counterclockwise positive. Joint angles are relative, segment angles absolute ([1.3.2](./1.3.2-kinematics.md)).
- Kinetics is the book's main focus because it reveals the cause of movement ([1.3.3](./1.3.3-kinetics.md)).
- An analysis is only as accurate as its anthropometric data ([1.3.4](./1.3.4-anthropometry.md)).
- Forward synthesis was of poor validity in 2009, for lack of correct anthropometrics and degrees of freedom ([1.3.7](./1.3.7-synthesis-human-movement.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [1.3.1 Signal processing](./1.3.1-signal-processing.md) | Why all variables are treated as signals → Ch. 2 | choosing filters, spring bandwidths or sample rates |
| [1.3.2 Kinematics](./1.3.2-kinematics.md) | Relative vs absolute frames, the 2D angle convention → Chs. 3, 7 | reading Winter's angles or mapping them to local/world rotations |
| [1.3.3 Kinetics](./1.3.3-kinetics.md) | Forces, moments, power, energy as the causes → Chs. 5–7 | needing joint torques or power magnitudes |
| [1.3.4 Anthropometry](./1.3.4-anthropometry.md) | Segment parameters every model needs → Ch. 4 | setting ragdoll masses, CoMs or inertias |
| [1.3.5 Muscle and joint biomechanics](./1.3.5-muscle-joint-biomechanics.md) | Force-length/velocity, passive properties, joint limits → Ch. 9 | modelling muscle-like springs or joint limits |
| [1.3.6 Electromyography](./1.3.6-electromyography.md) | EMG as the muscle's final control signal → Ch. 10 | thinking about activation or co-contraction |
| [1.3.7 Synthesis of human movement](./1.3.7-synthesis-human-movement.md) | Forward dynamics and its validity limits → Ch. 8 | designing physics-driven motion |
| [1.3.8 Biomechanical motor synergies](./1.3.8-biomechanical-motor-synergies.md) | Multi-joint cooperation toward one goal → Ch. 11 | coordinating balance and support across joints |

## Relevance to migera

Use this section as the router. For `src/character/anim`, the
highest-yield chapters are 4 (anthropometry, for ragdoll bodies), 5–6
(joint moments and power, for PD budgets), 8 (forward synthesis, which is
the ragdoll's own problem) and 11 (balance synergies, for pelvis and gait
start/stop). Chapters 2–3 matter for filtering. Chapter 10 (EMG) matters
least.

## Where to read in the book

- pp. 9–12 (PDF 22–25): all eight subsections, about a paragraph each.
