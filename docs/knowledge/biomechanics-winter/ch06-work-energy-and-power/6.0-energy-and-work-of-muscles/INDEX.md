---
title: 6.0 Introduction
description: "Winter's energetics vocabulary: energy vs work, per-segment conservation, internal vs external work, positive/negative muscle work, joint power P = M·ω, phase-wise work integration, F·V power, passive transfer through joints. Read before computing or interpreting any joint power or work number."
type: index
status: current
tags:
  - biomechanics
  - energetics
  - muscle
  - inverse-dynamics
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §6.0, pp. 139–149 (PDF pp. 152–162)"
---

# 6.0 Introduction

> **Source:** Winter (2009) §6.0, pp. 139–149 ·
> [open PDF at p. 139](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=152) ·
> Up: [Chapter 6 — Mechanical work, energy, and power](../INDEX.md)

The introduction sets up every tool the rest of the chapter uses. Energy is
a state, work a flow; a segment's energy changes only by flows across its
joints and tendons. Muscles are the only generators and the main absorbers:
concentric action (moment and angular velocity same sign) generates,
eccentric action absorbs, and joint power $P_m = M_j\omega_j$ captures both
with its sign. Work is that power integrated **phase by phase**. Separately,
joint reaction forces moving with the joint centre transfer energy passively
between neighbouring segments, costing nothing and summing to zero over the
body.

Winter opens (p. 139) by ranking energetics as the most informative variable
in biomechanics: joint powers were the most discriminating measure in all his
pathological-gait assessments, catching problems that EMG or moment curves
alone missed.

The notes run from definitions (6.0.1–6.0.3), through the sign of muscle
work and its rate (6.0.4–6.0.7), to the two ways energy enters a body from
outside a muscle: a force on a moving load (6.0.8) and a force at a moving
joint (6.0.9).

## Key facts

- Energy is state (J at an instant), work is flow between bodies over time ([6.0.1](./6.0.1-mechanical-energy-work.md)).
- A segment's energy change equals the signed sum of its boundary flows; in Fig. 6.1, 6.0 J or 300 W ([6.0.2](./6.0.2-law-conservation-energy.md)).
- Level walking has no external work; raising one's own body weight counts as external ([6.0.3](./6.0.3-internal-vs-external-work.md)).
- $M_j\omega_j>0$ is concentric generation, $<0$ eccentric absorption ([6.0.4](./6.0.4-positive-work-muscles.md), [6.0.5](./6.0.5-negative-work-muscles.md)).
- Even a single elbow flex-extend gives two generation and two absorption bursts; moment and velocity run ~90° out of phase ([6.0.6](./6.0.6-muscle-mechanical-power.md)).
- A movement that returns to its start has zero net work; integrate positive and negative phases separately ([6.0.7](./6.0.7-mechanical-work-muscles.md)).
- Power of a force is $\mathbf F\cdot\mathbf V$; 100 N on 1 kg for 180 ms does 162 J ([6.0.8](./6.0.8-work-on-an-external-load.md)).
- Joint reaction force × joint velocity is a passive, zero-sum transfer; late-swing leg energy flows up into the trunk ([6.0.9](./6.0.9-energy-transfer-between-segments.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [6.0.1 Mechanical energy and work](./6.0.1-mechanical-energy-work.md) | Energy (state) vs work (flow) | defining any energy metric |
| [6.0.2 Law of conservation of energy](./6.0.2-law-conservation-energy.md) | Segment energy change = sum of boundary flows; power-balance form | building an energy-balance check for the ragdoll |
| [6.0.3 Internal versus external work](./6.0.3-internal-vs-external-work.md) | Internal vs external work, own-weight exception, coupled cyclists | separating body motion from work on a load or a hit |
| [6.0.4 Positive work of muscles](./6.0.4-positive-work-muscles.md) | Concentric = moment and ω same sign = generation | labelling a joint power burst as generation |
| [6.0.5 Negative work of muscles](./6.0.5-negative-work-muscles.md) | Eccentric = opposite signs = absorption by an external-force-driven stretch | interpreting braking/weight-acceptance phases |
| [6.0.6 Muscle mechanical power](./6.0.6-muscle-mechanical-power.md) | Eq. 6.1 $P=M_j\omega_j$; four-burst elbow example | before computing a joint power curve |
| [6.0.7 Mechanical work of muscles](./6.0.7-mechanical-work-muscles.md) | Eq. 6.2; integrate between zero crossings, net can be zero | turning power into a cost or effort score |
| [6.0.8 Mechanical work done on an external load](./6.0.8-work-on-an-external-load.md) | Eqs 6.3–6.7, F·V power; baseball examples | computing power of a contact or hit force |
| [6.0.9 Mechanical energy transfer between segments](./6.0.9-energy-transfer-between-segments.md) | Passive joint-force transfer, zero-sum; swing-to-trunk transfer | auditing energy through ragdoll joints |

## Relevance to migera

This section is the measurement kit for judging a procedural gait by its
energetics: log $\boldsymbol\tau\cdot\boldsymbol\omega_\text{rel}$ per ragdoll
joint (or per rig joint after inverse dynamics), integrate positive and
negative phases per stride, and check per-body energy against boundary
flows. The sign rules tell you whether a joint is driving or braking, which
is what distinguishes an ankle-driven push-off from a gait that is simply
dragged along by the root.

## Where to read in the book

- pp. 139–141 (PDF 152–154): motivation, energy vs work, conservation (Fig. 6.1).
- pp. 141–143 (PDF 154–156): internal vs external work (Figs 6.2, 6.3).
- pp. 143–146 (PDF 156–159): muscle work sign and power (Figs 6.4–6.6, Eqs 6.1–6.2).
- pp. 146–149 (PDF 159–162): F·V power, baseball examples (Fig. 6.7), passive transfer (Fig. 6.8).
