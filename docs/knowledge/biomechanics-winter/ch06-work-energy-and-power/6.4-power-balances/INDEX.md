---
title: 6.4 Power balances at joints and within segments
description: "Completes segment energy accounting: muscles transfer energy between segments rotating the same way (Eq. 6.27, Table 6.2), and each segment's energy rate equals two passive plus two muscle power terms (Eq. 6.28), worked for leg and thigh. Read before writing a per-body energy audit."
type: index
status: current
tags:
  - biomechanics
  - energetics
  - inverse-dynamics
  - verification
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §6.4, pp. 167–173 (PDF pp. 180–186)"
---

# 6.4 Power balances at joints and within segments

> **Source:** Winter (2009) §6.4, pp. 167–173 ·
> [open PDF at p. 167](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=180) ·
> Up: [Chapter 6 — Mechanical work, energy, and power](../INDEX.md)

The chapter already has conservation within a segment (6.0.2), muscle power
(6.0.6) and passive transfer across joints (6.0.9). One piece is missing
for a complete segment-by-segment power balance: active muscles also
**transfer** energy between segments, on top of generating and absorbing
it. 6.4.1 adds that piece; 6.4.2 assembles the four-term balance and proves
it on real walking data.

## Key facts

- A moment across a joint whose segments rotate the same way moves energy from one to the other; only $M(\omega_1-\omega_2)$ is generated or absorbed ([6.4.1](./6.4.1-energy-transfer-via-muscles.md)).
- An isometric muscle in a moving limb is a pure transfer device ([6.4.1](./6.4.1-energy-transfer-via-muscles.md)).
- $dE_s/dt = P_{jp}+P_{mp}+P_{jd}+P_{md}$ with muscle terms using the segment's own ω (Eq. 6.28, [6.4.2](./6.4.2-power-balance-within-segments.md)).
- In early swing the knee extensors absorb 15.89 W while passing 7.19 W thigh → leg; 44.81 W crosses the knee passively ([6.4.2](./6.4.2-power-balance-within-segments.md)).
- The book's leg balance has a velocity typo (0.7 for 0.07); corrected residual is ≈3.6 W, confirmed by Table A.7 ([6.4.2](./6.4.2-power-balance-within-segments.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [6.4.1 Energy transfer via muscles](./6.4.1-energy-transfer-via-muscles.md) | Eq. 6.27, Table 6.2 generation/absorption/transfer cases | splitting a joint's power, or deciding whether a PD drive is an internal joint torque |
| [6.4.2 Power balance within segments](./6.4.2-power-balance-within-segments.md) | Eq. 6.28, Example 6.5 worked numbers and corrections | building a per-body energy oracle or unit-test fixture |

## Relevance to migera

Together these give a closed energy audit for a jointed body: per body,
state-derived $dE/dt$ must equal constraint power at each joint plus drive
power. For migera's ragdoll the audit also exposes a modelling fact: its PD
drive is applied as an angular acceleration on each body alone (no parent
reaction), so it behaves as an external moment, not a Winter joint moment
([6.4.1](./6.4.1-energy-transfer-via-muscles.md)).

## Where to read in the book

- pp. 167–169 (PDF 180–182): 6.4 intro, Fig. 6.18, Eq. 6.27, Table 6.2.
- pp. 168–173 (PDF 181–186): Fig. 6.19, Eq. 6.28, Example 6.5, Fig. 6.20.
