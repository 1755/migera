---
title: 6.2 Forms of energy storage
description: "Gives segment potential, translational and rotational kinetic energy (Eqs 6.14–6.17) with a thrown-ball worked example, then within-segment PE/KE exchange (HAT pendulum in gait, conservation Eqs 6.18–6.22) and total body energy (Eq. 6.23). Read when computing segment energies or tuning pelvis bob against forward speed."
type: index
status: current
tags:
  - biomechanics
  - energetics
  - physics
  - locomotion
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §6.2, pp. 155–162 (PDF pp. 168–175)"
aliases:
  - segment energy
  - potential energy
  - kinetic energy
---

# 6.2 Forms of energy storage

> **Source:** Winter (2009) §6.2, pp. 155–162 ·
> [open PDF at p. 155](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=168) ·
> Up: [Chapter 6 — Mechanical work, energy, and power](../INDEX.md)

A segment stores energy in three forms — gravitational potential,
translational kinetic and rotational kinetic — and its total can stay
constant while they trade among themselves. The section defines the three,
shows the trade on a thrown ball, then (6.2.1) measures how much trading a
real segment does, and (6.2.2) sums segments into total body energy and
explains what that sum hides.

## The three stores (pp. 155–157)

$$PE = mgh \quad[\text{J}] \tag{6.14}$$

m mass (kg), g = 9.8 m/s², h height of the COM (m). The datum is arbitrary
but should be chosen for the problem — usually the lowest point the body
reaches (water level for a diver, the lowest point of the walkway).

$$\text{translational } KE = \tfrac12 m v^2 \tag{6.15}$$
$$\text{rotational } KE = \tfrac12 I\omega^2 \tag{6.16}$$

v COM velocity (m/s), I moment of inertia about the COM (kg·m²), ω segment
angular velocity (rad/s). Both scale with velocity squared, so direction
does not matter and the minimum is zero at rest.

$$E_s = PE + KE_t + KE_r = mgh + \tfrac12 mv^2 + \tfrac12 I\omega^2 \quad[\text{J}] \tag{6.17}$$

**Example 6.3** (the 1 kg ball of Example 6.1 thrown vertically, released
2 m up after 100 N net for 180 ms): release speed 18 m/s, KE 162 J (equal to
the work done on it), PE 19.6 J, total 181.6 J. At the apex KE = 0, so
$h_2 = 181.6/(1.0\times9.8) = 18.5$ m. At the ground PE = 0, KE = 181.6 J,
v = 19.1 m/s — a bit faster than release because it fell 2 m further.

## Key facts

- Segment energy is $mgh + \tfrac12mv^2 + \tfrac12I\omega^2$; choose the PE datum as the lowest point of the movement (Eq. 6.17, this INDEX).
- HAT height peaks at midstance and forward velocity peaks in double support — a near-pendular exchange ([6.2.1](./6.2.1-segment-energy-and-within-segment-exchange.md)).
- The walking leg conserves only ≈0.49 J of 16.65 J of component swings: almost all its energy change is real work ([6.2.1](./6.2.1-segment-energy-and-within-segment-exchange.md)).
- Percent conservation $C_s = (W_s' - W_s)/W_s'$ with sums of absolute changes (Eqs 6.19–6.22, [6.2.1](./6.2.1-segment-energy-and-within-segment-exchange.md)).
- Total body energy hides inter-segment transfer and cross-joint cancellation ([6.2.2](./6.2.2-multisegment-total-energy.md)).
- In steady cyclic motion internal positive work per stride equals internal negative work ([6.2.2](./6.2.2-multisegment-total-energy.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [6.2.1 Energy of a body segment and exchanges of energy within the segment](./6.2.1-segment-energy-and-within-segment-exchange.md) | HAT pendular exchange in gait (Fig. 6.13), exchange formulas, leg example | phasing pelvis bob against forward speed, or scoring conservation |
| [6.2.2 Total energy of a multisegment system](./6.2.2-multisegment-total-energy.md) | Eq. 6.23, what the sum hides, muscled pendulum, per-stride work balance | using total body energy as a steady-state or drift test |

## Relevance to migera

Eq. 6.17 is the per-body energy of each avian ragdoll body (mass and
inertia from anthropometry, [Chapter 4](../../ch04-anthropometry/INDEX.md)),
and summing it gives a drift detector. For the kinematic walk, 6.2.1 gives
the phase relationship between pelvis height and forward speed that a
natural gait shows; migera's `gait.rs` already has the bob amplitude but
its root speed is held nearly constant.

## Where to read in the book

- pp. 155–157 (PDF 168–170): Eqs 6.14–6.17, Example 6.3.
- pp. 157–160 (PDF 170–173): within-segment exchange (Figs 6.13–6.15, Eqs 6.18–6.22).
- pp. 160–162 (PDF 173–175): total body energy (Eq. 6.23, Fig. 6.16).
