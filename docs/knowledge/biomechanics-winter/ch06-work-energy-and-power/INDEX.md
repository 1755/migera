---
title: Chapter 6 — Mechanical work, energy, and power
description: "Winter's movement energetics: joint power P = M·ω (generation vs absorption), energy transfer between segments, pendular PE/KE exchange in gait, why work methods mis-count, gait power bursts (ankle push-off), and segment power balances. Read before judging a gait by energy or auditing ragdoll energy."
type: index
status: current
tags:
  - biomechanics
  - energetics
  - locomotion
  - inverse-dynamics
  - muscle
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Chapter 6, pp. 139–175 (PDF pp. 152–188)"
aliases:
  - joint power
  - mechanical energetics
---

# Chapter 6 — Mechanical work, energy, and power

> **Source:** Winter (2009) Ch. 6, pp. 139–175 ·
> [open PDF at p. 139](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=152) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

## Chapter in one minute

Muscles are the body's only energy generators and its main absorbers.
Joint power $P = M_j\omega_j$ says which: positive when moment and angular
velocity agree (concentric, generation), negative when they oppose
(eccentric, absorption). Energy also moves **without** muscle cost — through
joint reaction forces (passive transfer), through muscles spanning segments
that rotate the same way (active transfer), and within a segment as PE and
KE trade like a pendulum. The upper body in walking is nearly pendular
(highest and slowest at midstance, lowest and fastest in double support);
the leg is not. Because of these hidden exchanges, every cheap way of
computing "work" (summed energy increases, whole-body COM energy, summed
segment energies) is wrong in a known direction; joint power is the
practical standard, and in walking it shows one dominant source — the
ankle plantarflexors just before toe-off. Waste comes from four causes:
cocontraction, isometric holding, generation at one joint cancelled by
absorption at another, and jerky stop-start motion.

**Start here:** [6.0.6 Muscle mechanical power](./6.0-energy-and-work-of-muscles/6.0.6-muscle-mechanical-power.md),
then [6.1.1 Causes of inefficient movement](./6.1-efficiency/6.1.1-causes-of-inefficient-movement.md),
[6.2.1 Segment energy and exchange](./6.2-forms-energy-storage/6.2.1-segment-energy-and-within-segment-exchange.md)
and [6.3.1 Internal work calculation](./6.3-internal-and-external-work-calculation/6.3.1-internal-work-calculation.md).

## Key facts

- Joint power $M_j\omega_j$: + = generation (concentric), − = absorption (eccentric); even an elbow flex-extend alternates four bursts ([6.0.6](./6.0-energy-and-work-of-muscles/6.0.6-muscle-mechanical-power.md)).
- Integrate positive and negative phases separately; net work of a movement that returns to its start is zero ([6.0.7](./6.0-energy-and-work-of-muscles/6.0.7-mechanical-work-muscles.md)).
- Joint reaction force · joint velocity transfers energy passively and sums to zero; late-swing leg energy flows up into the trunk ([6.0.9](./6.0-energy-and-work-of-muscles/6.0.9-energy-transfer-between-segments.md)).
- Efficiency ratios can reward bad control: a jerky gait scored 40% vs a smooth one's 33% ([6.1](./6.1-efficiency/INDEX.md)).
- Four causes of waste: cocontraction (%COCON, 24% at the ankle in walking), isometric holding, cross-joint cancellation, jerkiness ([6.1.1](./6.1-efficiency/6.1.1-causes-of-inefficient-movement.md)).
- HAT height (≈1.24–1.29 m) peaks at midstance while forward speed (≈1.25–1.6 m/s) is lowest: pendular exchange ([6.2.1](./6.2-forms-energy-storage/6.2.1-segment-energy-and-within-segment-exchange.md)).
- Steady cyclic motion does equal positive and negative internal work per stride ([6.2.2](./6.2-forms-energy-storage/6.2.2-multisegment-total-energy.md)).
- COM energy underestimates work, summed energy increases overestimate; only joint/muscle power sees cross-joint cancellation (Table 6.1, [6.3.1](./6.3-internal-and-external-work-calculation/6.3.1-internal-work-calculation.md)).
- Walking's biggest burst is ankle push-off, +272 W just before toe-off (Table A.7); the knee mostly absorbs ([6.3.1](./6.3-internal-and-external-work-calculation/6.3.1-internal-work-calculation.md)).
- Per segment, $dE_s/dt = P_{jp}+P_{mp}+P_{jd}+P_{md}$; muscles also transfer energy, only $M(\omega_1-\omega_2)$ is generated/absorbed ([6.4](./6.4-power-balances/INDEX.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [6.0 Introduction](./6.0-energy-and-work-of-muscles/INDEX.md) | Energy vs work, conservation, internal/external work, positive/negative muscle work, joint power, F·V power, passive transfer | before computing or reading any joint power or work number |
| [6.1 Efficiency](./6.1-efficiency/INDEX.md) | Efficiency definitions and their anomaly; four causes of inefficiency; energy-flow chain | designing a metric that flags stiff, fighting or jerky generated motion |
| [6.2 Forms of energy storage](./6.2-forms-energy-storage/INDEX.md) | PE and KE formulas, pendular exchange in gait, conservation %, total body energy | phasing pelvis bob against forward speed, or building an energy-drift test |
| [6.3 Calculation of internal and external work](./6.3-internal-and-external-work-calculation/INDEX.md) | Work methods and blind spots (Table 6.1), knee K1–K5, walking power bursts, external work | choosing an effort metric, or deciding which joint should power push-off |
| [6.4 Power balances at joints and within segments](./6.4-power-balances/INDEX.md) | Muscle transfer (Table 6.2, Eq. 6.27), four-term segment balance (Eq. 6.28), worked example | writing a per-body energy audit or unit-test oracle for the ragdoll |
| [6.5 Problems based on kinetic and kinematic data](./6.5-problems-kinetic-kinematic-data.md) | Seven exercises with answers in Tables A.6–A.7 | looking for known-answer fixtures for energy/power code |
| [6.6 References](./6.6-references.md) | The cited works most useful for animation and physics | chasing a primary source |

## Relevance to migera

- **Energy as a naturalness metric for procedural gait.** Log per-joint
  power $\boldsymbol\tau\cdot\boldsymbol\omega_\text{rel}$ (ragdoll) or
  compute it by inverse dynamics on the kinematic walk; sum
  $|W^+|+|W^-|$ per stride. Human walking is roughly 100 J of mechanical work
  per stride, half positive, half negative
  ([6.1](./6.1-efficiency/INDEX.md)). Extra work, opposite-sign power at
  different joints outside double support, and rapid power reversals are
  the measurable signatures of stiff, fighting and jerky motion
  ([6.1.1](./6.1-efficiency/6.1.1-causes-of-inefficient-movement.md)).
- **Where push-off should come from.** The ankle plantarflexors in the last
  ~10% of stance, then the hip flexors pulling the thigh into swing; the
  knee is mostly a brake ([6.3.1](./6.3-internal-and-external-work-calculation/6.3.1-internal-work-calculation.md)).
  A walk driven by root motion has no energy source at all; a self-powered
  ragdoll walk should reproduce this pattern.
- **Pendular COM exchange as a pelvis target.** Pelvis height should peak at
  midstance and bottom in double support, with forward speed in antiphase
  and a roughly equal PE and KE swing ([6.2.1](./6.2-forms-energy-storage/6.2.1-segment-energy-and-within-segment-exchange.md)).
  migera's `gait.rs` has the bob amplitude; its root speed is currently
  held within ±2% (test `a_walking_body_travels_at_a_steady_speed` in
  `locomotion.rs`), while real HAT speed swings about ±12%.
- **Energy audits for the ragdoll.** Per-body power balance (Eq. 6.28) and
  per-stride total-energy return (6.2.2) localize energy injection; the
  PD drive is applied as a per-body angular acceleration with no parent
  reaction, i.e. an external moment rather than a joint moment
  ([6.4.1](./6.4-power-balances/6.4.1-energy-transfer-via-muscles.md)).

## Where to read in the book

- pp. 139–149 (PDF 152–162): definitions, Figs 6.1–6.8, Eqs 6.1–6.7.
- pp. 149–155 (PDF 162–168): efficiency, causes, energy flow (Figs 6.9–6.12).
- pp. 155–162 (PDF 168–175): energy storage and exchange — **Fig. 6.13 HAT height vs speed** (p. 158).
- pp. 162–167 (PDF 175–180): work methods — **Fig. 6.17 runner's knee power**, **Table 6.1**.
- pp. 167–173 (PDF 180–186): power balances — **Table 6.2**, **Fig. 6.19**, Example 6.5.
- Appendix A Table A.7 (pp. 358–360, PDF 371–373): walking joint powers and transfers per frame.

## See also

- [Chapter 5 — Kinetics: forces and moments of force](../ch05-kinetics-forces-and-moments/INDEX.md) — prerequisite: the joint moments and reaction forces every power here multiplies.
- [Chapter 4 — Anthropometry](../ch04-anthropometry/INDEX.md) — prerequisite: segment masses, COMs and inertias for segment energies.
- [Chapter 7 — Three-dimensional kinematics and kinetics](../ch07-three-dimensional-kinematics-and-kinetics/INDEX.md) — deeper: 3D joint powers (7.4.4) and averaged gait power curves with named bursts (7.4.5).
- [Ragdoll and physics](../../character-animation/ragdoll-and-physics/INDEX.md) — applies: the avian ragdoll these energy audits would target.
