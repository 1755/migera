---
title: 6.1 Efficiency
description: "Defines metabolic, mechanical and work efficiency (Eqs 6.8–6.11), shows why a jerky gait can score higher efficiency than a smooth one, and lists the four causes of inefficient movement and the metabolic-to-mechanical energy chain. Read before using any efficiency or effort number to judge motion."
type: index
status: current
tags:
  - biomechanics
  - energetics
  - muscle
  - locomotion
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §6.1, pp. 149–155 (PDF pp. 162–168)"
aliases:
  - mechanical efficiency
  - work efficiency
---

# 6.1 Efficiency

> **Source:** Winter (2009) §6.1, pp. 149–155 ·
> [open PDF at p. 149](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=162) ·
> Up: [Chapter 6 — Mechanical work, energy, and power](../INDEX.md)

"Efficiency" is the most abused term in movement energetics, because both
numerator and denominator get defined carelessly (Gaesser and Brooks 1975;
Whipp and Wasserman 1969). Two separate things lower it: poor conversion of
metabolic to mechanical energy at the tendon (**metabolic/muscle
efficiency**, depending on conditioning, fatigue, diet, disease) and poor
**neural control** of that energy. A standard efficiency ratio only measures
the first — so a clumsy mover can look *more* efficient than a skilled one.
The children then list the four control-side causes and the full energy
chain.

## Efficiency definitions (pp. 150–151)

$$\text{metabolic (muscle) efficiency} = \frac{\sum\text{mechanical work done by all muscles}}{\text{metabolic work of muscles}} \tag{6.8}$$

Not computable: it needs every muscle's force and velocity history and each
muscle's own metabolic cost. The practical compromises:

$$\text{mechanical efficiency} = \frac{\text{mechanical work (internal + external)}}{\text{metabolic cost} - \text{resting metabolic cost}} \tag{6.9}$$

$$\text{work efficiency} = \frac{\text{external mechanical work}}{\text{metabolic cost} - \text{zero-work metabolic cost}} \tag{6.10}$$

Resting cost e.g. sitting still on the bicycle; zero-work cost e.g.
freewheeling. Since positive work costs more than equal negative work, and
level gait has equal amounts of each (uphill more positive, downhill more
negative), all three ratios depend on the positive/negative mix. The
split form avoids that:

$$\frac{\text{positive work}}{\eta_+} + \frac{\text{negative work}}{\eta_-} = \text{metabolic cost} \tag{6.11}$$

with $\eta_+$, $\eta_-$ the positive and negative work efficiencies.

**The anomaly (p. 151).** Healthy adult: 100 J mechanical work per stride
(half positive, half negative), 300 J metabolic → 33%. Neurologically
disabled adult with a jerky gait: 200 J mechanical, 500 J metabolic → 40%.
The disabled walker converts metabolic energy at the tendon well but
controls it badly; the ratio rewards the extra wasted work.

## Key facts

- Efficiency ratios measure metabolic conversion, not control quality; a jerky gait scored 40% vs a smooth one's 33% (this INDEX, p. 151).
- Level walking does equal positive and negative work; efficiency must account for the mix (Eq. 6.11, this INDEX).
- Four causes of wasted muscle work: cocontraction, isometric holding, cross-joint cancellation, jerkiness ([6.1.1](./6.1.1-causes-of-inefficient-movement.md)).
- Cocontraction index %COCON = 2·common/(A+B); tibialis anterior vs soleus in walking = 24% ([6.1.1](./6.1.1-causes-of-inefficient-movement.md)).
- In double support, trailing-leg push-off (positive) overlaps leading-leg weight acceptance (negative) — necessary, not pathological ([6.1.1](./6.1.1-causes-of-inefficient-movement.md)).
- Mechanical energy at the tendon goes to cocontraction, holding, absorption elsewhere, or net body energy / external work ([6.1.2](./6.1.2-summary-energy-flows.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [6.1.1 Causes of inefficient movement](./6.1.1-causes-of-inefficient-movement.md) | The four causes, %COCON (Eqs 6.12–6.13), push-off vs weight acceptance | designing a detector for stiff, fighting or jerky generated motion |
| [6.1.2 Summary of energy flows](./6.1.2-summary-energy-flows.md) | Metabolic → heats → tendon → mechanical sinks (Fig. 6.12) | deciding what an effort metric can and cannot see |

## Relevance to migera

migera has no metabolism, so efficiency ratios themselves are not usable.
The durable lesson is the anomaly: **a ratio can reward bad motion**. Any
naturalness score for a procedural gait should penalise total absolute work
per stride and its cancellation patterns, not normalise work away. The four
causes in 6.1.1 translate one-to-one into signals on the PD ragdoll
(stiffness, static torque, opposite-sign joint powers, power reversals).

## Where to read in the book

- pp. 149–150 (PDF 162–163): the two reasons for inefficiency; Eqs 6.8–6.11.
- p. 151 (PDF 164): the healthy-vs-disabled anomaly; start of 6.1.1.
- pp. 151–155 (PDF 164–168): causes (Figs 6.9–6.11) and energy-flow summary (Fig. 6.12).
