---
title: 1.1 Measurement, description, analysis, and assessment
description: Winter's hierarchy of movement study — measure, describe and monitor, analyze (including the link-segment inverse solution), then assess (a decision) — and why skipping a level yields speculation. Read when deciding what a measurement or visualization can actually prove.
type: index
status: current
tags:
  - biomechanics
  - inverse-dynamics
  - verification
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §1.1, pp. 2–7 (PDF pp. 15–20)"
---

# 1.1 Measurement, description, analysis, and assessment

> **Source:** Winter (2009) §1.1, pp. 2–7 ·
> [open PDF at p. 2](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=15) ·
> Up: [Chapter 1 — Biomechanics as an interdiscipline](../INDEX.md)

Winter sorts out terms the field often confuses. Descriptions get passed
off as assessments, and measurements get advertised as analyses. Every
quantitative assessment must be preceded by measurement and description,
and a meaningful diagnosis usually needs a biomechanical analysis.

The section intro (Fig. 1.1) shows three levels of assessment, each ending
in a human decision:
1. **Direct observation.** This overloads even an expert, is subjective,
   and cannot be compared with earlier sessions.
2. **Measured and described data.** Change can be quantified and simple
   analysis done.
3. **Full biomechanical analysis.** This is compared with normative data
   and can find the exact cause.

The same measurement tools serve very different goals. Athletes chase
changes of a few percent. An amputee wants safe walking, not fine detail.
Ergonomics looks for peak tissue stress.

## Key facts

- One device can be described many ways, and one description can come from many devices ([1.1.1](./1.1.1-measurement-description-monitoring.md)).
- Monitoring documents change but not its cause ([1.1.1](./1.1.1-measurement-description-monitoring.md)).
- Analysis produces variables that cannot be measured. The inverse solution turns kinematics and anthropometry into joint forces and moments ([1.1.2](./1.1.2-analysis.md)).
- The EMG linear envelope is full-wave rectification plus a low-pass filter (≈3 Hz) ([1.1.2](./1.1.2-analysis.md)).
- Assessment is a decision. Reading raw GRF curves shows *that* something changed, never *why* ([1.1.3](./1.1.3-assessment-interpretation.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [1.1.1 Measurement, description, and monitoring](./1.1.1-measurement-description-monitoring.md) | Device vs presentation; stick diagrams; monitoring ≠ causation | choosing how to visualise motion or compare runs |
| [1.1.2 Analysis](./1.1.2-analysis.md) | Linear envelope; link-segment model; inverse solution (Fig. 1.4) | placing inverse vs forward dynamics, or designing a muscle-like activation signal |
| [1.1.3 Assessment and interpretation](./1.1.3-assessment-interpretation.md) | Assessment as a decision; limits of pattern-reading GRF curves | deciding what a check must measure to prove a cause |

## Relevance to migera

The hierarchy maps directly onto migera's verification discipline.
Screenshots and gizmo stick figures are *description*. Comparing them is
*monitoring*. A claim that a fix worked needs *analysis* of a specific
quantity (joint angle, body-vs-target error), and the question must be
stated before looking. Fig. 1.4 is also the block diagram of the active
ragdoll, run forward.

## Where to read in the book

- p. 2 (PDF 15): Fig. 1.1, three levels of assessment.
- p. 4 (PDF 17): Fig. 1.2, camera → time history and stick diagram.
- pp. 5–6 (PDF 18–19): Figs. 1.3–1.4, linear envelope and the neural–kinetic–kinematic block diagram.
- p. 7 (PDF 20): Fig. 1.5, GRF curve used diagnostically.
