---
title: Chapter 1 — Biomechanics as an interdiscipline
description: Winter's framing chapter — measure, describe, analyze, assess; the inverse solution vs synthesis; four levels of neuromusculoskeletal integration with the joint moment as the CNS's control signal; and a scope map routing to Chapters 2–11. Read first to pick the chapters you need.
type: index
status: current
tags:
  - biomechanics
  - inverse-dynamics
  - verification
  - ragdoll
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Ch. 1, pp. 1–13 (PDF pp. 14–26)"
---

# Chapter 1 — Biomechanics as an interdiscipline

> **Source:** Winter (2009) Ch. 1, pp. 1–13 ·
> [open PDF at p. 1](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=14) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** Biomechanics describes, analyzes and assesses
human movement. The four verbs are distinct steps. Measurement and
description (plots, stick diagrams) record what happened. Monitoring
tracks change but cannot show its cause. Analysis turns data into
quantities that cannot be measured, most importantly the **inverse
solution** of a link-segment model: kinematics + anthropometry in, joint
forces and moments out. Assessment is a decision made from that analysis.
The neuromuscular system converges through four levels (motoneuron,
tendon, joint moment, multi-joint synergy). The **net joint moment** is the
CNS's final control signal, and control operates at the joint or synergy
level (Bernstein). The chapter ends with a scope map of the book, one
paragraph per chapter.

**Start here:** §1.2 for the control-level argument, §1.3 to choose a
chapter. §1.0–1.1 are framing.

## Key facts

- The inverse solution computes joint forces and moments that cannot be measured, from kinematics and anthropometry. Synthesis runs the same model forward ([1.1.2](./1.1-measurement-description-analysis-assessment/1.1.2-analysis.md), [1.3.7](./1.3-scope-of-the-textbook/1.3.7-synthesis-human-movement.md)).
- Monitoring documents change but not its cause; reading raw GRF curves is speculative ([1.1.1](./1.1-measurement-description-analysis-assessment/1.1.1-measurement-description-monitoring.md), [1.1.3](./1.1-measurement-description-analysis-assessment/1.1.3-assessment-interpretation.md)).
- The EMG linear envelope is full-wave rectification then a ≈3 Hz low-pass ([1.1.2](./1.1-measurement-description-analysis-assessment/1.1.2-analysis.md)).
- The joint moment (N·m) is the CNS's desired output. Synergies such as the support moment M_s = M_a + M_k + M_h combine joints ([1.2](./1.2-relationship-with-physiology-and-anatomy.md)).
- Variability decreases EMG > moments > kinematics. The redundancy is what allows compensation ([1.2](./1.2-relationship-with-physiology-and-anatomy.md)).
- 2D convention: 0° = +X horizontal, counterclockwise positive; joint angles are relative, segment angles absolute ([1.3.2](./1.3-scope-of-the-textbook/1.3.2-kinematics.md)).
- Winter rated forward-synthesis models of 2009 poorly valid, for lack of correct anthropometrics and degrees of freedom ([1.3.7](./1.3-scope-of-the-textbook/1.3.7-synthesis-human-movement.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [1.0 Introduction](./1.0-introduction.md) | Definition of biomechanics; place within kinesiology | you need the book's framing |
| [1.1 Measurement, description, analysis, and assessment](./1.1-measurement-description-analysis-assessment/INDEX.md) | The four-step hierarchy, Figs. 1.1–1.5, the inverse solution | deciding what a check or visualisation can prove, or placing inverse vs forward dynamics |
| [1.2 Biomechanics and its relationship with physiology and anatomy](./1.2-relationship-with-physiology-and-anatomy.md) | Four integration levels (Fig. 1.6), joint moment as control signal, support moment | designing per-joint or synergy-level control for the ragdoll |
| [1.3 Scope of the textbook](./1.3-scope-of-the-textbook/INDEX.md) | Chapter-by-chapter scope map | choosing which chapter to read |
| [1.4 References](./1.4-references.md) | The six cited works (Bernstein, Winter 1980/84/89, …) | tracing the support-moment or joint-level-control claims |

## Relevance to migera

Two ideas carry over. First, Fig. 1.4's neural → muscle model →
link-segment model → kinematics diagram *is* the active ragdoll run
forward: PD controllers are the muscle model, avian is the link-segment
model. Winter's warning about synthesis (bad anthropometry or too few
degrees of freedom makes the output diverge) is why the ragdoll tracks an
animated target. Second, Winter's describe/monitor/analyze distinction is
migera's verification discipline in other words. A screenshot or gizmo
stick figure is a description. A before/after comparison is monitoring.
Only a measured quantity with a question stated in advance counts as
analysis.

## Where to read in the book

- pp. 1–2 (PDF 14–15): definition.
- pp. 2–7 (PDF 15–20): §1.1 with Figs. 1.1–1.5 (Fig. 1.4, p. 6, is the key block diagram).
- pp. 7–9 (PDF 20–22): §1.2 with Fig. 1.6 (p. 8), four integration levels.
- pp. 9–12 (PDF 22–25): §1.3, scope map.
- pp. 12–13 (PDF 25–26): references.
