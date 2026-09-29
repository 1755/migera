---
title: 6.3 Calculation of internal and external work
description: "Surveys internal-work methods and their blind spots (Table 6.1), a runner's knee power phases, walking power bursts (ankle push-off), and how external work is measured at the load interface. Read before choosing a work or effort metric for a generated gait."
type: index
status: current
tags:
  - biomechanics
  - energetics
  - locomotion
  - inverse-dynamics
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §6.3, pp. 162–167 (PDF pp. 175–180)"
---

# 6.3 Calculation of internal and external work

> **Source:** Winter (2009) §6.3, pp. 162–167 ·
> [open PDF at p. 162](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=175) ·
> Up: [Chapter 6 — Mechanical work, energy, and power](../INDEX.md)

Researchers have computed internal and external work in many incompatible
ways: some treat whole-body COM energy as all the internal work, some count
only "vertical work" of the COM, and exercise physiology often ignores
internal work altogether. Winter's remedy is a **checklist** of every source
of metabolically costly muscle activity, used to see how complete a given
analysis is — the same list as the causes of inefficiency in
[6.1.1](../6.1-efficiency/6.1.1-causes-of-inefficient-movement.md). 6.3.1
walks the internal-work methods from crudest to best using gait as the
example; 6.3.2 covers external work.

## Key facts

- Summing segment energy increases (Fenn 1929) overestimates — sprinters at 3 hp — because it ignores exchange and transfer ([6.3.1](./6.3.1-internal-work-calculation.md)).
- Whole-body COM energy underestimates: reciprocal segment motions cancel in the vector COM but not in scalar energy ([6.3.1](./6.3.1-internal-work-calculation.md)).
- Only joint power reveals cross-joint generation vs absorption; only muscle power reveals cocontraction; nothing mechanical sees isometric holding (Table 6.1, [6.3.1](./6.3.1-internal-work-calculation.md)).
- Runner's knee: −53, +31, −11, −24, +5 J (K1–K5) — mostly absorption ([6.3.1](./6.3.1-internal-work-calculation.md)).
- In walking the largest generation burst is ankle push-off, +272 W just before toe-off in Table A.7 ([6.3.1](./6.3.1-internal-work-calculation.md)).
- External work can only be separated by measuring force and velocity at the body–load contact (Eq. 6.26, [6.3.2](./6.3.2-external-work-calculation.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [6.3.1 Internal work calculation](./6.3.1-internal-work-calculation.md) | Five methods, Table 6.1 blind spots, Eqs 6.24–6.25, knee K1–K5, walking ankle/knee/hip power bursts | choosing an energy metric, or deciding which joint should power push-off |
| [6.3.2 External work calculation](./6.3.2-external-work-calculation.md) | Eq. 6.26 at the load interface | measuring work done on an object, by a hit, or leaking through a sliding foot |

## Relevance to migera

Table 6.1 is the reason to measure a procedural gait with per-joint power
rather than with COM or summed segment energy: the cheaper metrics are
blind to exactly the artefacts (joints fighting, reciprocal limbs) a
procedural stack produces. The Table A.7 burst pattern is the target shape
for where a self-powered walking ragdoll should generate and absorb energy.

## Where to read in the book

- p. 162 (PDF 175): section intro and checklist idea.
- pp. 162–166 (PDF 175–179): 6.3.1, Fig. 6.17, Table 6.1.
- p. 167 (PDF 180): 6.3.2.
