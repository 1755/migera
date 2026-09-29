---
title: 9.0 Introduction
description: "Motor units, recruitment and the size principle, fast/slow twitch types, the twitch as a critically damped impulse response F0·(t/T)·e^(−t/T), and why voluntary force takes ~200 ms to rise and ~300 ms to fall. Read before adding activation dynamics or strength timing to the ragdoll."
type: index
status: current
tags:
  - biomechanics
  - muscle
  - springs
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §9.0, pp. 224–231 (PDF pp. 237–244)"
---

# 9.0 Introduction

> **Source:** Winter (2009) §9.0, pp. 224–231 ·
> [open PDF at p. 224](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=237) ·
> Up: [Chapter 9 — Muscle mechanics](../INDEX.md)

This section builds muscle force from the bottom up: the smallest controllable
unit (the motor unit), how units are switched on (recruitment by size, then
rate coding), what one unit's force looks like over time (the twitch, a
critically damped second-order impulse response), and how twitches add up
into a voluntary contraction that needs ~200 ms to build and ~300 ms to
release. The chapter's own opening paragraph frames muscle as the "living"
part of the system and promises to relate unit-level properties to
whole-muscle function; that promise is kept here for time behaviour and in
§9.1–9.2 for length and velocity.

Read in order: 9.0.1 → 9.0.2 → 9.0.3 establish *which* units fire; 9.0.4 →
9.0.5 establish *how fast* each responds; 9.0.6 combines them.

## Key facts

- A motor unit spans 3 to ~2000 fibers; sarcomeres run 1.5 / 2.5 / 4.0 µm (shortest / rest / longest) ([9.0.1](./9.0.1-motor-unit.md)).
- Force rises by faster firing and by recruiting more units; units drop out in reverse, at lower rates than they joined ([9.0.2](./9.0.2-recruitment-motor-units.md)).
- Smallest units first; firing 5–13 Hz at recruitment, 15–60 Hz max, up to 120 Hz ballistic ([9.0.3](./9.0.3-size-principle.md)).
- Slow-twitch peaks in 60–120 ms, fast-twitch in 10–50 ms ([9.0.4](./9.0.4-fast-and-slow-twitch-motor-units.md)).
- Twitch $F(t)=F_0\,(t/T)\,e^{-t/T}$, peak $F_0/e$ at $t=T$; T ≈ 45–116 ms in human muscles ([9.0.5](./9.0.5-muscle-twitch.md)).
- Rapid MVC: ~200 ms on, ~300 ms off, force lasts ~150 ms past the EMG ([9.0.6](./9.0.6-shape-graded-contractions.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [9.0.1 The motor unit](./9.0.1-motor-unit.md) | Motor unit, sarcomere lengths, series/parallel connective tissue | deciding the granularity of an actuator model |
| [9.0.2 Recruitment of motor units](./9.0.2-recruitment-motor-units.md) | Rate coding + recruitment; reverse-order dropout with hysteresis | mapping a strength command to force |
| [9.0.3 Size principle](./9.0.3-size-principle.md) | Small units first; firing-rate ranges | adding force noise/tremor or fine-vs-coarse control |
| [9.0.4 Types of motor units—fast- and slow-twitch classification](./9.0.4-fast-and-slow-twitch-motor-units.md) | Type I vs. II properties; time to peak 60–120 vs. 10–50 ms | choosing per-joint response times |
| [9.0.5 The muscle twitch](./9.0.5-muscle-twitch.md) | Eq. 9.1 twitch; T per muscle; temperature effect | implementing activation dynamics or a strength filter |
| [9.0.6 Shape of graded contractions](./9.0.6-shape-graded-contractions.md) | 200 ms on / 300 ms off; 150 ms force after EMG ends | timing ragdoll strength changes, stun and recovery |

## Relevance to migera

migera's active ragdoll (`src/character/anim/ragdoll.rs`) changes strength
instantly: `RagdollStrength` scales the PD torque ceiling on the next step.
This section says real force is a **low-passed** version of the command: a
critically damped second-order filter with T ≈ 50–100 ms, slower to release
than to build. Because migera's critical springs have exactly that impulse
response (halflife = T·ln 2), adding activation dynamics would be cheap. It is
not modeled today.

## Where to read in the book

- pp. 224–225 (PDF 237–238): motor unit and sarcomere (Fig. 9.1).
- pp. 225–228 (PDF 238–241): recruitment and size principle (Figs. 9.2, 9.3).
- pp. 228–230 (PDF 241–243): fiber types (Fig. 9.4), twitch Eq. 9.1 and Fig. 9.5.
- pp. 230–231 (PDF 243–244): graded contractions, Fig. 9.6.
