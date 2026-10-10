---
title: 9.3 Muscle modeling
description: "Muscle models as a force generator plus linear or nonlinear springs and dampers; Fung's equivalence between series/parallel spring-damper arrangements (Eq. 9.4); the exponential active state; and a worked EMG-driven mass-spring-damper model. Read before building a muscle-like actuator from spring/damper elements."
type: index
status: current
tags:
  - biomechanics
  - muscle
  - springs
  - math
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §9.3, pp. 243–247 (PDF pp. 256–260)"
aliases:
  - Hill-type model
  - active state
  - Fung equivalence
---

# 9.3 Muscle modeling

> **Source:** Winter (2009) §9.3, pp. 243–247 ·
> [open PDF at p. 243](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=256) ·
> Up: [Chapter 9 — Muscle mechanics](../INDEX.md)

Muscle models combine an **active force generator** (the contractile element)
with **passive springs and dampers** for the series and parallel tissue. For
linear elements the arrangement doesn't matter — any two-spring-one-damper
layout can be converted to another with identical dynamics. The section
surveys the element types and then gives one complete working example driven by
EMG ([9.3.1](./9.3.1-emg-driven-model-example.md)).

## Section intro, distilled

- **Model lineage:** Crowe (1970) and Gottlieb & Agarwal (1971) — contractile
  component + linear series and parallel elastic + linear viscous damper;
  Glantz (1974) — nonlinear elastic + linear viscous; Winter (1976) — mass +
  linear spring + damper to reproduce the critically damped twitch. The book
  explicitly does not rank them.
- **Element catalog** (Fig. 9.16): linear spring $F = kx$ and damper
  $F = k\dot{x}$; nonlinear springs $F = kx^a$ and $F = k(e^x - 1)$; nonlinear
  dampers $F = k\dot{x}^a$ and $F = k(e^{\dot{x}} - 1)$. The exponent $a$ is
  usually > 1; viscous friction often goes roughly as **velocity squared**.
- **Equivalent linear layouts** (Fig. 9.17a, Fung, 1971; Eq. 9.4, checked on
  the rendered figure). Layout 1: $k_1$ in series with ($k_2 \parallel b_1$).
  Layout 2: $k_3$ in parallel with ($k_4$ in series with $b_2$). They are
  dynamically identical when

  $$k_1 = k_3 + k_4, \qquad \frac{k_1 k_2}{k_1 + k_2} = k_3, \qquad \frac{b_1}{k_1 + k_2} = \frac{b_2}{k_4}$$

  i.e. equal instantaneous stiffness, equal static stiffness, and equal
  relaxation time.
- **Active state** (Fig. 9.17b): the contractile element's force time course,
  often assumed to be an **exponential** response to a stimulus; passing it
  through the viscoelastic elements gives the twitch-shaped tendon force
  $F_t$.

## Key facts

- Linear spring/damper layouts are interchangeable via Eq. 9.4 (Fung, 1971) (this INDEX, above).
- Nonlinear passive elements are power-law or exponential; viscous friction ≈ velocity² (this INDEX, above).
- A twitch-shaped tendon force arises from an exponential or impulsive CE force filtered by passive elements ([9.3.1](./9.3.1-emg-driven-model-example.md)).
- With a critically damped M–B–K model, B/M = 2/T and K/B = 1/2T, so the twitch time T fixes the dynamics; only an EMG gain remains ([9.3.1](./9.3.1-emg-driven-model-example.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [9.3.1 Example of a model—EMG driven](./9.3.1-emg-driven-model-example.md) | Rectified EMG → impulsive CE force → M–B–K → tendon force; parameters from T | building an activation filter, or driving a ragdoll from recorded muscle activity |

## Relevance to migera

migera's ragdoll PD (`src/math/pd.rs`) is itself a linear
spring (kp) and damper (kd) acting in parallel between target and body,
with a hard clamp. This section's vocabulary maps onto proposed additions: a
**series spring** between controller and body (compliance), **nonlinear
passive springs** at joint limits ($k(e^x - 1)$ is exactly the stiffening
shape of the parallel element), and an **active-state filter** on the
strength command. Eq. 9.4 is also a practical tool: if a spring-damper
network is easier to integrate stably in one layout, swap to it without
changing its response. None of this is modeled today.

## Where to read in the book

- p. 243 (PDF 256): model lineage, Fig. 9.16 element catalog, Eq. 9.4.
- p. 244 (PDF 257): Fig. 9.17 equivalent layouts and active-state model.
- pp. 244–247 (PDF 257–260): the EMG-driven example (9.3.1).
