---
title: 9.1 Force-length characteristics of muscles
description: "Muscle force vs. length: the active bell curve from cross-bridge overlap, the passive parallel element that stiffens past rest length, the series elastic element and internal shortening, and the one clean in vivo curve (soleus). Read before making ragdoll strength angle-dependent or adding soft joint limits."
type: index
status: current
tags:
  - biomechanics
  - muscle
  - ragdoll
  - springs
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §9.1, pp. 231–236 (PDF pp. 244–249)"
---

# 9.1 Force-length characteristics of muscles

> **Source:** Winter (2009) §9.1, pp. 231–236 ·
> [open PDF at p. 231](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=244) ·
> Up: [Chapter 9 — Muscle mechanics](../INDEX.md)

A muscle's isometric force depends on its length through two elements: the
**active contractile element**, whose force peaks at resting length and falls
on both sides, and **passive connective tissue**, part of it in parallel
(slack below rest, stiffening steeply beyond) and part in series (the tendon,
which stretches under load). The section intro states just this: net
force-length is the combination of active and passive characteristics.
9.1.1 and 9.1.2 build the total curve and how activation scales it; 9.1.3
covers the series element, which matters for dynamics rather than the static
curve; 9.1.4 shows how hard it is to see any of this in a living human.

## Key facts

- Active force peaks at sarcomere ≈ 2.5 µm (l0), is zero at ≈ 4.0 µm and strongly reduced but nonzero at ≈ 1.5 µm ([9.1.1](./9.1.1-contractile-element-force-length-curve.md)).
- Passive parallel force is zero at l ≤ l0 and rises nonlinearly beyond; tendon force $F_t = F_c + F_p$ ([9.1.2](./9.1.2-parallel-connective-tissue.md)).
- Activation scales only the active part; passive force is not under voluntary control ([9.1.2](./9.1.2-parallel-connective-tissue.md)).
- Series elastic stretch = internal CE shortening, a few percent (up to 7%) of rest length at max tension ([9.1.3](./9.1.3-series-elastic-tissue.md)).
- Series elastic energy storage is too small to explain the prestretch benefit (van Ingen Schenau, 1984) ([9.1.3](./9.1.3-series-elastic-tissue.md)).
- Soleus moment peaks at 15° dorsiflexion and falls roughly linearly to near zero at 30° plantarflexion ([9.1.4](./9.1.4-in-vivo-force-length.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [9.1.1 Force-length curve of the contractile element](./9.1.1-contractile-element-force-length-curve.md) | Bell-shaped active curve and its 1.5/2.5/4.0 µm anchors | making joint strength depend on angle |
| [9.1.2 Influence of parallel connective tissue](./9.1.2-parallel-connective-tissue.md) | Passive one-sided stiffening; $F_t = F_c + F_p$; activation family | adding soft, strength-independent joint limits |
| [9.1.3 Series elastic tissue](./9.1.3-series-elastic-tissue.md) | Tendon compliance, internal shortening, quick-release test | considering compliance between actuator and body |
| [9.1.4 In vivo force-length measures](./9.1.4-in-vivo-force-length.md) | Moment-angle confounds; the soleus curve | authoring a per-joint strength-vs-angle curve |

## Relevance to migera

migera's ragdoll joint strength (`default_joint_params` in
`src/character/anim/ragdoll.rs`) is a constant torque ceiling per joint, scaled
by `RagdollStrength`, with range enforced by hard avian joint limits. This
section suggests two independent additions, neither modeled today:

1. **Force-length → angle-dependent ceiling:** multiply the ceiling by a
   per-joint curve of angle, peaked mid-range, from moment-angle data like
   9.1.4.
2. **Parallel element → passive soft limits:** a torque that is zero in the
   normal range and grows exponentially near each limit, applied *outside* the
   strength dial so a limp joint still stiffens at end of range.

The series element (9.1.3) is lower priority.

## Where to read in the book

- pp. 231–232 (PDF 244–245): Fig. 9.7 active curve.
- p. 233 (PDF 246): Figs. 9.8 and 9.9, total and activation-scaled curves — the
  most useful figures of the section.
- pp. 233–235 (PDF 246–248): Figs. 9.10 and 9.11, series element.
- pp. 235–236 (PDF 248–249): in vivo measures.
