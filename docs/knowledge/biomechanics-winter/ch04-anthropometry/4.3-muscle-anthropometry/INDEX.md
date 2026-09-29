---
title: 4.3 Muscle anthropometry
description: Muscle dimensions needed to split joint moments into muscle forces — PCA and pennation (Tables 4.3–4.4), length vs joint angle, specific tension 20–100 N/cm², angle-dependent moment arms, and biarticular muscles as energy transferers and stance extensors. Read when sizing joint strength ratios or coupling leg joints.
type: index
status: current
tags:
  - biomechanics
  - muscle
  - anthropometry
  - locomotion
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §4.3, pp. 100–104 (PDF pp. 113–117)"
---

# 4.3 Muscle anthropometry

> **Source:** Winter (2009) §4.3, pp. 100–104 ·
> [open PDF at p. 100](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=113) ·
> Up: [Chapter 4 — Anthropometry](../INDEX.md)

Computing individual muscle forces needs the muscles' own dimensions.
Muscles in a group probably share load in proportion to their
cross-sectional areas, and each has its own mechanical advantage, set by
its moment arms and by any structure that redirects its tendon. The section
gives the chain from muscle geometry to joint moment: PCA (4.3.1) × stress
(4.3.3) × moment arm (4.3.4). It adds how length follows joint angle
(4.3.2) and what two-joint muscles do (4.3.5).

## Key facts
- PCA = m·cosθ/(d·l), with d = 1.056 g/cm³; the soleus (58 cm²) is the largest PCA in Table 4.3 ([4.3.1](./4.3.1-muscle-cross-sectional-area.md)).
- Ankle plantarflexors hold ~91% of the PCA crossing the ankle against ~9% for dorsiflexors ([4.3.1](./4.3.1-muscle-cross-sectional-area.md)).
- Gastrocnemius length is nearly linear in ankle and knee angle: −8.5% to +4% over the ankle range ([4.3.2](./4.3.2-muscle-length-change-during-movement.md)).
- Specific tension is 20–100 N/cm²: ~70 N/cm² for quadriceps in running and jumping, ~100 N/cm² in isometric MVC ([4.3.3](./4.3.3-muscle-stress.md)).
- Knee muscle moment arms peak near 45° flexion ([4.3.4](./4.3.4-muscle-mechanical-advantage.md)).
- Biarticular leg muscles are net extensors in stance and feed the support moment ([4.3.5](./4.3.5-multijoint-muscles.md)).
- The book's text credits the hamstrings with an ankle moment arm; the figure shows it belongs to the gastrocnemius ([4.3.5](./4.3.5-multijoint-muscles.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [4.3.1 Cross-sectional area of muscles](./4.3.1-muscle-cross-sectional-area.md) | PCA Eqs. 4.17–4.18; Tables 4.3 and 4.4 in full | apportioning joint strength or setting per-joint ceiling ratios |
| [4.3.2 Change in muscle length during movement](./4.3.2-muscle-length-change-during-movement.md) | Gastrocnemius length vs ankle and knee angle | modelling a muscle's length from joint angles |
| [4.3.3 Force per unit cross-sectional area (stress)](./4.3.3-muscle-stress.md) | Specific tension values and their conditions | converting PCA to force or sanity-checking torque scale |
| [4.3.4 Mechanical advantage of muscle](./4.3.4-muscle-mechanical-advantage.md) | Moment arm definition; angle dependence | making joint strength angle-dependent |
| [4.3.5 Multijoint muscles](./4.3.5-multijoint-muscles.md) | Energy transfer; Fig. 4.10 moment arms; support moment | coupling hip, knee and ankle in a stance or swing controller |

## Relevance to migera

migera has no muscle model, and the `muscle` module was deleted. The
section still gives two things. First, **strength ratios and asymmetries**
for the ragdoll's per-joint PD ceilings in `src/character/anim/ragdoll.rs`,
which are hand-authored, symmetric and angle-independent today. Second,
the **stance-extensor synergy** as a design target for any future balance
or active-stance controller.

## Where to read in the book
- p. 100 (PDF 113): intro and PCA.
- p. 101 (PDF 114): Tables 4.3–4.4. Render the page; extraction scrambles Table 4.4.
- p. 102 (PDF 115): length change, stress, mechanical advantage.
- pp. 102–104 (PDF 115–117): multijoint muscles, Fig. 4.10.
