---
title: 9.2 Force-velocity characteristics
description: "Muscle force vs. velocity: Hill's hyperbola for shortening, the 1.1–1.8 Fmax eccentric plateau for lengthening, the length-velocity force surface, and muscle–load equilibrium incl. the walking ankle's moment-velocity loop. Read before making ragdoll torque ceilings velocity-dependent."
type: index
status: current
tags:
  - biomechanics
  - muscle
  - ragdoll
  - math
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §9.2, pp. 236–243 (PDF pp. 249–256)"
---

# 9.2 Force-velocity characteristics

> **Source:** Winter (2009) §9.2, pp. 236–243 ·
> [open PDF at p. 236](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=249) ·
> Up: [Chapter 9 — Muscle mechanics](../INDEX.md)

Movement requires length change, and a muscle's force depends strongly on how
fast its length changes: it weakens hyperbolically when shortening (Hill) and
strengthens, up to a plateau, when forcibly lengthened. The section's intro
makes the point that isometric data — where most physiology is done — are only
the zero-velocity special case, since every movement alternates shortening and
lengthening. 9.2.1 and 9.2.2 give the two halves of the curve, 9.2.3 joins them
with force-length into one surface, and 9.2.4 puts the muscle against a load
and follows the operating point through a real walking stride.

## Key facts

- Hill: $(P + a)(V + b) = (P_0 + a)\,b$, $b = a V_0/P_0$; fits isotonic contractions near rest length ([9.2.1](./9.2.1-concentric-contractions.md)).
- Force loss with speed acts like a viscous damper (cross-bridge cycling + fluid viscosity) ([9.2.1](./9.2.1-concentric-contractions.md)).
- Vmax ≈ 6 l0/s in animals, > 10 l0/s estimated for human soleus ([9.2.1](./9.2.1-concentric-contractions.md)).
- Eccentric force plateaus at 1.1–1.8 Fmax; isovelocity stretch can drop instead ([9.2.2](./9.2.2-eccentric-contractions.md)).
- Level walking has equal positive and negative muscle work ([9.2.2](./9.2.2-eccentric-contractions.md)).
- Force is a surface over (length, velocity), one per activation level ([9.2.3](./9.2.3-length-and-velocity-vs-force.md)).
- The operating point is the muscle–load intersection; ankle stance goes negative, negative, then positive work ([9.2.4](./9.2.4-muscle-load-equilibrium.md)).

## Contents

| Note | What it establishes | Read when |
|---|---|---|
| [9.2.1 Concentric contractions](./9.2.1-concentric-contractions.md) | Fenn–Marsh and Hill equations; Vmax values | making torque fall with joint speed |
| [9.2.2 Eccentric contractions](./9.2.2-eccentric-contractions.md) | Lengthening force above Fmax; protocol dependence | deciding how hard a joint resists being forced or landing |
| [9.2.3 Combination of length and velocity versus force](./9.2.3-length-and-velocity-vs-force.md) | Force-length-velocity surface | combining angle and velocity factors in one ceiling |
| [9.2.4 Combining muscle characteristics with load characteristics: equilibrium](./9.2.4-muscle-load-equilibrium.md) | Operating point; spring/gravity examples; ankle moment-velocity loop in walking | predicting where a weakened joint settles; checking push-off work |

## Relevance to migera

The ragdoll's `pd_torque_at` (`src/character/anim/math/pd.rs`) clamps torque to
a velocity-independent `max_torque`. This section supplies the shape of a
better ceiling: fall off Hill-style when the joint already moves with the
torque (a natural speed limit and "weight"), rise to ~1.5× when the joint is
forced against it (firm braking, landings, shoves). Because only the ceiling
changes, the PD gains and their `kd·dt < 2` bound are untouched. None of this
is modeled today.

## Where to read in the book

- p. 237 (PDF 250): **Fig. 9.12**, the force-velocity family at 25–100%
  activation, both sides of zero — the key figure.
- p. 238 (PDF 251): Hill Eq. 9.3 and Vmax.
- p. 239 (PDF 252): Fig. 9.13 surface.
- pp. 240–242 (PDF 253–255): Fig. 9.14 equilibria, Fig. 9.15 ankle loop.
