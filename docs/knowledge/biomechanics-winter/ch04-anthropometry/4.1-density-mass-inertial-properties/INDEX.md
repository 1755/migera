---
title: 4.1 Density, mass, and inertial properties
description: The body-segment-parameter core of the book — densities, Table 4.1 segment mass/COM/radius-of-gyration fractions, whole-body COM, I = mρ², parallel-axis theorem, worked examples and Winter's 14-segment COM model. Read before setting ragdoll masses/COMs/inertias or computing a character's COM.
type: index
status: current
tags:
  - biomechanics
  - anthropometry
  - ragdoll
  - physics
  - balance
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §4.1, pp. 83–96 (PDF pp. 96–109)"
---

# 4.1 Density, mass, and inertial properties

> **Source:** Winter (2009) §4.1, pp. 83–96 ·
> [open PDF at p. 83](../../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=96) ·
> Up: [Chapter 4 — Anthropometry](../INDEX.md)

Kinetic analysis needs, per segment, a mass, a COM location and a moment of
inertia. The section's short intro notes three ways these were obtained:
cadaver dissection, measured segment volumes times density tables, and
(more recently) cross-sectional scanning along the segment. The section
then builds up from density (4.1.1–4.1.2) to the master
table of segment parameters (4.1.3, Table 4.1), combines segments into a
whole-body COM (4.1.4), defines inertia and its tabulated form, the radius
of gyration (4.1.5), shows how to move inertia between axes (4.1.6), and
works everything through with real marker data (4.1.7).

## Key facts
- Segment mass = fixed fraction × body mass: thigh 0.100, shank 0.0465, foot 0.0145, upper arm 0.028, forearm+hand 0.022, head-neck 0.081, thorax 0.216, abdomen 0.139, pelvis 0.142 — these sum to 1.000 ([4.1.3](./4.1.3-segment-mass-and-center-of-mass.md)).
- Limb COMs sit ~0.43 L from the proximal joint, not at the midpoint; forearm+hand at 0.682 L ([4.1.3](./4.1.3-segment-mass-and-center-of-mass.md)).
- Table 4.1's head-neck "ρ about proximal = 0.116" is a misprint for 1.116, exposed by the parallel-axis check ([4.1.3](./4.1.3-segment-mass-and-center-of-mass.md)).
- Whole-body COM = Σ fᵢ·COMᵢ; body mass cancels ([4.1.7](./4.1.7-anthropometric-tables-with-kinematic-data.md)).
- COM cannot measure body energy: reciprocal limb motions cancel in it ([4.1.4](./4.1.4-multisegment-center-of-mass.md)).
- I₀ = m ρ₀²; long limb segments have ρ₀ ≈ 0.30–0.32 L, close to a thin rod's 0.289 L ([4.1.5](./4.1.5-moment-of-inertia-and-radius-of-gyration.md)).
- I = I₀ + m x²; a locked leg about the hip is ~20× its centroidal inertia ([4.1.6](./4.1.6-parallel-axis-theorem.md)).
- Winter's 21-marker, 14-segment model needs four trunk segments because the trunk's mass shifts internally ([4.1.7](./4.1.7-anthropometric-tables-with-kinematic-data.md)).
- Any COM estimate must satisfy COP − COM = −(I/Wh)·COM̈ ([4.1.7](./4.1.7-anthropometric-tables-with-kinematic-data.md)).
- Body density ≈ 1.06 kg/l = 0.69 + 0.9·h/m^(1/3) ([4.1.1](./4.1.1-whole-body-density.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [4.1.1 Whole-body density](./4.1.1-whole-body-density.md) | Density from the ponderal index, Eqs. 4.1–4.2 | converting volume to mass or choosing a physical density |
| [4.1.2 Segment densities](./4.1.2-segment-densities.md) | Fig. 4.2: segment density vs body density; distal segments are denser | a per-limb density is needed |
| [4.1.3 Segment mass and center of mass](./4.1.3-segment-mass-and-center-of-mass.md) | **Complete Table 4.1**, COM integral Eqs. 4.3–4.5, migera ragdoll mapping | setting or checking any ragdoll body's mass, COM or inertia |
| [4.1.4 Center of mass of a multisegment system](./4.1.4-multisegment-center-of-mass.md) | Whole-body COM Eqs. 4.6–4.7; COM is no energy measure | writing COM code for balance or jumps |
| [4.1.5 Mass moment of inertia and radius of gyration](./4.1.5-moment-of-inertia-and-radius-of-gyration.md) | I = Σmx², I₀ = mρ₀² | turning ρ/L into a body inertia |
| [4.1.6 Parallel-axis theorem](./4.1.6-parallel-axis-theorem.md) | I = I₀ + mx², Example 4.3 | moving inertia to a joint or merging segments |
| [4.1.7 Use of anthropometric tables and kinematic data](./4.1.7-anthropometric-tables-with-kinematic-data.md) | Worked masses/COMs/inertias, symmetric-gait COM, 14-segment model, COP−COM check | computing whole-body COM from a pose, or verifying a COM estimate |

## Relevance to migera

This section is the physical-parameter source for the active ragdoll.
Verified on 2026-09-28: `default_body_masses()` in
`src/character/anim/ragdoll_plugin.rs` already uses the Table 4.1 mass
fractions for all 14 bodies. The COM (capsule midpoint, 0.5 L) and the
inertia (uniform capsule, ρ₀ ≈ 0.38–0.39 L) are not from the table. The
table would move limb COMs toward the proximal joint (0.43 L; 0.68 L for
forearm+hand) and cut the long-limb transverse inertia by roughly a third.
It could be applied through avian 0.7's `CenterOfMass`/`AngularInertia`
components ([details](./4.1.3-segment-mass-and-center-of-mass.md#relevance-to-migera)).
The same fractions give a cheap whole-body COM of the animated pose for
pelvis and balance work ([4.1.7](./4.1.7-anthropometric-tables-with-kinematic-data.md)).

## Where to read in the book
- pp. 83–85 (PDF 96–98): densities, Fig. 4.2.
- **p. 86 (PDF 99): Table 4.1** (landscape; render and rotate).
- pp. 87–91 (PDF 100–104): COM, inertia, radius of gyration, parallel-axis theorem, Examples 4.2–4.3.
- pp. 91–95 (PDF 104–108): Examples 4.4–4.7, Table 4.2, **Fig. 4.6** (14-segment model).
