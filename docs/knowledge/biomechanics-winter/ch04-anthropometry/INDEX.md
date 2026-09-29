---
title: Chapter 4 — Anthropometry
description: Winter's body-segment-parameter chapter — lengths per height (Fig. 4.1), the full Dempster Table 4.1 (mass, COM, radius of gyration), whole-body COM, parallel axis, a 14-segment COM model, and muscle PCA/stress/moment arms. Read before setting ragdoll masses, COMs, inertias or joint strengths.
type: index
status: current
tags:
  - biomechanics
  - anthropometry
  - ragdoll
  - muscle
  - balance
updated: 2026-09-28
verified: 2026-09-28
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), Chapter 4, pp. 82–106 (PDF pp. 95–119)"
---

# Chapter 4 — Anthropometry

> **Source:** Winter (2009) Ch. 4, pp. 82–106 ·
> [open PDF at p. 82](../../../books/bmcoh/BIOMECHANICS_AND_MOTOR_CONTROL_OF_HUMAN.pdf#page=95) ·
> Up: [Winter — Biomechanics and Motor Control of Human Movement](../INDEX.md)

**Chapter in one minute.** Every kinetic calculation in the book needs,
for each body segment, its length, mass, center-of-mass (COM) location
and moment of inertia. This chapter supplies them. Lengths come as
fractions of stature H (Fig. 4.1). Masses come as fractions of body mass,
and COMs and radii of gyration as fractions of segment length, all in the
Dempster-derived **Table 4.1**, which the rest of the book uses. The chapter
shows how to combine segments into a whole-body COM, how to move inertia
between axes (I = I₀ + mx²), and how to do both from marker data, including
Winter's own 21-marker, 14-segment balance model. It closes with the
muscle dimensions that turn a joint moment into muscle forces: PCA,
specific tension, moment arms and two-joint muscles.

**Start here:** for ragdoll work, read
[4.1.3 (Table 4.1 and the migera mapping)](./4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md)
first, then [4.1.7](./4.1-density-mass-inertial-properties/4.1.7-anthropometric-tables-with-kinematic-data.md).
For balance or COM work, read 4.1.4 and then 4.1.7.

## Key facts
- Thigh ≈ shank ≈ 0.245 H, upper arm 0.186 H, forearm 0.146 H, hip joint at 0.530 H ([4.0.1](./4.0-scope-and-segment-dimensions/4.0.1-segment-dimensions.md)).
- Table 4.1 mass fractions (pelvis 0.142, abdomen 0.139, thorax 0.216, head-neck 0.081, upper arm 0.028, forearm+hand 0.022, thigh 0.100, shank 0.0465, foot 0.0145) sum to exactly 1.000 over a 14-body split, and migera's ragdoll already uses them ([4.1.3](./4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md)).
- Limb COMs sit at ~0.43 L from the proximal joint, not 0.5; long-limb ρ₀ ≈ 0.30–0.32 L ([4.1.3](./4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md)).
- Table 4.1 passes a parallel-axis self-check except for one misprint (head-neck ρ_prox 0.116 → 1.116) ([4.1.3](./4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md)).
- Whole-body COM = Σ fᵢ·COMᵢ, but COM energy misses the energy of reciprocal limb motion ([4.1.4](./4.1-density-mass-inertial-properties/4.1.4-multisegment-center-of-mass.md)).
- I = I₀ + mx²; a locked leg about the hip ≈ 20× I₀ ([4.1.6](./4.1-density-mass-inertial-properties/4.1.6-parallel-axis-theorem.md)).
- A COM estimate must satisfy COP − COM = −(I/Wh)·COM̈; the pelvis is not a COM proxy in a hip strategy ([4.1.7](./4.1-density-mass-inertial-properties/4.1.7-anthropometric-tables-with-kinematic-data.md)).
- Skin-marker joint centers miss the true axes by centimetres; instantaneous-axis fits fail when |ω| < 0.5 rad/s ([4.2.4](./4.2-direct-experimental-measures/4.2.4-joint-axes-rotation.md)).
- Muscle force ≈ PCA × 20–100 N/cm²; ankle plantarflexors have ~10× the PCA of dorsiflexors ([4.3.1](./4.3-muscle-anthropometry/4.3.1-muscle-cross-sectional-area.md), [4.3.3](./4.3-muscle-anthropometry/4.3.3-muscle-stress.md)).
- Biarticular leg muscles are net extensors in stance and move energy between joints ([4.3.5](./4.3-muscle-anthropometry/4.3.5-multijoint-muscles.md)).

## Contents
| Note | What it establishes | Read when |
|---|---|---|
| [4.0 Scope of anthropometry in movement biomechanics](./4.0-scope-and-segment-dimensions/INDEX.md) | Which body measures movement analysis needs; Fig. 4.1 lengths per height | sizing or checking rig proportions |
| [4.1 Density, mass, and inertial properties](./4.1-density-mass-inertial-properties/INDEX.md) | Densities, **Table 4.1**, COM, I, parallel axis, worked examples, 14-segment model | setting ragdoll masses/COMs/inertias or computing a COM |
| [4.2 Direct experimental measures](./4.2-direct-experimental-measures/INDEX.md) | Balance board, quick release, instantaneous joint axes | measuring a COM, inertia or pivot rather than assuming it |
| [4.3 Muscle anthropometry](./4.3-muscle-anthropometry/INDEX.md) | PCA, stress, moment arms, biarticular muscles | setting joint strength ratios or coupling leg joints |
| [4.4 Problems based on anthropometric data](./4.4-problems-anthropometric-data.md) | Problem sets with printed answers | writing COM/inertia test fixtures |
| [4.5 References](./4.5-references.md) | Sources behind the chapter's numbers | tracing a value to Dempster, Drillis & Contini or Winter 1998 |

## Relevance to migera

This is the most directly usable chapter for `src/character/anim`'s
14-body active ragdoll. Verified on 2026-09-28 by reading the code:

- **Masses already follow Table 4.1.** `default_body_masses()` in
  `src/character/anim/ragdoll_plugin.rs` uses the Dempster fractions for a
  70 kg body. `density_for` back-solves a capsule density to hit them.
- **COMs and inertias do not follow it.** Each body is a uniform capsule
  centred on its segment midpoint, sized with radius 0.22 L capped at
  0.09 m, so avian puts the COM at 0.5 L with ρ₀ ≈ 0.38–0.39 L. Table 4.1
  says 0.43 L and 0.30–0.32 L for long limb segments, and 0.682 L for
  forearm-and-hand. avian 0.7's `CenterOfMass`/`AngularInertia`
  components could impose the table values. The ragdoll's PD loop is
  acceleration-shaped, so this changes hit response and constraint
  sharing, not tracking ([details](./4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md#relevance-to-migera)).
- **Joint strength** (`default_joint_params` in `ragdoll.rs`) is
  hand-authored, symmetric and angle-independent. §4.3 gives the real
  ratios, the asymmetries (plantar- vs dorsiflexion) and the angle
  dependence (knee moment arms peak near 45°) if that ever needs to be
  more human.
- **Balance and pelvis:** Σ fᵢ·COMᵢ over rig joints is a cheap whole-body
  COM for pelvis/stance logic and for a future balance controller.
  COP − COM = −K·COM̈ is a physics test oracle.
- The §4.4 problems are ready numeric fixtures for inertia and COM helpers.

## Where to read in the book
- p. 83 (PDF 96): **Fig. 4.1** segment lengths per height.
- p. 86 (PDF 99): **Table 4.1**, a landscape page; render and rotate it.
- pp. 87–91 (PDF 100–104): COM, inertia, parallel-axis equations.
- p. 94 (PDF 107): **Fig. 4.6**, the 14-segment COM model.
- p. 101 (PDF 114): **Tables 4.3–4.4**, muscle PCA.
- p. 104 (PDF 117): **Fig. 4.10**, biarticular moment arms.

## See also
- [5.1 Basic link-segment equations — the free-body diagram](../ch05-kinetics-forces-and-moments/5.1-link-segment-equations-free-body-diagram.md) — the next chapter consumes every parameter defined here.
- [Ragdoll and physics](../../character-animation/ragdoll-and-physics/INDEX.md) — migera's ragdoll notes, where these parameters apply.
