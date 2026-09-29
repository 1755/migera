---
title: Hybrid renderer plans
description: Design documents written for the hybrid renderer before or during the rewrite — analytic SDF normals (built, measured, removed), the three-tier SDF/analytic/3DGS hardening survey, and tier-transition popping. None is a live plan. Read before re-proposing any of these ideas.
type: index
status: current
tags:
  - hybrid-renderer
  - lod
  - raymarching
updated: 2026-09-28
---

# Hybrid renderer plans

Plans and design research that fed the hybrid renderer. **None of them is a
live plan today.** The three-tier (SDF near / analytic mid / 3DGS far)
architecture they assume was not rebuilt in `src/hybrid`, and since
2026-09-21 the hybrid renderer is no longer the character-rendering target
(see [the pivot decision](../migera-pivot-to-bevy-pbr-for-characters.md)).
They are kept so the reasoning and the measured outcome aren't re-derived.

| Note | What it establishes | Read when |
|---|---|---|
| [Hardening research for the tiered hybrid renderer](./hybrid-renderer-hardening.md) | **Stale.** SOTA pitfalls for a three-tier renderer; the SDF stepping (over-relaxation, Lipschitz pruning), WGSL `fma`, storage-texture and BVH-refit findings still apply; tier sections are moot. | Before optimizing the SDF march or BVH, or writing read-write storage textures in WGSL. |

## Archived / superseded

| Note | What it establishes | Read when |
|---|---|---|
| [Analytical SDF gradients plan](./analytical-sdf-gradients.md) | Built in `src/hybrid`, measured equal to 6-tap central differences at `--stress 10000`, removed. The gradient formulas remain correct. | Before proposing analytic normals again, or when you need an `sdg*` formula. |
| [Tier-transition popping](./tier-transition-popping.md) | Error-model tier selection and hysteresis for tiers that were never built; its smin facts and GI plan outlived it. | Only if a multi-representation LOD scheme comes back. |
