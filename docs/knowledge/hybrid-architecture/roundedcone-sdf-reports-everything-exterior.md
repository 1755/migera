---
title: RoundedCone's SDF reports every point as exterior
description: sdf::primitives::RoundedCone::distance and raymarch.wgsl's sdf_rounded_cone return positive distance everywhere, even on the centreline; the third max term dominates. src/hybrid skips RoundedCone with a pinned unimplemented! instead of porting it. Read before using, porting or fixing RoundedCone.
type: lesson
status: current
tags:
  - sdf
  - correctness
  - primitives
  - hybrid-renderer
updated: 2026-09-06
verified: 2026-09-28
code:
  - src/sdf/primitives.rs
  - assets/shaders/raymarch.wgsl
  - src/hybrid/cpu_ref.rs
  - src/hybrid/extract.rs
sources:
  - Claude memory roundedcone_sdf_bug (2026-09-06)
  - PROGRESS.md "Six more Shape primitives ported" entry
aliases:
  - sdRoundCone
  - rounded cone bug
  - sdf_rounded_cone
---

# RoundedCone's SDF reports every point as exterior

`sdf::primitives::RoundedCone::distance` (`src/sdf/primitives.rs`, the
3-max-term `sdRoundCone`-style formula) and its WGSL twin `sdf_rounded_cone`
(`assets/shaders/raymarch.wgsl`) have a confirmed correctness bug: **every
point reports as exterior** (positive distance), including the shape's own
centreline and both endpoints. That cannot be right for a valid SDF.

## What happened

- Found while porting `Shape` primitives into `src/hybrid`.
- **Not a transcription error.** The formula was diffed byte for byte against
  the original, and the intermediate terms (`d`, `e`, `f`, `g`, `h`,
  `clamped`, `t`, `q`, and the three `max`-ed terms) were hand-computed outside
  the codebase. The third term, `(p - b).length() - r1`, dominates the `max`
  far more often than it should and pushes the result positive deep inside the
  solid.
- The user chose "skip RoundedCone for now" over fixing it or porting it as is.
  Fixing shipped code was out of scope for the porting task.

## Why it matters

Plausible-looking SDF code that compiles can still be wrong everywhere.
Porting it into a second renderer would have spread the bug.

## How to apply

- Before using or porting `RoundedCone` anywhere, fix the formula first.
  Likely route: swap in an exact `sdRoundCone` variant, structurally like the
  correct-looking `sdg_rounded_cone` gradient function in `raymarch.wgsl`
  (which uses a different `a2`/`k`/branching approach, not three `max` terms).
- Verify any SDF against real point samples (centreline, endpoints, points
  just inside and outside), the way this bug was found.

## Evidence

- `src/hybrid/cpu_ref.rs`: `local_distance` has
  `unimplemented!("local_distance: RoundedCone is skipped ...")`, pinned by the
  `#[should_panic]` test `rounded_cone_is_explicitly_unimplemented_not_silently_wrong`.
- `src/hybrid/extract.rs` maps `Shape::RoundedCone` to no GPU tag, and
  `hybrid_trace.wgsl` has no case for it. Still true on 2026-09-28.

## Related
- [Primitive shapes](../sdf-3d/primitives-and-operators/primitive-shapes.md) — deeper: the reference catalog of exact primitive SDFs.
- [Exact vs. bound SDFs](../sdf-3d/fundamentals/exact-vs-bound-sdfs.md) — prerequisite: what a valid distance field must satisfy, which this one violates.
- [Analytical SDF gradients plan](./plans/analytical-sdf-gradients.md) — example: `RoundedCone` was also excluded from the analytic-normals comparison.
