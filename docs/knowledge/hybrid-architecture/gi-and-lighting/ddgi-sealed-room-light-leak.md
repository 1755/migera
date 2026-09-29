---
title: DDGI's shading-time grid lookup used the X axis for all three axes
description: hybrid_trace.wgsl's ddgi_probe_grid_cell divided by origin_x/spacing_x on every axis (scalar broadcast over a vec3), so every shading-time DDGI sample read a wrong probe cell and a sealed gi_room rendered lit; fixed in 646a388. Read when DDGI looks subtly wrong, especially with uneven per-axis probe spacing.
type: lesson
status: current
tags:
  - global-illumination
  - correctness
  - wgsl
  - hybrid-renderer
updated: 2026-09-18
verified: 2026-09-28
code:
  - assets/shaders/hybrid_trace.wgsl
  - assets/shaders/hybrid_ddgi_relight.wgsl
  - src/hybrid/ddgi_ref.rs
sources:
  - Claude memory ddgi_sealed_room_light_leak_fix (2026-09-18)
  - commit 646a388
  - PROGRESS.md "DDGI: fixed a real sealed-room light leak" entry
aliases:
  - sealed-room leak
  - ddgi_probe_grid_cell
  - scalar broadcast bug
  - DdgiGridUniform
---

# DDGI's shading-time grid lookup used the X axis for all three axes

The first of four sealed-room light leaks fixed together in commit `646a388`.
`hybrid_trace.wgsl::ddgi_probe_grid_cell`, the **shading-time** probe-grid
lookup called from `shade()`'s `GI_METHOD_DDGI` branch, computed
`local = (world_pos - ddgi_grid.origin_x) / ddgi_grid.spacing_x`. WGSL
broadcasts a scalar across a `vec3`, so the X-axis origin and spacing were
used for **all three axes**. Every shading-time DDGI sample, which is what
actually paints pixels, read from a wrong cell whenever Y/Z spacing differs
from X, which is always.

## What happened

- On 2026-09-18, re-examining the DDGI vs Radiance Cascades comparison, the
  user reported that DDGI (the shipping default) looked worse than cascades:
  light in a fully sealed room, and "squared" artifacts.
- It was a port bug specific to `hybrid_trace.wgsl`'s `DdgiGridUniform`, which
  stores separate scalar `origin_x/y/z` fields. The relight pass's own copy in
  `hybrid_ddgi_relight.wgsl` uses a `vec3` uniform and was never affected.
- The caller recomputed the trilinear `frac` correctly per axis, so `cell` and
  `frac` disagreed with each other rather than being uniformly wrong.
- **Fix:** build real `vec3<f32>` origin and spacing from the per-axis scalar
  fields first, matching `ddgi_ref::probe_grid_cell` exactly.

**Dead ends, kept as regression tests:**
1. DDGI's self-referential "infinite bounce" loop. Ruled out by a 20-frame CPU
   simulation of the real relight + temporal blend from zero history, which
   stays exactly zero.
2. A thin-slab shadow-march divergence against the roof panel at grazing sun
   angles. Ruled out by CPU-reference tests. The first "confirmation" was a bug
   in the test itself: it passed the panel's own entity as `origin_entity`, so
   `trace_shadow` skipped it.

**A false alarm:** right after the fix, an A/B against a pre-fix screenshot
looked like a new regression (vivid purple/green cubes gone washed out). It was
leftover debug code from the investigation (`indirect = irradiance * 5.0`,
bypassing `diffuse_color`). Removing it reproduced the correct result exactly.

## Why it matters

- A scene whose per-axis spacings happen to be close (`gi_room`: `spacing_x =
  1.2`, `spacing_y ≈ 1.1`) masks this class of bug almost entirely away from
  cell boundaries. It took a fully sealed room to make it obvious.
- The CPU reference was right; one of two WGSL copies was ported wrong.

## How to apply

- If DDGI looks subtly wrong in a scene with very different per-axis spacing
  (a tall, narrow room), confirm this fix is in place and test a sealed or
  enclosed case.
- When one function exists as a CPU reference and several WGSL copies, check
  every copy against the reference.
- Remove temporary visualisation code before any A/B screenshot.

## Evidence

- Commit `646a388`; PROGRESS.md entry "DDGI: fixed a real sealed-room light
  leak".

## Related
- [Bounce GI unconditional leak](./bounce-gi-unconditional-leak.md) — deeper: the second leak, visible once this one was fixed.
- [Shadow margin / VIS_CUTOFF leak](./shadow-margin-vis-cutoff-leak.md) — deeper: the third leak in the same room.
- [DDGI probe-grid bounds wall-embedding leak](./ddgi-probe-grid-bounds-wall-embedding-leak.md) — deeper: the fourth and deepest leak.
- [Radiance Cascades experiment](./radiance-cascades-experiment.md) — prerequisite: the comparison that exposed this bug.
- [Grep other consumers before declaring a fix done](../../engineering-practice/debugging/grep-other-consumers-before-declaring-a-fix-done.md) — same-trap: duplicated code where one copy was wrong.
- [Verify, don't assert from memory](../../engineering-practice/debugging/verify-dont-assert-from-memory.md) — same-trap: the "regression" that was leftover debug code.
