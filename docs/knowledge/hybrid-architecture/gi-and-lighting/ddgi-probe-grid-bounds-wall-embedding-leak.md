---
title: DDGI's probe grid was built from wall-inclusive bounds, placing probes inside the walls
description: extract_hybrid_scene fed the unshrunk root BVH AABB (including the walls' outer faces) to probe_grid_from_bounds, so corner probes sat on or past the wall and saw sunlight from the first relight; fixed by shrinking by DDGI_GRID_WALL_SAFETY_MARGIN + spacing (646a388). Read when DDGI leaks near thin enclosing geometry.
type: lesson
status: current
tags:
  - global-illumination
  - correctness
  - hybrid-renderer
  - bounding-volumes
updated: 2026-09-19
verified: 2026-09-28
code:
  - src/hybrid/extract.rs
  - src/hybrid/ddgi_ref.rs
  - examples/gi_room.rs
sources:
  - Claude memory ddgi_probe_grid_bounds_wall_embedding_leak_fix (2026-09-19)
  - commit 646a388
  - PROGRESS.md "DDGI: a fourth, deeper sealed-room light leak" entry
aliases:
  - probe_grid_from_bounds
  - DDGI_GRID_WALL_SAFETY_MARGIN
  - probe inside wall
  - probe grid bounds
---

# DDGI's probe grid was built from wall-inclusive bounds, placing probes inside the walls

The fourth and deepest sealed-room leak fixed in `646a388`. The CPU tests
hand-shrank the probe-grid bounds before calling `probe_grid_from_bounds`, but
the real pipeline (`extract_hybrid_scene` in `src/hybrid/extract.rs`) passed
the scene's **unshrunk** root BVH AABB. That box includes the outer faces of
`gi_room`'s wall shell, so outer-layer and corner probes landed on or beyond
the walls, outside the sealed cavity, and picked up direct sunlight.

## What happened

- After the first three fixes, the user still saw a structured, coloured leak
  on the real GPU: a purple halo around the purple cube and glowing ceiling
  patches.
- **The reframing that broke the case:** capture at `--at-frame 20` with
  `probes_per_frame: 999999` (full-grid relight) and `max_history_length: 1.0`
  (no history mixing). The leak was already fully present on the **first**
  relight from a zero atlas. That ruled out every multi-frame explanation
  (temporal EMA, rotation aliasing, cross-probe feedback) at once.
- The CPU reference
  (`sealed_room_with_real_cube_furniture_at_production_ddgi_density_stays_dark`)
  reported exactly `0.0` for the same scenario, and two line-by-line
  WGSL-vs-CPU audits had found no formula divergence. So the bug had to be in
  the **inputs** fed to the math, not the math.
- **Root cause:** the walls extend outward from the interior by
  `WALL_THICKNESS (0.3) + WALL_OVERLAP (0.2)`. The real root AABB is
  `(±8.6, ±3.6, ±7.1)`, not the interior `(±8.0, ±3.0, ±6.5)` the CPU tests
  approximated by hand. `probe_grid_from_bounds`' half-cell inset
  (`origin = bounds.min + spacing/2`) only centres probe 0 in its cell. At
  `probe_spacing = 1.2` that inset (0.6) nearly equals the wall thickness, so
  the outer layer sat almost on the interior wall plane and a corner probe sat
  at `(8.8, -3.0, -6.5)`, outside the room. Peak irradiance there: 0.435.
- **Compounding effect:** `dims_x`/`dims_z` use `ceil(extent / spacing)`, so the
  grid overshoots the input extent by up to one spacing, all on the max side
  (origin is pinned to `bounds.min` by a tested contract). A margin sized only
  for wall thickness (0.75 alone) was eaten on the max side: the max-X probe
  still sat at 8.35, past the interior wall at 8.0.
- **Fix:** `extract_hybrid_scene` shrinks `root_bounds` inward by
  `DDGI_GRID_WALL_SAFETY_MARGIN (0.75) + probe_spacing` on every axis before
  calling `probe_grid_from_bounds`, and skips the shrink if it would invert
  min/max in a tiny scene. DDGI only; Radiance Cascades still uses raw bounds.

## Why it matters

- Tests that hand-approximate the real pipeline's inputs pass while the real
  pipeline is wrong. Two formula audits could not find a bug that was in the
  data.
- Checking the first frame from zero state removes whole families of
  hypotheses in one capture.

## How to apply

- If a scene leaks DDGI light near thin enclosing geometry, check that
  `DDGI_GRID_WALL_SAFETY_MARGIN + probe_spacing` still exceeds that scene's
  real wall or shell thickness. The margin was sized against `gi_room`'s worst
  case (0.5), not as a universal constant.
- When CPU tests pass and the GPU is wrong, compare the *inputs* each receives.

## Evidence

- Tests in `src/hybrid/ddgi_ref.rs`:
  `probe_grid_from_unshrunk_real_room_bvh_bounds_leaks_light_from_frame_one`
  (the bug reproduces with real bounds, peak > 0.1) and
  `probe_grid_from_extract_rs_shrink_formula_stays_dark` (the real shrink
  formula, peak < 0.01).
- Live, `gi_room --at-frame 60`, all `DdgiConfig` defaults: before the fix
  `ddgi` max RGB `[86,86,84]`, about 4x `none`'s average brightness, worst
  pixel `sum_abs_diff = 235`. After: `ddgi` and `none` are **bit-for-bit
  identical** across the 1280×720 frame. `cargo test --release --lib`:
  363/363.

## Related
- [DDGI sealed-room light leak](./ddgi-sealed-room-light-leak.md) — prerequisite: the first of the four leaks in this investigation.
- [Shadow margin / VIS_CUTOFF leak](./shadow-margin-vis-cutoff-leak.md) — prerequisite: the third leak, fixed just before this one.
- [Radiance Cascades experiment](./radiance-cascades-experiment.md) — contrast: cascades still uses raw bounds.
- [Replicate the real frame loop in a unit test](../../engineering-practice/testing/replicate-the-real-frame-loop-in-a-unit-test.md) — same-trap: tests that model the real system's inputs by hand.
- [Parse the asset, don't transcribe it](../../engineering-practice/testing/parse-the-asset-dont-transcribe-it.md) — same-trap: hand-approximated test data drifting from the real data.
- [The symptom is far from the cause in the rig chain](../../engineering-practice/debugging/symptom-is-far-from-cause-in-the-rig-chain.md) — same-trap: correct downstream math reacting to bad input.
- [A/B test a feature on the same input](../../engineering-practice/measurement/ab-test-on-the-same-input.md) — example: `none` vs `ddgi` on the same frame as the acceptance test.
