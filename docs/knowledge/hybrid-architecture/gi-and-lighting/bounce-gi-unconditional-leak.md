---
title: Reflection and refraction bounces ran cone-traced GI even under GiMethod::None
description: shade_for_reflection_bounce / shade_for_refraction_bounce always called cone_trace_indirect_single, so --gi-method none still leaked light around the glass cube in a sealed room; fixed with a bounce_gi_enabled gate in CPU reference and WGSL (646a388). Read when light leaks near reflective or transmissive objects.
type: lesson
status: current
tags:
  - global-illumination
  - correctness
  - hybrid-renderer
  - lighting
updated: 2026-09-18
verified: 2026-09-28
code:
  - src/hybrid/reflect_ref.rs
  - src/hybrid/refract_ref.rs
  - src/hybrid/cpu_ref.rs
  - assets/shaders/hybrid_trace.wgsl
sources:
  - Claude memory bounce_gi_unconditional_leak_fix (2026-09-18)
  - commit 646a388
  - PROGRESS.md "--gi-method none: reflection/refraction bounce shading" entry
aliases:
  - bounce_gi_enabled
  - cone_trace_indirect_single
  - glass cube leak
---

# Reflection and refraction bounces ran cone-traced GI even under GiMethod::None

The second of four sealed-room leaks fixed in `646a388`. The bounce-shading
functions `shade_for_reflection_bounce` and `shade_for_refraction_bounce`
(in `hybrid_trace.wgsl` and their CPU twins in `reflect_ref.rs` and
`refract_ref.rs`) call `cone_trace_indirect_single`, a one-bounce cone-traced
diffuse "final gather" at the bounce's hit point. That call ran
**unconditionally**, with no check of the scene's primary `GiMethod`, so
`--gi-method none` still produced light.

## What happened

- After the first DDGI fix, the user asked why `--gi-method none` in the sealed,
  sun-only `gi_room` still showed faint lit patches on cube silhouettes.
  `none` disables every primary GI technique, so this was a separate leak.
- Calling `cone_trace_indirect_single` instead of recursing into `shade()` was
  a correct design choice (it avoids unbounded recursion into the
  `GiMethod::ConeTrace` branch). The doc comments justified *which* mechanism
  to use, never *whether* it should run under `GiMethod::None`.
- **Isolation:** temporary `--no-reflect`/`--no-transmission` toggles in
  `gi_room.rs` and a throwaway PNG region inspector (`examples/pixel_probe.rs`),
  both removed afterwards. `--no-reflect` alone changed nothing.
  `--no-transmission` alone dropped the region's max RGB from 77 to 52, which
  pointed at the glass cube (`transmission = 1.0, reflectance = 0.9, ior = 1.5`)
  and its refraction bounce.
- **Fix:** a `bounce_gi_enabled: bool` field on `cpu_ref::ReflectionParams` and
  `TransmissionParams`, threaded through `reflect_trace_ray`/`refract_trace_ray`
  into the bounce-shading functions, gating the call. Real callers pass
  `gi_method != GiMethod::None`. In WGSL a direct
  `if (scene.gi_method != GI_METHOD_NONE)` at each call site sufficed.

## Why it matters

A design comment that explains *how* something runs can hide the missing
question of *when* it should run.

## How to apply

- If a sealed or dark scene leaks light around a reflective or transmissive
  object under `GiMethod::None`, check this gate at both the CPU-reference and
  WGSL call sites first.
- Isolate a leak with per-feature toggles on the same frame, not by changing
  scenes.

## Evidence

- Regression tests
  `reflect_ref::bounce_gi_enabled_false_strictly_reduces_energy_when_real_gi_is_available`
  and the identically named `refract_ref` test. Each proves the flag gates real
  energy, using an emissive fixture.
- Live: in the flagged region, average RGB went from `[12.58, 13.04, 11.64]` to
  `[7.14, 7.90, 6.87]`, and the peak lost its warm colour cast (`[77,76,70]` to
  a uniform `[60,60,60]`, pure film grain). Identical with reflection and
  transmission both forced off. `gallery.rs --gi-method ddgi` showed no
  regression.
- **Known, not fixed:** a residual max ≈ 60 grain floor on near-black pixels.
  `hybrid_post.wgsl`'s `signal_factor = clamp(1.0 - luminance, 0.15, 1.0)` gives
  the most grain to the darkest pixels. Cosmetic, not a leak.

## Related
- [DDGI sealed-room light leak](./ddgi-sealed-room-light-leak.md) — prerequisite: the first leak, whose fix revealed this one.
- [Shadow margin / VIS_CUTOFF leak](./shadow-margin-vis-cutoff-leak.md) — deeper: the third leak, left after this fix.
- [A/B test a feature on the same input](../../engineering-practice/measurement/ab-test-on-the-same-input.md) — same-trap: the per-feature toggles that isolated this leak are the rule in practice.
