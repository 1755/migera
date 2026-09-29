---
title: Hybrid renderer GI and lighting
description: Global-illumination and lighting decisions and bugs in src/hybrid — temporal accumulation, the Radiance Cascades vs DDGI experiment, four sealed-room light leaks fixed in 646a388, and DDGI any-hit occlusion. Read when light leaks, GI looks wrong, or before changing DDGI, shadows or bounce shading.
type: index
status: current
tags:
  - global-illumination
  - shadows
  - hybrid-renderer
  - correctness
updated: 2026-09-28
---

# Hybrid renderer GI and lighting

DDGI is the shipping indirect-diffuse method; cone tracing and Radiance
Cascades remain selectable through `GiMethod`. The `gi_room` sealed-room
investigation (2026-09-18/19) found four independent leaks, each hidden behind
the previous one. They are the best worked examples in the project of per-feature
A/B toggles and of CPU-reference vs WGSL divergence.

**Reading order for a light leak:** the four leak notes in the order listed
below. Each fix revealed the next.

| Note | What it establishes | Read when |
|---|---|---|
| [Temporal accumulation fixes indirect-diffuse banding](./hybrid-gi-temporal-accumulation.md) | Banding was estimator variance; real reprojection + history (0b2c92b) beat more samples or a wider blur. | Before touching `hybrid_temporal.wgsl` or GI denoising. |
| [Radiance Cascades experiment](./radiance-cascades-experiment.md) | Eight stages closed most of the gap to DDGI; DDGI stays default; Chebyshev visibility is the unbuilt lever. | Before resuming cascades or comparing GI methods. |
| [DDGI grid lookup used X for all axes](./ddgi-sealed-room-light-leak.md) | Leak 1: a scalar-broadcast port bug in the shading-time probe-cell lookup. | When DDGI looks subtly wrong, especially with uneven probe spacing. |
| [Bounce GI ran under GiMethod::None](./bounce-gi-unconditional-leak.md) | Leak 2: reflection/refraction bounces always cone-traced GI; fixed with `bounce_gi_enabled`. | When light leaks near reflective or glass objects. |
| [trace_shadow VIS_CUTOFF early exit](./shadow-margin-vis-cutoff-leak.md) | Leak 3: `margin_fade` + `VIS_CUTOFF` returned a stale nonzero visibility; fixed in three copies. | Before changing `trace_shadow`, `margin_fade` or `VIS_CUTOFF`. |
| [DDGI probe grid built from wall-inclusive bounds](./ddgi-probe-grid-bounds-wall-embedding-leak.md) | Leak 4: probes sat inside/outside walls; shrink by `DDGI_GRID_WALL_SAFETY_MARGIN + spacing`. | When DDGI leaks near thin enclosing geometry. |
| [DDGI any-hit occlusion](./ddgi-any-hit-occlusion.md) | Occlusion rays are ~half the frame; `any_hit` fixed a leak but measured zero speed-up. | Before optimizing DDGI occlusion or adding early-out queries. |

## See also
- [Performance findings](../performance-findings/INDEX.md) — the perf investigation that DDGI any-hit continued.
- [Soft shadows and AO](../../sdf-3d/rendering/soft-shadows-and-ao.md) — the SDF soft-shadow estimator behind `trace_shadow`.
- [Debugging lessons](../../engineering-practice/debugging/INDEX.md) — the general rules these hunts illustrate.
