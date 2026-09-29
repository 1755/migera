---
title: Radiance Cascades closed most of the gap to DDGI but DDGI stays the default
description: An 8-stage experiment added Radiance Cascades as a selectable GiMethod and A/B'd it against DDGI on gi_room; bounce, hemisphere gather and trilinear blend each helped, but DDGI stays brighter and smoother (Chebyshev visibility is the unbuilt lever). Read before resuming cascades or comparing GI methods.
type: decision
status: current
tags:
  - global-illumination
  - hybrid-renderer
  - performance
  - verification
updated: 2026-09-18
verified: 2026-09-28
code:
  - src/hybrid/radiance_cascades_ref.rs
  - assets/shaders/hybrid_radiance_cascades.wgsl
  - assets/shaders/hybrid_trace.wgsl
  - examples/gi_room.rs
sources:
  - Claude memory radiance_cascades_experiment_result (2026-09-18)
  - commits c02e290, c8eb0d8, 04dc658, e5c5c1f, e778f27, 4abb80e, 646a388
  - PROGRESS.md "Radiance Cascades experimental GI" Stages 1-8
aliases:
  - radiance cascades
  - Sannikov
  - GiMethod::RadianceCascades
  - cascade_sample_level_trilinear
---

# Radiance Cascades closed most of the gap to DDGI but DDGI stays the default

The user commissioned Radiance Cascades (Alexander Sannikov) as a second,
selectable GI technique (`GiMethod::RadianceCascades`) to A/B against DDGI on
`examples/gi_room.rs`. After eight stages, cascades is much improved but
**DDGI is still slightly brighter and smoother**, and DDGI remains the
shipping default. The remaining gap is attributed to a named, unbuilt
mechanism: DDGI's Chebyshev depth-visibility term.

## Context

Commits: `c02e290` (CPU reference), `c8eb0d8` (GPU pass), `04dc658`
(`gi_room` CLI wiring), `e5c5c1f` (first A/B, later corrected), `e778f27`
(correction), `4abb80e` (multi-bounce). Stages 6–8 landed in `646a388`
together with the sealed-room leak fixes.

## Decision and stages

- **Corrected finding (Stage 4).** Cascades' near-total darkness on `gi_room`
  was not a GPU bug. `relight_cascade_texel` had no indirect/bounce term (an
  explicit Stage 1 scope cut), while DDGI's light in that view is almost all
  indirect. The two were never comparable before Stage 5.
- **Stage 5 (`4abb80e`): multi-bounce.** Added `indirect_at_hit`-style
  recursion, mirroring `ddgi_ref.rs::probe_ray`. Cascades has no cross-frame
  accumulation, so one relight pass's bounce read depends on GPU scheduling
  order. Fixed with `RadianceCascadesConfig::bounce_passes`, re-dispatching the
  relight N times per frame (`gi_room.rs --cascade-bounces N`).
- **Stage 6: hemisphere gather.** Bounce light spread in one direction only
  (a "line" artifact). Both hierarchy-sample call sites passed the bare surface
  normal as a single-point `direction`: right for a view ray, wrong for a
  diffuse bounce, which needs a hemisphere integral. Added
  `cascade_cosine_weighted_hierarchy_at_hit` and
  `radiance_cascades_cosine_weighted_hierarchy`, mirroring DDGI's
  `ddgi_cosine_weighted_probe_irradiance` (5 cosine-weighted samples).
- **Stage 7: the real A/B** at `--corridor-cam --at-frame 900`, roof open in
  both. The corridor geometry was checked against `gi_room.rs` first (the roof
  gap exposes the whole -X half of the ceiling; all 7 cubes sit at
  `x ≈ -6.5..-6.8`). Cascades improved hugely (floor strip and cube faces went
  from black to visible coloured bounce), but DDGI was still brighter, most
  clearly on one large shadowed cube face. Judged visually only; no pixel-diff
  tool was on the dev shell's `PATH`.
- **Stage 8: trilinear spatial blend, CPU reference first.** Stage 7's leading
  explanation was that cascades used nearest-probe lookup while DDGI blends 8
  probes. Added `cascade_probe_grid_cell`/`cascade_sample_level_trilinear` to
  `radiance_cascades_ref.rs` (4 new tests, 21/21 in the file), then ported to
  both WGSL call sites. The shadowed face now shows a dark-purple bounce tint.
  Cost: about 2.7x relight time on `gi_room` (8.87 ms vs ~3.3 ms) and more
  visible grain.

## Alternatives considered

- **Add DDGI's Chebyshev visibility term to cascades** (`hybrid_ddgi_relight.wgsl`
  around lines 936–947 at the time). Explicitly not built without first seeing
  a concrete artifact that traces to its absence.

## Consequences

- DDGI stays the shipping default regardless. Cascades is a scoped,
  non-shipping experiment and a reasonable stopping point.
- Radiance Cascades still uses the raw, unshrunk scene bounds for its grid; the
  wall-embedding fix was applied to DDGI only.
- DDGI itself had a sealed-room leak during Stages 5–8, found only when the
  user said DDGI looked worse than cascades. The comparison was partly against
  a broken baseline.

## Revisit when

- A concrete cascades artifact traces to missing depth visibility, or DDGI's
  per-probe occlusion cost becomes the thing to beat.
- Resume from `646a388` and PROGRESS.md's Stage 5–8 entries; do not re-derive
  them.

## Related
- [DDGI sealed-room light leak](./ddgi-sealed-room-light-leak.md) — deeper: the DDGI bug found while re-examining this comparison.
- [DDGI probe-grid bounds wall-embedding leak](./ddgi-probe-grid-bounds-wall-embedding-leak.md) — contrast: the bounds fix applied to DDGI but not to cascades.
- [A measurement of a broken system](../../engineering-practice/measurement/a-measurement-of-a-broken-system.md) — same-trap: Stage 4 compared a feature gap, and later stages compared against a leaking DDGI.
- [A/B test a feature on the same input](../../engineering-practice/measurement/ab-test-on-the-same-input.md) — applies: the matched-camera, matched-frame protocol of Stage 7.
- [Temporal accumulation fixes indirect-diffuse banding](./hybrid-gi-temporal-accumulation.md) — contrast: the cross-frame accumulation cascades lacks.
