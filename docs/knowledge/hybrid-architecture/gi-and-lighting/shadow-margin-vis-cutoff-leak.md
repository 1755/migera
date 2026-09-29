---
title: trace_shadow could exit on VIS_CUTOFF with a stale nonzero visibility
description: After margin_fade forced a first sample near 1.0, the inner shadow-march loop exited on VIS_CUTOFF and returned vis = 0.0148 instead of a hard hit, leaking light at the glass cube's silhouette; fixed in cpu_ref.rs and both WGSL copies (646a388). Read before changing trace_shadow, margin_fade or VIS_CUTOFF.
type: lesson
status: current
tags:
  - shadows
  - correctness
  - raymarching
  - hybrid-renderer
updated: 2026-09-18
verified: 2026-09-28
code:
  - src/hybrid/cpu_ref.rs
  - assets/shaders/hybrid_trace.wgsl
  - assets/shaders/hybrid_ddgi_relight.wgsl
sources:
  - Claude memory shadow_margin_vis_cutoff_leak_fix (2026-09-18)
  - commit 646a388
  - PROGRESS.md "trace_shadow: a third sealed-room light leak" entry
aliases:
  - margin_fade
  - VIS_CUTOFF
  - shadow ray leak
  - soft shadow early exit
---

# trace_shadow could exit on VIS_CUTOFF with a stale nonzero visibility

The third of four sealed-room leaks fixed in `646a388`. It lives in
**direct-light shadow visibility**, not in any GI path. `trace_shadow`'s inner
march loop could exit on `vis > VIS_CUTOFF` one step before reaching the true
surface, returning `Soft { vis }` with a small leftover value instead of the
fully opaque result the rest of the function treats that threshold as meaning.

## What happened

- After the first two fixes, a faint, structured grey edge remained on the
  glass cube's silhouette under `--gi-method none`. It was identical with
  reflection and transmission both forced off, so it was in direct shadowing.
- **Mechanism:** the per-candidate march can spend its first sample against a
  thin, wide occluder (the roof panel) right at the padded candidate-AABB
  `margin` boundary. `margin_fade`, a correct fix for a different problem
  (a visible polygonal seam at the AABB entry), forces that sample's `vis`
  toward 1.0 by design. The second sample finds real occlusion
  (`vis ≈ 0.0148`, below `VIS_CUTOFF = 0.02`). The loop condition
  `while ... && vis > VIS_CUTOFF` is only re-checked at the top of the next
  iteration, so the loop exits before a third sample can register a `HardHit`.
  The outer per-candidate loop already treats `vis < VIS_CUTOFF` as opaque.
- **Fix:** inside the inner loop, right after `vis = vis.min(faded_vis)`, add
  `if vis <= VIS_CUTOFF { return Soft { vis: 0.0 } }`. Applied in three
  places: `cpu_ref.rs::trace_shadow`, `hybrid_trace.wgsl` and
  `hybrid_ddgi_relight.wgsl`.

## Why it matters

- **Grid sweeps over synthetic points do not replace the real query.** Two
  dense-grid sweep tests
  (`no_shadow_ray_near_the_sealed_roof_leaks_light_through_a_panel_seam` and
  its room-wide sibling) used fixed synthetic origins, and none landed on the
  margin-boundary geometry the real camera pixel's shadow query hit.
- Two correct fixes (`margin_fade`, the `VIS_CUTOFF` early-out) combined into
  a bug.

## How to apply

- Make a threshold mean the same thing everywhere in a function: if the outer
  loop treats "below cutoff" as opaque, the inner loop must too.
- Build a test from the actual geometry a failing pixel queries, not only from
  a sweep.
- Apply shadow-march fixes to all three copies.
- To see a sealed-roof pixel live, do not fight `--at-frame` against the
  variable 5–9 s GPU warm-up and `ROOF_DELAY_SECS = 5.0`. Use a fixed sleep, a
  temporary uncommitted `ROOF_DELAY_SECS` change, or a static scene.

## Evidence

- New test `shadow_ray_from_the_glass_cubes_own_silhouette_edge_is_a_hard_hit_not_a_soft_leak`
  (`src/hybrid/cpu_ref.rs`) with the `gi_room_sealed_shell_with_glass_cube`
  fixture. It fires from the real silhouette point `Vec3::new(1.205, -0.991, 1.0)`
  toward the real sun. It failed with `vis = 0.0148` (matching a hand
  prediction) before the fix and passes after.
- The two sweep tests now return exactly `vis = 0.0` everywhere; their
  `.unwrap()` on "worst violation found" became `if let Some(...)`.
- No live screenshot of the sealed state was captured (timing, see above). The
  CPU-reference test was treated as authoritative; `gallery.rs` soft shadows
  showed no regression.

## Related
- [Bounce GI unconditional leak](./bounce-gi-unconditional-leak.md) — prerequisite: the second leak, whose fix left this one visible.
- [DDGI probe-grid bounds wall-embedding leak](./ddgi-probe-grid-bounds-wall-embedding-leak.md) — deeper: the fourth leak, still present after this fix.
- [Soft shadows and AO](../../sdf-3d/rendering/soft-shadows-and-ao.md) — deeper: the soft-shadow estimator and its known artifacts.
- [Grep other consumers before declaring a fix done](../../engineering-practice/debugging/grep-other-consumers-before-declaring-a-fix-done.md) — same-trap: the fix had to land in three copies.
- [Replicate the real frame loop in a unit test](../../engineering-practice/testing/replicate-the-real-frame-loop-in-a-unit-test.md) — same-trap: synthetic sweeps missed what the real query hit.
