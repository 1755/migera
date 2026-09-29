---
title: Grep other consumers before declaring a fix done
description: A fix to shared state is verified only when every reader of that state has been checked; the walk-in-place fix anchored the mesh while the debug gizmo overlay, which read the solver directly, kept walking. Read before saying "fixed" on a change to a resource, component, cache or a duplicated WGSL/CPU function.
type: lesson
status: current
tags:
  - debugging
  - verification
  - correctness
updated: 2026-09-22
verified: 2026-09-28
sources:
  - Claude memory grep_other_consumers_before_declaring_fix_done (2026-09-22)
aliases:
  - shared state fix
  - second consumer
  - duplicated shader code
---

# Grep other consumers before declaring a fix done

Checking that a change does what you intended verifies one reader. Before
saying "fixed" on any change to shared state, **grep for every other reader**
of the same field, resource or function and check each one.

## What happened

The fix was in the old muscle simulation (deleted in commit 9981e16, but the
pattern recurs):
- `apply_solved_sim_to_skeleton` was changed to cancel walk-cycle root drift on
  `Transform`, for "walk in place". A screenshot showed the mesh and capsule rig
  staying anchored, and the fix was reported as verified.
- `draw_muscle_debug_gizmos`, the white-line overlay, read
  `MuscleSim::position_of()` directly and bypassed `Transform`. It was a second
  consumer of the same raw solved positions, and the fix never touched it.
- That function's doc comment said explicitly that it draws "straight from
  `MuscleSim`'s own solved joint positions", and the comment had been read
  earlier in the same session. The user caught it live: "White debug skeleton
  still walking".

## Why it matters

This is a search problem, not a reasoning problem. The tools to catch it were
available before the fix was declared done. Debug layers in
`character_gallery.rs` deliberately show raw and derived readouts side by side
for ground-truth comparison, which is exactly the pattern that creates two
paths that can silently diverge.

## How to apply

- Before saying "confirmed", "fixed" or "verified" on a change to a `Resource`,
  a component read by several systems, a cache or a derived value, grep for all
  read sites.
- A screenshot or test that exercises one consumer is not full verification
  when there are several.
- In the hybrid renderer, the same rule covers **duplicated code**. Shader
  logic exists as a CPU reference plus one or more WGSL copies, and a fix must
  land in every copy.

## Evidence

- The hybrid renderer's DDGI leak was a port bug in one WGSL copy of the
  probe-grid lookup while the other copy was correct, and the shadow
  `VIS_CUTOFF` fix had to land in three places. See the Related links.

## Related
- [Shadow margin / VIS_CUTOFF leak](../../hybrid-architecture/gi-and-lighting/shadow-margin-vis-cutoff-leak.md) — example: one fix applied to `cpu_ref.rs` and two WGSL copies.
- [DDGI sealed-room light leak](../../hybrid-architecture/gi-and-lighting/ddgi-sealed-room-light-leak.md) — example: one of two copies of the same lookup was ported wrong.
- [Gizmos need --show-real-mesh off](../../character-animation/animation-core/gizmos-need-show-real-mesh-off.md) — applies: the gizmo overlay is a separate reader of skeleton state and must be checked on its own.
- [The muscle module was deleted](../../character-animation/animation-core/muscle-deleted-anim-is-the-only-stack.md) — prerequisite: why the functions named here no longer exist.
- [A measurement of a broken system](../measurement/a-measurement-of-a-broken-system.md) — same-trap: fixing something load-bearing also invalidates constants measured against it; grep for those too.
