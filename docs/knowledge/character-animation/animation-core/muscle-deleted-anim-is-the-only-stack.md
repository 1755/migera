---
title: The muscle module is deleted; anim is the only animation stack
description: "Records that src/character/muscle was deleted in commit 9981e16 (7,158 lines) and src/character/anim is the only animation stack: RON poses, anim_bench cost of ~1.8 µs/character/frame. Read when a doc or memory mentions MuscleSim/MuscleConfig, or before measuring animation cost."
type: decision
status: current
tags:
  - character-animation
  - performance
  - poses
  - springs
updated: 2026-09-25
code:
  - src/character/anim
  - assets/anim
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - commit 9981e16
  - CHARACTER_PROGRESS.md "Phase 7 — src/character/muscle deleted (7,158 lines)"
aliases:
  - Phase 7 cutover
  - MuscleSim
  - MusclePlugin
  - position-space muscle solver
---

# The muscle module is deleted; anim is the only animation stack

As of 2026-09-25 (commit 9981e16), `src/character/muscle` no longer exists.
It was 7,158 lines. `src/character/anim` is the only animation stack. Any doc
or memory that mentions `MuscleSim`, `MuscleConfig`, `MusclePlugin`,
`write_muscle_sim_to_bones`, `muscle_joint_pairs` or `character::muscle::pose`
describes deleted code.

## Context

`muscle` was a position-space solver modelled on Lugaru. It simulated joint
positions and derived each bone's rotation from them. `anim` authors and
springs each bone's local rotation directly, and position falls out of Bevy's
transform propagation. The domain [INDEX](../INDEX.md) explains why that
inversion removes whole bug classes. Once `anim` covered everything `muscle`
did, the old module had nothing left to do.

## Decision

Delete `muscle` outright rather than keep it as an A/B baseline. What
remained afterwards:

- **Poses** live in `assets/anim/*.pose.ron` and hot-reload. They are also
  embedded into `anim::poses` with `include_str!` as the compiled-in
  fallback. Nothing computes them at startup any more.
- **Tests** went from 759 to 654. The removed tests belonged to the deleted
  solver.
- **`--anim-backend` is gone** from `examples/character_gallery.rs` because
  nothing is left to A/B against. `--muscle-targets` became `--joint-chain`.
- **Performance harness:** `cargo run --release --example anim_bench --
  --characters N --frames N`. It measured about 1.8 µs per character per
  frame, linear up to 1000 characters (1.84 ms p50, p99/p50 ≈ 1.02).
- **Progress log:** `CHARACTER_PROGRESS.md`, kept separate from
  `PROGRESS.md`, which belongs to the hybrid renderer.

## Alternatives considered

- **Keep `muscle` behind `--anim-backend`.** This lost because `anim`
  covered every feature `muscle` had. The flag would only have kept 7,158
  lines compiling for comparisons nobody needed.
- **Keep Lugaru's position-space model and fix its bugs.** This lost
  because four of its hardest bug classes cannot even be expressed in
  rotation space (see [Lugaru's joint/muscle system](../lugaru-joint-muscle-system.md)).
  The one Lugaru idea carried forward is the continuous per-joint `strength`
  dial, which now scales PD torque ceilings.

## Consequences

- Never measure animation cost from `character_gallery`'s frame time. It is
  vsync-capped at about 16.7 ms and says nothing about animation cost.
- The Lugaru note stays as prior art, with a status banner.
- Everything this note listed as "remaining" at deletion time has since
  landed: Phase 8, the anim studio ([Anim studio is complete](./anim-studio-is-complete.md)),
  ragdoll joint limits, and a full-rig ragdoll spawn (`CHARACTER_PROGRESS.md`).

## Revisit when

Never for `muscle` itself. Re-run `anim_bench` and update the numbers here
if the per-frame pipeline grows a new stage.

## Related

- [Lugaru's joint/muscle system](../lugaru-joint-muscle-system.md) — contrast: the position-space design the deleted module implemented.
- [Anim studio is complete](./anim-studio-is-complete.md) — deeper: the Phase 8 tooling built on the surviving stack.
- [botica's character animation system](../botica-character-animation-system.md) — prerequisite: the Mixamo-standard skeleton `anim` targets.
- [migera pivots to Bevy PBR for characters](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — prerequisite: why characters use Bevy's pipeline at all.
- [Gizmos need --show-real-mesh off](./gizmos-need-show-real-mesh-off.md) — applies: how to verify poses on the surviving stack.
