---
title: Animation core
description: "The state of migera's rotation-space animation stack (muscle deleted, anim only), its authoring studio, spring numerics, and how to visually verify a pose. Read before measuring animation cost, touching springs or the studio, or taking a verification screenshot."
type: index
status: current
tags:
  - character-animation
  - springs
  - tooling
  - verification
updated: 2026-09-28
---

# Animation core

What `src/character/anim` is as a whole: the decision that made it the only
stack, its authoring tool, the spring maths every bone runs through, and the
verification view every pose change must pass.

## Start here

Read [The muscle module is deleted](./muscle-deleted-anim-is-the-only-stack.md)
first if an older doc mentions `MuscleSim`. Read
[Gizmos need --show-real-mesh off](./gizmos-need-show-real-mesh-off.md) before
your first screenshot.

| Note | What it establishes | Read when |
|---|---|---|
| [The muscle module is deleted; anim is the only animation stack](./muscle-deleted-anim-is-the-only-stack.md) | `src/character/muscle` deleted in 9981e16; RON poses; ~1.8 µs/character/frame via `anim_bench` | a doc mentions `MuscleSim`/`MusclePlugin`, or before measuring animation cost |
| [Anim studio is complete (Phase 8)](./anim-studio-is-complete.md) | Studio modules behind `--features anim_studio`, model/UI split, reference-pose pipeline, measured ragdoll behaviour | before touching `src/character/anim/studio` or `clip.rs`, or re-deriving ragdoll chatter numbers |
| [The rational exp approximation in spring code diverges](./spring-exp-approximation-diverges.md) | The cubic `exp(-x)` approximation is 74x wrong at x = 10; use `f32::exp` | before writing or optimizing spring/damper/inertialization maths |
| [Gizmos need --show-real-mesh off](./gizmos-need-show-real-mesh-off.md) | Depth-tested gizmos are hidden by the skinned mesh | before any pose-verification screenshot in `character_gallery` |

## See also

- [Lugaru's joint/muscle animation system](../lugaru-joint-muscle-system.md) — the prior art the deleted module implemented.
- [Replicate the real frame loop in a unit test](../../engineering-practice/testing/replicate-the-real-frame-loop-in-a-unit-test.md) — the headless check to run before a screenshot.
