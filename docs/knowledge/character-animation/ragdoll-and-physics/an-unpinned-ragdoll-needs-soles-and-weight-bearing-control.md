---
title: An unpinned ragdoll needs soles to stand on and joints that carry its weight
description: "Unpinned, the ragdoll buckled at 0.5 s (Winter 8.1) and its capsule feet skated 0.8 m. Sole-block feet stand (0.1° / 0.16 mm vs 26.6° / 25 mm), but the acceleration-shaped PD holds weight only by switching gravity off. Read before unpinning a ragdoll or making it balance."
type: lesson
status: current
tags:
  - ragdoll
  - physics
  - balance
  - correctness
updated: 2026-09-30
verified: 2026-09-30
code:
  - src/character/anim/ragdoll_plugin.rs
  - src/character/anim/foot.rs
sources:
  - "test ragdoll_plugin::tests::a_foot_stands_flat_on_its_sole"
  - "probe ragdoll_plugin::tests::probe_unpinned_ragdoll_stands (ignored; PROBE_FEET=1 for sole blocks)"
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §8.1; §4.0.1 Fig. 4.1 (foot breadth)"
aliases:
  - unpinned ragdoll
  - ragdoll feet
  - sole block
  - sole_blocks
  - support_own_weight
---

# An unpinned ragdoll needs soles to stand on and joints that carry its weight

The active ragdoll has only ever stood with its root pinned to the
animation (`RagdollSpawnConfig::pin_root`). Released onto a floor, it
failed three separate ways.

## What happened

Headless spike, `puppet_base`, unpinned, friction-1 floor, full gravity,
full-strength PD toward the bind pose:

- **It buckled at 0.5 s** (hips −10 cm), then repeatedly sank as far as
  −34 cm and recovered. This is Winter §8.1's null result: a forward
  simulation without feedback collapses within ~500 ms.
- **Its feet skated and rolled.** The left foot slid 0.8 m in 2.5 s and
  turned 67°, with the pelvis never more than ~4° off upright. Each foot
  was a capsule from ankle to ball: no heel and no flat underside.
- **Gravity had to be put back by hand.** `support_own_weight` sets
  `GravityScale = 1 − strength`, so at full strength the ragdoll weighs
  nothing. That is how the PD, an angular *acceleration* per body and
  blind to load, holds the body up. A balancing ragdoll must carry its
  real weight through its feet, so this compensation cannot be its answer.

## What is fixed

**Soles.** `RagdollSpawnConfig::feet` (from `sole_blocks(rig)`) gives each
foot body a flat block, not a capsule:
- heel to toe tip along the walk's own `foot::Sole` (`Sole::block`);
- Winter's breadth, 0.362 of the length (Fig. 4.1: 0.055 H / 0.152 H);
- 3 cm thick, friction 1.0;
- in the ankle bone's frame, which is the body's (`sole_collider`).

A single foot dropped onto the floor, left for 2 s:

| | turned | slid |
|---|---|---|
| sole block | 0.10° | 0.16 mm |
| capsule | 26.6° | 25.1 mm |

With soles on, the whole ragdoll's feet no longer roll (2° instead of 11°).
They still crawl ~0.85 m, because the legs buckle and re-extend: that part
is the controller.

## Still to do

The load-bearing joints (legs, trunk) need torque-shaped control against
real gravity, capped by Winter's per-kg budgets (§7.4.5). The budgets
then become the controller, not just its limits. See step 4 of
`WINTER_MOTION_PLAN.md`.

## Related

- [Use avian's apply_angular_acceleration, not apply_torque](./avian-apply-angular-acceleration-not-torque.md) — context: why the PD is acceleration-shaped, and what changes if it carries load.
- [Full-strength read-back hides the physics](./full-strength-readback-hides-the-physics.md) — same-trap: at full strength both the screen and the gravity say nothing about the physics.
- [8.1 Review of forward solution models](../../biomechanics-winter/ch08-synthesis-forward-solutions/8.1-review-of-forward-solution-models.md) — source: the ~500 ms collapse.
- [Walking foot rocker contact model](../ik-and-locomotion/walking-foot-rocker-contact-model.md) — prerequisite: the sole the blocks are built from.
