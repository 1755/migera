---
title: A sweep that starts in contact reads as a hit at distance zero
description: "A camera slid against a wall then swept from that contact point, and avian (and an SDF sphere tracer) reported a hit at distance 0 on every later sweep: boom 0 m, a false 0° ceiling cap. Fix: stop slides a skin short and ignore origin contact when moving away. Read before chaining shape casts."
type: lesson
status: current
tags:
  - camera
  - physics
  - correctness
  - debugging
updated: 2026-10-10
verified: 2026-10-10
code:
  - src/camera/collision.rs
  - src/camera/probe.rs
sources:
  - "test camera::collision::tests::a_shoulder_offset_into_a_wall_slides_in"
  - "avian3d 0.7 ShapeCastConfig::ignore_origin_penetration (shape_caster.rs)"
  - "live: camera_playground --script wall, BRP read of CameraRigState, 2026-10-10"
aliases:
  - ignore_origin_penetration
  - contact skin
  - sweep from contact
---

# A sweep that starts in contact reads as a hit at distance zero

Camera collision chains its sweeps:
- origin → shoulder;
- shoulder → eye;
- shoulder → up, for the ceiling.

When the first sweep stops exactly *at* contact, every later sweep starts touching the
surface. A shape cast from a touching start reports a hit at distance 0, even when it
moves away from or parallel to the surface. A boom running along a wall then collapses to
0 m, and the ceiling sweep reports a ceiling at 0 m.

## What happened

- **The scene** (2026-10-10, `camera_playground --script wall`): the character stood
  against the long wall with the camera turned along it.
- **The slide:** the right-shoulder offset pointed into the wall, so the shoulder point
  slid into contact with it.
- **What the HUD showed:**
  - boom **0.00 m of 3.2 m**;
  - pitch forced from 15° to **7.7°**, because the ceiling sweep "hit" at 0 m and capped
    pitch at `asin(0)`;
  - the high fallback view latched on.
- **Why the tests missed it:** the pure tests used `SdfProbe`, and its sphere tracer had
  the same flaw: `d − r < ε` at `t = 0` is a hit. The shoulder-slide test only asserted
  that the eye was outside the wall, never that the boom kept its length.

## Why it matters

Any chain of casts where one stops at a surface and the next starts there hits this. It
looks like collision being "too eager", so tuning radii or timings won't fix it.

## How to apply

- **Stop slides a skin short** of the hit (1 cm here, in `collision.rs::reach`). The next
  sweep then starts in free space.
- **Ignore contact at the origin when moving away from the surface.** avian has
  `ShapeCastConfig::ignore_origin_penetration: true`, and `SdfProbe` does the same: at
  `t = 0`, within ε of the surface, with `normal·direction > 0`.
- **Ask about penetration separately**, with an overlap test (`CameraProbe::overlaps`).
  That is the question the zero-distance hit was standing in for.
- **Assert what the player sees.** A slide test must check that the boom keeps its length
  and that no false cap appears, not only that nothing overlaps.

## Evidence

- The extended `a_shoulder_offset_into_a_wall_slides_in` fails with both halves of the
  fix undone ("the boom collapsed to 0").
- After the fix, the same live shot shows boom 1.39 m of 3.2 and pitch 15°. That shot also
  uses the shoulder swap.

## Related

- [Camera collision and occlusion techniques](./camera-collision-and-occlusion-techniques.md) — applies: the sweep chain this trap lives in.
- [Third-person camera design](./third-person-camera-design.md) — applies: `resolve_boom`'s stages.
