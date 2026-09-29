---
title: Anim studio is complete (Phase 8)
description: "Maps the anim studio behind --features anim_studio: pose editing, viewport drag, spring tuning, clips, phase oscillators, IK effectors, its model/UI split, the reference-pose pipeline, and measured ragdoll behaviour. Read before touching src/character/anim/studio or re-deriving ragdoll chatter numbers."
type: reference
status: current
tags:
  - tooling
  - poses
  - springs
  - ragdoll
  - character-animation
updated: 2026-09-25
code:
  - src/character/anim/studio
  - src/character/anim/clip.rs
  - examples/import_reference_pose.rs
  - examples/character_gallery.rs
sources:
  - CHARACTER_PROGRESS.md "Phase 8, part one" through "part five"
  - CHARACTER_PROGRESS.md "Correction: the joint-limit chatter was stale"
aliases:
  - Phase 8
  - egui authoring studio
  - pose editor
---

# Anim studio is complete (Phase 8)

Phase 8 finished on 2026-09-26, which completed the procedural-animation
roadmap (Phases 0–8). The anim studio is an egui authoring tool inside
`character_gallery`. It edits poses, springs, clips, phase oscillators and IK
effectors, and every module keeps its model separate from its UI.

Run it with:

```
cargo run --release --example character_gallery --features anim_studio
```

Some panels default to closed. These flags open them for screenshots:
`--studio-timeline`, `--studio-demo-clip`, `--studio-phase`.

## What exists

Everything under `src/character/anim/studio/` is gated behind the
`anim_studio` feature. The feature gates the studio's own code, not
`bevy_egui`, which four examples already use directly.

| Module | What it does |
|---|---|
| `edit` / `pose_editor` | Per-bone axis + degrees editing, RON load/save, mirror, capture-from-rig |
| `drag` / `drag_plugin` | Grab a joint in the viewport and rotate its parent |
| `effector` | Grab a hand or foot and the two-bone chain solves |
| `tuning` / `tuning_editor` | Per-bone DHO half-life and damping, presets, step-response plot |
| `timeline` | Clip keyframes, playback, contact lanes |
| `phase_editor` | Oscillator layer with an overlaid multi-wave plot |

`src/character/anim/clip.rs` is the clip model (sparse keyframes, stepped
contacts). It is new in Phase 8. The rewrite had no clip type before it.

## Structural convention: model split from UI

Every studio module keeps model and UI apart. `edit`, `drag`, `effector`,
`tuning` and `clip` are plain values and plain functions with no egui, and
they are tested without a window. The panels are thin. `ragdoll` and
`ragdoll_plugin` follow the same pattern, and it is why most bugs could be
found headlessly.

## Reference-pose pipeline

```
blender --background --python tools/dump_animation_pose.py -- \
    assets/models/idle.glb --frame 0 --ron /tmp/x.positions.ron
cargo run --release --example import_reference_pose -- \
    /tmp/x.positions.ron assets/anim/NAME.pose.ron
```

The pipeline emits **world positions**, never rotations. Blender's local
rotations are relative to Mixamo's bind pose, and ours are relative to this
crate's T-pose. `tools/dump_animation_pose.py` is deleted in the working tree
as of 2026-09-28 (uncommitted), so check `git status` before running the
first step. `examples/import_reference_pose.rs` still exists.

## Test counts and CI

At Phase 8 completion there were 791 tests with `--features anim_studio` and
692 without. **CI must build both**, or the gated code rots silently.

## Deleted; do not look for these

- `src/bench`, the old renderer harness. Nothing used it, so it was deleted
  along with the `image` crate dependency it alone needed. `PROGRESS.md`
  entries recorded against it stay as history but cannot be reproduced
  without rebuilding it.
- `poses::relaxed_stand_v2`, exploratory studio output with no provenance.
  `idle_stand` covers the same shape assertions and has reproducible
  provenance.

## Measured ragdoll behaviour at Phase 8 (do not re-derive from older notes)

- A joint **pinned at its rotation limit** is still: 0.062 rad/s, holding
  exactly at the limit. An earlier "6.5 rad/s chatter" claim was measured
  before `stable_damping` and the frame fixes, and is stale.
- A joint holding a **mid-range** target oscillates at 2–4 rad/s at the
  torque ceilings shipped then. The body's centre of mass sits half a bone
  from the joint anchor. Commanded angular acceleration therefore demands
  linear motion, the point constraint cancels it, and the controller
  commands it again. The effect persists at ζ = 0 and scales with anchor
  offset, so it is not a control-law defect.
- At those ceilings the dominant failure was **undershoot**, not noise: a
  70 rad/s² arm reached about 6.8° of a 15° target. The ceilings have since
  been raised to `CEILING_SCALE = 12`, and the real-rig ragdoll now tracks
  (see [Full-strength read-back hides the physics](../ragdoll-and-physics/full-strength-readback-hides-the-physics.md)).

## Related

- [The muscle module is deleted](./muscle-deleted-anim-is-the-only-stack.md) — prerequisite: the stack this studio edits.
- [Two-bone IK pivots at the upper joint](../ik-and-locomotion/two-bone-ik-pivots-at-upper-not-root.md) — example: the bug found in `studio::effector`.
- [Gizmos need --show-real-mesh off](./gizmos-need-show-real-mesh-off.md) — applies: the verification view to screenshot studio output with.
- [PD damping has an explicit-integration bound](../ragdoll-and-physics/pd-damping-explicit-integration-bound.md) — deeper: the `stable_damping` fix behind the corrected chatter numbers.
