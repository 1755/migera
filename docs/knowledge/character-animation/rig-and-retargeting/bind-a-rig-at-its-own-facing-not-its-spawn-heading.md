---
title: Bind a rig at its own facing, not its spawn heading
description: "Binding a glTF humanoid captures its hips' parent world rotation; spawned already turned, that capture took the heading as part of the bind and relaxed_stand held both hands overhead. Bind with the facing correction but without the heading. Read before binding a rig spawned at any yaw."
type: lesson
status: current
tags:
  - retargeting
  - rig
  - correctness
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/humanoid.rs
sources:
  - "live: physics_character_playground --physics kinematic --speed 0 --camera 0,1.4,4, before and after the fix"
aliases:
  - bind_gltf_humanoids
  - hips parent rest rotation
  - spawn heading
  - hands overhead
---

# Bind a rig at its own facing, not its spawn heading

`HumanoidSkeleton::for_other_rig` takes the hips' parent's rest WORLD
rotation, read when the rig binds. That rotation includes whatever the
character's root was turned to. If it was spawned already facing some
way, the heading was captured as part of the rig's bind, and every pose
delta was then written about the wrong axes.

## What happened

`examples/physics_character_playground.rs` spawns characters at random
yaws. Standing in `relaxed_stand`, the first one held both hands straight
overhead. The same character in `character_gallery` stood normally. The
gallery always binds at heading zero (the root carries only the asset's
180° facing correction), and it turns its characters only after binding,
which the retargeting handles (a 1.2 m/s, 0.5 rad/s circle walks the
expected 2.35 m radius).

`humanoid::bind_gltf_humanoids` now binds with
`correction · root⁻¹ · hips_parent`: the asset's facing correction, which
is a fact about the model, without the heading, which is a fact about
the moment. Measured on the same frame: arms hanging at the sides,
facing the spawn yaw.

## Why it matters

A rig binding reads live world transforms once and keeps them. Anything
in those transforms that is not part of the model (spawn heading,
position, a parent the character was attached to) becomes a permanent
error in every pose. A test or example that always spawns at the origin
facing forward cannot show it. This is the binding-time cousin of
[a pose delta's world is the character's frame](./a-pose-deltas-world-is-the-characters-frame.md).

## How to apply

- Bind from transforms relative to the character's root, with only the
  model's own corrections composed back in.
- When adding a spawn path, check one character at a non-zero yaw
  standing still, close up, before trusting walking views. A turned rest
  pose is the quickest way to see a captured heading.

## Related

- [A pose delta's world is the character's frame](./a-pose-deltas-world-is-the-characters-frame.md) — same-trap: converting pose deltas against the live facing instead of the bound one.
- [puppet_base fixture faces away from the rendered character](./puppet-base-fixture-faces-away-from-the-rendered-character.md) — same symptom (hands overhead) from measuring on a rig turned 180°.
- [The live rig geometry must match the rendered rig](./live-rig-geometry-must-match-the-rendered-rig.md) — deeper: what else the binding must capture from the real rig.
