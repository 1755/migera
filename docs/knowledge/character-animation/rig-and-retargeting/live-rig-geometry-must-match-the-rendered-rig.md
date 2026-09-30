---
title: The live rig geometry must match the rendered rig, in metres and at its own rest
description: "The geometry the gait and foot IK solve on read bone translations raw (centimetres under a Mixamo rig's 0.01 node: a 46 m thigh) and put the hips at the synthetic 0.94 m, not the rig's rest. character.glb never walked; puppet_base (metres, hips 0.949) hid both. Read before building rig geometry or adding a rig."
type: lesson
status: current
tags:
  - rig
  - ik
  - locomotion
  - correctness
  - testing
updated: 2026-09-30
verified: 2026-09-30
code:
  - src/character/anim/plugin.rs
  - src/character/skeleton.rs
  - src/character/anim/stance.rs
  - src/character/anim/gltf_rig.rs
sources:
  - "test gltf_rig::tests::the_live_rig_geometry_stands_where_the_asset_does_in_metres"
  - "test gltf_rig::tests::a_stance_keeps_the_soles_where_the_asset_stands_them"
  - "live BRP, character_gallery --character-model models/character.glb --anim-speed-schedule 0:0,4:1.2,10:0"
aliases:
  - live_rig_geometry
  - bone_translation_scale
  - hips_rest_offset
  - character.glb walks sideways
  - centimetre rig
---

# The live rig geometry must match the rendered rig, in metres and at its own rest

The gait, foot IK and root motion all solve on one `RigGeometry`, built
live from the skeleton (`plugin::live_rig_geometry`). If it places any joint
somewhere other than where the renderer does, every foot is grounded
against the wrong floor.

## What happened

`character.glb` (a Mixamo rig) never walked:

- It travelled 0.27 m/s when 1.2 was asked.
- Its hips faced 63° off its travel.
- Its planted feet slid ~200 mm.

Standing, it looked fine. `puppet_base` walked correctly throughout, and
every measurement in the progress log came from it. There were two causes,
and `puppet_base` hid both:

1. **Units.** Bone offsets were each bone's raw `Transform.translation`.
   `character.glb` sits under Blender's FBX-correction `Armature` node,
   scale 0.01, so its translations are centimetres: a 46.4 "m" thigh. The
   hips' own translation already handled this scale
   (`hips_parent_rest_world_scale`); the other bones did not.
   `puppet_base` is authored in metres.
2. **Rest height.** The hips offset was the synthetic table's 0.94 m. The
   renderer puts the hips at the rig's real rest plus the root translation
   (`hips_local_translation_for`), so the solve's hips sat at 0.94 m while
   the screen's sat at the asset's rest. On `character.glb` (1.126 m) the
   IK crouched the legs to reach a floor 0.186 m too high, and the feet
   floated by exactly that. On `puppet_base` (0.949 m) the error was 44 mm:
   9 mm in height, plus the pelvis's 43 mm forward offset.

Fixing the first alone made `character.glb` walk at speed, but crouched and
floating. Fixing the second exposed a third problem, on `puppet_base`:
`stance_on_rig` bent the knees without lowering the hips, so the stance's
feet floated 6.9 mm. The 9 mm the old geometry had wrong read as slack; with
it gone, the foot IK pitched each foot 4.5° toe-down to reach the floor.

## The rule

- **Build rig geometry in one place.** `live_rig_geometry(skeleton, read)`
  takes bone translations times `HumanoidSkeleton::bone_translation_scale`,
  and the hips at `HumanoidSkeleton::hips_rest_offset`. The old test copied
  the construction by hand and passed while the live one drifted.
- **Check geometry against the asset, not against itself.**
  `the_live_rig_geometry_stands_where_the_asset_does_in_metres` compares
  forward kinematics of the rest pose to the parsed asset's bind, joint by
  joint, for `puppet_base` and a centimetre copy of it under a 0.01 node.
  Sabotaged, it fails by 11.5 m (scale) and 44 mm (hips).
- **A pose that shortens the legs lowers the hips.** Only the pelvis-drop
  solver lowered them before, and its 2 cm deadband swallows a stance's
  7 mm.
- **One metre-authored rig proves nothing about units.** Check a new
  behaviour on both rigs, `--character-model models/character.glb`.

## Measured, live, after

| | `puppet_base` | `character.glb` (before) |
|---|---|---|
| speed at 1.2 m/s asked | 1.18 | 1.18 (0.27) |
| step width | 129 mm | 148 mm |
| steady planted slide | 3.4 / 3.5 mm | 5.1 / 3.7 mm (~200) |
| pelvis–chest correlation | −0.97 | −0.97 (walking sideways) |

## Still open

The foot IK and the walk define the sole differently:

- The IK's `toe_contact_offset` is the toe joint's height above the lower
  of joint and tip. On `puppet_base` the two are level, so it plants the
  toe joint itself on the floor.
- The walk's `Sole` puts the contacts at the bind pose's floor, 15 mm below
  that joint.

Standing, the ball renders at 0.0016 m where the asset binds it at
0.0152 m. That predates this fix: the ball stood at 0.0102 m before it.

## Related

- [Symptom is far from cause in the rig chain](../../engineering-practice/debugging/symptom-is-far-from-cause-in-the-rig-chain.md) — same-trap: the first hips-offset bug (wrong frame), in the same substitute.
- [Rig authored at critical extension](../ik-and-locomotion/rig-authored-at-critical-extension.md) — why a stance that leaves the hips up has no leg to reach the floor with.
- [Synthetic-rig tests are blind to retargeting](./synthetic-rig-tests-are-blind-to-retargeting.md) — same-trap: a test rig that cannot show the bug.
