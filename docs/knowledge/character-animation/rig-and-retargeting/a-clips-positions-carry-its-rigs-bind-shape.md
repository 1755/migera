---
title: A clip's world positions carry its rig's bind shape
description: "A mocap clip's joint positions converted into bends from the straight T-pose store the source rig's bind curvature as a bend, applied again on a target's own bind: relaxed_stand's back was over-arched. Convert against the source's bind. Read before importing or editing a reference pose."
type: lesson
status: current
tags:
  - retargeting
  - poses
  - rig
  - correctness
  - testing
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/convert.rs
  - src/character/anim/stance.rs
  - src/character/anim/poses.rs
  - examples/import_reference_pose.rs
  - examples/rebase_pose_onto_bind.rs
  - tools/dump_bind_positions.py
  - assets/anim/relaxed_stand.pose.ron
  - assets/anim/idle_stand.pose.ron
  - assets/anim/idle_bind.positions.ron
sources:
  - "assets/models/idle.glb skin inverse bind matrices (tools/dump_bind_positions.py)"
  - "tests poses::tests::relaxed_stand_still_matches_its_reference_data, the_relaxed_stand_stands_upright_and_balanced; probes probe_spine_profile, probe_pose_against_clip, probe_balance_lean"
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed., Example 5.1 (static stance: centre of pressure 4 cm anterior to the ankle)"
aliases:
  - hyperlordosis
  - over-arched back
  - bind-relative retarget
  - pose_from_world_positions_against
  - rebase_onto_bind
  - balance_over_feet
  - STANDING_COM_AHEAD
---

# A clip's world positions carry its rig's bind shape

A `LocalPose` is a bend away from a rig's bind: the renderer applies it to
whatever shape each rig was bound in. A mocap clip's world joint positions
describe the actor's shape, and part of that is the shape the source rig
was bound in. Converted into bends from this crate's straight synthetic
T-pose (`convert::pose_from_world_positions`), that bind shape is stored as
if the actor had bent that way. A target rig then bends its own bind by
it. Wherever the source rig's bind isn't straight, the shape is counted
twice. Convert against the source's bind instead
(`pose_from_world_positions_against`).

## What happened

puppet_base stood with its back visibly over-arched (reported by eye).
Measured (`probe_spine_profile`), each spine segment's forward lean from
vertical, pelvis up:

| | pelvis→lower | lower→mid | mid→upper | chest→neck |
|---|---|---|---|---|
| puppet_base bind | +16.4° | +5.8° | +1.3° | −13.0° |
| `relaxed_stand` (old) | +16.4° | +1.5° | −12.0° | −23.6° |
| Mixamo bind (`idle.glb`) | −0.3° | −0.3° | −14.1° | −12.2° |
| `relaxed_stand` (fixed) | +14.6° | −0.4° | −0.1° | −13.5° |

The idle clip stands almost exactly in its own bind: relative to it, its
spine bends −4.1, +0.8 and +1.6°. Converted from straight, those segments
stored about 4, 13 and 11° of backward bend: Mixamo's curvature. On
puppet_base, the base of the neck sat 79 mm behind its bind's.

## Why it matters

- **World positions are not frame-free.** The import's own documentation
  said so ("world positions have no reference frame to get wrong"), and
  so did the test pinning `relaxed_stand` to the clip's positions on the
  straight synthetic rig. That test encoded the error.
- **A test of the trunk can be fooled by the shoulders.** The upright test
  measured hips to shoulder joints and passed. puppet_base's collarbones
  are bound swept 98 mm back; the actor's slope 37 mm forward (Mixamo's
  are straight out). So the transferred shoulders sit forward and hid the
  arched spine. Measured to the base of the neck, the old pose fails by
  8.3°.
- **Fixing the shape moved the balance.** The arch had held the upper
  body's mass back. Straightened, the stance stood 7.5 cm ahead of its
  ankles against Winter's 4. Nothing aligned a stance over its feet;
  `stance::balance_over_feet` now leans it about the ankles (feet flat,
  gaze kept), 2.1° here.

## How to apply

- **Import with the bind:**
  `tools/dump_bind_positions.py <model> <bind.positions.ron>`, then
  `import_reference_pose <clip> <out> <bind>`. Without it the tool warns.
- **A hand-finished pose that can't be re-imported:**
  `examples/rebase_pose_onto_bind` rebases chosen bones
  (`convert::rebase_onto_bind`), keeping every other bone's world
  orientation. `relaxed_stand` got its spine only. Its neck was solved
  for gaze, not converted (rebased, the head went 23° back). Its legs are
  the stance's. Its arms moved 0.1–1.1° rebased, so they were left.
- **Judge a converted pose against the clip on a rig bound like the
  source** (`convert::forward_kinematics_against`), not on the synthetic
  rig. On a different rig, segments where the two binds differ won't match
  the actor's absolute directions (puppet_base's mid-spine is 14° off
  Mixamo's), and they shouldn't: the target keeps its own modelled
  posture.
- **Measure posture on the spine itself,** not via a landmark another
  chain moves.

## Evidence

2026-10-01:

- `relaxed_stand_still_matches_its_reference_data` now checks the spine
  bind-relative; the old pose fails it by 14.07°.
- `the_relaxed_stand_stands_upright_and_balanced` now measures the trunk
  to the neck; the old pose fails by 8.3°.
- Re-importing `idle_stand` the old way reproduced the committed file
  exactly before the bind-relative version replaced it (worst direction
  error 0.028°).
- Live, both rigs, Left and Front views with gizmos: the spine line is
  near straight where it bowed back, and the head is level.

## Related

- [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) — prerequisite: what a pose's bend is relative to.
- [The puppet_base fixture faces away from the rendered character](./puppet-base-fixture-faces-away-from-the-rendered-character.md) — same-trap: another reference frame the pose data silently assumed.
- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: the reference test compared the pose with data converted the same wrong way.
- [Push recovery is Winter's pendulum](../ik-and-locomotion/push-recovery-is-winters-pendulum.md) — applies: the stance it sways about now stands with its centre of mass where Winter puts it.
