---
title: A pose delta's world is the character's frame, not the scene's
description: "The renderer reads a pose delta about the character's axes as bound, so it turns with the character. Converting deltas to scene rotations on a rig rooted at the live facing is right only facing the spawn direction; the rest pose hides it. Read before converting pose deltas to or from scene-space rotations."
type: lesson
status: current
tags:
  - retargeting
  - rig
  - ragdoll
  - correctness
  - testing
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/ragdoll_plugin.rs
  - src/character/anim/rig.rs
  - src/character/anim/retarget.rs
sources:
  - commit feecaa8
  - "tests ragdoll_plugin::tests::a_pinned_ragdoll_walks_and_turns_with_its_character, a_turned_character_reads_back_the_pose_its_bodies_hold"
  - "live BRP, character_gallery --ragdoll on --push-schedule 3:0:1.6, puppet_base and character.glb"
aliases:
  - character_frame
  - character turn
  - T-pose arms after getting up
  - rise turn snap
---

# A pose delta's world is the character's frame, not the scene's

[A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md),
but that "world" is the character's frame at bind time.
`retarget::write_pose_to_skeleton` conjugates each delta by the bind-time
accumulated binds, so the delta turns with the character. Any code that
turns a pose into scene rotations, or reads scene rotations back into a
pose, must work in the bind-rooted rig. It applies the character's turn
`F = live hips-parent rotation · bind root⁻¹` only where the result goes
out to the scene.

## What happened

`ragdoll_plugin::rig_geometry` roots the rig at the hips' parent's LIVE
rotation, which is right for positions and a facing. The ragdoll also used
it to convert the rendered pose into body targets
(`accumulate_world_rotations`) and body rotations back into a pose
(`delta_from_world`). Those conversions read every delta as a rotation
about a scene axis, while the renderer read it about the turned
character's axis. The two agreed only while the character faced its spawn
direction.

Getting up turns the character to face along its body, 86° in the traced
run, and both symptoms came from that turn:

- **A snap at the rise's start.** The lying body, drawn from its bodies,
  was read back off by the turn: the neck moved 534 mm in one frame.
- **A T-pose after standing.** The pinned bodies' targets were off by the
  turn, so the physical arms held out sideways under a correctly drawn
  relaxed pose. The screen showed the animation (strength 1), so only
  the bodies showed it.

The fix is `character_frame`: the bind-rooted rig plus `F`. Targets are
`F · accumulate_world_rotations(pose, bind-rooted rig)`. Read-back takes
`F⁻¹ · body rotation` before inverting. Afterwards: bodies within
0.1–2.8° of their targets once standing, arms hanging 82° below
horizontal, and no per-frame neck move over 34 mm after landing, on both
rigs.

## Why it matters

The test for a turned character existed and passed. It drove the **rest
pose**, where every delta is the identity, and an identity delta reads the
same about any axis. It could not tell the two frames apart, the same
blind spot as a delta parallel to its bind axis in the world-axis lesson.

## How to apply

- Convert poses ↔ scene rotations in the bind-rooted rig
  (`plugin::live_rig_geometry`, or `character_frame` in the ragdoll), then
  apply the character's turn at the boundary.
- A rig rooted at the live facing is fine for positions and for "which
  way is forward". It is wrong for reading or writing deltas.
- Test frame conversions with a non-rest pose on a turned character.
  Turning alone, or a non-rest pose alone, can't tell the frames apart.
- To see whether bodies hold the animation, compare them to their
  targets over BRP (`JointTarget` vs avian `Rotation`), not the picture.

## Evidence

Commit feecaa8 (2026-09-30). With the fix disabled, the turning test (now
in `relaxed_stand`) fails with Spine 8.8° off its drawn bone. The
read-back test, turned a quarter while falling so the screen shows the
bodies, fails with the hips 90° off. Both pass with it.

## Related

- [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) — prerequisite: the conjugation this frame sits under.
- [Full-strength read-back hides the physics](../ragdoll-and-physics/full-strength-readback-hides-the-physics.md) — same-trap: why the T-pose arms were invisible on screen.
- [Getting up goes through key poses](../ragdoll-and-physics/getting-up-is-a-timed-blend-then-a-re-pin.md) — example: the rise's turn is what exposed it.
- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: a test whose input could not distinguish right from wrong.
