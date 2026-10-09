---
title: A perch is the get-up's squat, feet together, forearms on the knees
description: "Step 16: a perch is the get-up's deep squat, feet 0.14 m apart, forearms over the knees, the trunk turned till the COM is over the feet; eased in from standing over 0.8 s, the legs blended by their feet. It rises before it walks or jumps. Read before changing parkour/perch.rs."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/perch.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::perch::tests::a_perch_crouches_over_its_feet"
  - "live: character_gallery --post 0,-1.2,0.3 --onto-at 3,0,0.3,-1.2 --perch-at 6.5 --look-round-at 9, Xvfb, gizmos on/mesh off Left and Front, mesh on"
  - "anim_bench --gait perch --characters 100"
aliases:
  - perch
  - perched
  - viewpoint
  - look round
  - Walker::perch
---

# A perch is the get-up's squat, feet together, forearms on the knees

Step 16 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md):
crouched on a post or a narrow top, Assassin's Creed's perch. There is no
perch data; the shape is by eye on the get-up's squat.

## Decision

**The shape** (`perch_pose`), made once per rig:
- the get-up's deep squat (`getup::squat`: flat feet, the shins leant 35°
  forward, its lean solved to put the COM over the feet);
- the feet brought 0.14 m apart (`stance::narrow_feet`);
- each wrist 0.12 m ahead of its knee and 0.06 m under it, the forearms
  laid over the knees, the hands hanging in front;
- the trunk turned back at the waist (bisected) until the COM is over the
  feet's middle again, since the arms forward carried it ahead;
- placed with its feet's middle where standing's is.

**The layer** (`perch`) blends the pose toward the shape by a weight, the
legs blended by their feet (`sitting::legs_by_their_feet`), so the feet
stay on the top all the way down and up.

**The walker** (`Walker::perch`) eases the weight over 0.8 s while
standing still and asked. It rises (the weight back to 0) when not asked,
or when asked to walk or to jump onto a top (`Walker::onto`, which waits
for it). While perched, its speed is held at 0. Stood on a top it jumped
onto, the beam's balance arms are under the perch and give way to it as
it crouches.

**Looking round** (`Walker::look_round`, a viewpoint's): the gaze is swept
1.1 rad either side of the facing and back every 7 s, level, through the
walker's look.

## Alternatives considered

- **A new floor pose** (`sitting::FloorPose::Perch`), to reuse sitting's
  keys and posture: every match over the ways of sitting, and the tests
  that sweep them all, would take a pose they were not written for. The
  perch is one shape with no seat; a layer eased like the beam's balance
  is enough.
- **The sneak's crouch** (`sneak::Footing`): it goes no deeper than 0.18 of
  a leg, a stalk, not a perch.

## Consequences

**Headless** (`puppet_base`):
- the feet were 0.14 m apart, both on a 0.3 m top;
- the COM was between the heels and the balls;
- the hips were under 60 % of standing's height;
- each wrist was ahead of its knee, within 0.25 m;
- nothing went under the floor;
- crouched down and up over 0.8 s each, an ankle stayed within 1 cm of
  standing's height and no joint's step changed over 1 cm in a frame.

**Live**: after a precision jump onto a 0.3 m post 1.2 m ahead, it crouched
into the perch on the post, its feet on its top, forearms on its knees
(seen Left and Front, gizmos and mesh), then looked round. Asked onto a
post 1.6 m ahead at 0.3 m up, out of a standing jump's reach, it stayed
and perched on the floor.

**Cost**: `anim_bench --gait perch --characters 100`, 10 µs a character at
p50.

## Revisit when

- **Onto a perch from a hang below**, onto a top too small to stand on: the
  climb-up ends standing; a climb-up ending in the perch is not built.
- **From a perch**: a leap rises to standing first (0.8 s); a leap out of
  the crouch itself (the jump starting at its countermovement's bottom),
  and a drop from the perch to a hang, are not built.
- **A perch on a beam's end or a rail**: only flat tops are found; the
  feet on a rail would want turning across it.

## Related

- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 16's design.
- [A precision jump is a standing jump handed to a fall at its top](./a-precision-jump-is-a-standing-jump-handed-to-a-fall-at-its-top.md) — prerequisite: how it gets onto the top it perches on.
- [Sitting down and standing up go through solved keys](../ik-and-locomotion/sitting-down-and-standing-up-go-through-solved-keys.md) — deeper: the squat key and the blend by the feet this reuses.
