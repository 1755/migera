---
title: Sample the ground in the world, not in the pose's frame
description: "Foot IK and the get-up sampled the ground at pose-frame points or at the entity's height, right only for flat ground or a character facing -Z on a straight slope; turned across a 0.2 grade both feet stood level, and a rise went 359 mm into the hillside. Map pose points into the world first. Read before sampling AnimGround anywhere."
type: lesson
status: current
tags:
  - ik
  - locomotion
  - ragdoll
  - correctness
  - testing
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/plugin.rs
  - src/character/anim/ragdoll_plugin.rs
  - src/character/anim/ground.rs
  - examples/character_gallery.rs
sources:
  - "test plugin::tests::a_turned_character_stands_on_the_slope_where_its_feet_are (fails by 66 mm with pose-frame sampling)"
  - "test ragdoll_plugin::tests::a_ragdoll_rising_on_a_slope_keeps_clear_of_the_ground_under_it (fails at 359 mm with the entity's height as the floor)"
  - "live BRP, character_gallery --anim-slope 0.2 --ragdoll on --fall-at-frame 120, every joint against the slope"
aliases:
  - GroundProbe world position
  - sample_ground
  - rise clearance on a slope
  - uneven ground
---

# Sample the ground in the world, not in the pose's frame

`GroundProbe::sample` takes a world point. Two callers handed it something
else:

- **Foot IK** passed the toe's position in the pose's frame. That frame
  sits at the character, so on flat ground any point gives the right height,
  and on a straight slope along −Z, sampled relative to an entity already
  standing on it, it happens to as well. Turned 90° across a 0.2 grade, the
  slope ran along the character's own forward. Both feet stood level, one
  33 mm into the hillside and the other above it (66 mm apart in the test).
- **The get-up** kept the rising skeleton above one flat height, the
  entity's. Live on a 0.2 grade, a calf went 47 mm into the hillside. In a
  test where the body fell uphill of its entity, a toe went 359 mm in.

## Why it matters

Flat ground hides both, and so does the commonest slope test: a character
walking straight up a grade that runs along −Z. A test of the get-up
falling DOWN the slope passed with the bug in place too: everything lay
below the entity's height, where a floor at that height is never too low.
The case that exposes it puts the body above its entity.

## How to apply

- Map a pose point into the world before sampling: the character's origin
  (its own `Transform` when it has no parent, since the `GlobalTransform`
  is a frame old) plus the turn of whatever the hips hang from, against
  the bind (`HumanoidSkeleton::hips_root_rotation`). Don't take the turn
  from the hips themselves, which carry the walk's pelvic twist. Convert
  the hit back: height minus the origin's, normal by the inverse turn
  (`solve_foot_ik`'s `sample_ground`).
- The rise samples the character's `AnimGround` under each joint it keeps
  clear and under each tucked tip (`write_simulated_pose`, `tuck_foot`),
  falling back to the entity's height without one.
- Test a turned character, and a body above its reference height.

## Not covered

The get-up keys are still posed against a flat floor: on a slope the
uphill contacts lift the whole skeleton, and the downhill ones float. A
hand has no body, so a fallen arm lying down a slope rests its hand joint
up to 4 cm under the surface.

## Related

- [Feet stand on the physics world through ground sampled under them](./feet-stand-on-the-physics-world-through-sampled-ground.md) — applies: a probe answering these world queries from the physics world.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — same-trap: the other slope fault, a lock blind to the body's rise.
- [Foot IK on uneven ground has two feedback loops](./foot-ik-feedback-loops.md) — prerequisite: why the IK samples the ANIMATED toe, which still holds.
- [Getting up goes through key poses chosen by how the body lies](../ragdoll-and-physics/getting-up-is-a-timed-blend-then-a-re-pin.md) — applies: the rise whose clearance this fixed.
