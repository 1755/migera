---
title: Feet stand on the physics world through ground sampled under them
description: "The foot IK's GroundProbe has no world access, so PhysicsGround raycasts a grid under each foot every frame and hands a SampledGround over; the body rises to the mean under its soles. Costs measured: rays ~0.1 ms, a ragdoll ~0.25-0.3 ms, a capsule ~0 per character. Read before grounding characters on physics props."
type: decision
status: current
tags:
  - ik
  - locomotion
  - physics
  - performance
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/physics_ground.rs
  - examples/physics_character_playground.rs
sources:
  - "test physics_ground::tests::sampled_ground_stands_on_a_prop_the_floor_elsewhere_and_the_body_on_its_support"
  - "live: physics_character_playground --plank 0.15 --trace-feet; --bench 10 --characters 16 --physics none|kinematic|ragdoll [--ground flat]"
aliases:
  - PhysicsGround
  - SampledGround
  - PhysicsGroundPlugin
  - feet on props
  - physics LOD cost
---

# Feet stand on the physics world through ground sampled under them

The foot IK asks a `GroundProbe` what lies under a point, from inside its
solve, where there is no access to the physics world. So
`physics_ground::sample_physics_ground` raycasts beforehand, each frame, a
9 × 9 grid at 5 cm under each foot (where the foot was last frame; it moves
a few centimetres a frame). It hands the hits to a `SampledGround`, which
answers the IK.

## Context

`examples/physics_character_playground.rs` drops props for characters to
walk among, and asked that their feet stand on them, not pass through.
The walker's own height follows the same probe, sampled at the character's
origin.

## Decision

- **Under a foot:** the highest hit within a grid spacing. Taking the
  nearest sample instead, a foot on a prop's edge would flicker between
  the prop and the floor as the nearest sample changed.
- **Too tall to stand on:** hits more than `max_step` (0.35 m) above the
  ground the body stands on (`support`) are ignored, so the foot does not
  climb a big prop; the body pushes it instead. The rays start above
  `support + max_step`. Measured from the floor instead, the limit hid
  every stair above the second riser and every ramp above 0.35 m. With it
  relative, a walker climbs a 5 m stair.
- **Drops are seen:** `under` is this frame's raw mean under the soles,
  before the ease. Well below `support`, the feet are over a drop; the
  playground fells a ragdolled walker on it.
- **The rays skip the character itself:** its ragdoll bodies and any
  proxy it has (`PhysicsGround::ignore`). They must, or the rays hit the
  character's own capsule and discard everything below it.
- **The body stands on what its feet stand on:** the probe answers the
  character's origin (within 6 cm, a frame's travel) with `support`, the
  mean height under the two soles, eased over 0.1 s.
  - Answered with the floor instead, the pelvis stayed down on a 0.15 m
    platform and the legs bent under it.
  - Eased over 0.15 s, the body still crouched with both feet already up.
- **The kinematic capsule** standing in for a far character starts at the
  step height (0.35 m), not the floor. Down to the floor, it shoved every
  prop out from under the feet before they reached it.

## Alternatives considered

- **Raycasting inside the probe:** impossible; the probe is a plain trait
  object with no world access.
- **A heightfield of the whole room:** wasteful, since only the ground under
  the feet matters, and props move.
- **Keeping the body on the floor:** rejected after the platform test above.

## Consequences

- **Measured on a 0.15 m platform** (`--plank 0.15`, ankles and hips per
  frame):
  - Stances on it at 0.236-0.238 m, the floor's 0.087 plus 0.15.
  - The hips rise from 0.935 to about 1.085 m (+0.15) and come back down
    after.
- **Cost, at 16 characters,** whole-frame wall time, vsync off, two runs
  each, median frame; ±1 ms between runs:

  | What | Extra per character per frame |
  |---|---|
  | The rays (~160 per character) | 0.09-0.11 ms |
  | A kinematic capsule | within noise (~0) |
  | A pinned ragdoll | 0.24-0.31 ms; at 16 ragdolls p99 rises from ~15 to ~25 ms |

- **One foot on a small prop lifts the body by half the prop's height.**
  It is the mean of the two soles: the max would leave the other leg
  unable to reach the floor (this rig stands near full extension).

## Revisit when

- Props are tall or uneven enough that a foot should step on a slope:
  the grid's normals are kept but not yet used to tilt the foot.
- Many characters crowd one area: the rays could be shared or done in one
  batched query.

## Related

- [Sample the ground in the world, not the pose](./sample-the-ground-in-the-world-not-the-pose.md) — prerequisite: the IK's ground queries are world points, which this grid answers.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — applies: the body's rise onto a platform is travel the locks must be told of.
- [Rig authored at critical extension](./rig-authored-at-critical-extension.md) — why the body follows the mean of the soles rather than the higher one.
- [Steer over terrain by the ground profile ahead](./steer-over-terrain-by-the-ground-profile-ahead.md) — applies: the steering matched to `max_step`, and the ledge falls read from `under`.
