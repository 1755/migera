---
title: Steer over terrain by the ground profile ahead, not a ray at one height
description: "A walker decides what it can walk onto from downward rays every 0.2 m ahead (rise ≤ 0.3 m between samples, slope ≤ 40°), measured from the ground under it, not its eased height. A horizontal ray hit risers and ramps like walls; the eased height turned a walker back off a 35° ramp. Read before steering characters over stairs, ramps or ledges."
type: decision
status: current
tags:
  - locomotion
  - physics
  - correctness
updated: 2026-10-02
verified: 2026-10-02
code:
  - examples/physics_character_playground.rs
  - src/character/anim/physics_ground.rs
sources:
  - "live: physics_character_playground --physics ragdoll --props 0 --trace-feet --start X,Z,YAW, one run per ramp, the stair and each fence"
  - "live BRP: --characters 16 --physics ragdoll --seed 11, every pelvis sampled 13 times over 60 s"
aliases:
  - turn_from_terrain
  - blocked_along
  - terrain steering
  - walkable slope
  - ledge fall
---

# Steer over terrain by the ground profile ahead, not a ray at one height

A walker tells what it can walk onto from the ground's height profile
along its way: one ray straight down every 0.2 m out to 2 m ahead. It is
blocked by a rise of more than 0.3 m between two samples, or by a slope
steeper than 40°, up or down. A drop does not block it: it steps off and,
with a ragdoll, falls.

## Context

The playground's walkers turned from walls with a horizontal ray at chest
height. The room gained ramps of 15° to 50°, a 5 m stair with 167 mm
risers, and fences 0.1 to 1.6 m high. Some of these are climbable and some
are not, and a ray at any one height cannot tell which. Low enough to
catch a 0.4 m fence, it also met a stair's riser and a gentle ramp head-on.

## Decision

`blocked_along` in `examples/physics_character_playground.rs` works as
follows:

- **Samples.** One downward ray every 0.2 m, starting 4 m above the last
  sample's height, against the terrain's own collision layer (props never
  steer). The spacing is under a tread (0.3 m), so no two risers fall
  between two samples.
- **Blocks.**
  - A rise of more than 0.3 m between samples. That is under the feet's
    own `PhysicsGround::max_step` (0.35 m), so a foot always stands on
    whatever the walker walks onto.
  - A hit normal more than 40° from vertical, uphill or downhill.
  - Past a wall.
- **The face it reflects off.** Going up, a ray across just above the
  lower sample hits the face. Going down a steep slope, the face is the
  slope's normal turned back toward the walker.
- **Ledges.** `PhysicsGround::under` is the raw mean under the soles. When
  it is more than 0.45 m below `support`, a ragdolled walker is told to
  fall (`Walker::fall_now`). That threshold is one foot over a 1 m drop,
  never a riser or a slope's step.

## Two traps it hit

- **Measure rises from the ground under the walker, not its height.** The
  walker's height is the eased (0.1 s) mean under both soles. Halfway up
  the 35° ramp it lagged 0.18 m below the ground under the body. The
  first sample read as a 0.32 m riser, and the walker U-turned off the
  ramp's side. The probe now starts from a ray down at the walker's own
  position.
- **A solid that is hollow underneath gives a ray across a wrong face.** A
  ramp built as a tilted 0.3 m slab left a hollow under it. The ray across,
  0.1 m above the floor, passed under the slab and hit its underside, whose
  normal faces away. The walker was reflected into the ramp and walked
  through it at floor height. Ramps are now solid wedges (a convex hull of
  an extruded triangle).

## Alternatives considered

- **A horizontal ray at a fixed height:** rejected, for the reasons in
  Context.
- **A ray at `support + max_step` plus a normal test:** a riser's face is
  vertical, so stairs would still block.
- **Blocking drops too:** rejected. Stepping off and falling is what the
  playground tests, and a walker with nowhere to fall is stuck on a
  platform.

## Consequences

Measured live, ragdoll mode, one walker started at each structure:

| Structure | Highest support | Outcome |
|---|---|---|
| Ramps 15°/25°/35° | 1.00 / 1.50 / 2.00 m (the platform) | climbs, steps off the platform, falls, gets up, walks on |
| Ramps 45°/50° | 0.31 / 0.27 m | turned at the foot |
| Stair, 30 × 167 mm | 5.00 m | climbs, steps off the landing, falls 5 m, gets up |
| Fences 0.1/0.2 m | 0.10 / 0.20 m | steps over |
| Fences 0.4/0.8 m | 0 | turned |

- 16 ragdolled walkers with props over 60 s: 0 of 208 pelvis samples were
  outside the room or below the terrain under them, and there were 5 falls
  and 5 get-ups.
- **Kinematic (far) walkers** have no body to fall. Off a 1.5 m platform
  they sink to the floor in about 0.3 s, at the ease of `support`.
- **Tight pockets trap the steering.** A walker turns at 2.5 rad/s, about a
  0.5 m radius at 1.2 m/s, and takes no new turn until one completes.
  Against a wall, a ramp with a 1 m gap beside it caught a walker that
  fell into the gap, and it walked through the platform and the wall. The
  structures stand 2-2.5 m apart and from the walls for this reason.
- **Cost:** not measured separately. It is up to 31 rays per walker per
  frame, against the 160 of `PhysicsGround` (about 0.1 ms per character).

## Revisit when

- Walkers need paths rather than reflections (to aim for a ramp, say): a
  navmesh, not more probes.
- Structures must stand closer together than about 2 m: the steering
  needs to re-check while it turns.

## Related

- [Feet stand on the physics world through ground sampled under them](./feet-stand-on-the-physics-world-through-sampled-ground.md) — prerequisite: `support`, `under` and the relative `max_step` this steering is matched to.
- [Sample the ground in the world, not the pose](./sample-the-ground-in-the-world-not-the-pose.md) — same-trap: a ground query taken from the wrong reference height.
