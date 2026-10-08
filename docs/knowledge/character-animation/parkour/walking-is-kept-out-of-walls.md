---
title: Walking is kept out of walls
description: "The walker had no collision: run on after landing at a wall's foot, it went through the block. Ground read from above higher than a step stops the body 0.2 m off (keep_off_walls, sliding along), and a wall ahead within the stopping distance stops the walk. Read before changing how the walker moves its root."
type: decision
status: current
tags:
  - locomotion
  - correctness
updated: 2026-10-08
verified: 2026-10-08
code:
  - src/character/anim/walker.rs
sources:
  - "test walker::tests::walking_is_kept_out_of_walls"
  - "live: character_gallery --block 0,-8,0,3.0,2.0,12 --side-ledge 0,-10.5,180,3.0,2.0 --start-height 3.0 --anim-speed-schedule 1:3.0; --block 0,-1,180,2.25,2.0,1.0 --hang-at 1 --climb-up-at 18"
aliases:
  - keep_off_walls
  - BODY_RADIUS
  - wall collision
  - walking into a wall
---

# Walking is kept out of walls

The walker's root went wherever the gait carried it. Nothing stopped it at
a wall: having fallen off a top and landed at the foot of a wall in front,
it ran on straight through the block. Walking (not hanging, climbing,
falling or jumping, which place the body themselves) is now kept out of
walls.

## Decision

**A wall is ground more than a step higher** (`parkour::fall::STEP_DOWN`,
0.3 m) than the ground the root stands on. The ground is read from far
above each point, so the probe gives the highest top there, whatever its
own reach below a top.

**The body is a circle of 0.2 m round the root** (`BODY_RADIUS`, less than
the 0.25 m the hips stand off a wall faced), probed 16 ways round
(`keep_off_walls`, in `ride_rendered_feet`):
- a move whose circle would reach a wall loses its part into the wall, the
  way out being away from the probes that hit;
- if it still reaches one, it does not move.

**It stops walking for a wall straight ahead** within its stopping distance:
0.2 m, plus 0.15, plus 0.3 s at its speed (`drive_walkers`). Stopped only
once held at the wall, it set off again each time it stood clear of it,
and shuffled in place. Walking to a spot by a wall (a ledge's, a ladder's,
a chair's) is exempt, because that approach ends nearer.

## Alternatives considered

- **Physics colliders for the walker**: the walker is kinematic, its root
  moved by the rendered feet. A probe of the ground it already reads is
  enough for the level's blocks.
- **Stopping when held**: the shuffle above.

## Consequences

- **Headless**: walking into a 3 m block's face, it stops 0.2 m off it,
  held; diagonally it slides along the face; away it goes free; on the
  block's top, walking to its edge, nothing holds it.
- **Live**: run off a 3 m top, landed at a far wall's foot, it stands
  there, the pelvis 0.29 m off the wall, after one small step. A ledge
  grab, the climb up, and standing on the top all still work.
- A step up of 0.3 m or less is walked onto as before.

## Revisit when

- **Steering round a wall**: it stops; it does not choose a way round.
- **A wall at a run**: the stop is as quick as the gait's own stopping.

## Related

- [A fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md) — contrast: the same walls kept off by a fall, with a planned room for its landing.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — context: the ground snap that turns walking off a top into a fall, the other end of the same root move.
