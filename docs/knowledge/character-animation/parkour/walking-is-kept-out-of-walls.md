---
title: Walking is kept out of walls
description: "The walker had no collision. Ground read from above higher than a step pushes the body (0.2 m circle) back out, sliding along (keep_off_walls); a wall ahead is gone round by the least clear turn, or stopped at in a dead end (way_round). Costs linear in ledges. Read before changing how the walker moves its root."
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
  - "tests walker::tests::{walking_is_kept_out_of_walls, walking_goes_round_a_wall}"
  - "anim_bench --gait walk --characters 100 --walls N [--walls-away]"
  - "live: character_gallery --block 0,-8,0,3.0,2.0,12 --side-ledge 0,-10.5,180,3.0,2.0 --start-height 3.0 --anim-speed-schedule 1:3.0; --block 0,-1,180,2.25,2.0,1.0 --hang-at 1 --climb-up-at 18"
aliases:
  - keep_off_walls
  - way_round
  - BODY_RADIUS
  - walking round a wall
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
the 0.25 m the hips stand off a wall faced; `keep_off_walls`, in
`ride_rendered_feet`). It is moved, then pushed back out of any wall it
reaches, up to four rounds. Each push goes along the probe that found the
wall, by how far in the wall starts along it (bisected). There are 32
probes, every other one tried first: clear of those 16, the rest are not
looked at (a corner can come 0.4 cm in between them). It slides along
walls and round corners, and stops only square on.

**With a wall ahead within its stopping distance it goes round it**
(`way_round`, in `drive_walkers`). The stopping distance is 0.2 m, plus
0.15, plus 0.3 s at its speed. It turns off its way by the least angle
that is clear, in 15° steps up to a quarter turn, and keeps to the side it
chose. A turn counts only if clear for 1.5 m (`DETOUR_CLEAR`) with the
body's width swept along it. It walks along the wall, and past its end its
way is clear again, so it turns back onto it, on a parallel line. Steered
toward a facing, that is its way; else, the facing it had when it turned
off. With no clear way within a quarter turn (a dead end), it stops.
Walking to a spot by a wall (a ledge's, a ladder's, a chair's) is exempt,
because that approach ends nearer; so is walking a circle.

## Alternatives considered

- **Physics colliders for the walker**: the walker is kinematic, its root
  moved by the rendered feet. A probe of the ground it already reads is
  enough for the level's blocks.
- **Stopping when held**: stopped only once held at the wall, it set off
  again each time it stood clear of it, and shuffled in place.
- **Cutting the move to its part along the wall** (the way out from the
  probes that hit): at a block's corner that way out was 22° off, sliding
  along it came nearer the corner, the move was refused, and it stuck.
- **Turns cleared by a line from the body's middle**: one cleared a
  corner the body could not pass, and it stuck there.
- **Any clear turn**: in a dead end the turn clear only to a side wall
  sent it back and forth.
- **Checking 8 probes first**: a corner came 2.7 cm in between them.

## Traps

- **A box grown by the body is not the body's reach** at a corner: a test
  using one reported a penetration where the round body was clear.

## Consequences

- **Headless**: walking into a 3 m block's face, it stops 0.2 m off it,
  held; diagonally it slides along the face; away it goes free; on the
  block's top, walking to its edge, nothing holds it.
- **Headless, going round** (a point walker turning at 3 rad/s): straight
  at a 2 m block at 1.4 m/s it goes round it, past it, facing its way
  again; along a 40 m wall it follows it; in a dead end it stops; never into
  a wall (1 cm).
- **Live**: run off a 3 m top, landed at a far wall's foot, it stands
  there, the pelvis 0.29 m off the wall, after one small step. Walking
  straight at a 2 m block, it turns off, passes along its side 1.23 m out
  (its edge at 1.0) and walks on along its way. A ledge grab, the climb up,
  and standing on the top all still work.
- A step up of 0.3 m or less is walked onto as before.
- **Cost** (`anim_bench --gait walk --characters 100 --walls N`, on top of
  the walk's 24 µs a character a frame): linear in the ledges, because
  `LedgeGround` samples every ledge.

| Blocks (ledges) | Open floor (`--walls-away`) | Held at a wall |
|---|---|---|
| 1 (4) | 1.0 µs | 5.3 µs |
| 10 (40) | 6.6 µs | 34 µs |
| 50 (200) | 31 µs | 161 µs |

## Revisit when

- **A level with many blocks**: `LedgeGround` needs a spatial index; the
  cost grows with every ledge in the level, not the ones nearby.
- **Steering back onto its line**: it goes on along a parallel line past
  a block, not back to the line it was on.
- **A wall at a run**: the stop is as quick as the gait's own stopping.

## Related

- [A fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md) — contrast: the same walls kept off by a fall, with a planned room for its landing.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — context: the ground snap that turns walking off a top into a fall, the other end of the same root move.
