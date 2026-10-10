---
title: Walking is kept out of walls
description: "The walker had no collision. Ground higher than a step pushes the body (0.2 m circle, and each shoulder 0.08 m) back out, sliding (keep_off_walls); a wall ahead is gone round by the least clear turn and back to its line, or stopped at (way_round); a 1 m grid. Read before changing how the walker moves its root."
type: decision
status: current
tags:
  - locomotion
  - correctness
updated: 2026-10-10
verified: 2026-10-10
code:
  - src/character/anim/walker.rs
  - src/character/anim/parkour/geometry.rs
  - src/character/anim/ground.rs
sources:
  - "tests walker::tests::{walking_is_kept_out_of_walls, walking_goes_round_a_wall, the_shoulders_are_kept_out_of_walls}, parkour::geometry::tests::the_grid_finds_what_every_ledge_finds"
  - "anim_bench --gait walk --characters 100 --walls N [--walls-away]"
  - "live: character_gallery --block 0,-8,0,3.0,2.0,12 --side-ledge 0,-10.5,180,3.0,2.0 --start-height 3.0 --anim-speed-schedule 1:3.0; --block 0,-1,180,2.25,2.0,1.0 --hang-at 1 --climb-up-at 18"
aliases:
  - keep_off_walls
  - way_round
  - back_to_line
  - LedgeGround grid
  - BODY_RADIUS
  - SHOULDER_RADIUS
  - keep_shoulders_off
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

**A wall is something solid from a step higher** (`parkour::fall::STEP_DOWN`,
0.3 m) than the ground the root stands on **up to the body's headroom**
(`HEADROOM`, 2 m): `GroundProbe::blocks`. By default that is the ground
read from far above each point, so the highest top there, whatever its own
reach below a top. `LedgeGround` answers from each block's own extent (its
top down to its wall), so a bar 2.3 m up is walked under. Read from above,
the walker routed round it instead of under it to the spot to grab it
([a bar is swung on](./a-bar-is-swung-on-pumped-and-let-go-of-at-a-bar-ahead.md)).

**The body is a circle of 0.2 m round the root** (`BODY_RADIUS`, less than
the 0.25 m the hips stand off a wall faced; `keep_off_walls`, in
`ride_rendered_feet`). It is moved, then pushed back out of any wall it
reaches, up to four rounds. Each push goes along the probe that found the
wall, by how far in the wall starts along it (bisected). There are 32
probes, every other one tried first: clear of those 16, the rest are not
looked at (a corner can come 0.4 cm in between them). It slides along
walls and round corners, and stops only square on.

**Its shoulders are kept 0.08 m off a wall too** (`SHOULDER_RADIUS`,
`keep_shoulders_off`, after the body's push). The shoulders stand about as
far either side as the circle reaches. Side-on at the circle's 0.2 m,
turning to face the wall, the outer one swept 2 cm into it, and its arm
hung 8 cm in. Each is a small circle of probes: every fourth first, the
rest only if one of those is in a wall. It pushes the root straight out of
the nearest face by what it lacks. The body is in effect wider across the
shoulders than front to back.

**With a wall ahead within its stopping distance it goes round it**
(`way_round`, in `drive_walkers`). The stopping distance is 0.2 m, plus
0.15, plus 0.3 s at its speed. It turns off its way by the least angle
that is clear, in 15° steps up to a quarter turn, and keeps to the side it
chose. A turn counts only if clear for 1.5 m (`DETOUR_CLEAR`) with the
body's width swept along it. It walks along the wall, and past its end its
way is clear again. Then it goes back to its line, the line through where
it turned off (`Detour::from`, `back_to_line`): it heads for the point on
it 1.5 m ahead, at most 45° off its way, if that heading is clear too. It
is done within 5 cm of the line and 0.01 rad of its way, with its way set
as the facing to hold. Steered toward a facing, that is its way; else, the
facing it had when it turned off. With no clear way within a quarter turn (a dead end), it stops.
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

**The ground finds tops through a grid** (`LedgeGround::new`): 1 m squares
across the floor, each listing the ledges whose top's bounds reach into
it, in Bevy's map (a fast hash). A sample looks only at its own square's
ledges. Looking at every ledge, the wall checks cost grew with the level:
161 µs a character a frame among 50 blocks.

## Traps

- **A box grown by the body is not the body's reach** at a corner: a test
  using one reported a penetration where the round body was clear.
- **Ending the way back at the last heading for the line**: that heading
  is a little off the way (about 2° at 5 cm off), and walking straight
  held it, 0.13 m off the line in 13 m. On ending, its way is set as the
  facing.
- **The standard library's map** made the grid slower than looking at
  every ledge of one block (10 µs held at a wall against 5); Bevy's map
  took it to 7.

## Consequences

- **Headless**: walking into a 3 m block's face, it stops 0.2 m off it,
  held; diagonally it slides along the face; away it goes free; on the
  block's top, walking to its edge, nothing holds it.
- **Headless, going round** (a point walker turning at 3 rad/s): straight
  at a 2 m block at 1.4 m/s it goes round it, back onto its line within
  5 cm, facing its way within 0.05 rad; along a 40 m wall it follows it; in
  a dead end it stops; never into a wall (1 cm).
- **Headless, the grid**: among 40 blocks of assorted sizes, headings and
  heights, it finds what looking at every ledge finds, at 20 000 points.
- **Live**: run off a 3 m top, landed at a far wall's foot, it stands
  there, the pelvis 0.29 m off the wall, after one small step. Walking
  straight at a 2 m block, it turns off, passes along its side 1.2 m out
  (its edge at 1.0), is back on its line 5 m past it, and stays on it
  (1-2 cm over the next 16 m). A ledge grab, the climb up, and standing on
  the top all still work.
- A step up of 0.3 m or less is walked onto as before.
- **Headless, overhead**: a bar 2.3 m up across the way is walked straight
  under; one 1.5 m up is gone round.
- **Headless, shoulders**: kept 0.2 m off a wall, turned from facing it to
  side-on a sixteenth of a quarter at a time, the nearer shoulder never
  under 0.08 m from the face, pushed straight out by no more than it
  needs; facing it, not pushed (`the_shoulders_are_kept_out_of_walls`).
  Live, walking up to a hold wall 0.6 m ahead (the approach loops along
  it): no hand comes nearer the face than 1.6 cm, where one went 11 cm in.
- **Cost** (`anim_bench --gait walk --characters 100 --walls N`, on top of
  the walk's 24 µs a character a frame): flat in the level's size through
  the grid. With the shoulders and the hands kept out too (2026-10-10):
  1.4 µs on open floor, 8.2 µs held at a wall (1 block) and 8.9 µs (10).
  The probes on open floor were 2 µs until they tried a coarse ring first.

| Blocks (ledges) | Open floor (`--walls-away`) | Held at a wall | Without the grid, held |
|---|---|---|---|
| 1 (4) | 0.7 µs | 7.3 µs | 5.3 µs |
| 10 (40) | 0.7 µs | 8.4 µs | 34 µs |
| 200 (800) | 0.6 µs | 8.5 µs | (161 µs at 50) |

## Revisit when

- **Ledges far longer than a cell**: one 40 m ledge is listed in 40+
  cells; a sample is still one cell, but building the grid is per cell.
- **A wall at a run**: the stop is as quick as the gait's own stopping.

## Related

- [A fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md) — contrast: the same walls kept off by a fall, with a planned room for its landing.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — context: the ground snap that turns walking off a top into a fall, the other end of the same root move.
- [A hand rests on a wall beside the body](./a-hand-rests-on-a-wall-beside-the-body.md) — same-trap (2026-10-10): the arms the body's circle does not keep out; any hand in a wall is swung out of it after the springs.
