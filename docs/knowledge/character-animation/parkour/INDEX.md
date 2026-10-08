---
title: Parkour
description: "Parkour moves for a platformer on character::anim: ledges, mantles, landings from height, walls, bars, poles, ropes, beams and tight spaces. Read before starting or changing any of them."
type: index
status: current
tags:
  - character-animation
  - locomotion
  - ik
updated: 2026-10-07
---

# Parkour

The moves a Prince-of-Persia-style platformer needs, built as procedural
animation against the level's geometry. The design comes first; notes on
each move as it is built follow it.

| Note | What it establishes | Read when |
|---|---|---|
| [Parkour moves, step by step](./parkour-moves-implementation-design.md) | The step-by-step design: foundations shared with the jump and ladder, then ten steps from grabbing a ledge to crawling, each with its model, data, API, tests and done criteria | before starting or changing any parkour move |
| [A ledge is caught near the top of a jump and hung from](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) | Step 1: a standing jump up and in, the hands meeting the lip before its top; the arms' give and a pendulum about the grip; braced on the wall or free; hands hooked over the lip. Traps: grips in their own frame, the lip at the knuckle, the walk's stop, the approach's side entry | before changing `parkour/hang.rs`, `hand::hooked`, or any hold on an edge |
| [A hang is climbed up from by a pull, a press and a step on](./a-hang-is-climbed-up-from-by-pull-press-and-step-on.md) | Step 2: the hips on a C² spline through shapes measured from the hands (pull, turn over, press folded over the top, step on, stand); feet up the face then over; a mid-swing ask waits. Traps: a knee's arc round its socket into the corner, a thigh turned round at the hip, the hips joint vs the root translation | before changing `parkour/hang/up.rs`, or any move bringing a leg up over an edge |
| [A ledge is shimmied hand over hand, and round corners](./a-ledge-is-shimmied-hand-over-hand-and-round-corners.md) | Step 3: hand over hand, pulled up, the hips on the hang's sideways spring; at a corner the hands match up and the body rides the one rigid motion between the two hangs. Traps: the facing turn composed twice (hidden by half turns), feet measured off the face only, per-kind corner timing | before changing `parkour/hang/shimmy.rs`, or turning a body at a corner |
| [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) | Step 4, first part: walking off an edge falls under gravity and lands as measured (time and knee depth keyed by drop height, braking `v(1-s)^n(1+n·s)`). Traps: the study's touchdown speeds are not free fall's, a gap measured a frame early, the ground snap hiding drops | before changing `parkour/fall.rs`, any landing, or the walker's ground snap |
| [A hang is dropped into, let go of, and caught from a fall](./a-hang-is-dropped-into-let-go-of-and-caught-from-a-fall.md) | Step 5: dropping down runs the climb-up backward, slower; letting go falls, onto a step under the feet if there is room; a jump over an edge falls on at its centre of mass's velocity; any fall catches a ledge in reach, swept over the frame. Traps: a frame-end-only catch, the hips not ballistic in a jump, the root over the edge before take-off, overlapping blocks | before changing the hand-offs between `parkour/hang.rs`, `hang/up.rs`, `fall.rs` and the walker's jump |
| [A fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md) | A fall has no collision: facing a wall that reaches the ground, its hips stop 0.25 m off (a 0.2 m give, about 2.3 g), its landing's room is planned over time, its hands and toes are kept off, the hurt lean dropped; a hang caught there keeps hips and knees out | before changing a fall's path, a landing near a wall, or a caught hang's legs |
| [Walking is kept out of walls](./walking-is-kept-out-of-walls.md) | The walker had no collision: ground read from above higher than a step pushes the body (0.2 m) back out, sliding along; a wall ahead is gone round by the least clear turn, or stopped at in a dead end; costs linear in the level's ledges | before changing how the walker moves its root, or adding many blocks |
| [Parkour movement data](./parkour-movement-data.md) | Measured human data per move (landings from 0.75-2.7 m, wall climbs and runs, bar swings, beams, crawling) and the moves with none (ledge catch, braced hang, shimmy, climb-up timing) | before timing or shaping a parkour move |

## See also

- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — the holds model, hand grips and shoulder lift these moves build on.
- [A jump is planned as its centre of mass's path](../ik-and-locomotion/a-jump-is-planned-as-its-centre-of-mass-path.md) — the COM-path planning every airborne move uses.
