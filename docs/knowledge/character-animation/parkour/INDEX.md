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
| [Parkour movement data](./parkour-movement-data.md) | Measured human data per move (landings from 0.75-2.7 m, wall climbs and runs, bar swings, beams, crawling) and the moves with none (ledge catch, braced hang, shimmy, climb-up timing) | before timing or shaping a parkour move |

## See also

- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — the holds model, hand grips and shoulder lift these moves build on.
- [A jump is planned as its centre of mass's path](../ik-and-locomotion/a-jump-is-planned-as-its-centre-of-mass-path.md) — the COM-path planning every airborne move uses.
