---
title: A ledge is shimmied hand over hand, and round corners by one rigid motion
description: "Hanging::shimmy: hand over hand along the lip, pulled up a little, the hips following on the hang's sideways spring; at a corner the hands match up, then the body is carried by the one rigid motion between the two hangs. Read before changing hang/shimmy.rs or any move turning a body at a corner."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-07
verified: 2026-10-07
code:
  - src/character/anim/parkour/hang/shimmy.rs
  - src/character/anim/parkour/hang.rs
  - src/character/anim/parkour/geometry.rs
  - src/character/anim/walker.rs
sources:
  - "tests parkour::hang::shimmy::tests::* (a 2.15 m wall; a 1.6 x 1.0 m block's corner; an inside corner; braced and free)"
  - "live: character_gallery --hang-at 1 --shimmy-at 12,right,3 [--block 0,-1,180,2.25,1.6,1.0], Xvfb, BRP"
aliases:
  - Shimmy
  - HangAsk::Shimmy
  - set_others
  - Ledge::joined
  - Ledge::block
  - shimmy
  - hanging traverse
  - corner
---

# A ledge is shimmied hand over hand, and round corners by one rigid motion

Hanging from a ledge (step 1), a walker asked `HangAsk::Shimmy(Left|Right)`
goes along it hand over hand while asked, and round a corner onto another
ledge meeting its end (`Walker::ledges`, `Ledge::joined`). It is step 3 of
the [parkour design](./parkour-moves-implementation-design.md).

## Decision

**A step** (1 s, 0.25 m): the lead hand (on the side it goes) comes 5 cm up
off the lip and hooks on a stride along; the grip's middle moves along from
the start of the step; the trail hand follows to a shoulder's width behind
the lead. Braced, each foot steps along the wall in turn. One hand always
holds; the hands never cross. There is no hanging-traverse data
([movement data](./parkour-movement-data.md)); the timing is set by eye.

**The hips follow on the hang's sideways spring**: moving the grip's middle
keeps their offset from it, so they lag and catch up as a hanging body does,
and free legs swing a little with them.

**Shimmying, it is pulled up 6 cm** from the hang's length (the arms' give
spring), and a step starts only once it is up. Braced feet rise with it.

**At an edge's end** with no ledge meeting it, it stops, the hands 8 cm short
of the end.

**At a corner** (a ledge meeting the end at its height):
1. it comes to 0.12 m from an outside corner, 0.3 m from an inside one;
2. the trail hand comes up to the lead, 0.12 m apart;
3. the corner step (2.4 s): the lead hand goes out round the corner onto the
   next ledge, the trail hand follows, the palms turning to the new face;
   the body, face axes and facing turn by the one rigid motion carrying the
   hang on the first face to the hang on the next. In the plane that is a
   rotation about its fixed point, found from the two grips. Braced feet
   come off the wall and plant again on the new face; round an inside
   corner the elbows tuck back;
4. the next steps spread the hands to a shoulder's width again.

Outside and inside corners time their hands differently (see the trap
below).

## Alternatives considered

- **Turning about the corner itself**: right for neither. At an inside
  corner it drives the body into the side wall.
- **Going round with the hands a shoulder's width apart**: the trail hand,
  left on the first face, was 17 cm out of reach as the body turned.
- **Carrying the braced feet round on the wall**: their toes went 6.5 cm into
  the block by the corner.
- **Starting the first step at once**: the lead hand was 3.3 mm short in the
  air before the body was up.

## Traps

- **The facing turn composed twice.** A foot's ankle-from-ball offset was
  `attitude · attitudes⁻¹ · (turn · ankles)`, where `attitude` already
  carries `turn`. On every wall faced so far the facing was a half turn,
  which squares to none, so it never showed; after a corner's quarter turn
  the ankle went 14 cm along the face. It dates from step 1. The test
  `braced_feet_are_on_their_holds_on_a_wall_facing_any_way` fails on it (a
  foot 20 cm off facing +X) and passes on +Z.
- **A foot measured off the face only**: a toe 20 cm from its hold, dragged
  along the face, passed. Measure the ball against its hold.
- **A block measure without its far side**: a toe on a block's front read a
  whole block deep from its back face.
- **The lead hand put nearest the corner** on the next ledge crossed the
  hands: the lead goes on along the new edge.
- **Hand timing**: outside and inside corners pull opposite ways. Round an
  inside corner the trail hand must go before the body has turned past
  about a third (else it was 7 cm out of reach, hanging free); round an
  outside one that early, the lead hand was 1.2 cm out of reach. Each has
  its own timing.
- **Too near an inside corner to turn**: grabbed with the lead hand 0.19 m
  from it, the shoulder was against the side wall. It stops there.

## Consequences

**Headless** (`puppet_base`, 2.15 m; braced and free; left and right; a
block's corner and an inside corner):
- a hand always on the lip; held and moving wrists within 0.1 mm of their
  hooks and ways;
- nothing inside any block;
- braced feet on their holds;
- the hips at most 0.94 m/s² along a wall and 1.8 round a corner, and at
  rest once stopped;
- three steps go 0.75 m; round a corner it ends square to the new face,
  turned ±90°.

**Live** (BRP): along a wall the hands step 0.25, 0.49, 0.75, 0.99 m along
the lip, the hips following. Round a 1.6 m block's corner the lead hand
stops 0.12 m short of it and the trail hand comes up to 0.13 m behind; on
the side face the wrists are 1 cm out from it, the pelvis 0.40 m, a braced
ball on it, the hands spread to 0.44 m again. Seen from above, the wrists
look to be over the top: perspective, which BRP settled.

**Cost** (`anim_bench --characters 20`, each frame posed from the start):
48 µs a character a frame shimmying, 51 round a corner (hanging 46).

## Revisit when

- **The move controller arrives**: it asks the way from input, and turns a
  shimmy into a climb up or a drop.
- **Corners other than right angles**: `joined` takes any turn over 0.5 rad,
  but only right angles are tested.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is step 3 of.
- [A ledge is caught near the top of a jump and hung from](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) — prerequisite: the hang, its grips and its sideways spring that the shimmy moves.
- [A hang is climbed up from by a pull, a press and a step on](./a-hang-is-climbed-up-from-by-pull-press-and-step-on.md) — applies: it waits for a step to finish before climbing up.
- [Parkour movement data](./parkour-movement-data.md) — context: why the traverse is timed by eye.
