---
title: A leap of faith is a ballistic swan dive flipped onto the back
description: "Step 12: from a top 2.5 m or more over a pile, a spring off the edge, the COM ballistic into the pile's middle; the swan's shape pitched whole into a dive, held, then on over a half flip to land on its back; it sinks to near the floor and rises out through the get-up's keys. Read before changing parkour/faith.rs."
type: decision
status: current
tags:
  - locomotion
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/faith.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::faith::tests::a_leap_of_faith_lands_on_its_back_in_the_hay"
  - "live: character_gallery --step-seconds 0.0333333 --start-height 6 --block 0,-0.3,0,6,1.0,0.6 --hay 0,-3,1,1.2 --faith-at 2, Xvfb, gizmos on/mesh off Left, mesh on Back and Left; BRP pelvis and neck"
  - "anim_bench --gait faith --characters 100"
aliases:
  - leap of faith
  - swan dive
  - haystack
  - Haystack
  - LeapOfFaith
  - Walker::leap_of_faith
---

# A leap of faith is a ballistic swan dive flipped onto the back

Step 12 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md):
Assassin's Creed's leap of faith into a pile of hay. There is no data;
the shapes and times are by eye on the game's.

## Decision

**The pile** (`Haystack`) is the middle of its top, a radius and a height
over the floor under it. It is not ground: the walker passes into it.

**The leap** (`LeapOfFaith`), from standing on a top 2.5 m or more over the
pile, which must be ahead:
- **Springing off** (0.35 s): the COM goes on a cubic from rest to its
  leaving point, 0.15 m out and 0.05 m up, at the leaving velocity. The
  pose blends from standing into the swan's shape and pitches forward
  0.3 rad.
- **The flight**: the COM is ballistic from leaving at 1.2 m/s up to just
  over the pile's middle, at most 4 m/s across. The swan's shape (arms
  spread a little forward and up, elbows straight, legs straight, toes
  pointed) is pitched whole about its hips:
  - into the dive, face down at π/2, by 0.55 of the flight;
  - held there;
  - then on over a half front flip to on its back (3π/2) at the landing.
  The root rides the COM.
- **Sinking**: the COM goes from the pile's top to 0.2 m over the floor
  under it on a cubic from the landing velocity to rest, over at least
  0.25 s, carrying on half its sideways way.
- **Hidden**: 0.6 s, lying sunk in it.
- **Rising out**: on the floor under it, the root where it lies, it goes
  through the get-up's face-up keys (sitting up, squatting), then standing,
  the legs blended by their feet.

**The walker** (`Walker::leap_of_faith`): standing, it turns on the spot to
face the pile, then leaps. Risen, it stands where the pile is.

## Alternatives considered

- **A ragdoll into the pile**: the pile is soft and no data on landing in
  hay exists. A planned path stays continuous and lands where it aims.
- **Turning over by a roll about the long axis** (prone to supine): the
  game turns the body over by a forward flip, and the flip keeps the dive's
  pitching motion going.
- **Landing on its back by the get-up's lying pose**: the swan pitched to
  3π/2 lies on its back already; the get-up's keys take it from there.

## Consequences

**Headless** (`puppet_base`; tops 6 and 10 m, piles 1 m high, 1.5-3 m
out):
- the flight's COM was ballistic within 0.009 m/s²;
- it landed with its chest straight up, 0.11-0.14 m off the pile's middle;
- sunk, the head and hips were under the pile's top;
- no joint went under the floor (2 cm for the toes);
- every pose was finite;
- outside the landing's impact, no step changed over 2.5 cm, nor any joint
  over 8.5 m/s about the hips;
- it ended standing on the floor under the pile;
- a pile only 1.5 m below, or behind, was not leapt into.

**Live** (30 fps step): from a 6 m block, it leant out, dived (the neck
ahead and below the pelvis), turned over (the neck behind it), landed in
the pile 0.4 m from its middle and sank to the floor, then rose out. From
behind on the mesh, the dive is a swan's, arms wide, legs together.

**Cost**: `anim_bench --gait faith --characters 100`, 27 µs a character at
p50.

## Revisit when

- **A pile not under the top's edge**: it springs from where it stands;
  walking to the edge first is not built.
- **The hay itself**: no straw thrown up, no dent; the pile is a drawn
  cylinder.
- **Climbing out to the pile's side**: it stands where the pile is.

## Related

- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 12's design.
- [Sitting down and standing up go through solved keys](../ik-and-locomotion/sitting-down-and-standing-up-go-through-solved-keys.md) — deeper: the get-up's keys and the blend by the feet it rises through.
- [A low obstacle is speed-vaulted as a reshaped running leap](./a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md) — same-trap: a shape turned whole about the COM keeps the flight ballistic.
