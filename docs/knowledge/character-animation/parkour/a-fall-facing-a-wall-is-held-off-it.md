---
title: A fall facing a wall is held off it
description: "Falling has no collision: facing a wall (Falling::against) its hips stop 0.25 m off over a 0.2 m give, the landing's room is planned over time, hands and toes kept off, no hurt lean; a hang caught there keeps hips and knees out. Read before changing a fall's path, a landing near a wall, or a caught hang."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-08
verified: 2026-10-08
code:
  - src/character/anim/parkour/fall.rs
  - src/character/anim/parkour/hang.rs
sources:
  - "tests parkour::hang::tests::{a_jump_falling_short_catches_the_far_ledge, letting_go_over_a_step_lands_on_it_or_clears_it}"
aliases:
  - Falling::against
  - Falling::facing_wall
  - HIPS_OFF_WALL
  - wall room
  - falling into a wall
---

# A fall facing a wall is held off it

A `parkour::Falling` flies on a planned path with no collision. Fallen short
of a wall in front (a missed jump), or let go onto a step under the wall it
hung from, it flew on into the wall: shoulders 18 cm in, a head 34 cm in
landing. A fall that faces a wall is now told of it (`Falling::against`, or
`facing_wall` directly) and kept off it.

## Context

Which wall counts (`Falling::against`, from the walker's ledges):
- it faces it within 60°;
- it starts in front of the wall's face;
- it would come to rest within 1 m of it (`WALL_NEAR`);
- the wall's top stands more than a step above where it lands;
- its face reaches down to that ground. A slab's short face is not a wall
  it falls out from under.

## Decision

**The hips stop 0.25 m off the face** (`HIPS_OFF_WALL`: standing facing a
wall, the hips are about that far off it). They slow from 0.2 m further out
(`WALL_GIVE`), as if the arms took the impact, and come to rest without
penetrating. The stop is the C¹ soft clamp `(x+s)²/4s` on the distance
past the stop. At 3 m/s it brakes at about 2.3 g; over 0.1 m it was 7.5 g.

**Landing, the hips come back as far as the landing's shape needs.** The
room is planned once, when the wall is set:
1. every 10 ms from touchdown, how far any joint but the arms reaches ahead
   of the hips, plus 5 cm;
2. kept at its greatest so far;
3. held to its greatest over twice 0.08 s ahead, then averaged over 0.08 s
   either side, twice.

Because the need only grows, the smoothed room never falls short of it, and
it is C¹. A box average once left kinks that pushed the hips at 6 g. The
rest spot and the planted feet use the final room. In the air, the stop is
the flight's 0.25 m: a braced hang's feet, on the wall, are ahead of the
hips by 0.42 m from the first frame.

**The hands, toes and knees are kept off its face under its top.** The
wrists are held 0.1 m off it, eased in as the arms are let go. Each arm is
solved toward the shifted wrist with its own elbow plane. The ankles move
out as far as the levelling toes would go in, and in the air as far as a
knee would come within 2 cm. A running jump's legs reach ahead for its
landing; the hips held off, a knee went 5 cm in.

**A hurt landing against a wall does not lean over onto its hands.** It
keeps the deeper knees and the hold. The room for the lean (0.65 m) pushed
the hips back off the wall while still in the air, out of a ledge's reach.
It does not roll either.

**A braced hang caught near the wall**:
- each foot goes only as far onto the wall as keeps its knee 1 cm out,
  found by bisection on the blend, more as the hips swing out; caught with
  the knee in already (a running jump's leg reaching ahead, 15 cm), the
  foot moves out as far;
- the braced swing cannot bring the hips nearer than 0.15 m. At 0.11 m a
  shoulder touched the wall.

## Alternatives considered

- **Collision against the level's geometry**: the ledges are the level here.
  A wall known from them is enough for the fall's one plane in front.
- **A fixed landing distance ramped over the landing**: too slow. The hurt
  lean put the head 17-22 cm in before the hips got back.
- **Pushing the hips back as far as the hurt lean needs**: it pushed them
  0.4 m off the wall in the air, before the catch, and shoulders passing
  the lip 0.47 m out missed it.

## Consequences

- **A missed jump**, not reaching, from 3 m: it lands against the wall,
  braked and pushed back at most 1.9 g. No joint goes into the wall or
  under the floor.
- **A running jump** slamming into it at 4.5 m/s stops at 3.6-3.9 g, about
  4.5² / 2 over the 0.2 m give either side, a slam at a run. It is bounded,
  not softened.
- **Caught** at the wall (reaching): the hang settles, held within 1 mm,
  nothing into the wall.
- **Let go onto a step** under its wall: it lands standing 0.13 m off the
  wall. Aiming the rest spot runs through the wall's stop, a few rounds of
  correcting by the miss.
- **Cost**: `anim_bench --gait let-go` went from 19 to 25 µs a character a
  frame. That is the soft stop and the hand and toe checks each frame.

## Revisit when

- **A wall at an angle**, or two walls (a corner): one plane only.
- **The hands placed on the wall** landing against it, instead of only kept
  off.

## Related

- [A hang is dropped into, let go of, and caught from a fall](./a-hang-is-dropped-into-let-go-of-and-caught-from-a-fall.md) — prerequisite: the falls (a missed jump, a let-go) that need the wall.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — contrast: the landing and the roll's planned resting height, smoothed the same way.
