---
title: Any wall is climbed on holds grown from its roughness, the climb kept clear of where an elbow cannot be solved
description: "Step 14: a rough wall's holds are a hashed, jittered grid from its seed and roughness, climbed by the step-13 climber. Irregular holds found climber faults: an elbow flip at its pole (moves kept clear), the fingers' sideways flip, a get-on sweep into the wall. Read before changing HoldWall::rough or the climber's arms."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/holds.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::holds::tests::{a_rough_wall_grows_the_same_holds_from_the_same_seed, rough_walls_are_climbed_to_the_top, a_wall_of_holds_is_climbed_limb_by_limb_every_way}"
  - "anim_bench --gait free-climb, before and after"
aliases:
  - climbing any wall
  - rough wall
  - procedural holds
  - HoldWall::rough
  - elbow pole singularity
  - hairy ball
---

# Any wall is climbed on holds grown from its roughness, the climb kept clear of where an elbow cannot be solved

Step 14 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md): any wall is climbable,
without holds placed by hand.

## Decision

**A rough wall** (`HoldWall::rough(face, out, width, height, roughness,
seed)`) grows its own holds:
- **A jittered grid**, one hold per cell, each up to 0.4 of a cell off the
  cell's middle, so no two are nearer than a fifth of a cell.
- **Each cell's place and kind are drawn from a hash** of the seed and the
  cell (SplitMix64's finaliser, integers only). The same wall always has
  the same holds, on any machine, and nothing is stored but the wall's own
  numbers.
- **Roughness 0-1 sets the density**: the cells go from a 0.4 × 0.3 m
  climbing wall's to 0.65 of it, and the share of holds that take a hand
  goes from 70% to 95%. The rest are footholds only.
- **The top is a ledge**, which the climber tops out onto.

The step-13 climber climbs it unchanged in kind: it chooses each limb's
next hold the way asked, three limbs held
([a wall of holds is free climbed](./a-wall-of-holds-is-free-climbed-limb-by-limb-a-gap-jumped-for.md)).
A wall rough enough (0.8 and up) is climbed hold to hold. A sparser one
needs dynos.

## Traps: what irregular holds found in the climber

Its tests used a regular grid, which never reached any of these:

- **An elbow flipping, 34-44 cm in a frame.** The arm's pole (the side the
  elbow bends to) depends only on the arm's own direction. When a hand
  reaches up past the face, the arm points almost exactly against its
  pole, and the elbow's side is undefined.
  - No pole rule that depends only on the arm's direction is defined for
    every direction (the hairy-ball theorem), so mending the pole cannot
    work.
  - A fallback toward straight down was itself opposite an arm reaching up.
    One toward back from the wall lay along an arm reaching in to the wall.
    Both flipped worse.
  - Instead **the climb keeps out of where it cannot be solved**. A move is
    chosen only if, at 8 points along it, neither arm comes within about 8°
    of pointing against its pole (`ELBOW_CLEAR`, the cosine). Within 18° it
    is chosen the less the nearer (`ELBOW_AVOID`). Both are judged on an
    estimate made from the hips and a fixed shoulder (`against_pole`), for
    reachable holds only.
  - Ruled out from 18°, a climb stuck where only such a move went on.
    Discouraged from 25°, the grid's diagonal climb turned aside.
  - Checking each move on the pose itself instead, at the same 8 points,
    changed no climb in the tests by a single bit, and climbing cost 5.3
    times as much. Computing the estimate for every hold, reachable or
    not, cost 2.8 times as much.
- **The fingers swinging from one side of the hold to the other.** A held
  hand's fingers follow its forearm sideways. When a forearm pointing down
  passed straight down, the sign flipped, and the wrist went round its hold
  7.6 cm. The fingers now go aside only as far as the forearm does
  (`HOOK_SOFT`). The grid's own worst change of step fell from 2.9 to
  1.3 cm with it.
- **Getting on, a hand swept 2.8 cm into the wall** reaching round the
  shoulder to a hold out to the side. The sweep is now kept no nearer the
  face than where the hooked wrist ends.
- **The test harness itself** measured the topping-out blend, where the
  climb hands over to a hang and the feet leave their holds by design. It
  read 22-52 cm "off the holds". It now leaves that blend out.

## Consequences

**Headless** (`puppet_base` with its own fingers' grips, 60 fps; 2.4 ×
5 m walls, climbed up from standing):
- **Hold to hold, roughness 0.8 and 1.0, every seed:**
  - each wall got on and topped out into a hang;
  - climbed twice, it ended bit-identically;
  - the held hands were on their holds to 1 µm and the feet to 1 µm;
  - nothing went into the wall, and there were no dynos;
  - the largest change of step was 1.8-4.1 cm: a moving arm's elbow
    sweeping round over several frames, never a jump.
- **Sparser, 0.3 and 0.6:** each wall got on and topped out, with 0-2
  dynos. The hands were within 1 cm of their holds.
- **The grid's own tests** pass, its worst change of step better
  (1.3 cm).
- **The same seed grows the same holds**, another seed others. A rough
  wall has 2.2 times the holds of a smooth one, more of them for hands.

**Cost**: `anim_bench --gait free-climb` (3 s of climbing re-played every
frame from its start): 0.108 ms a character at p50, as before. The p99
rose from 0.112 to 0.313 ms, on the frames that choose a move.

**Live** (gallery `--rough-wall 0,-0.6,180,2.4,5,1.0,1 --free-climb
1,0,1`, gizmos on with the mesh off from the left and from in front of
the face, then the mesh): on the wall, the hands on holds out wide and up,
the feet on holds below, nothing through the face.

## Revisit when

- **A dyno on a sparse wall** changes its steps 6-22 cm and lets its feet
  hang off their holds before the flight: the dyno's own behaviour, which
  no test bounds (`a_hold_out_of_reach_is_jumped_for` bounds none). A
  held hand can also slip about 8 mm there.
- **Getting on a sparse wall** can find no two hand holds over the
  shoulders, and no climb starts.
- **Walls not flat**: the holds lie on one plane; overhangs and corners
  are step 15's.
- **The pole itself**: an elbow model that keeps state (the side it was
  on) would remove the singularity rather than steer round it.

## Related

- [A wall of holds is free climbed limb by limb, a gap jumped for](./a-wall-of-holds-is-free-climbed-limb-by-limb-a-gap-jumped-for.md) — prerequisite: the climber this feeds, and its dyno.
- [Restoring-force constraints need perturbed input](../../engineering-practice/testing/restoring-force-constraints-need-perturbed-input.md) — same-trap: input tame enough to satisfy the code hides its faults; a regular grid of holds never reached the climber's singular arms.
