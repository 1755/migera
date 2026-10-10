---
title: A hand rests on a wall beside the body
description: "Step 11, fifth part: beside a tall wall within 0.62 m of a shoulder, that hand rests on it palm flat, 3 cm off the face found square (slanting too); any other hand in a wall is swung out of it after the springs. Read before changing parkour/wallhand.rs or a walker's arms near walls."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-10
verified: 2026-10-10
code:
  - src/character/anim/parkour/wallhand.rs
  - src/character/anim/walker.rs
  - src/character/anim/plugin.rs
  - examples/anim_bench.rs
sources:
  - "tests parkour::wallhand::tests::{a_hand_rests_on_a_wall_beside_it, a_hand_rests_on_a_wall_met_slanting, a_hand_is_kept_out_of_a_wall}"
  - "live: character_gallery --holds-wall 0,-0.6,180,9,13 --free-climb 1,0,1, BRP over the walk-up"
  - "live: character_gallery --side-ledge 0.62,0,90,2.5,6 (standing) and --side-ledge 0.62,0,90,2.5,20 --anim-speed 1.0 (walking), Xvfb, gizmos on/mesh off Front, Back and Left, mesh on"
  - "anim_bench --gait wall-hand --characters 100"
aliases:
  - hand on wall
  - wall touch
  - brushing a wall
  - wallhand
  - Beside
  - keep_hands_off
  - hands_off_walls
  - hand in a wall
---

# A hand rests on a wall beside the body

Step 11 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md),
fifth part: a hand on a nearby wall, resting on it when standing beside
one and brushing it when walking close. There is no data; the reach and
the hand's place are by eye.

## Decision

**The wall** (`wallhand::beside`) is probed straight out from each
shoulder, level, every 2 cm to 0.62 m, then narrowed to its face by
bisection. The probe uses the walker's ground (`blocks`, solid higher than
0.2 m under the shoulder). The nearer side wins.
- **Its face's own way out** comes from two more probes 0.2 rad either
  side, reaching twice as far: the line through their hits. The distance
  is measured square to that face.
- **A face turned more than 60° toward the front or the back** is ahead
  or behind, not beside, and is not leant on.

**Any hand in a wall** (`wallhand::keep_hands_off`), one ahead that is not
leant on, is swung out about its shoulder, the arm whole:
- just far enough to keep the wrist 3 cm off the face, held there from
  the moment it comes within 3 cm, so it never jumps;
- in the sprung pose, after the springs, in `solve_foot_ik`, whenever the
  walker sets `AnimFootIk::hands_off_walls` (standing, off holds, not
  jumping, no reach);
- a shoulder already in the wall is left alone: the walker keeps the
  shoulders out ([walking is kept out of
  walls](./walking-is-kept-out-of-walls.md)).

**The weight** (`Beside::weight`): 0 at 0.62 m, eased to 1 by 0.45 m; none
nearer than 0.18 m, with the arm folded against it.

**The hand** (`rest_hand`):
- the wrist goes 0.12 m ahead of the shoulder along the face, 0.08 m
  under it, 3 cm off the face;
- the arm is solved to it with the elbow pole down and back;
- the palm is turned flat onto the face, the fingers up and a little
  ahead;
- all of it is blended from the hanging wrist by the weight.

**The walker** applies it standing or walking (not running), off holds,
out of a jump, uncrouched, and not while a beam's balance, a squeeze, a
teeter or a reach has the arms. It eases the weight over 0.3 s and keeps
the last wall while the hand comes off. Walking along a wall, the target
moves with the shoulder, so the hand goes along the face as the body
passes.

## Alternatives considered

- **The hand put on the wall where it was first touched**, held there as
  the body walks on: it would trail behind and come off within a stride.
  Resting it at the shoulder keeps it on the face, sliding along it.
- **Fully on within 0.1 m of distance**: the wrist moved 4.5 cm for 5 mm of
  the wall's nearing. Widened to 0.17 m, and eased over time in the
  walker.

## Traps

- **The face found a probe step late** (2 cm) put the wrist 2 cm nearer
  it than asked. The probe's last step is narrowed by bisection to within
  0.02 mm.
- **A face met slanting, taken square to the probe**, with "ahead" taken
  straight ahead, put the wrist into it. The test had only walls parallel
  to the body, and could not see it.
- **The hands go into walls the rest never sees.** Walking up to a hold
  wall 0.6 m away, the approach loops along the face, turns back and
  turns to face it. As it turned, the face came ahead (not leant on) and
  a swinging hand hung 8 cm into it.
  - Held off the face in the target pose, the springs' lag still carried
    it 7 cm in. Hence after the springs.
  - It went no further once its shoulder, swept round in the turn, was
    itself 2 cm into the face. The shoulders are now kept out too.

## Consequences

**Headless** (`puppet_base`; tall walls 0.3, 0.45 and 0.55 m from either
shoulder):
- the wall was found on the right side to within 1 mm;
- fully on, the wrist was 3 cm off the face (within 1 cm), under the
  shoulder;
- no joint went through the wall;
- a wall 0.9 m out, or one waist high, was not leant on;
- the wrist moved under 3 cm for each 5 mm the wall came nearer;
- walls slanting 20-45° toward the front (0.35 m square off): found
  square to within 1 cm and its way out within 0.01, the wrist 3 cm off
  the face, nothing into it; at 70° it is ahead and not leant on
  (`a_hand_rests_on_a_wall_met_slanting`; taken square to the probe, it
  fails);
- a wall brought in on a hanging hand, square, slanting 45° and 80°, 5 mm
  a step: the hand untouched while 3 cm clear; then held 3 cm off the
  face (within 2 mm), the wrist and elbow never in it, no step over 1 cm
  (`a_hand_is_kept_out_of_a_wall`).

**Live** walk-up to a hold wall 0.6 m ahead (`--holds-wall
0,-0.6,180,9,13 --free-climb 1,0,1`, BRP): a hand went 11 cm into the face
before, and now comes no nearer it than 1.6 cm.

**Live**: standing beside a 2.5 m wall 0.44 m from the shoulder, the hand
rested on its face (seen front, back and on the mesh, in front of the
face). Walking along a 20 m wall at 1 m/s, the hand stayed on it through
the stride.

**Cost**: `anim_bench --gait wall-hand --characters 100`, 44 µs a
character at p50 against the walk's 24 (the probe and the arm solve each
frame).

## Revisit when

- **A corner**, the two side probes on two faces: the line through them
  is neither face.
- **An elbow in a wall** with its wrist clear: only the wrist is checked.
- **A hand pushing off a wall** in a turn, or touching it only briefly as
  it passes, rather than resting there.

## Related

- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 11's design.
- [Walking is kept out of walls](./walking-is-kept-out-of-walls.md) — context: the wall probe (`blocks`) this reads, and the 0.2 m the body is kept off a wall.
- [A sneak carries its hands, placed by arm IK](../ik-and-locomotion/a-sneak-carries-its-hands-placed-by-arm-ik.md) — prerequisite: the arm solve toward a pole.
