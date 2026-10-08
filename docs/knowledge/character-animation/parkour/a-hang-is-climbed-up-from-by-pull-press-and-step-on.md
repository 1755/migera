---
title: A hang is climbed up from by a pull, a press over the top and a step on
description: "Hanging::climb_up: the hips on a C² spline through shapes measured from the hands (pull, turn over, press folded over the top, step on, stand). Read before changing hang/up.rs or bringing a leg up over an edge. Traps: a knee's arc into the corner, a thigh turned at the hip, hips joint vs root."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-07
verified: 2026-10-07
code:
  - src/character/anim/parkour/hang/up.rs
  - src/character/anim/parkour/hang.rs
  - src/character/anim/parkour/geometry.rs
  - src/character/anim/walker.rs
sources:
  - "tests parkour::hang::up::tests::* (ledges 1.95 and 2.35 m; braced, and free from mid-swing)"
  - "live: character_gallery --hang-at 1 --climb-up-at 16 --step-seconds 0.0166667, Xvfb, BRP"
aliases:
  - ClimbUp
  - climb_up
  - HangAsk::ClimbUp
  - LedgeGround
  - climb up
  - mantle from a hang
---

# A hang is climbed up from by a pull, a press over the top and a step on

Hanging from a ledge (step 1), a walker asked `HangAsk::ClimbUp` climbs onto
the top and stands there, 0.3 m back from the edge, on its ground
(`parkour::LedgeGround`). `Hanging::climb_up` plans it once; each frame poses
the body onto the plan, as the hang does. It is step 2 of the
[parkour design](./parkour-moves-implementation-design.md).

## Decision

**The hips follow one path, planned once**, through five shapes, each
measured from where the hands are, on the leant trunk's own shoulders
(`jump::upper` posed and measured):

| Part | Time (s) | Shape at its end |
|---|---|---|
| Pull | 0.9 | the shoulders 0.2 m out from the face, 0.1 m below the lip, leant 0.25 rad; the hands still hooked |
| Turn over | 0.4 | the shoulders over the lip (0.05 out, 0.15 up), leant 0.55; one hand, then the other, up over the lip and pressing flat on the top 0.12 m back |
| Press | 0.6 | the shoulders over the hands at 0.88 of the arm, the trunk folded 1.4 rad, near flat over the top: the hips high |
| Step on | 0.5 | the lead foot up the face, the knee over the lip, then onto its spot on the top |
| Stand up | 1.4 | the other foot on (first half), the hips held out 0.42 s, the hands letting go (0.3-0.6 of it), standing at the foot IK's drop |

The path is a **clamped cubic spline** (C²): from the hang's own velocity to
rest. The trunk's lean is a second one. There are no climb-up timings
([movement data](./parkour-movement-data.md)); the pull-up's elbow range is
the proxy, and these were set by eye and by the checks below.

**A foot comes up the face, then over.** Each foot goes from where it hangs
to an ankle 0.15 m out from the face and 0.28 m below the lip (the shin down
the face, the knee over the lip), then straight up to 0.25 m above the lip,
then onto its spot: the standing pose's ankle under the final root.

**Braced, the feet stay on the wall through the pull.** Each one steps up the
face to where its leg pushes at 0.88 of its length. Out of reach as the hips
rise, a foot smears up the face until it reaches. The knees are turned out
0.6 of the way to sideways (about 55° at the hip). The feet leave the wall
through the press.

**Pressing, the elbows point back** toward the hips: each arm's elbow pole
turns from the hang's (out to the side and back) as its hand comes over.
With the hang's pole the elbows went 18.5 cm out past the
shoulder-to-wrist line; a quarter out, 6.8 cm; straight back, 0.6 cm.

**Asked before it hangs**, `ClimbUp` grabs the ledge first and climbs
straight on.

**Asked mid-swing, it waits** for the hips to slow below 0.25 m/s, at the end
of a swing, before it pulls.

**It refuses** a top less than 0.55 m deep.

## Alternatives considered

- **Chord velocities at each knot** (Catmull-Rom): the acceleration jumped
  at the knots, 0.74 g as the hips came in to a crouch.
- **A crouch over the feet before standing**: 0.9 g coming in, with the
  hands still reaching for the top.
- **Standing 0.45 m back**: the lead foot could not reach its spot from hips
  still out over the face.
- **Starting at once mid-swing**: the pull had to turn a 1.05 m/s swing round
  at 6.7 m/s².

## Traps

- **A knee swings round its socket on the thigh's length.** Coming up past
  the lip, the knee clears the corner only if the arc does:
  `socket_up ≥ √(thigh² − socket_out²)` (with the socket `socket_out` out
  from the face). With the trunk leant 0.9 at the press, the sockets were
  0.23 m above the lip and the knee went into the corner. Folding the trunk
  to 1.4 raised them clear.
- **No pose has the socket over the top with its foot still below the lip.**
  The leg lies across the corner: bent forward the knee goes into it (3 cm);
  aimed up it can only turn out backward. The hips are held out until the
  trailing foot is up.
- **A thigh turned about its own line.** Aiming a knee by turning the leg
  about its socket-to-ankle line turns the thigh at the hip. A knee-fold
  check in the thigh's own frame cannot see that. Measured in the pelvis's
  frame, the knees aimed out-and-back had turned 116°, and straight aside
  90°.
- **The hips joint is not `root_translation`.** On this rig it is the root
  translation plus the hips bone's own offset (0.95 m up). Placing the
  ankles from the root translation put the feet 0.95 m above their marks.
- **The shoulders from turning the whole trunk about the hips** were 6 cm
  higher than the leant pose's own, enough to pull the hands off the top.
  Measure them on `jump::upper`.

## Consequences

**Headless** (`puppet_base`; 1.95 and 2.35 m; braced, and free from 1.5 s
into the hang):
- the hands on their hooks within 0.1 mm pulling, and on their presses
  pressing;
- no joint inside the block, wall or top;
- every bent knee's hinge within 0.94 rad of standing's, in the pelvis's
  frame;
- the braced feet on the wall through the pull, and each foot on its spot
  once on the top, within 1 mm;
- the hips at most 4.3 m/s² from the hang on;
- it ends on its spot in the standing pose.

**Live** (the default wall, 2.25 m, BRP): standing on the top 3.8 s after the
ask, the pelvis 3.19 m up (the top plus 0.94), 0.3 m back from the face, the
ankles 9 cm above the top.

## Revisit when

- **The move controller arrives**: it chooses when to climb, and a run
  straight on from the top.
- **A knee onto the top** instead of a foot: a lower press would do, but
  needs a knee hold.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is step 2 of.
- [A ledge is caught near the top of a jump and hung from](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) — prerequisite: the hang this climbs up from, its grips and its swing.
- [Parkour movement data](./parkour-movement-data.md) — context: why the climb-up's timings are set by eye.
- [A block is mantled as a climb up from the floor](./a-block-is-mantled-as-a-climb-up-from-the-floor.md) — applies: a mantle shares this plan's press, step on and stand, with its own start; the foot path's no-nearer-the-face rule came from it.
- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — applies: the landing hand-off (`LadderGround`) `LedgeGround` copies.
- [Unsigned measurements cannot see direction](../../engineering-practice/testing/unsigned-measurements-cannot-see-direction.md) — deeper: why the knee check is a hinge in the pelvis's frame, not the thigh's.
