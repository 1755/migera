---
title: A wall is kicked off toward a lip round a corner
description: "Step 8, second part: a tic-tac is the run up's take-off and foot on the wall, the hips leaving where a steady push brings them, then a fall aimed at another wall's lip. It catches lips 2.3-2.7 m high from 0.6-0.9 rad off square, taken off up to 0.2 m near or 0.1 m far. Read before changing WallRun::kick."
type: decision
status: current
tags:
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/wall.rs
  - src/character/anim/parkour/fall.rs
  - src/character/anim/stance.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::wall::tests::{it_kicks_off_a_wall_and_catches_a_lip_round_the_corner, a_kick_off_the_wrong_foot_too_askew_or_toward_too_high_a_lip_is_not_taken}"
  - "live: character_gallery --step-seconds 0.0333333 --ledge 1.930,-8.748,132.97,2.5,3.0 --side-ledge -0.6435,-8.4005,222.97,4.0,4.241 --wall-kick-at 0.5 --anim-speed 4.0, Xvfb, gizmos on/mesh off and mesh on"
  - "anim_bench --features real_rig --gait wall-kick --characters 100"
aliases:
  - tic-tac
  - wall kick
  - wall jump
  - HangAsk::WallKick
  - WallRun::kick
---

# A wall is kicked off toward a lip round a corner

Step 8 of the [parkour design](./parkour-moves-implementation-design.md),
second part: the tic-tac. Running at a wall at a slant, the walker puts the
foot nearer the wall on it and kicks off up and across. It turns in the air
and catches the lip of another wall, out of a standing jump's reach. It is
built on the [run up a wall](./a-wall-is-run-up-off-one-foot-and-its-lip-caught.md):
`WallRun::kick` shares that move's take-off, foot hold, wall-phase pose and
release.

## Decision

**What differs from the run up** (`WallRun::plan_on` with a target):
- **Met 0.6-1.1 rad off square**, not up to 0.3.
- **Off the foot farther from the wall**, the nearer foot going on it
  (`WallRun::kick_leg`). Off the other foot, its leg crosses in front: at
  0.6 rad a 2.5 m lip wanted 3.57 m/s up.
- **The hold 1.2 m up**, not 1.0. The hips meet the wall 0.6 m out, not
  0.75, within ±0.2 m.
- **The catch point**: the lip point nearest a point 0.5 m out from where
  the hips leave the kicked wall. Taken nearest where they leave, in a
  corner the flight went along the wall, not off it. The shoulders end 8 cm
  under the lip and the hips 0.42 m out from its face, as a
  [leap from a hang](./a-hang-is-leapt-from-up-aside-or-back.md) plans its
  catch.
- **The flight**: to the top of the rise, or longer if crossing at 4 m/s
  takes longer. It goes at most 3.5 m/s up and at least 0.5 m/s out from
  the wall kicked.
- **Where it leaves the wall**: where a steady push from the meeting
  velocity to the leaving one brings the hips. That is the contact's
  constant-acceleration point: meeting point + (v_meet + v_leave)/2 · T.
  The leaving velocity is the flight's from there, so it is found by fixed
  point (4 passes). It goes no nearer the face than 0.42 m.
- **How long on the wall** (0.15-0.35 s, tried longest first in 0.01 s
  steps): the longest the leg on its hold reaches through. The leg's reach
  is measured from the socket's actual distance across from the ankle.
- **It turns in the air** to face the lip (`Falling::spin_round`, 0.4 s);
  the fall is aimed at it as a leap's is.
- **The arm swing and the knee drive keep the run up's 0.37 s span.** The
  kick leaves part-way through and the fall coasts them on.

**It refuses** when:
- the take-off foot is the nearer one;
- the run meets the wall more than 1.1 rad off square;
- the lip needs more than 3.5 m/s up, or a push into the wall;
- the hips would meet the wall more than 0.2 m off 0.6 m out;
- the hold is out of the leg's reach, or the hips would brake over 3 g (as
  for a run up).

**The walker** (`HangAsk::WallKick`, `Walker::ledge` the lip to catch) runs
at a wall among `Walker::ledges`. `WallRun::kick_off` picks the nearest
ahead met within 1.1 rad, its face reaching the floor, tall enough over the
hold. The walker:
- takes off from a contact of the right foot within `KICK_TAKEOFF` (0.2 m
  nearer to 0.1 m farther) of `WallRun::kick_takeoff`;
- otherwise sets its pace for a whole number of steps to the window's
  middle, one more or fewer for the right foot to land there (as a lazy
  vault forces its leg).

## Alternatives considered

- **The run up's leave point** (0.42 m out, as high as the leg reaches),
  with a share of the along-wall velocity carried on: the ends of the
  Hermite disagreed with their velocities, and the hips braked at 6.4 g.
  With the steady-push point, the Hermite's acceleration is near constant:
  1.6-2.9 g.
- **A fixed 0.3 s on the wall**: at 0.15-0.2 s the leg lifts the hips only
  0.24 m, while 0.3 s of steady push wanted 0.39. The contact time now comes
  from the leg's reach.
- **The run up's 1.0 m hold**: nearly level with the hips, the leg could
  not lift them (5.8 g). At 1.2 m it can.
- **Meeting 0.75 m out** (the run up's): taken off 0.2 m farther, the leg
  could not lift the hips; 0.4 m farther, it could not reach the hold.
  Nearer at 0.6, the window is on both sides of the best.
- **A 0.35 m meeting slack** (the run up's): taken off 0.4 m nearer at
  3.5 m/s, a kick planned to a 2.5 m lip missed it (the fall's wall room
  held the hips out of reach). The kick's slack is 0.2 m.

## Traps

- **An arm swing squeezed into a 0.16 s contact** threw the hands at
  15 m/s; the swing keeps the run up's span.
- **A fall's knee kept off a wall in two passes** stayed 1.5 cm in: the
  take-off leg's driven knee, turned to face the lip. Each ankle move brings
  the knee about half the way; `fall.rs` takes six passes (0.1 mm).
- **A dead-straight leg's IK hinge flipped between frames**: the leg that
  pushed off, coasting, swung its knee 24 cm in a frame. See
  [a near-straight leg is bent toward its kneecap](../ik-and-locomotion/a-near-straight-leg-bends-toward-its-kneecap.md).
- **The elbow on the wall's side went 2-8 cm into it**, taken off near. The
  wrist was kept off the face, the elbow not. Elbows within 0.2 m of the
  face are now turned along it, wholly by 0.05 m, for run ups too.
- **A test with only the best take-off hid all of the above but the
  arms.** The walker never takes off exactly there. Sweep the take-off
  across the window it will really use.
- **The walker from a standstill 6 m away** accelerated through its last
  steps, mis-planned the foot, and dropped the ask at the wall. A kick
  wants a run-in long enough to settle the pace (9 m in the gallery).

## Consequences

**Headless** (`puppet_base`, 60 fps; lips 2.3, 2.5, 2.7 m; 0.6, 0.75 and
0.9 rad off square; 3.5 and 4.5 m/s; take-offs 0.2 m near to 0.2 m far of
the best; 89 of 90 planned, the one refused 0.2 m far at 3.1 g):
- the foot is held within 0.011 mm;
- nothing goes into either wall beyond 0.11 mm;
- no joint goes over 13.4 m/s about the hips;
- the hips brake at 1.6-2.9 g on the wall;
- every one catches its lip.

**Refused**: off the nearer foot, 1.2 rad off square, a 3.2 m lip.

**Live**: at 4 m/s, 0.75 rad off square, at a 4 m wall, toward a 2.5 m lip
at right angles to it. It adjusted its steps, put the near foot on the face
at hip height with the arms swinging up, kicked off, turned and caught the
lip, hanging braced.

**Cost**: `anim_bench --features real_rig --gait wall-kick --characters
100`, 50 µs a character at p50 (the run up: 31.5). The difference is the
fall facing the target wall: its knee passes and the turn.

## Revisit when

- **Kicking to a lip straight across a corridor**, or **chaining kicks**
  (wall to wall): the catch point and the turn assume a lip to the side.
- **A kick off without a lip to catch** (onto a top, or just away): not
  built; the plan needs a catch.
- **A run-in too short to set the pace**: the walker could fall back to a
  run up or a plain jump instead of dropping the ask.

## Related

- [A wall is run up off one foot and its lip caught](./a-wall-is-run-up-off-one-foot-and-its-lip-caught.md) — prerequisite: the move this one is built on.
- [A hang is leapt from, up, aside or back](./a-hang-is-leapt-from-up-aside-or-back.md) — same-pattern: the catch planned from a launch, the turn in the air.
- [A low obstacle is speed-vaulted as a reshaped running leap](./a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md) — same-pattern: the walker's pace and forced take-off leg.
- [A fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md) — deeper: the fall's wall room and knee keep-off the kick relies on.
- [A near-straight leg is bent toward its kneecap](../ik-and-locomotion/a-near-straight-leg-bends-toward-its-kneecap.md) — deeper: the IK fix this move needed.
