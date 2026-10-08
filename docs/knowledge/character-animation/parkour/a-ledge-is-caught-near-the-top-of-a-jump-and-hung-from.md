---
title: A ledge is caught near the top of a jump and hung from, the arms giving
description: "parkour::Hanging: a standing jump up and in, the hands meeting the lip before its top; the arms' give and a pendulum about the grip take the catch; braced or free; hands hooked over the lip. Read before changing hang.rs or a hold on an edge. Traps: grips' frame, the walk's stop, arms swung into the wall."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-08
verified: 2026-10-08
code:
  - src/character/anim/parkour/hang.rs
  - src/character/anim/parkour/geometry.rs
  - src/character/anim/hand.rs
  - src/character/anim/armik.rs
  - src/character/anim/walker.rs
sources:
  - "tests parkour::hang::tests::* (ledges 1.95, 2.15, 2.35 m; braced and free)"
  - "live: character_gallery --hang-at 1 [--ledge X,Z,HEADING,HEIGHT,WIDTH,BELOW] --step-seconds 0.0166667, Xvfb, BRP"
aliases:
  - Hanging
  - Ledge
  - HangAsk
  - ledge grab
  - ledge hang
  - braced hang
  - free hang
  - hooked
  - hook_lip
---

# A ledge is caught near the top of a jump and hung from, the arms giving

A walker asked to grab a ledge (`Walker::ledge`, `Walker::hang =
HangAsk::Grab`) walks out in front of it and straight in to a spot under
it, jumps up and in, catches the lip with both hands a little before the
top of the flight, and hangs: braced with its feet on the wall below, or
free from a slab with nothing under it. `parkour::Hanging` plans and poses
it. It is step 1 of the
[parkour design](./parkour-moves-implementation-design.md).

## Decision

**The jump is the standing jump** (`jump::Jump`), up and in: the lowest one
whose hands, from the shoulders lifted toward them (`armik::shoulder_lift`),
reach the lip within 0.9 of the arm at the top of its flight, 4 cm higher
(`CATCH_MARGIN`); caught the first moment on the way up they reach it. It
stands with its hips 0.55 m out from the face (`SPOT_OUT`) and jumps 0.2 m
in (`JUMP_IN`). On `puppet_base` that reaches ledges 1.9 to 2.35 m high;
lower, the lip is in reach standing (a mantle, step 7), and the grab is
refused. Stopped short of the spot, it jumps in twice as far again as it
fell short, up to 0.4 m (`STOPPED_SHORT`): the hands meet the lip near the
top of the flight, half way in.

**The arms reach for the lip through the push**, each wrist on the straight
line from where it was as the push began to its hook: the jump's own arm
swing, from 0.35 m out, put the hands 16 cm into the wall.

**At contact the hips keep their velocity and the arms take it.** The hips'
state about the grip, in the plane square to the wall, is polar:
- radially, a damped spring to the hang's length (`GIVE`, 10 rad/s, 0.9);
- braced, a critically damped spring to the braced lean (`BRACED_SETTLE`,
  5.5 rad/s);
- free, a compound pendulum about the grip, `ω² = g·d/(d² + k²)` with
  `k = 0.5 m`: about 2.4 s for a person, the hanger damping it at 0.2 of
  critical (no data; `SWING_DAMPING`).

There is no measured catch or braced-hang data
([movement data](./parkour-movement-data.md)): these are set from the rig's
limits and checked by eye and by the catch's deceleration.

**Braced or free** is decided once: braced if the wall goes down past where
the feet go, with a 0.1 m margin. Braced, the hips settle 0.42 m out from
the face (`BRACED_OUT`), the legs reaching the wall at 0.95 of their length,
the feet pitched 1.1 rad toes-up on it; free, the legs hang under the body.

**The hands hook over the lip** (`hand::hooked`, `hand::hook_lip`): the palm
against the face, the fingers bent 90° at the knuckle over the lip and near
flat on the top (5° and 5°), the thumb in the palm's plane. The lip sits
inside the bent knuckle, a finger's half thickness out of the palm and back
along the hand. `RelaxedHands::hook` picks this shape over the bar's;
`close_hands` draws it.

**The walker** goes first to a point 1.2 m out in front of the spot
(`LEDGE_LEAD_IN`), then straight in by the approach; on its holds it is
like a ladder's climb (`WalkerState::on_holds`): the root rides the hips,
the legs posed, the look on the neck and head, the idle's arm and chest
oscillators faded.

## Alternatives considered

- **Standing 0.35 m out and jumping straight up** reached 1.9-2.5 m, but the
  walk up to the spot swung a hand 14 cm into the wall.
- **Standing 0.65 m out, jumping 0.3 m in**: the body was still short of the
  wall at the top of the flight, and only a 1.9 m ledge was reached.
- **The wall's footprint as the approach's obstacle**, or the approach alone:
  made to end with a chair behind it, it came at a spot faced at a wall from
  beside it and turned in along the face, a fingertip 18-21 cm into it.

## Traps

- **A hand grip in its own frame, used as if in its rest frame.**
  `RelaxedHands::grips` are in each hand's own frame; the ladder turns them
  by the hand's accumulated bind (`set_grips`). Taken raw, the hands pointed
  out from the wall, the middle knuckles 13 cm off the lip, while the test
  comparing the hand to its own target passed. The test now hooks the rig's
  real middle finger on the posed hand and measures its knuckle and tip.
- **The facing turn composed twice** in a braced foot's ankle-from-ball
  offset (`attitude · attitudes⁻¹ · (turn · ankles)`, `attitude` already
  turned): every wall tested faced with a half turn, which squares to none.
  Found going round a corner (step 3): facing +X, a foot was 20 cm off its
  hold. Fixed in `ankle_from_ball`; tested on four facings.
- **The lip at the knuckle's height** lays the bent fingers through the top;
  curled 15° and 10° past the knuckle, the tips went 16 mm into it.
- **The relaxed thumb** sticks out in front of the palm, 5 cm into the face.
- **The hips' acceleration over the whole grab** is the jump's push
  (74 m/s² at the hips at take-off), not the catch: measure from the catch.
- **A heading convention**: walker headings are `atan2(-x, -z)` (zero along
  -Z); `atan2(x, z)` sent the walk the other way, through the wall.
- **Settled is not reached in seconds at 0.15 of critical damping**: a free
  hang still drifted 1.3 cm/s after 12 s.
- **The walk's stop also falls short.** One live grab in three stopped
  0.2-0.3 m short of its spot (seen as well on a build from before the
  walls were added), the lip was out of the fixed jump's reach, and the
  grab was dropped. Hence the jump in grows with the shortfall
  (`a_grab_from_short_of_its_spot_jumps_in_further`).

## Consequences

**Headless** (`puppet_base`; ledges 1.95, 2.15 and 2.35 m; braced and free):
- each hand's wrist on its hook and its lip point on the lip within 1 mm,
  the palm on the face within 0.02 rad;
- the real middle finger's knuckle within 2.5 cm of the lip, its tip on the
  top;
- no joint into the wall below the edge, through the jump and the hang;
- the hips at most 3 body weights from the catch on;
- a free hang swings at a person's period (2.0-2.8 s); both come to rest.

**Live** (the default wall, 2.25 m): the middle knuckles 1.3 cm above the top
and 1.4 cm out from the face, the fingertips on the top 8.4 cm back, the
balls of the feet 1.1 cm from the face. The walk's stop lands about 11 cm
past its spot, and a fingertip brushes the face (up to 5 cm in) as it does.

**Cost** (`anim_bench --gait hang|hang-free --characters 20`, each frame
posed from the grab's start): 47 µs a character a frame braced, 58 free;
a ladder's climb 53, a walk 23.

## Revisit when

- **The move controller arrives**: it replaces the walk to the spot (and its
  overshoot) and asks the grab from wherever the jump starts, or from a fall
  (step 5).
- **Higher ledges**: past 2.35 m a run-up's jump (`jump::leap`) or a wall
  run (step 8).

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is step 1 of.
- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — prerequisite: the shoulder lift, hand turn and led pose this shares (`armik`).
- [A jump is planned as its centre of mass's path](../ik-and-locomotion/a-jump-is-planned-as-its-centre-of-mass-path.md) — prerequisite: the jump caught here, and its root motion (`travelled_at`).
- [Parkour movement data](./parkour-movement-data.md) — context: why the catch and braced hang have no measured timings.
