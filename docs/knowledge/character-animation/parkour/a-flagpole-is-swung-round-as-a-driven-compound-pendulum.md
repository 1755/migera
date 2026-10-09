---
title: A flagpole is swung round as a driven compound pendulum, caught from a fall
description: "Step 17: a pole out of a wall is caught from a fall coming by it, swung right round as a compound pendulum about the grip (driven over the top at 2 rad/s), let go 0.6 rad past the bottom after a turn, flung on as a fall. The catch's traps were all elbows. Read before changing parkour/flagpole.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/flagpole.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
  - examples/anim_bench.rs
sources:
  - "test parkour::flagpole::tests::a_flagpole_is_caught_swung_round_and_let_go_of"
  - "live: character_gallery --step-seconds 0.0333333 --start-height 1.5 --block 0,-6,0,1.5,2.0,10 --block 0.75,-7,90,3.5,6,1 --flagpole 0.75,-6.9,90,2.8,1.5 --flagpole-at 0.5 --anim-speed 3, Xvfb, gizmos on/mesh off Left, mesh on Left; BRP pelvis, hands and feet"
  - "anim_bench --gait flagpole --characters 100"
aliases:
  - flagpole
  - giant swing
  - Flagpole
  - parkour::flagpole::Swinging
  - Walker::flagpole
---

# A flagpole is swung round as a driven compound pendulum, caught from a fall

Step 17 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), second part: a pole
sticking straight out of a wall, caught, swung right round (a gymnast's
giant swing), and let go of flung forward, as Assassin's Creed's flagpoles
are. No flagpole data was found.

## Decision

**A flagpole** (`parkour::flagpole::Flagpole`): where its axis leaves the
wall, the way out (level), and its length. The swing is in the plane
square to the pole, along the wall.

**Caught from a fall** (`Swinging::caught`) whose hips are about as far
from the axis as they hang: 0.35 m nearer to 0.15 m farther, no more than
1.2 rad round from straight below, and along the pole with room for the
hands. Forward is the way the fall goes across the pole. The walker
(`Walker::flagpole`) reaches up while falling with the ask, and tries the
catch each frame of any fall, off an edge or a jump gone over one.

**The swing** is a compound pendulum about the grip: the hips' hang
length with a 0.5 m radius of gyration, about 1.3 m. It is integrated at
240 Hz.
- **Driven over the top.** While short of the energy that goes over the
  top at 2 rad/s, a drive adds 18 rad²/s³ of the pendulum's energy a
  second, as the hang's pump does but harder. Caught at a run's 3-4 m/s, a
  body cannot go round on its own (it needs about 6.6 m/s at the bottom);
  a gymnast pumps.
- **Let go** 0.6 rad past straight below on the way up, after one turn
  (`TURNS`, `RELEASE`). The fall starts at the hips' velocity there,
  about 5.9 m/s up and on (3.3 up). It lands on the first top along its
  flight (`land_on`), or rolls on the floor.

**The body** is a straight line turned whole about the pole's axis: the
hips rotated, the legs piked up to 0.25 rad ahead through the bottom by
the swing's speed. The hands are a shoulder's width apart round the pole,
the palm the way the body faces round it, the fingers on along the forearm
(two passes, as every held hand: [a hand grip is measured in the hand's own
frame](../rig-and-retargeting/a-hand-grip-is-measured-in-the-hands-own-frame.md)).
The elbows bend back from the body, a little out.

**The catch** (0.4 s) carries the fall's motion into the swing:
- the swing starts at the fall's angular velocity;
- the hips' distance from the axis goes on at the fall's radial speed (a
  Hermite) to the hang;
- the facing turns from the fall's;
- the fall's pose eases out, except the arms;
- the arms are solved throughout: the wrists from where the fall had them
  (carried round with the body) swept round the shoulder to the pole, each
  elbow's swivel from the fall's way to the swing's.

## Alternatives considered

- **Pendulum dynamics without a drive**: from a run's speed the swing
  stops near 1.5 rad and comes back. Round once needs the energy added.
- **Stopping the swing's integration at the release angle**: the frame it
  let go in went on less than a frame's time. See Traps.

## Traps

Every catch fault was an elbow:
- **The arms put on the pole at once**: an arm swung 98 cm in a frame.
  Hence wrists that move.
- **Wrists going straight** from beside the body to overhead passed the
  shoulder, and the elbow flipped. They are swept round it, as the pole's
  get-on.
- **The fall's wrists carried on the world's axes** while the body swung
  about 1 rad slid across it: a hand 8 cm a frame. They turn with the
  swing.
- **The arms left to IK alone** took the swing's elbow side at once
  (28 cm). Blended by rotation from the fall's, an elbow flipped half way
  (20 cm). The elbow's swivel is turned instead: the swing's way, turned
  back about the arm's line by the swivel as caught, easing to none. Taken
  from the fall's elbow carried each frame, it passed near the arm's line
  and jumped 2.5 rad. Unwrapped per frame near ±π, it flipped sides (42 cm).
- **Elbows bending mostly out** (a hang's way) lay along an arm reaching
  out along the pole as it was caught, and the elbow flipped 37 cm. They
  bend back now.
- **The fall's radial speed dropped** in a frame: the hips' distance from
  the axis eased from rest.
- **The release mid-frame** (stopping the integration there) lost the rest
  of the frame's time: a toe swinging at 12 m/s stepped 9 cm short. It now
  lets go at the frame's end.

## Consequences

**Headless** (`puppet_base` and its own fingers' grips, 60 fps; run off a
1.5 m top at 3 and 4 m/s, poles 2.6-2.8 m up, 0.8-1.2 m out, reaching):
- caught every time, swung round once (6.88-6.94 rad), over the top at
  2.0 rad/s;
- let go up and on, landed on the floor 10-11 m on;
- the hands on the pole within 1 µm after the catch, the wrists not bent
  back;
- no joint's step changed over 2 cm in a frame swinging, 6.7 cm as
  caught, 7.2 cm catching (the arms coming in from the fall's reach as the
  body swings at 4 rad/s).

**Live** (Xvfb, BRP): ran off a 1.5 m block, caught a 2.8 m pole 0.9 m
past its end (the wrists at 2.65-2.9 m), swung round with the pelvis over
the top at 3.83 m, let go, flew 5.5 m, rolled and stood. From the left, a
straight inverted body over the pole, legs together.

**Cost**: `anim_bench --gait flagpole --characters 100`, 65 µs a character
at p50, the bench replaying the swing from the catch each frame.

## Revisit when

- **More turns, or none** (a half swing straight to the release): `TURNS`
  is a constant; the walker could ask.
- **A catch from a running jump** (a leap whose flight passes the pole)
  without going over an edge: only falls are tested for the catch.
- **The release aimed** at a ledge or bar ahead, as a lache is.
- **Data**: no flagpole or giant swing timings; the drive and the release
  are set by eye.

## Related

- [A bar is swung on, pumped, and let go of at a bar ahead](./a-bar-is-swung-on-pumped-and-let-go-of-at-a-bar-ahead.md) — contrast: the bar's pumped swing that stops at 1.2 rad; its release window.
- [A pole is climbed as an inchworm, hands over a leg clamp](./a-pole-is-climbed-as-an-inchworm-hands-over-a-leg-clamp.md) — same-trap: the catch from a fall, and the sweep round the shoulder.
- [A hand grip is measured in the hand's own frame](../rig-and-retargeting/a-hand-grip-is-measured-in-the-hands-own-frame.md) — prerequisite: the held hands' frame and the wrist check.
- [Monkey bars are crossed hand over hand](./monkey-bars-are-crossed-hand-over-hand-the-body-hung-from-the-hands-carrying-it.md) — contrast: step 17's other bar move.
