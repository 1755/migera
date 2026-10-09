---
title: A corner post is swung round as a running leap bent round it, one hand on the post
description: "Step 17's last part: running past a post at a corner, the near hand catches it and the body swings a quarter or a sixth turn round it on the arm, then runs on the new way. It is a running leap whose facing turns through the flight, the hand IK'd to the post, the body banked. Read before changing parkour/corner.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/corner.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
  - examples/anim_bench.rs
sources:
  - "test parkour::corner::tests::a_corner_post_is_swung_round_on_one_arm"
  - "live: character_gallery --corner -0.6,-8,90 --corner-at 0.5 --anim-speed 4, Xvfb, BRP hands and pelvis, gizmos on/mesh off Top, mesh on Front"
  - "anim_bench --gait corner --characters 1 and 100"
aliases:
  - corner swing
  - pole swing round a corner
  - CornerSwing
  - parkour::corner
  - Walker::corner
---

# A corner post is swung round as a running leap bent round it, one hand on the post

Step 17 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), last part: a post or
drainpipe at a building's corner, grabbed in passing with the near hand,
the body swung round it on the arm to run on along the other face, as in
Mirror's Edge and Assassin's Creed. No corner-swing timings were found.

## Decision

**A running leap whose flight is bent round the post**
(`parkour::corner::plan`). It is a `Jump::from_run` that keeps running,
with a rise of 0.05-0.5 m. Through the flight the facing turns
`turn·ease(u)` (quintic, `CornerSwing::turned_at`). The walker carries the
leap's travel along its facing, so the path is an arc about the post.
- Taken off when the post is level with the hips as the flight starts,
  within `ABEAM` 0.25 m (`takeoff_ahead`). The last strides are paced to
  bring a foot down there, as the vault does (`vault_pace`).
- The post must be on the turn's side, 0.45-0.7 m off the line of the run
  (`NEAREST`, `FARTHEST`). The flight is as long as the turn needs at that
  speed.
- The run carries on out of the landing at 3-4.2 m/s from 3.5-4.5 m/s.

**The near hand holds the post** (`CornerSwing::hold`/`held`):
- It takes the post over the take-off's last 0.12 s and lets go over the
  flight's last 0.15 s, or half the flight if that is shorter
  (`holding`).
- The arm is solved to the post, at most 0.97 of its reach. The arm bones
  are then blended from the free pose along one fixed arc
  (`fall::arc`), each sign fixed against the rest pose.
- The fingers point along the way it travels, squared to the palm.
- **Banked toward the post** as the turn would bank it on its own
  (`tan θ = v·ω/g`, at the mean rate). The bank rises and falls over the
  hold as `sin²` and saturates smoothly at 0.45 rad
  (`MOST·tanh(x/MOST)`).

The walker (`Walker::corner = Some((post, turn))`, gallery
`--corner X,Z,DEGREES`) leaps at the footfall that brings the post
nearest. The ask is dropped once taken, or once the post is run past or
is off the turn's side.

## Alternatives considered

- **A pendulum about the post**, as the flagpole is: the body would fly
  on a circle round the grip. The real move is mostly the run's momentum,
  with the arm only bending the path. A leap already lands running, so
  bending its path keeps the take-off and landing for free.
- **Turning the facing at a constant rate over the flight**: the rate
  starts and stops all at once. See Traps.

## Traps

- **A constant turn rate** stepped the trailing toe at take-off and
  landing, and overshot the turn on the last frame. The turn is now eased
  over the flight from the push.
- **The fingers along the arm**: when the hand passed the post's line,
  the finger direction flipped. Hence the travel tangent.
- **The elbow flipped as the hand let go**: IK alone, blended back to the
  free arm by a shortest-arc slerp, crossed the hemisphere. Hence the
  arm-bone `arc` blend, and letting go before the landing.
- **A hard-clamped bank** passed its clamp in one frame and swung a toe;
  so did a bank that followed the per-frame rate. Hence the `sin²` envelope
  and the tanh saturation.
- **A straight arm**: at 0.75 m out the arm reached full length, the
  elbow had no steady side, and it swung 25 cm in a frame. Hence the 0.97
  reach clamp, and `FARTHEST` lowered to 0.7.

## Consequences

**Headless** (`puppet_base` and its own fingers' grips, 60 fps; 3.5 and
4.5 m/s; quarter and sixth turns, both ways; posts 0.55-0.68 m aside):
- turned exactly the turn asked;
- the hips stayed 0.55-0.73 m from the post, within 0.1 m of where it
  stood aside;
- the hand on the post to within 1 µm while held;
- ran on at 3.0-4.2 m/s.

The largest step change is the running leap's own: 14-20 cm, the known
toe-off and touchdown kink of the run's leap. At 4.5 m/s the turn adds
6 cm over the unturned leap, within what its acceleration accounts for
(`(ω²+α)·1.2 m·dt²`).

**Live** (Xvfb, BRP): running along −Z past a post 0.6 m to the left, the
left hand reached the post, the body swung round it at about 0.64 m and
ran on along −X at 4 m/s. From above, the arm runs from the shoulder to
the post mid-swing. From the front, the mesh banks into the turn with the
arm out.

**Cost**: `anim_bench --gait corner`, 76 µs a character at p50 (74 µs
with 100 characters). The bench re-advances the leap from its start every
frame; the walker advances it one frame at a time.

## Revisit when

- **Turns past a quarter**: the hips stay outside the post only because
  the leap's travel matches the arc. A half turn round a pole wants the
  pendulum.
- **The leap's own kink** (8-20 cm at toe-off and touchdown) is fixed in
  the running leap: this move would inherit the fix.
- **Data**: the hold timings, the bank and the reach band are set by eye.

## Related

- [A flagpole is swung round as a driven compound pendulum](./a-flagpole-is-swung-round-as-a-driven-compound-pendulum.md) — contrast: a swing about a grip, where this one only bends a leap's path; also source of the fixed-sign `arc`.
- [A wall is run along on two steps of a lifted leap](./a-wall-is-run-along-on-two-steps-of-a-lifted-leap.md) — same-pattern: a running leap reshaped by a fixture, landing running on.
- [A run leans whole into a turn](./a-run-leans-whole-into-a-turn-and-its-trunk-with-its-speed.md) — same-pattern: the bank from `v·ω/g`.
- [A hand grip is measured in the hand's own frame](../rig-and-retargeting/a-hand-grip-is-measured-in-the-hands-own-frame.md) — prerequisite: the held hand's grip.
