---
title: A toe tip pivots on the floor, so it needs its own lock and a lift into swing
description: "The walk's toe tips skimmed 60-200 mm: a rigid toe under the floor in pre-swing, dragged out by the foot IK, nothing holding the tip. Fixed by a per-foot tip lock aiming the toe, held by the gait's clock, never through the floor, and a swing lift. Read before changing toes, foot locks or swing clearance."
type: lesson
status: current
tags:
  - ik
  - locomotion
  - correctness
updated: 2026-10-05
verified: 2026-10-05
code:
  - src/character/anim/walk.rs
  - src/character/anim/plugin.rs
  - src/character/anim/legik.rs
  - src/character/anim/walker.rs
  - src/character/anim/foot.rs
  - src/character/anim/footlock.rs
sources:
  - "Winter 2009, Tables A.2(c)/(d): the toe marker stays down through pre-swing while the fifth metatarsal climbs 6 cm"
  - "tests walk::tests::a_walking_toe_swings_clear_of_the_floor, legik::tests::a_toe_tip_aimed_at_a_point_lands_on_it_or_on_the_line_to_it, footlock::tests::a_followed_foot_never_locks_and_a_let_go_eases_out_of_its_pin"
  - "live: character_gallery --anim-speed-schedule 1:1.2,6:0 / 1:1.85,6:0 / 1:0.6,6:0, --step-seconds 0.0166667, Xvfb, BRP; baseline the build before running was added"
aliases:
  - toe tip skim
  - tip lock
  - AnimFootIk::tips
  - tips_leaving
  - gait_swing
  - aimed_off_the_floor
  - FootLock::let_go
  - FootLock::follow
  - conform_toes
  - aim_toe_tip
  - walk::swing_clearance
---

# A toe tip pivots on the floor, so it needs its own lock and a lift into swing

The walk's toe tips skimmed along the floor at every step, 60-75 mm at
1.2 m/s and 200 mm at 1.85. Three things combined, and each needed its own
fix.

## What happened

- **The rigid toe went under the floor.** The walk replays Winter's foot
  pitch with a rigid toe. On the rig's longer foot the tip went 25-31 mm
  under the floor in pre-swing, with the ball already 8-10 mm up, and was
  still 22 mm under early in swing. The walk's pelvis fit lets a planted
  foot press into the floor, and the foot IK lifts it.
- **The foot IK dragged the tip out.** The IK lifted the tip out of the
  floor (`lift_toe_end_out_of_the_ground`) but nothing held it. The foot's
  own lock holds the toe joint, and a rising ball lets that go. So the tip
  followed the swinging foot along the floor.
- **The rendered foot trailed the target pose.** The legs' springs lag the
  pose, so a tip the pose had lifted stayed down a few more frames (the
  run's toe-off lesson, see
  [running](./running-replays-measured-strides-at-their-froude-number.md)).

## What was tried

- **Toes bent by the foot's pitch** (the run's rule, `foot::toe_bend`):
  the tip, which carries part of the body in pre-swing, rose off the floor
  and the foot floated 9.6 mm.
- **Bending inside the walk cycle's own posing:** the thigh correction and
  the pelvis fit are built on the rigid foot's contacts. Rebuilt on bent
  ones, the ankle's rate jumped across the heel strike and the pelvis
  lurched (7 tests failed).
- **Toes conformed to the floor through stance,** on the finished pose: the
  tip stayed on the floor but slid 7-18 mm along it per pre-swing. Root
  motion follows the contacts, so it took that slide in and stepped
  0.16 m/s as the tip left (the walk's velocity test allows 0.1).
- **A one-sided Newton step** for the bend overshot and left the tip 7 mm
  up. Bisection, bending both ways within straight to 34°, lands it.
- **The swing read off the foot's clearance** (`AnimFootIk::clear`, zero
  through a start's and a stop's fades), and a tip let go before its swing
  held 12 mm up until the swing began:
  - a start's first swing never let its tip go: the lock held it 12 mm up
    through the whole next stance, the ball 6 mm off the floor;
  - in a steady walk the tip let go a frame before the swing, jumped to
    12 mm, then dropped 4-10 mm under the floor as the swing let it go,
    moving 4-16 mm a frame (missed at first: the slide measure counted only
    moves with both frames within 2 mm of the floor);
  - the release, snapped out or left paused through the hold, came back at
    the swing and pulled the tip 13 mm under the floor.

## How it is fixed

- **Swing, on the pose:** `WalkCycle::conform_toes` bends a swinging toe up
  until the tip is `walk::swing_clearance` off the floor. The lift rises
  `x(2 − x)` from wherever the tip left the floor, so the toe does not jump
  at toe-off.
- **Swing, on the rendered foot:** the walker asks the foot IK for the same
  clearance (`AnimFootIk::clear`), fully walking only. A start's and a
  stop's fades lift and set down their own swings.
- **Stance, in the foot IK:** `AnimFootIk::tips` gives each toe tip its own
  lock.
  - A tip that comes down within 4 mm of where it stands flat, and slow, is
    pinned in the world, and the toe is turned about its joint to point at
    the pin (`legik::aim_toe_tip`).
  - **A gait's clock says when it goes** (`AnimFootIk::gait_swing`, set by
    the walker whenever a gait has weight, fades included). Through the
    gait's stance the tip is held whatever its speed, even out of the toe's
    reach (it then lies on the line to its pin, off the floor), and lets go
    by its speed once the swing begins; only a pin 60 % past the toe's
    reach lets go before. With no gait, it lets go by its speed or once
    15 % past the toe's reach (at 4 % a still-low tip skimmed 31-49 mm).
  - **A let-go eases out of the pin** (`FootLock::let_go`), and a swinging
    tip is never locked again (`FootLock::follow`). The release holds the
    tip back across the floor only: a swinging tip rises with its foot.
    Held down too, a start's first swing dragged its tip 19 mm along the
    floor as the foot rose 12 mm.
  - **A tip is never aimed through the floor** (`aimed_off_the_floor`),
    and a pin it could only reach so lets go. The toe is rigid, so aimed
    at a pin nearer than its length its tip goes past: a foot carried
    37 mm over its pin in one frame (the run's change to a walk) swung the
    tip 40 mm under the floor.
  - A tip is never under the floor, nor in a swing nearer it than its
    foot's clearance.
- **Rigid in stance in the walk's own model,** so the support weights,
  root motion and pelvis fit read the contacts they were built for.

## Evidence

Live, every frame at 60 Hz, the toe-tip joint's worst move in a frame with
both frames within 5 mm of its standing height, start and stop included,
and the lowest it went:

| | Before | Tip lock on the foot's clearance | Now |
|---|---|---|---|
| 1.2 m/s | 60-75 mm | 26 mm, 9.8 mm under the floor | 1.7 mm, 0.1 under |
| 1.85 m/s | 208 mm | 16 mm in the first step | 5.4 mm |
| 0.6 m/s | — | 6 mm in the first step | 4.0 mm |

The planted ball moved at most 2.0 mm. The first swing from a stand now
lifts its tip with the foot, 0.3 → 12 → 24 mm over its first three frames.
Cost: the swing lift added 3.8 µs per character per frame to the walk
(`anim_bench --gait walk`, 20.1 → 23.9 µs).

## How to apply

- **Rigid parts:** a rig part replayed rigid (a toe) that real anatomy bends
  will meet the floor somewhere. Find where with the contacts' heights over
  a cycle before tuning anything downstream.
- **Locks:** a foot lock holds one joint. A contact that pivots about
  another point (a toe tip, a heel at strike) needs its own anchor.
- **Lifts:** judge a lift on the rendered foot as well as on the pose. The
  springs trail a fast-turning foot by tens of degrees.
- **Measuring a skim:** sample every frame, and count moves within a few
  millimetres of the floor and the lowest point, not only moves within
  2 mm: a tip flickering between 4 mm under the floor and 6 mm over passed
  a 2 mm band at every step.
- **Contact from the clock:** a gait knows which foot swings, through its
  fades too; a guess from a clipped signal (the clearance, zero in a fade)
  misses exactly the steps that start and end the gait.

## Related

- [Walking foot rocker contact model](./walking-foot-rocker-contact-model.md) — prerequisite: the heel, ball and tip contacts, the tip following the toe bone.
- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — same-trap: the run's rigid toes and its rendered-foot lift.
- [A speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md) — context: the foot locks a tip lock sits beside.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — prerequisite: the travel a tip lock is told, as a foot's.
