---
title: A run's flight plan bends the stance legs a few degrees for a ballistic flight
description: "run::FlightPlan shortens each stance leg by one curve (landing sink, mid-stance dip, push out of toe-off) so the pelvis flies at g, leaves the ground rising as recorded and spans the recorded bob, for 7.5-10° of knee (16-17° at 4.5-6 m/s). Read before changing FlightPlan, the run's pelvis or the pelvis drop."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - correctness
  - performance
updated: 2026-10-05
verified: 2026-10-05
code:
  - src/character/anim/run.rs
  - src/character/anim/plugin.rs
  - src/character/anim/pelvis.rs
sources:
  - "Fukuchi, Fukuchi & Duarte 2017, PeerJ 5:e3298 (the recorded pelvis: take-off rate, bob)"
  - "test run::tests::a_running_body_flies_at_g_and_its_legs_give_little_for_it"
  - "live: character_gallery --anim-speed-schedule 1:1.2,4:3.0,9:4.5,13:2.5,17:1.0,20:0 and 1:4.0,7:0,10:5.5,15:2.3,18:0, every frame at 60 Hz over BRP"
aliases:
  - FlightPlan
  - flight plan
  - DIP_AT
  - MOST_DIP
  - pelvis_at
  - recorded_span
  - PLAN_SPEED_STEP
  - pelvis drop for a swinging foot
---

# A run's flight plan bends the stance legs a few degrees for a ballistic flight

A run's pelvis rides its planted legs, and the legs replayed as recorded
do not throw it on a ballistic arc. `run::FlightPlan` bends each stance leg
a few degrees, by one curve over stance, so the body flies at g, leaves the
ground rising at the recording's rate and dips as deep as the recording.

## Context

- **The legs lead:** in stance the pelvis sits where the planted foot puts
  it, as in the walk
  ([running](./running-replays-measured-strides-at-their-froude-number.md)).
  In flight it follows a cubic from its take-off height and rate to its
  landing's. That cubic is a fall at g only if the two ends agree.
- **With the curves corrected** to angles from vertical
  ([the zero](./an-angle-from-standing-is-not-an-angle-from-vertical.md)),
  the ends nearly agree on height: toe-off 9 mm over contact at 3.5 m/s,
  the recording 11. But not on rates: the rig's toes stop bending at 34°,
  and the legs' own rise halves in the last 1 % of stance. And the stance
  dips a third less than the runners' (44 mm against 64).

## Decision

- **One curve of shortening over stance,** `L(q)`, two Hermite halves
  meeting at 45 % of stance (`DIP_AT`). The shortening is a two-link
  knee-and-thigh change keeping the ankle where it was.
  - **At contact** `c` (0-20 mm), leaving at rate `σ`: the body keeps
    falling at the rate it landed.
  - **The dip** `D` (at most 30 mm, `MOST_DIP`), where the knee is bent most
    and gives the most height for its angle, as deep as makes the pelvis
    span the recording's bob (`recorded_span`).
  - **At toe-off,** coming out at rate `τ`: the leg's push, so the body
    leaves the ground rising at the recording's rate (`pelvis_at`). It is
    capped where the landing would come out higher than the legs reach, or
    the dip deeper than its most.
- **Solved in closed form:** the toe-off state, thrown under gravity, lands
  on the contact's height and rate, which sets `c` and `σ`. The dip is
  found by bisection on the bob. All of it is refined six times on the
  corrected legs' own heights.
- **Through swing** the change eases out of toe-off's and into contact's.
  It starts without the push's rate: carried on, the knee straightened into
  the swing and left a foot 4.9 mm off the floor at 2.2 m/s.
- **On a grid:** plans are worked out every 0.1 m/s and every 0.05 of pace,
  and blended between. They are cached for every character on the same rig
  and stance.
- **The hips drop only for feet bearing weight:** the foot IK lowers the
  hips for a foot that cannot reach (`solve_pelvis_drop`), but only for a
  foot a gait has down and not yet in the last 15 % of its stance
  (`AnimFootIk::gait_bearing`, `walker::BEARING`).
  - A run's trailing leg leaves the floor nearly straight, and its lock's
    release out to where it stood put that out of reach. At 4.7 m/s the
    hips dropped 41 mm each flight and sprang back at the landing.
  - Its last frame on its toes, pinned behind a body speeding up, dropped
    them 21 mm for a frame, twice in a 75 s schedule.

## Alternatives considered

- **The legs alone:** on the curves from standing, the pelvis fell 5 cm a
  flight, rising first, at −14 to −5 m/s².
- **The first plan,** a bump over the first 40 % of stance and a ramp over
  the last 40 %. It could only take height off the take-off. On those
  curves the body left level, landed at 0.8-1.2 m/s and spanned 75-98 mm
  against 92-96, for 8-10° of knee (17 at 4.5 m/s). On the corrected
  curves it still left level, having no push.
- **Other ways to a ballistic flight** (human proportions, Python):
  - a least-squares shortening weighted by each sample's knee cost finds
    the early shape as a one-sample spike (~7°);
  - unweighted, it piles 8-14 cm onto toe-off, where a nearly straight leg
    pays the most knee per millimetre;
  - imposing the recording's whole pelvis costs 22-26°.
- **A plan worked out at every speed:** one costs 1.5-2 ms, and a run
  speeding up asks for a new speed each frame. On the grid, a speed-up from
  2 to 4.5 m/s costs 0.36 ms a frame, once per rig.

## Consequences

- **Model** (`puppet_base`, headless):

  | m/s | Take-off | Landing | Bob (recorded) | Most knee change |
  |---|---|---|---|---|
  | 2.2 | +0.22 | −0.42 | 97 (97) | 7.5° |
  | 2.5 | +0.27 | −0.46 | 96 (96) | 9.7° |
  | 3.5 | +0.42 | −0.59 | 92 (91) | 9.1° |
  | 4.5 | +0.27 | −0.75 | 72 (78) | 16.9° |
  | 6.0 | +0.29 | −0.50 | 71 (78) | 10.2° |

  In flight the pelvis accelerates at −10.0 to −9.3 m/s². The runners'
  hips leave at +0.53 and land at −0.73 m/s (3.5 m/s). Without the push,
  the flight accelerates at +3 m/s²; on the curves from standing, the plan
  takes off level and needs 19.8° of knee. Both fail the test.
- **Live,** fitted over the five frames round each flight's top: −8.4 to
  −9.7 m/s² from 2 to 6 m/s. The build before had −24 to −29 at
  4-4.5 m/s, the hips dropping for the trailing foot.
- **Stance accelerations** stay sharp near toe-off: the legs alone reach
  −122..+106 m/s² at 6 m/s, and the plan adds up to a third. Both are from
  the foot's contacts changing from heel to ball to tip, the same on the
  old curves.
- **Cost:** the blended plans add 2.8 µs a frame to a running character
  (`anim_bench --gait run`, 38.9 → 41.7 µs).

## Revisit when

- **The take-off** is under the recording's (0.42 against 0.53 m/s at
  3.5 m/s), held there because the body would otherwise land higher than
  the legs reach at contact. Toes that bent on past 34° would lift the
  legs' own rise at the end of stance.
- **A recording above 4.5 m/s** replaces the held stride: at 4.5-6 m/s the
  plan needs its full dip and 16-17° of knee.

## Related

- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — context: the run this plan corrects.
- [A segment angle measured from standing is not an angle from vertical](./an-angle-from-standing-is-not-an-angle-from-vertical.md) — prerequisite: the corrected curves that made a small plan enough.
- [The recorded pelvis path and leg angles conflict](./recorded-pelvis-path-and-leg-angles-conflict.md) — contrast: the walk keeps its legs' pelvis, with no flight to make physical.
- [A speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md) — context: the run's planted feet from the clock, the pelvis drop's feet on the ground.
