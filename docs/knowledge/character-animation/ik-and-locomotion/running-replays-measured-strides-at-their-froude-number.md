---
title: Running replays measured strides at their Froude number, the legs leading the pelvis
description: "Above Froude 0.5 (2.09 m/s) a walker runs Fukuchi's treadmill strides (2.5-4.5 m/s) replayed as segment attitudes, changed to and from the walk in one step of single support, its feet gripped as they land. Read before changing run.rs, the walk-run change, or foot contact at speed."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-06
verified: 2026-10-06
code:
  - src/character/anim/phase.rs
  - src/character/anim/run.rs
  - src/character/anim/gait.rs
  - src/character/anim/walker.rs
  - src/character/anim/foot.rs
  - src/character/anim/footlock.rs
  - src/character/anim/plugin.rs
  - tools/extract_running_strides.py
  - examples/anim_bench.rs
sources:
  - "Fukuchi, Fukuchi & Duarte 2017, PeerJ 5:e3298, data doi:10.6084/m9.figshare.4543435 (28 runners, 2.5/3.5/4.5 m/s)"
  - "Kram, Domingo & Ferris 1997, J Exp Biol 200:821-826 (walk-run change at Froude ~0.5)"
  - "Segers, Aerts, Lenoir & De Clercq 2006, Gait Posture 24:247-254 (one transition step)"
  - "Thorstensson, Nilsson, Carlson & Zomlefer 1984, Acta Physiol Scand 121:9-22 (trunk lean 6-13 deg, running 2-6 m/s)"
  - "Arellano & Kram 2011, J Biomech 44:1291-1295 (running step width 3.95 % of leg length)"
  - "McDonald et al. 2016, PLOS ONE 11:e0152602 (toe dorsiflexion at push-off 34.2 deg barefoot, 30.1 shod)"
  - "Macadam et al. 2018, Strength Cond J 40(5):14-23 (sprint arm ranges)"
  - "tests run::tests::*, footlock::tests::a_gripping_foot_locks_where_it_lands_at_any_speed, foot::tests::the_tip_goes_with_the_toe_as_it_bends"
  - "live: character_gallery --anim-speed-schedule 1:1.2,4:3.0,9:4.5,13:2.5,17:1.0,20:0 and 1:4.0,7:0,10:5.5,15:2.3,18:0, --step-seconds 0.0166667, Xvfb, BRP"
aliases:
  - run
  - running
  - jog
  - LegCurves::Run
  - GaitParams::running_for
  - running_like_recorded
  - RunCycle
  - run::Gaits
  - run::paced
  - changeover_speed
  - CHANGEOVER_FROUDE
  - BACK_TO_WALK
  - toe_bend
  - swing_clearance
  - AnimFootIk::grip
  - AnimFootIk::clear
  - update_gripped
  - GRIP_HEIGHT
  - fukuchi_running_strides.csv
  - walk-run transition
---

# Running replays measured strides at their Froude number, the legs leading the pelvis

Contents: [Context](#context) · [Decision](#decision) ·
[Alternatives considered](#alternatives-considered) · [Traps it hit](#traps-it-hit) ·
[Consequences](#consequences) · [Revisit when](#revisit-when) · [Related](#related)

A walker asked for more than a walk's speed runs a measured human run: the
legs replay recorded treadmill strides, the pelvis rides them, and the gait
changes to and from the walk in one step where only one foot is down.

## Context

`GaitParams::running()` was a hand-shaped placeholder that the walker
switched to, fixed and unscaled, at 2.2 m/s. No recorded run backed it.
Winter records only a walk.

## Decision

- **Data:** Fukuchi et al. 2017. 28 runners on an instrumented treadmill at
  2.5, 3.5 and 4.5 m/s. `tools/extract_running_strides.py` reduces their raw
  markers and vertical force to one mean stride per speed
  (`assets/anim/reference/fukuchi_running_*.csv`), with these properties:
  - every curve is a segment attitude: thigh and shank from vertical along
    the bones (each cluster's change from the runner's standing trial,
    plus the bone's standing attitude, see
    [an angle from standing](./an-angle-from-standing-is-not-an-angle-from-vertical.md)),
    knee, foot from flat, pelvis tilt and pelvis height;
  - contact is taken where the force exceeds 20 N;
  - 2,100-2,400 strides per speed.

  The extracted strides match the running literature:

  | | 2.5 m/s | 3.5 m/s | 4.5 m/s |
  |---|---|---|---|
  | Stride | 0.745 s | 0.703 s | 0.657 s |
  | Cadence | 161 steps/min | 171 | 183 |
  | Foot down (share of stride) | 0.385 | 0.337 | 0.322 |
  | Contact | 0.29 s | 0.24 s | 0.21 s |
  | Swing knee fold | 92° | 108° | 119° |
  | Pelvis bob | 93 mm | 89 mm | 76 mm |

- **Replay** (`run::RunCycle`) works like the measured walk: the thigh is
  driven by attitude, the knee by angle, and the foot is placed by pitch.
  - Between recordings each curve is interpolated, after retiming each so
    its toe-off falls at the interpolated stance share. Below the slowest
    the curves are extrapolated, to 2.0 m/s; above the fastest (4.5) the
    stride holds and only the cadence rises (`LegCurves::Run`'s `pace`).
  - A rig proportioned unlike the runners runs at the same Froude number.
- **The legs lead and the pelvis follows,** as in the walk: in stance it
  rides where the planted foot holds it, in flight a cubic from take-off to
  landing. A flight plan bends the stance legs a few degrees so that cubic
  is a fall at g, leaving the ground rising and dipping as deep as the
  recording ([the flight plan](./a-runs-flight-plan-bends-the-stance-legs-for-a-ballistic-flight.md)).
- **Toes bend at push-off,** up to 34° (McDonald) while the foot pitches
  toe-down, and straighten over early swing. `foot::Sole` now carries the
  tip in the toe bone's frame, so the contacts see the bend. A pose that
  leaves the toes alone reads exactly the contacts it did before.
- **Trunk and pelvis:**
  - the trunk leans 6° to 13° from 2 to 6 m/s (Thorstensson);
  - the pelvis tilts forward and rocks as recorded;
  - the step width is 3.95 % of leg length (Arellano & Kram).
- **Arms:** authored from published ranges, since the recordings have no
  arm markers. The elbow stays near a right angle (68-106° at 3.5 m/s) and
  the upper arm swings mostly behind the shoulder. The forearm turns in
  toward the midline as it comes forward (`GaitParams::arm_inward`,
  0.35 rad at the front of the swing): from the front, the hand comes to
  0.17 m off the midline, inside the shoulder (0.22), where it was carried
  0.24 m out, outside it; at the back it stays by the hip.
- **In the walker:**
  - **When it runs:** above `run::changeover_speed`, Froude 0.5 (Kram):
    2.09 m/s on `puppet_base`. It walks again below 0.9 of that.
  - **Changing gait (`run::Gaits`):** one step, on the walk's single
    support, from the other foot's toe-off to the run's own toe-off. That
    is the only stretch where both gaits have the same one foot down
    (Segers: one transition step). The gait coming in is posed where its
    planted foot matches the one going out's, and the clock moves onto it
    at the end
    ([two gaits at one clock](./two-gaits-blended-at-one-clock-disagree-on-the-planted-foot.md)).
  - **Speed is eased (`run::paced`):** it gathers at 2 m/s², and above a
    walk only once the walk is fully in. Coming down it sheds 3 m/s², holds
    at the back-to-walk speed until the gait is a walk, sheds on to a
    slower walk the same way, then stops as a
    walk does. A walk's speed holds while both its feet are down
    ([two gaits at one clock](./two-gaits-blended-at-one-clock-disagree-on-the-planted-foot.md)).
  - **Root motion while running:** the run's own speed, every frame. Its
    clock is set so a stride covers exactly that, so over a stride the feet
    do not drift, and the locks hold each foot while it is down. A runner's
    body changes speed by a few per cent through a stride.
  - **No walk sway on a run:** the locomotion layer's walk sway and pelvic
    turn fade out with the run (`walker::fade_walk_sway`). The run's
    pelvis tilt is in its own pose.
  - **Feet:** the feet down come from the clock (`AnimFootIk::planted`). A
    landing foot locks within 1 cm of where its toe joint stands flat
    (`grip`), at that height. A swinging foot's rendered sole is held off
    the floor from toe-off on (`clear`).

## Alternatives considered

- **Imposing the recorded pelvis on the recorded legs:** 4 % more leg than
  the rig has and 21-23° more knee, on curves from standing, not vertical
  ([the zero](./an-angle-from-standing-is-not-an-angle-from-vertical.md)).
  Other ways to a ballistic flight are in the flight plan's note.
- **Extrapolating the strides past 4.5 m/s,** to 6: the landing would have
  had to come down 9 cm (38° of knee) to keep the flight ballistic, and
  past 4.6 m/s capped at 2 cm it could not. Holding the fastest stride, the
  flight at 5-6 m/s is ballistic again.
- **Joint angles relative to the pelvis** (Fukuchi's processed Visual3D
  files, the hip at 33-60°): these carry the pelvis's own 2-9° rocking onto
  the leg.
- **The rendered contacts as root motion while running,** as the walk does:
  every landing braked the body from 3.2 to 0.3-1.5 m/s for a frame or two
  and it ran 15 % slow. The sprung leg is still swinging forward behind its
  target when it lands, and a run has no second foot down to carry the body
  through that.
- **The gait's contact velocity as root motion** (`locomotion::
  root_velocity_of` on the target, coasting through flight): the velocity
  that keeps the stance foot's contact still. The recorded heel lands still
  moving forward (18 mm over the first tenth of stance, carried at the
  run's speed), so read off it the body slowed to 2.4 m/s at every contact
  of a 4 m/s run. The pelvis stepped 45 mm in a frame of 70, once a step.
  Moved at the run's speed instead, it goes exactly 4.00 m/s every frame;
  the planted points' frame moves fell (balls 1.2 to 0.5 mm, toe tips 5.7
  to 2.4).

## A walk's sway on a run's clock

The locomotion layer turns the pelvis about whichever feet a walk's stance
timing (a 0.6 stance share) has loaded. On a run's clock, a third of the
stride down and flights between, that pivot jumped between the feet once a
step, and the root with it: 15 mm back in three frames. The root itself
moved at exactly 4.00 m/s while the pelvis read 3.6-4.08. Faded out with
the run, the pelvis goes at the run's speed. Test
`walker::tests::running_the_layer_leaves_the_root_going_steadily`, which
also shows the walk's sway on a run's clock jolting the root over 3 mm.

## Traps it hit

- **Joint heights cannot tell a heel strike from the air.** Judged by the
  ankle, ball and toe-tip joints each against its standing height, the
  rendered feet were both "up" 55-58 % of the time at 4-5.5 m/s against the
  pose's 36 %: a "landing 6 frames late". A heel strike has the foot toes
  up, all three joints high, the heel on the floor. Measured on the sole
  in the foot IK, a foot the clock has down is on the floor from its first
  frame (0.4 mm on average, 10 mm in 1 % of frames, the first). A
  goal-velocity leg spring, tried against the supposed lag, took that to
  4.5 mm but let the walks slide at their sharp events: not taken.
- **Rigid toes lifted the pelvis.** Recorded feet pitch 44° toe-down by
  toe-off. A rigid toe tip reached 5 cm below the ball, and the pelvis rose
  65 mm over standing at toe-off, its highest point there instead of in
  mid-flight. With bent toes the bob at 3.5 m/s is 88 mm against the
  recording's 89 (92 scaled up to the rig's longer leg).
- **The planted flag only keeps a lock; it never makes one.** A run's
  landing foot never slowed to the locks' speed test (0.15 m/s), bobbed
  20 mm in mid-stance, and slid with the springs' lag. Grip locks it.
- **A lock anchored at the pitch-following surface held the foot up.**
  Gripped at heel strike with the toes up, the toe joint was pinned 21 mm
  (and once 35 mm) above where it stands flat, all stance.
- **The lift has to be judged on the rendered foot.** The pose's toe was
  3 cm up 0.02 of a cycle after toe-off, but the legs' springs trail a foot
  pitching about 1000° a second by about 20°. The rendered tip stayed on
  the floor five frames and skimmed 2 cm a frame. This is the stop's lesson
  again (see the fade note).
- **The trailing foot dipped below the floor** by up to 26 mm at the slowest
  run, just before the other foot's contact. A knee guard (a few degrees
  more fold) lifts it.
- **Gathering speed through the start** took the walk's first step at
  3-4 m/s, and a foot slid 44 mm. The speed is now held at the changeover
  until the walk is in.
- **Shedding speed while still running** carried the gait down to 0.2 m/s as
  a run. The change waits for a step's stretch, the clock had all but
  stopped, and the last foot hung for 1.4 s. The speed is now held at the
  back-to-walk speed until the change is through.

## Consequences

- **Model** (`run::tests`, `puppet_base`):
  - cadence 4.3-5.8 % quicker than recorded at the same Froude number
    (within 1 % with rigid toes, which lifted the pelvis);
  - in flight the pelvis falls at −10.0 to −9.3 m/s², leaving the ground
    rising and landing at 0.42-0.75 m/s, the bob within 15 % of the
    recording, the knee off it by 7.5-17° (numbers in the flight plan's
    note);
  - from the front, a hand at the front of its swing inside the shoulder
    line (`a_running_hand_comes_in_across_the_body_at_the_front_of_its_swing`);
  - a foot down within 1 mm of the floor, never two down, and flight
    exactly `1 − 2·duty` of the cycle;
  - a swinging foot never more than 1.5 mm into the floor, over 5 mm up
    through 5-90 % of its swing, and over its landing's own clearance after
    (landing slower, it comes down 0.9 mm per 1 % of swing at 2.2 m/s);
  - the legs are mirrors of each other within 2 mrad;
  - the knee folds 1.5-1.75 rad at 2.5 m/s and 1.95-2.2 rad at 4.5 m/s.
- **Live** (gallery, BRP, every frame at 60 Hz):
  - speeds as asked: 4.07 and 5.58 m/s by displacement;
  - the pelvis spans 99 / 75 / 100 mm at 3.2 / 4.7 / 2.6 m/s;
  - fitted over the five frames round each flight's top, the pelvis
    accelerates at −8.4 to −9.7 m/s² from 2 to 6 m/s (the build before:
    −24 to −29 at 4-4.5 m/s, the hips dropping for the trailing foot);
  - the planted ball moved at most 1.6 mm floor-to-floor, through walk →
    run → faster → slower → walk → stop, and through a start straight into
    a run and a stop straight from one;
  - toe tips moved up to 5.2 mm floor-to-floor while running, and never
    went under the floor;
  - a stop from 2.3 m/s is at rest 1.1 s after it is asked.
- **Seen** at 3 m/s, Left and Front, gizmos then the mesh: a stance shank
  near vertical under forward-leaning hips; the swing heel kicked up
  behind; a flight with both feet clearly off the floor, the trailing leg
  near straight behind, toes pointed; no knee bent backwards; a narrow
  track with no crossing; elbows near a right angle, hands ahead of them.
- **Cost** (`anim_bench --gait`, synthetic rig, 100 characters, per
  character per frame): run 41.7 µs, walk 24.1, no gait 4.1 (p99 5.1 ms per
  100 running; the plans blended from the grid cost the run 2.8 µs). The first flight plan's leg changes added 7 µs to the run
  (31.9 before); the walk's swing toe lift added 3.8 (20.1 before). A plan
  is worked out in 1.5-2 ms the first time any character of a rig runs at
  a speed on its grid.

## Revisit when

- **The arms** are authored, the hands' crossing included. A recording
  with arm markers would settle their shape.
- **Above 4.5 m/s** the stride holds and only the cadence rises (about
  250 steps a minute at 6 m/s). Sprinting needs a recording.
- **The walk's toe tips:** fixed separately, see
  [a toe tip pivots on the floor](./a-toe-tip-pivots-on-the-floor-and-needs-its-own-lock.md).

## Related

- [Replay a recorded gait by segment attitudes](./replay-a-recorded-gait-by-segment-attitudes.md) — prerequisite: the zeros both replays use.
- [Two gaits blended at one clock disagree on where the planted foot is](./two-gaits-blended-at-one-clock-disagree-on-the-planted-foot.md) — detail: how the walk-run change keeps the planted foot's pace, and the speed easing.
- [A segment angle measured from standing is not an angle from vertical](./an-angle-from-standing-is-not-an-angle-from-vertical.md) — prerequisite: why the run's thigh and shank carry the bones' standing lean, and the 4-5 cm it cost without.
- [A run's flight plan bends the stance legs a few degrees for a ballistic flight](./a-runs-flight-plan-bends-the-stance-legs-for-a-ballistic-flight.md) — detail: how the pelvis is made to fly at g, push off and dip, and what it costs the knee.
- [The recorded pelvis path and leg angles conflict](./recorded-pelvis-path-and-leg-angles-conflict.md) — context: the walk's version of the same conflict, resolved the same way.
- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — prerequisite: why the walk-run change runs in single support, and the rendered-foot lift.
- [A speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md) — context: planted feet from the clock, and now gripped.
- [Walking foot rocker contact model](./walking-foot-rocker-contact-model.md) — context: the three contacts, the tip now following the toe.
- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — contrast: why a run moves at its own speed instead.
- [Walk step width and sideways sway](./walk-step-width-and-sideways-sway.md) — context: the walk sway a run fades out.
