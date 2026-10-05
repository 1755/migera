---
title: A jump is planned as its centre of mass's path, the pelvis solved onto it
description: "jump::Jump plans a standing jump as the COM's path (crouch, push to √(2gh), parabola at g, landing) timed from measured jumps, and solves each pose's pelvis onto it. Traps: heels rising only once the legs are straight; a landing pinned where the sprung foot is. Read before changing jump.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-05
verified: 2026-10-05
code:
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
  - src/character/anim/plugin.rs
  - src/character/anim/footlock.rs
sources:
  - "Winter 2009, p. 88: the COM's path is decided at take-off"
  - "McMahon et al. 2017/2018; McHugh et al. 2024; McInnis & Donahue 2024: countermovement depth 0.31-0.36 m, durations, 0.43 BW least load, 2.2-2.5 BW peak push"
  - "Lees et al. 2004 (arm swing); Vanrenterghem et al. 2008 (trunk lean); Myers et al. 2011, DeVita & Skelly 1992 (landing knee flexion and forces)"
  - "tests jump::tests::*, plugin::tests::a_jumps_landing_pins_each_foot_where_it_lands_not_where_the_sprung_foot_is"
  - "live: character_gallery --jump-at 2:0.35,6:0.15 --step-seconds 0.0166667, Xvfb, BRP every frame"
aliases:
  - jump::Jump
  - Walker::jump
  - countermovement jump
  - AnimFootIk::touchdown
  - Touchdown
  - FootLock::pin
  - Jump::pose_led
  - spring lead
  - jump::lead_of
---

# A jump is planned as its centre of mass's path, the pelvis solved onto it

## Context

A jump has to look physically right: the body flies on a parabola under
gravity, leaves at the speed its height needs, and takes forces through the
floor that a person can make. The walker is kinematic (the ragdoll is
optional), and its height lives in the pose's root translation, which is
not sprung.

## Decision

`jump::Jump::plan(height, stood, rig)` plans the COM's path once:

| Phase | COM path | Duration (0.35 m jump, `puppet_base`) |
|---|---|---|
| Down | Hermite, at rest both ends, 0.85 × height deep | 0.57 s, never below 0.43 body weight on the floor |
| Push | constant acceleration from rest to `√(2·g·h)` | 0.31 s, 1.85 BW on average |
| Flight | parabola at g | 0.55 s |
| Land | constant deceleration, `LANDING_DECELERATION` (g) | 0.28 s, 2 BW |
| Recover | Hermite to standing, at rest | 0.57 s |

Each frame the pose's shape is set first, and then the pelvis is solved so
that the pose's real COM (`anthropometry::centre_of_mass`) lies on the path:
- trunk lean grows with depth;
- the arms swing back in the drop and forward-up through the push;
- the legs stand on the feet (heels rising about the toe tips) or are free
  in flight.

Measured on the posed body, not on the hips. Arms swinging up raise the
COM, and the pelvis then rises less, as a real jumper's does.

## Alternatives considered

- **Drive the ragdoll** and let physics fly: truly simulated, but height
  and distance are hard to hit, and it is not deterministic.
- **A ballistic pelvis** instead of the COM: the arms and the tuck move the
  COM in the body, so the body's real path would not be a parabola at g.
- **A keyframed clip:** no control of height, and the forces are whatever
  the keys imply.

## Traps found live

- **Heels risen only once the legs are straight.** The first plan raised
  them only when straight legs could no longer reach their feet. Pushing at
  constant acceleration, that is the last 0.1 m of the rise: about two
  frames. The sprung legs could not follow, and the pelvis hitched at
  take-off at 357 m/s². A real push straightens hip, then knee, then ankle,
  so the heels now rise over the push's last 60 % (`heel_at`): 33 m/s² at
  worst, the heels rising over 10 frames.
- **A landing pinned where the sprung foot is.** The plan folds the legs
  fast to take the landing, the springs lag, and the rendered toe was 13.8
  mm off and 42 mm under the floor its first frame down. Its foot locks
  pinned it there. Gripped by height it was not pinned at all: a forefoot
  landing has the ball 26 mm up as the tip touches.
  - The walker now passes, every landing frame, where the plan has each
    foot (`AnimFootIk::touchdown`, a `Touchdown`), and the foot IK pins it
    there (`FootLock::pin`).
  - It has to be the plan's ball at the plan's height. Pinned once at the
    flat foot's ball, the foot shifted the ~5 mm the ball rolls as the heel
    comes down about the tip. Pinned at the flat foot's height, the ball
    came 5.5 mm nearer the tip's pin than the toe is long, and the tip,
    put past its pin, crept back 1-2 mm a frame.
  - And the tip's pin has to be the point the tip lock holds, the toe's
    end joint (`legik::toe_tip`), not the sole's tip on the floor under it,
    which a pitched foot puts elsewhere across the floor.
- **Feet carried with the pelvis in flight.** The pelvis moves under the
  arms' swing to keep the COM on its path, and feet hanging from it came
  down moving up to 3.9 mm a frame across. Each ankle's way across the
  floor is now held in the world, from where it left to where it lands,
  the pelvis solved round it.
- **The springs trail a fast plan.** A critical spring trails a target
  moving steadily by `2ζ/ω` (`2·halflife/ln 2`): 0.043 s for the legs,
  0.087 s for the arms. Unled, the arms reached shoulder height at take-off
  where the plan had them 25° above (16.9° off), and the legs, still
  straightening, put the rendered COM 30 mm off the parabola.
  `Jump::pose_led` takes each bone's rotation from the plan its spring's
  lag ahead, not past the phase it is in: led across touchdown, the
  landing's folding legs swung the feet 16 mm the frame before they
  landed. A whole pose (pelvis solved) per lead cost 91 µs a character a
  frame, so the trunk and arms are led from their shape alone.
- **A forefoot touchdown.** Planned flat-footed, the COM touched down below
  where it took off, and a 10 cm hop absorbed 0.29 m. The feet now meet the
  floor on the forefoot (heels 0.35 rad up, knees 15° bent, Myers 2011),
  and the heels come down over the landing's first 40 %.
- **The heel rise sets the take-off height.** On `puppet_base` the heels
  carry 0.094 m of the COM's rise at 0.5 rad, the arms 0.035 m, straight
  knees almost none. At 0.5 rad the COM left 0.14 m above standing. At
  0.35 rad (20°) it leaves 0.11 m, against 0.09-0.11 m from force-plate
  displacements. That is less plantarflexion than joint-angle studies give
  at take-off, a tension kept on the side of the measured COM.

## Consequences

- **Measured against the literature** (`puppet_base`, 0.35 m):
  - knee flexion 103° at the bottom of the countermovement and 105° at the
    bottom of the landing (soft landings flex 117°);
  - the COM leaves 0.11 m above standing (0.09-0.11 m measured);
  - the arms swing with the elbows bent (`ARMS_*` are swing and elbow):
    35° back, 17° up, guarding at 46-69° landing. Swung straight, they
    looked stiff.
  - the arms swing as hard as the jump (`FULL_ARMS`): fully from 0.35 m,
    below it by the square of the height's share (8 % at 0.1 m, a third at
    0.2 m). Scaled in proportion, a 0.1 m hop still swung them 40°
    forward; a small hop hardly moves the arms.
- **Live, two jumps (0.35 and 0.15 m):**
  - pelvis at −9.5 m/s² on average in flight;
  - each toe moves 0.3 mm the frame it lands (9.9 before the pins);
  - feet back within 2.3 mm of where they stood.
- **Rendered against the plan** (headless, through the springs): the COM
  6.6 mm off the parabola in flight, the upper arm 3.7° off at take-off.
- **Cost:** 53 µs per character per frame (`anim_bench --gait jump`, led
  as the walker poses it), against a walk's 23 and a run's 41: two pelvis
  solves a frame.

## Revisit when

- A jump forward or out of a run is added: the COM's horizontal path, the
  feet landing elsewhere, the travel handed to root motion.
- Jumps are many at once: the pelvis solve (8 steps of legs and COM) is
  most of the cost.

## Related

- [A run's flight plan bends the stance legs for a ballistic flight](./a-runs-flight-plan-bends-the-stance-legs-for-a-ballistic-flight.md) — same-trap: the run's flight, ballistic on the pelvis.
- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — context: grip on landing, the sprung legs lagging a fast body.
- [A toe tip pivots on the floor and needs its own lock](./a-toe-tip-pivots-on-the-floor-and-needs-its-own-lock.md) — context: the tip lock the jump's feet use.
- [4.1.4 Multisegment center of mass](../../biomechanics-winter/ch04-anthropometry/4.1-density-mass-inertial-properties/4.1.4-multisegment-center-of-mass.md) — prerequisite: the COM the pelvis is solved onto.
