---
title: A ladder is climbed limb by limb between holds, the hips set by the lowest foot's reach
description: "ladder::Climbing: limb moves between holds, four-beat lateral, hips where the lowest foot reaches; trunk upright, hips in as far as holds the hands highest, a far reach lifting its shoulder, hips bowing out for a knee to pass a rung. Read before touching ladder.rs. Traps: knee turns into rails, led arms on a fast lean."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-07
code:
  - src/character/anim/ladder.rs
  - src/character/anim/walker.rs
  - src/character/anim/plugin.rs
  - src/character/anim/hand.rs
sources:
  - "McIntyre 1983, Human Movement Science 2:187-195: lateral and four-beat lateral the most used free-choice ladder gaits; contact phases longer than airborne"
  - "Jensen and Holland 2020, IJERPH 17:2897: four-beat limb patterns on rungs; rungs gripped in 59 of 80 tasks"
  - "Simeonov et al. 2020, Applied Ergonomics 82:102911: 0.39-0.44 m/s on rungs 0.305-0.356 m apart, 8.8 % slower down"
  - "US patent 10,240,392 (vertically oriented ladder apparatus), background, as summarised by a web search: on conventional vertical ladders the climber's torso is inclined away from the ladder (the patent itself not read)"
  - "tests ladder::tests::* (7 ladders: 0.22-0.45 m apart, 0.32-0.6 m wide, 0.25 rad lean; 4 also with a landing)"
  - "live: character_gallery --climb-schedule 1:up,16:down (and 1:up,14:slide; and --landing) --step-seconds 0.0166667, Xvfb"
aliases:
  - Climbing
  - Ladder
  - Climb
  - Hold
  - Pattern
  - climbing a ladder
  - ladder climb
  - sliding down a ladder
  - pose_led
  - off_floor
  - LadderGround
  - HandGrip
  - gripped
  - close_hands
  - bow_for
  - shoulder_lift
  - hips_out
  - getting off at the top of a ladder
  - gripping a rung
  - hunched climbing posture
---

# A ladder is climbed limb by limb between holds, the hips set by the lowest foot's reach

Contents: [Decision](#decision) · [Alternatives](#alternatives-considered) ·
[Traps](#traps) · [Consequences](#consequences) · [Revisit when](#revisit-when) ·
[Related](#related)

A walker asked up a ladder (`Walker::ladder`, `Walker::climb`) walks to a
spot square in front of it, gets on, climbs to the top and holds on, or
steps off onto a landing if the ladder has one. Asked down it climbs down
(getting on from the landing first) and steps off; asked to slide it
slides down the rails. `ladder::Climbing` plans and poses all of it.

## Decision

**A climb is a sequence of steps; a step is limb moves between holds.** A
limb holds the floor, a rung, a rail, the landing (`Hold::Top` past the
top, `Hold::Landing` standing on it), or (a hand) nothing. Each move has a
window of its step; its path pulls out of the ladder by its toe's or
fingers' length, goes across, and sets back in.

| Step | Moves |
|---|---|
| Reach | both hands from hanging to their first rungs (from a landing: the rail tops, the feet shuffling to the edge) |
| Up | the lower foot up, then the hand of the foot going next; past the top, the feet onto the landing, the hands onto the rails |
| Down | the hand of the foot that went down last, then the upper foot down |
| Release | both hands let go, standing (floor or landing) |
| Grip, Slide, Land, StepBack | the hips out to `HIPS_OUT` as hands then feet go onto the rails; down; feet on the floor; back to the spot |

Four-beat lateral: one limb at a time, each hand ahead of its own foot
going up, behind it coming down.

**The hips go where the lowest foot just reaches**: square to the ladder,
`hips_out` from the rungs, as high as lets every ankle reach at `REACH`
(0.94) of the leg. The boundary is the lower foot's lift-off going up (the
hips slowest while it swings, fastest as it pushes) and the reaching foot's
landing going down.

**The trunk is upright** (`LEAN` 0): parallel to the rails, which on a
vertical ladder is about 2° back. On a vertical ladder a climber's torso
inclines away from it, not in (a patent's background; see `sources`, a
summary, not read in full). Leant in 0.22 rad with the hips 0.55 of the leg
out, the body folded at the hips over hands held at the chest.

**The hips come in as far as holds the hands highest**
(`pattern_and_hips_for`). The rig's arms (0.50 m to the wrist against
0.89 m legs) are about 17% short for its legs, so from the old 0.55 of the
leg the hands' rung was the one at the chest. Twenty distances from
`HIPS_OUT` (0.55) in to `HIPS_IN_MOST` (0.36) are tried. Each gets its
pattern and the middle of its hand's height above the shoulder through a
hold (`hand_height`). The one holding the hands highest wins, the farther
of equals. Only candidates whose feet pass each other as the spacing has
them (`gap_for`) count, unless none reaches.

**The spacing picks how the feet go** (`gap_for`): a foot passes the other
onto the rung `gap` above it (rise about 0.6 of the leg), or both feet go
to each rung when that is past the leg or the hands' reach. The hand grips
the highest rung whose wrist is within `HAND_STRETCH` (0.85) of the arm
from its lifted shoulder at the step's start (`stretched`).

| Spacing | Feet | Hips out (of leg) | Hands' median rung above the shoulder (of arm) |
|---|---|---|---|
| 0.22 m | pass | 0.42 | +0.46 |
| 0.3 m (0.32-0.6 wide) | pass | 0.37-0.41 | +0.38 to +0.40 |
| 0.3 m, 0.25 rad lean | pass | 0.54 | -0.08 |
| 0.36 m | both to each | 0.55 | +0.10 |
| 0.45 m | both to each | 0.55 | -0.17 |

**A far reach lifts its shoulder** (`shoulder_lift`): past `LIFT_FROM`
(0.85 of the arm), the clavicle (`Bone::LeftShoulder`, pivoting at its
root) turns toward the wrist's target, solved in closed form, at most
`MOST_SHOULDER_LIFT` (0.4 rad) straight up, half that forward, a quarter
down. The same lift is planned with (`lifted`, `reach_to`): reach checks,
grip frames and the trunk's reaching lean all use the lifted shoulder.

**The hips bow out for a knee to pass a rung** (`bow_for`). With the hips
in, a knee comes into the ladder between rungs, as a climber's does, but a
swinging leg's knee rose through a rung. Each Up or Down step's hips bow out
`bow·sin²(πs)` (still at both ends) by the least of `BOWS` (0-0.15 m) that
keeps each knee joint `KNEE_CLEAR` (5.5 cm) from every rung's axis and
rail's middle line, judged on the legs alone (`posed(.., false)`) at 31
moments. Going up needs none; going down about 12.5 cm. Bows are cached
by step (`BowKey`), shared with the copies `pose_led` runs ahead.

**At the top the hips ease back out** (`top_out`) over `TOP_EASE` (2)
rungs as the feet climb past where the hands can follow.

**The elbows point down and back** (`ELBOW_POLE` no sideways part; on a
rail, `RAIL_ELBOW_POLE` out to the side), the pole turning from the held
bar's to the next over the hand's move.

**The fingers close round the bar** (`hand::gripped`): each phalanx a chord
round the bar's circle (16 mm plus the finger's 9 mm half thickness) about
one axis across the palm; `close_hands` eases them in as the hand arrives
and out as it leaves. **The hand is placed for its wrist** (`grip_frame`):
fingers along the forearm, estimated from an analytic elbow (from the
lifted shoulder), tipped `GRIP_FLEX` over the bar; the forearm takes the
roll (`turn_hand`).

**A landing** (`Ladder::landing`): the rails run on 1.07 m past the top
rung and rail grips count as rungs there (`highest_grip`); a foot crossing
the top goes out, up, then across (`over_top`); from the landing the walker
gets on at the edge (`TOP_APPROACH_IN`), or in place within 0.6 m and
0.5 rad. `LadderGround` adds the landing's slab to the ground.

**In the walker:** the root rides the hips; the legs are left free
(`AnimFootIk::legs_free`, `off_floor`); every bone is led ahead of its
spring (`Climbing::pose_led`); the look turns the neck and head only; the
idle's arm and chest oscillators fade as the hands hold, its sway while
getting on; it starts and ends at the foot IK's pelvis drop.

**The slide is the sailor's:** hands round the rails' outsides above the
shoulders, the feet's insides pressed to them, toes out 0.5 rad; down at up
to 2 m/s, 4 m/s² each way, landing at 1 m/s beside the rails, the knees
taking 0.11 of the leg.

## Alternatives considered

- **Turn each knee out about its hip-to-ankle line** (closed form, to keep
  it 0.07-0.1 m out of the rungs' plane, at most 0.6-0.8 rad). It worked with
  the hips 0.55 out. With them in for the hands it splayed the knees past
  the rails (frog-legged), and on a 0.6 m ladder onto them (0.5 cm from a
  rail's middle). Kept out of the plane only near a rung's height, the turn
  still swung knees sideways into the rails (0.1-1.2 cm) and jolted them at
  455-833 m/s². Removed for the bow.
- **A knee check at one posed moment** (an upper foot just set down) to cap
  how near the hips come: passed hips whose knee came 4.9 cm from a rail
  stepping off the floor, failed hips whose knees kept 8 cm. Judged over the
  real motion instead, the knee, not the arm, bound almost every ladder.
- **The hips' distance as the farthest keeping the hands within a share of
  the arm below the shoulder**: rungs 0.45 m apart were held at the chest.
- **A longer arm** would let the hips stay out; a rig change, not taken.

## Traps

- **A ladder normal pointing into the ladder.** `left × up` points away from
  the climber; grips sat behind their rungs, up to 10 cm off. It is
  `up × left`.
- **A down step that ends with the hand** put the foot on its rung before
  the hips lowered: 2-9 cm short. It ends at the foot's landing.
- **The look's spine share and the idle's oscillators move held hands**:
  8-9 cm live from the look's quarter turn of Spine1/Spine2.
- **The handback**: back on the floor the body jumped 4 cm aside and 7.6 mm
  down in a frame (145 m/s²) from the idle's weight shift and the foot IK's
  pelvis drop returning at once.
- **The springs trail a rising root**: unled, held limbs crept 2-4 mm a
  frame; led, 0.23 mm (95th percentile).
- **Arms led ahead of a fast lean go through the ladder.** `pose_led` leads
  each bone by its own spring's lag: the arms (0.087 s) were solved for the
  trunk ahead and set on the trunk now (led less). A reaching lean of 0.49
  rad coming and going within a step put both fists 15-33 cm through the
  ladder. Plain poses were clean. Keep the trunk from moving fast: the hands'
  rung is chosen with margin (0.85 of the arm, the lean's limit 0.95) so the
  lean stays under 0.06 rad.
- **The elbow's pole switched at a step's start**: a hand still on its rung,
  about to go to a rail, took the rail's sideways pole, the elbow went out,
  and the wrist bent 76° round the rung. The pole turns over the move.
- **A searched knee turn jolted** (bisection to its range's end, 1,400
  m/s²; sides flipping near ±π, 2,900 m/s²), and **a soft maximum through
  `exp` overflowed**: kept for whoever turns a knee again.
- **A finger's bend axis taken from its own pose flips** past 90°, the end
  joint bending backward: one fixed axis across the palm.
- **Too few samples miss a swinging knee**: judged at 11 moments a step,
  the bow search saw only the step's ends and chose none; at 31 it sees the
  swing.
- **An antipodal guard that misses**: aiming a lifted arm, `look_rotation`
  landed the elbow 3-8 mm off; see
  [the lesson](./an-antipodal-guard-must-still-land-on-the-target.md).

## Consequences

**Headless** (`puppet_base`, 7 ladders, 4 also with a landing; up and down
or slid; `ladder::tests`):
- the trunk at most 0.06 rad in (0.053 on rungs 0.45 m apart; -0.03 on the
  rest); before, 0.16 rad throughout and up to 0.49 reaching;
- held hands within 0.1 mm of their grips, planted feet within 0.5 mm; no
  moving wrist through the rungs' plane, plain or led;
- knees at least 4.7 cm from any rung's axis and 6.6 cm from a rail's
  middle line; arms at least 9.7 cm from their rail's;
- wrists bent at most 49°;
- the hips accelerate at most 8.5 m/s², a knee 249 m/s²;
- up at 0.42 m/s and down at 0.38 (±0.03, standard); each climb ends within
  1 mm of where it got on.

**Live** (measured before the upright posture, on the leant one): hands
median 3 mm from their grips, planted balls 0.23 mm a frame (95th
percentile), the pelvis at most 3.9 m/s² on the ladder; the middle finger's
joints 20-30 mm from the rung's axis.

**Seen** (Left and Back, gizmos on and mesh off; then the bare mesh), up
and down the standard ladder and up the wide one: the trunk upright, a hand
overhead and one at the face, the knees between the rails, a swinging knee
into the ladder between two rungs.

**Cost** (`anim_bench --gait climb|slide --characters 20`): 52 µs a
character a frame climbing (p99 56), 45 sliding, a walk 24. Without the
bows' cache, every copy `pose_led` ran ahead re-searched its step: 744 µs.

## Revisit when

- **A rig with arms in proportion:** the hips could stay out at `HIPS_OUT`,
  the bows and the shoulder lift shrink.
- **Rungs 0.36 m apart or more:** the hands stay near the shoulders (the
  next rung up is out of reach); a hand moving one rung at a time would
  narrow its range.
- **The approach:** its last steps onto the spot spike the pelvis 15-21
  m/s² sideways; that is the walk's stop.

## Related

- [An antipodal guard must still land on the target](./an-antipodal-guard-must-still-land-on-the-target.md) — deeper: the `look_rotation` miss the shoulder lift exposed.
- [A reach compared against what its own lift clamps it to is noise](./a-reach-compared-against-what-its-lift-clamps-it-to-is-noise.md) — trap: the hand rung's choice was a tie by construction (the lift brings a reach to exactly `HAND_STRETCH`); now within `STRETCH_SLACK`, and passing feet only within `PASSING_LOWER` of the best hands.
- [A sneak carries its hands, placed by arm IK](./a-sneak-carries-its-hands-placed-by-arm-ik.md) — prerequisite: `armik::solve_arm_toward`, the elbow-pole arm solve every grip uses.
- [A jump is planned as its centre of mass's path](./a-jump-is-planned-as-its-centre-of-mass-path.md) — context: `jump::upper` and leading each bone ahead of its spring.
- [A walking arm swings back and its hand hangs relaxed](./a-walking-arm-swings-back-and-its-hand-hangs-relaxed.md) — contrast: the relaxed finger curl `RelaxedHands` holds when not gripping.
- [Walk pelvis rides one sinusoid per step](./walk-pelvis-rides-one-sinusoid-per-step.md) — contrast: the walk's per-step pelvis rate, which a climb's beat mirrors.
