---
title: A block is mantled as a climb up from the floor
description: "Step 7: a mantle onto a block 0.85 m to chest high is the climb-up's plan with a new head (hands on as the knees dip, a drive off the floor, feet onto the face), then its press, step on and stand; also straight from a walk. Read before changing Hanging::mantle or the climb-up's plan."
type: decision
status: current
tags:
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/hang/up.rs
  - src/character/anim/parkour/hang.rs
  - src/character/anim/walker.rs
  - src/character/anim/stance.rs
sources:
  - "tests parkour::hang::up::tests::{it_mantles_onto_a_block_waist_to_chest_high, a_block_too_low_too_high_or_too_shallow_is_not_mantled}"
  - "live: character_gallery --step-seconds 0.0333333 --block 0,-1,180,1.1,2.0,1.0 --mantle-at 1, Xvfb, gizmos on, mesh off"
  - "anim_bench --gait mantle --characters 100"
aliases:
  - mantle
  - Hanging::mantle
  - HangAsk::Mantle
  - climb onto a box
  - FromWalk
  - mantle from a walk
---

# A block is mantled as a climb up from the floor

Step 7 of the [parkour design](./parkour-moves-implementation-design.md),
first part: standing in front of a block waist to chest high, put the
hands on its top and get up onto it.

## Decision

**A mantle is the climb-up's plan with a different start**
(`Hanging::mantle`, `ClimbUp::stand`). The [climb
up](./a-hang-is-climbed-up-from-by-pull-press-and-step-on.md) plans the
hips on a C² spline through shapes measured from the hands: pull, turn
over, press, step on, stand. A mantle replaces the first two knots:
1. **Hands on, knees dip** (0.45 s): the hips 0.1 m down and 5 cm in, the
   trunk leant 0.45 rad. The lead hand moves from where it hangs onto its
   press over the first 0.8 of it, the other over the last 0.8. The arms
   are blended in from the standing pose's over the first quarter.
2. **Drive** (0.25 s): the hips back to standing height and 8 cm in, the
   trunk leant 0.7 rad. The feet stay where they stood until here.
3. Then the climb-up's **press** (0.4 s), **step on** (0.6 s) and **stand
   up** (1.2 s).

Leaving the floor, each foot goes onto the face where its leg pushes at the
press (braced). Under a high table with no face to the floor, it hangs
under the hips instead.

**The foot IK's drop** is eased out over the dip and back in standing up,
so the hips start on the walker's and the root ends on the top's.

**From a walk** (`FromWalk`), it mantles without stopping. Walking in to its
spot, square to the wall within 0.2 rad, faster than 0.3 m/s, the first
foot down within 0.5 m of the spot starts it:
- that foot stays where it came down;
- the other steps in beside it over the dip, lifted 8 cm;
- the hips' path starts at the walk's hips and velocity;
- the whole pose blends from the walk's over 0.25 s, the legs too, before
  their ankles are placed;
- the dip takes 0.6 s, not 0.45: there is the walk's speed to brake, and a
  hand swung back is 1 m from its press.

Each hand reaches from where standing has it at the spot, the arms taking
hold over the blend. The hand swung forward (opposite the foot down) leads;
the feet keep their own lead.

**Its reach** is 0.85 m (about the hips) up to 0.1 m under the standing
shoulders (1.30 m on `puppet_base`, chest high). Lower is a step or a vault.
Higher, the walker grabs and climbs. The top must be 0.55 m deep, and both
hands must reach their presses at the dip and the drive. The walker stands
with its hips 0.3 m from the face (`MANTLE_OUT`).

## Alternatives considered

- **A move of its own**: the press, step on and stand were already planned
  and tested against the block. Only the start differs.
- **A mantle as a grab's jump**: a jump that only reaches a chest-high
  top's lip is not a hang. People press on the top from a stand.

## Traps

- **Each foot is planted until the drive, then both go.** Held on the
  floor a moment longer, the trail foot was 4 cm out of its leg's reach as
  the hips rose.
- **The foot's path up the face bowed into it.** The spline from the hold
  toward the spot on the top pulled inward, a toe 8 mm into the face.
  Below the lip a foot now stays no nearer the face than it started, or
  than the face knot (0.15 m).
- **Leaning to the dip at a shoulder-high wall put a shoulder 9 cm in.**
  The lean is cut to keep the shoulders 0.18 m out under the lip. Even so,
  at 1.4 m an elbow went 1.4 cm in and flipped at 10 m/s, and the hips rose
  at 9.4 m/s². That is a jump's work, hence the chest-high limit.
- **A foot dangling free (a table) from where it stood went out of reach,**
  11 cm, as the hips rose. It moves in its socket's frame, its reach eased
  from the planted leg's own (0.991 of the leg standing) to 0.97. Capped
  at once at 0.97, it moved the planted foot. Capped at the whole leg, the
  knee fell 5 mm short.
- **A block at 0.75 m pulled a hand 3.5 cm off its press** at the drive,
  the shoulders rising out of reach. The reach is checked at the drive as
  well as the dip.
- **The test's starting pose must not carry the foot IK's drop.** With a
  1 cm drop, near-straight knees swing 3 cm forward; that was the test's
  doing, not the mantle's.
- **Walking in, the hands must not reach from the walk's own wrists.** A
  hand swung back was held behind and then brought past the body, its
  elbow flipping round at 24 m/s.
- **Walking in, the swung-back hand must not lead.** It moved at 6.4-6.9
  m/s. Leading the feet with the stepping foot as well put a knee 1.8 cm
  into a chest-high wall, so only the hands swap.
- **A foot's reach cap must start from the leg's own reach.** At a heel's
  strike the leg is near straight (0.998 of it), and a 0.995 cap pulled the
  planted foot in, the knee 1.8 cm off. Capped at the leg's present reach,
  the rising hips asked for the whole leg and the knee fell 5 mm short. The
  cap now eases from the reach it began with to 0.995 by take-off. The foot
  stepping in is capped too: left behind at toe-off at 1.4 m/s, it was 3 mm
  out of reach.

## Consequences

**Headless** (`puppet_base`, 60 fps; blocks 0.85, 0.95, 1.15 and 1.28 m,
and a 1.15 m table with 0.3 m of face):
- it starts in the standing pose where it stood, under 0.1 mm;
- the feet stay planted until they leave, under 0.01 mm, then on the face
  within 1 mm;
- the hands are on their presses within 0.1 mm;
- nothing goes into the block;
- no joint goes over 5.5 m/s (4.9 measured below chest high);
- the hips peak at 7.7 m/s² (the drive, chest high; bound 1 g);
- it stands on the top at its spot.

Walking in at 1.0 and 1.4 m/s, onto every block, the left foot just down:
- it starts within 0.04 mm of the walk's pose;
- the hips peak at 7.8 m/s², counting the hand-over from the walk's speed;
- no joint goes over 4.9 m/s;
- everything else holds as from a stand.

0.75 m, 1.4 m and a 0.3 m deep top are refused.

**Live** (Xvfb, 1/30 s steps, gizmos on, mesh off; a 1.1 m block):
- from the left: the hands reach the lip, then the trunk is over the top,
  the feet on the face, a foot comes on, and it stands;
- from the back: the hands on the top, a shoulder's width apart;
- from the top: nothing inside the block;
- walking in to a 1.1 m block 4 m off, it mantled from the walk at
  0.7 m/s (the approach slowing), 0.33 m from the spot, through the press
  to standing on the top (from the left).

**Cost**: `anim_bench --gait mantle --characters 100`, 44 µs a character,
the same as a climb up (45-50 µs).

## Revisit when

- **From a run**: a run is not mantled from; it would need the walk's
  hand-over at running speed and a jump's drive.
- **Data**: no mantle has measured timings. The LAAS parkour motion
  database has muscle-ups and vaults to measure them from.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is step 7 of.
- [A hang is climbed up from by a pull, a press and a step on](./a-hang-is-climbed-up-from-by-pull-press-and-step-on.md) — prerequisite: the plan whose press, step on and stand a mantle shares.
- [A low obstacle is speed-vaulted as a reshaped running leap](./a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md) — contrast: the other half of step 7, below a mantle's height.
- [Parkour movement data](./parkour-movement-data.md) — deeper: why the timings are by eye.
