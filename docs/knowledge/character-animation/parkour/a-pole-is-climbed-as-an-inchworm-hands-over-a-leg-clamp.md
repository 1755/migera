---
title: A pole is climbed as an inchworm, hands over a leg clamp
description: "Step 9, second part: a vertical pole is jumped onto, the hands one above the other and the feet clamping it, then climbed in 0.6 m cycles of 1.8 s (the arms pull as the legs fold up, the clamped legs stand as the hands go up one over the other). It is also climbed down, slid down braked, gone round, and let go of. The stroke is bounded by the hands' reach. Read before changing parkour/pole.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/pole.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::pole::tests::{a_pole_is_climbed_up_and_down_and_slid_down, a_pole_is_gone_round_climbed_to_its_top_and_let_go_of}"
  - "live: character_gallery --step-seconds 0.0333333 --pole 0,-1.2,5 --pole-ask 1,up,12 --pole-ask 14,left,2.1 --pole-ask 17,down,3.6 --pole-ask 22,slide, Xvfb, gizmos on/mesh off Left and Back, mesh on; BRP capture of pelvis and hands"
  - "anim_bench --gait pole --characters 100"
aliases:
  - pole climb
  - Pole
  - Poling
  - PoleAsk
  - Walker::on_pole
---

# A pole is climbed as an inchworm, hands over a leg clamp

Step 9 of the [parkour design](./parkour-moves-implementation-design.md),
second part: a vertical pole is gotten onto, climbed up and down, slid
down, gone round, and let go of. No pole data exists. The pace is a rope
climb's, about 3 s a metre (an unverified summary,
[movement data](./parkour-movement-data.md)).

## Decision

**A pole** (`parkour::Pole`) is a foot, a height and a radius (2.5 cm).
Holding it (`Poling`):
- the hips are 0.3 m from its axis, facing it, the trunk leant 0.15 rad
  toward it;
- each hand closes round it at its own height, the fingers across toward
  the other side, the palm toward the axis (the ladder's `frame_turn` of
  the hand's own grip);
- each ankle is 0.07 m to its side of the axis, the soles turned in
  0.7 rad, clamping it.

**The climb is an inchworm** of 1.8 s, the hips rising a 0.6 m stroke:
- **the pull** (0.4 of the cycle): the hands held, the hips rise 0.25 m
  on the arms while the feet unclamp and fold up 0.6 m;
- **the stand**: the feet clamped, the legs stand up 0.35 m. The lower hand
  goes up over the upper (0.55-0.85 of the cycle), then the other
  (0.85-1.0), each 3 cm off the pole on its way (`sin²`).

Climbing down is the same cycle run backward from the one that would have
ended where it holds. A cycle under way finishes. It climbs no higher than
the upper hand 0.1 m under the top. At the bottom, asked down, it lets go
onto the floor.

**Getting on** (0.8 s): the hips rise 0.35 m as each hand is raised
forward round its shoulder to the pole (its direction slerped, the elbow
bent on the way), and the feet come off the floor no farther from the hips
than clamped. The standing pose eases out over the first 0.2 s.

**The other moves:**
- **Slide:** gravity braked to 0.3 of itself, to at most 2.5 m/s, the legs
  0.6 m under the hips. It lets go 0.15 m above the floor into a fall at
  that speed, which lands.
- **Round:** while asked and held, it goes round the axis at 1.5 rad/s,
  eased in and out over 0.3 s.
- **Let go:** pushed off at 1 m/s into a fall.

**The walker** (`Walker::pole`, `Walker::on_pole`): `Up` walks to the spot
in front of it (`Poling::spot`, from where it comes) and gets on. The rest
are taken on it. A let go hands it to the fall, reaching to catch if
`Walker::catch`, and drops the ask.

## Alternatives considered

- **A 0.78 m stroke** (2 s cycles): each hand must rise a stroke per
  cycle. Held, it must not fall below about 0.15 m over the hips; put, it
  must be within its arm's reach, about 0.78 m over them. That is a 0.65 m
  range: the hand held longest hung straight down at the hips and its
  elbow swung off its path. 0.6 m fits.
- **The ladder's limb-by-limb climber**: it climbs between discrete rungs
  with feet on them; a pole has neither rungs nor footholds, only a clamp
  that slides.

## Traps

- **Facing along the bearing out to the hips** faced away from the pole.
  The arms reached behind the back to it and still hit it to 1 µm. A lean
  toward the pole made the hands miss by 7 cm; one away fixed them. That
  sign flip was the clue, not a fix.
- **The hand's straight path from beside the thigh to the pole overhead**
  passed 5 cm from the shoulder: the arm folded shut and the elbow flipped
  17 cm in a frame. Swept round the shoulder, then also bent on the way (at
  full length, near straight, the swivel swung 4 cm a frame).
- **The hips rising ahead of the feet getting on** stretched the leg past
  straight (0.9 m of 0.87) and the knee flipped 4.7 cm in a frame.
- **An upper hand put before the hips are high enough** is out of reach;
  the wrist stopped 2-5 cm short.
- **A hand arcing off the pole with `sin`** stops dead on it (3.5 cm
  kink); `sin²` has no speed at either end. A 6 cm arc, toward the body
  and so the shoulder, still swung the elbow 2.07 cm.
- **Let go from the slide at the holds it slid from**: the hips went back
  up 3 m in a frame.

## Consequences

**Headless** (`puppet_base`, 60 fps; a 5 m pole met from three headings,
a 3.2 m one):
- got on, three cycles up (1.8 m, exact), down again to where it got on;
- slid to the floor at no more than 2.5 m/s and landed;
- went round a half turn and back;
- climbed to 0.1 m under the top and no higher; let go and landed clear.

Throughout:
- the held hands within 0.6 µm of the pole and the feet within 0.6 µm of
  their clamps;
- nothing into the pole;
- no joint over 4.5 m/s about the hips;
- no joint's step changing over 1.5 cm in a frame, the slide's hand-off
  to the fall included.

**Live**: walked to the pole, got on, climbed to its top limit (hands at
4.2 and 4.4 m on a 5 m pole), went half round, climbed down two cycles,
slid down and stood.

**Cost**: `anim_bench --gait pole --characters 100`, 39 µs a character at
p50 for a climbing cycle.

## Revisit when

- **A pole swing from a run** (grabbing a pole on the way past and swinging
  round it to fling off): not built; going round is only held.
- **Poles not vertical**, or ending in a ledge to climb onto.
- **Pole data**: the pace and the stroke are a rope climb's and the
  reach's.

## Related

- [A bar is swung on, pumped, and let go of at a bar ahead](./a-bar-is-swung-on-pumped-and-let-go-of-at-a-bar-ahead.md) — contrast: step 9's horizontal bar.
- [A ladder is climbed limb by limb between holds](../ik-and-locomotion/a-ladder-is-climbed-limb-by-limb-between-holds.md) — contrast: the hand grip and shoulder lift this reuses, on discrete holds.
- [A near-straight leg is bent toward its kneecap](../ik-and-locomotion/a-near-straight-leg-bends-toward-its-kneecap.md) — same-trap: a two-bone solve's hinge near straight.
