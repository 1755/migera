---
title: A run leans whole into a turn, and its trunk with its speed
description: "Step 11, first part: a run leans by its acceleration, tan θ = a/g. Into a turn the whole body rolls by v·ω about its outer ankle, the feet kept where the gait put them; gathering or shedding speed only the trunk pitches. A whole-body pitch leaves the trailing foot out of reach. Read before changing parkour/lean.rs."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/lean.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
sources:
  - "tests parkour::lean::tests::{a_turn_rolls_the_whole_body_over_the_outer_foot, gathering_speed_pitches_the_trunk_by_its_acceleration, a_lean_eases_into_and_out_of_a_turn}"
  - "live: character_gallery --anim-speed 4 --anim-turn 1.0 (BRP pelvis and neck, circle fit); --anim-speed-schedule 16:5,24:0 (trunk pitch); Xvfb, gizmos on/mesh off and mesh on, Back"
  - "anim_bench --gait run-lean --characters 100"
aliases:
  - turn lean
  - sprint start lean
  - acceleration lean
  - Lean
  - lean_for
---

# A run leans whole into a turn, and its trunk with its speed

Step 11 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md),
first part: a running body whose speed or way changes leans by its
acceleration, `tan θ = a/g`. This is mechanics, not data: the ground's
push must pass through the centre of mass.

## Decision

**What the lean asks** (`lean_for`):
- into a turn, a roll of `atan(v·ω/g)`, with `v` the speed and `ω` the
  facing's turn rate; a positive turn about `+Y` is to the body's left
  whichever way the rig faces;
- gathering speed, a pitch of `atan(a/g)` forward; shedding it, back;
- at most 0.55 rad of roll and 0.45 rad of pitch.

**How it is posed** (`lean`):
- **The roll is the whole body's**, about a line along the body through
  the outer ankle (the right one leaning left). The hips move toward the
  turn. Each ankle is put back where the gait had it and each foot turned
  back to its attitude. The outer leg keeps its length exactly; the inner
  one bends.
- **The pitch is the trunk's only**, from the first spine bone. Nothing
  below the hips moves.

**The walker** springs the lean toward what is asked (critically damped,
half-life 0.12 s), scaled by how far its gait is a run, and only walking
fully, off holds and out of a jump. The turn rate is last frame's facing
change. The acceleration is the pace's change, so a run gathering at
`run::ACCELERATION` (2 m/s²) leans its trunk 11.5° further forward, and
shedding at 3 m/s² it leans 17° back.

## Alternatives considered

- **Pitching the whole body forward too**, about the ground under the
  hips: over a steady run's stance, the trailing foot at toe-off is about
  0.35 m behind the hips and the leg is nearly straight. At 11.5° the hips
  must sink about 9 cm to keep it reachable, or it comes about 7 cm short;
  the run locks that foot to the ground, so it slides. A whole-body lean in
  acceleration needs foot placements a steady run's cycle does not have.
- **Pivoting about the stance foot**: the stance foot moves back through
  the pose frame at running speed, and a rigid lean about a moving pivot
  adds `θ·v` of vertical speed to the hips: 0.8 m/s at 0.2 rad and 4 m/s,
  a 16 cm bob a stance.
- **Rolling about the ground under the outer foot** instead of its ankle:
  a straight outer leg came 1 mm short.

## Traps

- **A turn starts at the facing's full rate.** Leant at once, the roll
  moved the hips 5 cm in a frame. Sprung with a 0.08 s half-life, a
  joint's step still changed 1.3 cm in a frame; at 0.12 s, 4.8 mm.
- **The sign of a roll about the rig's forward** depends on the rig's
  facing. The lean is built as the arc from `+Y` to `Y·cos θ + left·sin θ`,
  so it needs no sign.

## Consequences

**Headless** (`puppet_base`, a 3 and a 4.5 m/s run through a stride):
- rolled 0.2, −0.35 and 0.55 rad, the trunk tilted by the roll within 1°;
- the hips moved toward the turn and each foot stayed within 1 mm and
  turned 0;
- pitched by 2, −3 and 4 m/s², the trunk pitched `atan(a/g)` within 1°,
  nothing below it moving;
- a sudden 4 m/s turn at 1.5 rad/s: the lean reached 98 % in 1 s, never
  overshot, and no joint's step changed over 4.8 mm a frame for it.

**Live**:
- running a circle at 4 m/s and 1 rad/s, the trunk rolled 22.2° toward
  the centre (sd 0.6°), as `atan(4/9.81)` = 22.2°;
- the pelvis circled 0.3 m inside the feet;
- from behind, the stance foot was planted outward with the body leant
  over it;
- starting to 5 m/s, the trunk pitched 14-17° gathering, 7-9° at speed
  (the run's own lean) and −7 to −9° braking.

**Cost**: `anim_bench --gait run-lean --characters 100`, 54 µs a character
at p50 against the plain run's 44.

## Revisit when

- **Acceleration foot placements**: a sprint start's stance feet behind
  the hips; then the whole body can pitch.
- **A walk's turns**: unleant now (its sideways acceleration is taken by
  where it steps).
- **A move handed a leant run** (a vault or jump at a turn): it re-poses
  the run unleant and the lean vanishes in a frame, smoothed only by the
  springs.

## Related

- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 11's design.
- [Running replays measured strides at their Froude number](../ik-and-locomotion/running-replays-measured-strides-at-their-froude-number.md) — prerequisite: the run cycle this leans, and its own trunk lean.
- [Foot locks need the body's travel](../ik-and-locomotion/foot-locks-need-the-bodys-travel.md) — context: why the lean keeps the feet where the gait put them.
