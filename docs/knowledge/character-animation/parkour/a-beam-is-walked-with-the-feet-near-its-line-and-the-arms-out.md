---
title: A beam is walked with the feet near its line and the arms out
description: "Step 10, first part: on a beam the walk keeps its feet 0.4 of their step width apart, slows to 0.7 m/s, holds the arms out 75° swaying against the trunk, and steers onto the line, taking a walker a step below beside it. Read before changing parkour/beam.rs or the walk's step width."
type: decision
status: current
tags:
  - locomotion
  - balance
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/beam.rs
  - src/character/anim/gait.rs
  - src/character/anim/walk.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::beam::tests::{a_beam_holds_a_root_on_it, on_a_beam_the_feet_walk_on_its_line, on_a_beam_the_arms_are_held_out_against_the_sway}"
  - "live: character_gallery --step-seconds 0.0333333 --beam 0.05,-1,0.05,-8,0.25 --anim-speed 1.2, Xvfb, gizmos on/mesh off Front and Left, mesh on, Top; BRP capture of pelvis, feet and hands"
  - "anim_bench --gait beam / walk --speed 0.7 --characters 100"
  - "da Silva Costa et al. 2022; Lambrich et al. 2025 (parkour-movement-data, Beams)"
aliases:
  - beam walk
  - Beam
  - balance on a beam
  - GaitParams::feet_apart
  - Walker::beams
---

# A beam is walked with the feet near its line and the arms out

Step 10 of the [parkour design](./parkour-moves-implementation-design.md),
first part: walking along a narrow beam.

## Decision

**A beam** (`parkour::beam::Beam`) is a line at its top, `a` to `b`, 10 cm
wide by default, standing on the floor. Its top is a ledge (`Beam::ledge`)
in the walker's ground, so it is walked on as any top. The walker's beams
are `Walker::beams`.

**On one** (the root over it within 0.15 m of its edges, from a step below
its top to just above), the balance eases in over 0.4 s:
- **feet:** the walk's step width is cut to 0.4 of itself
  (`GaitParams::feet_apart`), each foot's middle about 2.6 cm off the
  line. The width changes in four steps, each a walk cycle built and
  cached;
- **pace:** at most 0.7 m/s, the measured 0.69-0.82 on 10-6 cm beams;
- **steering:** back onto the beam's line (`back_to_line`), along it the
  way nearer its facing;
- **arms:** held out 75° up from hanging, the elbows bent 0.35 rad forward,
  the walk's arm swing gone (`beam::balance`);
- **trunk:** a sideways sway of 0.08 rad at 0.55 Hz, the arms tilted
  against it like a seesaw.

## Alternatives considered

- **The feet exactly on the line**: the swinging foot passes through the
  planted one at mid-swing. 5 cm apart, it passes beside it, both still on
  a 10 cm beam.
- **A beam taken only with the root up at its height**: a walker 9.5 cm
  off its line walked its whole length straddling it, one foot on it, one
  on the floor. A beam now also takes a walker a step below, beside its
  line, and steers it on.

## Traps

- **The elbow bent about the arm's own lift** folded the forearm up and
  stood the hand by the head. Bent forward, it reads as balancing.
- **The walk's arm swing kept under the lift** sent one arm forward and
  the other back.
- **A share of the step width changing smoothly** would build a new walk
  cycle every frame (the cycle cache keys on it); four steps cache four.

## Consequences

**Headless**:
- walking, the feet sit 7.2 cm off the line under the hips; on a beam,
  3.7 cm at most and at least 5.4 cm apart;
- held out, each hand is over 0.35 m beyond its shoulder and no lower
  than 0.25 m under it; the side swayed to is raised.

**Live**: walked onto a 0.25 m beam from 9.5 cm off its line, then along it.
The pelvis rode 0.25 m up, the feet within 3.5 and 2 cm of the line, the
hands 0.6-0.75 m out at shoulder height rising and falling with the sway,
at about 0.7 m/s.

**Cost**: `anim_bench --characters 100`, 26.6 µs a character at p50 against
a 0.7 m/s walk's 24.4.

## Revisit when

- **A slip or a wobble that needs a recovery** (step 10's teeter): the
  sway is a fixed oscillation, not a response to the body's balance.
- **Beams too high to step onto**: only a step up gets on; a jump onto one
  is step 11's precision landing.

## Related

- [Parkour movement data](./parkour-movement-data.md) — data: beam paces and the arms' and trunk's ranges.
- [Walking is kept out of walls](./walking-is-kept-out-of-walls.md) — context: `back_to_line`, the steer that holds it on the beam.
