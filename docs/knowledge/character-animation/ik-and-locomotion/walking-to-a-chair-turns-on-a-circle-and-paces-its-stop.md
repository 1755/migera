---
title: Walking to a chair turns round on a small circle, paces its last steps to stop on the spot, and the seat makes up the rest
description: "A walker given a chair walks a Dubins path (straight, then a 0.25 m circle, Robinson's ~1.5 s 180° turn) to stand with its back to it. Stops come only in half strides (0.29 m off), so the last 1.5 m are paced; the seat's depth takes the last centimetres. Read before changing approach.rs."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - correctness
updated: 2026-10-03
verified: 2026-10-03
code:
  - src/character/anim/approach.rs
  - src/character/anim/walker.rs
  - src/character/anim/sitting.rs
  - examples/character_gallery.rs
sources:
  - "Robinson et al. (2018), The Timed 180° Turn Test for Assessing People with Hemiplegia from Chronic Stroke, BioMed Res Int, https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5820648/ — healthy adults turn 180° in a median 2.5 steps and 1.5 s"
  - "Lee, Lishman & Thomson (1982), Regulation of gait in long jumping, J Exp Psychol Hum Percept Perform 8(3):448 — step length adjusted over the last few strides to land on a target"
  - "tests approach::tests (a point walk that stops as the transition does, from five starts at four stride phases)"
  - "live: character_gallery --sit chair:upright --chair X,Z,HEADING --step-seconds 0.0166667 on Xvfb, BRP joint positions and the walker's arrival log"
aliases:
  - approach
  - Approach
  - Chair
  - stand_spot
  - fitted_pace
  - stop_distance
  - Seat::back
  - Walker::chair
  - turn to sit
---

# Walking to a chair turns round on a small circle, paces its last steps to stop on the spot, and the seat makes up the rest

`Walker::chair = Some(Chair { seat, forward, height })` with a chair way of
sitting: the walker walks to the chair, turns its back to it, and sits.

## Decision

- **Where it stands** (`stand_spot`): where the upright seated pose puts
  the hips over the seat with the feet unmoved (`sitting::seat_offset`),
  facing the chair's way.
- **The path:** straight to a tangent of a 0.25 m circle through that
  spot, then round the circle, arriving with the chair behind it.
  - The shorter of the two circles (left or right).
  - Re-planned from where the walker is, every frame.
  - The walk at 0.5 m/s round it turns half a circle in ~1.6 s and ~2.5
    steps, as healthy adults turn 180° (Robinson: 1.5 s median).
  - It looks at the chair on the way, until it turns.
- **Too close to turn onto the spot** (within both circles, or a straight
  under 0.6 m): it walks out 1.2 m in front of the spot first.
- **The stop is paced.** Told to stop, the walk goes on to a footfall and
  then a last step (`transition`). So a stop can end only in half-stride
  steps, 0.39 m apart at the turning pace.
  - Stopped at the nearest one, the walker stood up to 0.29 m off its
    spot.
  - Over the last 1.5 m it picks the coming footfall whose stride is
    nearest the turning pace's, and walks at the speed whose stride lands
    the stop on the spot (stride ∝ speed^0.65, `gait`). It is paced again
    only when that drifts past 3 cm.
  - It stops when that stop ends nearer the spot than the next could.
  - This is how a person closes on a target (Lee, Lishman & Thomson 1982).
- **The seat makes up the rest** (`sitting::Seat::back`). As far as the
  walk stopped past or short of its spot, along the chair's facing (up to
  ±12 cm), the shins swing so the hips land on the seat with the feet
  where they are. Across the chair, the walker sits that far off the
  middle.

## Traps it hit

- **The stride the gait computes is a straight walk's.** On the tight
  circle the body covered ~82–95 % of it a cycle; on the straight at
  1 m/s, 110 %. A stop timed by it fell up to 15 cm short.
  - A decayed average of distance over stride was no better: the sprung
    legs lag a speed change, and just after slowing it read 120 %.
  - Now the stride is two of the last whole steps walked at one speed,
    scaled for a new speed (`walker::Walked`).
- **Re-pacing on every small error made the speed hop** (0.38–0.65 m/s).
  Each new speed also costs a stride measurement, and spoils the step
  being measured. So it re-paces only past 3 cm, and the seat takes the
  rest.
- **The arc's remaining length must use the radius actually walked.** A
  walker wide of the circle travels further for the same turn. Measured at
  the circle's radius, the stop fell 2–4 cm short.
- **A stop short of the path's end never turned the rest of the way.** It
  kept steering along the path. Once at rest it now turns to the arrival
  heading.

## Consequences

- **Tests:** a point walk modelled on the transition's stop arrives within
  4 cm of the spot, facing within 0.03 rad. It runs from ahead, beside,
  behind, close and too close, each at four phases of the stride.
- **The pacing test can fail:** without the pacing, both it and the
  half-stride test fail.
- **Live,** for three chair placements:
  - the root stopped 28–103 mm off its spot;
  - the seated hips landed on the seat in depth, 26–43 mm off its middle
    across;
  - the feet slid ≤ 4 mm while sitting;
  - nothing went under the floor.
- **Seen** from the side and the front, skeleton and mesh:
  - mid-turn, beside the chair, one foot planted and the other stepping;
  - seated on the seat, back to the backrest, feet flat, knees ~90°.
- **Known:** standing up after a walk, the right foot settles 11 mm
  sideways at the end of the rise, from the last step's width to the
  stance's.

## Revisit when

- **The chair is an obstacle:** a walker behind it plans its path through
  it. Nothing routes round furniture.
- **A crowd, or chairs set at a table:** the turning circle needs ~0.5 m
  free beside the spot.
- **Sideways error matters** (a narrow seat, a bench): the stop is paced
  only along the path.

## Related

- [Sitting down and standing up go through solved keys](./sitting-down-and-standing-up-go-through-solved-keys.md) — prerequisite: the chair poses this walks to, and `Seat::back`.
- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — context: why a stop ends a footfall plus a last step on, the quantum the pacing works round.
- [Steer over terrain by the ground profile ahead](./steer-over-terrain-by-the-ground-profile-ahead.md) — context: the playground's steering, which this does not use.
