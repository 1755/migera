---
title: Walking to a chair turns round on a small circle, paces its last steps to stop on the spot, and the seat makes up the rest
description: "A walker given a chair walks round it if needed, then a Dubins path (straight, a 0.25 m circle: Robinson's ~1.5 s 180° turn) ending 10 cm in front of its spot. Stops come in half strides, so the last steps are paced; the seat moves back and across to take the rest. Read before changing approach.rs."
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
  - "tests approach::tests (a point walk that stops as the transition does: five starts at four stride phases, the gallery's chairs, from behind)"
  - "tests sitting::tests::a_seat_off_where_the_feet_stand_is_met_by_the_shins_with_the_feet_kept, a_chair_sat_on_off_its_spot_keeps_the_planted_feet_still"
  - "live: character_gallery --sit chair:upright --chair X,Z,HEADING --step-seconds 0.0166667 on Xvfb, BRP joint positions and the walker's arrival log"
aliases:
  - approach
  - Approach
  - Chair
  - stand_spot
  - fitted_pace
  - stop_distance
  - Seat::back
  - Seat::across
  - Walker::chair
  - turn to sit
  - walk round the chair
---

# Walking to a chair turns round on a small circle, paces its last steps to stop on the spot, and the seat makes up the rest

`Walker::chair = Some(Chair::standard(seat, forward))` with a chair way of
sitting: the walker walks to the chair (round it if it is in the way),
turns its back to it, and sits.

Contents: [Decision](#decision) · [Alternatives considered](#alternatives-considered) · [Traps it hit](#traps-it-hit) · [Consequences](#consequences) · [Revisit when](#revisit-when)

## Decision

- **Where it stands** (`stand_spot`): where the upright seated pose puts
  the hips over the seat with the feet unmoved (`sitting::seat_offset`),
  facing the chair's way.
- **The path:** straight to a tangent of a 0.25 m circle, then round it,
  arriving facing away from the chair.
  - The shorter of the two circles (left or right), re-planned every frame.
  - At 0.5 m/s round it, half a circle takes ~1.6 s and ~2.5 steps, as
    healthy adults turn 180° (Robinson: 1.5 s median).
  - The turn ends `TURN_AHEAD` (10 cm) in front of the spot: a circle
    ending on the spot dips a radius behind it.
  - Turns sharper than 0.8 rad are walked at the turning pace.
  - It looks at the chair on the way, until it turns.
- **Round the chair:** a straight that would come within 0.2 m of the
  chair's footprint (`Chair::size`, `Chair::ahead`) heads for a corner
  0.45 m out from it.
  - It takes the corner giving the shortest way round, and never goes back
    to a corner it has reached.
  - Once clear, it plans afresh (the shorter circle from beside the chair
    is not the one from behind it).
  - It never stops while going round.
- **Too close to turn onto the spot** (within both circles, or a straight
  under 0.6 m that close): it walks out 1.2 m in front of the spot first.
- **The stop is paced.** A stop can end only in half-stride steps (a
  footfall, then a last step, `transition`): 0.39 m apart at the turning
  pace, up to 0.29 m off the spot.
  - Over the last `FIT_WITHIN` (the arc and 0.6 m) it walks at the speed
    whose stride lands a footfall's stop on the spot (stride ∝ speed^0.65).
    This is how a person closes on a target (Lee, Lishman & Thomson 1982).
  - It re-paces at most once a step, and only if the stop has drifted
    past 3 cm.
  - It stops only once paced and walking at that pace.
- **The seat makes up the rest** (`sitting::Seat`), so the hips land on
  the seat's middle with the feet where they stopped:
  - `back` (±15 cm, along the chair) swings the shins forward or back;
  - `across` (±8 cm) slants them sideways.

## Alternatives considered

- **A smaller turning circle**, to keep the turn's dip off the chair. At
  0.12 m and at 0.18 m the walk could not follow it: the body trails its
  heading by about a step, and it stood 18–22 cm off its spot, past what
  the seat takes up.
- **A pivot on the spot** (a very slow walk turning hard, 0.18 m/s at
  2 rad/s). The body stays within ~12 cm, but the feet swing up to 30 cm
  out, as far past the chair's front as the circle's dip.
- **Ending the turn further forward** (≥ 18 cm) to clear the dip entirely:
  the seat would have to move the feet that far, the legs stretched out.

## Traps it hit

- **The stride the gait computes is a straight walk's.** On the turning
  circle the body covered ~82–95 % of it a cycle, and a stop timed by it
  fell up to 15 cm short.
  - A decayed average of distance over stride read 120 % just after
    slowing (the sprung legs lag).
  - Now the stride is two of the last whole steps walked at one speed
    (`walker::Walked`).
- **Re-pacing every frame made the speed hop** (0.39–0.66 m/s). Each new
  speed's stride is measured a frame late, and spoils the step being
  measured. A stop decided at 0.72 m/s on a stride for 0.39 ended mid-turn,
  25 cm short, beside the chair.
- **Stopping before pacing** stopped from a full-speed stride (1.4 m at
  1.2 m/s), 1.3 m out.
- **The arc's remaining length must use the radius actually walked.**
  Wide of the circle, measured at its radius, the stop fell 2–4 cm short.
- **A stop short of the path's end never turned the rest of the way**: it
  kept steering along the path. At rest it now turns to the arrival
  heading.
- **Routing:**
  - A routing that gave up within its margin as "beside the chair" walked
    12 cm into the seat closing on a corner.
  - Skipping only the corner it was at, a slowed walk orbited a corner.
- **A seat moved only along the forward axis lost its `across`**
  (`getup::placed` moves a pose along the forward only). Chair poses are now
  set on their feet's middle both ways (`sitting::on_feet`).
- **Slanted shins exposed a leg IK bug**: the feet turned 9° about their
  toes (see [the knee hinge note](./a-knee-hinge-must-be-square-to-the-line-to-the-target.md)).
- **After the rise both feet stay planted 0.3 s** (`walker::STOOD_HOLD`).
  Let go at once, a foot still moving with the extending legs was released
  by its speed and slid 11–17 mm, from where the turning walk left it to
  the stance.

## Consequences

- **Tests:**
  - A point walk that stops as the transition does arrives within 4 cm,
    from ahead, beside, behind, close and too close, at four stride phases.
  - Round the chair it stays within 6 cm of its own 0.2 m margin.
  - The gallery's three chairs, from three headings each, end within 6 cm.
  - Through the turn, the body's middle is never more than 7 cm past the
    seat's front edge.
  - Without the pacing or the routing, their tests fail.
- **Live,** four chair placements, including one with its back to the
  walker:
  - stopped 53–125 mm off the spot;
  - seated hips 0–5 mm from the seat's middle;
  - feet 0 mm of slide sitting and rising;
  - the body's middle never inside the footprint.
- **Known:** through the turn a foot passes up to 8 cm under the seat's
  front. In one case (the default chair) the heel came to its front leg,
  touching or clipping it by a centimetre or two for a moment.

## Revisit when

- **The chair is one of several, or at a table:** the routing knows only
  this chair, and the turn needs ~0.5 m free beside the spot.
- **The heel at the chair's leg matters:** the walk should step past the
  corner, or the chair's legs become obstacles for the feet.
- **Other seats** (a bench, a sofa): `Chair::standard` is one chair's size.

## Related

- [Sitting down and standing up go through solved keys](./sitting-down-and-standing-up-go-through-solved-keys.md) — prerequisite: the chair poses this walks to, and `Seat`.
- [A two-bone IK knee hinge must be square to the line to the target](./a-knee-hinge-must-be-square-to-the-line-to-the-target.md) — deeper: the leg IK bug the sideways seat shift exposed.
- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — context: why a stop ends a footfall plus a last step on, the quantum the pacing works round.
- [Steer over terrain by the ground profile ahead](./steer-over-terrain-by-the-ground-profile-ahead.md) — context: the playground's steering, which this does not use.
