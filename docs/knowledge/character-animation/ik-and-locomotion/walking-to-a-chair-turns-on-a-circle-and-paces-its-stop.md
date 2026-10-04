---
title: Walking to a chair turns round on a small circle, paces its last steps to stop on the spot, and the seat makes up the rest
description: "A walker given a chair routes round obstacles (visibility graph), comes at its spot from an entry where needed, then turns on a 0.18 m circle in short steps (~1.5 s 180°) ending 10 cm in front of it. The stop is paced on the stride the gait really takes; the seat takes the rest. Read before changing approach.rs."
type: decision
status: current
tags:
  - locomotion
  - biomechanics
  - correctness
updated: 2026-10-04
verified: 2026-10-04
code:
  - src/character/anim/approach.rs
  - src/character/anim/walker.rs
  - src/character/anim/gait.rs
  - src/character/anim/sitting.rs
  - src/character/anim/footlock.rs
  - src/character/anim/obstacles.rs
  - src/character/anim/physics_obstacles.rs
  - examples/physics_character_playground.rs
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
  - SHORT_STEPS
  - stride_speeds
  - miss_cost
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
- **The path:** straight to a tangent of a 0.18 m circle, then round it,
  arriving facing away from the chair.
  - The shorter of the two circles (left or right), re-planned every frame.
  - At 0.4 m/s round it, half a circle takes 1.4 s, as healthy adults turn
    180° (Robinson: 1.5 s median).
  - The turn ends `TURN_AHEAD` (10 cm) in front of the spot: a circle
    ending on the spot dips a radius behind it, toward the seat's front
    corner.
- **In short steps** (`gait::SHORT_STEPS`): walking to a chair, the
  stride shortens with speed down to 0.3 of the recorded one's excursions,
  not the usual 0.5. That is a 0.23 m step on `puppet_base`, as a person
  shuffles closing on a chair. Only in short steps can the walk follow the
  0.18 m circle and pace its stop onto the spot.
  - Turns sharper than 0.8 rad are walked at the turning pace.
  - It looks at the chair on the way, until it turns.
- **Round obstacles** (`route`): its own chair (`Chair::footprint`,
  passed first) and any others (`obstacles::RouteObstacles`, from the
  physics world by `physics_obstacles` at body height, a table top
  included).
  - **A visibility graph:** each obstacle's corners 0.45 m out (those not
    within 0.2 m of another), linked where a straight keeps 0.2 m clear,
    searched from the goal with Dijkstra, re-planned every frame.
  - **Ways into the goal:** they may come within the margin over their last
    0.2 m, and of its own chair keep out of the chair alone (the spot lies
    within its margin by design).
  - **The corner being walked to is kept** unless another way is 0.3 m
    shorter. Once clear, it plans afresh. It never stops while going
    round.
  - **With no way round it waits;** never straight on.
  - **Within 0.5 m of an obstacle it walks at the turning pace.**
- **By an entry** (`entry`), when the turn has no room or is in the way:
  1.2 m from where the turn ends, in front of it or to its left or right.
  - It takes the nearest that is clear of every obstacle, has a clear way
    in, and whose whole final approach (straight and turn, sampled) keeps
    0.1 m from all but its own chair.
  - The entry is re-checked every frame, as obstacles are found on the way.
  - A walk from an entry is committed: not re-routed onto the tangent.
  - A person comes at a chair from in front in the open, and along the
    gap from one side at a table.
- **The stop is paced.** A stop can end only in half-stride steps (a
  footfall, then a last step, `transition`): about 0.33 m apart at the
  turning pace, up to a quarter stride off the spot.
  - Over the last `FIT_WITHIN` (the arc and 0.6 m) it walks at the speed
    whose stride lands a footfall's stop on the spot. This is how a person
    closes on a target (Lee, Lishman & Thomson 1982).
  - The stride grows as speed^0.65 only between `gait::stride_speeds`;
    slower, only the cadence drops. In short steps that is from 0.25 m/s.
  - It paces only once lined up with the path (heading within 0.8 rad).
  - It re-paces at most once a step, and only if the stop has drifted
    past 3 cm.
  - It stops only once paced and walking at that pace, at the better of
    this footfall's stop and the next (`miss_cost`). Past the turn's end
    the seat takes up only 5 cm; short of it, round the circle, 25 cm back
    but only 8 cm to the side.
- **The seat makes up the rest** (`sitting::Seat`), so the hips land on
  the seat's middle with the feet where they stopped:
  - `back` (±15 cm, along the chair) swings the shins forward or back;
  - `across` (±8 cm) slants them sideways.
- **The feet keep clear of the chair** through the foot IK's obstacles:
  the gallery gives its character `Chair::footprint` as an
  `AnimObstacles` (see [feet keep clear of obstacles](./feet-keep-clear-of-obstacles-in-the-foot-ik.md)).

## Alternatives considered

- **A smaller turning circle in the walk's own steps.** At 0.12 m and at
  0.18 m the walk could not follow it: the body trails its heading by
  about a step (0.39 m), and it stood 18–22 cm off its spot, past what the
  seat takes up. In short steps it follows 0.18 m (taken).
- **The 0.25 m circle, walked 30° short of the heading, the rest turned
  standing**, to keep the dip off the chair. A turn at rest pivots the
  planted feet with the body (`AnimFootIk::turn`): ~5 cm of foot slide.
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
- **The stride stops shortening below 0.54 m/s** (`gait::walking_for`
  holds its excursions at half the recorded ones). Paced to 0.30-0.37 m/s
  on `speed^0.65`, four of sixteen live walks to the table stopped 12-20 cm
  past the turn's end, beyond what the seat takes up. The prediction of
  where the stop ends was right in every one; the stride it was given was
  not (0.52 m where the walk took 0.77).
- **Paced while turning onto a short straight** (from an entry, at
  0.8 m/s, turning at 2 rad/s), the walk swung wide and met the circle
  1.3 rad off its heading, ending 23 cm to the side.
- **Weighing a stop's miss by distance alone** took 30 cm short (16 cm to
  the side, round the circle) over 4 cm past. And a cost held flat beyond
  half a circle short gave two stops a step apart the same cost: the walk
  stopped a metre out.
- **The arc's remaining length must use the radius actually walked.**
  Wide of the circle, measured at its radius, the stop fell 2–4 cm short.
- **A stop short of the path's end never turned the rest of the way**: it
  kept steering along the path. At rest it now turns to the arrival
  heading.
- **Routing:**
  - A routing that gave up within its margin as "beside the chair" walked
    12 cm into the seat closing on a corner.
  - Skipping only the corner it was at, a slowed walk orbited a corner.
  - Routed round its own chair only, at the playground's table it walked
    through the table (~6 s inside its footprint).
  - Corner by corner of one box, at a table with chairs pulled out, a
    corner lay inside the next obstacle.
  - With no way found it walked straight on: through its own chair, and
    through the table. It now waits.
  - At 1.2 m/s the turn onto a line past a chair's corner swung 0.6 m wide
    and came within 8 cm (hence slow near furniture).
- **Where to come from:**
  - Walking out in front of the spot when too close went through the
    table at a table.
  - Back the way it came, it came the same way again, too close again:
    a loop.
  - An entry chosen before the far chairs were found lay 15 cm from one.
    It was unreachable and the walk went straight on; now re-checked, and
    the physics region covers the chair too.
- **The spot is 0.51 m in front of the seated hips**, not 0.40: the
  standing hips sit 0.11 behind the root. A model test assuming 0.40
  passed a table the live walk could not reach.
- **A chair at a table must be pulled out ~0.75 m** from the table's edge
  to stand in front of. At 0.65 m the turn's end was 12 cm from the table,
  with no entry to come from.
- **A seat moved only along the forward axis lost its `across`**
  (`getup::placed` moves a pose along the forward only). Chair poses are now
  set on their feet's middle both ways (`sitting::on_feet`).
- **Slanted shins exposed a leg IK bug**: the feet turned 9° about their
  toes (see [the knee hinge note](./a-knee-hinge-must-be-square-to-the-line-to-the-target.md)).
- **Feet in the chair's legs.** Through the turn, every placement put a
  foot ~5 cm into a front leg (measured against the gallery chair's posts
  over BRP), and a heel 14 cm under the seat front. Its fix and traps are
  in [feet keep clear of obstacles](./feet-keep-clear-of-obstacles-in-the-foot-ik.md).
- **After the rise both feet stay planted 0.3 s** (`walker::STOOD_HOLD`).
  Let go at once, a foot still moving with the extending legs was released
  by its speed and slid 11–17 mm, from where the turning walk left it to
  the stance.

## Consequences

- **Tests:**
  - A point walk that stops as the transition does ends where the seat
    takes it up, with 2 cm to spare, from ahead, beside, behind, close and
    too close, at four stride phases.
  - Round the chair it stays within 6 cm of its own 0.2 m margin.
  - The gallery's three chairs, from three headings each, likewise.
  - Through the turn, the body's middle stays 11 cm clear of the chair.
  - Paced on a stride that ignores the gait's floor, or before lined up,
    or without the routing, their tests fail.
  - The model walk has no step lag, so it follows a small circle in any
    step: only live can show the short steps are needed for it.
- **Live,** four chair placements, including one with its back to the
  walker:
  - stopped 53–125 mm off the spot;
  - seated hips 0–5 mm from the seat's middle;
  - feet 0 mm of slide sitting and rising;
  - the body's middle never inside the footprint;
  - the feet at least 3.0 cm from any chair leg (a 4.5 cm half-wide foot
    against the 3.5 cm posts), none under the seat front.
- **Seen** from behind at the turn's closest moment: the foot beside the
  chair, floor between it and the front leg.
- **At a table** (`approach::tests`): the playground's dining set, each of
  four chairs from four sides of the room. Never into the table or another
  chair, 10 cm clear of its own, where the seat makes up the rest.
- **The model's walks** (48, in short steps on the 0.18 m circle) end
  4.7 cm short to 3.0 cm past the turn's end, at most 1.1 cm to the side;
  the body's middle at least 12.2 cm from the chair (5 cm on 0.25 m).
- **Live at the table** (playground `--sit-at-table N --start X,Z,YAW`,
  physics obstacles), sixteen walks, the four chairs from four starts:
  - before: four of sixteen stopped 12–20 cm past the turn's end, beyond
    the seat (paced below where the stride shortens);
  - on the stride the gait takes, in its own steps: all sixteen within
    the seat, but two at its edge (23.5 cm short of the turn's end and
    7.3 cm across; 8 cm across);
  - in short steps on the 0.18 m circle: all sixteen from 9.8 cm short to
    2.1 cm past the turn's end, 2.0–4.9 cm across (the seat takes 25 / 5 /
    8);
  - at two chairs measured over BRP, the body's middle 19–34 cm clear of
    the table, feet at least 2.3 cm from every leg (with no loose props:
    see Revisit), hips 0 mm off the seat.
- **Live at the gallery's chairs** (four placements, one from behind), A/B
  on the same input: through the turn the body's middle came within
  2.8–4.8 cm of the chair on the 0.25 m circle, 8.4–13.4 cm on 0.18 m.
  Hips 0–2 mm off the seat in all eight.
- **Seen** mid-turn, gizmos and mesh, Front and Left: short steps beside
  the chair, feet a shoe apart, no leg crossing or twisted foot.

## Revisit when

- **A steady 2–5 cm sideways miss live** that the model does not show
  (≤ 1.1 cm there): the seat takes it, but its cause is not known.
- **Feet on a loose prop by the chair:** the feet's obstacle band is
  measured from the ground the body stands on, so standing on a prop
  0.3 m up a foot went 3.8 cm into a chair leg below it (with no props,
  2.3 cm clear). The band wants each foot's own ground.
- **Moving obstacles, crowds:** the graph is re-planned every frame but
  knows nothing of other walkers.
- **Other seats** (a bench, a sofa): `Chair::standard` is one chair's size.

## Related

- [Sitting down and standing up go through solved keys](./sitting-down-and-standing-up-go-through-solved-keys.md) — prerequisite: the chair poses this walks to, and `Seat`.
- [A two-bone IK knee hinge must be square to the line to the target](./a-knee-hinge-must-be-square-to-the-line-to-the-target.md) — deeper: the leg IK bug the sideways seat shift exposed.
- [Feet keep clear of obstacles in the foot IK](./feet-keep-clear-of-obstacles-in-the-foot-ik.md) — deeper: how the feet are kept out of the chair's legs through the turn.
- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — context: why a stop ends a footfall plus a last step on, the quantum the pacing works round.
- [Steer over terrain by the ground profile ahead](./steer-over-terrain-by-the-ground-profile-ahead.md) — context: the playground's steering, which this does not use.
