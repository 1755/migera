---
title: A steep slope is slid down on the feet as a block with friction, its creases rounded for the hips and feet
description: "Step 18: a roof (low, a hand trailing) or a steep face (leant back, arms out) is slid down at g(sin θ − μ cos θ) − k w², integrated once at a fixed step. It runs out to a stand or goes over a drop as a fall, leaping or braking to catch the eave. Read before changing parkour/slope.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/slope.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
  - examples/anim_bench.rs
sources:
  - "tests parkour::slope::tests::{a_slopes_surface_is_stood_on_over_it, slopes_are_slid_down_on_the_feet}"
  - "live: character_gallery --step-seconds 0.0333333 --start-height 6.0 --block 0,-1,0,6.0,3,4 --roof 0,-1,0,6.0,3,3,2.5 --anim-speed-schedule 1:1.4 [--catch | --jump-at 2.3:0.35], BRP; gizmos on/mesh off Left and Front, mesh Left"
  - "live: the same with --steep 0,-1,0,6.0,3,5,6.0, BRP; gizmos Left and Front, mesh Left"
  - "anim_bench --gait roof-slide / face-slide"
aliases:
  - roof slide
  - slope slide
  - steep-surface slide
  - pyramid slide
  - SlopeSlide
  - Slope
  - SlopeGround
  - Walker::slopes
---

# A steep slope is slid down on the feet as a block with friction, its creases rounded for the hips and feet

Step 18 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), first part: a slope
too steep to walk is slid down on the feet. There are two shapes:
- **A roof:** low, with the up-slope hand trailing on the roof.
- **A steep face** (a pyramid's): leant back, the arms out.

At its foot the slide either runs out onto the ground or goes over a drop
(an eave). Off a drop it can leap, or brake to catch the eave. No slide
data was found; the friction is a shoe sliding on tiles.

## Decision

**The geometry** (`parkour::slope::Slope`):
- a plane falling away from its top edge, with its run, drop, width and
  kind;
- `SlopeGround` puts it on the character's ground, so the foot IK stands
  on it;
- the walker is told about it through `Walker::slopes`.

It is slid down if its grade is over 0.6. The walker must reach it walking
or running down it, within 60° of straight down, near its top edge.

**The motion** is a block sliding on its feet. Along its path the speed
changes by `g (sin θ − μ cos θ) − k w²`:
- μ is 0.4 (`FRICTION`);
- k is 0.11 per metre (`DRAG`), giving a terminal speed of about
  5.5 m/s down a 40° roof;
- it is integrated once when the slide begins, every 1/240 s, so the
  motion does not depend on the frame rate;
- friction comes in over the top edge's rounding. Walked on at full
  friction, the flat behind the edge stops it dead.

**The path** is the slope's line rounded over the top edge and, where it
runs out, over its foot (`FILLET` 0.4 m either side). The hips' vertical
speed never steps.

**The feet have their own path** (`foot_height`), never under the
surface:
- **Over the top edge:** that rounding cuts into the convex edge by
  `g r/4`. It is lifted by `(g r/4)(1 − (x/r)²)²`, which is at least the
  cut everywhere (`(1−u²)² ≥ (1−u)²`). The feet skim up to 4 cm over the
  slope there.
- **At the foot's concave crease:** the rounding (0.25 m) is already above
  the surface.
- **Soles:** soled to the path's own slope.

**The shapes** (`SlopeKind`):
- **Roof:** hips 0.6 m down, trunk 0.35 rad back up the slope. The left
  foot is 0.3 m ahead down the slope and the right 0.15 m behind. The
  right hand trails on the roof 0.45 m behind and 0.3 m out.
- **Face:** hips 0.35 m down, leant back 0.3 rad, the feet near together,
  the arms out.

Going in, the slide blends from the walk's pose and hips over 0.35 s. It
turns to face down the slope, and any drift across it dies away. The feet
are lifted softly off the surface meanwhile, as the skid does.

**At its foot**:
- **Run out:** it brakes on the flat (the same friction and drag) to
  0.8 m/s, then rises to standing over 0.5 s, the feet sliding into
  standing's stance.
- **At a drop:** the walker hands over to a fall at the hips' velocity,
  moved on by the rest of the frame (`Falling::off`).
- **A jump asked** (`SlopeSlide::leap`): the hips are pushed up off the
  feet over the slide's last 0.2 s, their speed rising as a smoothstep to
  3 m/s, the legs straightening. The fall goes on with it.
- **Asked to catch** (`braking_to_catch`): it brakes evenly over the last
  1 m to 1.2 m/s, then turns round in the air over 0.3 s. It catches the
  eave as any fall catches a ledge it is given. The gallery makes each
  roof's eave a ledge, with the house under it as its block.

## Alternatives considered

- **Leaping by adding 3 m/s at the edge:** the hips' step changed 6 cm in
  the frame. A jump needs its push.
- **Catching at the slide's own speed:** going over at 3 m/s, the body was
  1 m out from the eave by the time it had turned to face it. A catch
  needs the edge reached slowly, with the hands and feet braking.
- **Feet on the true surface:** a foot dived 12 cm in a frame at the top
  edge, and a toe passing the foot edge turned level in a frame.
- **Feet on the hips' rounding over the top:** it passes under the edge's
  corner.

## Traps

- **A walking start sees no slope**: friction at full strength on the top
  edge's rounding, where the slope is still nearly level, stopped the
  slide where it began. Friction is ramped in over the rounding.
- **The leg reach clamp shorter than standing's own leg**: the risen stand
  had its knee bent 8 cm off standing's. The clamp is no tighter than
  standing.
- **A foot put directly under the hips along the slope** dropped
  standing's own forward offset: 8.8 cm off standing at the end.
- **The walker's gait travel kept going on the slide's first frame**
  (`ride_rendered_feet`): 4.5 cm too far. Sliding is now among the moves
  that set the root themselves.
- **The last frame, cut short at the slide's end**, reads as a change of
  step. The walker gives its leftover time to the fall.

## Consequences

**Headless** (`puppet_base`, 60 fps; a 40° roof from 1.4 and 4 m/s, plain,
leaping and braking to catch; a 50° face from 1.4 and 4 m/s, run out):
- the speed on the straight part matches the closed form
  `W tanh(√(ak) t + c)` to 0.01 m/s;
- the feet are on the surface to 4 mm, and nothing goes more than 7 mm
  under it;
- the roof's hand is within 3.6 cm of the roof (7.5 mm unless leaping);
- the largest change of step is no more than the gait's own plus 1 cm
  (the walk's toe-off and the run's knee, both 3.9 cm). It is 1.6-4.6 cm,
  at the entry.
- At the edge, the hips' velocity goes on into the fall to 5 cm/s.
  Leaping, 3.0 m/s up over the slide's own; braking, 1.14 m/s.
- Run out, it ends exactly standing.

**Live** (Xvfb, BRP at 30 Hz):
- **The roof:** the feet a soled ankle's height over it, the trailing
  hand about 6 cm off it. At the eave the pelvis fell on at gravity's own
  change of step (1.1 cm), landed and rolled.
- **Leaping:** the pelvis rose from 3.95 to 4.14 m over the eave and came
  down 0.6 m further out; the largest change of step was 2.25 cm, through
  the push.
- **With `--catch`:** it slowed, turned over the edge and hung from the
  eave, hands at 3.40 m on its line.
- **The face:** it reached about 6 m/s, ran out and stood 4 m on.

From the left and front with gizmos, and in the mesh, the roof slide is
low with a hand back on the roof, and the face slide leant back with the
arms out.

**Cost** (`anim_bench`, each frame posed from the slide's start): the roof
29 µs a character (28 with 100), the face 22 µs; the skid stop 29 µs.

## Revisit when

- **Landing on a slope from a fall** lands as on flat ground: a slide
  starts only from walking or running onto one near its top edge.
- **Steering across the slope** and **walking up a steep face** from below
  are not modelled (`SlopeGround` blocks nothing).
- **A leap mid-slope**: it is kept until the edge; a jump on a run-out
  slope is ignored.
- **Data**: the friction, drag, shapes and catch speed are set by eye.

## Related

- [A skid stop slides side-on and rises over stuck feet](./a-skid-stop-slides-side-on-and-rises-over-stuck-feet.md) — same-pattern: a planned slide whose pose is solved on the rig each frame, the entry lift off the floor taken from it.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — applies: the fall a slide hands to at a drop, and its landing.
- [A ledge is caught near the top of a jump and hung from](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) — applies: the catch the eave is caught by.
