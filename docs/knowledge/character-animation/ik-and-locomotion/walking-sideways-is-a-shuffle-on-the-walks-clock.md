---
title: Walking sideways is a shuffle on the walk's own clock, the feet sweeping across
description: "Walker::aside walks sideways as a gait cycle (LegCurves::Shuffle): the walk's timing, each foot's stance sweeping it across, the stance widened so the feet never cross, legs placed by move_pelvis_and_feet; root motion, cadence and start/stop are the walk's. Read before changing shuffle.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-04
verified: 2026-10-04
code:
  - src/character/anim/shuffle.rs
  - src/character/anim/gait.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
sources:
  - "tests shuffle::tests::a_shuffle_walks_aside_at_the_speed_its_cadence_is_set_for, the_feet_never_cross_and_stand_planted_through_their_stance"
  - "live: character_gallery --aside-schedule 1:0.4,7:-0.6,13:0 and 1:0.2,4:0.6,8:0.3,12:0, --step-seconds 0.0166667 on Xvfb, the pelvis and feet over BRP"
aliases:
  - side shuffle
  - shuffle
  - shuffling
  - sideways walk
  - strafe
  - Walker::aside
  - LegCurves::Shuffle
  - SHUFFLE_CLOSEST
  - SHUFFLE_REGEAR
  - shuffle_speed
  - --aside-schedule
---

# Walking sideways is a shuffle on the walk's own clock, the feet sweeping across

`Walker::aside` (m/s, positive to the character's left) walks sideways
continuously: the side shuffle. It is a shape of the walk cycle
(`gait::LegCurves::Shuffle`, posed by `shuffle::shuffle_pose`), so the
walker's whole gait pipeline drives it unchanged: the clock and the
cadence from the stride the pose travels (`locomotion::distance_per_cycle`),
the start and the stop (`transition`), and root motion read off the
planted feet (`locomotion::root_displacement_between`), which knows no
direction.

## Decision

- **The walk's timing, across.** Each foot stands `SHUFFLE_DUTY` (0.65) of
  the cycle, the left at phase 0 and the right at 0.5 (`gait::leg_phase`).
  Through its stance a foot sweeps under the body from half a step toward
  the travel to half a step away, at a constant rate; its swing carries it
  back, eased, on a 5 cm arc.
  - Both feet down move alike under the body, so neither slides: root
    motion follows them.
- **The feet never cross.** Walking, the feet pass front to back; sideways
  they must not. The gap between them swings by a stride (the body's
  travel a cycle) about its mean, so the mean is `SHUFFLE_CLOSEST` (0.14 m,
  toe to toe) plus half a stride, never narrower than they stood. The
  leading foot steps out, the trailing one closes: the gap opens to a
  stride past the closest and shuts again.
- **The legs are placed, not curved:** `stance::move_pelvis_and_feet`, the
  pelvis taking the height that keeps every foot down in reach (spread
  wide, it sinks, as a shuffling body does).
- **Stride from speed:** `0.8 × speed`, 0.12–0.55 m a cycle; the cadence
  carries the rest.
- **The stride and width are set from the speed asked, not the speed the
  legs step at**, held through the stop, set outright from a stand and
  eased to a new speed on the way (`SHUFFLE_REGEAR`, 0.4 m/s a second).
- **In the walker:** asked neither to walk nor to sit. Turning the other
  way, or walking on, it stops first and starts again.

## Alternatives considered

- **Repeated steps aside** (the balance's step and close, the first
  version): ~0.24 m/s at most, a halting rhythm, the body pausing between
  steps. A single step aside still is one
  (see [a step aside](./a-step-aside-is-the-balances-side-step-and-close.md)).
- **Sideways leg curves** (hip abduction through stance and swing, as the
  walk's recorded sagittal angles): there is no recorded sideways stride to
  replay, and placing the feet gave a cycle that cannot slide by
  construction.
- **The forward walk with the trunk turned:** a crab walk, not a shuffle.

## Traps it hit

- **The walk's feet, turned across, cross.** With a mean gap of the
  closest plus a whole step (as a walk's feet pass), the feet spread
  0.69 m; the gap swings by a stride, not twice the step.
- **A foot just set down carries no load,** and the leg solver heeds a leg
  only over 0.05 of it: the pelvis stood too high for that foot to reach
  the floor (7–10 mm up). Every foot down now counts at least 0.2.
- **A height-only test of "planted" reads a skimming swing as a slide:**
  the end of a swing within 4 mm of the floor gave 53 mm "slid". Within
  1.5 mm of the floor, the feet down move at most 5 mm (the standing idle
  alone shows 5.0 on the same check).

## Consequences

- **Model** (`shuffle::tests`, the real rig): a cycle carries the body one
  stride across within 1 %, under 1 cm along; the feet never nearer than
  0.135 m; every foot down on the floor within 2 mm; both feet down move
  alike within 0.5 mm a frame (a non-linear sweep fails it at 3.1 mm).
- **Live** (gallery, 0.4 left then 0.6 right, then stop; and 0.2 → 0.6 →
  0.3 on the way):
  - 0.37 / 0.54 m/s, and 0.18 / 0.58 / 0.29 m/s;
  - the feet down move at most 5.0 mm; never nearer than 0.143 m;
  - the pelvis at most 32 mm down;
  - stopped, the feet side by side at their standing width and height.
- **Seen** at 0.5 m/s, Front and Left, gizmos then the mesh: a foot lifted
  mid-swing beside one standing, the stance opening and closing, legs
  never crossing, knees bent forward, the trunk upright.

## Revisit when

- **Walking and shuffling at once** (strafing diagonally): this is
  sideways only.
- **The arms:** they hang as the base pose has them; a shuffle's arms are
  carried a little out and swing little.
- **The first step after turning back** lifts its foot ~8 mm and moves it
  2.4 cm outward before its swing (the start's release).

## Related

- [A step aside is the balance's side step and close](./a-step-aside-is-the-balances-side-step-and-close.md) — contrast: one step aside from a stand.
- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — prerequisite: why the shuffle moves without sliding with no root motion of its own.
- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — context: the start and the stop the shuffle borrows.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — context: the planted feet's locks under a body moving across.
