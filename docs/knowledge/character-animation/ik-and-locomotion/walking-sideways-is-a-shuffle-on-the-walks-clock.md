---
title: Walking sideways is a shuffle on the walk's own clock, the feet sweeping across
description: "Walker::aside walks sideways as a gait cycle (LegCurves::Shuffle) on the walk's timing, feet sweeping across, the stance widened so they never cross; with forward too, a diagonal shuffle or the walk turned to its way. Root motion, cadence, start/stop are the walk's. Read before changing shuffle.rs or strafing."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-06
verified: 2026-10-06
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
  - shuffle_ahead
  - WalkerState::strafe
  - diagonal
  - carry_arms
  - shuffle::planted
  - --aside-schedule
  - smoothed_drop
  - shuffle pelvis
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
  back on a 5 cm arc, across by three quarters of the swing
  (`SHUFFLE_ACROSS_BY`) and then straight down.
  - Both feet down move alike under the body, so neither slides: root
    motion follows them.
- **The feet never cross.** Walking, the feet pass front to back; sideways
  they must not. The gap between them swings by a stride (the body's
  travel a cycle) about its mean, so the mean is `SHUFFLE_CLOSEST` (0.12 m,
  toe to toe) plus half a stride, never narrower than they stood. The
  leading foot steps out, the trailing one closes: the gap opens to a
  stride past the closest and shuts again.
- **The legs are placed, not curved:** `stance::move_pelvis_and_feet`.
  Spread wide, the pelvis sinks, as a shuffling body does.
- **The pelvis rides one sinusoid a step** (`smoothed_drop`), fitted at
  or under the height every foot down allows over the cycle:
  - 48 samples, plus each foot's last instant down;
  - lowered whole by whatever the fit still rides above;
  - under it the legs bend a little more, and the feet stay placed.
- **The locomotion layer's walk sway is faded out while shuffling**
  (`walker::fade_walk_sway`, passed 1 as for a run). It is the walk's: it
  re-solves the pelvis over the feet a walk's stance timing loads.
- **Stride from speed:** `0.55 × speed` (`SHUFFLE_STRIDE_PER_SPEED`),
  0.10–0.45 m a cycle, about 1.8 cycles a second: short, quick steps.
- **The stride, width and diagonal are set from what is asked, not the
  speed the legs step at**, held through the stop, set outright from a
  stand and eased on the way (`SHUFFLE_REGEAR` 0.4 m/s a second,
  `SHUFFLE_REAIM` 0.5 a second).
- **The feet down are told to the foot locks** (`shuffle::planted`), from
  the clock; while the gait fades in or out, never the foot the fade
  swings (a start's first, a stop's last).
- **The arms** are carried 0.2 rad out from the body, the elbows 0.35 rad
  more bent, each swaying 0.05 rad further out as the opposite leg swings
  (`carry_arms`). Authored, not measured: no recording of a shuffle's arms
  was found. A walk's arms swing against the legs to cancel the body's
  twist about the vertical; across, the legs swing in the frontal plane.
- **Aside and forward at once** (`Walker::speed` and `Walker::aside`):
  - mostly across (45° or more off forward): the shuffle on a diagonal,
    its stride laid along the way, the stance widened for the part across
    only (front to back the feet pass, apart across, as a walk's do);
  - mostly forward: the walk, the body turned toward its way by the angle
    off where it was steered (`WalkerState::strafe`), the head looking
    where it faced.
- **In the walker:** not asked to sit. Changing between shuffle and walk,
  or the shuffle's side, it stops first and starts again.

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
  the end of a swing within 4 mm of the floor gave 53 mm "slid". The
  honest check is a foot that moves between two samples both on the floor.
- **Restarting the other way, a standing foot crept 2.4 cm, 8 mm up.** The
  fade's blend sank it 2 cm in the pose, the locks' speed test let it go
  mid-stance, and the lock's release eased it toward the sunken foot. Told
  the locks which feet are down, it held; then the stance widening 7.5 cm
  a side within the first swing (at 0.8 a stride) left the sprung leg
  lagging, and it flicked 1.7 cm out at lift-off. Shorter strides widen it
  2 cm; nothing is left.
- **The clock alone counted a first swing's foot down** while the fade
  still held it 9 mm up and 1 cm short: locked there, it would have been
  dropped short (10.4 mm, headless). Hence the fade's swinging foot is
  never planted.
- **The swing set down on the arc's own schedule** met the floor still
  going, its last 6–15 mm on the floor each step; across by three quarters
  of the swing, 4–9 mm (in the air, live, between samples).

- **The pelvis took the lowest any foot down allowed, a step a step.**
  A foot set down out wide counted at once (at least 0.2 of the load), and
  its leg asked the pelvis lower that frame:
  - 230-1260 m/s² over 1/240 of a cycle headless;
  - 8-10 m/s² live through the springs, a 7 mm dip at every step.

  Fitted as one sinusoid a step, it was still clipped to the legs' reach
  in the trailing foot's last instant down, which no regular sample caught:
  118 m/s² at 0.6 m/s, until that instant was checked too.
- **The walk's sway put the dip back.** Composed on after the gait, the
  locomotion layer's walk sway re-solved the pelvis over the walk's loaded
  feet, wide apart: 403 m/s² headless, the live dip unchanged by the fit.
  Faded out shuffling, 0.9 m/s² live.

## Consequences

- **The pelvis:**
  - headless, at most 3.2 m/s² through a steady shuffle (0.3-0.6 m/s,
    across and diagonal);
  - live at 0.4 m/s, 0.9 steady and 1.9 starting or changing side;
  - crouched (`sneak`), 1.0-2.6.
- **Model** (`shuffle::tests`, the real rig): a cycle carries the body one
  stride along its way within 1 %, under 1 cm off it, across and on
  diagonals forward and back; the feet never nearer than the closest less
  5 mm; every foot down on the floor within 2 mm; both feet down move
  alike within 0.5 mm a frame (a non-linear sweep fails it at 3.1 mm); a
  start from a stand keeps its planted feet within 8 mm (5.1 / 3 mm, the
  toe joint through the first fade); the hands carried out and forward,
  both sides alike.
- **Live** (gallery):
  - 0.4 left then 0.6 right, then stop: 0.37 / 0.55 m/s; floor-to-floor,
    3 moves over 2 mm in the run (4.7 mm the stop's last foot, 3 mm each
    first swing); the pelvis at most 16 mm down (32 at 0.8 a stride);
    never nearer than 0.125 m; stopped, standing as it stood;
  - 0.25 forward and 0.5 left: 64° left of forward (asked 63°), 0.53 m/s;
  - 1.0 forward and 0.4 left (the walk turned): 21° (asked 22°),
    1.06 m/s, no foot moving on the floor.
- **Seen** at 0.5 m/s, Front and Left, gizmos then the mesh: a foot lifted
  mid-swing beside one standing, legs never crossing, knees bent forward,
  the trunk upright, the arms carried out a little, elbows bent; the walk
  turned to its way, the head kept toward where it faced.

## Revisit when

- **A faded swing lands abruptly.** The start's first swing (and a
  restart's) fades in from mid-swing to the footfall, 6 frames at 1.8
  cycles a second. The foot comes down 50 mm in about 3 frames, still
  going across. The sprung leg trails and lands it pitched toe-down 8°, so
  its tip drags 15-20 mm over 2-3 frames; the ball lands still. A stop's
  last swing does the same.
  - Tried and refused: fading in by three quarters of the swing (the tips
    slid 17-20 mm a frame); keeping the lifted foot level (no change: the
    posed foot is level, the pitch is the springs').
  - A fix wants the shuffle's first step faded over its whole swing (still
    single support), which needs the first step's fade timed apart from
    the stop's (one `TransitionConfig::mid_swing` times both now).
- **The arms' shape** is authored; a recording of a side shuffle would
  settle it.
- **Backward diagonals** from `Walker::speed` below zero: the walk does
  not go backward.

## Related

- [A step aside is the balance's side step and close](./a-step-aside-is-the-balances-side-step-and-close.md) — contrast: one step aside from a stand.
- [Root motion is the rendered contact's displacement](./root-motion-is-the-rendered-contacts-displacement.md) — prerequisite: why the shuffle moves without sliding with no root motion of its own.
- [Fade a gait only through single support](./fade-a-gait-only-through-single-support.md) — context: the start and the stop the shuffle borrows.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — context: the planted feet's locks under a body moving across.
