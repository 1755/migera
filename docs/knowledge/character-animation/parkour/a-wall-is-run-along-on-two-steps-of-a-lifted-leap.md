---
title: A wall is run along on two steps of a lifted leap
description: "Step 8, third part: a run along a wall is a running leap that lands running, held up by two pushes off the face (Lift, 1.3 m/s up), reshaped onto it: the body leant off, each foot held on a hold sole to the face as the body passes. Read before changing parkour/along.rs, Lift or the walker's run along."
type: decision
status: current
tags:
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/along.rs
  - src/character/anim/jump/leap.rs
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "tests parkour::along::tests::{it_runs_along_a_wall_and_runs_on, a_wall_too_far_askew_or_short_or_off_the_wrong_foot_is_not_run_along}"
  - "live: character_gallery --step-seconds 0.0333333 --ledge -0.55,-7,-90,3.0,8.0 --run-along-at 0.5 --anim-speed 4.0, Xvfb, gizmos on/mesh off and mesh on"
  - "anim_bench --features real_rig --gait run-along --characters 100"
aliases:
  - wall run along
  - horizontal wall run
  - run along a wall
  - HangAsk::RunAlong
  - Jump::along_wall
  - Lift
---

# A wall is run along on two steps of a lifted leap

Step 8 of the [parkour design](./parkour-moves-implementation-design.md),
third part: running beside a wall, two steps along its face, then on. It is
a running leap (`Jump::from_run`, landing on the other foot and running
on), its flight held up by the wall and reshaped onto it (`AlongWall`, as a
[vault](./a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md)
reshapes its leap).

## Decision

**The flight held up** (`jump::Lift`): each step on the face pushes up by a
sin² pulse, its force rising from none and back to none, so the flight's
acceleration has no step:
- 0.7 m/s over 0.16 s, 0.12 s after take-off;
- 0.6 m/s over 0.16 s, 0.06 s after the first.

The flight's time solves for touchdown with the pushes in, and the landing
stance takes the slower fall. With no measured data, the gains are what
friction against the push into the wall allows: about μ times the lateral
velocity the wall turns (~1.5 m/s), not a climb.

**The plan** (`Jump::along_wall`, once, at the take-off foot's contact):
- a leap asked 0.3 m up, from the foot farther from the wall
  (`AlongWall::takeoff_leg`);
- the near foot steps first, the far foot second;
- each step's ankle is under the hips midway through its step, 0.5 and
  0.45 m below them, the ball on the face, the sole turned to it and the
  toes pitched up 0.3 rad;
- the plan poses each step's samples and refuses if a foot is off its hold
  by 1 mm.

**The reshape** (`AlongWall::reshape`, each frame):
- the hips rolled 0.45 rad off the wall, the trunk taking back half;
- each stepping foot brought onto its hold, its knee aimed ahead and a
  little up;
- the root moved so the COM stays the leap's;
- the ball kept out of the face and the knee 2 cm off it, by moving the
  ankle out.

**It refuses** when:
- the run is more than 0.3 rad off parallel;
- it would take off from the near foot;
- the hips are not 0.4-0.75 m off the face;
- a hold is past the wall's ends (0.3 m in) or top (0.3 m under);
- a foot cannot hold its hold.

**The walker** (`HangAsk::RunAlong`, `Walker::ledge` the wall) runs on beside
the wall. At the first contact of the far foot whose run along plans, it
hands the jump to the walker's jump, as a vault does. It drops the ask with
under 1 m of wall left ahead.

## Alternatives considered

- **A sideways path in and out from the wall** (the wall's push turning the
  run): a lateral offset would have to be handed to the run at the landing,
  and its end can't be both still and ballistic. Not built: the run is along
  the wall, the lean takes the feet to it.
- **The ball under the hips midway** (as a run up's hold): the ankle sits
  0.13 m behind the ball, so the leg swept unevenly and the far foot fell off
  its hold at 4 m/s. The ankle is centred instead.
- **A stepping knee aimed up** (as a run up's wall knee): square to a leg
  reaching down to the face, "up" leans into the face, within 2 cm of it.
  Aimed up and out, nearly along the leg, it swung round at 35 m/s. It is
  aimed ahead and a little up.
- **Turning a near knee out from the face as a correction**: the same
  near-degenerate aim, 21-35 m/s. Moving the ankle out keeps it off.

## Traps

- **A foot eased toward a hold fixed in the world** chases a target running
  back past the body at the run's speed: the near toe went 18 m/s. Coming
  on, it eases toward where the hold will be about the body at contact (fixed
  in the pose's frame), blended into the world hold by then (w² of the
  travel since).
- **The far foot coming from behind the body** needs all the time from
  take-off; brought on in 0.12 s, its toe went 18-22 m/s.
- **Re-posing a foot through passes** compounded its rotation's norm until
  FK grew the foot 0.7 % and the ball sank 1.7 mm: see
  [rotations turned over and over in a frame need renormalizing](../rig-and-retargeting/rotations-turned-over-and-over-in-a-frame-need-renormalizing.md).
- **The gallery's shots were timed by guess** and missed the move by
  0.7 s; a print of the walker's trigger (where it took off) found it.

## Consequences

**Headless** (`puppet_base`, 60 fps; a wall on the left and on the right,
0.45, 0.55 and 0.65 m off; 3.5, 4 and 4.5 m/s; all 18 run along):
- each foot held within 0.0005 mm through its step;
- nothing into the wall;
- no joint over 13.9 m/s about the hips (the leap's own up to 13.1);
- the holds 0.73-0.75 m up;
- the COM's acceleration stepping at most 2.7 m/s² a frame;
- every one runs on.

Left and right mirror exactly. **Refused**: off the near foot, 1.2 and
0.1 m off, 0.4 rad askew, a 1 m wall.

**Live**: at 4 m/s beside a 3 m wall 0.55 m off, it took off from the far
foot, put the near foot on the face then the far one, leant off the wall,
landed and ran on.

**Cost**: `anim_bench --features real_rig --gait run-along --characters
100`, 143 µs a character at p50, about a vault's (147 µs): the reshape poses
its feet in passes.

## Revisit when

- **A third step, or more height**: two steps lift 1.3 m/s, holds about
  0.75 m up. More needs a sideways path to come back to the wall.
- **Running along from a slanted approach**: the walker must be running
  beside the wall already; it does not steer onto it.
- **The cost**: the reshape's 3 × 4 passes could stop early.

## Related

- [A low obstacle is speed-vaulted as a reshaped running leap](./a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md) — same-pattern: a leap's flight reshaped, its COM kept.
- [A wall is run up off one foot and its lip caught](./a-wall-is-run-up-off-one-foot-and-its-lip-caught.md) — contrast: a foot on a wall that carries the body up into a catch.
- [A wall is kicked off toward a lip round a corner](./a-wall-is-kicked-off-toward-a-lip-round-a-corner.md) — contrast: one foot on a wall, the flight turned to another's lip.
- [A jump from a run replays the run's stance on a planned COM](../ik-and-locomotion/a-jump-from-a-run-replays-the-runs-stance-on-a-planned-com.md) — prerequisite: the leap this holds up.
- [Rotations turned over and over in a frame need renormalizing](../rig-and-retargeting/rotations-turned-over-and-over-in-a-frame-need-renormalizing.md) — deeper: the trap found here.
