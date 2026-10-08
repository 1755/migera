---
title: A wall is run up off one foot and its lip caught
description: "Step 8, first part: a run up a wall is a braked running leap, one foot planted on the face 1 m up for 0.37 s (the hips on a Hermite curve), then a fall aimed at the lip; it catches walls up to 2.6 m on puppet_base, lands under higher ones. Read before changing parkour/wall.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/wall.rs
  - src/character/anim/parkour/fall.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
sources:
  - "tests parkour::wall::tests::{it_runs_up_a_wall_and_catches_its_lip_or_lands, a_wall_met_askew_or_from_the_wrong_spot_is_not_run_up}"
  - "live: character_gallery --step-seconds 0.0333333 --block 0,-8,180,2.5,3.0,1.0 --wall-run-at 0.5 --anim-speed 4.0, Xvfb, gizmos on, mesh off"
  - "anim_bench --features real_rig --gait wall-run --characters 100"
  - "Croft, Schroeder and Bertram 2019; Lawson 2015 (parkour-movement-data)"
aliases:
  - wall run
  - wallpass
  - run up a wall
  - WallRun
  - HangAsk::WallRun
---

# A wall is run up off one foot and its lip caught

Step 8 of the [parkour design](./parkour-moves-implementation-design.md),
first part: running at a wall, plant a foot on it, drive up, catch the lip.
This is Assassin's Creed's wallpass.

## Decision

**Three parts, planned once at take-off** (`WallRun::plan`, in the take-off
leap's frame):
1. **Take-off.** A running leap (`Jump::from_run`) leaving the floor at
   2.9 m/s up (Lawson: 2.93), asked to land short so its plant leg brakes
   all a leap can. The lead (free) foot is brought onto its hold, from the
   take-off on: the ball on the face 1.0 m up (Croft: the first wall step
   1.0 m) under its socket, toes up 1.1 rad. It meets the face 0.1 s after
   leaving the floor.
2. **On the wall** (0.37 s, Lawson). The foot is held on the face. The hips
   go on a cubic Hermite from the leap's position and velocity to a leave
   state: 0.42 m out, and as high as the leg on its hold reaches (0.96 of
   it). The rest of the body moves with them:
   - the pose blends from the leap's over 0.1 s;
   - the trunk is upright;
   - the arms swing up to nearly overhead, the elbows bent through the
     middle of the swing;
   - the take-off leg's knee drives up, kept 0.25 m off the face;
   - the knees point up and out to their own sides;
   - the wrists are kept 0.1 m off the face under the lip.
3. **Off it.** A `Falling` from the leave state at the hips' velocity:
   - up: as fast as brings the shoulders to 8 cm under the lip at the top
     (a leap's catch), between 1.0 and 2.6 m/s;
   - aimed at the lip, held off the wall;
   - the legs and trunk coasting on, the foot on the wall peeling off it at
     1 m/s.

   It catches the lip as any fall does, or lands at the wall's foot.

**It refuses** (`None`) when:
- the run meets the wall more than 0.3 rad off square;
- the hips would meet it more than 0.35 m off their 0.75 m;
- the hold is past 0.98 of the leg from the socket as the foot meets it;
- the hips would brake on the wall harder than 3 g.

`WallRun::reaches_lip` says whether the catch comes (the shoulders topping
out within 0.18 m under the lip).

**The walker** (`HangAsk::WallRun`, `Walker::ledge` the wall's lip) takes it
as a vault: at each foot's contact running at the wall, it adjusts its pace
for a foot to come down at the best take-off (`WallRun::takeoff`). It takes
off from the first foot whose run up plans, where the next foot would be no
nearer the best. It does not go round the wall while running at it.

## Alternatives considered

- **Lawson's 0.84 m rise on the wall**: the COM's rise is helped by the
  ankle's push off the toes, which this pose has not. The foot was dragged
  off its hold, the leg out of reach (19 cm). The rise is now as far as the
  leg reaches.
- **Meeting the wall at the run's speed**: at 4.5 m/s the hips braked at
  2.8 g on it, and from 0.1 m nearer past 3 g. Lawson's run up meets the
  wall at 2.35 m/s, so the take-off brakes first.
- **Planning the flight's top an arm's reach under the lip**: the hands
  never came within reach on the way down. A leap's catch plans the
  shoulders just under it.

## Traps

- **An arm swung straight up points at the wall mid-way**, 7-17 cm in; the
  elbows bend through the swing, and the wrists are kept off the face. An
  elbow within 0.2 m of the face is turned along it (taken off near, a
  kick's elbow went 2-8 cm in).
- **A knee aimed one way before the foot meets the wall and another way
  after** turned round at 16 m/s.
- **A foot's move to its hold must not start before the take-off itself**:
  part-way there at the first frame, it went 14 m/s.
- **The fall's landing switched its arms from reaching to landing in a
  frame**: a missed lip swung a hand at 40 m/s. The fall now eases its arms
  down over 0.15 s (`fall.rs`, `ARMS_DOWN`), for any fall reaching for a
  lip.
- **The synthetic rig cannot plan it.** Its leg bones are shifted a joint,
  so the hip socket's bone sits at the knee and the straight leg measures
  0.49 m. Its cost is measured on the real rig (`--features real_rig`).

## Consequences

**Headless** (`puppet_base`, 60 fps; walls 2.3-3.4 m; 3.5 and 4.5 m/s;
either foot; take-offs from 0.25 m near to 0.25 m far of the best, 72
planned, the rest refused for room to brake or reach):
- the foot is held within 0.013 mm;
- nothing goes into the wall beyond 0.12 mm (the fall's toe keep-off is
  allowed 2 mm);
- no joint goes over 12.8 m/s about the hips;
- the hips brake at 1.1-2.7 g on the wall;
- it catches walls up to 2.6 m and lands under 2.7 and 3.4 m, as planned.

**Live**: at 4 m/s at a 2.5 m block, it took off, put its foot on the face,
drove up with its arms overhead, and caught the lip, hanging braced.

**Cost**: `anim_bench --features real_rig --gait wall-run --characters 100`,
32 µs a character at p50 (p99 12 ms for 100, the catch's sweep).

## Revisit when

- **Higher walls**: a second wall step (a hand or a foot) goes higher
  (Lawson). This rig, its arms 17 % short, catches 2.6 m.
- **Sliding down a wall missed**: a run up too high falls back free; the
  slide down is built from a braced hang only ([its
  note](./a-wall-is-slid-down-as-a-braked-fall.md)). The kick off a wall
  toward another's lip is built on this move ([its
  note](./a-wall-is-kicked-off-toward-a-lip-round-a-corner.md)); the run
  along a wall is a lifted leap instead ([its
  note](./a-wall-is-run-along-on-two-steps-of-a-lifted-leap.md)).

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is step 8 of.
- [A hang is leapt from, up, aside or back](./a-hang-is-leapt-from-up-aside-or-back.md) — same-pattern: a launch handed to a fall aimed at a lip, the release's coasting.
- [A low obstacle is speed-vaulted as a reshaped running leap](./a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md) — same-pattern: the walker's pace adjusted to a take-off.
- [Parkour movement data](./parkour-movement-data.md) — deeper: Croft's and Lawson's wall-climb numbers.
- [Synthetic rig's leg segments are shifted a joint](../rig-and-retargeting/synthetic-rig-leg-segments-are-shifted-a-joint.md) — same-trap: why its cost is measured on the real rig.
- [A wall is kicked off toward a lip round a corner](./a-wall-is-kicked-off-toward-a-lip-round-a-corner.md) — applies: the tic-tac, this move's take-off and wall phase with a leave planned for another wall's lip.
