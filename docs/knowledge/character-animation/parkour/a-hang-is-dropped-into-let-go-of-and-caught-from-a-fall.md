---
title: A hang is dropped into, let go of, and caught from a fall
description: "Step 5: dropping down runs the climb-up backward; letting go falls, onto a step under the feet if roomy; a jump over an edge falls on at its centre of mass's velocity; any fall catches a ledge in reach, swept over the frame. Read before changing hand-offs between hang.rs, up.rs, fall.rs and the walker's jump."
type: decision
status: current
tags:
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-08
verified: 2026-10-08
code:
  - src/character/anim/parkour/hang.rs
  - src/character/anim/parkour/hang/up.rs
  - src/character/anim/parkour/fall.rs
  - src/character/anim/walker.rs
  - src/character/anim/parkour/geometry.rs
sources:
  - "tests parkour::hang::up::tests::it_lowers_itself_down_into_the_hang, parkour::hang::tests::{letting_go_falls_and_lands, falling_past_a_ledge_it_catches_it, a_ledge_falling_past_is_caught_at_any_frame_rate, letting_go_over_a_step_lands_on_it_or_clears_it, a_jump_falling_short_catches_the_far_ledge, a_jump_falling_short_catches_at_any_frame_rate}"
  - "live: --block 0,1,180,3.6,1.2,3.0 --side-ledge 0,1.35,180,1.4,1.2 --start-height 3.6 --drop-down-at 2 --let-go-at 25; --block 0,-0.3,0,3.0,1.2,3.0 --side-ledge 0,-1.7,180,3.0,1.2 --start-height 3.0 --jump-at 6:0.3:1.2 [--catch]"
  - "live: character_gallery --block 0,1,180,2.0,1.2,3.0 --start-height 2.0 --drop-down-at 2 --let-go-at 25; and --block 0,1,180,3.6,1.2,3.0,0.15 --side-ledge 0,0.65,180,1.9,1.2 --start-height 3.6 --drop-down-at 2 --let-go-at 30 --catch, Xvfb, BRP"
aliases:
  - HangAsk::DropDown
  - HangAsk::LetGo
  - Walker::catch
  - lower_down
  - let_go_velocity
  - Falling::catches
  - Hanging::caught
  - Hanging::let_go
  - Falling::from_jump
  - Falling::land_at
  - missed jump
  - landing on a step
  - dropping down to a hang
  - letting go
  - catching a ledge
---

# A hang is dropped into, let go of, and caught from a fall

Step 5 of the [parkour design](./parkour-moves-implementation-design.md)
joins the hang (steps 1-3) and the fall (step 4) both ways. Standing on a
top it drops down into the hang. Hanging, it lets go and falls. Falling
past a ledge, it catches it.

## Decision

**Dropping down is the climb-up run backward** (`HangAsk::DropDown`,
`Hanging::lower_down`). The walker walks straight to the spot where a climb
up from the hang would end (`standing_spot`), with no lead-in in front of a
wall. It stands there facing as the hang will, its back to the edge, and the
climb-up's plan ([step 2](./a-hang-is-climbed-up-from-by-pull-press-and-step-on.md))
runs from its end back to the hang at 0.75 of its rate (`LOWER_RATE`;
lowering is eccentric and slower). The walker never stops exactly on the
spot. Its miss is carried as an offset on the whole body and eased out over
the first 0.6 s (`MISS_EASED`), so the first frame is the standing pose
where it actually stood.

**Letting go hands the hang to the fall** (`HangAsk::LetGo`,
`Hanging::let_go`). The fall takes the hang's root, facing and pose, and
`let_go_velocity()`: the hips' velocity, plus a push off the wall with the
feet when braced (0.6 m/s, `PUSH_OFF`, set by eye). The fall's own trunk,
arms and foot attitudes are blended from the hang's over 0.35 s
(`ARMS_FREE`). The legs already reach from the leaving pose.

**Where it lands** is the ground under where that push would bring it to
rest, sampled at the feet's height. A step under the feet (higher by more
than 0.3 m) changes the landing:
- with room to stand (0.2 m of top out from the ankles, `STEP_ROOM`), it
  lands on the step: aimed (`Falling::land_at`) so the ankles come 0.13 m
  further out than hanging (`STAND_OFF`), facing the wall above;
- with no room, it clears the step's edge, its ankles landing 0.35 m past it
  (`STEP_CLEAR`).

**A jump in the air goes over an edge into a fall** (`Falling::from_jump`).
When the ground under an airborne jump drops more than a step, the walker
hands the jump to a fall. The fall leaves with the **centre of mass's**
velocity: in a jump that, not the hips, is ballistic. The legs and the trunk
and arms coast on their own swing for 0.15 s (`LEGS_COAST`). While the jump
still pushes off the edge, the root is held up over the drop. A fall then
catches as any other: with `Walker::catch`, a standing jump of 1.2 m off a
3 m top catches a wall as high 1.7 m off.

A fall facing a wall is kept off it, catching or landing (see
[a fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md)).

**Catching is step 1's catch from any falling body** (`Walker::catch`).
Asked to catch, the falling arms go up overhead (`Falling::reach`). The fall
catches a ledge when it faces it within 45°, its hips are 0.05-0.8 m out
from the face, and its shoulders pass under the lip within 0.9 of an arm's
length. `Hanging::caught` then starts the hang from the body's hips,
velocity and pose, with step 1's spring and pendulum taking it. Braced, the
feet swing out in an arc (0.3 m) onto the wall over 0.5 s.

**The catch is swept over the frame.** The shoulders at the frame's start
are the body moved back as the hips moved (ballistic, exact for the hips).
The test is made where they passed the lip's height. The walker runs on the
variable frame time.

## Alternatives considered

- **A drop-down plan of its own**: the climb-up's plan already goes through
  every shape (the hands placed on the top, the press, the legs over). Run
  backward and slower, it needed only the miss easing and one elbow fix
  (see the traps).
- **Catching tested at the frame's end alone**: see the traps.

## Traps

- **The press's elbow pole lay along the arm going down.** Lowering, an
  elbow flipped across at 10 m/s. The pole now has a downward part
  (`PRESS_ELBOW_DOWN`, 0.5), and the press's weight turns over in the second
  half of the hand's window. The fastest joint is now 4.9 m/s.
- **The fall's foot attitudes snapped.** Let go from a braced hang, a toe
  jumped 17 cm in a frame, because the feet went straight to the flight's
  attitude. They are blended from the leaving pose with the trunk and arms.
- **Dropped straight down from a braced hang**, a hand swung forward
  landing went 13 cm into the wall. Hence the push off.
- **The feet straight onto the wall after a catch** put a knee 6.8 cm into
  it. Hence the arc out.
- **A catch tested at the frame's end alone is frame-rate dependent.** The
  lip is in reach for only about 0.36 m of the fall, and at 5 m/s that is
  about 70 ms. At 9 fps (a hitch, or lavapipe live at 6) the shoulders fell
  past between two frames and it fell on to the ground. Swept, it catches
  at every rate from 6 to 120 fps.
- **A step under braced feet is landed on, not fallen through.** Falling to
  the ground under the root, the feet went 0.99 m into a step 0.35 m out
  under them. The root itself hangs over the step, so the ground must be
  read under the rest spot at the feet's height.
- **The hips are not ballistic in a jump; the centre of mass is.** Thrown
  at the hips' own velocity, which swings about the centre of mass's as the
  legs tuck (1.43 m/s forward one frame, 1.04 two later), the fall from a
  jump landed 0.3 m shorter at 30 fps than at 60, and missed the ledge.
- **The root crosses the edge before the feet leave.** A standing jump
  travels 0.33 m while it pushes off. The ground check snapped the root
  down to the gap's floor, and the jump landed there. It is held up until
  the jump is airborne.
- **Overlapping blocks**: `LedgeGround` read the first-listed top, and a
  step built into a wall made the ground near the top's edge 2.2 m lower.
  The walker shuffled 55 s before dropping down. It reads the highest.
- **Comparing the hand-off with the step before it** shows the jump's own
  take-off. Compare the fall's first step with the jump's own next.

## Consequences

**Headless** (`puppet_base`):
- **Dropping down** from a 2.15 m wall's top, braced and free, standing
  7 cm off the spot:
  - it starts within 1 cm of the standing pose;
  - nothing goes into the block (under 1 mm);
  - the fastest joint is 4.9 m/s;
  - it ends hanging with the hands held.
- **Letting go** of a 3 m hang, braced and free:
  - no joint jumps more than 3 cm as it lets go;
  - nothing goes into the wall;
  - it lands and stands.
- **Catching** the 1.9 m wall 0.35 m under a 3.6 m slab's edge:
  - the wrists are held within 0.09 mm;
  - 3.5 body weights at the hips;
  - nothing goes into the wall;
  - it settles;
  - it catches at every one of 41 frame rates from 6 to 120 fps.
- **Letting go over a step** 1.4 m high under a 3.6 m braced hang: on one
  0.35 m deep it lands standing on it, the ankles 5 cm or more in from its
  edge; past one 0.1 m deep, it lands on the floor beyond, the ankles 15 cm
  or more past its edge. No joint jumps 3 cm, none goes into the wall or
  the step.
- **A jump falling short**, off a 3 m top toward a wall as high:
  - the fall's first step within 1 cm of the jump's own next, every joint;
  - from 0.3 m back of the edge (the wall 1.7 m off), reaching, it catches,
    held, nothing into the wall, at 5-60 fps;
  - from 0.5 m back (1.9 m off) it passes the lip 0.6 m out and lands;
  - not reaching, it lands against the wall, pushed back off it at most
    1.9 g, nothing under the floor.

**Live** (Xvfb, BRP, gizmos on, mesh off; back and side views):
- **Dropping down** off a 2.0 m top: the wrists end 10 cm under the lip, the
  balls of the feet on the wall's face.
- **Letting go**: it lands standing 0.55 m out.
- **The catch**: it drops down off the 3.6 m slab, lets go, and catches the
  1.9 m wall. The wrist is 10 cm under its lip, the feet are braced on its
  face, and it holds steady.
- **Onto a step**: let go from a 3.6 m wall over a 1.4 m step 0.35 m out,
  it stands on the step, the balls of the feet 13 cm off the wall.
- **A missed jump**, off a 3 m top toward a 3 m wall 1.7 m off, at 5 fps:
  with `--catch` it hangs from the far lip, the feet braced on its face;
  without, it lands at the wall's foot, the hands 10 cm off it on the way
  down, standing 0.47 m out.

**Cost** (`anim_bench --characters 20`, a character a frame):

| Mode | Cost |
|---|---|
| `drop-down` | 42 µs, as climbing up |
| `let-go` | 25 µs (19 before the wall it faces was kept off) |
| `catch` | 24 µs (with the swept catch test each frame) |
| `jump-catch` | 30 µs |

## Revisit when

- **A running jump**: only a standing jump falling short is tested.
- **The ground under one foot only**: the step is landed on or cleared
  whole.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is step 5 of.
- [A ledge is caught near the top of a jump and hung from](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) — prerequisite: the catch's spring and pendulum a fall's catch reuses.
- [A hang is climbed up from by a pull, a press and a step on](./a-hang-is-climbed-up-from-by-pull-press-and-step-on.md) — prerequisite: the plan dropping down runs backward.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — prerequisite: the fall letting go hands the body to.
- [A fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md) — deeper: how a fall from a jump or a let-go stays out of the wall it faces, and the caught hang after it.
- [A jump is planned as its centre of mass's path](../ik-and-locomotion/a-jump-is-planned-as-its-centre-of-mass-path.md) — prerequisite: why a jump's centre of mass, not its hips, is what a fall carries on.
