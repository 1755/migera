---
title: A window is climbed through crouched on its sill, in by a drop, out by a turn and a lowering
description: "Step 15: a window is its sill and its opening's height. In, a hang from the sill climbs through and crouches on it under the lintel, then drops into the room; out, a mantle from the room crouches on it, turns round and lowers into a hang. Read before changing parkour/window.rs or the through variant in hang/up.rs."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-10
verified: 2026-10-10
code:
  - src/character/anim/parkour/window.rs
  - src/character/anim/parkour/hang/up.rs
  - src/character/anim/parkour/hang.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
  - examples/anim_bench.rs
sources:
  - "tests parkour::hang::up::tests::{it_climbs_in_through_a_window, it_climbs_out_of_a_window_into_a_hang}"
  - "live: character_gallery --step-seconds 0.0333333 --window 0,-1,180,1.95,1.0,1.2,0.3,1.05 --hang-at 1 --climb-up-at 4, BRP; gizmos on/mesh off from outside, mesh"
  - "live: character_gallery --step-seconds 0.0333333 --start-height 1.05 --window 0,-1,0,1.95,1.0,1.2,0.3,1.05,1 --mantle-at 1, BRP; gizmos on/mesh off from the room, mesh"
  - "anim_bench --gait window-in / hang-up"
aliases:
  - window
  - climbing through a window
  - sill
  - lintel
  - through_window
  - mantle_through
  - off_window
  - SillTurn
  - Walker::windows
---

# A window is climbed through crouched on its sill, in by a drop, out by a turn and a lowering

Step 15 of the [parkour steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), first part: climbing
in through a window and out of one into a hang.

## Decision

**A window** (`parkour::window::Window`) has three parts:
- its **sill**, a ledge on the outside whose depth is the wall's
  thickness;
- its **opening's height** over the sill (the lintel);
- the **room's floor** height.

The sill seen from the room (`Window::inner`) is the same line, moved in by
the wall's thickness and facing in.

**Both ways use the climb-up's "through" variant**
(`Hanging::through_window`, `Hanging::mantle_through`):
- **Pull, turn over and press** are the climb-up's own: folded over the
  sill, the trunk near flat, the body fits under a normal lintel.
- **The lead foot** comes onto the sill, with the hips 6 cm higher than
  for a top (`STEPPED_THROUGH_UP`).
- **The trail foot follows** and it crouches, both feet on the sill, the
  toes at its inner edge. The hips are 0.5 m over the sill. The trunk
  leans as little as keeps the head 8 cm under the lintel, 0.5 rad at
  least. The hands let go as it crouches.

The sill may be as shallow as 0.2 m (a top to stand on needs 0.55 m).

**Climbing in**: a hang from the sill, asked to climb up, ends crouched on
the sill facing in. Then it steps off into the room (`Hanging::off_window`),
falling on at 1.2 m/s, its feet pushed off the sill (`coast_legs`). The
fall lands on the room's floor.

**Climbing out**, in three steps:
1. From the room, asked to mantle at the inner edge, it mantles up through
   and crouches on the sill facing out.
2. It turns round crouched on it (`SillTurn`, 1 s): the poses and the hips
   blend from crouch to crouch and the yaw turns half round. The feet step
   in turn, lifted 5 cm, the second starting at 0.45 of the turn.
3. It lowers itself into a hang outside: the climb in, played backward
   (`Hanging::lower_down`).

The walker (`Walker::windows`) picks the through variant whenever the
ledge it climbs or mantles is a window's sill.

## Alternatives considered

- **Pressing the sill ahead of the feet as it crouches**: crouched under
  the lintel, the shoulders are too high, and a hand was 40 cm short.
  Hence letting go.
- **Out by stepping off, turning round in the air and catching the sill**
  (the roof's eave catch, `spin_round`):
  - turning beside the wall swung the crouched knees into it, 15 cm;
  - held off the wall as a fall facing one is, the hips, which start
    behind the face inside the opening, were shoved out 50 cm in a frame;
  - keeping only the legs off the wall, a knee still changed step 27 cm
    as it pressed against it.

  People turn on the sill and lower themselves. So does this.

## Traps

- **The hips planned straight down to the crouch** let the lead foot come
  up beside its own hip as it came over the sill, the leg folded to a
  quarter of its length: its knee flipped 1.3-1.6 m in a frame. Raising
  the hips 15 cm instead pulled the pressing hands 7 cm off the sill; 6 cm
  does both.
- **Feet left on the sill under the falling hips** sank 29 cm into it
  before clearing its inner edge. They are pushed off forward and up, and
  start with the toes at the edge.
- **The hands held into the crouch's first part** were carried 7.5 cm out
  of reach by the hips going in. They let go from its start.
- **The end root** belongs under the crouched hips, as the climb's own
  root is, not on the sill (0.44 m apart).

## Consequences

**Headless** (`puppet_base` with its own fingers' grips, 60 fps; a sill
1.95 m up, 0.3 m deep, under a 1.2 m opening, the room 0.9 m down):
- **In, braced and free:**
  - nothing over the lintel, past a jamb or into the wall within its
    thickness;
  - the hands on their hooks and presses to 1 µm;
  - the ankles on the sill to 1 µm;
  - the largest change of step 4.8-5.4 cm, a forearm at the turn-over:
    the plain climb-up's own;
  - off the sill, landed on the room's floor standing, nothing over the
    lintel or into the sill.
- **Out:**
  - nothing over the lintel or into the wall;
  - hanging from the sill at the end;
  - the largest change of step 8.4 cm, a knee 0.7 s into the mantle,
    exactly a plain mantle's own onto a top as high.

**Live** (Xvfb, BRP; gizmos on with the mesh off, and the mesh):
- **In:** hung from the sill, climbed up, crouched on it with the feet on
  its top and the neck at 2.97 m under a 3.15 m lintel, dropped in and
  stood on the room's floor 0.6 m in.
- **Out:** mantled onto the sill and crouched facing out (the neck at
  2.98 m), turned round with the head under the lintel, and lowered
  itself out, the hands on the outer lip.

**Cost**: `anim_bench --gait window-in`, 46 µs a character, as a climb-up
onto a top.

## Revisit when

- **The hands on the frame**: crouched, they let go rather than hold the
  jambs or the lintel.
- **A window level with the room's floor** (a French window): the crouch
  should rise and step in, not drop.
- **Climbing out when the hang is lower than the feet can reach**: the
  lowering hangs wherever the sill is; on a low sill the feet reach the
  ground.

## Related

- [A hang is climbed up from by pull, press and step on](./a-hang-is-climbed-up-from-by-pull-press-and-step-on.md) — prerequisite: the climb whose through variant this is.
- [A block is mantled as a climb up from the floor](./a-block-is-mantled-as-a-climb-up-from-the-floor.md) — prerequisite: the mantle that climbs out onto the sill.
- [A hang is dropped into, let go of and caught from a fall](./a-hang-is-dropped-into-let-go-of-and-caught-from-a-fall.md) — prerequisite: lowering into a hang by playing the climb backward.
- [A steep slope is slid down on the feet as a block with friction](./a-steep-slope-is-slid-down-on-the-feet-as-a-block-with-friction.md) — contrast: the eave catch by a turn in the air that a window's wall rules out.
