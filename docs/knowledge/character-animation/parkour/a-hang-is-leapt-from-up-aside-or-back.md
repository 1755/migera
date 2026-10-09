---
title: A hang is leapt from, up, aside or back, as a launch and a ballistic flight
description: "Step 6: a leap from a hang is a launch (the hips on a Hermite curve to the release velocity, the hands letting go over its last 0.12 s), then a Falling aimed so the shoulders come just under the target's lip; back turns round in the air. Read before changing hang/leap.rs, the release hand-off, or the catch sweep."
type: decision
status: current
tags:
  - locomotion
  - ik
  - biomechanics
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/hang/leap.rs
  - src/character/anim/parkour/hang.rs
  - src/character/anim/parkour/hang/up.rs
  - src/character/anim/parkour/fall.rs
  - src/character/anim/walker.rs
sources:
  - "tests parkour::hang::tests::{leaps_from_a_hang_catch_or_land, leaps_catch_at_low_frame_rates}"
  - "live: character_gallery --step-seconds 0.0333333 --block 0,-1,180,2.25,2.0,1.0 --side-ledge 0,2.0,0,2.0,3.0 --hang-at 1 --leap-at 16,back (and up from a step, aside to --side-ledge along), Xvfb, gizmos on, mesh off"
  - "anim_bench --gait leap-up|leap-aside|leap-back --characters 100"
  - "Hiley and Yeadon 2003, bar release window (parkour-movement-data)"
aliases:
  - Leap
  - HangAsk::Leap
  - Hanging::leap
  - Hanging::release
  - Falling::spin_round
  - Falling::aim_at
  - cat leap
  - leap back
  - leap up
  - leap aside
  - jump from a hang
---

# A hang is leapt from, up, aside or back, as a launch and a ballistic flight

Step 6 of the [parkour design](./parkour-moves-implementation-design.md):
from a hang, leap up to a ledge above, aside to the next ledge along, or
back off the wall, turning round, to catch a ledge behind or land.

## Decision

**A leap is a launch in the hang, then a fall** (`Hanging::leap`,
`HangAsk::Leap`). In the launch (0.35 s, `LAUNCH`) the hands keep their
hold. The hips go from where they hang to where they let go on a Hermite
curve, from the hang's own velocity to the release velocity:
- **up**: pulled up 0.25 m and out 0.25 m;
- **aside**: swung 0.15 m along and up;
- **back**: pushed 0.2 m out with the feet and 0.1 m up.

Let go, the flight is a `Falling` (step 4), so it is ballistic and lands
or catches as any fall (step 5).

**The hands let go over the launch's last 0.12 s** (`LETTING_GO`), inside a
bar release's 73-157 ms window (Hiley and Yeadon 2003). The arms are blended
from the hang's toward the pose they leave in, the wrists 5 cm up off the
lip. The grips fade with them.

**The release velocity is planned from the ledge leapt at.** The catch is
planned where the shoulders come 8 cm under its lip (`UNDER_LIP`), the
hips 0.42 m out from its face (`CATCH_OUT`, a braced hang's rest). The
flight takes the longer of:
- the time to the top of the flight;
- the distance across at 3 m/s (`FASTEST`).

The longer time means it catches falling, after the top, when crossing is
long. The vertical speed then follows from the height to the catch, at most
4.5 m/s (`MOST_UP`). The release does not happen if it would need more.

**Reach**:
- up, a ledge 0.3-1.1 m above the lip held;
- aside, a gap of up to 1.8 m, its lip within 0.6 m up or down;
- back, a face 0.8-3.5 m behind, its lip at most 0.5 m higher.

The ledges come from `Hanging::others`. Back with nothing in reach, it
leaves at 2.2 m/s out and 1.6 up and lands turned round.

**Back turns round in the air** (`Falling::spin_round`, π). The turn
starts 0.1 s after letting go and eases in with a smootherstep. It takes
0.5 s, or 0.8 of the flight if that is shorter. The catch waits until the
turn is done.

**The hand-off carries the body's motion** (`Hanging::release`). The fall
starts from the hang's root and pose with the launch's velocity. The legs
coast on their swing, read over the launch's last 10 ms. The trunk and arms
coast on their pose carried ahead the same way. The fall is aimed at the
target (`aim_at`). The hand clamp treats the target's wall as it treats a
wall the fall faces, and the fall is held off the wall it faces
(`against`).

**The walker** releases into the fall when the launch ends. During the
flight it tests the catch against the target alone.

**Diagonal ejects** (step 12 of the [steps beyond the first
ten](./parkour-moves-beyond-the-first-ten-steps.md), added 2026-10-09):
- `Leap::UpAside`: to a ledge 0.3-1.1 m up whose nearest grip is 0.4-2.15 m
  along to that side, on the same face. It launches from the aside leap's
  point (swung along and up).
- `Leap::BackAside`: to a face behind whose nearest grip is 0.4 m or more
  off to that side (the hang's own side), turning round in the air. It
  launches from the back push, swung along too.
- The flight is aimed as any leap's, at the target's nearest grip.

## Alternatives considered

- **A flight of its own** instead of `Falling`: the fall already has the
  ballistic path, the landings, the hold off a wall and the catch. Only the
  spin and the aim were new.
- **Catching at the top of the flight only**: a ledge level along needed
  5.9 m/s across. Taking the longer of the two times keeps every leap at or
  under 3 m/s.

## Traps

- **The hands held to the lip until the instant they let go** jolted the
  wrists 7.8 cm in the release frame. The arms now blend toward the
  leaving pose over the last 0.12 s. Chasing the wrist with IK instead made
  a straight arm's elbow jump 3.4 cm.
- **Each part of the body must carry its own motion over the hand-off**:
  - a toe from rest jumped 2.2 cm;
  - the head jumped 2.0 cm;
  - a knee turned at once by an immediate spin jumped 2.2 cm.

  Hence the coasting legs, the trunk and arms carried ahead, and the
  delayed, eased spin.
- **Rising straight up from under the lip** put the shoulders and knees
  11 cm through it. Hence the pull out.
- **A catch at 0.3 m out** leant the trunk to the grip and put an elbow
  3.9 cm into the target's wall. Hence 0.42.
- **Farther out is no cure.** At 0.55 m the leap up missed its catch and
  the leap aside never caught: the rig's arms are short.
- **The fall's hand clamp pushed the hands onto the face's plane** above
  the top (6 cm). It now acts only under the top.
- **The feet left on the wall below** went 9 cm into it in a leap up. Hence
  the fall is always held off the wall it faces.
- **Caught feet on a step** went into the step. A braced foot is pushed out
  onto a parallel, protruding block (`Hanging::face_at`).
- **The catch estimated by moving the body back as the hips moved** missed
  the leap back at 5 fps, because the body turns in the frame. The sweep
  now poses the body at eight times across the frame (step 5's note has
  the cost).
- **A ballistic test must check each case, not stop at the first.** The
  first ballistic check stopped at the leap up and hid two other faults:
  - **the release was pushed off a wall in line.** Leaping aside, the
    target's face lies in the plane of the wall let go. The braced hips
    (0.42 m out) began inside the hold's give (0.45 m), and were pushed
    1.1 mm out in the first frame (4 m/s²). The give now starts where the
    hips leave ([the hold's note](./a-fall-facing-a-wall-is-held-off-it.md)).
  - **the landing's room came in before the catch.** Leaping back 2 m, the
    hips arrive at 3 m/s. The room the landing at the wall's foot will
    need is ramped in about 0.3 s before touchdown, and that ramp starts
    before the catch. It pushes the hips out at 3.6 g over the last two
    frames, about 1 cm in all. Dropping the ramp for a leap would land a
    missed catch with a jolt, so it is kept.
- **A wall's plane is not the wall.** Leaping up and aside to a higher
  ledge on the same face, the hands, still over the lip of the ledge left,
  were behind the target's plane. The fall keeps the hands off the
  target's face, and pushed them out onto its plane though they were off
  its side: an elbow swung 12-15 cm the frame it let go. Moving the launch
  point did not change it. The keep-off (margin and plane both) now fades
  over 0.2 m outside each wall's ends.

## Consequences

**Headless** (`puppet_base`, 60 fps; up from a step 0.12 m proud in a 2.5 m
wall to its lip 0.8 m above; aside across 1 m and 2.06 m gaps; back to 2 m
walls 2.5 and 3 m behind; back with nothing behind):
- **the release**: no joint's step changes 2 cm in the frame it lets go;
- **the flight is ballistic**: the hips fall at g, within 0.003 m/s². No
  push acts along the wall or across it. The only other force is the faced
  wall holding the hips out: 0.17 g up (the feet coming up the wall
  below), 3.6 g back 2 m, 0.2 g back 3 m, none aside;
- **caught**, the wrists are held within 1 mm, at the height of the ledge
  leapt at;
- **back with nothing behind**, it lands turned round within 0.01 rad, its
  root on the floor;
- **nothing goes into a block**, the launch, flight and hang (under 1 mm);
- **frame rate**: aside 2 m and back 3 m catch at 60, 20, 10 and 5 fps.

The ballistic check fails, as it should, with the give's cut undone.

**Live** (Xvfb, a fixed 1/30 s step, gizmos on, mesh off):
- **up**: from the 2.25 m step's lip it catches the wall's lip above (wrist
  2.95 against a lip of 3.05), the feet braced on the step's face;
- **aside**: it hangs from the next ledge along;
- **back**: from the 2.25 m block's front it pushes off and turns round.
  It catches the 2.0 m wall behind, the hands over its top and the
  feet on its face. Checked from the right side and the top.

**Cost** (`anim_bench --characters 100`, a character a frame, launch posed
from its start and the catch swept each frame):

| Mode | Cost |
|---|---|
| `leap-up` | 42 µs |
| `leap-aside` | 41 µs |
| `leap-back` | 45 µs |

Similar to a grab and hang (55 µs) and a climb up (42 µs).

**Diagonals** (headless, the same test): up and right 1.0 and 1.5 m along
to a lip 0.6 m higher, and back and right to a face 2.5 m behind, 1 m
along. Each caught its target with the hands held, the flight ballistic
and nothing into a block. Letting go changed a step by 1.15-1.4 cm, as
the straight leaps do. Live, up and right to a 2.85 m lip 1.8 m along, it
caught it and hung 1.38 m along. `anim_bench --gait leap-up-aside` 37
µs, `leap-back-aside` 40 µs a character at p50.

## Revisit when

- **A leap's room before the catch**: plan the hold's landing room from the
  catch rather than ahead of the touchdown, if the 3.6 g shove before a
  fast catch shows.
- **Data**: no leap from a hang has measured numbers beyond the release
  window. The launch's distances and times are set by eye.

## Related

- [Parkour moves, step by step](./parkour-moves-implementation-design.md) — prerequisite: the design this is step 6 of.
- [A hang is dropped into, let go of, and caught from a fall](./a-hang-is-dropped-into-let-go-of-and-caught-from-a-fall.md) — prerequisite: the fall a leap releases into and the swept catch that ends it.
- [A fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md) — deeper: the hold and its give, and why a leap's hips leave inside it.
- [A ledge is caught near the top of a jump and hung from](./a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) — prerequisite: the catch-near-the-top plan the release velocity follows, and the hang a catch starts.
- [A drop is fallen ballistically and landed to the measured time and depth](./a-drop-is-fallen-ballistically-and-landed-to-the-measured-time-and-depth.md) — prerequisite: the landing a leap back with nothing behind ends in.
- [Parkour movement data](./parkour-movement-data.md) — deeper: the bar release window, the only leap data there is.
