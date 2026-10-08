---
title: A wall is slid down as a braked fall
description: "Step 8, fourth part: sliding down a wall from a braced hang is the let-go fall with half its gravity braked off (Falling::slide), palms on the face above the shoulders, feet braced on it for 60 % of the flight, landed as from half the drop. Read before changing Falling::slide or Hanging::slide_down."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/fall.rs
  - src/character/anim/parkour/hang.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::hang::tests::sliding_down_a_wall_brakes_the_fall_and_lands"
  - "live: character_gallery --step-seconds 0.0333333 --block 0,1,180,4.5,1.2,3.0 --start-height 4.5 --drop-down-at 2 --slide-down-at 25, Xvfb, gizmos on/mesh off and mesh on"
  - "anim_bench --gait wall-slide --characters 100"
aliases:
  - wall slide
  - slide down a wall
  - HangAsk::SlideDown
  - Falling::slide
---

# A wall is slid down as a braked fall

Step 8 of the [parkour design](./parkour-moves-implementation-design.md),
fourth part: hanging braced from a wall, slide down its face, hands and feet
braking, and land softer than letting go. It is the
[let-go fall](./a-hang-is-dropped-into-let-go-of-and-caught-from-a-fall.md)
with a brake, posed against its wall.

## Decision

**The fall** (`Falling::slide`):
- gravity in the air is cut by `SLIDE_BRAKE` (0.5): the hips' path, the time
  to touchdown and the touch speed all use the braked value;
- `Falling::dropped` reports the drop free fall would meet the ground as
  fast from, half the height. The landing's time and depth, a hurt landing
  and a fatal one are all judged on it. A slide from a hang 6.5 m up lands;
  letting go there is fatal.

With no measured data, half: it touches down at 0.71 of free fall's speed.

**The pose** while sliding:
- **Arms:** the reaching shape, each palm held on the face (the wrist 4 cm
  off it), no higher than 0.25 m above its shoulder. The hands come down to
  that over 0.35 s from letting go, the elbows turned down and out to their
  sides (their own pole eased into that over 0.1 s), and they let go of the
  face over 0.25 s once landed.
- **Legs:** braced on the face as they left the hang for 60 % of the
  flight, the feet keeping their attitude, then brought to the landing's
  shape.
- Held off the wall by the fall's own wall room
  ([a fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md)),
  with no push off the wall.

**From a hang** (`Hanging::slide_down`): braced only, not over a step under
the feet, and only with the wall found reaching the ground. Otherwise
`None`. The walker (`HangAsk::SlideDown`) then lets go instead, and it does
not reach to catch a ledge while sliding.

## Alternatives considered

- **The flying arms' 0.1 m keep-off, kept on**: the hands hang in the air
  off the face, which reads as a fall, not a slide.
- **The palms at the reaching arms' own height**: overhead, with the
  shoulders 0.35-0.41 m out (the wall room), the arm could not reach the
  face: 7-10 cm off.

## Traps

- **Pulling the hands down as they come under the top**: over 0.2 m of fall
  (0.2 s), the forearms went 21 m/s. The drop comes over time from letting
  go instead.
- **Letting the face go the frame it lands**: a hand at 17.7 m/s.
- **The arm passing straight on its way from overhead to the face**: its
  own elbow pole is noise there, and the elbow turned round at 16-19 m/s.
  The same class as [a near-straight leg's
  hinge](../ik-and-locomotion/a-near-straight-leg-bends-toward-its-kneecap.md).
  A fixed pole fixes it, but snapped in at once it jumped the elbow 4.9 cm;
  eased in over 0.35 s it was still half its own way at the flip (14.8 m/s).
  0.1 s does both.
- **The walker's fall loop re-asks `reach(walker.catch)` every frame**,
  which reset the slide's arms. It is skipped while sliding.
- **`--gait slide` was already the ladder's**, and its 46 µs was measured
  for the wrong move. The wall slide is `--gait wall-slide`.

## Consequences

**Headless** (`puppet_base`, 60 fps; braced hangs on walls 3, 4.5 and
6.5 m high):
- no joint moves over 1.9 cm the frame it lets go;
- nothing into the wall beyond 0.1 mm;
- no joint over 13.1 m/s about the hips;
- the palms within 6 mm of their place on the face in the air;
- touchdown at 3.21, 4.99 and 6.66 m/s, against letting go's 4.53, 7.06 and
  9.40;
- at 6.5 m letting go is fatal, sliding lands;
- every one stands at the wall's foot.

Hanging free, it does not slide.

**Live**: from a 4.5 m block's top, it dropped down into a hang, then slid
down the face, palms flat on it above the shoulders and toes braced, and
landed.

**Cost**: `anim_bench --gait wall-slide --characters 100`, 38 µs a character
at p50 (letting go 26 µs): the arms' solve each frame.

## Revisit when

- **Sliding from a run up that misses its lip**, or from standing at a top's
  edge: only a braced hang slides.
- **A slide controlled in speed** (hands gripping harder): the brake is one
  constant.

## Related

- [A hang is dropped into, let go of, and caught from a fall](./a-hang-is-dropped-into-let-go-of-and-caught-from-a-fall.md) — prerequisite: the let-go this brakes.
- [A fall facing a wall is held off it](./a-fall-facing-a-wall-is-held-off-it.md) — deeper: the wall room and keep-offs a slide inherits.
- [A near-straight leg is bent toward its kneecap](../ik-and-locomotion/a-near-straight-leg-bends-toward-its-kneecap.md) — same-trap: a two-bone solve's bend plane on a straight limb.
- [A wall is run up off one foot and its lip caught](./a-wall-is-run-up-off-one-foot-and-its-lip-caught.md) — contrast: the wall's other step-8 moves.
