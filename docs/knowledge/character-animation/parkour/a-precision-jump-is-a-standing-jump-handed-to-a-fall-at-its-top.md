---
title: A precision jump is a standing jump handed to a fall at its top
description: "Step 11, fourth part: a jump onto a small top is a standing jump planned so its COM comes down over it (Jump::onto), handed at its apex to a fall that lands it there; stood, it balances, arms out. Handed over at take-off the fall went NaN. Read before changing parkour/precision.rs."
type: decision
status: current
tags:
  - locomotion
  - correctness
  - biomechanics
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/precision.rs
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::precision::tests::a_post_is_jumped_onto_and_stood_on"
  - "live: character_gallery --step-seconds 0.0333333 --post 0,-1.6,0.2 --onto-at 3,0,0.2,-1.6, Xvfb, gizmos on/mesh off Left and Front, mesh on; BRP pelvis and feet"
  - "anim_bench --gait onto --characters 100"
aliases:
  - precision jump
  - precision landing
  - Jump::onto
  - Walker::onto
  - perched
---

# A precision jump is a standing jump handed to a fall at its top

Step 11 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md),
fourth part: a precision landing on a small target, a post's top or a
beam's end. There is no precision-jump data; the standing jump's own
timings and limits stand.

## Decision

**The plan** (`Jump::onto(ahead, rise)`) is a standing jump
(`Jump::plan`), its height and distance found by correction:
- the centre of mass's ballistic flight from the take-off comes down to
  its own touchdown height plus the rise;
- there, the feet are as far ahead of it as at its own touchdown, over the
  top's middle, within 1 cm;
- it tops out at least 0.15 m over that;
- out of reach (the height past 0.6 m, or a distance the take-off speed
  cap of 3.43 m/s cannot give), it returns `None`.

**The hand-off** (`precision::hands_over`, `fall_onto`): once in the air
and past its apex, the jump becomes a `Falling::from_jump` onto the top's
height. `land_at` re-aims it so the hips come to rest over the top's
middle, taking up the plan's few millimetres of miss.

**The walker** (`Walker::onto`, a top's middle among its ledges): standing,
it turns on the spot to face the top, then jumps. Landed, it balances with
the beam's arms out (`state.perched`) until it steps more than 0.35 m off
it.

## Alternatives considered

- **A standing jump landing higher**: the jump plans its COM back to its
  take-off height and has no landing height. The fall already lands on
  any height, with any velocity.
- **The fall's `land_on`** (the first top under the feet along the
  flight): it takes only tops more than a step (0.3 m) over its ground. A
  0.2 m post was ignored and the body fell through it.
- **A jump's own flight to touchdown, then a landing on the top**: the
  jump's landing shape and timing are for its take-off height.

## Traps

- **Handed to the fall at take-off**, the fall lands higher than it
  leaves. Its hips start under the height they touch down at, its
  landing's depth is 0 and its braking `2vT/depth` infinite, and the legs
  went NaN at a 30 fps step (at 60 fps too, but NaN fails no `<` and the
  checks missed it). Handed over at the apex, the fall always drops onto
  the top. A fall landing at the height it left (a pole's slide let go
  0.15 m up) has the same 0 depth and infinite braking, and survives it.
  Clamping that braking to stay finite dropped the slide's hips 29 cm in a
  frame, so the fall is left as it was.
- **A check that compares**, `into > x`, passes NaN. The test now checks
  every pose is finite.
- **The perch given up in the air**: "walked off the top" was judged
  while falling onto it, the root still 0.8 m short, so the balance was
  dropped before it landed. It is judged only off holds.

## Consequences

**Headless** (`puppet_base`; posts 0.4 m square: 1, 1.4 and 1.8 m ahead
level, 0.8-1.6 m at 0.2 m up, 0.7 m at 0.4 m up):
- every jump planned;
- both feet landed on the top, heel to ball;
- the centre of mass came to rest 2.6 cm from the middle;
- nothing went into the post (the toe joints at most 2 cm, as the foot IK
  keeps them out live);
- every pose was finite at 30 fps;
- the hand-off added no change of step beyond the take-off's own (2.7-4.9
  cm, the jump's own 5.8-14.2);
- 1.8 m ahead onto a 0.4 m post is out of reach.

**Live** (30 fps step): it turned, crouched, jumped with its arms swung
up, and landed on a 0.2 m post 1.6 m ahead (ankles at 0.286 m). It stood
there, its arms out, and swayed a few millimetres over 6 s.

**Cost**: `anim_bench --gait onto --characters 100`, 44 µs a character at
p50 over the jump and the fall.

## Revisit when

- **Landing on the balls of the feet**: the fall's landing is a flat
  squat; the design asked for the balls, the heels up.
- **Higher or farther targets**: a standing jump rises 0.6 m at most and
  leaves at 3.43 m/s. A running precision jump would need the run's leap
  handed over the same way.
- **Step 16's perch**: landing into a crouch on the top instead of
  standing.

## Related

- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 11's design.
- [A jump is planned as its centre of mass's path](../ik-and-locomotion/a-jump-is-planned-as-its-centre-of-mass-path.md) — prerequisite: the standing jump `Jump::onto` plans.
- [A hang is dropped into, let go of, and caught from a fall](./a-hang-is-dropped-into-let-go-of-and-caught-from-a-fall.md) — prerequisite: a jump going on as a fall, the hand-off this times at the apex.
- [A beam is walked with the feet near its line and the arms out](./a-beam-is-walked-with-the-feet-near-its-line-and-the-arms-out.md) — context: the balance it stands in.
- [A springboard is a running leap whose take-off foot rides the board down](./a-springboard-is-a-running-leap-whose-foot-rides-the-board-down.md) — same-trap: the same NaN met again by a fall handed over rising toward a higher top, and why fixing it in the fall moved other jumps.
