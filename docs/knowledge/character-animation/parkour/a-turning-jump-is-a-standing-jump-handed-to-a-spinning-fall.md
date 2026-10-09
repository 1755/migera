---
title: A turning jump is a standing jump handed to a spinning fall
description: "Step 12: a turning jump from a stand is a standing jump straight up (0.4 m), handed 0.1 s into its flight to a fall that turns the body about +Y (spin_round, as a leap back) and lands it facing the new way. Handed over as it left, the knee's step changed 9.9 cm. Read before changing parkour/spin.rs."
type: decision
status: current
tags:
  - locomotion
  - correctness
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/parkour/spin.rs
  - src/character/anim/walker.rs
  - examples/anim_bench.rs
  - examples/character_gallery.rs
sources:
  - "test parkour::spin::tests::a_standing_jump_turns_round_in_the_air"
  - "live: character_gallery --step-seconds 0.0333333 --spin-jump-at 2 (BRP feet and pelvis)"
  - "anim_bench --gait spin-jump --characters 100"
aliases:
  - turning jump
  - 180 jump
  - spin jump
  - Walker::spin_jump
---

# A turning jump is a standing jump handed to a spinning fall

Step 12 of the [parkour steps beyond the first ten](./parkour-moves-beyond-the-first-ten-steps.md):
a 180° turning jump from a stand. There is no data; the turn reuses the
leap back's.

## Decision

- **The jump**: a standing jump straight up, its centre of mass rising
  0.4 m, which gives about 0.55 s of flight.
- **The turn**: 0.1 s into the flight, it is handed to a fall
  (`Falling::from_jump`) onto the floor it left. The fall turns it about
  `+Y` by the turn asked (`Falling::spin_round`, as a leap back turns): over
  at most 0.5 s or 0.8 of the flight, starting 0.1 s in and eased. The
  landing is planned facing the new way.
- **The walker** (`Walker::spin_jump`, the turn in radians): from standing,
  the ask dropped once taken.

## Traps

- **Handed over the instant it left the floor**, the take-off knee was
  still straightening fast and the fall's coasting legs did not match it:
  a knee's step changed 9.9 cm in a frame, against the take-off's own 5.
  0.1 s into the flight the legs have finished pushing.

## Consequences

**Headless** (`puppet_base`; half turns either way, a quarter turn):
- it landed facing the turn within 0.0001 rad, on the floor, 0.12-0.2 m
  from where it left;
- every pose was finite;
- no step changed beyond the take-off's own (4.1-6.4 cm against 10);
- no joint went over 11.7 m/s about the hips.

**Live** (30 fps step): the feet's left-to-right vector reversed through
the flight, and it landed and stood facing back.

**Cost**: `anim_bench --gait spin-jump --characters 100`, 29 µs a
character at p50.

## Revisit when

- **A turning jump forward or from a run**: only straight up from a stand.
- **Arms pulled in to turn**: the arms take the fall's flying shape; a real
  spin tucks them.

## Related

- [Parkour moves beyond the first ten steps](./parkour-moves-beyond-the-first-ten-steps.md) — prerequisite: step 12's design.
- [A hang is leapt from, up, aside or back](./a-hang-is-leapt-from-up-aside-or-back.md) — prerequisite: the fall's turn in the air, from a leap back.
- [A precision jump is a standing jump handed to a fall at its top](./a-precision-jump-is-a-standing-jump-handed-to-a-fall-at-its-top.md) — contrast: the same hand-off timed at the apex, for landing higher.
