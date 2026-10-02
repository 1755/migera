---
title: A drawn floor correction is held between frames, not worked out afresh
description: "Floor corrections on the drawn ragdoll (wrist turn, a rising arm or knee clearing the floor) solved afresh each frame leapt: a hand turned 68-163° a frame. Each is now last frame's turn changed as little as clearing needs, eased back at a set rate; arms lift at the shoulder. Read before adding a drawn correction."
type: lesson
status: current
tags:
  - ragdoll
  - correctness
  - verification
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/ragdoll_plugin.rs
  - src/character/anim/ragdoll.rs
sources:
  - "test ragdoll_plugin::tests::a_drawn_hand_turns_no_faster_than_its_body (five falls and rises on puppet_base, every frame)"
  - "live: character_gallery --ragdoll on --fall-at-frame N, BRP hand and forearm rotations, and a temporary per-frame report inside write_simulated_pose"
aliases:
  - hold_clear
  - wrist_lift
  - arm_lift
  - tucks
  - turn_up_clear
  - tuck_foot
  - hand pop
---

# A drawn floor correction is held between frames, not worked out afresh

While falling and rising, `write_simulated_pose` corrects the drawn skeleton
to keep ends out of the floor. A correction solved from scratch each frame
is continuous only if its answer is. Near a threshold or a degenerate axis,
the answer jumps, and so does the limb. Each correction now starts from the
turn it drew last frame and changes it as little as clearing the floor
needs.

## What happened

Live, a hand turned up to 163° between two samples of a fall and rise. The
build before any hand work (`f997487`) did the same. Measured per frame,
headless and live, there were three causes:

| Correction | What leapt | Cause |
|---|---|---|
| Wrist turned up to clear the fingertips (`turn_up_clear`) | 68-70° in a frame, the hand's body turning 15-19° | It snapped on and off as the tip crossed the floor. |
| Its axis, the level line across the hand | 78° in a frame | Over a hand hanging straight down the line is undefined and flipped, so the same lift swung the other way. |
| Rising arm's elbow folded to clear the wrist (`tuck_foot`) | forearm 80-115° in a frame, the wrist 11-37 cm | A nearly straight arm needs ~2 rad of fold, and the answer jumped between none and nearly all of it. |

## Why it matters

A correction that is right on every frame can still be wrong as motion. The
tests checked the end state, so none of them could see these: "no fingertip
under the floor" and "hips within 6 cm of the keys" both held while the hand
popped. The first fixes each traded one artifact for another:

- **Limiting the rate both ways** left fingertips 25 mm under the floor, or
  a hand under it, so the rise's whole-body lift hoisted the hips 72-99 mm.
- **Keeping the axis as an angle plus an axis:** a change of axis carried
  the angle over, and a rising hand turned 134°.
- **Folding the elbow at a limited rate** stopped the pops but hoisted the
  side rise 99 mm.
- **Eight times the wrist's damping** did nothing. Those turns were contact
  impulses, not joint motion.

## How to apply

- **Hold the correction as one rotation** in the bone's own frame (`Ragdoll::wrist_lift`,
  `arm_lift`). Each frame, in `hold_clear`:
  1. Ease it back toward none at a set rate (wrist and shoulder 3 rad/s), as
     far as the point stays clear.
  2. If the point is still under, turn further up about the level line as
     drawn now, just far enough.

  The change per frame is then what the motion itself demands.
- **Move a correction to the joint where it is small.** A straight arm
  clears the floor with a few degrees at the shoulder, not two radians at
  the elbow. Knees still tuck (`tucks`, deepening at once, letting go at
  6 rad/s).
- **Test the motion, not only the end state.** Compare each frame's drawn
  turn with the physics body's: the drawn hand may turn at most 30° a frame
  faster than its body falling, and 15° a frame rising (bodies still).
  Include a plain collapse; the pushed falls alone missed the elbow pop.

## Evidence

- `a_drawn_hand_turns_no_faster_than_its_body` covers a plain collapse plus
  pushes forward, back and on both diagonals. It measures 6-23° faster than
  the body while falling (was 49-59°) and 4-8° a frame rising (was 26-30°).
  With the rates set to 1000 rad/s it fails at 33°.
- Live, 10 plain collapses: no forearm turned over 50° in a frame (before,
  4 of 6). One hand turned 50°, of which 22° was correction and the rest
  the body landing.
- What is left is physical: a hand's body slapping the floor turns up to
  45° a frame, as much about the wrist.
- Fingertips stay out of the floor (lowest 0.000 m live). The existing rise
  and fall clearance tests pass unchanged.

## Related

- [Getting up goes through key poses](./getting-up-is-a-timed-blend-then-a-re-pin.md) — applies: the rise's tucks, lift and wrist turn this holds.
- [A falling body has hinged knees and elbows and solid flesh](./a-falling-body-is-hinged-and-fleshed.md) — context: why drawn toes and fingers need correcting while the bodies dip.
- [A fall test samples one chaotic landing](./a-fall-test-samples-one-chaotic-landing.md) — same-trap: a per-frame bound over several launches, not one landing's outcome.
- [A walking arm swings back, and its hand hangs curled](../ik-and-locomotion/a-walking-arm-swings-back-and-its-hand-hangs-relaxed.md) — context: the fingers' own bends follow the same rule (held, rate-limited).
