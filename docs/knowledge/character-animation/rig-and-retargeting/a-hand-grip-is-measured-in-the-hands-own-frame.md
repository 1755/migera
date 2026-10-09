---
title: A hand grip is measured in the hand's own frame, and a held hand follows its forearm
description: "HandGrip from the rig's fingers is in the hand bone's own frame; the hand turn reads the rest frame, so it must be turned by the wrist's bind (hand::bound_grips), or every held hand bends ~90° at the wrist. Tests on the forearm-estimated grip cannot see it. Read before writing a mover that holds anything."
type: lesson
status: current
tags:
  - rig
  - ik
  - correctness
  - testing
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/hand.rs
  - src/character/anim/armik.rs
  - src/character/anim/parkour/pole.rs
  - src/character/anim/parkour/holds.rs
  - src/character/anim/parkour/monkey.rs
  - src/character/anim/parkour/hang.rs
  - src/character/anim/ladder.rs
sources:
  - "tests parkour::{monkey::tests::monkey_bars_are_crossed_hand_over_hand, pole::tests::a_pole_is_climbed_up_and_down_and_slid_down, holds::tests::a_wall_of_holds_is_climbed_limb_by_limb_every_way} (hand::wrist_bend)"
  - "user report 2026-10-09: wrists and palms bent the wrong way on the bars, the pole and before"
aliases:
  - bound_grips
  - HandGrip::bound
  - wrist_bend
  - grips' frame
  - wrist bent the wrong way
---

# A hand grip is measured in the hand's own frame, and a held hand follows its forearm

Two separate faults turned held hands the wrong way at the wrist. Both were
invisible to the tests.

## What happened

**The frame.** `hand::grip_of` (and `RelaxedHands::grips`, what the walker
hands a mover) gives a `HandGrip` (`bar`, `palm`, `along`) in the hand
bone's own glTF frame. The hand turn (`armik::frame_turn` with
`turn_hand`) reads it in the hand's rest frame: the bind frame, the
accumulated bind rotation applied. The ladder and the ledge hang turned
the grip by the wrist's bind. The pole, the free climb on holds, and the
monkey bars took it as measured.

On `puppet_base` the two frames are a quarter turn apart: the fingers run
along the hand bone's own `+Y`, and along the forearm once bound. So in
the gallery, where the real fingers' grips are used, every hand on a
pole, a hold or a bar was turned about 90° at the wrist. The independent
measure read 1.64 rad. Every test passed: the movers' tests used the
grip estimated from the forearm (fingers along it, palm down), which is
already in the rest frame and matches the bound real grip within 2°.

**The forearm.** With the frame fixed, a hand turned to an ideal (fingers
straight up over a hold; level round a pole; along a line from the
standing shoulder to a bar) ignored where the arm IK put the forearm:
- an arm reaching in from the side bent a climber's wrist 1.7 rad
  sideways;
- a palm laid flat on the wall, the forearm coming in toward it, bent the
  wrist 1.05 rad back;
- on the pole, 0.71 rad sideways and 0.6 back.

## Why it matters

A grip in the wrong frame is a quarter-turn hand on every hold, and only a
look at the mesh shows it. Each mover had its own `set_grips`: the
conversion lived in two of them, and the trap recorded in the hang's note
did not reach the three written later.

## How to apply

- **Turn every measured grip by its wrist's bind**: `hand::bound_grips`
  (or `HandGrip::bound`). Never copy a `RelaxedHands` grip straight into a
  mover's body.
- **Test held hands with the rig's real grips** (`hand::puppet_grips()`),
  not the estimate. Measure the wrist with `hand::wrist_bend`, straight off
  the hand's world rotation and the grip as measured. Measured through
  `bound_grips`, the check passed with the conversion disabled: the same
  function on both sides.
- **Turn the hand toward the forearm the arm solved to**, within limits,
  and solve the arm again (two passes; the ladder's way):
  - bars: the fingers on along the forearm;
  - holds: toward the forearm in the wall's plane up to 1.4 rad off up (a
    sidepull), tipped into the wall up to 0.9 rad (the heel of the hand off
    the face);
  - pole: turned round the pole toward the forearm's way in, up to 1 rad,
    tilted with its rise up to 0.5 rad.

  Read back where the hands are from the same posed result (`posed`), so
  the "on its hold" checks see the final wrist.
- `wrist_bend` gives the sideways deviation and the signed flexion
  (positive toward the palm). A negative flexion (bent back) beyond a few
  tenths of a radian is what reads as "the wrong way".

## Evidence

With the rig's grips, after the fixes (`puppet_base`, 60 fps):

| Move | Sideways | Flexion |
|---|---|---|
| monkey bars | 0.07 rad | 0 to 0.10 |
| pole climb | 0.20 rad | −0.20 to 0.17 |
| free climb | 0.46 rad | −0.52 to 0.17 |

Without the bind turn, the monkey bars read 1.64 rad. The ledge hang
(already bound) reads 0.46 rad sideways, bent back 0.59 rad (fingers
hooked over a lip, the body braced out); the bar hang −0.15. Live, on the
bars, the pole and the wall the hands continue their forearms.

## Related

- [A ledge is caught near the top of a jump and hung from](../parkour/a-ledge-is-caught-near-the-top-of-a-jump-and-hung-from.md) — same-trap: where the grips' frame was first found (the hands pointed out from the wall).
- [Synthetic-rig tests are blind to retargeting](./synthetic-rig-tests-are-blind-to-retargeting.md) — same-trap: a test fixture that matches the code's assumption cannot catch it.
- [Same function both sides is a vacuous test](../../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: the wrist check measured through the conversion it was testing.
- [Monkey bars are crossed hand over hand](../parkour/monkey-bars-are-crossed-hand-over-hand-the-body-hung-from-the-hands-carrying-it.md) — example: the move where the bent hands were reported.
