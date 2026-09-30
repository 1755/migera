---
title: A foot may lift only when the other can hold the body
description: "In balance.rs, lifting a foot while the COM is beyond the valid lean of the other makes the validity bound snap the COM into range and discard its velocity: a 130 mm jump, 1.05 m/s lost. Every lift (join or recovery step) is gated by Balance::holds. Read before changing when a stumbling foot lifts."
type: lesson
status: current
tags:
  - balance
  - locomotion
  - correctness
  - testing
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/balance.rs
sources:
  - "tests balance::tests::a_hard_side_catch_joins_early_instead_of_lunging, a_stumble_steps_cleanly_on_the_real_rig, a_catch_does_not_depend_on_frame_times, a_push_past_a_catchable_step_falls"
  - "probe balance::tests::probe_catch_table (ignored)"
aliases:
  - Balance::holds
  - early join
  - weight transfer at touchdown
  - validity bound snap
---

# A foot may lift only when the other can hold the body

`balance::Balance` is Winter's inverted pendulum, valid only within
`Support::valid` (8° of lean) of the feet bearing weight. Beyond that the
tick clamps the COM back to the bound and zeroes its velocity. Lifting a
foot shrinks the support to the other foot at once. If the COM is then
further than `valid` from that foot, the clamp does not model a fall: it
teleports the COM and deletes its momentum. So **no foot may lift until
the other alone holds the COM** (`Balance::holds`). That applies to a join
and to every recovery step after the first.

## What happened

Making the trailing foot join early after a long step (see
[the stumble note](./a-stumble-is-a-capture-point-step-then-a-join.md))
hit the same fault three ways:

1. **The join.** With the weight moving onto a recovery step as it lands,
   the capture point was already inside the stepped foot at touchdown. The
   trailing foot lifted with the COM 0.26 m short of the stepped foot:
   the COM jumped 130 mm in a frame (pelvis jolt 151 mm).
2. **A second recovery step.** Under uneven frame times a crossover
   landed with the capture point just outside, and the next step was
   planned at once. It lifted the foot the COM leaned on, 0.22 m from the
   remaining one. The clamp discarded 1.05 m/s sideways while recording
   only 0.006 as `lost`, and the next step, planned for the old momentum,
   overshot and stepped back inward.
3. **The wrong foot.** After a crossover and a side step the join picked
   "the foot displaced furthest" to stay, the crossover foot, and lifted
   the side-stepped foot the weight was on: 0.39 m/s lost, a fall. The
   foot that stays is the one the weight was transferred onto.

## Why it matters

The clamp is a safety net for the model's validity, not a physical event.
It fires silently, and `lost` counts only velocity pointing away from the
feet, so a lift that deletes momentum pointing toward them does not show
there. The symptoms were a pelvis jump, an odd inward step, or a catch that
depended on frame timing.

## How to apply

- Gate every lift on `holds(support, stance_leg)`. When a recovery step
  is blocked, the body goes on moving over the stance foot with both feet
  down and steps a tick or so later.
- A join also needs the capture point not on the trailing foot's side of
  the stepped one. Inside it or past it (another step follows) are both
  fine. Requiring "inside, with a margin" made a 5 mm stance asymmetry
  delay the right side's join 0.2 s and sink it 78 mm deeper.
- Any landing that sets the feet side by side ends the stumble
  (`travelled`), a recovery step too: otherwise the weight stays
  transferred with no join to take, and the body never settles.
- To test a lift rule, check the pelvis jolt and `lost` both, under
  uneven frames too. A jolt catches the snap; `lost` alone does not.

## Evidence

2026-10-01. Each rule, disabled on its own, fails a test:

- no transfer at touchdown: `a_hard_side_catch_joins_early_instead_of_lunging`;
- no join gate: four tests, the 130.9 mm jolt among them;
- no recovery-step gate: `a_catch_does_not_depend_on_frame_times`;
- inside-only join: the left/right symmetry assertion;
- no feet-together end: back 1.4 m/s "not settled after 8 s";
- join foot by distance: the frame-times test (a fall).

## Related

- [A stumble is a capture-point step, then a join](./a-stumble-is-a-capture-point-step-then-a-join.md) — prerequisite: the stepping these gates sit in.
- [Push recovery is Winter's pendulum](./push-recovery-is-winters-pendulum.md) — prerequisite: the validity bound and COP law.
- [A fall hands the body to physics](../ragdoll-and-physics/a-fall-hands-the-body-to-physics.md) — contrast: when momentum really is lost, the forecast says fall.
