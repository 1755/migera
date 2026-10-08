---
title: A near-straight leg is bent toward its kneecap, not about its own hinge
description: "keep_ankle bent the knee about thigh × shin; on a leg within ~6° of straight that hinge is noise and flipped sign between frames, and a fall's knee swung 24 cm in one. Below sin 0.1 it now bends toward the kneecap, eased into its own hinge. Read before changing keep_ankle, place_ankle or any two-bone IK."
type: lesson
status: current
tags:
  - ik
  - correctness
  - numerics
  - testing
updated: 2026-10-09
verified: 2026-10-09
code:
  - src/character/anim/stance.rs
  - src/character/anim/parkour/fall.rs
sources:
  - "test stance::tests::a_nearly_straight_leg_bends_its_knee_forward_whichever_way_it_was_off"
  - "test parkour::wall::tests::it_kicks_off_a_wall_and_catches_a_lip_round_the_corner"
aliases:
  - knee flip
  - straight leg hinge
  - knee swings round
  - STRAIGHT_KNEE
---

# A near-straight leg is bent toward its kneecap, not about its own hinge

A two-bone IK that bends the knee about its own hinge (thigh × shin) has no
hinge when the leg comes in straight. Within a few degrees of straight, the
cross product is a tiny vector whose direction is noise, and it can flip sign
from one frame to the next. The knee then bends the other way round the
socket-ankle line. `stance::keep_ankle` (behind `place_ankle`) now bends a
leg whose bend has a sine under `STRAIGHT_KNEE` (0.1, about 6°) toward its
kneecap. The kneecap is the rest pose's forward, turned by how far the thigh
has turned from rest. Through that range it eases into the leg's own hinge.

## What happened

A wall kick's fall coasted the leg that had pushed off the wall. That leg
was extending fast as it left, and its coasted rotation was blended toward
the standing leg. Before the IK, it arrived dead straight:
- the leg measured 0.889 m against femur plus shin of 0.889;
- the hinge was 0.001-0.01 long against femur × shin of 0.197.

Between two frames the hinge went from (0.0029, -0.0002, 0.0040) to
(-0.0013, 0.0000, -0.0014). The knee swung 24 cm round the leg's line in one
frame, 19.6 m/s about the hips. The same flip happened at 13-15 m/s in other
kicks.

Two plausible fixes did not touch it:
- **Limiting how far the fall coasts a knee** (no straighter than it left or
  than standing): the spikes stayed.
- **Looking for an antipodal aim** in the IK's `from_rotation_arc`: there
  was none (the aim was 0.3-0.8 rad).

Printing the hinge going into `keep_ankle` found it.

## Why it matters

Any two-bone solve that reads its bend plane off the current pose inherits
the pose's degeneracies. This repo's legs come straight often:
- the bind pose is a straight-knee singularity;
- a coasted or blended leg can pass through straight;
- a stance leg near full reach is almost straight.

A test that starts from a bent knee never sees it. A test that perturbs a
straight leg by a hair can also miss it. On `puppet_base` the bind knee is
already bent about 0.11 rad (2.5 cm ahead of its line), so ±1e-3 rad never
crossed straight. That first version of the test passed with the fix
disabled.

## How to apply

- **Never trust a hinge from a near-straight limb.** Below a threshold on
  |thigh × shin| / (|thigh||shin|), use the anatomical bend direction:
  - the knee: the kneecap, the rest forward turned by the thigh's turn from
    rest;
  - the elbow: its pole.

  Ease from it into the limb's own hinge as it bends, so the switch is
  continuous.
- **Measure the bend signed against the natural side.** The bend wanted
  minus the bend now (+ toward the kneecap) is then one rotation, whichever
  side of straight the leg came in on.
- **Test both sides of straight, found, not assumed.** Find straight by
  bisection on the knee's side of its socket-ankle line. Then start a hair
  either side of it and assert both inputs bend the same way. Also assert
  the precondition, that the two inputs start bent opposite ways.
- **Prove the test fails with the fix off** (`STRAIGHT_KNEE = 0`). The
  regression test bends the knee 23 cm backward that way.

## Evidence

- `stance::tests::a_nearly_straight_leg_bends_its_knee_forward_whichever_way_it_was_off`:
  - straddles straight at ±0.02 rad on `puppet_base_as_rendered`;
  - the knee bends forward > 0.1 m every time, the three knees within
    1.5 cm of each other;
  - with the fix disabled, one bends 0.23 m backward.
- The kick sweep (89 kicks): fastest joint 13.4 m/s about the hips, from
  19.6 before.
- No other test changed (1262 pass); the vault, mantle and run-up costs
  were unchanged within noise.

## Related

- [A two-bone IK knee hinge must be square to the line to the target](./a-knee-hinge-must-be-square-to-the-line-to-the-target.md) — same-class: another raw knee hinge gone wrong (`legik`'s, leaning sideways).
- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — deeper: why legs come straight in this repo at all.
- [An antipodal guard must still land on the target](./an-antipodal-guard-must-still-land-on-the-target.md) — same-trap: a degenerate direction handled by a guard that the test never really exercised.
- [KNEE_AXIS positive swings forward](../rig-and-retargeting/knee-axis-positive-swings-forward.md) — contrast: why the kneecap is taken from the rig's forward, not a fixed axis sign.
- [A wall is kicked off toward a lip round a corner](../parkour/a-wall-is-kicked-off-toward-a-lip-round-a-corner.md) — example: the move that found it.
