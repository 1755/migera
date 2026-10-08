---
title: A two-bone IK knee hinge must be square to the line to the target
description: "legik turned the leg about the raw knee axis; when the hip-to-ankle line leaned sideways (slanted seated shins, a wide stance) the ankle missed its target and the foot, aimed from there at its planted toe, turned 9°. Take the axis's part along the line off first. Read before writing or changing a two-bone IK."
type: lesson
status: current
tags:
  - ik
  - correctness
  - testing
updated: 2026-10-09
verified: 2026-10-03
code:
  - src/character/anim/legik.rs
  - src/character/anim/sitting.rs
sources:
  - "test legik::tests::a_target_out_to_the_side_is_reached_without_turning_the_foot (fails with the old hinge: the foot turned 2.9°)"
  - "live: character_gallery --sit chair:upright --chair 1.5,-2.0,90, BRP ankle and toe positions while sitting"
aliases:
  - knee hinge
  - hinge not square
  - foot turns about the toe
---

# A two-bone IK knee hinge must be square to the line to the target

## What happened

- **The symptom:** a character sitting on a chair, shins slanted ~6°
  sideways (`sitting::Seat::across`), had both ankles swing 22–25 mm
  sideways while its toes stayed put. Each foot turned ~9° about its
  planted toe.
- **The poses were right:** the keys and the posture frame by frame kept
  the feet within 3–5 mm (tests). The fault was downstream, in the leg IK.
- **The cause:** `legik::solve_leg_grounded` builds the knee by turning
  the hip-to-ankle line about a hinge.
  - It used the configured knee axis as the hinge, unchanged. Its comment
    said the hinge was taken square to the line, but the code didn't.
  - Turned about an axis with a part along the line, the two segments
    don't bring the ankle back onto the line, so the ankle misses its
    target.
  - The foot is then aimed from that wrong ankle at the planted toe, and
    turns.

## The fix

The axis's part along the line is taken off before it is used:
`hinge = (axis − d·(d·axis)).normalize()`.

- With the leg in a sagittal plane (most of the walk) nothing changes: the
  axis is already square to the line.
- All 1118 tests passed unchanged.
- Live, the seated feet moved 0 mm, sitting and rising.

## Why nothing caught it

- `a_reachable_toe_target_is_reached` allowed 2 cm, more than this error.
- Every other leg IK test reached targets in the leg's own plane.
- The new test reaches sideways-and-up targets on the real rig, toe within
  1 mm and the foot turned < 0.8°. It fails with the old hinge (2.9°).
  - On the synthetic rig the same targets made the IK's deviation guard
    refuse the solve (its shin is a 0.07 m stub), which hid the case
    entirely.

## Related

- [Two-bone IK pivots at the upper joint, not the root](./two-bone-ik-pivots-at-upper-not-root.md) — same-class: another two-bone IK geometry error that a loose reach test let through.
- [A near-straight leg is bent toward its kneecap](./a-near-straight-leg-bends-toward-its-kneecap.md) — same-class: `stance::keep_ankle`'s raw hinge, noise on a straight leg.
- [Synthetic rig's leg segments are shifted a joint](../rig-and-retargeting/synthetic-rig-leg-segments-are-shifted-a-joint.md) — why the test runs on the real rig.
- [Walking to a chair turns on a circle and paces its stop](./walking-to-a-chair-turns-on-a-circle-and-paces-its-stop.md) — context: the sideways seat shift that exposed it.
