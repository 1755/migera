---
title: The puppet_base fixture faces away from the rendered character
description: "gltf_rig::puppet_base() is turned 180° from the rendered character, so a world-axis-authored pose (relaxed_stand, clip imports) measures wrong on it: hands overhead, COM high. Measure shape, COM and balance on puppet_base_as_rendered(). Read before measuring a pose or its balance on the real rig."
type: lesson
status: current
tags:
  - rig
  - retargeting
  - poses
  - verification
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/gltf_rig.rs
  - src/character/anim/poses.rs
  - src/character/anim/anthropometry.rs
  - src/character/anim/balance.rs
  - src/character/anim/phase.rs
  - src/character/anim/locomotion.rs
  - src/character/anim/lookat.rs
  - src/character/anim/ragdoll.rs
sources:
  - "test gltf_rig::tests::the_rendered_rig_matches_the_live_character"
  - "BRP on character_gallery --anim-speed 0, 2026-09-29"
aliases:
  - puppet_base_as_rendered
  - facing correction
  - arms overhead in forward kinematics
---

# The puppet_base fixture faces away from the rendered character

`gltf_rig::puppet_base()` parses `puppet_base.gltf` exactly as exported,
facing +Z. The gallery renders the character turned half round, facing
this crate's −Z. Anything written against the rig's own forward
(`RigGeometry::forward`, `stance::facing_sign`) is the same on both. A pose
authored as **fixed world-axis rotations** is not. That covers
`relaxed_stand`'s arms and spine and anything imported from a clip. Such a
pose means different things on the two rigs. Measure its rendered shape on
**`gltf_rig::puppet_base_as_rendered()`**.

## What happened

Measuring the relaxed stance's posture on `puppet_base()`:

- the hands came out 1.04 m above the hips, above the head, while the live
  character hangs them at its sides;
- the trunk leaned 11° forward and the centre of mass sat 9.7 cm ahead of
  the ankles, a slouch that does not exist on screen.

Over BRP, the live relaxed stance puts `upperarm_l`, `lowerarm_l` and
`hand_l` at +0.456, +0.207 and −0.034 m above the pelvis. The same pose with
every rotation turned 180° about +Y reproduces those heights to 2 mm, and so
does `puppet_base_as_rendered()` (the fixture with its root turned). The
bind pose alone agrees on both rigs to the millimetre (shoulder 0.506
against 0.508). Only a posed rig shows the difference. The
forward-kinematics-versus-retargeting agreement test did not catch it
either, because that test compares rotations on one fixture.

On the rendered rig the stance's real fault was different: the head looked
30.5° down at the floor. The balance was fine, the centre of mass 4.8 cm
ahead of the ankles, as Winter's static stance puts it.

## Why it matters

A rig-facing convention is invisible in every rig-relative test, so the gait,
feet and stance all passed on either fixture. A world-axis-authored pose
measured on the wrong one looks plausible enough (a lean, a slouch) that the
number gets trusted.

## How to apply

- For a question about what the character LOOKS like in a pose (posture,
  centre of mass, hand placement), use `puppet_base_as_rendered()`.
  `the_rendered_rig_matches_the_live_character` pins it to the live numbers.
- **Anything that depends on the centre of mass does too**, however
  rig-relative its maths: balance, pendulum heights, support. The balance
  tests stood `relaxed_stand` on `puppet_base()` until 2026-10-01: arms
  overhead, COM 8 cm high (k 0.104 against 0.095), soles reaching back
  0.18 m and forward 0.11 instead of 0.12 and 0.17. Every catch limit was
  inverted: forward 1.2 m/s "fell" and sideways 1.2 "was caught", the
  reverse of the live character (`the_balance_fixture_stands_as_the_character_is_drawn`).
  The walk's pendulum ratio `COM_OVER_HIPS` was pinned on it too: 1.19
  (a COM 62% up the body) instead of 1.09.
- **So does anything that reads a world-axis rotation's sign or a bone's
  own axis**: the ragdoll's forearm twist stop was fitted to −90.5° for
  the wave, which reads +90.7° on the drawn character, so the waving
  forearm held 3.7° short against the stop live. And the look-at tests
  measured the head bone's own −Z, which is the back of the head on the
  drawn rig; on plain `puppet_base()` it started at −Z and the tests
  passed while measuring the back of the head.
- For gait and foot maths that never reads the arms or the COM (joint
  angles, sole contact), either fixture works.
- **To find every test that cares, swap the fixtures.** Make
  `puppet_base()` return the turned rig for one test run, so both
  helpers return the other one. On 2026-10-01, 17 of 1044 failed: 8 on
  the drawn rig (correct), 5 comparing against the asset or Bevy's
  render of the file (correct on plain), and 4 that were wrong. The
  1027 that pass either way don't depend on the facing.
- If a posed measurement disagrees with a screenshot, check the facing
  before the maths: compare one BRP joint height.

## Related

- [A pose delta names a world axis](./a-pose-delta-names-a-world-axis.md) — prerequisite: why a world-axis delta depends on the rig's orientation.
- [Knee axis positive swings forward](./knee-axis-positive-swings-forward.md) — same-trap: the leg-side version of a facing assumption, fixed with `facing_sign`.
- [Synthetic-rig tests are blind to retargeting](./synthetic-rig-tests-are-blind-to-retargeting.md) — contrast: a different fixture limitation (synthetic translations), not orientation.
- [A clip's world positions carry its rig's bind shape](./a-clips-positions-carry-its-rigs-bind-shape.md) — same-trap: another frame `relaxed_stand`'s data silently assumed (the source rig's bind).
- [A stumble is a capture-point step, then a join](../ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md) — example: its catch limits were measured on this fixture, inverted, until 2026-10-01.
