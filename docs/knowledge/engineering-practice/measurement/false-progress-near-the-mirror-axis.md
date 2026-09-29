---
title: False progress near the mirror axis
description: An IK error metric dropped from 0.36 m to 0.34 mm while a left-hand target still lifted the right arm, because the probe target sat near the centreline where a mirror is nearly the identity. Choose probe targets asymmetric on every axis. Read before picking a test target or celebrating a big metric drop.
type: lesson
status: current
tags:
  - verification
  - ik
  - testing
updated: 2026-09-26
verified: 2026-09-28
code:
  - src/character/anim/armik.rs
sources:
  - Claude memory false_progress_near_the_mirror_axis (2026-09-26)
aliases:
  - symmetric probe target
  - whole-rig mirror
  - scalar error metric
---

# False progress near the mirror axis

A scalar error metric collapses a 3D failure into one number, and a target
chosen for convenience often sits exactly where the remaining bug is
invisible. **Choose probe targets that are asymmetric on every axis the bug
could mirror or swap**, never on a symmetry plane.

## What happened

- After the pose-space convention was fixed, `hand_l`'s X error went from wrong
  on every axis to **0.34 mm** (from about 0.36 m), measured live over BRP. That
  looked like the arm IK working.
- It was not. Screenshots showed a **left**-hand target lifting the character's
  **right** arm: a whole-rig mirror.
- The probe target sat near the character's centreline, where a mirror about X
  is nearly the identity. The number improved for a reason unrelated to the
  thing it seemed to confirm.
- Had the check stopped at BRP numbers and skipped the mandatory screenshots,
  this would have shipped as "fixed".

## Why it matters

A dramatic improvement feels like confirmation and ends the investigation. It
only confirms the claim if it improved *for the reason claimed*.

## How to apply

- Put probe targets off every symmetry plane of the rig and of the suspected
  bug.
- When a number improves dramatically, confirm the mechanism before calling it
  done.
- Keep the project's Front + Left gizmo screenshot protocol. It is what caught
  this, and it is not a formality.

## Evidence

- Numbers: about 0.36 m → 0.34 mm on `hand_l` X while the wrong arm moved.
- The arm IK frame bugs were fixed in commits 026d9e8 and 3f245be.

## Related
- [Gizmos need --show-real-mesh off](../../character-animation/animation-core/gizmos-need-show-real-mesh-off.md) — applies: the screenshot protocol that caught the mirror.
- [A pose delta names a world axis](../../character-animation/rig-and-retargeting/a-pose-delta-names-a-world-axis.md) — example: the convention fix after which this false progress appeared.
- [A measurement of a broken system](./a-measurement-of-a-broken-system.md) — same-trap: a real measurement that meant something other than it seemed.
- [A/B test on the same input](./ab-test-on-the-same-input.md) — same-trap: a comparison that differs in more than the one thing under test.
- [The symptom is far from the cause in the rig chain](../debugging/symptom-is-far-from-cause-in-the-rig-chain.md) — deeper: the remaining bug was a yaw node further down the chain.
