---
title: Bind-pose zero leg slack is normal
description: "A T-pose bind puts the hip exactly one leg length above the sole, a straight-knee IK singularity; puppet_base.gltf has it too (0.9712/0.9712). Fix it with an authored bent-knee stance, never by editing bone lengths. Also: LeftUpLeg's entity sits at the knee. Read before foot IK or stance work."
type: lesson
status: current
tags:
  - ik
  - rig
  - poses
  - locomotion
updated: 2026-09-24
code:
  - src/character/anim/stance.rs
  - src/character/anim/legik.rs
aliases:
  - IK singularity
  - bent-knee stance
  - avoid the dinosaur
---

# Bind-pose zero leg slack is normal

A T-pose bind puts the hip socket exactly femur + shin + ankle above the
sole. The knee sits at 180° with **zero horizontal reach budget**, a
singularity where foot IK has no solution space and no bend direction. That
is simply what a bind pose is. Fix it with authored stance data, not by
changing rig geometry.

## What happened

It looked like a defect in migera's synthetic `t_pose_offset` table (0.940
reach against 0.940 hip height). BRP-measuring the real `puppet_base.gltf`
showed the same property (0.9712 / 0.9712).

So the fix is **authored data**. `anim::stance` composes a bent-knee stance
onto a pose. It works on any rig because rotations are rig-independent. It
keeps `t_pose_offset` as the clean straight-chain reference the retargeting
maths depends on, and it needs no cross-rig re-verification. Editing bone
lengths would have changed nothing visible, because retargeting composes
deltas onto each rig's own bind pose.

## Geometry worth remembering

- The hip/knee/ankle split (`+f/2`, `-f`, `+f/2`) keeps the shin vertical and
  the foot flat. It folds each segment by `f/2`, so the leg shortens by
  `L·(1 - cos(f/2))`, half of a bare knee bend.
- Reach budget grows with the sine of the fold, crouch with its cosine. 9°
  buys about 0.055 m of budget for about 3 mm of height. 0.15 m would need
  more than 20° and reads as a squat. Prefer a little foot sliding ("avoid
  the dinosaur").

## The bone-naming trap

Each bone entity sits where its bone *starts*. So the `LeftUpLeg` entity is
at the KNEE, and the hip socket is `Hips` itself. Two measurement helpers got
this wrong in different ways before it was caught.

## How to apply

- Treat zero slack at bind as expected. Solve it in stance data.
- Never change bone lengths to buy IK room.
- Since 4c7fbc9, the standing knee flex lives in `stance_on_rig`, not in
  `relaxed_stand.pose.ron`, because a stored rotation cannot know which rig
  (and facing) it will drive.

## Related

- [Rig authored at critical extension](./rig-authored-at-critical-extension.md) — deeper: the resulting 0.0108 m permanent shortfall and the deadband it forces.
- [Foot IK feedback loops](./foot-ik-feedback-loops.md) — applies: the foot IK that needs this stance.
- [Synthetic rig's leg segments are shifted a joint](../rig-and-retargeting/synthetic-rig-leg-segments-are-shifted-a-joint.md) — same-trap: bone names versus where the length really is.
- [Knee axis positive swings forward](../rig-and-retargeting/knee-axis-positive-swings-forward.md) — applies: why the stance's knee flex must be applied per rig facing.
