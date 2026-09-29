---
title: The symptom is far from the cause in the rig chain
description: In the animation stack a visible symptom routinely sits several links from its cause (toes-up was a Hips offset in the wrong frame, a missed hand was a yaw node), because downstream code reacts correctly to bad input. Measure each link instead of guessing the culprit. Read when a pose looks wrong.
type: lesson
status: current
tags:
  - debugging
  - rig
  - retargeting
  - ik
updated: 2026-09-27
verified: 2026-09-28
code:
  - src/character/anim/rig.rs
  - src/character/anim/gltf_rig.rs
  - src/character/anim/retarget.rs
sources:
  - Claude memory symptom_is_far_from_cause_in_the_rig_chain (2026-09-27)
  - commit 026d9e8
  - commit 3f245be
  - commit 522c73e
aliases:
  - toes pointing up
  - hand misses IK target
  - rig chain debugging
---

# The symptom is far from the cause in the rig chain

In migera's animation stack the visible symptom is routinely several links
away from its cause. The chain is long: authored delta → FK → retarget →
loader → Bevy transform propagation. An error anywhere renders as a
plausible-looking pose somewhere else. **Measure one link at a time** instead
of reasoning about which link is most likely.

## What happened

Stacked frame bugs, fixed in commits 026d9e8, 3f245be and 522c73e, each
looked unrelated to their cause:
- **Toes pointing at the sky.** The substitute `Hips` offset was in the
  synthetic Y-up frame while the rig's root correction is Z-up. That laid the
  character on its back *inside the solver*. Leg IK then reacted correctly,
  rotating each toe 113° to rescue a tip it believed was a metre underground.
- **A hand missing its target.** A 180° yaw correction node sat *below* the
  character entity the plugin read. The entity reported identity while every
  live bone carried the yaw.
- **Toes still slightly up.** `from_skeleton` can only *estimate* the toe tip
  (about 27° off), where `from_gltf` measures it. The toe-lift rescued a tip
  that was never penetrating.

The glTF loader was the obvious suspect twice and innocent twice. That was
settled by dumping its captured binds against the file's own parse and finding
them bit-for-bit identical.

## Why it matters

Downstream code reacting *correctly* to bad input is what makes the symptom
misleading. The link that shows the symptom is usually working as designed.

## How to apply

- Compare adjacent links directly: FK vs renderer, file vs loader, estimate vs
  measured.
- Do not rank suspects by plausibility. Dump both sides of one link, compare,
  move to the next.
- Watch for frame mismatches (Y-up vs Z-up, a yaw node above or below the
  entity you read). Three of these bugs were frame bugs.

## Evidence

- Commits 026d9e8, 3f245be, 522c73e. Numbers: 113° toe rotation, about 27°
  toe-tip estimate error.

## Related
- [A pose delta names a world axis](../../character-animation/rig-and-retargeting/a-pose-delta-names-a-world-axis.md) — example: the pose-space convention fix in this chain.
- [Bisect your own code before grepping dependency source](./bisect-before-grepping-dependency-source.md) — same-trap: bounded, link-by-link search beats guessing.
- [False progress near the mirror axis](../measurement/false-progress-near-the-mirror-axis.md) — example: a metric that improved while a later link was still wrong.
- [Foot IK feedback loops](../../character-animation/ik-and-locomotion/foot-ik-feedback-loops.md) — example: the toe joint is not the contact point.
- [DDGI probe-grid bounds wall-embedding leak](../../hybrid-architecture/gi-and-lighting/ddgi-probe-grid-bounds-wall-embedding-leak.md) — same-trap: in the renderer, the shading math was correct and the bug was in the inputs fed to it.
