---
title: Gizmos need --show-real-mesh off
description: "character_gallery's gizmos are depth-tested, so the skinned mesh hides the whole joint chain; --gizmos on alone shows almost nothing and can pass for a verified skeleton view. Read before taking any pose-verification screenshot."
type: lesson
status: current
tags:
  - verification
  - debugging
  - poses
  - character-animation
updated: 2026-09-25
code:
  - examples/character_gallery.rs
aliases:
  - skeleton overlay
  - white-line skeleton
  - gizmo overlay invisible
---

# Gizmos need --show-real-mesh off

In `examples/character_gallery.rs` gizmos are **depth-tested**, so the
skinned mesh hides the joint chain that runs inside it. Always pair
`--gizmos on` with `--show-real-mesh off` when verifying a pose.

## What happened

With `--gizmos on` alone, only the few markers that poke past the silhouette
render: the shoulder rest markers and the foot axes. The white joint-chain
lines are completely invisible.

## Why it matters

It looks like "the gizmo overlay is broken". Worse, it can be mistaken for a
*verified* skeleton view while it shows almost nothing. That defeats the
project rule that gizmos are the primary verification view: they come
straight from the bones' real `GlobalTransform`s, independent of skinning,
occlusion and foreshortening.

## How to apply

Verify poses with:

```
--gizmos on --show-real-mesh off --camera-preset front|left
```

Check both Front and Left. Add the mesh back only for the final appearance
pass. This rule is also written into the root `AGENTS.md` pose-verification
rules (rule 3).

## Related

- [Verify, don't assert from memory](../../engineering-practice/debugging/verify-dont-assert-from-memory.md) — same-trap: a check that seems done but was never actually looked at.
- [Replicate the real frame loop in a unit test](../../engineering-practice/testing/replicate-the-real-frame-loop-in-a-unit-test.md) — contrast: the cheaper headless check to run before any screenshot.
- [Full-strength read-back hides the physics](../ragdoll-and-physics/full-strength-readback-hides-the-physics.md) — same-trap: a screen that cannot show the defect being checked for.
- [Anim studio is complete](./anim-studio-is-complete.md) — applies: studio output is verified with this view.
