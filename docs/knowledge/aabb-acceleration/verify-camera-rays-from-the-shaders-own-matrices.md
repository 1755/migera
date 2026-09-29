---
title: Verify camera rays from the shader's own matrices, never a hand-rolled guess
description: A ground-truth ray check built from a guessed camera (look-at, FOV, aspect) compares the wrong ray to the wrong pixel and once reversed a real diagnosis; run the check inside the shader on its own ro/rd and read it back as marker colors. Read before building any external ray/pixel ground-truth check.
type: lesson
status: current
tags:
  - debugging
  - verification
  - ray-tracing
  - hybrid-renderer
updated: 2026-08-30
code:
  - assets/shaders/hybrid_trace.wgsl
sources:
  - commit 370d44b
aliases:
  - external raycast script
  - debug output color
  - camera basis mismatch
---

# Verify camera rays from the shader's own matrices, never a hand-rolled guess

When a screen-space artifact needs ground truth ("which object *should* win this
pixel"), reconstruct nothing outside the renderer. Read the exact `ro`/`rd` the shader
traces and run the analytic check inside the shader, then diff against its own
`hit.t`/`hit.obj`.

## What happened

While debugging a sphere resting flush on a ground plate that rendered with a flat
"cut" chord in its silhouette (commit 370d44b, 2026-08-30, the legacy hybrid renderer),
a small standalone script reconstructed the camera ray from a guessed look-at target,
FOV and aspect, and intersected it analytically. The guessed basis was subtly wrong.
It produced a clean diagonal boundary that *looked* like proof of correct occlusion,
and reversed the diagnosis for a full debugging pass before it was caught. The real
cause was a per-object march clipped to a cross-object merged interval list (see
[the flat-cut artifact](../sdf-3d/rendering/raymarching-artifacts-and-fixes.md#flat-cut-or-chord-bitten-out-of-a-round-silhouette-near-where-two-objects-touch)).

## Why it matters

A wrong camera basis (look-at target, FOV convention, aspect handling) yields a
confident, plausible-looking "proof" that compares the wrong ray to the wrong pixel.
The failure is silent: nothing about the output says the ray is wrong.

## How to apply

1. Take the ray from the real ray-generation code (in `src/hybrid` today:
   `view.world_from_clip * near_clip` in `hybrid_trace.wgsl`).
2. Run the analytic ground-truth test in the shader with those exact values.
3. Read the comparison back through a screenshot by encoding it as a discrete color,
   e.g. `select(vec3(50,0,0), vec3(0,50,0), condition)`. Tonemapping and sRGB bend
   continuous values non-linearly, so use maximally separated marker colors or repeated
   boolean threshold splits, not a continuous gradient.

## Evidence

- Commit 370d44b ("Fix flat silhouette cut where objects touch"), which also recorded
  this gotcha in the aabb-acceleration INDEX before it was split into this note.

## Related
- [Common raymarching artifacts and their causes](../sdf-3d/rendering/raymarching-artifacts-and-fixes.md) — example: the flat-cut symptom this trap misdiagnosed.
- [Ray–AABB intersection: the slab method](./ray-aabb-slab-test.md) — prerequisite: the interval test whose per-object use was the real bug.
- [Verify, don't assert from memory](../engineering-practice/debugging/verify-dont-assert-from-memory.md) — same-trap: a confident-looking check that was never checked against the real system.
- [Prefer BRP over prints for live ECS state](../engineering-practice/debugging/prefer-brp-over-prints-for-live-ecs-state.md) — same-trap: read the live system's own state instead of reconstructing it.
