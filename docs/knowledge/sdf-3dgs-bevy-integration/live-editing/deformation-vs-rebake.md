---
title: When to transform already-baked splats vs. re-bake from the SDF
description: Gives a rule for SDF-baked splats (rigid move → transform splats with Σ' = RΣRᵀ; shape change or smooth-union neighbour change → re-bake) and surveys non-rigid middle ground (PhysGaussian MPM, VR-GS XPBD, SC-GS control points). Archived with the splat renderer. Read before animating any baked representation.
type: design
status: archived
tags:
  - sdf
  - 3dgs
  - physics
  - baking
  - state-of-the-art
updated: 2026-08-15
sources:
  - PhysGaussian, CVPR 2024, arXiv 2311.12198
  - commit 684490c (splat renderer removed)
aliases:
  - deformation
  - re-bake
  - MPM
  - XPBD
  - SC-GS
  - AnimatedRing
---

# When to transform already-baked splats vs. re-bake from the SDF

> **Archived (2026-09-28):** the `AnimatedRing`/`animate_rings` mechanism and baked `SplatCloud` this builds on were deleted in commit 22d3b91 (2026-08-16), and splats were dropped entirely in 684490c (2026-09-06); nothing past the rigid-transform step was built.

## The decision this project already made once, informally

This project's animated rings (`AnimatedRing`/`animate_rings` in `main.rs`, spinning
each ring's `Transform` locally around Y between periodic re-bakes — see
[streaming-and-invalidation](../baking-pipeline/streaming-and-invalidation.md)'s
`REANIMATE_INTERVAL_SECS` mechanism) already draws, in effect, the exact distinction
this document makes explicit: a **rigid transform** (rotation with no shape change)
gets applied to the `Transform` component and only surfaces in the render via the *next
periodic re-bake*, not a continuous per-frame re-sample. That works because a rigid
transform doesn't change any splat's position **relative to its owning primitive's
local frame** — the whole baked splat cloud for that primitive is still exactly correct,
just needs its parent transform reapplied. This document generalizes that observation
into an explicit rule and surveys the realtime-Gaussian-deformation literature for the
cases where a plain re-applied `Transform` isn't enough.

## The rule

| What changed | What's needed | Why |
|---|---|---|
| A primitive's `Transform` (position/rotation), shape unchanged | Reapply the transform to already-baked splats — **no re-bake** | The SDF surface *relative to the primitive's own local frame* is identical; only its placement in world space moved. Splat position, orientation, and covariance all transform rigidly along with it (`p' = R·p + t`, `Σ' = R·Σ·Rᵀ` — see [gaussian-primitive-parameters](../../3dgs/fundamentals/gaussian-primitive-parameters.md) for the covariance decomposition this rotates). |
| A primitive's `Shape` parameters (radius, half-extents, blend radius) | **Re-bake** the affected region | The surface itself moved relative to the primitive's own frame — every sample point, normal, and curvature-derived scale for that primitive is now stale. |
| Non-rigid deformation (squash, bend, fracture, fluid) | Neither a plain transform nor a from-scratch re-bake alone is sufficient/efficient — see the physics-driven techniques below | Splat *positions* need to move non-rigidly, but this is usually cheaper as a direct update to already-baked splat data (driven by a deformation field or simulation) than as a full re-sample of a changed SDF, if the topology isn't changing. |
| A smooth-union neighbor's shape/position changed (even if this primitive didn't move) | **Re-bake** the region within this primitive's own blend radius of the change | Smooth-union blending means a primitive's *rendered* surface depends on nearby primitives too, not just its own SDF term — see [combination-operators](../../sdf-3d/primitives-and-operators/combination-operators.md). |

The load-bearing distinction is: **does this change alter the SDF value at points near
the surface, evaluated in the primitive's own local coordinate frame?** A rigid
transform, by construction, does not (that's what "rigid" means — an isometry, exactly
this project's `Isometry` type in `sdf/scene.rs`, translation + rotation with no scale
or shear). Everything else does, to varying degrees, and needs either a re-bake or a
non-rigid update to the baked splat data itself.

## Realtime Gaussian deformation techniques (the non-rigid middle ground)

For cases where re-baking from the SDF on every frame would be too expensive but a
plain rigid transform isn't expressive enough (squashing, bending, physically simulated
motion), the Gaussian-splatting literature has substantial prior art for updating
already-baked Gaussian parameters directly, without going back through the SDF at all.
None of this is SDF-specific — it operates purely on Gaussian primitives — but it is
directly applicable to this project's baked `SplatCloud` as a way to animate content
between re-bakes more expressively than a rigid `Transform`:

- **PhysGaussian** (CVPR 2024,
  [project page](https://xpandora.github.io/PhysGaussian/),
  [arXiv 2311.12198](https://arxiv.org/abs/2311.12198)) runs a Material Point Method
  (MPM) simulation where the Gaussian kernels *are* the simulation particles directly —
  "what you see is what you simulate," no auxiliary mesh or cage needed. Each
  particle's covariance updates from its accumulated deformation gradient `F_p`:
  `Σ(t) = F_p(t)·A·F_p(t)ᵀ` (`A` = initial material-space covariance), with a
  cheaper incremental form for stability:
  `Σⁿ⁺¹ = Σⁿ + Δt·(∇v·Σⁿ + Σⁿ·∇vᵀ)`. Demonstrated across elastic, plastic
  (metal/sand/snow-like), and viscoplastic (jam/cake-like) materials at 25-36 FPS on an
  RTX 3090. This is the most directly reusable technique if this project wants
  physically-simulated deformable props (not just rigid motion) — the deformation
  gradient `F_p` this needs per particle is exactly the kind of per-primitive local
  frame data this project's `Isometry`-based leaf transforms already track, generalized
  from rigid (`R, t`) to full affine (`F`).
- **VR-GS** (SIGGRAPH 2024,
  [project page](https://yingjiang96.github.io/VR-GS/),
  [arXiv 2401.16663](https://arxiv.org/abs/2401.16663)) is architecturally the closest
  match to "an SDF-authored primitive deforms and its baked splats follow": build a
  bounding mesh around a Gaussian cluster, **tetrahedralize** it into a cage, embed the
  Gaussians inside via interpolation weights (classic cage-based deformation, same
  family as mesh skinning), simulate the cage with **eXtended Position-Based Dynamics
  (XPBD)**, and let the deformed cage drive Gaussian position/covariance through the
  same embedding weights. For an SDF primitive, the natural cage is derivable directly
  from the primitive's own analytic shape (a torus's cage doesn't need estimating from
  point-cloud reconstruction the way VR-GS does for captured scenes — this project
  already knows the exact analytic surface).
- **SC-GS** (CVPR 2024, [arXiv 2312.14937](https://arxiv.org/abs/2312.14937),
  [code](https://github.com/CVMI-Lab/SC-GS)) decomposes motion into dense Gaussians
  (appearance) plus a much smaller set of **sparse control points** carrying 6-DOF
  transforms; each Gaussian's motion is a learned weighted interpolation of nearby
  control points — a linear-blend-skinning analogue where "bones" are sparse control
  points rather than a rig, regularized by an ARAP (as-rigid-as-possible) loss for local
  rigidity. Directly relevant as a **hand-authored** (not learned) pattern too: this
  project could place a handful of control points per primitive (e.g. one per torus
  "segment") and drive splat motion by simple distance-weighted interpolation of those
  control points' transforms — cheaper than full MPM, more expressive than one rigid
  transform per primitive, useful for squash/stretch/bend without needing a physics
  solver at all.
- **Skeleton/LBS-bound Gaussians** (GauHuman, HUGS, GASPACHO — surveyed in
  [dynamic-4d-gaussian-splatting](../../3dgs/state-of-the-art/dynamic-4d-gaussian-splatting.md)'s
  neighborhood but focused there on *captured* human performance) bind each Gaussian to
  a skeleton via classic Linear Blend Skinning — directly the technique to reach for if
  this project ever adds articulated (jointed) SDF characters, though LBS's known
  failure modes (candy-wrapper twisting artifacts, volume loss at joints) apply exactly
  as they do to mesh skinning.
- **Gaussian Splashing** ([project page](https://gaussiansplashing.github.io/),
  [arXiv 2401.15318](https://arxiv.org/abs/2401.15318)) couples Position-Based Dynamics
  (solids/rigid bodies) with Position-Based Fluids (SPH-based incompressibility) and
  transfers simulation motion onto Gaussian kernels via **generalized moving least
  squares (GMLS)** rather than a rigid 1:1 particle-to-Gaussian mapping — the technique
  to reach for specifically if this project ever wants fluid content (a splashing
  pool, viscous terrain) alongside solid SDF geometry, since GMLS decouples the
  simulation particle count from the rendered Gaussian count.

## What this means for this project concretely, in order of effort

1. **Already done**: rigid `Transform` animation between periodic re-bakes (rings,
   pillar). No further work needed for purely rigid motion.
2. **Next natural step, low effort**: extend the ECS-authored `Isometry`
   (`sdf/scene.rs`) from rigid-only to a small, explicit set of *non-rigid but still
   cheap* per-primitive parameters — e.g. non-uniform squash/stretch applied directly
   to already-baked splat scale/covariance, without touching splat *positions* at all
   (valid only for small deformations that don't need the surface to actually
   re-sample; call this out explicitly as an approximation, not physically exact,
   distinct from Step 3 below).
3. **Higher effort, physically correct**: sparse control-point interpolation (SC-GS's
   pattern, hand-authored rather than learned) for expressive bend/squash on a specific
   primitive, without a full physics solver.
4. **Highest effort**: MPM- or XPBD-based physical simulation (PhysGaussian / VR-GS
   pattern) for genuinely physically-simulated deformable props — likely only justified
   if this project adds interactive terraforming or physics-driven props as a gameplay
   mechanic, not for purely cosmetic animation like the current rings.

Each step up this list trades implementation cost for expressiveness; nothing here
requires jumping straight to Step 4 to get value from Steps 2-3, and Step 1 (already
implemented) remains the correct choice for any future purely-rigid animated content
(more rings, a rotating platform, an orbiting prop) — resist the temptation to reach for
a heavier technique than a scene actually needs.

## When to dive in

- Adding new rigid animated content (more spinning/orbiting primitives) → no new
  technique needed; extend the existing `AnimatedRing`-style pattern.
- Wanting squash/stretch or bend on an SDF primitive without full physics → Step 2 or 3
  above; start with Step 2's direct covariance scaling for the cheapest possible
  effect, escalate to SC-GS-style control points only if that proves visually
  insufficient.
- Adding physically-simulated deformable or fluid props → PhysGaussian (solids,
  elastic/plastic materials), VR-GS (cage-embedded interactive manipulation), or
  Gaussian Splashing (fluids) are the concrete techniques to prototype from, chosen by
  which material behavior is needed.
- Unsure whether a given scene change needs a re-bake at all → apply the local-frame
  test from the rule table above before reaching for either a transform update or a
  full re-bake.

## Related

- [Making a local SDF edit re-bake fast enough to feel live](./incremental-rebake.md) — deeper: the re-bake half of the rule.
- [Streaming, LOD, and cache invalidation for large baked worlds](../baking-pipeline/streaming-and-invalidation.md) — prerequisite: the periodic re-bake the rings relied on.
- [One SDF driving collision, physics, and lighting](./unified-physics-and-lighting.md) — deeper: getting physics results back onto splats.
- [Dynamic and 4D Gaussian Splatting](../../3dgs/state-of-the-art/dynamic-4d-gaussian-splatting.md) — contrast: learned deformation fields for captured video.
- [The Gaussian primitive: parameters and math](../../3dgs/fundamentals/gaussian-primitive-parameters.md) — prerequisite: the covariance decomposition a rigid transform rotates.
- [Combining SDFs: boolean CSG and smooth blending](../../sdf-3d/primitives-and-operators/combination-operators.md) — prerequisite: why smooth-union neighbours force a re-bake.
