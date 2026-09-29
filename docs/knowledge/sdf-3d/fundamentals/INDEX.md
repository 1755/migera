---
title: Fundamentals
description: The mathematical vocabulary every other sdf-3d note assumes — what an SDF guarantees (Eikonal equation, gradient = normal), exact vs. bound fields, the four representation families, truncated narrow-band fields. Read first when new to SDFs or when a note uses "bound" or "narrow-band" unexplained.
type: index
status: current
tags:
  - sdf
  - math
  - raymarching
updated: 2026-09-28
---

# Fundamentals

Before touching primitives, rendering, or conversion techniques you need the
mathematical vocabulary: what an SDF actually guarantees, why that guarantee
sometimes breaks (and why that's often fine), and the handful of
storage/representation strategies every SDF tool or paper is a variation of.

## Start here

1. [What is a signed distance field?](./what-is-an-sdf.md) — always first.
2. [Exact vs. bound distance fields](./exact-vs-bound-sdfs.md) — most raymarching
   artifacts (holes, punch-through, banding) trace back to a broken distance guarantee.
3. The other two as the task demands.

## Key facts

- `|∇f| = 1` (Eikonal) is what makes the value a distance, and on the surface `∇f` is the normal — see [what-is-an-sdf](./what-is-an-sdf.md).
- Sphere tracing needs only "never overestimates", so bounds (scaled, blended, repeated fields) are safe with step damping — see [exact-vs-bound-sdfs](./exact-vs-bound-sdfs.md).
- A TSDF is not safe to sphere-trace beyond its truncation band; pair it with an occupancy structure — see [narrow-band-and-truncated-sdfs](./narrow-band-and-truncated-sdfs.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [What is a signed distance field?](./what-is-an-sdf.md) | The definition, the Eikonal equation `\|∇f\|=1`, gradient = normal, and why these explain almost everything SDFs are good for. | First time working with SDFs, or before implementing a raymarcher. |
| [Exact vs. bound distance fields](./exact-vs-bound-sdfs.md) | True distance vs. Lipschitz-bounded conservative fields; which operations (scaling, blending, warping) produce bounds; step damping keeps them safe. | Before writing/combining custom primitives, or when a raymarch shows holes or punch-through. |
| [The four ways an SDF can be stored/evaluated](./sdf-representations.md) | Procedural, sampled grid, hybrid (converted for rendering), neural — storage, cost and use-case tradeoffs. | Choosing how to store/represent SDF content for a new feature. |
| [Narrow-band and truncated SDFs (TSDF)](./narrow-band-and-truncated-sdfs.md) | Storing accurate distance only near the surface: cheap storage and live-fusion updates, unsafe long-range marching. | Building a scanning/fusion pipeline, or wondering why an engine's "SDF" isn't a global distance field. |

## See also

- [Rendering](../rendering/INDEX.md) — where these guarantees get consumed by sphere tracing.
- [Raymarching via compute](../../compute-shaders/raymarching-via-compute.md) — the GPU implementation side.
