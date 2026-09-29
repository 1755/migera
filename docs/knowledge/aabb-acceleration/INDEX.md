---
title: AABBs & spatial acceleration — Knowledge Base
description: Axis-aligned bounding boxes for migera - the slab ray/AABB test, the grid/octree/kd-tree/BVH landscape, Bevy 0.19.1's own AABB types, CPU vs GPU placement, and a camera-ray debugging lesson. Read before writing any ray/box test, choosing a grid vs BVH, or touching culling.
type: index
status: current
tags:
  - bounding-volumes
  - spatial-acceleration
  - ray-tracing
  - bevy
updated: 2026-09-28
---

# AABBs & spatial acceleration — Knowledge Base

Research-grounded notes on axis-aligned bounding boxes: the math of fast ray/AABB tests,
the acceleration-structure landscape around them, Bevy 0.19.1's own AABB primitives
(verified against crate source), and where each piece of work belongs — CPU or compute
shader. The tree was started when per-leaf *bounding-sphere* culling in the legacy
marcher failed twice (phantom shells, overshoot): a thin ground slab's bounding sphere
had radius ~9.2, while its box is a thin slab. The `src/hybrid` rewrite now uses
per-object world-space AABBs in a SAH BVH (`src/hybrid/scene.rs`, `src/hybrid/bvh.rs`).
BVH-specific depth lives in [hierarchical-volumes](../hierarchical-volumes/INDEX.md).

## Key facts

1. **The slab method is *the* ray/AABB test** (Kay & Kajiya 1986): branchless with per-ray `dir_inv`; IEEE infinities absorb zero-direction components; NaN needs the clamped update ([slab test](./ray-aabb-slab-test.md)).
2. **Return `[t_near, t_far]` intervals, not booleans** — they compose into ordered traversal and interval skipping ([slab test](./ray-aabb-slab-test.md)).
3. **Never march a per-object interval through a cross-object merged interval list** — the merged list has no per-object memory and can skip an object's real surface; use it only as a whole-ray reject ([raymarching artifacts](../sdf-3d/rendering/raymarching-artifacts-and-fixes.md#flat-cut-or-chord-bitten-out-of-a-round-silhouette-near-where-two-objects-touch)).
4. **Structure choice follows density**: grids for dense/uniform geometry and O(N) builds, SAH BVHs for sparse scenes ([acceleration structures](./acceleration-structures.md)).
5. **In Bevy 0.19, AABBs live on both sides**: `RayCast3d::aabb_intersection_at` on the CPU; `bevy_render`'s occlusion culling tests AABBs in compute against Hi-Z ([Bevy primitives](./bevy-aabb-primitives.md)).
6. **Keep AABBs in local space** for rotated primitives and transform the ray, or fit conservative world boxes from the 8 transformed corners ([slab test](./ray-aabb-slab-test.md)).
7. **GPU AABBs ship as six scalar f32s**, never nested `vec3`s, so Rust `repr(C)` matches WGSL ([Bevy primitives](./bevy-aabb-primitives.md)).
8. **Data residency decides CPU vs GPU placement** ([placement](./cpu-vs-gpu-placement.md)).
9. **Debug ray/pixel mismatches with the shader's own `ro`/`rd`**, never a reconstructed camera ([camera-ray lesson](./verify-camera-rays-from-the-shaders-own-matrices.md)).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Ray–AABB intersection: the slab method](./ray-aabb-slab-test.md) | Naive → branchless → NaN-safe → boundary-inclusive slab tests, intervals, reference WGSL, where `src/hybrid` uses it | writing or changing any ray/box test |
| [Acceleration structures around AABBs](./acceleration-structures.md) | Grids, octrees, kd-trees, BVHs, SAH/LBVH, grid hierarchies; per-workload choice table | choosing a spatial structure for a new subsystem |
| [AABB primitives in Bevy 0.19.1 (source-verified)](./bevy-aabb-primitives.md) | `Aabb3d`/`RayCast3d` vs the `Aabb` ECS component, GPU occlusion-culling precedent, WGSL layout convention | using Bevy bounds types or uploading AABBs |
| [Where AABB work belongs: CPU or compute shader](./cpu-vs-gpu-placement.md) | Placement table with data residency as tiebreaker; worst practices on both sides | deciding where a bounds test or build runs |
| [Verify camera rays from the shader's own matrices](./verify-camera-rays-from-the-shaders-own-matrices.md) | A guessed-camera ground-truth script reversed a real diagnosis; check inside the shader instead | before building any external ray/pixel ground-truth check |

## See also

- [Hierarchical volume structures](../hierarchical-volumes/INDEX.md) — BVH deep dive, VDB-style grids, frustum culling.
- [Analytic surfaces & exact ray intersection](../analytic-intersections/INDEX.md) — exact intersectors that pair with AABB pre-tests (retired from the live renderer).
- [Module and stage skeleton](../hybrid-architecture/module-and-stage-skeleton.md) — the measured AABB/BVH win in `src/hybrid` and the declined inner-AABB idea.
- [Compute Shaders](../compute-shaders/INDEX.md) — running these tests on the GPU efficiently.
