---
title: Performance and Production
description: The concrete SDF cost model, the sparse/hierarchical storage and empty-space structures production relies on, and the Dreams vs. Claybook shipped architectures. Read when estimating an SDF approach's feasibility or designing a new SDF engine's render/physics architecture.
type: index
status: current
tags:
  - sdf
  - performance
  - spatial-acceleration
updated: 2026-09-28
---

# Performance and Production

Theory is necessary but not sufficient to ship: this topic covers the cost model,
the storage/acceleration structures production systems rely on, and two shipped
games that made different, well-reasoned architecture choices.

## Key facts

- Raymarch cost = steps × per-step evaluation for procedural fields, but memory bandwidth for grids — the fix differs — see [performance-characteristics](./performance-characteristics.md).
- Dense grids are only practical at single-asset scale; beyond that use octrees, hashing or clipmaps — see [sparse-and-hierarchical-structures](./sparse-and-hierarchical-structures.md).
- Dreams converts SDFs to points for rendering; Claybook raymarches SDF grids directly for both rendering and physics — see [production-case-studies](./production-case-studies.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Performance characteristics of SDF rendering](./performance-characteristics.md) | Steps × evaluation cost, grid bandwidth with Claybook's numbers, iteration budgets, wins/losses vs. polygons. | Estimating a budget, or a raymarcher is too slow. |
| [Sparse and hierarchical acceleration structures for grid SDFs](./sparse-and-hierarchical-structures.md) | SVO, SVDAG, voxel hashing, clipmaps, nested fields for empty-space skipping; how to choose. | A grid SDF uses too much memory or marches too long through empty space. |
| [Production case studies: Dreams and Claybook](./production-case-studies.md) | Two shipped architectures and the rationale behind each. | Designing an SDF engine architecture, or justifying SDF physics. |

## See also

- [Hybrid architecture performance findings](../../hybrid-architecture/INDEX.md) — measured costs in migera's own `src/hybrid`.
- [Hierarchical volumes](../../hierarchical-volumes/INDEX.md) — traversal-side treatment of the same structures.
