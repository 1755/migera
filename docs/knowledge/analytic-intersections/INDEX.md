---
title: Analytic surfaces & exact ray intersection — Knowledge Base
description: Exact ray intersection with algebraic surfaces - degree limits, iq's intersector catalog, CSG on spans, f32 robustness, Bevy primitives - and why migera removed its analytic tier (5-6x slower than marching). Not used by the live renderer. Read before reintroducing exact intersection or writing a ray/primitive solver.
type: index
status: current
tags:
  - ray-tracing
  - primitives
  - csg
  - numerics
updated: 2026-09-28
---

# Analytic surfaces & exact ray intersection — Knowledge Base

> **Not used by the live renderer.** Migera's analytic intersection tier was removed on
> 2026-08-30 (commit dcf8a70) after RenderDoc showed it cost ~5.5-6x the GPU dispatch
> time of SDF marching in lit/shadowed/reflective scenes — see
> [the production finding](./production-performance-finding.md). The current
> `src/hybrid` renderer is SDF marching only. The research notes here remain valid,
> source-verified references for picking, collision or editor queries.

Research-grounded notes on quadric/algebraic surfaces and analytic primitives: the
practical catalog, exact ray-primitive intersectors near the theoretical operation
minimum, CSG on intervals, numerical robustness, limitations, and what Bevy 0.19
provides. Exact intersection was explored as a complement to sphere tracing (closed-form
t-values, no iteration, exact normals); smooth blending (smin) always needed marching.

## Start here

Read [the production finding](./production-performance-finding.md) first — it decides
whether the rest applies to your problem.

## Key facts

1. **Per-frame exact intersection lost to marching ~5.5-6x on the GPU**, because every shadow/reflection ray repeats a full solve ([production finding](./production-performance-finding.md)).
2. **Degree decides everything**: plane 1 root, quadrics 2, torus/superquadrics 4, and degree >= 5 has no closed form (Abel-Ruffini) ([quadrics](./quadrics-algebraic-surfaces.md)).
3. **iq's intersector collection** is the reference set, hand-minimized per primitive ([catalog](./primitive-intersection-catalog.md)).
4. **CSG composes exactly on spans, not points** (Roth 1982); single-hit variants store at most two candidate hits ([CSG on intervals](./csg-intervals.md)).
5. **The schoolbook quadratic is a precision bug**: use the q-formula, the geometric reformulation for far/small spheres, and an FMA discriminant ([robustness](./robustness-and-limits.md)).
6. **An f32 GPU torus quartic fell back to dense sampling on most adversarial rays** — slower than marching, so torus was dropped ([robustness](./robustness-and-limits.md)).
7. **Smooth blends are outside this world**: smin/smax surfaces are not algebraic ([CSG on intervals](./csg-intervals.md)).
8. **Bevy ships shape primitives but no exact intersectors**, only slab/sphere bounding tests ([Bevy integration](./bevy-integration.md)).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [The analytic tier cost 5-6x SDF marching and was removed](./production-performance-finding.md) | RenderDoc measurement, per-ray fan-out root cause, alternatives tried, removal decision | before reintroducing exact intersection anywhere per-frame |
| [Quadric surfaces & algebraic surfaces](./quadrics-algebraic-surfaces.md) | Surface catalog by degree, closed-form limit, analytic normals, degenerate cases | deciding whether a shape can be intersected exactly |
| [Exact ray-primitive intersectors - catalog with costs](./primitive-intersection-catalog.md) | Per-primitive algorithms and op counts, canonical space, mandatory bounding pre-tests | implementing a ray-primitive test |
| [CSG on exact intersections](./csg-intervals.md) | Span operations per boolean, cut-face normals, single-hit tables, interval bisection, smooth-blend limit | combining exact intersectors with booleans |
| [Robustness, best/worst practices & limitations](./robustness-and-limits.md) | Stable quadratic roots in f32, practice lists, torus retro, limits of the approach | writing any root solver in f32/WGSL |
| [Bevy 0.19.1 primitives & integration (source-verified)](./bevy-integration.md) | `bevy_math` primitive inventory, `Ray3d`/`RayCast3d`, picking, historical integration path | adding exact picking or collision queries |

## See also

- [AABBs & spatial acceleration](../aabb-acceleration/INDEX.md) — the slab test every exact intersector is paired with.
- [Sphere tracing](../sdf-3d/rendering/sphere-tracing.md) — the technique the renderer uses instead.
- [Primitives and Operators (SDF)](../sdf-3d/primitives-and-operators/INDEX.md) — the same shapes and booleans as distance fields.
