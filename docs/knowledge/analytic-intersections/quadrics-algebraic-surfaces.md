---
title: Quadric surfaces & algebraic surfaces
description: Defines algebraic surfaces and quadrics, catalogs practical surfaces by polynomial degree (planes, quadrics, torus/superquadric quartics), states the Abel-Ruffini degree-5 limit of closed-form intersection, and lists analytic normals and degenerate cases. Read before deciding whether a shape can be intersected exactly.
type: concept
status: current
tags:
  - ray-tracing
  - math
  - primitives
updated: 2026-08-23
sources:
  - Cambridge AGraph course notes
  - Scratchapixel ray-sphere lesson
  - Ray Tracing Gems
aliases:
  - quadric
  - algebraic surface
  - Abel-Ruffini
  - implicit surface degree
---

# Quadric surfaces & algebraic surfaces

Sources: Cambridge AGraph course notes (cl.cam.ac.uk node2), Scratchapixel ray-sphere
lesson, RT Gems chapters, standard algebraic geometry references.

## Definitions

An **algebraic surface** is the zero set of a polynomial in x,y,z:
`F(x,y,z) = 0`, of total degree d. An **analytic (closed-form) primitive** is one whose
ray intersection reduces to solving such a polynomial - i.e. t-roots exist in radicals.
A **quadric** is degree 2; it is the largest family with ONE uniform solution method
(the quadratic formula) covering every member.

Substituting the ray p(t)=o+t*d into F yields a degree-d polynomial in t with at most d
real roots = at most d intersection points, ordered along the ray.

## The practical catalog by degree

| Deg | Surface | Canonical implicit form |
|-----|---------|------------------------|
| 1 | Plane/half-space | n.p + w = 0 |
| 1 | Slab (infinite box face pair) | min_d <= p_d <= max_d |
| 2 | Sphere | |p-c|^2 = r^2 |
| 2 | Ellipsoid | sum ((p_i-c_i)/r_i)^2 = 1 (sphere in scaled space) |
| 2 | Infinite cylinder | x^2+z^2 = r^2 (axis-aligned canonical) |
| 2 | Cone (double/infinite) | x^2+z^2 = (k*y)^2 |
| 2 | Paraboloid | y = a(x^2+z^2) |
| 2 | Hyperboloid (1/2-sheet) | x^2/a^2 + z^2/b^2 - y^2/c^2 = +-1 |
| 2 | Spheroid/quadric general | p^T A p + b^T p + c = 0 |
| capped variants | finite cylinder/cone, disc | quadric AND slab(s) - still quadratic per test |
| 4 | Torus | (|p_xy|-R)^2 + p_z^2 = r^2 |
| 4 | Superquadrics/Goursat (Sphere4 etc.) | |x|^n+|y|^n+|z|^n = r^n for even n<=4 |
| 4 | Capsule/RoundedCone/CapsuleCone unions | NOT single polynomials - piecewise quadrics; handled as geometry (segment-sphere), not algebra |

**Hard limit**: degree >= 5 has no general closed-form root formula (Abel-Ruffini).
Anything beyond quartics needs iteration (marching/Newton/bisection) or Sturm-sequence
bracketing. This is *the* boundary between "analytic primitive" and "SDF-marched shape".

## Normals

For implicit F, the normal at hit point p is the normalized gradient:
`n = normalize(grad F(p))`. Cheap closed forms exist per primitive:
sphere `p-c`; torus `normalize(p*(dot(p,p)-r^2-R^2*vec3(1,1,-1)))`;
superquadric `normalize(4*p^3)` style gradient powers; boxes from which-slab logic.
Never numerically differentiate an exact surface - the analytic gradient is free.

## Degenerate & edge cases to respect

- Ray origin ON the surface (t=0 root) - shadow/AO rays live here constantly.
- Tangent rays (discriminant = 0): grazing hits, double root.
- Ray parallel to cylinder/cone axes: the quadratic degenerates to linear.
- Capped solids are INTERSECTIONS of curved surface and half-spaces: solve both,
  keep consistent intervals (see csg-intervals).
- Non-closed primitives (single-sided planes/discs) break solid-CSG assumptions.

## Related
- [Exact ray-primitive intersectors - catalog with costs](./primitive-intersection-catalog.md) — deeper: the per-primitive algorithms for the surfaces listed here.
- [Robustness, best/worst practices & limitations](./robustness-and-limits.md) — deeper: solving these polynomials stably in f32, and why the quartic torus failed on the GPU.
- [CSG on exact intersections](./csg-intervals.md) — applies: capped solids and composite shapes as interval intersections.
- [The canonical 3D SDF primitive set](../sdf-3d/primitives-and-operators/primitive-shapes.md) — contrast: the same shapes as distance fields, which migera renders instead.
