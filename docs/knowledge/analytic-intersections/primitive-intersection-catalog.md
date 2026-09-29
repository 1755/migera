---
title: Exact ray-primitive intersectors - catalog with costs
description: Catalogs operation-minimized exact ray intersectors (iq's set) per primitive - plane, sphere, ellipsoid, box, rounded box, cylinder, cone, capsule, torus, hex prism - with canonical-space conventions, theoretical-minimum notes and mandatory bounding pre-tests. Read before implementing any exact ray-primitive test.
type: reference
status: current
tags:
  - ray-tracing
  - primitives
  - bounding-volumes
  - performance
updated: 2026-08-23
sources:
  - https://iquilezles.org/articles/intersectors/
  - arXiv 2301.03191 (torus formulations)
  - Ray Tracing Gems
aliases:
  - intersector
  - Inigo Quilez intersectors
  - ray-sphere
  - ray-capsule
  - ray-torus
---

# Exact ray-primitive intersectors - catalog with costs

Primary source: Inigo Quilez "Ray-surface intersectors"
(https://iquilezles.org/articles/intersectors/) - hand-derived, operation-minimized
implementations with Shadertoy demos per primitive. Supporting: Cambridge course notes,
torus formulations paper (arXiv 2301.03191), RT Gems.

## Conventions

- Canonical space: every primitive is defined centered/axis-aligned; transform the RAY
  into primitive space (inverse isometry) once, intersect, use t back in world space.
  Never generalize the surface formula (Cambridge notes: option 2 of 2).
- Return [tN,tF] intervals where cheap - CSG and shadow rays need them.
- iq's functions assume origin OUTSIDE the primitive unless noted; inside-origin needs
  the tF branch.

## Per-primitive algorithms & costs

| Primitive | Method | Notes on minimality |
|---|---|---|
| Plane | one dot | t = -(w + n.o)/n.d |
| Sphere | quadratic, a=1 if rd normalized | early-out: c>0 && b>0 => behind; q-formula for roots |
| Ellipsoid | scale space by 1/radii, then sphere | unnormalized rd handled via a=dot(rd2,r2) form |
| AABB / box | slabs: m=1/rd (precompute!), k=abs(m)*halfsize | ZERO sqrt; returns tN/tF + face normal from step() compares; iq's version also gives inside-origin normal |
| Rounded box | slab test on size+rad, corner sphere solve only when needed | falls through to exact only near corners |
| Cylinder (capped) | infinite-cylinder quadratic AND two cap planes; pick consistent interval | reject via y-range test before solving caps |
| Cone (capped) | same shape as cylinder with k*y radius term | one cap check suffices (iq optimization) |
| Capsule | quadratic vs segment: a=baba-bard^2 etc., body hit iff y in (0,baba); else ONE cap | iq checks ONE spherical cap only - provably sufficient; ~15 mults total |
| Rounded cone | body cubic-free formulation + both caps | heavier; consider capsule fallback |
| Torus | quartic via resolvent cubic (iq): coefficients from dot products; degenerate-coefficient swap branch (`po`) handles grazing axis rays; bounding cylinder (R+r)^2 pre-test rejects most rays before any quartic work; hole-aware bounding (two coaxial cylinders + planes) also rejects through-the-hole rays (Skala 2023) | ~60-80 ops worst case; always pre-bound |
| Hex prism | 4 plane pairs (3 side normals at 60deg + top/bottom), max-entry/min-exit like slabs | no quadratics at all |
| Triangle | Moller-Trumbore or watertight Woop variant | out of scope here; see RT Gems ch. |
| Disc | plane hit + center-distance check | trivial |

## Theoretical-minimum notes

- A quadric hit fundamentally needs: substitute -> 3-term quadratic -> discriminant ->
  sqrt -> root. You cannot do better than ~1 sqrt + O(10) mul/adds; all real gains come
  from (a) skipping primitives via bounds, (b) never computing roots you discard
  (early-outs on signs), (c) reusing per-ray precomputation (rd_inv, sign(rd)).
- Shadow/occlusion queries need only "any root < t_max": skip root selection entirely
  after discriminant sign check where possible.
- Distance-bound clipping: keep global closest-so-far `distBound.y`; each intersector
  takes it and rejects instantly when tN > bound. This composes free LOD across scene.

## Bounding-volume pairing (mandatory practice)

Every non-trivial intersector gets an AABB/sphere pre-test (see aabb-acceleration
knowledge): torus->coaxial cylinder pair, rounded shapes->outer slab box, capped
solids->AABB. Rejection rate >99% in typical scenes makes the expensive path rare.

## Related
- [Quadric surfaces & algebraic surfaces](./quadrics-algebraic-surfaces.md) — prerequisite: why each primitive needs a linear, quadratic or quartic solve.
- [Robustness, best/worst practices & limitations](./robustness-and-limits.md) — deeper: stable root formulas and the test cases every intersector needs.
- [Ray–AABB intersection: the slab method](../aabb-acceleration/ray-aabb-slab-test.md) — prerequisite: the box intersector and the pre-test every other intersector is paired with.
- [The analytic tier cost 5-6x SDF marching and was removed](./production-performance-finding.md) — contrast: why these intersectors lost to marching as a per-frame shading path in migera.
- [The canonical 3D SDF primitive set](../sdf-3d/primitives-and-operators/primitive-shapes.md) — contrast: distance-function versions of the same shapes.
