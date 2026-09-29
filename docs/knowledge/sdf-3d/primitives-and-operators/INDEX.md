---
title: Primitives and Operators
description: The vocabulary and grammar of procedural SDF modeling — the canonical primitive set, boolean/smooth-blend combination, single-shape modifiers, and domain operations (mirror, repeat). Read when building or extending procedural SDF content, including src/sdf/primitives.rs.
type: index
status: current
tags:
  - sdf
  - primitives
  - csg
updated: 2026-09-28
---

# Primitives and Operators

The primitive shape library everything is built from, plus the three operator
families (combination, single-shape modification, domain transformation) that
compose primitives into arbitrarily complex scenes. Nearly all hand-authored SDF
content — including migera's `src/sdf` scenes — is a tree of these building blocks.

## Start here

Start with [primitive-shapes](./primitive-shapes.md), then
[combination-operators](./combination-operators.md). If a raymarching artifact
appeared right after adding an operator, check whether that operator preserves
exactness ([exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md)).

## Key facts

- Union/intersection/subtraction are `min`/`max`/`max(a,-b)` — no clipping algorithm — see [combination-operators](./combination-operators.md).
- Smooth union bulges outward at the seam by design; reduce `k` rather than "fixing" it — see [combination-operators](./combination-operators.md).
- Rounding and elongation stay exact; displacement, twist and bend need step damping — see [modifier-operators](./modifier-operators.md).
- `mod()` repetition is free instancing, but only valid when the shape is small relative to its period — see [domain-operations](./domain-operations.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [The canonical 3D SDF primitive set](./primitive-shapes.md) | IQ's ~28 primitives, which are exact vs. bounds, and which ones migera implements. | Adding a primitive to `src/sdf/primitives.rs` or choosing building blocks. |
| [Combining SDFs: boolean CSG and smooth blending](./combination-operators.md) | min/max booleans; polynomial vs. exponential vs. root smin; C² and the bulge artifact; choosing `k`. | Blending two or more shapes, or when a blend seam looks wrong. |
| [Single-shape modifiers](./modifier-operators.md) | Rounding, onion, elongation (exact); displacement, twist, bend (approximate). | Rounding, hollowing, stretching or warping one shape. |
| [Domain operations](./domain-operations.md) | abs() symmetry, mod() infinite and clamped finite repetition, and the large-shape seam caveat. | Tiling or mirroring content, or when repeated geometry shows seams. |

## See also

- [Analytic intersections](../../analytic-intersections/INDEX.md) — closed-form ray hits for the same primitive shapes.
- [RoundedCone SDF reports everything exterior](../../hybrid-architecture/roundedcone-sdf-reports-everything-exterior.md) — open bug in migera's RoundedCone.
