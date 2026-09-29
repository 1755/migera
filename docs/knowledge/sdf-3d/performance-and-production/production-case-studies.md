---
title: "Production case studies: Dreams and Claybook"
description: Contrasts two shipped SDF games — Dreams (SDF CSG for authoring/physics, converted to splatted point clouds) and Claybook (SDF grids raymarched directly for rendering and physics at 60 Hz) — and the lesson of choosing per job. Read before designing an SDF engine architecture or justifying SDF physics.
type: reference
status: current
tags:
  - sdf
  - case-study
  - prior-art
  - physics
  - performance
updated: 2026-08-15
aliases:
  - Dreams
  - Media Molecule
  - Claybook
  - Sebastian Aaltonen
---

# Production case studies: Dreams and Claybook

Two shipped PS4/Xbox-generation games are the most widely-cited real-world proof points
for SDF-based rendering and physics at production scale, and they made notably different
architectural choices worth contrasting directly.

## Media Molecule's *Dreams*: SDF as authoring tool, points for rendering

*Dreams* is built almost entirely on the PS4's compute units, with **no triangle
rasterization pipeline for its user-created geometry at all**. The architecture, per its
creators' own public technical description:

1. Scenes are authored and represented as **Operationally Transformed CSG trees** — that
   is, trees of primitive SDFs combined via boolean/smooth-blend operators (exactly the
   operator vocabulary covered in
   [combination-operators](../primitives-and-operators/combination-operators.md)), with
   "operationally transformed" referring to how edits to the tree are represented/merged
   (relevant to Dreams' collaborative multiplayer editing feature).
2. These CSG trees are evaluated on-the-fly into **high-resolution signed distance
   fields**.
3. The SDFs are then converted into **dense multi-resolution point clouds** — not
   triangles, not raymarched pixels directly.
4. The point clouds are what actually gets rendered on-screen, via GPU compute (splatting/
   point-based rendering), not rasterization of triangles and not per-pixel raymarching
   of the SDF itself.

**Why this specific architecture**: it separates three genuinely distinct concerns — (a)
an authoring representation optimized for artist-friendly, non-destructive, boolean-
combinable editing (SDFs via CSG trees are excellent for this), (b) a simulation-friendly
representation for physics (SDFs, again, for the tunneling-avoidance reasons discussed in
[performance-characteristics](./performance-characteristics.md)), and (c) a rendering
representation optimized for what the target hardware (PS4 compute units) could push
fastest (point clouds via compute-based rendering, rather than raymarching or
rasterization). Rather than forcing one representation to serve all three roles — which
raymarching-everything would attempt, at a real performance cost given 2010s-era hardware
— Dreams converts between representations at each stage boundary, picking whichever is
best for that specific job.

## Sebastian Aaltonen's *Claybook*: SDF for both rendering and physics, via raymarching

*Claybook* took a more direct approach: SDFs are used for **both** graphics **and**
physics, with actual GPU raytracing (sphere tracing) of the SDF as the rendering method —
no conversion to points or triangles for the final image.

Key architectural facts from the shipped engine (GDC 2018, Sebastian Aaltonen):

- **World SDF representation**: a volume texture with mipmaps, `1024 x 1024 x 512`
  resolution at 8-bit signed format, totaling 586 MB across 5 mip levels — see
  [performance-characteristics](./performance-characteristics.md) for why this specific
  precision/mip choice makes sense.
- **Acceleration**: a hierarchy of SDF grids with adaptive resolution selection during
  sphere-tracing traversal, maximizing safe step size at each stage — see
  [sparse-and-hierarchical-structures](./sparse-and-hierarchical-structures.md).
- **Physics**: a custom GPGPU physics solver (clay deformation + fluid simulation)
  operating directly against the SDF representation, deliberately chosen over a
  triangle-based physics representation because SDFs "solve tunneling elegantly with
  negative inner distances" — triangles, being an infinitely thin shell with no interior
  concept, cannot represent "how far a fast-moving thin object has already penetrated,"
  which SDFs do trivially via their negative-inside values.
- **No baked lighting**: shadows, ambient occlusion, and lighting are computed entirely
  in real time via the SDF raymarching machinery (see
  [soft-shadows-and-ao](../rendering/soft-shadows-and-ao.md)) — no precomputed lightmaps
  or baked AO, since the whole world is dynamically deformable (clay simulation) and
  baked lighting would be invalidated continuously.
- **Shipped performance**: locked 60Hz on Xbox One at original release, and the same
  content runs at 120Hz on more modern hardware (e.g. ROG Ally) — demonstrating the
  technique scaled down to a full 60fps budget on 2013-era console hardware, not just as
  a high-end tech demo.
- **Platform reach**: also shipped on Nintendo Switch, demonstrating the approach's
  viability even on significantly more constrained mobile-class GPU hardware.

## The key architectural lesson

Dreams and Claybook represent the two ends of a real, still-relevant design spectrum:
**"SDF for authoring/physics, converted to a different representation for rendering"**
(Dreams) versus **"SDF used directly through the entire pipeline including final
rendering"** (Claybook). Neither is universally "more correct" — Dreams' choice reflects
prioritizing an extremely flexible, artist-facing collaborative editing tool where
render-time conversion cost is amortized across a huge, diverse library of user content;
Claybook's choice reflects a tightly-scoped, fully-dynamic simulation-driven world where
avoiding any conversion step keeps the physics-to-render pipeline coherent and simple.
Understanding both architectures — and *why* each team made the choice it did — is more
useful for a new project than treating either as a universal template.

## When to dive in

- Designing a new SDF-based engine and deciding on the rendering/physics architecture →
  read both case studies fully before committing; the "right" choice depends heavily on
  whether content is highly dynamic/simulated (favors Claybook's direct approach) or
  authored/edited by users with a premium on flexible non-destructive tooling (favors
  Dreams' conversion approach).
- Justifying SDF-based physics over mesh-based physics to a team → Aaltonen's tunneling-
  avoidance rationale (negative interior distances vs. thin triangle shells) is the
  concrete, citable technical argument.
- Looking for shipped-game evidence that SDF techniques are production-viable at 60fps on
  console-class hardware, not just tech demos → Claybook's cross-platform 60Hz+ shipping
  numbers are the reference point.
- Designing SDF-driven collision/physics for a project that renders via Gaussian
  splatting rather than points (Dreams) or direct raymarching (Claybook) → see
  [unified-physics-and-lighting](../../sdf-3dgs-bevy-integration/live-editing/unified-physics-and-lighting.md),
  which develops both architectures' physics rationale directly against a Gaussian-splat
  rendering target — a combination neither shipped game addresses, since both predate
  3D Gaussian Splatting.

## Related
- [Rendering grid-based and hybrid SDF representations](../rendering/hybrid-and-grid-rendering.md) — deeper: the hybrid patterns these games exemplify.
- [Unreal Lumen distance fields](../state-of-the-art/unreal-lumen-distance-fields.md) — contrast: the third production pattern, SDFs for secondary rays only.
- [Unified physics and lighting](../../sdf-3dgs-bevy-integration/live-editing/unified-physics-and-lighting.md) — deeper: both games' physics rationale applied to a splat renderer (archived, never built).
- [migera pivots to Bevy PBR for characters](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — applies: migera's own per-job split (rasterized characters, SDF for effects/worldgen).
