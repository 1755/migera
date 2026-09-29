---
title: Converting GLB scenes to SDF BSN scenes
description: Archived 2026-08 plan to fit the Gothic II Khorinis GLB with SDF primitives + ECS scenes — gap analysis vs. the then-4-primitive raymarcher, new primitives, a Python analyse→fit→codegen pipeline, triplanar tiers, 10,000+ primitive estimate. Read only for history or before reviving mesh→primitive fitting.
type: design
status: archived
tags:
  - sdf
  - assets
  - mesh-conversion
  - primitives
  - ecs
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/sdf/primitives.rs
  - src/sdf/components.rs
  - assets/shaders/raymarch.wgsl
  - tools/gothic_export
sources:
  - commit 43017bb (Capsule, RoundedCone, Ellipsoid, BoxFrame, HexPrism added)
  - commit 3d5d570 (Torus removed)
aliases:
  - glb_to_bsn
  - Khorinis
  - NewWorld
  - Gothic II world
  - primitive fitting
  - BSN scene
---

# Converting GLB scenes to SDF BSN scenes

> **Archived:** plan dropped. As of 2026-09-28 `assets/models/khorinis.glb`,
> `assets/levels/newworld.glb` and the `tools/glb_to_bsn` script are deleted from the
> working tree, no Khorinis example exists, and migera moved characters to Bevy PBR
> (SDF kept for effects/worldgen). Phase 1 did land: Capsule, RoundedCone, Ellipsoid,
> BoxFrame and HexPrism (commit 43017bb) and `STACK_CAPACITY = 32`; Torus was later
> removed (3d5d570). The "current" tables below describe the pre-43017bb state.
> `tools/gothic_export` (ZenGin → GLB exporter) still exists.

Contents: [Core challenge](#the-core-challenge) · [Gap analysis](#gap-analysis-what-we-have-vs-what-we-need) ·
[New primitives](#required-new-primitives) · [Renderer changes](#required-renderer-changes) ·
[Pipeline](#conversion-pipeline-architecture) · [Materials](#material-handling-strategy) ·
[Performance](#performance-considerations) · [Implementation order](#implementation-order) ·
[Related](#related)

This document covers the complete pipeline for converting a traditional polygonal mesh
scene (specifically a GLB file like our exported Khorinis world) into a scene described
using this project's SDF primitives and ECS components (`Shape`, `BlendMode`, `Material`)
— what we call a "BSN scene" (the ECS entity hierarchy that `assembly::assemble_scene`
folds into a `Node` CSG tree).

## The core challenge

A GLB file contains arbitrary triangle meshes with UV-mapped PBR textures. Our SDF
raymarcher evaluates exactly 4 leaf primitives (sphere, rounded box, torus, rounded
cylinder) combined via CSG operators. The gap between "arbitrary triangle mesh" and "4
SDF primitives + CSG" is the central problem this document addresses.

### Why this is hard (and what to expect)

**Fundamental limitation**: no algorithm can losslessly convert an arbitrary triangle mesh
into a small number of analytic SDF primitives. A Gothic 2 building with window frames,
doorways, roof tiles, and decorative stonework cannot be represented as 5 spheres and 3
boxes — it would need hundreds of primitives, each an approximation. This is the same
tradeoff every production SDF system makes:

- **Claybook**: uses CSG-composed primitives (spheres, boxes, cylinders) as the *authoring
  tool*, not the *import format*. The world is built from scratch using CSG, not converted
  from meshes.
- **Dreams** (Media Molecule): converts SDF CSG trees to point clouds for rendering, but
  the CSG trees are *authored*, not *imported from meshes*.
- **Unreal Lumen**: bakes per-mesh distance fields as *grid textures* (sampled SDF), not
  as analytic primitive compositions. This is the production solution for "imported mesh →
  SDF" — but it requires a 3D texture per mesh, not a compact CSG tree.

**What we can realistically achieve**: a *lossy but visually recognizable* SDF
approximation of the scene, using a combination of:

1. **Primitive decomposition** — fit boxes, cylinders, capsules, and spheres to the
   major geometric forms (buildings, walls, terrain patches, tree trunks).
2. **CSG composition** — use smooth union to blend overlapping primitives into organic
   forms, and subtract to carve openings (doors, windows).
3. **Triplanar texture projection** — project the original GLB textures onto the SDF
   surface via triplanar mapping, recovering much of the visual detail that geometry
   decomposition loses.
4. **Fallback grid SDF** (future) — for geometry too complex for primitive decomposition,
   bake a per-section grid SDF as a 3D texture leaf (see
   [hybrid-baked-and-procedural-scenes](./mesh-conversion/hybrid-baked-and-procedural-scenes.md)).

## Gap analysis: what we have vs what we need

### Current SDF primitives (`src/sdf/primitives.rs`)

| Primitive | Params | GPU tag | What it represents |
|-----------|--------|---------|-------------------|
| `Sphere` | radius | `TAG_LEAF_SPHERE` | Spheres, balls, approximate organic shapes |
| `RoundedBox` | half_extents, corner_radius | `TAG_LEAF_ROUNDED_BOX` | Buildings, walls, slabs, boxes with beveled edges |
| `Torus` | major_radius, minor_radius | `TAG_LEAF_TORUS` | Rings, toruses, curved pipes |
| `RoundedCylinder` | radius, half_height, edge_radius | `TAG_LEAF_ROUNDED_CYLINDER` | Pillars, columns, barrels, tree trunks |

### Current CSG operations (`src/sdf/components.rs`)

| Operation | What it does | Limitation |
|-----------|-------------|-----------|
| `Union` / `SmoothUnion(k)` | Merge two shapes | Only combines, can't carve |
| `Subtract(k)` | Carve one shape from another | Only on child→parent relationship |
| `Repeat` | Infinite XZ tiling | One global setting, not per-object |

### Current materials (`src/sdf/components.rs`)

| Material type | What it provides |
|--------------|-----------------|
| `Material` | `base_color` (Vec3), `metallic` (f32), `roughness` (f32) |
| `ProceduralPattern` | Shader-dispatched pattern with two materials + params |

### What a GLB world scene needs (Khorinis analysis)

The Khorinis world GLB contains:

- **Static world mesh**: terrain, cliffs, quays — large, mostly flat or gently curved
  geometry, covered in triplanar-friendly tiling textures (stone, grass, sand, water).
- **Vob meshes** (placed objects): buildings (walls, roofs, chimneys, doors, windows),
  vegetation (tree trunks, canopies, bushes), props (barrels, crates, fences, lamps),
  furniture, rocks.
- **Materials**: PBR metallic-roughness with base-color textures, alpha-masked foliage,
  some emissive materials (lamps).

**Estimated primitive counts for recognizable Khorinis sections**:

| Section | Approach | Est. primitives |
|---------|----------|----------------|
| Single building exterior | 3-8 boxes (walls, roof) + 2-4 cylinders (chimney, pillars) + subtracted boxes (windows, door) | 10-20 |
| Tree trunk + canopy | 1-2 cylinders (trunk) + 1-3 spheres (canopy) | 3-5 |
| Barrel/crate | 1-2 cylinders or boxes | 1-2 |
| Fence segment | 3-5 thin boxes (posts + rails) | 3-5 |
| Terrain patch | 1-2 thin rounded boxes | 1-2 |
| **Entire Khorinis** | Would need 10,000+ primitives for full coverage | impractical |

**Conclusion**: full-scene primitive decomposition is impractical for Khorinis. The
viable approach is **section-based**: decompose individual buildings/props into primitives,
project their textures via triplanar mapping, and leave complex terrain as a grid SDF
(future work) or as an ordinary Bevy mesh alongside the SDF content.

## Required new primitives

### High priority (needed for basic GLB conversion)

#### Capsule (sphere-swept line segment)

The most commonly fitted primitive for organic/elongated shapes: tree trunks, fence
posts, limbs, pipes, horns. An exact closed-form SDF exists.

```
capsule_sdf(p, a, b, r) = length(p - a - clamp(dot(p-a, b-a)/dot(b-a, b-a), 0, 1) * (b-a)) - r
```

Parameters: `start_point` (Vec3), `end_point` (Vec3), `radius` (f32).

GPU encoding: param_a = radius; translation = start_point; rotation encodes direction
to end_point as a quaternion. Alternatively, store as two translation endpoints using
existing `param_a..param_d` slots — but a single-rotation + single-translation is
simpler for the shader.

**New `Shape` variant**: `Shape::Capsule { a: Vec3, b: Vec3, radius: f32 }`.

**New GPU tag**: `TAG_LEAF_CAPSULE = 4` (bumps `LEAF_TAG_COUNT` to 5).

#### RoundedCone (tapered cylinder)

Roofs, chimneys, stalactites, conical structures. Exact SDF exists (Inigo Quilez).

```
rounded_cone_sdf(p, a, b, r1, r2) = ...
```

Parameters: `start_point`, `end_point`, `radius_start`, `radius_end`.

**New `Shape` variant**: `Shape::RoundedCone { a: Vec3, b: Vec3, r1: f32, r2: f32 }`.

**New GPU tag**: `TAG_LEAF_ROUNDED_CONE = 5`.

### Medium priority (improve decomposition quality)

#### Ellipsoid (approximate)

Non-uniformly scaled sphere. The standard formula is a *bound* (not exact), but
sufficient for raymarching with conservative step-damping. Needed for rocks, eggs,
asymmetric organic shapes.

```
ellipsoid_sdf(p, radii) ≈ ...  (IQ's iterative/projective approximation)
```

**New `Shape` variant**: `Shape::Ellipsoid { radii: Vec3 }`.

**New GPU tag**: `TAG_LEAF_ELLIPSOID = 6`.

#### HexagonalPrism / TriangularPrism

For Gothic architecture (hexagonal pillars, triangular roof cross-sections). Exact SDFs
exist per IQ's reference.

### Low priority (specialized)

#### RoundTorus (cut torus)

Partial torus segments for arched doorways, curved bridges.

#### BoxFrame

Hollow wireframe box for window frames, fence sections.

## Required renderer changes

### More leaf tags in `raymarch.wgsl`

Each new primitive needs:
1. A new `TAG_LEAF_*` constant in the shader
2. A new `sdf_*` distance function in the shader
3. A new match arm in `eval_leaf`
4. A corresponding `GpuShapeKind` variant in `primitives.rs`
5. A `gpu_record()` implementation returning the right params

The existing 4-param-per-leaf encoding (`param_a..param_d`) is sufficient for all
primitives above — capsule needs radius + encoded direction, rounded cone needs two
radii, ellipsoid needs 3 radii (with one param slot spare).

### Bigger eval_stack capacity

The current `STACK_CAPACITY = 16` is adequate for our demo scene (depth ~4). A
decomposed building with 10-20 primitives combined in a CSG tree could reach depth
8-12. Raising to `STACK_CAPACITY = 32` is cheap insurance.

### More lights

The current `MAX_LIGHTS` in the shader handles the scene's single directional light
well. A richer scene might need 2-4 lights (sun + ambient fill + a point light or two).
The light buffer is already a dynamic-length array; just raise the cap.

### Texture sampling (future)

For triplanar texture projection, the shader needs:
1. A 2D texture binding (the original GLB's base-color texture, baked to a PNG/texture
   asset)
2. A triplanar sampling function: `textureSample(tex, tex_sampler, p.xy) * |n.z| +
   textureSample(tex, tex_sampler, p.xz) * |n.y| + textureSample(tex, tex_sampler,
   p.yz) * |n.x|`
3. A new `TAG_LEAF_*` or a `pattern_id`-based dispatch for triplanar-material leaves

This is the biggest renderer change and should be a separate phase from the primitive
additions.

## Conversion pipeline architecture

### Overview

```
GLB file (Khorinis world)
        │
        ▼
┌─────────────────────────┐
│ Phase 1: Scene Analysis │  Python (gltf library)
│ - Parse node hierarchy  │
│ - Compute world bounds  │
│ - Group nearby meshes   │
│ - Classify sections     │
└─────────┬───────────────┘
          │ Vec<Section> (bounding boxes + mesh data + materials)
          ▼
┌─────────────────────────┐
│ Phase 2: Primitive Fit  │  Python (custom + optional SuperFit/MarchingPrimitives)
│ - Per-section decomp    │
│ - Fit boxes/cylinders   │
│ - Boolean subtractions  │
│   for doors/windows     │
│ - Assign materials      │
└─────────┬───────────────┘
          │ Vec<Section> with fitted primitives
          ▼
┌─────────────────────────┐
│ Phase 3: Code Gen       │  Python → Rust source
│ - Generate commands.spawn│
│ - Shape + Transform     │
│ - Material + BlendMode  │
│ - Output as .rs file    │
└─────────┬───────────────┘
          │
          ▼
   examples/khorinis_sdf.rs  (or similar)
```

### Phase 1: Scene Analysis

**Goal**: understand the GLB's structure and group meshes into decomposable sections.

**Steps**:
1. Parse the GLB using `pygltflib` (already used by `export_world.py`).
2. Walk the node tree, composing transforms (world matrix per node).
3. Extract triangle soup from all mesh primitives (positions + indices).
4. Compute per-node bounding boxes.
5. **Spatial grouping**: cluster nearby nodes into sections using a simple grid or
   BVH-based merge. The world mesh (terrain) is its own section; individual vob
   meshes within ~5m of each other form a group.
6. **Section classification**: for each group, determine the dominant shape type:
   - Mostly flat/horizontal → terrain patch (rounded box)
   - Mostly vertical + rectangular bounds → building (boxes)
   - Mostly cylindrical bounds → tree/pillar (cylinders)
   - Small + convex → prop (sphere/box/capsule)

**Output**: `Vec<Section>` where each section has:
- Bounding box (min/max Vec3)
- List of triangles (for primitive fitting)
- Material info (base_color, roughness, metallic, texture reference)
- Section type hint (terrain, building, vegetation, prop)

### Phase 2: Primitive Fitting

**Goal**: for each section, produce a list of SDF primitives that approximate its
geometry, plus CSG operations that carve openings.

**Algorithm** (simplified ResFit/Marching-Primitives approach):

For each section:

1. **Decompose into convex parts** using V-HACD or CoACD (run offline, emit a JSON
   intermediate).
2. **Fit SDF primitives to each convex part**:
   - Compute the part's principal axes (PCA on vertex positions).
   - Compute axis-aligned bounding box → `Shape::RoundedBox`.
   - If the part is elongated with circular cross-section → `Shape::RoundedCylinder`.
   - If the part is elongated with two distinct endpoints → `Shape::Capsule`.
   - If the part is roughly spherical → `Shape::Sphere`.
   - If the part tapers → `Shape::RoundedCone`.
3. **Detect and carve openings**: for each wall-face of a building section, check if
   there's a concavity (door, window) by sampling the mesh's signed distance at candidate
   positions. Carve detected openings as `BlendMode::Subtract` children.
4. **Assign materials**: read the GLB material's base_color/metallic/roughness for each
   fitted primitive, mapped from the nearest triangle's material.

**Fallback for complex sections**: if a section's geometry is too complex for primitive
fitting (many small features, organic shapes), emit a single `Shape::RoundedBox` that
bounds the section and rely on triplanar texture projection to supply visual detail.

### Phase 3: Code Generation

**Goal**: emit a Rust source file that, when compiled as a Bevy example, spawns the
same ECS entity hierarchy that `assembly::assemble_scene` can fold into a `Node` tree.

**Output format**: a standalone Rust file (example) that:
1. Imports `migera::sdf::components::*` and `migera::sdf::assembly::SdfSceneRoot`.
2. Defines a `setup` system that spawns:
   - A `SdfSceneRoot` entity.
   - Child entities with `Shape`, `Transform`, `Material`, `BlendMode`, and optionally
     `ChildOf(parent)` for CSG grouping.
3. Uses `RaymarchRenderPlugin` for rendering.
4. Optionally includes an orbit camera for navigation.

**Example output** (for a single building):

```rust
// Auto-generated by tools/glb_to_bsn.py — DO NOT EDIT
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::*;

fn setup(mut commands: Commands) {
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();

    // Building: stone_house_01 at (42.5, 0.0, -18.3)
    let building = commands.spawn((
        ChildOf(root),
        Transform::from_xyz(42.5, 0.0, -18.3),
    )).id();

    // Main volume
    commands.spawn((
        ChildOf(building),
        Shape::RoundedBox { half_extents: Vec3::new(4.0, 3.0, 5.0), corner_radius: 0.1 },
        Transform::from_xyz(0.0, 3.0, 0.0),
        Material::new(Vec3::new(0.65, 0.60, 0.50), 0.0, 0.85),
    ));

    // Roof
    commands.spawn((
        ChildOf(building),
        Shape::RoundedCone { a: Vec3::new(0.0, 6.0, -5.5), b: Vec3::new(0.0, 6.0, 5.5), r1: 0.2, r2: 0.2 },
        Material::new(Vec3::new(0.55, 0.20, 0.15), 0.0, 0.70),
    ));

    // Door opening (subtracted from main volume)
    commands.spawn((
        ChildOf(building),
        Shape::RoundedBox { half_extents: Vec3::new(0.8, 1.5, 0.5), corner_radius: 0.05 },
        Transform::from_xyz(0.0, 1.5, 5.0),
        BlendMode::Subtract(0.05),
    ));

    // Window opening
    commands.spawn((
        ChildOf(building),
        Shape::RoundedBox { half_extents: Vec3::new(0.6, 0.6, 0.5), corner_radius: 0.05 },
        Transform::from_xyz(2.0, 3.5, 5.0),
        BlendMode::Subtract(0.05),
    ));
}
```

## Material handling strategy

### Tier 1: flat PBR materials (immediate)

Each fitted primitive gets a single `Material` component with the GLB material's
`base_color`, `metallic`, and `roughness` values. This works well for buildings (uniform
stone/wood materials) and terrain (uniform grass/stone).

The color must be converted from sRGB to linear for the raymarcher (same conversion
`export_world.py` already applies).

### Tier 2: triplanar texture projection (future)

For geometry covered in tiled textures (terrain, walls), project the original GLB
texture via triplanar mapping from the hit point. This requires:

1. A new `ProceduralPattern`-like dispatch for triplanar materials.
2. The GLB's base-color texture exported as a separate PNG asset.
3. A new shader that samples the texture at X/Y/Z projections weighted by the surface
   normal.

This is the highest-impact visual improvement and should be prioritized after Tier 1.

### Tier 3: per-primitive UV baking (future)

For geometry with unique UV layouts (detailed props, signage), bake the texture into a
2D atlas per primitive (SuperFit's approach). Complex and memory-intensive; only for
hero assets.

## Performance considerations

### Primitive count vs frame rate

The raymarcher evaluates `map()` ~90 times per pixel (sphere-trace steps + normal taps +
shadow/AO steps). Each `map()` call walks all static primitives. Our current scene has
~10 static primitives. A decomposed building section might add 10-20 primitives.

**Estimated cost scaling**: if the total static primitive count goes from 10 to 200,
`eval_stack` time roughly doubles per `map()` call, so frame rate halves. This is
manageable if:
- Sections outside the view are excluded (future: spatial culling).
- The `AnimGroup` bounding-radius early-out pattern is extended to static sections.
- Step counts (MAX_STEPS_NEAR, SHADOW_MAX_STEPS) are tuned down for complex scenes.

### Memory: PrimitiveRecordCpu size

Each `PrimitiveRecordCpu` is 112 bytes (28 × f32). 200 primitives = 22.4 KB — trivial
for GPU storage.

## Implementation order

### Phase 1: New primitives (Rust + WGSL)
1. Add `Capsule` to `primitives.rs` + `components.rs` + `raymarch.wgsl`
2. Add `RoundedCone` to `primitives.rs` + `components.rs` + `raymarch.wgsl`
3. Bump `STACK_CAPACITY` to 32
4. Tests: verify capsule/cone distance matches IQ's reference formulas

### Phase 2: Conversion script (Python)
1. `tools/glb_to_bsn.py` — GLB parser + scene analyzer
2. Spatial grouping + section classification
3. Primitive fitting (box/cylinder/capsule detection from bounding geometry)
4. Rust code generation
5. Test against a single Khorinis building vob

### Phase 3: Triplanar textures (WGSL + Rust)
1. Add triplanar sampling pattern to `assets/shaders/patterns/`
2. Wire texture assets through `ProceduralPattern` component
3. Test with terrain materials

### Phase 4: Integration + optimization
1. Spatial culling for sections outside camera view
2. Adaptive step count based on scene complexity
3. Test with a full Khorinis district

## When to dive in

- Implementing new SDF primitives → start with `Capsule` (most commonly fitted shape),
  then `RoundedCone`. Follow the existing pattern in `primitives.rs` (implement `Sdf`
  trait + `GpuPrimitive`), `components.rs` (new `Shape` variant + `sdf()` match arm),
  and `raymarch.wgsl` (new tag constant + distance function + `eval_leaf` match arm).
- Building the conversion script → start with Phase 1 (scene analysis) using
  `pygltflib`; Phase 2 (primitive fitting) can use the Rust `mesh_to_sdf` crate's
  BVH for nearest-triangle queries, or ship the fitting as an offline Python step.
- Deciding whether to attempt full-scene or section-based conversion → section-based is
  the pragmatic choice; full-scene primitive decomposition of Khorinis would need
  10,000+ primitives and is not real-time feasible on current hardware.

## Related
- [The canonical 3D SDF primitive set](./primitives-and-operators/primitive-shapes.md) — applies: the primitives this plan added, and the current list.
- [RoundedCone SDF reports everything exterior](../hybrid-architecture/roundedcone-sdf-reports-everything-exterior.md) — same-trap: the RoundedCone added by this plan's Phase 1 is still broken.
- [Integrating a baked mesh SDF into this project's raymarcher](./mesh-conversion/hybrid-baked-and-procedural-scenes.md) — contrast: the grid-bake fallback this plan deferred.
- [Importing glTF/GLB models](./mesh-conversion/gltf-import-pipeline.md) — prerequisite: GLB parsing and mesh-defect concerns.
- [Texturing without UVs](./materials-and-texturing/uv-less-texturing.md) — deeper: the triplanar Tier 2 material plan.
- [migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — contrast: the direction change that left this plan behind.
