---
title: GPU compute-shader baking and Bevy asset-pipeline integration
description: Proposes running the SDF-to-splat bake as an embarrassingly parallel WGSL compute pass built on Bevy's UninitBufferVec/ShaderBuffer/PrepareResources machinery, sharing one SDF module between preview and bake. Never built (migera's bake was CPU) and archived. Read before writing a GPU SDF sampling pass.
type: design
status: archived
tags:
  - sdf
  - 3dgs
  - baking
  - gpu-compute
  - bevy
updated: 2026-08-15
sources:
  - commit 684490c (3DGS dropped)
aliases:
  - GPU bake
  - UninitBufferVec
  - compute dispatch
---

# GPU compute-shader baking and Bevy asset-pipeline integration

> **Archived (2026-09-28):** never implemented — migera's splat bake ran on the CPU and the whole splat pipeline was deleted in commits 22d3b91 (2026-08-16) and 684490c (2026-09-06).

## Why bake on the GPU rather than the CPU

The four-step bake algorithm (see
[sdf-to-splat-baking](./sdf-to-splat-baking.md)) is embarrassingly parallel — every
sample point's position, normal, curvature, orientation, scale, and (for procedural
appearance) color are computed **independently** of every other sample point, with no
cross-sample dependency until the final packing step. This is precisely the shape of
problem WGSL compute shaders are built for, and precisely the shape of problem an SDF
procedural scene description (a shader-evaluable expression tree, per
[primitive-shapes](../../sdf-3d/primitives-and-operators/primitive-shapes.md) and
[combination-operators](../../sdf-3d/primitives-and-operators/combination-operators.md))
is already naturally expressed in — the same closed-form GLSL/WGSL SDF formulas used for
raymarching shader art are directly reusable as the compute shader's SDF evaluation
function, with no translation step.

## Where this fits in Bevy's existing GPU-buffer machinery

Rather than inventing a bespoke GPU dispatch-and-readback mechanism, the bake pipeline
should be built from Bevy's existing primitives, documented in
[gpu-buffers-and-allocation](../../bevy-rendering/resources-and-assets/gpu-buffers-and-allocation.md):

- **`UninitBufferVec`** is the natural fit for the compute shader's *output* buffer — "a
  compute shader writes into it, no CPU-side mirror needed" is exactly the pattern
  described for GPU-driven visibility/culling results elsewhere in Bevy's own renderer,
  and applies identically here: the bake compute shader writes packed splat parameters
  (position, rotation, scale, opacity, and — if using procedural appearance — color)
  directly into a `UninitBufferVec<PackedSplat>`, with no CPU round-trip needed for the
  values themselves.
- **`ShaderBuffer`** (the asset-oriented raw storage buffer type, see
  [gpu-buffers-and-allocation](../../bevy-rendering/resources-and-assets/gpu-buffers-and-allocation.md))
  is the natural fit for the compute shader's *input*: the SDF scene description
  (serialized as a flat array of primitive/operator nodes — an expression-tree
  representation any WGSL SDF evaluator can walk) can be uploaded once via the standard
  `RenderAssetPlugin` extract/prepare lifecycle (see
  [render-assets](../../bevy-rendering/resources-and-assets/render-assets.md)) rather
  than through a hand-rolled upload path.
- **`RenderSystems::PrepareResources`** (part of the ordinary `Render` schedule, see
  [render-app-and-extraction](../../bevy-rendering/architecture/render-app-and-extraction.md))
  is where the bake dispatch belongs when baking needs to happen on a schedule (e.g. a
  streaming world loading new regions — see
  [streaming-and-invalidation](./streaming-and-invalidation.md)), reusing the exact
  system-set ordering guarantees (buffers prepared before `Queue`/`Render`) every other
  GPU-resource-preparing system already relies on, rather than inventing custom
  synchronization.

## The compute dispatch shape

A single compute pass, dispatched with one invocation per candidate sample point:

```wgsl
@group(0) @binding(0) var<storage, read> sdf_scene: array<SdfNode>;
@group(0) @binding(1) var<storage, read> sample_points: array<vec3<f32>>;
@group(0) @binding(2) var<storage, read_write> out_splats: array<PackedSplat>;

@compute @workgroup_size(64)
fn bake_splat(@builtin(global_invocation_id) id: vec3<u32>) {
    let p = sample_points[id.x];
    let d = eval_sdf(sdf_scene, p);              // reuse ordinary raymarching SDF eval
    let n = estimate_gradient(sdf_scene, p);      // same finite-difference technique as
                                                    // normal-estimation.md, applied once
                                                    // per sample instead of once per
                                                    // rendered pixel per frame
    let curvature = estimate_curvature(sdf_scene, p, n);

    out_splats[id.x] = pack_splat(
        p, orientation_from_normal(n), scale_from_curvature(curvature), /* ... */
    );
}
```

`eval_sdf` and `estimate_gradient` here are **exactly** the same functions a real-time
SDF raymarcher would use (see
[sphere-tracing](../../sdf-3d/rendering/sphere-tracing.md) and
[normal-estimation](../../sdf-3d/rendering/normal-estimation.md)) — the bake shader is not
a different piece of SDF-evaluation code, it's the identical scene-evaluation function
invoked once per surface sample instead of once per raymarch step per rendered pixel per
frame, which is precisely the amortization argument made in
[world-representation](../architecture/world-representation.md) for why baking beats
per-frame raymarching for this design's goals. In practice, sharing one WGSL SDF-
evaluation module between a debug/editor raymarch preview shader and the production bake
compute shader (via Bevy's
[shader-system](../../bevy-rendering/materials-and-shaders/shader-system.md) `#import`
mechanism) is both a correctness safeguard (the baked splats always match what the SDF
"actually" describes, with no drift between a separate preview and bake implementation)
and a direct engineering-effort savings.

## Sample-point generation: also GPU-side where practical

Grid/Marching-Cubes-derived sampling (see
[sdf-to-splat-baking](./sdf-to-splat-baking.md), Step 1) is itself parallelizable across
grid cells and can run as a preceding compute pass populating the `sample_points` buffer
above — the "GPU-parallelized Marching Cubes variants" already identified as the standard
production choice for runtime isosurface extraction in
[sdf-to-mesh-extraction](../../sdf-3d/mesh-conversion/sdf-to-mesh-extraction.md) apply
directly here, repurposed to emit point samples rather than triangles.

## Why not just use the CPU

For static, offline-authored worlds, CPU baking (using any of the mesh-to-SDF-adjacent
CPU acceleration structures like BVH/octree traversal, see
[exact-point-to-mesh-distance](../../sdf-3d/mesh-conversion/exact-point-to-mesh-distance.md)
for the analogous CPU-side pattern) is a perfectly reasonable choice and avoids GPU
compute-shader complexity entirely — appropriate for a build-time asset pipeline where
bake latency of seconds-to-minutes is acceptable. GPU compute baking specifically earns
its complexity when **bake latency itself matters at runtime** — live SDF editing with
fast visual feedback, or streaming newly-loaded world regions without a noticeable
loading stall (see
[streaming-and-invalidation](./streaming-and-invalidation.md)) — where sub-frame or
few-frame bake latency for a region is the actual design requirement, not just a nice-to-
have.

## When to dive in

- Building an interactive SDF-scene editor with live splat preview → GPU compute baking
  is close to a requirement, not an optimization, given the latency requirement; this
  page's dispatch shape is the starting point.
- Building a purely offline/build-time content pipeline → CPU baking is simpler to build
  and debug; only move to GPU compute baking if profiling shows bake time is actually a
  bottleneck for the target workflow.
- Wanting to guarantee the "what you see in editor preview" and "what actually got baked"
  never drift apart → share one WGSL SDF-evaluation module between both, via Bevy's
  `#import` shader-library mechanism, rather than maintaining two implementations.

## Related

- [Baking a Gaussian splat cloud from an SDF](./sdf-to-splat-baking.md) — prerequisite: the algorithm being parallelized.
- [Making a local SDF edit re-bake fast enough to feel live](../live-editing/incremental-rebake.md) — applies: live editing is what would require a GPU bake.
- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — prerequisite: how compute pipelines are wired in Bevy.
- [CPU<->GPU data flow - uploads, readback, profiling](../../compute-shaders/cpu-gpu-data-flow.md) — deeper: readback and staging if the bake result must reach the CPU.
- [GPU buffer types and allocation strategies](../../bevy-rendering/resources-and-assets/gpu-buffers-and-allocation.md) — prerequisite: `UninitBufferVec` and friends.
