---
title: Clustered forward rendering — now on the GPU
description: Bevy 0.19.1 assigns point/spot lights, probes and decals to a froxel grid (ClusterConfig modes None/Single/XYZ/FixedZ) so each fragment loops only its cluster's lights; 0.19 moved clustering to the GPU (rasterizer as intersection tester plus prefix sum, ~20x faster). Read when light count hurts or lights miss.
type: reference
status: current
tags:
  - bevy
  - lighting
  - culling
  - gpu-compute
  - performance
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_light-0.19.1/src/cluster/mod.rs
  - bevy_pbr-0.19.1/src/cluster/
  - Bevy 0.19 release notes (GPU clustering speedup figure)
aliases:
  - froxels
  - ClusterConfig
  - GPU light clustering
  - clustered forward shading
---

# Clustered forward rendering — now on the GPU

## The algorithm

Documented in `bevy_light::cluster::mod.rs` (citing aortiz.me and Doom 2016's Siggraph
talk as lineage): the camera's view frustum is divided into a 3D grid of **clusters**
("froxels" — frustum-shaped voxels). X/Y tiling follows screen-space tiles; Z-slicing uses
a special near-plane-favoring first slice (`ClusterZConfig::first_slice_depth`) followed
by either a fixed depth (`ClusterFarZMode::Constant`) or a size derived from last frame's
visible light range (`MaxClusterableObjectRange`). `ClusterConfig` exposes four modes:
`None` (disable), `Single` (one cluster — good for globally-affecting lights), `XYZ {
dimensions, .. }` (explicit grid), `FixedZ { total, z_slices, .. }` (square X/Y clusters
computed from a total budget + fixed Z-slice count, useful for top-down games with shallow
depth ranges).

Once clusters exist, every "clusterable object" (point lights, spot lights, light probes,
decals — tagged via `ClusterVisibilityClass`) is tested against each cluster's AABB and,
on intersection, appended to that cluster's index list. At shading time, the fragment
shader looks up its cluster (from screen position + view-space depth) and only iterates
the lights in that cluster's index list — turning an O(lights × fragments) per-pixel light
loop into something closer to O(lights-per-cluster × fragments).

## The 0.19 change: GPU clustering

`bevy_light::cluster::GlobalClusterSettings` now carries an `Option<
GlobalClusterGpuSettings>`; when `Some`, clustering runs entirely on GPU via
`bevy_pbr::cluster::gpu::GpuClusteringPlugin`, replacing the CPU AABB-intersection loop
for most platforms (disabled on Android/iOS-simulator or when storage buffers/compute
aren't available — `render_device.limits().max_storage_buffers_per_shader_stage == 0`).

The module doc lays out a five-stage pipeline that cleverly repurposes the **rasterizer**
as a parallel intersection tester instead of using compute shaders directly for the
geometry test:

1. **Z-slicing** (`cluster_z_slice.wgsl`, compute, workgroup size 64) — generates D
   indirect draw instances per clusterable object, one per Z-slice the object's range
   spans.
2. **Count rasterization** (`cluster_raster.wgsl`) — indirect-instanced rasterization of
   W×H quads per Z-slice with color writes off; each rasterized fragment is a (cluster,
   object) pair, and the fragment shader atomically increments a per-cluster counter if
   the object truly intersects that cluster (no object IDs recorded yet).
3. **Local allocation** (`cluster_allocate.wgsl`, workgroup size 256) — a Hillis–Steele
   parallel prefix-sum over each chunk of ≤256 clusters (wgpu's max workgroup size) to
   compute per-cluster offsets into a shared object-index buffer.
4. **Global allocation** — a sequential pass propagating the prefix sum across 256-cluster
   chunk boundaries (step 3 can't cross that boundary).
5. **Populate rasterization** — the *same* rasterization pass as step 2, re-run; this time
   each intersecting fragment writes the object's ID into its allocated slot using a
   scratch atomic buffer tracking the next free slot per cluster list.

This is the "Bevy now clusters lights on the GPU (~20x improvement)" change from the
migration guide — moving from a CPU nested-loop AABB test (bounded by single-threaded
throughput, requiring a CPU→GPU upload of resulting index lists every frame) to a GPU
pipeline that computes and consumes the cluster assignment without a round-trip, and which
parallelizes both the intersection test (via the rasterizer, one invocation per fragment)
and the list-packing (via the prefix-sum compute shaders).

`cluster.wgsl` is the shared shader library (types + the AABB/sphere intersection test
used by both raster passes); `clustered_forward.wgsl` (in `bevy_pbr/src/render/`) is what
the main PBR fragment shader includes to *consume* the resulting cluster/index buffers
when shading. CPU clustering remains as the fallback path (`assign_objects_to_clusters` in
`bevy_light::cluster::assign`) for unsupported backends.

## When to dive in

- Tuning light performance at scale (many dynamic lights) → check `ClusterConfig` mode and
  whether GPU clustering is actually active for the target platform before assuming a
  clustering bug.
- Debugging light-cluster artifacts (a light not affecting an object that should be in
  range) → check cluster grid dimensions/Z-slicing config; clusters too coarse can miss
  correct assignment at grazing angles.
- Adding a new "clusterable object" type (e.g. a custom light-like effect) → tag it with
  `ClusterVisibilityClass` following the pattern `PointLight`/`SpotLight` use.

## Related
- [bevy_light: light components, atmosphere, gizmos](./light-components-and-atmosphere.md) — prerequisite: the light components that get clustered.
- [StandardMaterial and the Material trait](./standard-material-and-pbr.md) — deeper: the `bevy_pbr` module map, including `cluster/` and `clustered_forward.wgsl`.
- [Shadow rendering](./lighting-and-shadows.md) — contrast: the other per-light cost (shadow views) that clustering does not reduce.
