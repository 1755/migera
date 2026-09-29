---
title: Recent research directions (2024-2026)
description: Dated snapshot of active SDF-adjacent research — iSDF robot perception, faster generalized winding numbers, differentiable isosurfaces, SDF/Gaussian-splatting hybrids, sparse voxel rasterization, hardware trends, non-SDF GI that also ships. Read when judging how settled a technique is before betting on it.
type: research
status: current
tags:
  - sdf
  - state-of-the-art
  - 3dgs
  - ray-tracing
  - global-illumination
updated: 2026-08-15
aliases:
  - iSDF
  - SIGGRAPH
  - SDF Gaussian hybrid
---

# Recent research directions (2024-2026)

A survey of where active SDF-adjacent research is currently pushing, for readers deciding
where to look next or evaluating how mature a given technique is.

## Real-time robot perception (iSDF and successors)

Real-time neural SDF construction for robot perception (iSDF and its lineage) fits an SDF
online, incrementally, as a robot explores an environment — combining the [neural SDF](./neural-and-learned-sdfs.md)
representation's compactness and continuous-query advantages with the real-time
incremental-update requirements more traditionally associated with
[narrow-band/TSDF](../fundamentals/narrow-band-and-truncated-sdfs.md) fusion approaches.
This is a genuinely active convergence point between the neural-SDF and classical-TSDF-
fusion research lines, rather than the two remaining fully separate.

## Faster, more robust generalized winding numbers

Sign determination for imperfect meshes (see
[sign-determination-methods](../mesh-conversion/sign-determination-methods.md)) remains an
active algorithmic target — recent work like the "Antipodal Method" specifically targets
faster and more robust generalized winding number computation, indicating that even this
relatively mature technique (GWN itself dates to 2013) continues to see meaningful
algorithmic improvement rather than being a fully closed problem.

## Differentiable and neural isosurface extraction

FlexiCubes and the broader Neural Marching Cubes / Neural Dual Contouring line (see
[sdf-to-mesh-extraction](../mesh-conversion/sdf-to-mesh-extraction.md)) continue to
develop, driven by the needs of neural-SDF training pipelines that require gradients to
flow through the mesh-extraction step itself — a requirement classical Marching
Cubes/Dual Contouring were never designed to satisfy, since they predate differentiable
optimization as a design constraint entirely.

## SDF and Gaussian splatting hybrids

3D Gaussian Splatting has become a dominant technique for real-time novel-view synthesis
since 2023, and current research increasingly explores **hybrid SDF/voxel + Gaussian
representations** — for example, dual-scaffolding approaches combining a voxel/SDF-like
structure with Gaussian primitives for faster, more accurate monocular surface
reconstruction, and octree-structured Gaussian approaches for consistent level-of-detail
rendering. The throughline connecting this to SDFs specifically: Gaussian splatting alone,
like plain NeRF density fields, doesn't have an inherently crisp, well-defined *surface*
the way an SDF's zero level set does — so hybrids reintroducing SDF-like structure are a
direct response to wanting splatting's rendering speed/quality *plus* SDF's clean
extractable geometry, echoing the same motivation that drove NeuS to combine SDFs with
NeRF-style volume rendering (see [neural-and-learned-sdfs](./neural-and-learned-sdfs.md)).

## Sparse voxel rasterization for surface reconstruction

Recent work (e.g. "SVRecon: Sparse Voxel Rasterization for Surface Reconstruction")
explores rasterizing sparse voxel/SDF-adjacent structures directly, rather than either
raymarching them or converting them to triangles first — a third path distinct from both
approaches covered in
[hybrid-and-grid-rendering](../rendering/hybrid-and-grid-rendering.md), motivated by
wanting rasterization's hardware-friendly performance characteristics applied more
directly to volumetric/implicit data rather than requiring a full mesh-extraction step
first.

## Hardware acceleration trends

As noted in [hybrid-and-grid-rendering](../rendering/hybrid-and-grid-rendering.md),
dedicated ray-tracing hardware (RT cores) remains built around triangle/BVH intersection
rather than general sphere tracing. SIGGRAPH 2025-2026 research activity in adjacent areas
(RTX Mega Geometry for large-scale scene complexity, GPU-accelerated curve/tube rendering,
hardware tensor-core-accelerated neural materials via MLPs run directly in shaders) shows
the industry's hardware-acceleration investment concentrated on triangle geometry and
neural-network inference specifically, rather than on dedicated SDF/implicit-surface
hardware primitives — reinforcing that hybrid architectures (SDF for what benefits from
it, triangles/hardware RT for primary visibility) remain the pragmatic near-term path
rather than betting on future SDF-specific hardware.

## Real-time global illumination without SDFs, as context

It's worth noting that not all cutting-edge real-time GI work relies on SDFs at all — id
Software's idTech 8 (DOOM: The Dark Age) achieves real-time global illumination replacing
pre-baked lighting through other techniques entirely, and HypeHype's stochastic tile-based
lighting targets fully dynamic local lighting on low-end mobile GPUs via a different
algorithmic approach. This is useful context for calibrating how central SDFs actually are
to the current real-time-GI research landscape: important and widely deployed (Lumen), but
one technique family among several competing approaches, not an unchallenged default.

## When to dive in

- Tracking the cutting edge of SDF-adjacent research specifically → this page is a
  snapshot, not a literature review; treat named papers/techniques as starting points for
  further reading rather than exhaustive coverage, and expect this landscape to keep
  moving quickly given the pace of 2024-2026 activity in neural representations generally.
- Evaluating whether to invest in Gaussian-splatting/SDF hybrids for a reconstruction
  project → this is genuinely active, fast-moving research rather than a settled
  production technique as of 2025-2026; expect more iteration before a clear best-
  practice consolidates, unlike the more mature techniques covered elsewhere in this
  knowledge base (sphere tracing, classical mesh-to-SDF conversion, Marching Cubes).
- Deciding whether to bet a new real-time GI system's architecture on SDFs specifically →
  weigh Lumen's proven deployment against the reminder that competing non-SDF approaches
  (idTech 8, HypeHype's mobile technique) are simultaneously shipping successfully;
  SDFs are a strong, proven option, not the only one.

## Related
- [Neural and learned SDFs](./neural-and-learned-sdfs.md) — deeper: the neural representations behind several directions here.
- [SDF + 3DGS + Bevy integration](../../sdf-3dgs-bevy-integration/INDEX.md) — example: migera's own (archived) SDF/Gaussian-splat synthesis.
- [Sign determination](../mesh-conversion/sign-determination-methods.md) — prerequisite: the GWN method being accelerated.
- [Radiance cascades experiment](../../hybrid-architecture/gi-and-lighting/radiance-cascades-experiment.md) — example: a non-SDF-specific GI technique migera measured against DDGI.
