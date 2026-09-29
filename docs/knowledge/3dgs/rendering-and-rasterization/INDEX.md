---
title: Rendering and Rasterization
description: How a trained 3DGS scene becomes pixels: the sort-once-per-tile rasterizer, 3DGRT ray tracing for shadows/reflections/non-pinhole cameras, and implementations (gsplat, WebGPU vs. WebGL) as of 2025-2026. Read before implementing or choosing a splat renderer.
type: index
status: current
tags:
  - 3dgs
  - rasterization
  - ray-tracing
  - performance
updated: 2026-09-28
---

# Rendering and Rasterization

The tile-based rasterizer is what made 3DGS real-time. Ray tracing is the emerging
alternative for effects rasterization cannot provide, and a set of implementations
(native CUDA, browser WebGPU/WebGL) is what you would actually deploy.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [The tile-based rasterizer](./tile-based-rasterizer.md) | Five steps: tile, cull, one entry per overlap, sort per tile not per pixel, blend front-to-back with early termination; also serves training | Writing or debugging a splat rasterizer, or blending/occlusion looks wrong |
| [Ray tracing Gaussians: an alternative to rasterization](./ray-tracing-gaussians.md) | 3DGRT traces icosahedral proxies in a hardware BVH for per-ray sorting, shadows, reflections and mixed mesh+splat scenes, still slower than rasterization | Needing secondary rays or non-pinhole cameras on splat content |
| [Rasterizer implementations across platforms](./rasterizer-implementations.md) | gsplat's memory/speed gains over the reference code; WebGPU 60-135x faster than WebGL; mobile WebGPU still rolling out | Choosing a training backend or building a web viewer |
