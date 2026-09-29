---
title: Performance and Compression
description: How to make trained 3DGS scenes shippable, as of 2025-2026: pruning and quantization for size (hundreds of MB to GB at full precision), and explicit LOD plus streaming for distance-dependent render cost at large scale. Read before distributing a scene or rendering city-scale content.
type: index
status: current
tags:
  - 3dgs
  - compression
  - lod
  - performance
updated: 2026-09-28
---

# Performance and Compression

A full-precision trained 3DGS scene is large, and large scenes render at inconsistent
cost across viewing distances. Neither problem is solved by the base method; both have
well-characterized fixes worth applying by default. The two levers are independent:
compress each scene, and structure large scenes for LOD-aware rendering.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Compression techniques](./compression-techniques.md) | Pruning plus quantization are complementary; LightGaussian ~15x with faster rendering, TC3DGS ~67x for dynamic scenes | Preparing any trained scene for web, mobile, streaming or lower VRAM |
| [Level-of-detail, streaming, and large-scene techniques](./large-scene-techniques.md) | Octree-GS anchor LOD, streaming to exceed memory, LapisGS layered streaming for dynamic content | Render speed varies with camera distance, or the scene is city-scale |
