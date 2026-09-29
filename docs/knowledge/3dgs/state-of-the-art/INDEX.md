---
title: State of the Art
description: What moved fastest in 3DGS since 2023, as of 2025-2026: geometry-accurate variants (2DGS, SuGaR), dynamic/4D splatting, and real production maturity in Unreal/Unity (rendering mature; relighting and physics still gaps). Read when needing meshes from splats, dynamic content, or a production-readiness check.
type: index
status: current
tags:
  - 3dgs
  - state-of-the-art
  - integration
updated: 2026-09-28
---

# State of the Art

The base 3DGS method (2023) has stayed a stable foundation: the tile-based rasterizer,
adaptive density control and the Gaussian primitive are largely unchanged in current
systems. Three directions moved fast after it: representations designed for accurate
geometry, extensions across time for dynamic content, and shipping in real production
pipelines. These notes are dated snapshots (2025-2026); re-check them before relying
on them.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [2D Gaussian Splatting and mesh extraction](./2d-gaussian-splatting.md) | 3DGS lacks geometric grounding; 2DGS flattens primitives to surface disks, SuGaR adds a surface-alignment loss plus Poisson meshing | Needing a mesh or accurate surface from splats |
| [Dynamic and 4D Gaussian Splatting](./dynamic-4d-gaussian-splatting.md) | Deformation fields over canonical Gaussians vs. native 4D primitives (better for abrupt motion), plus anchor-based and consistency-focused variants | Reconstructing video-like captures or animating splats |
| [Game engine integration and production status (2025-2026)](./game-engine-integration.md) | Unreal/Unity plugins hit ~60 fps under ~1M Gaussians on mid-range GPUs; relighting research-only, physics needs meshes | Evaluating splats for a game or VFX pipeline |
