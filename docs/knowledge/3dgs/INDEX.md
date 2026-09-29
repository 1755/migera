---
title: 3D Gaussian Splatting
description: External 3DGS research as of 2025-2026 (migera does not use splats): the Gaussian primitive, SfM-based training, the tile rasterizer and ray-traced alternative, artifacts and capture, compression and LOD, 2DGS/4DGS and game-engine status. Read before any splat capture, training, rendering or evaluation work.
type: index
status: current
tags:
  - 3dgs
  - rasterization
  - prior-art
  - state-of-the-art
updated: 2026-09-28
---

# 3D Gaussian Splatting

3D Gaussian Splatting (3DGS) reframed real-time photorealistic novel-view synthesis
around an explicit, differentiable, directly rasterizable primitive — a set of
anisotropic 3D Gaussians — instead of the implicit, ray-marched neural fields (NeRF)
that preceded it. That one choice explains both its rendering speed and most of its
characteristic failure modes. This tree is external research grounded in the original
SIGGRAPH 2023 paper (Kerbl et al.) and follow-up work through 2025-2026; the
state-of-the-art notes are dated and should be re-checked before relying on them.

**Status in migera (2026-09-28):** migera does not use splatting. A splat renderer was
built and then removed twice — the SDF-to-splat demo in commit 22d3b91 (2026-08-16) and
the hybrid renderer's splat far tier in commit 684490c (2026-09-06). This tree is
reference material; migera's own splat design history is archived in
[sdf-3dgs-bevy-integration](../sdf-3dgs-bevy-integration/INDEX.md).

## Start here

New to 3DGS: read [fundamentals](./fundamentals/INDEX.md) first. Knowing what a
Gaussian stores and why it is rasterized rather than ray-marched explains nearly
everything downstream. Otherwise jump to the topic that matches the task.

## Key facts

1. A 3DGS scene is an explicit list of primitives, not a neural network, so it can be rasterized and edited directly — [What is 3D Gaussian Splatting?](./fundamentals/what-is-3dgs.md)
2. Rendering speed comes from one depth sort per 16×16 screen tile instead of one per pixel — [The tile-based rasterizer](./rendering-and-rasterization/tile-based-rasterizer.md)
3. Training changes the primitive count itself (clone/split/prune); the original heuristic's gradient collision was fixed by AbsGS — [Adaptive density control](./optimization-and-training/adaptive-density-control.md)
4. Most quality problems, floaters above all, start at capture and SfM initialization, not in training hyperparameters — [Capture best practices](./quality-and-artifacts/capture-best-practices.md)
5. Standard 3DGS is not a geometry representation; clean meshes need 2DGS or SuGaR-style regularization — [2D Gaussian Splatting and mesh extraction](./state-of-the-art/2d-gaussian-splatting.md)
6. Compression is close to free: published results show 10-25x+ with near-imperceptible loss, sometimes with faster rendering — [Compression techniques](./performance-and-compression/compression-techniques.md)

## Topics

| Topic | What it establishes | Read when |
|---|---|---|
| [Fundamentals](./fundamentals/INDEX.md) | What a Gaussian stores (position, covariance, opacity, SH), the projection math, and the explicit-rasterization vs. implicit-ray-marching contrast with NeRF | Always first; before any other topic |
| [Optimization and Training](./optimization-and-training/INDEX.md) | The capture-to-trained-scene pipeline: SfM initialization, the L1 + D-SSIM training loop, adaptive density control and its gradient-collision weakness | Setting up training, or debugging poor convergence, floaters or blur |
| [Rendering and Rasterization](./rendering-and-rasterization/INDEX.md) | The sort-once-per-tile rasterizer, the 3DGRT ray-tracing alternative, and implementations (gsplat, WebGPU) | Implementing or choosing a splat renderer, or needing shadows/reflections on splats |
| [Quality and Artifacts](./quality-and-artifacts/INDEX.md) | Floaters, popping, needles, SH colour outliers and zoom aliasing mapped to root causes, plus capture practice | Any visible splat quality problem, or before a new capture |
| [Performance and Compression](./performance-and-compression/INDEX.md) | Why trained scenes are hundreds of MB to GB and how pruning, quantization, LOD and streaming fix size and distance-dependent cost | Preparing a scene for distribution, or working at city scale |
| [State of the Art](./state-of-the-art/INDEX.md) | 2DGS/SuGaR geometry, 4D dynamic scenes, and game-engine production maturity as of 2025-2026 | Needing meshes from splats, dynamic content, or a production-readiness assessment |

## See also

- [SDF + 3DGS + Bevy integration (archived)](../sdf-3dgs-bevy-integration/INDEX.md) — migera's former design for baking SDFs into splats rendered by Bevy.
- [3D Signed Distance Fields](../sdf-3d/INDEX.md) — the representation migera renders instead, by sphere tracing.
