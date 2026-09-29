---
title: Rasterizer implementations across platforms
description: Surveys splat rasterizer implementations as of 2025-2026: gsplat (CUDA, Nerfstudio backend, up to 4x less training memory), and WebGPU browser renderers reported 60-135x faster than WebGL, with mobile WebGPU still rolling out. Read when choosing a training backend or building a web splat viewer.
type: research
status: current
tags:
  - 3dgs
  - rasterization
  - tooling
  - performance
  - state-of-the-art
updated: 2026-08-15
sources:
  - gsplat (Nerfstudio project)
aliases:
  - gsplat
  - WebGPU
  - WebGL
  - Nerfstudio
  - CUDA
---

# Rasterizer implementations across platforms

The original 3DGS paper shipped a reference CUDA rasterizer, but the ecosystem has since
diversified substantially — different implementations targeting different tradeoffs
between memory efficiency, developer ergonomics, and deployment platform (native GPU,
web browser, mobile).

## gsplat: the widely-adopted CUDA reimplementation

`gsplat` is an open-source, CUDA-accelerated differentiable rasterization library with
Python bindings, directly inspired by but not identical to the original paper's reference
implementation. It's used as the rasterization backend for Nerfstudio (a widely-used
research/production framework for radiance-field methods) and several downstream
projects. Compared to the original reference implementation, `gsplat` reports up to **4x
less training memory footprint** and up to **15% less training time** on standard
benchmark captures (Mip-NeRF 360 scenes), while adding features the original didn't
have: batch rasterization, N-dimensional feature rendering (rendering arbitrary per-
Gaussian feature vectors, not just RGB color — useful for semantic/feature-field
extensions), depth rendering, sparse gradients, and multi-GPU distributed rasterization.

The memory and speed improvements matter in practice specifically because training
memory (see
[training-loop-and-loss](../optimization-and-training/training-loop-and-loss.md)) is a
real constraint when working with high-Gaussian-count scenes — a more memory-efficient
rasterizer directly translates into being able to train larger/higher-quality scenes on
the same hardware, or training on more modest GPUs at all.

## WebGPU-based browser rendering

Rendering 3DGS scenes directly in a web browser (no native app install required) has
matured substantially, driven by WebGPU's compute-shader capabilities, which allow the
sorting and rasterization work to happen fully on the GPU rather than requiring CPU
involvement:

- **WebGPU vs. WebGL performance gap**: WebGPU-based implementations report substantial
  speedups over prior WebGL-based viewers — full GPU-resident sorting and compute-shader
  preprocessing in a WebGPU pipeline (rather than WebGL's more limited compute
  capabilities, which historically forced some sorting work onto the CPU) has been
  measured at 60-135x faster than the best available WebGL alternatives in some
  comparisons — a genuinely dramatic gap, not a marginal improvement.
- **Practical frame times**: WebGPU-based engines report per-frame render times in the
  low single-digit to teens of milliseconds on high-end desktop GPU hardware (RTX
  4090-class), i.e. comfortably real-time.
- **Platform rollout status (as of 2025-2026)**: WebGPU support is well-established on
  desktop browsers but is still rolling out on mobile Safari and Chrome Android through
  2025-2026 — desktop remains the primary reliable target for browser-based 3DGS
  viewing/training as of this writing, with mobile support improving but not yet uniform.
- **Browser memory constraints**: typical browser memory caps (commonly in the 2-8 GB
  range depending on browser/platform) mean medium-scale scenes work comfortably in-
  browser, while very large scenes (city-scale captures, extremely high Gaussian counts)
  may require the LOD/streaming techniques discussed in
  [large-scene-techniques](../performance-and-compression/large-scene-techniques.md)
  rather than attempting to load an entire scene into browser memory at once.
- **In-browser training**: beyond just rendering, some projects (e.g. Google DeepMind's
  Brush) demonstrate both training *and* rendering entirely within a browser via WebGPU
  — a notable capability since it removes the need for any native GPU-compute
  environment (CUDA, a Python environment, etc.) to go from captured photos to a viewable
  trained scene, at the cost of the performance and tooling maturity a native CUDA
  pipeline still generally offers.

## Practical implementation choice

| Need | Implementation approach |
|---|---|
| Research/production training pipeline, Python ecosystem integration | `gsplat` (or the original reference CUDA implementation) |
| Maximum training memory efficiency on constrained GPU hardware | `gsplat` specifically, given its documented memory-footprint improvements |
| Zero-install web deployment, viewer-only | WebGPU-based viewer (dramatically faster than WebGL alternatives) |
| Broadest browser/device compatibility including older browsers without WebGPU | WebGL-based viewer, accepting the performance penalty |
| Fully in-browser capture-to-viewing pipeline with no native tooling | WebGPU-based training+rendering (e.g. Brush-style projects), while accepting this is a less mature path than native CUDA training |

## When to dive in

- Choosing an implementation to build a training pipeline on top of → `gsplat` is the
  well-supported, actively-maintained default for most research/production use given its
  memory and speed advantages over the original reference code.
- Building a web-deployed viewer for captured scenes → prioritize WebGPU over WebGL given
  the scale of the performance gap, but confirm target-audience browser/device support
  first, especially if mobile users are a significant part of the audience.
- Deploying to mobile specifically → verify current WebGPU mobile rollout status at
  implementation time, since this was actively changing through 2025-2026 and a decision
  made early in that window may need revisiting.

## Related

- [The tile-based rasterizer](./tile-based-rasterizer.md) — prerequisite: the algorithm these libraries implement.
- [Level-of-detail, streaming, and large-scene techniques](../performance-and-compression/large-scene-techniques.md) — applies: needed when a scene exceeds browser memory caps.
- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: the wgpu/WebGPU compute path a Bevy splat renderer would use.
