---
title: Resources and Assets
description: Bevy 0.19.1's layer between render-world ECS data and wgpu calls - async PipelineCache and specialization, the RenderAsset lifecycle, RenderDevice/RenderContext/AsBindGroup, and GPU buffer and allocation primitives. Read when writing a custom material, GPU-backed asset, compute pass, or GPU-driven feature.
type: index
status: current
tags:
  - bevy
  - render-pipeline
  - gpu-compute
  - assets
updated: 2026-09-28
---

# Resources and Assets

Once data is extracted into the render world (see [Architecture](../architecture/INDEX.md)),
it has to become GPU objects: compiled pipelines, uploaded textures and buffers, bind
groups. Most custom rendering features — materials, custom asset types, GPU-driven and
compute techniques — plug in at this layer.

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [PipelineCache and pipeline specialization](./pipeline-cache-and-specialization.md) | Async compilation (`Queued → Creating → Ok/Err`); `SpecializedRenderPipeline`/`SpecializedMeshPipeline` and the composable `Specializer`. | Adding shader variants, or a new material/effect hitches or is missing on first frames. |
| [RenderAsset: the CPU-asset to GPU-resource lifecycle](./render-assets.md) | `RenderAsset` + `RenderAssetPlugin` extract/prepare, `RetryNextUpdate`, `RenderAssetUsages`, `TextureCache`. | Adding a GPU-backed asset type, or "asset not ready" stalls and memory growth. |
| [RenderDevice, RenderContext, and bind groups](./gpu-resources-and-device.md) | `RenderDevice` wrapper (unchecked `create_shader_module`), `RenderContext::begin_tracked_render_pass`, `BindGroup`/`BindGroupLayout`, `AsBindGroup`. | Writing a pass system or custom `Material`, or bind-group-layout mismatches. |
| [GPU buffer types and allocation strategies](./gpu-buffers-and-allocation.md) | `RawBufferVec`, `UninitBufferVec`, `SparseBufferVec` and friends; `SlabAllocator`; `GpuComponentArrayBufferPlugin`; `ShaderBuffer`. | Building GPU-driven or compute features that own their buffers. |

## See also

- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: pipelines, bind groups and storage textures for migera's compute passes.
- [CPU-GPU data flow](../../compute-shaders/cpu-gpu-data-flow.md) — deeper: uploads, readback and GPU timing below Bevy's wrappers.
