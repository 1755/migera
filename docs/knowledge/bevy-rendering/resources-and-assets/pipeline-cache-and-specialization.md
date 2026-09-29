---
title: PipelineCache and pipeline specialization
description: Bevy 0.19.1's PipelineCache returns a pipeline ID at once and compiles off-thread, so a new pipeline can take frames to appear; specialization (SpecializedRenderPipeline/SpecializedMeshPipeline or the newer Specializer) memoizes variants by key. Read before adding shader variants or on first-draw hitches.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - performance
  - wgsl
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_render-0.19.1/src/render_resource/pipeline_cache.rs, pipeline_specializer.rs
aliases:
  - PipelineCache
  - SpecializedRenderPipeline
  - SpecializedMeshPipeline
  - Specializer
  - pipeline compile stall
  - shader variants
---

# PipelineCache and pipeline specialization

## Why this exists

Compiling a `wgpu::RenderPipeline` (and the shader modules it depends on) is slow — often
milliseconds. Blocking the render thread on it would stall frames. `PipelineCache`
(`bevy_render::render_resource::pipeline_cache`) lets systems *request* a pipeline
synchronously (getting an ID back immediately) while the actual GPU object is built
off-thread and polled to completion over subsequent frames.

Separately, naively re-queuing a full pipeline descriptor every time a slightly different
variant is needed (with/without a shader define, different vertex layout, different blend
mode) would explode compile time and GPU memory with duplicates.
**Specialization** solves this: a small, hashable key represents "which variant," and the
result is memoized so identical requests reuse the same in-flight/compiled pipeline.

## How PipelineCache works

Calling `queue_render_pipeline(descriptor)` / `queue_compute_pipeline(descriptor)` does
**not** touch the GPU — it pushes a `CachedPipeline { descriptor, state: Queued }` into an
internal `Mutex<Vec<CachedPipeline>>` and immediately returns a `CachedRenderPipelineId`
(an index). This method takes `&self`, so it's callable from parallel systems.

`CachedPipelineState` is a small state machine: `Queued → Creating(Task<Result<Pipeline,
ShaderCacheError>>) → Ok(Pipeline) | Err(...)`.

Every frame, `PipelineCache::process_pipeline_queue_system` runs — chained immediately
before `render_system` inside `RenderSystems::Render` (see
[render-app-and-extraction](../architecture/render-app-and-extraction.md)) — and drives
each queued/creating pipeline forward:

- For a `Queued` pipeline: resolves shader module(s) via a shared `ShaderCache` (compiling
  on cache miss), resolves bind group layouts/`PipelineLayout` via an internal dedup
  cache, wraps `device.create_render_pipeline(...)` in an async future, and spawns it on
  `AsyncComputeTaskPool` (native multi-threaded builds) — or blocks immediately on wasm/
  single-threaded builds or when `synchronous_pipeline_compilation` is set.
- For a `Creating` pipeline: non-blocking poll (`check_ready`); on success the state
  becomes `Ok`; recoverable errors (`ShaderNotLoaded`, `ShaderImportNotYetAvailable`) are
  retried by resetting to `Queued`; genuine compile errors are terminal and logged.

`get_render_pipeline_state(id)` / `get_render_pipeline(id)` let a render system check
readiness — `Option<&RenderPipeline>` is `None` until compiled, which is the standard
pattern for skipping draw calls whose pipeline isn't ready yet.

Shader hot-reload flows through the same mechanism: `PipelineCache::extract_shaders` (an
`ExtractSchedule` system) watches `AssetEvent<Shader>` and marks dependent pipelines
`Queued` again on change.

## Two specialization APIs

1. **`SpecializedRenderPipeline`/`SpecializedComputePipeline`** (older, simple):
   `fn specialize(&self, key: Self::Key) -> RenderPipelineDescriptor`.
   `SpecializedRenderPipelines<S>` (a `Resource`) wraps `HashMap<S::Key,
   CachedRenderPipelineId>` and only calls `queue_render_pipeline` on cache miss.

   `SpecializedMeshPipeline` is the mesh-aware variant:
   `specialize(&self, key, layout: &MeshVertexBufferLayoutRef) -> Result<...>` — mesh
   pipelines also depend on the mesh's actual vertex layout. `SpecializedMeshPipelines<S>`
   caches on `(MeshVertexBufferLayoutRef, Key)`.

2. **`Specializer<T>`** (newer, composable, `render_resource/specializer.rs`): lets
   `#[derive(Specializer)]` combine several independent specializers (e.g. one for MSAA
   sample count, one for shadow settings) into a struct whose keys compose into a tuple
   automatically. `SpecializerKey::IS_CANONICAL` distinguishes keys where two different
   values always produce different pipelines from non-canonical ones needing a secondary
   dedup pass on the resulting descriptor.

## When to dive in

- Writing a custom `Material`/render feature with more than one shader variant (shader
  defines, MSAA, bindless mode) → implement `SpecializedMeshPipeline` (mesh materials) or
  `SpecializedRenderPipeline` (full-screen/post-process passes), store a
  `SpecializedRenderPipelines<T>`/`SpecializedMeshPipelines<T>` resource, and call
  `.specialize(&pipeline_cache, self, key)` in a `Queue`/`Prepare` system.
- Seeing dropped frames on first draw of a new material/effect → that's expected async
  compile latency; check `get_render_pipeline_state` handling rather than assuming a bug.
- See [render-phases-and-batching](../architecture/render-phases-and-batching.md) for how
  `DirtySpecializations` avoids re-specializing every visible entity every frame.

## Related
- [Render phases and batching](../architecture/render-phases-and-batching.md) — deeper: change-list re-specialization per entity.
- [bevy_shader: WGSL, preprocessing, and the shader cache](../materials-and-shaders/shader-system.md) — prerequisite: shader defs and `ShaderCache` that variants compile from.
- [bevy_material: shared material plumbing](../materials-and-shaders/material-system.md) — deeper: the type-erased material keys fed to specialization.
- [RenderDevice, RenderContext, and bind groups](./gpu-resources-and-device.md) — contrast: validated versus unchecked shader-module creation.
- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: queuing compute pipelines and waiting on their async state.
