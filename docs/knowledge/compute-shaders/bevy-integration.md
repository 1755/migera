---
title: Compute shaders in Bevy 0.19
description: Establishes where compute passes live in Bevy 0.19's render-graph-as-systems model, how to build compute pipelines, bind groups and storage textures, and how to display compute output. Read before wiring any new compute pass into the render world.
type: reference
status: current
tags:
  - bevy
  - gpu-compute
  - render-pipeline
  - wgsl
  - integration
updated: 2026-08-23
verified: 2026-08-23
sources:
  - Bevy 0.19 example shader/compute_shader_game_of_life.rs
  - Bevy 0.19 example shader_advanced/compute_mesh.rs
  - Bevy 0.19 example custom_post_processing
  - Bevy 0.18->0.19 migration guide (Render Graph as Systems)
aliases:
  - storage texture
  - render graph node
  - camera_driver
---

# Compute shaders in Bevy 0.19

Verified against Bevy 0.19's own examples: `shader/compute_shader_game_of_life.rs`,
`shader_advanced/compute_mesh.rs`, `shaders/post_processing/custom_post_processing`,
and the 0.18->0.19 migration guide ("Render Graph as Systems"). Note: older tutorials
(and our own earlier research notes) describe the `Node`-trait render graph - in 0.19
render passes are **systems** scheduled on/around the `RenderGraph` schedule.

## Where compute work runs (0.19 model)

```rust
let render_app = app.get_sub_app_mut(RenderApp).unwrap();
render_app.add_systems(RenderStartup, init_compute_pipeline);
render_app.add_systems(Render, prepare_buffers.in_set(RenderSet::PrepareResources));
// View-independent compute, before any camera renders:
render_app.add_systems(RenderGraph, my_compute_system.before(camera_driver));
// Or per-view post-processing style, inside the 3d core set:
render_app.add_systems(Render, my_view_system.in_set(Core3dSystems::PostProcess));
```

- `RenderGraph`-schedule systems run before `camera_driver`; use for scene-global
  passes (bake, simulation). `compute_mesh.rs` is the canonical example.
- Per-view work that must interleave with the camera pipeline uses `Core3d` sets
  (`Core3dSystems::Prepass/MainPass/PostProcess/...`) with a `ViewQuery` - see the
  custom-post-processing example for the full pattern (it also shows manual
  `begin_render_pass` writing into `ViewTarget::post_process_write()`).

## Pipeline & bind groups

```rust
let layout = BindGroupLayoutDescriptor::new("...", &BindGroupLayoutEntries::sequential(
    ShaderStages::COMPUTE,
    (uniform_buffer::<Params>(false),
     storage_buffer::<Vec<Record>>(false /*read*/),   // true = read_write
     texture_storage_2d(Some(Rgba16Float), StorageAccess::ReadWrite)),
));
let id = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
    label: Some("...".into()), layout: vec![layout.clone()],
    shader: asset_server.load(SHADER_PATH), entry_point: "main".into(), ..default()
});
```

- Queue at `RenderStartup`/`FromWorld`; dispatch only after
  `pipeline_cache.get_compute_pipeline(id)` returns `Some` (async compilation).
- Bind groups: build once per frame in `RenderSet::PrepareBindGroups`, or per-dispatch
  inline (compute_mesh does the latter for per-target uniforms).
- Push constants exist via `ComputePipelineDescriptor.push_constant_ranges` but need
  device feature; prefer uniform buffers.

## Storage textures (the raymarcher-relevant path)

An `Image` can be bound read/write by compute and sampled later:

```rust
Image::new_fill(extent, TextureDimension::D2, &[0,0,0,255], TextureFormat::Rgba16Float,
                RenderAssetUsages::RENDER_WORLD)
// required usages:
image.texture_descriptor.usage = TextureUsages::STORAGE_BINDING
                               | TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST;
```

- Declare on an `AsBindGroup` resource: `#[storage_texture(0, image_format = Rgba16Float, access = ReadWrite)]`
  (game_of_life pattern) - or hand-write the layout entry as above.
- Extract main-world handle into render world via
  `ExtractResourcePlugin::<MyImageResource>::default()`.
- Format support is driver-dependent (Rgba8Unorm widely supported; Rgba16Float
  storage needs checking per adapter - query `render_device` features; fall back to
  buffer-backed output + copy if unavailable).
- WGSL side: `@group(0) @binding(2) var out_tex: texture_storage_2d<rgba16float, write>;`
  written with `textureStore(out_tex, coord, color);`

## Displaying compute results

1. **Sprite** (debug/tooling): spawn a `Sprite` using the same image handle -
   game_of_life does this; zero extra passes.
2. **Fullscreen blit into the view target**: small fragment pass sampling the storage
   texture into `ViewTarget::post_process_write()` destination - custom-post-processing
   example pattern. This keeps Tonemapping/bloom/etc. downstream working. This is the
   shape migera's compute raymarcher should take (see raymarching-via-compute).
3. **Direct write into view target**: possible (color attachment of a render pass) but
   pointless for compute output - you cannot render-compute into one pass; the blit
   costs ~nothing next to a raymarcher.

## GPU->GPU chaining

Compute writes storage texture -> fragment samples it: implicit barriers are inserted
by wgpu between passes; no manual sync needed within a frame's encoder. Multiple
compute passes in one encoder chain naturally (game_of_life ping-pongs two textures).
Avoid CPU readback purely to feed another GPU pass - keep it on-GPU.

## Pitfalls verified from examples/migration guides

- `get_compute_pipeline()` readiness check before every dispatch (first frames).
- `RenderAssetUsages::RENDER_WORLD` for GPU-only textures avoids main-world copies.
- MeshAllocator slabs can be bound directly as storage when
  `MeshAllocatorSettings.extra_buffer_usages` includes STORAGE (compute_mesh).
- The old `impl Node` graph still exists but new code should use schedule systems;
  mixing both in one sub_graph requires slot-edge plumbing (Simon Ekstrom's deferred-
  compute write-up predates 0.19 - adapt concepts, not code).

## Related
- [The render graph is gone: render-graph-as-systems](../bevy-rendering/architecture/render-graph-as-systems.md) — deeper: the full 0.19 schedule model these compute systems plug into.
- [Camera-driven scheduling: Core3d and Core2d](../bevy-rendering/core-pipeline/camera-driven-scheduling.md) — deeper: the `Core3dSystems` sets used for per-view compute work.
- [PipelineCache and pipeline specialization](../bevy-rendering/resources-and-assets/pipeline-cache-and-specialization.md) — deeper: why `get_compute_pipeline` returns `None` for the first frames.
- [CPU↔GPU data flow](./cpu-gpu-data-flow.md) — deeper: uploads, async readback and timestamp profiling for the passes wired here.
- [Raymarching via compute](./raymarching-via-compute.md) — applies: the compute-trace + fullscreen-blit shape (option 2 above) proposed for migera's marcher.
- [Module and stage skeleton](../hybrid-architecture/module-and-stage-skeleton.md) — applies: how `src/hybrid` actually places its compute trace in `Core3dSystems::MainPass`.
