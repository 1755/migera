---
title: RenderDevice, RenderContext, and bind groups
description: Bevy 0.19.1 wraps wgpu::Device as RenderDevice (create_shader_module is unsafe and unvalidated), gives pass systems RenderContext for command encoding and begin_tracked_render_pass, and derives bind groups with AsBindGroup. Read when writing a pass system, a custom Material, or hitting bind-group-layout mismatches.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - materials
  - gpu-compute
  - debugging
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_render-0.19.1/src/renderer/, render_resource/bind_group.rs
aliases:
  - RenderDevice
  - RenderContext
  - AsBindGroup
  - bind group layout
  - wgpu device
---

# RenderDevice, RenderContext, and bind groups

## RenderDevice

A `Clone`-able `Resource` wrapping `WgpuWrapper<wgpu::Device>` (`WgpuWrapper` is a
`!Send`-safety-hatch newtype asserting single-threaded/pinned usage per Bevy's thread
model). Re-exposes most `wgpu::Device` creation methods (`create_bind_group`,
`create_buffer`, `create_texture`, `create_shader_module`, `create_command_encoder`,
`poll`, ...) but returns Bevy's thin wrapper types instead of raw wgpu types, so the rest
of the crate never touches `wgpu::Device` directly.

`create_shader_module` is `unsafe` (bypasses wgpu's shader validation for performance —
the safety contract is on the caller); `create_and_validate_shader_module` performs full
validation. `PipelineCache`'s module loading picks between them based on a
`ValidateShader` setting.

## RenderContext

The `SystemParam` render systems use to record GPU commands — this is what pass systems
(see [render-graph-as-systems](../architecture/render-graph-as-systems.md)) receive
instead of the old `Node` trait's context parameter.

- Backed by a `SystemBuffer`: command buffers accumulated during a system are flushed into
  a shared `PendingCommandBuffers` resource at the end of that system's execution, in
  topological system order — not immediately.
- `command_encoder()` lazily creates a `wgpu::CommandEncoder` on first use per system.
- **`begin_tracked_render_pass(descriptor)`** is the standard way to open a render pass: it
  ensures a device/encoder exist, calls `command_encoder.begin_render_pass(...)`, and
  wraps the result in a `TrackedRenderPass` (from `render_phase`) — a bookkeeping wrapper
  that dedupes redundant bind/pipeline/vertex-buffer `set_*` calls to reduce driver
  overhead.
- `add_command_buffer(cmd)` appends an already-finished `CommandBuffer` directly, flushing
  any pending encoder first to preserve submission order.

**Submission**: finished encoders are pushed into `PendingCommandBuffers`.
`FlushCommands::flush()` (used in the root `RenderGraph` schedule's `Submit` set) drains
and calls `RenderQueue::submit(buffers)`. `render_system` itself does one more direct
encoder + submit pass after running the graph, specifically for screenshot/GPU-readback
commands.

## BindGroup / BindGroupLayout

Thin, `Clone`-able, `Send + Sync` wrappers around wgpu types, each carrying an `AtomicId`
for cheap equality/hash comparisons without comparing the wgpu handle. They `Deref` to the
underlying wgpu type. `BindGroupLayout`s are normally created through
`RenderDevice::create_bind_group_layout`, but `PipelineCache` also owns an internal
deduplicating cache keyed on `BindGroupLayoutDescriptor` so multiple materials describing
the same layout shape reuse one wgpu object.

## AsBindGroup

Defined directly in **`bevy_render`**, at `render_resource/bind_group.rs` — not in
`bevy_material`, despite that crate owning most other material-adjacent plumbing (see
[materials-and-shaders](../materials-and-shaders/INDEX.md)). Its derive macro is the
standard way custom `Material` types declare their GPU bindings via field attributes:

```rust
#[derive(AsBindGroup)]
struct CoolMaterial {
    #[uniform(0)]
    color: LinearRgba,
    #[texture(1)]
    #[sampler(2)]
    texture: Handle<Image>,
}
```

Key methods: `as_bind_group(...)` (default-implemented via `unprepared_bind_group` +
`render_device.create_bind_group`), `unprepared_bind_group` (returns binding-index →
`OwnedBindingResource` pairs; implementors needing something outside the simple per-
binding model, like bindless texture arrays, can opt into
`AsBindGroupError::CreateBindGroupDirectly`), and `bind_group_layout`/
`bind_group_layout_entries`. Threads a `RenderDevice`, `PipelineCache` (for layout dedup),
and an associated `Param: SystemParam` (typically `SRes<RenderAssets<GpuImage>>` +
`SRes<FallbackImage>`) through so texture handles resolve to `GpuImage`s.

Returning `AsBindGroupError::RetryNextUpdate` is the standard way to say "texture asset
not loaded yet" — propagates up through material prepare systems as
`PrepareAssetError::RetryNextUpdate` (see [render-assets](./render-assets.md)).

## When to dive in

- Writing a custom render-graph pass/system → use `RenderContext::
  begin_tracked_render_pass`; rarely touch `RenderDevice` creation methods directly except
  when pre-building resources in `Prepare`-stage systems.
- Writing a custom `Material` → implement/derive `AsBindGroup`; you'll interact with
  `BindGroup`/`BindGroupLayout` only indirectly through the derive macro in almost all
  cases.
- Seeing "bind group layout mismatch" or similar wgpu validation errors → check whether
  your bind group layout is actually being deduplicated as expected via the
  `PipelineCache` layout cache, or whether two logically-different layouts are colliding.

## Related
- [PipelineCache and pipeline specialization](./pipeline-cache-and-specialization.md) — deeper: shader-module validation choice and bind-group-layout caching.
- [GPU buffer types and allocation](./gpu-buffers-and-allocation.md) — deeper: lower-level buffers when `AsBindGroup` is not enough.
- [StandardMaterial and the Material trait](../pbr-and-lighting/standard-material-and-pbr.md) — applies: the main consumer of `AsBindGroup`.
- [The render graph is gone](../architecture/render-graph-as-systems.md) — prerequisite: pass systems that receive `RenderContext`.
- [Compute shaders in Bevy 0.19](../../compute-shaders/bevy-integration.md) — applies: bind groups and dispatch for migera's compute passes.
