---
title: GPU buffer types and allocation strategies
description: Bevy 0.19.1 offers a family of buffer wrappers (RawBufferVec, UninitBufferVec, SparseBufferVec, ...), SlabAllocator for variable-size sub-allocation, GpuComponentArrayBufferPlugin for one slot per entity, and ShaderBuffer assets. Read before building GPU-driven or compute features that manage their own buffers.
type: reference
status: current
tags:
  - bevy
  - gpu-compute
  - render-pipeline
  - performance
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_render-0.19.1/src/render_resource/buffer_vec.rs, buffer.rs
  - bevy_render-0.19.1/src/slab_allocator.rs
aliases:
  - RawBufferVec
  - SlabAllocator
  - storage buffer
  - GpuComponentArrayBufferPlugin
  - ShaderBuffer
---

# GPU buffer types and allocation strategies

## Why this exists

GPU-driven rendering (indirect drawing, compute-based mesh preprocessing/culling — see
[render-phases-and-batching](../architecture/render-phases-and-batching.md)) needs
several different flavors of "get CPU/GPU data into a buffer," each with different cost
tradeoffs. Bevy provides a small family of buffer wrapper types rather than one
general-purpose abstraction, because the access patterns genuinely differ.

## Buffer types (`render_resource/buffer_vec.rs`, `buffer.rs`)

- **`RawBufferVec<T: NoUninit>`** — a `Vec<T>` mirrored to a GPU buffer via
  `write_buffer`; used for index/vertex/instance data with no alignment requirements.
  `reserve` grows the underlying buffer; `write_buffer` uploads via `RenderQueue::
  write_buffer`.
- **`AtomicRawBufferVec<T: AtomicPod>`** — same idea, but backed by atomics so multiple
  threads can update *existing* elements concurrently (new pushes still need exclusive
  access). Used where per-entity GPU data is written in parallel by multiple extraction
  systems.
- **`UninitBufferVec<T: GpuArrayBufferable>`** — reserves GPU-side space (`add`/
  `add_multiple` just bump a length counter) without ever holding CPU-side values.
  Intended as an output buffer a compute shader writes into (e.g. GPU-driven visibility/
  culling results, indirect draw parameter buffers) — used pervasively in
  `batching::gpu_preprocessing` for `PreprocessWorkItem`s and indirect draw metadata.
- **`PartialBufferVec<T: NoUninit>`** — hybrid: CPU-pushed values followed by a trailing
  block of GPU-only uninitialized slots; enforces (debug builds) that CPU pushes happen
  before any uninit reservation.
- **`GpuArrayBuffer<T>`** (`gpu_array_buffer.rs`) — a higher-level enum, `Uniform(
  BatchedUniformBuffer<T>)` or `Storage(BufferVec<T>)`, chosen automatically based on
  whether the device supports storage buffers (falls back to dynamic-offset uniform buffer
  batches on old/WebGL hardware). Backs `GpuComponentArrayBufferPlugin` (below).
- **`sparse_buffer_vec.rs` / `SparseBufferPlugin`** — tracks dirty "pages" (bitset of
  `AtomicU64` words) and only reuploads changed regions when the dirty fraction is below
  ~15% (via a small compute shader that scatters changed pages); above that threshold,
  falls back to a full reupload. A "many entities changed a few bytes each" optimization,
  distinct from the slab allocator below (which handles variable-size *allocation*, not
  update-diffing).

## SlabAllocator — variable-size sub-allocation

`SlabAllocator<I: SlabItem>` (`slab_allocator.rs`) is a general-purpose variable-size
sub-allocator over a set of GPU buffers ("slabs"), built on the `offset_allocator` crate (a
Rust port of Sebastian Aaltonen's O(1) hard-real-time `OffsetAllocator`). You implement
`SlabItem` (associated `Key`, `Layout: SlabItemLayout` describing per-element byte size/
alignment and required `BufferUsages`) to describe what's being packed.

Workflow is transactional: `stage_allocation()` → `AllocationStage::allocate(key, layout)`
(repeat per item) → `commit(render_device, render_queue)`; deallocation mirrors this
(`stage_deallocation()`/`free(key)`/`commit()`). Objects too large for the slab's normal
size class bypass the packed allocator and get their own dedicated buffer
(`LargeObjectSlab`).

This is the mechanism behind `mesh::allocator::MeshAllocator`, which packs many meshes'
vertex/index data into a handful of large shared buffers rather than one buffer per mesh —
critical for GPU-driven indirect multi-draw, where all geometry needs to be addressable
from a small set of bound buffers.

## GpuComponentArrayBufferPlugin — the simple per-entity case

`GpuComponentArrayBufferPlugin<C: Component + GpuArrayBufferable>`
(`gpu_component_array_buffer.rs`) is a much narrower, higher-level convenience: it
automatically collects every entity's `C` value each frame (`Query<(Entity, &C)>`), pushes
them into a `GpuArrayBuffer<C>` resource, and writes back a per-entity index/dynamic-
offset component so shaders know "your data is at slot N." Wired into
`RenderSystems::PrepareResources`.

## `ShaderBuffer` — the asset-oriented case

`storage.rs` defines `ShaderBuffer` — a plain `Asset` (raw bytes + a `wgpu::
BufferDescriptor` + `RenderAssetUsages`) whose GPU counterpart is prepared through the
same `RenderAssetPlugin` machinery as any other asset (see
[render-assets](./render-assets.md)). This is the escape hatch for a raw, asset-driven
storage buffer (e.g. for a custom compute shader) without hand-rolling extract/prepare
systems — install `StoragePlugin` and treat `ShaderBuffer` like any other asset.

## How to choose

| Need | Use |
|------|-----|
| Content authored once, uploaded via the standard asset path | `ShaderBuffer` |
| One GPU slot per live entity, recollected every frame, no manual allocation | `GpuComponentArrayBufferPlugin` |
| Heterogeneous, changing-size data needing stable persistent GPU addresses across frames | `SlabAllocator` |
| Compute shader output buffer, no CPU-side mirror needed | `UninitBufferVec` |
| Many entities, small per-entity byte changes, want to avoid full reupload | `SparseBufferPlugin` |

## When to dive in

- Writing GPU-driven rendering features (indirect drawing, compute-based culling/
  preprocessing, per-instance data buffers) → these types are the primitives you compose.
- Writing an ordinary `Material`/mesh feature → you likely never touch these directly;
  `AsBindGroup` and `GpuComponentArrayBufferPlugin` abstract them away (see
  [gpu-resources-and-device](./gpu-resources-and-device.md)).

## Related
- [Render phases and batching](../architecture/render-phases-and-batching.md) — prerequisite: the GPU-driven batching that motivates these buffer types.
- [RenderDevice, RenderContext, and bind groups](./gpu-resources-and-device.md) — contrast: the higher-level `AsBindGroup` path most features use instead.
- [CPU-GPU data flow](../../compute-shaders/cpu-gpu-data-flow.md) — deeper: upload and readback cost trade-offs at the wgpu level.
- [GPU compute-shader baking](../../sdf-3dgs-bevy-integration/baking-pipeline/gpu-compute-baking.md) — applies: how a compute bake fits Bevy's buffer machinery.
