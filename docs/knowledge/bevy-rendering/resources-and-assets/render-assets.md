---
title: "RenderAsset: the CPU-asset to GPU-resource lifecycle"
description: Bevy 0.19.1's RenderAsset trait and RenderAssetPlugin turn main-world assets (Mesh, Image) into GPU resources in RenderAssets<A> via extract then prepare, RenderAssetUsages controls whether CPU copies are kept, and TextureCache serves transient textures. Read before adding a GPU-backed asset type.
type: reference
status: current
tags:
  - bevy
  - assets
  - render-pipeline
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_render-0.19.1/src/render_asset.rs, texture/
aliases:
  - RenderAsset
  - RenderAssetPlugin
  - RenderAssetUsages
  - GpuImage
  - TextureCache
---

# RenderAsset: the CPU-asset to GPU-resource lifecycle

## Why this exists

Bevy assets (`Mesh`, `Image`, etc.) live as plain data in the main world (`Assets<T>`).
The renderer needs GPU-resident counterparts (vertex/index buffers, textures) that must be
created on a `RenderDevice`, which only exists in the render world. `RenderAsset`
formalizes the two-step main-world → render-world → GPU pipeline and standardizes
memory-usage tradeoffs via `RenderAssetUsages`.

## The trait

```rust
pub trait RenderAsset: Send + Sync + 'static + Sized {
    type SourceAsset: Asset + Clone;
    type Param: SystemParam;
    fn asset_usage(_: &Self::SourceAsset) -> RenderAssetUsages { .. }
    fn byte_len(_: &Self::SourceAsset) -> Option<usize> { None }
    fn prepare_asset(source_asset, id, param, previous_asset: Option<&Self>)
        -> Result<Self, PrepareAssetError<Self::SourceAsset>>;
    fn unload_asset(...) { }
    fn take_gpu_data(...) -> Result<Self::SourceAsset, AssetExtractionError> { .. }
}
```

`GpuImage` (the `RenderAsset` for `Image`) is the canonical example: `Param = (SRes<
RenderDevice>, SRes<RenderQueue>, SRes<DefaultImageSampler>)`, and `prepare_asset` decides
whether to reuse the previous `Texture` (descriptor unchanged, just re-upload pixels via
`write_texture`) or allocate fresh.

## The two-phase lifecycle, driven by `RenderAssetPlugin<A, AFTER = ()>`

1. **Extract** (`extract_render_asset::<A>`, `ExtractSchedule`, set
   `AssetExtractionSystems`): reads `AssetEvent<A::SourceAsset>`, tracks
   Added/Modified/Removed(Unused) ids, and for each that needs extracting and whose
   `asset_usage()` includes `RENDER_WORLD`, clones (or *moves*, via `take_gpu_data`,
   stripping the main-world copy, if usage is `RENDER_WORLD`-only) the source asset into
   `ExtractedAssets<A>`.
2. **Prepare** (`prepare_assets::<A>`, `RenderSystems::PrepareAssets` — before
   `PrepareMeshes`/`Prepare`/`Queue`/`Render`): calls `A::prepare_asset(...)` on each newly
   extracted asset. Success → inserted into `RenderAssets<A>` (a `HashMap<AssetId<
   SourceAsset>, A>` resource — what systems actually query, e.g. `RenderAssets<GpuImage>`).
   `PrepareAssetError::RetryNextUpdate(asset)` → pushed into `PrepareNextFrameAssets<A>` to
   retry next frame (the classic "texture bytes not loaded yet" case). Uploads are
   throttled via `RenderAssetBytesPerFrameLimiter` (using `A::byte_len`) to avoid frame
   hitches from large asset bursts.

`AFTER` lets one `RenderAsset` declare a hard ordering dependency on another's prepare
system — e.g. `RenderAssetPlugin::<RenderMesh, GpuImage>` ensures mesh preparation (which
may need morph-target images) runs after image preparation.

`ErasedRenderAsset` (`erased_render_asset.rs`) mirrors this pattern for cases needing
type-erased/dynamic storage of the prepared representation (`type ErasedAsset` decoupled
from `Self`), registered/finished in two build phases rather than one.

## `RenderAssetUsages`

Defined in **`bevy_asset`** (not `bevy_render`) as a `bitflags` type with `MAIN_WORLD` and
`RENDER_WORLD` bits (default: both). Governs whether the CPU-side copy survives GPU
upload: `RENDER_WORLD`-only frees the main-world copy after extraction (saves RAM, loses
CPU read/reload ability); `MAIN_WORLD`-only means the asset never reaches the render world
at all (e.g. an intermediate image that only exists to be decoded into another).

## Texture-specific pieces

- **`GpuImage`** (`texture/gpu_image.rs`) bundles `texture`, `texture_view`, `sampler`,
  plus the original descriptors and a `had_data` flag (detects double-extraction of
  `RENDER_WORLD`-only images).
- **`TextureCache`** (`texture/texture_cache.rs`) is separate and simpler: for
  *transient*, per-frame textures (post-process intermediates, per-view shadow maps) not
  tied to an `Image` asset. A `HashMap<TextureDescriptor, Vec<CachedTextureMeta>>`;
  `get(device, descriptor)` returns a matching untaken texture or allocates new; a
  periodic sweep evicts entries unused for several frames.
- **`FallbackImage`** — a small default texture bound when a material's optional texture
  slot is empty.

## When to dive in

- Introducing a new asset type that needs a GPU-side representation → implement
  `RenderAsset` for the target GPU type, `app.add_plugins(RenderAssetPlugin::<
  YourGpuType>::default())`, consume via `Res<RenderAssets<YourGpuType>>`. This is one of
  the two most common renderer-extension entry points (the other is `AsBindGroup`/
  `Material`, itself built on this machinery for images — see
  [gpu-resources-and-device](./gpu-resources-and-device.md)).
- Needing a per-frame scratch texture (post-process target, dynamically-sized shadow map)
  → use `TextureCache`, not a hand-rolled allocation.
- Debugging "asset not ready" stalls or memory growth → check `RetryNextUpdate` handling
  and `RenderAssetUsages` on the asset in question before assuming a leak.

## Related
- [Entity sync and extraction patterns](../architecture/entity-sync-and-extraction-patterns.md) — contrast: the extraction path for components rather than assets.
- [bevy_mesh: the mesh data model](../materials-and-shaders/mesh-data-model.md) — example: the most common `RenderAsset` source type.
- [Gizmo rendering](../2d-and-ui/gizmo-rendering.md) — example: `GizmoAsset` uses this lifecycle for per-frame debug geometry.
- [RenderDevice, RenderContext, and bind groups](./gpu-resources-and-device.md) — deeper: the device calls `prepare_asset` makes.
- [Native integration with Bevy's render pipeline](../../sdf-3dgs-bevy-integration/architecture/bevy-pipeline-integration.md) — applies: GPU-resident splat buffers as a custom `RenderAsset`.
