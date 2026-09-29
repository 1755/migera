---
title: Native integration with Bevy's render pipeline
description: Records how migera's deleted splat renderer joined Bevy 0.19's Core3d schedule (SplatAsset RenderAsset, opaque Splat3d BinnedPhaseItem, reused Transparent3d, one shared ViewTarget) and the proven pass-ordering trap (.after opaque AND .before transparent). Read before adding any custom geometry pass to Core3d.
type: design
status: archived
tags:
  - 3dgs
  - bevy
  - render-pipeline
  - integration
  - correctness
updated: 2026-08-15
sources:
  - commit eac219e (demo built)
  - commit 22d3b91 (src/splat removed)
  - commit 684490c (3DGS dropped from src/hybrid)
aliases:
  - Splat3d
  - PhaseItem
  - Core3dSystems::MainPass
  - ViewTarget
  - Transparent3d
---

# Native integration with Bevy's render pipeline

> **Archived (2026-09-28):** the `src/splat/` code this describes was deleted in commits 22d3b91 (2026-08-16) and 684490c (2026-09-06); the Bevy mechanics (RenderAsset, phase items, `.after`/`.before` pass ordering, shared `ViewTarget`) still apply to any custom Core3d pass but re-verify them against Bevy 0.19 source.

Contents: [Design goal](#the-design-goal-precisely-stated) · [Component and asset model](#component-and-asset-model) · [RenderAsset](#renderasset-gpu-resident-splat-buffers) · [PhaseItems](#the-custom-phaseitems-splat3d-opaque--reused-transparent3d-translucent) · [Passes](#the-passes-registering-into-core3dsystemsmainpass) · [Why downstream works](#why-everything-downstream-just-works) · [When to dive in](#when-to-dive-in)

> **Updated from a working, proven implementation** (the `migera` demo in this
> repository). The original version of this page designed `Splat3d` as a single
> `BinnedPhaseItem` relying on Weighted Sum Rendering for order-independence. That
> design is now known to be wrong for opaque content — see
> [sort-free-compositing](../render-integration/sort-free-compositing.md) for the full
> story of why and what broke. The "custom `PhaseItem`" section below is corrected to
> match the proven two-pipeline architecture; the rest of this page (component/asset
> model, `RenderAsset` lifecycle, `ViewTarget` sharing) is unaffected and still
> accurate.

## The design goal, precisely stated

"Native integration" means: a baked splat cloud is a **first-class citizen of Bevy's
existing render-phase/schedule system** (see
[render-graph-as-systems](../../bevy-rendering/architecture/render-graph-as-systems.md)
and
[render-phases-and-batching](../../bevy-rendering/architecture/render-phases-and-batching.md)),
not a bolted-on side-channel with its own separate compute-dispatch-and-blit pass
disconnected from `Core3dSystems`. Concretely, this means:

1. Splat data is a `RenderAsset` (see
   [render-assets](../../bevy-rendering/resources-and-assets/render-assets.md)), going
   through the same extract → prepare lifecycle every other GPU asset uses.
2. A splat-covered entity is queued into a genuine `PhaseItem`-based render phase (see
   [render-phases-and-batching](../../bevy-rendering/architecture/render-phases-and-batching.md)),
   consumed by a pass system inserted into `Core3dSystems::MainPass` alongside
   `main_opaque_pass_3d`/`main_transparent_pass_3d` — not a separate schedule, not a
   parallel `RenderGraph`-adjacent mechanism.
3. The pass writes into the **same `ViewTarget`** every other `Core3d` pass writes into
   (see
   [camera-and-view-system](../../bevy-rendering/scene-and-views/camera-and-view-system.md)),
   so everything downstream in the schedule — `EarlyPostProcess`, `tonemapping`,
   `PostProcess`, `upscaling` — sees splat-rendered pixels exactly as it would see
   mesh-rendered pixels, with **zero splat-specific code required in any post-process
   pass**. This is the mechanism that makes "reuse built-in tools with custom rendering"
   actually true rather than aspirational.

## Component and asset model

Following the same pattern `Mesh3d`/`MeshMaterial3d` establishes for ordinary meshes:

```rust
#[derive(Component)]
struct SplatCloud3d(Handle<SplatAsset>);   // main-world component, analogous to Mesh3d

// The main-world asset (analogous to `Mesh`), produced by the bake pipeline
// (see baking-pipeline/sdf-to-splat-baking.md)
#[derive(Asset, TypePath, Clone)]
struct SplatAsset {
    positions: Vec<Vec3>,
    // rotation as quaternion + scale, per gaussian-primitive-parameters.md
    rotations: Vec<Quat>,
    scales: Vec<Vec3>,
    opacities: Vec<f32>,
    sh_coefficients: Vec<[f32; 48]>,   // degree-3 SH, per Gaussian — see fundamentals doc
    asset_usage: RenderAssetUsages,
}
```

A world-space splat "cloud" entity is spawned exactly like a mesh entity:

```rust
commands.spawn((
    SplatCloud3d(splat_assets.add(baked_splats)),
    Transform::from_xyz(0.0, 0.0, 0.0),
    Visibility::default(),   // participates in ordinary Bevy visibility/culling
));
```

Because it carries `Transform`/`Visibility`, it automatically participates in Bevy's
existing visibility propagation and (with an `ExtractComponentPlugin`-driven visibility
class, see below) frustum culling — see
[camera-and-view-system](../../bevy-rendering/scene-and-views/camera-and-view-system.md)
for the `ViewVisibility`/`VisibleEntities` machinery this reuses rather than reimplements.

## RenderAsset: GPU-resident splat buffers

```rust
struct GpuSplatCloud {
    // A SlabAllocator-backed or plain storage buffer holding packed per-Gaussian data —
    // see gpu-buffers-and-allocation.md for why SlabAllocator (not a plain Vec-mirroring
    // RawBufferVec) is the right choice once many independent splat clouds need to be
    // packed into shared, indirect-draw-addressable buffers.
    buffer: Buffer,
    count: u32,
    bind_group: BindGroup,
}

impl RenderAsset for GpuSplatCloud {
    type SourceAsset = SplatAsset;
    type Param = (SRes<RenderDevice>, SRes<RenderQueue>);

    fn prepare_asset(source, id, (device, queue), previous) -> Result<Self, PrepareAssetError<SplatAsset>> {
        // pack positions/rotations/scales/opacities/SH into a tightly-laid-out storage
        // buffer (see gaussian-primitive-parameters.md for the 59-scalar-per-Gaussian
        // layout this mirrors), reusing `previous`'s buffer if size/layout is unchanged
        // — exactly the reuse-when-possible pattern GpuImage's prepare_asset follows.
        ...
    }
}

app.add_plugins(RenderAssetPlugin::<GpuSplatCloud>::default());
```

This is the standard `RenderAsset` two-phase lifecycle (see
[render-assets](../../bevy-rendering/resources-and-assets/render-assets.md)) applied
verbatim: extraction copies the `SplatAsset` into the render world on change, `prepare_
assets::<GpuSplatCloud>` (running in `RenderSystems::PrepareAssets`, ahead of `Queue`)
builds/updates the GPU buffer, and `Res<RenderAssets<GpuSplatCloud>>` is what the queue
and draw systems consume — no bespoke asset-loading side-channel.

## The custom PhaseItem(s): `Splat3d` (opaque) + reused `Transparent3d` (translucent)

> This section originally designed a single `Splat3d` `BinnedPhaseItem` relying on
> Weighted Sum Rendering for order-independence, on the reasoning that splats are
> "neither" opaque nor sortable-transparent. Building a real demo proved that framing
> wrong for the common case: **splats representing solid, opaque surfaces are exactly
> as depth-testable as any other opaque geometry** — nothing about being a splat
> (a Gaussian-falloff-shaped billboard instead of a mesh triangle) changes that. Only
> splats that are *genuinely translucent* need anything beyond a plain depth test. See
> [sort-free-compositing](../render-integration/sort-free-compositing.md) for the full
> failure story. The corrected design below splits into two pipelines by material,
> matching how ordinary opaque+transparent mesh rendering is already split.

### Opaque splats: `Splat3d`, a real depth-tested `BinnedPhaseItem`

```rust
struct Splat3d {
    representative_entity: (Entity, MainEntity),
    pipeline: CachedRenderPipelineId,
    draw_function: DrawFunctionId,
    batch_range: Range<u32>,
    extra_index: PhaseItemExtraIndex,
}

impl PhaseItem for Splat3d { /* entity/draw_function/batch_range/extra_index accessors */ }
impl BinnedPhaseItem for Splat3d {
    type BatchSetKey = Splat3dBatchSetKey;  // { pipeline, draw_function }
    type BinKey = Splat3dBinKey;            // { asset_id } — one bin per splat cloud asset
    fn new(...) -> Self { ... }
}
impl CachedRenderPipelinePhaseItem for Splat3d { ... }
```

`BinnedPhaseItem` here for the same reason `Opaque3d`/`AlphaMask3d` are binned: draw
order genuinely doesn't matter, because correctness comes from the **real hardware
depth test and write** in the pipeline (`depth_write_enabled: true`,
`depth_compare: CompareFunction::GreaterEqual` matching Bevy's reversed-Z convention),
not from any property of the compositing math. The fragment shader alpha-tests (or,
better — see below — drives alpha-to-coverage) rather than blending, so this is a
`BinnedPhaseItem` for the *ordinary* reason meshes use one, with no dependency on WSR
or any other order-independence trick.

**Alpha-to-coverage over a hard alpha-test threshold.** A binary discard-below-X /
opaque-above-X test makes each splat's own disc edge a visible hard boundary against
its neighbors (a stippled/blotchy surface). Setting
`multisample.alpha_to_coverage_enabled: true` and letting the fragment shader's real
Gaussian falloff drive MSAA sample coverage instead resolves overlapping splat edges
into a smooth blend during MSAA resolve, with depth correctness fully preserved (each
covered *sample* still writes its own true depth) — the standard fix for exactly this
class of alpha-tested-billboard artifact (foliage, particle sprites), applied here to
splats.

`ViewBinnedRenderPhases<Splat3d>` (a resource, keyed by `RetainedViewEntity` — see
[render-phases-and-batching](../../bevy-rendering/architecture/render-phases-and-batching.md))
is populated by a `queue_splat_clouds` system in `RenderSystems::Queue`, one bin entry
per (splat cloud asset, camera) pair.

### Translucent splats: reuse Bevy's own `Transparent3d`, don't invent a new type

Splats whose material is genuinely translucent (not just "has soft edges" — a
near-opaque billboard's soft antialiased edge is still opaque-pass content) route to a
**second** pipeline that queues into Bevy's existing `Transparent3d` phase directly,
unmodified. `Transparent3d`'s fields (`sorting_info`, `distance`, plain `Entity`/
`MainEntity`, `draw_function`, `batch_range`) turned out to already be generic enough
for non-mesh splat content — no bespoke sorted phase item was needed. This gets a real
per-instance back-to-front sort "for free" from Bevy's existing
`sort_phase_system::<Transparent3d>`, depth-tested (not written) against whatever the
opaque pass (splats or ordinary meshes) already wrote to the shared depth buffer.

## The passes: registering into `Core3dSystems::MainPass`

```rust
render_app.add_systems(Core3d,
    splat_opaque_pass
        .after(main_opaque_pass_3d)
        .before(main_transparent_pass_3d)   // see the warning below — both bounds are required
        .in_set(Core3dSystems::MainPass)
);
// Translucent splats need no separate pass system at all: they're queued directly
// into Transparent3d, so Bevy's own main_transparent_pass_3d draws them.

fn splat_opaque_pass(
    world: &World,
    view: ViewQuery<(&ExtractedView, &ViewTarget, &ViewDepthTexture, &ViewUniformOffset)>,
    splat_phases: Res<ViewBinnedRenderPhases<Splat3d>>,
    mut ctx: RenderContext,
) {
    let (extracted_view, target, depth, _) = view.into_inner();
    let Some(splat_phase) = splat_phases.get(&extracted_view.retained_view_entity) else { return };

    let mut render_pass = ctx.begin_tracked_render_pass(RenderPassDescriptor {
        label: Some("splat_opaque_pass"),
        color_attachments: &[Some(target.get_color_attachment())],
        depth_stencil_attachment: Some(depth.get_attachment(StoreOp::Store)),  // real write, shared buffer
        ..default()
    });
    splat_phase.render(&mut render_pass, world, view.entity())?;
}
```

This is structurally identical to `main_opaque_pass_3d` (see
[render-graph-as-systems](../../bevy-rendering/architecture/render-graph-as-systems.md)
for that reference implementation) — same `ViewQuery`/`RenderContext` params, same
`target.get_color_attachment()`/phase-lookup-by-`retained_view_entity` pattern, sharing
the exact same depth buffer `main_opaque_pass_3d` and `main_transparent_pass_3d` use —
no splat-private depth texture at all.

> **Ordering pitfall, proven the hard way**: `Core3dPlugin` only orders
> `(main_opaque_pass_3d, main_transparent_pass_3d)` against **each other** via
> `.chain()` — that says nothing about where a third system in the same
> `Core3dSystems::MainPass` set falls relative to either one. A `splat_opaque_pass`
> declared only `.after(main_opaque_pass_3d)`, without an explicit
> `.before(main_transparent_pass_3d)` too, is left free by the scheduler to run
> *after* the transparent pass as well. When it does: the transparent pass
> depth-tests translucent splats against a depth buffer the opaque splats haven't
> written to yet (so translucent fragments always pass, drawing wherever told), and
> then the opaque pass runs afterward and unconditionally overwrites color+depth
> wherever its alpha-tested/covered fragments land — painting over the
> already-composited translucent content. Symptom observed in practice: a
> translucent object appearing to be "hidden behind everything," which reads like a
> a depth bug but is actually a missing ordering constraint. Both `.after(...)` and
> `.before(...)` are required; one alone is not sufficient.

Because both passes share the exact same `ViewTarget`/`ViewDepthTexture` every other
`Core3d` pass writes into, splats correctly depth-test against (and are occluded by)
any ordinary Bevy mesh geometry sharing the scene too — this is the concrete mechanism
by which SDF-authored hard-surface elements kept as ordinary Bevy meshes (per the
physics/collision rationale in [world-representation](./world-representation.md))
correctly interleave with the baked splat rendering of the same or adjacent scene
regions, with zero splat-specific occlusion logic required for that interop to work.

## Why everything downstream "just works"

Because `splat_opaque_pass` (and, for translucent splats, Bevy's own unmodified
`main_transparent_pass_3d`) writes to the exact same `ViewTarget` (via `target.
get_color_attachment()`, reading the *current* side of the ping-pong pair and implicitly
becoming the new "current" for whatever runs next — see
[camera-and-view-system](../../bevy-rendering/scene-and-views/camera-and-view-system.md)
for the `post_process_write()` mechanism), every system later in the `Core3d` schedule —
`tonemapping`, any `EarlyPostProcess`/`PostProcess` effect (bloom, depth of field, FXAA,
TAA, the vignette/lens-distortion effect stack — see
[post-processing](../../bevy-rendering/post-processing/INDEX.md)), and `upscaling` —
operates on the composited opaque+transparent+splat image with **no awareness that
splats were ever involved**. This is not a special integration point that had to be
built — it falls out for free from writing the splat passes as ordinary
`Core3dSystems::MainPass` systems rather than a separate side-channel. See
[render-integration/screen-space-effect-compatibility](../render-integration/screen-space-effect-compatibility.md)
for the explicit walkthrough of specific built-in effects and why each one composes
correctly.

## When to dive in

- Implementing the actual `Splat3d` phase/pass code → this page's component/asset/phase/
  pass breakdown is the concrete blueprint, cross-referencing the exact Bevy mechanisms
  (`RenderAsset`, `BinnedPhaseItem`, `ViewQuery`/`RenderContext`) each piece builds on.
- Deciding whether opaque splats need their own phase type or can reuse `Opaque3d`/
  `AlphaMask3d` directly, and whether translucent splats need a custom sorted phase or
  can reuse `Transparent3d` → see this page's corrected "custom PhaseItem(s)" section;
  the proven answer for `Transparent3d` was "reuse it unmodified," and a similar
  investigation is worth doing before assuming `Opaque3d`/`AlphaMask3d` (mesh-shaped
  batch/bin keys) don't fit rather than building a from-scratch `Splat3d` by default.
- Understanding why order-independent (WSR) compositing is NOT the default choice for
  opaque splat content, and what replaced it → see
  [sort-free-compositing](../render-integration/sort-free-compositing.md).
- Verifying a specific built-in Bevy effect (bloom, TAA, SSAO, tonemapping) will actually
  work correctly on splat-rendered pixels → see
  [screen-space-effect-compatibility](../render-integration/screen-space-effect-compatibility.md).

## Related

- [SDF as world description, 3DGS as world rendering](./world-representation.md) — prerequisite: why the splats existed at all.
- [Compositing: Weighted Sum Rendering vs. real depth-tested opaque rendering](../render-integration/sort-free-compositing.md) — deeper: why the opaque phase writes depth instead of using WSR.
- [Why built-in Bevy screen-space effects work on splat-rendered pixels for free](../render-integration/screen-space-effect-compatibility.md) — deeper: the payoff of sharing `ViewTarget`.
- [Render phases, PhaseItems, and GPU-driven batching](../../bevy-rendering/architecture/render-phases-and-batching.md) — prerequisite: `BinnedPhaseItem` vs. `SortedPhaseItem`.
- [Camera-driven scheduling: Core3d and Core2d](../../bevy-rendering/core-pipeline/camera-driven-scheduling.md) — prerequisite: the `Core3dSystems` sets the passes were ordered in.
- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — contrast: how migera's current SDF renderer integrates with Bevy instead.
- [Game engine integration and production status (2025-2026)](../../3dgs/state-of-the-art/game-engine-integration.md) — contrast: how Unreal/Unity plugins integrate splats.
