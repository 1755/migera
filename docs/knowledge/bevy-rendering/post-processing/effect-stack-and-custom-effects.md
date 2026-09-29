---
title: The effect stack (vignette, lens distortion, chromatic aberration) and writing a custom effect
description: Bevy 0.19.1 fuses vignette, lens distortion and chromatic aberration into one pre-tonemap pass; this note traces Vignette end to end (component, extraction, pipeline, ViewTarget ping-pong, system placement) as the template for any custom post-process effect. Read before writing a post-process pass.
type: reference
status: current
tags:
  - bevy
  - post-processing
  - render-pipeline
  - integration
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_post_process-0.19.1/src/effect_stack/
aliases:
  - vignette
  - lens distortion
  - chromatic aberration
  - custom post-process effect
  - post_process_write
---

# The effect stack (vignette, lens distortion, chromatic aberration) and writing a custom effect

Contents: [Fused effect stack](#chromatic-aberration-lens-distortion-vignette--fused-into-one-pass) · [Worked example: Vignette](#worked-example-tracing-vignette-end-to-end) · [Custom effect recipe](#recipe-for-a-custom-effect) · [When to dive in](#when-to-dive-in) · [Related](#related)

## Chromatic Aberration, Lens Distortion, Vignette — fused into one pass

`bevy_post_process/src/effect_stack/` deliberately fuses three simple per-pixel UV/color
transforms into a **single combined fragment pass/pipeline**
(`PostProcessingPipeline`, shader `post_process.wgsl`) rather than three separate passes,
for efficiency:

- **Chromatic Aberration**: simulates lens color-fringing by sampling R/G/B channels at
  slightly different UV offsets along a radial direction, using a small gradient LUT
  texture. Component: `ChromaticAberration { intensity, max_samples, color_lut }`.
- **Lens Distortion** (new in 0.19): simulates barrel/pincushion lens warping via a
  simplified Brown-Conrady radial model (k1/k2 only). Component: `LensDistortion {
  intensity, scale, edge_curvature, center, multiplier }`.
- **Vignette** (new in 0.19): darkens/tints the screen edges. Component: `Vignette {
  intensity, radius, smoothness, roundness, center, edge_compensation, color }`.

All three are optional per-camera components composited in one pass:
`post_processing.after(depth_of_field).before(tonemapping)` in `Core3d`
(`.after(bloom)` in `Core2d`) — still pre-tonemap.

## Worked example: tracing Vignette end-to-end

Every effect in `effect_stack` follows four stages — this is the general Bevy pattern for
a custom post-process effect, worth internalizing before writing your own.

### (a) Component = the on/off + settings API

A camera opts in by inserting the component:

```rust
commands.spawn((Camera3d::default(), Vignette { intensity: 0.6, ..default() }));
```

`Vignette` is a plain `#[derive(Reflect, Component, Clone)]` struct in the main world.

### (b) Extraction with an early-out

`ExtractComponent` is implemented manually (not derived) so it can early-out: if the
effect is visually negligible, extraction returns `None` and the render-world entity gets
no component that frame — cheap disabling:

```rust
impl ExtractComponent for Vignette {
    type QueryData = Read<Vignette>;
    type QueryFilter = With<Camera>;
    type Out = Self;
    fn extract_component(vignette: QueryItem<Self::QueryData>) -> Option<Self::Out> {
        if vignette.intensity > 1e-4 { Some(vignette.clone()) } else { None }
    }
}
```

Chromatic Aberration and Lens Distortion get identical treatment; all three share one
`EffectStackPlugin`. See
[entity-sync-and-extraction-patterns](../architecture/entity-sync-and-extraction-patterns.md)
for the general `ExtractComponent` mechanism this customizes.

### (c) A specialized render pipeline, built once at `RenderStartup`

Rather than three pipelines, `EffectStackPlugin` builds **one** `PostProcessingPipeline`
whose bind-group layout has slots for the source texture/sampler plus the chromatic-
aberration LUT and three uniform buffers. Vignette's GPU-mirror struct:

```rust
#[derive(ShaderType, Default)]
pub struct VignetteUniform {
    intensity: f32, radius: f32, smoothness: f32, roundness: f32,
    center: Vec2, edge_compensation: f32, unused: u32, color: Vec4,
}
```

`PostProcessingPipeline` implements `SpecializedRenderPipeline` (see
[pipeline-cache-and-specialization](../resources-and-assets/pipeline-cache-and-specialization.md)),
specializing only on target texture format. The fused fragment shader:

```wgsl
@fragment
fn fragment_main(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let distorted_uv = lens_distortion(in.uv);
    let color = chromatic_aberration(distorted_uv);
    return vec4(vignette(in.uv, color), 1.0);
}
```

Each effect's WGSL is a plain function imported and composed, internally gated by its own
intensity threshold so disabled effects are no-ops even inside the shared shader.

Per-frame `RenderSystems::Prepare` systems handle GPU bookkeeping:
`prepare_post_processing_pipelines` specializes/caches the `CachedRenderPipelineId` per
view; `prepare_post_processing_uniforms` pushes each present-or-default uniform struct
into a `DynamicUniformBuffer` and records the byte offset.

### (d) The render system: read source, write destination, let ViewTarget swap

```rust
pub(crate) fn post_processing(
    view: ViewQuery<(&ViewTarget, &PostProcessingPipelineId,
                     AnyOf<(&ChromaticAberration, &Vignette, &LensDistortion)>,
                     &PostProcessingUniformBufferOffsets)>,
    /* pipeline_cache, pipeline, uniform buffers, image assets, ctx: RenderContext */
) {
    let (view_target, pipeline_id, post_effects, offsets) = view.into_inner();
    if post_effects_are_all_none { return; }
    let pipeline = pipeline_cache.get_render_pipeline(**pipeline_id)?;
    let post_process = view_target.post_process_write();  // the ping-pong step

    let mut pass = ctx.begin_tracked_render_pass(RenderPassDescriptor {
        color_attachments: &[Some(RenderPassColorAttachment {
            view: post_process.destination, ..default()
        })], ..default()
    });
    pass.set_render_pipeline(pipeline);
    pass.set_bind_group(0, &bind_group, &[offsets.chromatic_aberration, offsets.vignette, offsets.lens_distortion]);
    pass.draw(0..3, 0..1);   // fullscreen triangle
}
```

The crux is `view_target.post_process_write()` (see
[camera-and-view-system](../scene-and-views/camera-and-view-system.md)): it atomically
flips `ViewTarget`'s A/B index and returns `{ source, destination }`, where `source` is
whatever the *previous* system in the chain wrote and `destination` is the other texture.
Reading only `source` and writing only `destination` via a fullscreen-triangle pass means
the *next* system in the chain (`tonemapping`) transparently receives this system's output
as its new `source` — no manual bookkeeping, no graph edges, just call-order via
`.before()`/`.after()`/set membership.

## Recipe for a custom effect

1. Define a `Component` + `Default` with your settings.
2. Implement/derive `ExtractComponent` (optionally early-out like `Vignette`).
3. Build a `RenderStartup` system creating a `BindGroupLayoutDescriptor` (source texture +
   sampler + your uniform) and a `SpecializedRenderPipeline` around a fullscreen-triangle
   vertex stage (`FullscreenShader::to_vertex_state()`) + your fragment shader.
4. Add a `RenderSystems::Prepare` system uploading a `DynamicUniformBuffer` per view.
5. Add your render system into `Core3dSystems::PostProcess` (post-tonemap, LDR) or
   `EarlyPostProcess` (pre-tonemap, HDR, needs motion vectors/history) ordered with
   `.before()`/`.after()` relative to neighbors like `bloom`, `depth_of_field`,
   `tonemapping`. Inside it, call `view_target.post_process_write()`, bind `source`,
   render to `destination`.

For the simplest case (single fragment shader, no extra uniforms/bind groups beyond the
source texture), skip most of this and use `FullscreenMaterialPlugin` — see
[passes-and-fullscreen-effects](../core-pipeline/passes-and-fullscreen-effects.md).

## When to dive in

- Writing any custom post-process effect → this document's recipe and worked example are
  the template; read it before hand-rolling pipeline/bind-group boilerplate.
- Wondering why three unrelated-looking settings (vignette, lens distortion, chromatic
  aberration) share one pipeline → it's a deliberate fusion for efficiency, not a
  coincidence of code organization.

## Related
- [Camera and the view system](../scene-and-views/camera-and-view-system.md) — prerequisite: the conceptual `ViewTarget`/`post_process_write()` mechanism this example uses.
- [Prepass, tonemapping, upscaling, deferred, and OIT](../core-pipeline/passes-and-fullscreen-effects.md) — contrast: `FullscreenMaterialPlugin`, the lower-boilerplate route for simple effects.
- [PipelineCache and pipeline specialization](../resources-and-assets/pipeline-cache-and-specialization.md) — deeper: the `SpecializedRenderPipeline` step in the recipe.
- [Bloom, depth of field, motion blur, auto exposure](./effects-bloom-dof-motion-blur.md) — contrast: the neighbouring effects this pass is ordered after.
- [Screen-space effect compatibility for splats](../../sdf-3dgs-bevy-integration/render-integration/screen-space-effect-compatibility.md) — applies: why `ViewTarget`-based effects work on any renderer's output.
