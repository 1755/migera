//! Post-tonemap "lens/sensor" pass: film grain + vignette + chromatic
//! aberration — see `assets/shaders/hybrid_post.wgsl`'s own doc comment
//! for why this is a SEPARATE pass scheduled after Bevy's own tonemapping
//! system rather than folded into `hybrid_blit.wgsl`'s pre-tonemap linear
//! output. Structurally this is the same "fullscreen fragment pass over
//! `ViewTarget`" shape `hybrid_blit.wgsl`/`pipeline.rs`'s own blit
//! pipeline already establishes, just reading/writing via
//! `ViewTarget::post_process_write()`'s ping-pong (the same mechanism
//! `bevy_core_pipeline`'s own tonemapping/upscaling passes use — read as
//! a pattern reference there, not imported) instead of writing directly
//! into the view's main texture. Because this pass runs AFTER
//! tonemapping (`Core3dSystems::PostProcess`, `.after(tonemapping)`), its
//! target format is whatever the camera's real output format is (the
//! display-referred swapchain format) rather than the HDR-float
//! intermediate — specialized on `TextureFormat` exactly like
//! `hybrid_blit_pipeline`'s own `HybridBlitKey` specializes on
//! `target_format`/`has_depth`, since a `RenderPipelineDescriptor`'s
//! fragment target format must be fixed at pipeline-creation time.

use bevy::core_pipeline::FullscreenShader;
use bevy::core_pipeline::prepass::PreviousViewUniformOffset;
use bevy::diagnostic::FrameCount;
use bevy::prelude::*;
use bevy::render::diagnostic::RecordDiagnostics;
use bevy::render::render_resource::binding_types::{sampler, texture_2d, uniform_buffer};
use bevy::render::render_resource::*;
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery};
use bevy::render::view::{ExtractedView, ViewTarget, ViewUniform, ViewUniformOffset, ViewUniforms};
use bytemuck::{Pod, Zeroable};

use crate::hybrid::extract::RenderHybridPostConfig;

fn hybrid_post_view_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_post_view_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::FRAGMENT,
            (
                uniform_buffer::<ViewUniform>(true),
                texture_2d(TextureSampleType::Float { filterable: true }),
                sampler(SamplerBindingType::Filtering),
            ),
        ),
    )
}

fn hybrid_post_uniform_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_post_uniform_layout",
        &BindGroupLayoutEntries::single(ShaderStages::FRAGMENT, uniform_buffer::<HybridPostUniform>(false)),
    )
}

/// Mirrors `PostUniform` in `hybrid_post.wgsl` field-for-field — see that
/// struct's own doc comment for why grain/vignette/aberration are plain
/// always-applied coefficients (0.0 = off) rather than a shader permutation
/// or a dispatch-skip: this pass is a single cheap fullscreen fragment
/// pass already, so branching it off entirely would only save a
/// std140-uniform write, not a real dispatch.
#[derive(Clone, Copy, Debug, Default, ShaderType, Pod, Zeroable)]
#[repr(C)]
struct HybridPostUniform {
    grain_strength: f32,
    vignette_strength: f32,
    aberration_strength: f32,
    frame_seed: f32,
}

#[derive(Resource)]
pub struct HybridPostPipeline {
    view_layout: BindGroupLayoutDescriptor,
    uniform_layout: BindGroupLayoutDescriptor,
    shader: Handle<Shader>,
    fullscreen_shader: FullscreenShader,
    sampler: Sampler,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy)]
pub struct HybridPostKey {
    pub target_format: TextureFormat,
}

impl SpecializedRenderPipeline for HybridPostPipeline {
    type Key = HybridPostKey;

    fn specialize(&self, key: Self::Key) -> RenderPipelineDescriptor {
        RenderPipelineDescriptor {
            label: Some("hybrid_post_pipeline".into()),
            layout: vec![self.view_layout.clone(), self.uniform_layout.clone()],
            vertex: self.fullscreen_shader.to_vertex_state(),
            fragment: Some(FragmentState {
                shader: self.shader.clone(),
                entry_point: Some("fragment".into()),
                targets: vec![Some(ColorTargetState { format: key.target_format, blend: None, write_mask: ColorWrites::ALL })],
                ..default()
            }),
            primitive: PrimitiveState::default(),
            depth_stencil: None,
            multisample: MultisampleState::default(),
            immediate_size: 0,
            zero_initialize_workgroup_memory: false,
        }
    }
}

pub fn init_hybrid_post_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    fullscreen_shader: Res<FullscreenShader>,
    render_device: Res<RenderDevice>,
) {
    let sampler = render_device.create_sampler(&SamplerDescriptor {
        mag_filter: FilterMode::Linear,
        min_filter: FilterMode::Linear,
        ..default()
    });
    commands.insert_resource(HybridPostPipeline {
        view_layout: hybrid_post_view_layout(),
        uniform_layout: hybrid_post_uniform_layout(),
        shader: asset_server.load("shaders/hybrid_post.wgsl"),
        fullscreen_shader: fullscreen_shader.clone(),
        sampler,
    });
}

#[derive(Resource)]
pub struct HybridPostUniformBuffer(UniformBuffer<HybridPostUniform>);

pub fn init_hybrid_post_buffers(mut commands: Commands) {
    commands.insert_resource(HybridPostUniformBuffer(UniformBuffer::default()));
}

/// Uploads this frame's `HybridPostUniform` — the frame-seed hash input
/// changes every frame (see `hybrid_post.wgsl`'s own doc comment on why:
/// no per-pixel RNG primitive exists in this renderer, so grain instead
/// re-hashes a spatial pattern against a changing seed) even though
/// grain/vignette/aberration strengths themselves rarely change.
pub fn prepare_hybrid_post_uniform(
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    post_config: Res<RenderHybridPostConfig>,
    frame_count: Res<FrameCount>,
    mut buffer: ResMut<HybridPostUniformBuffer>,
) {
    let cfg = post_config.0;
    buffer.0.set(HybridPostUniform {
        grain_strength: cfg.grain_strength,
        vignette_strength: cfg.vignette_strength,
        aberration_strength: cfg.aberration_strength,
        frame_seed: frame_count.0 as f32,
    });
    buffer.0.write_buffer(&render_device, &render_queue);
}

/// Fullscreen pass: reads `ViewTarget::post_process_write().source` (the
/// image Bevy's own tonemapping system just wrote), writes `.destination`
/// — same ping-pong mechanism `bevy_core_pipeline::tonemapping::node`
/// uses, read as a pattern reference (see this module's own doc comment).
#[allow(clippy::too_many_arguments)]
pub fn hybrid_post_pass(
    view: ViewQuery<(&'static ExtractedView, &'static ViewTarget, &'static ViewUniformOffset, Option<&'static PreviousViewUniformOffset>)>,
    pipeline: Option<Res<HybridPostPipeline>>,
    mut pipelines: ResMut<SpecializedRenderPipelines<HybridPostPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    render_device: Res<RenderDevice>,
    view_uniforms: Res<ViewUniforms>,
    uniform_buffer: Option<Res<HybridPostUniformBuffer>>,
    mut ctx: RenderContext,
) {
    let Some(pipeline) = pipeline else { return };
    let Some(uniform_buffer) = uniform_buffer else { return };
    // Same "first frame(s) before PrepassPlugin has run" early-out
    // hybrid_pass's own doc comment explains — this pass doesn't actually
    // need previous-frame data, but gating on it keeps this pass from
    // running (and burning a ping-pong swap) before the rest of the
    // hybrid pipeline is warmed up.
    let (extracted_view, target, view_uniform_offset, previous_view_offset) = view.into_inner();
    if previous_view_offset.is_none() {
        return;
    }
    let Some(view_binding) = view_uniforms.uniforms.binding() else {
        return;
    };
    let Some(post_binding) = uniform_buffer.0.binding() else {
        return;
    };

    let key = HybridPostKey { target_format: extracted_view.target_format };
    let pipeline_id = pipelines.specialize(&pipeline_cache, &pipeline, key);
    let Some(render_pipeline) = pipeline_cache.get_render_pipeline(pipeline_id) else {
        return; // Still compiling.
    };

    let post_process = target.post_process_write();

    let view_bind_group = render_device.create_bind_group(
        "hybrid_post_view_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.view_layout),
        &BindGroupEntries::sequential((view_binding, post_process.source, &pipeline.sampler)),
    );
    let uniform_bind_group = render_device.create_bind_group(
        "hybrid_post_uniform_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.uniform_layout),
        &BindGroupEntries::single(post_binding),
    );

    let diagnostics = ctx.diagnostic_recorder();
    let diagnostics = diagnostics.as_deref();

    let mut render_pass = ctx.begin_tracked_render_pass(RenderPassDescriptor {
        label: Some("hybrid_post_pass"),
        color_attachments: &[Some(RenderPassColorAttachment {
            view: post_process.destination,
            depth_slice: None,
            resolve_target: None,
            ops: Operations { load: LoadOp::Clear(default()), store: StoreOp::Store },
        })],
        depth_stencil_attachment: None,
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    });
    let post_span = diagnostics.pass_span(&mut render_pass, "hybrid_post");
    render_pass.set_render_pipeline(render_pipeline);
    render_pass.set_bind_group(0, &view_bind_group, &[view_uniform_offset.offset]);
    render_pass.set_bind_group(1, &uniform_bind_group, &[]);
    render_pass.draw(0..3, 0..1);
    post_span.end(&mut render_pass);
}
