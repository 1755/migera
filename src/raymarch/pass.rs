//! The raymarch render pass: draws one full-screen triangle per view, directly into
//! `ViewTarget` — see `pipeline.rs`'s module doc for the bind group layout this
//! depends on.

use bevy::prelude::*;
use bevy::render::diagnostic::RecordDiagnostics;
use bevy::render::render_resource::{
    PipelineCache, RenderPassDescriptor, SpecializedRenderPipelines,
};
use bevy::render::renderer::{RenderContext, ViewQuery};
use bevy::render::view::{ExtractedView, Msaa, ViewTarget, ViewUniformOffset};

use super::pipeline::{
    RaymarchPipeline, RaymarchPipelineKey, RaymarchSceneBindGroup, RaymarchViewBindGroup,
};

fn msaa_sample_count(msaa: Option<&Msaa>) -> u32 {
    match msaa.copied().unwrap_or_default() {
        Msaa::Off => 1,
        Msaa::Sample2 => 2,
        Msaa::Sample4 => 4,
        Msaa::Sample8 => 8,
    }
}

/// Draws the full-screen raymarch triangle for one view. No `PhaseItem`/
/// `BinnedPhaseItem`/`DrawFunctions` machinery: the raymarcher draws exactly once per
/// view (one full-screen triangle), so pipeline specialization (by target format/MSAA
/// sample count only) happens inline here. Deliberately does not take a `world: &World`
/// param — that would conflict with this system's own `ResMut<
/// SpecializedRenderPipelines<RaymarchPipeline>>` param (Bevy's `SystemParam`
/// validation rejects a system requesting both whole-world read access and an
/// exclusive resource write in the same signature).
pub fn raymarch_pass(
    view: ViewQuery<(
        &ExtractedView,
        &ViewTarget,
        &ViewUniformOffset,
        Option<&Msaa>,
    )>,
    raymarch_pipeline: Res<RaymarchPipeline>,
    mut pipelines: ResMut<SpecializedRenderPipelines<RaymarchPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    scene_bind_group: Option<Res<RaymarchSceneBindGroup>>,
    view_bind_groups: Query<&RaymarchViewBindGroup>,
    mut ctx: RenderContext,
) {
    let Some(scene_bind_group) = scene_bind_group else {
        return; // Buffers not prepared yet (first few frames).
    };

    let view_entity = view.entity();
    let (extracted_view, target, view_uniform_offset, msaa) = view.into_inner();

    let Ok(view_bind_group) = view_bind_groups.get(view_entity) else {
        return;
    };

    let pipeline_id = pipelines.specialize(
        &pipeline_cache,
        &raymarch_pipeline,
        RaymarchPipelineKey {
            target_format: extracted_view.target_format,
            msaa_samples: msaa_sample_count(msaa),
        },
    );
    let Some(pipeline) = pipeline_cache.get_render_pipeline(pipeline_id) else {
        return; // Still compiling.
    };

    let diagnostics = ctx.diagnostic_recorder();

    let mut render_pass = ctx.begin_tracked_render_pass(RenderPassDescriptor {
        label: Some("raymarch_pass"),
        color_attachments: &[Some(target.get_color_attachment())],
        depth_stencil_attachment: None,
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    });

    let pass_span = diagnostics
        .as_ref()
        .map(|d| d.pass_span(&mut render_pass, "raymarch_pass"));

    render_pass.set_render_pipeline(pipeline);
    render_pass.set_bind_group(0, &view_bind_group.value, &[view_uniform_offset.offset]);
    render_pass.set_bind_group(1, &scene_bind_group.value, &[]);
    render_pass.draw(0..3, 0..1);

    if let Some(span) = pass_span {
        span.end(&mut render_pass);
    }
}
