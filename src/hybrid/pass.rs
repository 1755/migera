//! The hybrid pass: compute trace dispatch + fullscreen blit into the view
//! target, ordered against Bevy's opaque/transparent passes exactly like
//! `hybrid_legacy::pass::hybrid_pass` documents (read as a pattern
//! reference for the dispatch/bind-group sequencing shape — freshly
//! authored here, not reused).

use bevy::core_pipeline::prepass::PreviousViewUniformOffset;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy::render::diagnostic::RecordDiagnostics;
use bevy::render::render_resource::{
    ComputePassDescriptor, PipelineCache, RenderPassDescriptor, SpecializedRenderPipelines, StoreOp,
};
use bevy::render::renderer::{RenderContext, ViewQuery};
use bevy::render::view::{ExtractedView, ViewDepthTexture, ViewTarget, ViewUniformOffset};

use super::extract::{GiMethod, RenderHybridScene};
use super::pipeline::{
    HybridBlitBindGroup, HybridBlitKey, HybridComputeBindGroup, HybridDdgiBindGroup, HybridDenoiseBindGroup,
    HybridDofBindGroup, HybridPipeline, HybridRadianceCascadesBindGroup, HybridReflectTemporalBindGroup,
    HybridTargetsRes, HybridTemporalBindGroup, HybridTraceDdgiBindGroup, HybridTraceRadianceCascadesBindGroup,
    HybridTransmitTemporalBindGroup, HybridViewBindGroup,
};

/// Every per-frame bind-group resource `hybrid_pass` reads, bundled into
/// one `SystemParam` — plain Bevy systems cap out at a fixed parameter
/// tuple arity, and this function's own param list grew past it once
/// DOF's own bind group was added as parameter #18. Bundling is the
/// standard Bevy answer to that limit (see `bevy_ecs::system::SystemParam`'s
/// own docs), not a sign any of these bind groups are logically related
/// beyond "this pass reads all of them."
#[derive(SystemParam)]
pub struct HybridPassBindGroups<'w> {
    compute_bg: Option<Res<'w, HybridComputeBindGroup>>,
    temporal_bg: Option<Res<'w, HybridTemporalBindGroup>>,
    reflect_temporal_bg: Option<Res<'w, HybridReflectTemporalBindGroup>>,
    transmit_temporal_bg: Option<Res<'w, HybridTransmitTemporalBindGroup>>,
    denoise_bg: Option<Res<'w, HybridDenoiseBindGroup>>,
    dof_bg: Option<Res<'w, HybridDofBindGroup>>,
    blit_bg: Option<Res<'w, HybridBlitBindGroup>>,
    ddgi_bg: Option<Res<'w, HybridDdgiBindGroup>>,
    trace_ddgi_bg: Option<Res<'w, HybridTraceDdgiBindGroup>>,
    radiance_cascades_bg: Option<Res<'w, HybridRadianceCascadesBindGroup>>,
    trace_radiance_cascades_bg: Option<Res<'w, HybridTraceRadianceCascadesBindGroup>>,
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn hybrid_pass(
    view: ViewQuery<(
        &'static ExtractedView,
        &'static ViewTarget,
        &'static ViewUniformOffset,
        Option<&'static ViewDepthTexture>,
        Option<&'static PreviousViewUniformOffset>,
    )>,
    hybrid_pipeline: Option<Res<HybridPipeline>>,
    mut pipelines: ResMut<SpecializedRenderPipelines<HybridPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    bind_groups: HybridPassBindGroups,
    scene_data: Option<Res<RenderHybridScene>>,
    targets: Option<Res<HybridTargetsRes>>,
    view_bind_groups: Query<&HybridViewBindGroup>,
    mut ctx: RenderContext,
    mut logged_ready: Local<bool>,
) {
    let Some(hybrid_pipeline) = hybrid_pipeline else {
        return; // RenderStartup hasn't queued the pipeline yet.
    };
    let (
        Some(compute_group),
        Some(temporal_group),
        Some(reflect_temporal_group),
        Some(transmit_temporal_group),
        Some(denoise_group),
        Some(dof_group),
        Some(blit_group),
        Some(ddgi_group),
        Some(trace_ddgi_group),
        Some(radiance_cascades_group),
        Some(trace_radiance_cascades_group),
    ) = (
        bind_groups.compute_bg.as_deref(),
        bind_groups.temporal_bg.as_deref(),
        bind_groups.reflect_temporal_bg.as_deref(),
        bind_groups.transmit_temporal_bg.as_deref(),
        bind_groups.denoise_bg.as_deref(),
        bind_groups.dof_bg.as_deref(),
        bind_groups.blit_bg.as_deref(),
        bind_groups.ddgi_bg.as_deref(),
        bind_groups.trace_ddgi_bg.as_deref(),
        bind_groups.radiance_cascades_bg.as_deref(),
        bind_groups.trace_radiance_cascades_bg.as_deref(),
    )
    else {
        return; // First frame(s) before prepare_hybrid_scene/prepare_hybrid_temporal/prepare_hybrid_ddgi/prepare_hybrid_radiance_cascades has anything to bind.
    };
    let Some(scene_data) = scene_data else {
        return; // First frame(s), same reason as above.
    };
    // This frame's own trace-resolution working size (RenderScaleConfig
    // experiment, step 1b) — read from HybridTargetsRes rather than
    // recomputed here from extracted_view.viewport * scene_data.
    // render_scale, so there is exactly ONE place (prepare_hybrid_scene's
    // own trace_size local) that resolves real-viewport-size * scale into
    // actual texels, and every dispatch site here agrees with whatever
    // size the storage textures were ACTUALLY created at (avoids a
    // rounding-mismatch class of bug if the two computations ever drifted).
    let Some(trace_size) = targets.as_deref().and_then(|t| t.0.as_ref()).map(|t| t.size) else {
        return; // First frame(s), same reason as above.
    };
    let view_entity = view.entity();
    let (extracted_view, target, view_uniform_offset, depth_texture, previous_view_offset) = view.into_inner();
    let Ok(view_group) = view_bind_groups.get(view_entity) else {
        return;
    };
    // First frame(s): PreviousViewUniformOffset isn't inserted on the
    // render-world view entity until bevy_pbr's own
    // prepare_previous_view_uniforms has run at least once — normal at
    // startup, matching every other "still warming up" early-return here.
    let Some(previous_view_offset) = previous_view_offset else {
        return;
    };

    // 1) Compute trace pass.
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(hybrid_pipeline.trace_pipeline) else {
        return; // Pipeline still compiling — normal at startup.
    };
    let Some(temporal_pipeline) = pipeline_cache.get_compute_pipeline(hybrid_pipeline.temporal_pipeline) else {
        return; // Pipeline still compiling — normal at startup.
    };
    // Only resolved/required when reflections are actually on — see the
    // dispatch-skip below for why an always-on pipeline-not-ready check
    // here would block every other pass on a pipeline this frame doesn't
    // even use.
    let reflect_temporal_pipeline = if scene_data.reflection_enabled {
        let Some(p) = pipeline_cache.get_compute_pipeline(hybrid_pipeline.reflect_temporal_pipeline) else {
            return; // Pipeline still compiling — normal at startup.
        };
        Some(p)
    } else {
        None
    };
    // Same "only resolved/required when actually on" reasoning as
    // reflect_temporal_pipeline above.
    let transmit_temporal_pipeline = if scene_data.transmission_enabled {
        let Some(p) = pipeline_cache.get_compute_pipeline(hybrid_pipeline.transmit_temporal_pipeline) else {
            return; // Pipeline still compiling — normal at startup.
        };
        Some(p)
    } else {
        None
    };
    let Some(denoise_pipeline) = pipeline_cache.get_compute_pipeline(hybrid_pipeline.denoise_pipeline) else {
        return; // Pipeline still compiling — normal at startup.
    };
    let Some(dof_pipeline) = pipeline_cache.get_compute_pipeline(hybrid_pipeline.dof_pipeline) else {
        return; // Pipeline still compiling — normal at startup.
    };
    // Only resolved/required when DDGI is actually the active technique —
    // same "don't block every other pass on a pipeline this frame doesn't
    // even use" reasoning as reflect_temporal_pipeline/
    // transmit_temporal_pipeline above.
    let ddgi_pipeline = if scene_data.gi_method == GiMethod::Ddgi as u32 {
        let Some(p) = pipeline_cache.get_compute_pipeline(hybrid_pipeline.ddgi_pipeline) else {
            return; // Pipeline still compiling — normal at startup.
        };
        Some(p)
    } else {
        None
    };
    // Only resolved/required when Radiance Cascades is actually the
    // active technique — same reasoning as ddgi_pipeline immediately
    // above.
    let radiance_cascades_pipeline = if scene_data.gi_method == GiMethod::RadianceCascades as u32 {
        let Some(p) = pipeline_cache.get_compute_pipeline(hybrid_pipeline.radiance_cascades_pipeline) else {
            return; // Pipeline still compiling — normal at startup.
        };
        Some(p)
    } else {
        None
    };
    if !*logged_ready {
        *logged_ready = true;
        info!("hybrid pass: trace pipeline READY");
    }

    // GPU timestamp spans (see this module's own doc comment on why): a
    // `None` recorder (RenderDiagnosticsPlugin not installed, or the
    // active backend doesn't support timestamp queries) makes every
    // RecordDiagnostics call below a no-op — no `#[cfg]` needed at any
    // call site, per that trait's own doc comment.
    let diagnostics = ctx.diagnostic_recorder();
    let diagnostics = diagnostics.as_deref();

    // 0) DDGI probe-relight pass: runs FIRST, before trace — trace_main
    // needs to sample an ALREADY-relit atlas this frame (see
    // hybrid_ddgi_relight.wgsl's own header comment for the "read-before-
    // write" ordering constraint, the same one that puts
    // hybrid_temporal.wgsl between trace and denoise). Dispatch domain is
    // [probes_per_frame * tile_size, tile_size] in the X/Y workgroup grid
    // — gid.x encodes both "which of this frame's relit probes" and
    // "which texel column," see that shader's own ddgi_relight_main doc
    // comment. Skipped entirely (not just a cheap pass-through) when
    // DDGI isn't the active technique — a probe relight is a real batch
    // of trace() calls, not a cheap per-pixel copy, so there's a genuine
    // dispatch to save. Gated on GiMethod::Ddgi being the ACTIVE
    // technique (see extract::GiMethod's own doc comment) — selecting a
    // different technique skips this dispatch.
    if let Some(ddgi_pipeline) = ddgi_pipeline {
        let encoder = ctx.command_encoder();
        let mut cpass =
            encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("hybrid_ddgi_pass"), timestamp_writes: None });
        let ddgi_span = diagnostics.pass_span(&mut cpass, "hybrid_ddgi");
        cpass.set_pipeline(ddgi_pipeline);
        cpass.set_bind_group(0, &ddgi_group.value, &[]);
        let total_probes = scene_data.ddgi_grid.probe_count().max(1) as u32;
        let probes_per_frame = scene_data.ddgi_probes_per_frame.clamp(1, total_probes);
        let tile_size = scene_data.ddgi_tile_size.max(1);
        let wg_x = (probes_per_frame * tile_size).div_ceil(8);
        let wg_y = tile_size.div_ceil(8);
        cpass.dispatch_workgroups(wg_x, wg_y, 1);
        ddgi_span.end(&mut cpass);
    }

    // 0b) Radiance Cascades' own relight pass — an experimental
    // alternative to DDGI (see extract::GiMethod::RadianceCascades's own
    // doc comment), runs FIRST alongside DDGI's own relight (before
    // trace, same "trace_main needs an already-relit atlas" ordering
    // constraint) — gated on GiMethod::RadianceCascades being the ACTIVE
    // technique, skipped entirely otherwise (same "a relight is a real
    // batch of trace() calls, not a cheap copy" reasoning as DDGI's own
    // dispatch-skip above). Dispatch domain covers level 0's own atlas
    // (the largest level — most probes, finest tile, see
    // hybrid_radiance_cascades.wgsl's own radiance_cascades_relight_main
    // doc comment for why X/Y is sized to level 0 and Z selects which of
    // the 4 levels) — level 0's own geometry is recomputed here from the
    // same base_* scalars/root_bounds prepare_hybrid_radiance_cascades
    // already used to build this frame's CascadeLevelUniform array, so
    // the dispatch size always matches what was actually written there.
    if let Some(radiance_cascades_pipeline) = radiance_cascades_pipeline {
        let bounds = crate::prim::Aabb { min: scene_data.radiance_cascades_root_bounds_min, max: scene_data.radiance_cascades_root_bounds_max };
        let level0_params = crate::hybrid::radiance_cascades_ref::cascade_level_params(
            0,
            scene_data.radiance_cascades_base_spacing,
            scene_data.radiance_cascades_base_ray_count,
            scene_data.radiance_cascades_base_interval,
        );
        let level0_grid = crate::hybrid::radiance_cascades_ref::cascade_grid_from_bounds(bounds, level0_params);
        let level0_total_probes = (level0_grid.probe_count() as u32).max(1);
        let level0_tile_size = crate::hybrid::radiance_cascades_ref::cascade_tile_side(level0_params.ray_count).max(1);
        let level0_layout = crate::hybrid::ddgi_ref::AtlasLayout::exact_fit(level0_total_probes, level0_tile_size);
        let atlas_side = level0_layout.atlas_pixels;
        let wg_x = atlas_side.div_ceil(8);
        let wg_y = atlas_side.div_ceil(8);
        let wg_z = crate::hybrid::extract::RADIANCE_CASCADES_LEVEL_COUNT;
        // Bounce depth: re-dispatch this SAME relight pass
        // `radiance_cascades_bounce_passes` times, each a genuinely
        // separate begin_compute_pass call — WGPU/Vulkan's own
        // pass-boundary-as-barrier guarantee means pass N+1's atlas reads
        // (relight_cascade_texel's own indirect_at_hit-equivalent call,
        // cascade_sample_hierarchy_at_hit in the WGSL) see pass N's
        // fully-written atlas, not a same-dispatch race — see
        // RadianceCascadesConfig::bounce_passes's own doc comment for why
        // this replaces DDGI's cross-frame temporal accumulation with an
        // explicit, real N-bounce depth per frame instead. Each pass gets
        // its own "hybrid_radiance_cascades" pass_span call (same name,
        // multiple diagnostic samples per frame) — the HUD/log readout
        // (gpu_pass_ms/FpsStats) reports Bevy's own smoothed per-sample
        // average, i.e. roughly ONE pass's own cost, not the sum across
        // all bounce_passes this frame; multiply by bounce_passes for the
        // real total relight cost this frame.
        let bounce_passes = scene_data.radiance_cascades_bounce_passes.max(1);
        for _ in 0..bounce_passes {
            let encoder = ctx.command_encoder();
            let mut cpass = encoder
                .begin_compute_pass(&ComputePassDescriptor { label: Some("hybrid_radiance_cascades_pass"), timestamp_writes: None });
            let radiance_cascades_span = diagnostics.pass_span(&mut cpass, "hybrid_radiance_cascades");
            cpass.set_pipeline(radiance_cascades_pipeline);
            cpass.set_bind_group(0, &radiance_cascades_group.value, &[]);
            cpass.dispatch_workgroups(wg_x, wg_y, wg_z);
            radiance_cascades_span.end(&mut cpass);
        }
    }

    // 1) Trace pass: one invocation per pixel, self-shading (see this
    // crate's top-level module doc comment) — cone tracing's own indirect-
    // diffuse contribution is computed entirely inline here, no separate
    // pass or bind group of its own (see conetrace_ref.rs's own module doc
    // comment for why: no persistent GPU structure, every shaded pixel
    // re-marches its own cone bundle directly against the BVH/SDF).
    // Group 2 (the DDGI atlas read) is bound unconditionally, regardless
    // of which GiMethod is active — trace_main's own group-2 bind group
    // layout is fixed at pipeline-creation time, so it must always be
    // bound to something valid; `shade()` itself only actually SAMPLES
    // the atlas when `gi_method == GI_METHOD_DDGI` (see hybrid_trace.wgsl's
    // own doc comment), so binding it when DDGI is inactive is harmless
    // — the data is simply never read.
    {
        let encoder = ctx.command_encoder();
        let mut cpass =
            encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("hybrid_trace_pass"), timestamp_writes: None });
        let trace_span = diagnostics.pass_span(&mut cpass, "hybrid_trace");
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &view_group.value, &[view_uniform_offset.offset]);
        cpass.set_bind_group(1, &compute_group.value, &[]);
        cpass.set_bind_group(2, &trace_ddgi_group.value, &[]);
        cpass.set_bind_group(3, &trace_radiance_cascades_group.value, &[]);
        let wg_x = trace_size.x.div_ceil(8);
        let wg_y = trace_size.y.div_ceil(8);
        cpass.dispatch_workgroups(wg_x, wg_y, 1);
        trace_span.end(&mut cpass);
    }

    // 2) Temporal-accumulation pass: reprojects last frame's indirect-
    // diffuse history against this frame's trace output (via
    // PreviousViewUniformOffset — bevy_pbr's own last-frame camera
    // matrices, populated unconditionally by PrepassPlugin in this app),
    // blends, and writes accumulated_indirect_view — see
    // hybrid_temporal.wgsl's own doc comment.
    {
        let encoder = ctx.command_encoder();
        let mut cpass = encoder
            .begin_compute_pass(&ComputePassDescriptor { label: Some("hybrid_temporal_pass"), timestamp_writes: None });
        let temporal_span = diagnostics.pass_span(&mut cpass, "hybrid_temporal");
        cpass.set_pipeline(temporal_pipeline);
        cpass.set_bind_group(0, &temporal_group.read, &[previous_view_offset.offset]);
        cpass.set_bind_group(1, &temporal_group.write, &[]);
        let wg_x = trace_size.x.div_ceil(8);
        let wg_y = trace_size.y.div_ceil(8);
        cpass.dispatch_workgroups(wg_x, wg_y, 1);
        temporal_span.end(&mut cpass);
    }

    // 2b) Specular reflection's own temporal-accumulation pass — same
    // reprojection shape as (2), but reads reflect_view/reflect_motion_view
    // and writes accumulated_reflect_view into a SEPARATE ping-pong
    // history (see HybridReflectHistory's own doc comment for why
    // reflection needs independent history rather than sharing (2)'s).
    // Skipped entirely (not dispatched, not just a copy-through) when
    // reflections are disabled — `hybrid_trace.wgsl` never writes anything
    // meaningful into `reflect_view` in that case (the Fresnel gate in
    // `shade()` never fires), so accumulating it is pure wasted dispatch
    // cost; this was previously an always-on tax (~3.5-4.4ms) regardless
    // of the `reflection_enabled` toggle, now fixed.
    if let Some(reflect_temporal_pipeline) = reflect_temporal_pipeline {
        let encoder = ctx.command_encoder();
        let mut cpass = encoder
            .begin_compute_pass(&ComputePassDescriptor { label: Some("hybrid_reflect_temporal_pass"), timestamp_writes: None });
        let reflect_temporal_span = diagnostics.pass_span(&mut cpass, "hybrid_reflect_temporal");
        cpass.set_pipeline(reflect_temporal_pipeline);
        cpass.set_bind_group(0, &reflect_temporal_group.read, &[previous_view_offset.offset]);
        cpass.set_bind_group(1, &reflect_temporal_group.write, &[]);
        let wg_x = trace_size.x.div_ceil(8);
        let wg_y = trace_size.y.div_ceil(8);
        cpass.dispatch_workgroups(wg_x, wg_y, 1);
        reflect_temporal_span.end(&mut cpass);
    }

    // 2c) Transmission's own temporal-accumulation pass — same
    // reprojection shape as (2b), but reads refract_view/motion_view
    // (.w = refracting_roughness)/refract_motion_view and writes
    // accumulated_refract_view into a THIRD, independent ping-pong
    // history (see HybridTransmitHistory's own doc comment for why
    // transmission needs history separate from both diffuse GI's and
    // reflection's). Skipped entirely when transmission is disabled,
    // same "not dispatched, not just a copy-through" reasoning as (2b) —
    // applying the always-on-tax lesson reflection's own initial landing
    // found (see PROGRESS.md) from the start here, rather than
    // discovering it as a follow-up fix.
    if let Some(transmit_temporal_pipeline) = transmit_temporal_pipeline {
        let encoder = ctx.command_encoder();
        let mut cpass = encoder
            .begin_compute_pass(&ComputePassDescriptor { label: Some("hybrid_transmit_temporal_pass"), timestamp_writes: None });
        let transmit_temporal_span = diagnostics.pass_span(&mut cpass, "hybrid_transmit_temporal");
        cpass.set_pipeline(transmit_temporal_pipeline);
        cpass.set_bind_group(0, &transmit_temporal_group.read, &[previous_view_offset.offset]);
        cpass.set_bind_group(1, &transmit_temporal_group.write, &[]);
        let wg_x = trace_size.x.div_ceil(8);
        let wg_y = trace_size.y.div_ceil(8);
        cpass.dispatch_workgroups(wg_x, wg_y, 1);
        transmit_temporal_span.end(&mut cpass);
    }

    // 3) Denoise pass: same-frame edge-aware blur of the (temporally-
    // accumulated) indirect-diffuse term, recombined into a dedicated
    // denoised-color texture the blit pass reads next — see
    // hybrid_denoise.wgsl's own doc comment.
    {
        let encoder = ctx.command_encoder();
        let mut cpass = encoder
            .begin_compute_pass(&ComputePassDescriptor { label: Some("hybrid_denoise_pass"), timestamp_writes: None });
        let denoise_span = diagnostics.pass_span(&mut cpass, "hybrid_denoise");
        cpass.set_pipeline(denoise_pipeline);
        cpass.set_bind_group(0, &denoise_group.value, &[]);
        let wg_x = trace_size.x.div_ceil(8);
        let wg_y = trace_size.y.div_ceil(8);
        cpass.dispatch_workgroups(wg_x, wg_y, 1);
        denoise_span.end(&mut cpass);
    }

    // 3.5) Stochastic depth-of-field: resolves hybrid_denoise.wgsl's own
    // sharp/composited output into a defocus-blurred (or, when
    // DofConfig::enabled is false, bit-for-bit unchanged) result the
    // blit pass reads next — see hybrid_dof.wgsl's own header comment
    // for the full technique. Runs its OWN full trace() against the
    // scene (a separate ray from the primary one — see that file's own
    // doc comment for why), so it needs group 0 (view, same bind group
    // trace_main/blit already use) in addition to its own read/write
    // groups.
    {
        let encoder = ctx.command_encoder();
        let mut cpass =
            encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("hybrid_dof_pass"), timestamp_writes: None });
        let dof_span = diagnostics.pass_span(&mut cpass, "hybrid_dof");
        cpass.set_pipeline(dof_pipeline);
        cpass.set_bind_group(0, &view_group.value, &[view_uniform_offset.offset]);
        cpass.set_bind_group(1, &dof_group.read, &[]);
        cpass.set_bind_group(2, &dof_group.write, &[]);
        let wg_x = trace_size.x.div_ceil(8);
        let wg_y = trace_size.y.div_ceil(8);
        cpass.dispatch_workgroups(wg_x, wg_y, 1);
        dof_span.end(&mut cpass);
    }

    // 4) Blit into the view target with reverse-Z depth reconstruction.
    let key = HybridBlitKey { target_format: extracted_view.target_format, has_depth: depth_texture.is_some() };
    let pipeline_id = pipelines.specialize(&pipeline_cache, &hybrid_pipeline, key);
    let Some(render_pipeline) = pipeline_cache.get_render_pipeline(pipeline_id) else {
        return; // Still compiling.
    };

    let depth_attachment = depth_texture.map(|d| d.get_attachment(StoreOp::Store));

    let mut render_pass = ctx.begin_tracked_render_pass(RenderPassDescriptor {
        label: Some("hybrid_blit_pass"),
        color_attachments: &[Some(target.get_color_attachment())],
        depth_stencil_attachment: depth_attachment,
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    });

    let blit_span = diagnostics.pass_span(&mut render_pass, "hybrid_blit");
    render_pass.set_render_pipeline(render_pipeline);
    render_pass.set_bind_group(0, &view_group.value, &[view_uniform_offset.offset]);
    render_pass.set_bind_group(1, &blit_group.value, &[]);
    render_pass.draw(0..3, 0..1);
    blit_span.end(&mut render_pass);
}
