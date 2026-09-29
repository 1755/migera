//! The prepass-probe writer: compute-dispatch the sphere trace, then a
//! fullscreen blit that writes the REAL `ViewDepthTexture` and
//! `ViewPrepassTextures.deferred`/`deferred_lighting_pass_id` attachments —
//! see `super`'s module doc comment for why this runs in
//! `Core3dSystems::Prepass` (before `main_opaque_pass_3d`), unlike
//! `crate::hybrid::pass::hybrid_pass` which runs after it.

use bevy::core_pipeline::prepass::ViewPrepassTextures;
use bevy::pbr::{LightEntity, ShadowView, ViewLightEntities};
use bevy::prelude::*;
use bevy::render::render_resource::{
    BindGroupEntries, ComputePassDescriptor, PipelineCache, RenderPassDescriptor, StoreOp,
    UniformBuffer,
};
use bevy::render::renderer::{RenderContext, RenderDevice, RenderQueue, ViewQuery};
use bevy::render::view::{ExtractedView, ViewDepthTexture, ViewUniformOffset};

use super::pipeline::{
    ProbeBlitBindGroup, ProbeComputeBindGroup, ProbePipeline, ProbeViewBindGroup,
    ShadowSphereUniform, ShadowViewUniform, ShadowWritePipeline,
};
use super::SdfProbeSphere;

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn probe_prepass_write(
    view: ViewQuery<(
        &'static ExtractedView,
        &'static ViewUniformOffset,
        &'static ViewDepthTexture,
        &'static ViewPrepassTextures,
    )>,
    probe_pipeline: Option<Res<ProbePipeline>>,
    pipeline_cache: Res<PipelineCache>,
    compute_bg: Option<Res<ProbeComputeBindGroup>>,
    blit_bg: Option<Res<ProbeBlitBindGroup>>,
    view_bind_groups: Query<&ProbeViewBindGroup>,
    mut ctx: RenderContext,
    mut logged_ready: Local<bool>,
) {
    let (Some(pp), Some(compute_group), Some(blit_group)) =
        (probe_pipeline.as_deref(), compute_bg.as_deref(), blit_bg.as_deref())
    else {
        return;
    };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pp.trace_pipeline) else {
        return; // Still compiling.
    };
    let Some(blit_pipeline) = pipeline_cache.get_render_pipeline(pp.blit_pipeline) else {
        return; // Still compiling.
    };

    let view_entity = view.entity();
    let (extracted_view, view_uniform_offset, view_depth_texture, view_prepass_textures) =
        view.into_inner();
    let Ok(view_group) = view_bind_groups.get(view_entity) else {
        return;
    };
    // This spike only enables DepthPrepass + DeferredPrepass (no
    // Normal/MotionVector), so these are the only two attachments we need.
    let (Some(deferred), Some(pass_id)) = (
        view_prepass_textures.deferred.as_ref(),
        view_prepass_textures.deferred_lighting_pass_id.as_ref(),
    ) else {
        return;
    };

    if !*logged_ready {
        *logged_ready = true;
        info!("prepass_probe: writer READY, dispatching sphere trace");
    }

    // 1) Compute dispatch: ray-sphere intersection -> scratch t/normal.
    {
        let encoder = ctx.command_encoder();
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("probe_trace_pass"),
            timestamp_writes: None,
        });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &view_group.value, &[view_uniform_offset.offset]);
        cpass.set_bind_group(1, &compute_group.value, &[]);
        let wg_x = extracted_view.viewport.z.div_ceil(8);
        let wg_y = extracted_view.viewport.w.div_ceil(8);
        cpass.dispatch_workgroups(wg_x, wg_y, 1);
    }

    // 2) Blit: write real depth + deferred G-buffer + pass-id from the
    // scratch textures.
    {
        let depth_attachment = view_depth_texture.get_attachment(bevy::render::render_resource::StoreOp::Store);
        let mut render_pass = ctx.begin_tracked_render_pass(RenderPassDescriptor {
            label: Some("probe_blit_pass"),
            color_attachments: &[Some(deferred.get_attachment()), Some(pass_id.get_attachment())],
            depth_stencil_attachment: Some(depth_attachment),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        render_pass.set_render_pipeline(blit_pipeline);
        render_pass.set_bind_group(0, &view_group.value, &[view_uniform_offset.offset]);
        render_pass.set_bind_group(1, &blit_group.value, &[]);
        render_pass.draw(0..3, 0..1);
    }

    // 3) Copy the real depth buffer into ViewPrepassTextures.depth, exactly
    // like bevy_core_pipeline's own deferred prepass node does after
    // rendering to the view depth texture (deferred/node.rs) — required
    // since the stock deferred lighting shader reconstructs world position
    // from THIS texture, not from any field in the G-buffer itself.
    if let Some(prepass_depth) = &view_prepass_textures.depth {
        ctx.command_encoder().copy_texture_to_texture(
            view_depth_texture.texture.as_image_copy(),
            prepass_depth.texture.texture.as_image_copy(),
            view_prepass_textures.size,
        );
    }
}

// ---------------------------------------------------------------------------
// Shadow-map writer: makes the sphere CAST a shadow via a direct write into
// a light's own `ShadowView` depth attachment, bypassing bevy_pbr's normal
// `queue_shadows`/`Shadow`-phase path (Mesh3d-only, no extension point for a
// non-mesh caster — confirmed by source read). See `sphere_shadow_write.wgsl`
// for the per-texel ray-sphere intersection this draws.
//
// Ordering (registered in `super::mod`): after bevy_pbr's own
// `per_view_shadow_pass::<LATE_SHADOW_PASS>`/`shared_shadow_pass::<LATE_SHADOW_PASS>`,
// before `Core3dSystems::MainPass`. This isn't just "don't get overwritten" —
// `DepthAttachment::get_attachment`'s internal "first call this frame" flag
// decides LoadOp::Clear vs LoadOp::Load; running after Bevy's own mesh
// shadow draw makes OUR call the non-first one, so we correctly get
// LoadOp::Load and composite atop (not erase) any real mesh shadow caster
// already rendered into the same shadow map.
// ---------------------------------------------------------------------------

/// Builds the per-shadow-view uniform bind group and issues the draw — the
/// part shared between the point/spot and directional entry points below
/// (they differ only in how they enumerate which `ShadowView`s to hit).
#[allow(clippy::too_many_arguments)]
fn write_sphere_shadow(
    sphere: &SdfProbeSphere,
    shadow_view: &ShadowView,
    extracted_view: &ExtractedView,
    swp: &ShadowWritePipeline,
    pipeline_cache: &PipelineCache,
    render_device: &RenderDevice,
    render_queue: &RenderQueue,
    ctx: &mut RenderContext,
) {
    let Some(pipeline) = pipeline_cache.get_render_pipeline(swp.pipeline) else {
        return; // Still compiling.
    };

    let mut sphere_uniform = UniformBuffer::from(ShadowSphereUniform {
        center: sphere.center,
        radius: sphere.radius,
    });
    sphere_uniform.write_buffer(render_device, render_queue);
    let Some(sphere_binding) = sphere_uniform.binding() else {
        return;
    };

    // world_from_clip inverted once here (glam, CPU-side) rather than in
    // WGSL — see sphere_shadow_write.wgsl's header comment.
    let view_from_world = extracted_view.world_from_view.to_matrix().inverse();
    let clip_from_world = extracted_view
        .clip_from_world
        .unwrap_or(extracted_view.clip_from_view * view_from_world);
    let world_from_clip = (extracted_view.clip_from_view * view_from_world).inverse();
    let mut view_uniform = UniformBuffer::from(ShadowViewUniform {
        world_from_clip,
        clip_from_world,
        world_position: extracted_view.world_from_view.translation(),
        res_x: extracted_view.viewport.z as f32,
        res_y: extracted_view.viewport.w as f32,
    });
    view_uniform.write_buffer(render_device, render_queue);
    let Some(view_binding) = view_uniform.binding() else {
        return;
    };

    let bind_group = render_device.create_bind_group(
        "probe_shadow_write_bind_group",
        &pipeline_cache.get_bind_group_layout(&swp.layout),
        &BindGroupEntries::sequential((sphere_binding, view_binding)),
    );

    let depth_attachment = shadow_view.depth_attachment.get_attachment(StoreOp::Store);
    let mut render_pass = ctx.begin_tracked_render_pass(RenderPassDescriptor {
        label: Some("probe_shadow_write_pass"),
        color_attachments: &[],
        depth_stencil_attachment: Some(depth_attachment),
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    });
    render_pass.set_render_pipeline(pipeline);
    render_pass.set_bind_group(0, &bind_group, &[]);
    render_pass.draw(0..3, 0..1);
}

/// Point/spot light shadow views are `Core3d` ROOT views (the whole schedule
/// reruns once per cube face / spot light) — a plain `ViewQuery` here fires
/// automatically once per such view, no manual enumeration needed.
pub fn probe_shadow_write_point_spot(
    view: ViewQuery<(&'static ShadowView, &'static ExtractedView, &'static LightEntity)>,
    spheres: Query<&SdfProbeSphere>,
    swp: Option<Res<ShadowWritePipeline>>,
    pipeline_cache: Res<PipelineCache>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    mut ctx: RenderContext,
) {
    let Some(swp) = swp.as_deref() else { return };
    let (shadow_view, extracted_view, light_entity) = view.into_inner();
    // Only Point/Spot views should reach us given how this system is
    // registered, but the match keeps the branch explicit/defensive rather
    // than assuming.
    if matches!(light_entity, LightEntity::Directional { .. }) {
        return;
    }
    // `--stress N`: every grid instance's sphere casts its own shadow into
    // this same shadow-map view — each draw after the first correctly gets
    // LoadOp::Load (DepthAttachment's "first call this frame" flag flips on
    // the first draw), so N spheres' shadows composite via the ordinary
    // depth test, same as compositing against a real mesh caster.
    for sphere in &spheres {
        write_sphere_shadow(
            sphere,
            shadow_view,
            extracted_view,
            swp,
            &pipeline_cache,
            &render_device,
            &render_queue,
            &mut ctx,
        );
    }
}

/// Directional light cascades are NOT root views — they hang off the real
/// camera's `ViewLightEntities` component instead, so this system fires on
/// the camera and iterates each cascade's own shadow-view entity manually.
#[allow(clippy::too_many_arguments)]
pub fn probe_shadow_write_directional(
    view: ViewQuery<&'static ViewLightEntities>,
    shadow_views: Query<(&ShadowView, &ExtractedView, &LightEntity)>,
    spheres: Query<&SdfProbeSphere>,
    swp: Option<Res<ShadowWritePipeline>>,
    pipeline_cache: Res<PipelineCache>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    mut ctx: RenderContext,
) {
    let Some(swp) = swp.as_deref() else { return };
    let view_light_entities = view.into_inner();
    for &light_view_entity in &view_light_entities.lights {
        let Ok((shadow_view, extracted_view, light_entity)) = shadow_views.get(light_view_entity)
        else {
            continue;
        };
        if !matches!(light_entity, LightEntity::Directional { .. }) {
            continue;
        }
        // See probe_shadow_write_point_spot's matching comment: one draw per
        // grid instance, composited via the ordinary depth test.
        for sphere in &spheres {
            write_sphere_shadow(
                sphere,
                shadow_view,
                extracted_view,
                swp,
                &pipeline_cache,
                &render_device,
                &render_queue,
                &mut ctx,
            );
        }
    }
}
