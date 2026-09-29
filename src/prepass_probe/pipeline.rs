//! Pipelines, scratch textures, and bind groups for the prepass-probe spike.
//! Mirrors `crate::hybrid::pipeline`'s shape (compute-then-blit, group 0 =
//! Bevy's View uniform, group 1 = our own data) but much smaller: one sphere,
//! one small uniform, two scratch storage textures instead of a full
//! BVH/CSG/object-array upload.

use bevy::core_pipeline::FullscreenShader;
use bevy::core_pipeline::core_3d::CORE_3D_DEPTH_FORMAT;
use bevy::math::UVec2;
use bevy::prelude::*;
use bevy::render::render_resource::binding_types::{
    texture_2d, texture_storage_2d, uniform_buffer,
};
use bevy::render::render_resource::*;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::view::{ExtractedView, ViewUniform, ViewUniforms};

use super::extract::{MAX_PROBE_LIGHTS, ProbeLights};
use super::{SdfProbeGroundPlane, SdfProbeSphere};

fn probe_view_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "probe_view_layout",
        &BindGroupLayoutEntries::single(
            ShaderStages::FRAGMENT | ShaderStages::COMPUTE,
            uniform_buffer::<ViewUniform>(true),
        ),
    )
}

fn probe_compute_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "probe_compute_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SphereUniform>(false),
                uniform_buffer::<PlaneUniform>(false),
                uniform_buffer::<ProbeLightsUniform>(false),
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly),
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly),
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly),
            ),
        ),
    )
}

fn probe_blit_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "probe_blit_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::FRAGMENT,
            (
                uniform_buffer::<SphereUniform>(false),
                texture_2d(TextureSampleType::Float { filterable: false }),
                texture_2d(TextureSampleType::Float { filterable: false }),
                texture_2d(TextureSampleType::Float { filterable: false }),
            ),
        ),
    )
}

/// `--stress N`: every grid instance shares one material/radius, only
/// position differs — see this module's `prepare_probe_scene` doc comment
/// and `sphere_prepass_trace.wgsl`'s matching constant for why this is a
/// small fixed-size array rather than a full per-instance uniform array or a
/// storage buffer.
pub const MAX_STRESS_INSTANCES: usize = 16;

/// Mirrors `sphere_prepass_trace.wgsl`'s `SphereUniform` struct exactly
/// (std140 layout: every scalar field here is naturally 4-byte aligned and
/// each vec3 already ends on a 16-byte boundary given the f32 that follows,
/// so no explicit padding is needed for those — `encase`, which Bevy's
/// `UniformBuffer` uses internally, computes the layout itself from the
/// derive below; `centers` uses `Vec4` even though only `.xyz` is used,
/// matching std140's array-of-vec3 stride-must-be-16-bytes rule).
/// `sphere_prepass_blit.wgsl`'s own `SphereUniform` is a prefix of this same
/// layout (it never needs per-instance position) — see that shader's struct
/// doc comment.
#[derive(Clone, Copy, Debug, ShaderType)]
pub struct SphereUniform {
    pub radius: f32,
    pub base_color: Vec3,
    pub metallic: f32,
    pub roughness: f32,
    pub reflectance: f32,
    pub res_x: f32,
    pub res_y: f32,
    pub instance_count: u32,
    pub centers: [Vec4; MAX_STRESS_INSTANCES],
}

impl Default for SphereUniform {
    fn default() -> Self {
        Self {
            radius: 1.0,
            base_color: Vec3::ONE,
            metallic: 0.0,
            roughness: 0.5,
            reflectance: 0.5,
            res_x: 0.0,
            res_y: 0.0,
            instance_count: 0,
            centers: [Vec4::ZERO; MAX_STRESS_INSTANCES],
        }
    }
}

/// Mirrors `sphere_prepass_trace.wgsl`'s `PlaneUniform` — the reflection
/// ray's own analytic copy of every grid instance's ground plane geometry
/// (see `SdfProbeGroundPlane`'s doc comment). `instances[i] = (center.x,
/// y, center.z, _)` — each plane is fully self-describing, NOT paired with
/// `SphereUniform`'s `centers[i]` by array index (see
/// `SdfProbeGroundPlane`'s doc comment for why: query iteration order
/// between the two component types isn't guaranteed to match).
#[derive(Clone, Copy, Debug, ShaderType)]
pub struct PlaneUniform {
    pub half_size: f32,
    pub base_color: Vec3,
    pub roughness: f32,
    pub instance_count: u32,
    pub instances: [Vec4; MAX_STRESS_INSTANCES],
}

impl Default for PlaneUniform {
    fn default() -> Self {
        Self {
            half_size: 4.0,
            base_color: Vec3::splat(0.5),
            roughness: 0.3,
            instance_count: 0,
            instances: [Vec4::ZERO; MAX_STRESS_INSTANCES],
        }
    }
}

/// Mirrors `sphere_prepass_trace.wgsl`'s `ProbeLight`/`ProbeLightsUniform` —
/// `encase` requires array-typed uniform fields to have a fixed size known
/// at compile time, hence the fixed `MAX_PROBE_LIGHTS`-length array rather
/// than a runtime-sized one (a storage buffer would allow that, but a small
/// fixed uniform is simpler for this spike's light count).
#[derive(Clone, Copy, Debug, Default, ShaderType)]
pub struct ProbeLightUniform {
    pub position: Vec3,
    pub color: Vec3,
    pub intensity: f32,
}

#[derive(Clone, Copy, Debug, ShaderType)]
pub struct ProbeLightsUniform {
    pub count: u32,
    pub lights: [ProbeLightUniform; MAX_PROBE_LIGHTS],
}

impl Default for ProbeLightsUniform {
    fn default() -> Self {
        Self {
            count: 0,
            lights: [ProbeLightUniform::default(); MAX_PROBE_LIGHTS],
        }
    }
}

#[derive(Resource)]
pub struct ProbePipeline {
    pub view_layout: BindGroupLayoutDescriptor,
    pub compute_layout: BindGroupLayoutDescriptor,
    pub blit_layout: BindGroupLayoutDescriptor,
    pub blit_shader: Handle<Shader>,
    pub fullscreen_shader: FullscreenShader,
    pub trace_pipeline: CachedComputePipelineId,
    pub blit_pipeline: CachedRenderPipelineId,
}

pub fn init_probe_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    fullscreen_shader: Res<FullscreenShader>,
    pipeline_cache: ResMut<PipelineCache>,
) {
    commands.init_resource::<ProbeTargetsRes>();
    let trace_shader = asset_server.load("shaders/sphere_prepass_trace.wgsl");
    let blit_shader: Handle<Shader> = asset_server.load("shaders/sphere_prepass_blit.wgsl");
    let view_layout = probe_view_layout();
    let compute_layout = probe_compute_layout();
    let blit_layout = probe_blit_layout();

    let trace_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("probe_trace_pipeline".into()),
        layout: vec![view_layout.clone(), compute_layout.clone()],
        shader: trace_shader,
        entry_point: Some("trace".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    // Deferred G-buffer + pass-id color targets, no MSAA (deferred textures
    // are always single-sampled — see this crate's `Core3dPlugin::prepare_prepass_textures`).
    let blit_pipeline = pipeline_cache.queue_render_pipeline(RenderPipelineDescriptor {
        label: Some("probe_blit_pipeline".into()),
        layout: vec![view_layout.clone(), blit_layout.clone()],
        vertex: fullscreen_shader.to_vertex_state(),
        fragment: Some(FragmentState {
            shader: blit_shader.clone(),
            entry_point: Some("fragment".into()),
            targets: vec![
                Some(ColorTargetState {
                    format: TextureFormat::Rgba32Uint,
                    blend: None,
                    write_mask: ColorWrites::ALL,
                }),
                Some(ColorTargetState {
                    format: TextureFormat::R8Uint,
                    blend: None,
                    write_mask: ColorWrites::ALL,
                }),
            ],
            ..default()
        }),
        primitive: PrimitiveState::default(),
        // Same "we own this frame's depth" reasoning as hybrid_blit's pipeline
        // (see crate::hybrid::pipeline::HybridPipeline::specialize's comment):
        // Always+write, since our fragment computes the true reverse-Z depth
        // for every covered pixel from the analytic sphere hit.
        depth_stencil: Some(DepthStencilState {
            format: CORE_3D_DEPTH_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(CompareFunction::Always),
            stencil: StencilState::default(),
            bias: DepthBiasState::default(),
        }),
        multisample: MultisampleState::default(),
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(ProbePipeline {
        view_layout,
        compute_layout,
        blit_layout,
        blit_shader,
        fullscreen_shader: fullscreen_shader.clone(),
        trace_pipeline,
        blit_pipeline,
    });
}

// ---------------------------------------------------------------------------------
// Shadow-map writer — makes the sphere CAST a shadow, via a direct write into
// a light's own ShadowView depth attachment (see pass.rs's
// `probe_shadow_write_point_spot`/`probe_shadow_write_directional` and
// `sphere_shadow_write.wgsl`'s header comment for why this bypasses
// bevy_pbr's normal Mesh3d-only `queue_shadows` path entirely).
// ---------------------------------------------------------------------------------

/// Mirrors `sphere_shadow_write.wgsl`'s `SphereUniform` — deliberately just
/// center/radius, no material fields (a shadow map only needs depth).
#[derive(Clone, Copy, Debug, Default, ShaderType)]
pub struct ShadowSphereUniform {
    pub center: Vec3,
    pub radius: f32,
}

/// Mirrors `sphere_shadow_write.wgsl`'s `ShadowViewUniform` — one light-view's
/// own matrices (NOT the main camera's), rebuilt per shadow-view each frame
/// in `pass.rs` from that view's own `ExtractedView`.
#[derive(Clone, Copy, Debug, Default, ShaderType)]
pub struct ShadowViewUniform {
    pub world_from_clip: Mat4,
    pub clip_from_world: Mat4,
    pub world_position: Vec3,
    pub res_x: f32,
    pub res_y: f32,
}

fn shadow_write_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "probe_shadow_write_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::FRAGMENT,
            (
                uniform_buffer::<ShadowSphereUniform>(false),
                uniform_buffer::<ShadowViewUniform>(false),
            ),
        ),
    )
}

#[derive(Resource)]
pub struct ShadowWritePipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub shader: Handle<Shader>,
    pub pipeline: CachedRenderPipelineId,
}

pub fn init_shadow_write_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: ResMut<PipelineCache>,
) {
    let shader: Handle<Shader> = asset_server.load("shaders/sphere_shadow_write.wgsl");
    let layout = shadow_write_layout();

    let pipeline = pipeline_cache.queue_render_pipeline(RenderPipelineDescriptor {
        label: Some("probe_shadow_write_pipeline".into()),
        layout: vec![layout.clone()],
        vertex: VertexState {
            shader: shader.clone(),
            entry_point: Some("vs_main".into()),
            buffers: vec![],
            ..default()
        },
        fragment: Some(FragmentState {
            shader: shader.clone(),
            entry_point: Some("fragment".into()),
            targets: vec![],
            ..default()
        }),
        primitive: PrimitiveState::default(),
        // Same reverse-Z convention as bevy_pbr's own shadow maps (see this
        // pipeline's module doc comment) — GreaterEqual/write so a nearer
        // (larger, reverse-Z) sphere depth correctly wins over whatever a
        // real mesh shadow caster already wrote (LoadOp::Load, see pass.rs),
        // and vice versa.
        depth_stencil: Some(DepthStencilState {
            format: CORE_3D_DEPTH_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(CompareFunction::GreaterEqual),
            stencil: StencilState::default(),
            bias: DepthBiasState::default(),
        }),
        multisample: MultisampleState::default(),
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(ShadowWritePipeline {
        layout,
        shader,
        pipeline,
    });
}

#[derive(Resource)]
pub struct ProbeTargets {
    pub size: UVec2,
    pub t_view: TextureView,
    pub normal_view: TextureView,
    pub reflect_view: TextureView,
}

#[derive(Resource, Default)]
pub struct ProbeTargetsRes(pub Option<ProbeTargets>);

#[derive(Resource)]
pub struct ProbeComputeBindGroup {
    pub value: BindGroup,
}

#[derive(Resource)]
pub struct ProbeBlitBindGroup {
    pub value: BindGroup,
}

#[derive(Component)]
pub struct ProbeViewBindGroup {
    pub value: BindGroup,
}

/// Uploads the sphere uniform, (re)creates the scratch storage textures on
/// resize, rebuilds both group-1 bind groups, and the per-view group-0 bind
/// group — everything `pass::probe_prepass_write` needs, all in one system
/// (mirrors `crate::hybrid::pipeline::prepare_hybrid_scene`'s combined shape).
#[allow(clippy::too_many_arguments)]
pub fn prepare_probe_scene(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    pipeline_cache: Res<PipelineCache>,
    probe_pipeline: Option<Res<ProbePipeline>>,
    mut targets: ResMut<ProbeTargetsRes>,
    spheres: Query<&SdfProbeSphere>,
    planes: Query<&SdfProbeGroundPlane>,
    probe_lights: Res<ProbeLights>,
    view_uniforms: Res<ViewUniforms>,
    extracted_views: Query<(Entity, &ExtractedView)>,
) {
    let Some(pp) = probe_pipeline else { return };
    // `--stress N`: every grid instance shares one material (see
    // MAX_STRESS_INSTANCES's doc comment) — take that shared material from
    // the first sphere/plane entity, and every entity's own `center`/`y`
    // becomes one slot in the shared arrays below. `spheres.iter().count()`
    // beyond `MAX_STRESS_INSTANCES` are silently dropped (with a one-time
    // warning) rather than panicking — a spike-appropriate cap, not a hard
    // error.
    let Some(first_sphere) = spheres.iter().next() else {
        return;
    };
    let first_plane = planes.iter().next().copied().unwrap_or_default();

    let Some((_, view)) = extracted_views.iter().next() else {
        return;
    };
    let res_x = view.viewport.z as f32;
    let res_y = view.viewport.w as f32;

    let sphere_list: Vec<&SdfProbeSphere> = spheres.iter().collect();
    if sphere_list.len() > MAX_STRESS_INSTANCES {
        warn!(
            "prepass_probe: {} SdfProbeSphere instances requested, capping at MAX_STRESS_INSTANCES={}",
            sphere_list.len(),
            MAX_STRESS_INSTANCES
        );
    }
    let mut sphere_uniform_data = SphereUniform {
        radius: first_sphere.radius,
        base_color: first_sphere.base_color,
        metallic: first_sphere.metallic,
        roughness: first_sphere.roughness,
        reflectance: first_sphere.reflectance,
        res_x,
        res_y,
        instance_count: sphere_list.len().min(MAX_STRESS_INSTANCES) as u32,
        ..Default::default()
    };
    for (slot, s) in sphere_uniform_data
        .centers
        .iter_mut()
        .zip(sphere_list.iter())
    {
        *slot = s.center.extend(0.0);
    }
    let mut sphere_uniform = UniformBuffer::from(sphere_uniform_data);
    sphere_uniform.write_buffer(&render_device, &render_queue);
    let Some(sphere_binding) = sphere_uniform.binding() else {
        return;
    };

    let plane_list: Vec<&SdfProbeGroundPlane> = planes.iter().collect();
    let mut plane_uniform_data = PlaneUniform {
        half_size: first_plane.half_size,
        base_color: first_plane.base_color,
        roughness: first_plane.roughness,
        instance_count: plane_list.len().min(MAX_STRESS_INSTANCES) as u32,
        ..Default::default()
    };
    for (slot, p) in plane_uniform_data
        .instances
        .iter_mut()
        .zip(plane_list.iter())
    {
        *slot = Vec4::new(p.center_xz.x, p.y, p.center_xz.y, 0.0);
    }
    let mut plane_uniform = UniformBuffer::from(plane_uniform_data);
    plane_uniform.write_buffer(&render_device, &render_queue);
    let Some(plane_binding) = plane_uniform.binding() else {
        return;
    };

    let mut lights_data = ProbeLightsUniform {
        count: probe_lights.lights.len().min(MAX_PROBE_LIGHTS) as u32,
        ..Default::default()
    };
    for (slot, light) in lights_data
        .lights
        .iter_mut()
        .zip(probe_lights.lights.iter())
    {
        *slot = ProbeLightUniform {
            position: light.position,
            color: light.color,
            intensity: light.intensity,
        };
    }
    let mut lights_uniform = UniformBuffer::from(lights_data);
    lights_uniform.write_buffer(&render_device, &render_queue);
    let Some(lights_binding) = lights_uniform.binding() else {
        return;
    };

    // (Re)create scratch textures on resize.
    let want = UVec2::new(res_x as u32, res_y as u32);
    let needs_new = match targets.0.as_ref() {
        Some(t) => t.size != want,
        None => true,
    };
    if needs_new && want.x > 0 && want.y > 0 {
        let make = |format: TextureFormat, label: &str| {
            render_device
                .create_texture(&TextureDescriptor {
                    label: Some(label),
                    size: Extent3d {
                        width: want.x,
                        height: want.y,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: TextureDimension::D2,
                    format,
                    usage: TextureUsages::STORAGE_BINDING | TextureUsages::TEXTURE_BINDING,
                    view_formats: &[],
                })
                .create_view(&TextureViewDescriptor::default())
        };
        targets.0 = Some(ProbeTargets {
            size: want,
            t_view: make(TextureFormat::R32Float, "probe_t"),
            normal_view: make(TextureFormat::Rgba32Float, "probe_normal"),
            reflect_view: make(TextureFormat::Rgba32Float, "probe_reflect"),
        });
    }
    let Some(targets_ref) = targets.0.as_ref() else {
        return;
    };

    let compute_bg = render_device.create_bind_group(
        "probe_compute_bind_group",
        &pipeline_cache.get_bind_group_layout(&pp.compute_layout),
        &BindGroupEntries::sequential((
            sphere_binding.clone(),
            plane_binding,
            lights_binding,
            &targets_ref.t_view,
            &targets_ref.normal_view,
            &targets_ref.reflect_view,
        )),
    );
    commands.insert_resource(ProbeComputeBindGroup { value: compute_bg });

    let blit_bg = render_device.create_bind_group(
        "probe_blit_bind_group",
        &pipeline_cache.get_bind_group_layout(&pp.blit_layout),
        &BindGroupEntries::sequential((
            sphere_binding,
            &targets_ref.t_view,
            &targets_ref.normal_view,
            &targets_ref.reflect_view,
        )),
    );
    commands.insert_resource(ProbeBlitBindGroup { value: blit_bg });

    // Per-view group 0: Bevy's own View uniform.
    let Some(view_binding) = view_uniforms.uniforms.binding() else {
        return;
    };
    for (entity, _) in &extracted_views {
        let bind_group = render_device.create_bind_group(
            "probe_view_bind_group",
            &pipeline_cache.get_bind_group_layout(&pp.view_layout),
            &BindGroupEntries::single(view_binding.clone()),
        );
        commands
            .entity(entity)
            .insert(ProbeViewBindGroup { value: bind_group });
    }
}
