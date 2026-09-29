//! Render pipeline, bind groups, and buffer preparation for the raymarch pass — one
//! `RaymarchPipeline`, specialized only on target format/MSAA sample count, since the
//! raymarcher draws exactly once per view: a single full-screen triangle.
//!
//! Bind group layout (must match `assets/shaders/raymarch.wgsl`'s bindings exactly):
//! - group 0, binding 0: `View` uniform.
//! - group 1, binding 0: `RaymarchScene` uniform (tile_period, counts, anim-group ranges).
//! - group 1, binding 1: `primitives` storage buffer (static + all anim groups' records).
//! - group 1, binding 2: `anim_isometries` storage buffer (per-frame live rotations).
//! - group 1, binding 3: `lights` storage buffer (per-frame).

use bevy::core_pipeline::FullscreenShader;
use bevy::core_pipeline::core_3d::CORE_3D_DEPTH_FORMAT;
use bevy::prelude::*;
use bevy::render::render_resource::binding_types::{storage_buffer_read_only, uniform_buffer};
use bevy::render::render_resource::*;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::view::{ExtractedView, ViewUniform, ViewUniforms};

use super::extract::{
    LightKindCpu, RaymarchDebugFlags, RenderRaymarchAnimIsometries, RenderRaymarchLights,
    RenderRaymarchStaticScene,
};
use super::flatten::PrimitiveRecordCpu;

/// Maximum number of `AnimGroup`s the fixed-size `RaymarchScene.anim_group_ranges`
/// array can name — this demo has 1 (the pillar, see `sdf::world`'s
/// `PILLAR_ANIM_GROUP`), with headroom to spare so a runtime-sized uniform array
/// isn't needed (WGSL uniform buffers can't have runtime-sized arrays; only storage
/// buffers can, and this data is tiny/fixed-shape enough that a small fixed-size
/// uniform array is simpler than a second storage buffer just for ranges).
const MAX_ANIM_GROUPS: usize = 8;

/// Mirrors `raymarch.wgsl`'s `RaymarchScene` uniform struct exactly.
#[derive(Clone, Copy, Default, ShaderType)]
pub struct RaymarchSceneUniform {
    pub tile_period: f32,
    pub static_count: u32,
    pub anim_group_count: u32,
    pub light_count: u32,
    /// Seconds since app startup — reused for the shader's own time-driven effects
    /// (currently the rings' emissive pulse, see raymarch.wgsl's `is_glowing_ring`
    /// use). Bevy already extracts `Time` into the render world every frame (see
    /// `bevy_render::globals::GlobalsPlugin`), so `prepare_raymarch_buffers` just reads
    /// it directly rather than wiring up a second bind group for Bevy's own
    /// `Globals`/`GlobalsBuffer` uniform, which this scene uniform's existing
    /// per-frame upload already makes redundant for this one field.
    pub time_secs: f32,
    /// Bitmask of debug flags: bit 0 = disable shadows, bit 1 = disable AO,
    /// bit 2 = disable reflection, bit 3 = step-count heatmap. Fed from the
    /// main world via `RaymarchDebugFlags` resource (see `extract`).
    pub debug_flags: u32,
    /// `(start, count)` into the shared `primitives` buffer per anim group, `id` order
    /// implicit (index == group id) — `x`/`y` used, `z`/`w` padding (WGSL array
    /// elements of `vec4<u32>` avoid the stride surprises a `vec2<u32>` array element
    /// would have under std140-style uniform buffer layout rules).
    pub anim_group_ranges: [UVec4; MAX_ANIM_GROUPS],
}

/// Mirrors `raymarch.wgsl`'s `AnimIsometry` storage-buffer element exactly — see
/// `raymarch::flatten::PrimitiveRecordCpu`'s doc comment for why plain scalar fields
/// (not `Vec3`/`Vec4`) are used: `bytemuck::Pod` requires the Rust layout to exactly
/// match what `encase`'s `ShaderType` derive computes for the WGSL side, which a
/// `glam::Vec3` field can silently violate.
#[derive(Clone, Copy, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct AnimIsometryGpu {
    pub pivot_x: f32,
    pub pivot_y: f32,
    pub pivot_z: f32,
    /// Conservative bounding-sphere radius around `pivot`, in world units — lets
    /// `map()` skip this group's `eval_stack` call entirely for query points far
    /// outside it (see `raymarch.wgsl`'s `map`), the single highest-leverage
    /// performance fix given `map()` is called ~90 times per pixel (sphere-trace
    /// steps + normal taps + shadow/AO steps) and unconditionally evaluated every
    /// AnimGroup on every call otherwise.
    pub bounding_radius: f32,
    pub rotation_x: f32,
    pub rotation_y: f32,
    pub rotation_z: f32,
    pub rotation_w: f32,
}

/// Mirrors `raymarch.wgsl`'s `Light` storage-buffer element exactly.
#[derive(Clone, Copy, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct LightGpu {
    pub kind: u32,
    pub color_r: f32,
    pub color_g: f32,
    pub color_b: f32,
    pub direction_or_position_x: f32,
    pub direction_or_position_y: f32,
    pub direction_or_position_z: f32,
    pub intensity: f32,
    pub spot_direction_x: f32,
    pub spot_direction_y: f32,
    pub spot_direction_z: f32,
    pub range: f32,
    pub inner_angle: f32,
    pub outer_angle: f32,
    /// Soft-shadow penumbra hardness (Quilez's `k` — see
    /// `docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md`): inversely
    /// tied to the light's apparent angular size, larger = harder-edged.
    /// Only read by `hybrid_trace.wgsl`'s `trace_shadow`; `raymarch.wgsl`
    /// still treats this slot as padding.
    pub shadow_softness_k: f32,
    pub _pad1: f32,
}

fn raymarch_view_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "raymarch_view_layout",
        &BindGroupLayoutEntries::single(
            ShaderStages::FRAGMENT,
            uniform_buffer::<ViewUniform>(true),
        ),
    )
}

fn raymarch_scene_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "raymarch_scene_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::FRAGMENT,
            (
                uniform_buffer::<RaymarchSceneUniform>(false),
                storage_buffer_read_only::<PrimitiveRecordCpu>(false),
                storage_buffer_read_only::<AnimIsometryGpu>(false),
                storage_buffer_read_only::<LightGpu>(false),
            ),
        ),
    )
}

#[derive(Resource)]
pub struct RaymarchPipeline {
    pub view_layout: BindGroupLayoutDescriptor,
    pub scene_layout: BindGroupLayoutDescriptor,
    pub shader: Handle<Shader>,
    /// Held only to keep `material.wgsl` loaded/alive — `raymarch.wgsl` `#import`s it
    /// by its `#define_import_path` (`migera::material`), which needs the shader
    /// asset to actually be loaded somewhere for naga_oil's import resolution to find
    /// it (see `assets/shaders/material.wgsl`'s doc comment and this codebase's
    /// docs/knowledge/bevy-rendering/materials-and-shaders/shader-system.md on why
    /// `load_shader_library!`'s `mem::forget` trick exists for exactly this
    /// "imported-only, no direct handle-holding caller" case — this field is the
    /// same idea without that macro, since this project loads shaders as loose
    /// asset files rather than embedding them into a plugin).
    #[allow(dead_code)]
    pub material_shader: Handle<Shader>,
    pub fullscreen_shader: FullscreenShader,
}

#[derive(Clone, PartialEq, Eq, Hash)]
pub struct RaymarchPipelineKey {
    pub target_format: TextureFormat,
    pub msaa_samples: u32,
}

impl SpecializedRenderPipeline for RaymarchPipeline {
    type Key = RaymarchPipelineKey;

    fn specialize(&self, key: Self::Key) -> RenderPipelineDescriptor {
        RenderPipelineDescriptor {
            label: Some("raymarch_pipeline".into()),
            layout: vec![self.view_layout.clone(), self.scene_layout.clone()],
            // Reuses Bevy's own `FullscreenShader`'s vertex state (the same "one
            // oversized triangle, no vertex buffer, 3 bare `@builtin(vertex_index)`
            // vertices" trick `bevy_core_pipeline::fullscreen_material` uses) rather
            // than hand-writing an equivalent — see this module's doc comment on why
            // `FullscreenMaterialPlugin` itself is the wrong tool here (it's a
            // texture-to-texture post-process filter; the raymarcher IS primary
            // visibility, with no "source texture" to sample), but its vertex-state
            // resource is plain, public, and unconditionally initialized by
            // `Core3dPlugin`, so it's still directly reusable standalone.
            vertex: self.fullscreen_shader.to_vertex_state(),
            fragment: Some(FragmentState {
                shader: self.shader.clone(),
                entry_point: Some("fragment".into()),
                targets: vec![Some(ColorTargetState {
                    format: key.target_format,
                    blend: None,
                    write_mask: ColorWrites::ALL,
                })],
                ..default()
            }),
            primitive: PrimitiveState::default(),
            // No depth attachment — this app has no other opaque Core3d geometry, so
            // there's nothing else to depth-test against for v1 (see `pipeline.rs`'s
            // `CORE_3D_DEPTH_FORMAT` note below for the future extension point).
            depth_stencil: None,
            multisample: MultisampleState {
                count: key.msaa_samples,
                ..default()
            },
            immediate_size: 0,
            zero_initialize_workgroup_memory: false,
        }
    }
}

/// `CORE_3D_DEPTH_FORMAT` import currently unused now that `depth_stencil: None` is
/// final for v1 — kept as a `use` here would be dead; referenced in this doc comment
/// instead so a future depth-testing extension finds the right constant without
/// re-deriving it.
#[allow(dead_code)]
const _: () = {
    let _ = CORE_3D_DEPTH_FORMAT;
};

pub fn init_raymarch_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    fullscreen_shader: Res<FullscreenShader>,
) {
    commands.insert_resource(RaymarchPipeline {
        view_layout: raymarch_view_layout(),
        scene_layout: raymarch_scene_layout(),
        shader: asset_server.load("shaders/raymarch.wgsl"),
        material_shader: asset_server.load("shaders/material.wgsl"),
        fullscreen_shader: fullscreen_shader.clone(),
    });
}

// ---------------------------------------------------------------------------------
// View bind group (group 0).
// ---------------------------------------------------------------------------------

#[derive(Component)]
pub struct RaymarchViewBindGroup {
    pub value: BindGroup,
}

pub fn prepare_raymarch_view_bind_groups(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline_cache: Res<PipelineCache>,
    raymarch_pipeline: Res<RaymarchPipeline>,
    view_uniforms: Res<ViewUniforms>,
    views: Query<Entity, With<ExtractedView>>,
) {
    let Some(view_binding) = view_uniforms.uniforms.binding() else {
        return;
    };
    for entity in &views {
        let bind_group = render_device.create_bind_group(
            "raymarch_view_bind_group",
            &pipeline_cache.get_bind_group_layout(&raymarch_pipeline.view_layout),
            &BindGroupEntries::single(view_binding.clone()),
        );
        commands
            .entity(entity)
            .insert(RaymarchViewBindGroup { value: bind_group });
    }
}

// ---------------------------------------------------------------------------------
// Scene bind group (group 1) — one shared bind group for the whole frame (not
// per-view, per-entity): the raymarcher has exactly one scene to evaluate.
// ---------------------------------------------------------------------------------

#[derive(Resource)]
pub struct RaymarchBuffers {
    pub scene_uniform: UniformBuffer<RaymarchSceneUniform>,
    pub primitives: RawBufferVec<PrimitiveRecordCpu>,
    pub anim_isometries: RawBufferVec<AnimIsometryGpu>,
    pub lights: RawBufferVec<LightGpu>,
}

#[derive(Resource)]
pub struct RaymarchSceneBindGroup {
    pub value: BindGroup,
}

pub fn init_raymarch_buffers(mut commands: Commands) {
    commands.insert_resource(RaymarchBuffers {
        scene_uniform: UniformBuffer::default(),
        primitives: RawBufferVec::new(BufferUsages::STORAGE),
        anim_isometries: RawBufferVec::new(BufferUsages::STORAGE),
        lights: RawBufferVec::new(BufferUsages::STORAGE),
    });
}

fn light_kind_gpu(kind: LightKindCpu) -> u32 {
    match kind {
        LightKindCpu::Directional => 0,
        LightKindCpu::Point => 1,
        LightKindCpu::Spot => 2,
    }
}

/// Builds this frame's `RaymarchSceneUniform`/`primitives`/`anim_isometries`/`lights`
/// buffers and the one shared scene bind group pointing at them. Runs every frame (the
/// per-frame-cheap `anim_isometries`/`lights` re-upload dominates; re-uploading the
/// already-flattened static `primitives` list every frame too is simpler than
/// diffing/caching it GPU-side and is still cheap — a few hundred bytes for this demo's
/// scene, not the expensive part of this pipeline).
pub fn prepare_raymarch_buffers(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    pipeline_cache: Res<PipelineCache>,
    raymarch_pipeline: Res<RaymarchPipeline>,
    mut buffers: ResMut<RaymarchBuffers>,
    static_scene: Option<Res<RenderRaymarchStaticScene>>,
    anim_isometries: Option<Res<RenderRaymarchAnimIsometries>>,
    lights: Option<Res<RenderRaymarchLights>>,
    debug_flags: Res<RaymarchDebugFlags>,
    time: Res<Time>,
) {
    let Some(static_scene) = static_scene else {
        return; // Nothing flattened yet (first few frames before setup() spawns the tile cluster).
    };

    buffers.primitives.clear();
    for record in static_scene
        .records
        .iter()
        .chain(static_scene.anim_group_records.iter())
    {
        buffers.primitives.push(*record);
    }
    buffers
        .primitives
        .write_buffer(&render_device, &render_queue);

    let mut anim_group_ranges = [UVec4::ZERO; MAX_ANIM_GROUPS];
    // Anim-group records were appended after the static records above, so their
    // buffer-relative start offsets need the static count added back in.
    let static_count = static_scene.records.len() as u32;
    for &(group_id, start, count) in &static_scene.anim_group_ranges {
        if let Some(slot) = anim_group_ranges.get_mut(group_id as usize) {
            *slot = UVec4::new(static_count + start, count, 0, 0);
        }
    }

    // The shader indexes `anim_isometries` directly by `record.anim_group - 1` (i.e.
    // by group id, see raymarch.wgsl's eval_leaf), NOT by upload/iteration order —
    // Bevy's `Query` iteration order isn't guaranteed to match group-id order, so this
    // must build a dense array indexed by id explicitly rather than pushing entries in
    // whatever order the ECS query produced them. Slots for any id with no live entity
    // this frame (shouldn't happen for this demo's fixed AnimGroup set, but not
    // guaranteed) default to identity, matching PrimitiveRecordCpu::op's own
    // identity-rotation default.
    // Generous fixed bound covering every AnimGroup this demo has (pillar: ~1.6 half-
    // height cylinder + a 0.9-radius bite sphere offset up to (0,0.8,0.7) from pivot;
    // rings: ~0.9 max ring_radius + 0.27 max bead_radius) — not derived from actual
    // per-shape extents (no such tracking exists yet), but comfortably larger than any
    // of them with margin to spare. A future improvement could compute this per-group
    // from the flattened records instead of a single shared constant.
    const ANIM_GROUP_BOUNDING_RADIUS: f32 = 3.0;

    let group_count = static_scene.anim_group_ranges.len();
    let identity = AnimIsometryGpu {
        pivot_x: 0.0,
        pivot_y: 0.0,
        pivot_z: 0.0,
        bounding_radius: ANIM_GROUP_BOUNDING_RADIUS,
        rotation_x: 0.0,
        rotation_y: 0.0,
        rotation_z: 0.0,
        rotation_w: 1.0,
    };
    let mut dense_isometries = vec![identity; group_count.max(1)];
    if let Some(anim_isometries) = &anim_isometries {
        for iso in &anim_isometries.0 {
            if let Some(slot) = dense_isometries.get_mut(iso.group_id as usize) {
                *slot = AnimIsometryGpu {
                    pivot_x: iso.pivot.x,
                    pivot_y: iso.pivot.y,
                    pivot_z: iso.pivot.z,
                    bounding_radius: ANIM_GROUP_BOUNDING_RADIUS,
                    rotation_x: iso.rotation.x,
                    rotation_y: iso.rotation.y,
                    rotation_z: iso.rotation.z,
                    rotation_w: iso.rotation.w,
                };
            }
        }
    }

    buffers.anim_isometries.clear();
    for iso in dense_isometries {
        buffers.anim_isometries.push(iso);
    }
    buffers
        .anim_isometries
        .write_buffer(&render_device, &render_queue);

    let light_count = lights.as_ref().map_or(0, |l| l.0.len() as u32);
    buffers.lights.clear();
    if let Some(lights) = &lights {
        for light in &lights.0 {
            buffers.lights.push(LightGpu {
                kind: light_kind_gpu(light.kind),
                color_r: light.color.x,
                color_g: light.color.y,
                color_b: light.color.z,
                direction_or_position_x: light.direction_or_position.x,
                direction_or_position_y: light.direction_or_position.y,
                direction_or_position_z: light.direction_or_position.z,
                intensity: light.intensity,
                spot_direction_x: light.spot_direction.x,
                spot_direction_y: light.spot_direction.y,
                spot_direction_z: light.spot_direction.z,
                range: light.range,
                inner_angle: light.inner_angle,
                outer_angle: light.outer_angle,
                shadow_softness_k: 0.0,
                _pad1: 0.0,
            });
        }
    }
    if buffers.lights.is_empty() {
        buffers.lights.push(LightGpu {
            kind: 0,
            color_r: 0.0,
            color_g: 0.0,
            color_b: 0.0,
            direction_or_position_x: 0.0,
            direction_or_position_y: -1.0,
            direction_or_position_z: 0.0,
            intensity: 0.0,
            spot_direction_x: 0.0,
            spot_direction_y: 0.0,
            spot_direction_z: 0.0,
            range: 0.0,
            inner_angle: 0.0,
            outer_angle: 0.0,
            shadow_softness_k: 0.0,
            _pad1: 0.0,
        });
    }
    buffers.lights.write_buffer(&render_device, &render_queue);

    buffers.scene_uniform.set(RaymarchSceneUniform {
        tile_period: static_scene.tile_period,
        static_count,
        anim_group_count: static_scene.anim_group_ranges.len() as u32,
        light_count,
        time_secs: time.elapsed_secs(),
        debug_flags: debug_flags.0,
        anim_group_ranges,
    });
    buffers
        .scene_uniform
        .write_buffer(&render_device, &render_queue);

    let (Some(scene_binding), Some(primitives_binding), Some(anim_binding), Some(lights_binding)) = (
        buffers.scene_uniform.binding(),
        buffers.primitives.buffer().map(|b| b.as_entire_binding()),
        buffers
            .anim_isometries
            .buffer()
            .map(|b| b.as_entire_binding()),
        buffers.lights.buffer().map(|b| b.as_entire_binding()),
    ) else {
        return;
    };

    let bind_group = render_device.create_bind_group(
        "raymarch_scene_bind_group",
        &pipeline_cache.get_bind_group_layout(&raymarch_pipeline.scene_layout),
        &BindGroupEntries::sequential((
            scene_binding,
            primitives_binding,
            anim_binding,
            lights_binding,
        )),
    );
    commands.insert_resource(RaymarchSceneBindGroup { value: bind_group });
}
