//! Bind group layout + pipeline registration for the physics predict pass
//! — follows `hybrid::pipeline::hybrid_ddgi_layout`/`init_hybrid_pipeline`'s
//! own conventions exactly (single bind group, `queue_compute_pipeline` in
//! a `RenderStartup` system).

use bevy::asset::AssetServer;
use bevy::ecs::system::Commands;
use bevy::prelude::*;
use bevy::render::render_resource::binding_types::{storage_buffer, storage_buffer_read_only, uniform_buffer};
use bevy::render::render_resource::*;

use super::types::{ApplyUniform, ApplyVelocityUniform, BroadphaseScatterUniform, ContactGenUniform, ContactGpu, ExtractPositionsUniform, HashCountUniform, PhysicsAccumulatorGpu, PhysicsBodyGpu, PhysicsPredictUniform, PhysicsShapeGpu, SamplePointsGpu, SamplePointsUniform, ScanUniform, ScatterCopyUniform, ScatterUniform, SubstepStartGpu};

/// The predict pass's single bind group layout: gravity/timestep uniform,
/// the persistent body buffer (`read_write` — predict both reads and
/// mutates it in place), and the substep-start snapshot buffer
/// (`read_write` — written by this pass, read by later apply passes in
/// subsequent pieces).
pub fn physics_predict_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_predict_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<PhysicsPredictUniform>(false),
                storage_buffer::<PhysicsBodyGpu>(false),
                storage_buffer::<SubstepStartGpu>(false),
            ),
        ),
    )
}

/// The position-round contact-scatter pass's bind group layout: contact
/// count/fixed-point-scale uniform, the body buffer (`read`-only — this
/// pass never mutates bodies, only reads position/rotation/inverse-mass/
/// inverse-inertia for the correction math), the contact buffer
/// (`read`-only, uploaded fresh by the CPU each substep), and the
/// accumulator buffer (`read_write` — every invocation `atomicAdd`s into
/// it; note the Rust-side `PhysicsAccumulatorGpu` type used here for size/
/// alignment validation has plain `i32`/`u32` fields while the WGSL struct
/// declares them `atomic<i32>`/`atomic<u32>` — wgpu's binding layout only
/// cares about buffer size/read-write-ness, not field-level atomicity, so
/// this mismatch is intentional and harmless, not a layout bug).
pub fn physics_scatter_position_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_scatter_position_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<ScatterUniform>(false),
                storage_buffer_read_only::<PhysicsBodyGpu>(false),
                storage_buffer_read_only::<ContactGpu>(false),
                storage_buffer::<PhysicsAccumulatorGpu>(false),
            ),
        ),
    )
}

/// The position-round apply-average pass's bind group layout: clamp/
/// damping-constant uniform, the body buffer (`read_write` — this pass
/// both reads current state and applies the correction/damping in
/// place), the substep-start snapshot buffer (`read`-only — written by
/// predict, read here to derive velocity without the position-delta
/// precision bug), and the accumulator buffer (`read`-only from this
/// pass's perspective — it only ever `atomicLoad`s, matching
/// `physics_apply_position.wgsl`'s own doc comment on why a plain,
/// non-atomic Rust mirror type is fine here for the bind-group's size/
/// alignment validation).
pub fn physics_apply_position_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_apply_position_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<ApplyUniform>(false),
                storage_buffer::<PhysicsBodyGpu>(false),
                storage_buffer_read_only::<SubstepStartGpu>(false),
                storage_buffer_read_only::<PhysicsAccumulatorGpu>(false),
            ),
        ),
    )
}

/// The velocity-round apply-average pass's bind group layout — no
/// `SubstepStart` binding (unlike the position round's apply pass, this
/// round only ADDS an impulse to the body's existing velocity, it never
/// derives velocity fresh — see `physics_apply_velocity.wgsl`'s own
/// header comment for why).
pub fn physics_apply_velocity_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_apply_velocity_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (uniform_buffer::<ApplyVelocityUniform>(false), storage_buffer::<PhysicsBodyGpu>(false), storage_buffer_read_only::<PhysicsAccumulatorGpu>(false)),
        ),
    )
}

#[derive(Resource)]
pub struct PhysicsGpuPipeline {
    pub predict_layout: BindGroupLayoutDescriptor,
    pub predict_pipeline: CachedComputePipelineId,
    pub scatter_position_layout: BindGroupLayoutDescriptor,
    pub scatter_position_pipeline: CachedComputePipelineId,
    pub apply_position_layout: BindGroupLayoutDescriptor,
    pub apply_position_pipeline: CachedComputePipelineId,
    /// Shares `scatter_position_layout`'s exact bind group shape (same
    /// `ScatterUniform`/`PhysicsBodyGpu`/`ContactGpu`/`PhysicsAccumulatorGpu`
    /// sequence — see `physics_scatter_velocity.wgsl`'s own header
    /// comment) but is a SEPARATE pipeline since it points at a different
    /// shader/entry point.
    pub scatter_velocity_pipeline: CachedComputePipelineId,
    pub apply_velocity_layout: BindGroupLayoutDescriptor,
    pub apply_velocity_pipeline: CachedComputePipelineId,
}

pub fn init_physics_gpu_pipeline(mut commands: Commands, asset_server: Res<AssetServer>, pipeline_cache: ResMut<PipelineCache>) {
    let predict_shader = asset_server.load("shaders/physics_predict.wgsl");
    let predict_layout = physics_predict_layout();
    let predict_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_predict_pipeline".into()),
        layout: vec![predict_layout.clone()],
        shader: predict_shader,
        entry_point: Some("physics_predict_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let scatter_position_shader = asset_server.load("shaders/physics_scatter_position.wgsl");
    let scatter_position_layout = physics_scatter_position_layout();
    let scatter_position_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_scatter_position_pipeline".into()),
        layout: vec![scatter_position_layout.clone()],
        shader: scatter_position_shader,
        entry_point: Some("physics_scatter_position_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let apply_position_shader = asset_server.load("shaders/physics_apply_position.wgsl");
    let apply_position_layout = physics_apply_position_layout();
    let apply_position_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_apply_position_pipeline".into()),
        layout: vec![apply_position_layout.clone()],
        shader: apply_position_shader,
        entry_point: Some("physics_apply_position_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let scatter_velocity_shader = asset_server.load("shaders/physics_scatter_velocity.wgsl");
    let scatter_velocity_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_scatter_velocity_pipeline".into()),
        layout: vec![scatter_position_layout.clone()], // same bind group shape, see PhysicsGpuPipeline::scatter_velocity_pipeline's own doc comment
        shader: scatter_velocity_shader,
        entry_point: Some("physics_scatter_velocity_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let apply_velocity_shader = asset_server.load("shaders/physics_apply_velocity.wgsl");
    let apply_velocity_layout = physics_apply_velocity_layout();
    let apply_velocity_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_apply_velocity_pipeline".into()),
        layout: vec![apply_velocity_layout.clone()],
        shader: apply_velocity_shader,
        entry_point: Some("physics_apply_velocity_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(PhysicsGpuPipeline {
        predict_layout,
        predict_pipeline,
        scatter_position_layout,
        scatter_position_pipeline,
        apply_position_layout,
        apply_position_pipeline,
        scatter_velocity_pipeline,
        apply_velocity_layout,
        apply_velocity_pipeline,
    });
}

/// The sample-point generation pass's bind group layout: shape-count
/// uniform, the shape buffer (`read`-only — this pass never mutates
/// shapes), and the per-body sample-point output buffer (`read_write`,
/// though this pass only ever writes to it — `read_write` rather than
/// write-only since WGSL storage buffers have no separate write-only
/// binding mode).
pub fn physics_sample_points_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_sample_points_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (uniform_buffer::<SamplePointsUniform>(false), storage_buffer_read_only::<PhysicsShapeGpu>(false), storage_buffer::<SamplePointsGpu>(false)),
        ),
    )
}

#[derive(Resource)]
pub struct SamplePointsGpuPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub pipeline: CachedComputePipelineId,
}

pub fn init_sample_points_gpu_pipeline(mut commands: Commands, asset_server: Res<AssetServer>, pipeline_cache: ResMut<PipelineCache>) {
    let shader = asset_server.load("shaders/physics_sample_points.wgsl");
    let layout = physics_sample_points_layout();
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_sample_points_pipeline".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("physics_sample_points_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(SamplePointsGpuPipeline { layout, pipeline });
}

/// The Hillis-Steele scan pass's single bind group layout, shared by BOTH
/// entry points (`physics_scan_step_main` and
/// `physics_scan_to_exclusive_main`) — same uniform/input/output binding
/// shape either way, only the shader logic differs, matching
/// `physics_scatter_position_layout`'s own precedent for one layout
/// shared by two distinct pipelines.
pub fn physics_scan_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_scan_layout",
        &BindGroupLayoutEntries::sequential(ShaderStages::COMPUTE, (uniform_buffer::<ScanUniform>(false), storage_buffer_read_only::<u32>(false), storage_buffer::<u32>(false))),
    )
}

#[derive(Resource)]
pub struct ScanGpuPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub step_pipeline: CachedComputePipelineId,
    pub to_exclusive_pipeline: CachedComputePipelineId,
}

pub fn init_scan_gpu_pipeline(mut commands: Commands, asset_server: Res<AssetServer>, pipeline_cache: ResMut<PipelineCache>) {
    let shader = asset_server.load("shaders/physics_scan.wgsl");
    let layout = physics_scan_layout();

    let step_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_scan_step_pipeline".into()),
        layout: vec![layout.clone()],
        shader: shader.clone(),
        entry_point: Some("physics_scan_step_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let to_exclusive_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_scan_to_exclusive_pipeline".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("physics_scan_to_exclusive_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(ScanGpuPipeline { layout, step_pipeline, to_exclusive_pipeline });
}

/// The broad-phase hash/count passes' shared bind group layout: hash/
/// count uniform, the positions buffer (`read`-only), the hashes buffer
/// (`read_write` — written by the hash pass, read by the count and
/// scatter passes), and the bucket-counts buffer (`read_write` — every
/// count-pass invocation `atomicAdd`s into it; same intentional plain-
/// Rust-type-vs-atomic-WGSL-declaration mismatch already established for
/// `PhysicsAccumulatorGpu`/`ScanGpuBuffers`).
pub fn physics_broadphase_hash_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_broadphase_hash_layout",
        &BindGroupLayoutEntries::sequential(ShaderStages::COMPUTE, (uniform_buffer::<HashCountUniform>(false), storage_buffer_read_only::<[f32; 4]>(false), storage_buffer::<u32>(false), storage_buffer::<u32>(false))),
    )
}

#[derive(Resource)]
pub struct BroadphaseHashPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub hash_pipeline: CachedComputePipelineId,
    pub count_pipeline: CachedComputePipelineId,
}

pub fn init_broadphase_hash_pipeline(mut commands: Commands, asset_server: Res<AssetServer>, pipeline_cache: ResMut<PipelineCache>) {
    let shader = asset_server.load("shaders/physics_broadphase_hash.wgsl");
    let layout = physics_broadphase_hash_layout();

    let hash_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_broadphase_hash_pipeline".into()),
        layout: vec![layout.clone()],
        shader: shader.clone(),
        entry_point: Some("physics_broadphase_hash_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let count_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_broadphase_count_pipeline".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("physics_broadphase_count_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(BroadphaseHashPipeline { layout, hash_pipeline, count_pipeline });
}

/// The broad-phase copy pass's bind group layout (`bucket_start` ->
/// `cursor`).
pub fn physics_broadphase_copy_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_broadphase_copy_layout",
        &BindGroupLayoutEntries::sequential(ShaderStages::COMPUTE, (uniform_buffer::<ScatterCopyUniform>(false), storage_buffer_read_only::<u32>(false), storage_buffer::<u32>(false))),
    )
}

/// The broad-phase scatter pass's bind group layout: scatter uniform, the
/// hashes buffer (`read`-only), the cursor buffer (`read_write` — every
/// invocation `atomicAdd`s into it), and `bucket_items` (`read_write`,
/// though this pass only ever writes to it — same WGSL-has-no-write-only-
/// storage-mode reasoning as `physics_sample_points_layout`'s own output
/// binding).
pub fn physics_broadphase_scatter_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_broadphase_scatter_layout",
        &BindGroupLayoutEntries::sequential(ShaderStages::COMPUTE, (uniform_buffer::<BroadphaseScatterUniform>(false), storage_buffer_read_only::<u32>(false), storage_buffer::<u32>(false), storage_buffer::<u32>(false))),
    )
}

#[derive(Resource)]
pub struct BroadphaseScatterPipeline {
    pub copy_layout: BindGroupLayoutDescriptor,
    pub copy_pipeline: CachedComputePipelineId,
    pub scatter_layout: BindGroupLayoutDescriptor,
    pub scatter_pipeline: CachedComputePipelineId,
}

pub fn init_broadphase_scatter_pipeline(mut commands: Commands, asset_server: Res<AssetServer>, pipeline_cache: ResMut<PipelineCache>) {
    let shader = asset_server.load("shaders/physics_broadphase_scatter.wgsl");
    let copy_layout = physics_broadphase_copy_layout();
    let scatter_layout = physics_broadphase_scatter_layout();

    let copy_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_broadphase_copy_pipeline".into()),
        layout: vec![copy_layout.clone()],
        shader: shader.clone(),
        entry_point: Some("physics_broadphase_copy_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let scatter_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_broadphase_scatter_pipeline".into()),
        layout: vec![scatter_layout.clone()],
        shader,
        entry_point: Some("physics_broadphase_scatter_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(BroadphaseScatterPipeline { copy_layout, copy_pipeline, scatter_layout, scatter_pipeline });
}

/// The contact-generation passes' shared bind group layout — all three of
/// `physics_contacts_dynamic_main`, `physics_contacts_kinematic_main`, and
/// `physics_contacts_static_main` bind the exact same 8 resources (only
/// their dispatch shape and which body-index pairs they generate differ),
/// same one-layout-many-pipelines precedent as `physics_broadphase_hash_layout`.
/// `bodies`/`shapes`/
/// `sample_points` are all `read`-only (this pass never mutates body
/// state, only queries it); `bucket_start`/`bucket_items` are the broad-
/// phase's own CSR output, also `read`-only here; `contact_cursor` is
/// `read_write` (every invocation that finds a contact `atomicAdd`s into
/// it); `contacts_out` is `read_write` though this pass only ever writes
/// to it (same WGSL-has-no-write-only-storage-mode reasoning as
/// `physics_sample_points_layout`'s own output binding).
pub fn physics_contacts_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_contacts_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<ContactGenUniform>(false),
                storage_buffer_read_only::<PhysicsBodyGpu>(false),
                storage_buffer_read_only::<PhysicsShapeGpu>(false),
                storage_buffer_read_only::<SamplePointsGpu>(false),
                storage_buffer_read_only::<u32>(false),
                storage_buffer_read_only::<u32>(false),
                storage_buffer::<u32>(false),
                storage_buffer::<ContactGpu>(false),
            ),
        ),
    )
}

#[derive(Resource)]
pub struct ContactGenPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub dynamic_pipeline: CachedComputePipelineId,
    pub kinematic_pipeline: CachedComputePipelineId,
    pub static_pipeline: CachedComputePipelineId,
}

pub fn init_contact_gen_pipeline(mut commands: Commands, asset_server: Res<AssetServer>, pipeline_cache: ResMut<PipelineCache>) {
    let shader = asset_server.load("shaders/physics_contacts.wgsl");
    let layout = physics_contacts_layout();

    let dynamic_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_contacts_dynamic_pipeline".into()),
        layout: vec![layout.clone()],
        shader: shader.clone(),
        entry_point: Some("physics_contacts_dynamic_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let kinematic_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_contacts_kinematic_pipeline".into()),
        layout: vec![layout.clone()],
        shader: shader.clone(),
        entry_point: Some("physics_contacts_kinematic_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let static_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_contacts_static_pipeline".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("physics_contacts_static_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(ContactGenPipeline { layout, dynamic_pipeline, kinematic_pipeline, static_pipeline });
}

/// The extract-positions pass's bind group layout (Piece 5's own tiny
/// bridge pass — see `assets/shaders/physics_extract_positions.wgsl`'s
/// own doc comment for why it exists).
pub fn physics_extract_positions_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "physics_extract_positions_layout",
        &BindGroupLayoutEntries::sequential(ShaderStages::COMPUTE, (uniform_buffer::<ExtractPositionsUniform>(false), storage_buffer_read_only::<PhysicsBodyGpu>(false), storage_buffer::<[f32; 4]>(false))),
    )
}

#[derive(Resource)]
pub struct ExtractPositionsPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub pipeline: CachedComputePipelineId,
}

pub fn init_extract_positions_pipeline(mut commands: Commands, asset_server: Res<AssetServer>, pipeline_cache: ResMut<PipelineCache>) {
    let shader = asset_server.load("shaders/physics_extract_positions.wgsl");
    let layout = physics_extract_positions_layout();
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("physics_extract_positions_pipeline".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("physics_extract_positions_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    commands.insert_resource(ExtractPositionsPipeline { layout, pipeline });
}
