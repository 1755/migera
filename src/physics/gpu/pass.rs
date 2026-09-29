//! Dispatch system for the physics predict pass — deliberately NOT a
//! `Core3d`/`ViewQuery`-scoped system like `hybrid::pass::hybrid_pass`:
//! physics has no camera/view dependency (it's a simulation step, not a
//! rendering pass), so it runs as a plain `Render`-schedule system in
//! `RenderSystems::Render` that builds its own `CommandEncoder` and
//! submits directly — the same pattern Bevy's own `render_system`
//! (`bevy_render::renderer::render_system`) and `slab_allocator`'s buffer-
//! growth use internally for view-independent GPU work. wgpu serializes
//! queue submissions in submission order, so an extra `submit` here does
//! not race the main per-view command buffer.

use bevy::render::render_resource::{BindGroupEntries, BufferDescriptor, BufferUsages, CommandEncoderDescriptor, ComputePassDescriptor, MapMode, PollType};
use bevy::render::renderer::{RenderDevice, RenderQueue};

use super::buffers::{BroadphaseGpuState, ContactGenGpuState, PhysicsGpuState, SamplePointsGpuState, ScanGpuState, write_scan_uniform};
use super::pipelines::{BroadphaseHashPipeline, BroadphaseScatterPipeline, ContactGenPipeline, ExtractPositionsPipeline, PhysicsGpuPipeline, SamplePointsGpuPipeline, ScanGpuPipeline};

/// Synchronous GPU->CPU buffer readback: copy into a `MAP_READ` staging
/// buffer, submit, block via `RenderDevice::poll(PollType::wait_indefinitely())`
/// until the map callback fires. **Deliberately blocking** — unlike the
/// per-frame body-state readback the whole one-frame-latency design exists
/// to avoid (see `physics::gpu::readback`'s own doc comment), this is used
/// ONLY by `sample_cache::ensure_sample_points_cached` on a cache MISS
/// (a genuinely new shape parameterization appearing for the first time),
/// which is rare in practice (shape kind/parameters are set at spawn time
/// and essentially never mutated in this codebase today — see
/// `sample_cache.rs`'s own doc comment) and small (the sample-point output
/// for however many new shapes appeared this frame, not the full body
/// state). A one-time, explicitly-documented stall on an infrequent path
/// is an accepted tradeoff, not a violation of the no-synchronous-
/// mid-frame-readback doctrine that design targets the PER-FRAME path.
pub fn read_buffer_sync<T: bytemuck::Pod>(render_device: &RenderDevice, render_queue: &RenderQueue, buffer: &bevy::render::render_resource::Buffer, count: usize) -> Vec<T> {
    let byte_size = (count * std::mem::size_of::<T>()) as u64;
    let device = render_device.wgpu_device();

    let staging = device.create_buffer(&BufferDescriptor {
        label: Some("physics_gpu_readback_staging"),
        size: byte_size,
        usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor { label: Some("physics_gpu_readback_encoder") });
    encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, byte_size);
    render_queue.submit(std::iter::once(encoder.finish()));

    let slice = staging.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(MapMode::Read, move |result| {
        tx.send(result).expect("readback channel should still be open");
    });
    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");
    rx.recv().expect("map_async callback should have fired after a blocking poll").expect("buffer map should succeed");

    let data = slice.get_mapped_range();
    let result: Vec<T> = bytemuck::cast_slice(&data).to_vec();
    drop(data);
    staging.unmap();
    result
}

/// Dispatches the predict pass once, over `body_count` bodies (workgroup
/// size 64, matching this pass's own `@workgroup_size(64, 1, 1)` — see
/// `physics_predict.wgsl`'s own doc comment for why a flat 1D dispatch is
/// the right shape for "N bodies" work, unlike the 2D-viewport-shaped
/// convention every pixel pass in this codebase uses).
///
/// Piece 1 only: called directly by the parity test harness with a known
/// body set, not yet wired into the real per-frame render path (see
/// `gpu/mod.rs`'s own doc comment on the `PhysicsGpuEnabled` opt-in
/// decision — this lands once the full substep is ported, not before).
pub fn dispatch_physics_predict(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &PhysicsGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &PhysicsGpuState) {
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.predict_pipeline) else {
        return; // Pipeline still compiling -- normal at startup, same convention as hybrid_pass's own early return.
    };
    let Some(uniform_binding) = buffers.predict_uniform.binding() else { return };

    let bind_group = render_device.create_bind_group(
        "physics_predict_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.predict_layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.bodies.as_entire_buffer_binding(), buffers.substep_start.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_predict_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_predict_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = buffers.body_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Dispatches the position-round contact-scatter pass once, over the
/// current substep's contact count (workgroup size 64, same 1D-dispatch
/// shape as predict — see that function's own doc comment). No-ops if
/// there are zero contacts this substep (nothing to scatter) or if the
/// accumulator hasn't been cleared/allocated yet.
///
/// Piece 2 only: called directly by its own parity test, not yet wired
/// into the real per-frame render path — see `gpu/mod.rs`'s own doc
/// comment on the `PhysicsGpuEnabled` opt-in decision.
pub fn dispatch_physics_scatter_position(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &PhysicsGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &PhysicsGpuState, contact_count: u32) {
    if contact_count == 0 {
        return;
    }
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(contacts_buffer) = buffers.contacts.as_ref() else { return };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.scatter_position_pipeline) else {
        return; // Pipeline still compiling -- same convention as dispatch_physics_predict.
    };
    let Some(uniform_binding) = buffers.scatter_uniform.binding() else { return };

    let bind_group = render_device.create_bind_group(
        "physics_scatter_position_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.scatter_position_layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.bodies.as_entire_buffer_binding(), contacts_buffer.as_entire_buffer_binding(), buffers.accumulators.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_scatter_position_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_scatter_position_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = contact_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Same as `dispatch_physics_scatter_position`, but reads contacts from an
/// EXTERNALLY-provided buffer rather than `PhysicsGpuState`'s own
/// CPU-uploaded `contacts` field — the real per-frame GPU path never
/// uploads contacts from the CPU at all (unlike every earlier piece's own
/// parity tests, which fed a CPU-computed `Vec<Contact>` through
/// `upload_contacts`); it consumes `ContactGenGpuBuffers::contacts_out`
/// directly, GPU-to-GPU, with no CPU round-trip. Same `ContactGpu` layout
/// either way (`physics_scatter_position.wgsl`'s own `array<Contact>`
/// binding doesn't care which buffer produced the bytes), so this is
/// purely a "which buffer" parameterization, not new shader logic.
pub fn dispatch_physics_scatter_position_from_buffer(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &PhysicsGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &PhysicsGpuState, contacts_buffer: &bevy::render::render_resource::Buffer, contact_count: u32) {
    if contact_count == 0 {
        return;
    }
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.scatter_position_pipeline) else {
        return; // Pipeline still compiling -- same convention as dispatch_physics_predict.
    };
    let Some(uniform_binding) = buffers.scatter_uniform.binding() else { return };

    let bind_group = render_device.create_bind_group(
        "physics_scatter_position_from_buffer_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.scatter_position_layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.bodies.as_entire_buffer_binding(), contacts_buffer.as_entire_buffer_binding(), buffers.accumulators.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_scatter_position_from_buffer_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_scatter_position_from_buffer_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = contact_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Same as `dispatch_physics_scatter_velocity`, but reads contacts from an
/// externally-provided buffer — see
/// `dispatch_physics_scatter_position_from_buffer`'s own doc comment for
/// why (identical reasoning, velocity round instead of position round).
pub fn dispatch_physics_scatter_velocity_from_buffer(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &PhysicsGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &PhysicsGpuState, contacts_buffer: &bevy::render::render_resource::Buffer, contact_count: u32) {
    if contact_count == 0 {
        return;
    }
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.scatter_velocity_pipeline) else {
        return; // Pipeline still compiling -- same convention as dispatch_physics_predict.
    };
    let Some(uniform_binding) = buffers.scatter_uniform.binding() else { return };

    let bind_group = render_device.create_bind_group(
        "physics_scatter_velocity_from_buffer_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.scatter_position_layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.bodies.as_entire_buffer_binding(), contacts_buffer.as_entire_buffer_binding(), buffers.accumulators.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_scatter_velocity_from_buffer_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_scatter_velocity_from_buffer_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = contact_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Dispatches the position-round apply-average pass once, over
/// `body_count` bodies (same 1D-dispatch shape as predict/scatter — see
/// `dispatch_physics_predict`'s own doc comment). Must run AFTER
/// `dispatch_physics_scatter_position` within the same substep (reads the
/// accumulator buffer that pass filled) — this function does not enforce
/// that ordering itself, callers are responsible for sequencing the two
/// dispatches correctly, same as the CPU reference's own scatter-then-
/// apply loop structure within `solve_substep_jacobi`.
///
/// Piece 3 only: called directly by its own parity test, not yet wired
/// into the real per-frame render path — see `gpu/mod.rs`'s own doc
/// comment on the `PhysicsGpuEnabled` opt-in decision.
pub fn dispatch_physics_apply_position(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &PhysicsGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &PhysicsGpuState) {
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.apply_position_pipeline) else {
        return; // Pipeline still compiling -- same convention as dispatch_physics_predict.
    };
    let Some(uniform_binding) = buffers.apply_uniform.binding() else { return };

    let bind_group = render_device.create_bind_group(
        "physics_apply_position_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.apply_position_layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.bodies.as_entire_buffer_binding(), buffers.substep_start.as_entire_buffer_binding(), buffers.accumulators.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_apply_position_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_apply_position_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = buffers.body_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Dispatches the velocity-round contact-scatter pass once, over the
/// current substep's contact count — same 1D-dispatch shape and bind
/// group layout as `dispatch_physics_scatter_position` (see
/// `PhysicsGpuPipeline::scatter_velocity_pipeline`'s own doc comment for
/// why they share a layout but are separate pipelines). Must run AFTER
/// `dispatch_physics_apply_position` within the same substep — the
/// velocity round reads each body's ALREADY-corrected position/rotation
/// (needed for the lever-arm/relative-velocity math) and ALREADY-damped
/// velocity (needed so this round only adds an impulse on top of, not
/// instead of, the position round's own result) — same caller-responsible
/// sequencing convention as every other dispatch function here.
///
/// Piece 4 only: called directly by its own parity test, not yet wired
/// into the real per-frame render path — see `gpu/mod.rs`'s own doc
/// comment on the `PhysicsGpuEnabled` opt-in decision.
pub fn dispatch_physics_scatter_velocity(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &PhysicsGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &PhysicsGpuState, contact_count: u32) {
    if contact_count == 0 {
        return;
    }
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(contacts_buffer) = buffers.contacts.as_ref() else { return };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.scatter_velocity_pipeline) else {
        return; // Pipeline still compiling -- same convention as dispatch_physics_predict.
    };
    let Some(uniform_binding) = buffers.scatter_uniform.binding() else { return };

    let bind_group = render_device.create_bind_group(
        "physics_scatter_velocity_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.scatter_position_layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.bodies.as_entire_buffer_binding(), contacts_buffer.as_entire_buffer_binding(), buffers.accumulators.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_scatter_velocity_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_scatter_velocity_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = contact_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Dispatches the velocity-round apply-average pass once, over
/// `body_count` bodies. Must run AFTER `dispatch_physics_scatter_velocity`
/// within the same substep — same caller-responsible sequencing
/// convention as every other dispatch function here.
///
/// Piece 4 only: called directly by its own parity test, not yet wired
/// into the real per-frame render path — see `gpu/mod.rs`'s own doc
/// comment on the `PhysicsGpuEnabled` opt-in decision.
pub fn dispatch_physics_apply_velocity(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &PhysicsGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &PhysicsGpuState) {
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.apply_velocity_pipeline) else {
        return; // Pipeline still compiling -- same convention as dispatch_physics_predict.
    };
    let Some(uniform_binding) = buffers.apply_velocity_uniform.binding() else { return };

    let bind_group = render_device.create_bind_group(
        "physics_apply_velocity_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.apply_velocity_layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.bodies.as_entire_buffer_binding(), buffers.accumulators.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_apply_velocity_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_apply_velocity_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = buffers.body_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Dispatches the sample-point generation pass once, over `shape_count`
/// shapes (same 1D-dispatch shape as every other pass here). Broad-phase/
/// contact-generation port, Piece 1 — not yet consumed by anything else in
/// this port (broad-phase and contact generation, its actual downstream
/// consumers, land in later pieces); called directly by its own parity
/// test for now.
pub fn dispatch_physics_sample_points(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &SamplePointsGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &SamplePointsGpuState) {
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
        return; // Pipeline still compiling -- same convention as dispatch_physics_predict.
    };
    let Some(uniform_binding) = buffers.uniform.binding() else { return };

    let bind_group = render_device.create_bind_group(
        "physics_sample_points_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.shapes.as_entire_buffer_binding(), buffers.outputs.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_sample_points_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_sample_points_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = buffers.shape_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// The parts of a scan pass dispatch that stay fixed across every
/// individual step call — bundled purely to keep `dispatch_one_scan_pass`
/// under clippy's argument-count lint, not because these fields have any
/// deeper relationship.
struct ScanPassPipeline<'a> {
    pipeline_id: bevy::render::render_resource::CachedComputePipelineId,
    layout: &'a bevy::render::render_resource::BindGroupLayoutDescriptor,
    pipeline_cache: &'a bevy::render::render_resource::PipelineCache,
}

/// The ping-pong buffer pair for one scan pass — `source` is read,
/// `dest` is written (see `dispatch_physics_scan`'s own ping-pong
/// bookkeeping for which of `buffer_a`/`buffer_b` is which for a given
/// pass).
struct ScanPassBuffers<'a> {
    source: &'a bevy::render::render_resource::Buffer,
    dest: &'a bevy::render::render_resource::Buffer,
}

/// Dispatches one scan pass (either a Hillis-Steele step or the final
/// exclusive-shift).
fn dispatch_one_scan_pass(render_device: &RenderDevice, render_queue: &RenderQueue, pass_pipeline: ScanPassPipeline, uniform_binding: bevy::render::render_resource::BindingResource, buffers: ScanPassBuffers, count: u32, label: &str) -> bool {
    let pipeline_cache = pass_pipeline.pipeline_cache;
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pass_pipeline.pipeline_id) else {
        return false; // Pipeline still compiling -- same convention as dispatch_physics_predict.
    };

    let bind_group = render_device.create_bind_group(label, &pipeline_cache.get_bind_group_layout(pass_pipeline.layout), &BindGroupEntries::sequential((uniform_binding, buffers.source.as_entire_buffer_binding(), buffers.dest.as_entire_buffer_binding())));

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some(label) });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some(label), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Runs the full Hillis-Steele exclusive scan: `log2(count)` step passes
/// (ping-ponging `buffer_a`/`buffer_b` — see `physics_scan.wgsl`'s own
/// doc comment for why in-place isn't safe), then one final
/// exclusive-shift pass. Returns which of `buffer_a`/`buffer_b` holds the
/// final result (`true` = `buffer_a`, `false` = `buffer_b`) — since the
/// number of ping-pong passes is data-dependent (`log2(count)`), the
/// result doesn't always land in the same buffer every call.
///
/// Piece 2 only: proven standalone here against a trivial CPU exclusive-
/// scan reference, not yet consumed by the broad-phase hash itself
/// (Piece 3's own job) — see `gpu/mod.rs`'s own doc comment on why each
/// piece of this port lands independently verified before becoming
/// load-bearing for the next.
pub fn dispatch_physics_scan(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &ScanGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &mut ScanGpuState) -> bool {
    let Some(count) = state.0.as_ref().map(|b| b.count) else { return false };
    if count == 0 {
        return false;
    }

    // Ceiling log2(count), matching Hillis-Steele's own requirement that
    // after `ceil(log2(n))` passes every element has accumulated
    // contributions from all `n` elements to its left -- a non-power-of-
    // two count (e.g. table_size = max(n*2, 256), not guaranteed to be a
    // clean power of two per the plan's own deliberate test emphasis)
    // still converges correctly, it just does marginally more passes than
    // the tightest possible bound for that exact count.
    let num_steps = if count <= 1 { 0 } else { u32::BITS - (count - 1).leading_zeros() };

    // true = buffer_a currently holds the latest data (starts true: the
    // caller's own ensure_scan_buffers uploads the raw input into
    // buffer_a).
    let mut a_is_current = true;

    for step in 0..num_steps {
        let offset = 1u32 << step;
        write_scan_uniform(render_device, render_queue, state, offset);
        let Some(buffers) = state.0.as_ref() else { return false };
        let (source, dest) = if a_is_current { (&buffers.buffer_a, &buffers.buffer_b) } else { (&buffers.buffer_b, &buffers.buffer_a) };
        let Some(uniform_binding) = buffers.uniform.binding() else { return false };
        let ok = dispatch_one_scan_pass(render_device, render_queue, ScanPassPipeline { pipeline_id: pipeline.step_pipeline, layout: &pipeline.layout, pipeline_cache }, uniform_binding, ScanPassBuffers { source, dest }, count, "physics_scan_step");
        if !ok {
            return false;
        }
        a_is_current = !a_is_current;
    }

    // Final exclusive-shift pass: same ping-pong bookkeeping, one more
    // pass reading the latest inclusive-scan result and writing the
    // exclusive-shifted form.
    write_scan_uniform(render_device, render_queue, state, 0);
    let Some(buffers) = state.0.as_ref() else { return false };
    let (source, dest) = if a_is_current { (&buffers.buffer_a, &buffers.buffer_b) } else { (&buffers.buffer_b, &buffers.buffer_a) };
    let Some(uniform_binding) = buffers.uniform.binding() else { return false };
    let ok = dispatch_one_scan_pass(render_device, render_queue, ScanPassPipeline { pipeline_id: pipeline.to_exclusive_pipeline, layout: &pipeline.layout, pipeline_cache }, uniform_binding, ScanPassBuffers { source, dest }, count, "physics_scan_to_exclusive");
    if !ok {
        return false;
    }
    a_is_current = !a_is_current;

    a_is_current
}

/// Dispatches the broad-phase hash pass once, over `body_count` bodies.
fn dispatch_broadphase_hash(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &BroadphaseHashPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &BroadphaseGpuState) -> bool {
    let Some(buffers) = state.0.as_ref() else { return false };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.hash_pipeline) else { return false };
    let Some(uniform_binding) = buffers.hash_count_uniform.binding() else { return false };

    let Some(bucket_counts) = buffers.scan.0.as_ref().map(|s| &s.buffer_a) else { return false };
    let bind_group = render_device.create_bind_group(
        "physics_broadphase_hash_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.positions.as_entire_buffer_binding(), buffers.hashes.as_entire_buffer_binding(), bucket_counts.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_broadphase_hash_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_broadphase_hash_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = buffers.body_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Dispatches the broad-phase count pass once, over `body_count` bodies —
/// must run AFTER `dispatch_broadphase_hash` (reads the hashes it wrote)
/// and after the bucket-counts buffer has been zeroed for this frame (see
/// `ensure_broadphase_buffers`'s own zero-reupload).
fn dispatch_broadphase_count(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &BroadphaseHashPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &BroadphaseGpuState) -> bool {
    let Some(buffers) = state.0.as_ref() else { return false };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.count_pipeline) else { return false };
    let Some(uniform_binding) = buffers.hash_count_uniform.binding() else { return false };

    let Some(bucket_counts) = buffers.scan.0.as_ref().map(|s| &s.buffer_a) else { return false };
    let bind_group = render_device.create_bind_group(
        "physics_broadphase_count_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.positions.as_entire_buffer_binding(), buffers.hashes.as_entire_buffer_binding(), bucket_counts.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_broadphase_count_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_broadphase_count_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = buffers.body_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Dispatches the broad-phase copy pass once (`bucket_start` -> `cursor`)
/// — must run AFTER the scan has produced `bucket_start` (whichever of
/// `scan.buffer_a`/`buffer_b` `scan_result_in_a` indicates).
fn dispatch_broadphase_copy(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &BroadphaseScatterPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &BroadphaseGpuState) -> bool {
    let Some(buffers) = state.0.as_ref() else { return false };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.copy_pipeline) else { return false };
    let Some(uniform_binding) = buffers.copy_uniform.binding() else { return false };
    let Some(scan_buffers) = buffers.scan.0.as_ref() else { return false };
    let bucket_start = if buffers.scan_result_in_a { &scan_buffers.buffer_a } else { &scan_buffers.buffer_b };

    let bind_group = render_device.create_bind_group(
        "physics_broadphase_copy_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.copy_layout),
        &BindGroupEntries::sequential((uniform_binding, bucket_start.as_entire_buffer_binding(), buffers.cursor.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_broadphase_copy_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_broadphase_copy_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = (buffers.table_size + 1).div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Dispatches the broad-phase scatter pass once, over `body_count` bodies
/// — must run AFTER `dispatch_broadphase_copy` (reads/mutates the cursor
/// it just initialized).
fn dispatch_broadphase_scatter(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &BroadphaseScatterPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &BroadphaseGpuState) -> bool {
    let Some(buffers) = state.0.as_ref() else { return false };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.scatter_pipeline) else { return false };
    let Some(uniform_binding) = buffers.scatter_uniform.binding() else { return false };

    let bind_group = render_device.create_bind_group(
        "physics_broadphase_scatter_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.scatter_layout),
        &BindGroupEntries::sequential((uniform_binding, buffers.hashes.as_entire_buffer_binding(), buffers.cursor.as_entire_buffer_binding(), buffers.bucket_items.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_broadphase_scatter_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_broadphase_scatter_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = buffers.body_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Dispatches the extract-positions pass once, over `body_count` bodies —
/// bridges the solver's live `PhysicsBodyGpu` buffer into the
/// `[f32;4]`-per-element layout `BroadphaseGpuBuffers::positions` expects,
/// so broad-phase can be rebuilt from each substep's CURRENT positions
/// with no CPU round-trip (see `assets/shaders/physics_extract_positions.wgsl`'s
/// own doc comment). Writes directly into `broadphase_state`'s own
/// `positions` buffer — `ensure_broadphase_buffers` must have already
/// allocated it this frame (this function does not allocate).
pub fn dispatch_physics_extract_positions(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &ExtractPositionsPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, physics_state: &PhysicsGpuState, broadphase_state: &mut BroadphaseGpuState) -> bool {
    let Some(physics_buffers) = physics_state.0.as_ref() else { return false };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else { return false };

    // Bound this dispatch by the DYNAMIC body count (`broadphase_buffers`'s
    // own `body_count`, set by `ensure_broadphase_buffers` from the
    // dynamics-only position slice), not the solver's full `PhysicsGpuBuffers::body_count`
    // (dynamics + kinematics + statics). `positions_out` (`broadphase_buffers.positions`)
    // is only ever allocated to hold `dynamic_count` elements -- writing
    // past that with the full body count was a real, previously-
    // undetected out-of-bounds GPU write, live since Piece 5 landed
    // whenever `static_count > 0` (every soak/parity scene has a static
    // floor/planet, so this was always in effect, just apparently not
    // corrupting anything visibly-checked until Stage 3.5 Piece 2's own
    // kinematic-platform soak test pushed `body_count` past whatever
    // threshold made the corruption visible: a resting box on a moving
    // kinematic platform launched to Y=52 from a start near Y=9.5 within
    // 50 frames, confirmed absent on the CPU path with the identical
    // scene, isolating the cause to this GPU-only dispatch).
    let Some(broadphase_buffers) = broadphase_state.0.as_ref() else { return false };
    let dynamic_count = broadphase_buffers.body_count;
    super::buffers::write_extract_positions_uniform(render_device, render_queue, broadphase_state, dynamic_count);
    let Some(broadphase_buffers) = broadphase_state.0.as_ref() else { return false };
    let Some(uniform_binding) = broadphase_buffers.extract_positions_uniform.binding() else { return false };

    let bind_group = render_device.create_bind_group(
        "physics_extract_positions_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((uniform_binding, physics_buffers.bodies.as_entire_buffer_binding(), broadphase_buffers.positions.as_entire_buffer_binding())),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_extract_positions_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_extract_positions_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = dynamic_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Copies the solver's own live `PhysicsGpuState::bodies` buffer into
/// `ContactGenGpuBuffers::bodies` — a GPU-to-GPU copy, no CPU round-trip.
/// Needed because contact generation must see each substep's CURRENT
/// body positions (the solver mutates `PhysicsGpuState::bodies` in place
/// every substep via predict/apply), but `ContactGenGpuState` owns a
/// SEPARATE buffer (uploaded once from the extracted CPU snapshot at the
/// start of the frame) rather than binding the solver's buffer directly —
/// keeping the two buffer sets independently owned avoids parameterizing
/// every contact-gen dispatch function over an externally-provided bodies
/// buffer (the same aliasing-hazard risk `dispatch_contacts_static`'s own
/// doc comment already flags for a DIFFERENT binding in this exact pass,
/// found and fixed once already in Piece 4 — safer to keep bind groups
/// pointing at buffers this module fully owns and re-sync between them
/// with a plain copy). Shapes/sample-points never need this treatment —
/// they don't change mid-frame, only bodies (positions/rotations/
/// velocities) do.
pub fn copy_live_bodies_into_contact_gen(render_device: &RenderDevice, render_queue: &RenderQueue, physics_state: &PhysicsGpuState, contact_gen_state: &ContactGenGpuState) {
    let Some(physics_buffers) = physics_state.0.as_ref() else { return };
    let Some(contact_gen_buffers) = contact_gen_state.0.as_ref() else { return };
    let byte_size = (physics_buffers.body_count as u64) * (std::mem::size_of::<super::types::PhysicsBodyGpu>() as u64);
    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_copy_live_bodies_encoder") });
    encoder.copy_buffer_to_buffer(&physics_buffers.bodies, 0, &contact_gen_buffers.bodies, 0, byte_size);
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Copies the contact-generation pass's own atomic cursor (the TRUE
/// contact count, discovered on GPU) directly into the scatter-round
/// uniform's `contact_count` field (`ScatterUniform`'s first field, at
/// byte offset 0 — same size, `u32`, as the cursor itself) — a 4-byte
/// GPU-to-GPU copy, no CPU round-trip, no blocking poll.
///
/// Exists specifically to eliminate a real driver-stability bug found
/// during this piece's own soak testing: reading the cursor back to the
/// CPU every substep (`create_buffer` + `map_async` +
/// `poll(PollType::wait_indefinitely())`, a full GPU pipeline drain) 8
/// times per frame, every frame, was frequent and heavy enough to trigger
/// a genuine AMD/radv driver context loss (`"The CS has been cancelled
/// because the context is lost"`) during an extended run — not a flaky
/// test, a real crash. This function replaces that per-substep CPU
/// readback entirely for the real per-frame path (`frame.rs`'s own
/// substep loop): the scatter/apply dispatch workgroup COUNT is sized
/// conservatively at `contact_capacity` (the known fixed upper bound, no
/// readback needed for that), and each shader's own `params.contact_count`
/// bounds check (already present, see `physics_scatter_position.wgsl`'s
/// own `if i >= params.contact_count { return; }`) correctly skips
/// invocations beyond the true count once this copy has landed.
///
/// Caller is still responsible for calling `write_scatter_uniform_for_count`
/// FIRST (to set `fixed_point_scale`/the clamp fields correctly) — this
/// function only overwrites the `contact_count` field afterward, relying
/// on wgpu's same-queue submission ordering (the prior `write_buffer` call
/// queues before this function's own `copy_buffer_to_buffer`, and queue
/// operations execute in submission order) to guarantee the overwrite
/// lands last. The one place in this whole pipeline that still reads the
/// TRUE contact count back to the CPU is `frame.rs`'s own end-of-frame
/// overflow check (`PHYSICS_GPU_CONTACT_OVERFLOW`) — a once-per-frame,
/// not once-per-substep, cost, and only for the warning path, not for
/// sizing any dispatch.
pub fn copy_contact_count_into_scatter_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, contact_gen_state: &ContactGenGpuState, physics_state: &PhysicsGpuState) {
    let Some(contact_gen_buffers) = contact_gen_state.0.as_ref() else { return };
    let Some(physics_buffers) = physics_state.0.as_ref() else { return };
    let Some(scatter_uniform_buffer) = physics_buffers.scatter_uniform.buffer() else { return };
    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_copy_contact_count_encoder") });
    encoder.copy_buffer_to_buffer(&contact_gen_buffers.cursor, 0, scatter_uniform_buffer, 0, std::mem::size_of::<u32>() as u64);
    render_queue.submit(std::iter::once(encoder.finish()));
}

/// Runs the FULL broad-phase pipeline: hash -> count -> scan (reusing
/// Piece 2's own `dispatch_physics_scan` unchanged) -> copy -> scatter --
/// producing a GPU-built CSR `bucket_start`/`bucket_items` pair, mirroring
/// `SpatialHash::build`'s own five-line body exactly in shape (count ->
/// prefix-sum -> scatter), just spread across GPU dispatches instead of a
/// CPU loop. Records which buffer holds `bucket_start` on
/// `state.0.scan_result_in_a` for later reads (the copy pass and any
/// future query pass need to know).
///
/// Piece 3 only: called directly by its own parity test, not yet wired
/// into the real per-frame render path — see `gpu/mod.rs`'s own doc
/// comment on the `PhysicsGpuEnabled` opt-in decision.
pub fn dispatch_physics_broadphase(render_device: &RenderDevice, render_queue: &RenderQueue, hash_pipeline: &BroadphaseHashPipeline, scatter_pipeline: &BroadphaseScatterPipeline, scan_pipeline: &ScanGpuPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &mut BroadphaseGpuState) -> bool {
    if !dispatch_broadphase_hash(render_device, render_queue, hash_pipeline, pipeline_cache, state) {
        return false;
    }
    if !dispatch_broadphase_count(render_device, render_queue, hash_pipeline, pipeline_cache, state) {
        return false;
    }

    let Some(buffers) = state.0.as_mut() else { return false };
    let scan_result_in_a = dispatch_physics_scan(render_device, render_queue, scan_pipeline, pipeline_cache, &mut buffers.scan);
    buffers.scan_result_in_a = scan_result_in_a;

    if !dispatch_broadphase_copy(render_device, render_queue, scatter_pipeline, pipeline_cache, state) {
        return false;
    }
    if !dispatch_broadphase_scatter(render_device, render_queue, scatter_pipeline, pipeline_cache, state) {
        return false;
    }

    true
}

/// Dispatches the dynamic-vs-dynamic contact-generation pass once, over
/// `dynamic_count` bodies — one invocation per dynamic body, each walking
/// its own 27-cell neighborhood via the broad-phase's own CSR
/// `bucket_start`/`bucket_items` (`broadphase_state`, produced by a prior
/// `dispatch_physics_broadphase` call this frame — this function does not
/// enforce that ordering itself, same caller-responsible sequencing
/// convention as every other dispatch function here). Must run AFTER
/// `ensure_contact_gen_buffers`/`write_contact_gen_uniform` have reset the
/// atomic cursor for this pass — see those functions' own doc comments.
fn dispatch_contacts_dynamic(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &ContactGenPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &ContactGenGpuState, broadphase_state: &BroadphaseGpuState) -> bool {
    let Some(buffers) = state.0.as_ref() else { return false };
    let Some(broadphase_buffers) = broadphase_state.0.as_ref() else { return false };
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.dynamic_pipeline) else { return false };
    let Some(uniform_binding) = buffers.uniform.binding() else { return false };
    let Some(scan_buffers) = broadphase_buffers.scan.0.as_ref() else { return false };
    let bucket_start = if broadphase_buffers.scan_result_in_a { &scan_buffers.buffer_a } else { &scan_buffers.buffer_b };

    let bind_group = render_device.create_bind_group(
        "physics_contacts_dynamic_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            uniform_binding,
            buffers.bodies.as_entire_buffer_binding(),
            buffers.shapes.as_entire_buffer_binding(),
            buffers.sample_points.as_entire_buffer_binding(),
            bucket_start.as_entire_buffer_binding(),
            broadphase_buffers.bucket_items.as_entire_buffer_binding(),
            buffers.cursor.as_entire_buffer_binding(),
            buffers.contacts_out.as_entire_buffer_binding(),
        )),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_contacts_dynamic_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_contacts_dynamic_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let wg_x = buffers.dynamic_count.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Dispatches the dynamic-vs-kinematic contact-generation pass once, over
/// the flattened `dynamic_count * kinematic_count` pair grid — no broad-
/// phase dependency, mirrors `dispatch_contacts_static` exactly (same
/// bind-group-aliasing-avoidance reasoning, same shared layout/bind group
/// shape) just against the kinematic sub-range instead of the static one.
/// No-ops if there are zero kinematic bodies (nothing to generate).
fn dispatch_contacts_kinematic(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &ContactGenPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &ContactGenGpuState) -> bool {
    let Some(buffers) = state.0.as_ref() else { return false };
    if buffers.kinematic_count == 0 {
        return true;
    }
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.kinematic_pipeline) else { return false };
    let Some(uniform_binding) = buffers.uniform.binding() else { return false };

    // Same bind-group-aliasing-avoidance reasoning as dispatch_contacts_static
    // below: bodies/shapes stand in for the two unused bucket_start/
    // bucket_items placeholders since this pass, like the static one,
    // never touches the broad-phase's own CSR output.
    let bind_group = render_device.create_bind_group(
        "physics_contacts_kinematic_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            uniform_binding,
            buffers.bodies.as_entire_buffer_binding(),
            buffers.shapes.as_entire_buffer_binding(),
            buffers.sample_points.as_entire_buffer_binding(),
            buffers.bodies.as_entire_buffer_binding(),
            buffers.shapes.as_entire_buffer_binding(),
            buffers.cursor.as_entire_buffer_binding(),
            buffers.contacts_out.as_entire_buffer_binding(),
        )),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_contacts_kinematic_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_contacts_kinematic_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let total_pairs = buffers.dynamic_count * buffers.kinematic_count;
        let wg_x = total_pairs.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Dispatches the dynamic-vs-static contact-generation pass once, over
/// the flattened `dynamic_count * static_count` pair grid — no broad-phase
/// dependency (mirrors `generate_all_contacts`'s own direct-loop design
/// for statics, see `physics_contacts.wgsl`'s own header comment). No-ops
/// if there are zero static bodies (nothing to generate).
fn dispatch_contacts_static(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &ContactGenPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &ContactGenGpuState) -> bool {
    let Some(buffers) = state.0.as_ref() else { return false };
    if buffers.static_count == 0 {
        return true;
    }
    let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.static_pipeline) else { return false };
    let Some(uniform_binding) = buffers.uniform.binding() else { return false };

    // Same bind group shape as the dynamic pass, but bucket_start/
    // bucket_items are unused by physics_contacts_static_main's own
    // logic -- still bound, since the bind group layout is shared between
    // both pipelines and wgpu requires every declared binding to be
    // filled. Uses `bodies`/`shapes` as the two placeholders rather than
    // reusing `cursor` for both (an earlier version did that and silently
    // produced zero contacts every time -- confirmed via an unconditional
    // debug write to contacts_out[0] that never landed on GPU despite the
    // dispatch running with the correct workgroup count and uniform
    // values: binding the SAME buffer object to a read-only slot AND
    // the read_write `contact_cursor` slot within one bind group is an
    // aliasing hazard wgpu doesn't validate against at bind-group-creation
    // time but produces silently-dropped writes at execution time on this
    // adapter). `bodies`/`shapes` are large enough and never aliased with
    // `cursor`/`contacts_out`, avoiding the hazard entirely.
    let bind_group = render_device.create_bind_group(
        "physics_contacts_static_bind_group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            uniform_binding,
            buffers.bodies.as_entire_buffer_binding(),
            buffers.shapes.as_entire_buffer_binding(),
            buffers.sample_points.as_entire_buffer_binding(),
            buffers.bodies.as_entire_buffer_binding(),
            buffers.shapes.as_entire_buffer_binding(),
            buffers.cursor.as_entire_buffer_binding(),
            buffers.contacts_out.as_entire_buffer_binding(),
        )),
    );

    let mut encoder = render_device.create_command_encoder(&bevy::render::render_resource::CommandEncoderDescriptor { label: Some("physics_contacts_static_encoder") });
    {
        let mut cpass = encoder.begin_compute_pass(&ComputePassDescriptor { label: Some("physics_contacts_static_pass"), timestamp_writes: None });
        cpass.set_pipeline(compute_pipeline);
        cpass.set_bind_group(0, &bind_group, &[]);
        let total_pairs = buffers.dynamic_count * buffers.static_count;
        let wg_x = total_pairs.div_ceil(64);
        cpass.dispatch_workgroups(wg_x, 1, 1);
    }
    render_queue.submit(std::iter::once(encoder.finish()));
    true
}

/// Runs all three contact-generation passes (dynamic-vs-dynamic, dynamic-
/// vs-kinematic, then dynamic-vs-static) against the SAME atomic cursor/
/// output buffer, so contacts from all three passes land in one combined
/// list — matching `generate_all_contacts`'s own single `Vec<Contact>`
/// output exactly, just produced by three GPU dispatches instead of three
/// CPU loops appending to the same `Vec`.
pub fn dispatch_physics_contacts(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &ContactGenPipeline, pipeline_cache: &bevy::render::render_resource::PipelineCache, state: &ContactGenGpuState, broadphase_state: &BroadphaseGpuState) -> bool {
    if !dispatch_contacts_dynamic(render_device, render_queue, pipeline, pipeline_cache, state, broadphase_state) {
        return false;
    }
    if !dispatch_contacts_kinematic(render_device, render_queue, pipeline, pipeline_cache, state) {
        return false;
    }
    if !dispatch_contacts_static(render_device, render_queue, pipeline, pipeline_cache, state) {
        return false;
    }
    true
}
