//! The real per-frame GPU physics orchestration — Piece 5's own dispatch
//! shape, reproducing `solve_world`'s algorithm (predict, regenerate
//! contacts, position-round scatter/apply, velocity-round scatter/apply,
//! per substep) through render-world dispatches instead of a CPU loop.
//! Every individual dispatch function this orchestrates already exists
//! and is independently parity-tested (see `pass.rs`) — this file's own
//! job is sequencing and the render-world resources needed to do that,
//! not new solver/collision math.
//!
//! A plain `Render`-schedule system (not `Core3d`/`ViewQuery`-scoped),
//! same reasoning as every other dispatch function in this port (see
//! `pass.rs`'s own header comment) — physics has no camera/view
//! dependency.

use bevy::prelude::*;
use bevy::render::render_resource::PipelineCache;
use bevy::render::renderer::{RenderDevice, RenderQueue};

use super::super::solve_rigid::{MAX_ANGULAR_CORRECTION, MAX_LINEAR_CORRECTION, MAX_VELOCITY_IMPULSE};
use super::super::solve_world::SUBSTEPS;
use super::buffers::{
    BroadphaseGpuState, ContactGenGpuState, ContactGenInputs, PhysicsGpuState, clear_accumulators, ensure_broadphase_buffers, ensure_contact_gen_buffers, ensure_physics_buffers, reset_broadphase_bucket_counts, reset_contact_gen_cursor,
    write_apply_uniform, write_apply_velocity_uniform, write_broadphase_copy_uniform, write_broadphase_scatter_uniform, write_contact_gen_uniform, write_kinematic_bodies, write_predict_uniform, write_scatter_uniform_for_count,
};
use super::extract::RenderPhysicsGpuFrame;
use super::pass::{
    copy_contact_count_into_scatter_uniform, copy_live_bodies_into_contact_gen, dispatch_physics_apply_position, dispatch_physics_apply_velocity, dispatch_physics_broadphase, dispatch_physics_contacts, dispatch_physics_extract_positions,
    dispatch_physics_predict, dispatch_physics_scatter_position_from_buffer, dispatch_physics_scatter_velocity_from_buffer, read_buffer_sync,
};
use super::pipelines::{BroadphaseHashPipeline, BroadphaseScatterPipeline, ContactGenPipeline, ExtractPositionsPipeline, PhysicsGpuPipeline, SamplePointsGpuPipeline, ScanGpuPipeline};
use super::readback::{PhysicsGpuReadbackState, request_physics_readback};
use super::sample_cache::{SamplePointsCache, ensure_sample_points_cached};
use super::types::SamplePointsGpu;

/// Per-frame headroom multiplier for the contact-generation output
/// buffer's fixed capacity — the plan's own recommended heuristic,
/// adopted here for the first live call site (Piece 4's own tests only
/// ever hardcoded small capacities for their fixed test scenes).
const CONTACT_CAPACITY_MULTIPLIER: u32 = 8;

/// `Render`-schedule system: runs the full GPU physics frame — predict,
/// then `SUBSTEPS` rounds of (extract positions -> broad-phase -> contact
/// generation -> position scatter/apply -> velocity scatter/apply) — then
/// requests the non-blocking readback for next frame's apply. No-ops
/// entirely if `RenderPhysicsGpuFrame` is empty (either `PhysicsGpuEnabled`
/// is off, or `dt <= 0.0` this frame — see `extract_physics_bodies`'s own
/// early-return shape).
#[allow(clippy::too_many_arguments)]
pub fn dispatch_physics_gpu_frame(
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    pipeline_cache: Res<PipelineCache>,
    frame: Res<RenderPhysicsGpuFrame>,
    physics_pipeline: Res<PhysicsGpuPipeline>,
    sample_points_pipeline: Res<SamplePointsGpuPipeline>,
    broadphase_hash_pipeline: Res<BroadphaseHashPipeline>,
    broadphase_scatter_pipeline: Res<BroadphaseScatterPipeline>,
    scan_pipeline: Res<ScanGpuPipeline>,
    contact_gen_pipeline: Res<ContactGenPipeline>,
    extract_positions_pipeline: Res<ExtractPositionsPipeline>,
    mut physics_state: ResMut<PhysicsGpuState>,
    mut sample_points_cache: ResMut<SamplePointsCache>,
    mut broadphase_state: ResMut<BroadphaseGpuState>,
    mut contact_gen_state: ResMut<ContactGenGpuState>,
    mut readback_state: ResMut<PhysicsGpuReadbackState>,
) {
    let body_count = frame.bodies.len() as u32;
    if body_count == 0 {
        return;
    }

    // Every pipeline this frame's dispatch chain can possibly need must be
    // ready BEFORE this function does ANY work -- checked here, once, up
    // front, rather than letting each individual dispatch_* call bail out
    // on its own mid-chain (as they already each do defensively). Without
    // this, `dispatch_physics_predict` (no dependency on the broad-phase/
    // contact-generation pipelines) could run for a partial handful of
    // substeps before a later `dispatch_physics_broadphase`/
    // `dispatch_physics_contacts` call bailed via its own `return false`,
    // leaving those substeps' gravity integration applied with zero
    // contact correction. Skipping the ENTIRE frame (not just the failing
    // dispatch) when any pipeline isn't ready yet means everything is
    // deferred together -- bodies simply sit exactly where they are for
    // one more frame, the same "nothing runs yet" behavior every other
    // not-yet-ready dispatch function already has individually, just
    // enforced consistently for the whole chain.
    let all_pipelines_ready = pipeline_cache.get_compute_pipeline(physics_pipeline.predict_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(physics_pipeline.scatter_position_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(physics_pipeline.apply_position_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(physics_pipeline.scatter_velocity_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(physics_pipeline.apply_velocity_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(sample_points_pipeline.pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(scan_pipeline.step_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(scan_pipeline.to_exclusive_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(broadphase_hash_pipeline.hash_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(broadphase_hash_pipeline.count_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(broadphase_scatter_pipeline.copy_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(broadphase_scatter_pipeline.scatter_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(contact_gen_pipeline.dynamic_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(contact_gen_pipeline.kinematic_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(contact_gen_pipeline.static_pipeline).is_some()
        && pipeline_cache.get_compute_pipeline(extract_positions_pipeline.pipeline).is_some();
    if !all_pipelines_ready {
        return;
    }

    // Once the GPU's own body buffer already holds this many bodies, it is
    // the AUTHORITATIVE state -- it must NOT be re-uploaded from this
    // frame's CPU-extracted snapshot every call. `extract_physics_bodies`'s
    // own snapshot reflects whatever `Transform`/`RigidBody` the LAST
    // readback wrote, which is itself several frames stale (the whole
    // point of the non-blocking readback's latency) -- re-uploading it
    // every frame would silently throw away everything the GPU computed
    // across the substep loop since that readback landed and restart from
    // stale data, forever. A real bug found via this exact symptom: a
    // persistent period-2 position oscillation during a real windowed run
    // (never caught by the headless Tier 1/2 tests, which only run a
    // handful of bodies for a fixed frame count and happened not to
    // exhibit a visible steady-state oscillation in their own tolerance
    // window) -- confirmed by logging applied positions frame-by-frame and
    // seeing a body's X coordinate alternate between two fixed values
    // indefinitely instead of converging, with Z still monotonically
    // increasing (still genuinely falling) at the same time. Only
    // re-upload when the buffer doesn't exist yet or the body COUNT
    // changed (a body was added/removed) -- exactly the same "size-
    // defining input change" trigger `ensure_physics_buffers` itself
    // already uses to decide whether to reallocate, just also gating the
    // upload on it here rather than uploading unconditionally every call
    // (existing parity tests still call `ensure_physics_buffers` directly
    // and rely on its own unconditional-upload contract for a fixed test
    // scene -- this gating lives here in the real per-frame path only, not
    // in that shared function).
    let needs_upload = match physics_state.0.as_ref() {
        Some(buffers) => buffers.body_count != body_count,
        None => true,
    };
    if needs_upload {
        ensure_physics_buffers(&render_device, &render_queue, &mut physics_state, &frame.bodies);
    }

    // Kinematics are the mirror-image case of dynamics: they must be
    // re-uploaded EVERY frame the GPU path runs (an external animation/
    // script system drives their Transform/RigidBody every frame, unlike
    // dynamics which are GPU-authoritative once running) via a partial
    // write into just their own sub-range of the shared bodies buffer —
    // see `write_kinematic_bodies`'s own doc comment for the full
    // reasoning. Runs unconditionally, after the `needs_upload` gate
    // above and before predict, so a fresh full upload (if one just
    // happened) isn't immediately stale for the kinematic range.
    if frame.kinematic_count > 0 {
        let kinematic_start = frame.dynamic_count as usize;
        let kinematic_end = kinematic_start + frame.kinematic_count as usize;
        write_kinematic_bodies(&render_queue, &physics_state, frame.dynamic_count, &frame.bodies[kinematic_start..kinematic_end]);
    }

    write_predict_uniform(
        &render_device,
        &render_queue,
        &mut physics_state,
        super::types::PhysicsPredictUniform { gravity_center_x: frame.gravity_center.x, gravity_center_y: frame.gravity_center.y, gravity_center_z: frame.gravity_center.z, gravity_magnitude: frame.gravity_magnitude, substep_dt: frame.substep_dt, body_count, _pad0: 0, _pad1: 0 },
    );

    // Sample points are shape-keyed and pose-independent (see
    // sample_cache.rs's own doc comment) -- resolved once per frame here,
    // not per substep, since a body's shape never changes mid-frame.
    if !ensure_sample_points_cached(&render_device, &render_queue, &sample_points_pipeline, &pipeline_cache, &mut sample_points_cache, &frame.shapes) {
        return; // Pipeline still compiling -- same convention as every other dispatch function.
    }
    let sample_points: Vec<SamplePointsGpu> = frame.shapes.iter().map(|shape| sample_points_cache.get(shape).unwrap_or_else(bytemuck::Zeroable::zeroed)).collect();

    let contact_capacity = body_count * CONTACT_CAPACITY_MULTIPLIER;
    ensure_contact_gen_buffers(&render_device, &render_queue, &mut contact_gen_state, ContactGenInputs { bodies: &frame.bodies, shapes: &frame.shapes, sample_points: &sample_points }, frame.dynamic_count, frame.kinematic_count, contact_capacity);

    // Broad-phase buffers are allocated once per frame from the frame's
    // own dynamic-body positions (substep 0's positions match exactly
    // what was extracted) -- `ensure_broadphase_buffers` unconditionally
    // overwrites `positions` from its CPU slice argument every call, so
    // calling it again mid-substep-loop would stomp the GPU-computed
    // positions `dispatch_physics_extract_positions` writes each substep.
    // Substeps 1.. refresh positions via that GPU-side pass instead (see
    // its own doc comment for why a raw buffer-to-buffer copy can't
    // bridge the two layouts).
    let dynamic_positions: Vec<[f32; 4]> = frame.bodies[..frame.dynamic_count as usize].iter().map(|b| [b.position_x, b.position_y, b.position_z, 0.0]).collect();
    if frame.dynamic_count > 0 {
        ensure_broadphase_buffers(&render_device, &render_queue, &mut broadphase_state, &dynamic_positions, frame.broadphase_cell_size);
        write_broadphase_copy_uniform(&render_device, &render_queue, &mut broadphase_state);
        write_broadphase_scatter_uniform(&render_device, &render_queue, &mut broadphase_state);
    }

    for substep in 0..SUBSTEPS {
        if substep > 0 && frame.dynamic_count > 0 {
            // Refresh broad-phase positions from this substep's own
            // (already predict/apply-advanced) live body buffer.
            if !dispatch_physics_extract_positions(&render_device, &render_queue, &extract_positions_pipeline, &pipeline_cache, &physics_state, &mut broadphase_state) {
                return;
            }
        }

        dispatch_physics_predict(&render_device, &render_queue, &physics_pipeline, &pipeline_cache, &physics_state);

        if frame.dynamic_count > 0 {
            // Broad-phase's own bucket-counts buffer must be zeroed before
            // EVERY substep's count pass, not just once per frame --
            // ensure_broadphase_buffers (which also zeroes it) is only
            // called once per frame, before this loop, since calling it
            // again mid-loop would stomp the GPU-computed positions
            // dispatch_physics_extract_positions writes each substep. A
            // real bug found via this piece's own soak testing: without
            // this reset, atomicAdd's from every substep's count pass
            // accumulated on top of every prior substep's counts within
            // the same frame, corrupting bucket_start's CSR ranges and
            // producing exponentially growing contact-generation cost
            // substep over substep (confirmed via per-phase timing: a
            // single frame's contact-gen cost grew from under a
            // millisecond to over 10 seconds by substep 5 before this fix).
            reset_broadphase_bucket_counts(&render_queue, &broadphase_state);
            if !dispatch_physics_broadphase(&render_device, &render_queue, &broadphase_hash_pipeline, &broadphase_scatter_pipeline, &scan_pipeline, &pipeline_cache, &mut broadphase_state) {
                return;
            }
        }

        copy_live_bodies_into_contact_gen(&render_device, &render_queue, &physics_state, &contact_gen_state);
        // Same reasoning as reset_broadphase_bucket_counts above: contacts
        // must be regenerated fresh every substep, but ensure_contact_gen_buffers
        // (which also resets this cursor) only runs once per frame -- a
        // second real bug this piece's soak testing caught, compounding
        // with the broad-phase one above.
        reset_contact_gen_cursor(&render_queue, &contact_gen_state);
        write_contact_gen_uniform(&render_device, &render_queue, &mut contact_gen_state, frame.broadphase_cell_size, broadphase_state.0.as_ref().map(|b| b.table_size).unwrap_or(256));
        if !dispatch_physics_contacts(&render_device, &render_queue, &contact_gen_pipeline, &pipeline_cache, &contact_gen_state, &broadphase_state) {
            return;
        }

        let Some(contact_gen_buffers) = contact_gen_state.0.as_ref() else { return };
        let contacts_buffer = contact_gen_buffers.contacts_out.clone();

        // Position round. Dispatch workgroups are sized at the fixed
        // `contact_capacity` upper bound, not the true (GPU-discovered)
        // contact count -- no CPU readback needed to size them. Each
        // shader's own `params.contact_count` bounds check (fed via the
        // GPU-to-GPU copy below) correctly skips invocations beyond the
        // true count. See `copy_contact_count_into_scatter_uniform`'s own
        // doc comment for why this replaced a per-substep blocking CPU
        // readback that was causing real GPU driver instability.
        clear_accumulators(&render_queue, &physics_state);
        write_scatter_uniform_for_count(&render_device, &render_queue, &mut physics_state, contact_capacity, MAX_LINEAR_CORRECTION, MAX_ANGULAR_CORRECTION);
        copy_contact_count_into_scatter_uniform(&render_device, &render_queue, &contact_gen_state, &physics_state);
        dispatch_physics_scatter_position_from_buffer(&render_device, &render_queue, &physics_pipeline, &pipeline_cache, &physics_state, &contacts_buffer, contact_capacity);
        write_apply_uniform(&render_device, &render_queue, &mut physics_state, frame.substep_dt);
        dispatch_physics_apply_position(&render_device, &render_queue, &physics_pipeline, &pipeline_cache, &physics_state);

        // Velocity round.
        clear_accumulators(&render_queue, &physics_state);
        write_scatter_uniform_for_count(&render_device, &render_queue, &mut physics_state, contact_capacity, MAX_VELOCITY_IMPULSE, MAX_VELOCITY_IMPULSE);
        copy_contact_count_into_scatter_uniform(&render_device, &render_queue, &contact_gen_state, &physics_state);
        dispatch_physics_scatter_velocity_from_buffer(&render_device, &render_queue, &physics_pipeline, &pipeline_cache, &physics_state, &contacts_buffer, contact_capacity);
        write_apply_velocity_uniform(&render_device, &render_queue, &mut physics_state);
        dispatch_physics_apply_velocity(&render_device, &render_queue, &physics_pipeline, &pipeline_cache, &physics_state);
    }

    // The one place in the whole frame that still reads the true contact
    // count back to the CPU -- once per FRAME, not once per substep, and
    // only for the loud overflow warning, never to size a dispatch (see
    // copy_contact_count_into_scatter_uniform's own doc comment).
    if let Some(contact_gen_buffers) = contact_gen_state.0.as_ref() {
        let cursor: Vec<u32> = read_buffer_sync(&render_device, &render_queue, &contact_gen_buffers.cursor, 1);
        let true_contact_count = cursor[0];
        if true_contact_count > contact_capacity {
            eprintln!("PHYSICS_GPU_CONTACT_OVERFLOW: {true_contact_count} contacts generated in the final substep, capacity was {contact_capacity} -- some contacts were silently clamped this frame");
        }
    }

    request_physics_readback(&render_device, &render_queue, &physics_state, frame.dynamic_entities.clone(), &mut readback_state);
}
