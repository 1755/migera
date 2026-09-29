//! `PhysicsGpuState` — the persistent, `read_write`, manually-allocated
//! body-state buffers, following `hybrid::pipeline::HybridDdgiAtlas`'s own
//! allocation pattern (`create_buffer` + zero-init via `write_buffer`,
//! reallocated only when a size-defining input — here, body count —
//! changes) rather than `ObjectGpu`'s `RawBufferVec` pattern, which is
//! fully cleared and re-pushed from ECS every frame and would silently
//! stomp any GPU-side physics write a frame later. See `gpu/mod.rs`'s own
//! doc comment for the full rationale.

use bevy::prelude::*;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::render_resource::{Buffer, BufferDescriptor, BufferUsages, UniformBuffer};

use super::types::{ApplyUniform, ApplyVelocityUniform, BroadphaseScatterUniform, ContactGenUniform, ContactGpu, HashCountUniform, PhysicsAccumulatorGpu, PhysicsBodyGpu, PhysicsPredictUniform, PhysicsShapeGpu, SamplePointsGpu, SamplePointsUniform, ScanUniform, ScatterCopyUniform, ScatterUniform, SubstepStartGpu};

/// Render-world resource holding the two persistent physics body buffers
/// plus the small per-dispatch predict uniform. `None` until the first
/// body upload — mirrors `HybridDdgiAtlasRes`'s own `Option<T>`-wrapped-
/// resource convention for "not yet allocated."
#[derive(Resource, Default)]
pub struct PhysicsGpuState(pub Option<PhysicsGpuBuffers>);

pub struct PhysicsGpuBuffers {
    pub body_count: u32,
    pub bodies: Buffer,
    pub substep_start: Buffer,
    pub predict_uniform: UniformBuffer<PhysicsPredictUniform>,
    pub accumulators: Buffer,
    pub scatter_uniform: UniformBuffer<ScatterUniform>,
    pub apply_uniform: UniformBuffer<ApplyUniform>,
    pub apply_velocity_uniform: UniformBuffer<ApplyVelocityUniform>,
    /// Contact upload buffer — NOT persistent-and-mutated like the other
    /// fields here (`ObjectGpu`'s `RawBufferVec` pattern is the right
    /// template for this one instead of `HybridDdgiAtlas`'s: contacts are
    /// fully regenerated CPU-side every substep, see `contacts_capacity`'s
    /// own doc comment for the reallocation policy this uses to avoid
    /// reallocating on every single substep's contact-count fluctuation).
    pub contacts: Option<Buffer>,
    pub contacts_capacity: u32,
}

impl PhysicsGpuBuffers {
    fn byte_size(body_count: u32) -> (u64, u64, u64) {
        let bodies = (body_count as u64) * (std::mem::size_of::<PhysicsBodyGpu>() as u64);
        let substep_start = (body_count as u64) * (std::mem::size_of::<SubstepStartGpu>() as u64);
        let accumulators = (body_count as u64) * (std::mem::size_of::<PhysicsAccumulatorGpu>() as u64);
        (bodies, substep_start, accumulators)
    }
}

/// Allocates (or reallocates, on a body-count change) the persistent
/// physics buffers, then uploads `initial_bodies` into the `bodies`
/// buffer. Mirrors `prepare_hybrid_ddgi`'s own `needs_new` gating exactly:
/// only body COUNT forces a reallocation, matching this codebase's
/// established "reallocate on size-defining input change, not every
/// frame" convention.
///
/// A body count of `0` is treated the same as "nothing to allocate" —
/// `create_buffer` with `size: 0` is valid in wgpu but pointless here, and
/// callers (Piece 1's test harness, later the real per-frame path) should
/// simply not call this when there are no dynamic bodies, exactly like
/// `solve_world`'s own `if states.is_empty() { return; }` early exit.
pub fn ensure_physics_buffers(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut PhysicsGpuState, initial_bodies: &[PhysicsBodyGpu]) {
    let body_count = initial_bodies.len() as u32;
    if body_count == 0 {
        return;
    }

    let needs_new = match state.0.as_ref() {
        Some(buffers) => buffers.body_count != body_count,
        None => true,
    };

    if needs_new {
        let (bodies_byte_size, substep_start_byte_size, accumulators_byte_size) = PhysicsGpuBuffers::byte_size(body_count);

        let bodies = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_bodies"),
            size: bodies_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&bodies, 0, &vec![0u8; bodies_byte_size as usize]);

        let substep_start = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_substep_start"),
            size: substep_start_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&substep_start, 0, &vec![0u8; substep_start_byte_size as usize]);

        let accumulators = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_accumulators"),
            size: accumulators_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&accumulators, 0, &vec![0u8; accumulators_byte_size as usize]);

        state.0 = Some(PhysicsGpuBuffers {
            body_count,
            bodies,
            substep_start,
            predict_uniform: UniformBuffer::default(),
            accumulators,
            scatter_uniform: UniformBuffer::default(),
            apply_uniform: UniformBuffer::default(),
            apply_velocity_uniform: UniformBuffer::default(),
            contacts: None,
            contacts_capacity: 0,
        });
    }

    let Some(buffers) = state.0.as_ref() else { return };
    render_queue.write_buffer(&buffers.bodies, 0, bytemuck::cast_slice(initial_bodies));
}

/// Re-uploads ONLY the kinematic sub-range of the persistent `bodies`
/// buffer (`dynamic_count..dynamic_count+kinematic_count`, per
/// `solve_world`'s own three-range body-index convention) — the mirror-
/// image case of `ensure_physics_buffers`'s own dynamics/statics upload
/// policy. Dynamics are GPU-authoritative once running (never re-uploaded
/// here, see `frame.rs`'s own `needs_upload` gating and its doc comment on
/// the real bug that gating fixed) and statics never move after their
/// first upload, but kinematics are driven externally every frame the
/// GPU path runs (an animation/script system writes their `Transform`/
/// `RigidBody` on the main-world side, `extract_physics_bodies` snapshots
/// that every extract) — so this function is called unconditionally every
/// frame, after `ensure_physics_buffers`'s own reallocation-only-on-count-
/// change gate, using wgpu's `Queue::write_buffer`'s arbitrary-offset
/// partial-write support (already used at offset 0 everywhere else in
/// this port) to touch only the kinematic bytes, leaving the dynamics and
/// statics ranges completely untouched. A logically separate kinematics
/// buffer/binding was considered and rejected (see the plan's own Stage
/// 3.5 GPU-port section): every existing WGSL shader already indexes
/// `bodies[i]` by one flat index space, and a second binding would force
/// an `if (i < dynamic_count) { ... } else { ... }` branch at every access
/// site across every shader in this port for zero benefit over this same-
/// cost partial write into the existing buffer.
pub fn write_kinematic_bodies(render_queue: &RenderQueue, state: &PhysicsGpuState, dynamic_count: u32, kinematic_bodies: &[PhysicsBodyGpu]) {
    if kinematic_bodies.is_empty() {
        return;
    }
    let Some(buffers) = state.0.as_ref() else { return };
    let offset = (dynamic_count as u64) * (std::mem::size_of::<PhysicsBodyGpu>() as u64);
    render_queue.write_buffer(&buffers.bodies, offset, bytemuck::cast_slice(kinematic_bodies));
}

/// Uploads this substep's gravity/timestep parameters into the predict
/// uniform buffer — called once per dispatch, same convention as
/// `prepare_hybrid_ddgi`'s own per-frame `grid_uniform.set(...);
/// grid_uniform.write_buffer(...)` sequence.
pub fn write_predict_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut PhysicsGpuState, params: PhysicsPredictUniform) {
    let Some(buffers) = state.0.as_mut() else { return };
    buffers.predict_uniform.set(params);
    buffers.predict_uniform.write_buffer(render_device, render_queue);
}

/// Zeroes the accumulator buffer — called before each scatter round (once
/// for the position round, once for the velocity round in a later piece).
/// Done via a direct `write_buffer` of zeroed bytes rather than a
/// dedicated clear compute shader, per the plan's own stated v1 choice:
/// simplest to implement, revisit only if profiling shows this write
/// itself is a bottleneck.
pub fn clear_accumulators(render_queue: &RenderQueue, state: &PhysicsGpuState) {
    let Some(buffers) = state.0.as_ref() else { return };
    let byte_size = (buffers.body_count as usize) * std::mem::size_of::<PhysicsAccumulatorGpu>();
    render_queue.write_buffer(&buffers.accumulators, 0, &vec![0u8; byte_size]);
}

/// Uploads this substep's contact list, following `ObjectGpu`'s own
/// `RawBufferVec`-style reallocation policy rather than
/// `HybridDdgiAtlas`'s (see `PhysicsGpuBuffers::contacts`'s own doc
/// comment for why): only grows the buffer when the incoming contact
/// count exceeds the current capacity, so a typical substep-to-substep
/// contact-count fluctuation doesn't reallocate 8 times a frame — the
/// same amortized-growth idea `RawBufferVec` itself uses, made explicit
/// here since contact count is also read back CPU-side to size the
/// scatter dispatch (`ScatterUniform::contact_count`).
/// `max_linear_or_velocity_correction`/`max_angular_correction` select
/// which round's per-contact clamp applies (`MAX_LINEAR_CORRECTION`/
/// `MAX_ANGULAR_CORRECTION` for the position round,
/// `MAX_VELOCITY_IMPULSE` for both fields in the velocity round — see
/// `ScatterUniform`'s own doc comment for why this must be clamped BEFORE
/// scattering, not just after the apply pass's per-body average).
pub fn upload_contacts(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut PhysicsGpuState, contacts: &[ContactGpu], max_linear_or_velocity_correction: f32, max_angular_correction: f32) {
    let Some(buffers) = state.0.as_mut() else { return };
    let contact_count = contacts.len() as u32;
    if contact_count == 0 {
        buffers.scatter_uniform.set(ScatterUniform { contact_count: 0, fixed_point_scale: super::types::FIXED_POINT_SCALE, max_linear_or_velocity_correction, max_angular_correction });
        buffers.scatter_uniform.write_buffer(render_device, render_queue);
        return;
    }

    if buffers.contacts.is_none() || contact_count > buffers.contacts_capacity {
        let byte_size = (contact_count as u64) * (std::mem::size_of::<ContactGpu>() as u64);
        buffers.contacts = Some(render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_contacts"),
            size: byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }));
        buffers.contacts_capacity = contact_count;
    }

    let Some(contacts_buffer) = buffers.contacts.as_ref() else { return };
    render_queue.write_buffer(contacts_buffer, 0, bytemuck::cast_slice(contacts));

    buffers.scatter_uniform.set(ScatterUniform { contact_count, fixed_point_scale: super::types::FIXED_POINT_SCALE, max_linear_or_velocity_correction, max_angular_correction });
    buffers.scatter_uniform.write_buffer(render_device, render_queue);
}

/// Sets/writes `ScatterUniform` for a contact count that's ALREADY known
/// (the real per-frame GPU path's own contact-gen dispatch produces its
/// contacts entirely on GPU, in `ContactGenGpuBuffers::contacts_out` —
/// there's no CPU-side `Vec<ContactGpu>` to derive a length from, unlike
/// `upload_contacts`, which also allocates/uploads a CPU-sourced contacts
/// buffer this function deliberately does not touch). Paired with
/// `dispatch_physics_scatter_position_from_buffer`/
/// `dispatch_physics_scatter_velocity_from_buffer`, which read contacts
/// from that externally-provided buffer directly.
pub fn write_scatter_uniform_for_count(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut PhysicsGpuState, contact_count: u32, max_linear_or_velocity_correction: f32, max_angular_correction: f32) {
    let Some(buffers) = state.0.as_mut() else { return };
    buffers.scatter_uniform.set(ScatterUniform { contact_count, fixed_point_scale: super::types::FIXED_POINT_SCALE, max_linear_or_velocity_correction, max_angular_correction });
    buffers.scatter_uniform.write_buffer(render_device, render_queue);
}

/// Uploads this substep's apply-pass parameters, reading every clamp/
/// damping constant directly from `solve_rigid`'s own `pub(crate)`
/// constants — the single source of truth both backends must agree on
/// (see `ApplyUniform`'s own doc comment for why this matters: a
/// hand-copied second set of magic numbers in WGSL could silently drift
/// out of sync with the CPU reference if either side changed without the
/// other).
pub fn write_apply_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut PhysicsGpuState, substep_dt: f32) {
    let Some(buffers) = state.0.as_mut() else { return };
    buffers.apply_uniform.set(ApplyUniform {
        body_count: buffers.body_count,
        substep_dt,
        fixed_point_scale: super::types::FIXED_POINT_SCALE,
        max_linear_correction: super::super::solve_rigid::MAX_LINEAR_CORRECTION,
        max_angular_correction: super::super::solve_rigid::MAX_ANGULAR_CORRECTION,
        linear_damping: super::super::solve_rigid::LINEAR_DAMPING,
        angular_damping: super::super::solve_rigid::ANGULAR_DAMPING,
        max_angular_velocity: super::super::solve_rigid::MAX_ANGULAR_VELOCITY,
        max_linear_velocity: super::super::solve_rigid::MAX_LINEAR_VELOCITY,
        _pad0: 0,
        _pad1: 0,
        _pad2: 0,
    });
    buffers.apply_uniform.write_buffer(render_device, render_queue);
}

/// Uploads this substep's velocity-round apply-pass parameters —
/// `MAX_VELOCITY_IMPULSE` only, read from `solve_rigid`'s own
/// `pub(crate)` constant, same single-source-of-truth rationale as
/// `write_apply_uniform`.
pub fn write_apply_velocity_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut PhysicsGpuState) {
    let Some(buffers) = state.0.as_mut() else { return };
    buffers.apply_velocity_uniform.set(ApplyVelocityUniform {
        body_count: buffers.body_count,
        fixed_point_scale: super::types::FIXED_POINT_SCALE,
        max_velocity_impulse: super::super::solve_rigid::MAX_VELOCITY_IMPULSE,
        _pad0: 0,
    });
    buffers.apply_velocity_uniform.write_buffer(render_device, render_queue);
}

/// Render-world resource holding the sample-point generation pass's own
/// buffers — deliberately SEPARATE from `PhysicsGpuState`: sample-point
/// generation is keyed by SHAPE count (one dispatch per distinct shape a
/// body uses), not body count, and is a standalone concern from the
/// substep solver's own predict/scatter/apply buffers (broad-phase and
/// contact generation, still to come in later pieces of this port, are
/// the actual consumers of this pass's output — the substep solver
/// consumes CONTACTS, never sample points directly). `None` until the
/// first shape upload, same `Option`-wrapped-resource convention as
/// `PhysicsGpuState`.
#[derive(Resource, Default)]
pub struct SamplePointsGpuState(pub Option<SamplePointsGpuBuffers>);

pub struct SamplePointsGpuBuffers {
    pub shape_count: u32,
    pub shapes: Buffer,
    pub outputs: Buffer,
    pub uniform: UniformBuffer<SamplePointsUniform>,
}

/// Allocates (or reallocates, on a shape-count change) the sample-point
/// generation buffers, then uploads `shapes` — same `HybridDdgiAtlas`-style
/// allocation pattern as `ensure_physics_buffers`, reallocating only on a
/// size-defining input (shape count) change.
pub fn ensure_sample_points_buffers(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut SamplePointsGpuState, shapes: &[PhysicsShapeGpu]) {
    let shape_count = shapes.len() as u32;
    if shape_count == 0 {
        return;
    }

    let needs_new = match state.0.as_ref() {
        Some(buffers) => buffers.shape_count != shape_count,
        None => true,
    };

    if needs_new {
        let shapes_byte_size = (shape_count as u64) * (std::mem::size_of::<PhysicsShapeGpu>() as u64);
        let outputs_byte_size = (shape_count as u64) * (std::mem::size_of::<SamplePointsGpu>() as u64);

        let shapes_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_sample_points_shapes"),
            size: shapes_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&shapes_buffer, 0, &vec![0u8; shapes_byte_size as usize]);

        let outputs_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_sample_points_outputs"),
            size: outputs_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&outputs_buffer, 0, &vec![0u8; outputs_byte_size as usize]);

        state.0 = Some(SamplePointsGpuBuffers { shape_count, shapes: shapes_buffer, outputs: outputs_buffer, uniform: UniformBuffer::default() });
    }

    let Some(buffers) = state.0.as_mut() else { return };
    render_queue.write_buffer(&buffers.shapes, 0, bytemuck::cast_slice(shapes));
    buffers.uniform.set(SamplePointsUniform { shape_count, _pad0: 0, _pad1: 0, _pad2: 0 });
    buffers.uniform.write_buffer(render_device, render_queue);
}

/// Render-world resource holding the Hillis-Steele scan pass's own
/// ping-pong buffer pair — deliberately SEPARATE from every other buffer
/// resource here: the scan is a standalone, reusable utility (proven here
/// in isolation per the plan's own Piece 2 milestone, consumed by the
/// broad-phase's own bucket-offset computation in Piece 3), not owned by
/// any one specific caller. `None` until the first upload, same
/// `Option`-wrapped-resource convention as every other GPU state resource
/// here.
#[derive(Resource, Default)]
pub struct ScanGpuState(pub Option<ScanGpuBuffers>);

pub struct ScanGpuBuffers {
    pub count: u32,
    /// Two buffers, ping-ponged across scan passes (each pass reads one,
    /// writes the other — see `physics_scan.wgsl`'s own doc comment for
    /// why this can't be a single in-place buffer: every invocation reads
    /// a neighbor another invocation in the SAME dispatch may also be
    /// writing).
    pub buffer_a: Buffer,
    pub buffer_b: Buffer,
    pub uniform: UniformBuffer<ScanUniform>,
}

/// Allocates (or reallocates, on a count change) the scan's ping-pong
/// buffers and uploads `input` into `buffer_a` — the same
/// `HybridDdgiAtlas`-style allocation pattern as every other buffer here.
pub fn ensure_scan_buffers(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut ScanGpuState, input: &[u32]) {
    let count = input.len() as u32;
    if count == 0 {
        return;
    }

    let needs_new = match state.0.as_ref() {
        Some(buffers) => buffers.count != count,
        None => true,
    };

    if needs_new {
        let byte_size = (count as u64) * (std::mem::size_of::<u32>() as u64);

        let buffer_a = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_scan_buffer_a"),
            size: byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&buffer_a, 0, &vec![0u8; byte_size as usize]);

        let buffer_b = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_scan_buffer_b"),
            size: byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&buffer_b, 0, &vec![0u8; byte_size as usize]);

        state.0 = Some(ScanGpuBuffers { count, buffer_a, buffer_b, uniform: UniformBuffer::default() });
    }

    let Some(buffers) = state.0.as_ref() else { return };
    render_queue.write_buffer(&buffers.buffer_a, 0, bytemuck::cast_slice(input));
}

/// Uploads this dispatch's scan-pass parameters (`count`, and `offset =
/// 2^d` for scan step `d`; pass `offset = 0` for the exclusive-shift
/// pass, which ignores it).
pub fn write_scan_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut ScanGpuState, offset: u32) {
    let Some(buffers) = state.0.as_mut() else { return };
    buffers.uniform.set(ScanUniform { count: buffers.count, offset, _pad0: 0, _pad1: 0 });
    buffers.uniform.write_buffer(render_device, render_queue);
}

/// Render-world resource holding the full broad-phase GPU pipeline's own
/// buffers — positions (input), hashes (intermediate), the scan's own
/// `bucket_counts`/`bucket_start` pair (owned via a nested `ScanGpuState`,
/// reusing Piece 2's proven scan machinery unchanged), the scatter
/// cursor, and `bucket_items` (the final CSR output, mirroring
/// `SpatialHash`'s own `bucket_start`/`bucket_items` field pair). `None`
/// until the first upload, same `Option`-wrapped-resource convention as
/// every other GPU state resource here.
#[derive(Resource, Default)]
pub struct BroadphaseGpuState(pub Option<BroadphaseGpuBuffers>);

pub struct BroadphaseGpuBuffers {
    pub body_count: u32,
    pub table_size: u32,
    pub cell_size: f32,
    pub positions: Buffer,
    pub hashes: Buffer,
    /// `bucket_counts` (pre-scan) and `bucket_start` (post-scan) are the
    /// SAME underlying storage — `scan.0`'s own `buffer_a`/`buffer_b`
    /// ping-pong pair, sized `table_size + 1` (matching `SpatialHash`'s
    /// own `bucket_start` length). Whichever buffer holds the final
    /// scanned result (`bucket_start`) is tracked by `scan_result_in_a`
    /// after each `dispatch_physics_scan` call.
    pub scan: ScanGpuState,
    pub scan_result_in_a: bool,
    pub cursor: Buffer,
    pub bucket_items: Buffer,
    pub hash_count_uniform: UniformBuffer<HashCountUniform>,
    pub copy_uniform: UniformBuffer<ScatterCopyUniform>,
    pub scatter_uniform: UniformBuffer<BroadphaseScatterUniform>,
    /// The extract-positions pass's own uniform (Piece 5's tiny bridge
    /// pass — see `assets/shaders/physics_extract_positions.wgsl`'s own
    /// doc comment). Lives here (not a fresh `UniformBuffer` allocated per
    /// dispatch call) because a per-call `UniformBuffer::from(...)`
    /// allocates a brand-new backing GPU buffer every time — harmless
    /// once, but this pass runs up to `SUBSTEPS - 1` times per frame,
    /// every frame, and a real soak test caught the consequence directly:
    /// thousands of orphaned buffer allocations over a few hundred frames
    /// eventually triggered a genuine GPU driver context loss
    /// (`radv/amdgpu: The CS has been cancelled because the context is
    /// lost`), not just a performance smell. Every other persistent
    /// per-dispatch uniform in this port already lives in its owning
    /// state struct for exactly this reason — this one was the one
    /// exception, found and fixed via that failure.
    pub extract_positions_uniform: UniformBuffer<super::types::ExtractPositionsUniform>,
}

/// Allocates (or reallocates, on a body-count change) the broad-phase's
/// own buffers, then uploads `positions` — same `HybridDdgiAtlas`-style
/// allocation pattern as every other buffer here. `table_size` follows
/// `SpatialHash::build`'s own formula exactly (`max(body_count * 2, 256)`)
/// — see that function's own doc comment for why the floor matters at
/// small body counts.
pub fn ensure_broadphase_buffers(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut BroadphaseGpuState, positions: &[[f32; 4]], cell_size: f32) {
    let body_count = positions.len() as u32;
    if body_count == 0 {
        return;
    }
    let table_size = (body_count * 2).max(256);

    let needs_new = match state.0.as_ref() {
        Some(buffers) => buffers.body_count != body_count || buffers.table_size != table_size,
        None => true,
    };

    if needs_new {
        let positions_byte_size = (body_count as u64) * (std::mem::size_of::<[f32; 4]>() as u64);
        let hashes_byte_size = (body_count as u64) * (std::mem::size_of::<u32>() as u64);
        let bucket_byte_size = ((table_size as u64) + 1) * (std::mem::size_of::<u32>() as u64);
        let bucket_items_byte_size = (body_count as u64) * (std::mem::size_of::<u32>() as u64);

        let positions_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_broadphase_positions"),
            size: positions_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&positions_buffer, 0, &vec![0u8; positions_byte_size as usize]);

        let hashes_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_broadphase_hashes"),
            size: hashes_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&hashes_buffer, 0, &vec![0u8; hashes_byte_size as usize]);

        let cursor_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_broadphase_cursor"),
            size: bucket_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&cursor_buffer, 0, &vec![0u8; bucket_byte_size as usize]);

        let bucket_items_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_broadphase_bucket_items"),
            size: bucket_items_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&bucket_items_buffer, 0, &vec![0u8; bucket_items_byte_size as usize]);

        state.0 = Some(BroadphaseGpuBuffers {
            body_count,
            table_size,
            cell_size,
            positions: positions_buffer,
            hashes: hashes_buffer,
            scan: ScanGpuState::default(),
            scan_result_in_a: true,
            cursor: cursor_buffer,
            bucket_items: bucket_items_buffer,
            hash_count_uniform: UniformBuffer::default(),
            copy_uniform: UniformBuffer::default(),
            scatter_uniform: UniformBuffer::default(),
            extract_positions_uniform: UniformBuffer::default(),
        });
    }

    let Some(buffers) = state.0.as_mut() else { return };
    buffers.cell_size = cell_size;
    render_queue.write_buffer(&buffers.positions, 0, bytemuck::cast_slice(positions));

    // Zero the bucket-counts buffer (scan's own buffer_a) before this
    // frame's count pass atomicAdd's into it -- ensure_scan_buffers
    // itself only zeroes on first allocation, not every call, so an
    // explicit re-upload of zeros here matches the solver port's own
    // clear_accumulators precedent (the accumulator buffer needs the
    // same "zero before every scatter round" treatment for the same
    // reason: atomicAdd accumulates onto whatever was already there).
    let zero_counts = vec![0u32; table_size as usize + 1];
    ensure_scan_buffers(render_device, render_queue, &mut buffers.scan, &zero_counts);

    buffers.hash_count_uniform.set(HashCountUniform { body_count, table_size, cell_size, _pad0: 0 });
    buffers.hash_count_uniform.write_buffer(render_device, render_queue);
}

/// Uploads this dispatch's extract-positions-pass parameters
/// (`body_count`) — see `BroadphaseGpuBuffers::extract_positions_uniform`'s
/// own doc comment for why this lives here as a persistent field rather
/// than a fresh `UniformBuffer` allocated per call.
pub fn write_extract_positions_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut BroadphaseGpuState, body_count: u32) {
    let Some(buffers) = state.0.as_mut() else { return };
    buffers.extract_positions_uniform.set(super::types::ExtractPositionsUniform { body_count, _pad0: 0, _pad1: 0, _pad2: 0 });
    buffers.extract_positions_uniform.write_buffer(render_device, render_queue);
}

/// Zeroes the broad-phase's own bucket-counts buffer (the scan's
/// `buffer_a`) WITHOUT reallocating or touching `positions` — for the
/// real per-frame GPU path, where broad-phase must be rebuilt every
/// SUBSTEP (bodies move during the solve) but `ensure_broadphase_buffers`
/// itself is only called once per frame (calling it again mid-substep-
/// loop would stomp the GPU-computed positions
/// `dispatch_physics_extract_positions` writes each substep — see that
/// function's own doc comment). **Found via a real bug this piece's own
/// soak testing caught**: without this reset, `atomicAdd`s from every
/// substep's own count pass accumulate on TOP of every prior substep's
/// counts within the same frame, growing `bucket_start`'s CSR ranges
/// unboundedly — confirmed as the root cause of `dispatch_physics_contacts`
/// itself measuring exponentially growing per-substep cost (0.6ms -> 49ms
/// -> 3s -> 10s across six consecutive substeps in one frame): each
/// substep's dynamic-vs-dynamic pass walks larger and larger corrupted
/// bucket ranges, producing far more spurious/duplicate candidate pairs
/// every substep, each one still paying full sample-point-based contact
/// generation cost.
pub fn reset_broadphase_bucket_counts(render_queue: &RenderQueue, state: &BroadphaseGpuState) {
    let Some(buffers) = state.0.as_ref() else { return };
    let Some(scan_buffers) = buffers.scan.0.as_ref() else { return };
    let byte_size = (scan_buffers.count as u64) * (std::mem::size_of::<u32>() as u64);
    render_queue.write_buffer(&scan_buffers.buffer_a, 0, &vec![0u8; byte_size as usize]);
}

/// Uploads this dispatch's copy-pass parameters (`count = table_size +
/// 1`, matching `bucket_start`'s own length).
pub fn write_broadphase_copy_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut BroadphaseGpuState) {
    let Some(buffers) = state.0.as_mut() else { return };
    let count = buffers.table_size + 1;
    buffers.copy_uniform.set(ScatterCopyUniform { count, _pad0: 0, _pad1: 0, _pad2: 0 });
    buffers.copy_uniform.write_buffer(render_device, render_queue);
}

/// Uploads this dispatch's scatter-pass parameters (`body_count`).
pub fn write_broadphase_scatter_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut BroadphaseGpuState) {
    let Some(buffers) = state.0.as_mut() else { return };
    let body_count = buffers.body_count;
    buffers.scatter_uniform.set(BroadphaseScatterUniform { body_count, _pad0: 0, _pad1: 0, _pad2: 0 });
    buffers.scatter_uniform.write_buffer(render_device, render_queue);
}

/// Render-world resource holding the contact-generation pass's own
/// buffers: bodies and shapes (both owned here rather than reused from
/// `PhysicsGpuState`/`SamplePointsGpuBuffers` — those are keyed by
/// solver-body-count and shape-upload-count respectively, which happen to
/// coincide with this pass's own body indexing in this port, but keeping
/// a separate, explicitly-owned buffer here avoids a hidden coupling
/// between three different pieces' allocation lifecycles), the fixed-
/// capacity contact OUTPUT buffer (unlike every earlier persistent buffer
/// in this port, this one's real element count is discovered at runtime
/// via the atomic cursor, not known ahead of dispatch — see
/// `physics_contacts.wgsl`'s own header comment), and a single-element
/// atomic cursor buffer. `None` until the first upload, same
/// `Option`-wrapped-resource convention as every other GPU state resource
/// here.
#[derive(Resource, Default)]
pub struct ContactGenGpuState(pub Option<ContactGenGpuBuffers>);

pub struct ContactGenGpuBuffers {
    pub dynamic_count: u32,
    pub kinematic_count: u32,
    pub static_count: u32,
    pub contact_capacity: u32,
    pub bodies: Buffer,
    pub shapes: Buffer,
    pub sample_points: Buffer,
    pub cursor: Buffer,
    pub contacts_out: Buffer,
    pub uniform: UniformBuffer<ContactGenUniform>,
}

/// The per-body input slices `ensure_contact_gen_buffers` uploads —
/// bundled purely to keep that function under clippy's argument-count
/// lint (same rationale as `ScanPassPipeline`/`ScanPassBuffers` in
/// `pass.rs`), not because these three slices have any deeper
/// relationship beyond all being indexed by the same body index.
pub struct ContactGenInputs<'a> {
    pub bodies: &'a [PhysicsBodyGpu],
    pub shapes: &'a [PhysicsShapeGpu],
    pub sample_points: &'a [SamplePointsGpu],
}

/// Allocates (or reallocates, on a body-count or capacity change) the
/// contact-generation pass's own buffers, then uploads `inputs.bodies`/
/// `inputs.shapes`/`inputs.sample_points` (all indexed identically:
/// `0..dynamic_count` are dynamic bodies,
/// `dynamic_count..dynamic_count+kinematic_count` are kinematics,
/// `dynamic_count+kinematic_count..` are statics — matching
/// `solve_world::generate_all_contacts`'s own three-range indexing
/// convention exactly) and resets the atomic cursor to zero (every call is
/// a fresh contact-generation pass, never an accumulation across calls,
/// same "clear before scatter" precedent `clear_accumulators` already
/// established for the solver port). `contact_capacity` sizes the
/// fixed-capacity output buffer — chosen by the caller with real headroom
/// (the plan's own recommendation: `body_count * 8`), not a tight bound
/// likely to be hit in practice.
pub fn ensure_contact_gen_buffers(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut ContactGenGpuState, inputs: ContactGenInputs, dynamic_count: u32, kinematic_count: u32, contact_capacity: u32) {
    let ContactGenInputs { bodies, shapes, sample_points } = inputs;
    let body_count = bodies.len() as u32;
    if body_count == 0 {
        return;
    }
    let static_count = body_count - dynamic_count - kinematic_count;

    let needs_new = match state.0.as_ref() {
        Some(buffers) => buffers.dynamic_count + buffers.kinematic_count + buffers.static_count != body_count || buffers.contact_capacity != contact_capacity,
        None => true,
    };

    if needs_new {
        let bodies_byte_size = (body_count as u64) * (std::mem::size_of::<PhysicsBodyGpu>() as u64);
        let shapes_byte_size = (body_count as u64) * (std::mem::size_of::<PhysicsShapeGpu>() as u64);
        let sample_points_byte_size = (body_count as u64) * (std::mem::size_of::<SamplePointsGpu>() as u64);
        let cursor_byte_size = std::mem::size_of::<u32>() as u64;
        let contacts_out_byte_size = (contact_capacity as u64) * (std::mem::size_of::<ContactGpu>() as u64);

        let bodies_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_contact_gen_bodies"),
            size: bodies_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&bodies_buffer, 0, &vec![0u8; bodies_byte_size as usize]);

        let shapes_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_contact_gen_shapes"),
            size: shapes_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&shapes_buffer, 0, &vec![0u8; shapes_byte_size as usize]);

        let sample_points_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_contact_gen_sample_points"),
            size: sample_points_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&sample_points_buffer, 0, &vec![0u8; sample_points_byte_size as usize]);

        let cursor_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_contact_gen_cursor"),
            size: cursor_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&cursor_buffer, 0, &vec![0u8; cursor_byte_size as usize]);

        let contacts_out_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("physics_contact_gen_contacts_out"),
            size: contacts_out_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&contacts_out_buffer, 0, &vec![0u8; contacts_out_byte_size as usize]);

        state.0 = Some(ContactGenGpuBuffers {
            dynamic_count,
            kinematic_count,
            static_count,
            contact_capacity,
            bodies: bodies_buffer,
            shapes: shapes_buffer,
            sample_points: sample_points_buffer,
            cursor: cursor_buffer,
            contacts_out: contacts_out_buffer,
            uniform: UniformBuffer::default(),
        });
    }

    let Some(buffers) = state.0.as_mut() else { return };
    buffers.dynamic_count = dynamic_count;
    buffers.kinematic_count = kinematic_count;
    buffers.static_count = static_count;
    render_queue.write_buffer(&buffers.bodies, 0, bytemuck::cast_slice(bodies));
    render_queue.write_buffer(&buffers.shapes, 0, bytemuck::cast_slice(shapes));
    render_queue.write_buffer(&buffers.sample_points, 0, bytemuck::cast_slice(sample_points));
    // Reset the atomic cursor to zero before every contact-generation
    // dispatch -- each call is a fresh pass over the current frame's
    // bodies, never an accumulation across calls.
    render_queue.write_buffer(&buffers.cursor, 0, &0u32.to_le_bytes());
}

/// Resets the contact-generation atomic cursor to zero WITHOUT
/// re-uploading bodies/shapes/sample-points — for the real per-frame path,
/// where contacts are regenerated every SUBSTEP (bodies move during the
/// solve, per `solve_world`'s own doc comment) but the body/shape/sample-
/// point identity doesn't change mid-frame, so re-running the full
/// `ensure_contact_gen_buffers` (which re-uploads all three every call) at
/// every substep would be pure waste. **Found via a real bug this piece's
/// own soak testing caught**: `frame.rs`'s substep loop originally called
/// only the full `ensure_contact_gen_buffers` once, before the loop, and
/// never reset the cursor again — every substep's `dispatch_physics_contacts`
/// then `atomicAdd`ed MORE contacts on top of whatever the cursor already
/// held from every PRIOR substep this same frame, with no bound. The
/// cursor value itself climbing unboundedly wasn't the visible symptom
/// (dispatch workgroup counts are fixed at `contact_capacity`, not sized
/// from the cursor); the actual measured symptom was `dispatch_physics_contacts`
/// itself taking seconds by substep 4-5 of a single frame (confirmed via
/// per-phase timing: 1ms -> 124ms -> 5s -> 10s across consecutive substeps)
/// — each substep's dynamic-vs-dynamic pass re-walks the SAME already-
/// enormous contact list's worth of atomic contention on one ever-growing
/// cursor value, and (separately) the static pass's own fixed
/// `dynamic_count * static_count` dispatch does real work regardless, but
/// an ever-larger already-written contacts_out range plausibly degrades
/// cache/memory behavior on the GPU as capacity is exceeded many times
/// over. Whatever the precise mechanism, resetting the cursor every
/// substep (matching the CPU reference's own "contacts start empty every
/// substep" semantics exactly) is unambiguously the correct fix
/// independent of the exact slowdown mechanism.
pub fn reset_contact_gen_cursor(render_queue: &RenderQueue, state: &ContactGenGpuState) {
    let Some(buffers) = state.0.as_ref() else { return };
    render_queue.write_buffer(&buffers.cursor, 0, &0u32.to_le_bytes());
}

/// Uploads this dispatch's contact-generation parameters (`dynamic_count`,
/// `static_count`, `contact_capacity`, and the broad-phase's own
/// `cell_size`/`table_size` — see `ContactGenUniform`'s own doc comment
/// for why the dynamic-vs-dynamic pass needs the latter two).
pub fn write_contact_gen_uniform(render_device: &RenderDevice, render_queue: &RenderQueue, state: &mut ContactGenGpuState, cell_size: f32, table_size: u32) {
    let Some(buffers) = state.0.as_mut() else { return };
    buffers.uniform.set(ContactGenUniform {
        dynamic_count: buffers.dynamic_count,
        static_count: buffers.static_count,
        contact_capacity: buffers.contact_capacity,
        cell_size,
        table_size,
        kinematic_count: buffers.kinematic_count,
        _pad1: 0,
        _pad2: 0,
    });
    buffers.uniform.write_buffer(render_device, render_queue);
}
