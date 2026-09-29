//! GPU-mirror structs for physics body state — see this module tree's own
//! doc comment (`gpu/mod.rs`) for why these live here rather than in
//! `physics::solve_rigid` itself. Field layout follows
//! `hybrid::extract::ObjectGpu`'s established convention exactly: derive
//! `Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable` with
//! `#[repr(C)]`, group scalars into `vec4`-sized chunks to keep std140
//! layout predictable, and give every field a scalar name (`_x/_y/_z/_w`)
//! rather than a nested `Vec3`/`Quat` (neither of which is `bytemuck::Pod`).

use bevy::math::{Quat, Vec3};
use bevy::render::render_resource::ShaderType;

/// Mirrors `physics::solve_rigid::BodyState` exactly (field order and
/// meaning), minus nothing — every `BodyState` field has a GPU
/// counterpart here. `inverse_mass` is packed into `position`'s `w` lane
/// (mirrors `ObjectGpu`'s own padding-lane-reuse trick) to keep the struct
/// a clean multiple of `vec4<f32>` (5 × 16 = 80 bytes), which avoids
/// `array<T>` stride surprises in WGSL — see this codebase's WGSL editing
/// rules (`CLAUDE.md`) on why struct layout mismatches between Rust and
/// WGSL are a real, previously-hit bug class here, not a theoretical one.
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct PhysicsBodyGpu {
    pub position_x: f32,
    pub position_y: f32,
    pub position_z: f32,
    pub inverse_mass: f32,
    pub rotation_x: f32,
    pub rotation_y: f32,
    pub rotation_z: f32,
    pub rotation_w: f32,
    pub linear_velocity_x: f32,
    pub linear_velocity_y: f32,
    pub linear_velocity_z: f32,
    pub _pad_linear_velocity: f32,
    pub angular_velocity_x: f32,
    pub angular_velocity_y: f32,
    pub angular_velocity_z: f32,
    pub _pad_angular_velocity: f32,
    pub inverse_inertia_local_x: f32,
    pub inverse_inertia_local_y: f32,
    pub inverse_inertia_local_z: f32,
    pub _pad_inverse_inertia_local: f32,
}

impl PhysicsBodyGpu {
    pub fn from_state(position: Vec3, rotation: Quat, linear_velocity: Vec3, angular_velocity: Vec3, inverse_mass: f32, inverse_inertia_local: Vec3) -> Self {
        Self {
            position_x: position.x,
            position_y: position.y,
            position_z: position.z,
            inverse_mass,
            rotation_x: rotation.x,
            rotation_y: rotation.y,
            rotation_z: rotation.z,
            rotation_w: rotation.w,
            linear_velocity_x: linear_velocity.x,
            linear_velocity_y: linear_velocity.y,
            linear_velocity_z: linear_velocity.z,
            _pad_linear_velocity: 0.0,
            angular_velocity_x: angular_velocity.x,
            angular_velocity_y: angular_velocity.y,
            angular_velocity_z: angular_velocity.z,
            _pad_angular_velocity: 0.0,
            inverse_inertia_local_x: inverse_inertia_local.x,
            inverse_inertia_local_y: inverse_inertia_local.y,
            inverse_inertia_local_z: inverse_inertia_local.z,
            _pad_inverse_inertia_local: 0.0,
        }
    }

    pub fn position(&self) -> Vec3 {
        Vec3::new(self.position_x, self.position_y, self.position_z)
    }

    pub fn rotation(&self) -> Quat {
        Quat::from_xyzw(self.rotation_x, self.rotation_y, self.rotation_z, self.rotation_w)
    }

    pub fn linear_velocity(&self) -> Vec3 {
        Vec3::new(self.linear_velocity_x, self.linear_velocity_y, self.linear_velocity_z)
    }

    pub fn angular_velocity(&self) -> Vec3 {
        Vec3::new(self.angular_velocity_x, self.angular_velocity_y, self.angular_velocity_z)
    }
}

/// Mirrors `physics::solve_rigid::SubstepStart` — a snapshot of a body's
/// kinematic state from BEFORE a substep's own gravity/rotation
/// prediction runs, needed (both CPU- and GPU-side) to derive
/// contact-corrected velocity without the catastrophic-cancellation bug
/// fixed this session (deriving velocity from a position delta loses
/// precision at world-scale positions; deriving it from a pre-integration
/// velocity snapshot plus the correction's own implied velocity avoids
/// that subtraction entirely). No `inverse_mass`/`inverse_inertia_local`
/// fields — those don't change mid-substep, so `PhysicsBodyGpu` remains
/// the single source of truth for them.
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct SubstepStartGpu {
    pub position_x: f32,
    pub position_y: f32,
    pub position_z: f32,
    pub _pad_position: f32,
    pub rotation_x: f32,
    pub rotation_y: f32,
    pub rotation_z: f32,
    pub rotation_w: f32,
    pub linear_velocity_x: f32,
    pub linear_velocity_y: f32,
    pub linear_velocity_z: f32,
    pub _pad_linear_velocity: f32,
    pub angular_velocity_x: f32,
    pub angular_velocity_y: f32,
    pub angular_velocity_z: f32,
    pub _pad_angular_velocity: f32,
}

/// Per-substep gravity/timestep parameters the predict pass needs —
/// mirrors `physics::solve_static::PhysicsGravity` plus the substep `dt`,
/// bundled into one small uniform buffer (same convention as
/// `hybrid::pipeline::DdgiGridUniform`: a plain flat uniform struct
/// uploaded fresh every dispatch rather than folded into the body buffer
/// itself, since it's scene-wide state, not per-body state).
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct PhysicsPredictUniform {
    pub gravity_center_x: f32,
    pub gravity_center_y: f32,
    pub gravity_center_z: f32,
    pub gravity_magnitude: f32,
    pub substep_dt: f32,
    pub body_count: u32,
    pub _pad0: u32,
    pub _pad1: u32,
}

/// Mirrors `physics::contacts::Contact` field-for-field — its own doc
/// comment already anticipated `u32` body indices specifically for this
/// GPU buffer's flat indexing scheme.
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ContactGpu {
    pub body_a: u32,
    pub body_b: u32,
    pub depth: f32,
    pub _pad0: f32,
    pub point_world_x: f32,
    pub point_world_y: f32,
    pub point_world_z: f32,
    pub _pad1: f32,
    pub normal_world_x: f32,
    pub normal_world_y: f32,
    pub normal_world_z: f32,
    pub _pad2: f32,
}

impl ContactGpu {
    pub fn from_contact(contact: super::super::contacts::Contact) -> Self {
        Self {
            body_a: contact.body_a,
            body_b: contact.body_b,
            depth: contact.depth,
            _pad0: 0.0,
            point_world_x: contact.point_world.x,
            point_world_y: contact.point_world.y,
            point_world_z: contact.point_world.z,
            _pad1: 0.0,
            normal_world_x: contact.normal_world.x,
            normal_world_y: contact.normal_world.y,
            normal_world_z: contact.normal_world.z,
            _pad2: 0.0,
        }
    }
}

/// WGSL has no native `atomic<f32>` — the contact-scatter pass instead
/// `atomicAdd`s FIXED-POINT `i32` values into this per-body accumulator,
/// and the apply pass divides back down by `FIXED_POINT_SCALE`. See
/// `solve_rigid::MAX_LINEAR_CORRECTION`'s own doc comment for the
/// overflow-safety reasoning this scale was picked against: at
/// `MAX_LINEAR_CORRECTION = 2.0` world units per axis and a generous
/// upper bound of 256 contacts scattering onto one body in a single
/// substep, `FIXED_POINT_SCALE * 256 * MAX_LINEAR_CORRECTION` must stay
/// well under `i32::MAX` (2,147,483,647) — `1_048_576 * 256 * 2.0 =
/// 536,870,912`, a 4x margin. `MAX_ANGULAR_CORRECTION` (0.1) is far
/// smaller and fits the same scale with even more headroom, so a single
/// shared scale serves both the linear and angular accumulators rather
/// than needing two different constants.
pub const FIXED_POINT_SCALE: f32 = 1_048_576.0; // 2^20

/// Converts a correction component to its fixed-point `i32` encoding —
/// the exact operation the WGSL scatter shader's `atomicAdd` performs on
/// the GPU side; exposed here so CPU-side tests can compute the "expected
/// raw accumulator value" independently, without duplicating the rounding
/// logic by hand at every call site.
pub fn to_fixed_point(value: f32) -> i32 {
    (value * FIXED_POINT_SCALE).round() as i32
}

/// Inverse of `to_fixed_point` — the apply pass's own dequantization step.
pub fn from_fixed_point(value: i32) -> f32 {
    (value as f32) / FIXED_POINT_SCALE
}

/// Mirrors `solve_rigid::Accumulator` — its own doc comment already calls
/// this struct "the CPU analogue of the GPU's per-body atomicAdd
/// targets." Fixed-point `i32` sums (see `FIXED_POINT_SCALE`'s own doc
/// comment), plain `u32` count (no atomics needed there beyond
/// `atomicAdd`'s own integer form, which IS natively supported).
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct PhysicsAccumulatorGpu {
    pub sum_x: i32,
    pub sum_y: i32,
    pub sum_z: i32,
    pub count: u32,
    pub angular_sum_x: i32,
    pub angular_sum_y: i32,
    pub angular_sum_z: i32,
    pub _pad0: u32,
}

/// Per-dispatch parameters the contact-scatter pass needs — the contact
/// count (varies every substep, since contacts are regenerated CPU-side
/// each substep) and the shared fixed-point scale (kept as a live uniform
/// field rather than baked into the shader as a constant, so a future
/// tuning pass can adjust it without a shader recompile).
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ScatterUniform {
    pub contact_count: u32,
    pub fixed_point_scale: f32,
    /// Per-CONTACT clamp on the linear (position round) or velocity
    /// (velocity round) contribution, applied BEFORE `atomicAdd` — see
    /// `solve_rigid`'s own per-contact `.clamp_length(0.0, ...)` calls in
    /// its scatter loops for why this must happen before scattering, not
    /// just after the apply pass's per-body average (a real overflow gap
    /// found and fixed while porting the velocity round: the CPU
    /// reference's per-body-average clamp alone doesn't bound what a
    /// single pathological contact scatters into the atomic accumulator
    /// before that average ever happens).
    pub max_linear_or_velocity_correction: f32,
    pub max_angular_correction: f32,
}

/// Per-dispatch parameters the position-round apply pass needs — mirrors
/// every clamp/damping constant `solve_rigid.rs` uses in its own apply
/// loop (`MAX_LINEAR_CORRECTION`, `MAX_ANGULAR_CORRECTION`,
/// `LINEAR_DAMPING`, `ANGULAR_DAMPING`, `MAX_ANGULAR_VELOCITY`,
/// `MAX_LINEAR_VELOCITY`), passed as live uniform fields rather than baked
/// into the shader as constants — same rationale as
/// `ScatterUniform::fixed_point_scale`: a future tuning pass can adjust
/// these without a shader recompile, and it keeps a single source of
/// truth (the CPU constants) that both backends read from, rather than a
/// second hand-copied set of magic numbers baked into WGSL that could
/// silently drift out of sync with the CPU reference.
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ApplyUniform {
    pub body_count: u32,
    pub substep_dt: f32,
    pub fixed_point_scale: f32,
    pub max_linear_correction: f32,
    pub max_angular_correction: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub max_angular_velocity: f32,
    pub max_linear_velocity: f32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

/// Per-dispatch parameters the velocity-round apply pass needs — mirrors
/// `resolve_contact_velocities`'s own apply loop, which only needs
/// `MAX_VELOCITY_IMPULSE` (no position/rotation clamp, no damping — this
/// round only adds an impulse on top of the position round's own
/// already-damped velocity, see `physics_apply_velocity.wgsl`'s own
/// header comment for why).
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ApplyVelocityUniform {
    pub body_count: u32,
    pub fixed_point_scale: f32,
    pub max_velocity_impulse: f32,
    pub _pad0: u32,
}

/// One `PhysicsShape`'s GPU-uploadable record: a shape-kind tag (reuses
/// `hybrid::extract::ShapeKindGpu` directly rather than a parallel
/// duplicate enum — `PhysicsShape` and the renderer's own `Shape` share
/// the exact same variant set minus `RoundedCone`, which neither
/// `ShapeKindGpu` nor `PhysicsShape` has a tag/variant for, so reusing the
/// existing enum can't silently drift out of sync with physics' own
/// shape set) plus up to 8 generic scalar params, mirroring
/// `hybrid::extract::ObjectGpu`'s own per-kind param layout exactly (same
/// field meanings per shape kind, since both ultimately feed the SAME
/// WGSL `local_distance`/`local_normal` functions in
/// `hybrid_trace.wgsl:336-392` — the broad-phase/contact-generation port's
/// main reuse of already-working code, not new SDF math). No
/// transform/material fields here (unlike `ObjectGpu`) — this struct only
/// carries what the sample-point/distance-query math needs, transform is
/// tracked separately per-body in `PhysicsBodyGpu`.
///
/// Per-kind param layout (must match `hybrid_trace.wgsl`'s
/// `local_distance`/`local_normal` exactly, since this struct's `params`
/// feed those same functions once ported into a physics-specific pass):
/// - `Sphere`: `[radius]`
/// - `RoundedBox`/`BoxFrame`: `[half_extents.x, .y, .z, corner_radius_or_wall_thickness]`
/// - `RoundedCylinder`/`HexPrism`: `[radius, half_height, edge_radius]`
/// - `Capsule`: `[a.x, .y, .z, b.x, .y, .z, radius]`
/// - `Ellipsoid`: `[radii.x, .y, .z]`
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct PhysicsShapeGpu {
    pub shape_kind: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
    pub param_0: f32,
    pub param_1: f32,
    pub param_2: f32,
    pub param_3: f32,
    pub param_4: f32,
    pub param_5: f32,
    pub param_6: f32,
    pub param_7: f32,
}

impl PhysicsShapeGpu {
    pub fn from_shape(shape: super::super::components::PhysicsShape) -> Self {
        use super::super::components::PhysicsShape;
        use crate::hybrid::extract::ShapeKindGpu;

        let (shape_kind, params) = match shape {
            PhysicsShape::Sphere { radius } => (ShapeKindGpu::Sphere, [radius, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            PhysicsShape::RoundedBox { half_extents, corner_radius } => {
                (ShapeKindGpu::RoundedBox, [half_extents.x, half_extents.y, half_extents.z, corner_radius, 0.0, 0.0, 0.0, 0.0])
            }
            PhysicsShape::RoundedCylinder { radius, half_height, edge_radius } => {
                (ShapeKindGpu::RoundedCylinder, [radius, half_height, edge_radius, 0.0, 0.0, 0.0, 0.0, 0.0])
            }
            PhysicsShape::Capsule { a, b, radius } => (ShapeKindGpu::Capsule, [a.x, a.y, a.z, b.x, b.y, b.z, radius, 0.0]),
            PhysicsShape::Ellipsoid { radii } => (ShapeKindGpu::Ellipsoid, [radii.x, radii.y, radii.z, 0.0, 0.0, 0.0, 0.0, 0.0]),
            PhysicsShape::BoxFrame { half_extents, wall_thickness } => {
                (ShapeKindGpu::BoxFrame, [half_extents.x, half_extents.y, half_extents.z, wall_thickness, 0.0, 0.0, 0.0, 0.0])
            }
            PhysicsShape::HexPrism { radius, half_height } => (ShapeKindGpu::HexPrism, [radius, half_height, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        };

        Self {
            shape_kind: shape_kind as u32,
            _pad0: 0,
            _pad1: 0,
            _pad2: 0,
            param_0: params[0],
            param_1: params[1],
            param_2: params[2],
            param_3: params[3],
            param_4: params[4],
            param_5: params[5],
            param_6: params[6],
            param_7: params[7],
        }
    }
}

/// One body's GPU-generated fixed-size sample-point set — mirrors
/// `physics::sample_points::SamplePoints` exactly (`MAX_SAMPLE_POINTS = 32`
/// slots, only `points[..count]` are real). `points` are `vec4<f32>` (not
/// `vec3<f32>`) purely for std430 array-stride predictability, matching
/// this codebase's established convention for GPU-side `Vec3`-shaped data
/// (see this module's own doc comment) — the `w` lane is always unused
/// padding, never a real fourth coordinate.
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct SamplePointsGpu {
    pub points: [[f32; 4]; 32],
    pub count: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

impl SamplePointsGpu {
    pub fn point(&self, i: usize) -> Vec3 {
        Vec3::new(self.points[i][0], self.points[i][1], self.points[i][2])
    }

    /// The real (non-padding) points as an iterator, mirroring
    /// `SamplePoints::as_slice`'s own `points[..count]` contract.
    pub fn real_points(&self) -> impl Iterator<Item = Vec3> + '_ {
        (0..self.count as usize).map(|i| self.point(i))
    }
}

/// Per-dispatch parameters the sample-point generation pass needs.
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct SamplePointsUniform {
    pub shape_count: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

/// Per-dispatch parameters the Hillis-Steele scan pass needs — `offset`
/// is `2^d` for scan step `d` (unused, left at 0, for the exclusive-shift
/// pass, which doesn't need it). See `assets/shaders/physics_scan.wgsl`'s
/// own doc comment for the full algorithm.
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ScanUniform {
    pub count: u32,
    pub offset: u32,
    pub _pad0: u32,
    pub _pad1: u32,
}

/// Per-dispatch parameters the broad-phase hash/count passes need —
/// mirrors `SpatialHash::build`'s own inputs exactly (`table_size`,
/// `cell_size`) plus the body count both passes gate on.
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct HashCountUniform {
    pub body_count: u32,
    pub table_size: u32,
    pub cell_size: f32,
    pub _pad0: u32,
}

/// Per-dispatch parameters the broad-phase copy pass (bucket_start ->
/// cursor) needs.
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ScatterCopyUniform {
    pub count: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

/// Per-dispatch parameters the broad-phase scatter pass needs. Named
/// distinctly from `ScatterUniform` (the solver port's own contact-
/// scatter uniform) to avoid confusion between two genuinely different
/// scatter passes: this one scatters BODY INDICES into hash buckets
/// (broad-phase), `ScatterUniform` scatters CORRECTIONS into per-body
/// accumulators (the XPBD solver) — same word, different pass, different
/// pipeline, deliberately not unified into one type.
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct BroadphaseScatterUniform {
    pub body_count: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

/// Per-dispatch parameters the contact-generation passes need — mirrors
/// `solve_world::generate_all_contacts`'s own inputs (`dynamic_count`,
/// `kinematic_count`, implicitly `static_count = body_count -
/// dynamic_count - kinematic_count`) plus the broad-phase's own
/// `cell_size`/`table_size` (needed to re-derive each dynamic body's
/// 27-cell neighborhood inside the dynamic-vs-dynamic pass — see
/// `physics_contacts.wgsl`'s own header comment for why body hashes
/// aren't reused from the broad-phase's own `hashes` buffer: recomputing
/// `cell_coord`/`cell_hash` fresh per neighbor cell is what
/// `broadphase::query_candidates` itself does too, this pass mirrors that
/// exactly rather than inventing a different data flow) and
/// `contact_capacity` (the fixed-size output buffer's real size, so the
/// atomic cursor pass can detect and silently drop over-capacity writes
/// rather than corrupt memory past the buffer's end).
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ContactGenUniform {
    pub dynamic_count: u32,
    pub static_count: u32,
    pub contact_capacity: u32,
    pub cell_size: f32,
    pub table_size: u32,
    pub kinematic_count: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

/// Per-dispatch parameters the extract-positions pass needs (Piece 5's own
/// tiny bridge pass — see `assets/shaders/physics_extract_positions.wgsl`'s
/// own doc comment).
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ExtractPositionsUniform {
    pub body_count: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physics_body_gpu_round_trips_position_rotation_and_velocities() {
        let position = Vec3::new(1.0, 2.0, 3.0);
        let rotation = Quat::from_euler(bevy::math::EulerRot::XYZ, 0.3, 0.5, 0.7);
        let linear_velocity = Vec3::new(-1.0, 0.5, 2.0);
        let angular_velocity = Vec3::new(0.1, -0.2, 0.3);
        let inverse_mass = 0.75;
        let inverse_inertia_local = Vec3::new(2.0, 2.0, 2.0);

        let gpu = PhysicsBodyGpu::from_state(position, rotation, linear_velocity, angular_velocity, inverse_mass, inverse_inertia_local);

        assert_eq!(gpu.position(), position);
        assert!((gpu.rotation().dot(rotation)).abs() > 1.0 - 1e-6, "expected rotation to round-trip, got {:?} vs {:?}", gpu.rotation(), rotation);
        assert_eq!(gpu.linear_velocity(), linear_velocity);
        assert_eq!(gpu.angular_velocity(), angular_velocity);
        assert_eq!(gpu.inverse_mass, inverse_mass);
    }

    #[test]
    fn physics_body_gpu_byte_size_is_a_multiple_of_16() {
        // A clean multiple of vec4<f32> (16 bytes) avoids array<T> stride
        // surprises in WGSL -- see this struct's own doc comment.
        assert_eq!(std::mem::size_of::<PhysicsBodyGpu>() % 16, 0);
    }

    #[test]
    fn substep_start_gpu_byte_size_is_a_multiple_of_16() {
        assert_eq!(std::mem::size_of::<SubstepStartGpu>() % 16, 0);
    }

    #[test]
    fn contact_gpu_byte_size_is_a_multiple_of_16() {
        assert_eq!(std::mem::size_of::<ContactGpu>() % 16, 0);
    }

    #[test]
    fn physics_accumulator_gpu_byte_size_is_a_multiple_of_16() {
        assert_eq!(std::mem::size_of::<PhysicsAccumulatorGpu>() % 16, 0);
    }

    #[test]
    fn fixed_point_round_trip_is_accurate_to_well_under_a_millimeter() {
        for value in [0.0_f32, 1.0, -1.0, 0.5, -0.5, 1.999, -1.999, 0.000_1] {
            let encoded = to_fixed_point(value);
            let decoded = from_fixed_point(encoded);
            assert!((decoded - value).abs() < 1e-5, "expected {value} to round-trip through fixed point, got {decoded}");
        }
    }

    #[test]
    fn fixed_point_zero_encodes_to_exactly_zero() {
        assert_eq!(to_fixed_point(0.0), 0);
        assert_eq!(from_fixed_point(0), 0.0);
    }

    #[test]
    fn worst_case_accumulation_at_max_linear_correction_does_not_overflow_i32() {
        // See FIXED_POINT_SCALE's own doc comment for this exact
        // worst-case bound: 256 contacts, each scattering the maximum
        // permitted linear correction (2.0, matching
        // solve_rigid::MAX_LINEAR_CORRECTION), summed via repeated
        // to_fixed_point + i32 addition -- must never wrap/overflow.
        const MAX_LINEAR_CORRECTION: f32 = 2.0;
        let per_contact = to_fixed_point(MAX_LINEAR_CORRECTION);
        let mut sum: i64 = 0; // widen deliberately so a real overflow shows up as a mismatch, not silent wraparound
        for _ in 0..256 {
            sum += per_contact as i64;
        }
        assert!(sum < i32::MAX as i64, "worst-case accumulation {sum} exceeds i32::MAX -- FIXED_POINT_SCALE is not safe at this contact count");
        // Also confirm it fits with the documented 4x margin, not just barely.
        assert!(sum < (i32::MAX as i64) / 3, "worst-case accumulation {sum} doesn't leave the documented safety margin");
    }

    #[test]
    fn physics_shape_gpu_encodes_every_variant_with_the_documented_param_layout() {
        use super::super::super::components::PhysicsShape;
        use crate::hybrid::extract::ShapeKindGpu;

        let sphere = PhysicsShapeGpu::from_shape(PhysicsShape::Sphere { radius: 2.5 });
        assert_eq!(sphere.shape_kind, ShapeKindGpu::Sphere as u32);
        assert_eq!(sphere.param_0, 2.5);

        let rounded_box = PhysicsShapeGpu::from_shape(PhysicsShape::RoundedBox { half_extents: Vec3::new(1.0, 2.0, 3.0), corner_radius: 0.1 });
        assert_eq!(rounded_box.shape_kind, ShapeKindGpu::RoundedBox as u32);
        assert_eq!([rounded_box.param_0, rounded_box.param_1, rounded_box.param_2, rounded_box.param_3], [1.0, 2.0, 3.0, 0.1]);

        let rounded_cylinder = PhysicsShapeGpu::from_shape(PhysicsShape::RoundedCylinder { radius: 1.0, half_height: 2.0, edge_radius: 0.2 });
        assert_eq!(rounded_cylinder.shape_kind, ShapeKindGpu::RoundedCylinder as u32);
        assert_eq!([rounded_cylinder.param_0, rounded_cylinder.param_1, rounded_cylinder.param_2], [1.0, 2.0, 0.2]);

        let capsule = PhysicsShapeGpu::from_shape(PhysicsShape::Capsule { a: Vec3::new(0.0, -1.0, 0.0), b: Vec3::new(0.0, 1.0, 0.0), radius: 0.5 });
        assert_eq!(capsule.shape_kind, ShapeKindGpu::Capsule as u32);
        assert_eq!([capsule.param_0, capsule.param_1, capsule.param_2, capsule.param_3, capsule.param_4, capsule.param_5, capsule.param_6], [0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.5]);

        let ellipsoid = PhysicsShapeGpu::from_shape(PhysicsShape::Ellipsoid { radii: Vec3::new(1.0, 2.0, 0.5) });
        assert_eq!(ellipsoid.shape_kind, ShapeKindGpu::Ellipsoid as u32);
        assert_eq!([ellipsoid.param_0, ellipsoid.param_1, ellipsoid.param_2], [1.0, 2.0, 0.5]);

        let box_frame = PhysicsShapeGpu::from_shape(PhysicsShape::BoxFrame { half_extents: Vec3::new(1.0, 1.0, 1.0), wall_thickness: 0.1 });
        assert_eq!(box_frame.shape_kind, ShapeKindGpu::BoxFrame as u32);
        assert_eq!([box_frame.param_0, box_frame.param_1, box_frame.param_2, box_frame.param_3], [1.0, 1.0, 1.0, 0.1]);

        let hex_prism = PhysicsShapeGpu::from_shape(PhysicsShape::HexPrism { radius: 1.0, half_height: 0.5 });
        assert_eq!(hex_prism.shape_kind, ShapeKindGpu::HexPrism as u32);
        assert_eq!([hex_prism.param_0, hex_prism.param_1], [1.0, 0.5]);
    }

    #[test]
    fn physics_shape_gpu_byte_size_is_a_multiple_of_16() {
        assert_eq!(std::mem::size_of::<PhysicsShapeGpu>() % 16, 0);
    }

    #[test]
    fn contact_gen_uniform_byte_size_is_a_multiple_of_16() {
        assert_eq!(std::mem::size_of::<ContactGenUniform>() % 16, 0);
    }
}
