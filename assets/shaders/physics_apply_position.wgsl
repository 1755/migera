// GPU physics Piece 3: apply-average pass (position/rotation round). One
// invocation per BODY. Reads the accumulator buffer
// physics_scatter_position.wgsl's atomicAdd calls filled this substep,
// divides by count, clamps (MAX_LINEAR_CORRECTION/MAX_ANGULAR_CORRECTION
// -- see src/physics/solve_rigid.rs's own doc comments for why these
// exist independently of damping), applies the correction to
// position/rotation, derives velocity from the SubstepStart snapshot
// (NOT a position delta -- see SubstepStartGpu's own doc comment for the
// catastrophic-cancellation bug that approach has at world-scale
// positions), then applies per-substep damping and the hard linear/
// angular velocity ceilings (MAX_LINEAR_VELOCITY/MAX_ANGULAR_VELOCITY --
// a direct clamp on the derived STATE, not just the correction that
// produced it: dividing even a correctly-clamped correction by a small
// substep dt can still produce an arbitrarily large velocity, see
// MAX_LINEAR_VELOCITY's own doc comment for the real regression this
// closed). Mirrors solve_rigid.rs's solve_substep_jacobi apply loop
// through its position/rotation round EXACTLY (up to but not including
// resolve_contact_velocities's own velocity-round scatter/apply pair,
// which is Piece 4's scope, not this pass's).
//
// A body with acc.count == 0 (no contact touched it this substep) is left
// completely untouched by the correction/velocity-derivation block below
// -- its position/rotation/velocity already hold exactly what the
// predict pass produced -- but damping/clamping still runs
// UNCONDITIONALLY every substep, contact or not, matching the CPU
// reference's own documented fix for a real regression (a body that lost
// its last contact right after picking up a large angular velocity must
// still get damped/clamped, not keep that velocity forever).
//
// Self-contained (no shared imports), per this codebase's established
// per-pass-file convention.

struct PhysicsBody {
    position: vec4<f32>,              // xyz = position, w = inverse_mass
    rotation: vec4<f32>,              // xyzw quaternion
    linear_velocity: vec4<f32>,       // xyz, w unused
    angular_velocity: vec4<f32>,      // xyz, w unused
    inverse_inertia_local: vec4<f32>, // xyz, w unused
}

struct SubstepStart {
    position: vec4<f32>,          // xyz, w unused
    rotation: vec4<f32>,          // xyzw quaternion
    linear_velocity: vec4<f32>,   // xyz, w unused
    angular_velocity: vec4<f32>,  // xyz, w unused
}

// Matches src/physics/gpu/types.rs::PhysicsAccumulatorGpu exactly -- this
// pass only ever READS the accumulator (atomicLoad), never atomicAdd's
// into it, so a plain (non-atomic) struct is fine here even though the
// scatter pass's own copy of this struct declares atomic fields -- WGSL
// requires the underlying buffer's bind-group usage to match across
// passes in terms of size/layout, not atomicity, and this buffer is bound
// read_write here (never written) purely so a future pass reusing the
// same bind group slot doesn't need a second layout.
struct Accumulator {
    sum_x: i32,
    sum_y: i32,
    sum_z: i32,
    count: u32,
    angular_sum_x: i32,
    angular_sum_y: i32,
    angular_sum_z: i32,
    _pad0: u32,
}

struct ApplyUniform {
    body_count: u32,
    substep_dt: f32,
    fixed_point_scale: f32,
    max_linear_correction: f32,
    max_angular_correction: f32,
    linear_damping: f32,
    angular_damping: f32,
    max_angular_velocity: f32,
    max_linear_velocity: f32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> params: ApplyUniform;
@group(0) @binding(1) var<storage, read_write> bodies: array<PhysicsBody>;
@group(0) @binding(2) var<storage, read> substep_start: array<SubstepStart>;
@group(0) @binding(3) var<storage, read> accumulators: array<Accumulator>;

fn from_fixed_point(value: i32) -> f32 {
    return f32(value) / params.fixed_point_scale;
}

fn quat_mul(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    );
}

@compute @workgroup_size(64, 1, 1)
fn physics_apply_position_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.body_count {
        return;
    }

    var body = bodies[i];
    let acc = accumulators[i];

    if acc.count > 0u {
        let count = f32(acc.count);
        var correction = vec3<f32>(from_fixed_point(acc.sum_x), from_fixed_point(acc.sum_y), from_fixed_point(acc.sum_z)) / count;
        var angular_correction = vec3<f32>(from_fixed_point(acc.angular_sum_x), from_fixed_point(acc.angular_sum_y), from_fixed_point(acc.angular_sum_z)) / count;

        let linear_magnitude = length(correction);
        if linear_magnitude > params.max_linear_correction {
            correction = correction * (params.max_linear_correction / linear_magnitude);
        }

        let angular_magnitude = length(angular_correction);
        if angular_magnitude > params.max_angular_correction {
            angular_correction = angular_correction * (params.max_angular_correction / angular_magnitude);
        }

        body.position = vec4<f32>(body.position.xyz + correction, body.position.w);

        let delta_q = vec4<f32>(angular_correction, 0.0);
        let derivative = quat_mul(delta_q, body.rotation);
        let updated = body.rotation + 0.5 * derivative;
        body.rotation = updated / length(updated);

        let start = substep_start[i];
        body.linear_velocity = vec4<f32>(start.linear_velocity.xyz + correction / params.substep_dt, 0.0);
        body.angular_velocity = vec4<f32>(start.angular_velocity.xyz + angular_correction / params.substep_dt, 0.0);
    }

    var linear_velocity = body.linear_velocity.xyz * max(1.0 - params.linear_damping * params.substep_dt, 0.0);
    var angular_velocity = body.angular_velocity.xyz * max(1.0 - params.angular_damping * params.substep_dt, 0.0);

    let linear_velocity_magnitude = length(linear_velocity);
    if linear_velocity_magnitude > params.max_linear_velocity {
        linear_velocity = linear_velocity * (params.max_linear_velocity / linear_velocity_magnitude);
    }

    let angular_velocity_magnitude = length(angular_velocity);
    if angular_velocity_magnitude > params.max_angular_velocity {
        angular_velocity = angular_velocity * (params.max_angular_velocity / angular_velocity_magnitude);
    }

    body.linear_velocity = vec4<f32>(linear_velocity, 0.0);
    body.angular_velocity = vec4<f32>(angular_velocity, 0.0);

    bodies[i] = body;
}
