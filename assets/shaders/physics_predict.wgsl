// GPU physics Piece 1: predict pass. One invocation per BODY (not per
// substep, not per contact — this pass runs once per substep, dispatched
// with body_count work items). Mirrors src/physics/solve_world.rs's own
// substep-loop prediction step bit-for-bit: (1) snapshot the body's
// pre-integration state into `substep_start` (needed to later derive
// contact-corrected velocity without the catastrophic-cancellation bug a
// position-delta derivation has at world-scale positions — see
// SubstepStartGpu's own Rust-side doc comment), (2) integrate gravity into
// velocity then position (symplectic Euler: the NEWLY updated velocity is
// what advances position, matching solve_world.rs's `state.linear_velocity
// += g * substep_dt; state.position += state.linear_velocity *
// substep_dt;` exactly), (3) separately predict rotation from the body's
// CURRENT (gravity-untouched) angular velocity via the same half-step
// quaternion-derivative formula: `q' = normalize(q + 0.5*dt*[w,0]*q)`.
//
// Static bodies (inverse_mass == 0.0) skip translation, matching
// solve_world.rs's `if state.inverse_mass > 0.0` gate. Bodies with zero
// inverse inertia (on every axis) skip rotation prediction, matching
// solve_world.rs's `if state.inverse_inertia_local != Vec3::ZERO` gate —
// a body that cannot rotate must never be predicted to, even by a tiny
// amount from floating-point noise.
//
// Self-contained (no shared imports) per this codebase's established
// per-pass-file convention (see hybrid_ddgi_relight.wgsl's own doc comment
// for why: hybrid_denoise.wgsl/hybrid_temporal.wgsl already each carry
// their own copies of shared structs rather than importing a library).

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

// Flat scalar fields, NOT a vec3 for gravity_center -- must match
// src/physics/gpu/types.rs's PhysicsPredictUniform exactly, field for
// field. A `vec3<f32>` here would NOT reliably pack with a following
// scalar the same way encase's ShaderType derive lays out four
// consecutive f32/u32 Rust fields (this was a real bug caught by this
// pass's own parity test: gravity silently read as zero for every body
// because vec3-then-scalar WGSL layout didn't match the flat-scalar Rust
// struct's actual byte offsets).
struct PhysicsPredictUniform {
    gravity_center_x: f32,
    gravity_center_y: f32,
    gravity_center_z: f32,
    gravity_magnitude: f32,
    substep_dt: f32,
    body_count: u32,
    _pad0: u32,
    _pad1: u32,
}

@group(0) @binding(0) var<uniform> params: PhysicsPredictUniform;
@group(0) @binding(1) var<storage, read_write> bodies: array<PhysicsBody>;
@group(0) @binding(2) var<storage, read_write> substep_start: array<SubstepStart>;

// Quaternion multiply, Hamilton convention (x,y,z,w) — matches
// bevy::math::Quat's own multiplication order, since this must reproduce
// `delta_q * state.rotation` (delta_q on the LEFT) bit-for-bit.
fn quat_mul(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    );
}

// Point-source gravity — mirrors physics::solve_static::PhysicsGravity::
// acceleration_at exactly: constant magnitude (not inverse-square),
// direction toward `center`, zero at the center itself rather than NaN
// from normalizing a zero-length vector.
fn gravity_acceleration_at(position: vec3<f32>) -> vec3<f32> {
    let center = vec3<f32>(params.gravity_center_x, params.gravity_center_y, params.gravity_center_z);
    let to_center = center - position;
    let distance = length(to_center);
    if distance < 1e-8 {
        return vec3<f32>(0.0, 0.0, 0.0);
    }
    return (to_center / distance) * params.gravity_magnitude;
}

@compute @workgroup_size(64, 1, 1)
fn physics_predict_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.body_count {
        return;
    }

    var body = bodies[i];

    // (1) Snapshot BEFORE integration mutates anything.
    substep_start[i].position = vec4<f32>(body.position.xyz, 0.0);
    substep_start[i].rotation = body.rotation;
    substep_start[i].linear_velocity = vec4<f32>(body.linear_velocity.xyz, 0.0);
    substep_start[i].angular_velocity = vec4<f32>(body.angular_velocity.xyz, 0.0);

    let inverse_mass = body.position.w;

    // (2) Linear: symplectic Euler, gravity direction from CURRENT position.
    if inverse_mass > 0.0 {
        let g = gravity_acceleration_at(body.position.xyz);
        let new_linear_velocity = body.linear_velocity.xyz + g * params.substep_dt;
        body.linear_velocity = vec4<f32>(new_linear_velocity, 0.0);
        body.position = vec4<f32>(body.position.xyz + new_linear_velocity * params.substep_dt, inverse_mass);
    }

    // (3) Rotation: half-step quaternion derivative from CURRENT angular velocity.
    if any(body.inverse_inertia_local.xyz != vec3<f32>(0.0, 0.0, 0.0)) {
        let half_dt_omega = body.angular_velocity.xyz * (0.5 * params.substep_dt);
        let delta_q = vec4<f32>(half_dt_omega, 0.0);
        let derivative = quat_mul(delta_q, body.rotation);
        let predicted = body.rotation + derivative;
        body.rotation = predicted / length(predicted);
    }

    bodies[i] = body;
}
