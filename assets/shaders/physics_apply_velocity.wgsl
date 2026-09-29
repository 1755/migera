// GPU physics Piece 4: apply-average pass (velocity round). One
// invocation per BODY. Mirrors resolve_contact_velocities's own apply
// loop exactly: divides the scattered linear/angular sums by count,
// clamps each to MAX_VELOCITY_IMPULSE (see
// physics_scatter_velocity.wgsl's own header comment -- the CPU
// reference's `linear_accumulators`/`angular_accumulators` always
// increment in lockstep from the same contact, so the shared `count`
// field this pass reads from the accumulator buffer is exactly
// equivalent), then ADDS to the body's existing velocity (not derived
// fresh the way the position round's apply pass derives velocity from
// SubstepStart -- this round only ever adds an impulse on top of
// whatever velocity already exists, since it runs strictly after the
// position round's own apply pass has already finished deriving that
// velocity for this substep).
//
// A body with count == 0 (no approaching contact touched it) is left
// completely untouched -- resolve_contact_velocities's own CPU loop has
// no unconditional damping/clamping step here (unlike the position
// round's apply pass), since MAX_ANGULAR_VELOCITY/damping already ran
// once this substep in the position round and this round only adds
// impulses, it doesn't re-integrate or re-damp anything.
//
// Self-contained, per this codebase's established per-pass-file
// convention.

struct PhysicsBody {
    position: vec4<f32>,              // xyz = position, w = inverse_mass
    rotation: vec4<f32>,              // xyzw quaternion
    linear_velocity: vec4<f32>,       // xyz, w unused
    angular_velocity: vec4<f32>,      // xyz, w unused
    inverse_inertia_local: vec4<f32>, // xyz, w unused
}

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

struct ApplyVelocityUniform {
    body_count: u32,
    fixed_point_scale: f32,
    max_velocity_impulse: f32,
    _pad0: u32,
}

@group(0) @binding(0) var<uniform> params: ApplyVelocityUniform;
@group(0) @binding(1) var<storage, read_write> bodies: array<PhysicsBody>;
@group(0) @binding(2) var<storage, read> accumulators: array<Accumulator>;

fn from_fixed_point(value: i32) -> f32 {
    return f32(value) / params.fixed_point_scale;
}

fn clamp_length(v: vec3<f32>, max_length: f32) -> vec3<f32> {
    let len = length(v);
    if len > max_length {
        return v * (max_length / len);
    }
    return v;
}

@compute @workgroup_size(64, 1, 1)
fn physics_apply_velocity_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.body_count {
        return;
    }

    let acc = accumulators[i];
    if acc.count == 0u {
        return;
    }

    var body = bodies[i];
    let count = f32(acc.count);

    let linear_correction = clamp_length(vec3<f32>(from_fixed_point(acc.sum_x), from_fixed_point(acc.sum_y), from_fixed_point(acc.sum_z)) / count, params.max_velocity_impulse);
    let angular_correction = clamp_length(vec3<f32>(from_fixed_point(acc.angular_sum_x), from_fixed_point(acc.angular_sum_y), from_fixed_point(acc.angular_sum_z)) / count, params.max_velocity_impulse);

    body.linear_velocity = vec4<f32>(body.linear_velocity.xyz + linear_correction, 0.0);
    body.angular_velocity = vec4<f32>(body.angular_velocity.xyz + angular_correction, 0.0);

    bodies[i] = body;
}
