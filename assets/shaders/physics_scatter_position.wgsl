// GPU physics Piece 2: contact-scatter pass (position/rotation round). One
// invocation per CONTACT (not per body, not per substep). Computes the
// same generalized-inverse-mass positional correction
// src/physics/solve_rigid.rs's solve_substep_jacobi computes per contact
// (Müller et al. 2020, eq. 2-4 — see that function's own doc comment),
// then scatters it into both referenced bodies' accumulators via
// atomicAdd -- the CPU reference's serial per-contact accumulation loop
// and this pass's per-invocation atomicAdd calls are the SAME algorithm
// (accumulate-then-divide Jacobi), just executed with real GPU
// concurrency instead of a CPU loop; that's the property this pass's own
// parity test must prove, not just "produces a plausible-looking number."
//
// WGSL has no native atomic<f32> -- corrections are encoded as
// fixed-point i32 before atomicAdd (see
// src/physics/gpu/types.rs::FIXED_POINT_SCALE's own doc comment for the
// overflow-safety bound this scale was picked against, and
// solve_rigid::MAX_LINEAR_CORRECTION/MAX_ANGULAR_CORRECTION for why both
// backends clamp corrections identically before this pass ever needs to
// encode them).
//
// Self-contained (no shared imports), per this codebase's established
// per-pass-file convention (see hybrid_ddgi_relight.wgsl's own doc
// comment for why).

struct PhysicsBody {
    position: vec4<f32>,              // xyz = position, w = inverse_mass
    rotation: vec4<f32>,              // xyzw quaternion
    linear_velocity: vec4<f32>,       // xyz, w unused
    angular_velocity: vec4<f32>,      // xyz, w unused
    inverse_inertia_local: vec4<f32>, // xyz, w unused
}

struct Contact {
    body_a: u32,
    body_b: u32,
    depth: f32,
    _pad0: f32,
    point_world: vec4<f32>,  // xyz, w unused
    normal_world: vec4<f32>, // xyz, w unused
}

// Matches src/physics/gpu/types.rs::PhysicsAccumulatorGpu exactly --
// fixed-point i32 sums (see this file's own header comment), plain u32
// count. `atomic<i32>`/`atomic<u32>` per WGSL's atomic-storage rules: a
// storage buffer field can be declared atomic directly, no separate
// non-atomic mirror struct needed since this buffer is NEVER read as
// plain i32/u32 -- only ever atomicAdd'd into (scatter passes) or
// atomicLoad'd from (apply passes, in a later piece).
struct AccumulatorAtomic {
    sum_x: atomic<i32>,
    sum_y: atomic<i32>,
    sum_z: atomic<i32>,
    count: atomic<u32>,
    angular_sum_x: atomic<i32>,
    angular_sum_y: atomic<i32>,
    angular_sum_z: atomic<i32>,
    _pad0: u32,
}

struct ScatterUniform {
    contact_count: u32,
    fixed_point_scale: f32,
    // Per-CONTACT clamp, applied BEFORE atomicAdd -- see
    // src/physics/gpu/types.rs::ScatterUniform's own doc comment for why
    // this must happen here, not just after the apply pass's per-body
    // average (a real overflow gap found and fixed while porting the
    // velocity round).
    max_linear_correction: f32,
    max_angular_correction: f32,
}

@group(0) @binding(0) var<uniform> params: ScatterUniform;
@group(0) @binding(1) var<storage, read> bodies: array<PhysicsBody>;
@group(0) @binding(2) var<storage, read> contacts: array<Contact>;
@group(0) @binding(3) var<storage, read_write> accumulators: array<AccumulatorAtomic>;

fn to_fixed_point(value: f32) -> i32 {
    return i32(round(value * params.fixed_point_scale));
}

fn clamp_length(v: vec3<f32>, max_length: f32) -> vec3<f32> {
    let len = length(v);
    if len > max_length {
        return v * (max_length / len);
    }
    return v;
}

// Body-local diagonal inverse inertia rotated into world space, applied to
// a vector -- mirrors solve_rigid::BodyState::apply_world_inverse_inertia
// exactly: `R * (diag(I^-1) * (R^-1 * v))`.
fn apply_world_inverse_inertia(body: PhysicsBody, v: vec3<f32>) -> vec3<f32> {
    let inv_rotation = vec4<f32>(-body.rotation.xyz, body.rotation.w);
    let local_v = quat_rotate(inv_rotation, v);
    let scaled = local_v * body.inverse_inertia_local.xyz;
    return quat_rotate(body.rotation, scaled);
}

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let qv = q.xyz;
    let t = 2.0 * cross(qv, v);
    return v + q.w * t + cross(qv, t);
}

@compute @workgroup_size(64, 1, 1)
fn physics_scatter_position_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.contact_count {
        return;
    }

    let contact = contacts[i];
    let ia = contact.body_a;
    let ib = contact.body_b;
    let body_a = bodies[ia];
    let body_b = bodies[ib];
    let inv_mass_a = body_a.position.w;
    let inv_mass_b = body_b.position.w;

    let ra = contact.point_world.xyz - body_a.position.xyz;
    let rb = contact.point_world.xyz - body_b.position.xyz;
    let n = contact.normal_world.xyz;

    let angular_a = cross(ra, n);
    let angular_b = cross(rb, n);
    let w_a = inv_mass_a + dot(angular_a, apply_world_inverse_inertia(body_a, angular_a));
    let w_b = inv_mass_b + dot(angular_b, apply_world_inverse_inertia(body_b, angular_b));
    let total_w = w_a + w_b;
    if total_w <= 0.0 {
        return;
    }

    let lambda = contact.depth / total_w;
    let impulse = n * lambda;

    if inv_mass_a > 0.0 {
        let correction_a = clamp_length(impulse * inv_mass_a, params.max_linear_correction);
        let angular_correction_a = clamp_length(apply_world_inverse_inertia(body_a, cross(ra, impulse)), params.max_angular_correction);
        atomicAdd(&accumulators[ia].sum_x, to_fixed_point(correction_a.x));
        atomicAdd(&accumulators[ia].sum_y, to_fixed_point(correction_a.y));
        atomicAdd(&accumulators[ia].sum_z, to_fixed_point(correction_a.z));
        atomicAdd(&accumulators[ia].angular_sum_x, to_fixed_point(angular_correction_a.x));
        atomicAdd(&accumulators[ia].angular_sum_y, to_fixed_point(angular_correction_a.y));
        atomicAdd(&accumulators[ia].angular_sum_z, to_fixed_point(angular_correction_a.z));
        atomicAdd(&accumulators[ia].count, 1u);
    }
    if inv_mass_b > 0.0 {
        let correction_b = clamp_length(impulse * inv_mass_b, params.max_linear_correction);
        let angular_correction_b = clamp_length(apply_world_inverse_inertia(body_b, cross(rb, impulse)), params.max_angular_correction);
        atomicAdd(&accumulators[ib].sum_x, -to_fixed_point(correction_b.x));
        atomicAdd(&accumulators[ib].sum_y, -to_fixed_point(correction_b.y));
        atomicAdd(&accumulators[ib].sum_z, -to_fixed_point(correction_b.z));
        atomicAdd(&accumulators[ib].angular_sum_x, -to_fixed_point(angular_correction_b.x));
        atomicAdd(&accumulators[ib].angular_sum_y, -to_fixed_point(angular_correction_b.y));
        atomicAdd(&accumulators[ib].angular_sum_z, -to_fixed_point(angular_correction_b.z));
        atomicAdd(&accumulators[ib].count, 1u);
    }
}
