// GPU physics end-to-end wiring (Piece 5): extracts just the position
// (xyz) from the solver's own live PhysicsBody buffer into the
// [f32;4]-per-element layout the broad-phase pass expects
// (physics_broadphase_hash.wgsl's own `positions: array<vec4<f32>>`
// binding). Needed because the solver's body buffer packs position as
// only the first 3 floats of an 80-byte struct (plus rotation/velocity/
// inertia), and broad-phase needs to be rebuilt every substep (bodies
// move during the solve, per solve_world.rs's own doc comment) -- this
// tiny dispatch is the cheapest GPU-side way to bridge the two layouts
// with no CPU round-trip, one invocation per body, run immediately before
// each substep's own broad-phase dispatch chain.
//
// Self-contained (no shared imports), per this codebase's established
// per-pass-file convention.

struct PhysicsBody {
    position: vec4<f32>,              // xyz = position, w = inverse_mass
    rotation: vec4<f32>,
    linear_velocity: vec4<f32>,
    angular_velocity: vec4<f32>,
    inverse_inertia_local: vec4<f32>,
}

struct ExtractPositionsUniform {
    body_count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> params: ExtractPositionsUniform;
@group(0) @binding(1) var<storage, read> bodies: array<PhysicsBody>;
@group(0) @binding(2) var<storage, read_write> positions_out: array<vec4<f32>>;

@compute @workgroup_size(64, 1, 1)
fn physics_extract_positions_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.body_count {
        return;
    }
    positions_out[i] = vec4<f32>(bodies[i].position.xyz, 0.0);
}
