// GPU physics broad-phase/contact-gen port, Piece 4: contact generation.
// Stage 3.5 Piece 2 extended this to a third path: dynamic-vs-kinematic.
// Ports src/physics/contacts.rs's generate_contacts (per-shape sd_*/
// local_distance/local_normal math already ported once in
// physics_sample_points.wgsl's own shape dispatch, reused here rather than
// duplicated a third time) plus solve_world.rs's generate_all_contacts
// three-path split: dynamic-vs-dynamic (consumes the broad-phase's own CSR
// bucket_start/bucket_items output directly, no CPU round-trip),
// dynamic-vs-kinematic, and dynamic-vs-static (both small direct flat-grid
// passes, no spatial hash — mirrors that module's own doc comment on why
// folding kinematics/statics into the dynamic-sized hash would be wrong).
// Deliberately no kinematic-vs-kinematic or kinematic-vs-static pass —
// both pairs have inverse_mass == 0.0 on both sides, a guaranteed no-op
// once fed through the solver, so generating them would be pure wasted
// sample-point-query work (see solve_world.rs's own generate_all_contacts
// doc comment for the CPU-side version of this same reasoning).
//
// Body indexing matches solve_world.rs's own three-range convention
// exactly: bodies buffer is `dynamics ++ kinematics ++ statics`,
// `dynamic_count` and `dynamic_count + kinematic_count` are the two split
// points. Sample points and shape params are precomputed once per body
// (Piece 1's own pass, run once per frame before this one — sample points
// are in BODY-LOCAL space and don't change unless the body's own shape
// changes, so recomputing them per substep would be pure waste).
//
// Output: a fixed-capacity contact buffer plus an atomic<u32> write
// cursor -- contact generation on GPU doesn't know its own output size
// ahead of time, unlike the solver port's contacts (uploaded from a
// CPU-computed Vec<Contact> with a known length). A contact is only
// written if its atomicAdd's pre-increment cursor value is still within
// capacity; the CPU reads back the final cursor value to know how many
// contacts are valid, and is responsible for a loud warning (not a silent
// truncation) if the cursor ever exceeds capacity -- this shader itself
// has no way to signal that beyond the cursor value itself being larger
// than the buffer it's paired with.
//
// Self-contained (no shared imports), per this codebase's established
// per-pass-file convention.

const SHAPE_SPHERE: u32 = 0u;
const SHAPE_ROUNDED_BOX: u32 = 1u;
const SHAPE_ROUNDED_CYLINDER: u32 = 2u;
const SHAPE_CAPSULE: u32 = 3u;
const SHAPE_ELLIPSOID: u32 = 4u;
const SHAPE_BOX_FRAME: u32 = 5u;
const SHAPE_HEX_PRISM: u32 = 6u;

struct PhysicsShapeGpu {
    shape_kind: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    param_0: f32,
    param_1: f32,
    param_2: f32,
    param_3: f32,
    param_4: f32,
    param_5: f32,
    param_6: f32,
    param_7: f32,
}

struct SamplePointsGpu {
    points: array<vec4<f32>, 32>, // xyz = point, w unused
    count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

// Mirrors physics::gpu::types::PhysicsBodyGpu's own field layout exactly
// (this pass only ever READS position/rotation, never inverse_mass/
// inverse_inertia/velocity -- but the binding must match the real struct's
// byte layout, so every field is declared here even if unused).
struct PhysicsBodyGpu {
    position: vec4<f32>, // w = inverse_mass, unused here
    rotation: vec4<f32>,
    linear_velocity: vec4<f32>,
    angular_velocity: vec4<f32>,
    inverse_inertia_local: vec4<f32>,
}

struct ContactGpu {
    body_a: u32,
    body_b: u32,
    depth: f32,
    _pad0: f32,
    point_world: vec4<f32>, // w unused
    normal_world: vec4<f32>, // w unused
}

struct ContactGenUniform {
    dynamic_count: u32,
    static_count: u32,
    contact_capacity: u32,
    cell_size: f32,
    table_size: u32,
    kinematic_count: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> params: ContactGenUniform;
@group(0) @binding(1) var<storage, read> bodies: array<PhysicsBodyGpu>;
@group(0) @binding(2) var<storage, read> shapes: array<PhysicsShapeGpu>;
@group(0) @binding(3) var<storage, read> sample_points: array<SamplePointsGpu>;
@group(0) @binding(4) var<storage, read> bucket_start: array<u32>;
@group(0) @binding(5) var<storage, read> bucket_items: array<u32>;
@group(0) @binding(6) var<storage, read_write> contact_cursor: array<atomic<u32>>; // single-element buffer
@group(0) @binding(7) var<storage, read_write> contacts_out: array<ContactGpu>;

// --- sd_*/local_distance, verbatim copies of physics_sample_points.wgsl's
// sibling hybrid_trace.wgsl twins, operating on PhysicsShapeGpu directly
// (see this file's own header for why this is a deliberate third copy,
// not a shared import) ---

fn sd_sphere(p_local: vec3<f32>, radius: f32) -> f32 {
    return length(p_local) - radius;
}

fn sd_rounded_box(p_local: vec3<f32>, half_extents: vec3<f32>, corner_radius: f32) -> f32 {
    let q = abs(p_local) - half_extents + vec3<f32>(corner_radius);
    return length(max(q, vec3<f32>(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0) - corner_radius;
}

fn sd_rounded_cylinder(p_local: vec3<f32>, radius: f32, half_height: f32, edge_radius: f32) -> f32 {
    let d = vec2<f32>(
        length(p_local.xz) - radius + edge_radius,
        abs(p_local.y) - half_height + edge_radius,
    );
    return min(max(d.x, d.y), 0.0) + length(max(d, vec2<f32>(0.0))) - edge_radius;
}

fn sd_capsule(p_local: vec3<f32>, a: vec3<f32>, b: vec3<f32>, radius: f32) -> f32 {
    let ab = b - a;
    let ap = p_local - a;
    let t = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0);
    let closest = a + ab * t;
    return length(p_local - closest) - radius;
}

fn sd_ellipsoid(p_local: vec3<f32>, radii: vec3<f32>) -> f32 {
    let k0 = length(p_local / radii);
    let k1 = length(p_local / (radii * radii));
    return k0 * (k0 - 1.0) / k1;
}

fn sd_box_frame(p_local: vec3<f32>, half_extents: vec3<f32>, wall_thickness: f32) -> f32 {
    let q = abs(p_local) - half_extents;
    let outer = length(max(q, vec3<f32>(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0);
    let inner_q = q + vec3<f32>(wall_thickness);
    let inner = length(max(inner_q, vec3<f32>(0.0))) + min(max(inner_q.x, max(inner_q.y, inner_q.z)), 0.0);
    return max(outer, -inner);
}

fn sd_hex_prism(p_local: vec3<f32>, radius: f32, half_height: f32) -> f32 {
    let q = abs(p_local);
    let k = 0.866025404;
    let hex_d = max(max(q.x, abs(0.5 * q.x + k * q.z)), abs(0.5 * q.x - k * q.z)) - radius;
    let axial_d = q.y - half_height;
    return max(hex_d, axial_d);
}

fn local_distance(shape: PhysicsShapeGpu, p_local: vec3<f32>) -> f32 {
    switch shape.shape_kind {
        case SHAPE_SPHERE: {
            return sd_sphere(p_local, shape.param_0);
        }
        case SHAPE_ROUNDED_BOX: {
            return sd_rounded_box(p_local, vec3<f32>(shape.param_0, shape.param_1, shape.param_2), shape.param_3);
        }
        case SHAPE_ROUNDED_CYLINDER: {
            return sd_rounded_cylinder(p_local, shape.param_0, shape.param_1, shape.param_2);
        }
        case SHAPE_CAPSULE: {
            let a = vec3<f32>(shape.param_0, shape.param_1, shape.param_2);
            let b = vec3<f32>(shape.param_3, shape.param_4, shape.param_5);
            return sd_capsule(p_local, a, b, shape.param_6);
        }
        case SHAPE_ELLIPSOID: {
            return sd_ellipsoid(p_local, vec3<f32>(shape.param_0, shape.param_1, shape.param_2));
        }
        case SHAPE_BOX_FRAME: {
            return sd_box_frame(p_local, vec3<f32>(shape.param_0, shape.param_1, shape.param_2), shape.param_3);
        }
        case SHAPE_HEX_PRISM: {
            return sd_hex_prism(p_local, shape.param_0, shape.param_1);
        }
        default: {
            return sd_sphere(p_local, 0.0);
        }
    }
}

const NORMAL_EPSILON: f32 = 1e-3;

fn local_normal(shape: PhysicsShapeGpu, p_local: vec3<f32>) -> vec3<f32> {
    let e = NORMAL_EPSILON;
    let dx = local_distance(shape, p_local + vec3<f32>(e, 0.0, 0.0)) - local_distance(shape, p_local - vec3<f32>(e, 0.0, 0.0));
    let dy = local_distance(shape, p_local + vec3<f32>(0.0, e, 0.0)) - local_distance(shape, p_local - vec3<f32>(0.0, e, 0.0));
    let dz = local_distance(shape, p_local + vec3<f32>(0.0, 0.0, e)) - local_distance(shape, p_local - vec3<f32>(0.0, 0.0, e));
    let g = vec3<f32>(dx, dy, dz);
    let len = length(g);
    if len < 1e-8 {
        return vec3<f32>(0.0);
    }
    return g / len;
}

// rotation * v (forward quaternion rotation -- PhysicsBodyGpu.rotation
// stores the FORWARD rotation, matching contacts.rs's own
// BodySnapshot.rotation convention: p_world = rotation * local + translation).
fn rotate_by_quat(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    return q.xyz * 2.0 * dot(q.xyz, v) + v * (q.w * q.w - dot(q.xyz, q.xyz)) + cross(q.xyz, v) * 2.0 * q.w;
}

fn quat_inverse(q: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(-q.xyz, q.w);
}

fn world_to_local(p_world: vec3<f32>, translation: vec3<f32>, rotation: vec4<f32>) -> vec3<f32> {
    return rotate_by_quat(quat_inverse(rotation), p_world - translation);
}

// contacts.rs::query_body_point, ported verbatim: distance + normal (in
// world space) of p_world against `shape`'s own field.
struct QueryResult {
    distance: f32,
    normal_world: vec3<f32>,
}

fn query_body_point(shape: PhysicsShapeGpu, translation: vec3<f32>, rotation: vec4<f32>, p_world: vec3<f32>) -> QueryResult {
    let p_local = world_to_local(p_world, translation, rotation);
    let distance = local_distance(shape, p_local);
    let normal_local = local_normal(shape, p_local);
    var result: QueryResult;
    result.distance = distance;
    result.normal_world = rotate_by_quat(rotation, normal_local);
    return result;
}

// Appends a contact to contacts_out via the atomic cursor, silently
// dropping it if the cursor has already exceeded capacity -- the CPU
// reads the final cursor value back every frame and is responsible for
// the loud over-capacity warning (see this file's own header comment),
// this shader has no output channel for that beyond the cursor value
// itself.
fn push_contact(body_a: u32, body_b: u32, point_world: vec3<f32>, normal_world: vec3<f32>, depth: f32) {
    let slot = atomicAdd(&contact_cursor[0], 1u);
    if slot >= params.contact_capacity {
        return;
    }
    var c: ContactGpu;
    c.body_a = body_a;
    c.body_b = body_b;
    c.depth = depth;
    c._pad0 = 0.0;
    c.point_world = vec4<f32>(point_world, 0.0);
    c.normal_world = vec4<f32>(normal_world, 0.0);
    contacts_out[slot] = c;
}

// contacts.rs::generate_contacts, ported verbatim -- both directions
// (a's samples vs b's field, b's samples vs a's field), same
// own_local_distance defensive-depth-correction term, same normal-flip
// convention on the b-vs-a direction.
fn generate_contacts_between(body_a: u32, body_b: u32) {
    let shape_a = shapes[body_a];
    let shape_b = shapes[body_b];
    let translation_a = bodies[body_a].position.xyz;
    let rotation_a = bodies[body_a].rotation;
    let translation_b = bodies[body_b].position.xyz;
    let rotation_b = bodies[body_b].rotation;

    let samples_a = sample_points[body_a];
    for (var i: u32 = 0u; i < samples_a.count; i = i + 1u) {
        let local_point = samples_a.points[i].xyz;
        let p_world = rotate_by_quat(rotation_a, local_point) + translation_a;
        let q = query_body_point(shape_b, translation_b, rotation_b, p_world);
        let separation = q.distance + local_distance(shape_a, local_point);
        if separation < 0.0 {
            push_contact(body_a, body_b, p_world, q.normal_world, -separation);
        }
    }

    let samples_b = sample_points[body_b];
    for (var i: u32 = 0u; i < samples_b.count; i = i + 1u) {
        let local_point = samples_b.points[i].xyz;
        let p_world = rotate_by_quat(rotation_b, local_point) + translation_b;
        let q = query_body_point(shape_a, translation_a, rotation_a, p_world);
        let separation = q.distance + local_distance(shape_b, local_point);
        if separation < 0.0 {
            push_contact(body_a, body_b, p_world, -q.normal_world, -separation);
        }
    }
}

fn cell_coord(point: vec3<f32>, cell_size: f32) -> vec3<i32> {
    return vec3<i32>(floor(point / cell_size));
}

fn cell_hash(cell: vec3<i32>, table_size: u32) -> u32 {
    let h = (cell.x * 92837111i) ^ (cell.y * 689287499i) ^ (cell.z * 283923481i);
    return u32(h) % table_size;
}

// solve_world.rs::generate_all_contacts's dynamic-vs-dynamic path: one
// invocation per dynamic body `i`, walking its own 27-cell neighborhood
// via the broad-phase's own CSR bucket_start/bucket_items (mirrors
// broadphase.rs::query_candidates exactly, including the `j > i` dedup
// generate_all_contacts's own caller loop applies -- each unordered pair
// is generated exactly once, from the lower index's own invocation, not
// duplicated by BOTH bodies' own neighbor queries).
@compute @workgroup_size(64, 1, 1)
fn physics_contacts_dynamic_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.dynamic_count {
        return;
    }

    let position_i = bodies[i].position.xyz;
    let cell = cell_coord(position_i, params.cell_size);

    for (var dx: i32 = -1; dx <= 1; dx = dx + 1) {
        for (var dy: i32 = -1; dy <= 1; dy = dy + 1) {
            for (var dz: i32 = -1; dz <= 1; dz = dz + 1) {
                let neighbor = cell + vec3<i32>(dx, dy, dz);
                let h = cell_hash(neighbor, params.table_size);
                let range_start = bucket_start[h];
                let range_end = bucket_start[h + 1u];
                for (var slot: u32 = range_start; slot < range_end; slot = slot + 1u) {
                    let j = bucket_items[slot];
                    if j <= i {
                        continue;
                    }
                    generate_contacts_between(i, j);
                }
            }
        }
    }
}

// solve_world.rs::generate_all_contacts's dynamic-vs-kinematic path: a
// direct dynamic_count * kinematic_count flat grid, no spatial hash --
// mirrors the dynamic-vs-static pass below exactly, just over the
// kinematic sub-range (dynamic_count..dynamic_count+kinematic_count)
// instead of the static one. One invocation per (dynamic_index,
// kinematic_index) pair via a flattened 1D dispatch.
@compute @workgroup_size(64, 1, 1)
fn physics_contacts_kinematic_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let pair_index = gid.x;
    let total_pairs = params.dynamic_count * params.kinematic_count;
    if pair_index >= total_pairs {
        return;
    }
    let i = pair_index / params.kinematic_count;
    let kinematic_local_index = pair_index % params.kinematic_count;
    let j = params.dynamic_count + kinematic_local_index;
    generate_contacts_between(i, j);
}

// solve_world.rs::generate_all_contacts's dynamic-vs-static path: a
// direct dynamic_count * static_count flat grid, no spatial hash -- same
// design that module's own doc comment settles on (folding a large static
// collider into a hash sized for small dynamic bodies would be wrong, and
// profiling already confirmed statics were never the O(n^2) bottleneck).
// One invocation per (dynamic_index, static_index) pair via a flattened
// 1D dispatch, matching every other 1D-shaped pass in this port. Static
// bodies start at dynamic_count + kinematic_count, not just dynamic_count
// -- kinematics sit between dynamics and statics in the shared bodies
// buffer, per solve_world.rs's own three-range indexing convention.
@compute @workgroup_size(64, 1, 1)
fn physics_contacts_static_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let pair_index = gid.x;
    let total_pairs = params.dynamic_count * params.static_count;
    if pair_index >= total_pairs {
        return;
    }
    let i = pair_index / params.static_count;
    let static_local_index = pair_index % params.static_count;
    let j = params.dynamic_count + params.kinematic_count + static_local_index;
    generate_contacts_between(i, j);
}
