// Direct GPU sphere-traced raymarcher (see crate::raymarch's module doc). Evaluates
// the CSG scene live, every pixel, every frame — no baking step, with real
// multi-light shadowing/AO computed per-pixel.
//
// One full-screen triangle per view (bevy_core_pipeline::FullscreenShader's vertex
// state, reused as-is — see crate::raymarch::pipeline's module doc), no vertex buffer.
// `map()` walks a flattened, post-order (RPN) primitive-record list with a small fixed
// stack — the non-recursive form of sdf::scene::Node::distance's own recursion (WGSL
// has no recursion) — see crate::raymarch::flatten's module doc for the full design
// rationale.

#import bevy_render::view::{View, frag_coord_to_uv, uv_to_ndc}
#import migera::material::{Material, material_new, material_f0, fresnel_schlick}
#import migera::pattern_dispatch::dispatch_pattern

@group(0) @binding(0) var<uniform> view: View;

const MAX_ANIM_GROUPS: u32 = 8u;

struct RaymarchScene {
    tile_period: f32,
    static_count: u32,
    anim_group_count: u32,
    light_count: u32,
    time_secs: f32,
    debug_flags: u32,
    anim_group_ranges: array<vec4<u32>, MAX_ANIM_GROUPS>, // (start, count, _, _), index == group id
};

// Mirrors crate::raymarch::flatten::PrimitiveRecordCpu field-for-field. Material
// fields are meaningless on TAG_OP_* (operator) records — only ever read from leaf
// records, per resolve_leaf_material/dispatch_material below.
struct PrimitiveRecord {
    tag: u32,
    anim_group: u32,
    param_a: f32,
    param_b: f32,
    param_c: f32,
    param_d: f32,
    param_e: f32,
    param_f: f32,
    param_g: f32,
    param_h: f32,
    translation_x: f32,
    translation_y: f32,
    translation_z: f32,
    rotation_is_identity: u32,
    rotation_x: f32,
    rotation_y: f32,
    rotation_z: f32,
    rotation_w: f32,
    base_color_r: f32,
    base_color_g: f32,
    base_color_b: f32,
    metallic: f32,
    roughness: f32,
    pattern_id: u32,
    material_b_r: f32,
    material_b_g: f32,
    material_b_b: f32,
    metallic_b: f32,
    roughness_b: f32,
    pattern_params_x: f32,
    pattern_params_y: f32,
    pattern_params_z: f32,
    pattern_params_w: f32,
    center_x: f32,
    center_y: f32,
    center_z: f32,
    bounding_radius: f32,
};

// Mirrors crate::raymarch::pipeline::AnimIsometryGpu field-for-field.
struct AnimIsometry {
    pivot_x: f32,
    pivot_y: f32,
    pivot_z: f32,
    bounding_radius: f32,
    rotation_x: f32,
    rotation_y: f32,
    rotation_z: f32,
    rotation_w: f32,
};

const LIGHT_KIND_DIRECTIONAL: u32 = 0u;
const LIGHT_KIND_POINT: u32 = 1u;
const LIGHT_KIND_SPOT: u32 = 2u;

// Mirrors crate::raymarch::pipeline::LightGpu field-for-field.
struct Light {
    kind: u32,
    color_r: f32,
    color_g: f32,
    color_b: f32,
    direction_or_position_x: f32,
    direction_or_position_y: f32,
    direction_or_position_z: f32,
    intensity: f32,
    spot_direction_x: f32,
    spot_direction_y: f32,
    spot_direction_z: f32,
    range: f32,
    inner_angle: f32,
    outer_angle: f32,
    _pad0: f32,
    _pad1: f32,
};

@group(1) @binding(0) var<uniform> scene: RaymarchScene;
@group(1) @binding(1) var<storage, read> primitives: array<PrimitiveRecord>;
@group(1) @binding(2) var<storage, read> anim_isometries: array<AnimIsometry>;
@group(1) @binding(3) var<storage, read> lights: array<Light>;

const TAG_LEAF_SPHERE: u32 = 0u;
const TAG_LEAF_ROUNDED_BOX: u32 = 1u;
const TAG_LEAF_ROUNDED_CYLINDER: u32 = 2u;
const TAG_LEAF_CAPSULE: u32 = 3u;
const TAG_LEAF_ROUNDED_CONE: u32 = 4u;
const TAG_LEAF_ELLIPSOID: u32 = 5u;
const TAG_LEAF_BOX_FRAME: u32 = 6u;
const TAG_LEAF_HEX_PRISM: u32 = 7u;
const LEAF_TAG_COUNT: u32 = 8u;
const TAG_OP_UNION: u32 = 8u;
const TAG_OP_SMOOTH_UNION: u32 = 9u;
const TAG_OP_SUBTRACT: u32 = 10u;
const TAG_OP_SMOOTH_SUBTRACT: u32 = 11u;
// Stage-1 op-set completion — keep in sync with crate::raymarch::flatten.
const TAG_OP_INTERSECT: u32 = 12u;
const TAG_OP_SMOOTH_INTERSECT: u32 = 13u;

// Debug-flag bit masks (read from scene.debug_flags uniform).
const DBG_DISABLE_SHADOWS: u32 = 1u;
const DBG_DISABLE_AO: u32 = 2u;
const DBG_DISABLE_REFLECTION: u32 = 4u;
const DBG_HEATMAP: u32 = 8u;
const DBG_DISABLE_BOUNDING_SPHERE: u32 = 16u;

// ---------------------------------------------------------------------------------
// Primitive distance formulas — direct ports of src/sdf/primitives.rs's closed forms.
// These distance-only functions are retained for eval_leaf (raymarching steps only
// need distance, not gradient). The gradient-aware sdg_* variants below are used by
// calc_normal for single-leaf analytical normals.
// ---------------------------------------------------------------------------------

fn sdf_sphere(p: vec3<f32>, radius: f32) -> f32 {
    return length(p) - radius;
}

// primitives.rs:57-61 (RoundedBox3).
fn sdf_rounded_box(p: vec3<f32>, half_extents: vec3<f32>, corner_radius: f32) -> f32 {
    let q = abs(p) - half_extents + vec3<f32>(corner_radius);
    return length(max(q, vec3<f32>(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0) - corner_radius;
}

// primitives.rs:138-145 (RoundedCylinder).
fn sdf_rounded_cylinder(p: vec3<f32>, radius: f32, half_height: f32, edge_radius: f32) -> f32 {
    let dx = length(vec2<f32>(p.x, p.z)) - radius + edge_radius;
    let dy = abs(p.y) - half_height + edge_radius;
    let d = vec2<f32>(dx, dy);
    return min(max(d.x, d.y), 0.0) + length(max(d, vec2<f32>(0.0))) - edge_radius;
}

// primitives.rs Capsule — sphere swept along a line segment.
fn sdf_capsule(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, radius: f32) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let t = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0);
    let closest = a + ab * t;
    return length(p - closest) - radius;
}

// primitives.rs RoundedCone — cone with spherical caps at both ends.
fn sdf_rounded_cone(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, r0: f32, r1: f32) -> f32 {
    let ba = b - a;
    let pa = p - a;
    let m0 = dot(ba, ba);
    let m1 = dot(ba, pa);
    let m2 = dot(pa, pa);

    let d = m0 - m1;
    let e = m1 - m2;
    let f = m2 + m0 * m0 - 2.0 * m1;
    let g = d * d * m0;
    let h = m0 * (m0 - d);
    let clamped = max(d * e * m0 - f * h, 0.0);
    let t = m0 * (f * d - e * clamped) / (g + h * clamped) - m1;

    let t_clamped = clamp(t, 0.0, m0);
    let q = length(a + ba * t_clamped / m0 - p) - r0 - (r1 - r0) * t_clamped / m0;
    return max(max(q, length(pa) - r0), length(p - b) - r1);
}

// primitives.rs Ellipsoid — three independent radii.
fn sdf_ellipsoid(p: vec3<f32>, radii: vec3<f32>) -> f32 {
    let k0 = length(p / radii);
    let k1 = length(p / (radii * radii));
    return k0 * (k0 - 1.0) / k1;
}

// primitives.rs BoxFrame — hollow box with uniform wall thickness.
fn sdf_box_frame(p: vec3<f32>, half_extents: vec3<f32>, wall_thickness: f32) -> f32 {
    let q = abs(p) - half_extents;
    let outer = length(max(q, vec3<f32>(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0);
    let inner_q = q + vec3<f32>(wall_thickness);
    let inner = length(max(inner_q, vec3<f32>(0.0))) + min(max(inner_q.x, max(inner_q.y, inner_q.z)), 0.0);
    return max(outer, -inner);
}

// primitives.rs HexPrism — hexagonal prism cross-section in XZ, extruded along Y.
fn sdf_hex_prism(p: vec3<f32>, radius: f32, half_height: f32) -> f32 {
    let q = abs(p);
    let k = 0.866025404;
    let hex_d = max(max(q.x, abs(0.5 * q.x + k * q.z)), abs(0.5 * q.x - k * q.z)) - radius;
    let axial_d = q.y - half_height;
    return max(hex_d, axial_d);
}

// ---------------------------------------------------------------------------------
// Analytical SDF + gradient results. Each sdg_* function returns both distance and
// the unit-length gradient (surface normal) in a single evaluation, used by
// calc_normal to compute an analytical normal from a single sdg_dispatch call.
// Reference: https://iquilezles.org/articles/distgradfunctions3d/
// ---------------------------------------------------------------------------------

struct SdfResult {
    distance: f32,
    grad: vec3<f32>,
};

fn safe_normalize(v: vec3<f32>) -> vec3<f32> {
    let l = length(v);
    return select(v / l, vec3<f32>(0.0, 1.0, 0.0), l < 1e-10);
}

fn sdg_sphere(p: vec3<f32>, r: f32) -> SdfResult {
    let l = length(p);
    return SdfResult(l - r, safe_normalize(p));
}

fn sdg_rounded_box(p: vec3<f32>, b: vec3<f32>, r: f32) -> SdfResult {
    let q = abs(p) - b + vec3<f32>(r);
    let w = max(q, vec3<f32>(0.0));
    let g = max(q.x, max(q.y, q.z));
    let l = length(w);
    if (g > 0.0) {
        return SdfResult(l - r, safe_normalize(w) * sign(p));
    }
    var n = vec3<f32>(0.0);
    if (q.x >= q.y && q.x >= q.z) { n = vec3<f32>(1.0, 0.0, 0.0); }
    else if (q.y >= q.z) { n = vec3<f32>(0.0, 1.0, 0.0); }
    else { n = vec3<f32>(0.0, 0.0, 1.0); }
    return SdfResult(g - r, n * sign(p));
}

fn sdg_rounded_cylinder(p: vec3<f32>, radius: f32, half_height: f32, edge_radius: f32) -> SdfResult {
    let dx = length(vec2<f32>(p.x, p.z)) - radius + edge_radius;
    let dy = abs(p.y) - half_height + edge_radius;
    let d = vec2<f32>(dx, dy);
    let outside = max(d, vec2<f32>(0.0));
    let inside = min(max(d.x, d.y), 0.0);
    let l = length(outside);
    let g = max(d.x, d.y);
    if (g > 0.0) {
        let rl = max(length(vec2<f32>(p.x, p.z)), 1e-10);
        let radial = vec2<f32>(p.x, p.z) / rl;
        return SdfResult(l - edge_radius, safe_normalize(vec3<f32>(radial.x * outside.x, outside.y, radial.y * outside.x)));
    }
    var n: vec3<f32>;
    if (d.x > d.y) {
        let rl = max(length(vec2<f32>(p.x, p.z)), 1e-10);
        n = vec3<f32>(p.x / rl, 0.0, p.z / rl);
    } else {
        n = vec3<f32>(0.0, select(-1.0, 1.0, p.y > 0.0), 0.0);
    }
    return SdfResult(inside - edge_radius, n);
}

fn sdg_capsule(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, r: f32) -> SdfResult {
    let ba = b - a;
    let pa = p - a;
    let t = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    let q = pa - ba * t;
    let l = length(q);
    return SdfResult(l - r, safe_normalize(q));
}

// IQ's sdgRoundCone — 3-branch analytical gradient.
// https://iquilezles.org/articles/distgradfunctions3d/
fn sdg_rounded_cone(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, r1: f32, r2: f32) -> SdfResult {
    let ba = b - a;
    let l2 = dot(ba, ba);
    let rr = r1 - r2;
    let a2 = l2 - rr * rr;
    let il2 = 1.0 / l2;
    let pa = p - a;
    let pb = p - b;
    let y = dot(pa, ba);
    let z = y - l2;
    let x2 = l2 * dot(pa, pa) - y * y;
    let k = sign(rr) * rr * rr * x2;
    if (sign(z) * a2 * z * z > k) {
        let w = sqrt(il2 * (x2 + z * z));
        return SdfResult(w - r2, safe_normalize(pb / w));
    }
    if (sign(y) * a2 * y * y < k) {
        let w = sqrt(il2 * (x2 + y * y));
        return SdfResult(w - r1, safe_normalize(pa / w));
    }
    let w = sqrt(x2 * a2);
    return SdfResult((w + y * rr) * il2 - r1, safe_normalize(il2 * (rr * ba + a2 * (pa * l2 - y * ba) / w)));
}

fn sdg_ellipsoid(p: vec3<f32>, r: vec3<f32>) -> SdfResult {
    let k0 = length(p / r);
    let k1 = length(p / (r * r));
    return SdfResult(k0 * (k0 - 1.0) / k1, safe_normalize(p / (r * r * k1)));
}

fn sdg_box_frame(p: vec3<f32>, b: vec3<f32>, e: f32) -> SdfResult {
    let q = abs(p) - b;
    let oq = max(q, vec3<f32>(0.0));
    let og = max(q.x, max(q.y, q.z));
    let outer = length(oq) + min(og, 0.0);
    let iq = q + vec3<f32>(e);
    let iiq = max(iq, vec3<f32>(0.0));
    let ig = max(iq.x, max(iq.y, iq.z));
    let inner = length(iiq) + min(ig, 0.0);
    let dist = max(outer, -inner);
    if (outer > -inner) {
        var n: vec3<f32>;
        if (og > 0.0) { n = safe_normalize(oq) * sign(p); }
        else if (q.x >= q.y && q.x >= q.z) { n = vec3<f32>(1.0, 0.0, 0.0) * sign(p); }
        else if (q.y >= q.z) { n = vec3<f32>(0.0, 1.0, 0.0) * sign(p); }
        else { n = vec3<f32>(0.0, 0.0, 1.0) * sign(p); }
        return SdfResult(dist, n);
    }
    var n: vec3<f32>;
    if (ig > 0.0) { n = safe_normalize(iiq) * sign(p); }
    else if (iq.x >= iq.y && iq.x >= iq.z) { n = vec3<f32>(1.0, 0.0, 0.0) * sign(p); }
    else if (iq.y >= iq.z) { n = vec3<f32>(0.0, 1.0, 0.0) * sign(p); }
    else { n = vec3<f32>(0.0, 0.0, 1.0) * sign(p); }
    return SdfResult(dist, -n);
}

fn sdg_hex_prism(p: vec3<f32>, radius: f32, half_height: f32) -> SdfResult {
    let q = abs(p);
    let k = 0.866025404;
    let hx = 0.5 * q.x;
    let d1 = q.x;
    let d2 = abs(hx + k * q.z);
    let d3 = abs(hx - k * q.z);
    let hex_d = max(max(d1, d2), d3) - radius;
    let axial_d = q.y - half_height;
    if (hex_d > axial_d) {
        var g2 = vec2<f32>(1.0, 0.0);
        if (d2 > d1 && d2 > d3) { g2 = vec2<f32>(0.5, k) * sign(hx + k * q.z); }
        else if (d3 > d1) { g2 = vec2<f32>(0.5, -k) * sign(hx - k * q.z); }
        else { g2 = vec2<f32>(1.0, 0.0) * sign(q.x); }
        return SdfResult(hex_d, safe_normalize(vec3<f32>(g2.x, 0.0, g2.y)));
    }
    return SdfResult(axial_d, vec3<f32>(0.0, select(-1.0, 1.0, p.y > 0.0), 0.0));
}

fn sdg_dispatch(record: PrimitiveRecord, local: vec3<f32>) -> SdfResult {
    switch (record.tag) {
        case TAG_LEAF_SPHERE: { return sdg_sphere(local, record.param_a); }
        case TAG_LEAF_ROUNDED_BOX: { return sdg_rounded_box(local, vec3<f32>(record.param_a, record.param_b, record.param_c), record.param_d); }
        case TAG_LEAF_ROUNDED_CYLINDER: { return sdg_rounded_cylinder(local, record.param_a, record.param_b, record.param_c); }
        case TAG_LEAF_CAPSULE: {
            return sdg_capsule(local, vec3<f32>(record.param_a, record.param_b, record.param_c), vec3<f32>(record.param_d, record.param_e, record.param_f), record.param_g);
        }
        case TAG_LEAF_ROUNDED_CONE: {
            return sdg_rounded_cone(local, vec3<f32>(record.param_a, record.param_b, record.param_c), vec3<f32>(record.param_d, record.param_e, record.param_f), record.param_g, record.param_h);
        }
        case TAG_LEAF_ELLIPSOID: { return sdg_ellipsoid(local, vec3<f32>(record.param_a, record.param_b, record.param_c)); }
        case TAG_LEAF_BOX_FRAME: { return sdg_box_frame(local, vec3<f32>(record.param_a, record.param_b, record.param_c), record.param_d); }
        case TAG_LEAF_HEX_PRISM: { return sdg_hex_prism(local, record.param_a, record.param_b); }
        default: { return SdfResult(1e6, vec3<f32>(0.0, 1.0, 0.0)); }
    }
}

// ---------------------------------------------------------------------------------
// CSG combine operators — direct ports of sdf/scene.rs's smin/smax.
// ---------------------------------------------------------------------------------

fn smin(a: f32, b: f32, k: f32) -> f32 {
    if (k <= 0.0) {
        return min(a, b);
    }
    let h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
    return mix(b, a, h) - k * h * (1.0 - h);
}

fn smax(a: f32, b: f32, k: f32) -> f32 {
    return -smin(-a, -b, k);
}

// Direct port of sdf/scene.rs's repeat_xz — WGSL has no rem_euclid, so this uses two
// floored modulos to get the same "always non-negative" behavior for negative inputs
// Rust's rem_euclid provides and plain WGSL `%` (truncating, like Rust's own `%`)
// doesn't.
fn floored_mod(x: f32, m: f32) -> f32 {
    return x - m * floor(x / m);
}

fn repeat_xz(p: vec3<f32>, period: f32) -> vec3<f32> {
    let half = period * 0.5;
    return vec3<f32>(
        floored_mod(p.x + half, period) - half,
        p.y,
        floored_mod(p.z + half, period) - half,
    );
}

// quat * vector rotation (Hamilton product form), matching glam::Quat's convention.
fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let qv = q.xyz;
    let t = 2.0 * cross(qv, v);
    return v + q.w * t + cross(qv, t);
}

fn quat_inverse(q: vec4<f32>) -> vec4<f32> {
    // Unit quaternion: inverse == conjugate.
    return vec4<f32>(-q.xyz, q.w);
}

// ---------------------------------------------------------------------------------
// eval_leaf/eval_stack — the non-recursive walk of one [start, start+count) sub-range
// of `primitives`, mirroring Node::distance's recursion exactly (see
// crate::raymarch::flatten's module doc): a post-order (RPN) list, evaluated with an
// explicit value stack instead of returning up a Rust call stack.
// ---------------------------------------------------------------------------------

fn eval_leaf(record: PrimitiveRecord, world_p: vec3<f32>) -> f32 {
    var p = world_p;

    // AnimGroup live rotation (see crate::raymarch::extract's doc comment / this
    // shader's module doc): un-rotate/un-translate by the group's current live pivot +
    // rotation BEFORE applying the leaf's own rest-pose-local isometry below — the
    // same two-step "outer pivot transform, then this leaf's own transform" composition
    // Isometry::inverse_transform_point already does for a single transform, just with
    // one extra outer layer for animated leaves.
    if (record.anim_group != 0u) {
        let iso = anim_isometries[record.anim_group - 1u];
        let pivot = vec3<f32>(iso.pivot_x, iso.pivot_y, iso.pivot_z);
        let rotation = vec4<f32>(iso.rotation_x, iso.rotation_y, iso.rotation_z, iso.rotation_w);
        p = quat_rotate(quat_inverse(rotation), p - pivot);
    }

    let translation = vec3<f32>(record.translation_x, record.translation_y, record.translation_z);
    var local = p - translation;
    if (record.rotation_is_identity == 0u) {
        let rotation = vec4<f32>(record.rotation_x, record.rotation_y, record.rotation_z, record.rotation_w);
        local = quat_rotate(quat_inverse(rotation), local);
    }

    switch (record.tag) {
        case TAG_LEAF_SPHERE: {
            return sdf_sphere(local, record.param_a);
        }
        case TAG_LEAF_ROUNDED_BOX: {
            return sdf_rounded_box(local, vec3<f32>(record.param_a, record.param_b, record.param_c), record.param_d);
        }
        case TAG_LEAF_ROUNDED_CYLINDER: {
            return sdf_rounded_cylinder(local, record.param_a, record.param_b, record.param_c);
        }
        case TAG_LEAF_CAPSULE: {
            let a = vec3<f32>(record.param_a, record.param_b, record.param_c);
            let b = vec3<f32>(record.param_d, record.param_e, record.param_f);
            return sdf_capsule(local, a, b, record.param_g);
        }
        case TAG_LEAF_ROUNDED_CONE: {
            let a = vec3<f32>(record.param_a, record.param_b, record.param_c);
            let b = vec3<f32>(record.param_d, record.param_e, record.param_f);
            return sdf_rounded_cone(local, a, b, record.param_g, record.param_h);
        }
        case TAG_LEAF_ELLIPSOID: {
            return sdf_ellipsoid(local, vec3<f32>(record.param_a, record.param_b, record.param_c));
        }
        case TAG_LEAF_BOX_FRAME: {
            return sdf_box_frame(local, vec3<f32>(record.param_a, record.param_b, record.param_c), record.param_d);
        }
        case TAG_LEAF_HEX_PRISM: {
            return sdf_hex_prism(local, record.param_a, record.param_b);
        }
        default: {
            return 1e6;
        }
    }
}

// Fixed-size value stack: generous headroom over this demo's actual tree depth (ground
// + 4 blob spheres + 3 rings + pillar = 9 leaves combined pairwise, tree depth ~4) —
// see crate::raymarch::flatten's module doc.
const STACK_CAPACITY: u32 = 32u;

fn eval_stack(world_p: vec3<f32>, start: u32, count: u32) -> f32 {
    var stack: array<f32, STACK_CAPACITY>;
    var sp: i32 = -1;
    for (var i = start; i < start + count; i = i + 1u) {
        let record = primitives[i];
        if (record.tag < LEAF_TAG_COUNT) {
            sp = sp + 1;
            stack[sp] = eval_leaf(record, world_p);
        } else {
            let b = stack[sp];
            sp = sp - 1;
            let a = stack[sp];
            sp = sp - 1;
            var result: f32;
            switch (record.tag) {
                case TAG_OP_UNION: {
                    result = min(a, b);
                }
                case TAG_OP_SMOOTH_UNION: {
                    result = smin(a, b, record.param_a);
                }
                case TAG_OP_INTERSECT: {
                    result = max(a, b);
                }
                case TAG_OP_SMOOTH_INTERSECT: {
                    result = smax(a, b, record.param_a);
                }
                case TAG_OP_SUBTRACT: {
                    result = max(a, -b);
                }
                case TAG_OP_SMOOTH_SUBTRACT: {
                    result = smax(a, -b, record.param_a);
                }
                default: {
                    result = a;
                }
            }
            sp = sp + 1;
            stack[sp] = result;
        }
    }
    return stack[sp];
}

// The scene's overall distance function — static (tiled) geometry unioned with every
// live AnimGroup's own (un-tiled, world-space) subtree. See crate::raymarch::flatten's
// module doc on why Repeat is a single scalar applied once here, not a per-primitive
// property: O(1) regardless of camera distance, matching Node::Repeat's own guarantee.
fn map(p: vec3<f32>) -> f32 {
    let p_static = repeat_xz(p, scene.tile_period);
    var d = eval_stack(p_static, 0u, scene.static_count);

    for (var g = 0u; g < scene.anim_group_count; g = g + 1u) {
        let range = scene.anim_group_ranges[g];
        if (range.y == 0u) {
            continue;
        }
        // Bounding-sphere early-out: skip this group's eval_stack call entirely for
        // query points far outside it. (Contributes nothing to the min rather than
        // a distance-to-bound: a lower bound would make rays converge onto the
        // bounding sphere itself — phantom shells; see flatten.rs bounding fields.)
        let iso = anim_isometries[g];
        let pivot = vec3<f32>(iso.pivot_x, iso.pivot_y, iso.pivot_z);
        if (distance(p, pivot) > iso.bounding_radius) {
            continue;
        }
        let d_group = eval_stack(p, range.x, range.y);
        d = min(d, d_group);
    }

    return d;
}

// The winning leaf from nearest_leaf_index: its record index, which contiguous
// [range_start, range_start+range_count) range it belongs to (the static range, or
// one particular AnimGroup's range — needed so resolve_material/blended_material_color
// scan the SAME range the winner was found in, not always the static one), and the
// coordinate space that range is evaluated in (tiled p_static for the static range,
// raw untiled p for an AnimGroup range — matching map()'s own per-range coordinate
// choice, see that function).
struct NearestLeaf {
    index: u32,
    range_start: u32,
    range_count: u32,
    eval_p: vec3<f32>,
};

// Which leaf record is nearest at a converged hit point — used once per ray hit (not
// per march step, unlike map() above) to resolve that hit's material. Deliberately
// skips eval_stack's actual CSG-combined distance (which map() computes) and only
// scans raw per-leaf distances via nearest_leaf: the true smooth-union/subtract-
// blended distance isn't needed to answer "which primitive is this", just the nearest
// individual leaf, which is half the work (one leaf-only scan per range instead of an
// eval_stack pass AND a leaf scan) — worth it since this runs on every primary hit.
// Returns the leaf's index into `primitives` (not just its tag) so the caller can read
// that leaf's own material_new/pattern fields directly off the record, rather than a
// second Rust/WGSL-side table keyed by tag — the record IS the material's source of
// truth now, matching sdf::components::Material/ProceduralPattern being authored
// per-entity rather than per-shape-kind.
fn nearest_leaf_index(p: vec3<f32>) -> NearestLeaf {
    let p_static = repeat_xz(p, scene.tile_period);
    var best_d = 1e6;
    var best_index = 0u;
    var best_range_start = 0u;
    var best_range_count = scene.static_count;
    var best_eval_p = p_static;
    nearest_leaf_in_range(p_static, 0u, scene.static_count, &best_d, &best_index);

    for (var g = 0u; g < scene.anim_group_count; g = g + 1u) {
        let range = scene.anim_group_ranges[g];
        if (range.y == 0u) {
            continue;
        }
        let iso = anim_isometries[g];
        let pivot = vec3<f32>(iso.pivot_x, iso.pivot_y, iso.pivot_z);
        if (distance(p, pivot) > iso.bounding_radius) {
            continue;
        }
        let d_before = best_d;
        nearest_leaf_in_range(p, range.x, range.y, &best_d, &best_index);
        if (best_d < d_before) {
            best_range_start = range.x;
            best_range_count = range.y;
            best_eval_p = p;
        }
    }

    return NearestLeaf(best_index, best_range_start, best_range_count, best_eval_p);
}

// Scans one [start, start+count) leaf/op range for the single leaf record whose own
// (untagged, single-primitive) distance is closest to `world_p`, updating `best_d`/
// `best_index` in place ONLY when this range's own nearest leaf beats the running
// best — so callers can chain multiple ranges (static + each anim group) against a
// shared running best across calls without a later, worse-matching range ever
// clobbering an earlier winner. A coarser approximation than eval_stack's actual
// CSG-combined distance (doesn't account for smooth-union/subtract blending), which
// is fine here: this only decides which primitive's material a converged surface hit
// belongs to, where the nearest individual leaf is already a very good proxy for
// which shape the hit point visually belongs to. Callers must seed `*best_d` to 1e6
// before the first call.
fn nearest_leaf_in_range(world_p: vec3<f32>, start: u32, count: u32, best_d: ptr<function, f32>, best_index: ptr<function, u32>) {
    for (var i = start; i < start + count; i = i + 1u) {
        let record = primitives[i];
        if (record.tag >= LEAF_TAG_COUNT) {
            continue;
        }
        let d = abs(eval_leaf(record, world_p));
        if (d < *best_d) {
            *best_d = d;
            *best_index = i;
        }
    }
}

fn record_material_a(record: PrimitiveRecord) -> Material {
    return material_new(vec3<f32>(record.base_color_r, record.base_color_g, record.base_color_b), record.metallic, record.roughness);
}

fn record_material_b(record: PrimitiveRecord) -> Material {
    return material_new(vec3<f32>(record.material_b_r, record.material_b_g, record.material_b_b), record.metallic_b, record.roughness_b);
}

// ---------------------------------------------------------------------------------
// Per-primitive materials, blended at smooth-union seams by reusing the SAME smin
// blend weight the geometry itself was combined with (see docs/knowledge/sdf-3d/
// materials-and-texturing/material-blending.md's "reusing smin's own blend weight"
// technique) — not a nearest-leaf hard switch, which would show a visible hard
// material seam right where the geometry is smoothly blended. Generalized over
// whichever leaf range the winning leaf (from nearest_leaf_index) belongs to, rather
// than hardcoded to a fixed "the blob cluster is leaves 1..4" range: scans every leaf
// in the same static/anim-group range the winner came from, so any smooth-unioned
// cluster in the scene gets this treatment automatically, not just one hardcoded one.
// ---------------------------------------------------------------------------------

// Blend radius used to color-blend leaves at their smooth-union seams — matches
// assembly::DEFAULT_BLEND (the join radius sdf::world's clusters actually union
// with) rather than trying to read it back out of the flattened op records (the CSG
// op list's blend radius belongs to the union operators between leaves, not to the
// leaves being blended themselves, so duplicating the constant here is simpler than
// threading it through).
const MATERIAL_BLEND_K: f32 = 0.4;

// Resolves a smoothly material-blended color for a point, scanning every leaf in
// [start, start+count) — the same range nearest_leaf_index found the winning leaf
// in — and folding their record_material_a().base_color pairwise through the same
// polynomial smin weight `h` used for geometry (see this file's own `smin`), applied
// to color instead of distance. This mirrors eval_stack's own smooth-union folding
// order exactly (left to right through the range) so the color blend seam sits
// exactly where the geometry blend seam does, rather than needing a second,
// independently-tuned blend radius.
fn blended_material_color(p: vec3<f32>, start: u32, count: u32) -> vec3<f32> {
    var color = record_material_a(primitives[start]).base_color;
    var d_acc = eval_leaf(primitives[start], p);
    for (var i = start + 1u; i < start + count; i = i + 1u) {
        let record = primitives[i];
        if (record.tag >= LEAF_TAG_COUNT) {
            continue;
        }
        let d_next = eval_leaf(record, p);
        let h = clamp(0.5 + 0.5 * (d_next - d_acc) / MATERIAL_BLEND_K, 0.0, 1.0);
        color = mix(record_material_a(record).base_color, color, h);
        d_acc = min(d_acc, d_next) - MATERIAL_BLEND_K * h * (1.0 - h);
    }
    return color;
}

// Analytical normal from the nearest leaf's own signed-distance gradient. Instead of
// re-evaluating the entire CSG tree with gradient propagation (eval_stack_grad), this
// finds the single leaf nearest_leaf_index already identified, transforms the world
// point to that leaf's local space, and calls sdg_dispatch once. The result is the
// gradient of the winning primitive only — not the composed CSG surface — but this is
// a very good approximation: at smooth-union seams the blend zone is narrow, and at
// hard union/subtract edges the leaf's gradient is exactly correct on the interior
// side. Trades full CSG-correct normals (expensive: duplicate pipeline + register
// pressure) for a single-leaf gradient (cheap: one sdg_dispatch call, no pipeline
// duplication).
fn calc_normal(nl: NearestLeaf) -> vec3<f32> {
    let record = primitives[nl.index];
    var p = nl.eval_p;

    if (record.anim_group != 0u) {
        let iso = anim_isometries[record.anim_group - 1u];
        let pivot = vec3<f32>(iso.pivot_x, iso.pivot_y, iso.pivot_z);
        let rotation = vec4<f32>(iso.rotation_x, iso.rotation_y, iso.rotation_z, iso.rotation_w);
        p = quat_rotate(quat_inverse(rotation), p - pivot);
    }

    let translation = vec3<f32>(record.translation_x, record.translation_y, record.translation_z);
    var local = p - translation;
    if (record.rotation_is_identity == 0u) {
        let rotation = vec4<f32>(record.rotation_x, record.rotation_y, record.rotation_z, record.rotation_w);
        local = quat_rotate(quat_inverse(rotation), local);
    }

    return safe_normalize(sdg_dispatch(record, local).grad);
}

// Sphere-traced soft shadow (IQ's penumbra trick: running minimum of k*h/t while
// marching toward the light). An earlier, lower step count (16) starved shadow rays
// near curved contact points (blob cluster, ring/pillar base), producing a
// broken-up/patchy shadow instead of one coherent soft blob — same step-starvation
// failure mode as MAX_STEPS_FAR's ground-plane bug above, just for shadow rays.
// Raised well past that, since per that same fix, the cap is a ceiling most rays
// don't reach.
const SHADOW_MAX_STEPS: u32 = 32u;
const SHADOW_MAX_DIST: f32 = 12.0;
const SHADOW_MIN_HIT: f32 = 1e-3;

// Contact-hardening: softness ramps from CONTACT_K (tight, sharp penumbra) up to the
// caller's own `k` (the scene's tuned SHADOW_SOFTNESS_K) as the ray travels from the
// shadow-casting surface out toward CONTACT_HARDEN_DIST — real soft shadows are sharp
// right at the contact point and blur out with distance from the occluder, which a
// single fixed `k` (the old behavior) can't reproduce. Reuses `t` (already tracked by
// the existing penumbra-trick loop below) as the distance proxy, so this is a couple
// of extra ALU ops per step, not a new pass.
const CONTACT_K: f32 = 48.0;
const CONTACT_HARDEN_DIST: f32 = 1.5;

fn soft_shadow(p: vec3<f32>, light_dir: vec3<f32>, k: f32) -> f32 {
    var t = SHADOW_MIN_HIT;
    var res = 1.0;
    for (var i = 0u; i < SHADOW_MAX_STEPS; i = i + 1u) {
        let h = map(p + light_dir * t);
        if (h < SHADOW_MIN_HIT) {
            return 0.0;
        }
        let harden_t = clamp(t / CONTACT_HARDEN_DIST, 0.0, 1.0);
        let effective_k = mix(CONTACT_K, k, harden_t);
        res = min(res, effective_k * h / t);
        // Deep-umbra early-out: res only ever decreases (min accumulation), so once
        // it's this close to black, the remaining budget can't meaningfully darken
        // the sample. Most shadow rays across the ground's large cast shadows land
        // here within a few steps instead of marching all SHADOW_MAX_STEPS.
        if (res < 0.02) {
            return res;
        }
        t = t + h;
        if (t >= SHADOW_MAX_DIST) {
            break;
        }
    }
    return clamp(res, 0.0, 1.0);
}

// Normal-offset ambient occlusion (march a few steps along the surface normal,
// accumulate how much closer the SDF is than the offset distance would suggest).
const AO_STEPS: u32 = 4u;
const AO_STEP_SIZE: f32 = 0.075;

// AO_CONTRAST steepens the occlusion falloff (pow on the [0,1] AO term) so cavities
// (blob-cluster crevices, ring/pillar contact points) read as more clearly darkened
// without changing how many steps are marched — same accumulated `occlusion` value,
// just reshaped, so this is free relative to the existing AO march.
const AO_CONTRAST: f32 = 1.6;

fn ambient_occlusion(p: vec3<f32>, normal: vec3<f32>) -> f32 {
    var occlusion = 0.0;
    var scale = 1.0;
    for (var i = 1u; i <= AO_STEPS; i = i + 1u) {
        let offset = AO_STEP_SIZE * f32(i);
        let h = map(p + normal * offset);
        occlusion = occlusion + max(offset - h, 0.0) * scale;
        scale = scale * 0.95;
    }
    let ao = clamp(1.0 - clamp(occlusion, 0.0, 1.0), 0.0, 1.0);
    return pow(ao, AO_CONTRAST);
}

// ---------------------------------------------------------------------------------
// Sphere tracing (plain, unrelaxed). An over-relaxed variant (Keinert et al.
// "Enhanced Sphere Tracing") was tried and reverted: this scene's blob cluster is a
// smooth-union (smin) of overlapping spheres, and smin's blend region is not a true
// distance field (it underestimates distance there), so a relaxed step's
// safe-step-sphere overlap check was unreliable near blend seams — visible as dense
// pitting/speckle across every curved surface (worst on the smooth-unioned blob
// cluster and the pillar's smooth-subtract bite) even after correcting the backtrack
// math. Plain sphere tracing (omega = 1) is unconditionally safe against any SDF,
// including smin/smax's approximate regions, at the cost of somewhat more steps to
// converge in open space — acceptable here since removing relaxation did not
// regress measured framerate (the backtrack retries it was meant to avoid were
// costing as much as the relaxation saved).
// ---------------------------------------------------------------------------------

struct RaymarchHit {
    did_hit: bool,
    position: vec3<f32>,
    steps: u32,
};

// Distance-based LOD: step-count budget shrinks with distance already marched (a cheap
// proxy for "distance from camera" with no separate depth pre-pass needed) — see
// crate::raymarch's module doc for why this, not per-tile primitive culling, is this
// demo's LOD axis (Node::Repeat's whole point is O(1) infinite tiling via a single
// remap, so there's no growing "primitive count with distance" to cull in the first
// place). Re-evaluated every iteration against the current marched distance `t` (not
// decided once up front from an unknown pre-trace distance estimate), so a ray that's
// still close after many steps keeps its full near-field budget, and one that's
// already traveled far gets an early cutoff.
// Tuned for real-time on integrated GPUs: each map() call itself evaluates the static
// scene plus every AnimGroup separately (this demo's 4 groups + 1 static = ~5
// sub-evaluations per map() call), so the primary-ray step budget multiplies directly
// into total per-pixel cost far more than a naive single-eval_stack raymarcher would
// suggest — measured ~6fps at 1280x720 on an AMD Radeon (RADV RENOIR) integrated GPU
// with an original MAX_STEPS_NEAR=128/SHADOW_MAX_STEPS=24 budget.
//
// These are ceilings, not fixed per-ray costs: the vast majority of rays terminate far
// below the cap (either converging on a surface in a handful of steps, or leaving
// MAX_MARCH_DIST as a sky miss), so raising the cap mainly affects the pathological
// slow-converging cases below, not average frame cost — confirmed empirically: going
// from 64/24 to 256/256 changed measured framerate by only a few fps (~33 either way)
// at 1280x720 on the same integrated GPU.
//
// A previous, much lower MAX_STEPS_FAR (24, with LOD_FAR_DISTANCE=40) starved exactly
// this case: a ground-plane ray at grazing incidence (shallow angle relative to the
// thin, half_extents.y=0.15 ground slab) needs many small steps to converge once
// close, because each step's safe-step-sphere radius shrinks to the slab's thin
// vertical extent long before the ray's horizontal position lines up with the slab —
// with too few steps available, the ray simply ran out of budget short of the
// epsilon threshold and was reported as a miss, showing as ground tiles vanishing
// (while taller, more convex objects like the blob spheres above them still
// converged fine in the same low step budget). Verified fixed by raising the cap;
// visible as sky-colored gaps under distant tile copies before this fix.
const MAX_STEPS_NEAR: u32 = 128u;
const MAX_STEPS_FAR: u32 = 64u;
const LOD_FAR_DISTANCE: f32 = 40.0;
const MAX_MARCH_DIST: f32 = 500.0;
const MAX_STEP_SIZE: f32 = 8.0;
// Base hit tolerance before distance scaling (eps = HIT_EPSILON * max(t,1)). 2e-3
// vs the original 1e-3: at typical view distances the difference is far below one
// pixel of silhouette offset, and it shortens every ray's final-approach phase —
// steps near a surface shrink geometrically, so doubling eps saves several of the
// most expensive (smallest) steps per ray across primary + reflection traces.
const HIT_EPSILON: f32 = 2e-3;

fn lod_max_steps(marched_distance: f32) -> u32 {
    let lod_t = clamp(marched_distance / LOD_FAR_DISTANCE, 0.0, 1.0);
    return u32(mix(f32(MAX_STEPS_NEAR), f32(MAX_STEPS_FAR), lod_t));
}

fn sphere_trace(ro: vec3<f32>, rd: vec3<f32>, max_dist: f32) -> RaymarchHit {
    var t = 0.0;

    for (var i = 0u; i < MAX_STEPS_NEAR; i = i + 1u) {
        if (i >= lod_max_steps(t)) {
            break;
        }

        let radius = abs(map(ro + rd * t));

        // Distance-scaled epsilon: both a numerical-precision safeguard at long march
        // distances and a soft LOD (coarser surface-placement tolerance far away is
        // imperceptible but lets the tracer terminate in fewer steps).
        let eps = HIT_EPSILON * max(t, 1.0);
        if (radius < eps) {
            return RaymarchHit(true, ro + rd * t, i + 1u);
        }

        // Clamp step size to prevent overshooting in open space with large SDF values
        t = t + min(radius, MAX_STEP_SIZE);
        if (t > max_dist) {
            break;
        }
    }
    return RaymarchHit(false, vec3<f32>(0.0), MAX_STEPS_NEAR);
}

// ---------------------------------------------------------------------------------
// Multi-light shading.
// ---------------------------------------------------------------------------------

// Brings Bevy's photometric units (directional illuminance in lux, point/spot
// intensity in lumens) into a visually reasonable [0,1]-ish output range for this
// unlit-style direct lighting model — not a real photometric-to-radiometric
// conversion (bevy_pbr's own clustered-forward lighting does that properly); a single
// tune-by-eye scale is the pragmatic choice for a raymarcher whose main point is
// demonstrating live shadows/AO/multi-light, not matching bevy_pbr's exposure model
// exactly.
const LIGHT_SCALE: f32 = 1.0 / 8000.0;
// k=16.0/bias=1e-3 — tuned against this scene, replacing an earlier
// independently-tuned k=12.0/bias=0.01 that showed patchier shadow contact points.
const SHADOW_SOFTNESS_K: f32 = 16.0;
const SHADOW_BIAS: f32 = 1e-3;

// ---------------------------------------------------------------------------------
// Physically based material model (docs/knowledge/sdf-3d/materials-and-texturing/
// pbr-shading-model.md): the glTF/Disney metallic-roughness parameterization —
// base_color, metallic, roughness, emissive — carried per-material through this
// shader instead of the single flat SURFACE_COLOR/REFLECTIVITY constants a prior
// version of this file used for the whole scene. `metallic`/`roughness` drive
// Schlick-Fresnel reflectivity and Cook-Torrance-style specular directly (see
// `fresnel_schlick`/`shade_light` below) rather than the earlier ad-hoc
// FRESNEL_STRENGTH/SPECULAR_STRENGTH constants, so a shinier/more metallic material
// genuinely reflects and highlights more without needing separately-tuned knobs per
// effect. `Material`/`material_new`/`material_f0`/`fresnel_schlick` themselves live in
// the shared assets/shaders/material.wgsl library (imported at the top of this file)
// rather than being defined here, so pattern shaders (see assets/shaders/patterns/
// checkerboard.wgsl) can use the exact same type without depending on raymarch.wgsl.
// ---------------------------------------------------------------------------------

// Default material for a leaf with no sdf::components::Material/ProceduralPattern at
// all (pattern_id == 0 and every material field defaults to 0, see
// crate::raymarch::flatten's DEFAULT_MATERIAL and PrimitiveRecordCpu's op()/leaf()
// constructors) — a neutral mid-roughness dielectric, matching this demo's original
// flat SURFACE_COLOR from before per-primitive materials existed.
const FALLBACK_MATERIAL: Material = Material(vec3<f32>(0.72, 0.70, 0.66), 0.0, 0.5, vec3<f32>(0.0));

// Reflective only if the winning leaf's own resolved material is smooth enough to
// read as a mirror rather than a diffuse/rough surface (roughness < 0.5) — data-
// driven per-entity now (via sdf::components::Material::roughness), not the old
// hardcoded per-shape-tag allowlist: any primitive authored with low enough roughness
// is mirror-reflective, whether it's the ground, a sphere, or a ring.
fn is_mirror_reflective(hit_material: Material) -> bool {
    return hit_material.roughness < 0.5;
}

// Resolves the full material at a converged hit point, driven entirely by the winning
// leaf record's own fields (see nearest_leaf_index/record_material_a/
// record_material_b) rather than a hardcoded per-shape-tag table — extending the demo
// with a new per-primitive or procedural material means authoring sdf::components::
// Material/ProceduralPattern on the entity, not touching this shader at all:
// - pattern_id == 0: this leaf has a plain sdf::components::Material (or none — see
//   FALLBACK_MATERIAL) — base_color is smoothly blended against every other leaf in
//   the same CSG range (see blended_material_color), metallic/roughness taken
//   directly from the winning leaf (a hard switch, not blended: the visual seam is
//   dominated by the smoothly-blended color/normal, so re-deriving smoothly-blended
//   metallic/roughness isn't worth a second per-leaf scan for the marginal benefit).
// - pattern_id != 0: this leaf has a sdf::components::ProceduralPattern — dispatched
//   dynamically via the generated pattern_dispatch module (see extract::
//   dispatcher_shader_source) rather than any hardcoded pattern logic in this file.
fn resolve_material(nearest: NearestLeaf) -> Material {
    let record = primitives[nearest.index];
    if (record.pattern_id != 0u) {
        let mat_a = record_material_a(record);
        let mat_b = record_material_b(record);
        let params = vec4<f32>(record.pattern_params_x, record.pattern_params_y, record.pattern_params_z, record.pattern_params_w);
        return dispatch_pattern(record.pattern_id, nearest.eval_p, mat_a, mat_b, params);
    }

    var mat = record_material_a(record);
    mat.base_color = blended_material_color(nearest.eval_p, nearest.range_start, nearest.range_count);
    return mat;
}

// Blinn-Phong specular term, on top of the existing diffuse-only model — `view_dir`
// points from the surface back toward the camera (i.e. `-rd`), so the halfway vector
// is the standard `normalize(light_dir + view_dir)`. Shininess is derived from the
// material's own roughness (smoother = tighter, brighter highlight) and the specular
// color from Schlick-Fresnel at the light's halfway angle, tinted by the material's
// own F0 — a metal's specular highlight picks up its base_color tint, a dielectric's
// stays near-white, matching real material behavior instead of one scene-wide
// specular color/strength.
const SPECULAR_SHININESS_MIN: f32 = 8.0;
const SPECULAR_SHININESS_MAX: f32 = 256.0;

// `cast_shadows` is false for reflection-ray shading (see trace_reflection): the
// reflection is already blended into the primary color and viewed indirectly, so the
// full 48-step soft_shadow trace per light there buys little visible fidelity for a
// real per-pixel cost (see this shader's own MAX_STEPS_NEAR/SHADOW_MAX_STEPS tuning
// comments below on how directly step counts multiply into frame cost) — skipping it
// specifically for reflections keeps primary-ray shadow quality untouched.
fn shade_light(p: vec3<f32>, n: vec3<f32>, view_dir: vec3<f32>, mat: Material, light: Light, cast_shadows: bool) -> vec3<f32> {
    var light_dir: vec3<f32>;
    var attenuation = 1.0;

    if (light.kind == LIGHT_KIND_DIRECTIONAL) {
        light_dir = -vec3<f32>(light.direction_or_position_x, light.direction_or_position_y, light.direction_or_position_z);
    } else {
        let light_pos = vec3<f32>(light.direction_or_position_x, light.direction_or_position_y, light.direction_or_position_z);
        let to_light = light_pos - p;
        let dist = length(to_light);
        light_dir = to_light / max(dist, 1e-4);
        attenuation = 1.0 / max(dist * dist, 0.01);

        if (light.kind == LIGHT_KIND_SPOT) {
            let spot_dir = normalize(vec3<f32>(light.spot_direction_x, light.spot_direction_y, light.spot_direction_z));
            let cos_angle = dot(-light_dir, spot_dir);
            let spot_atten = smoothstep(cos(light.outer_angle), cos(light.inner_angle), cos_angle);
            attenuation = attenuation * spot_atten;
        }
    }

    let ndotl = max(dot(n, light_dir), 0.0);
    if (ndotl <= 0.0) {
        return vec3<f32>(0.0);
    }

    var shadow = 1.0;
    if (cast_shadows && (scene.debug_flags & DBG_DISABLE_SHADOWS) == 0u) {
        shadow = soft_shadow(p + n * SHADOW_BIAS, light_dir, SHADOW_SOFTNESS_K);
    }
    let color = vec3<f32>(light.color_r, light.color_g, light.color_b);
    let radiance = color * light.intensity * attenuation * LIGHT_SCALE;

    // A rougher surface spreads its highlight over a wider angle and dims its peak
    // (energy conservation, approximated rather than exactly normalized — a full
    // GGX/Smith normalization is more than this per-pixel budget needs, see
    // docs/knowledge/sdf-3d/materials-and-texturing/pbr-shading-model.md's cost
    // section on raymarcher BRDF budgets).
    let shininess = mix(SPECULAR_SHININESS_MAX, SPECULAR_SHININESS_MIN, mat.roughness);
    let halfway = normalize(light_dir + view_dir);
    let spec_fresnel = fresnel_schlick(max(dot(halfway, view_dir), 0.0), material_f0(mat));
    let spec_strength = pow(max(dot(n, halfway), 0.0), shininess) * (1.0 - mat.roughness * 0.7);
    let specular = spec_fresnel * spec_strength;

    // Metals have no diffuse response (all incident light is either reflected
    // specularly or absorbed) — mat.metallic drives diffuse energy down to 0 as it
    // approaches 1, standard metallic-roughness energy split.
    let diffuse = mat.base_color * (1.0 - mat.metallic) * ndotl;

    return radiance * shadow * (diffuse + specular);
}

// ---------------------------------------------------------------------------------
// Ray reconstruction + fragment entry point.
// ---------------------------------------------------------------------------------

const AMBIENT: vec3<f32> = vec3<f32>(0.05, 0.06, 0.08);

// Beyond REFLECTION_CUTOFF_DIST, skip the reflection trace outright (see fragment()) —
// tuned against this scene's orbit camera range (radius 6..60, main.rs's OrbitCamera).
// Reflection strength itself is faded to 0 over
// [REFLECTION_FADE_START, REFLECTION_CUTOFF_DIST] rather than held at full strength
// right up to the cutoff — a hard on/off at a single distance is exactly what makes
// the cutoff visible as a seam (reflective tiles just ahead of it look mirror-bright,
// tiles just past it look flat); fading removes the seam without changing where the
// (expensive) trace itself stops being paid for.
const REFLECTION_CUTOFF_DIST: f32 = 34.0;
const REFLECTION_FADE_START: f32 = 20.0;

// Emissive pulse added on top of the rings' own lit color (see fragment()'s use).
// Rings are now beaded circles of spheres, not a single torus leaf, so there is no
// per-primitive tag left to key off of — instead this uses `record.anim_group`
// (GPU encoding: `group_id + 1`, see PrimitiveRecordCpu::anim_group's doc comment),
// since every ring bead lives in one of sdf::world's RING_ANIM_GROUP_BASE..+3 groups
// while the only other AnimGroup member (the pillar) is group 0. Driven by
// `scene.time_secs` (see RaymarchSceneUniform's doc comment for where that comes
// from), a plain sine so every ring pulses in lockstep — this scene's 3 rings already
// spin at distinct per-instance speeds (see sdf::world::AnimatedRing), so a shared
// pulse still reads as "the rings" having one coherent identity rather than each
// spinning independently, without needing a per-ring phase offset plumbed through.
const RING_ANIM_GROUP_GPU_MIN: u32 = 2u; // sdf::world::RING_ANIM_GROUP_BASE (1) + 1
const GLOW_COLOR: vec3<f32> = vec3<f32>(0.3, 0.75, 1.0);
const GLOW_STRENGTH: f32 = 0.55;
const GLOW_PULSE_SPEED: f32 = 2.0;

fn ring_glow_color() -> vec3<f32> {
    let pulse = 0.5 + 0.5 * sin(scene.time_secs * GLOW_PULSE_SPEED);
    return GLOW_COLOR * GLOW_STRENGTH * pulse;
}

// Sun disk: a bright core (SUN_DISK_SIZE, tight) plus a wider soft glow
// (SUN_GLOW_SIZE) around the scene's own first directional light's direction — reuses
// `lights[]`, which is already bound and populated every frame (see extract::
// extract_raymarch_lights), rather than adding a separate sun-direction uniform. Both
// terms are `smoothstep`-shaped falloffs of `dot(rd, sun_dir)` (1.0 = looking directly
// at the sun), which is a handful of extra ALU ops paid only on sky-miss pixels — this
// scene's own tuning notes already note primary rays miss (sky-color) far less often
// than they hit geometry, so this is cheap in aggregate. Pairs well with Bloom
// (already on the camera, see main.rs) since a bright, tightly-clipped disk blooms
// nicely without needing HDR values much above 1.0.
const SUN_DISK_SIZE: f32 = 0.9995;
const SUN_GLOW_SIZE: f32 = 0.98;
const SUN_COLOR: vec3<f32> = vec3<f32>(1.0, 0.95, 0.85);
const SUN_GLOW_COLOR: vec3<f32> = vec3<f32>(1.0, 0.85, 0.6);

fn first_directional_light_dir() -> vec3<f32> {
    for (var i = 0u; i < scene.light_count; i = i + 1u) {
        let light = lights[i];
        if (light.kind == LIGHT_KIND_DIRECTIONAL) {
            return -vec3<f32>(light.direction_or_position_x, light.direction_or_position_y, light.direction_or_position_z);
        }
    }
    // No directional light in the scene: point "sun" straight up, where SUN_DISK_SIZE/
    // SUN_GLOW_SIZE's tight thresholds mean it's very unlikely any sky ray happens to
    // align with it — effectively a no-op rather than a special-cased branch.
    return vec3<f32>(0.0, 1.0, 0.0);
}

fn sky_color(rd: vec3<f32>) -> vec3<f32> {
    let t = clamp(rd.y * 0.5 + 0.5, 0.0, 1.0);
    var color = mix(vec3<f32>(0.10, 0.11, 0.14), vec3<f32>(0.45, 0.55, 0.75), t);

    let sun_dir = first_directional_light_dir();
    let sun_dot = dot(rd, sun_dir);
    let glow = smoothstep(SUN_GLOW_SIZE, 1.0, sun_dot);
    let disk = smoothstep(SUN_DISK_SIZE, 1.0, sun_dot);
    color = color + SUN_GLOW_COLOR * glow * 0.6;
    color = color + SUN_COLOR * disk;

    return color;
}

// Distance fog range (see fragment()'s use) — starts past where the orbit camera's
// close-in radius (6, main.rs's OrbitCamera) would ever show fogged geometry, and
// reaches full sky color well before MAX_MARCH_DIST=120 so the tiled ground fades out
// gradually rather than the raymarcher's own far-plane cutoff being visible as a hard
// edge.
const FOG_START: f32 = 1000.0;
const FOG_END: f32 = 2000.0;

// Non-metal rim brightening at grazing angles — Schlick-Fresnel evaluated against
// the dielectric F0 (0.04, see DIELECTRIC_F0 above) rather than a separately-tuned
// FRESNEL_STRENGTH constant, so the rim glow strength is physically tied to the same
// F0 that drives specular/reflection instead of an independent knob. Metals don't get
// this additive rim term — their grazing-angle brightening already comes through the
// tinted specular term in shade_light, adding a second white-ish rim on top would
// double-count and wash out their base_color tint.
const RIM_GLOW_MULTIPLIER: f32 = 3.0;

// Shading at a converged hit: direct multi-light (+ shadow + AO when `full_quality`) +
// a dielectric Fresnel rim glow, using `mat`'s own physically based parameters
// (base_color, metallic, roughness — see the Material struct above) rather than one
// flat scene-wide color/reflectivity. Primary hits always use full_quality=true;
// reflection hits use false (see trace_reflection) — a flat N.L term with no shadow
// ray/AO march reads as a very close approximation once blended into the primary
// color, and is dramatically cheaper per hit.
fn shade_surface(p: vec3<f32>, n: vec3<f32>, rd: vec3<f32>, mat: Material, full_quality: bool) -> vec3<f32> {
    let view_dir = -rd;
    var color = mat.base_color * (1.0 - mat.metallic) * AMBIENT + mat.emissive;
    for (var i = 0u; i < scene.light_count; i = i + 1u) {
        color = color + shade_light(p, n, view_dir, mat, lights[i], full_quality);
    }
    if (full_quality) {
        if ((scene.debug_flags & DBG_DISABLE_AO) == 0u) {
            color = color * ambient_occlusion(p, n);
        }
        if (mat.metallic < 0.5) {
            // Additive rim glow (not multiplied by AO/shadow): a cheap way to make
            // convex silhouettes — the blob cluster, pillar, ring tubes — read as lit
            // from behind/around rather than flat-shaded, and it composites well with
            // Bloom since grazing-edge brightening is exactly the kind of highlight
            // that blooms nicely.
            let rim = fresnel_schlick(max(dot(n, view_dir), 0.0), material_f0(mat));
            color = color + AMBIENT * RIM_GLOW_MULTIPLIER * rim;
        }
    }
    return color;
}

// Reduced march budget/distance for reflection rays: viewed indirectly and blended
// into the primary color, so the primary ray's full near-field precision is not
// needed. The distance cap matters far more than the step cap here — this scene's own
// tuning notes (see MAX_STEPS_NEAR below) already found step count has a smaller
// effect on frame cost than call count, and reflective ground fills most of the
// frame, so most reflection rays point up toward open sky with no nearby geometry: at
// the primary ray's full MAX_MARCH_DIST=120 budget, EVERY one of those sky-bound
// reflection rays has to march the entire 120 units (paying REFLECTION_MAX_STEPS
// worth of map() calls in full) before concluding "miss, use sky_color" — the exact
// case a reflection ray hits constantly and a primary ray rarely does. A much shorter
// cap reaches the same sky-color result in far fewer steps for that dominant case, at
// the cost of reflections not showing objects farther than REFLECTION_MAX_DIST away
// (acceptable: distant reflected detail is the least noticeable loss once blended).
const REFLECTION_MAX_STEPS: u32 = 24u;
const REFLECTION_MAX_DIST: f32 = 15.0;
// Minimum blended strength worth paying a second sphere-trace for: Schlick-Fresnel
// against the material's own F0 means a near-perpendicular view of any dielectric
// (the checkerboard ground seen from orbit, dot(n,v)≈1) reflects only ~4% — visually
// lost under fog/bloom regardless, but previously still paid the full reflection ray.
// Metals carry F0 ≈ base_color (≥~0.5) so they always clear this bar; dielectrics now
// trace only at grazing angles where reflections actually read.
const REFLECTION_MIN_STRENGTH: f32 = 0.10;

fn sphere_trace_reflection(ro: vec3<f32>, rd: vec3<f32>) -> RaymarchHit {
    var t = 0.0;
    for (var i = 0u; i < REFLECTION_MAX_STEPS; i = i + 1u) {
        let radius = abs(map(ro + rd * t));
        let eps = HIT_EPSILON * max(t, 1.0);
        if (radius < eps) {
            return RaymarchHit(true, ro + rd * t, i + 1u);
        }
        t = t + radius;
        if (t > REFLECTION_MAX_DIST) {
            break;
        }
    }
    return RaymarchHit(false, vec3<f32>(0.0), REFLECTION_MAX_STEPS);
}

// One-bounce mirror reflection off a reflective surface hit: traces a second,
// reduced-budget ray from the hit point along the mirror direction and shades it
// cheaply (no shadow ray, no AO march — see shade_surface/sphere_trace_reflection),
// since this scene is cheap enough per docs/knowledge that a second trace is still
// preferable to the probe/denoise machinery large-scene renderers need, as long as
// that second trace's own cost is kept well below the primary ray's.
fn trace_reflection(p: vec3<f32>, n: vec3<f32>, rd: vec3<f32>) -> vec3<f32> {
    let reflect_dir = reflect(rd, n);
    let reflect_origin = p + n * SHADOW_BIAS;
    let reflect_hit = sphere_trace_reflection(reflect_origin, reflect_dir);
    if (!reflect_hit.did_hit) {
        return sky_color(reflect_dir);
    }
    let reflect_nearest = nearest_leaf_index(reflect_hit.position);
    let reflect_n = calc_normal(reflect_nearest);
    let reflect_mat = resolve_material(reflect_nearest);
    return shade_surface(reflect_hit.position, reflect_n, reflect_dir, reflect_mat, false);
}

@fragment
fn fragment(@builtin(position) frag_coord: vec4<f32>) -> @location(0) vec4<f32> {
    let ndc = uv_to_ndc(frag_coord_to_uv(frag_coord.xy, view.viewport));

    // Bevy 0.19 uses an infinite reverse-Z projection: the clip-space far plane
    // (z=0) maps to w=0, so unprojecting it divides by zero. Reconstruct the ray
    // from the near-plane point only — for a perspective camera the pixel ray
    // passes through the camera position, so rd = normalize(near_pos - ro).
    let near_clip = vec4<f32>(ndc, 1.0, 1.0);
    let near_world = view.world_from_clip * near_clip;
    let near_pos = near_world.xyz / near_world.w;

    let ro = view.world_position;
    let rd = normalize(near_pos - ro);

    let hit = sphere_trace(ro, rd, MAX_MARCH_DIST);
    if (!hit.did_hit) {
        return vec4<f32>(sky_color(rd), 1.0);
    }

    // Debug heatmap: map sphere-trace step count to a blue→red gradient so hot
    // pixels (many steps) stand out immediately — the single most useful visual
    // for finding where the budget is being spent.
    if ((scene.debug_flags & DBG_HEATMAP) != 0u) {
        let t = clamp(f32(hit.steps) / f32(MAX_STEPS_NEAR), 0.0, 1.0);
        return vec4<f32>(t, 0.3, 1.0 - t, 1.0);
    }

    // Nearest leaf resolved first — calc_normal needs it to look up the winning
    // primitive's analytical gradient via sdg_dispatch.
    let nearest = nearest_leaf_index(hit.position);
    let n = calc_normal(nearest);

    // anim_group used by the ring emissive glow below.
    let hit_anim_group = primitives[nearest.index].anim_group;
    let mat = resolve_material(nearest);

    var color = shade_surface(hit.position, n, rd, mat, true);

    if (hit_anim_group >= RING_ANIM_GROUP_GPU_MIN) {
        color = color + ring_glow_color();
    }

    let camera_dist = distance(ro, hit.position);

    // Skip the reflection trace entirely past REFLECTION_CUTOFF_DIST: a reflective
    // ground tile that far from the camera already covers only a handful of screen
    // pixels (this scene's tile_period=16 ground slabs shrink fast with distance —
    // see sdf::world::TILE_PERIOD), so its reflection is imperceptible next to the
    // cost of paying a full second sphere-trace for it. The orbit camera in main.rs
    // swings out to radius_max=60, so without this cutoff every far tile in a wide
    // shot still pays full reflection cost for no visible gain. Mirror-reflectivity is
    // now purely a function of the resolved material's own roughness
    // (is_mirror_reflective) — data-driven per-entity, not a hardcoded shape allowlist
    // — so any low-roughness material anywhere in the scene gets a real traced
    // reflection; rougher materials rely on their own Fresnel rim term (see
    // shade_surface) instead.
    let view_dir = -rd;
    let reflect_strength = fresnel_schlick(max(dot(n, view_dir), 0.0), material_f0(mat));
    let dist_fade = 1.0 - smoothstep(REFLECTION_FADE_START, REFLECTION_CUTOFF_DIST, camera_dist);
    // Scalar gate strength: reflect_strength is a per-channel tinted vec3 (see
    // fresnel_schlick) so the threshold test uses its brightest channel.
    let reflect_gate = max(reflect_strength.r, max(reflect_strength.g, reflect_strength.b));

    // Physically based reflectivity: Schlick-Fresnel evaluated against the
    // material's own F0 (0.04 for the dark dielectric chessboard cell, tinted
    // base_color for a metal) rather than the earlier flat REFLECTIVITY constant
    // — grazing-angle views reflect more, and a metal's reflection is tinted by
    // its own color instead of every reflective surface sharing one blend curve.
    // Gated on blended strength (see REFLECTION_MIN_STRENGTH) so near-perpendicular
    // dielectric views skip the second sphere-trace entirely.
    if (camera_dist < REFLECTION_CUTOFF_DIST
        && reflect_gate * dist_fade > REFLECTION_MIN_STRENGTH
        && is_mirror_reflective(mat)
        && (scene.debug_flags & DBG_DISABLE_REFLECTION) == 0u)
    {
        let reflection = trace_reflection(hit.position, n, rd);
        color = mix(color, reflection, reflect_strength * dist_fade);
    }

    // Distance fog: blends toward the sky color with camera distance, using the same
    // `camera_dist` the reflection cutoff above already computes — reads as
    // atmospheric depth on the tiled ground stretching to the horizon instead of it
    // just clipping into flat color at MAX_MARCH_DIST. Pure math on existing data, no
    // extra map()/sphere_trace calls.
    let fog = smoothstep(FOG_START, FOG_END, camera_dist);
    color = mix(color, sky_color(rd), fog);

    return vec4<f32>(color, 1.0);
}
