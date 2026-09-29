// Radiance Cascades probe relighting (compute): Stage 2 of the approved
// experimental Radiance Cascades GI technique — an A/B alternative to
// DDGI (hybrid_ddgi_relight.wgsl), selectable via GiMethod::RadianceCascades.
// See src/hybrid/radiance_cascades_ref.rs's own module doc comment for the
// full rationale (DDGI's fixed-density probe grid relays bounce energy
// cell-to-cell, so a floor corridor far from the nearest bright surface
// stays dark; cascades instead use a HIERARCHY of levels trading probe
// density for angular ray density, so far levels sample distant geometry
// DIRECTLY with wide rays).
//
// This file is a faithful WGSL port of radiance_cascades_ref.rs's
// cascade_level_params/cascade_grid_from_bounds/cascade_probe_ray/
// relight_cascade_texel — RELIGHT ONLY. The MERGE step
// (merge_cascade_texel's front-to-back L_ac = L_ab + beta_ab*L_bc
// composite) is deliberately NOT a separate compute pass here: with only
// LEVEL_COUNT=4 small levels, walking the hierarchy at SHADING time
// (hybrid_trace.wgsl's own shade(), sampling this file's own atlas
// buffer level-by-level) is 4 atlas reads plus 4 merge_cascade_texel
// calls per shaded pixel — cheaper than an extra full-screen compute
// dispatch, and it keeps this experimental technique's own GPU footprint
// to exactly ONE new pass (relight), matching the plan's own "simple,
// scoped, not production-polish" framing. See hybrid_trace.wgsl's own
// GI_METHOD_RADIANCE_CASCADES branch for the merge-at-shade-time code.
//
// Duplicates hybrid_ddgi_relight.wgsl's own trace()/march_object()/
// shade_direct_only() logic rather than importing a shared library — this
// renderer's established self-contained-pass-file convention (see that
// file's own header comment for why).
//
// ALL FOUR cascade levels are relit in ONE dispatch: the atlas buffer is
// laid out as 4 consecutive regions (one per level, each level's own
// exact-fit tile packing computed on the Rust side — see
// src/hybrid/pipeline.rs's own RadianceCascadesLevelUniform/
// RadianceCascadesGridUniform doc comments), and gid.z selects which
// level this invocation belongs to (workgroup_size 8x8x1, dispatched with
// wg_z = LEVEL_COUNT so each level gets its own Z slice) — a single
// dispatch call covering the whole hierarchy is simpler than 4 separate
// dispatch_workgroups calls from pass.rs for an experimental technique
// this small, and WGSL has no per-invocation cost difference between the
// two shapes (it's still `total_probes_at_level * tile_size^2` invocations
// either way, just organized along one axis instead of 4 dispatch calls).

struct SceneUniform {
    object_count: u32,
    bvh_node_count: u32,
    light_count: u32,
    shadows_enabled: u32,
    background_r: f32,
    background_g: f32,
    background_b: f32,
    denoise_enabled: u32,
    temporal_enabled: u32,
    temporal_max_history_length: f32,
    gi_method: u32,
    conetrace_half_angle: f32,
    conetrace_origin_radius: f32,
    conetrace_max_t: f32,
    conetrace_max_bounces: u32,
    reflection_enabled: u32,
    reflection_max_bounces: u32,
    reflection_fresnel_cutoff: f32,
    reflection_max_t: f32,
    transmission_enabled: u32,
    transmission_max_bounces: u32,
    transmission_fresnel_cutoff: f32,
    transmission_max_t: f32,
    ddgi_probes_per_frame: u32,
    ddgi_total_probes: u32,
    ddgi_tile_size: u32,
    ddgi_frame_index: u32,
    ddgi_max_history_length: f32,
    ddgi_max_t: f32,
    dof_enabled: u32,
    dof_focal_distance: f32,
    dof_aperture_radius: f32,
    dof_frame_index: u32,
    dof_max_history_length: f32,
    // Primary-ray sub-pixel jitter (TAAU experiment step 1a) — unused in
    // this file (radiance cascades has no primary-ray dependency, see
    // src/hybrid/extract.rs's SceneUniform doc comment), kept only for
    // struct-layout parity with the other SceneUniform copies sharing
    // this same uniform buffer.
    jitter_enabled: u32,
    jitter_offset_x: f32,
    jitter_offset_y: f32,
    // Trace-resolution scale experiment (step 1b) — unused in this file,
    // kept only for struct-layout parity with the other SceneUniform
    // copies sharing this same uniform buffer.
    trace_size_x: u32,
    trace_size_y: u32,
}

struct ObjectGpu {
    shape_kind: u32,
    _pad_kind0: u32, _pad_kind1: u32, _pad_kind2: u32,
    param_0: f32, param_1: f32, param_2: f32, param_3: f32,
    param_4: f32, param_5: f32, param_6: f32, param_7: f32,
    translation_x: f32, translation_y: f32, translation_z: f32,
    _pad_translation: f32,
    inv_rotation_x: f32, inv_rotation_y: f32, inv_rotation_z: f32, inv_rotation_w: f32,
    base_color_r: f32, base_color_g: f32, base_color_b: f32,
    metallic: f32,
    roughness: f32, reflectance: f32,
    transmission: f32, ior: f32,
    emissive_r: f32, emissive_g: f32, emissive_b: f32,
    _pad_emissive: f32,
    prev_translation_x: f32, prev_translation_y: f32, prev_translation_z: f32,
    _pad_prev_translation: f32,
    prev_inv_rotation_x: f32, prev_inv_rotation_y: f32, prev_inv_rotation_z: f32, prev_inv_rotation_w: f32,
}

const SHAPE_KIND_SPHERE: u32 = 0u;
const SHAPE_KIND_ROUNDED_BOX: u32 = 1u;
const SHAPE_KIND_ROUNDED_CYLINDER: u32 = 2u;
const SHAPE_KIND_CAPSULE: u32 = 3u;
const SHAPE_KIND_ELLIPSOID: u32 = 4u;
const SHAPE_KIND_BOX_FRAME: u32 = 5u;
const SHAPE_KIND_HEX_PRISM: u32 = 6u;

struct BvhNode {
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    left_or_sentinel: u32,
    right_or_object: u32,
}

struct LightGpu {
    kind: u32,
    _pad_kind0: u32, _pad_kind1: u32, _pad_kind2: u32,
    color_r: f32, color_g: f32, color_b: f32,
    intensity: f32,
    direction_or_position_x: f32, direction_or_position_y: f32, direction_or_position_z: f32,
    range: f32,
    spot_direction_x: f32, spot_direction_y: f32, spot_direction_z: f32,
    inner_angle: f32,
    outer_angle: f32,
    shadow_softness_k: f32,
    _pad_tail1: f32, _pad_tail2: f32,
}

const LIGHT_KIND_DIRECTIONAL: u32 = 0u;
const LIGHT_KIND_POINT: u32 = 1u;
const LIGHT_KIND_SPOT: u32 = 2u;

// Mirrors src/hybrid/pipeline.rs's own RadianceCascadesLevelUniform
// exactly — one entry per cascade level (fixed LEVEL_COUNT=4, see this
// file's own header comment), each carrying that level's own
// cascade_level_params (probe_spacing/ray_count/interval_near/
// interval_far), its own CascadeGrid (origin/spacing/dims, mirroring
// ddgi_ref::ProbeGrid's shape), and its own exact-fit atlas sub-region
// (tile_size/tiles_per_row/atlas_texel_offset — where this level's own
// tiles start within the ONE shared atlas buffer, all 4 levels' regions
// laid out consecutively, sized on the Rust side by
// AtlasLayout::exact_fit per level).
struct CascadeLevelUniform {
    probe_spacing: f32,
    ray_count: u32,
    interval_near: f32,
    interval_far: f32,
    grid_origin: vec3<f32>,
    _pad0: f32,
    grid_dims: vec3<u32>,
    _pad1: u32,
    tile_size: u32,
    tiles_per_row: u32,
    atlas_texel_offset: u32,
    total_probes: u32,
}

const LEVEL_COUNT: u32 = 4u;

@group(0) @binding(0) var<uniform> scene: SceneUniform;
@group(0) @binding(1) var<storage, read> objects: array<ObjectGpu>;
@group(0) @binding(2) var<storage, read> bvh: array<BvhNode>;
@group(0) @binding(3) var<storage, read> lights: array<LightGpu>;
// One shared atlas across all 4 levels — row-major array<vec4<f32>>, each
// level's own texels living at [atlas_texel_offset, atlas_texel_offset +
// tiles_per_row^2 * tile_size^2) — see CascadeLevelUniform's own doc
// comment. read_write for the identical reason hybrid_ddgi_relight.wgsl's
// own atlas is: each invocation only ever writes ITS OWN texel (level +
// probe + ray_index uniquely determines one texel, never shared across
// invocations), so there is no ping-pong/cross-invocation race to guard
// against (see HybridDdgiAtlas's own doc comment for the fuller
// precedent this follows).
@group(0) @binding(4) var<storage, read_write> atlas: array<vec4<f32>>;
@group(0) @binding(5) var<storage, read> levels: array<CascadeLevelUniform, 4>;

// --- cpu_ref.rs's named constants, ported verbatim. --------------------

const MAX_MARCH_STEPS: u32 = 128u;
const HIT_EPSILON: f32 = 1e-4;
const LEAF_SENTINEL: u32 = 4294967295u; // u32::MAX
const MAX_STACK: u32 = 64u;

// --- cpu_ref.rs's sd_* functions, verbatim (identical to every other
// pass file's own copy). -------------------------------------------------

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

fn local_distance(obj: ObjectGpu, p_local: vec3<f32>) -> f32 {
    switch obj.shape_kind {
        case SHAPE_KIND_SPHERE: {
            return sd_sphere(p_local, obj.param_0);
        }
        case SHAPE_KIND_ROUNDED_BOX: {
            return sd_rounded_box(p_local, vec3<f32>(obj.param_0, obj.param_1, obj.param_2), obj.param_3);
        }
        case SHAPE_KIND_ROUNDED_CYLINDER: {
            return sd_rounded_cylinder(p_local, obj.param_0, obj.param_1, obj.param_2);
        }
        case SHAPE_KIND_CAPSULE: {
            let a = vec3<f32>(obj.param_0, obj.param_1, obj.param_2);
            let b = vec3<f32>(obj.param_3, obj.param_4, obj.param_5);
            return sd_capsule(p_local, a, b, obj.param_6);
        }
        case SHAPE_KIND_ELLIPSOID: {
            return sd_ellipsoid(p_local, vec3<f32>(obj.param_0, obj.param_1, obj.param_2));
        }
        case SHAPE_KIND_BOX_FRAME: {
            return sd_box_frame(p_local, vec3<f32>(obj.param_0, obj.param_1, obj.param_2), obj.param_3);
        }
        case SHAPE_KIND_HEX_PRISM: {
            return sd_hex_prism(p_local, obj.param_0, obj.param_1);
        }
        default: {
            return sd_sphere(p_local, 0.0);
        }
    }
}

const NORMAL_EPSILON: f32 = 1e-3;

fn local_normal(obj: ObjectGpu, p_local: vec3<f32>) -> vec3<f32> {
    let e = NORMAL_EPSILON;
    let dx = local_distance(obj, p_local + vec3<f32>(e, 0.0, 0.0)) - local_distance(obj, p_local - vec3<f32>(e, 0.0, 0.0));
    let dy = local_distance(obj, p_local + vec3<f32>(0.0, e, 0.0)) - local_distance(obj, p_local - vec3<f32>(0.0, e, 0.0));
    let dz = local_distance(obj, p_local + vec3<f32>(0.0, 0.0, e)) - local_distance(obj, p_local - vec3<f32>(0.0, 0.0, e));
    let g = vec3<f32>(dx, dy, dz);
    let len = length(g);
    if (len < 1e-8) {
        return vec3<f32>(0.0);
    }
    return g / len;
}

fn rotate_by_quat(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    return q.xyz * 2.0 * dot(q.xyz, v) + v * (q.w * q.w - dot(q.xyz, q.xyz)) + cross(q.xyz, v) * 2.0 * q.w;
}

struct MarchResult {
    hit: bool,
    t: f32,
}

// cpu_ref.rs::pixel_eps verbatim (see hybrid_trace.wgsl's own copy for the
// full rationale) -- march_object's convergence tolerance grows with t.
fn pixel_eps(t: f32) -> f32 {
    return max(t * 0.0016, 2e-4);
}

// cpu_ref.rs::march_object verbatim -- convergence tolerance grows with t
// via pixel_eps (see hybrid_trace.wgsl's own copy of this doc comment for
// the full rationale: cuts step count on distant geometry without losing
// near-field precision).
fn march_object(obj: ObjectGpu, ray_origin: vec3<f32>, ray_dir: vec3<f32>, t_start: f32, t_max: f32) -> MarchResult {
    let inv_rotation = vec4<f32>(obj.inv_rotation_x, obj.inv_rotation_y, obj.inv_rotation_z, obj.inv_rotation_w);
    let translation = vec3<f32>(obj.translation_x, obj.translation_y, obj.translation_z);
    var t = max(t_start, 0.0);
    for (var i = 0u; i < MAX_MARCH_STEPS; i = i + 1u) {
        if (t > t_max) {
            return MarchResult(false, 0.0);
        }
        let p_world = ray_origin + t * ray_dir;
        let p_local = rotate_by_quat(inv_rotation, p_world - translation);
        let d = local_distance(obj, p_local);
        if (d < max(HIT_EPSILON, pixel_eps(t))) {
            return MarchResult(true, t);
        }
        t = t + d;
    }
    return MarchResult(false, 0.0);
}

fn slab_hit(ray_origin: vec3<f32>, ray_dir: vec3<f32>, bmin: vec3<f32>, bmax: vec3<f32>, t_max: f32) -> vec2<f32> {
    let inv_dir = 1.0 / ray_dir;
    let t0 = (bmin - ray_origin) * inv_dir;
    let t1 = (bmax - ray_origin) * inv_dir;
    let t_small = min(t0, t1);
    let t_big = max(t0, t1);
    let t_near = max(max(t_small.x, t_small.y), max(t_small.z, 0.0));
    let t_far = min(min(t_big.x, t_big.y), min(t_big.z, t_max));
    return vec2<f32>(t_near, t_far);
}

struct Hit {
    did_hit: bool,
    t: f32,
    world_normal: vec3<f32>,
    obj_id: u32,
}

// Interval-clipped trace, mirroring radiance_cascades_ref.rs::
// relight_cascade_texel's own trace()+interval-clip logic (NOT
// cpu_ref.rs::trace()'s unbounded-max_t search): fires out to t_max
// exactly like every other trace() in this codebase (closest-hit-wins BVH
// traversal, same slab test), the interval-near clip against the result
// happens in the CALLER (relight_cascade_texel below) — matching the
// Rust reference's own split (trace() first, then check `hit.t >=
// interval_near` at the call site) so this function stays a faithful,
// reusable port of the SAME trace() every other pass file already has,
// rather than a cascade-specific variant.
fn trace(ray_origin: vec3<f32>, ray_dir: vec3<f32>, t_max: f32) -> Hit {
    var best_hit = false;
    var best_t = t_max;
    var best_normal = vec3<f32>(0.0);
    var best_obj_id = 0u;

    if (scene.bvh_node_count == 0u) {
        return Hit(false, 0.0, vec3<f32>(0.0), 0u);
    }

    var stack: array<u32, MAX_STACK>;
    var sp: u32 = 0u;
    stack[0] = 0u;
    sp = 1u;

    loop {
        if (sp == 0u) {
            break;
        }
        sp = sp - 1u;
        let node_index = stack[sp];
        let node = bvh[node_index];

        let limit = select(t_max, best_t, best_hit);
        let hit = slab_hit(ray_origin, ray_dir, vec3<f32>(node.min_x, node.min_y, node.min_z),
            vec3<f32>(node.max_x, node.max_y, node.max_z), limit);
        let t_near = hit.x;
        let t_far = hit.y;
        if (t_near > t_far) {
            continue;
        }

        if (node.left_or_sentinel == LEAF_SENTINEL) {
            let obj_id = node.right_or_object;
            if (obj_id >= scene.object_count) {
                continue;
            }
            let object = objects[obj_id];
            let march_limit = select(t_far, min(best_t, t_far), best_hit);
            let march = march_object(object, ray_origin, ray_dir, t_near, march_limit);
            if (march.hit && (!best_hit || march.t < best_t)) {
                best_hit = true;
                best_t = march.t;
                let inv_rotation = vec4<f32>(object.inv_rotation_x, object.inv_rotation_y, object.inv_rotation_z, object.inv_rotation_w);
                let translation = vec3<f32>(object.translation_x, object.translation_y, object.translation_z);
                let p_world = ray_origin + march.t * ray_dir;
                let p_local = rotate_by_quat(inv_rotation, p_world - translation);
                let local_n = local_normal(object, p_local);
                best_normal = rotate_by_quat(vec4<f32>(-inv_rotation.xyz, inv_rotation.w), local_n);
                best_obj_id = obj_id;
            }
            continue;
        }

        let left = node.left_or_sentinel;
        let right = node.right_or_object;
        let left_node = bvh[left];
        let right_node = bvh[right];
        let left_hit = slab_hit(ray_origin, ray_dir, vec3<f32>(left_node.min_x, left_node.min_y, left_node.min_z),
            vec3<f32>(left_node.max_x, left_node.max_y, left_node.max_z), limit);
        let right_hit = slab_hit(ray_origin, ray_dir, vec3<f32>(right_node.min_x, right_node.min_y, right_node.min_z),
            vec3<f32>(right_node.max_x, right_node.max_y, right_node.max_z), limit);
        let left_visitable = left_hit.x <= left_hit.y;
        let right_visitable = right_hit.x <= right_hit.y;
        if (left_visitable && right_visitable) {
            if (sp + 2u <= MAX_STACK) {
                if (left_hit.x <= right_hit.x) {
                    stack[sp] = right;
                    sp = sp + 1u;
                    stack[sp] = left;
                    sp = sp + 1u;
                } else {
                    stack[sp] = left;
                    sp = sp + 1u;
                    stack[sp] = right;
                    sp = sp + 1u;
                }
            }
        } else if (left_visitable) {
            if (sp + 1u <= MAX_STACK) {
                stack[sp] = left;
                sp = sp + 1u;
            }
        } else if (right_visitable) {
            if (sp + 1u <= MAX_STACK) {
                stack[sp] = right;
                sp = sp + 1u;
            }
        }
    }

    return Hit(best_hit, best_t, best_normal, best_obj_id);
}

const EXPOSURE: f32 = 0.0005;

struct LightSample {
    to_light: vec3<f32>,
    radiance: vec3<f32>,
    shadow_max_t: f32,
}

const DIRECTIONAL_SHADOW_MAX_T: f32 = 12.0;

fn sample_light(lgt: LightGpu, p_world: vec3<f32>) -> LightSample {
    var to_light: vec3<f32>;
    var attenuation: f32;
    var shadow_max_t: f32;
    if (lgt.kind == LIGHT_KIND_DIRECTIONAL) {
        let direction = vec3<f32>(lgt.direction_or_position_x, lgt.direction_or_position_y, lgt.direction_or_position_z);
        to_light = -normalize(direction);
        attenuation = 1.0;
        shadow_max_t = DIRECTIONAL_SHADOW_MAX_T;
    } else {
        let light_pos = vec3<f32>(lgt.direction_or_position_x, lgt.direction_or_position_y, lgt.direction_or_position_z);
        let delta = light_pos - p_world;
        let dist = length(delta);
        to_light = delta / max(dist, 1e-4);
        let range = max(lgt.range, 1e-3);
        let dist_atten = clamp(1.0 - pow(dist / range, 4.0), 0.0, 1.0) / pow(max(dist, 1.0), 2.0);
        if (lgt.kind == LIGHT_KIND_SPOT) {
            let spot_direction = vec3<f32>(lgt.spot_direction_x, lgt.spot_direction_y, lgt.spot_direction_z);
            let cos_angle = dot(-to_light, normalize(spot_direction));
            attenuation = dist_atten * smoothstep(cos(lgt.outer_angle), cos(lgt.inner_angle), cos_angle);
        } else {
            attenuation = dist_atten;
        }
        shadow_max_t = dist;
    }

    var intensity: f32;
    if (lgt.kind == LIGHT_KIND_DIRECTIONAL) {
        intensity = lgt.intensity;
    } else {
        intensity = lgt.intensity / (4.0 * 3.14159265);
    }
    let color = vec3<f32>(lgt.color_r, lgt.color_g, lgt.color_b);
    return LightSample(to_light, color * intensity * EXPOSURE * attenuation, shadow_max_t);
}

fn ggx_distribution(n_dot_h: f32, alpha: f32) -> f32 {
    let alpha_sq = alpha * alpha;
    let denom = n_dot_h * n_dot_h * (alpha_sq - 1.0) + 1.0;
    return alpha_sq / max(3.14159265 * denom * denom, 1e-8);
}

fn ggx_visibility(n_dot_l: f32, n_dot_v: f32, alpha: f32) -> f32 {
    let alpha_sq = alpha * alpha;
    let lambda_v = n_dot_l * sqrt(n_dot_v * n_dot_v * (1.0 - alpha_sq) + alpha_sq);
    let lambda_l = n_dot_v * sqrt(n_dot_l * n_dot_l * (1.0 - alpha_sq) + alpha_sq);
    return 0.5 / max(lambda_v + lambda_l, 1e-4);
}

fn fresnel_schlick(f0: vec3<f32>, cos_theta: f32) -> vec3<f32> {
    let m = clamp(1.0 - cos_theta, 0.0, 1.0);
    return f0 + (vec3<f32>(1.0) - f0) * pow(m, 5.0);
}

fn dielectric_f0(reflectance: f32) -> f32 {
    return 0.16 * reflectance * reflectance;
}

// No trace_shadow()/soft-shadow candidate gathering here (unlike
// hybrid_ddgi_relight.wgsl) — radiance_cascades_ref.rs::relight_cascade_texel
// calls cpu_ref::shade with a plain HARD shadow test (its own `shade`
// call passes `bvh`/`objects` straight through to shade()'s own internal
// occlusion check, which cpu_ref.rs's shade() resolves via a single
// trace() to the light, not the soft penumbra k-based marching
// hybrid_ddgi_relight.wgsl's own shade_direct_only additionally performs)
// — mirrored here as a single hard trace() shadow ray per light, matching
// the Rust reference's own actual behavior exactly rather than
// introducing extra softness the CPU-tested math contract doesn't have.
fn shade_direct_only(obj: ObjectGpu, obj_id: u32, p_world: vec3<f32>, world_normal: vec3<f32>, view_dir: vec3<f32>) -> vec3<f32> {
    let n = normalize(world_normal);
    let v = normalize(view_dir);
    let n_dot_v = max(dot(n, v), 1e-4);

    let albedo = vec3<f32>(obj.base_color_r, obj.base_color_g, obj.base_color_b);
    let metallic = clamp(obj.metallic, 0.0, 1.0);
    let alpha = max(pow(clamp(obj.roughness, 0.0, 1.0), 2.0), 1e-3);
    let f0 = mix(vec3<f32>(dielectric_f0(obj.reflectance)), albedo, metallic);
    let diffuse_color = albedo * (1.0 - metallic);
    let shadow_origin = p_world + n * 0.01;

    var radiance = vec3<f32>(0.0);
    for (var i = 0u; i < scene.light_count; i = i + 1u) {
        let lgt = lights[i];
        let sample = sample_light(lgt, p_world);
        let l = sample.to_light;
        let n_dot_l = max(dot(n, l), 0.0);
        if (n_dot_l <= 0.0) {
            continue;
        }
        if (all(sample.radiance == vec3<f32>(0.0))) {
            continue;
        }

        var shadow_vis = 1.0;
        if (scene.shadows_enabled != 0u) {
            let shadow_hit = trace(shadow_origin, l, sample.shadow_max_t);
            if (shadow_hit.did_hit) {
                shadow_vis = 0.0;
            }
        }
        if (shadow_vis <= 0.0) {
            continue;
        }

        let h = normalize(l + v);
        let n_dot_h = max(dot(n, h), 0.0);
        let v_dot_h = max(dot(v, h), 0.0);

        let f = fresnel_schlick(f0, v_dot_h);
        let d = ggx_distribution(n_dot_h, alpha);
        let vis = ggx_visibility(n_dot_l, n_dot_v, alpha);
        let specular = f * (d * vis);
        let diffuse = diffuse_color * (vec3<f32>(1.0) - f) / 3.14159265;

        radiance = radiance + (diffuse + specular) * sample.radiance * n_dot_l * shadow_vis;
    }
    let emissive = vec3<f32>(obj.emissive_r, obj.emissive_g, obj.emissive_b);
    return radiance + emissive;
}

// ddgi_ref.rs::octahedral_decode verbatim (this file's own copy).
fn octahedral_decode(uv: vec2<f32>) -> vec3<f32> {
    let f = uv * 2.0 - vec2<f32>(1.0, 1.0);
    var n = vec3<f32>(f.x, f.y, 1.0 - abs(f.x) - abs(f.y));
    let t = max(-n.z, 0.0);
    let sx = select(-1.0, 1.0, n.x >= 0.0);
    let sy = select(-1.0, 1.0, n.y >= 0.0);
    n.x = n.x - t * sx;
    n.y = n.y - t * sy;
    return normalize(n);
}

// radiance_cascades_ref.rs::cascade_probe_ray's own direction mapping —
// ddgi_ref.rs::texel_to_direction reused verbatim (see that Rust
// function's own doc comment: same octahedral mapping DDGI's own probes
// already use, just evaluated at THIS level's own tile resolution).
fn texel_to_direction(x: u32, y: u32, tile_size: u32) -> vec3<f32> {
    let uv = vec2<f32>((f32(x) + 0.5) / f32(tile_size), (f32(y) + 0.5) / f32(tile_size));
    return octahedral_decode(uv);
}

const PROBE_RAY_BIAS: f32 = 0.01;

// radiance_cascades_ref.rs::CascadeGrid::probe_position verbatim.
fn cascade_probe_position(level: CascadeLevelUniform, x: u32, y: u32, z: u32) -> vec3<f32> {
    return level.grid_origin + vec3<f32>(level.probe_spacing) * vec3<f32>(f32(x), f32(y), f32(z));
}

fn cascade_probe_position_flat(level: CascadeLevelUniform, flat_index: u32) -> vec3<f32> {
    let plane = max(level.grid_dims.x * level.grid_dims.y, 1u);
    let z = flat_index / plane;
    let rem = flat_index % plane;
    let y = rem / max(level.grid_dims.x, 1u);
    let x = rem % max(level.grid_dims.x, 1u);
    return cascade_probe_position(level, x, y, z);
}

// Row-major index into the shared `atlas` buffer for texel `(x, y)`
// within level `level`'s own tile-packed region — mirrors
// hybrid_ddgi_relight.wgsl::atlas_index/atlas_tile_origin, offset by this
// level's own `atlas_texel_offset` (see CascadeLevelUniform's own doc
// comment for why all 4 levels share one buffer). Moved above
// relight_cascade_texel (this file's own "functions must be declared
// before use" WGSL convention) since the new bounce-sampling functions
// immediately below need it too.
fn cascade_atlas_index(level: CascadeLevelUniform, probe_flat_index: u32, texel_x: u32, texel_y: u32) -> u32 {
    let tiles_per_row = max(level.tiles_per_row, 1u);
    let row = probe_flat_index / tiles_per_row;
    let col = probe_flat_index % tiles_per_row;
    let tile_origin_x = col * level.tile_size;
    let tile_origin_y = row * level.tile_size;
    let atlas_width = tiles_per_row * level.tile_size;
    let local_index = (tile_origin_y + texel_y) * atlas_width + (tile_origin_x + texel_x);
    return level.atlas_texel_offset + local_index;
}

// hybrid_trace.wgsl::radiance_cascades_probe_irradiance_at verbatim
// (duplicated per this file's own self-containment convention — see this
// file's own header comment) — reads ONE probe's own octahedral tile at
// `direction`, given that probe's own integer grid coords (NOT a world
// position; see cascade_sample_level_trilinear below for the world-space
// entry point that calls this once per trilinear-blended neighbor).
// Split out of the old cascade_nearest_probe_irradiance (which combined
// world->cell snapping with the texel read in one function) so the
// Stage 7 trilinear fix can call this 8 times, once per surrounding
// probe, instead of only ever reading the single nearest one.
fn cascade_probe_irradiance_at(level: CascadeLevelUniform, coords: vec3<u32>, direction: vec3<f32>) -> vec4<f32> {
    let plane = max(level.grid_dims.x * level.grid_dims.y, 1u);
    let probe_flat_index = coords.x + coords.y * max(level.grid_dims.x, 1u) + coords.z * plane;
    if (probe_flat_index >= level.total_probes) {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    var n = direction / (abs(direction.x) + abs(direction.y) + abs(direction.z));
    let octahedral_wrap = vec2<f32>((1.0 - abs(n.y)) * select(-1.0, 1.0, n.x >= 0.0), (1.0 - abs(n.x)) * select(-1.0, 1.0, n.y >= 0.0));
    let n_xy = select(octahedral_wrap, n.xy, n.z >= 0.0);
    let uv = n_xy * 0.5 + vec2<f32>(0.5, 0.5);
    let tile_size = max(level.tile_size, 1u);
    let max_index = f32(tile_size) - 1.0;
    let texel_x = u32(min(uv.x * f32(tile_size), max_index));
    let texel_y = u32(min(uv.y * f32(tile_size), max_index));
    let texel_index = cascade_atlas_index(level, probe_flat_index, texel_x, texel_y);
    return atlas[texel_index];
}

// radiance_cascades_ref.rs::cascade_sample_level_trilinear verbatim:
// trilinearly blends ONE cascade level's own 8 surrounding probes at
// `world_pos` — the Stage 7 spatial-blend fix, layered UNDER the
// existing hemisphere (angular) fix exactly the way
// ddgi_sample_probe_grid_diffuse's own trilinear loop sits under
// ddgi_cosine_weighted_probe_irradiance's hemisphere loop for DDGI (see
// cascade_cosine_weighted_hierarchy_at_hit below, which now calls this
// instead of cascade_nearest_probe_irradiance).
fn cascade_sample_level_trilinear(level: CascadeLevelUniform, world_pos: vec3<f32>, direction: vec3<f32>) -> vec4<f32> {
    let local = (world_pos - level.grid_origin) / max(level.probe_spacing, 1e-4);
    let max_cell = vec3<i32>(max(i32(level.grid_dims.x) - 2, 0), max(i32(level.grid_dims.y) - 2, 0), max(i32(level.grid_dims.z) - 2, 0));
    let cell_x = u32(clamp(i32(floor(max(local.x, 0.0))), 0, max_cell.x));
    let cell_y = u32(clamp(i32(floor(max(local.y, 0.0))), 0, max_cell.y));
    let cell_z = u32(clamp(i32(floor(max(local.z, 0.0))), 0, max_cell.z));
    let frac = vec3<f32>(clamp(local.x - f32(cell_x), 0.0, 1.0), clamp(local.y - f32(cell_y), 0.0, 1.0), clamp(local.z - f32(cell_z), 0.0, 1.0));

    var radiance_acc = vec3<f32>(0.0);
    var transmittance_acc = 0.0;
    for (var dz = 0u; dz < 2u; dz = dz + 1u) {
        for (var dy = 0u; dy < 2u; dy = dy + 1u) {
            for (var dx = 0u; dx < 2u; dx = dx + 1u) {
                let coords = vec3<u32>(cell_x + dx, cell_y + dy, cell_z + dz);
                let wx = select(frac.x, 1.0 - frac.x, dx == 0u);
                let wy = select(frac.y, 1.0 - frac.y, dy == 0u);
                let wz = select(frac.z, 1.0 - frac.z, dz == 0u);
                let weight = wx * wy * wz;
                let sample = cascade_probe_irradiance_at(level, coords, direction);
                radiance_acc = radiance_acc + sample.rgb * weight;
                transmittance_acc = transmittance_acc + sample.a * weight;
            }
        }
    }
    return vec4<f32>(radiance_acc, transmittance_acc);
}

// hybrid_trace.wgsl::merge_cascade_texel verbatim: L_ac = L_ab + beta_ab * L_bc.
fn cascade_merge_texel(near_radiance: vec3<f32>, near_transmittance: f32, far_radiance: vec3<f32>) -> vec3<f32> {
    return near_radiance + near_transmittance * far_radiance;
}

// hybrid_trace.wgsl::radiance_cascades_sample_hierarchy verbatim: walks
// all 4 levels FARTHEST-to-NEAREST, folding each level's own nearest-
// probe sample via cascade_merge_texel. Reads `levels`/`atlas` — THIS
// SAME PASS's own bind group, mid-write this very frame (some levels
// already relit this dispatch, some not yet, depending on GPU scheduling
// order) — the identical "read whatever's currently there, stale or
// fresh" hazard hybrid_ddgi_relight.wgsl's own
// ddgi_sample_probe_grid_diffuse already accepts at relight time (see
// that function's own call site comment). Unlike DDGI, cascades has NO
// temporal accumulation to smooth over a single frame's inconsistency —
// an accepted, already-documented scope cut (this file's own header
// comment: "no rotating subset... unlike DDGI"), not a new problem this
// bounce feature introduces.
fn cascade_sample_hierarchy_at_hit(world_pos: vec3<f32>, direction: vec3<f32>) -> vec3<f32> {
    var accumulated = vec3<f32>(0.0);
    for (var i = 0u; i < LEVEL_COUNT; i = i + 1u) {
        let level_index = LEVEL_COUNT - 1u - i;
        let level = levels[level_index];
        let sample = cascade_sample_level_trilinear(level, world_pos, direction);
        accumulated = cascade_merge_texel(sample.rgb, sample.a, accumulated);
    }
    return accumulated;
}

// hybrid_ddgi_relight.wgsl::ddgi_tangent_basis verbatim (duplicated per
// this file's own self-containment convention) -- builds an orthonormal
// tangent frame with `n` as the Z axis, used below to rotate
// HEMISPHERE_SAMPLES' own local-space directions into world space around
// a surface normal.
fn cascade_tangent_basis(n: vec3<f32>) -> mat3x3<f32> {
    let s = select(-1.0, 1.0, n.z >= 0.0);
    let a = -1.0 / (s + n.z);
    let b = n.x * n.y * a;
    let t = vec3<f32>(1.0 + s * n.x * n.x * a, s * b, -s * n.x);
    let bt = vec3<f32>(b, s + n.y * n.y * a, -n.y);
    return mat3x3<f32>(t, bt, n);
}

// hybrid_ddgi_relight.wgsl::HEMISPHERE_SAMPLES verbatim (same 5-direction
// cosine-weighted set DDGI's own ddgi_cosine_weighted_probe_irradiance
// uses -- see that function's own call site for why this exact constant,
// not a fresh one, is reused here: it's already a proven, tested
// cosine-weighted hemisphere sampling, not something this bounce fix
// needs to re-derive).
const CASCADE_HEMISPHERE_SAMPLE_COUNT: u32 = 5u;
const CASCADE_HEMISPHERE_SAMPLES: array<vec3<f32>, 5> = array<vec3<f32>, 5>(
    vec3<f32>(0.0, 0.0, 1.0),
    vec3<f32>(0.6614, 0.0, 0.75),
    vec3<f32>(-0.2044, 0.6285, 0.75),
    vec3<f32>(-0.5350, -0.3886, 0.75),
    vec3<f32>(0.5350, -0.3886, 0.75),
);

// The actual bounce-dispersion fix: cascade_sample_hierarchy_at_hit on
// its own point-samples the hierarchy at EXACTLY `direction` (one ray
// direction's own discrete angular bin per level) -- correct for
// shading-time viewing rays, but WRONG for a diffuse bounce, which must
// gather incoming light over the WHOLE hemisphere around a surface
// normal (Lambertian: irradiance = integral of incoming radiance over
// the hemisphere), not a single direction. Using the bare normal as
// `direction` confined bounce light to whatever single cascade ray
// happened to point along that exact normal, appearing as light only
// propagating in one direction instead of dispersing -- mirrors
// hybrid_ddgi_relight.wgsl's own ddgi_cosine_weighted_probe_irradiance,
// which exists for the identical reason on the DDGI side.
fn cascade_cosine_weighted_hierarchy_at_hit(world_pos: vec3<f32>, surface_normal: vec3<f32>) -> vec3<f32> {
    let basis = cascade_tangent_basis(surface_normal);
    var acc = vec3<f32>(0.0);
    for (var i = 0u; i < CASCADE_HEMISPHERE_SAMPLE_COUNT; i = i + 1u) {
        let world_dir = normalize(basis * CASCADE_HEMISPHERE_SAMPLES[i]);
        acc = acc + cascade_sample_hierarchy_at_hit(world_pos, world_dir);
    }
    return acc / f32(CASCADE_HEMISPHERE_SAMPLE_COUNT);
}

// radiance_cascades_ref.rs::relight_cascade_texel verbatim: interval-
// clipped trace (fires out to level.interval_far, the SAME t_max-as-
// search-bound convention trace() above already uses), a hit closer than
// interval_near belongs to a NEARER cascade level (reported as a miss for
// THIS level — transmittance=1.0, radiance=0.0 — see the Rust reference's
// own doc comment for why both that case and a genuine miss collapse to
// the identical transmittance value). A genuine hit within
// [interval_near, interval_far) is shaded via shade_direct_only exactly
// like relight_cascade_texel's own call to cpu_ref::shade, reporting
// transmittance=0.0 — PLUS, on that same hit, an indirect bounce term
// sampled from the cascade hierarchy's own current state at the hit
// point (radiance_cascades_ref.rs::relight_cascade_texel's own
// `indirect_at_hit` parameter, ported here as a direct call rather than
// a closure, exactly the way hybrid_ddgi_relight.wgsl's own probe_ray
// ports ddgi_ref.rs::probe_ray's identically-named parameter).
struct CascadeTexelResult {
    radiance: vec3<f32>,
    transmittance: f32,
}

fn relight_cascade_texel(origin: vec3<f32>, direction: vec3<f32>, level: CascadeLevelUniform) -> CascadeTexelResult {
    let hit = trace(origin, direction, level.interval_far);
    if (hit.did_hit && hit.t >= level.interval_near) {
        if (hit.obj_id >= scene.object_count) {
            return CascadeTexelResult(vec3<f32>(0.0), 1.0);
        }
        let obj = objects[hit.obj_id];
        let p_world = origin + hit.t * direction;
        let view_dir = -direction;
        let n = normalize(hit.world_normal);
        let radiance = shade_direct_only(obj, hit.obj_id, p_world, hit.world_normal, view_dir);
        let diffuse_color = vec3<f32>(obj.base_color_r, obj.base_color_g, obj.base_color_b) * (1.0 - clamp(obj.metallic, 0.0, 1.0));
        let indirect = diffuse_color * cascade_cosine_weighted_hierarchy_at_hit(p_world, n);
        return CascadeTexelResult(radiance + indirect, 0.0);
    }
    return CascadeTexelResult(vec3<f32>(0.0), 1.0);
}

// One invocation per (level, probe, texel) triple this frame — ALL
// probes of ALL 4 levels are relit every frame (no rotating "probes per
// frame" subset the way DDGI has — see this file's own header comment:
// an experimental technique scoped small enough that relighting every
// level's own full probe set each frame is affordable, unlike DDGI's
// much larger single-density grid). Dispatch domain: X/Y cover the
// LARGEST level's own atlas_pixels (level 0, most probes/finest tiles),
// Z selects which of the 4 levels (see pass.rs's own dispatch-size
// calculation) — an invocation whose (x, y) falls outside ITS OWN level's
// smaller atlas simply early-returns, wasted but harmless (matches this
// codebase's existing convention of over-dispatching and gating with an
// early bounds check, e.g. ddgi_relight_main's own texel_x/texel_y guard).
@compute @workgroup_size(8, 8, 1)
fn radiance_cascades_relight_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let level_index = gid.z;
    if (level_index >= LEVEL_COUNT) {
        return;
    }
    let level = levels[level_index];
    let tile_size = max(level.tile_size, 1u);
    let atlas_side = max(level.tiles_per_row, 1u) * tile_size;
    if (gid.x >= atlas_side || gid.y >= atlas_side) {
        return;
    }

    let tiles_per_row = max(level.tiles_per_row, 1u);
    let probe_col = gid.x / tile_size;
    let probe_row = gid.y / tile_size;
    let probe_flat_index = probe_row * tiles_per_row + probe_col;
    if (probe_flat_index >= level.total_probes) {
        return;
    }
    let texel_x = gid.x % tile_size;
    let texel_y = gid.y % tile_size;

    let probe_position = cascade_probe_position_flat(level, probe_flat_index);
    let direction = texel_to_direction(texel_x, texel_y, tile_size);
    let origin = probe_position + direction * PROBE_RAY_BIAS;

    let result = relight_cascade_texel(origin, direction, level);

    let texel_index = cascade_atlas_index(level, probe_flat_index, texel_x, texel_y);
    // .a channel carries transmittance (beta in the merge formula) — read
    // back by hybrid_trace.wgsl's own GI_METHOD_RADIANCE_CASCADES branch
    // to walk the hierarchy with merge_cascade_texel at shading time (see
    // this file's own header comment for why merge happens there, not in
    // a separate pass). No temporal accumulation here (unlike DDGI's own
    // clamped-EMA blend) — an explicit scope cut: this experimental
    // technique relights every probe fresh every frame (see this
    // function's own doc comment), so there is no history to blend with;
    // adding temporal smoothing is left to a future iteration if the raw
    // per-frame noise turns out to matter for this A/B comparison.
    atlas[texel_index] = vec4<f32>(result.radiance, result.transmittance);
}
