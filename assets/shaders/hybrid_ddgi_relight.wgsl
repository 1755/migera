// DDGI probe relighting (compute): one invocation per RELIT PROBE TEXEL
// this frame — not per pixel, per probe. Fires one ray from a probe's
// world position along that texel's own octahedral-decoded direction,
// evaluates REAL direct-lit radiance at the hit point (shade, indirect
// disabled — capped at one order of indirection, see this file's own
// "shade_direct_only" doc comment), and blends into a ping-ponged
// irradiance atlas via the same clamped-EMA formula temporal
// accumulation already established for per-pixel history — see
// `src/hybrid/ddgi_ref.rs`'s own module doc comment for the full design
// (this is a faithful WGSL port of that file's `probe_ray`/
// `relight_probe_texel`/`ddgi_probe_relight_start`).
//
// Duplicates hybrid_trace.wgsl's own trace()/march_object()/shade() logic
// rather than importing a shared library — this renderer's own
// established convention throughout src/hybrid is per-pass-file
// self-containment (hybrid_denoise.wgsl/hybrid_temporal.wgsl each carry
// their own SceneUniform copy already); `shade_direct_only` here is
// `shade` with its Stage-B `indirect_enabled` branch removed entirely
// (not just disabled), since a probe ray never has a per-pixel jitter
// seed or a reason to call indirect_diffuse at all.
//
// Dispatched over PROBE-TEXEL count, not pixel count — see
// src/hybrid/pass.rs's own workgroup-count calculation for this pass.
// Runs FIRST in hybrid_pass, before trace_main: trace_main's own
// shading (once DDGI replaces Stage B there) needs to sample an
// ALREADY-relit atlas, the same "read-before-write" ordering constraint
// that already put hybrid_temporal.wgsl between trace and denoise.

// Full mirror of src/hybrid/extract.rs's SceneUniform (field-for-field,
// same convention as hybrid_trace.wgsl/hybrid_temporal.wgsl/
// hybrid_denoise.wgsl/hybrid_blit.wgsl's own copies) — this pass only
// reads object_count/bvh_node_count/light_count/ddgi_*, but the WGSL
// uniform struct's field offsets must match the real buffer layout up to
// the last field it touches, and the simplest way to guarantee that is
// the same full mirror every other file already uses rather than a
// truncated prefix (a truncated prefix here was itself a real bug this
// file had before this comment was written — it silently misaligned
// once reflection/transmission fields were added to the real struct
// after this file was first authored).
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
    // DDGI-specific scalars — see src/hybrid/extract.rs's own DdgiConfig
    // doc comment for what each drives.
    ddgi_probes_per_frame: u32,
    ddgi_total_probes: u32,
    ddgi_tile_size: u32,
    ddgi_frame_index: u32,
    ddgi_max_history_length: f32,
    ddgi_max_t: f32,
    // Unused here — see src/hybrid/extract.rs's SceneUniform's own DOF
    // field doc comments. Mirrored for struct-layout parity only.
    dof_enabled: u32,
    dof_focal_distance: f32,
    dof_aperture_radius: f32,
    dof_frame_index: u32,
    dof_max_history_length: f32,
    // Primary-ray sub-pixel jitter (TAAU experiment step 1a) — unused in
    // this file (probe relight has no primary-ray dependency, see
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

// Mirrors src/hybrid/extract.rs's ObjectGpu exactly — same struct
// hybrid_trace.wgsl declares, duplicated here per this file's own header
// comment (self-contained pass files, not a shared import).
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
    // Unused here (relight only needs direct-lit shading, not
    // transmission) — reuses what src/hybrid/extract.rs's ObjectGpu now
    // calls transmission/ior (previously _pad_material0/_pad_material1
    // padding, no layout change either way).
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

@group(0) @binding(0) var<uniform> scene: SceneUniform;
@group(0) @binding(1) var<storage, read> objects: array<ObjectGpu>;
@group(0) @binding(2) var<storage, read> bvh: array<BvhNode>;
@group(0) @binding(3) var<storage, read> lights: array<LightGpu>;
// The probe atlas — a SINGLE `array<vec4<f32>>` storage buffer (row-
// major, `atlas_pixels * atlas_pixels` texels), read_write, NOT ping-
// ponged. See src/hybrid/pipeline.rs's own HybridDdgiAtlas/
// hybrid_ddgi_layout doc comments for why: each relit probe only ever
// reads/writes its OWN texels (never another probe's), so there's no
// cross-invocation race the way HybridHistory's reprojection genuinely
// has — ping-ponging here was a real bug (probes not relit in a given
// frame lost track of their own last-known-good value across the
// swap, producing a hard two-value stroboscope flicker once the probe
// count exceeded probes_per_frame). `read_write` storage TEXTURES
// aren't available in this codebase, hence a manually-indexed buffer.
@group(0) @binding(4) var<storage, read_write> atlas: array<vec4<f32>>;
// Same row-major indexing as `atlas` above — per-texel `(mean_distance,
// mean_distance_squared)`, the Chebyshev depth-visibility test's own
// input (see `src/hybrid/pipeline.rs`'s own `HybridDdgiAtlas::
// distance_atlas` doc comment for the full rationale: a single hard
// occlusion ray fired from the SHADED POINT's own side only catches
// "something sits between the shaded point and the probe," not "this
// probe's own stored irradiance is itself unreliable because its
// relight rays skimmed through/near thin geometry from the probe's own
// vantage point"). A SEPARATE buffer from `atlas`, not a wider per-texel
// stride, for the identical reason that doc comment gives.
@group(0) @binding(5) var<storage, read_write> distance_atlas: array<vec2<f32>>;
// Per-probe accumulated-history-length — one scalar per PROBE, not per
// texel (every texel in a probe's own tile shares the same
// history_length, since they're all relit together whenever that
// probe's turn in the rotation comes up). Also a single read_write
// buffer, not ping-ponged, for the identical reason the atlas above
// isn't.
@group(0) @binding(6) var<storage, read_write> probe_history_length: array<f32>;

// --- cpu_ref.rs's named constants, ported verbatim (same values
// hybrid_trace.wgsl already uses — kept in sync by hand, no shared
// import, per this file's own header comment). ------------------------

const MAX_MARCH_STEPS: u32 = 128u;
const HIT_EPSILON: f32 = 1e-4;
const LEAF_SENTINEL: u32 = 4294967295u; // u32::MAX
const MAX_STACK: u32 = 64u;

// --- cpu_ref.rs's sd_* functions, verbatim (identical to
// hybrid_trace.wgsl's own copies). --------------------------------------

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

fn quat_conjugate(q: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(-q.xyz, q.w);
}

struct MarchResult {
    hit: bool,
    t: f32,
}

// cpu_ref.rs::pixel_eps/shadow_bias verbatim. Relocated above march_object
// (WGSL requires declaration before use) since march_object's own
// convergence tolerance now needs pixel_eps too -- see march_object's doc
// comment just below.
fn pixel_eps(t: f32) -> f32 {
    return max(t * 0.0016, 2e-4);
}

fn shadow_bias(hit_t: f32) -> f32 {
    return max(pixel_eps(hit_t) * 2.0, 0.01);
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
                best_normal = rotate_by_quat(quat_conjugate(inv_rotation), local_n);
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

// hybrid_trace.wgsl::any_hit verbatim (that file's own copy carries the
// full rationale doc comment — this file duplicates the definition, not
// the explanation, per this module's own established "duplicate small
// formulas across files" convention). "Does anything other than
// exclude_obj_id converge somewhere in [0, t_max] along this ray?" — used
// by this file's own DDGI hard-occlusion check below, which only ever
// reads a boolean and never needed trace()'s full nearest-hit result.
fn any_hit(ray_origin: vec3<f32>, ray_dir: vec3<f32>, t_max: f32, exclude_obj_id: u32) -> bool {
    if (scene.bvh_node_count == 0u) {
        return false;
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

        let hit = slab_hit(ray_origin, ray_dir, vec3<f32>(node.min_x, node.min_y, node.min_z),
            vec3<f32>(node.max_x, node.max_y, node.max_z), t_max);
        let t_near = hit.x;
        let t_far = hit.y;
        if (t_near > t_far) {
            continue;
        }

        if (node.left_or_sentinel == LEAF_SENTINEL) {
            let obj_id = node.right_or_object;
            if (obj_id >= scene.object_count || obj_id == exclude_obj_id) {
                continue;
            }
            let object = objects[obj_id];
            let march = march_object(object, ray_origin, ray_dir, t_near, t_far);
            if (march.hit) {
                return true;
            }
            continue;
        }

        let left = node.left_or_sentinel;
        let right = node.right_or_object;
        let left_node = bvh[left];
        let right_node = bvh[right];
        let left_hit = slab_hit(ray_origin, ray_dir, vec3<f32>(left_node.min_x, left_node.min_y, left_node.min_z),
            vec3<f32>(left_node.max_x, left_node.max_y, left_node.max_z), t_max);
        let right_hit = slab_hit(ray_origin, ray_dir, vec3<f32>(right_node.min_x, right_node.min_y, right_node.min_z),
            vec3<f32>(right_node.max_x, right_node.max_y, right_node.max_z), t_max);
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

    return false;
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

const VIS_CUTOFF: f32 = 0.02;
const DIVERGENCE_FACTOR: f32 = 2.5;
const MAX_SHADOW_CANDIDATES: u32 = 64u;
const PENUMBRA_REACH: f32 = 15.75;

fn shadow_candidate_margin(k: f32) -> f32 {
    return VIS_CUTOFF * k * PENUMBRA_REACH;
}

fn gather_candidates_padded(
    ray_origin: vec3<f32>, ray_dir: vec3<f32>, t_min: f32, t_max: f32, margin: f32,
    out_ids: ptr<function, array<u32, MAX_SHADOW_CANDIDATES>>,
    out_near: ptr<function, array<f32, MAX_SHADOW_CANDIDATES>>,
) -> u32 {
    var count = 0u;
    if (scene.bvh_node_count == 0u) {
        return 0u;
    }
    var stack: array<u32, MAX_STACK>;
    var sp: u32 = 0u;
    stack[0] = 0u;
    sp = 1u;
    loop {
        if (sp == 0u || count >= MAX_SHADOW_CANDIDATES) {
            break;
        }
        sp = sp - 1u;
        let node_index = stack[sp];
        let node = bvh[node_index];
        let is_leaf = node.left_or_sentinel == LEAF_SENTINEL;
        let bmin = vec3<f32>(node.min_x, node.min_y, node.min_z) - vec3<f32>(margin);
        let bmax = vec3<f32>(node.max_x, node.max_y, node.max_z) + vec3<f32>(margin);
        let hit = slab_hit(ray_origin, ray_dir, bmin, bmax, t_max);
        let near = hit.x;
        let far = hit.y;
        if (near > far || far < t_min) {
            continue;
        }
        if (is_leaf) {
            let obj_id = node.right_or_object;
            var already = false;
            for (var i = 0u; i < count; i = i + 1u) {
                if ((*out_ids)[i] == obj_id) {
                    already = true;
                }
            }
            if (!already) {
                (*out_ids)[count] = obj_id;
                (*out_near)[count] = max(near, t_min);
                count = count + 1u;
            }
            continue;
        }
        if (sp + 2u <= MAX_STACK) {
            stack[sp] = node.left_or_sentinel;
            sp = sp + 1u;
            stack[sp] = node.right_or_object;
            sp = sp + 1u;
        }
    }
    return count;
}

struct ShadowResult {
    hard_hit: bool,
    vis: f32,
}

fn trace_shadow(
    ray_origin: vec3<f32>, ray_dir: vec3<f32>, max_t: f32, k: f32, has_origin: bool, origin_obj_id: u32,
) -> ShadowResult {
    let margin = shadow_candidate_margin(k);
    var cand_ids: array<u32, MAX_SHADOW_CANDIDATES>;
    var cand_near: array<f32, MAX_SHADOW_CANDIDATES>;
    let cand_count = gather_candidates_padded(ray_origin, ray_dir, 0.001, max_t, margin, &cand_ids, &cand_near);

    var vis = 1.0;
    for (var ci = 0u; ci < cand_count; ci = ci + 1u) {
        if (vis < VIS_CUTOFF) {
            return ShadowResult(false, 0.0);
        }
        let obj_id = cand_ids[ci];
        if (has_origin && obj_id == origin_obj_id) {
            continue;
        }
        if (obj_id >= scene.object_count) {
            continue;
        }
        let object = objects[obj_id];
        var t = max(cand_near[ci], 0.01);
        var ph = 1e20;
        var prev_step = 1e20;
        var best_h = 3.4e38;
        loop {
            if (!(t <= max_t && vis > VIS_CUTOFF)) {
                break;
            }
            let inv_rotation = vec4<f32>(object.inv_rotation_x, object.inv_rotation_y, object.inv_rotation_z, object.inv_rotation_w);
            let translation = vec3<f32>(object.translation_x, object.translation_y, object.translation_z);
            let p_world = ray_origin + ray_dir * t;
            let p_local = rotate_by_quat(inv_rotation, p_world - translation);
            let h = local_distance(object, p_local);
            if (h < HIT_EPSILON) {
                return ShadowResult(true, 0.0);
            }
            if (h < best_h) {
                best_h = h;
            } else if (h > best_h * DIVERGENCE_FACTOR) {
                break;
            }
            let y = min(h * h / (2.0 * ph), prev_step);
            let d = sqrt(max(h * h - y * y, 0.0));
            let raw_vis = clamp(d / (k * max(t - y, 1e-4)), 0.0, 1.0);
            let margin_fade = smoothstep(margin * 0.5, margin, h);
            let faded_vis = raw_vis + (1.0 - raw_vis) * margin_fade;
            vis = min(vis, faded_vis);
            // cpu_ref.rs::trace_shadow's own margin_fade/VIS_CUTOFF
            // interaction fix verbatim (mirrors hybrid_trace.wgsl's own
            // identical fix, per this file's own self-containment
            // convention) — see that function's own comment for the full
            // writeup.
            if (vis <= VIS_CUTOFF) {
                return ShadowResult(false, 0.0);
            }
            ph = h;
            var step = h * 1.2;
            if (t + step > max_t) {
                step = h;
                if (t + step > max_t) {
                    break;
                }
            }
            prev_step = step;
            t = t + step;
        }
    }
    return ShadowResult(false, vis);
}

// cpu_ref.rs::shade verbatim MINUS the indirect_enabled/indirect_diffuse
// branch — ddgi_ref.rs::probe_ray's own CPU reference calls the real
// `shade` with `indirect_enabled: false`, which is functionally
// identical to this function (the branch is skipped either way); this
// WGSL mirror removes the dead branch entirely rather than porting an
// always-false gate, since a probe ray has no pixel-jitter seed and
// this pass never needs the toggle. Returns direct_and_emissive-only
// radiance (no ShadeResult split needed — DDGI has no separate noisy
// per-pixel term to keep out of a blur the way Stage B's own split
// existed for).
fn shade_direct_only(obj: ObjectGpu, obj_id: u32, p_world: vec3<f32>, world_normal: vec3<f32>, view_dir: vec3<f32>, hit_t: f32) -> vec3<f32> {
    let n = normalize(world_normal);
    let v = normalize(view_dir);
    let n_dot_v = max(dot(n, v), 1e-4);

    let albedo = vec3<f32>(obj.base_color_r, obj.base_color_g, obj.base_color_b);
    let metallic = clamp(obj.metallic, 0.0, 1.0);
    let alpha = max(pow(clamp(obj.roughness, 0.0, 1.0), 2.0), 1e-3);
    let f0 = mix(vec3<f32>(dielectric_f0(obj.reflectance)), albedo, metallic);
    let diffuse_color = albedo * (1.0 - metallic);
    let shadow_origin = p_world + n * shadow_bias(hit_t);

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
            let shadow = trace_shadow(shadow_origin, l, sample.shadow_max_t, lgt.shadow_softness_k, true, obj_id);
            shadow_vis = select(shadow.vis, 0.0, shadow.hard_hit);
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

// --- ddgi_ref.rs::octahedral_decode verbatim (this file's own copy —
// see this file's header comment for why not a shared import). ---------

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

// ddgi_ref.rs::texel_to_direction verbatim.
fn texel_to_direction(x: u32, y: u32, tile_size: u32) -> vec3<f32> {
    let uv = vec2<f32>((f32(x) + 0.5) / f32(tile_size), (f32(y) + 0.5) / f32(tile_size));
    return octahedral_decode(uv);
}

// ddgi_ref.rs::ddgi_probe_relight_start verbatim.
fn ddgi_probe_relight_start(probes_per_frame: u32, total_probes: u32, frame_index: u32) -> u32 {
    let total = max(total_probes, 1u);
    let per_frame = clamp(probes_per_frame, 1u, total);
    return (frame_index * per_frame) % total;
}

// Grid placement + atlas layout scalars, mirrors ddgi_ref.rs::ProbeGrid
// (origin/spacing/dims) and ddgi_ref.rs::AtlasLayout (tiles_per_row) —
// kept as their own small uniform rather than folded into SceneUniform
// since these are GRID-CONFIG-scoped (change only when the grid itself
// is rebuilt), not per-frame scalars like SceneUniform's own
// ddgi_frame_index. tiles_per_row is `ceil(sqrt(total_probes))`,
// computed ONCE on the Rust side by AtlasLayout::exact_fit — NOT
// recomputed here (a hardcoded/guessed value was a real mistake caught
// during this file's own first draft, before it ever ran). Declared
// ahead of probe_ray (unlike its original position after probe_ray) —
// WGSL requires functions/types be declared before use, and probe_ray
// now needs atlas_index/atlas_tile_origin/probe_position for its own
// infinite-bounce grid sample below.
struct DdgiGridUniform {
    origin: vec3<f32>,
    _pad0: f32,
    spacing: vec3<f32>,
    tiles_per_row: u32,
    dims: vec3<u32>,
    _pad2: u32,
}

@group(0) @binding(7) var<uniform> ddgi_grid: DdgiGridUniform;

// ddgi_ref.rs::ProbeGrid::probe_position_flat verbatim.
fn probe_position(flat_index: u32) -> vec3<f32> {
    let dims = ddgi_grid.dims;
    let plane = max(dims.x * dims.y, 1u);
    let z = flat_index / plane;
    let rem = flat_index % plane;
    let y = rem / max(dims.x, 1u);
    let x = rem % max(dims.x, 1u);
    return ddgi_grid.origin + ddgi_grid.spacing * vec3<f32>(f32(x), f32(y), f32(z));
}

// ddgi_ref.rs::AtlasLayout::tile_origin verbatim.
fn atlas_tile_origin(flat_index: u32, tile_size: u32) -> vec2<i32> {
    let tiles_per_row = max(ddgi_grid.tiles_per_row, 1u);
    let row = flat_index / tiles_per_row;
    let col = flat_index % tiles_per_row;
    return vec2<i32>(i32(col * tile_size), i32(row * tile_size));
}

// Row-major index into the `atlas` storage buffer for a given texel
// coordinate — the atlas is square, `tiles_per_row * tile_size` texels
// per side (see AtlasLayout::exact_fit's own doc comment), so the row
// stride is derivable from ddgi_grid.tiles_per_row and scene.ddgi_tile_size
// without a separate atlas_pixels uniform field.
fn atlas_index(texel: vec2<i32>) -> u32 {
    let atlas_width = i32(max(ddgi_grid.tiles_per_row, 1u) * scene.ddgi_tile_size);
    return u32(texel.y * atlas_width + texel.x);
}

// ---------------------------------------------------------------------------------
// Infinite-bounce sampling: reads the atlas/distance_atlas this SAME
// pass is writing (read_write — see this file's own header comment:
// each invocation only ever WRITES its own texel, never another
// probe's) to answer "what does the grid ALREADY know about the light
// arriving at this probe ray's own hit point". **Known, accepted race:**
// unlike the write side, this read side is NOT restricted to a probe's
// own texel — a probe relighting this frame can read a neighboring
// probe's texel while THAT probe's own invocation (same dispatch,
// unordered) is mid-write to it, when probes_per_frame is large enough
// for two nearby probes to both be in this frame's relit set. This is a
// stale/torn-value race, not a memory-safety one (GPUs don't fault on
// concurrent storage-buffer read/write to the same word) — worst case a
// sample reads last frame's value instead of this frame's, which is
// still a legitimate, temporally-close irradiance estimate either way.
// RTXGI's own published reference implementation has this identical
// characteristic with its own in-place-updated probe buffer and
// documents it as an accepted approximation, not a bug to fix — no
// double-buffering added here for the same reason. A faithful port of
// hybrid_trace.wgsl::ddgi_sample_probe_grid, duplicated here (not
// shared) per this file's own header comment's established convention.
// Fed into probe_ray below as the term that makes bounce light actually
// propagate probe-to-probe across frames: a probe relighting this frame
// samples its neighbors' LAST-KNOWN irradiance at its own hit point,
// which itself already contains a sample of ITS neighbors from the
// frame before, and so on — RTXGI's own published "infinite bounce"
// trick, no extra rays or storage, just reusing the grid that already
// exists. Uses full-diffuse (roughness=1.0) hemisphere spread — a
// probe ray's hit point is being treated as a generic Lambertian
// bounce surface for gathering incoming light, the same convention
// hybrid_trace.wgsl::shade already applies via diffuse_color for its
// own direct-light indirect term, not the (specular-relevant) roughness
// of whatever object the ray actually hit.
// ---------------------------------------------------------------------------------

fn ddgi_tangent_basis(n: vec3<f32>) -> mat3x3<f32> {
    let s = select(-1.0, 1.0, n.z >= 0.0);
    let a = -1.0 / (s + n.z);
    let b = n.x * n.y * a;
    let t = vec3<f32>(1.0 + s * n.x * n.x * a, s * b, -s * n.x);
    let bt = vec3<f32>(b, s + n.y * n.y * a, -n.y);
    return mat3x3<f32>(t, bt, n);
}

const HEMISPHERE_SAMPLE_COUNT: u32 = 5u;
const HEMISPHERE_SAMPLES: array<vec3<f32>, 5> = array<vec3<f32>, 5>(
    vec3<f32>(0.0, 0.0, 1.0),
    vec3<f32>(0.6614, 0.0, 0.75),
    vec3<f32>(-0.2044, 0.6285, 0.75),
    vec3<f32>(-0.5350, -0.3886, 0.75),
    vec3<f32>(0.5350, -0.3886, 0.75),
);

fn ddgi_probe_irradiance(flat_index: u32, direction: vec3<f32>, tile_size: u32) -> vec3<f32> {
    var n = direction / (abs(direction.x) + abs(direction.y) + abs(direction.z));
    let octahedral_wrap = vec2<f32>((1.0 - abs(n.y)) * select(-1.0, 1.0, n.x >= 0.0), (1.0 - abs(n.x)) * select(-1.0, 1.0, n.y >= 0.0));
    let n_xy = select(octahedral_wrap, n.xy, n.z >= 0.0);
    let uv = n_xy * 0.5 + vec2<f32>(0.5, 0.5);
    let max_index = f32(tile_size) - 1.0;
    let texel_x = i32(min(uv.x * f32(tile_size), max_index));
    let texel_y = i32(min(uv.y * f32(tile_size), max_index));
    let tile_origin = atlas_tile_origin(flat_index, tile_size);
    let texel_index = atlas_index(tile_origin + vec2<i32>(texel_x, texel_y));
    return atlas[texel_index].rgb;
}

fn ddgi_probe_distance_moments(flat_index: u32, direction: vec3<f32>, tile_size: u32) -> vec2<f32> {
    var n = direction / (abs(direction.x) + abs(direction.y) + abs(direction.z));
    let octahedral_wrap = vec2<f32>((1.0 - abs(n.y)) * select(-1.0, 1.0, n.x >= 0.0), (1.0 - abs(n.x)) * select(-1.0, 1.0, n.y >= 0.0));
    let n_xy = select(octahedral_wrap, n.xy, n.z >= 0.0);
    let uv = n_xy * 0.5 + vec2<f32>(0.5, 0.5);
    let max_index = f32(tile_size) - 1.0;
    let texel_x = i32(min(uv.x * f32(tile_size), max_index));
    let texel_y = i32(min(uv.y * f32(tile_size), max_index));
    let tile_origin = atlas_tile_origin(flat_index, tile_size);
    let texel_index = atlas_index(tile_origin + vec2<i32>(texel_x, texel_y));
    return distance_atlas[texel_index];
}

fn ddgi_cosine_weighted_probe_irradiance(flat_index: u32, surface_normal: vec3<f32>, tile_size: u32) -> vec3<f32> {
    let basis = ddgi_tangent_basis(surface_normal);
    var acc = vec3<f32>(0.0);
    for (var i = 0u; i < HEMISPHERE_SAMPLE_COUNT; i = i + 1u) {
        let world_dir = normalize(basis * HEMISPHERE_SAMPLES[i]);
        acc = acc + ddgi_probe_irradiance(flat_index, world_dir, tile_size);
    }
    return acc / f32(HEMISPHERE_SAMPLE_COUNT);
}

fn ddgi_cosine_weighted_probe_distance_moments(flat_index: u32, surface_normal: vec3<f32>, tile_size: u32) -> vec2<f32> {
    let basis = ddgi_tangent_basis(surface_normal);
    var acc = vec2<f32>(0.0, 0.0);
    for (var i = 0u; i < HEMISPHERE_SAMPLE_COUNT; i = i + 1u) {
        let world_dir = normalize(basis * HEMISPHERE_SAMPLES[i]);
        acc = acc + ddgi_probe_distance_moments(flat_index, world_dir, tile_size);
    }
    return acc / f32(HEMISPHERE_SAMPLE_COUNT);
}

const CHEBYSHEV_VARIANCE_FLOOR: f32 = 1e-3;

fn chebyshev_visibility_weight(dist: f32, mean: f32, mean_sq: f32) -> f32 {
    if (dist <= mean) {
        return 1.0;
    }
    let variance = max(mean_sq - mean * mean, CHEBYSHEV_VARIANCE_FLOOR);
    let delta = dist - mean;
    return variance / (variance + delta * delta);
}

fn ddgi_probe_grid_cell(world_pos: vec3<f32>) -> vec3<u32> {
    let local = (world_pos - ddgi_grid.origin) / ddgi_grid.spacing;
    let max_cell = vec3<i32>(i32(ddgi_grid.dims.x) - 2, i32(ddgi_grid.dims.y) - 2, i32(ddgi_grid.dims.z) - 2);
    let cell_x = clamp(i32(floor(max(local.x, 0.0))), 0, max(max_cell.x, 0));
    let cell_y = clamp(i32(floor(max(local.y, 0.0))), 0, max(max_cell.y, 0));
    let cell_z = clamp(i32(floor(max(local.z, 0.0))), 0, max(max_cell.z, 0));
    return vec3<u32>(u32(cell_x), u32(cell_y), u32(cell_z));
}

fn ddgi_probe_flat_index(x: u32, y: u32, z: u32) -> u32 {
    return x + y * ddgi_grid.dims.x + z * ddgi_grid.dims.x * ddgi_grid.dims.y;
}

const DDGI_FALLBACK_RAMP_WEIGHT: f32 = 0.2;

// Faithful port of ddgi_ref.rs::sample_probe_grid / hybrid_trace.wgsl::
// ddgi_sample_probe_grid, fixed at roughness=1.0 (full diffuse spread —
// see this section's own header comment for why a probe ray's hit point
// always gathers as a Lambertian surface regardless of what it hit).
// `origin_obj_id` excludes the ray's own hit object from its own
// occlusion query, mirroring every other self-intersection exclusion in
// this codebase.
fn ddgi_sample_probe_grid_diffuse(world_pos: vec3<f32>, surface_normal: vec3<f32>, tile_size: u32, origin_obj_id: u32) -> vec3<f32> {
    let cell = ddgi_probe_grid_cell(world_pos);
    let frac = vec3<f32>(
        clamp((world_pos.x - ddgi_grid.origin.x) / max(ddgi_grid.spacing.x, 1e-4) - f32(cell.x), 0.0, 1.0),
        clamp((world_pos.y - ddgi_grid.origin.y) / max(ddgi_grid.spacing.y, 1e-4) - f32(cell.y), 0.0, 1.0),
        clamp((world_pos.z - ddgi_grid.origin.z) / max(ddgi_grid.spacing.z, 1e-4) - f32(cell.z), 0.0, 1.0),
    );

    var acc = vec3<f32>(0.0);
    var weight_sum = 0.0;
    var raw_sum = vec3<f32>(0.0);
    var raw_count = 0.0;
    let bias = 0.01;
    for (var dz = 0u; dz < 2u; dz = dz + 1u) {
        for (var dy = 0u; dy < 2u; dy = dy + 1u) {
            for (var dx = 0u; dx < 2u; dx = dx + 1u) {
                let cx = cell.x + dx;
                let cy = cell.y + dy;
                let cz = cell.z + dz;
                let probe_pos = probe_position(ddgi_probe_flat_index(cx, cy, cz));
                let flat_index = ddgi_probe_flat_index(cx, cy, cz);

                let wx = select(frac.x, 1.0 - frac.x, dx == 0u);
                let wy = select(frac.y, 1.0 - frac.y, dy == 0u);
                let wz = select(frac.z, 1.0 - frac.z, dz == 0u);
                let trilinear_weight = wx * wy * wz;

                let irradiance = ddgi_cosine_weighted_probe_irradiance(flat_index, surface_normal, tile_size);
                raw_sum = raw_sum + irradiance;
                raw_count = raw_count + 1.0;

                let to_probe = probe_pos - world_pos;
                let dist = length(to_probe);
                var hard_occluded = false;
                var chebyshev_weight = 1.0;
                if (dist > 1e-4) {
                    hard_occluded = any_hit(world_pos + surface_normal * bias, to_probe / dist, dist - bias, origin_obj_id);
                    let moments = ddgi_cosine_weighted_probe_distance_moments(flat_index, surface_normal, tile_size);
                    chebyshev_weight = chebyshev_visibility_weight(dist, moments.x, moments.y);
                }
                let combined_weight = select(chebyshev_weight, 0.0, hard_occluded);
                if (combined_weight > 0.0) {
                    acc = acc + irradiance * trilinear_weight * combined_weight;
                    weight_sum = weight_sum + trilinear_weight * combined_weight;
                }
            }
        }
    }
    if (raw_count < 0.5) {
        return vec3<f32>(0.0);
    }
    let fallback = raw_sum / raw_count;
    let t = clamp(weight_sum / DDGI_FALLBACK_RAMP_WEIGHT, 0.0, 1.0);
    let mix_amount = t * t * (3.0 - 2.0 * t);
    let weighted = acc / max(weight_sum, 1e-6);
    return mix(fallback, weighted, mix_amount);
}

// ddgi_ref.rs::probe_ray verbatim (calls shade_direct_only in place of
// the real shade — see that function's own doc comment for why they're
// functionally identical here). A miss reports exactly black, NOT a
// mocked sky gradient — see ddgi_ref.rs::probe_ray's own doc comment
// (Rust side) for why: trusting a miss as real sky is the same class of
// light-leak bug conetrace_ref.rs's own coverage^2 fix addressed, and a
// probe embedded near/inside solid geometry (a sealed room's own walls)
// must not bake a bright fake-sky value into its stored irradiance.
//
// On a hit, additionally samples ddgi_sample_probe_grid_diffuse at the
// hit point and adds diffuse_color * that sample on top of direct
// light — ddgi_ref.rs::probe_ray's own `indirect_at_hit` parameter,
// ported here as a direct call rather than a closure (WGSL has none) —
// see this section's own header comment for why this one addition is
// the whole infinite-bounce mechanism.
const PROBE_RAY_BIAS: f32 = 0.01;

// ddgi_ref.rs::probe_ray's own return shape ((Vec3, f32) — radiance and
// hit distance) as a struct, since WGSL has no tuple return type.
// `distance` feeds distance_atlas's own Chebyshev depth-visibility
// input — see ddgi_ref.rs::probe_ray's own doc comment for why a miss
// reports `max_t` (the farthest a real ray could have found nothing),
// not a sentinel.
struct ProbeRayResult {
    radiance: vec3<f32>,
    distance: f32,
}

fn probe_ray(origin: vec3<f32>, direction: vec3<f32>, max_t: f32, tile_size: u32) -> ProbeRayResult {
    let hit = trace(origin, direction, max_t);
    if (hit.did_hit) {
        let obj = objects[hit.obj_id];
        let p_world = origin + hit.t * direction;
        let view_dir = -direction;
        let n = normalize(hit.world_normal);
        let radiance = shade_direct_only(obj, hit.obj_id, p_world, hit.world_normal, view_dir, hit.t);
        let diffuse_color = vec3<f32>(obj.base_color_r, obj.base_color_g, obj.base_color_b) * (1.0 - clamp(obj.metallic, 0.0, 1.0));
        let indirect = diffuse_color * ddgi_sample_probe_grid_diffuse(p_world, n, tile_size, hit.obj_id);
        return ProbeRayResult(radiance + indirect, hit.t);
    }
    return ProbeRayResult(vec3<f32>(0.0), max_t);
}

// temporal_ref.rs::temporal_blend verbatim — same clamped-EMA formula
// hybrid_temporal.wgsl already ports, reused here at probe-texel
// granularity (see ddgi_ref.rs::relight_probe_texel's own doc comment).
fn temporal_blend(current: vec3<f32>, history: vec3<f32>, history_length: f32, max_history_length: f32) -> vec4<f32> {
    let new_length = min(history_length + 1.0, max(max_history_length, 1.0));
    let alpha = 1.0 / new_length;
    let blended = mix(history, current, alpha);
    return vec4<f32>(blended, new_length);
}

// One invocation per (relit probe, texel) pair this frame — workgroup
// domain is [ddgi_probes_per_frame * ddgi_tile_size, ddgi_tile_size],
// see src/hybrid/pass.rs's own dispatch-size calculation for this pass;
// gid.x encodes both "which of this frame's relit probes" (high bits)
// and "which texel column" (low bits, mod tile_size) so a single 2D
// dispatch covers every relit probe's own full tile in one pass, rather
// than one dispatch per probe.
@compute @workgroup_size(8, 8, 1)
fn ddgi_relight_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let tile_size = scene.ddgi_tile_size;
    let probe_slot = gid.x / tile_size; // which of THIS frame's relit probes, 0..ddgi_probes_per_frame
    let texel_x = gid.x % tile_size;
    let texel_y = gid.y;
    if (probe_slot >= scene.ddgi_probes_per_frame || texel_x >= tile_size || texel_y >= tile_size) {
        return;
    }

    let start = ddgi_probe_relight_start(scene.ddgi_probes_per_frame, scene.ddgi_total_probes, scene.ddgi_frame_index);
    let probe_index = (start + probe_slot) % scene.ddgi_total_probes;
    let pos = probe_position(probe_index);

    let direction = texel_to_direction(texel_x, texel_y, tile_size);
    let current = probe_ray(pos + direction * PROBE_RAY_BIAS, direction, scene.ddgi_max_t, tile_size);

    let tile_origin = atlas_tile_origin(probe_index, tile_size);
    let atlas_texel = tile_origin + vec2<i32>(i32(texel_x), i32(texel_y));
    let texel_index = atlas_index(atlas_texel);
    let history = atlas[texel_index].rgb;
    let history_length = probe_history_length[probe_index];

    let result = temporal_blend(current.radiance, history, history_length, scene.ddgi_max_history_length);
    atlas[texel_index] = vec4<f32>(result.rgb, 1.0);
    probe_history_length[probe_index] = result.w;

    // Distance moments blended via the SAME temporal_blend formula,
    // packed (mean_distance, mean_distance_squared, 0.0) into its Vec3
    // shape purely to reuse it without a new 2-component variant — see
    // ddgi_ref.rs::relight_probe_texel's own doc comment. Uses the SAME
    // history_length as irradiance above (both are relit by the exact
    // same ray on the exact same schedule, so there is only one real
    // "how many samples has this texel accumulated" answer).
    let history_moments = vec3<f32>(distance_atlas[texel_index], 0.0);
    let current_moments = vec3<f32>(current.distance, current.distance * current.distance, 0.0);
    let moments_result = temporal_blend(current_moments, history_moments, history_length, scene.ddgi_max_history_length);
    distance_atlas[texel_index] = moments_result.xy;
}
