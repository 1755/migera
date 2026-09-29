// Stochastic (jittered-lens) depth of field (compute): one invocation per
// pixel, run AFTER hybrid_denoise.wgsl's sharp/composited color is ready
// and BEFORE hybrid_blit.wgsl. See src/hybrid/dof_ref.rs's own module doc
// comment for the full technique and — importantly — WHY this fires a
// SEPARATE ray rather than perturbing hybrid_trace.wgsl's own primary
// ray: that ray's hit point/depth feeds Bevy's own depth test and every
// existing temporal accumulator (GI, reflection, refraction), none of
// which can distinguish "lens aperture jitter" from a real disocclusion.
//
// Per pixel, per frame:
//   1. Fire a SEPARATE ray from a simulated aperture-disk offset,
//      re-aimed through the same focal-plane point the sharp (unjittered)
//      primary ray hit — dof_ref.rs::dof_jittered_ray.
//   2. Trace it against the SAME scene geometry (this file's own
//      self-contained trace(), duplicated per this codebase's established
//      per-pass-file convention — see hybrid_ddgi_relight.wgsl's own
//      header comment for the identical reasoning).
//   3. Reproject the jittered ray's own hit point back through the
//      CURRENT (unperturbed) camera's clip_from_world to find which
//      pixel of the ALREADY-SHADED sharp image to resample color from —
//      dof_ref.rs::dof_resample_uv. A miss (or a point that reprojects
//      behind the camera) falls back to this pixel's own sharp color
//      unchanged (see dof_resample_uv's own doc comment).
//   4. Temporally accumulate the resampled color via a DEDICATED history
//      (own ping-pong buffer, own convergence window
//      scene.dof_max_history_length) — gated by disocclusion-rejection
//      against the SHARP pixel's own depth/normal (from depth_tex/
//      normal_tex, computed by the UNPERTURBED primary ray, completely
//      unaffected by DOF jitter), not the jittered ray's own hit. This is
//      what correctly distinguishes "the lens sampled a different point
//      this frame" (expected, should still accumulate) from "the camera/
//      object actually moved" (a real disocclusion, history must reset).
//
// scene.dof_enabled == 0 skips steps 1-3 entirely and copies the sharp
// color straight through (history reset to 0, matching every other
// technique-disabled pass in this codebase's own "cheap copy-through,
// not a conditional dispatch skip" convention).

#import bevy_render::view::{View, frag_coord_to_uv, uv_to_ndc}

@group(0) @binding(0) var<uniform> view: View;

// Full mirror of src/hybrid/extract.rs's SceneUniform — see
// hybrid_ddgi_relight.wgsl's own header comment for why a full mirror,
// not a truncated prefix.
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
    // Primary-ray sub-pixel jitter (TAAU experiment step 1a) — see
    // src/hybrid/taa_ref.rs's own module doc comment. Mirrored field-for-
    // field from src/hybrid/extract.rs's SceneUniform; this file's own
    // generate_primary_ray must apply the identical offset hybrid_trace.
    // wgsl's copy does, or DOF's resample target will disagree with the
    // sharp image's own traced geometry.
    jitter_enabled: u32,
    jitter_offset_x: f32,
    jitter_offset_y: f32,
    // This pass's own working resolution (RenderScaleConfig experiment,
    // step 1b) — see src/hybrid/extract.rs's SceneUniform::trace_size_x
    // doc comment. Used for this file's dispatch-bounds check and ray-gen's
    // own UV conversion (dof_main's own textureDimensions(sharp_color_tex)
    // read already resolves to the same value independently, since that
    // texture is itself sized to trace_size — kept as a cross-check, not a
    // second source of truth).
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

@group(1) @binding(0) var<uniform> scene: SceneUniform;
@group(1) @binding(1) var<storage, read> objects: array<ObjectGpu>;
@group(1) @binding(2) var<storage, read> bvh: array<BvhNode>;

// This frame's already-shaded/composited sharp image (hybrid_denoise.
// wgsl's own denoised_color_out) — sampled (not storage-loaded) so the
// resample step gets real bilinear filtering at the reprojected UV, not
// a single texel snap. depth_tex/normal_tex are the SAME sharp/
// unperturbed textures hybrid_temporal.wgsl already reads — used here
// ONLY for this pass's own disocclusion test against the DOF history,
// never perturbed by the jittered ray.
@group(1) @binding(3) var sharp_color_tex: texture_2d<f32>;
@group(1) @binding(4) var sharp_color_sampler: sampler;
@group(1) @binding(5) var depth_tex: texture_2d<f32>;
@group(1) @binding(6) var normal_tex: texture_2d<f32>;

// Previous-frame ping-pong slot (history to blend with) — own dedicated
// buffers, NOT shared with hybrid_temporal.wgsl's own GI history (see
// this file's own header comment for why DOF needs an independent
// disocclusion/convergence window).
@group(1) @binding(7) var history_color_tex: texture_2d<f32>;
@group(1) @binding(8) var history_length_tex: texture_2d<f32>;
@group(1) @binding(9) var history_depth_tex: texture_2d<f32>;
@group(1) @binding(10) var history_normal_tex: texture_2d<f32>;

@group(2) @binding(0) var dof_color_out: texture_storage_2d<rgba16float, write>;
@group(2) @binding(1) var history_color_out: texture_storage_2d<rgba16float, write>;
@group(2) @binding(2) var history_length_out: texture_storage_2d<r32float, write>;
@group(2) @binding(3) var history_depth_out: texture_storage_2d<r32float, write>;
@group(2) @binding(4) var history_normal_out: texture_storage_2d<rgba32float, write>;

const MAX_MARCH_STEPS: u32 = 128u;
const HIT_EPSILON: f32 = 1e-4;
const LEAF_SENTINEL: u32 = 4294967295u; // u32::MAX
const MAX_STACK: u32 = 64u;

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

// No world_normal/obj_id needed by this pass (no shading happens here) —
// just whether/where the jittered ray hit, unlike hybrid_ddgi_relight.
// wgsl's own fuller Hit struct.
struct Hit {
    did_hit: bool,
    t: f32,
}

fn trace(ray_origin: vec3<f32>, ray_dir: vec3<f32>, t_max: f32) -> Hit {
    var best_hit = false;
    var best_t = t_max;

    if (scene.bvh_node_count == 0u) {
        return Hit(false, 0.0);
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

    return Hit(best_hit, best_t);
}

// Sentinel matching hybrid_trace.wgsl's own SKY_T (both files define it
// separately, no cross-shader constant sharing available — see this
// file's own header comment).
const SKY_T: f32 = 1e6;

// dof_ref.rs::vogel_disk_sample verbatim.
const DOF_GOLDEN_ANGLE_RADIANS: f32 = 2.3999633;

fn dof_vogel_disk_sample(index: u32, total: u32, radius: f32) -> vec2<f32> {
    let t = max(total, 1u);
    let r = radius * sqrt((f32(index) + 0.5) / f32(t));
    let theta = f32(index) * DOF_GOLDEN_ANGLE_RADIANS;
    return vec2<f32>(r * cos(theta), r * sin(theta));
}

// dof_ref.rs::dof_jittered_ray verbatim.
struct DofRay {
    origin: vec3<f32>,
    direction: vec3<f32>,
}

fn dof_jittered_ray(ro: vec3<f32>, rd: vec3<f32>, forward: vec3<f32>, right: vec3<f32>, up: vec3<f32>, focus_distance: f32, disk: vec2<f32>) -> DofRay {
    let cos_theta = max(dot(rd, forward), 1e-4);
    let t_focus = focus_distance / cos_theta;
    let focus_point = ro + rd * t_focus;
    let jittered_origin = ro + right * disk.x + up * disk.y;
    let jittered_dir = normalize(focus_point - jittered_origin);
    return DofRay(jittered_origin, jittered_dir);
}

// Must apply the IDENTICAL jitter offset hybrid_trace.wgsl's own
// generate_primary_ray does — see that function's own doc comment for the
// full rationale. This unperturbed-primary-ray value is what dof_jittered_
// ray then offsets further by the lens aperture disk; if this base ray
// disagreed with the one that actually produced the sharp image being
// resampled, DOF's own disk offset would be computed relative to the
// wrong reference direction.
// UV computed directly against scene.trace_size, not view.viewport — see
// hybrid_trace.wgsl's own generate_primary_ray doc comment for the full
// rationale (this pass shares the SAME trace-resolution working size).
fn generate_primary_ray(pixel: vec2<u32>) -> vec3<f32> {
    let trace_size = vec2<f32>(f32(scene.trace_size_x), f32(scene.trace_size_y));
    let uv = (vec2<f32>(f32(pixel.x), f32(pixel.y)) + 0.5) / trace_size;
    var ndc = uv_to_ndc(uv);
    if (scene.jitter_enabled != 0u) {
        ndc = ndc + vec2<f32>(scene.jitter_offset_x, scene.jitter_offset_y);
    }

    let near_clip = vec4<f32>(ndc, 1.0, 1.0);
    let near_world = view.world_from_clip * near_clip;
    let near_pos = near_world.xyz / near_world.w;
    let ro = view.world_position;
    return normalize(near_pos - ro);
}

// dof_ref.rs::dof_resample_uv verbatim — returns a validity flag in `.z`
// (WGSL has no Option<T>), matching hybrid_temporal.wgsl's own
// world_to_previous_uv return-shape convention exactly.
fn dof_resample_uv(p_world: vec3<f32>) -> vec3<f32> {
    let clip = view.clip_from_world * vec4<f32>(p_world, 1.0);
    if (clip.w <= 0.0) {
        return vec3<f32>(0.0, 0.0, 0.0);
    }
    let ndc = clip.xyz / clip.w;
    let uv = vec2<f32>(ndc.x * 0.5 + 0.5, 1.0 - (ndc.y * 0.5 + 0.5));
    return vec3<f32>(uv, 1.0);
}

// temporal_ref.rs::TEMPORAL_DEPTH_SIGMA/TEMPORAL_NORMAL_COS_THRESHOLD
// verbatim (same values hybrid_temporal.wgsl already uses) — this pass's
// own disocclusion test runs against the SHARP pixel's own depth/normal
// (unaffected by DOF jitter), so the identical thresholds tuned for the
// GI accumulator's real camera/object-motion disocclusion case apply
// unchanged here too; no DOF-specific retuning needed since this test
// never sees the jittered ray's own depth at all.
const TEMPORAL_DEPTH_SIGMA: f32 = 0.05;
const TEMPORAL_NORMAL_COS_THRESHOLD: f32 = 0.9;

fn disocclusion_rejected(current_depth: f32, history_depth: f32, current_normal: vec3<f32>, history_normal: vec3<f32>) -> bool {
    let depth_scale = TEMPORAL_DEPTH_SIGMA * max(current_depth, 1e-4);
    let depth_diff = abs(current_depth - history_depth);
    if (depth_diff > depth_scale) {
        return true;
    }
    let normal_similarity = dot(normalize(current_normal), normalize(history_normal));
    if (normal_similarity < TEMPORAL_NORMAL_COS_THRESHOLD) {
        return true;
    }
    return false;
}

// temporal_ref.rs::temporal_blend verbatim.
fn temporal_blend(current: vec3<f32>, history: vec3<f32>, history_length: f32, max_history_length: f32) -> vec4<f32> {
    let new_length = min(history_length + 1.0, max(max_history_length, 1.0));
    let alpha = 1.0 / new_length;
    let blended = mix(history, current, alpha);
    return vec4<f32>(blended, new_length);
}

@compute @workgroup_size(8, 8, 1)
fn dof_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    // Bounds-checked against scene.trace_size, NOT view.viewport — see
    // hybrid_trace.wgsl's own trace_main doc comment for why.
    if (gid.x >= scene.trace_size_x || gid.y >= scene.trace_size_y) {
        return;
    }
    let pixel = vec2<i32>(gid.xy);
    let size = vec2<u32>(textureDimensions(sharp_color_tex));
    let uv_center = (vec2<f32>(pixel) + vec2<f32>(0.5)) / vec2<f32>(size);
    let sharp_color = textureSampleLevel(sharp_color_tex, sharp_color_sampler, uv_center, 0.0).rgb;
    let current_depth = textureLoad(depth_tex, pixel, 0).r;
    let current_normal = textureLoad(normal_tex, pixel, 0).rgb;

    if (scene.dof_enabled == 0u) {
        // Copy-through: no jitter, no accumulation — bit-for-bit the
        // sharp color, history reset so a later re-enable starts fresh
        // (same convention every other technique-disabled pass in this
        // codebase already establishes).
        textureStore(dof_color_out, pixel, vec4<f32>(sharp_color, 1.0));
        textureStore(history_color_out, pixel, vec4<f32>(sharp_color, 1.0));
        textureStore(history_length_out, pixel, vec4<f32>(0.0, 0.0, 0.0, 0.0));
        textureStore(history_depth_out, pixel, vec4<f32>(current_depth, 0.0, 0.0, 0.0));
        textureStore(history_normal_out, pixel, vec4<f32>(current_normal, 0.0));
        return;
    }

    let rd = generate_primary_ray(gid.xy);
    let ro = view.world_position;
    let right = normalize(view.world_from_view[0].xyz);
    let up = normalize(view.world_from_view[1].xyz);
    let forward = normalize(-view.world_from_view[2].xyz);
    let disk = dof_vogel_disk_sample(scene.dof_frame_index, max(u32(scene.dof_max_history_length), 1u), scene.dof_aperture_radius);
    let jittered = dof_jittered_ray(ro, rd, forward, right, up, scene.dof_focal_distance, disk);

    let hit = trace(jittered.origin, jittered.direction, SKY_T);
    // A miss still has a well-defined "point" far along the jittered ray
    // (SKY_T) to reproject — a background/sky pixel's own defocus is a
    // real, if subtle, lens effect too (distant sky blurs the same way
    // any far-field content does), not a special case to skip.
    let hit_t = select(SKY_T, hit.t, hit.did_hit);
    let p_world_jittered = jittered.origin + jittered.direction * hit_t;

    let reprojected = dof_resample_uv(p_world_jittered);
    var current_sample = sharp_color;
    if (reprojected.z > 0.5 && reprojected.x >= 0.0 && reprojected.x <= 1.0 && reprojected.y >= 0.0 && reprojected.y <= 1.0) {
        current_sample = textureSampleLevel(sharp_color_tex, sharp_color_sampler, reprojected.xy, 0.0).rgb;
    }

    // Disocclusion test against THIS SAME PIXEL's own history (no
    // separate reprojection needed for the history lookup itself — DOF's
    // resample already happened above; the accumulator here is blending
    // "this frame's resampled value" with "last frame's resampled value
    // at this same screen pixel," gated on whether the SHARP scene at
    // this pixel changed, not on any jittered-ray motion).
    let history_depth = textureLoad(history_depth_tex, pixel, 0).r;
    let history_normal = textureLoad(history_normal_tex, pixel, 0).rgb;
    let history_length = textureLoad(history_length_tex, pixel, 0).r;
    let history_color = textureLoad(history_color_tex, pixel, 0).rgb;
    let rejected = disocclusion_rejected(current_depth, history_depth, current_normal, history_normal);
    var result: vec4<f32>;
    if (rejected) {
        result = temporal_blend(current_sample, vec3<f32>(0.0), 0.0, scene.dof_max_history_length);
    } else {
        result = temporal_blend(current_sample, history_color, history_length, scene.dof_max_history_length);
    }
    let blended = result.rgb;
    let new_length = result.w;

    textureStore(dof_color_out, pixel, vec4<f32>(blended, 1.0));
    textureStore(history_color_out, pixel, vec4<f32>(blended, 1.0));
    textureStore(history_length_out, pixel, vec4<f32>(new_length, 0.0, 0.0, 0.0));
    textureStore(history_depth_out, pixel, vec4<f32>(current_depth, 0.0, 0.0, 0.0));
    textureStore(history_normal_out, pixel, vec4<f32>(current_normal, 0.0));
}
