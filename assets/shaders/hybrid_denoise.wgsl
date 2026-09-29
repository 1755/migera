// Same-frame edge-aware spatial denoise for hybrid_trace.wgsl's
// indirect-diffuse output (Stage B follow-up fix, not a new GI-trajectory
// stage): a small fixed-radius bilateral blur over `indirect_view`,
// weighted by how similar each neighboring pixel's world-space shading
// normal (`normal_view`) and linear depth (`out_depth`) are to the center
// pixel's own — smooths flat noisy regions while refusing to blur across
// a real edge (a different object, a silhouette, a sharp corner). See
// cpu_ref.rs::blur_indirect_at's doc comment for the full rationale and
// the CPU-tested reference this is a faithful port of.
//
// Reads out_color/indirect_view/normal_view/out_depth (indirect_view here
// is actually accumulated_indirect_view — hybrid_temporal.wgsl's
// temporally-blended output, bound in indirect_view's place; this file's
// own blur logic is unchanged, only its Rust-side bind-group source
// changed once temporal accumulation landed) — all sampled here as plain
// texture_2d — no filtering, exact texel loads via textureLoad, same
// convention hybrid_blit.wgsl already establishes — and writes the
// recombined `direct_and_emissive +
// blurred_indirect` into a NEW storage texture, `denoised_color_view` —
// WGSL/WGPU storage textures cannot be both read and write-bound at once
// (no `read_write` precedent for a texture anywhere in this codebase, see
// this file's own history), so recombining in-place into out_color isn't
// possible; the blit pass instead reads `denoised_color_view` in
// out_color's place (a one-line rebind, not a shader-logic change).
//
// Blur strength is ADAPTIVE on this pixel's own temporal history_length
// (bound here as history_length_tex — this frame's own freshly-written
// ping-pong slot from hybrid_temporal.wgsl, NOT the previous frame's read
// slot: the blur needs to know how converged THIS frame's own
// accumulated value already is) — see cpu_ref.rs::blur_strength's own
// doc comment for the fade curve and rationale: full blur strength for a
// brand-new/just-rejected pixel (history_length <= 1, the same single-
// frame-noisy case this pass always existed to cover), fading to zero
// blur once history has converged (temporal accumulation has already
// done the noise-reduction job by then; blurring further only costs
// sharpness for no remaining benefit). scene.temporal_enabled == 0 means
// history_length is always reset to 0 by hybrid_temporal.wgsl's own
// copy-through path, so this naturally falls back to always-full-strength
// blur when temporal accumulation itself is off — no separate branch
// needed here.
//
// scene.denoise_enabled == 0 skips the weight-loop entirely (a straight
// unblurred add) rather than skipping this pass's dispatch outright — see
// SceneUniform::denoise_enabled's own doc comment for why.

// Bound to the SAME buffer as hybrid_trace.wgsl's own SceneUniform
// (pipeline.rs's single scene_uniform: UniformBuffer<SceneUniform>,
// shared across trace/denoise/temporal/blit) — this struct's own field
// list had drifted out of sync with the real (extract.rs) layout since
// gi_method/cone-tracing fields were added there (this copy still had
// the pre-cone-tracing indirect_enabled/indirect_max_t names), which
// made every field read here past background_b misaligned garbage from
// the real buffer's actual bytes. Must be kept field-for-field IDENTICAL
// to hybrid_trace.wgsl's own SceneUniform mirror (and to
// extract::SceneUniform) — std140 layout is strict and offset-dependent,
// there is no cross-shader shared-struct mechanism in WGSL to enforce
// this automatically.
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
    // Unused here — see src/hybrid/extract.rs's SceneUniform's own DDGI
    // field doc comments. Mirrored for struct-layout parity only.
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
    // this file (denoise has no primary-ray dependency), kept only for
    // struct-layout parity with the other SceneUniform copies sharing
    // this same uniform buffer.
    jitter_enabled: u32,
    jitter_offset_x: f32,
    jitter_offset_y: f32,
    // Trace-resolution scale experiment (step 1b) — unused in this file
    // directly (denoise already derives its own working size from
    // textureDimensions on the trace-resolution textures it reads), kept
    // only for struct-layout parity with the other SceneUniform copies
    // sharing this same uniform buffer.
    trace_size_x: u32,
    trace_size_y: u32,
}

@group(0) @binding(0) var<uniform> scene: SceneUniform;
@group(0) @binding(1) var indirect_tex: texture_2d<f32>;
@group(0) @binding(2) var normal_tex: texture_2d<f32>;
@group(0) @binding(3) var depth_tex: texture_2d<f32>;
@group(0) @binding(4) var color_tex: texture_2d<f32>;
@group(0) @binding(5) var history_length_tex: texture_2d<f32>;
// Temporally-accumulated specular reflection — composited directly, with
// NO spatial blur (unlike indirect_tex above): a low-roughness reflection
// needs its sharpness preserved, and a high-roughness one already
// self-blurred spatially via its own wide reflection cone (see
// reflect_trace_ray's own doc comment) — blurring it again here would
// only cost sharpness for no remaining benefit, the same "blur strength
// fades once the signal is already clean" principle blur_strength
// already applies to indirect_tex, just decided once at compile time for
// this channel instead of per-pixel.
@group(0) @binding(6) var reflect_tex: texture_2d<f32>;
// Temporally-accumulated transmission/refraction — same "composited
// directly, no spatial blur" reasoning as reflect_tex above.
@group(0) @binding(7) var refract_tex: texture_2d<f32>;
@group(0) @binding(8) var denoised_color_out: texture_storage_2d<rgba16float, write>;

// cpu_ref.rs::BLUR_RADIUS/BLUR_NORMAL_SIGMA/BLUR_DEPTH_SIGMA verbatim —
// see those constants' own doc comments in cpu_ref.rs for the reasoning
// behind each value.
const BLUR_RADIUS: i32 = 2;
const BLUR_NORMAL_SIGMA: f32 = 0.1;
const BLUR_DEPTH_SIGMA: f32 = 0.05;

// cpu_ref.rs::blur_strength verbatim — see that function's own doc
// comment for the fade curve and rationale.
fn blur_strength(history_length: f32, max_history_length: f32) -> f32 {
    let denom = max(max_history_length - 1.0, 1e-4);
    return clamp(1.0 - (history_length - 1.0) / denom, 0.0, 1.0);
}

// cpu_ref.rs::blur_indirect_at verbatim, with `sample_at` inlined as
// direct texelLoads clamped to the viewport (WGSL has no closures) —
// same clamp-to-edge convention a real sampler's edge mode would apply,
// documented on cpu_ref.rs::blur_indirect_at's own `sample_at` parameter.
fn blur_indirect(pixel: vec2<i32>, size: vec2<i32>) -> vec3<f32> {
    let center_indirect = textureLoad(indirect_tex, pixel, 0).rgb;
    let center_normal = normalize(textureLoad(normal_tex, pixel, 0).rgb);
    let center_depth = textureLoad(depth_tex, pixel, 0).r;

    var acc = vec3<f32>(0.0);
    var weight_sum = 0.0;
    for (var dy = -BLUR_RADIUS; dy <= BLUR_RADIUS; dy = dy + 1) {
        for (var dx = -BLUR_RADIUS; dx <= BLUR_RADIUS; dx = dx + 1) {
            let np = clamp(pixel + vec2<i32>(dx, dy), vec2<i32>(0, 0), size - vec2<i32>(1, 1));
            let n_indirect = textureLoad(indirect_tex, np, 0).rgb;
            let n_normal = normalize(textureLoad(normal_tex, np, 0).rgb);
            let n_depth = textureLoad(depth_tex, np, 0).r;

            let normal_similarity = max(dot(center_normal, n_normal), 0.0);
            let normal_weight = pow(normal_similarity, 1.0 / max(BLUR_NORMAL_SIGMA, 1e-4));
            let depth_scale = BLUR_DEPTH_SIGMA * max(center_depth, 1e-4);
            let depth_diff = abs(center_depth - n_depth);
            let depth_weight = exp(-depth_diff / max(depth_scale, 1e-4));
            let weight = normal_weight * depth_weight;

            acc = acc + n_indirect * weight;
            weight_sum = weight_sum + weight;
        }
    }
    if (weight_sum > 1e-6) {
        return acc / weight_sum;
    }
    return center_indirect;
}

@compute @workgroup_size(8, 8, 1)
fn denoise_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(indirect_tex);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    let pixel = vec2<i32>(gid.xy);
    let center_indirect = textureLoad(indirect_tex, pixel, 0).rgb;

    var indirect = center_indirect;
    if (scene.denoise_enabled != 0u) {
        let history_length = textureLoad(history_length_tex, pixel, 0).r;
        let strength = blur_strength(history_length, scene.temporal_max_history_length);
        // Skip the 25-tap weight loop entirely once a converged pixel's
        // strength rounds to zero — same "costs nothing when there's
        // nothing to do" principle this codebase already applies to
        // denoise_enabled/temporal_enabled themselves, just per-pixel
        // instead of per-frame.
        if (strength > 1e-4) {
            let blurred = blur_indirect(pixel, vec2<i32>(size));
            indirect = mix(center_indirect, blurred, strength);
        }
    }

    let direct_and_emissive = textureLoad(color_tex, pixel, 0).rgb;
    let reflect_color = textureLoad(reflect_tex, pixel, 0).rgb;
    let refract_color = textureLoad(refract_tex, pixel, 0).rgb;
    textureStore(denoised_color_out, pixel, vec4<f32>(direct_and_emissive + indirect + reflect_color + refract_color, 1.0));
}
