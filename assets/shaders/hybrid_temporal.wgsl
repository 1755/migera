// Temporal accumulation of hybrid_trace.wgsl's indirect-diffuse output —
// the architectural fix for the banding a same-frame spatial blur alone
// (hybrid_denoise.wgsl) can't fully hide. See src/hybrid/temporal_ref.rs's
// own module doc comment for the full picture: this is a faithful WGSL
// port of that file's `reproject_world_point`/`world_to_previous_uv`/
// `disocclusion_rejected`/`temporal_blend` functions, composed here into
// one compute pass.
//
// Pipeline position: runs between hybrid_trace.wgsl and hybrid_denoise.wgsl
// — reads this frame's raw indirect_view, blends it with the previous
// frame's history (reprojected to account for camera AND per-object
// motion), and writes accumulated_indirect_view, which hybrid_denoise.wgsl
// then reads in indirect_view's place (a spatial blur still runs on TOP
// of the temporally-accumulated signal — see cpu_ref.rs::blur_indirect_at's
// own doc comment for why keeping both is the standard SVGF-style shape:
// pixels with no/rejected history, e.g. disocclusion or the first frame,
// are still just one frame's noisy estimate, which the spatial blur alone
// cleans up until temporal accumulation has had a few frames to converge).
//
// Two-pass split with trace_main: trace_main computes and stores each
// pixel's REPROJECTED PREVIOUS-FRAME WORLD position into motion_view (the
// per-OBJECT rigid-motion undo/reapply step — it has this frame's object
// data on hand already) but cannot finish reprojecting into last frame's
// SCREEN space itself, since it has no PreviousViewData bound. This pass
// finishes that: world_to_previous_uv, using PreviousViewData (bevy_pbr's
// own last-frame camera matrices, populated automatically by
// PrepassPlugin — no extraction of our own needed).
//
// scene.temporal_enabled == 0 skips reprojection/blend entirely (a
// straight copy-through of this frame's raw indirect_view into
// accumulated_indirect_view, history left untouched) rather than skipping
// this pass's dispatch outright — same "one cheap copy-pass, not a
// conditional dispatch skip" reasoning as SceneUniform::denoise_enabled's
// own doc comment.

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
    // this file directly (reprojection reads the jittered hit's own
    // motion_view output, not the jitter offset itself), kept only for
    // struct-layout parity with the other SceneUniform copies sharing
    // this same uniform buffer.
    jitter_enabled: u32,
    jitter_offset_x: f32,
    jitter_offset_y: f32,
    // Trace-resolution scale experiment (step 1b) — unused in this file
    // directly (this pass already derives its own working size from
    // textureDimensions on the trace-resolution textures it reads, which
    // are resized to match automatically), kept only for struct-layout
    // parity with the other SceneUniform copies sharing this same uniform
    // buffer.
    trace_size_x: u32,
    trace_size_y: u32,
}

// Mirrors bevy_core_pipeline::prepass::PreviousViewData exactly (field
// order and type) — see that struct's own doc comment: "View matrices
// from the previous frame."
struct PreviousViewData {
    view_from_world: mat4x4<f32>,
    clip_from_world: mat4x4<f32>,
    clip_from_view: mat4x4<f32>,
    world_from_clip: mat4x4<f32>,
    view_from_clip: mat4x4<f32>,
}

@group(0) @binding(0) var<uniform> scene: SceneUniform;
@group(0) @binding(1) var<uniform> previous_view: PreviousViewData;
// This frame's trace-pass outputs.
@group(0) @binding(2) var indirect_tex: texture_2d<f32>;
@group(0) @binding(3) var depth_tex: texture_2d<f32>;
@group(0) @binding(4) var normal_tex: texture_2d<f32>;
@group(0) @binding(5) var motion_tex: texture_2d<f32>; // reprojected previous-frame world pos (see this file's own header)
// Previous-frame ping-pong slot (history to blend with).
@group(0) @binding(6) var history_color_tex: texture_2d<f32>;
@group(0) @binding(7) var history_length_tex: texture_2d<f32>;
@group(0) @binding(8) var history_depth_tex: texture_2d<f32>;
@group(0) @binding(9) var history_normal_tex: texture_2d<f32>;

// This frame's own ping-pong slot (seeding next frame's history) + the
// blended output the denoise pass reads next.
@group(1) @binding(0) var accumulated_indirect_out: texture_storage_2d<rgba16float, write>;
@group(1) @binding(1) var history_color_out: texture_storage_2d<rgba16float, write>;
@group(1) @binding(2) var history_length_out: texture_storage_2d<r32float, write>;
@group(1) @binding(3) var history_depth_out: texture_storage_2d<r32float, write>;
@group(1) @binding(4) var history_normal_out: texture_storage_2d<rgba32float, write>;

// --- reflect_temporal_main's own bind groups (SEPARATE pipeline, SEPARATE
// bind groups from temporal_main's above — see this file's own
// reflect_temporal_main doc comment for why reflection needs independent
// history rather than sharing indirect_view's). Same 0/1 group SPLIT
// (Rust-side layout only, not shared WGSL @group numbers — WGSL/WGPU
// scope @group declarations per-pipeline, so reusing group(0)/group(1)
// here for a DIFFERENT pipeline is correct, not a collision with
// temporal_main's own group 0/1 above). ---------------------------------

@group(0) @binding(0) var<uniform> reflect_scene: SceneUniform;
@group(0) @binding(1) var<uniform> reflect_previous_view: PreviousViewData;
@group(0) @binding(2) var reflect_tex: texture_2d<f32>;
@group(0) @binding(3) var reflect_depth_tex: texture_2d<f32>;
// .xyz = world-space shading normal (unused here); .w = the PRIMARY
// (reflecting) surface's own roughness — see hybrid_trace.wgsl's own
// normal_view doc comment. Read here to gate virtual-point reprojection:
// below REFLECT_ROUGHNESS_GATE, this pixel's single reflection hit point
// is a well-defined stand-in for the whole (near-mirror) reflection,
// worth reprojecting and accumulating; at or above it, the reflection
// cone is wide enough that any single point is a poor stand-in for the
// blurred lobe it actually represents, so accumulation is skipped
// entirely and the cone's own spatial footprint is relied on instead
// (the identical "already spatially self-blurred, no further denoising
// benefit" reasoning hybrid_denoise.wgsl's own reflect_tex doc comment
// already establishes for skipping the spatial blur pass on this
// channel — here applied to skipping TEMPORAL accumulation instead).
@group(0) @binding(4) var reflect_normal_tex: texture_2d<f32>;
@group(0) @binding(5) var reflect_motion_tex: texture_2d<f32>;
@group(0) @binding(6) var reflect_history_color_tex: texture_2d<f32>;
@group(0) @binding(7) var reflect_history_length_tex: texture_2d<f32>;
@group(0) @binding(8) var reflect_history_depth_tex: texture_2d<f32>;
@group(0) @binding(9) var reflect_history_normal_tex: texture_2d<f32>;

@group(1) @binding(0) var accumulated_reflect_out: texture_storage_2d<rgba16float, write>;
@group(1) @binding(1) var reflect_history_color_out: texture_storage_2d<rgba16float, write>;
@group(1) @binding(2) var reflect_history_length_out: texture_storage_2d<r32float, write>;
@group(1) @binding(3) var reflect_history_depth_out: texture_storage_2d<r32float, write>;
@group(1) @binding(4) var reflect_history_normal_out: texture_storage_2d<rgba32float, write>;

// Below this roughness, a reflection hit point is treated as precise
// enough to reproject and temporally accumulate; at or above it,
// accumulation is skipped (see reflect_normal_tex's own doc comment) —
// a reasoned engineering cutoff (not a cited SOTA threshold), chosen as
// "clearly still glossy, not yet diffuse-like" on this renderer's own
// alpha = roughness^2 GGX remap (alpha ~= 0.09 at this cutoff).
const REFLECT_ROUGHNESS_GATE: f32 = 0.3;

// temporal_ref.rs::TEMPORAL_DEPTH_SIGMA/TEMPORAL_NORMAL_COS_THRESHOLD
// verbatim — see those constants' own doc comments for the reasoning.
const TEMPORAL_DEPTH_SIGMA: f32 = 0.05;
const TEMPORAL_NORMAL_COS_THRESHOLD: f32 = 0.9;

// temporal_ref.rs::world_to_previous_uv verbatim. Returns a UV plus a
// validity flag in `.z` (1.0 = valid, 0.0 = invalid/behind-camera) since
// WGSL has no Option<T> — callers check `.z` before trusting `.xy`,
// mirroring the Rust function's own `Option<Vec2>` contract exactly.
fn world_to_previous_uv(p_world_previous: vec3<f32>) -> vec3<f32> {
    let clip = previous_view.clip_from_world * vec4<f32>(p_world_previous, 1.0);
    if (clip.w <= 0.0) {
        return vec3<f32>(0.0, 0.0, 0.0);
    }
    let ndc = clip.xyz / clip.w;
    let uv = vec2<f32>(ndc.x * 0.5 + 0.5, 1.0 - (ndc.y * 0.5 + 0.5));
    return vec3<f32>(uv, 1.0);
}

// temporal_ref.rs::disocclusion_rejected verbatim.
fn disocclusion_rejected(
    current_depth: f32,
    history_depth: f32,
    current_normal: vec3<f32>,
    history_normal: vec3<f32>,
    uv_in_bounds: bool,
) -> bool {
    if (!uv_in_bounds) {
        return true;
    }
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

// temporal_ref.rs::temporal_blend verbatim. Returns (blended.rgb, new_length)
// packed into a vec4 since WGSL has no tuple return.
fn temporal_blend(current: vec3<f32>, history: vec3<f32>, history_length: f32, max_history_length: f32) -> vec4<f32> {
    let new_length = min(history_length + 1.0, max(max_history_length, 1.0));
    let alpha = 1.0 / new_length;
    let blended = mix(history, current, alpha);
    return vec4<f32>(blended, new_length);
}

@compute @workgroup_size(8, 8, 1)
fn temporal_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(indirect_tex);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    let pixel = vec2<i32>(gid.xy);
    let current_indirect = textureLoad(indirect_tex, pixel, 0).rgb;

    if (scene.temporal_enabled == 0u) {
        // Copy-through: no reprojection/blend, history untouched (this
        // frame's own history slot is still written below so a later
        // re-enable doesn't read stale/uninitialized content — but with
        // history_length reset to 0 so it's treated as "no history" the
        // moment accumulation resumes, exactly the fresh-pixel case
        // temporal_blend already handles).
        textureStore(accumulated_indirect_out, pixel, vec4<f32>(current_indirect, 1.0));
        textureStore(history_color_out, pixel, vec4<f32>(current_indirect, 1.0));
        textureStore(history_length_out, pixel, vec4<f32>(0.0, 0.0, 0.0, 0.0));
        textureStore(history_depth_out, pixel, textureLoad(depth_tex, pixel, 0));
        textureStore(history_normal_out, pixel, textureLoad(normal_tex, pixel, 0));
        return;
    }

    let current_depth = textureLoad(depth_tex, pixel, 0).r;
    let current_normal = textureLoad(normal_tex, pixel, 0).rgb;
    let p_world_previous = textureLoad(motion_tex, pixel, 0).rgb;

    let reprojected = world_to_previous_uv(p_world_previous);
    let uv_valid = reprojected.z > 0.5 && reprojected.x >= 0.0 && reprojected.x <= 1.0 && reprojected.y >= 0.0 && reprojected.y <= 1.0;

    // Default: no valid/accepted history — temporal_blend's own
    // history_length==0 contract makes this exactly `current_indirect`
    // with a new history_length of 1.0 (this frame becomes the first
    // accumulated sample).
    var result = temporal_blend(current_indirect, vec3<f32>(0.0), 0.0, scene.temporal_max_history_length);
    if (uv_valid) {
        let history_pixel = clamp(vec2<i32>(reprojected.xy * vec2<f32>(size)), vec2<i32>(0, 0), vec2<i32>(size) - vec2<i32>(1, 1));
        let history_depth = textureLoad(history_depth_tex, history_pixel, 0).r;
        let history_normal = textureLoad(history_normal_tex, history_pixel, 0).rgb;
        let rejected = disocclusion_rejected(current_depth, history_depth, current_normal, history_normal, uv_valid);
        if (!rejected) {
            let history_color = textureLoad(history_color_tex, history_pixel, 0).rgb;
            let prior_length = textureLoad(history_length_tex, history_pixel, 0).r;
            result = temporal_blend(current_indirect, history_color, prior_length, scene.temporal_max_history_length);
        }
    }
    let blended = result.rgb;
    let history_length = result.w;

    textureStore(accumulated_indirect_out, pixel, vec4<f32>(blended, 1.0));
    textureStore(history_color_out, pixel, vec4<f32>(blended, 1.0));
    textureStore(history_length_out, pixel, vec4<f32>(history_length, 0.0, 0.0, 0.0));
    textureStore(history_depth_out, pixel, vec4<f32>(current_depth, 0.0, 0.0, 0.0));
    textureStore(history_normal_out, pixel, vec4<f32>(current_normal, 0.0));
}

// Specular reflection's own temporal-accumulation pass — same overall
// shape as temporal_main above (reproject, disocclusion-test, blend), but
// with two real differences specific to specular reflection, both
// documented on the bindings/constant above:
//
// 1) VIRTUAL-POINT reprojection: `reflect_motion_tex` already holds the
//    REFLECTED hit's own reprojected previous-frame world position (see
//    hybrid_trace.wgsl's own reflect_p_world_previous), not the
//    reflecting surface's — reusing world_to_previous_uv/
//    disocclusion_rejected/temporal_blend UNCHANGED here is correct
//    specifically BECAUSE trace_main already did the hard part (choosing
//    the right point to reproject); this pass doesn't need its own
//    separate reprojection math, only different INPUT textures.
//
// 2) ROUGHNESS-GATED accumulation: below REFLECT_ROUGHNESS_GATE, behaves
//    exactly like temporal_main (reproject + blend); at or above it,
//    skips accumulation entirely and copy-throughs this frame's own raw
//    reflect_tex value, matching temporal_main's own
//    scene.temporal_enabled==0 copy-through shape (same "one cheap
//    copy-pass, not a conditional dispatch skip" reasoning, applied
//    per-pixel here instead of per-frame) — see reflect_normal_tex's own
//    doc comment for why a high-roughness reflection's single hit point
//    is a poor stand-in for its own blurred lobe.
@compute @workgroup_size(8, 8, 1)
fn reflect_temporal_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(reflect_tex);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    let pixel = vec2<i32>(gid.xy);
    let current_reflect = textureLoad(reflect_tex, pixel, 0).rgb;
    let reflecting_roughness = textureLoad(reflect_normal_tex, pixel, 0).w;

    if (reflect_scene.temporal_enabled == 0u || reflecting_roughness >= REFLECT_ROUGHNESS_GATE) {
        textureStore(accumulated_reflect_out, pixel, vec4<f32>(current_reflect, 1.0));
        textureStore(reflect_history_color_out, pixel, vec4<f32>(current_reflect, 1.0));
        textureStore(reflect_history_length_out, pixel, vec4<f32>(0.0, 0.0, 0.0, 0.0));
        textureStore(reflect_history_depth_out, pixel, textureLoad(reflect_depth_tex, pixel, 0));
        textureStore(reflect_history_normal_out, pixel, textureLoad(reflect_normal_tex, pixel, 0));
        return;
    }

    let current_depth = textureLoad(reflect_depth_tex, pixel, 0).r;
    let current_normal = textureLoad(reflect_normal_tex, pixel, 0).rgb;
    let p_world_previous = textureLoad(reflect_motion_tex, pixel, 0).rgb;

    let clip = reflect_previous_view.clip_from_world * vec4<f32>(p_world_previous, 1.0);
    var reprojected = vec3<f32>(0.0);
    if (clip.w > 0.0) {
        let ndc = clip.xyz / clip.w;
        reprojected = vec3<f32>(ndc.x * 0.5 + 0.5, 1.0 - (ndc.y * 0.5 + 0.5), 1.0);
    }
    let uv_valid = reprojected.z > 0.5 && reprojected.x >= 0.0 && reprojected.x <= 1.0 && reprojected.y >= 0.0 && reprojected.y <= 1.0;

    var result = temporal_blend(current_reflect, vec3<f32>(0.0), 0.0, reflect_scene.temporal_max_history_length);
    if (uv_valid) {
        let history_pixel = clamp(vec2<i32>(reprojected.xy * vec2<f32>(size)), vec2<i32>(0, 0), vec2<i32>(size) - vec2<i32>(1, 1));
        let history_depth = textureLoad(reflect_history_depth_tex, history_pixel, 0).r;
        let history_normal = textureLoad(reflect_history_normal_tex, history_pixel, 0).rgb;
        let rejected = disocclusion_rejected(current_depth, history_depth, current_normal, history_normal, uv_valid);
        if (!rejected) {
            let history_color = textureLoad(reflect_history_color_tex, history_pixel, 0).rgb;
            let prior_length = textureLoad(reflect_history_length_tex, history_pixel, 0).r;
            result = temporal_blend(current_reflect, history_color, prior_length, reflect_scene.temporal_max_history_length);
        }
    }
    let blended = result.rgb;
    let history_length = result.w;

    textureStore(accumulated_reflect_out, pixel, vec4<f32>(blended, 1.0));
    textureStore(reflect_history_color_out, pixel, vec4<f32>(blended, 1.0));
    textureStore(reflect_history_length_out, pixel, vec4<f32>(history_length, 0.0, 0.0, 0.0));
    textureStore(reflect_history_depth_out, pixel, vec4<f32>(current_depth, 0.0, 0.0, 0.0));
    textureStore(reflect_history_normal_out, pixel, vec4<f32>(current_normal, reflecting_roughness));
}

// --- transmit_temporal_main's own bind groups (SEPARATE pipeline from
// BOTH temporal_main and reflect_temporal_main — see HybridTransmitHistory's
// own doc comment for why transmission needs independent history from
// both diffuse GI and reflection). Same 0/1 group split convention as
// reflect_temporal_main above (WGSL/WGPU scope @group per-pipeline, so
// reusing the numbers is correct, not a collision). ---------------------

@group(0) @binding(0) var<uniform> transmit_scene: SceneUniform;
@group(0) @binding(1) var<uniform> transmit_previous_view: PreviousViewData;
@group(0) @binding(2) var transmit_tex: texture_2d<f32>;
@group(0) @binding(3) var transmit_depth_tex: texture_2d<f32>;
// .xyz = reprojected previous-frame world pos for the PRIMARY (entry)
// surface (unused here — motion_tex's own .rgb role, see hybrid_trace.wgsl's
// own binding doc comment); .w = the entry surface's own roughness
// (refracting_roughness), packed into motion_view's previously-unused .w
// channel — same "gate temporal accumulation by roughness" idea
// reflect_normal_tex's own .w already establishes for reflection, applied
// here to transmission's own entry surface instead of the reflecting one.
@group(0) @binding(4) var transmit_normal_tex: texture_2d<f32>;
@group(0) @binding(5) var transmit_motion_tex: texture_2d<f32>;
@group(0) @binding(6) var transmit_history_color_tex: texture_2d<f32>;
@group(0) @binding(7) var transmit_history_length_tex: texture_2d<f32>;
@group(0) @binding(8) var transmit_history_depth_tex: texture_2d<f32>;
@group(0) @binding(9) var transmit_history_normal_tex: texture_2d<f32>;

@group(1) @binding(0) var accumulated_refract_out: texture_storage_2d<rgba16float, write>;
@group(1) @binding(1) var transmit_history_color_out: texture_storage_2d<rgba16float, write>;
@group(1) @binding(2) var transmit_history_length_out: texture_storage_2d<r32float, write>;
@group(1) @binding(3) var transmit_history_depth_out: texture_storage_2d<r32float, write>;
@group(1) @binding(4) var transmit_history_normal_out: texture_storage_2d<rgba32float, write>;

// Same cutoff/reasoning as REFLECT_ROUGHNESS_GATE, applied to the ENTRY
// surface's own roughness for transmission instead of the reflecting
// surface's.
const TRANSMIT_ROUGHNESS_GATE: f32 = 0.3;

// Transmission's own temporal-accumulation pass — same overall shape as
// reflect_temporal_main above (VIRTUAL-POINT reprojection via a
// pre-computed motion texture, ROUGHNESS-GATED accumulation), applied to
// the transmission channel: `transmit_motion_tex` already holds the
// transmitted ray's own first-EXIT hit reprojected previous-frame world
// position (see hybrid_trace.wgsl's own refract_p_world_previous), not
// the entry surface's — reusing world_to_previous_uv/disocclusion_rejected/
// temporal_blend UNCHANGED here is correct for the identical reason it is
// in reflect_temporal_main.
@compute @workgroup_size(8, 8, 1)
fn transmit_temporal_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let size = textureDimensions(transmit_tex);
    if (gid.x >= size.x || gid.y >= size.y) {
        return;
    }
    let pixel = vec2<i32>(gid.xy);
    let current_transmit = textureLoad(transmit_tex, pixel, 0).rgb;
    let refracting_roughness = textureLoad(transmit_normal_tex, pixel, 0).w;

    if (transmit_scene.temporal_enabled == 0u || refracting_roughness >= TRANSMIT_ROUGHNESS_GATE) {
        textureStore(accumulated_refract_out, pixel, vec4<f32>(current_transmit, 1.0));
        textureStore(transmit_history_color_out, pixel, vec4<f32>(current_transmit, 1.0));
        textureStore(transmit_history_length_out, pixel, vec4<f32>(0.0, 0.0, 0.0, 0.0));
        textureStore(transmit_history_depth_out, pixel, textureLoad(transmit_depth_tex, pixel, 0));
        textureStore(transmit_history_normal_out, pixel, textureLoad(transmit_normal_tex, pixel, 0));
        return;
    }

    let current_depth = textureLoad(transmit_depth_tex, pixel, 0).r;
    let current_normal = textureLoad(transmit_normal_tex, pixel, 0).rgb;
    let p_world_previous = textureLoad(transmit_motion_tex, pixel, 0).rgb;

    let clip = transmit_previous_view.clip_from_world * vec4<f32>(p_world_previous, 1.0);
    var reprojected = vec3<f32>(0.0);
    if (clip.w > 0.0) {
        let ndc = clip.xyz / clip.w;
        reprojected = vec3<f32>(ndc.x * 0.5 + 0.5, 1.0 - (ndc.y * 0.5 + 0.5), 1.0);
    }
    let uv_valid = reprojected.z > 0.5 && reprojected.x >= 0.0 && reprojected.x <= 1.0 && reprojected.y >= 0.0 && reprojected.y <= 1.0;

    var result = temporal_blend(current_transmit, vec3<f32>(0.0), 0.0, transmit_scene.temporal_max_history_length);
    if (uv_valid) {
        let history_pixel = clamp(vec2<i32>(reprojected.xy * vec2<f32>(size)), vec2<i32>(0, 0), vec2<i32>(size) - vec2<i32>(1, 1));
        let history_depth = textureLoad(transmit_history_depth_tex, history_pixel, 0).r;
        let history_normal = textureLoad(transmit_history_normal_tex, history_pixel, 0).rgb;
        let rejected = disocclusion_rejected(current_depth, history_depth, current_normal, history_normal, uv_valid);
        if (!rejected) {
            let history_color = textureLoad(transmit_history_color_tex, history_pixel, 0).rgb;
            let prior_length = textureLoad(transmit_history_length_tex, history_pixel, 0).r;
            result = temporal_blend(current_transmit, history_color, prior_length, transmit_scene.temporal_max_history_length);
        }
    }
    let blended = result.rgb;
    let history_length = result.w;

    textureStore(accumulated_refract_out, pixel, vec4<f32>(blended, 1.0));
    textureStore(transmit_history_color_out, pixel, vec4<f32>(blended, 1.0));
    textureStore(transmit_history_length_out, pixel, vec4<f32>(history_length, 0.0, 0.0, 0.0));
    textureStore(transmit_history_depth_out, pixel, vec4<f32>(current_depth, 0.0, 0.0, 0.0));
    textureStore(transmit_history_normal_out, pixel, vec4<f32>(current_normal, refracting_roughness));
}
