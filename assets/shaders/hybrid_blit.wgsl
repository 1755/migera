// Fresh-start hybrid blit: fullscreen fragment pass copying hybrid_trace's
// flat-color output into the view target, reconstructing reverse-Z
// frag_depth from the stored linear t so hybrid content owns the frame's
// depth buffer exactly like rasterized geometry — matching
// hybrid_legacy_blit.wgsl's reconstruction approach (reproject the world
// hit point through the camera's real projection), read as a pattern, not
// imported.
//
// Two entry points, same specialization-key pattern as hybrid_legacy_blit:
// "fragment" writes depth (requires the depth attachment), "fragment_nodepth"
// is color-only. Kept as two entry points rather than one simplified path
// because HybridBlitKey/pipeline specialization (mirroring hybrid_legacy's
// precedent) already gates on `has_depth` at the Rust level for exactly
// this reason — Bevy's own view may or may not have a depth attachment
// bound depending on the pass configuration, and `@builtin(frag_depth)`
// cannot be present on a pipeline with no `DepthStencilState`, so a single
// shared entry point isn't actually simpler here, just illegal in the
// no-depth case.

#import bevy_render::view::{View, frag_coord_to_uv, uv_to_ndc}

@group(0) @binding(0) var<uniform> view: View;

// Full mirror of src/hybrid/extract.rs's SceneUniform (field-for-field,
// same convention as hybrid_trace.wgsl/hybrid_temporal.wgsl/
// hybrid_denoise.wgsl's own copies) — this pass only reads background_*,
// but the WGSL uniform struct's field offsets must match the real buffer
// layout up to the last field it touches, and the simplest way to
// guarantee that is the same full mirror every other file already uses
// rather than a truncated prefix. This pass writes raw linear HDR — no
// tonemap here, that's Bevy's own Node3d::Tonemapping node's job (see
// HybridRenderPlugin's camera setup: Hdr + Tonemapping + Exposure).
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
    // Primary-ray sub-pixel jitter (TAAU experiment step 1a) — see
    // src/hybrid/taa_ref.rs's own module doc comment. Mirrored field-for-
    // field from src/hybrid/extract.rs's SceneUniform; this file's own
    // primary_ray must apply the identical offset hybrid_trace.wgsl's
    // generate_primary_ray does, or this pass's depth reconstruction will
    // disagree with the traced geometry.
    jitter_enabled: u32,
    jitter_offset_x: f32,
    jitter_offset_y: f32,
    // The trace pass's own (possibly smaller) working resolution
    // (RenderScaleConfig experiment, step 1b) — see src/hybrid/extract.rs's
    // SceneUniform::trace_size_x doc comment. This file's own fragment
    // shader uses this to convert its REAL frag_coord into a UV for
    // sampling color_tex/depth_tex at THEIR resolution, then upscales via
    // a bilinear sample (color) / nearest texel (depth) — see fragment's
    // own body below.
    trace_size_x: u32,
    trace_size_y: u32,
}

@group(1) @binding(0) var<uniform> scene: SceneUniform;
@group(1) @binding(1) var color_tex: texture_2d<f32>;
@group(1) @binding(2) var color_sampler: sampler;
@group(1) @binding(3) var depth_tex: texture_2d<f32>;

// Must match hybrid_trace.wgsl's own SKY_T exactly — both files define it
// separately (no cross-shader constant sharing available here), documented
// in both places.
const SKY_T: f32 = 1e6;

// `trace_px` is a NEAREST-NEIGHBOR mapped coordinate into depth_tex's own
// (possibly smaller than the real output) trace resolution — see this
// file's own trace_pixel_for helper. Depth is intentionally NOT bilinearly
// upscaled like color is: interpolating a depth VALUE across a silhouette
// edge produces a physically meaningless in-between depth (neither
// surface's true distance), a well-known upscaling pitfall — nearest
// keeps every written depth a real, previously-traced value.
fn linear_t_to_frag_depth(trace_px: vec2<i32>, ro: vec3<f32>, rd: vec3<f32>) -> f32 {
    let t = textureLoad(depth_tex, trace_px, 0).r;
    if (t >= SKY_T) {
        // Reverse-Z far plane.
        return 0.0;
    }
    let world = ro + rd * t;
    let clip = view.clip_from_world * vec4<f32>(world, 1.0);
    return clamp(clip.z / clip.w, 0.0, 1.0);
}

// Must apply the IDENTICAL jitter offset hybrid_trace.wgsl's own
// generate_primary_ray does — see that function's own doc comment for the
// full rationale. This function's ray feeds linear_t_to_frag_depth's own
// depth reconstruction; a mismatched jitter here would reconstruct depth
// for a ray that doesn't match the one that actually produced depth_tex's
// stored t, misplacing the depth write relative to the traced geometry.
//
// Computed at the REAL output resolution (view.viewport), NOT scene.
// trace_size — unlike hybrid_trace.wgsl's own generate_primary_ray (which
// generates a ray for a TRACE-resolution pixel), this function reconstructs
// the ray for the REAL screen pixel `frag_coord` refers to, since that's
// the ray whose depth-plane geometry `linear_t_to_frag_depth` needs to
// match this fragment's own true screen position.
fn primary_ray(frag_coord: vec2<f32>) -> vec3<f32> {
    let uv = frag_coord_to_uv(frag_coord, view.viewport);
    var ndc = uv_to_ndc(uv);
    if (scene.jitter_enabled != 0u) {
        ndc = ndc + vec2<f32>(scene.jitter_offset_x, scene.jitter_offset_y);
    }
    let near_clip = vec4<f32>(ndc, 1.0, 1.0);
    let near_world = view.world_from_clip * near_clip;
    let near_pos = near_world.xyz / near_world.w;
    return normalize(near_pos - view.world_position);
}

// This REAL fragment's own UV within the output viewport, `[0, 1]` — the
// shared basis both the bilinear color upscale and the nearest-neighbor
// depth lookup key off, so they always agree on which trace-resolution
// texel(s) correspond to this real screen pixel regardless of
// RenderScaleConfig::scale.
fn trace_uv(frag_coord: vec2<f32>) -> vec2<f32> {
    return frag_coord_to_uv(frag_coord, view.viewport);
}

// Nearest-neighbor trace-resolution pixel for a given output UV — see
// linear_t_to_frag_depth's own doc comment for why depth intentionally
// does NOT bilinearly interpolate. Clamped so a UV of exactly 1.0 (the
// bottom/right edge) doesn't read one texel past the end.
fn trace_pixel_for(uv: vec2<f32>) -> vec2<i32> {
    let trace_size = vec2<f32>(f32(scene.trace_size_x), f32(scene.trace_size_y));
    let px = vec2<i32>(uv * trace_size);
    let max_px = vec2<i32>(i32(scene.trace_size_x) - 1, i32(scene.trace_size_y) - 1);
    return clamp(px, vec2<i32>(0), max_px);
}

struct FragOut {
    @location(0) color: vec4<f32>,
    @builtin(frag_depth) depth: f32,
}

@fragment
fn fragment(@builtin(position) frag_coord: vec4<f32>) -> FragOut {
    let uv = trace_uv(frag_coord.xy);
    let ro = view.world_position;
    let rd = primary_ray(frag_coord.xy);

    // Bilinear upscale: bit-for-bit equal to the old direct textureLoad
    // when RenderScaleConfig::scale == 1.0 (a sample exactly at a texel
    // center returns that texel unchanged), a genuine upscale otherwise.
    let color = textureSampleLevel(color_tex, color_sampler, uv, 0.0).rgb;
    let depth = linear_t_to_frag_depth(trace_pixel_for(uv), ro, rd);
    return FragOut(vec4<f32>(color, 1.0), depth);
}

@fragment
fn fragment_nodepth(@builtin(position) frag_coord: vec4<f32>) -> @location(0) vec4<f32> {
    let uv = trace_uv(frag_coord.xy);
    return textureSampleLevel(color_tex, color_sampler, uv, 0.0);
}
