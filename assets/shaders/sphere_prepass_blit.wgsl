// Write-and-throw spike: reads the compute trace's scratch textures
// (linear hit-t, world-space normal, ray-traced reflection color) and writes:
//   - reverse-Z frag_depth into the REAL view depth attachment (same math as
//     `hybrid_blit.wgsl`'s `linear_t_to_frag_depth`, just using Bevy's own
//     `view.clip_from_world` instead of a hand-rolled view_proj uniform,
//     since this spike doesn't carry a SceneUniform at all).
//   - a `Rgba32Uint` deferred G-buffer value packed to EXACTLY match
//     `bevy_pbr::pbr_deferred_functions::deferred_gbuffer_from_pbr_input`'s
//     layout, so Bevy's own stock deferred lighting pass can shade this
//     fragment with zero custom shading code on our side. Field-for-field
//     citations (bevy_pbr-0.19.1):
//       r: pack_unorm4x8_(base_color_srgb, perceptual_roughness) — pbr_deferred_functions.wgsl:76
//       g: rgb9e5::vec3_to_rgb9e5_(emissive)                     — pbr_deferred_functions.wgsl:77
//       b: pack_unorm4x8_(reflectance, metallic, occlusion, clearcoat_props) — pbr_deferred_functions.wgsl:78 (non-WEBGL2 path)
//       a: pack_24bit_normal_and_flags(octahedral_encode(N), flags)         — pbr_deferred_functions.wgsl:79
//   - the `R8Uint` deferred_lighting_pass_id texture, set to
//     `bevy_pbr::deferred::DEFAULT_PBR_DEFERRED_LIGHTING_PASS_ID` (= 1u8) so
//     the stock PBR deferred lighting shader claims this fragment instead of
//     leaving it unlit.
//
// No mesh geometry anywhere in this example — everything downstream of this
// shader (SSAO if enabled, deferred lighting, tonemapping) is 100% stock
// Bevy code operating on textures this shader alone populated.

#import bevy_render::view::{View, frag_coord_to_uv, uv_to_ndc}

@group(0) @binding(0) var<uniform> view: View;

// `--stress N`: identical material across every grid instance, so this
// shader (which only needs the shared material, never per-instance
// position) doesn't need any instance-awareness at all — see
// sphere_prepass_trace.wgsl's SphereUniform for the full struct including
// the per-instance `centers` array this one omits (struct layout up to
// `centers` must still match exactly, since both are read from the same
// bind group; this struct simply stops before that field).
struct SphereUniform {
    radius: f32,
    base_color: vec3<f32>,
    metallic: f32,
    roughness: f32,
    reflectance: f32,
    res_x: f32,
    res_y: f32,
    instance_count: u32,
}

@group(1) @binding(0) var<uniform> sphere: SphereUniform;
@group(1) @binding(1) var scratch_t: texture_2d<f32>;
@group(1) @binding(2) var scratch_normal: texture_2d<f32>;
@group(1) @binding(3) var scratch_reflect: texture_2d<f32>;

const SKY_T: f32 = 400.0;

// --- vendored from bevy_pbr::render::rgb9e5 (rgb9e5.wgsl) ---
const RGB9E5_EXP_BIAS: i32 = 15;
const RGB9E5_MANTISSA_BITS: i32 = 9;
const RGB9E5_MANTISSA_BITSU: u32 = 9u;
const RGB9E5_MANTISSA_VALUES: i32 = 512;
const MAX_RGB9E5_: f32 = 65408.0;

fn floor_log2_(x: f32) -> i32 {
    let f = bitcast<u32>(x);
    let biasedexponent = (f & 0x7F800000u) >> 23u;
    return i32(biasedexponent) - 127;
}

fn vec3_to_rgb9e5_(rgb_in: vec3<f32>) -> u32 {
    let rgb = clamp(rgb_in, vec3(0.0), vec3(MAX_RGB9E5_));
    let maxrgb = max(rgb.r, max(rgb.g, rgb.b));
    var exp_shared = max(-RGB9E5_EXP_BIAS - 1, floor_log2_(maxrgb)) + 1 + RGB9E5_EXP_BIAS;
    var denom = exp2(f32(exp_shared - RGB9E5_EXP_BIAS - RGB9E5_MANTISSA_BITS));
    let maxm = i32(floor(maxrgb / denom + 0.5));
    if (maxm == RGB9E5_MANTISSA_VALUES) {
        denom *= 2.0;
        exp_shared += 1;
    }
    let n = vec3<u32>(floor(rgb / denom + 0.5));
    return (u32(exp_shared) << 27u) | (n.b << 18u) | (n.g << 9u) | (n.r << 0u);
}

// --- vendored from bevy_pbr::deferred::pbr_deferred_types ---
fn pack_unorm4x8_(values: vec4<f32>) -> u32 {
    let v = vec4<u32>(saturate(values) * 255.0 + 0.5);
    return (v.w << 24u) | (v.z << 16u) | (v.y << 8u) | v.x;
}

fn pack_24bit_normal_and_flags(octahedral_normal: vec2<f32>, flags: u32) -> u32 {
    let unorm1 = u32(saturate(octahedral_normal.x) * 4095.0 + 0.5);
    let unorm2 = u32(saturate(octahedral_normal.y) * 4095.0 + 0.5);
    return (unorm1 & 0xFFFu) | ((unorm2 & 0xFFFu) << 12u) | ((flags & 0xFFu) << 24u);
}

// --- vendored from bevy_pbr::render::utils ---
fn octahedral_encode(v: vec3<f32>) -> vec2<f32> {
    var n = v / (abs(v.x) + abs(v.y) + abs(v.z));
    let octahedral_wrap = (1.0 - abs(n.yx)) * select(vec2(-1.0), vec2(1.0), n.xy > vec2f(0.0));
    let n_xy = select(octahedral_wrap, n.xy, n.z >= 0.0);
    return n_xy * 0.5 + 0.5;
}

// bevy_pbr::deferred::DEFAULT_PBR_DEFERRED_LIGHTING_PASS_ID (deferred/mod.rs:33)
const DEFAULT_PBR_DEFERRED_LIGHTING_PASS_ID: u32 = 1u;

struct FragOut {
    @location(0) deferred: vec4<u32>,
    @location(1) deferred_lighting_pass_id: u32,
    @builtin(frag_depth) depth: f32,
}

@fragment
fn fragment(@builtin(position) frag_coord: vec4<f32>) -> FragOut {
    let px = vec2<i32>(frag_coord.xy);
    let t = textureLoad(scratch_t, px, 0).r;

    if (t >= SKY_T) {
        // Miss: leave depth at the reverse-Z far plane (0.0) and write an
        // "unlit"-flagged gbuffer entry with pass id 0 so the deferred
        // lighting shader skips it (matches how Bevy treats untouched
        // background pixels).
        return FragOut(vec4<u32>(0u, 0u, 0u, 0u), 0u, 0.0);
    }

    // Reverse-Z depth reconstruction (mirrors hybrid_blit.wgsl, using Bevy's
    // own view.clip_from_world instead of a private view_proj uniform).
    let uv = frag_coord_to_uv(frag_coord.xy, view.viewport);
    let ndc = uv_to_ndc(uv);
    let near_clip = vec4<f32>(ndc, 1.0, 1.0);
    let near_world = view.world_from_clip * near_clip;
    let near_pos = near_world.xyz / near_world.w;
    let ro = view.world_position;
    let rd = normalize(near_pos - ro);
    let world_pos = ro + rd * t;
    let clip = view.clip_from_world * vec4<f32>(world_pos, 1.0);
    let depth = clamp(clip.z / clip.w, 0.0, 1.0);

    let n = textureLoad(scratch_normal, px, 0).xyz;
    let reflection = textureLoad(scratch_reflect, px, 0).rgb;

    let base_color_srgb = pow(sphere.base_color, vec3(1.0 / 2.2));
    // Real ray-traced reflection, baked into the emissive channel — stock
    // deferred lighting still adds its own direct diffuse/specular/shadow
    // term from `base_color`/`metallic`/`roughness` on top of this via
    // apply_pbr_lighting.
    //
    // ROOT CAUSE of an earlier blown-out/hard-edged look (found via a raw
    // luminance heatmap dump): `apply_pbr_lighting` scales DIRECT lighting
    // by `view.exposure` (pbr_functions.wgsl:864,
    // `view.exposure * (direct_light + ...)`, but emissive only gets that
    // same scaling when its packed alpha is 1.0
    // (`emissive_light * mix(1.0, view.exposure, emissive.a)`,
    // pbr_functions.wgsl:841) — and the DEFERRED G-buffer's own unpack path
    // hardcodes that alpha to 0.0 (`pbr.material.emissive = vec4(emissive,
    // 0.0)`, pbr_deferred_functions.wgsl:105), so deferred-path emissive is
    // NEVER exposure-scaled. Our raw point-light radiance (~10,000+ at a
    // couple meters, matching bevy_pbr's own un-exposed candela-scale
    // values) was landing in emissive completely unscaled while direct
    // lighting at the same raw magnitude was correctly compressed by
    // `view.exposure` — hence the sharp, saturated look wherever the
    // reflection ray hit the plane. Multiplying by `view.exposure` here
    // ourselves puts the reflection back on the same footing as direct
    // lighting.
    //
    // Weighted by Schlick Fresnel on top of that (view-angle-dependent:
    // near-grazing angles reflect close to 100%, straight-on reflects much
    // less) — physically appropriate, and incidentally also softens the
    // sphere's silhouette-against-the-plane transition (see `SdfProbeSphere`
    // in the orbit example for how it's tuned).
    let view_dir = normalize(view.world_position - world_pos);
    let f0 = mix(vec3<f32>(0.04) + sphere.reflectance * 0.12, base_color_srgb, sphere.metallic);
    let cos_theta = clamp(dot(normalize(n), view_dir), 0.0, 1.0);
    let fresnel = f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - cos_theta, 5.0);
    let emissive = reflection * view.exposure * fresnel;
    let diffuse_occlusion = 1.0;
    let clearcoat_props = 0.0;
    let props = pack_unorm4x8_(vec4(sphere.reflectance, sphere.metallic, diffuse_occlusion, clearcoat_props));
    let octahedral_normal = octahedral_encode(normalize(n));
    // bevy_pbr::pbr_deferred_types::DEFERRED_MESH_FLAGS_SHADOW_RECEIVER_BIT
    // (1u << 2u) — apply_pbr_lighting's fetch_*_shadow calls are gated on
    // this bit (see pbr_functions.wgsl's shadow-sampling branches); without
    // it the sphere would shade fully lit regardless of any shadow map.
    let flags = 4u;

    let deferred = vec4<u32>(
        pack_unorm4x8_(vec4(base_color_srgb, sphere.roughness)),
        vec3_to_rgb9e5_(emissive),
        props,
        pack_24bit_normal_and_flags(octahedral_normal, flags),
    );

    return FragOut(deferred, DEFAULT_PBR_DEFERRED_LIGHTING_PASS_ID, depth);
}
