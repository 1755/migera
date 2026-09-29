// Fresh-start hybrid trace (compute): one invocation per pixel.
//
// Faithful WGSL port of src/hybrid/cpu_ref.rs's trace()/march_object()/
// slab_hit()/sd_rounded_box() — same marching tolerances, same slab-test
// formula, same closest-hit-wins BVH traversal. Full metallic-roughness
// GGX Cook-Torrance shading (see sample_light/ggx_distribution/
// ggx_visibility/fresnel_schlick/shade below, ported verbatim from
// cpu_ref.rs::shade) against up to `scene.light_count` lights (directional/
// point/spot), each light's distance/spot-cone attenuation ported verbatim
// from hybrid_legacy's own shade() formula. No AO/shadows/reflections yet
// — a hit's finite-difference surface normal (local_normal), the matched
// object's PBR material fields, and each light's contribution are the
// entire lighting model this step implements. A miss reports the scene's
// flat background color unlit. See cpu_ref.rs's own doc comment for why
// marching (not exact analytic intersection) is this project's established
// approach for resolving a box hit.
//
// Ray generation mirrors hybrid_legacy_blit.wgsl's near-plane unprojection
// (Bevy 0.19's infinite reverse-Z makes far-plane unprojection divide by
// zero) using Bevy's own View uniform directly — no duplicated view_proj,
// see src/hybrid/extract.rs's module doc comment for why.
//
// Output: direct+emissive HDR color (out_color, rgba16float) + linear
// hit-t "depth" (out_depth, r32float) + world-space shading normal
// (normal_view, rgba32float) + indirect-diffuse color (indirect_view,
// rgba16float, kept separate from out_color so hybrid_denoise.wgsl can
// blur just this noisy term — see ShadeResult's doc comment below). The
// blit pass reconstructs reverse-Z frag_depth from the stored t and
// reads out_color as final composited color (the denoise pass adds the
// blurred indirect term back into out_color before blit ever runs).

#import bevy_render::view::{View, frag_coord_to_uv, uv_to_ndc}

@group(0) @binding(0) var<uniform> view: View;

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
    // Which indirect-diffuse technique is active this frame — see
    // src/hybrid/extract.rs's GiMethod/SceneUniform::gi_method doc
    // comments. Mirrors GiMethod's own repr(u32) discriminants exactly
    // via the GI_METHOD_* constants declared below.
    gi_method: u32,
    // SDF cone tracing (GI_METHOD_CONETRACE) — see
    // src/hybrid/extract.rs's SceneUniform's own per-field doc comments;
    // mirrored here field-for-field. No separate relight pass reads
    // these — cone tracing has no persistent structure, see
    // conetrace_ref.rs's own module doc comment.
    conetrace_half_angle: f32,
    conetrace_origin_radius: f32,
    conetrace_max_t: f32,
    conetrace_max_bounces: u32,
    // Multi-bounce specular reflections — see reflect_trace_ray's own doc
    // comment. Independent of gi_method (reflections apply regardless of
    // which diffuse-GI technique is active), mirrored field-for-field
    // from src/hybrid/extract.rs's ReflectionConfig/SceneUniform.
    reflection_enabled: u32,
    reflection_max_bounces: u32,
    reflection_fresnel_cutoff: f32,
    reflection_max_t: f32,
    // Multi-bounce transmission/refraction — see refract_trace_ray's own
    // doc comment. Mirrored field-for-field from src/hybrid/extract.rs's
    // TransmissionConfig/SceneUniform.
    transmission_enabled: u32,
    transmission_max_bounces: u32,
    transmission_fresnel_cutoff: f32,
    transmission_max_t: f32,
    // DDGI (GI_METHOD_DDGI) — see src/hybrid/extract.rs's SceneUniform's
    // own per-field doc comments; mirrored here field-for-field. Read by
    // shade() at shading time to sample the atlas hybrid_ddgi_relight.wgsl
    // already relit THIS frame (see this file's own group-2 bind group
    // doc comment for the read-before-write ordering).
    ddgi_probes_per_frame: u32,
    ddgi_total_probes: u32,
    ddgi_tile_size: u32,
    ddgi_frame_index: u32,
    ddgi_max_history_length: f32,
    ddgi_max_t: f32,
    // Stochastic (jittered-lens) depth of field — see
    // src/hybrid/dof_ref.rs's own module doc comment for the full
    // technique; mirrored field-for-field from src/hybrid/extract.rs's
    // SceneUniform.
    dof_enabled: u32,
    dof_focal_distance: f32,
    dof_aperture_radius: f32,
    dof_frame_index: u32,
    dof_max_history_length: f32,
    // Primary-ray sub-pixel jitter (TAAU experiment step 1a) — see
    // src/hybrid/taa_ref.rs's own module doc comment and generate_
    // primary_ray's own doc comment just below for the full rationale.
    // Mirrored field-for-field from src/hybrid/extract.rs's SceneUniform.
    jitter_enabled: u32,
    jitter_offset_x: f32,
    jitter_offset_y: f32,
    // This pass's own working resolution (RenderScaleConfig experiment,
    // step 1b) — see src/hybrid/extract.rs's SceneUniform::trace_size_x
    // doc comment. Used for BOTH this file's dispatch-bounds check
    // (trace_main) and ray-gen's own UV conversion.
    trace_size_x: u32,
    trace_size_y: u32,
}

const GI_METHOD_NONE: u32 = 0u;
const GI_METHOD_DDGI: u32 = 1u;
const GI_METHOD_RADIANCE_CASCADES: u32 = 2u;
const GI_METHOD_CONETRACE: u32 = 3u;

// Mirrors src/hybrid/extract.rs's ObjectGpu exactly (field order and
// type) — see that struct's doc comment for the per-shape_kind param
// layout `march_object` below dispatches on.
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
    // Reuses what was previously _pad_material0/_pad_material1 padding.
    transmission: f32, ior: f32,
    emissive_r: f32, emissive_g: f32, emissive_b: f32,
    _pad_emissive: f32,
    // Last frame's translation/inverse-rotation — used only by
    // trace_main's motion-vector computation (temporal_ref.rs::
    // reproject_world_point's WGSL mirror lives there, not here — see
    // this file's own motion-vector section below).
    prev_translation_x: f32, prev_translation_y: f32, prev_translation_z: f32,
    _pad_prev_translation: f32,
    prev_inv_rotation_x: f32, prev_inv_rotation_y: f32, prev_inv_rotation_z: f32, prev_inv_rotation_w: f32,
}

// Mirrors src/hybrid/extract.rs's ShapeKindGpu exactly.
const SHAPE_KIND_SPHERE: u32 = 0u;
const SHAPE_KIND_ROUNDED_BOX: u32 = 1u;
const SHAPE_KIND_ROUNDED_CYLINDER: u32 = 2u;
const SHAPE_KIND_CAPSULE: u32 = 3u;
const SHAPE_KIND_ELLIPSOID: u32 = 4u;
const SHAPE_KIND_BOX_FRAME: u32 = 5u;
const SHAPE_KIND_HEX_PRISM: u32 = 6u;

// Mirrors src/hybrid/bvh.rs's BvhNodeGpu exactly.
struct BvhNode {
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    left_or_sentinel: u32,
    right_or_object: u32,
}

// Mirrors src/hybrid/extract.rs's LightGpu exactly (field order and
// type) — see that struct's and cpu_ref.rs::Light's doc comments for
// what each field means per light kind.
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

// Mirrors src/hybrid/extract.rs's LightKindGpu exactly.
const LIGHT_KIND_DIRECTIONAL: u32 = 0u;
const LIGHT_KIND_POINT: u32 = 1u;
const LIGHT_KIND_SPOT: u32 = 2u;

@group(1) @binding(0) var<uniform> scene: SceneUniform;
@group(1) @binding(1) var<storage, read> objects: array<ObjectGpu>;
@group(1) @binding(2) var<storage, read> bvh: array<BvhNode>;
@group(1) @binding(3) var<storage, read> lights: array<LightGpu>;
@group(1) @binding(4) var out_color: texture_storage_2d<rgba16float, write>;
@group(1) @binding(5) var out_depth: texture_storage_2d<r32float, write>;
// Written by trace_main alongside out_color/out_depth, read by the
// denoise pass (hybrid_denoise.wgsl) as its edge-stop guides — see
// cpu_ref.rs::ShadeResult's doc comment for why the indirect term is
// kept in its own texture instead of being folded into out_color here.
// .xyz = world-space shading normal; .w = the PRIMARY (reflecting)
// surface's own roughness (see reflecting_roughness's own doc comment in
// trace_main below) — every existing reader of this texture only ever
// touches .rgb, so packing roughness into the otherwise-unused .w here is
// non-breaking; read by hybrid_temporal.wgsl's own reflect_temporal_main
// to gate virtual-point reprojection.
@group(1) @binding(6) var normal_view: texture_storage_2d<rgba32float, write>;
@group(1) @binding(7) var indirect_view: texture_storage_2d<rgba16float, write>;
// Stores this pixel's REPROJECTED PREVIOUS-FRAME WORLD position, NOT yet
// a screen-space UV delta — trace_main only has this frame's `View`
// uniform bound (no PreviousViewData in this pass's own bind group,
// unlike hybrid_temporal.wgsl's), so it can undo/reapply the per-OBJECT
// rigid transform (temporal_ref.rs::reproject_world_point's WGSL mirror,
// below) but cannot finish the reprojection into last frame's screen
// space itself — that final "world_to_previous_uv" step
// (temporal_ref.rs's own function of that name) runs in
// hybrid_temporal.wgsl, which DOES have PreviousViewData bound. Stored
// as an rgba32float world-space XYZ (not rg16float UV) for exactly this
// reason: the value here is a world position, not yet a UV.
// `.w` carries refracting_roughness (the entry surface's own roughness,
// for hybrid_temporal.wgsl's transmit_temporal_main gate) — mirrors
// normal_view's own reuse of its previously-unused .w channel for
// reflecting_roughness; motion_view's own .w was likewise always 1.0/
// unused before this (confirmed via grep: no reader ever touches it).
@group(1) @binding(8) var motion_view: texture_storage_2d<rgba32float, write>;
// Multi-bounce specular reflection color (ShadeResult.reflect) — kept
// SEPARATE from indirect_view since it needs its own temporal history,
// see reflect_trace_ray's own doc comment.
@group(1) @binding(9) var reflect_view: texture_storage_2d<rgba16float, write>;
// The REFLECTED hit's own reprojected previous-frame world position (a
// "virtual point" reflecting whatever geometry the reflection ray
// actually hit, NOT the reflecting surface's own motion) — same two-pass
// reprojection split as motion_view above.
@group(1) @binding(10) var reflect_motion_view: texture_storage_2d<rgba32float, write>;
// Multi-bounce transmission/refraction color (ShadeResult.refract) — kept
// SEPARATE from both indirect_view and reflect_view, mirroring reflect_view's
// own "needs its own temporal history" rationale (a refracted image's own
// motion follows neither the entry surface's motion nor reflection's own
// virtual point).
@group(1) @binding(11) var refract_view: texture_storage_2d<rgba16float, write>;
// The transmitted ray's own first-EXIT hit reprojected previous-frame
// world position — same "virtual point" idea as reflect_motion_view,
// keyed to the EXIT surface (possibly a different, moving object) rather
// than the entry surface.
@group(1) @binding(12) var refract_motion_view: texture_storage_2d<rgba32float, write>;

// Mirrors src/hybrid/pipeline.rs's own DdgiGridUniform exactly — grid
// placement + exact-fit atlas layout, grid-config-scoped rather than
// per-frame, kept as its own small uniform rather than folded into
// SceneUniform.
struct DdgiGridUniform {
    origin_x: f32, origin_y: f32, origin_z: f32,
    _pad0: f32,
    spacing_x: f32, spacing_y: f32, spacing_z: f32,
    tiles_per_row: u32,
    dims_x: u32, dims_y: u32, dims_z: u32,
    _pad2: u32,
}

// Read-only access to the DDGI atlas that hybrid_ddgi_relight.wgsl
// already relit THIS frame (relight runs first in hybrid_pass, before
// trace — see pass::hybrid_pass's own dispatch ordering) — see
// pipeline.rs's own hybrid_trace_ddgi_read_layout doc comment for why
// this is a separate group (its own bind group, not folded into group 1
// above) and why it's bound unconditionally regardless of which
// GiMethod is active (this file's own shade() only SAMPLES it when
// scene.gi_method == GI_METHOD_DDGI).
@group(2) @binding(0) var<storage, read> ddgi_atlas: array<vec4<f32>>;
// Same row-major indexing as ddgi_atlas above — per-texel
// (mean_distance, mean_distance_squared), the Chebyshev depth-visibility
// test's own input. See src/hybrid/pipeline.rs's own
// HybridDdgiAtlas::distance_atlas doc comment for the full rationale.
@group(2) @binding(1) var<storage, read> ddgi_distance_atlas: array<vec2<f32>>;
@group(2) @binding(2) var<uniform> ddgi_grid: DdgiGridUniform;

// Mirrors src/hybrid/pipeline.rs's own CascadeLevelUniform exactly — see
// hybrid_radiance_cascades.wgsl's own copy of this same struct for the
// full field-meaning doc comment. Read-only here: this frame's own
// hybrid_radiance_cascades_relight_main pass already relit every level's
// texels before trace_main runs (same read-before-write ordering DDGI's
// own group-2 already established) — this file only SAMPLES it, via
// radiance_cascades_sample_hierarchy below, when
// scene.gi_method == GI_METHOD_RADIANCE_CASCADES.
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

const RADIANCE_CASCADES_LEVEL_COUNT: u32 = 4u;

// A separate bind group (group 3, not folded into group 2's own DDGI
// bindings) since the two techniques' own atlas shapes are unrelated —
// bound unconditionally regardless of which GiMethod is active, same
// "fixed at pipeline-creation time, must always be bound to something
// valid" reasoning group 2's own doc comment already gives; shade() only
// actually SAMPLES this when scene.gi_method == GI_METHOD_RADIANCE_CASCADES.
@group(3) @binding(0) var<storage, read> radiance_cascades_atlas: array<vec4<f32>>;
@group(3) @binding(1) var<storage, read> radiance_cascades_levels: array<CascadeLevelUniform, 4>;

// --- cpu_ref.rs's named constants, ported verbatim ------------------------

const MAX_MARCH_STEPS: u32 = 128u;
const HIT_EPSILON: f32 = 1e-4;
const LEAF_SENTINEL: u32 = 4294967295u; // u32::MAX

// Sentinel linear-t written to out_depth on a miss — the blit shader reads
// this back to know "sky, use the far plane" instead of a real hit
// distance. Named analogously to hybrid_legacy's SKY_T; kept as its own
// constant (not a shared import) since WGSL has no cross-file constant
// sharing here and this value must match hybrid_blit.wgsl's own copy
// exactly (documented there too).
const SKY_T: f32 = 1e6;

// Worst-case traversal stack depth is bounded by tree height: the
// SAH-bucketed builder (src/hybrid/bvh.rs's `recursive_sah`, falling back
// to a strict by-count median split on degenerate spans) always bisects
// its input span, so height is O(log2 N) even in the worst case the
// fallback can produce — not just the common case. log2(1,000,000) ~= 20,
// a scene size far beyond anything this renderer spawns today (the
// --stress harness tops out at 10,000, depth ~14). 64 leaves real
// headroom above that while staying a trivially small shader-local array
// (64 * 4 bytes = 256 bytes). Previously 32, which was fine for every
// scene actually run but left silent-overflow (see below) with much less
// margin than the math supports — raised here since the cost of doing so
// is negligible and the alternative (a tree deep enough to overflow 32)
// would silently drop real geometry from traversal with zero signal.
const MAX_STACK: u32 = 64u;

// --- cpu_ref.rs's sd_* functions, ported verbatim (see each function's
// counterpart in cpu_ref.rs for its own source/provenance doc comment) ---

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

// This object's signed distance from p_local, dispatching on shape_kind —
// mirrors cpu_ref.rs::local_distance's match exactly (see its doc comment
// for why RoundedCone has no case here: a real, pre-existing bug in
// sdf::primitives::RoundedCone::distance, out of scope to fix here).
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

// cpu_ref.rs::local_normal (verbatim central-difference gradient) --------
//
// A 4-tap "tetrahedron" pattern and closed-form per-shape analytic
// gradients were both tried and measured against this method at
// --stress 10000 — all three were statistically indistinguishable in
// GPU cost (BVH traversal + marching dominates total frame time so
// heavily that per-pixel normal-computation cost doesn't move the
// needle at this renderer's current scale), so both alternatives were
// removed rather than kept as unused shader code. See PROGRESS.md's
// "Analytic vs. finite-difference normals" entry for the full
// comparison and measured numbers.

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

// Rotate a vector by the inverse of a quaternion (x, y, z, w) — same
// formula WGSL callers elsewhere in this project use for quat*vec3
// (hybrid_legacy_trace.wgsl's leaf_distance), applied here to
// (world_p - translation) exactly like cpu_ref::march_object's
// `inv_rotation * (p_world - object.translation)`.
fn rotate_by_quat(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    return q.xyz * 2.0 * dot(q.xyz, v) + v * (q.w * q.w - dot(q.xyz, q.xyz)) + cross(q.xyz, v) * 2.0 * q.w;
}

// A unit quaternion's inverse is its conjugate: negate the vector part,
// keep w. Used to recover an object's forward rotation from the
// inv_rotation ObjectGpu actually stores (march_object only ever needs
// the inverse, see its own doc comment — the forward rotation is only
// needed here, to bring a local-space normal back into world space).
fn quat_conjugate(q: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(-q.xyz, q.w);
}

// --- cpu_ref.rs::march_object (verbatim logic) ----------------------------

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
// via pixel_eps (see that function's own doc comment for why: a hit far
// away doesn't need HIT_EPSILON-tight convergence since the position error
// is already sub-pixel, and this directly cuts step count on distant
// geometry).
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

// --- cpu_ref.rs::slab_hit (verbatim branchless Kay-Kajiya formula) --------

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

// --- cpu_ref.rs::sample_light / shade (GGX Cook-Torrance BRDF, ported from
// cpu_ref.rs's own port of hybrid_legacy's shade() — see cpu_ref.rs::shade's
// doc comment for why this deliberately does NOT replicate hybrid_legacy's
// verbatim formula: that shader double-applies F0/Fresnel in its specular
// term (a real, confirmed bug, left as-is there per this project's frozen-
// legacy-code convention), whereas this port applies F0 exactly once,
// matching real bevy_pbr source (crates/bevy_pbr/src/render/
// pbr_lighting.wgsl).) ------------------------------------------------

const EXPOSURE: f32 = 0.0005;

struct LightSample {
    to_light: vec3<f32>,
    radiance: vec3<f32>,
    shadow_max_t: f32,
}

// cpu_ref.rs::DIRECTIONAL_SHADOW_MAX_T verbatim — a fixed, scene-scale-
// tuned distance (NOT the scene's BVH root diagonal, NOR hybrid_legacy's
// own 60.0 — both were tried and found to be real bugs at --stress N: a
// shadow ray at this scene's ~70° sun elevation travels roughly
// max_t*0.36 horizontally before reaching max_t, so 60.0 reaches ~20
// units, crossing into neighboring --stress grid cells and picking up
// their real-but-locally-irrelevant geometry as shadow candidates, with
// shadow_candidate_margin padding those distant candidates' AABBs
// further still. 12.0 is generous headroom for any locally-relevant
// occluder while staying under this scene's grid-cell spacing — see
// cpu_ref.rs::DIRECTIONAL_SHADOW_MAX_T's doc comment for the full
// writeup and why this is a real, scene-scale-dependent tuning parameter,
// not a universal constant).
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
            // See cpu_ref.rs::sample_light's doc comment for why this is
            // `dot(-to_light, spot_direction)` with no extra negation on
            // spot_direction — a real bug caught by that module's own
            // cargo test before it ever reached WGSL.
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

// cpu_ref.rs::ggx_distribution verbatim (Trowbridge-Reitz/GGX normal
// distribution function).
fn ggx_distribution(n_dot_h: f32, alpha: f32) -> f32 {
    let alpha_sq = alpha * alpha;
    let denom = n_dot_h * n_dot_h * (alpha_sq - 1.0) + 1.0;
    return alpha_sq / max(3.14159265 * denom * denom, 1e-8);
}

// cpu_ref.rs::ggx_visibility verbatim (height-correlated Smith visibility,
// folded with the 1/(4 NdotV NdotL) Cook-Torrance denominator).
fn ggx_visibility(n_dot_l: f32, n_dot_v: f32, alpha: f32) -> f32 {
    let alpha_sq = alpha * alpha;
    let lambda_v = n_dot_l * sqrt(n_dot_v * n_dot_v * (1.0 - alpha_sq) + alpha_sq);
    let lambda_l = n_dot_v * sqrt(n_dot_l * n_dot_l * (1.0 - alpha_sq) + alpha_sq);
    return 0.5 / max(lambda_v + lambda_l, 1e-4);
}

// cpu_ref.rs::fresnel_schlick verbatim.
fn fresnel_schlick(f0: vec3<f32>, cos_theta: f32) -> vec3<f32> {
    let m = clamp(1.0 - cos_theta, 0.0, 1.0);
    return f0 + (vec3<f32>(1.0) - f0) * pow(m, 5.0);
}

// cpu_ref.rs::dielectric_f0 verbatim.
fn dielectric_f0(reflectance: f32) -> f32 {
    return 0.16 * reflectance * reflectance;
}

// --- cpu_ref.rs::trace_shadow (soft shadows: Aaltonen-refined k*h/t,
// ported from hybrid_legacy's VALIDATED, GPU-probe-cross-checked fixed
// state — not the buggy draft that motivated this project's fresh-start
// rewrite. Two coupled fixes: march to the ray's real max_t rather than a
// BVH candidate's own AABB slab exit, bounded by a scale-relative
// divergence early-out; and pad every leaf AABB before the slab test by a
// margin derived from THIS RAY's own max_t (NOT the whole scene's
// root_diagonal — see cpu_ref.rs::shadow_candidate_margin's doc comment
// for why root_diagonal was a real bug at --stress N: it grows
// unboundedly with scene size, producing spurious huge dark shadow bands
// across unrelated ground cells), or a ray that never enters an
// occluder's tight AABB gets zero candidates and reads unconditionally
// fully lit. See cpu_ref.rs::trace_shadow's doc comment for the full
// writeup this mirrors.) --------------------------------

const VIS_CUTOFF: f32 = 0.02;
const DIVERGENCE_FACTOR: f32 = 2.5;
const MAX_SHADOW_CANDIDATES: u32 = 64u;

// cpu_ref.rs::shadow_candidate_margin verbatim — PENUMBRA_REACH is a
// FIXED distance sized to this renderer's object scale (NOT this ray's
// max_t, NOR the scene's root_diagonal — both were tried and found to be
// real bugs; see cpu_ref.rs::shadow_candidate_margin's doc comment for
// the full writeup, including the numerically-confirmed failing case).
const PENUMBRA_REACH: f32 = 15.75;

fn shadow_candidate_margin(k: f32) -> f32 {
    return VIS_CUTOFF * k * PENUMBRA_REACH;
}

struct ShadowCandidate {
    obj_id: u32,
    near: f32,
}

// Descends the whole BVH collecting every leaf whose MARGIN-PADDED AABB
// (leaves AND internal nodes both — see the internal-node-padding bug
// this fixed, below) the ray's slab test intersects within [t_min,
// t_max], into a fixed-size array (WGSL has no dynamic Vec) — mirrors
// cpu_ref.rs::gather_candidates_padded. Silently truncates past
// MAX_SHADOW_CANDIDATES, which is generously sized above any realistic
// per-ray candidate count at this renderer's current scale (mirrors the
// same "wide margin above any real scene" reasoning MAX_STACK's own doc
// comment already established for BVH traversal).
//
// Internal nodes are padded too — a real bug, not present in
// hybrid_legacy's original (its shallow demo-scene trees apparently
// never exposed it): an internal node's bounds are the TIGHT union of
// its children's own tight bounds, so if only leaves were padded, a ray
// whose true path only entered a leaf's PADDED margin (missing the
// leaf's own tight box) could ALSO miss that leaf's parent's tight box,
// pruning the whole subtree before the leaf's own (correctly padded)
// test ever ran. Visually this produced a sharp "cut" on one side of a
// curved shape's soft shadow at --stress N — see
// cpu_ref.rs::gather_candidates_padded's doc comment for the full
// writeup, including a real ancestor-chain trace that pinned this down.
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

// cpu_ref.rs::trace_shadow verbatim (see this section's header comment for
// the two coupled fixes this reproduces). `origin_obj_id`/`has_origin`:
// the shaded object is fully excluded from its own candidate slab for its
// own shadow ray (single-leaf-only scope — no multi-record CSG objects in
// this renderer, matching cpu_ref.rs's own simplified scope).
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
            // cpu_ref.rs::trace_shadow's margin_fade verbatim — fades
            // toward fully lit as h approaches margin, removing the
            // visible polygonal seam at the padded-candidate-AABB
            // boundary (see cpu_ref.rs's own comment on this line for the
            // full writeup and how it was confirmed).
            let margin_fade = smoothstep(margin * 0.5, margin, h);
            let faded_vis = raw_vis + (1.0 - raw_vis) * margin_fade;
            vis = min(vis, faded_vis);
            // cpu_ref.rs::trace_shadow's own margin_fade/VIS_CUTOFF
            // interaction fix verbatim — see that function's own comment
            // on this exact line for the full writeup: without this,
            // a candidate whose march pushes vis below VIS_CUTOFF right
            // here falls through to this loop's own top-of-iteration
            // `vis > VIS_CUTOFF` check, exits, and this function returns
            // ShadowResult(false, vis) at its own end with THIS sample's
            // small nonzero leftover value instead of the fully-opaque
            // 0.0 a candidate this far below VIS_CUTOFF should report.
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

// --- cpu_ref.rs::trace (verbatim iterative stack-based BVH descent) -------
// Relocated here (was previously defined after `shade`) so `shade` can call
// the new indirect-diffuse functions below, which themselves call `trace` —
// WGSL requires declaration before use, and `trace` has no forward
// dependency on anything defined between its old and new location.

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
    var sp: u32 = 0u; // number of entries currently on the stack
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
            continue; // ray misses this node's box entirely; skip its whole subtree
        }

        if (node.left_or_sentinel == LEAF_SENTINEL) {
            let obj_id = node.right_or_object;
            if (obj_id >= scene.object_count) {
                continue; // stale/out-of-range leaf; skip (mirrors cpu_ref's "no matching scene data" guard)
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

        // Push the two children ordered so the nearer one pops first
        // (stack is LIFO: pushed last = popped first) — mirrors
        // cpu_ref.rs::trace's near/far push ordering exactly. Visiting
        // the closer subtree first tightens best_t sooner, so the
        // farther subtree's own slab test (evaluated once it's actually
        // popped) is more likely to already exceed the shrunk best_t and
        // get skipped outright. Same final result either way (traversal
        // order never changes which object wins, only how much dead
        // subtree work is skipped getting there) — this matters at this
        // renderer's current scale (tree depth ~14 at 20,000 objects,
        // where an unordered descent lets grazing/near-camera rays visit
        // far-before-near subtrees well before best_t narrows). A child
        // whose box the ray misses entirely is not pushed at all,
        // mirroring the early-skip already applied to popped nodes above.
        //
        // If the push guard below ever fails, that push is silently
        // skipped and the subtree is dropped from traversal — a wrong
        // (under-)result, not a crash, since a stack overrun in WGSL has
        // no defined recovery. This is only safe because MAX_STACK's own
        // doc comment establishes a real, by-construction depth bound
        // (SAH bisects every span; see there) with a wide margin above
        // any scene size this renderer spawns — not because overflow
        // here is otherwise handled.
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

// cpu_ref.rs::any_hit verbatim (see its own doc comment for the full
// rationale): "does anything other than exclude_obj_id converge somewhere
// in [0, t_max] along this ray?" — for occlusion-only callers that never
// read anything but a boolean (DDGI's per-probe hard-occlusion check is
// the motivating case). Unlike trace() above, this never shrinks its own
// t_max as hits are found — the FIRST convergence against a non-excluded
// object is immediately conclusive, so the whole traversal returns right
// there instead of continuing to search for the globally nearest hit.
// exclude_obj_id mirrors every existing occlusion call site's own
// `hit.obj_id != origin_obj_id` self-exclusion, baked into the traversal:
// a march that converges against the excluded object does NOT stop the
// search, it keeps going exactly as if that leaf had reported no hit —
// see cpu_ref.rs::any_hit's own doc comment for why a caller-side check
// after the fact is NOT equivalent (it can stop at a self-graze and never
// learn a real, separate occluder sits farther along the same ray).
// exclude_obj_id has no "no exclusion" sentinel distinct from a valid
// object id — every call site here always has a real origin object to
// exclude (an occlusion ray is always cast FROM some shaded surface), so
// callers with nothing to exclude pass scene.object_count (never a valid
// index) rather than needing an Option-like wrapper WGSL has no type for.
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

// ---------------------------------------------------------------------------------
// Hemisphere convolution: ddgi_tangent_basis / HEMISPHERE_SAMPLES —
// originally DDGI's own (ddgi_ref.rs::ddgi_tangent_basis /
// HEMISPHERE_SAMPLES verbatim), kept after DDGI's own removal because
// cone_trace_indirect below fires this SAME 5-sample cosine-weighted
// hemisphere bundle to gather indirect light around a shaded point's
// own normal — see that function's own doc comment.
// ---------------------------------------------------------------------------------

fn ddgi_tangent_basis(n: vec3<f32>) -> mat3x3<f32> {
    let s = select(-1.0, 1.0, n.z >= 0.0);
    let a = -1.0 / (s + n.z);
    let b = n.x * n.y * a;
    let t = vec3<f32>(1.0 + s * n.x * n.x * a, s * b, -s * n.x);
    let bt = vec3<f32>(b, s + n.y * n.y * a, -n.y);
    return mat3x3<f32>(t, bt, n);
}

// The exact literal values recovered from git history (commit 19a4cbb,
// Stage B's own removed INDIRECT_SAMPLES), not a new guess.
const HEMISPHERE_SAMPLE_COUNT: u32 = 5u;
const HEMISPHERE_SAMPLES: array<vec3<f32>, 5> = array<vec3<f32>, 5>(
    vec3<f32>(0.0, 0.0, 1.0),
    vec3<f32>(0.6614, 0.0, 0.75),
    vec3<f32>(-0.2044, 0.6285, 0.75),
    vec3<f32>(-0.5350, -0.3886, 0.75),
    vec3<f32>(0.5350, -0.3886, 0.75),
);

// ---------------------------------------------------------------------------------
// DDGI shading-time sampling: trilinear interpolation over the 8 probes
// surrounding a shaded point, each gated by a single occlusion ray — a
// faithful WGSL port of ddgi_ref.rs::sample_probe_grid, see that
// function's own doc comment (and ddgi_ref.rs's own module doc comment)
// for the full design/rationale. Reads ddgi_atlas (this frame's
// just-relit probe irradiance atlas, group 2) rather than firing fresh
// rays at shading time — the expensive ray-tracing already happened in
// hybrid_ddgi_relight.wgsl, spread across many frames; shading-time
// sampling here is just texture reads + a visibility gate.
// ---------------------------------------------------------------------------------

// ddgi_ref.rs::octahedral_decode verbatim (this file's own copy — see
// hybrid_ddgi_relight.wgsl's own header comment for why WGSL files in
// this renderer duplicate rather than share such logic).
fn ddgi_octahedral_decode(uv: vec2<f32>) -> vec3<f32> {
    let f = uv * 2.0 - vec2<f32>(1.0, 1.0);
    var n = vec3<f32>(f.x, f.y, 1.0 - abs(f.x) - abs(f.y));
    let t = max(-n.z, 0.0);
    let sx = select(-1.0, 1.0, n.x >= 0.0);
    let sy = select(-1.0, 1.0, n.y >= 0.0);
    n.x = n.x - t * sx;
    n.y = n.y - t * sy;
    return normalize(n);
}

// ddgi_ref.rs::AtlasLayout::tile_origin verbatim.
fn ddgi_atlas_tile_origin(flat_index: u32, tile_size: u32) -> vec2<i32> {
    let tiles_per_row = max(ddgi_grid.tiles_per_row, 1u);
    let row = flat_index / tiles_per_row;
    let col = flat_index % tiles_per_row;
    return vec2<i32>(i32(col * tile_size), i32(row * tile_size));
}

// Row-major index into the ddgi_atlas storage buffer — mirrors
// hybrid_ddgi_relight.wgsl's own atlas_index exactly (same square
// atlas_width = tiles_per_row * tile_size derivation, no separate
// atlas_pixels uniform field needed).
fn ddgi_atlas_index(texel: vec2<i32>, tile_size: u32) -> u32 {
    let atlas_width = i32(max(ddgi_grid.tiles_per_row, 1u) * tile_size);
    return u32(texel.y * atlas_width + texel.x);
}

// Reads probe flat_index's own stored irradiance for `direction` —
// octahedral-encodes the direction to find the texel within that
// probe's own atlas tile (ddgi_ref.rs::direction_to_texel verbatim),
// then loads it. No filtering — exact texel loads, same convention
// every other pass in this renderer already establishes.
fn ddgi_probe_irradiance(flat_index: u32, direction: vec3<f32>, tile_size: u32) -> vec3<f32> {
    var n = direction / (abs(direction.x) + abs(direction.y) + abs(direction.z));
    let octahedral_wrap = vec2<f32>((1.0 - abs(n.y)) * select(-1.0, 1.0, n.x >= 0.0), (1.0 - abs(n.x)) * select(-1.0, 1.0, n.y >= 0.0));
    let n_xy = select(octahedral_wrap, n.xy, n.z >= 0.0);
    let uv = n_xy * 0.5 + vec2<f32>(0.5, 0.5);
    let max_index = f32(tile_size) - 1.0;
    let texel_x = i32(min(uv.x * f32(tile_size), max_index));
    let texel_y = i32(min(uv.y * f32(tile_size), max_index));
    let tile_origin = ddgi_atlas_tile_origin(flat_index, tile_size);
    let texel_index = ddgi_atlas_index(tile_origin + vec2<i32>(texel_x, texel_y), tile_size);
    return ddgi_atlas[texel_index].rgb;
}

// Same octahedral texel lookup as ddgi_probe_irradiance above, reading
// ddgi_distance_atlas instead of ddgi_atlas — returns (mean_distance,
// mean_distance_squared) for the Chebyshev depth-visibility test below.
fn ddgi_probe_distance_moments(flat_index: u32, direction: vec3<f32>, tile_size: u32) -> vec2<f32> {
    var n = direction / (abs(direction.x) + abs(direction.y) + abs(direction.z));
    let octahedral_wrap = vec2<f32>((1.0 - abs(n.y)) * select(-1.0, 1.0, n.x >= 0.0), (1.0 - abs(n.x)) * select(-1.0, 1.0, n.y >= 0.0));
    let n_xy = select(octahedral_wrap, n.xy, n.z >= 0.0);
    let uv = n_xy * 0.5 + vec2<f32>(0.5, 0.5);
    let max_index = f32(tile_size) - 1.0;
    let texel_x = i32(min(uv.x * f32(tile_size), max_index));
    let texel_y = i32(min(uv.y * f32(tile_size), max_index));
    let tile_origin = ddgi_atlas_tile_origin(flat_index, tile_size);
    let texel_index = ddgi_atlas_index(tile_origin + vec2<i32>(texel_x, texel_y), tile_size);
    return ddgi_distance_atlas[texel_index];
}

// ---------------------------------------------------------------------------------
// Hemisphere convolution: ddgi_ref.rs::ddgi_tangent_basis /
// HEMISPHERE_SAMPLES / cosine_weighted_probe_irradiance verbatim — see
// that file's own header comment (right before sample_probe_grid) for
// the full rationale: a probe's own single-texel read for one exact
// direction is a raw single-ray sample, not convolved irradiance, which
// left away-facing surfaces pitch black and the overall indirect result
// too dim. Gathering 5 cosine-weighted samples around the read
// direction at shading time (not scattering at relight/write time —
// each atlas texel is written by exactly one GPU invocation in
// hybrid_ddgi_relight.wgsl, so scattering would be a real cross-
// invocation write race) fixes this without touching the write path at
// all.
// ---------------------------------------------------------------------------------

// Reuses ddgi_tangent_basis/HEMISPHERE_SAMPLES/HEMISPHERE_SAMPLE_COUNT
// already declared above (this file's own cone-tracing section kept
// them after DDGI's original removal, since cone_trace_indirect fires
// the identical 5-sample cosine-weighted hemisphere bundle — see that
// section's own header comment) rather than duplicating a second
// same-valued copy under a new name, since WGSL disallows two
// definitions of the same function/const name in one module scope
// (unlike separate Rust modules, where ddgi_ref.rs and conetrace_ref.rs
// each keep their own private copy).

// ddgi_ref.rs::cosine_weighted_probe_irradiance_roughness_aware verbatim
// — pulls each HEMISPHERE_SAMPLES direction toward the pole (0,0,1) by
// (1.0 - roughness) before transforming into world space, so a fully
// rough surface (roughness=1.0) samples the same full spread this
// renderer always used before roughness-aware sampling existed, while a
// fully smooth/glossy surface (roughness=0.0) converges onto a single
// mirror-like direction. Answers the "GI bounds disperse based on
// reflection level" request: a glossy surface's own indirect response
// looks tighter/more directional than a rough one's. The Rust CPU-ref
// (ddgi_ref.rs) keeps BOTH this function and the plain, non-roughness-
// aware `cosine_weighted_probe_irradiance` side by side (the plain one
// still has its own direct caller/test) — this WGSL file has no other
// caller for the plain variant, so only the roughness-aware one is
// ported here, per this codebase's own "no dead code" convention.
fn ddgi_cosine_weighted_probe_irradiance_roughness_aware(flat_index: u32, surface_normal: vec3<f32>, tile_size: u32, roughness: f32) -> vec3<f32> {
    let spread = clamp(roughness, 0.0, 1.0);
    let pole = vec3<f32>(0.0, 0.0, 1.0);
    let basis = ddgi_tangent_basis(surface_normal);
    var acc = vec3<f32>(0.0);
    for (var i = 0u; i < HEMISPHERE_SAMPLE_COUNT; i = i + 1u) {
        let narrowed = normalize(mix(HEMISPHERE_SAMPLES[i], pole, 1.0 - spread));
        let world_dir = normalize(basis * narrowed);
        acc = acc + ddgi_probe_irradiance(flat_index, world_dir, tile_size);
    }
    return acc / f32(HEMISPHERE_SAMPLE_COUNT);
}

// Same hemisphere convolution as ddgi_cosine_weighted_probe_irradiance
// above, over distance moments instead of irradiance —
// ddgi_ref.rs::cosine_weighted_probe_irradiance_scalar_pair verbatim
// (WGSL's vec2 stands in for the (f32, f32) tuple that function uses,
// since WGSL has no tuple type).
fn ddgi_cosine_weighted_probe_distance_moments(flat_index: u32, surface_normal: vec3<f32>, tile_size: u32) -> vec2<f32> {
    let basis = ddgi_tangent_basis(surface_normal);
    var acc = vec2<f32>(0.0, 0.0);
    for (var i = 0u; i < HEMISPHERE_SAMPLE_COUNT; i = i + 1u) {
        let world_dir = normalize(basis * HEMISPHERE_SAMPLES[i]);
        acc = acc + ddgi_probe_distance_moments(flat_index, world_dir, tile_size);
    }
    return acc / f32(HEMISPHERE_SAMPLE_COUNT);
}

// Floor under Chebyshev's own variance term — see ddgi_ref.rs::
// CHEBYSHEV_VARIANCE_FLOOR's own doc comment for why (a probe whose
// relit rays all reported nearly identical distances has variance
// approaching exactly zero, which would make the weight formula divide
// by a near-zero denominator).
const CHEBYSHEV_VARIANCE_FLOOR: f32 = 1e-3;

// ddgi_ref.rs::chebyshev_visibility_weight verbatim.
fn chebyshev_visibility_weight(dist: f32, mean: f32, mean_sq: f32) -> f32 {
    if (dist <= mean) {
        return 1.0;
    }
    let variance = max(mean_sq - mean * mean, CHEBYSHEV_VARIANCE_FLOOR);
    let delta = dist - mean;
    return variance / (variance + delta * delta);
}

// ddgi_ref.rs::probe_grid_cell verbatim: lower-corner cell coords
// (clamped to [0, dims-2] so the 8-probe neighborhood below never
// indexes past the grid) plus fractional trilinear weights within that
// cell.
fn ddgi_probe_grid_cell(world_pos: vec3<f32>) -> vec3<u32> {
    let grid_origin = vec3<f32>(ddgi_grid.origin_x, ddgi_grid.origin_y, ddgi_grid.origin_z);
    let grid_spacing = vec3<f32>(ddgi_grid.spacing_x, ddgi_grid.spacing_y, ddgi_grid.spacing_z);
    let local = (world_pos - grid_origin) / max(grid_spacing, vec3<f32>(1e-4));
    let max_cell = vec3<i32>(i32(ddgi_grid.dims_x) - 2, i32(ddgi_grid.dims_y) - 2, i32(ddgi_grid.dims_z) - 2);
    let cell_x = clamp(i32(floor(max(local.x, 0.0))), 0, max(max_cell.x, 0));
    let cell_y = clamp(i32(floor(max(local.y, 0.0))), 0, max(max_cell.y, 0));
    let cell_z = clamp(i32(floor(max(local.z, 0.0))), 0, max(max_cell.z, 0));
    return vec3<u32>(u32(cell_x), u32(cell_y), u32(cell_z));
}

// ddgi_ref.rs::ProbeGrid::probe_position_flat verbatim (dims-index form,
// not the flat-index form — this pass already has (x,y,z) grid
// coordinates from ddgi_probe_grid_cell's own corner loop below).
fn ddgi_probe_position(x: u32, y: u32, z: u32) -> vec3<f32> {
    return vec3<f32>(ddgi_grid.origin_x, ddgi_grid.origin_y, ddgi_grid.origin_z)
        + vec3<f32>(ddgi_grid.spacing_x, ddgi_grid.spacing_y, ddgi_grid.spacing_z) * vec3<f32>(f32(x), f32(y), f32(z));
}

fn ddgi_probe_flat_index(x: u32, y: u32, z: u32) -> u32 {
    return x + y * ddgi_grid.dims_x + z * ddgi_grid.dims_x * ddgi_grid.dims_y;
}

// The weight_sum value at which the fallback ramp below fully
// saturates to the pure trilinear-weighted-average path — ddgi_ref.rs's
// own FALLBACK_RAMP_WEIGHT constant, verbatim. See
// ddgi_sample_probe_grid's own comment for the full flicker-fix
// rationale.
const DDGI_FALLBACK_RAMP_WEIGHT: f32 = 0.2;

// ddgi_ref.rs::sample_probe_grid verbatim: trilinear blend over the 8
// probes surrounding world_pos, each visibility-gated by ONE occlusion
// ray (see this section's own header comment — no per-probe Chebyshev
// depth/visibility texture in this first pass). Falls back TOWARD the
// unweighted average of all 8 raw samples if few/none are visible (a
// shaded point near/fully enclosed by geometry), rather than returning
// black. origin_obj_id excludes the shaded object itself from its own
// occlusion query (mirrors trace_shadow's own exclusion) — a real bug
// without this: a rotating object's own occlusion ray toward a probe
// can graze back across its own geometry at specific rotation angles,
// self-intersecting and zeroing that probe for exactly the frames the
// rotation puts the ray in the self-clipping regime.
//
// The fallback is a smooth ramp (DDGI_FALLBACK_RAMP_WEIGHT), not a hard
// branch on weight_sum > 0 — a real, found-by-direct-visual-inspection
// flicker bug: the original version switched formulas outright at
// weight_sum == 0. smoothstep(0, DDGI_FALLBACK_RAMP_WEIGHT, weight_sum)
// blends continuously between the two formulas instead. See
// ddgi_ref.rs::sample_probe_grid's own doc comment for the full
// rationale (this is a verbatim mirror).
fn ddgi_sample_probe_grid(world_pos: vec3<f32>, surface_normal: vec3<f32>, roughness: f32, tile_size: u32, origin_obj_id: u32) -> vec3<f32> {
    let cell = ddgi_probe_grid_cell(world_pos);
    let grid_origin = vec3<f32>(ddgi_grid.origin_x, ddgi_grid.origin_y, ddgi_grid.origin_z);
    let grid_spacing = vec3<f32>(ddgi_grid.spacing_x, ddgi_grid.spacing_y, ddgi_grid.spacing_z);
    let frac = vec3<f32>(
        clamp((world_pos.x - grid_origin.x) / max(grid_spacing.x, 1e-4) - f32(cell.x), 0.0, 1.0),
        clamp((world_pos.y - grid_origin.y) / max(grid_spacing.y, 1e-4) - f32(cell.y), 0.0, 1.0),
        clamp((world_pos.z - grid_origin.z) / max(grid_spacing.z, 1e-4) - f32(cell.z), 0.0, 1.0),
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
                let probe_pos = ddgi_probe_position(cx, cy, cz);
                let flat_index = ddgi_probe_flat_index(cx, cy, cz);

                let wx = select(frac.x, 1.0 - frac.x, dx == 0u);
                let wy = select(frac.y, 1.0 - frac.y, dy == 0u);
                let wz = select(frac.z, 1.0 - frac.z, dz == 0u);
                let trilinear_weight = wx * wy * wz;

                let irradiance = ddgi_cosine_weighted_probe_irradiance_roughness_aware(flat_index, surface_normal, tile_size, roughness);
                raw_sum = raw_sum + irradiance;
                raw_count = raw_count + 1.0;

                let to_probe = probe_pos - world_pos;
                let dist = length(to_probe);
                // Two independent visibility signals, combined
                // multiplicatively — ddgi_ref.rs::sample_probe_grid's own
                // doc comment for the full rationale: the hard occlusion
                // ray (from the shaded point's own side) catches
                // "something sits directly between here and the probe,"
                // while the Chebyshev depth test (from the probe's own
                // stored distance moments) catches "this probe's own
                // irradiance is itself unreliable in roughly this
                // direction" — neither alone catches both failure modes.
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

// ---------------------------------------------------------------------------------
// Radiance Cascades (GI_METHOD_RADIANCE_CASCADES) — shading-time hierarchy
// walk + merge. See hybrid_radiance_cascades.wgsl's own header comment
// for why the MERGE step (radiance_cascades_ref.rs::merge_cascade_texel's
// own L_ac = L_ab + beta_ab*L_bc formula) lives HERE rather than in a
// separate compute pass: with only RADIANCE_CASCADES_LEVEL_COUNT=4 small
// levels, one nearest-probe atlas read per level plus 4
// merge_cascade_texel folds is cheaper per shaded pixel than dispatching
// an extra full-screen compute pass.
// ---------------------------------------------------------------------------------

// radiance_cascades_ref.rs::CascadeGrid::probe_position + AtlasLayout's
// own tile_origin, ported together (mirrors
// hybrid_radiance_cascades.wgsl::cascade_atlas_index's own combined
// shape) — reads ONE probe's own octahedral tile at `direction`, given
// that probe's own integer grid coords (NOT a world position; see
// radiance_cascades_sample_level_trilinear below for the world-space
// entry point, Stage 7's spatial-blend fix, that calls this once per
// trilinear-blended neighbor instead of only the single nearest probe).
fn radiance_cascades_probe_irradiance_at(level: CascadeLevelUniform, coords: vec3<u32>, direction: vec3<f32>) -> vec4<f32> {
    let plane = max(level.grid_dims.x * level.grid_dims.y, 1u);
    let probe_flat_index = coords.x + coords.y * max(level.grid_dims.x, 1u) + coords.z * plane;
    if (probe_flat_index >= level.total_probes) {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }

    // Same octahedral texel lookup as ddgi_probe_irradiance
    // (hybrid_ddgi_relight.wgsl) / ddgi_cosine_weighted_probe_irradiance
    // below — direction -> octahedral UV -> texel coords within this
    // probe's own tile.
    var n = direction / (abs(direction.x) + abs(direction.y) + abs(direction.z));
    let octahedral_wrap = vec2<f32>((1.0 - abs(n.y)) * select(-1.0, 1.0, n.x >= 0.0), (1.0 - abs(n.x)) * select(-1.0, 1.0, n.y >= 0.0));
    let n_xy = select(octahedral_wrap, n.xy, n.z >= 0.0);
    let uv = n_xy * 0.5 + vec2<f32>(0.5, 0.5);
    let tile_size = max(level.tile_size, 1u);
    let max_index = f32(tile_size) - 1.0;
    let texel_x = u32(min(uv.x * f32(tile_size), max_index));
    let texel_y = u32(min(uv.y * f32(tile_size), max_index));

    let tiles_per_row = max(level.tiles_per_row, 1u);
    let row = probe_flat_index / tiles_per_row;
    let col = probe_flat_index % tiles_per_row;
    let atlas_width = tiles_per_row * tile_size;
    let local_index = (row * tile_size + texel_y) * atlas_width + (col * tile_size + texel_x);
    let texel_index = level.atlas_texel_offset + local_index;
    // .rgb = radiance, .a = transmittance (beta) — see
    // hybrid_radiance_cascades.wgsl::radiance_cascades_relight_main's own
    // doc comment for why both are packed into one vec4 texel.
    return radiance_cascades_atlas[texel_index];
}

// radiance_cascades_ref.rs::cascade_sample_level_trilinear verbatim:
// trilinearly blends ONE cascade level's own 8 surrounding probes at
// `world_pos` — the Stage 7 spatial-blend fix (see this file's own
// header comment above for why a coarse nearest-probe sample was
// originally an accepted Stage 2 tradeoff, since revisited once the
// real gi_room A/B comparison showed cascades reading dimmer than DDGI
// in the same region even after the Stage 6 hemisphere fix).
fn radiance_cascades_sample_level_trilinear(level: CascadeLevelUniform, world_pos: vec3<f32>, direction: vec3<f32>) -> vec4<f32> {
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
                let sample = radiance_cascades_probe_irradiance_at(level, coords, direction);
                radiance_acc = radiance_acc + sample.rgb * weight;
                transmittance_acc = transmittance_acc + sample.a * weight;
            }
        }
    }
    return vec4<f32>(radiance_acc, transmittance_acc);
}

// radiance_cascades_ref.rs::merge_cascade_texel verbatim: L_ac = L_ab +
// beta_ab * L_bc.
fn merge_cascade_texel(near_radiance: vec3<f32>, near_transmittance: f32, far_radiance: vec3<f32>) -> vec3<f32> {
    return near_radiance + near_transmittance * far_radiance;
}

// Walks all 4 cascade levels FARTHEST-to-NEAREST, folding each level's
// own nearest-probe sample into the accumulator via merge_cascade_texel —
// mirrors radiance_cascades_ref.rs's own test loop (`cascades_reach_
// meaningful_irradiance_on_the_dark_corridor_floor`, `for level in
// (0..level_count).rev()`) exactly: farthest level's contribution starts
// the accumulator, then each nearer level's own beta (transmittance)
// attenuates what's already been accumulated before adding its own
// radiance on top.
fn radiance_cascades_sample_hierarchy(world_pos: vec3<f32>, direction: vec3<f32>) -> vec3<f32> {
    var accumulated = vec3<f32>(0.0);
    for (var i = 0u; i < RADIANCE_CASCADES_LEVEL_COUNT; i = i + 1u) {
        let level_index = RADIANCE_CASCADES_LEVEL_COUNT - 1u - i;
        let level = radiance_cascades_levels[level_index];
        let sample = radiance_cascades_sample_level_trilinear(level, world_pos, direction);
        accumulated = merge_cascade_texel(sample.rgb, sample.a, accumulated);
    }
    return accumulated;
}

// radiance_cascades_sample_hierarchy above point-samples the hierarchy at
// EXACTLY `direction` -- correct for a single view/shading ray, but wrong
// for indirect DIFFUSE response, which is a Lambertian integral of
// incoming radiance over the WHOLE hemisphere around the surface normal,
// not one direction. Passing the bare normal in confined visible bounce
// light to whichever single cascade ray happened to point along that
// exact normal (a "light only travels in one direction" artifact) instead
// of a properly dispersed result. Reuses this file's own already-shared
// ddgi_tangent_basis/HEMISPHERE_SAMPLES bundle (see this bundle's own
// header comment for why it's kept as a generic, non-DDGI-specific
// utility) exactly the way ddgi_sample_probe_grid's own diffuse term
// already does for DDGI.
fn radiance_cascades_cosine_weighted_hierarchy(world_pos: vec3<f32>, surface_normal: vec3<f32>) -> vec3<f32> {
    let basis = ddgi_tangent_basis(surface_normal);
    var acc = vec3<f32>(0.0);
    for (var i = 0u; i < HEMISPHERE_SAMPLE_COUNT; i = i + 1u) {
        let world_dir = normalize(basis * HEMISPHERE_SAMPLES[i]);
        acc = acc + radiance_cascades_sample_hierarchy(world_pos, world_dir);
    }
    return acc / f32(HEMISPHERE_SAMPLE_COUNT);
}

// ---------------------------------------------------------------------------------
// SDF cone tracing (GI_METHOD_CONETRACE) — conetrace_ref.rs verbatim
// port. Stateless: every function below reads only objects/bvh/scene
// (already-bound @group(1) globals every other technique in this file
// also reads) and calls this file's own trace-adjacent primitives
// directly — no new bind group, no new buffer, no separate relight
// pass. See conetrace_ref.rs's own module doc comment for the full
// design (including the structural "no temporal amortization" cost
// tradeoff this technique accepts).
// ---------------------------------------------------------------------------------

// cpu_ref.rs::shade verbatim MINUS the indirect_enabled/indirect_diffuse
// branch — same shape as hybrid_ddgi_relight.wgsl's/hybrid_hashgrid_
// update.wgsl's own shade_direct_only, needed here for the identical
// reason: this file's own shade() ALREADY bakes indirect-diffuse in
// (reading scene.gi_method itself), so calling it from a cone-traced
// hit's own shading would recurse cone_trace_indirect into itself.
// Direct-lit-only radiance is exactly what a single-bounce cone hit
// should shade with (mirrors conetrace_ref.rs::cone_trace_ray's own
// CPU-ref call to cpu_ref::shade, which likewise has no indirect
// dependency of its own).
fn shade_direct_only_for_cone(obj: ObjectGpu, obj_id: u32, p_world: vec3<f32>, world_normal: vec3<f32>, view_dir: vec3<f32>, hit_t: f32) -> vec3<f32> {
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

// conetrace_ref.rs::cone_radius_at verbatim.
fn cone_radius_at(r0: f32, half_angle: f32, t: f32) -> f32 {
    return r0 + t * tan(half_angle);
}

// conetrace_ref.rs::cone_slab_hit verbatim: two-pass per-node margin
// tightening — pass 1 pads by the caller's own worst-case-margin-t bound
// (a definitely-safe, wider margin), pass 2 re-pads using
// cone_radius_at at the tighter t_near pass 1 found. See that function's
// own doc comment (conetrace_ref.rs) for the fixed-point-monotonicity
// proof this never turns a real hit into a miss.
fn cone_slab_hit(ray_origin: vec3<f32>, ray_dir: vec3<f32>, bmin: vec3<f32>, bmax: vec3<f32>, t_query_max: f32, r0: f32, half_angle: f32, worst_case_margin_t: f32) -> vec2<f32> {
    let worst_case_margin = cone_radius_at(r0, half_angle, worst_case_margin_t);
    let hit1 = slab_hit(ray_origin, ray_dir, bmin - vec3<f32>(worst_case_margin), bmax + vec3<f32>(worst_case_margin), t_query_max);
    if (hit1.x > hit1.y) {
        return hit1;
    }
    let tight_margin = cone_radius_at(r0, half_angle, hit1.x);
    return slab_hit(ray_origin, ray_dir, bmin - vec3<f32>(tight_margin), bmax + vec3<f32>(tight_margin), t_query_max);
}

const MAX_CONE_MARCH_STEPS: u32 = 128u;
const MIN_CONE_STEP: f32 = 1e-3;

struct ConeMarchResult {
    did_hit: bool,
    t: f32,
    coverage: f32,
}

// conetrace_ref.rs::march_object_cone verbatim — see that function's
// own doc comment for the step-formula/coverage-smoothstep rationale.
fn march_object_cone(obj: ObjectGpu, ray_origin: vec3<f32>, ray_dir: vec3<f32>, t_start: f32, t_max: f32, r0: f32, half_angle: f32) -> ConeMarchResult {
    let inv_rotation = vec4<f32>(obj.inv_rotation_x, obj.inv_rotation_y, obj.inv_rotation_z, obj.inv_rotation_w);
    let translation = vec3<f32>(obj.translation_x, obj.translation_y, obj.translation_z);
    var t = max(t_start, 0.0);
    for (var i = 0u; i < MAX_CONE_MARCH_STEPS; i = i + 1u) {
        if (t > t_max) {
            return ConeMarchResult(false, 0.0, 0.0);
        }
        let p_world = ray_origin + t * ray_dir;
        let p_local = rotate_by_quat(inv_rotation, p_world - translation);
        let d = local_distance(obj, p_local);
        let radius = cone_radius_at(r0, half_angle, t);
        if (d < radius) {
            // WGSL's smoothstep is spec-indeterminate (can return NaN on
            // real drivers) when edge0 == edge1 — which radius == 0.0
            // hits on every degenerate point-ray bounce (bounce 2+, see
            // cone_trace_ray's own r0=0/half_angle=0 continuation).
            // conetrace_ref.rs's own hand-rolled smoothstep divides
            // through 0.0 too but Rust's f32::clamp resolves the
            // resulting +/-inf to a safe 0.0/1.0, so this divergence
            // never showed up in CPU-ref testing — only on real GPUs.
            // radius == 0.0 with d < radius (i.e. d < 0.0, already
            // inside the surface) is always full coverage.
            var coverage = 1.0;
            if (radius > 0.0) {
                coverage = clamp(smoothstep(radius, 0.0, d), 0.0, 1.0);
            }
            return ConeMarchResult(true, t, coverage);
        }
        t = t + max(d - radius, MIN_CONE_STEP);
    }
    return ConeMarchResult(false, 0.0, 0.0);
}

struct ConeHit {
    did_hit: bool,
    coverage: f32,
    t: f32,
    world_normal: vec3<f32>,
    obj_id: u32,
}

// conetrace_ref.rs::trace_cone verbatim: margin-padded BVH descent (see
// that function's own doc comment for why an unpadded trace()-style
// prune would clip valid cone candidates near a leaf boundary), closest-
// t-wins across every candidate's own march_object_cone convergence.
// origin_obj_id excludes the shaded object's own id from its own query
// — mirrors ddgi_sample_probe_grid's/hashgrid_sample's own
// origin_obj_id exclusion.
fn trace_cone(ray_origin: vec3<f32>, ray_dir: vec3<f32>, t_max: f32, r0: f32, half_angle: f32, origin_obj_id: u32) -> ConeHit {
    var best_hit = false;
    var best_t = t_max;
    var best_coverage = 0.0;
    var best_normal = vec3<f32>(0.0);
    var best_obj_id = 0u;

    if (scene.bvh_node_count == 0u) {
        return ConeHit(false, 0.0, 0.0, vec3<f32>(0.0), 0u);
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
        let hit = cone_slab_hit(ray_origin, ray_dir,
            vec3<f32>(node.min_x, node.min_y, node.min_z),
            vec3<f32>(node.max_x, node.max_y, node.max_z), limit, r0, half_angle, limit);
        let t_near = hit.x;
        let t_far = hit.y;
        if (t_near > t_far) {
            continue;
        }

        if (node.left_or_sentinel == LEAF_SENTINEL) {
            let obj_id = node.right_or_object;
            if (obj_id == origin_obj_id || obj_id >= scene.object_count) {
                continue;
            }
            let object = objects[obj_id];
            let march_limit = select(t_far, min(best_t, t_far), best_hit);
            let march = march_object_cone(object, ray_origin, ray_dir, t_near, march_limit, r0, half_angle);
            if (march.did_hit && (!best_hit || march.t < best_t)) {
                best_hit = true;
                best_t = march.t;
                best_coverage = march.coverage;
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
        let left_hit = cone_slab_hit(ray_origin, ray_dir,
            vec3<f32>(left_node.min_x, left_node.min_y, left_node.min_z),
            vec3<f32>(left_node.max_x, left_node.max_y, left_node.max_z), limit, r0, half_angle, limit);
        let right_hit = cone_slab_hit(ray_origin, ray_dir,
            vec3<f32>(right_node.min_x, right_node.min_y, right_node.min_z),
            vec3<f32>(right_node.max_x, right_node.max_y, right_node.max_z), limit, r0, half_angle, limit);
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

    return ConeHit(best_hit, best_coverage, best_t, best_normal, best_obj_id);
}

const CONE_RAY_BIAS: f32 = 0.01;

// Hard cap on the bounce loop below — WGSL loops need a compile-time-
// known upper bound for the driver's own unrolling/analysis, and this
// renderer's own ConeTraceConfig::max_bounces egui slider is clamped to
// the same range on the Rust side (see extract.rs's own doc comment),
// so this is a real ceiling, not an arbitrary guess.
const MAX_CONE_BOUNCES: u32 = 8u;

// conetrace_ref.rs::cone_trace_ray verbatim: multi-bounce cone march —
// up to max_bounces hits chained, direct-lit shading at each hit,
// contributes nothing (black) on a full miss at any bounce — see this
// function's own doc comment below for why a miss no longer falls back
// to a sky-color gradient. Bounce 1 uses the
// caller's own r0/half_angle (a real cone); every bounce after fires a
// degenerate point ray (r0=0, half_angle=0) continuing straight along
// the PREVIOUS hit's own surface normal — a fixed, deterministic
// direction (this renderer has no per-pixel RNG primitive, and
// introducing one here would add un-amortized noise with nothing to
// temporally denoise it away, since cone tracing is fully stateless).
// Keeps total cost linear in max_bounces, not exponential — see
// conetrace_ref.rs::cone_trace_ray's own doc comment for the full
// design rationale, agreed with the user directly.
//
// max_bounces == 1 is bit-for-bit identical to this function's own
// prior single-bounce behavior — the loop's first iteration always
// runs the same shade-and-accumulate path regardless of max_bounces,
// and only derives a next-bounce ray when max_bounces > 1.
//
// Does NOT blend a hit's own shaded color toward cone_sky_color by
// hit.coverage — a real, found-by-direct-visual-inspection light-leak
// bug in an earlier version did exactly that
// (direct * coverage + cone_sky_color * (1 - coverage)), on the wrong
// assumption that coverage < 1.0 meant "part of the cone's own solid
// angle escaped past this object to open sky." It doesn't: coverage
// (see march_object_cone's own doc comment) is purely a per-step
// convergence-quality signal, unrelated to whether the surrounding
// geometry is actually open to the sky. Inside a fully sealed, opaque
// room, a cone aimed at a nearby wall from a grazing angle can
// legitimately converge with LOW coverage (MIN_CONE_STEP's own coarse
// floor makes even a genuine solid hit's own final step overshoot past
// the true surface) while still having found real, solid geometry —
// the old formula blended in bright sky gradient for that "uncovered"
// fraction even though the cone never actually reached open sky,
// leaking light into gi_room.rs's own fully-sealed, roof-closed room.
// hit.did_hit == true already means the march converged against REAL
// geometry; a genuine miss (march_object_cone exhausting its own step/
// t_max budget with no convergence) is the ONLY case that means
// "nothing here." This coverage^2 scaling applies at EVERY bounce, not
// just the first — a low-coverage hit anywhere in the chain gets the
// identical treatment, or the same sealed-room leak this comment
// describes could reopen at bounce 2+ even though bounce 1 is fixed.
//
// Does NOT fall back to a sky-color gradient on a miss (the
// cone_sky_color function this file used to define is gone) — a real,
// found-by-direct-visual-inspection bug: gi_room.rs's own fully sealed,
// roof-closed room showed the sky's own blue gradient bleeding onto the
// ceiling/walls at max_bounces >= 2, even though conetrace_ref.rs's own
// CPU-reference sweep of the identical geometry never reports a miss
// anywhere in the sealed room — a real GPU-only divergence (RADV/Mesa)
// in trace_cone's own BVH descent, investigated at length but not
// root-caused. A miss here (especially bounce 2+, which continues along
// the previous hit's own surface normal) can be spurious GPU numerical
// divergence with no real correspondence to "this ray genuinely reached
// open sky," so trusting it as real sky is not safe. Black is the
// physically conservative choice.
fn cone_trace_ray(ray_origin: vec3<f32>, ray_dir: vec3<f32>, max_t: f32, r0: f32, half_angle: f32, origin_obj_id: u32, max_bounces: u32) -> vec3<f32> {
    let bounces = clamp(max_bounces, 1u, MAX_CONE_BOUNCES);
    var total = vec3<f32>(0.0);
    var throughput = vec3<f32>(1.0);
    var cur_origin = ray_origin;
    var cur_dir = ray_dir;
    var cur_r0 = r0;
    var cur_half_angle = half_angle;

    for (var bounce = 0u; bounce < bounces; bounce = bounce + 1u) {
        // A miss contributes nothing (black), not cone_sky_color — see
        // conetrace_ref.rs::cone_trace_ray's own doc comment for why
        // trusting a miss as real sky isn't safe here.
        let hit = trace_cone(cur_origin, cur_dir, max_t, cur_r0, cur_half_angle, origin_obj_id);
        if (!hit.did_hit) {
            break;
        }
        let obj = objects[hit.obj_id];
        let p_world = cur_origin + hit.t * cur_dir;
        let view_dir = -cur_dir;
        let result = shade_direct_only_for_cone(obj, hit.obj_id, p_world, hit.world_normal, view_dir, hit.t);
        // A low-coverage hit's own p_world can be a full cone-radius away
        // from the true surface (still floating in open space), so a
        // shadow ray cast from it can see the sun when the true surface
        // point never would — squaring (not just multiplying by) coverage
        // fades that residual toward darkness quickly enough to be
        // imperceptible while staying a smooth, non-hard-branched falloff.
        // Mirrors conetrace_ref.rs::cone_trace_ray exactly.
        let coverage2 = hit.coverage * hit.coverage;
        total = total + throughput * result * coverage2;

        let albedo = vec3<f32>(obj.base_color_r, obj.base_color_g, obj.base_color_b);
        let metallic = clamp(obj.metallic, 0.0, 1.0);
        let diffuse_color = albedo * (1.0 - metallic);

        if (bounce + 1u >= bounces) {
            // Geometric-series tail for un-traced bounces beyond
            // max_bounces — mirrors conetrace_ref.rs::cone_trace_ray's
            // own tail term exactly (see that function's own doc comment
            // for the full derivation/light-leak-safety reasoning):
            // derived entirely from THIS hit's own real `result` and
            // `diffuse_color`, so a hit with no real light (result ==
            // 0) contributes an exactly-zero tail — cannot manufacture
            // light in a scene that has none.
            let safe_rho = min(diffuse_color, vec3<f32>(0.95));
            let tail_ratio = safe_rho / (vec3<f32>(1.0) - safe_rho);
            total = total + throughput * result * coverage2 * tail_ratio;
            break;
        }

        // Continue toward the next bounce: diffuse-albedo-weighted
        // throughput (re-derived here, same formula
        // shade_direct_only_for_cone already computes internally, per
        // this file's own established "duplicate small formulas across
        // functions" convention), fixed normal-direction ray, degenerate
        // point ray for every bounce after the first.
        throughput = throughput * diffuse_color;
        cur_origin = p_world + hit.world_normal * CONE_RAY_BIAS;
        cur_dir = hit.world_normal;
        cur_r0 = 0.0;
        cur_half_angle = 0.0;
    }

    return total;
}

// conetrace_ref.rs::cone_trace_indirect verbatim: fires the SAME
// HEMISPHERE_SAMPLES/ddgi_tangent_basis bundle already declared earlier
// in this file (shared verbatim with DDGI's own read path — no new
// WGSL duplication needed here, unlike the Rust side's own import-vs-
// duplicate tradeoff, since this file already has exactly one copy
// both DDGI and cone tracing can call). max_bounces is threaded through
// to every one of the 5 cone_trace_ray calls — see that function's own
// doc comment for the multi-bounce design.
fn cone_trace_indirect(world_pos: vec3<f32>, surface_normal: vec3<f32>, max_t: f32, r0: f32, half_angle: f32, origin_obj_id: u32, max_bounces: u32) -> vec3<f32> {
    let basis = ddgi_tangent_basis(surface_normal);
    var acc = vec3<f32>(0.0);
    for (var i = 0u; i < HEMISPHERE_SAMPLE_COUNT; i = i + 1u) {
        let world_dir = normalize(basis * HEMISPHERE_SAMPLES[i]);
        acc = acc + cone_trace_ray(world_pos + world_dir * CONE_RAY_BIAS, world_dir, max_t, r0, half_angle, origin_obj_id, max_bounces);
    }
    return acc / f32(HEMISPHERE_SAMPLE_COUNT);
}

// conetrace_ref.rs::cone_trace_indirect_single verbatim: single-cone
// approximation of cone_trace_indirect, firing exactly one cone straight
// along surface_normal instead of the full 5-direction hemisphere.
// Reserved for call sites where the GI term is already second-order
// relative to the pixel's dominant contribution — see that function's
// own doc comment (reflect_trace_ray's own final-gather is the
// motivating case: it was paying for a full 5-cone hemisphere INSIDE a
// reflection bounce that already costs 2 cone marches of its own,
// roughly doubling per-pixel march cost on every reflective surface).
fn cone_trace_indirect_single(world_pos: vec3<f32>, surface_normal: vec3<f32>, max_t: f32, r0: f32, half_angle: f32, origin_obj_id: u32, max_bounces: u32) -> vec3<f32> {
    let world_dir = normalize(surface_normal);
    return cone_trace_ray(world_pos + world_dir * CONE_RAY_BIAS, world_dir, max_t, r0, half_angle, origin_obj_id, max_bounces);
}

// reflect_ref.rs verbatim: multi-bounce specular reflections. Reuses
// trace_cone/march_object_cone directly (see reflect_ref.rs's own module
// doc comment for why: this IS voxel-cone-tracing's own well-known
// roughness-to-aperture idea, applied to the SDF marching primitive this
// renderer already has — only the ray's origin direction (reflect
// vector, not a hemisphere sample) and aperture source (roughness, not a
// fixed config value) differ from cone_trace_indirect's own diffuse-GI
// cones). Fresnel-gated, deterministic (no RNG), terminates each bounce
// via ONE bounce of diffuse cone-traced GI ("final gather," matching
// Lumen's own documented MaxBounces=1 default) rather than nesting a
// further reflection ray.

const MAX_REFLECTION_BOUNCES: u32 = 4u;
const REFLECT_RAY_BIAS: f32 = 0.01;

// reflect_ref.rs::reflection_half_angle verbatim.
fn reflection_half_angle(alpha: f32) -> f32 {
    let epsilon = 0.01;
    return atan(alpha) + epsilon;
}

// reflect_ref.rs::luminance verbatim (Rec. 709).
fn reflect_luminance(c: vec3<f32>) -> f32 {
    return dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
}

// reflect_ref.rs::reflect_ref's own `reflect` helper — WGSL has a native
// `reflect` builtin with the identical `d - 2*(d.n)*n` formula, used
// directly at call sites instead of a wrapper function.

// reflect_ref.rs::shade_for_reflection_bounce verbatim: full direct-lit +
// shadowed + one-bounce-diffuse-GI shading at a reflection bounce's own
// hit point. Deliberately NOT a call to shade() itself — shade()'s own
// GI_METHOD_CONETRACE branch would recurse cone_trace_indirect back into
// this same reflection machinery with no termination; this function
// inlines the identical direct-lit GGX math plus one explicit
// max_bounces=1 cone_trace_indirect call instead, matching
// shade_direct_only_for_cone's own precedent of an intentionally separate
// shading variant.
fn shade_for_reflection_bounce(
    obj: ObjectGpu, obj_id: u32, p_world: vec3<f32>, world_normal: vec3<f32>, view_dir: vec3<f32>, hit_t: f32,
) -> vec3<f32> {
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

    // Single-cone final gather (cone_trace_indirect_single, NOT the full
    // 5-cone hemisphere) — see that function's own doc comment for why:
    // this reflection bounce's own GI term is second-order relative to
    // the reflection itself, so it doesn't need primary-ray GI's full
    // directional fidelity.
    //
    // gi_method != GI_METHOD_NONE: a REAL bug this gate fixes (found
    // 2026-09-18 investigating a sealed, sun-only gi_room still reading
    // faintly lit through/around any reflective surface even under
    // GiMethod::None, after DDGI's own separate sealed-room leak was
    // already fixed). This cone-traced bounce GI term used to run
    // UNCONDITIONALLY regardless of scene.gi_method — see
    // reflect_ref.rs::ReflectionParams::bounce_gi_enabled's own doc
    // comment (this file's CPU reference) for the full rationale.
    if (scene.gi_method != GI_METHOD_NONE) {
        let gi = cone_trace_indirect_single(p_world, n, scene.conetrace_max_t, scene.conetrace_origin_radius, scene.conetrace_half_angle, obj_id, 1u);
        radiance = radiance + diffuse_color * gi;
    }

    let emissive = vec3<f32>(obj.emissive_r, obj.emissive_g, obj.emissive_b);
    return radiance + emissive;
}

// reflect_ref.rs::reflect_trace_ray verbatim: one reflection ray's own
// multi-bounce chain. Every bounce fires a REAL roughness-widened cone
// (unlike cone_trace_ray's own diffuse-GI degenerate-after-bounce-1
// design) since a reflection ray has no hemisphere fan-out to collapse
// in the first place — see reflect_ref.rs's own module doc comment for
// why this makes reflection cost linear in bounce count by construction.
// Result of a full reflection chain — `color` is the usual accumulated
// contribution; `first_hit_*` describe ONLY the very first bounce's own
// hit (not the whole chain) — that's the geometry actually VISIBLE in the
// mirror this frame, and the only point whose motion the specular
// temporal history's own virtual-point reprojection needs (see
// hybrid_temporal.wgsl's own reflect_temporal_main).
struct ReflectTraceResult {
    color: vec3<f32>,
    first_hit_valid: bool,
    first_hit_obj_id: u32,
    first_hit_p_world: vec3<f32>,
}

fn reflect_trace_ray(
    ray_origin: vec3<f32>, ray_dir: vec3<f32>, max_t: f32, origin_obj_id: u32, max_bounces: u32,
) -> ReflectTraceResult {
    let bounces = clamp(max_bounces, 1u, MAX_REFLECTION_BOUNCES);
    var total = vec3<f32>(0.0);
    var throughput = vec3<f32>(1.0);
    var cur_origin = ray_origin;
    var cur_dir = ray_dir;
    var exclude_obj_id = origin_obj_id;
    var first_hit_valid = false;
    var first_hit_obj_id = 0u;
    var first_hit_p_world = vec3<f32>(0.0);

    for (var bounce = 0u; bounce < bounces; bounce = bounce + 1u) {
        // Aperture for THIS bounce depends on the hit material, unknown
        // until the hit resolves — a plain point-ray probe finds the hit
        // first, then a second, wider-aperture march (if needed) uses
        // that hit's own roughness. See reflect_ref.rs's own doc comment.
        let probe_hit = trace_cone(cur_origin, cur_dir, max_t, 0.0, 0.0, exclude_obj_id);
        if (!probe_hit.did_hit) {
            break;
        }
        let obj = objects[probe_hit.obj_id];
        let alpha = max(pow(clamp(obj.roughness, 0.0, 1.0), 2.0), 1e-3);
        let half_angle = reflection_half_angle(alpha);

        var hit = probe_hit;
        if (half_angle > 1e-4) {
            let widened = trace_cone(cur_origin, cur_dir, max_t, 0.0, half_angle, exclude_obj_id);
            if (!widened.did_hit) {
                break;
            }
            hit = widened;
        }

        let p_world = cur_origin + hit.t * cur_dir;
        let view_dir = -cur_dir;
        let result = shade_for_reflection_bounce(objects[hit.obj_id], hit.obj_id, p_world, hit.world_normal, view_dir, hit.t);
        // coverage^2, not linear — identical rationale to
        // cone_trace_ray's own fix (a low-coverage hit's own p_world can
        // float a full cone-radius off the true surface).
        total = total + throughput * result * hit.coverage * hit.coverage;

        if (bounce == 0u) {
            first_hit_valid = true;
            first_hit_obj_id = hit.obj_id;
            first_hit_p_world = p_world;
        }

        if (bounce + 1u >= bounces) {
            break;
        }

        let n = normalize(hit.world_normal);
        let next_dir = reflect(cur_dir, n);
        let n_dot_v = max(dot(n, -cur_dir), 1e-4);
        let hit_obj = objects[hit.obj_id];
        let f0 = mix(
            vec3<f32>(dielectric_f0(hit_obj.reflectance)),
            vec3<f32>(hit_obj.base_color_r, hit_obj.base_color_g, hit_obj.base_color_b),
            clamp(hit_obj.metallic, 0.0, 1.0),
        );
        let f = fresnel_schlick(f0, n_dot_v);
        throughput = throughput * f;
        if (reflect_luminance(throughput) < scene.reflection_fresnel_cutoff) {
            break;
        }
        cur_origin = p_world + n * REFLECT_RAY_BIAS;
        cur_dir = next_dir;
        exclude_obj_id = hit.obj_id;
    }

    return ReflectTraceResult(total, first_hit_valid, first_hit_obj_id, first_hit_p_world);
}

// refract_ref.rs verbatim: multi-bounce transmission/refraction. See that
// module's own doc comment for the full design (solid-dielectric model,
// interior march, Beer-Lambert absorption, TIR fallback, Fresnel-gated
// energy split, single-cone final gather).

const MAX_TRANSMISSION_BOUNCES: u32 = 4u;
const REFRACT_RAY_BIAS: f32 = 0.01;
const MIN_INTERIOR_STEP: f32 = 0.01;

// refract_ref.rs::refract verbatim: Snell's law, vector form. Returns
// did_refract=false on total internal reflection (see that function's own
// doc comment) — WGSL has no Option<T>, so this struct's own bool flag
// plays that role.
struct RefractResult {
    dir: vec3<f32>,
    did_refract: bool,
}

fn refract_ray(i: vec3<f32>, n: vec3<f32>, eta: f32) -> RefractResult {
    let cos_i = -dot(i, n);
    let sin2_t = eta * eta * (1.0 - cos_i * cos_i);
    if (sin2_t > 1.0) {
        return RefractResult(vec3<f32>(0.0), false);
    }
    let cos_t = sqrt(1.0 - sin2_t);
    return RefractResult(eta * i + (eta * cos_i - cos_t) * n, true);
}

// refract_ref.rs::march_object_interior verbatim: marches obj's OWN local
// SDF from INSIDE (ray_origin already known interior) forward until it
// re-emerges. Steps by max(|d|, MIN_INTERIOR_STEP) since sphere-tracing
// the raw (negative) distance would never advance near the entry point.
struct InteriorMarchResult {
    hit: bool,
    t: f32,
    exit_normal_outward: vec3<f32>,
}

fn march_object_interior(obj: ObjectGpu, ray_origin: vec3<f32>, ray_dir: vec3<f32>, t_max: f32) -> InteriorMarchResult {
    let inv_rotation = vec4<f32>(obj.inv_rotation_x, obj.inv_rotation_y, obj.inv_rotation_z, obj.inv_rotation_w);
    let rotation = quat_conjugate(inv_rotation);
    let translation = vec3<f32>(obj.translation_x, obj.translation_y, obj.translation_z);
    var t = 0.0;
    for (var i = 0u; i < MAX_MARCH_STEPS; i = i + 1u) {
        if (t > t_max) {
            return InteriorMarchResult(false, 0.0, vec3<f32>(0.0));
        }
        let p_world = ray_origin + t * ray_dir;
        let p_local = rotate_by_quat(inv_rotation, p_world - translation);
        let d = local_distance(obj, p_local);
        if (d > -HIT_EPSILON) {
            let n_local = local_normal(obj, p_local);
            let n_world = rotate_by_quat(rotation, n_local);
            return InteriorMarchResult(true, t, n_world);
        }
        t = t + max(abs(d), MIN_INTERIOR_STEP);
    }
    return InteriorMarchResult(false, 0.0, vec3<f32>(0.0));
}

// refract_ref.rs::shade_for_refraction_bounce verbatim.
fn shade_for_refraction_bounce(
    obj: ObjectGpu, obj_id: u32, p_world: vec3<f32>, world_normal: vec3<f32>, view_dir: vec3<f32>, hit_t: f32,
) -> vec3<f32> {
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

    // gi_method != GI_METHOD_NONE: see shade_for_reflection_bounce's own
    // identical gate above for the full rationale — this used to run
    // unconditionally regardless of scene.gi_method.
    if (scene.gi_method != GI_METHOD_NONE) {
        let gi = cone_trace_indirect_single(p_world, n, scene.conetrace_max_t, scene.conetrace_origin_radius, scene.conetrace_half_angle, obj_id, 1u);
        radiance = radiance + diffuse_color * gi;
    }

    let emissive = vec3<f32>(obj.emissive_r, obj.emissive_g, obj.emissive_b);
    return radiance + emissive;
}

// Result of a full transmission chain — `color` is the usual accumulated
// contribution; `first_exit_*` describe ONLY the very first bounce's own
// EXIT point (not the whole chain) — the geometry actually visible
// THROUGH the entry surface this frame, and the only point whose motion
// the transmission temporal history's own virtual-point reprojection
// needs (mirrors ReflectTraceResult's own first_hit_* fields exactly).
struct RefractTraceResult {
    color: vec3<f32>,
    first_exit_valid: bool,
    first_exit_obj_id: u32,
    first_exit_p_world: vec3<f32>,
}

// refract_ref.rs::refract_trace_ray verbatim: one transmission ray's own
// multi-bounce chain (entry surface already resolved by the caller).
fn refract_trace_ray(
    entry_p_world: vec3<f32>, entry_normal: vec3<f32>, view_dir: vec3<f32>, entry_obj_id: u32, max_t: f32, max_bounces: u32,
) -> RefractTraceResult {
    let bounces = clamp(max_bounces, 1u, MAX_TRANSMISSION_BOUNCES);
    var total = vec3<f32>(0.0);
    var throughput = vec3<f32>(1.0);
    var cur_obj_id = entry_obj_id;
    var cur_p_world = entry_p_world;
    var cur_normal = normalize(entry_normal);
    var cur_incoming = -normalize(view_dir);
    var first_exit_valid = false;
    var first_exit_obj_id = 0u;
    var first_exit_p_world = vec3<f32>(0.0);

    for (var bounce = 0u; bounce < bounces; bounce = bounce + 1u) {
        let cur_obj = objects[cur_obj_id];
        let ior = max(cur_obj.ior, 1.0001);
        let entering_eta = 1.0 / ior;
        let n_dot_i = dot(cur_normal, cur_incoming);

        var n_for_refract = cur_normal;
        var eta = entering_eta;
        if (n_dot_i >= 0.0) {
            n_for_refract = -cur_normal;
            eta = ior;
        }

        let refracted = refract_ray(cur_incoming, n_for_refract, eta);
        var interior_dir = refracted.dir;
        if (!refracted.did_refract) {
            interior_dir = reflect(cur_incoming, n_for_refract);
        }

        let march_origin = cur_p_world + interior_dir * REFRACT_RAY_BIAS;
        let interior = march_object_interior(cur_obj, march_origin, interior_dir, max_t);
        if (!interior.hit) {
            break;
        }
        let exit_p_world = march_origin + interior_dir * interior.t;

        let absorption = max(vec3<f32>(1.0) - vec3<f32>(cur_obj.base_color_r, cur_obj.base_color_g, cur_obj.base_color_b), vec3<f32>(0.0));
        let transmittance = exp(-absorption * interior.t);
        throughput = throughput * transmittance;

        let exit_n_dot_i = dot(interior.exit_normal_outward, interior_dir);
        var exit_n_for_refract = interior.exit_normal_outward;
        var exit_eta = entering_eta;
        if (exit_n_dot_i >= 0.0) {
            exit_n_for_refract = -interior.exit_normal_outward;
            exit_eta = ior;
        }
        let exit_refracted = refract_ray(interior_dir, exit_n_for_refract, 1.0 / exit_eta);
        var outgoing_dir = exit_refracted.dir;
        if (!exit_refracted.did_refract) {
            outgoing_dir = reflect(interior_dir, exit_n_for_refract);
        }

        let exit_view_dir = -interior_dir;
        let result = shade_for_refraction_bounce(cur_obj, cur_obj_id, exit_p_world, interior.exit_normal_outward, exit_view_dir, interior.t);
        total = total + throughput * result;

        if (bounce == 0u) {
            first_exit_valid = true;
            first_exit_obj_id = cur_obj_id;
            first_exit_p_world = exit_p_world;
        }

        let f0 = dielectric_f0(cur_obj.reflectance);
        let exit_n_dot_v = max(abs(dot(interior.exit_normal_outward, -outgoing_dir)), 1e-4);
        let f = fresnel_schlick(vec3<f32>(f0), exit_n_dot_v);
        throughput = throughput * (vec3<f32>(1.0) - f) * cur_obj.transmission;
        if (reflect_luminance(throughput) < scene.transmission_fresnel_cutoff) {
            break;
        }

        if (bounce + 1u >= bounces) {
            break;
        }

        let next_origin = exit_p_world + outgoing_dir * REFRACT_RAY_BIAS;
        let next_hit = trace_cone(next_origin, outgoing_dir, max_t, 0.0, 0.0, cur_obj_id);
        if (!next_hit.did_hit) {
            break;
        }
        if (objects[next_hit.obj_id].transmission <= 0.0) {
            // refract_ref.rs::refract_trace_ray verbatim: an OPAQUE next
            // object terminates the chain, but it is genuinely visible
            // through the glass just crossed, so it gets one final shade
            // before the chain ends — see that function's own doc
            // comment for the full "aquarium" bug this fixes and why
            // origin_entity (obj_id here) must be the OPAQUE object
            // itself, not the glass just exited.
            let next_p_world = next_origin + outgoing_dir * next_hit.t;
            let opaque_result = shade_for_refraction_bounce(
                objects[next_hit.obj_id], next_hit.obj_id, next_p_world, next_hit.world_normal, -outgoing_dir, next_hit.t,
            );
            total = total + throughput * opaque_result;
            break;
        }
        cur_p_world = next_origin + outgoing_dir * next_hit.t;
        cur_normal = normalize(next_hit.world_normal);
        cur_incoming = outgoing_dir;
        cur_obj_id = next_hit.obj_id;
    }

    return RefractTraceResult(total, first_exit_valid, first_exit_obj_id, first_exit_p_world);
}

// cpu_ref.rs::ShadeResult verbatim — kept split so the (noisy) indirect
// term can be blurred by the denoise pass without also softening
// legitimately-sharp direct-lit detail (specular highlights, shadow
// edges). `indirect` is already diffuse-albedo-multiplied final color,
// not raw irradiance — see cpu_ref.rs::ShadeResult's own doc comment for
// why (this renderer's materials are flat colors with no fine spatial
// detail to protect from the blur, so there is no benefit to keeping
// albedo out of the blurred signal, and doing it this way needs no
// separate albedo texture in the denoise pass at all).
struct ShadeResult {
    direct_and_emissive: vec3<f32>,
    indirect: vec3<f32>,
    // Multi-bounce specular reflection contribution — kept SEPARATE from
    // both direct_and_emissive and indirect for the identical reason
    // indirect is already split out (see this struct's own doc comment
    // above): reflections need their OWN temporal history, since a
    // reflected image's apparent motion does not follow the reflecting
    // surface's own motion (see hybrid_temporal.wgsl's own reflection
    // reprojection section) — bolting this into indirect_view's existing
    // diffuse-GI history would reproject it with the WRONG motion vector.
    reflect: vec3<f32>,
    // The reflection ray's own FIRST-bounce hit (the geometry actually
    // visible in the mirror this frame) — needed by trace_main to compute
    // this pixel's own virtual-point reprojection for the specular
    // temporal history. `reflect_hit_valid == false` when reflection is
    // disabled/gated-off/missed entirely (matches motion_view's own
    // "no object to undo motion for" miss convention).
    reflect_hit_valid: bool,
    reflect_hit_obj_id: u32,
    reflect_hit_p_world: vec3<f32>,
    // Multi-bounce transmission/refraction contribution and its own
    // first-EXIT hit info — mirrors reflect/reflect_hit_* above exactly
    // (see those fields' own doc comments), needed for transmission's own
    // dedicated temporal history's virtual-point reprojection.
    refract: vec3<f32>,
    refract_hit_valid: bool,
    refract_hit_obj_id: u32,
    refract_hit_p_world: vec3<f32>,
}

// cpu_ref.rs::shade verbatim: full metallic-roughness GGX Cook-Torrance
// shading at a hit point, summed over every light, plus the material's own
// emissive term.
fn shade(obj: ObjectGpu, obj_id: u32, p_world: vec3<f32>, world_normal: vec3<f32>, view_dir: vec3<f32>, hit_t: f32) -> ShadeResult {
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
    // Indirect diffuse — exactly one technique's own sample function
    // runs, gated entirely (not just zero-multiplied) on which technique
    // is active, so an inactive technique costs zero extra atlas reads/
    // occlusion rays (mirrors shadows_enabled's own "costs nothing when
    // disabled" pattern, generalized from a bool to GiMethod's small enum
    // now that more than one technique exists — see
    // src/hybrid/extract.rs's GiMethod doc comment for why this is
    // mutually exclusive, not additive). diffuse_color (not raw albedo)
    // multiplication matches the direct-light treatment above, so metals
    // correctly get no indirect response either. Kept out of `radiance`
    // — see ShadeResult's own doc comment above.
    var indirect = vec3<f32>(0.0);
    if (scene.gi_method == GI_METHOD_DDGI) {
        let irradiance = ddgi_sample_probe_grid(p_world, n, clamp(obj.roughness, 0.0, 1.0), scene.ddgi_tile_size, obj_id);
        indirect = diffuse_color * irradiance;
    } else if (scene.gi_method == GI_METHOD_RADIANCE_CASCADES) {
        let irradiance = radiance_cascades_cosine_weighted_hierarchy(p_world, n);
        indirect = diffuse_color * irradiance;
    } else if (scene.gi_method == GI_METHOD_CONETRACE) {
        let irradiance = cone_trace_indirect(p_world, n, scene.conetrace_max_t, scene.conetrace_origin_radius, scene.conetrace_half_angle, obj_id, scene.conetrace_max_bounces);
        indirect = diffuse_color * irradiance;
    }
    let emissive = vec3<f32>(obj.emissive_r, obj.emissive_g, obj.emissive_b);

    // Multi-bounce specular reflection — Fresnel-gated (skip firing the
    // ray entirely below scene.reflection_fresnel_cutoff, following
    // trace_shadow's own VIS_CUTOFF "costs nothing below threshold"
    // convention) so flat, low-reflectance dielectrics pay zero extra
    // cost. See reflect_trace_ray's own doc comment for the full
    // technique. Combined with F (not re-derived) so the split matches
    // the direct-light specular/diffuse energy split immediately above:
    // `f` here is the SAME per-pixel Fresnel term already computed for
    // the primary view direction, reused rather than recomputed.
    var reflect_color = vec3<f32>(0.0);
    var reflect_hit_valid = false;
    var reflect_hit_obj_id = 0u;
    var reflect_hit_p_world = vec3<f32>(0.0);
    if (scene.reflection_enabled != 0u) {
        let primary_f = fresnel_schlick(f0, n_dot_v);
        if (reflect_luminance(primary_f) >= scene.reflection_fresnel_cutoff) {
            let reflect_dir = reflect(-v, n);
            let reflected = reflect_trace_ray(p_world + n * REFLECT_RAY_BIAS, reflect_dir, scene.reflection_max_t, obj_id, scene.reflection_max_bounces);
            reflect_color = primary_f * reflected.color;
            reflect_hit_valid = reflected.first_hit_valid;
            reflect_hit_obj_id = reflected.first_hit_obj_id;
            reflect_hit_p_world = reflected.first_hit_p_world;
        }
    }

    // Multi-bounce transmission/refraction — see refract_trace_ray's own
    // doc comment. Fresnel-gated on the TRANSMITTABLE fraction (1 - F,
    // energy-complementary to reflection's own share), and additionally
    // gated on obj.transmission > 0.0 since a material with
    // transmission=0.0 is opaque regardless of scene.transmission_enabled
    // (matches metallic's own per-material override of an otherwise
    // scene-wide toggle).
    var refract_color = vec3<f32>(0.0);
    var refract_hit_valid = false;
    var refract_hit_obj_id = 0u;
    var refract_hit_p_world = vec3<f32>(0.0);
    if (scene.transmission_enabled != 0u && obj.transmission > 0.0) {
        let entry_f = fresnel_schlick(f0, n_dot_v);
        let transmittable = vec3<f32>(1.0) - entry_f;
        if (reflect_luminance(transmittable) >= scene.transmission_fresnel_cutoff) {
            let transmitted = refract_trace_ray(p_world, n, v, obj_id, scene.transmission_max_t, scene.transmission_max_bounces);
            refract_color = transmittable * obj.transmission * transmitted.color;
            refract_hit_valid = transmitted.first_exit_valid;
            refract_hit_obj_id = transmitted.first_exit_obj_id;
            refract_hit_p_world = transmitted.first_exit_p_world;
        }
    }

    var result: ShadeResult;
    result.direct_and_emissive = radiance + emissive;
    result.indirect = indirect;
    result.reflect = reflect_color;
    result.reflect_hit_valid = reflect_hit_valid;
    result.reflect_hit_obj_id = reflect_hit_obj_id;
    result.reflect_hit_p_world = reflect_hit_p_world;
    result.refract = refract_color;
    result.refract_hit_valid = refract_hit_valid;
    result.refract_hit_obj_id = refract_hit_obj_id;
    result.refract_hit_p_world = refract_hit_p_world;
    return result;
}

// --- temporal_ref.rs::reproject_world_point verbatim (the object-motion
// half only — see motion_view's own binding doc comment for why the
// camera-projection half lives in hybrid_temporal.wgsl instead). ---------

fn quat_inverse(q: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(-q.x, -q.y, -q.z, q.w);
}

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

// cpu_ref.rs::reproject_world_point verbatim: undoes this frame's rigid
// transform (bringing p_world_current into the object's own local
// space), then reapplies LAST frame's rigid transform to recover where
// that same local-space point was one frame ago. A miss (no object hit)
// is treated as world-space-static — camera motion only — matching
// temporal_ref.rs::reproject_world_point's own `None` case.
fn reproject_world_point(p_world_current: vec3<f32>, obj: ObjectGpu) -> vec3<f32> {
    let translation_current = vec3<f32>(obj.translation_x, obj.translation_y, obj.translation_z);
    let inv_rotation_current = vec4<f32>(obj.inv_rotation_x, obj.inv_rotation_y, obj.inv_rotation_z, obj.inv_rotation_w);
    let local = quat_rotate(inv_rotation_current, p_world_current - translation_current);

    let translation_previous = vec3<f32>(obj.prev_translation_x, obj.prev_translation_y, obj.prev_translation_z);
    let inv_rotation_previous =
        vec4<f32>(obj.prev_inv_rotation_x, obj.prev_inv_rotation_y, obj.prev_inv_rotation_z, obj.prev_inv_rotation_w);
    let rotation_previous = quat_inverse(inv_rotation_previous);
    return quat_rotate(rotation_previous, local) + translation_previous;
}

// --- Ray generation (near-plane unprojection, mirrors hybrid_legacy_blit's
// fragment ray-gen — see this file's top doc comment for why) ------------

// DEPTH-OF-FIELD jitter is still NOT applied here — see dof_ref.rs's own
// module doc comment for why: this ray's own hit point/depth feeds
// out_depth (Bevy's real depth test), the GI temporal accumulator's
// disocclusion test, and the reflection/refraction virtual-point
// reprojection, none of which are designed to distinguish "lens aperture
// jitter" from a real disocclusion. DOF is resolved entirely in a
// SEPARATE pass (hybrid_dof.wgsl) that runs after this frame's sharp
// image is fully shaded/composited, firing its own dedicated jittered
// ray and resampling the sharp image rather than perturbing this one.
//
// A SEPARATE, much smaller sub-pixel jitter (scene.jitter_enabled) CAN be
// applied here, experimentally (see src/hybrid/taa_ref.rs's own module
// doc comment for the full rationale — this is step 1a of a temporal-
// upscaling investigation, gated off by default). Unlike DOF's jitter
// (which can be many pixels wide, aperture-radius-scaled), TAAU jitter is
// sub-single-pixel by construction (see taa_ref::taa_jitter_offset's own
// [-0.5, 0.5] texel range) — small enough that this function's own doc
// comment above about depth/disocclusion tests not distinguishing jitter
// from real motion is an empirical risk to MEASURE via live A/B, not an
// a-priori argument against ever trying it at this much smaller scale.
// UV computed directly against scene.trace_size (this pass's own working
// resolution), NOT view.frag_coord_to_uv(pixel, view.viewport) — at
// RenderScaleConfig::scale < 1.0 the trace pass's own gid.xy range is
// SMALLER than view.viewport, and frag_coord_to_uv's own viewport.xy
// offset subtraction is meaningless here anyway (this pass's own pixel
// grid always starts at (0, 0), unlike a real fragment's frag_coord,
// which can be offset by a sub-viewport render target). See SceneUniform::
// trace_size_x's own doc comment for the full rationale.
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

@compute @workgroup_size(8, 8, 1)
fn trace_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    // Bounds-checked against scene.trace_size, NOT view.viewport — this
    // pass's own dispatch (pass.rs) is sized to trace_size, which can be
    // smaller than the real viewport at RenderScaleConfig::scale < 1.0.
    if (gid.x >= scene.trace_size_x || gid.y >= scene.trace_size_y) {
        return;
    }

    let ro = view.world_position;
    let rd = generate_primary_ray(gid.xy);

    let hit = trace(ro, rd, SKY_T);

    var color: vec3<f32>;
    var indirect: vec3<f32>;
    var normal: vec3<f32>;
    var depth_t: f32;
    var p_world_previous: vec3<f32>;
    // The PRIMARY (reflecting) surface's own roughness, stashed in
    // normal_view's otherwise-unused .w channel — read by
    // hybrid_temporal.wgsl's own reflect_temporal_main to gate virtual-
    // point reprojection: a low-roughness (near-mirror) surface's
    // reflection hit point is a well-defined single point worth
    // reprojecting; a high-roughness surface's reflection cone is wide
    // enough that any single hit point is a poor stand-in for the whole
    // blurred lobe, so temporal accumulation is skipped there instead
    // (relying on the cone's own spatial self-blur) — see that function's
    // own doc comment for the full reasoning.
    var reflecting_roughness: f32;
    var reflect_color: vec3<f32>;
    // Virtual-point reprojection target for the specular history: the
    // REFLECTED hit's own previous-frame world position, not the
    // reflecting surface's — see hybrid_temporal.wgsl's own
    // reflect_temporal_main for why this differs from p_world_previous.
    var reflect_p_world_previous: vec3<f32>;
    // Same idea as reflecting_roughness/reflect_color/reflect_p_world_
    // previous above, for transmission — see refract_view's own binding
    // doc comment.
    var refracting_roughness: f32;
    var refract_color: vec3<f32>;
    var refract_p_world_previous: vec3<f32>;
    if (hit.did_hit) {
        let p_world = ro + hit.t * rd;
        let view_dir = -rd;
        let result = shade(objects[hit.obj_id], hit.obj_id, p_world, hit.world_normal, view_dir, hit.t);
        color = result.direct_and_emissive;
        indirect = result.indirect;
        normal = normalize(hit.world_normal);
        depth_t = hit.t;
        p_world_previous = reproject_world_point(p_world, objects[hit.obj_id]);
        reflecting_roughness = clamp(objects[hit.obj_id].roughness, 0.0, 1.0);
        reflect_color = result.reflect;
        if (result.reflect_hit_valid) {
            reflect_p_world_previous = reproject_world_point(result.reflect_hit_p_world, objects[result.reflect_hit_obj_id]);
        } else {
            // No reflection this pixel (disabled/Fresnel-gated/missed) —
            // world-space-static, matching the primary miss convention
            // below; reflect_view's own color is already zero so nothing
            // downstream blends a meaningful value at this UV anyway.
            reflect_p_world_previous = p_world;
        }
        refracting_roughness = clamp(objects[hit.obj_id].roughness, 0.0, 1.0);
        refract_color = result.refract;
        if (result.refract_hit_valid) {
            refract_p_world_previous = reproject_world_point(result.refract_hit_p_world, objects[result.refract_hit_obj_id]);
        } else {
            refract_p_world_previous = p_world;
        }
    } else {
        color = vec3<f32>(scene.background_r, scene.background_g, scene.background_b);
        indirect = vec3<f32>(0.0);
        // A degenerate (zero) normal for a miss is fine: the denoise
        // pass's edge weight only ever compares a pixel's normal against
        // its own neighbors, and a sky pixel's indirect is already zero,
        // so nothing downstream reads this value as if it meant anything.
        normal = vec3<f32>(0.0);
        depth_t = SKY_T;
        // No object to undo motion for — world-space-static (camera
        // motion only), matching reproject_world_point's own `None` case
        // (cpu_ref.rs mirror) for a miss.
        p_world_previous = ro + rd * SKY_T;
        // 1.0 (max roughness) for a sky pixel: no reflection here at all,
        // and this value only gates whether reflect_temporal_main
        // reprojects — a sky pixel's own reflect_view is already zero, so
        // this just ensures it takes the "no accumulation, self-blur"
        // path rather than the mirror-precision path for no reason.
        reflecting_roughness = 1.0;
        reflect_color = vec3<f32>(0.0);
        reflect_p_world_previous = ro + rd * SKY_T;
        refracting_roughness = 1.0;
        refract_color = vec3<f32>(0.0);
        refract_p_world_previous = ro + rd * SKY_T;
    }

    textureStore(out_color, vec2<i32>(gid.xy), vec4<f32>(color, 1.0));
    textureStore(out_depth, vec2<i32>(gid.xy), vec4<f32>(depth_t, 0.0, 0.0, 0.0));
    textureStore(normal_view, vec2<i32>(gid.xy), vec4<f32>(normal, reflecting_roughness));
    textureStore(indirect_view, vec2<i32>(gid.xy), vec4<f32>(indirect, 1.0));
    textureStore(motion_view, vec2<i32>(gid.xy), vec4<f32>(p_world_previous, refracting_roughness));
    textureStore(reflect_view, vec2<i32>(gid.xy), vec4<f32>(reflect_color, 1.0));
    textureStore(reflect_motion_view, vec2<i32>(gid.xy), vec4<f32>(reflect_p_world_previous, 1.0));
    textureStore(refract_view, vec2<i32>(gid.xy), vec4<f32>(refract_color, 1.0));
    textureStore(refract_motion_view, vec2<i32>(gid.xy), vec4<f32>(refract_p_world_previous, 1.0));
}
