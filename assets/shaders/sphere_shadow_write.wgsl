// Write-and-throw spike, follow-up: makes the compute-rendered analytic
// sphere (see sphere_prepass_trace.wgsl/sphere_prepass_blit.wgsl) cast a
// real shadow, by writing directly into a light's shadow-map depth
// attachment — bypassing bevy_pbr's normal `queue_shadows`/`Shadow` phase
// entirely, since that phase is populated purely from `Mesh3d` entities with
// no extension point for a non-mesh caster (confirmed via source read of
// bevy_pbr-0.19.1's render/light.rs).
//
// Unlike sphere_prepass_trace.wgsl, this is a single fragment-only pass (no
// compute dispatch, no scratch textures): a shadow map only needs depth, and
// re-doing the ray-sphere intersection directly per shadow-map texel is
// cheap enough not to need a separate compute step. There is also no Bevy
// `View` uniform bind group available here — a shadow-map light-view isn't
// a `Camera3d` view — so the view/projection matrices are passed as plain
// uniform fields, populated per-invocation from that shadow view's own
// `ExtractedView` (see src/prepass_probe/pass.rs).
//
// Depth convention matches the shadow map exactly (same as the main
// ViewDepthTexture this project's other blit shader already targets):
// Depth32Float, reverse-Z, GreaterEqual compare, clear value 0.0 = far.
//
// On miss, the fragment is discarded (not written as "far") so this pass's
// LoadOp::Load composites correctly atop whatever a real mesh shadow caster
// already wrote into the same shadow map for this light-view.

struct SphereUniform {
    center: vec3<f32>,
    radius: f32,
}

struct ShadowViewUniform {
    // This light-view's own matrices — NOT the main camera's. `world_from_clip`
    // is precomputed CPU-side (`clip_from_view * view_from_world`, inverted
    // once in Rust via glam rather than inverting a matrix in WGSL) for
    // near-plane ray reconstruction; `clip_from_world` is the forward
    // matrix, needed to reproject a world-space hit back into this shadow
    // view's own depth (see pass.rs for how both are derived from that
    // view's `ExtractedView`).
    world_from_clip: mat4x4<f32>,
    clip_from_world: mat4x4<f32>,
    world_position: vec3<f32>,
    res_x: f32,
    res_y: f32,
}

@group(0) @binding(0) var<uniform> sphere: SphereUniform;
@group(0) @binding(1) var<uniform> shadow_view: ShadowViewUniform;

const SKY_T: f32 = 400.0;

// Identical formula to sphere_prepass_trace.wgsl's intersect_sphere — kept
// as a separate copy rather than a shared #import for this spike (only one
// tiny function, not worth the module-splitting ceremony here).
fn intersect_sphere(ro: vec3<f32>, rd: vec3<f32>) -> f32 {
    let oc = ro - sphere.center;
    let b = dot(oc, rd);
    let c = dot(oc, oc) - sphere.radius * sphere.radius;
    let disc = b * b - c;
    if (disc < 0.0) {
        return SKY_T;
    }
    let sq = sqrt(disc);
    let t0 = -b - sq;
    let t1 = -b + sq;
    if (t0 > 0.001) {
        return t0;
    }
    if (t1 > 0.001) {
        return t1;
    }
    return SKY_T;
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
}

// Fullscreen triangle, matching Bevy's own FullscreenShader convention
// (3 vertices, no vertex buffer) — see bevy_core_pipeline's
// fullscreen_vertex_shader.wgsl for the canonical version this mirrors.
@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    let uv = vec2<f32>(f32((vertex_index << 1u) & 2u), f32(vertex_index & 2u));
    let clip_position = vec4<f32>(uv * vec2<f32>(2.0, -2.0) + vec2<f32>(-1.0, 1.0), 0.0, 1.0);
    return VertexOutput(clip_position, uv);
}

@fragment
fn fragment(in: VertexOutput) -> @builtin(frag_depth) f32 {
    let ndc = in.uv * vec2<f32>(2.0, -2.0) + vec2<f32>(-1.0, 1.0);
    // Reconstruct this shadow-view's own ray the same way the main trace
    // shader reconstructs the camera's — via the near plane, since reverse-Z
    // infinite-far projections can't unproject the far plane.
    let near_clip = vec4<f32>(ndc, 1.0, 1.0);
    let near_world = shadow_view.world_from_clip * near_clip;
    let near_pos = near_world.xyz / near_world.w;
    let ro = shadow_view.world_position;
    let rd = normalize(near_pos - ro);

    let t = intersect_sphere(ro, rd);
    if (t >= SKY_T) {
        discard;
    }

    let world_pos = ro + rd * t;
    let clip = shadow_view.clip_from_world * vec4<f32>(world_pos, 1.0);
    return clamp(clip.z / clip.w, 0.0, 1.0);
}
