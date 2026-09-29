// Stage-2 shadow-map writer: makes the full BVH scene CAST shadows via a
// direct write into a light's own `ShadowView` depth attachment, bypassing
// bevy_pbr's normal `queue_shadows`/`Shadow`-phase path (Mesh3d-only, no
// extension point for a non-mesh caster — confirmed by source read, ported
// from `crate::prepass_probe`'s one-sphere proof). Fragment-only: shadow maps
// are created without STORAGE_BINDING, so a compute dispatch can't write them
// (structurally forced, not a style choice).
//
// Reuses `migera::hybrid_bvh`'s BVH traversal + SDF marching verbatim (same
// module hybrid_trace.wgsl imports) — this shader's own group-1 bindings
// 0..3 (scene/objects/primitives/bvh) come from that import; see its header
// comment for the shared-binding-index convention this depends on. Adds one
// more bind group (group 0) for the shadow view's own matrices, since a
// shadow-map render pass has no Bevy `View` uniform of its own to reuse (it's
// not a normal camera view).
//
// Ordering (registered in src/hybrid/mod.rs): after bevy_pbr's own
// per_view_shadow_pass::<LATE_SHADOW_PASS>/shared_shadow_pass::<LATE_SHADOW_PASS>,
// before Core3dSystems::MainPass — see crate::prepass_probe::pass's
// `write_sphere_shadow` doc comment for why this ordering matters
// (DepthAttachment's LoadOp::Load vs LoadOp::Clear depends on draw order).

#import migera::hybrid_bvh::{trace_primary, SKY_T}

struct ShadowViewUniform {
    world_from_clip: mat4x4<f32>,
    clip_from_world: mat4x4<f32>,
    world_position: vec3<f32>,
    res_x: f32,
    res_y: f32,
}

@group(0) @binding(0) var<uniform> shadow_view: ShadowViewUniform;

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
}

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    let uv = vec2<f32>(f32((vertex_index << 1u) & 2u), f32(vertex_index & 2u));
    let clip_position = vec4<f32>(uv * vec2<f32>(2.0, -2.0) + vec2<f32>(-1.0, 1.0), 0.0, 1.0);
    return VertexOutput(clip_position, uv);
}

@fragment
fn fragment(in: VertexOutput) -> @builtin(frag_depth) f32 {
    let ndc = in.uv * vec2<f32>(2.0, -2.0) + vec2<f32>(-1.0, 1.0);
    let near_clip = vec4<f32>(ndc, 1.0, 1.0);
    let near_world = shadow_view.world_from_clip * near_clip;
    let near_pos = near_world.xyz / near_world.w;
    let ro = shadow_view.world_position;
    let rd = normalize(near_pos - ro);

    let hit = trace_primary(ro, rd);
    if (hit.t >= SKY_T) { discard; }

    let world_pos = ro + rd * hit.t;
    let clip = shadow_view.clip_from_world * vec4<f32>(world_pos, 1.0);
    return clamp(clip.z / clip.w, 0.0, 1.0);
}
