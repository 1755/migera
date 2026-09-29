// Built-in checkerboard pattern — written as an ordinary pattern shader with no
// privileged access to the raymarcher: this is the reference implementation of the
// pattern-authoring contract every user-authored pattern shader follows (see this
// repo's docs/knowledge/sdf-3d/materials-and-texturing/material-blending.md and
// uv-less-texturing.md for the technique this implements — a procedural pattern
// evaluated directly at the world-space hit point, no UVs/textures needed).
//
// Contract: a pattern module must
//   1. declare its own `#define_import_path` (this repo's convention: `migera::
//      pattern::<name>`, but any distinct path works — src/raymarch/pipeline.rs's
//      generated dispatcher `#import`s whatever import_path the authoring entity's
//      ProceduralPattern component names, verbatim).
//   2. `#import migera::material::{Material, material_new}`.
//   3. export `fn evaluate_pattern(p: vec3<f32>, mat_a: Material, mat_b: Material,
//      params: vec4<f32>) -> Material` — `mat_a`/`mat_b` are the two Material payloads
//      authored on the ProceduralPattern's owning entity (see sdf::components::
//      ProceduralPattern's doc comment), `params` is that same component's generic
//      vec4 tunable (this pattern uses params.x as cell size, others are free to use
//      all four fields however they like), and `p` is the world-space hit point —
//      exactly what the raymarcher already has for free from sphere tracing (no
//      texture fetch, no triplanar projection needed for a single flat pattern axis).

#define_import_path migera::pattern::checkerboard

#import migera::material::Material

fn floored_mod(x: f32, m: f32) -> f32 {
    return x - m * floor(x / m);
}

fn evaluate_pattern(p: vec3<f32>, mat_a: Material, mat_b: Material, params: vec4<f32>) -> Material {
    let cell_size = max(params.x, 1e-4);
    let cx = floor(p.x / cell_size);
    let cz = floor(p.z / cell_size);
    let parity = floored_mod(cx + cz, 2.0);
    if (parity < 0.5) {
        return mat_a;
    }
    return mat_b;
}
