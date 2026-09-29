//! Parses the hybrid WGSL files with naga directly so parse errors surface
//! with line numbers (bevy's asset loader swallows them into ShaderNotLoaded).
//! `#import` lines are naga-oil directives — stripped before the raw parse.

use naga::front::wgsl::parse_str;

fn parse_file(path: &str) {
    let src = std::fs::read_to_string(path).unwrap();
    let stripped: String = src
        .lines()
        .filter(|l| !l.trim_start().starts_with("#import"))
        .collect::<Vec<_>>()
        .join("\n");
    // naga-oil resolves `ImportPath::function` calls and types at load time; strip the
    // qualified path so a bare name remains for naga's direct parse.
    let stripped = stripped.replace("checkerboard::evaluate_pattern", "evaluate_pattern");
    // Stub the imported types/helpers (naga-oil resolves them at load time).
    let stubbed = format!(
        "struct View {{\n    world_from_clip: mat4x4<f32>,\n    world_from_view: mat4x4<f32>,\n    view_from_world: mat4x4<f32>,\n    clip_from_view: mat4x4<f32>,\n    clip_from_world: mat4x4<f32>,\n    world_position: vec3<f32>,\n    viewport: vec4<f32>,\n}}\nstruct Material {{\n    base_color: vec3<f32>,\n    metallic: f32,\n    roughness: f32,\n    emissive: vec3<f32>,\n}}\nfn material_new(base_color: vec3<f32>, metallic: f32, roughness: f32) -> Material {{ return Material(base_color, metallic, roughness, vec3(0.0)); }}\nfn evaluate_pattern(p: vec3<f32>, mat_a: Material, mat_b: Material, params: vec4<f32>) -> Material {{ return mat_a; }}\nfn frag_coord_to_uv(fc: vec2<f32>, viewport: vec4<f32>) -> vec2<f32> {{ return (fc - viewport.xy) / viewport.zw; }}\nfn uv_to_ndc(uv: vec2<f32>) -> vec2<f32> {{ return uv * vec2(2.0, -2.0) + vec2(-1.0, 1.0); }}\n{}",
        stripped
    );
    match parse_str(&stubbed) {
        Ok(module) => {
            let _info = naga::valid::Validator::new(
                naga::valid::ValidationFlags::all(),
                naga::valid::Capabilities::all(),
            )
            .validate(&module)
            .expect("naga validation failed");
        }
        Err(e) => panic!("{path} parse error: {e}"),
    }
}

#[test]
fn hybrid_trace_wgsl_parses() {
    parse_file("assets/shaders/hybrid_trace.wgsl");
}

#[test]
fn hybrid_blit_wgsl_parses() {
    parse_file("assets/shaders/hybrid_blit.wgsl");
}

#[test]
fn splat_wgsl_parses() {
    parse_file("assets/shaders/splat.wgsl");
}

#[test]
fn hybrid_temporal_wgsl_parses() {
    parse_file("assets/shaders/hybrid_temporal.wgsl");
}

#[test]
fn hybrid_denoise_wgsl_parses() {
    parse_file("assets/shaders/hybrid_denoise.wgsl");
}

#[test]
fn hybrid_post_wgsl_parses() {
    parse_file("assets/shaders/hybrid_post.wgsl");
}

#[test]
fn hybrid_ddgi_relight_wgsl_parses() {
    parse_file("assets/shaders/hybrid_ddgi_relight.wgsl");
}

#[test]
fn hybrid_dof_wgsl_parses() {
    parse_file("assets/shaders/hybrid_dof.wgsl");
}
