//! Compose hybrid_trace.wgsl with naga_oil exactly the way bevy's
//! ShaderCache does, so shader *import resolution* (module paths, symbol
//! imports, namespaced calls) is exercised against the real asset files —
//! not the hand-stubbed single-file merge in wgsl_parse.rs.
//!
//! This guards the pattern-dispatch wiring: `#import migera::material::Material`
//! (symbol import) plus `#import migera::pattern::checkerboard` (whole-module
//! import used via `checkerboard::evaluate_pattern`), both of which must
//! resolve for the trace pipeline to compile under a real GPU.

use naga_oil::compose::{
    ComposableModuleDescriptor, Composer, NagaModuleDescriptor, ShaderLanguage,
};

const VIEW_STUB: &str = r#"#define_import_path bevy_render::view

struct ColorGrading {
    global_exposure: f32,
    white_point: vec3<f32>,
    temperature: f32,
    tint: f32,
    lower_exposure: vec3<f32>,
    upper_exposure: vec3<f32>,
    hsv_hue: f32,
    hsv_saturation: f32,
    hsv_value: f32,
    rgb_r: vec3<f32>,
    rgb_g: vec3<f32>,
    rgb_b: vec3<f32>,
    rgb_c: vec3<f32>,
    rgb_m: vec3<f32>,
    rgb_y: vec3<f32>,
    rgb_k: vec3<f32>,
    tonemap_luts: vec4<f32>,
}

struct View {
    clip_from_world: mat4x4<f32>,
    unjittered_clip_from_world: mat4x4<f32>,
    world_from_clip: mat4x4<f32>,
    world_from_view: mat4x4<f32>,
    view_from_world: mat4x4<f32>,
    clip_from_view: mat4x4<f32>,
    view_from_clip: mat4x4<f32>,
    world_position: vec3<f32>,
    exposure: f32,
    viewport: vec4<f32>,
    main_pass_viewport: vec4<f32>,
    frustum: array<vec4<f32>, 6>,
    lod_view_world_position: vec3<f32>,
    color_grading: ColorGrading,
    mip_bias: f32,
    frame_count: u32,
    near: f32,
    far: f32,
    delta_time: f32,
    main_sampler_available: u32,
    texture_compression_capabilities: u32,
    constraints: u32,
    dt: f32,
};
"#;

fn shader_dir() -> std::path::PathBuf {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    root.join("assets").join("shaders")
}

fn read(dir: &std::path::Path, rel: &str) -> String {
    let path = dir.join(rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn add_module(composer: &mut Composer, source: &str, file_path: &str) {
    composer
        .add_composable_module(ComposableModuleDescriptor {
            source,
            file_path,
            language: ShaderLanguage::Wgsl,
            as_name: None,
            additional_imports: &[],
            shader_defs: Default::default(),
        })
        .unwrap_or_else(|e| panic!("add module {file_path}: {e:?}"));
}

#[test]
fn hybrid_trace_resolves_imports_like_bevy() {
    let dir = shader_dir();

    // bevy inlines its own view shader for `bevy_render::view::View`; stub it
    // with the union of fields hybrid_trace touches to keep resolution real.
    let mut composer = Composer::default();
    add_module(&mut composer, VIEW_STUB, "<bevy_render::view::View>");
    add_module(&mut composer, &read(&dir, "material.wgsl"), "material.wgsl");
    add_module(
        &mut composer,
        &read(&dir, "patterns/checkerboard.wgsl"),
        "patterns/checkerboard.wgsl",
    );

    let hybrid = read(&dir, "hybrid_trace.wgsl");
    let module = composer
        .make_naga_module(NagaModuleDescriptor {
            source: &hybrid,
            file_path: "hybrid_trace.wgsl",
            shader_type: naga_oil::compose::ShaderType::Wgsl,
            shader_defs: Default::default(),
            additional_imports: &[],
        })
        .unwrap_or_else(|e| panic!("compose hybrid_trace.wgsl: {e:?}"));

    let mut validator = naga::valid::Validator::new(
        naga::valid::ValidationFlags::all(),
        naga::valid::Capabilities::all(),
    );
    validator
        .validate(&module)
        .unwrap_or_else(|e| panic!("validate hybrid_trace.wgsl: {e:?}"));
}
