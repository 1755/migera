// Shared physically based material data (docs/knowledge/sdf-3d/materials-and-texturing/
// pbr-shading-model.md's metallic-roughness parameterization) — imported by
// raymarch.wgsl (which shades with it) and by every pattern shader (which produces it,
// see patterns/checkerboard.wgsl for the reference implementation and this repo's
// pattern-authoring contract: `fn evaluate_pattern(p: vec3<f32>, params: vec4<f32>) ->
// Material`). Pulled into its own importable module — rather than left inline in
// raymarch.wgsl — specifically so a user-authored pattern shader can `#import
// migera::material::Material` without depending on (or being recompiled alongside) the
// whole raymarcher.

#define_import_path migera::material

struct Material {
    base_color: vec3<f32>,
    metallic: f32,
    roughness: f32,
    emissive: vec3<f32>,
};

fn material_new(base_color: vec3<f32>, metallic: f32, roughness: f32) -> Material {
    return Material(base_color, metallic, roughness, vec3<f32>(0.0));
}

// Dielectric F0 (normal-incidence reflectance) shared by every non-metal material —
// 0.04 is the standard real-time PBR constant for common dielectrics (glass, plastic,
// stone). Metals use their own base_color as F0 instead (metallic reflectance is
// tinted; dielectric reflectance isn't) — see `material_f0`.
const DIELECTRIC_F0: vec3<f32> = vec3<f32>(0.04);

fn material_f0(mat: Material) -> vec3<f32> {
    return mix(DIELECTRIC_F0, mat.base_color, mat.metallic);
}

// Schlick's Fresnel approximation: F = F0 + (1-F0)(1-cos_theta)^5 — reflectance rises
// toward 1 at grazing angles regardless of material, tinted by F0 at normal incidence.
fn fresnel_schlick(cos_theta: f32, f0: vec3<f32>) -> vec3<f32> {
    let c = clamp(1.0 - cos_theta, 0.0, 1.0);
    return f0 + (vec3<f32>(1.0) - f0) * (c * c * c * c * c);
}
