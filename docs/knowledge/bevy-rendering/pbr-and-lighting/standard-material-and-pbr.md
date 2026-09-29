---
title: StandardMaterial and the Material trait
description: Maps bevy_pbr 0.19.1's modules, the Material trait (in bevy_pbr, distinct from bevy_material's plumbing), MaterialExtension, and StandardMaterial's texture slots, UV channels, bindless arrays and shader defs, plus why some features force forward rendering. Read before writing or extending a PBR material.
type: reference
status: current
tags:
  - bevy
  - materials
  - lighting
  - render-pipeline
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_pbr-0.19.1/src/pbr_material.rs, material.rs, extended_material.rs
aliases:
  - bevy_pbr
  - Material trait
  - MaterialExtension
  - ExtendedMaterial
  - bindless materials
---

# StandardMaterial and the Material trait

## `bevy_pbr` top-level structure

`bevy_pbr/src/` is large and organized by rendering concern:

- **Material system**: `pbr_material.rs` (`StandardMaterial`), `material.rs` (the
  `Material` trait + `MaterialPlugin`), `extended_material.rs` (`MaterialExtension` for
  layering custom shading on top of `StandardMaterial`), `material_bind_groups.rs`
  (bindless/bind-group-array management), `parallax.rs`, `mesh_material.rs`.
- **`render/`**: the core forward/deferred mesh pipeline — `mesh.rs` (pipeline
  specialization), `light.rs` (light extraction + shadow-map view construction, see
  [lighting-and-shadows](./lighting-and-shadows.md)), `mesh_view_bindings.rs`,
  `gpu_preprocess.rs`, plus WGSL: `pbr.wgsl`, `pbr_functions.wgsl`, `pbr_lighting.wgsl`
  (BRDF), `shadows.wgsl`/`shadow_sampling.wgsl`, `clustered_forward.wgsl`.
- **`cluster/`**: CPU + GPU clustering (see
  [clustered-forward-rendering](./clustered-forward-rendering.md)).
- **`atmosphere/`**: the Bruneton-model sky renderer consuming `bevy_light::Atmosphere`/
  `ScatteringMedium` (see
  [light-components-and-atmosphere](./light-components-and-atmosphere.md)).
- **`ssao/`**: screen-space ambient occlusion (horizon-based, with depth preprocessing +
  spatial denoising).
- **`ssr/`**: screen-space reflections (see
  [screen-space-reflections](./screen-space-reflections.md)).
- **`volumetric_fog/`**: froxel-based volumetric fog/lighting.
- **`light_probe/`**: irradiance volumes and reflection (environment map) probes.
- **`decal/`**: `clustered.rs` (projected using the same cluster grid as lights) and
  `forward.rs`.
- **`contact_shadows.rs`**: see [lighting-and-shadows](./lighting-and-shadows.md).
- **`meshlet/`**, **`deferred/`**, **`prepass/`**, **`lightmap/`**,
  **`environment_map/`**, **`transmission/`**, **`ltc/`** (linearly-transformed cosines,
  area/rect-light specular), **`bluenoise/`** (dithering for temporal techniques):
  supporting subsystems.
- **`fog.rs`**, **`gltf.rs`**, **`wireframe.rs`**, **`diagnostic.rs`**: distance fog, glTF
  material import glue, wireframe overlay, render diagnostics.

## The `Material` trait

Defined in **`bevy_pbr::material`** (not `bevy_material` — see
[material-system](../materials-and-shaders/material-system.md) for why):

```rust
pub trait Material: Asset + AsBindGroup + Clone + Sized {
    fn vertex_shader() -> ShaderRef { ShaderRef::Default }
    fn fragment_shader() -> ShaderRef { ShaderRef::Default }
    fn alpha_mode(&self) -> AlphaMode { AlphaMode::Opaque }
    fn opaque_render_method(&self) -> OpaqueRendererMethod { OpaqueRendererMethod::Forward }
    fn depth_bias(&self) -> f32 { 0.0 }
    fn reads_view_transmission_texture(&self) -> bool { false }
    fn enable_prepass() -> bool { true }
    fn enable_shadows() -> bool { true }
    fn prepass_fragment_shader() -> ShaderRef { ShaderRef::Default }
    fn deferred_fragment_shader() -> ShaderRef { ShaderRef::Default }
    fn specialize(pipeline: &MaterialPipeline, descriptor: &mut RenderPipelineDescriptor,
                  layout: &MeshVertexBufferLayoutRef, key: MaterialPipelineKey<Self>)
                  -> Result<(), SpecializedMeshPipelineError> { Ok(()) }
    // ...
}
```

`Asset` makes it hot-reloadable/handle-addressable; `AsBindGroup` (derived, see
[gpu-resources-and-device](../resources-and-assets/gpu-resources-and-device.md)) generates
the WGSL-compatible uniform buffer layout and texture/sampler bindings automatically from
struct fields; `specialize` is the hook for varying the pipeline (shader defs, primitive
state) per-material-instance based on a `MaterialPipelineKey`.

## `StandardMaterial`

Bevy's implementation, following the metallic-roughness glTF PBR convention:

```rust
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone)]
#[bind_group_data(StandardMaterialKey)]
#[data(0, StandardMaterialUniform, binding_array(10))]
#[bindless(index_table(range(0..31)))]
pub struct StandardMaterial {
    pub base_color: Color,
    #[texture(1)] #[sampler(2)] #[dependency]
    pub base_color_texture: Option<Handle<Image>>,
    pub emissive: LinearRgba,
    pub perceptual_roughness: f32,
    pub metallic: f32,
    // ... diffuse/specular transmission, thickness, ior, attenuation (subsurface/glass)
    // ... normal_map_texture, occlusion_texture (AO)
    // ... specular_texture, specular_tint_texture (F0 tinting)
    // ... clearcoat_texture, clearcoat_roughness_texture, clearcoat_normal_texture
    // ... anisotropy_strength, anisotropy_rotation, anisotropy_texture
    pub alpha_mode: AlphaMode,
    // ...
}
```

Texture slots follow a consistent pattern: each optional `Handle<Image>` pairs with
`#[texture(N)] #[sampler(N+1)]` (derived by `AsBindGroup`) and an independently
selectable `UvChannel` (`base_color_channel`, `metallic_roughness_channel`, ...), letting
each texture sample a different UV set. `#[bindless(index_table(range(0..31)))]` opts into
Bevy's bindless-texture-array path where supported; `binding_array(10)` batches multiple
material instances' uniform data into one bind group for reduced draw-call state changes
(managed at the render-graph level by `material_bind_groups.rs`). Scalar/vector fields
pack into a companion `StandardMaterialUniform` GPU struct plus a
`StandardMaterialFlags` bitflag (packing `alpha_mode` + boolean feature toggles into one
u32) so the shader can branch cheaply on which optional features are active.

`impl Material for StandardMaterial` wires this in: `fragment_shader()`/
`prepass_fragment_shader()`/`deferred_fragment_shader()` all point at embedded
`render/pbr.wgsl` (or `pbr_prepass.wgsl`); `specialize()` translates a
`StandardMaterialKey` bitflag (`NORMAL_MAP`, `RELIEF_MAPPING`, `DIFFUSE_TRANSMISSION`,
`SPECULAR_TRANSMISSION`, `CLEARCOAT`, `CLEARCOAT_NORMAL_MAP`, `ANISOTROPY`, ...) into WGSL
`shader_defs` (e.g. `STANDARD_MATERIAL_CLEARCOAT`), letting one shader source
conditionally compile in only the BRDF terms a given material instance actually needs.

`opaque_render_method()` demonstrates a practical trait override: it forces
`OpaqueRendererMethod::Forward` when diffuse transmission is nonzero, because the deferred
G-buffer doesn't currently carry the extra data transmission needs — worth copying as a
pattern when extending `StandardMaterial` via `MaterialExtension` (`extended_material.rs`),
which layers additional bindings/shader logic on top of `StandardMaterial` without
reimplementing `Material` from scratch.

## When to dive in

- Writing a custom PBR material from scratch → implement `Material` + `AsBindGroup`,
  following `StandardMaterial`'s texture-slot/UV-channel/shader-def pattern.
- Extending `StandardMaterial` with extra behavior without reimplementing it →
  `MaterialExtension`/`extended_material.rs`.
- Understanding why a material forces forward rendering when it "should" support deferred
  → check `opaque_render_method()` overrides — some features (transmission) are
  incompatible with the deferred G-buffer layout.
- Looking for SSAO, SSR, volumetric fog, light probes, decals → each has its own file
  listed above; not covered in depth in this knowledge base yet.

## Related
- [bevy_material: shared material plumbing](../materials-and-shaders/material-system.md) — contrast: the shared keys and properties the `Material` trait builds on.
- [RenderDevice, RenderContext, and bind groups](../resources-and-assets/gpu-resources-and-device.md) — prerequisite: the `AsBindGroup` derive every material uses.
- [Shadow rendering](./lighting-and-shadows.md) — deeper: `render/light.rs` shadow views.
- [Clustered forward rendering](./clustered-forward-rendering.md) — deeper: the `cluster/` module.
- [migera pivots to Bevy PBR for characters](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — applies: why migera's characters render through `StandardMaterial` rather than `src/hybrid`.
