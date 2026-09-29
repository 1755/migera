---
title: "bevy_light: light components, atmosphere, gizmos"
description: Bevy 0.19.1's bevy_light is a GPU-free data crate - PointLight/SpotLight/DirectionalLight with their shadow-bias fields, Atmosphere as a standalone world entity (no longer a camera attachment), ScatteringMedium, and light debug gizmos; bevy_pbr owns the GPU side. Read when configuring lights, shadow bias, or sky.
type: reference
status: current
tags:
  - bevy
  - lighting
  - shadows
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_light-0.19.1/src
aliases:
  - PointLight
  - DirectionalLight
  - Atmosphere entity
  - ScatteringMedium
  - ShowLightGizmo
---

# bevy_light: light components, atmosphere, gizmos

## Provenance

`bevy_light`'s `Cargo.toml` description: "Keeps the lights on at Bevy Engine." Its
`lib.rs` doc: "Provides component types for lighting a bevy scene. This includes the
usual directional, point, and spot lights, as well as light probes, atmosphere, other
volumetrics, and shadow configuration." It depends only on `bevy_app`, `bevy_camera`,
`bevy_ecs`, `bevy_transform`, `bevy_asset`, `bevy_math`, `bevy_mesh`, `bevy_color`, and
optionally `bevy_gizmos` — notably **not** `bevy_render` or `wgpu` directly. This is
deliberate: `bevy_light` is a pure **data/logic** crate (components, CPU-side cascade
math, frustum culling, gizmo debug drawing); `bevy_pbr` owns the GPU-facing extraction,
buffer layout, and WGSL. This lets other renderers or headless simulations reuse light
data without pulling in the full PBR render graph. Confirms the migration-guide note that
`Atmosphere`, `ScatteringMedium`, and light gizmos moved here from `bevy_pbr`/
`bevy_gizmos`.

## Light components

`PointLight`, `SpotLight`, `DirectionalLight` are `#[derive(Component)]` structs using
`#[require(...)]` to auto-attach what their rendering needs (frusta, `VisibleEntities`,
`Transform`, `Visibility`, and a `VisibilityClass` tagged `ClusterVisibilityClass` — the
marker the clustering system uses to find "clusterable objects"). Each has an `on_add`
hook (`add_visibility_class::<ClusterVisibilityClass>`) registering it for clustering
automatically — spawning `PointLight::default()` is enough to participate in clustered
rendering with no extra setup.

- **`PointLight`**: `intensity` (lumens), `range`, `radius` (area-light approximation
  affecting specular highlight/penumbra size, not diffuse), plus shadow controls
  (`shadow_maps_enabled`, `shadow_depth_bias`/`shadow_normal_bias`, `shadow_map_near_z`,
  `contact_shadows_enabled`). Shadow resolution is global via `PointLightShadowMap { size
  }` (default 1024, per cubemap face).
- **`SpotLight`**: adds `inner_angle`/`outer_angle` for cone falloff, shares shadow
  bias/near-z knobs. Builds its view/projection as a 90°-scaled perspective by
  `outer_angle * 2`.
- **`DirectionalLight`**: uses `illuminance` (lux, not lumens — physically correct since
  a directional light has no falloff), delegates shadowing to **cascaded shadow maps** via
  required `Cascades`/`CascadeShadowConfig`/`CascadesFrusta`. Resolution via
  `DirectionalLightShadowMap { size }` (default 2048, must be power-of-two). A companion
  `SunDisk` component (requiring `DirectionalLight`) controls the visible solar disk when
  an `Atmosphere` is present, with `SunDisk::EARTH`/`SunDisk::OFF` presets.

All three share `PCSS`-gated `soft_shadows_enabled`/`soft_shadow_size` fields (feature
`experimental_pbr_pcss`), and `*Texture` companions (`PointLightTexture`,
`SpotLightTexture`, `DirectionalLightTexture`) applying an R-channel "cookie"/gobo mask.

## Atmosphere — no longer a camera attachment

```rust
#[derive(Clone, Component, FromTemplate)]
#[require(GlobalTransform)]
#[component(on_add = set_default_transform)]
pub struct Atmosphere {
    pub inner_radius: f32,
    pub outer_radius: f32,
    pub ground_albedo: Vec3,
    pub medium: Handle<ScatteringMedium>,
}
```

The doc comment: *"Add `AtmosphereSettings` to each 3D camera that should use it, the
nearest atmosphere is used for rendering."* `Atmosphere` is now spawned as its own entity
representing a **planet**, whose `GlobalTransform` is the planet's center in world space;
cameras opt in via a separate `AtmosphereSettings` component referencing "the nearest
atmosphere." The `on_add` hook places the entity `inner_radius` units below the origin
along `-Y` if no transform was set, so "ground" is roughly at scene origin by default.

**Why this changed**: in 0.18, `Atmosphere` (fixed Earth-like radii) lived directly on the
camera, making it a *view-space effect* rather than a *world object* — no multiple
planets, no positioning the camera at planetary scale, no reuse across cameras. As a
standalone entity with its own `Transform`, the transform's **scale** can now rescale the
whole planet to fit a scene's unit convention (no need for literal `inner_radius =
6_360_000.0` at kilometer-scale). It naturally supports multiple `Atmosphere` entities
(e.g. Earth and Mars in one scene) with the renderer picking the nearest per camera; the
transform's rotation is ignored since scattering is spherically symmetric.

## `ScatteringMedium`

An `Asset` describing *how a substance scatters light*, decoupled from `Atmosphere` (which
only holds a `Handle<ScatteringMedium>`). This is the physically-based multi-scattering
path for Bevy's Bruneton-style atmosphere renderer (LUT shaders in `bevy_pbr/src/
atmosphere/*.wgsl`: `transmittance_lut.wgsl`, `multiscattering_lut.wgsl`,
`sky_view_lut.wgsl`, `aerial_view_lut.wgsl`). A medium is a `SmallVec<[ScatteringTerm;
1]>`, each term carrying `absorption`/`scattering` (per-wavelength optical density, m⁻¹),
a `Falloff` (Linear, Exponential-scale-height, Tent, or arbitrary `Curve`) describing
density-vs-altitude, and a `PhaseFunction` (Isotropic, Rayleigh, Henyey–Greenstein `Mie {
asymmetry }`, or chromatic curve/texture) describing directional scattering.
`ScatteringMedium::earth()` builds the canonical three-term atmosphere (Rayleigh gas, Mie
aerosols, ozone absorption) with real physical constants (8 km/1.2 km scale heights);
`ScatteringMedium::mars()` demonstrates the chromatic-texture path for wavelength-
dependent dust phase functions. On the GPU side these terms bake into LUTs consumed by
`bevy_pbr::GpuScatteringMedium`.

## Light gizmos

`bevy_light::gizmos` (feature-gated on `bevy_gizmos`) provides `LightGizmoPlugin`, drawing
debug primitives for `PointLight` (radius/range spheres), `SpotLight` (radius sphere +
inner/outer cones + arcs), `DirectionalLight` (direction arrow), `RectLight` (rectangle
outline + arrow). Coloring is configurable per-light (`ShowLightGizmo { color: Option<
LightGizmoColor> }`) or globally (`LightGizmoConfigGroup`: `draw_all`, per-type default
colors, a strategy enum `Manual`/`Varied` (hashed from entity index)/`MatchLightColor`/
`ByLightType`).

## When to dive in

- Configuring shadow quality/bias for a specific light type → the shadow fields on
  `PointLight`/`SpotLight`/`DirectionalLight` described above; see
  [lighting-and-shadows](./lighting-and-shadows.md) for how these feed into the actual
  shadow-map rendering.
- Building a sky/atmosphere system, especially multi-planet or non-Earth-scale scenes →
  spawn `Atmosphere` as its own entity with a scaled `Transform`, not as a camera
  attachment.
- Adding debug visualization for lights → `ShowLightGizmo`/`LightGizmoConfigGroup`.

## Related
- [Shadow rendering](./lighting-and-shadows.md) — deeper: how these shadow fields drive shadow-map views, cascades and contact shadows.
- [Clustered forward rendering](./clustered-forward-rendering.md) — deeper: how point/spot lights are assigned to clusters.
- [Gizmo rendering](../2d-and-ui/gizmo-rendering.md) — deeper: the gizmo pipeline light gizmos draw through (3D gizmos are depth-tested).
- [Bevy-native integration](../../hybrid-architecture/bevy-native-integration.md) — applies: `src/hybrid` reads `DirectionalLight` as extraction input and documents its remaining light gap.
