---
title: "Shadow rendering: shadow maps, cascades, and contact shadows"
description: Bevy 0.19.1 renders shadow maps as per-light depth views keyed by RetainedViewEntity (6 cube faces per point light, 1 per spot, N cascades per directional), distributes cascades via CascadeShadowConfig, and adds contact shadows (needs DepthPrepass). Read when shadows alias, acne, detach, or look soft at contact.
type: reference
status: current
tags:
  - bevy
  - shadows
  - lighting
  - render-pipeline
  - debugging
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_pbr-0.19.1/src/render/light.rs
  - bevy_pbr-0.19.1/src/contact_shadows.rs
aliases:
  - cascaded shadow maps
  - CSM
  - contact shadows
  - shadow acne
  - peter-panning
  - shadow bias
---

# Shadow rendering: shadow maps, cascades, and contact shadows

## Shadow maps as per-light views

Shadow maps render as ordinary depth-only passes from **per-light views**, constructed
and managed in `bevy_pbr::render::light`. For each shadow-casting light, extraction builds
one or more `ShadowView`/`ExtractedView` entities — 6 per point light (cubemap faces,
near-infinite reversed-Z perspective per face via `Mat4::perspective_infinite_reverse_
rh`), 1 per spot light (single perspective cone), N per directional light (one per
cascade). Each auxiliary view is tagged with a `RetainedViewEntity`:

```rust
let retained_view_entity = RetainedViewEntity::new(*light_main_entity, None, face_index as u32);
```

`RetainedViewEntity` (see
[camera-and-view-system](../scene-and-views/camera-and-view-system.md)) is a stable key —
`(main_entity, auxiliary_entity, subview_index)` — letting the renderer persistently
identify "the shadow view for cube face 3 of light entity X" across frames, enabling
incremental/cached shadow map updates and correct entity extraction without recreating
render-world entities every frame.

Because shadow-map views aren't real cameras (no `Camera` component the user sees), each
is also inserted with `RootNonCameraView(Core3d.intern())` — a marker from
`bevy_core_pipeline::schedule` (see
[camera-driven-scheduling](../core-pipeline/camera-driven-scheduling.md)) telling the
render schedule this view belongs to the `Core3d` graph's root but isn't a top-level
camera view, so it participates in the same extraction/visibility/batching machinery
(mesh culling against the shadow frustum, indirect draws) while being excluded from "draw
one final image per camera" bookkeeping.

## Cascaded shadow maps (directional lights)

Built in `bevy_light::cascade`: `CascadeShadowConfig { bounds, overlap_proportion,
minimum_distance }`, populated via `CascadeShadowConfigBuilder` which geometrically
distributes cascade far-bounds using a power curve (`calculate_cascade_bounds`, `base =
(max_distance/near_bound)^(1/(n-1))`) so cascades grow exponentially with distance —
mitigating perspective aliasing (near-camera texels covering less world-space area than
far ones) without needing linear cascade counts. `build_directional_light_cascades` fits
each cascade's orthographic frustum tightly around the intersection of the view frustum
slice and the light direction. Each cascade gets its own `RetainedViewEntity` subview
index and its own slice of the (array-textured) directional shadow map, sized by
`DirectionalLightShadowMap` (power-of-two, enforced by `validate_shadow_map_size`).

## Contact shadows (new in 0.19)

Implemented in `bevy_pbr::contact_shadows` — **not** additional shadow-map views, but a
**screen-space raymarch** against the depth prepass:

```rust
#[require(bevy_core_pipeline::prepass::DepthPrepass)]
pub struct ContactShadows {
    pub linear_steps: u32,   // default 16
    pub thickness: f32,      // default 0.1 — assumed surface thickness for depth "cuboids"
    pub length: f32,         // default 0.3 — ray length in world space
}
```

A per-camera opt-in (`ContactShadows` component, requiring `DepthPrepass`), enabled per-
light via `contact_shadows_enabled` on `PointLight`/`SpotLight`/`DirectionalLight`. The
actual marching happens in `bevy_pbr/src/render/pbr_functions.wgsl`'s
`calculate_contact_shadow()`, walking the screen-space depth buffer along the light
direction from the shaded fragment, using `raymarch::depth_ray_march_to_ws` (shared with
[screen-space-reflections](./screen-space-reflections.md)'s raymarcher) and multiplying
the existing shadow-map-derived `shadow` factor by the contact-shadow result — contact
shadows **sharpen** shadow-map shadows near contact points (where shadow-map resolution/
bias typically causes gaps or light leaking) rather than replacing them; they only apply
when the pixel is flagged `MESH_FLAGS_SHADOW_RECEIVER_BIT` and the shadow-map term is
nonzero. This makes them cheap (a short, fixed-step raymarch, no extra shadow-map passes)
and purely a screen-space quality enhancement layered on top of the classic shadow-map
pipeline.

## Shadow bias tuning

`shadow_depth_bias`/`shadow_normal_bias` on each light type are tuned against two opposing
artifacts: "shadow acne" (self-shadowing noise from insufficient bias) vs. "Peter-panning"
(shadows visibly detached from their casters from excessive bias). There's no universal
correct value — it depends on shadow map resolution, scene scale, and light-to-surface
distance.

## When to dive in

- Directional light shadows look aliased at a distance → tune `CascadeShadowConfig`
  bounds/overlap, or increase `DirectionalLightShadowMap` resolution (must stay power-of-
  two).
- Shadows show acne or detach from casters → adjust `shadow_depth_bias`/
  `shadow_normal_bias` on the specific light.
- Contact/near-field shadow detail looks weak (typical shadow-map softness near contact
  points) → enable `ContactShadows` on the camera and `contact_shadows_enabled` on the
  relevant light; requires `DepthPrepass`.
- Implementing a new kind of auxiliary (non-camera) view → follow the
  `RetainedViewEntity` + `RootNonCameraView` pattern shadow views use, don't invent a
  parallel mechanism.

## Related
- [bevy_light: light components, atmosphere, gizmos](./light-components-and-atmosphere.md) — prerequisite: the per-light shadow and bias fields.
- [Camera and the view system](../scene-and-views/camera-and-view-system.md) — deeper: `RetainedViewEntity` and how views are extracted.
- [Screen-space reflections](./screen-space-reflections.md) — contrast: shares the depth-raymarch helper with contact shadows.
- [Prepass, tonemapping, upscaling, deferred, and OIT](../core-pipeline/passes-and-fullscreen-effects.md) — prerequisite: the `DepthPrepass` contact shadows need.
