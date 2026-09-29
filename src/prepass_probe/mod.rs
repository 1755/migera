//! Write-and-throw spike: does a hardcoded SDF sphere, ray-traced by a
//! compute pass, work if it writes Bevy's OWN prepass/deferred-G-buffer
//! textures instead of shading directly like `crate::hybrid` does?
//!
//! Two GPU programs, mirroring `crate::hybrid::pipeline`'s shape:
//! - `sphere_prepass_trace.wgsl @compute`: analytic ray-sphere intersection,
//!   one invocation per pixel, writes hit-t + world-space normal to two
//!   small scratch storage textures (NOT `ViewPrepassTextures` directly —
//!   those are created with `RENDER_ATTACHMENT | TEXTURE_BINDING` only, no
//!   `STORAGE_BINDING`, so a compute shader can't `textureStore` into them).
//! - `sphere_prepass_blit.wgsl @fragment`: fullscreen pass that reads the
//!   scratch textures and writes into the REAL `ViewDepthTexture` (frag_depth)
//!   and the REAL `ViewPrepassTextures.deferred`/`deferred_lighting_pass_id`
//!   attachments, packed to exactly match `bevy_pbr`'s deferred G-buffer
//!   layout — see that shader's doc comment for field-by-field citations.
//!
//! Critical ordering difference from `crate::hybrid`: this plugin's writer
//! system runs in `Core3dSystems::Prepass` (before `main_opaque_pass_3d`),
//! not `Core3dSystems::MainPass` — Bevy's SSAO dispatch runs strictly between
//! `Prepass` and `MainPass`, and the stock deferred lighting pass itself
//! also expects the G-buffer to be ready before the main pass starts.
//! `crate::hybrid::pass::hybrid_pass` runs `.after(main_opaque_pass_3d)`
//! specifically because ITS job is compositing already-shaded color into the
//! view target after the fact — the opposite goal from this plugin, which
//! hands off *unshaded* material data for Bevy's own main pass to shade.

pub mod extract;
pub mod pass;
pub mod pipeline;

use bevy::core_pipeline::Core3d;
use bevy::core_pipeline::schedule::Core3dSystems;
use bevy::prelude::*;
use bevy::render::Render;
use bevy::render::RenderApp;
use bevy::render::RenderStartup;
use bevy::render::RenderSystems;

/// Main-world description of the one sphere this spike renders — set once at
/// spawn time, extracted into the render world verbatim (no BVH, no CSG, no
/// per-frame scene diffing; this is deliberately as small as the existing
/// `crate::hybrid` extraction pipeline is large).
#[derive(Component, Clone, Copy, Debug, bevy::render::extract_component::ExtractComponent)]
pub struct SdfProbeSphere {
    pub center: Vec3,
    pub radius: f32,
    pub base_color: Vec3,
    pub metallic: f32,
    pub roughness: f32,
    pub reflectance: f32,
}

impl Default for SdfProbeSphere {
    fn default() -> Self {
        Self {
            center: Vec3::ZERO,
            radius: 1.0,
            base_color: Vec3::new(0.8, 0.2, 0.2),
            metallic: 0.0,
            roughness: 0.5,
            reflectance: 0.5,
        }
    }
}

/// Main-world description of the ground plane's geometry — mirrors the
/// `Mesh3d` plane an example spawns, but the compute trace needs its OWN
/// analytic copy to ray-intersect for reflection rays (see `extract`'s doc
/// comment for why a mesh can't be intersected analytically from a compute
/// shader without duplicating its geometry anyway). Carries its own
/// `center_xz` (not paired with `SdfProbeSphere` by array index) so
/// `--stress N` grid instances stay correct even if the two components'
/// query iteration orders ever diverge — each plane is fully self-describing.
#[derive(Component, Clone, Copy, Debug, bevy::render::extract_component::ExtractComponent)]
pub struct SdfProbeGroundPlane {
    pub center_xz: Vec2,
    pub y: f32,
    pub half_size: f32,
    pub base_color: Vec3,
    pub roughness: f32,
}

impl Default for SdfProbeGroundPlane {
    fn default() -> Self {
        Self {
            center_xz: Vec2::ZERO,
            y: -1.0,
            half_size: 4.0,
            base_color: Vec3::new(0.5, 0.5, 0.55),
            roughness: 0.3,
        }
    }
}

pub struct PrepassProbePlugin;

impl Plugin for PrepassProbePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            bevy::render::extract_component::ExtractComponentPlugin::<SdfProbeSphere>::default(),
            bevy::render::extract_component::ExtractComponentPlugin::<SdfProbeGroundPlane>::default(
            ),
        ));

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .init_resource::<extract::ProbeLights>()
            .add_systems(
                RenderStartup,
                (
                    pipeline::init_probe_pipeline,
                    pipeline::init_shadow_write_pipeline,
                ),
            )
            .add_systems(bevy::render::ExtractSchedule, extract::extract_probe_lights)
            .add_systems(
                Render,
                pipeline::prepare_probe_scene.in_set(RenderSystems::PrepareBindGroups),
            )
            .add_systems(
                Core3d,
                (
                    // Must finish before main_opaque_pass_3d: SSAO's own compute
                    // dispatch runs `.after(Core3dSystems::Prepass)
                    // .before(Core3dSystems::MainPass)`, and the stock deferred
                    // lighting pass reads ViewPrepassTextures.deferred as part of
                    // MainPass itself — both need our G-buffer write to have
                    // already happened.
                    //
                    // Also must run BEFORE Bevy's own mesh prepass systems
                    // (`early_prepass`/`early_deferred_prepass`/`late_prepass`/
                    // `late_deferred_prepass`, chained in that order): our blit
                    // draws a FULLSCREEN triangle every frame, writing a
                    // zeroed/"miss" G-buffer entry to every pixel our sphere
                    // didn't hit — including pixels a mesh (e.g. this example's
                    // ground plane) will draw its own real G-buffer data into.
                    // If we ran after the mesh prepass, our fullscreen write
                    // would clobber the mesh's data for every one of its own
                    // pixels with our miss value, leaving it unlit (this was
                    // observed directly: the ground plane rendered solid black
                    // until this ordering was added). Ordering before
                    // `early_prepass` — the first system in that chain — makes
                    // the mesh's own prepass draws happen strictly after ours
                    // and correctly win for their own pixels; `early_deferred_prepass`
                    // itself is `pub(crate)` in bevy_core_pipeline so it can't be
                    // named directly, but ordering against the chain's first
                    // (public) member transitively orders against the whole
                    // chain since `Core3dSystems::Prepass`'s own systems are
                    // `.chain()`-ordered internally.
                    pass::probe_prepass_write
                        .in_set(Core3dSystems::Prepass)
                        .before(bevy::core_pipeline::prepass::node::early_prepass),
                    // Shadow-map writers (sphere-casts-shadow follow-up): NOT
                    // in Core3dSystems::Prepass — bevy_pbr's own shadow-pass
                    // systems aren't members of that set either, only bounded
                    // by it (Prepass/MainPass are only used for the G-buffer/
                    // prepass writer above). Ordered after Bevy's own
                    // per-view/shared shadow passes (LATE_SHADOW_PASS variant,
                    // the last one to touch each shadow map this frame) so our
                    // write is the non-first DepthAttachment call this frame —
                    // i.e. we correctly get LoadOp::Load and composite atop any
                    // real mesh shadow caster instead of clearing the map back
                    // to empty (see pass.rs's module doc comment on
                    // `write_sphere_shadow`) — and before MainPass, since
                    // shading (which samples the shadow map) happens there.
                    pass::probe_shadow_write_point_spot
                        .after(bevy::pbr::shared_shadow_pass::<{ bevy::pbr::LATE_SHADOW_PASS }>)
                        .before(Core3dSystems::MainPass),
                    pass::probe_shadow_write_directional
                        .after(bevy::pbr::per_view_shadow_pass::<{ bevy::pbr::LATE_SHADOW_PASS }>)
                        .before(Core3dSystems::MainPass),
                ),
            );
    }
}
