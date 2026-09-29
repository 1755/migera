//! Direct GPU sphere-traced raymarcher: renders an SDF scene live, every pixel, every
//! frame, via sphere tracing — no baking step, the ECS-authored scene is flattened
//! into a GPU primitive buffer and evaluated fresh each frame. See `flatten`'s module
//! doc for the flattening scheme.
//!
//! Registration: a main-world `Update` system that assembles/flattens the ECS scene
//! (see `extract`'s doc comment), `ExtractSchedule` systems copying that into the
//! render world, `RenderStartup`/`Render`-schedule systems building the GPU
//! pipeline/buffers/bind groups (`pipeline.rs`), and one `Core3dSystems::MainPass`
//! system drawing the actual full-screen triangle (`pass.rs`).

pub mod extract;
pub mod flatten;
pub mod pass;
pub mod pipeline;

use bevy::core_pipeline::Core3d;
use bevy::core_pipeline::core_3d::{main_opaque_pass_3d, main_transparent_pass_3d};
use bevy::core_pipeline::schedule::Core3dSystems;
use bevy::prelude::*;
use bevy::render::render_resource::SpecializedRenderPipelines;
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderStartup, RenderSystems};

pub struct RaymarchRenderPlugin;

impl Plugin for RaymarchRenderPlugin {
    fn build(&self, app: &mut App) {
        extract::build_main_world(app);

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };

        render_app
            .init_resource::<SpecializedRenderPipelines<pipeline::RaymarchPipeline>>()
            .add_systems(
                RenderStartup,
                (
                    pipeline::init_raymarch_pipeline,
                    pipeline::init_raymarch_buffers,
                ),
            )
            .add_systems(
                ExtractSchedule,
                (
                    extract::extract_raymarch_static_scene,
                    extract::extract_raymarch_anim_isometries,
                    extract::extract_raymarch_lights,
                    extract::extract_raymarch_pattern_registry,
                    extract::extract_raymarch_debug_flags,
                ),
            )
            .add_systems(
                Render,
                (
                    pipeline::prepare_raymarch_view_bind_groups
                        .in_set(RenderSystems::PrepareBindGroups),
                    pipeline::prepare_raymarch_buffers.in_set(RenderSystems::PrepareBindGroups),
                ),
            )
            .add_systems(
                Core3d,
                // Explicit ordering against Bevy's own opaque/transparent passes is
                // required even though nothing else draws in this app:
                // `Core3dPlugin` only chains `(main_opaque_pass_3d,
                // main_transparent_pass_3d)` against each other via `.chain()`, which
                // says nothing about where a third system in the same
                // `Core3dSystems::MainPass` set runs relative to either of them.
                // Without this, `raymarch_pass` was observed running BEFORE
                // `main_opaque_pass_3d`, which clears/re-initializes the view target
                // on every frame regardless of whether it has anything queued — so the
                // raymarch draw was silently overwritten immediately after, producing
                // a permanently blank (clear-color) frame despite the pass genuinely
                // executing and drawing every frame.
                pass::raymarch_pass
                    .after(main_opaque_pass_3d)
                    .before(main_transparent_pass_3d)
                    .in_set(Core3dSystems::MainPass),
            );
    }
}
