//! The hybrid SDF renderer — ground-up rewrite.
//!
//! Started fresh after `src/hybrid_legacy` (its frozen predecessor)
//! accumulated a long history of subtle marching/formula bugs that were
//! slow and error-prone to debug directly on the GPU. The lesson carried
//! forward: build each piece of non-trivial math as a CPU-testable
//! reference first (see `cpu_ref`), prove it correct with `cargo test`,
//! and only then port it to WGSL — not the other way around. See
//! `src/hybrid_legacy/mod.rs`'s doc comment and
//! `docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md` for the full
//! history of what that cost.
//!
//! **Architecture: self-shading, one connected pipeline — not a Bevy
//! G-buffer writer.** `hybrid` traces and shades entirely on its own
//! (today fused into one compute dispatch, mirroring `hybrid_legacy`'s
//! `hybrid_trace.wgsl`), then blits already-lit color straight into
//! Bevy's `ViewTarget`/depth attachment. This was a deliberate choice
//! between two real options, made after weighing this project's own
//! constraints (an SDF-only scene, forever — no mesh interop) against a
//! prior attempt at the alternative; see
//! `docs/knowledge/hybrid-architecture/self-shading-vs-gbuffer-decision.md`
//! for the full reasoning, and `src/prepass_probe/mod.rs`'s doc comment
//! for the alternative that was considered and set aside (kept as a spike
//! for comparison, not extended). See
//! `docs/knowledge/hybrid-architecture/module-and-stage-skeleton.md` for
//! the module map and stage-ordering contract.
//!
//! First real trace step landed: BVH-accelerated primary rays,
//! sphere-marched against `RoundedBox` geometry, resolving to a flat
//! per-object color (no lighting/shadows/reflections yet — see
//! `cpu_ref`'s doc comment for that step's exact scope). Each feature gets
//! added as its own proven-correct step; see `PROGRESS.md` for what's
//! landed and `docs/knowledge/hybrid-architecture/` for the architecture
//! this builds toward.

pub mod bvh;
pub mod conetrace_ref;
pub mod cpu_ref;
pub mod ddgi_ref;
pub mod dof_ref;
pub mod extract;
pub mod grain_ref;
pub mod material;
pub mod motion;
pub mod pass;
pub mod pipeline;
pub mod post;
pub mod radiance_cascades_ref;
pub mod reflect_ref;
pub mod refract_ref;
pub mod scene;
pub mod taa_ref;
pub mod temporal_ref;

use bevy::core_pipeline::Core3d;
use bevy::core_pipeline::core_3d::main_opaque_pass_3d;
use bevy::core_pipeline::core_3d::main_transparent_pass_3d;
use bevy::core_pipeline::schedule::Core3dSystems;
use bevy::core_pipeline::tonemapping::tonemapping;
use bevy::prelude::*;
use bevy::render::ExtractSchedule;
use bevy::render::Render;
use bevy::render::RenderApp;
use bevy::render::RenderStartup;
use bevy::render::RenderSystems;
use bevy::render::render_resource::SpecializedRenderPipelines;

use crate::physics::gpu::buffers::{BroadphaseGpuState, ContactGenGpuState, PhysicsGpuState};
use crate::physics::gpu::extract::{RenderPhysicsGpuFrame, extract_physics_bodies};
use crate::physics::gpu::frame::dispatch_physics_gpu_frame;
use crate::physics::gpu::pipelines::{
    init_broadphase_hash_pipeline, init_broadphase_scatter_pipeline, init_contact_gen_pipeline, init_extract_positions_pipeline, init_physics_gpu_pipeline, init_sample_points_gpu_pipeline, init_scan_gpu_pipeline,
};
use crate::physics::gpu::readback::{MainWorldPhysicsGpuResult, PhysicsGpuReadbackState, physics_gpu_apply_readback, sync_physics_gpu_readback};
use crate::physics::gpu::sample_cache::SamplePointsCache;
use crate::physics::integrate::PhysicsGpuEnabled;
use crate::physics::solve_static::PhysicsGravity;

pub struct HybridRenderPlugin;

impl Plugin for HybridRenderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<bvh::PersistentBvh>()
            // `extract_physics_bodies` (registered on the render sub-app
            // below) reads this via `Extract<Res<PhysicsGpuEnabled>>` every
            // frame regardless of whether the app also adds
            // `physics::integrate::PhysicsPlugin` -- an app that ONLY wants
            // the SDF renderer (e.g. an avian3d-based example, which
            // deliberately does NOT add the demoted custom engine's own
            // PhysicsPlugin) would otherwise panic with "Resource does not
            // exist" the moment extraction runs. `init_resource` here means
            // `HybridRenderPlugin` never depends on plugin registration
            // order/presence elsewhere for a resource its own extraction
            // system requires -- `PhysicsPlugin`, if also added, simply
            // finds the resource already present (`init_resource` is a
            // no-op on a resource that already exists) and can still flip
            // it via `insert_resource` as it always has.
            .init_resource::<PhysicsGpuEnabled>()
            // Same reasoning as `PhysicsGpuEnabled` immediately above --
            // `extract_physics_bodies` also reads `Extract<Res<PhysicsGravity>>`
            // unconditionally every frame, regardless of whether the app
            // also adds the demoted custom CPU engine's own `PhysicsPlugin`.
            .init_resource::<PhysicsGravity>()
            .init_resource::<MainWorldPhysicsGpuResult>()
            .add_systems(Update, bvh::update_persistent_bvh)
            .add_systems(
                PreUpdate,
                (physics_gpu_apply_readback.before(motion::update_previous_shape_transforms), motion::update_previous_shape_transforms),
            )
            .add_plugins(extract::HybridExtractionPlugin);

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .init_resource::<SpecializedRenderPipelines<pipeline::HybridPipeline>>()
            .init_resource::<SpecializedRenderPipelines<post::HybridPostPipeline>>()
            .init_resource::<RenderPhysicsGpuFrame>()
            .init_resource::<PhysicsGpuState>()
            .init_resource::<SamplePointsCache>()
            .init_resource::<BroadphaseGpuState>()
            .init_resource::<ContactGenGpuState>()
            .init_resource::<PhysicsGpuReadbackState>()
            .add_systems(
                RenderStartup,
                (
                    pipeline::init_hybrid_pipeline,
                    pipeline::init_hybrid_buffers,
                    post::init_hybrid_post_pipeline,
                    post::init_hybrid_post_buffers,
                    init_physics_gpu_pipeline,
                    init_sample_points_gpu_pipeline,
                    init_scan_gpu_pipeline,
                    init_broadphase_hash_pipeline,
                    init_broadphase_scatter_pipeline,
                    init_contact_gen_pipeline,
                    init_extract_positions_pipeline,
                ),
            )
            .add_systems(ExtractSchedule, (extract_physics_bodies, sync_physics_gpu_readback))
            .add_systems(Render, dispatch_physics_gpu_frame.in_set(RenderSystems::Render))
            .add_systems(
                Render,
                (
                    pipeline::prepare_hybrid_view_bind_groups.in_set(RenderSystems::PrepareBindGroups),
                    pipeline::prepare_hybrid_scene.in_set(RenderSystems::PrepareBindGroups),
                    // Depends on prepare_hybrid_scene having already
                    // (re)created HybridTargets this frame (reads
                    // targets.size to decide whether history also needs
                    // resizing) — explicit ordering, not schedule-order
                    // luck. Also builds HybridDenoiseBindGroup (moved out
                    // of prepare_hybrid_scene — see hybrid_denoise_layout's
                    // own doc comment for why it needs this frame's
                    // ping-pong write_slot, only resolved here).
                    pipeline::prepare_hybrid_temporal
                        .in_set(RenderSystems::PrepareBindGroups)
                        .after(pipeline::prepare_hybrid_scene),
                    // Depends on prepare_hybrid_scene having already
                    // written this frame's SceneUniform/object/BVH/light
                    // buffers (reads their bindings directly) — same
                    // explicit-ordering reasoning as prepare_hybrid_temporal
                    // above, not schedule-order luck.
                    pipeline::prepare_hybrid_ddgi
                        .in_set(RenderSystems::PrepareBindGroups)
                        .after(pipeline::prepare_hybrid_scene),
                    // Same "depends on prepare_hybrid_scene's this-frame
                    // buffers" ordering as prepare_hybrid_ddgi immediately
                    // above — reads scene_uniform/objects/bvh/lights
                    // bindings directly.
                    pipeline::prepare_hybrid_radiance_cascades
                        .in_set(RenderSystems::PrepareBindGroups)
                        .after(pipeline::prepare_hybrid_scene),
                    // Depends on prepare_hybrid_scene having already
                    // (re)created HybridTargets this frame (reads
                    // targets.denoised_color_view/targets.size) and
                    // written this frame's object/BVH buffers (DOF fires
                    // its own trace ray) — same explicit-ordering
                    // reasoning as prepare_hybrid_temporal/
                    // prepare_hybrid_ddgi above.
                    pipeline::prepare_hybrid_dof
                        .in_set(RenderSystems::PrepareBindGroups)
                        .after(pipeline::prepare_hybrid_scene),
                    post::prepare_hybrid_post_uniform.in_set(RenderSystems::PrepareBindGroups),
                ),
            )
            .add_systems(
                Core3d,
                (
                    // Same ordering contract hybrid_legacy documents: Core3dPlugin
                    // chains only opaque->transparent; anything else in MainPass
                    // must pin itself against BOTH or risk drawing into a cleared
                    // target.
                    pass::hybrid_pass
                        .after(main_opaque_pass_3d)
                        .before(main_transparent_pass_3d)
                        .in_set(Core3dSystems::MainPass),
                    // Runs AFTER Bevy's own tonemapping system, not folded
                    // into hybrid_pass's own blit — see
                    // assets/shaders/hybrid_post.wgsl's own doc comment
                    // for why grain/vignette/chromatic-aberration are
                    // conventionally post-tonemap, display-referred
                    // effects rather than scene-linear ones. Requires the
                    // camera to carry Hdr + a non-None Tonemapping (same
                    // requirement hybrid_blit.wgsl's own linear-HDR output
                    // already has, see pass::hybrid_pass's doc comment) —
                    // without them tonemapping no-ops entirely (its own
                    // `if !camera.hdr { return }` early-out) and this pass
                    // still runs on the same untonemapped image, which is
                    // harmless (grain/vignette/CA still apply, just to
                    // linear-HDR content) but not the intended ordering.
                    post::hybrid_post_pass.in_set(Core3dSystems::PostProcess).after(tonemapping),
                ),
            );
    }
}
