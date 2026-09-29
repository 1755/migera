//! Piece 5's own verification tests — Tier 1 (toggle/plumbing wiring) and
//! Tier 2 (resting-position parity over real frames). See the plan's own
//! "Piece 5" section for why these are two deliberately separate tiers: a
//! wiring bug and a solver-accuracy bug are different failure classes that
//! should never be conflated in one test's pass/fail signal.
//!
//! Builds a headless `App` with a real `RenderPlugin` (same
//! `build_headless_render_app`-style harness `parity_test.rs` already
//! established) PLUS the physics-GPU-specific subset of
//! `HybridRenderPlugin`'s own registrations (not the full renderer
//! plugin — that needs shader assets/camera machinery this test has no
//! use for; only the physics extract/dispatch/readback wiring is under
//! test here).
//!
//! **Re-scoped to visual-effects-only** (see `effects::GpuDebrisBody`'s own
//! doc comment): every scene here spawns `GpuDebrisBody` instead of the
//! demoted CPU engine's own `RigidBody`/`Inertia`, and this file no longer
//! adds `physics::integrate::PhysicsPlugin` at all — there is no CPU
//! `solve_world` counterpart to this path's own debris bodies anymore, so
//! the old "GPU vs CPU parity" framing several of this file's tests used
//! doesn't apply. Every kinematic-body-specific test that used to live here
//! (moving-platform parity, kinematic-entity-untouched-by-readback,
//! kinematic+static coexistence, kinematic-near-point-source-gravity) was
//! REMOVED, not adapted — kinematic support has been deliberately dropped
//! from this path's own contract (interactive/moving-platform bodies are
//! now `avian3d`'s responsibility, see `crate::physics_avian`), so there is
//! nothing left for those tests to guard.

use bevy::app::PluginsState;
use bevy::asset::AssetPlugin;
use bevy::image::ImagePlugin;
use bevy::math::{EulerRot, Quat, Vec3};
use bevy::prelude::*;
use bevy::render::render_resource::PipelineCache;
use bevy::render::{ExtractSchedule, Render, RenderApp, RenderPlugin, RenderStartup, RenderSystems};
use bevy::time::TimeUpdateStrategy;

use super::buffers::{BroadphaseGpuState, ContactGenGpuState, PhysicsGpuState};
use super::effects::GpuDebrisBody;
use super::extract::{RenderPhysicsGpuFrame, extract_physics_bodies};
use super::frame::dispatch_physics_gpu_frame;
use super::pipelines::{
    BroadphaseHashPipeline, BroadphaseScatterPipeline, ContactGenPipeline, ExtractPositionsPipeline, PhysicsGpuPipeline, SamplePointsGpuPipeline, ScanGpuPipeline, init_broadphase_hash_pipeline, init_broadphase_scatter_pipeline, init_contact_gen_pipeline,
    init_extract_positions_pipeline, init_physics_gpu_pipeline, init_sample_points_gpu_pipeline, init_scan_gpu_pipeline,
};
use super::readback::{MainWorldPhysicsGpuResult, PhysicsGpuReadbackState, physics_gpu_apply_readback, sync_physics_gpu_readback};
use super::sample_cache::SamplePointsCache;
use super::super::components::PhysicsShape;
use super::super::inertia::box_inertia;
use super::super::integrate::PhysicsGpuEnabled;
use super::super::solve_static::PhysicsGravity;

/// Builds a headless `App` wired for the real GPU-effects physics path:
/// real `RenderPlugin` plus the physics-GPU-specific subset of
/// `HybridRenderPlugin`'s own registrations (pipeline init, extract,
/// dispatch, readback) — not the full renderer plugin, which needs shader
/// assets/camera machinery this test has no use for. Unlike this file's
/// own pre-re-scoping version, does NOT add `physics::integrate::PhysicsPlugin`
/// — there is no CPU solver on this path anymore, only
/// `PhysicsGpuEnabled` (inserted directly below) gates the GPU dispatch.
fn build_physics_gpu_test_app() -> App {
    let mut app = App::new();
    app.add_plugins(bevy::app::TaskPoolPlugin::default());
    app.add_plugins(bevy::diagnostic::FrameCountPlugin);
    app.add_plugins(bevy::time::TimePlugin);
    app.add_plugins(bevy::transform::TransformPlugin);
    app.add_plugins(bevy::window::WindowPlugin { primary_window: None, ..default() });
    app.add_plugins(AssetPlugin::default());
    app.add_plugins(RenderPlugin::default());
    app.add_plugins(ImagePlugin::default());
    app.add_plugins(bevy::mesh::MeshPlugin);
    app.add_plugins(bevy::camera::CameraPlugin);
    app.init_resource::<PhysicsGpuEnabled>();
    app.init_resource::<PhysicsGravity>();
    app.init_resource::<MainWorldPhysicsGpuResult>();
    app.add_systems(PreUpdate, physics_gpu_apply_readback.before(crate::hybrid::motion::update_previous_shape_transforms));
    app.add_systems(PreUpdate, crate::hybrid::motion::update_previous_shape_transforms);

    while app.plugins_state() == PluginsState::Adding {
        bevy::tasks::tick_global_task_pools_on_main_thread();
    }
    app.finish();
    app.cleanup();

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderPlugin should have created the RenderApp sub-app");
    render_app
        .init_resource::<RenderPhysicsGpuFrame>()
        .init_resource::<PhysicsGpuState>()
        .init_resource::<SamplePointsCache>()
        .init_resource::<BroadphaseGpuState>()
        .init_resource::<ContactGenGpuState>()
        .init_resource::<PhysicsGpuReadbackState>()
        .add_systems(
            RenderStartup,
            (init_physics_gpu_pipeline, init_sample_points_gpu_pipeline, init_scan_gpu_pipeline, init_broadphase_hash_pipeline, init_broadphase_scatter_pipeline, init_contact_gen_pipeline, init_extract_positions_pipeline),
        )
        .add_systems(ExtractSchedule, (extract_physics_bodies, sync_physics_gpu_readback))
        .add_systems(Render, dispatch_physics_gpu_frame.in_set(RenderSystems::Render));

    app
}

/// Pumps `app.update()` until every physics-GPU pipeline this test
/// exercises has reached `Ok` — same polling-not-guessing rationale as
/// `parity_test.rs`'s own `wait_for_pipeline_ready_generic`, ported here
/// directly since this test builds a differently-shaped app. Still checks
/// `contacts.kinematic_pipeline` — that pipeline/shader entry point still
/// exists (only `extract.rs`'s own ECS query was re-scoped, not the WGSL
/// dispatch chain, see `extract.rs`'s own doc comment for why), it simply
/// never receives any work on this path (`kinematic_count` is always `0`).
fn wait_for_all_physics_gpu_pipelines_ready(app: &mut App) -> bool {
    for _ in 0..300 {
        app.update();
        let render_app = app.get_sub_app(RenderApp).expect("RenderApp should exist");
        let world = render_app.world();
        let ready = (|| {
            let cache = world.get_resource::<PipelineCache>()?;
            let physics = world.get_resource::<PhysicsGpuPipeline>()?;
            let sample_points = world.get_resource::<SamplePointsGpuPipeline>()?;
            let scan = world.get_resource::<ScanGpuPipeline>()?;
            let broadphase_hash = world.get_resource::<BroadphaseHashPipeline>()?;
            let broadphase_scatter = world.get_resource::<BroadphaseScatterPipeline>()?;
            let contacts = world.get_resource::<ContactGenPipeline>()?;
            let extract_positions = world.get_resource::<ExtractPositionsPipeline>()?;
            let all_ready = cache.get_compute_pipeline(physics.predict_pipeline).is_some()
                && cache.get_compute_pipeline(physics.scatter_position_pipeline).is_some()
                && cache.get_compute_pipeline(physics.apply_position_pipeline).is_some()
                && cache.get_compute_pipeline(physics.scatter_velocity_pipeline).is_some()
                && cache.get_compute_pipeline(physics.apply_velocity_pipeline).is_some()
                && cache.get_compute_pipeline(sample_points.pipeline).is_some()
                && cache.get_compute_pipeline(scan.step_pipeline).is_some()
                && cache.get_compute_pipeline(scan.to_exclusive_pipeline).is_some()
                && cache.get_compute_pipeline(broadphase_hash.hash_pipeline).is_some()
                && cache.get_compute_pipeline(broadphase_hash.count_pipeline).is_some()
                && cache.get_compute_pipeline(broadphase_scatter.copy_pipeline).is_some()
                && cache.get_compute_pipeline(broadphase_scatter.scatter_pipeline).is_some()
                && cache.get_compute_pipeline(contacts.dynamic_pipeline).is_some()
                && cache.get_compute_pipeline(contacts.kinematic_pipeline).is_some()
                && cache.get_compute_pipeline(contacts.static_pipeline).is_some()
                && cache.get_compute_pipeline(extract_positions.pipeline).is_some();
            Some(all_ready)
        })();
        if ready == Some(true) {
            return true;
        }
    }
    false
}

/// A `GpuDebrisBody` box resting just above a static floor — the debris-
/// path equivalent of the old `RigidBody`-based scene this file's tests
/// used before the visual-effects re-scoping.
fn spawn_box_on_floor_scene(app: &mut App) -> Entity {
    let half_extents = Vec3::splat(0.5);
    let inverse_inertia = 1.0 / box_inertia(1.0, half_extents);

    app.world_mut().spawn((PhysicsShape::RoundedBox { half_extents: Vec3::new(10.0, 0.5, 10.0), corner_radius: 0.0 }, Transform::from_xyz(0.0, -0.5, 0.0)));

    app.world_mut()
        .spawn((
            PhysicsShape::RoundedBox { half_extents, corner_radius: 0.0 },
            Transform { translation: Vec3::new(0.0, 0.55, 0.0), rotation: Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, 0.1), ..default() },
            GpuDebrisBody { linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: inverse_inertia },
        ))
        .id()
}

/// Tier 1 — toggle/plumbing wiring test: confirms the full extract ->
/// dispatch -> readback -> apply loop executes end-to-end (not physical
/// accuracy, see this file's own doc comment for why that's a
/// deliberately separate concern).
#[test]
fn gpu_physics_toggle_wiring_moves_bodies_and_stays_finite() {
    let mut app = build_physics_gpu_test_app();
    app.insert_resource(PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 });
    app.insert_resource(PhysicsGpuEnabled(true));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0 / 60.0)));

    let entity = spawn_box_on_floor_scene(&mut app);
    let spawn_translation = app.world().get::<Transform>(entity).unwrap().translation;

    assert!(wait_for_all_physics_gpu_pipelines_ready(&mut app), "not every physics GPU pipeline reached Ok within the wait budget -- check for a shader compile error");

    // Poll for the position to actually change, rather than assuming a
    // fixed frame count -- the non-blocking map_async callback's own
    // completion timing is not guaranteed to land within any specific
    // number of app.update() calls.
    let mut moved = false;
    for _ in 0..60 {
        app.update();
        if app.world().get::<Transform>(entity).unwrap().translation != spawn_translation {
            moved = true;
            break;
        }
    }
    assert!(moved, "expected the GPU path to have moved the body from its spawn position within 60 frames -- the extract->dispatch->readback->apply loop may not have executed end-to-end");

    let transform = app.world().get::<Transform>(entity).unwrap();
    assert!(transform.translation.is_finite(), "expected a finite position after the GPU path ran, got {:?}", transform.translation);
    assert!(transform.rotation.is_finite(), "expected a finite rotation after the GPU path ran, got {:?}", transform.rotation);

    // Toggle off and confirm the body settles into a fixed position (no CPU
    // fallback exists on this path anymore -- unlike the old CPU/GPU dual-
    // backend version of this test, there is nothing to "fall back to").
    // The one-frame-latency readback means a dispatch already in flight at
    // the moment of the toggle can still land and apply ONE more time
    // after this point -- run a generous settle window before sampling
    // (a tighter 5-frame window was observed to flake once under system
    // load, presumably a slow-to-land in-flight readback still pending),
    // then confirm the position is bit-stable across several more frames.
    app.insert_resource(PhysicsGpuEnabled(false));
    for _ in 0..20 {
        app.update();
    }
    let settled = app.world().get::<Transform>(entity).unwrap().translation;
    for _ in 0..10 {
        app.update();
    }
    let after_toggle_off = app.world().get::<Transform>(entity).unwrap().translation;
    assert_eq!(after_toggle_off, settled, "expected the body to stay fixed once PhysicsGpuEnabled is toggled off and any in-flight readback has landed, with no CPU fallback on this path");
}

/// Tier 2 — resting-position stability over real frames: a settled body
/// stays settled (bounded velocity, finite position) over an extended run.
/// Unlike this file's own pre-re-scoping version, there is no CPU backend
/// left to compare against on this path — this test only confirms the GPU
/// path itself reaches and holds a stable rest state.
#[test]
fn gpu_path_settles_to_a_stable_resting_position() {
    let mut app = build_physics_gpu_test_app();
    app.insert_resource(PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 });
    app.insert_resource(PhysicsGpuEnabled(true));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0 / 60.0)));

    let entity = spawn_box_on_floor_scene(&mut app);
    assert!(wait_for_all_physics_gpu_pipelines_ready(&mut app), "not every physics GPU pipeline reached Ok within the wait budget -- check for a shader compile error");

    // Same 320-frame convention the pre-re-scoping version of this test
    // used (300 frames to settle, plus headroom for the GPU path's own
    // one-frame latency and pipeline-compile frames already consumed
    // above).
    for _ in 0..320 {
        app.update();
    }

    let debris_body = app.world().get::<GpuDebrisBody>(entity).unwrap();
    assert!(debris_body.angular_velocity.length() < 1.0, "expected a settled (bounded angular velocity) rest state, got {:?}", debris_body.angular_velocity);
    assert!(debris_body.linear_velocity.length() < 1.0, "expected a settled (bounded linear velocity) rest state, got {:?}", debris_body.linear_velocity);
    let transform = app.world().get::<Transform>(entity).unwrap();
    assert!(transform.translation.is_finite(), "expected a finite resting position, got {:?}", transform.translation);
}

/// Reproduction of a real large-dt-spike scenario close to
/// `physics_stability.rs`'s own soak scale: many dynamic debris bodies
/// plus a static planet, with a deliberately injected large single-frame
/// dt (simulating a real pipeline-compile stall's one-time large
/// `Time::delta_secs()`) right after the wait-for-pipelines-ready loop.
/// Originally written alongside a kinematic platform (removed here along
/// with every other kinematic-specific scenario, see this file's own doc
/// comment) — kept because the large-dt-spike-with-many-bodies condition
/// is a real regression risk independent of kinematics.
#[test]
fn many_dynamic_bodies_survive_a_large_dt_spike() {
    let mut app = build_physics_gpu_test_app();
    app.insert_resource(PhysicsGravity { center: Vec3::ZERO, magnitude: 9.81 });
    app.insert_resource(PhysicsGpuEnabled(true));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0 / 60.0)));

    let planet_radius = 8.0;
    app.world_mut().spawn((PhysicsShape::Sphere { radius: planet_radius }, Transform::IDENTITY));

    // 11 falling dynamic spheres, spread around the planet.
    let mut last_entity = None;
    for i in 0..11 {
        let angle = i as f32 * 0.3;
        let position = Vec3::new(angle.cos(), 0.0, angle.sin()) * (planet_radius + 3.0);
        let entity = app
            .world_mut()
            .spawn((
                PhysicsShape::Sphere { radius: 0.5 },
                Transform::from_translation(position),
                GpuDebrisBody { linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: Vec3::splat(1.0 / (0.4 * 0.5 * 0.5)) },
            ))
            .id();
        last_entity = Some(entity);
    }
    let dynamic_entity = last_entity.unwrap();

    assert!(wait_for_all_physics_gpu_pipelines_ready(&mut app), "not every physics GPU pipeline reached Ok within the wait budget -- check for a shader compile error");

    // Inject one deliberately large dt frame -- simulating a real
    // pipeline-compile stall's one-time large Time::delta_secs() spike.
    app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(0.25)));
    app.update();
    app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0 / 60.0)));

    for _ in 0..90 {
        app.update();
        let transform = app.world().get::<Transform>(dynamic_entity).unwrap();
        assert!(transform.translation.is_finite(), "expected a finite position, got {:?}", transform.translation);
        assert!(transform.translation.length() < planet_radius * 5.0, "expected debris to stay near the planet, not launch away, got {:?} at frame {}", transform.translation, app.world().resource::<bevy::diagnostic::FrameCount>().0);
    }
}
