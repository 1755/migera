//! CPU/GPU parity test harness — the first render-world-driving test in
//! this codebase. Bevy's render world runs async relative to the main
//! world (extract happens at a frame boundary, the render graph runs on
//! its own schedule), so a plain synchronous "call a function, get a
//! result" test doesn't work here — this builds a headless `App` with a
//! real `RenderPlugin` (real GPU adapter, no window/winit), pumps
//! `app.update()` enough times for `RenderStartup`/pipeline compilation to
//! settle, then forces a synchronous buffer readback via
//! `RenderDevice::poll(PollType::wait_indefinitely())` **for test purposes
//! only** — the production fold-in path (a later piece) must use the
//! non-blocking double-buffered readback described in the plan instead;
//! blocking every frame would serialize CPU/GPU and defeat the point of
//! the GPU port.
//!
//! No existing precedent for any of this existed in this codebase before
//! this test (confirmed via repo-wide search) — every prior `cargo test`
//! case here is pure-CPU. Piece 1 deliberately covers ONLY the predict
//! pass (no contacts, no atomics) so this harness's own correctness isn't
//! entangled with atomics semantics.

use bevy::app::PluginsState;
use bevy::asset::AssetPlugin;
use bevy::image::ImagePlugin;
use bevy::math::{Quat, Vec3};
use bevy::prelude::*;
use bevy::render::render_resource::{BufferDescriptor, BufferUsages, CommandEncoderDescriptor, PipelineCache, PollType};
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::{RenderApp, RenderPlugin};

use super::buffers::{BroadphaseGpuState, ContactGenGpuState, ContactGenInputs, PhysicsGpuState, SamplePointsGpuState, ScanGpuState, clear_accumulators, ensure_broadphase_buffers, ensure_contact_gen_buffers, ensure_physics_buffers, ensure_sample_points_buffers, ensure_scan_buffers, upload_contacts, write_apply_uniform, write_apply_velocity_uniform, write_broadphase_copy_uniform, write_broadphase_scatter_uniform, write_contact_gen_uniform, write_predict_uniform};
use super::pass::{dispatch_physics_apply_position, dispatch_physics_apply_velocity, dispatch_physics_broadphase, dispatch_physics_contacts, dispatch_physics_predict, dispatch_physics_sample_points, dispatch_physics_scan, dispatch_physics_scatter_position, dispatch_physics_scatter_velocity};
use super::pipelines::{BroadphaseHashPipeline, BroadphaseScatterPipeline, ContactGenPipeline, PhysicsGpuPipeline, SamplePointsGpuPipeline, ScanGpuPipeline, init_broadphase_hash_pipeline, init_broadphase_scatter_pipeline, init_contact_gen_pipeline, init_physics_gpu_pipeline, init_sample_points_gpu_pipeline, init_scan_gpu_pipeline};
use super::types::{ContactGpu, PhysicsAccumulatorGpu, PhysicsBodyGpu, PhysicsPredictUniform, PhysicsShapeGpu, SamplePointsGpu, from_fixed_point};
use super::super::broadphase::SpatialHash;
use super::super::contacts::{BodySnapshot, Contact, generate_contacts};
use super::super::components::PhysicsShape;
use super::super::solve_static::PhysicsGravity;
use super::super::solve_rigid::{self, BodyState, SubstepStart};
use super::super::sample_points::sample_points_local;

/// Builds a headless `App` with a real `RenderPlugin` (no window) and
/// drives it through plugin-add, finish/cleanup, and enough `update()`
/// calls for `RenderStartup` (pipeline registration) to run and for
/// `PipelineCache` to finish compiling the predict pipeline. Returns the
/// app ready for a test to reach into its `RenderApp` sub-app directly.
fn build_headless_render_app() -> App {
    // Mirrors DefaultPlugins' own ordering up through the plugins
    // RenderPlugin's `finish()` unconditionally assumes exist
    // (`ImagePlugin` for the default sampler, `WindowPlugin` for the
    // WindowResized message type, `MeshPlugin`/`CameraPlugin` for their
    // own extract systems) -- discovered empirically by running this test
    // and adding whichever plugin the next panic named, then confirmed
    // against bevy_internal's own DefaultPlugins ordering
    // (bevy_internal-0.19.1/src/default_plugins.rs) to make sure this
    // isn't a fragile just-enough-to-pass set. No WinitPlugin, no
    // WorldSerialization/Scene/CorePipeline/Sprite/Text/Ui/Gltf/Pbr --
    // none of those are needed for a compute-only dispatch test.
    let mut app = App::new();
    app.add_plugins(bevy::app::TaskPoolPlugin::default());
    app.add_plugins(bevy::diagnostic::FrameCountPlugin);
    app.add_plugins(bevy::time::TimePlugin);
    app.add_plugins(bevy::transform::TransformPlugin);
    // No actual OS window (no WinitPlugin, and no primary window entity
    // spawned) -- this only registers the message types/resources (e.g.
    // WindowResized) that bevy_render's own camera_system unconditionally
    // expects to exist.
    app.add_plugins(bevy::window::WindowPlugin { primary_window: None, ..default() });
    app.add_plugins(AssetPlugin::default());
    app.add_plugins(RenderPlugin::default());
    app.add_plugins(ImagePlugin::default());
    app.add_plugins(bevy::mesh::MeshPlugin);
    app.add_plugins(bevy::camera::CameraPlugin);

    // No ScheduleRunnerPlugin -- this test drives app.update() manually,
    // it doesn't want an event loop.
    while app.plugins_state() == PluginsState::Adding {
        bevy::tasks::tick_global_task_pools_on_main_thread();
    }
    app.finish();
    app.cleanup();

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderPlugin should have created the RenderApp sub-app");
    render_app.add_systems(bevy::render::RenderStartup, (init_physics_gpu_pipeline, init_sample_points_gpu_pipeline, init_scan_gpu_pipeline, init_broadphase_hash_pipeline, init_broadphase_scatter_pipeline, init_contact_gen_pipeline));

    app
}

/// Pumps `app.update()` (bounded) until the given pipeline reaches the
/// `Ok` compiled state. Shader-compile latency isn't a fixed frame count
/// (a real bug this test harness already caught once for the predict
/// pipeline — see this file's own doc comment), so polling for readiness
/// is the correct approach, not guessing an update() count.
fn wait_for_pipeline_ready(app: &mut App, get_pipeline_id: impl Fn(&PhysicsGpuPipeline) -> bevy::render::render_resource::CachedComputePipelineId) -> bool {
    wait_for_pipeline_ready_generic::<PhysicsGpuPipeline>(app, get_pipeline_id)
}

/// Same as `wait_for_pipeline_ready`, generalized over any resource type
/// carrying a `CachedComputePipelineId` -- `SamplePointsGpuPipeline` and
/// any later broad-phase/contact-gen pipeline resource this port adds use
/// this instead of duplicating `wait_for_pipeline_ready`'s own body.
///
/// Checks `CachedPipelineState` directly rather than only
/// `get_compute_pipeline().is_some()`: a real compile error surfaces as
/// `CachedPipelineState::Err` immediately (fail fast with the actual naga/
/// wgpu error text) instead of silently retrying until the iteration
/// budget below is exhausted and reporting a generic "never reached Ok"
/// with no diagnostic content. The iteration budget itself is generous
/// (300, not 60) because it was observed to genuinely flake at 60 once
/// this port's test suite grew to registering six pipelines' worth of
/// concurrent shader compilation per headless app spin-up (each test
/// builds its own fresh `App`, so every test pays full compile latency
/// for every pipeline this plugin registers, not just the one or two it
/// actually exercises) -- this is App startup cost, not a real per-frame
/// budget, so a generous cap costs nothing but wall-clock in the slow
/// path and actually reflects reality in the fast path (an `app.update()`
/// loop exits the instant the target pipeline reaches `Ok`, whichever
/// iteration that happens to be).
///
/// Deliberately does NOT treat `CachedPipelineState::Err` as terminal:
/// Bevy's own `PipelineCache::process_pipeline` (confirmed by reading its
/// source) automatically requeues `ShaderNotLoaded`/
/// `ShaderImportNotYetAvailable` back to `Queued` on the next
/// `process_queue` call -- those are a normal transient step in asset
/// loading (the WGSL file hasn't finished loading off disk yet), not a
/// compile failure, and this codebase saw exactly that: adding this
/// port's 4 new broad-phase pipelines made `Err(ShaderNotLoaded(..))`
/// common enough during the widened compilation window to occasionally
/// still be the state on this function's very last iteration under the
/// old "treat any Err as fatal" logic, which turned a transient loading
/// state into a spurious hard failure. Genuinely terminal shader errors
/// (`ProcessShaderError`, `CreateShaderModule`) still end up surfaced --
/// `Err`'s `Display` text is captured and included in the timeout panic
/// below so a real compile bug is still diagnosable, just not
/// short-circuited on the first transient `Err` seen.
fn wait_for_pipeline_ready_generic<P: Resource>(app: &mut App, get_pipeline_id: impl Fn(&P) -> bevy::render::render_resource::CachedComputePipelineId) -> bool {
    let mut last_err = None;
    for _ in 0..300 {
        app.update();
        let render_app = app.get_sub_app(RenderApp).expect("RenderApp should exist");
        let world = render_app.world();
        if let (Some(pipeline), Some(cache)) = (world.get_resource::<P>(), world.get_resource::<PipelineCache>()) {
            match cache.get_compute_pipeline_state(get_pipeline_id(pipeline)) {
                bevy::render::render_resource::CachedPipelineState::Ok(_) => return true,
                bevy::render::render_resource::CachedPipelineState::Err(err) => {
                    last_err = Some(err.to_string());
                }
                _ => {}
            }
        }
    }
    if let Some(err) = last_err {
        panic!("pipeline never reached Ok within the wait budget -- last seen error: {err}");
    }
    false
}

/// Reads a GPU storage buffer back to CPU: copy into a `MAP_READ` staging
/// buffer, submit, block via `RenderDevice::poll(PollType::wait_indefinitely())`
/// until the map callback fires, then `bytemuck::cast_slice` the mapped
/// range. Test-only — see this module's own doc comment for why the
/// production path must not block like this.
fn read_buffer_sync<T: bytemuck::Pod>(render_device: &RenderDevice, render_queue: &RenderQueue, buffer: &bevy::render::render_resource::Buffer, count: usize) -> Vec<T> {
    let byte_size = (count * std::mem::size_of::<T>()) as u64;
    let device = render_device.wgpu_device();

    let staging = device.create_buffer(&BufferDescriptor {
        label: Some("physics_gpu_test_readback_staging"),
        size: byte_size,
        usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor { label: Some("physics_gpu_test_readback_encoder") });
    encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, byte_size);
    render_queue.submit(std::iter::once(encoder.finish()));

    let slice = staging.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(bevy::render::render_resource::MapMode::Read, move |result| {
        tx.send(result).expect("readback channel should still be open");
    });
    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");
    rx.recv().expect("map_async callback should have fired after a blocking poll").expect("buffer map should succeed");

    let data = slice.get_mapped_range();
    let result: Vec<T> = bytemuck::cast_slice(&data).to_vec();
    drop(data);
    staging.unmap();
    result
}

/// CPU reference: exactly `solve_world.rs`'s own substep-loop prediction
/// step, isolated (no contacts, no solve — just gravity + rotation
/// prediction), so this test's "expected" values come from the real
/// production formula, not a re-derived approximation of it.
fn cpu_predict(position: Vec3, rotation: Quat, linear_velocity: Vec3, angular_velocity: Vec3, inverse_mass: f32, inverse_inertia_local: Vec3, gravity_center: Vec3, gravity_magnitude: f32, substep_dt: f32) -> (Vec3, Quat, Vec3) {
    let mut position = position;
    let mut linear_velocity = linear_velocity;
    let mut rotation = rotation;

    if inverse_mass > 0.0 {
        let to_center = gravity_center - position;
        let g = if to_center.length() < 1e-8 { Vec3::ZERO } else { to_center.normalize() * gravity_magnitude };
        linear_velocity += g * substep_dt;
        position += linear_velocity * substep_dt;
    }

    if inverse_inertia_local != Vec3::ZERO {
        let half_dt_omega = angular_velocity * (0.5 * substep_dt);
        let delta_q = Quat::from_xyzw(half_dt_omega.x, half_dt_omega.y, half_dt_omega.z, 0.0);
        let derivative = delta_q * rotation;
        let predicted = Vec4::from(rotation) + Vec4::from(derivative);
        rotation = Quat::from_vec4(predicted).normalize();
    }

    (position, rotation, linear_velocity)
}

#[test]
fn gpu_predict_matches_cpu_gravity_and_rotation_prediction() {
    let gravity_center = Vec3::new(0.0, -10_000.0, 0.0);
    let gravity_magnitude = 9.81;
    let substep_dt = 1.0 / 480.0; // 8 substeps at 60Hz, matching solve_world's SUBSTEPS.

    // A small, varied fixed scene: one static body (inverse_mass == 0,
    // must not move), one falling body with no rotation capability, one
    // falling+tumbling body -- exercises both prediction gates
    // (inverse_mass, inverse_inertia_local) independently, matching
    // solve_world.rs's own gating logic.
    struct Body {
        position: Vec3,
        rotation: Quat,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
        inverse_mass: f32,
        inverse_inertia_local: Vec3,
    }
    let bodies = vec![
        Body { position: Vec3::new(0.0, -10_000.0, 0.0), rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 0.0, inverse_inertia_local: Vec3::ZERO },
        Body { position: Vec3::new(3.0, 100.0, -2.0), rotation: Quat::IDENTITY, linear_velocity: Vec3::new(0.5, -1.0, 0.0), angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: Vec3::ZERO },
        Body {
            position: Vec3::new(-5.0, 50.0, 1.0),
            rotation: Quat::from_euler(bevy::math::EulerRot::XYZ, 0.2, 0.4, 0.1),
            linear_velocity: Vec3::new(0.0, 2.0, -0.5),
            angular_velocity: Vec3::new(0.3, -0.1, 0.2),
            inverse_mass: 1.0,
            inverse_inertia_local: Vec3::splat(1.0),
        },
    ];

    let gpu_bodies: Vec<PhysicsBodyGpu> = bodies
        .iter()
        .map(|b| PhysicsBodyGpu::from_state(b.position, b.rotation, b.linear_velocity, b.angular_velocity, b.inverse_mass, b.inverse_inertia_local))
        .collect();

    let mut app = build_headless_render_app();
    let pipeline_ready = wait_for_pipeline_ready(&mut app, |p| p.predict_pipeline);
    assert!(pipeline_ready, "physics predict pipeline never reached the Ok state within 60 update() calls -- check for a shader compile error");

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let mut gpu_state = PhysicsGpuState::default();
    ensure_physics_buffers(&render_device, &render_queue, &mut gpu_state, &gpu_bodies);
    write_predict_uniform(
        &render_device,
        &render_queue,
        &mut gpu_state,
        PhysicsPredictUniform { gravity_center_x: gravity_center.x, gravity_center_y: gravity_center.y, gravity_center_z: gravity_center.z, gravity_magnitude, substep_dt, body_count: gpu_bodies.len() as u32, _pad0: 0, _pad1: 0 },
    );

    {
        let pipeline = render_app.world().resource::<PhysicsGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        dispatch_physics_predict(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = gpu_state.0.as_ref().expect("buffers should be allocated after ensure_physics_buffers");
    let gpu_result: Vec<PhysicsBodyGpu> = read_buffer_sync(&render_device, &render_queue, &buffers.bodies, bodies.len());

    for (i, body) in bodies.iter().enumerate() {
        let (expected_position, expected_rotation, expected_linear_velocity) =
            cpu_predict(body.position, body.rotation, body.linear_velocity, body.angular_velocity, body.inverse_mass, body.inverse_inertia_local, gravity_center, gravity_magnitude, substep_dt);

        let gpu = &gpu_result[i];
        let position_diff = (gpu.position() - expected_position).length();
        let velocity_diff = (gpu.linear_velocity() - expected_linear_velocity).length();
        let rotation_dot = gpu.rotation().dot(expected_rotation).abs();

        assert!(position_diff < 1e-4, "body {i}: position mismatch, GPU {:?} vs CPU {:?} (diff {position_diff})", gpu.position(), expected_position);
        assert!(velocity_diff < 1e-4, "body {i}: linear_velocity mismatch, GPU {:?} vs CPU {:?} (diff {velocity_diff})", gpu.linear_velocity(), expected_linear_velocity);
        assert!(rotation_dot > 1.0 - 1e-4, "body {i}: rotation mismatch, GPU {:?} vs CPU {:?} (dot {rotation_dot})", gpu.rotation(), expected_rotation);
    }
}

/// Piece 2's own verification milestone (per the plan): isolate "did the
/// atomic fixed-point scatter scheme work at all" from "does the full
/// apply math match" (that comes in Piece 3) by reading back the raw
/// accumulator buffer directly and comparing against a HAND-COMPUTED
/// expected fixed-point value, not against `solve_rigid`'s own apply
/// logic (which doesn't exist on the GPU side yet).
///
/// Scene: body A (dynamic, zero inverse inertia so the angular
/// contribution to generalized inverse mass is exactly zero regardless of
/// lever arm — keeps the hand-computation to pure linear-inverse-mass
/// splitting, matching `solve_rigid`'s own
/// `a_single_contact_between_two_equal_mass_bodies_splits_the_correction_evenly`
/// test's style) and body B (static, `inverse_mass == 0.0`). A single
/// contact with `depth = 0.4`, normal pointing from B toward A. With B
/// static, `total_w == inv_mass_a` exactly, so `lambda = depth /
/// inv_mass_a = 0.4`, and A's full correction is `impulse * inv_mass_a =
/// normal * 0.4` — hand-computable without touching any solver code.
#[test]
fn gpu_scatter_position_matches_hand_computed_fixed_point_sum() {
    let normal = Vec3::new(-1.0, 0.0, 0.0); // points from B (at +X) toward A (at origin)
    let depth = 0.4;
    let inverse_mass_a = 1.0;

    let gpu_bodies = vec![
        PhysicsBodyGpu::from_state(Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO, inverse_mass_a, Vec3::ZERO),
        PhysicsBodyGpu::from_state(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO, 0.0, Vec3::ZERO),
    ];
    let contacts = vec![ContactGpu::from_contact(super::super::contacts::Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: normal, depth })];

    let mut app = build_headless_render_app();
    let predict_ready = wait_for_pipeline_ready(&mut app, |p| p.predict_pipeline);
    assert!(predict_ready, "predict pipeline never reached Ok -- check for a shader compile error");
    let scatter_ready = wait_for_pipeline_ready(&mut app, |p| p.scatter_position_pipeline);
    assert!(scatter_ready, "scatter_position pipeline never reached Ok -- check for a shader compile error");

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let mut gpu_state = PhysicsGpuState::default();
    ensure_physics_buffers(&render_device, &render_queue, &mut gpu_state, &gpu_bodies);
    clear_accumulators(&render_queue, &gpu_state);
    upload_contacts(&render_device, &render_queue, &mut gpu_state, &contacts, solve_rigid::MAX_LINEAR_CORRECTION, solve_rigid::MAX_ANGULAR_CORRECTION);

    {
        let pipeline = render_app.world().resource::<PhysicsGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        dispatch_physics_scatter_position(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state, contacts.len() as u32);
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
    let accumulators: Vec<PhysicsAccumulatorGpu> = read_buffer_sync(&render_device, &render_queue, &buffers.accumulators, gpu_bodies.len());

    // Hand-computed expected value: lambda = depth / inv_mass_a = 0.4,
    // impulse = normal * lambda, correction_a = impulse * inv_mass_a =
    // normal * 0.4 = (-0.4, 0.0, 0.0).
    let expected_correction_a = normal * (depth / inverse_mass_a) * inverse_mass_a;

    let acc_a = &accumulators[0];
    assert_eq!(acc_a.count, 1, "expected exactly one contact scattered onto body A, got count {}", acc_a.count);
    let got_x = from_fixed_point(acc_a.sum_x);
    let got_y = from_fixed_point(acc_a.sum_y);
    let got_z = from_fixed_point(acc_a.sum_z);
    assert!((got_x - expected_correction_a.x).abs() < 1e-4, "body A sum.x: expected {}, got {got_x} (raw {})", expected_correction_a.x, acc_a.sum_x);
    assert!((got_y - expected_correction_a.y).abs() < 1e-4, "body A sum.y: expected {}, got {got_y} (raw {})", expected_correction_a.y, acc_a.sum_y);
    assert!((got_z - expected_correction_a.z).abs() < 1e-4, "body A sum.z: expected {}, got {got_z} (raw {})", expected_correction_a.z, acc_a.sum_z);
    assert_eq!(acc_a.angular_sum_x, 0, "body A has zero inverse inertia -- angular_sum must be exactly zero");
    assert_eq!(acc_a.angular_sum_y, 0);
    assert_eq!(acc_a.angular_sum_z, 0);

    // Body B is static (inv_mass == 0.0) -- must receive NO scatter at all
    // (solve_rigid's own `if inv_mass_a > 0.0`/`if inv_mass_b > 0.0` gates
    // exclude static bodies from ever being written, matching this
    // shader's own identical gates).
    let acc_b = &accumulators[1];
    assert_eq!(acc_b.count, 0, "a static body must never receive a scatter, got count {}", acc_b.count);
    assert_eq!(acc_b.sum_x, 0);
    assert_eq!(acc_b.sum_y, 0);
    assert_eq!(acc_b.sum_z, 0);
}

/// The case flagged by the plan as most needing GPU-specific
/// verification: many contacts scattering onto the SAME body
/// concurrently, with real GPU concurrency (unlike the CPU reference's
/// serial per-contact loop) — a race in the atomic scatter would show up
/// as a dropped or corrupted contribution, i.e. `count` or `sum` not
/// matching the exact expected total. 64 identical-direction contacts
/// (each against its own static body, so every one of the 64 contacts
/// independently contributes the SAME `depth * inverse_mass` correction
/// to body A) makes the expected total trivial to hand-compute:
/// `64 * single_contact_correction`, exactly.
#[test]
fn many_concurrent_contacts_on_one_body_all_land_in_the_atomic_sum() {
    const CONTACT_COUNT: usize = 64;
    let normal = Vec3::new(-1.0, 0.0, 0.0);
    let depth = 0.1;
    let inverse_mass_a = 1.0;

    let mut gpu_bodies = vec![PhysicsBodyGpu::from_state(Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO, inverse_mass_a, Vec3::ZERO)];
    let mut contacts = Vec::with_capacity(CONTACT_COUNT);
    for i in 0..CONTACT_COUNT {
        let static_body_index = (i + 1) as u32;
        gpu_bodies.push(PhysicsBodyGpu::from_state(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO, 0.0, Vec3::ZERO));
        contacts.push(ContactGpu::from_contact(super::super::contacts::Contact { body_a: 0, body_b: static_body_index, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: normal, depth }));
    }

    let mut app = build_headless_render_app();
    let predict_ready = wait_for_pipeline_ready(&mut app, |p| p.predict_pipeline);
    assert!(predict_ready, "predict pipeline never reached Ok -- check for a shader compile error");
    let scatter_ready = wait_for_pipeline_ready(&mut app, |p| p.scatter_position_pipeline);
    assert!(scatter_ready, "scatter_position pipeline never reached Ok -- check for a shader compile error");

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let mut gpu_state = PhysicsGpuState::default();
    ensure_physics_buffers(&render_device, &render_queue, &mut gpu_state, &gpu_bodies);
    clear_accumulators(&render_queue, &gpu_state);
    upload_contacts(&render_device, &render_queue, &mut gpu_state, &contacts, solve_rigid::MAX_LINEAR_CORRECTION, solve_rigid::MAX_ANGULAR_CORRECTION);

    {
        let pipeline = render_app.world().resource::<PhysicsGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        dispatch_physics_scatter_position(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state, contacts.len() as u32);
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
    let accumulators: Vec<PhysicsAccumulatorGpu> = read_buffer_sync(&render_device, &render_queue, &buffers.accumulators, gpu_bodies.len());

    let single_contact_correction = normal * (depth / inverse_mass_a) * inverse_mass_a;
    let expected_total = single_contact_correction * CONTACT_COUNT as f32;

    let acc_a = &accumulators[0];
    assert_eq!(acc_a.count, CONTACT_COUNT as u32, "expected every one of the {CONTACT_COUNT} contacts to land in body A's count -- a lower count means the atomic scatter dropped a concurrent contribution");
    let got = Vec3::new(from_fixed_point(acc_a.sum_x), from_fixed_point(acc_a.sum_y), from_fixed_point(acc_a.sum_z));
    assert!((got - expected_total).length() < 1e-3, "expected body A's summed correction to be {expected_total:?} (64x a single contact), got {got:?} -- a mismatch here means the atomic scatter corrupted a concurrent write");
}

/// Runs a scene through the GPU predict-free scatter+apply pair (position/
/// rotation round only) and returns the resulting body state, read back
/// to CPU. Mirrors exactly what `solve_rigid::solve_substep_jacobi` does
/// when called directly (no gravity prediction involved) -- this is the
/// Piece 3 parity harness's core: both backends start from the SAME
/// initial `BodyState`/`Contact` set and must produce the same result.
fn run_gpu_scatter_and_apply(bodies: &[BodyState], contacts: &[Contact], substep_start: &[SubstepStart], dt: f32) -> Vec<PhysicsBodyGpu> {
    let gpu_bodies: Vec<PhysicsBodyGpu> = bodies.iter().map(|b| PhysicsBodyGpu::from_state(b.position, b.rotation, b.linear_velocity, b.angular_velocity, b.inverse_mass, b.inverse_inertia_local)).collect();
    let gpu_substep_start: Vec<super::types::SubstepStartGpu> = substep_start
        .iter()
        .map(|s| super::types::SubstepStartGpu {
            position_x: s.position.x,
            position_y: s.position.y,
            position_z: s.position.z,
            _pad_position: 0.0,
            rotation_x: s.rotation.x,
            rotation_y: s.rotation.y,
            rotation_z: s.rotation.z,
            rotation_w: s.rotation.w,
            linear_velocity_x: s.linear_velocity.x,
            linear_velocity_y: s.linear_velocity.y,
            linear_velocity_z: s.linear_velocity.z,
            _pad_linear_velocity: 0.0,
            angular_velocity_x: s.angular_velocity.x,
            angular_velocity_y: s.angular_velocity.y,
            angular_velocity_z: s.angular_velocity.z,
            _pad_angular_velocity: 0.0,
        })
        .collect();
    let gpu_contacts: Vec<ContactGpu> = contacts.iter().map(|c| ContactGpu::from_contact(*c)).collect();

    let mut app = build_headless_render_app();
    let scatter_ready = wait_for_pipeline_ready(&mut app, |p| p.scatter_position_pipeline);
    assert!(scatter_ready, "scatter_position pipeline never reached Ok -- check for a shader compile error");
    let apply_ready = wait_for_pipeline_ready(&mut app, |p| p.apply_position_pipeline);
    assert!(apply_ready, "apply_position pipeline never reached Ok -- check for a shader compile error");

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let mut gpu_state = PhysicsGpuState::default();
    ensure_physics_buffers(&render_device, &render_queue, &mut gpu_state, &gpu_bodies);

    // The predict pass never ran, so substep_start must be written
    // directly -- exactly analogous to the CPU test harness's own
    // `snapshot(&bodies)` being computed independently of any prediction
    // step when `solve_substep_jacobi` is called in isolation.
    {
        let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
        render_queue.write_buffer(&buffers.substep_start, 0, bytemuck::cast_slice(&gpu_substep_start));
    }

    clear_accumulators(&render_queue, &gpu_state);
    upload_contacts(&render_device, &render_queue, &mut gpu_state, &gpu_contacts, solve_rigid::MAX_LINEAR_CORRECTION, solve_rigid::MAX_ANGULAR_CORRECTION);
    write_apply_uniform(&render_device, &render_queue, &mut gpu_state, dt);

    {
        let pipeline = render_app.world().resource::<PhysicsGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        dispatch_physics_scatter_position(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state, gpu_contacts.len() as u32);
        dispatch_physics_apply_position(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
    read_buffer_sync(&render_device, &render_queue, &buffers.bodies, bodies.len())
}

fn assert_body_state_matches(label: &str, gpu: &PhysicsBodyGpu, cpu: &BodyState, tolerance: f32) {
    let position_diff = (gpu.position() - cpu.position).length();
    let velocity_diff = (gpu.linear_velocity() - cpu.linear_velocity).length();
    let angular_velocity_diff = (gpu.angular_velocity() - cpu.angular_velocity).length();
    let rotation_dot = gpu.rotation().dot(cpu.rotation).abs();

    assert!(position_diff < tolerance, "{label}: position mismatch, GPU {:?} vs CPU {:?} (diff {position_diff})", gpu.position(), cpu.position);
    assert!(velocity_diff < tolerance, "{label}: linear_velocity mismatch, GPU {:?} vs CPU {:?} (diff {velocity_diff})", gpu.linear_velocity(), cpu.linear_velocity);
    assert!(angular_velocity_diff < tolerance, "{label}: angular_velocity mismatch, GPU {:?} vs CPU {:?} (diff {angular_velocity_diff})", gpu.angular_velocity(), cpu.angular_velocity);
    assert!(rotation_dot > 1.0 - tolerance, "{label}: rotation mismatch, GPU {:?} vs CPU {:?} (dot {rotation_dot})", gpu.rotation(), cpu.rotation);
}

/// Piece 3's own verification milestone (per the plan): a full parity
/// test against `solve_substep_jacobi` ALONE (not yet the velocity round
/// -- that's Piece 4), on a single-contact scene between one dynamic and
/// one static body.
#[test]
fn gpu_scatter_and_apply_matches_cpu_solve_substep_jacobi_single_contact() {
    let dt = 1.0 / 60.0;
    let cpu_bodies = vec![
        BodyState { position: Vec3::ZERO, rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: Vec3::ZERO },
        BodyState { position: Vec3::new(1.0, 0.0, 0.0), rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 0.0, inverse_inertia_local: Vec3::ZERO },
    ];
    let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: Vec3::NEG_X, depth: 0.4 }];
    let substep_start: Vec<SubstepStart> = cpu_bodies.iter().map(|b| SubstepStart { position: b.position, rotation: b.rotation, linear_velocity: b.linear_velocity, angular_velocity: b.angular_velocity }).collect();

    let mut expected = cpu_bodies.clone();
    solve_rigid::solve_substep_jacobi(&mut expected, &contacts, dt, &substep_start);

    let gpu_result = run_gpu_scatter_and_apply(&cpu_bodies, &contacts, &substep_start, dt);

    for (i, (gpu, cpu)) in gpu_result.iter().zip(expected.iter()).enumerate() {
        assert_body_state_matches(&format!("body {i}"), gpu, cpu, 1e-3);
    }
}

/// The multi-contact case the plan specifically calls out: a single body
/// with TWO simultaneous contacts must have its correction AVERAGED, not
/// summed -- mirrors `solve_rigid`'s own
/// `multiple_contacts_on_one_body_are_averaged_not_summed` test scene
/// exactly, run through the real GPU scatter+apply pair instead.
#[test]
fn gpu_scatter_and_apply_matches_cpu_solve_substep_jacobi_multiple_contacts_on_one_body() {
    let dt = 1.0 / 60.0;
    let cpu_bodies = vec![
        BodyState { position: Vec3::ZERO, rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: Vec3::ZERO },
        BodyState { position: Vec3::new(1.0, 0.0, 0.0), rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 0.0, inverse_inertia_local: Vec3::ZERO },
        BodyState { position: Vec3::new(-1.0, 0.0, 0.0), rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 0.0, inverse_inertia_local: Vec3::ZERO },
    ];
    let contacts = vec![
        Contact { body_a: 0, body_b: 1, point_world: Vec3::ZERO, normal_world: Vec3::NEG_X, depth: 0.2 },
        Contact { body_a: 0, body_b: 2, point_world: Vec3::ZERO, normal_world: Vec3::NEG_X, depth: 0.6 },
    ];
    let substep_start: Vec<SubstepStart> = cpu_bodies.iter().map(|b| SubstepStart { position: b.position, rotation: b.rotation, linear_velocity: b.linear_velocity, angular_velocity: b.angular_velocity }).collect();

    let mut expected = cpu_bodies.clone();
    solve_rigid::solve_substep_jacobi(&mut expected, &contacts, dt, &substep_start);

    let gpu_result = run_gpu_scatter_and_apply(&cpu_bodies, &contacts, &substep_start, dt);

    for (i, (gpu, cpu)) in gpu_result.iter().zip(expected.iter()).enumerate() {
        assert_body_state_matches(&format!("body {i}"), gpu, cpu, 1e-3);
    }
}

/// Confirms rotation is actually exercised end-to-end through the real
/// GPU pipeline, not just linear correction — mirrors
/// `solve_rigid`'s own `an_off_center_contact_induces_rotation_when_inertia_is_finite`
/// test scene (a contact offset from the body's center of mass, nonzero
/// inverse inertia).
#[test]
fn gpu_scatter_and_apply_matches_cpu_solve_substep_jacobi_off_center_contact_induces_rotation() {
    let dt = 1.0 / 60.0;
    let cpu_bodies = vec![
        BodyState { position: Vec3::ZERO, rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: Vec3::splat(1.0) },
        BodyState { position: Vec3::new(1.0, 0.0, 0.0), rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 0.0, inverse_inertia_local: Vec3::ZERO },
    ];
    let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.5, 0.0), normal_world: Vec3::NEG_X, depth: 0.4 }];
    let substep_start: Vec<SubstepStart> = cpu_bodies.iter().map(|b| SubstepStart { position: b.position, rotation: b.rotation, linear_velocity: b.linear_velocity, angular_velocity: b.angular_velocity }).collect();

    let mut expected = cpu_bodies.clone();
    solve_rigid::solve_substep_jacobi(&mut expected, &contacts, dt, &substep_start);
    assert!(expected[0].angular_velocity.length() > 1e-4, "sanity check: the CPU reference itself should show nonzero angular velocity for this scene");

    let gpu_result = run_gpu_scatter_and_apply(&cpu_bodies, &contacts, &substep_start, dt);

    for (i, (gpu, cpu)) in gpu_result.iter().zip(expected.iter()).enumerate() {
        assert_body_state_matches(&format!("body {i}"), gpu, cpu, 1e-3);
    }
}

/// Runs one FULL substep (predict -> scatter-position -> apply-position ->
/// scatter-velocity -> apply-velocity, all 5 GPU dispatches in the correct
/// order) against a given body/contact set, mirroring
/// `solve_rigid::solve_substep_jacobi`'s own internal call to
/// `resolve_contact_velocities` at the end -- this is the Piece 4
/// milestone: the full substep, not just the position round Piece 3
/// already covered.
fn run_gpu_full_substep(bodies: &[PhysicsBodyGpu], gravity: PhysicsGravity, contacts: &[ContactGpu], substep_dt: f32) -> Vec<PhysicsBodyGpu> {
    let mut app = build_headless_render_app();
    for get_id in [
        (|p: &PhysicsGpuPipeline| p.predict_pipeline) as fn(&PhysicsGpuPipeline) -> bevy::render::render_resource::CachedComputePipelineId,
        |p: &PhysicsGpuPipeline| p.scatter_position_pipeline,
        |p: &PhysicsGpuPipeline| p.apply_position_pipeline,
        |p: &PhysicsGpuPipeline| p.scatter_velocity_pipeline,
        |p: &PhysicsGpuPipeline| p.apply_velocity_pipeline,
    ] {
        let ready = wait_for_pipeline_ready(&mut app, get_id);
        assert!(ready, "a physics GPU pipeline never reached Ok -- check for a shader compile error");
    }

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let mut gpu_state = PhysicsGpuState::default();
    ensure_physics_buffers(&render_device, &render_queue, &mut gpu_state, bodies);
    write_predict_uniform(
        &render_device,
        &render_queue,
        &mut gpu_state,
        PhysicsPredictUniform { gravity_center_x: gravity.center.x, gravity_center_y: gravity.center.y, gravity_center_z: gravity.center.z, gravity_magnitude: gravity.magnitude, substep_dt, body_count: bodies.len() as u32, _pad0: 0, _pad1: 0 },
    );
    upload_contacts(&render_device, &render_queue, &mut gpu_state, contacts, solve_rigid::MAX_LINEAR_CORRECTION, solve_rigid::MAX_ANGULAR_CORRECTION);
    write_apply_uniform(&render_device, &render_queue, &mut gpu_state, substep_dt);
    write_apply_velocity_uniform(&render_device, &render_queue, &mut gpu_state);

    {
        let pipeline = render_app.world().resource::<PhysicsGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();

        dispatch_physics_predict(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);

        clear_accumulators(&render_queue, &gpu_state);
        dispatch_physics_scatter_position(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state, contacts.len() as u32);
        dispatch_physics_apply_position(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);

        clear_accumulators(&render_queue, &gpu_state);
        dispatch_physics_scatter_velocity(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state, contacts.len() as u32);
        dispatch_physics_apply_velocity(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
    read_buffer_sync(&render_device, &render_queue, &buffers.bodies, bodies.len())
}

/// CPU reference for one full substep, mirroring `solve_world.rs`'s own
/// per-substep sequence exactly (snapshot -> predict -> generate contacts
/// -> solve_substep_jacobi, which itself calls resolve_contact_velocities
/// internally) -- isolated to a single substep so this test's "expected"
/// side is driven by the same real production code path, not a
/// hand-rolled approximation of it.
fn cpu_full_substep(bodies: &mut [BodyState], shapes: &[PhysicsShape], gravity: &PhysicsGravity, substep_dt: f32) {
    let substep_start: Vec<SubstepStart> = bodies.iter().map(|b| SubstepStart { position: b.position, rotation: b.rotation, linear_velocity: b.linear_velocity, angular_velocity: b.angular_velocity }).collect();

    for body in bodies.iter_mut() {
        if body.inverse_mass > 0.0 {
            let g = gravity.acceleration_at(body.position);
            body.linear_velocity += g * substep_dt;
            body.position += body.linear_velocity * substep_dt;
        }
        if body.inverse_inertia_local != Vec3::ZERO {
            let half_dt_omega = body.angular_velocity * (0.5 * substep_dt);
            let delta_q = Quat::from_xyzw(half_dt_omega.x, half_dt_omega.y, half_dt_omega.z, 0.0);
            let derivative = delta_q * body.rotation;
            let predicted = Vec4::from(body.rotation) + Vec4::from(derivative);
            body.rotation = Quat::from_vec4(predicted).normalize();
        }
    }

    let mut contacts = Vec::new();
    for i in 0..bodies.len() {
        for j in (i + 1)..bodies.len() {
            let snap_a = BodySnapshot { shape: shapes[i], translation: bodies[i].position, rotation: bodies[i].rotation };
            let snap_b = BodySnapshot { shape: shapes[j], translation: bodies[j].position, rotation: bodies[j].rotation };
            contacts.extend(generate_contacts(i as u32, &snap_a, j as u32, &snap_b));
        }
    }

    solve_rigid::solve_substep_jacobi(bodies, &contacts, substep_dt, &substep_start);
}

/// Piece 4's own verification milestone (per the plan): a full parity
/// test across a real `solve_world`-shaped substep -- a sphere falling
/// under gravity onto a static floor, resolved through the complete
/// 5-dispatch GPU pipeline (predict, scatter-position, apply-position,
/// scatter-velocity, apply-velocity) and compared against the same
/// sequence run through the real CPU `solve_substep_jacobi`.
#[test]
fn gpu_full_substep_matches_cpu_solve_world_single_substep() {
    let substep_dt = 1.0 / 480.0; // matches solve_world's SUBSTEPS = 8 at 60Hz
    let gravity = PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 };
    // Two RoundedBoxes -- mirrors contacts.rs's own already-proven
    // `a_box_resting_flat_on_another_box_produces_a_multi_point_manifold`
    // scene shape/penetration magnitude exactly (box A's bottom 0.1 below
    // box B's top), rather than a Sphere (whose sample-point-based
    // contact generation this test doesn't need to separately re-verify
    // -- that's sample_points.rs's/contacts.rs's own job).
    let shapes = [PhysicsShape::RoundedBox { half_extents: Vec3::ONE, corner_radius: 0.0 }, PhysicsShape::RoundedBox { half_extents: Vec3::new(2.0, 1.0, 2.0), corner_radius: 0.0 }];

    let cpu_bodies = vec![
        BodyState { position: Vec3::new(0.0, 1.9, 0.0), rotation: Quat::IDENTITY, linear_velocity: Vec3::new(0.0, -0.5, 0.0), angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: Vec3::ZERO },
        BodyState { position: Vec3::ZERO, rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 0.0, inverse_inertia_local: Vec3::ZERO },
    ];

    let mut expected = cpu_bodies.clone();
    cpu_full_substep(&mut expected, &shapes, &gravity, substep_dt);

    // The GPU side needs the SAME contacts the CPU side generated for
    // this substep -- generate them from the PRE-substep body state via
    // the identical predict math, mirroring cpu_full_substep's own
    // sequence (predict happens once, inside the GPU pipeline's own
    // predict pass; contacts must be generated from the POST-predict
    // state exactly like solve_world.rs does, so this test replicates
    // that same predict-then-generate ordering here on the CPU side
    // purely to produce the contact list to upload).
    let mut post_predict = cpu_bodies.clone();
    for body in post_predict.iter_mut() {
        if body.inverse_mass > 0.0 {
            let g = gravity.acceleration_at(body.position);
            body.linear_velocity += g * substep_dt;
            body.position += body.linear_velocity * substep_dt;
        }
    }
    let snap_a = BodySnapshot { shape: shapes[0], translation: post_predict[0].position, rotation: post_predict[0].rotation };
    let snap_b = BodySnapshot { shape: shapes[1], translation: post_predict[1].position, rotation: post_predict[1].rotation };
    let contacts = generate_contacts(0, &snap_a, 1, &snap_b);
    assert!(!contacts.is_empty(), "sanity check: this scene should produce a real contact for the substep to resolve");

    let gpu_bodies: Vec<PhysicsBodyGpu> = cpu_bodies.iter().map(|b| PhysicsBodyGpu::from_state(b.position, b.rotation, b.linear_velocity, b.angular_velocity, b.inverse_mass, b.inverse_inertia_local)).collect();
    let gpu_contacts: Vec<ContactGpu> = contacts.iter().map(|c| ContactGpu::from_contact(*c)).collect();
    let gpu_result = run_gpu_full_substep(&gpu_bodies, gravity, &gpu_contacts, substep_dt);

    for (i, (gpu, cpu)) in gpu_result.iter().zip(expected.iter()).enumerate() {
        assert_body_state_matches(&format!("body {i}"), gpu, cpu, 1e-3);
    }
}

/// Piece 4's own stated milestone (per the plan): a full parity test
/// across ALL 8 substeps of a real `solve_world`-shaped frame -- not just
/// one substep in isolation. Regenerates contacts every substep (matching
/// `solve_world.rs`'s own loop exactly: predict -> generate contacts ->
/// solve_substep_jacobi, repeated `SUBSTEPS` times) on both the CPU
/// reference and the GPU pipeline, feeding each substep's own output
/// state into the next -- exactly like the real per-frame loop, not a
/// series of independent single-substep snapshots.
#[test]
fn gpu_full_frame_matches_cpu_solve_world_across_all_substeps() {
    const SUBSTEPS: u32 = 8;
    let dt = 1.0 / 60.0;
    let substep_dt = dt / SUBSTEPS as f32;
    let gravity = PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 };
    let shapes = [PhysicsShape::RoundedBox { half_extents: Vec3::ONE, corner_radius: 0.0 }, PhysicsShape::RoundedBox { half_extents: Vec3::new(2.0, 1.0, 2.0), corner_radius: 0.0 }];

    // Starts falling from slightly above the floor (not yet in contact)
    // so the frame exercises BOTH free-flight prediction (early substeps)
    // AND active contact resolution (once it lands) -- a more realistic
    // stress than starting already-penetrating for every substep.
    let mut cpu_bodies = vec![
        BodyState { position: Vec3::new(0.0, 2.1, 0.0), rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: Vec3::ZERO },
        BodyState { position: Vec3::ZERO, rotation: Quat::IDENTITY, linear_velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, inverse_mass: 0.0, inverse_inertia_local: Vec3::ZERO },
    ];

    let mut app = build_headless_render_app();
    for get_id in [
        (|p: &PhysicsGpuPipeline| p.predict_pipeline) as fn(&PhysicsGpuPipeline) -> bevy::render::render_resource::CachedComputePipelineId,
        |p: &PhysicsGpuPipeline| p.scatter_position_pipeline,
        |p: &PhysicsGpuPipeline| p.apply_position_pipeline,
        |p: &PhysicsGpuPipeline| p.scatter_velocity_pipeline,
        |p: &PhysicsGpuPipeline| p.apply_velocity_pipeline,
    ] {
        let ready = wait_for_pipeline_ready(&mut app, get_id);
        assert!(ready, "a physics GPU pipeline never reached Ok -- check for a shader compile error");
    }

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let mut gpu_state = PhysicsGpuState::default();
    let mut gpu_bodies: Vec<PhysicsBodyGpu> = cpu_bodies.iter().map(|b| PhysicsBodyGpu::from_state(b.position, b.rotation, b.linear_velocity, b.angular_velocity, b.inverse_mass, b.inverse_inertia_local)).collect();
    ensure_physics_buffers(&render_device, &render_queue, &mut gpu_state, &gpu_bodies);

    for substep in 0..SUBSTEPS {
        // CPU side: exact solve_world.rs substep sequence.
        cpu_full_substep(&mut cpu_bodies, &shapes, &gravity, substep_dt);

        // GPU side: same predict-then-generate-contacts sequence, driven
        // from the GPU's own current body state (read back each substep,
        // same as the CPU side re-reading `bodies` each iteration) so
        // both sides regenerate contacts from a genuinely evolving state,
        // not a state frozen at frame start.
        let current_gpu_state: Vec<BodyState> = gpu_bodies
            .iter()
            .map(|b| BodyState { position: b.position(), rotation: b.rotation(), linear_velocity: b.linear_velocity(), angular_velocity: b.angular_velocity(), inverse_mass: b.inverse_mass, inverse_inertia_local: Vec3::ZERO })
            .collect();
        let mut gpu_post_predict = current_gpu_state.clone();
        for body in gpu_post_predict.iter_mut() {
            if body.inverse_mass > 0.0 {
                let g = gravity.acceleration_at(body.position);
                body.linear_velocity += g * substep_dt;
                body.position += body.linear_velocity * substep_dt;
            }
        }
        let snap_a = BodySnapshot { shape: shapes[0], translation: gpu_post_predict[0].position, rotation: gpu_post_predict[0].rotation };
        let snap_b = BodySnapshot { shape: shapes[1], translation: gpu_post_predict[1].position, rotation: gpu_post_predict[1].rotation };
        let contacts = generate_contacts(0, &snap_a, 1, &snap_b);
        let gpu_contacts: Vec<ContactGpu> = contacts.iter().map(|c| ContactGpu::from_contact(*c)).collect();

        write_predict_uniform(
            &render_device,
            &render_queue,
            &mut gpu_state,
            PhysicsPredictUniform { gravity_center_x: gravity.center.x, gravity_center_y: gravity.center.y, gravity_center_z: gravity.center.z, gravity_magnitude: gravity.magnitude, substep_dt, body_count: gpu_bodies.len() as u32, _pad0: 0, _pad1: 0 },
        );
        upload_contacts(&render_device, &render_queue, &mut gpu_state, &gpu_contacts, solve_rigid::MAX_LINEAR_CORRECTION, solve_rigid::MAX_ANGULAR_CORRECTION);
        write_apply_uniform(&render_device, &render_queue, &mut gpu_state, substep_dt);
        write_apply_velocity_uniform(&render_device, &render_queue, &mut gpu_state);

        {
            let pipeline = render_app.world().resource::<PhysicsGpuPipeline>();
            let pipeline_cache = render_app.world().resource::<PipelineCache>();

            dispatch_physics_predict(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);

            clear_accumulators(&render_queue, &gpu_state);
            dispatch_physics_scatter_position(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state, gpu_contacts.len() as u32);
            dispatch_physics_apply_position(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);

            clear_accumulators(&render_queue, &gpu_state);
            dispatch_physics_scatter_velocity(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state, gpu_contacts.len() as u32);
            dispatch_physics_apply_velocity(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);
        }

        render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

        let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
        gpu_bodies = read_buffer_sync(&render_device, &render_queue, &buffers.bodies, cpu_bodies.len());

        for (i, (gpu, cpu)) in gpu_bodies.iter().zip(cpu_bodies.iter()).enumerate() {
            assert_body_state_matches(&format!("substep {substep}, body {i}"), gpu, cpu, 1e-2);
        }
    }
}

/// Broad-phase/contact-generation port, Piece 1: verifies the GPU
/// sample-point generation pass against `sample_points::sample_points_local`'s
/// own CPU output, for every `PhysicsShape` variant -- both the exact
/// point-for-point values (same deterministic formulas, no reason for
/// them to differ even by float-rounding beyond the usual GPU/CPU
/// transcendental-function tolerance) and the plan's own stated
/// milestone: every GPU-produced sample point still lies ON its shape's
/// surface (the same invariant `sample_points.rs`'s own CPU tests check,
/// now checked against the GPU pipeline's real output instead of assumed
/// to carry over from the CPU-side proof).
#[test]
fn gpu_sample_points_match_cpu_sample_points_local_for_every_shape() {
    use super::super::sample_points::{MAX_SAMPLE_POINTS, sample_points_local};

    let shapes = [
        PhysicsShape::Sphere { radius: 1.5 },
        PhysicsShape::RoundedBox { half_extents: Vec3::new(1.0, 2.0, 3.0), corner_radius: 0.3 },
        PhysicsShape::RoundedCylinder { radius: 1.0, half_height: 2.0, edge_radius: 0.2 },
        PhysicsShape::Capsule { a: Vec3::new(0.0, -1.0, 0.0), b: Vec3::new(0.0, 1.0, 0.0), radius: 0.5 },
        PhysicsShape::Ellipsoid { radii: Vec3::new(1.0, 2.0, 0.5) },
        PhysicsShape::BoxFrame { half_extents: Vec3::splat(1.0), wall_thickness: 0.1 },
        PhysicsShape::HexPrism { radius: 1.0, half_height: 0.5 },
    ];

    let gpu_shapes: Vec<PhysicsShapeGpu> = shapes.iter().map(|s| PhysicsShapeGpu::from_shape(*s)).collect();

    let mut app = build_headless_render_app();
    let ready = wait_for_pipeline_ready_generic::<SamplePointsGpuPipeline>(&mut app, |p| p.pipeline);
    assert!(ready, "sample_points pipeline never reached Ok -- check for a shader compile error");

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let mut gpu_state = SamplePointsGpuState::default();
    ensure_sample_points_buffers(&render_device, &render_queue, &mut gpu_state, &gpu_shapes);

    {
        let pipeline = render_app.world().resource::<SamplePointsGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        dispatch_physics_sample_points(&render_device, &render_queue, pipeline, pipeline_cache, &gpu_state);
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
    let gpu_results: Vec<SamplePointsGpu> = read_buffer_sync(&render_device, &render_queue, &buffers.outputs, shapes.len());

    for (shape, gpu_result) in shapes.iter().zip(gpu_results.iter()) {
        let cpu_points = sample_points_local(shape);

        assert_eq!(gpu_result.count as usize, cpu_points.len(), "{shape:?}: GPU sample count {} != CPU sample count {}", gpu_result.count, cpu_points.len());
        assert!(gpu_result.count as usize <= MAX_SAMPLE_POINTS, "{shape:?}: GPU count {} exceeds MAX_SAMPLE_POINTS", gpu_result.count);

        for (i, cpu_point) in cpu_points.as_slice().iter().enumerate() {
            let gpu_point = gpu_result.point(i);
            let diff = (gpu_point - *cpu_point).length();
            assert!(diff < 1e-4, "{shape:?} sample {i}: GPU point {gpu_point:?} vs CPU point {cpu_point:?} (diff {diff})");
        }

        // The plan's own stated milestone: every GPU-produced point must
        // still lie ON the shape's surface, checked via the same
        // cpu_ref::local_distance the CPU reference's own tests use --
        // this is a genuine independent check, not just "did the GPU
        // reproduce the CPU's numbers" (a bug shared by both sides
        // wouldn't be caught by the point-for-point comparison above).
        let full_shape: crate::sdf::components::Shape = (*shape).into();
        for point in gpu_result.real_points() {
            let distance = crate::hybrid::cpu_ref::local_distance(&full_shape, point);
            assert!(distance.abs() < 1e-3, "{shape:?}: GPU sample point {point:?} is off-surface by {distance}");
        }
    }
}

/// Trivial CPU exclusive-scan reference: `output[0] = 0`, `output[i] =
/// output[i-1] + input[i-1]` -- the ground truth the GPU Hillis-Steele
/// scan is diffed against. Deliberately NOT the same algorithm shape (no
/// ping-pong, no log-steps) since this test's whole point is proving the
/// GPU scan's OUTPUT is correct against an obviously-correct reference,
/// not that the two implementations share a structure (unlike, say, the
/// solver port's `Accumulator`, which mirrors the GPU's actual
/// accumulate-then-divide shape because that structural correspondence
/// itself was the property under test there).
fn cpu_exclusive_scan(input: &[u32]) -> Vec<u32> {
    let mut output = vec![0u32; input.len()];
    let mut running = 0u32;
    for i in 0..input.len() {
        output[i] = running;
        running += input[i];
    }
    output
}

/// Runs the full GPU Hillis-Steele scan on a fresh headless app + fresh
/// buffers, returning the exclusive-scanned result read back to CPU.
fn run_gpu_scan(input: &[u32]) -> Vec<u32> {
    let mut app = build_headless_render_app();
    let step_ready = wait_for_pipeline_ready_generic::<ScanGpuPipeline>(&mut app, |p| p.step_pipeline);
    assert!(step_ready, "scan step pipeline never reached Ok -- check for a shader compile error");
    let exclusive_ready = wait_for_pipeline_ready_generic::<ScanGpuPipeline>(&mut app, |p| p.to_exclusive_pipeline);
    assert!(exclusive_ready, "scan to-exclusive pipeline never reached Ok -- check for a shader compile error");

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let mut gpu_state = ScanGpuState::default();
    ensure_scan_buffers(&render_device, &render_queue, &mut gpu_state, input);

    let a_is_current = {
        let pipeline = render_app.world().resource::<ScanGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        dispatch_physics_scan(&render_device, &render_queue, pipeline, pipeline_cache, &mut gpu_state)
    };

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
    let result_buffer = if a_is_current { &buffers.buffer_a } else { &buffers.buffer_b };
    read_buffer_sync(&render_device, &render_queue, result_buffer, input.len())
}

/// Piece 2's own verification milestone (per the plan): the GPU
/// Hillis-Steele scan proven standalone against a trivial CPU exclusive-
/// scan reference, isolated from hashing/scattering entirely, INCLUDING
/// deliberately awkward non-power-of-two lengths (since `table_size =
/// max(n*2, 256)` is not guaranteed to be a clean power of two).
#[test]
fn gpu_scan_matches_cpu_exclusive_scan_reference() {
    let test_sizes = [1usize, 2, 3, 7, 16, 17, 63, 64, 100, 255, 256, 257, 1000];
    for &size in &test_sizes {
        // Deterministic pseudo-random-ish input, varied per size so this
        // isn't just testing "all ones" or "all the same value" (which
        // would pass even with a subtly wrong scan that happens to
        // preserve sums of identical elements).
        let input: Vec<u32> = (0..size).map(|i| ((i as u32).wrapping_mul(2654435761) >> 24) % 17).collect();
        let expected = cpu_exclusive_scan(&input);
        let gpu_result = run_gpu_scan(&input);
        assert_eq!(gpu_result, expected, "size {size}: GPU scan result diverges from CPU reference. input={input:?}");
    }
}

/// Confirms the scan works at the actual target scale this port needs it
/// for: `table_size = max(body_count * 2, 256)` reaches ~40,000 at 20,000
/// bodies (the largest body count the GPU-vs-CPU sweep tested) -- a
/// dedicated test at that scale, not just the small sizes above, since a
/// bug specific to `ceil(log2(n))` step-count computation or ping-pong
/// bookkeeping at a large, non-power-of-two count could plausibly hide
/// at small sizes and only surface here.
#[test]
fn gpu_scan_matches_cpu_reference_at_the_largest_supported_table_size() {
    let size = 40_000usize; // matches table_size at 20,000 bodies (max(n*2, 256))
    let input: Vec<u32> = (0..size).map(|i| ((i as u32).wrapping_mul(2654435761) >> 24) % 5).collect();
    let expected = cpu_exclusive_scan(&input);
    let gpu_result = run_gpu_scan(&input);
    assert_eq!(gpu_result, expected, "GPU scan result diverges from CPU reference at the largest supported table_size ({size})");
}

/// Runs the full GPU broad-phase pipeline (hash -> count -> scan -> copy
/// -> scatter) on a fresh headless app + fresh buffers, returning
/// `(bucket_start, bucket_items, table_size)` read back to CPU --
/// mirroring `SpatialHash`'s own `(bucket_start(), bucket_items(),
/// table_size())` triple exactly, for direct comparison.
fn run_gpu_broadphase(positions: &[Vec3], cell_size: f32) -> (Vec<u32>, Vec<u32>, u32) {
    let mut app = build_headless_render_app();
    let hash_ready = wait_for_pipeline_ready_generic::<BroadphaseHashPipeline>(&mut app, |p| p.hash_pipeline);
    assert!(hash_ready, "broadphase hash pipeline never reached Ok -- check for a shader compile error");
    let count_ready = wait_for_pipeline_ready_generic::<BroadphaseHashPipeline>(&mut app, |p| p.count_pipeline);
    assert!(count_ready, "broadphase count pipeline never reached Ok -- check for a shader compile error");
    let scan_step_ready = wait_for_pipeline_ready_generic::<ScanGpuPipeline>(&mut app, |p| p.step_pipeline);
    assert!(scan_step_ready, "scan step pipeline never reached Ok -- check for a shader compile error");
    let scan_exclusive_ready = wait_for_pipeline_ready_generic::<ScanGpuPipeline>(&mut app, |p| p.to_exclusive_pipeline);
    assert!(scan_exclusive_ready, "scan to-exclusive pipeline never reached Ok -- check for a shader compile error");
    let copy_ready = wait_for_pipeline_ready_generic::<BroadphaseScatterPipeline>(&mut app, |p| p.copy_pipeline);
    assert!(copy_ready, "broadphase copy pipeline never reached Ok -- check for a shader compile error");
    let scatter_ready = wait_for_pipeline_ready_generic::<BroadphaseScatterPipeline>(&mut app, |p| p.scatter_pipeline);
    assert!(scatter_ready, "broadphase scatter pipeline never reached Ok -- check for a shader compile error");

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let gpu_positions: Vec<[f32; 4]> = positions.iter().map(|p| [p.x, p.y, p.z, 0.0]).collect();
    let mut gpu_state = BroadphaseGpuState::default();
    ensure_broadphase_buffers(&render_device, &render_queue, &mut gpu_state, &gpu_positions, cell_size);
    write_broadphase_copy_uniform(&render_device, &render_queue, &mut gpu_state);
    write_broadphase_scatter_uniform(&render_device, &render_queue, &mut gpu_state);

    let table_size = gpu_state.0.as_ref().expect("buffers should be allocated").table_size;

    {
        let hash_pipeline = render_app.world().resource::<BroadphaseHashPipeline>();
        let scatter_pipeline = render_app.world().resource::<BroadphaseScatterPipeline>();
        let scan_pipeline = render_app.world().resource::<ScanGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        let ok = dispatch_physics_broadphase(&render_device, &render_queue, hash_pipeline, scatter_pipeline, scan_pipeline, pipeline_cache, &mut gpu_state);
        assert!(ok, "dispatch_physics_broadphase returned false -- a pipeline was not ready or buffers were not allocated");
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = gpu_state.0.as_ref().expect("buffers should be allocated");
    let scan_buffers = buffers.scan.0.as_ref().expect("scan buffers should be allocated");
    let bucket_start_buffer = if buffers.scan_result_in_a { &scan_buffers.buffer_a } else { &scan_buffers.buffer_b };
    let bucket_start: Vec<u32> = read_buffer_sync(&render_device, &render_queue, bucket_start_buffer, (table_size + 1) as usize);
    let bucket_items: Vec<u32> = read_buffer_sync(&render_device, &render_queue, &buffers.bucket_items, positions.len());

    (bucket_start, bucket_items, table_size)
}

/// Piece 3's own verification milestone (per the plan): the full GPU
/// broad-phase pipeline compared against `SpatialHash::build`'s own CSR
/// output directly, on a small fixed body-position set.
#[test]
fn gpu_broadphase_matches_cpu_spatial_hash_csr_output() {
    let cell_size = 1.0;
    let positions = vec![
        Vec3::new(0.1, 0.1, 0.1),
        Vec3::new(0.2, 0.2, 0.2),
        Vec3::new(5.0, 5.0, 5.0),
        Vec3::new(0.95, 0.0, 0.0),
        Vec3::new(-3.0, 2.0, 1.0),
    ];

    let cpu_hash = SpatialHash::build(&positions, cell_size);
    let (gpu_bucket_start, gpu_bucket_items, gpu_table_size) = run_gpu_broadphase(&positions, cell_size);

    assert_eq!(gpu_table_size, cpu_hash.table_size(), "table_size mismatch");
    assert_eq!(gpu_bucket_start, cpu_hash.bucket_start(), "bucket_start (CSR offsets) mismatch");
    assert_eq!(gpu_bucket_items, cpu_hash.bucket_items(), "bucket_items (CSR body indices) mismatch");
}

/// Ports `broadphase.rs`'s own
/// `every_true_overlapping_pair_in_a_random_cluster_is_found` test to run
/// against the GPU-built hash instead of the CPU one -- the single most
/// important test in this piece, per that test's own doc comment: a
/// broad-phase that silently misses a genuinely close pair produces a
/// body that falls through the floor with no error at all. Since the raw
/// GPU CSR output already matched the CPU reference exactly in the test
/// above, this test queries candidates directly from the GPU-produced
/// `bucket_start`/`bucket_items` (mirroring `SpatialHash::query_candidates`'s
/// own logic) rather than re-deriving a second query implementation --
/// the CSR-equality test above is what actually proves the GPU query
/// would behave identically; this test proves the CPU reference's OWN
/// candidate-completeness property, confirming Piece 3 didn't just
/// reproduce a hypothetically-already-broken CPU reference.
#[test]
fn gpu_broadphase_finds_every_true_overlapping_pair_in_a_random_cluster() {
    let cell_size = 1.0;
    let mut positions = Vec::new();
    let mut state = 12345u64;
    let mut next = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((state >> 33) as f32 / u32::MAX as f32) * 10.0 - 5.0
    };
    for _ in 0..50 {
        positions.push(Vec3::new(next(), next(), next()));
    }

    let (bucket_start, bucket_items, table_size) = run_gpu_broadphase(&positions, cell_size);

    let cell_coord = |p: Vec3| -> (i32, i32, i32) { ((p.x / cell_size).floor() as i32, (p.y / cell_size).floor() as i32, (p.z / cell_size).floor() as i32) };
    let cell_hash = |cell: (i32, i32, i32)| -> u32 {
        let h = (cell.0.wrapping_mul(92_837_111)) ^ (cell.1.wrapping_mul(689_287_499)) ^ (cell.2.wrapping_mul(283_923_481));
        (h as u32) % table_size
    };
    let query_candidates = |body_index: usize| -> Vec<u32> {
        let cell = cell_coord(positions[body_index]);
        let mut candidates = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let neighbor = (cell.0 + dx, cell.1 + dy, cell.2 + dz);
                    let h = cell_hash(neighbor) as usize;
                    let range = bucket_start[h] as usize..bucket_start[h + 1] as usize;
                    for &candidate in &bucket_items[range] {
                        if candidate as usize != body_index {
                            candidates.push(candidate);
                        }
                    }
                }
            }
        }
        candidates
    };

    for i in 0..positions.len() {
        let candidates = query_candidates(i);
        for j in 0..positions.len() {
            if i == j {
                continue;
            }
            if (positions[i] - positions[j]).length() < cell_size {
                assert!(candidates.contains(&(j as u32)), "body {i} at {:?} and body {j} at {:?} are within cell_size but body {j} was not a candidate (GPU broad-phase)", positions[i], positions[j]);
            }
        }
    }
}

/// A snapshot's `(shape, translation, rotation)` bundled with whether it's
/// dynamic — dynamics must come first in the body buffer
/// (`0..dynamic_count`), statics after, matching
/// `solve_world::generate_all_contacts`'s own indexing convention exactly.
/// This helper type only exists to let `run_gpu_contacts`'s own caller
/// write scenes in natural (shape, pose) terms without hand-computing
/// `PhysicsBodyGpu`/`PhysicsShapeGpu` encodings inline at every test site.
struct GpuContactBody {
    shape: PhysicsShape,
    translation: Vec3,
    rotation: Quat,
}

fn gpu_contact_body(shape: PhysicsShape, translation: Vec3) -> GpuContactBody {
    GpuContactBody { shape, translation, rotation: Quat::IDENTITY }
}

/// Runs the full GPU contact-generation pipeline (broad-phase, then both
/// contact-gen passes sharing its CSR output) on a fresh headless app,
/// returning every valid contact read back to CPU. `dynamics` become
/// bodies `0..dynamic_count`, `statics` become `dynamic_count..`, mirroring
/// `generate_all_contacts`'s own indexing. Sample points are computed via
/// the already-independently-verified CPU `sample_points_local` (Piece 1's
/// own milestone) and uploaded directly rather than re-running the GPU
/// sample-point generation pass here — this test's own job is verifying
/// contact generation itself, not re-verifying sample-point generation a
/// second time.
fn run_gpu_contacts(dynamics: &[GpuContactBody], statics: &[GpuContactBody], contact_capacity: u32) -> Vec<Contact> {
    let mut app = build_headless_render_app();
    let hash_ready = wait_for_pipeline_ready_generic::<BroadphaseHashPipeline>(&mut app, |p| p.hash_pipeline);
    assert!(hash_ready, "broadphase hash pipeline never reached Ok -- check for a shader compile error");
    let count_ready = wait_for_pipeline_ready_generic::<BroadphaseHashPipeline>(&mut app, |p| p.count_pipeline);
    assert!(count_ready, "broadphase count pipeline never reached Ok -- check for a shader compile error");
    let scan_step_ready = wait_for_pipeline_ready_generic::<ScanGpuPipeline>(&mut app, |p| p.step_pipeline);
    assert!(scan_step_ready, "scan step pipeline never reached Ok -- check for a shader compile error");
    let scan_exclusive_ready = wait_for_pipeline_ready_generic::<ScanGpuPipeline>(&mut app, |p| p.to_exclusive_pipeline);
    assert!(scan_exclusive_ready, "scan to-exclusive pipeline never reached Ok -- check for a shader compile error");
    let copy_ready = wait_for_pipeline_ready_generic::<BroadphaseScatterPipeline>(&mut app, |p| p.copy_pipeline);
    assert!(copy_ready, "broadphase copy pipeline never reached Ok -- check for a shader compile error");
    let scatter_ready = wait_for_pipeline_ready_generic::<BroadphaseScatterPipeline>(&mut app, |p| p.scatter_pipeline);
    assert!(scatter_ready, "broadphase scatter pipeline never reached Ok -- check for a shader compile error");
    let dynamic_ready = wait_for_pipeline_ready_generic::<ContactGenPipeline>(&mut app, |p| p.dynamic_pipeline);
    assert!(dynamic_ready, "contacts dynamic pipeline never reached Ok -- check for a shader compile error");
    let static_ready = wait_for_pipeline_ready_generic::<ContactGenPipeline>(&mut app, |p| p.static_pipeline);
    assert!(static_ready, "contacts static pipeline never reached Ok -- check for a shader compile error");

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let dynamic_count = dynamics.len() as u32;
    let all_bodies: Vec<&GpuContactBody> = dynamics.iter().chain(statics.iter()).collect();

    let gpu_bodies: Vec<PhysicsBodyGpu> = all_bodies.iter().map(|b| PhysicsBodyGpu::from_state(b.translation, b.rotation, Vec3::ZERO, Vec3::ZERO, 0.0, Vec3::ZERO)).collect();
    let gpu_shapes: Vec<PhysicsShapeGpu> = all_bodies.iter().map(|b| PhysicsShapeGpu::from_shape(b.shape)).collect();
    let gpu_sample_points: Vec<SamplePointsGpu> = all_bodies
        .iter()
        .map(|b| {
            let points = sample_points_local(&b.shape);
            let mut gpu_points = [[0.0f32; 4]; 32];
            for (i, p) in points.as_slice().iter().enumerate() {
                gpu_points[i] = [p.x, p.y, p.z, 0.0];
            }
            SamplePointsGpu { points: gpu_points, count: points.len() as u32, _pad0: 0, _pad1: 0, _pad2: 0 }
        })
        .collect();

    // Cell size ~2x the largest DYNAMIC body's own bounding radius --
    // same sizing convention generate_all_contacts itself uses (only
    // dynamics feed the spatial hash, statics never do).
    let max_radius = dynamics.iter().map(|b| super::super::solve_static::bounding_radius(&b.shape)).fold(0.0f32, f32::max).max(0.01);
    let cell_size = max_radius * 2.0;

    let mut broadphase_state = BroadphaseGpuState::default();
    if dynamic_count > 0 {
        let dynamic_positions: Vec<[f32; 4]> = dynamics.iter().map(|b| [b.translation.x, b.translation.y, b.translation.z, 0.0]).collect();
        ensure_broadphase_buffers(&render_device, &render_queue, &mut broadphase_state, &dynamic_positions, cell_size);
        write_broadphase_copy_uniform(&render_device, &render_queue, &mut broadphase_state);
        write_broadphase_scatter_uniform(&render_device, &render_queue, &mut broadphase_state);
    }

    let mut contact_gen_state = ContactGenGpuState::default();
    ensure_contact_gen_buffers(&render_device, &render_queue, &mut contact_gen_state, ContactGenInputs { bodies: &gpu_bodies, shapes: &gpu_shapes, sample_points: &gpu_sample_points }, dynamic_count, 0, contact_capacity);
    let table_size = broadphase_state.0.as_ref().map(|b| b.table_size).unwrap_or(256);
    write_contact_gen_uniform(&render_device, &render_queue, &mut contact_gen_state, cell_size, table_size);

    {
        if dynamic_count > 0 {
            let hash_pipeline = render_app.world().resource::<BroadphaseHashPipeline>();
            let scatter_pipeline = render_app.world().resource::<BroadphaseScatterPipeline>();
            let scan_pipeline = render_app.world().resource::<ScanGpuPipeline>();
            let pipeline_cache = render_app.world().resource::<PipelineCache>();
            let ok = dispatch_physics_broadphase(&render_device, &render_queue, hash_pipeline, scatter_pipeline, scan_pipeline, pipeline_cache, &mut broadphase_state);
            assert!(ok, "dispatch_physics_broadphase returned false -- a pipeline was not ready or buffers were not allocated");
        }
        let contact_gen_pipeline = render_app.world().resource::<ContactGenPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        let ok = dispatch_physics_contacts(&render_device, &render_queue, contact_gen_pipeline, pipeline_cache, &contact_gen_state, &broadphase_state);
        assert!(ok, "dispatch_physics_contacts returned false -- a pipeline was not ready or buffers were not allocated");
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = contact_gen_state.0.as_ref().expect("buffers should be allocated");
    let cursor: Vec<u32> = read_buffer_sync(&render_device, &render_queue, &buffers.cursor, 1);
    let contact_count = cursor[0];
    assert!(contact_count <= contact_capacity, "GPU contact generation wrote {contact_count} contacts, exceeding the fixed capacity {contact_capacity} -- silently dropped contacts, size the test's own capacity with more headroom");

    let gpu_contacts: Vec<ContactGpu> = read_buffer_sync(&render_device, &render_queue, &buffers.contacts_out, contact_count as usize);
    gpu_contacts
        .into_iter()
        .map(|c| Contact {
            body_a: c.body_a,
            body_b: c.body_b,
            point_world: Vec3::new(c.point_world_x, c.point_world_y, c.point_world_z),
            normal_world: Vec3::new(c.normal_world_x, c.normal_world_y, c.normal_world_z),
            depth: c.depth,
        })
        .collect()
}

/// Compares two contact sets ignoring order (GPU scattering has no
/// guaranteed ordering, per this piece's own plan section) via each
/// (body_a, body_b) pair's own average depth and average normal --
/// mirroring the existing CPU `contact_generation_is_symmetric_in_depth_regardless_of_argument_order`
/// test's own averaging strategy for the same reason: a finite,
/// non-analytic sample lattice never guarantees identical per-sample-point
/// contact-for-contact correspondence between two independently-run
/// implementations, only that the same set of true contacts is found with
/// the same aggregate depth/normal.
fn assert_contact_sets_match(gpu: &[Contact], cpu: &[Contact], depth_tolerance: f32, normal_tolerance: f32) {
    assert_eq!(gpu.len(), cpu.len(), "GPU produced {} contacts, CPU produced {} -- expected the same count.\nGPU: {gpu:?}\nCPU: {cpu:?}", gpu.len(), cpu.len());

    let average = |contacts: &[Contact]| -> (f32, Vec3) {
        let depth = contacts.iter().map(|c| c.depth).sum::<f32>() / contacts.len() as f32;
        let normal = contacts.iter().map(|c| c.normal_world).fold(Vec3::ZERO, |a, b| a + b) / contacts.len() as f32;
        (depth, normal)
    };

    let (gpu_depth, gpu_normal) = average(gpu);
    let (cpu_depth, cpu_normal) = average(cpu);
    assert!((gpu_depth - cpu_depth).abs() < depth_tolerance, "average depth mismatch: GPU {gpu_depth} vs CPU {cpu_depth}");
    assert!((gpu_normal - cpu_normal).length() < normal_tolerance, "average normal mismatch: GPU {gpu_normal:?} vs CPU {cpu_normal:?}");
}

/// Piece 4's own verification milestone (per the plan): a fixed multi-body
/// scene mirroring `contacts.rs`'s own existing
/// `a_box_resting_flat_on_another_box_produces_a_multi_point_manifold`
/// test, compared against `generate_contacts`'s real CPU output. Both
/// boxes are dynamic (the dynamic-vs-dynamic path, via the GPU broad-phase).
#[test]
fn gpu_contacts_dynamic_vs_dynamic_matches_cpu_box_resting_on_box() {
    let a = gpu_contact_body(PhysicsShape::RoundedBox { half_extents: Vec3::new(1.0, 1.0, 1.0), corner_radius: 0.0 }, Vec3::new(0.0, 1.9, 0.0));
    let b = gpu_contact_body(PhysicsShape::RoundedBox { half_extents: Vec3::new(2.0, 1.0, 2.0), corner_radius: 0.0 }, Vec3::ZERO);

    let snap_a = BodySnapshot { shape: a.shape, translation: a.translation, rotation: a.rotation };
    let snap_b = BodySnapshot { shape: b.shape, translation: b.translation, rotation: b.rotation };
    let cpu_contacts = generate_contacts(0, &snap_a, 1, &snap_b);
    assert!(cpu_contacts.len() >= 4, "test setup sanity check: expected a multi-point manifold from the CPU reference itself");

    let gpu_contacts = run_gpu_contacts(&[a, b], &[], 64);
    assert_contact_sets_match(&gpu_contacts, &cpu_contacts, 0.02, 0.1);
}

/// Piece 4's own verification milestone, dynamic-vs-static path: the small
/// corner poking into a large flat static floor, mirroring `contacts.rs`'s
/// own existing `a_small_corner_poking_into_a_large_flat_face_is_still_caught`
/// test exactly, but with the floor as a genuinely STATIC body (no
/// dynamic-vs-dynamic broad-phase involvement at all -- the floor never
/// enters the spatial hash).
#[test]
fn gpu_contacts_dynamic_vs_static_matches_cpu_small_corner_on_large_floor() {
    let small = gpu_contact_body(PhysicsShape::RoundedBox { half_extents: Vec3::splat(0.1), corner_radius: 0.0 }, Vec3::new(0.0, 0.95, 0.0));
    let floor = gpu_contact_body(PhysicsShape::RoundedBox { half_extents: Vec3::new(20.0, 1.0, 20.0), corner_radius: 0.0 }, Vec3::ZERO);

    let snap_small = BodySnapshot { shape: small.shape, translation: small.translation, rotation: small.rotation };
    let snap_floor = BodySnapshot { shape: floor.shape, translation: floor.translation, rotation: floor.rotation };
    let cpu_contacts = generate_contacts(0, &snap_small, 1, &snap_floor);
    assert!(!cpu_contacts.is_empty(), "test setup sanity check: expected the CPU reference itself to catch this penetration");

    let gpu_contacts = run_gpu_contacts(&[small], &[floor], 64);
    assert_contact_sets_match(&gpu_contacts, &cpu_contacts, 0.02, 0.2);
}

/// A scene combining BOTH paths at once (a dynamic-vs-dynamic pair plus a
/// separate dynamic-vs-static pair, all in one dispatch), confirming the
/// two passes correctly share one contact buffer/cursor without
/// clobbering each other's writes -- the one interaction neither of the
/// two tests above exercises on its own.
#[test]
fn gpu_contacts_combines_dynamic_and_static_paths_in_one_buffer() {
    // Two overlapping dynamic spheres, well clear of the static floor.
    let dyn_a = gpu_contact_body(PhysicsShape::Sphere { radius: 1.0 }, Vec3::new(20.0, 20.0, 20.0));
    let dyn_b = gpu_contact_body(PhysicsShape::Sphere { radius: 1.0 }, Vec3::new(21.5, 20.0, 20.0));
    // A third dynamic body resting on a static floor, far from the pair above.
    let resting = gpu_contact_body(PhysicsShape::Sphere { radius: 1.0 }, Vec3::new(0.0, 0.9, 0.0));
    let floor = gpu_contact_body(PhysicsShape::RoundedBox { half_extents: Vec3::new(20.0, 1.0, 20.0), corner_radius: 0.0 }, Vec3::ZERO);

    let cpu_pair = generate_contacts(0, &BodySnapshot { shape: dyn_a.shape, translation: dyn_a.translation, rotation: dyn_a.rotation }, 1, &BodySnapshot { shape: dyn_b.shape, translation: dyn_b.translation, rotation: dyn_b.rotation });
    let cpu_resting = generate_contacts(2, &BodySnapshot { shape: resting.shape, translation: resting.translation, rotation: resting.rotation }, 3, &BodySnapshot { shape: floor.shape, translation: floor.translation, rotation: floor.rotation });
    assert!(!cpu_pair.is_empty(), "test setup sanity check: expected the CPU reference to catch the dynamic pair's overlap");
    assert!(!cpu_resting.is_empty(), "test setup sanity check: expected the CPU reference to catch the resting-on-floor overlap");

    let gpu_contacts = run_gpu_contacts(&[dyn_a, dyn_b, resting], &[floor], 64);

    let gpu_pair: Vec<Contact> = gpu_contacts.iter().copied().filter(|c| (c.body_a, c.body_b) == (0, 1)).collect();
    let gpu_resting: Vec<Contact> = gpu_contacts.iter().copied().filter(|c| (c.body_a, c.body_b) == (2, 3)).collect();
    assert_eq!(gpu_pair.len() + gpu_resting.len(), gpu_contacts.len(), "found contacts with an unexpected body_a/body_b pairing -- one path may have clobbered the other's slots");

    assert_contact_sets_match(&gpu_pair, &cpu_pair, 0.02, 0.15);
    assert_contact_sets_match(&gpu_resting, &cpu_resting, 0.02, 0.2);
}

/// Confirms the over-capacity path is a loud, detectable condition (a
/// non-zero cursor beyond the buffer's real size) rather than a silent
/// truncation -- per this piece's own plan section ("no silent capacity
/// drops"). A capacity of 1 against a scene that genuinely produces many
/// more contacts than that must report a cursor value larger than the
/// buffer actually holds, which `run_gpu_contacts`'s own assertion above
/// would have caught as a hard test failure if it silently truncated
/// instead -- this test exercises that same assertion path deliberately,
/// confirming it (a) doesn't panic on a benign over-capacity read (this
/// test reads back exactly `contact_capacity` valid contacts) and (b) the
/// cursor's raw value still reports the TRUE total generated, not the
/// clamped count, so a real caller can detect and warn about the overflow.
#[test]
fn gpu_contacts_over_capacity_cursor_reports_the_true_uncapped_count() {
    let a = gpu_contact_body(PhysicsShape::RoundedBox { half_extents: Vec3::new(1.0, 1.0, 1.0), corner_radius: 0.0 }, Vec3::new(0.0, 1.9, 0.0));
    let b = gpu_contact_body(PhysicsShape::RoundedBox { half_extents: Vec3::new(2.0, 1.0, 2.0), corner_radius: 0.0 }, Vec3::ZERO);

    let snap_a = BodySnapshot { shape: a.shape, translation: a.translation, rotation: a.rotation };
    let snap_b = BodySnapshot { shape: b.shape, translation: b.translation, rotation: b.rotation };
    let cpu_contacts = generate_contacts(0, &snap_a, 1, &snap_b);
    let true_count = cpu_contacts.len() as u32;
    assert!(true_count > 1, "test setup sanity check: need more than 1 true contact to exercise the over-capacity path");

    // Deliberately bypass run_gpu_contacts's own "cursor exceeds capacity"
    // assertion (that's the production-facing loud-warning contract, not
    // what THIS test wants to observe) by re-implementing just enough of
    // it inline to read the raw cursor value before that assertion would
    // fire.
    let mut app = build_headless_render_app();
    for ready in [
        wait_for_pipeline_ready_generic::<BroadphaseHashPipeline>(&mut app, |p| p.hash_pipeline),
        wait_for_pipeline_ready_generic::<BroadphaseHashPipeline>(&mut app, |p| p.count_pipeline),
        wait_for_pipeline_ready_generic::<ScanGpuPipeline>(&mut app, |p| p.step_pipeline),
        wait_for_pipeline_ready_generic::<ScanGpuPipeline>(&mut app, |p| p.to_exclusive_pipeline),
        wait_for_pipeline_ready_generic::<BroadphaseScatterPipeline>(&mut app, |p| p.copy_pipeline),
        wait_for_pipeline_ready_generic::<BroadphaseScatterPipeline>(&mut app, |p| p.scatter_pipeline),
        wait_for_pipeline_ready_generic::<ContactGenPipeline>(&mut app, |p| p.dynamic_pipeline),
        wait_for_pipeline_ready_generic::<ContactGenPipeline>(&mut app, |p| p.static_pipeline),
    ] {
        assert!(ready, "a pipeline never reached Ok -- check for a shader compile error");
    }

    let render_app = app.get_sub_app_mut(RenderApp).expect("RenderApp should exist");
    let render_device = render_app.world().resource::<RenderDevice>().clone();
    let render_queue = render_app.world().resource::<RenderQueue>().clone();

    let all_bodies = [&a, &b];
    let gpu_bodies: Vec<PhysicsBodyGpu> = all_bodies.iter().map(|b| PhysicsBodyGpu::from_state(b.translation, b.rotation, Vec3::ZERO, Vec3::ZERO, 0.0, Vec3::ZERO)).collect();
    let gpu_shapes: Vec<PhysicsShapeGpu> = all_bodies.iter().map(|b| PhysicsShapeGpu::from_shape(b.shape)).collect();
    let gpu_sample_points: Vec<SamplePointsGpu> = all_bodies
        .iter()
        .map(|b| {
            let points = sample_points_local(&b.shape);
            let mut gpu_points = [[0.0f32; 4]; 32];
            for (i, p) in points.as_slice().iter().enumerate() {
                gpu_points[i] = [p.x, p.y, p.z, 0.0];
            }
            SamplePointsGpu { points: gpu_points, count: points.len() as u32, _pad0: 0, _pad1: 0, _pad2: 0 }
        })
        .collect();

    let max_radius = super::super::solve_static::bounding_radius(&a.shape).max(super::super::solve_static::bounding_radius(&b.shape)).max(0.01);
    let cell_size = max_radius * 2.0;
    let contact_capacity = 1u32; // deliberately far below true_count

    let mut broadphase_state = BroadphaseGpuState::default();
    let dynamic_positions: Vec<[f32; 4]> = [&a, &b].iter().map(|b| [b.translation.x, b.translation.y, b.translation.z, 0.0]).collect();
    ensure_broadphase_buffers(&render_device, &render_queue, &mut broadphase_state, &dynamic_positions, cell_size);
    write_broadphase_copy_uniform(&render_device, &render_queue, &mut broadphase_state);
    write_broadphase_scatter_uniform(&render_device, &render_queue, &mut broadphase_state);

    let mut contact_gen_state = ContactGenGpuState::default();
    ensure_contact_gen_buffers(&render_device, &render_queue, &mut contact_gen_state, ContactGenInputs { bodies: &gpu_bodies, shapes: &gpu_shapes, sample_points: &gpu_sample_points }, 2, 0, contact_capacity);
    let table_size = broadphase_state.0.as_ref().expect("buffers should be allocated").table_size;
    write_contact_gen_uniform(&render_device, &render_queue, &mut contact_gen_state, cell_size, table_size);

    {
        let hash_pipeline = render_app.world().resource::<BroadphaseHashPipeline>();
        let scatter_pipeline = render_app.world().resource::<BroadphaseScatterPipeline>();
        let scan_pipeline = render_app.world().resource::<ScanGpuPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        let ok = dispatch_physics_broadphase(&render_device, &render_queue, hash_pipeline, scatter_pipeline, scan_pipeline, pipeline_cache, &mut broadphase_state);
        assert!(ok, "dispatch_physics_broadphase returned false");
        let contact_gen_pipeline = render_app.world().resource::<ContactGenPipeline>();
        let pipeline_cache = render_app.world().resource::<PipelineCache>();
        let ok = dispatch_physics_contacts(&render_device, &render_queue, contact_gen_pipeline, pipeline_cache, &contact_gen_state, &broadphase_state);
        assert!(ok, "dispatch_physics_contacts returned false");
    }

    render_device.poll(PollType::wait_indefinitely()).expect("poll should succeed on a real adapter");

    let buffers = contact_gen_state.0.as_ref().expect("buffers should be allocated");
    let cursor: Vec<u32> = read_buffer_sync(&render_device, &render_queue, &buffers.cursor, 1);
    assert_eq!(cursor[0], true_count, "cursor should report the TRUE total contact count generated ({true_count}), not the capped/truncated count, even though only {contact_capacity} were actually written to contacts_out");
}
