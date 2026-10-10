//! Headless CPU benchmark for the `camera` pipeline.
//!
//! Times one camera frame per rig (mode stack, anchor, orbit, rig pose)
//! with no window and no renderer: a windowed example's frame time is
//! vsync-capped and says nothing about camera cost. Each rig follows its
//! own target round a circle with the stick moving and a mode switch every
//! two seconds, so every stage does real work.
//!
//! `--collision` adds collision against avian: a headless physics world of
//! 500 static boxes (walls, pillars and crates strewn over 60 × 60 m), every
//! rig resolved through `AvianProbe` each frame, sweeps, feelers and all.
//!
//! Reports p50/p99 over the frames, since a mean hides spikes.
//!
//! ```text
//! cargo run --release --example camera_bench -- --cameras 64 --frames 2000 [--collision]
//! ```

use std::time::{Duration, Instant};

use avian3d::prelude::*;
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use migera::camera::probe::AvianProbe;
use migera::camera::{
    CameraClock, CameraConfig, CameraFrame, CameraInput, CameraInputSettings, CameraRig, ModeId,
    ModeRequest, TargetSample,
};

const DT: f32 = 1.0 / 60.0;

fn frame_for(f: usize, i: usize) -> CameraFrame {
    let t = f as f32 * DT;
    let phase = t * 0.8 + i as f32;
    let mode = if ((t / 2.0) as usize).is_multiple_of(2) { "explore" } else { "combat" };
    CameraFrame {
        clock: CameraClock::both(DT),
        input: CameraInput { look_stick: Vec2::new((t * 1.3 + i as f32).sin() * 0.6, 0.0), ..Default::default() },
        target: TargetSample {
            position: Vec3::new(phase.cos() * 6.0 + (i % 8) as f32 * 6.0 - 24.0, 0.0, phase.sin() * 6.0 + (i / 8) as f32 * 6.0 - 24.0),
            grounded: true,
            ..Default::default()
        },
        requests: if f.is_multiple_of(120) { vec![ModeRequest { id: ModeId::new(mode), blend: 0.4 }] } else { Vec::new() },
        goals: Vec::new(),
    }
}

fn main() {
    let (cameras, frames, collision) = parse_args();
    let (samples, checksum) = if collision { run_with_collision(cameras, frames) } else { run_plain(cameras, frames) };
    report(cameras, frames, collision, samples, checksum);
}

fn run_plain(cameras: usize, frames: usize) -> (Vec<f64>, f32) {
    let config = CameraConfig::default();
    let settings = CameraInputSettings::default();
    let mut rigs: Vec<CameraRig> = (0..cameras).map(|i| CameraRig::new(&config, i as f32 * 0.1)).collect();
    let mut samples = Vec::with_capacity(frames);
    let mut checksum = 0.0f32;
    for f in 0..frames {
        let frames_in: Vec<CameraFrame> = (0..cameras).map(|i| frame_for(f, i)).collect();
        let start = Instant::now();
        for (rig, frame) in rigs.iter_mut().zip(&frames_in) {
            checksum += rig.step(frame, &settings, &config).pose.eye.x;
        }
        samples.push(start.elapsed().as_secs_f64() * 1000.0);
    }
    (samples, checksum)
}

#[derive(Resource)]
struct Bench {
    rigs: Vec<CameraRig>,
    frame: usize,
    samples: Vec<f64>,
    checksum: f32,
}

fn run_with_collision(cameras: usize, frames: usize) -> (Vec<f64>, f32) {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, bevy::asset::AssetPlugin::default(), bevy::mesh::MeshPlugin, PhysicsPlugins::default(), TransformPlugin))
        .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(DT)));
    app.finish();
    // 500 static boxes, deterministic: walls, pillars and crates.
    let mut rng = fastrand::Rng::with_seed(7);
    app.world_mut().spawn((RigidBody::Static, Collider::cuboid(80.0, 0.2, 80.0), Transform::from_xyz(0.0, -0.1, 0.0)));
    for k in 0..499 {
        let size = match k % 3 {
            0 => Vec3::new(0.4, 3.0, 2.0 + rng.f32() * 6.0),
            1 => Vec3::new(0.6, 4.0, 0.6),
            _ => Vec3::splat(0.5 + rng.f32()),
        };
        let at = Vec3::new(rng.f32() * 60.0 - 30.0, size.y * 0.5, rng.f32() * 60.0 - 30.0);
        app.world_mut().spawn((RigidBody::Static, Collider::cuboid(size.x, size.y, size.z), Transform::from_translation(at).with_rotation(Quat::from_rotation_y(rng.f32() * 3.0))));
    }
    for _ in 0..3 {
        app.update();
    }
    let config = CameraConfig::default();
    app.insert_resource(Bench {
        rigs: (0..cameras).map(|i| CameraRig::new(&config, i as f32 * 0.1)).collect(),
        frame: 0,
        samples: Vec::with_capacity(frames),
        checksum: 0.0,
    });
    for _ in 0..frames {
        app.world_mut()
            .run_system_once(|query: SpatialQuery, mut bench: ResMut<Bench>| {
                let config = CameraConfig::default();
                let settings = CameraInputSettings::default();
                let f = bench.frame;
                let frames_in: Vec<CameraFrame> = (0..bench.rigs.len()).map(|i| frame_for(f, i)).collect();
                let probe = AvianProbe {
                    query: &query,
                    filter: SpatialQueryFilter::from_mask(LayerMask(config.collision.blockers)),
                    ignore: &|_| false,
                };
                let start = Instant::now();
                let mut sum = 0.0;
                for (rig, frame) in bench.rigs.iter_mut().zip(&frames_in) {
                    sum += rig.step_resolved(frame, &settings, &config, &probe, 0.131).1.eye.x;
                }
                let elapsed = start.elapsed().as_secs_f64() * 1000.0;
                bench.samples.push(elapsed);
                bench.checksum += sum;
                bench.frame += 1;
            })
            .unwrap();
    }
    let bench = app.world_mut().remove_resource::<Bench>().unwrap();
    (bench.samples, bench.checksum)
}

fn report(cameras: usize, frames: usize, collision: bool, mut samples: Vec<f64>, checksum: f32) {
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p50 = samples[samples.len() / 2];
    let p99 = samples[(samples.len() * 99 / 100).min(samples.len() - 1)];
    println!(
        "camera_bench{}: {cameras} cameras x {frames} frames: p50 {p50:.4} ms   p99 {p99:.4} ms   \
         per-camera p50 {:.2} µs   (checksum {checksum:.1})",
        if collision { " --collision" } else { "" },
        p50 * 1000.0 / cameras as f64,
    );
}

fn parse_args() -> (usize, usize, bool) {
    let (mut cameras, mut frames, mut collision) = (1, 2000, false);
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--cameras" => cameras = args.next().and_then(|v| v.parse().ok()).unwrap_or(cameras),
            "--frames" => frames = args.next().and_then(|v| v.parse().ok()).unwrap_or(frames),
            "--collision" => collision = true,
            other => panic!("unknown argument {other}; use --cameras N --frames N [--collision]"),
        }
    }
    (cameras, frames, collision)
}
