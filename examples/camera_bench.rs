//! Headless CPU benchmark for the `camera` pipeline.
//!
//! Times one camera frame per rig (mode stack, anchor, orbit, rig pose) with
//! no window and no renderer: a windowed example's frame time is
//! vsync-capped and says nothing about camera cost. Each rig follows its
//! own target round a circle with the stick moving and a mode switch every
//! two seconds, so every stage does real work.
//!
//! Reports p50/p99 over the frames, since a mean hides spikes.
//!
//! ```text
//! cargo run --release --example camera_bench -- --cameras 64 --frames 2000
//! ```

use std::time::Instant;

use bevy::math::{Vec2, Vec3};
use migera::camera::{
    CameraClock, CameraConfig, CameraFrame, CameraInput, CameraInputSettings, CameraRig, ModeId,
    ModeRequest, TargetSample,
};

fn main() {
    let (cameras, frames) = parse_args();
    let config = CameraConfig::default();
    let settings = CameraInputSettings::default();
    let dt = 1.0 / 60.0;
    let mut rigs: Vec<CameraRig> =
        (0..cameras).map(|i| CameraRig::new(&config, i as f32 * 0.1)).collect();

    let mut samples = Vec::with_capacity(frames);
    let mut checksum = 0.0f32;
    for f in 0..frames {
        let t = f as f32 * dt;
        let frames_in: Vec<CameraFrame> = (0..cameras)
            .map(|i| {
                let phase = t * 0.8 + i as f32;
                let mode = if (t / 2.0) as usize % 2 == 0 { "explore" } else { "combat" };
                CameraFrame {
                    clock: CameraClock::both(dt),
                    input: CameraInput {
                        look_stick: Vec2::new((t * 1.3 + i as f32).sin() * 0.6, 0.0),
                        ..Default::default()
                    },
                    target: TargetSample {
                        position: Vec3::new(phase.cos() * 6.0, 0.0, phase.sin() * 6.0),
                        grounded: true,
                        facing_yaw: None,
                        rebase: false,
                    },
                    requests: if f % 120 == 0 {
                        vec![ModeRequest { id: ModeId::new(mode), blend: 0.4 }]
                    } else {
                        Vec::new()
                    },
                    goals: Vec::new(),
                }
            })
            .collect();

        let start = Instant::now();
        for (rig, frame) in rigs.iter_mut().zip(&frames_in) {
            checksum += rig.step(frame, &settings, &config).pose.eye.x;
        }
        samples.push(start.elapsed().as_secs_f64() * 1000.0);
    }

    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p50 = samples[samples.len() / 2];
    let p99 = samples[(samples.len() * 99 / 100).min(samples.len() - 1)];
    println!(
        "camera_bench: {cameras} cameras x {frames} frames: p50 {p50:.4} ms   p99 {p99:.4} ms   \
         per-camera p50 {:.2} µs   (checksum {checksum:.1})",
        p50 * 1000.0 / cameras as f64,
    );
}

fn parse_args() -> (usize, usize) {
    let (mut cameras, mut frames) = (1, 2000);
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--cameras" => cameras = args.next().and_then(|v| v.parse().ok()).unwrap_or(cameras),
            "--frames" => frames = args.next().and_then(|v| v.parse().ok()).unwrap_or(frames),
            other => panic!("unknown argument {other}; use --cameras N --frames N"),
        }
    }
    (cameras, frames)
}
