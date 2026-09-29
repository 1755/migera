//! Headless CPU benchmark for the `character::anim` solve.
//!
//! Measures the animation stack's own cost, deliberately with no window,
//! no renderer and no GPU: `character_gallery`'s frame time is vsync-capped
//! at the display refresh, so it reports ~16.7 ms no matter how cheap or
//! expensive the animation is, and is useless as an animation number.
//!
//! What is timed is one simulated frame per character: advance the gait
//! phase, compose the phase-oscillator layer onto the target, integrate
//! every bone's damped harmonic oscillator, and run forward kinematics.
//! That is the per-character per-frame work `AnimPlugin` schedules.
//!
//! Reports p50/p99 over the sample, since a mean hides exactly the
//! occasional spike worth knowing about.
//!
//! ```text
//! cargo run --release --example anim_bench -- --characters 100 --frames 600
//! ```

use std::time::Instant;

use bevy::math::Vec3;
use migera::character::anim::dho::DhoState;
use migera::character::anim::phase::{GaitPhase, PhaseLayer};
use migera::character::anim::rig::forward_kinematics;
use migera::character::anim::{default_springs, poses};

fn main() {
    let (characters, frames) = parse_args();

    // 1/60 s, the rate `AnimPlugin` is driven at. Fixed rather than
    // measured so the numbers describe the solve, not the host's clock.
    const DT: f32 = 1.0 / 60.0;

    let springs = default_springs();
    // Stood on bent knees, as every character is: the straight bind leg
    // has no knee to take up a sway.
    let base = migera::character::anim::stance::stance(&poses::relaxed_stand());

    // Each character gets its own spring state and its own gait clock,
    // seeded so they do not all march in lockstep — identical phases
    // across every character would be an unrealistically branch-friendly
    // workload.
    let mut states: Vec<(DhoState, GaitPhase)> = (0..characters)
        .map(|index| {
            let phase = GaitPhase {
                gait: index as f32 / characters as f32,
                speed: 1.4,
                ..Default::default()
            };
            (DhoState::settled_on(&base), phase)
        })
        .collect();

    let layer = PhaseLayer::locomotion();

    // Warm up: first-touch page faults and cache population are real but
    // are not what the steady-state number is meant to describe.
    for _ in 0..60 {
        step(&mut states, &layer, &base, &springs, DT);
    }

    let mut samples: Vec<f64> = Vec::with_capacity(frames);
    for _ in 0..frames {
        let started = Instant::now();
        step(&mut states, &layer, &base, &springs, DT);
        samples.push(started.elapsed().as_secs_f64() * 1000.0);
    }

    samples.sort_by(|a, b| a.partial_cmp(b).expect("no NaN timings"));
    let p50 = samples[samples.len() / 2];
    let p99 = samples[(samples.len() * 99 / 100).min(samples.len() - 1)];

    println!(
        "anim_bench: {characters} characters x {frames} frames   \
         p50 {p50:.3} ms   p99 {p99:.3} ms   \
         per-character p50 {:.4} ms",
        p50 / characters as f64,
    );
}

/// One frame of the whole stack for every character.
fn step(
    states: &mut [(DhoState, GaitPhase)],
    layer: &PhaseLayer,
    base: &migera::character::anim::LocalPose,
    springs: &migera::character::anim::rig::BoneSet<
        migera::character::anim::SpringParams,
    >,
    dt: f32,
) {
    for (dho, phase) in states.iter_mut() {
        phase.advance(dt);

        // On a rig, as `AnimPlugin` does once one is bound: the layer's
        // sway over the feet needs it.
        let mut target = *base;
        layer.apply_on(phase, &mut target, &migera::character::anim::rig::RigGeometry::default());

        dho.advance(&target, springs, dt);

        // Forward kinematics stands in for Bevy's own transform
        // propagation, which does the same work on the real rig.
        std::hint::black_box(forward_kinematics(&dho.pose(Vec3::ZERO)));
    }
}

fn parse_args() -> (usize, usize) {
    let mut characters = 100;
    let mut frames = 600;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--characters" => {
                characters = args.next().and_then(|v| v.parse().ok()).unwrap_or(characters);
            }
            "--frames" => {
                frames = args.next().and_then(|v| v.parse().ok()).unwrap_or(frames);
            }
            _ => {}
        }
    }

    (characters.max(1), frames.max(1))
}
