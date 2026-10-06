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
//!
//! `--gait walk|run --speed V` adds the gait as the walker poses it each
//! frame, on the synthetic rig: the target pose at the gait clock and the root
//! velocity read off its planted foot (`locomotion::root_velocity_of`).

use std::time::Instant;

use bevy::math::Vec3;
use migera::character::anim::dho::DhoState;
use migera::character::anim::gait::{walk_pose_on, GaitParams};
use migera::character::anim::phase::{GaitPhase, PhaseLayer};
use migera::character::anim::rig::forward_kinematics;
use migera::character::anim::{default_springs, poses};

/// The gait posed each frame, if any (`--gait`), and its speed; or a jump
/// (`--gait jump`, `--speed` its height and `--distance` how far forward,
/// metres; `--from-run V [--run-on]` from a run), posed through its whole
/// length on each character's clock.
#[derive(Clone, Copy)]
enum Gait {
    None,
    Walk(f32),
    Run(f32),
    Jump(f32, f32),
}

fn main() {
    let (characters, frames, gait) = parse_args();

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
    // The synthetic rig: the parsed `puppet_base` is a test fixture. The
    // work per pose is the same.
    let rig = migera::character::anim::rig::RigGeometry::default();
    let stood = migera::character::anim::stance::stance_on_rig(&poses::relaxed_stand(), migera::character::anim::stance::DEFAULT_KNEE_FLEX, &rig);
    let params = match gait {
        Gait::Walk(speed) => Some(GaitParams::walking_on(speed, &rig)),
        Gait::Run(speed) => Some(GaitParams::running_on(speed, &rig)),
        Gait::None | Gait::Jump(..) => None,
    };
    let jump = match gait {
        Gait::Jump(height, distance) => {
            use migera::character::anim::jump::{Jump, JumpAsk, RunStart};
            let (run_speed, run_on) = from_run();
            Some(if run_speed > 0.0 {
                let ask = JumpAsk { keep_running: run_on, ..JumpAsk::forward(height, distance) };
                Jump::from_run(ask, RunStart { leg: 0, speed: run_speed }, &stood, &rig)
            } else {
                Jump::plan(JumpAsk::forward(height, distance), &stood, &rig)
            })
        }
        _ => None,
    };
    let posed = |cycle: f32| match (&jump, params) {
        // Led ahead of its springs, as the walker poses it.
        (Some(jump), _) => Some((jump.pose_led(cycle * jump.duration(), &stood, &rig, &springs), None)),
        (None, Some(p)) => Some((walk_pose_on(cycle, &p, &stood, &rig), Some(p))),
        (None, None) => None,
    };

    // Warm up: first-touch page faults and cache population are real but
    // are not what the steady-state number is meant to describe.
    for _ in 0..60 {
        step(&mut states, &layer, &base, &springs, DT, &posed, &rig);
    }

    let mut samples: Vec<f64> = Vec::with_capacity(frames);
    for _ in 0..frames {
        let started = Instant::now();
        step(&mut states, &layer, &base, &springs, DT, &posed, &rig);
        samples.push(started.elapsed().as_secs_f64() * 1000.0);
    }

    samples.sort_by(|a, b| a.partial_cmp(b).expect("no NaN timings"));
    let p50 = samples[samples.len() / 2];
    let p99 = samples[(samples.len() * 99 / 100).min(samples.len() - 1)];

    let gait = match gait {
        Gait::None => String::new(),
        Gait::Walk(speed) => format!("   walk {speed} m/s"),
        Gait::Run(speed) => format!("   run {speed} m/s"),
        Gait::Jump(height, distance) => format!("   jump {height} m up, {distance} m forward"),
    };
    println!(
        "anim_bench: {characters} characters x {frames} frames{gait}   \
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
    posed: &dyn Fn(f32) -> Option<(migera::character::anim::LocalPose, Option<GaitParams>)>,
    rig: &migera::character::anim::rig::RigGeometry,
) {
    for (dho, phase) in states.iter_mut() {
        phase.advance(dt);

        // The gait, as the walker poses it: the target at the clock, and
        // the root velocity read off its planted foot.
        let cycle = migera::character::anim::gait::cycle_of(phase);
        let mut target = match posed(cycle) {
            Some((pose, params)) => {
                if let Some(params) = params {
                    let at = |c: f32| posed(c).map_or(pose, |(p, _)| p);
                    std::hint::black_box(migera::character::anim::locomotion::root_velocity_of(cycle, 1.0, &params, &at, rig));
                }
                pose
            }
            None => *base,
        };

        // On a rig, as `AnimPlugin` does once one is bound: the layer's
        // sway over the feet needs it.
        layer.apply_on(phase, &mut target, &migera::character::anim::rig::RigGeometry::default());

        dho.advance(&target, springs, dt);

        // Forward kinematics stands in for Bevy's own transform
        // propagation, which does the same work on the real rig.
        std::hint::black_box(forward_kinematics(&dho.pose(Vec3::ZERO)));
    }
}

/// `--from-run V`: the jump taken from a run at V m/s (`jump::Jump::from_run`),
/// and with `--run-on`, landing on the other foot to run on.
fn from_run() -> (f32, bool) {
    let args: Vec<String> = std::env::args().collect();
    let speed = args.iter().position(|a| a == "--from-run").and_then(|i| args.get(i + 1)).and_then(|v| v.parse().ok()).unwrap_or(0.0);
    (speed, args.iter().any(|a| a == "--run-on"))
}

fn parse_args() -> (usize, usize, Gait) {
    let mut characters = 100;
    let mut frames = 600;
    let (mut gait, mut speed, mut distance) = (None::<String>, 1.4, 0.0);

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--characters" => {
                characters = args.next().and_then(|v| v.parse().ok()).unwrap_or(characters);
            }
            "--frames" => {
                frames = args.next().and_then(|v| v.parse().ok()).unwrap_or(frames);
            }
            "--gait" => gait = args.next(),
            "--speed" => speed = args.next().and_then(|v| v.parse().ok()).unwrap_or(speed),
            "--distance" => distance = args.next().and_then(|v| v.parse().ok()).unwrap_or(distance),
            _ => {}
        }
    }

    let gait = match gait.as_deref() {
        Some("walk") => Gait::Walk(speed),
        Some("run") => Gait::Run(speed),
        Some("jump") => Gait::Jump(speed, distance),
        _ => Gait::None,
    };
    (characters.max(1), frames.max(1), gait)
}
