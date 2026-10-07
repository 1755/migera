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
/// length on each character's clock; or a crouch (`--gait crouch`, `--speed`
/// its depth 0-1, `--on-toes`), going down and up through it on each
/// character's clock, posed from its feet every frame as the walker poses it;
/// or a sneak's walk (`--gait sneak`, `--speed`, `--crouch` 0-1,
/// `--on-toes`), the crouch it walks from posed every frame too.
#[derive(Clone, Copy)]
enum Gait {
    None,
    Walk(f32),
    Sneak(f32, f32, bool),
    Run(f32),
    Jump(f32, f32),
    Crouch(f32, bool),
    /// Up a standard ladder, or (`true`) sliding down it from the top.
    Climb(bool),
    /// Grabbing a 2.15 m ledge from a standing jump and hanging: braced
    /// against its wall, or (`false`) free from a slab.
    Hang(bool),
    /// Climbing up onto a 2.15 m wall from a braced hang.
    HangUp,
    /// Shimmying from a braced hang along a wall, or (`true`) on round a
    /// block's corner.
    Shimmy(bool),
    /// Walking off a top at 1.4 m/s and landing: 0.9 m, squatting, or
    /// (`true`) 2.2 m, rolling.
    Drop(bool),
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
        Gait::None | Gait::Jump(..) | Gait::Crouch(..) | Gait::Sneak(..) | Gait::Climb(..) | Gait::Hang(..) | Gait::HangUp | Gait::Shimmy(..) | Gait::Drop(..) => None,
    };
    // A grab (`parkour::hang`): the clock is how far from the jump's start
    // to two seconds hanging; or, climbing up, from the climb's start to
    // standing on the top (3.8 s). Posed led ahead of its springs, as the
    // walker poses it.
    // Shimmying, from hanging braced: along a wall (4 s), or on round a
    // block's corner (6 s).
    let hanging = match gait {
        Gait::Hang(..) | Gait::HangUp | Gait::Shimmy(..) => {
            use migera::character::anim::parkour::{hang::Shimmy, Hanging, Ledge};
            let block = Ledge::block(Vec3::new(-0.6, 0.0, -1.0), Vec3::Z, 1.6, 1.0, 2.15);
            let mut ledge = match gait {
                Gait::Shimmy(true) => block[0],
                _ => Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 3.0, 2.15, 1.0),
            };
            if let Gait::Hang(false) = gait {
                ledge.wall_below = 0.15;
            }
            let square = Hanging::square(&ledge, rig.forward());
            let others: &[Ledge] = if let Gait::Shimmy(_) = gait { &block } else { &[] };
            let spot = Hanging::spot(&ledge, others, Vec3::ZERO, square, &stood, &rig);
            let mut hanging = Hanging::grab(&ledge, spot, square, 0.0, &stood, &rig).expect("a 2.15 m ledge in a standing jump's reach");
            match gait {
                Gait::HangUp => {
                    hanging.advance(3.0);
                    assert!(hanging.climb_up() && hanging.is_climbing_up(), "a braced hang climbs up at once");
                    Some((hanging, 3.8))
                }
                Gait::Shimmy(corner) => {
                    hanging.set_others(others);
                    hanging.advance(3.0);
                    hanging.shimmy(Some(Shimmy::Right));
                    Some((hanging, if corner { 6.0 } else { 4.0 }))
                }
                _ => Some((hanging, 3.5)),
            }
        }
        _ => None,
    };
    // A drop (`parkour::fall`): the clock is how far from leaving the top to
    // standing below; posed led ahead of its springs, as the walker poses it.
    let falling = match gait {
        Gait::Drop(roll) => {
            let velocity = rig.forward() * 1.4;
            let height = if roll { 2.2 } else { 0.9 };
            Some(migera::character::anim::parkour::Falling::off(Vec3::new(0.0, height, 0.0), 0.0, velocity, &stood, 0.0, 0.0, &stood, &rig))
        }
        _ => None,
    };
    // A climb (`ladder`): the clock is how far through climbing a standard
    // ladder (8 s of 10), or through sliding it (grip, slide, landing and
    // stepping back off, 4.5 s). The walker advances its climb a frame and
    // poses it led ahead of its springs each frame; here a copy is advanced
    // to the clock from its start, a little more work.
    let climbing = match gait {
        Gait::Climb(slide) => {
            use migera::character::anim::ladder::{Climb, Climbing, Ladder};
            let ladder = Ladder::standard(Vec3::new(0.0, 0.0, -1.0), Vec3::Z);
            let square = 0.0;
            let spot = Climbing::spot(&ladder, square, &stood, &rig);
            let mut climbing = Climbing::new(&ladder, spot, square, square, 0.0, &stood, &rig);
            if slide {
                climbing.advance(Some(Climb::Up), 12.0);
            }
            Some((climbing, if slide { (Climb::Slide, 4.5) } else { (Climb::Up, 8.0) }))
        }
        _ => None,
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
    // A sneak's walk, as the walker sets it once a frame (the crouch posed
    // each frame is costed by `--gait crouch`); with `--crouch-from C`, half
    // way through changing from crouch C, two walks blended
    // (`sneak::SneakGait`).
    let sneaking = match gait {
        Gait::Sneak(speed, crouch, on_toes) => {
            use migera::character::anim::sneak::{Crouching, Footing, Sneak, SneakGait};
            let footing = Footing::of(&stood, &rig);
            let leg = migera::character::anim::gait::leg_length_of(&rig);
            let mut crouching = Crouching::default();
            if let Some(from) = crouch_from() {
                crouching.ask(Sneak { crouch: from, on_toes }.crouch_on(leg), footing.rise());
                crouching.advance(10.0);
            }
            crouching.ask(Sneak { crouch, on_toes }.crouch_on(leg), footing.rise());
            if crouch_from().is_some() {
                while crouching.gone() < 0.5 {
                    crouching.advance(1.0 / 240.0);
                }
            } else {
                crouching.advance(10.0);
            }
            Some(SneakGait::of(&crouching, &footing, speed, &stood, &rig))
        }
        _ => None,
    };
    let posed = |cycle: f32| match (&jump, params) {
        _ if let Some(falling) = &falling => {
            let mut now = falling.clone();
            now.advance(cycle * falling.ends()[2]);
            Some((now.pose_led(&rig, &springs), None))
        }
        _ if let Some((hanging, seconds)) = &hanging => {
            let mut now = hanging.clone();
            now.advance(cycle * seconds);
            Some((now.pose_led(&rig, &springs), None))
        }
        _ if let Some((climbing, (ask, seconds))) = &climbing => {
            let mut now = climbing.clone();
            now.advance(Some(*ask), cycle * seconds);
            Some((now.pose_led(&rig, &springs), None))
        }
        _ if let Some(sneak) = &sneaking => Some((sneak.pose(cycle, &rig), Some(sneak.target().1))),
        _ if let Gait::Crouch(depth, on_toes) = gait => {
            use migera::character::anim::sneak::{Footing, Sneak};
            let crouch = depth * 0.5 * (1.0 - (std::f32::consts::TAU * cycle).cos());
            let asked = Sneak { crouch, on_toes }.crouch_on(migera::character::anim::gait::leg_length_of(&rig));
            Some((Footing::of(&stood, &rig).pose(asked, &stood, &rig), None))
        }
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
        Gait::Crouch(depth, on_toes) => format!("   crouch {depth}{}", if on_toes { " on the toes" } else { "" }),
        Gait::Sneak(speed, crouch, on_toes) => format!("   sneak {speed} m/s, crouch {crouch}{}", if on_toes { " on the toes" } else { "" }),
        Gait::Climb(slide) => format!("   {} a ladder", if slide { "sliding down" } else { "climbing" }),
        Gait::Hang(braced) => format!("   grabbing a ledge, hanging {}", if braced { "braced" } else { "free" }),
        Gait::HangUp => "   climbing up onto a ledge from a braced hang".to_string(),
        Gait::Shimmy(corner) => format!("   shimmying {}", if corner { "round a corner" } else { "along a ledge" }),
        Gait::Drop(roll) => format!("   walking off a top and {}", if roll { "rolling (2.2 m)" } else { "squatting (0.9 m)" }),
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

/// `--crouch-from C`: a sneak's crouch changing from C, half-way.
fn crouch_from() -> Option<f32> {
    let args: Vec<String> = std::env::args().collect();
    args.iter().position(|a| a == "--crouch-from").and_then(|i| args.get(i + 1)).and_then(|v| v.parse().ok())
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
    let (mut gait, mut speed, mut distance, mut crouch) = (None::<String>, 1.4, 0.0, 1.0);

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
            "--crouch" => crouch = args.next().and_then(|v| v.parse().ok()).unwrap_or(crouch),
            _ => {}
        }
    }

    let gait = match gait.as_deref() {
        Some("walk") => Gait::Walk(speed),
        Some("run") => Gait::Run(speed),
        Some("jump") => Gait::Jump(speed, distance),
        Some("crouch") => Gait::Crouch(speed, std::env::args().any(|a| a == "--on-toes")),
        Some("sneak") => Gait::Sneak(speed, crouch, std::env::args().any(|a| a == "--on-toes")),
        Some("climb") => Gait::Climb(false),
        Some("slide") => Gait::Climb(true),
        Some("hang") => Gait::Hang(true),
        Some("hang-free") => Gait::Hang(false),
        Some("hang-up") => Gait::HangUp,
        Some("shimmy") => Gait::Shimmy(false),
        Some("shimmy-corner") => Gait::Shimmy(true),
        Some("drop") => Gait::Drop(false),
        Some("roll") => Gait::Drop(true),
        _ => Gait::None,
    };
    (characters.max(1), frames.max(1), gait)
}
