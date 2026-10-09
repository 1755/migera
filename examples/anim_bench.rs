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

use bevy::math::{Vec2, Vec3};
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
    /// Lowering itself from standing on a 2.15 m wall's top into a braced
    /// hang from it.
    DropDown,
    /// Letting go of a braced hang from a 3 m wall, falling and landing; or
    /// (`true`) of a free hang from a 3.6 m slab, reaching to catch the
    /// 1.9 m wall under it, the catch tested each frame.
    LetGo(bool),
    /// A standing jump off a 3 m top falling short of a wall as high
    /// across the gap, catching its lip.
    JumpShort,
    /// Leaping from a braced hang on a 2.5 m wall: up from a step in it to
    /// its lip 0.8 m above, aside across a 1 m gap, or back to a 2 m wall
    /// 2.5 m behind; from the launch to the catch, the catch tested each
    /// frame of the flight.
    Leap(migera::character::anim::parkour::hang::Leap),
    /// Mantling onto a 1.1 m wall from standing in front of it.
    Mantle,
    /// Vaulting a 0.9 m wall from a run at 3.5 m/s: a speed vault, or a
    /// lazy one.
    Vault(migera::character::anim::parkour::vault::VaultKind),
    /// Running up a 2.5 m wall at 4 m/s and catching its lip: the run up
    /// and the flight to the catch, the catch tested each frame of it.
    WallRun,
    /// Kicking off a wall met 0.75 rad off square at 4 m/s toward a 2.5 m
    /// lip at right angles to it (a tic-tac) and catching it.
    WallKick,
    /// Running along a wall 0.55 m off at 4 m/s: two steps on its face,
    /// landing and running on.
    RunAlong,
    /// Sliding down a 4.5 m wall from a braced hang, to standing.
    Slide,
    /// Swinging on a 2.3 m bar, pumped up.
    BarSwing,
    /// Climbing a pole: a cycle of the climb up.
    Pole,
    /// Walking on a beam at its pace, balancing.
    Beam,
    /// Sliding under a 0.9 m slab from a 5 m/s run, to standing past it.
    SlideUnder,
    /// Crawling on hands and knees: a cycle of the crawl.
    Crawl,
    /// Squeezing sideways: the side shuffle at its pace, the arms flat; or
    /// (`false`) the side shuffle alone, for comparison.
    Squeeze(bool),
    /// A lache from a pumped swing on a 2.3 m bar to one 2 m ahead: from
    /// letting go to the catch, the catch tested each frame of the flight.
    Lache,
    /// Free climbing up a wall of holds: 3 s of it, limb by limb.
    FreeClimb,
    /// A 4 m/s run turning at 1 rad/s and gathering at 2 m/s², leant with
    /// its acceleration.
    RunLean,
    /// A skid stop from 5 m/s (or, `true`, a plant-and-turn), from the
    /// run's footfall to standing.
    Skid(bool),
    /// A standing jump onto a 0.2 m post 1.2 m ahead: the jump to its top,
    /// then the fall landing on the post.
    Onto,
    /// A 1.2 m/s walk beside a wall, a hand on it: the wall probed for and
    /// the hand rested on it every frame.
    WallHand,
    /// Crouching into a perch and back up, a cycle: the layer blended by
    /// the feet each frame.
    Perch,
    /// A standing jump turning round in the air: the jump to its hand-over,
    /// then the fall turning and landing.
    SpinJump,
    /// A leap of faith from a 6 m top into a pile 2.5 m out, rising out of
    /// it: the whole of it.
    Faith,
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
    // A run up a wall plans only on the real rig (the synthetic rig's legs
    // are shifted a joint): built with `--features real_rig`, it poses on
    // that.
    let rig = match gait {
        #[cfg(feature = "real_rig")]
        Gait::WallRun | Gait::WallKick | Gait::RunAlong | Gait::SlideUnder | Gait::Skid(true) => migera::character::anim::gltf_rig::puppet_base_as_rendered(),
        #[cfg(not(feature = "real_rig"))]
        Gait::WallRun | Gait::WallKick | Gait::RunAlong | Gait::SlideUnder | Gait::Skid(true) => {
            panic!("--gait wall-run, wall-kick, run-along, slide-under and plant-turn need --features real_rig: only the real rig can plan them")
        }
        _ => migera::character::anim::rig::RigGeometry::default(),
    };
    let stood = migera::character::anim::stance::stance_on_rig(&poses::relaxed_stand(), migera::character::anim::stance::DEFAULT_KNEE_FLEX, &rig);
    let params = match gait {
        Gait::Walk(speed) => Some(GaitParams::walking_on(speed, &rig)),
        Gait::Run(speed) => Some(GaitParams::running_on(speed, &rig)),
        Gait::None | Gait::Jump(..) | Gait::Crouch(..) | Gait::Sneak(..) | Gait::Climb(..) | Gait::Hang(..) | Gait::HangUp | Gait::Shimmy(..) | Gait::Drop(..) | Gait::DropDown | Gait::LetGo(..) | Gait::JumpShort | Gait::Leap(..) | Gait::Mantle | Gait::Vault(..) | Gait::WallRun | Gait::WallKick | Gait::RunAlong | Gait::Slide | Gait::BarSwing | Gait::Lache | Gait::Pole | Gait::Beam | Gait::SlideUnder | Gait::Crawl | Gait::Squeeze(..) | Gait::FreeClimb | Gait::RunLean | Gait::Skid(..) | Gait::Onto | Gait::WallHand | Gait::Perch | Gait::SpinJump | Gait::Faith => None,
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
        // Swinging on a 2.3 m bar, pumped up: a swing's period (2.4 s).
        Gait::BarSwing => {
            use migera::character::anim::parkour::{Hanging, Ledge};
            let bar = Ledge::bar(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 3.0, 2.3);
            let mut hanging = Hanging::hung(&bar, &[], Vec3::new(0.2, 0.0, 1.0), 0.0, [None; 2], &stood, &rig);
            hanging.advance(1.0);
            hanging.pump(true);
            for _ in 0..(8.0 / DT) as usize {
                hanging.advance(DT);
            }
            Some((hanging, 2.4))
        }
        // Mantling onto a 1.1 m wall from standing: its whole length (2.9 s).
        Gait::Mantle => {
            use migera::character::anim::parkour::{Hanging, Ledge};
            let ledge = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 3.0, 1.1, 1.0);
            let square = Hanging::square(&ledge, rig.forward());
            let spot = Hanging::mantle_spot(&ledge, &[], Vec3::ZERO, square, &stood, &rig);
            let hanging = Hanging::mantle(&ledge, &[], spot, square, 0.0, [None; 2], None, &stood, &rig).expect("a 1.1 m wall mantled");
            Some((hanging, 2.9))
        }
        // Dropping down: from its start to hanging (5 s).
        Gait::DropDown => {
            use migera::character::anim::parkour::{Hanging, Ledge};
            let ledge = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 3.0, 2.15, 1.0);
            let mut hanging = Hanging::hung(&ledge, &[], Vec3::ZERO, 0.0, [None; 2], &stood, &rig);
            hanging.lower_down(hanging.standing_spot());
            Some((hanging, 5.0))
        }
        _ => None,
    };
    // Letting go: the clock is how far from letting go to standing below,
    // or to the catch.
    let catching = match gait {
        Gait::LetGo(catch) => {
            use migera::character::anim::parkour::{Hanging, Ledge};
            let lower =Ledge::wall(Vec3::new(0.0, 0.0, -1.5), Vec3::Z, 3.0, 1.9, 1.0);
            let ledge = if catch {
                Ledge { wall_below: 0.15, ..Ledge::wall(Vec3::new(0.0, 0.0, -1.15), Vec3::Z, 3.0, 3.6, 1.35) }
            } else {
                Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 3.0, 3.0, 1.0)
            };
            let mut hanging = Hanging::hung(&ledge, &[], Vec3::ZERO, 0.0, [None; 2], &stood, &rig);
            hanging.advance(1.0);
            let mut falling = hanging.let_go(&|_| Some(0.0), &stood, &rig);
            falling.reach(catch);
            let seconds = if catch {
                let (mut probe, mut seconds) = (falling.clone(), 0.0);
                while probe.catches(&[lower], &rig).is_none() {
                    assert!(probe.airborne() || seconds == 0.0, "never caught the 1.9 m wall");
                    probe.advance(DT);
                    seconds += DT;
                }
                seconds
            } else {
                falling.ends()[2]
            };
            Some((falling, seconds, catch.then_some(lower)))
        }
        // Sliding down a 4.5 m wall from hanging braced on it, to standing.
        Gait::Slide => {
            use migera::character::anim::parkour::{Hanging, Ledge};
            let ledge = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 3.0, 4.5, 1.0);
            let mut hanging = Hanging::hung(&ledge, &[], Vec3::ZERO, 0.0, [None; 2], &stood, &rig);
            hanging.advance(1.0);
            let falling = hanging.slide_down(&|_| Some(0.0), &stood, &rig).expect("a slide down a braced hang's wall");
            let seconds = falling.ends()[2];
            Some((falling, seconds, None))
        }
        // A standing jump off a 3 m top falling short of a wall as high
        // 1.7 m off: from leaving the top to the catch.
        Gait::JumpShort => {
            use migera::character::anim::jump::{Jump, JumpAsk};
            use migera::character::anim::parkour::{Falling, Ledge};
            let start = Vec3::new(0.0, 3.0, 0.0);
            let far = Ledge::wall(Vec3::new(0.0, 0.0, 0.0) + rig.forward() * 1.7, -rig.forward(), 3.0, 3.0, 1.0);
            let mut jump = Jump::plan(JumpAsk::forward(0.3, 1.2), &stood, &rig);
            while !jump.airborne() {
                jump.advance(DT);
            }
            let mut falling = Falling::from_jump(&jump, start + rig.forward() * jump.travelled(), 0.0, 0.0, 0.0, &stood, &rig);
            falling.against(&[far], &rig);
            falling.reach(true);
            let (mut probe, mut seconds) = (falling.clone(), 0.0);
            while probe.catches(&[far], &rig).is_none() {
                assert!(probe.airborne() || seconds == 0.0, "never caught the far wall");
                probe.advance(DT);
                seconds += DT;
            }
            Some((falling, seconds, Some(far)))
        }
        _ => None,
    };
    // A leap from a hang (`parkour::hang::Leap`): the clock is how far from
    // the launch to the catch; launching, the hang posed, and once let go,
    // the flight, as the walker poses each.
    let leaping = match gait {
        Gait::Leap(way) => {
            use migera::character::anim::parkour::{hang::Leap, Hanging, Ledge};
            let (from, target) = match way {
                Leap::Up => (Ledge::wall(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 2.0, 2.5, 0.12), Ledge::wall(Vec3::new(0.0, 0.0, -0.62), Vec3::Z, 2.0, 3.3, 1.0)),
                Leap::Aside(_) => (Ledge::wall(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 2.0, 2.5, 1.0), Ledge::wall(Vec3::new(3.0, 0.0, -0.5), Vec3::Z, 2.0, 2.5, 1.0)),
                Leap::Back => (Ledge::wall(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 2.0, 2.5, 1.0), Ledge::wall(Vec3::new(0.0, 0.0, 2.0), Vec3::NEG_Z, 3.0, 2.0, 1.0)),
                Leap::UpAside(_) => (Ledge::wall(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 2.0, 2.5, 1.0), Ledge::wall(Vec3::new(2.3, 0.0, -0.5), Vec3::Z, 2.0, 3.1, 1.0)),
                Leap::BackAside(_) => (Ledge::wall(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 2.0, 2.5, 1.0), Ledge::wall(Vec3::new(2.0, 0.0, 2.5), Vec3::NEG_Z, 2.0, 2.0, 1.0)),
            };
            let mut hanging = Hanging::hung(&from, &[target], Vec3::new(0.2, 0.0, 1.0), 0.0, [None; 2], &stood, &rig);
            hanging.advance(1.5);
            assert!(hanging.leap(way, &rig), "did not leap");
            let (mut probe, mut launch) = (hanging.clone(), 0.0);
            while !probe.is_released() {
                probe.advance(DT);
                launch += DT;
            }
            let falling = probe.release(&|_| Some(0.0), &[from, target], &stood, &rig);
            let (mut probe, mut flight) = (falling.clone(), 0.0);
            while probe.catches(&[target], &rig).is_none() {
                assert!(probe.airborne() || flight == 0.0, "never caught the ledge leapt at");
                probe.advance(DT);
                flight += DT;
            }
            Some((hanging, launch, falling, flight, target))
        }
        // A lache from a pumped swing on a 2.3 m bar to one 2 m ahead: from
        // letting go to the catch.
        Gait::Lache => {
            use migera::character::anim::parkour::{Hanging, Ledge};
            let (from, target) = (Ledge::bar(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 3.0, 2.3), Ledge::bar(Vec3::new(0.0, 0.0, -2.5), Vec3::Z, 3.0, 2.3));
            let mut hanging = Hanging::hung(&from, &[target], Vec3::new(0.2, 0.0, 1.0), 0.0, [None; 2], &stood, &rig);
            hanging.advance(1.0);
            let mut waited = 0.0;
            while !hanging.lache(&rig) {
                assert!(waited < 20.0, "never let go at the bar ahead");
                hanging.advance(DT);
                waited += DT;
            }
            let (mut probe, mut launch) = (hanging.clone(), 0.0);
            while !probe.is_released() {
                probe.advance(DT);
                launch += DT;
            }
            let falling = probe.release(&|_| Some(0.0), &[from, target], &stood, &rig);
            let (mut probe, mut flight) = (falling.clone(), 0.0);
            while probe.catches(&[target], &rig).is_none() {
                assert!(probe.airborne() || flight == 0.0, "never caught the bar ahead");
                probe.advance(DT);
                flight += DT;
            }
            Some((hanging, launch, falling, flight, target))
        }
        _ => None,
    };
    // A drop (`parkour::fall`): the clock is how far from leaving the top to
    // standing below; posed led ahead of its springs, as the walker poses it.
    // A slide under a slab (`parkour::underslide`): the clock is its whole
    // length, posed led ahead of its springs, as the walker poses it.
    let under_slide = match gait {
        Gait::SlideUnder => {
            use migera::character::anim::parkour::{underslide::UnderSlide, Ledge};
            let forward = rig.forward();
            let near = UnderSlide::start_distance(5.0, 0.8).expect("a start");
            let slab = Ledge { wall_below: 0.2, ..Ledge::wall(forward * near, -forward, 3.0, 1.1, 0.8) };
            let from = migera::character::anim::gait::walk_pose_on(0.0, &GaitParams::running_on(5.0, &rig), &stood, &rig);
            Some(UnderSlide::plan(&slab, Vec3::ZERO, 0.0, 5.0, 0, &from, &stood, &rig).expect("a slide under a 0.9 m slab"))
        }
        _ => None,
    };
    // A crawl (`parkour::crawl`): the clock is a second of crawling at its
    // pace, from got down and into it.
    let crawling = match gait {
        Gait::Crawl => {
            let mut crawl = migera::character::anim::parkour::crawl::Crawling::new(Vec3::ZERO, 0.0, &stood, &rig);
            crawl.advance(true, 3.0);
            Some(crawl)
        }
        _ => None,
    };
    // A pole (`parkour::pole`): the clock is a climbing cycle up, from its
    // start.
    let poling = match gait {
        Gait::Pole => {
            use migera::character::anim::parkour::pole::{PoleAsk, Pole, Poling};
            let pole = Pole::new(Vec3::new(0.0, 0.0, -1.0), 6.0);
            let root = Poling::spot(&pole, Vec3::ZERO, &stood, &rig);
            let mut poling = Poling::get_on(&pole, root, &stood, &rig);
            poling.advance(None, 1.5);
            poling.advance(Some(PoleAsk::Up), DT);
            Some(poling)
        }
        _ => None,
    };
    // A jump onto a post (`parkour::precision`): the clock is the jump to
    // its hand-off, then the fall to standing on the post.
    let onto = match gait {
        Gait::SpinJump => {
            use migera::character::anim::parkour::spin;
            let mut jump = spin::spin_jump(&stood, &rig);
            let full = jump.clone();
            while !spin::hands_over(&jump) {
                jump.advance(DT);
            }
            let falling = spin::spin_fall(&jump, rig.forward() * jump.travelled(), 0.0, 0.0, 0.0, std::f32::consts::PI, &stood, &rig);
            Some((full, jump.elapsed(), falling))
        }
        Gait::Onto => {
            use migera::character::anim::parkour::precision;
            let top = rig.forward() * 1.2 + Vec3::Y * 0.2;
            let mut jump = precision::jump_onto(top, Vec3::ZERO, 0.0, &stood, &rig).expect("a post in reach");
            let full = jump.clone();
            while !precision::hands_over(&jump) {
                jump.advance(DT);
            }
            let falling = precision::fall_onto(&jump, top, rig.forward() * jump.travelled(), 0.0, 0.0, &stood, &rig);
            Some((full, jump.elapsed(), falling))
        }
        _ => None,
    };
    let perched = matches!(gait, Gait::Perch).then(|| migera::character::anim::parkour::perch::perch_pose(&stood, &rig));
    let faith = matches!(gait, Gait::Faith).then(|| {
        use migera::character::anim::parkour::faith::{Haystack, LeapOfFaith};
        let hay = Haystack { top: rig.forward() * 2.5 + Vec3::Y, radius: 1.2, height: 1.0 };
        LeapOfFaith::plan(Vec3::Y * 6.0, 0.0, hay, &stood, &rig).expect("a pile in reach")
    });
    // A skid (`parkour::skid`): the clock is its whole length, from the
    // run's footfall.
    let skidding = match gait {
        Gait::Skid(round) => {
            let run = GaitParams::running_on(5.0, &rig);
            let from = walk_pose_on(0.0, &run, &stood, &rig);
            Some(migera::character::anim::parkour::skid::Skid::plan(Vec3::ZERO, 0.0, 5.0, 0, round, &from, &stood, &rig).expect("a skid"))
        }
        _ => None,
    };
    // Free climbing (`parkour::holds`): the clock is 3 s of climbing up a
    // wall of holds, from on it.
    let free_climbing = match gait {
        Gait::FreeClimb => {
            use migera::character::anim::parkour::holds::{FreeClimb, HoldWall};
            let wall = HoldWall::grid(Vec3::new(0.0, 0.0, -0.6), Vec3::Z, 9, 18, 0.4, 0.3, 0.35);
            let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
            climb.advance(None, 1.5);
            Some(climb)
        }
        _ => None,
    };
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
        // A speed vault over a 0.9 m wall 0.3 m deep from a run at 3.5 m/s,
        // from its best take-off: posed through its whole length, as a jump.
        Gait::Vault(kind) => {
            use migera::character::anim::jump::{Jump, RunStart};
            use migera::character::anim::parkour::vault::Obstacle;
            let start = RunStart { leg: 0, speed: 3.5 };
            // A hop over a 0.35 m rail; a vault over a 0.9 m wall.
            if kind == migera::character::anim::parkour::vault::VaultKind::Hop {
                let near = Jump::hop_takeoff(0.3, start, &stood, &rig);
                Some(Jump::vault(Obstacle::square(near, 0.3, 0.35), start, kind, &stood, &rig).expect("a 0.35 m rail hopped"))
            } else {
                let near = Jump::vault_takeoff(0.3, 0.9, start, &stood, &rig);
                Some(Jump::vault(Obstacle::square(near, 0.3, 0.9), start, kind, &stood, &rig).expect("a 0.9 m wall vaulted"))
            }
        }
        Gait::RunAlong => {
            use migera::character::anim::jump::{Jump, RunStart};
            use migera::character::anim::parkour::{along::AlongWall, Ledge};
            // A 3 m wall 0.55 m to the left, along the run.
            let (forward, left) = (rig.forward(), rig.left());
            let wall = Ledge::wall(left * 0.55 + forward * 2.0, -left, 8.0, 3.0, 1.0);
            let start = RunStart { leg: AlongWall::takeoff_leg(&wall, 0.0, &rig), speed: 4.0 };
            Some(Jump::along_wall(&wall, Vec3::ZERO, 0.0, start, &stood, &rig).expect("a run along a wall"))
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
    // A run up a wall: the clock covers the run up and the flight to the
    // catch.
    let wall_running = match gait {
        Gait::WallRun | Gait::WallKick => {
            use migera::character::anim::jump::RunStart;
            use migera::character::anim::parkour::{wall::WallRun, Hanging, Ledge};
            let run = if matches!(gait, Gait::WallRun) {
                let wall = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 4.0, 2.5, 1.0);
                let yaw = Hanging::square(&wall, rig.forward());
                let start = RunStart { leg: 0, speed: 4.0 };
                let origin = Vec3::new(0.0, 0.0, wall.a.z + WallRun::takeoff(start, 0.0, &stood, &rig));
                WallRun::plan(&wall, origin, yaw, start, 0.0, &stood, &rig).expect("a 2.5 m wall run up")
            } else {
                // A tall wall met 0.75 rad off square, kicked off toward a
                // 2.5 m lip at right angles to it (as `wall`'s tests do).
                let slant = 0.75f32;
                let wall = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 8.0, 4.0, 1.0);
                let target = Ledge::wall(Vec3::new(-2.2, 0.0, 0.5), Vec3::X, 3.0, 2.5, 1.0);
                let yaw = Hanging::square(&wall, rig.forward()) + slant;
                let forward = bevy::math::Quat::from_rotation_y(yaw) * rig.forward();
                let leg = WallRun::kick_leg(&wall, yaw, &stood, &rig);
                let start = RunStart { leg, speed: 4.0 };
                let hit = Vec3::new(-2.2 + 1.8 - 0.6 * slant.tan(), 0.0, -1.0);
                let origin = hit - forward * WallRun::kick_takeoff(start, slant, &stood, &rig);
                WallRun::kick(&wall, &target, origin, yaw, start, 0.0, &stood, &rig).expect("a kick toward a 2.5 m lip")
            };
            let wall = run.lip().expect("a lip to catch");
            let mut probe = run.clone();
            while !probe.is_released() {
                probe.advance(DT);
            }
            let up = probe.elapsed();
            let falling = probe.release(&|_| Some(0.0));
            let (mut probe, mut flight) = (falling.clone(), 0.0);
            while probe.catches(&[wall], &rig).is_none() {
                assert!(probe.airborne() || flight == 0.0, "never caught the lip");
                probe.advance(DT);
                flight += DT;
            }
            Some((run, up, falling, flight, wall))
        }
        _ => None,
    };
    let posed = |cycle: f32| match (&jump, params) {
        _ if let Some((run, up, falling, flight, wall)) = &wall_running => {
            let at = cycle * (up + flight);
            if at < *up {
                let mut now = run.clone();
                now.advance(at);
                Some((now.pose_led(&springs), None))
            } else {
                let mut now = falling.clone();
                now.advance((at - up - DT).max(0.0));
                now.advance(DT);
                std::hint::black_box(now.catches(&[*wall], &rig));
                Some((now.pose_led(&rig, &springs), None))
            }
        }
        // Advanced to a frame before the clock, then a frame, so the catch
        // sweeps a frame as the walker's does.
        _ if let Some((falling, seconds, lower)) = &catching => {
            let mut now = falling.clone();
            now.advance((cycle * seconds - DT).max(0.0));
            now.advance(DT);
            if let Some(lower) = lower {
                std::hint::black_box(now.catches(&[*lower], &rig));
            }
            Some((now.pose_led(&rig, &springs), None))
        }
        _ if let Some((hanging, launch, falling, flight, target)) = &leaping => {
            let at = cycle * (launch + flight);
            if at < *launch {
                let mut now = hanging.clone();
                now.advance(at);
                Some((now.pose_led(&rig, &springs), None))
            } else {
                let mut now = falling.clone();
                now.advance((at - launch - DT).max(0.0));
                now.advance(DT);
                std::hint::black_box(now.catches(&[*target], &rig));
                Some((now.pose_led(&rig, &springs), None))
            }
        }
        // Squeezing: the side shuffle at its pace, the arms flattened, as the
        // walker poses it.
        _ if let Gait::Squeeze(flat) = gait => {
            let params = migera::character::anim::shuffle::shuffling(migera::character::anim::parkour::squeeze::SQUEEZE_SPEED, 1.0, 0.0);
            let mut pose = walk_pose_on(cycle, &params, &stood, &rig);
            if flat {
                migera::character::anim::parkour::squeeze::flatten(&mut pose, &rig, 1.0);
            }
            Some((pose, Some(params)))
        }
        _ if let Some((jump, handed, falling)) = &onto => {
            let t = cycle * (handed + falling.ends()[2]);
            if t < *handed {
                Some((jump.pose_led(t, &stood, &rig, &springs), None))
            } else {
                let mut now = falling.clone();
                now.advance(t - handed);
                Some((now.pose_led(&rig, &springs), None))
            }
        }
        _ if let Some(faith) = &faith => {
            let mut now = faith.clone();
            now.advance(cycle * faith.end());
            Some((now.pose_led(&springs), None))
        }
        _ if let Some(skid) = &skidding => {
            let mut now = skid.clone();
            now.advance(cycle * skid.end());
            Some((now.pose_led(&springs), None))
        }
        _ if let Some(climb) = &free_climbing => {
            let mut now = climb.clone();
            now.advance(Some(Vec2::Y), cycle * 3.0);
            Some((now.pose_led(&springs), None))
        }
        _ if let Some(crawl) = &crawling => {
            let mut now = crawl.clone();
            now.advance(true, cycle);
            Some((now.pose(), None))
        }
        _ if let Some(slide) = &under_slide => {
            let mut now = slide.clone();
            now.advance(cycle * slide.end());
            Some((now.pose_led(&springs), None))
        }
        _ if let Some(poling) = &poling => {
            let mut now = poling.clone();
            now.advance(Some(migera::character::anim::parkour::pole::PoleAsk::Up), cycle * migera::character::anim::parkour::pole::CYCLE);
            Some((now.pose_led(&rig, &springs), None))
        }
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
        // On a beam: the walk with the feet narrowed and the arms out, as the
        // walker poses it.
        _ if let Gait::Beam = gait => {
            use migera::character::anim::parkour::beam;
            let walk = GaitParams::walking_on(beam::BEAM_SPEED, &rig);
            let params = GaitParams { feet_apart: beam::BEAM_FEET, arm_swing: 0.0, ..walk };
            let mut pose = walk_pose_on(cycle, &params, &stood, &rig);
            beam::balance(&mut pose, &rig, 1.0, beam::sway_at(cycle * 4.0));
            Some((pose, Some(params)))
        }
        // Perching: crouched down and up over the cycle, as the walker
        // eases it (its pose made once).
        _ if let Gait::Perch = gait => {
            let mut pose = stood;
            migera::character::anim::parkour::perch::perch(&mut pose, perched.as_ref().expect("a perch"), &rig, 0.5 - 0.5 * (std::f32::consts::TAU * cycle).cos());
            Some((pose, None))
        }
        // Walking beside a wall 0.45 m out from the left shoulder, a hand on
        // it, as the walker finds and rests it.
        _ if let Gait::WallHand = gait => {
            use migera::character::anim::parkour::wallhand;
            let params = GaitParams::walking_on(1.2, &rig);
            let mut pose = walk_pose_on(cycle, &params, &stood, &rig);
            let left = rig.left();
            let face = migera::character::anim::rig::forward_kinematics_on(&stood, &rig)[migera::character::skeleton::Bone::LeftArm].dot(left) + 0.45;
            let blocks = |p: Vec3, low: f32| p.dot(left) >= face && 2.5 > low;
            if let Some(wall) = wallhand::beside(&pose, Vec3::ZERO, 0.0, &rig, &blocks) {
                wallhand::rest_hand(&mut pose, Vec3::ZERO, 0.0, &rig, &wall, wall.weight());
            }
            Some((pose, Some(params)))
        }
        // Running round a turn, leant as the walker leans it.
        _ if let Gait::RunLean = gait => {
            use migera::character::anim::parkour::lean;
            let params = GaitParams::running_on(4.0, &rig);
            let mut pose = walk_pose_on(cycle, &params, &stood, &rig);
            lean::lean(&mut pose, &rig, lean::lean_for(2.0, lean::turning(4.0, 1.0)));
            Some((pose, Some(params)))
        }
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

    // `--walls N`: each character also kept out of walls and steered round
    // them as the walker is each frame (`walker::keep_off_walls`,
    // `walker::way_round`), against N blocks 2 m square in a row, each
    // character walking straight into one and held at it (the costly case:
    // in touch every frame).
    let walls = walls();
    let blocks: Vec<migera::character::anim::parkour::Ledge> = (0..walls)
        .flat_map(|i| migera::character::anim::parkour::Ledge::block(Vec3::new(3.0 * i as f32, 0.0, -1.0), Vec3::Z, 2.0, 2.0, 3.0))
        .collect();
    let ground = migera::character::anim::parkour::geometry::LedgeGround::new(Box::new(migera::character::anim::ground::FlatGround::default()), blocks);
    // With `--walls-away`, walking away from them instead: the open floor,
    // nothing in reach (the common case).
    let away = std::env::args().any(|a| a == "--walls-away");
    let way = if away { Vec3::Z } else { Vec3::NEG_Z };
    let mut walkers: Vec<Vec3> = (0..characters).map(|i| Vec3::new(3.0 * (i % walls.max(1)) as f32, 0.0, -0.5)).collect();
    let keep_off = |walkers: &mut Vec<Vec3>| {
        use migera::character::anim::walker::{keep_off_walls, way_round};
        for at in walkers.iter_mut() {
            std::hint::black_box(way_round(*at, 0.0, 0.0, 0.77, way, &ground));
            *at += keep_off_walls(*at, way * (1.4 * DT), &ground).0;
        }
    };

    let mut samples: Vec<f64> = Vec::with_capacity(frames);
    for _ in 0..frames {
        let started = Instant::now();
        step(&mut states, &layer, &base, &springs, DT, &posed, &rig);
        if walls > 0 {
            keep_off(&mut walkers);
        }
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
        Gait::DropDown => "   dropping down into a braced hang from a wall's top".to_string(),
        Gait::Mantle => "   mantling onto a 1.1 m wall from standing".to_string(),
        Gait::WallRun => "   running up a 2.5 m wall at 4 m/s and catching its lip".to_string(),
        Gait::WallKick => "   kicking off a wall at 4 m/s, 0.75 rad off square, and catching a 2.5 m lip round the corner".to_string(),
        Gait::RunAlong => "   running along a wall 0.55 m off at 4 m/s and running on".to_string(),
        Gait::Slide => "   sliding down a 4.5 m wall from a braced hang".to_string(),
        Gait::BarSwing => "   swinging on a 2.3 m bar, pumped up".to_string(),
        Gait::Pole => "   climbing a pole, a cycle of the climb up".to_string(),
        Gait::Beam => "   walking on a beam, balancing".to_string(),
        Gait::SlideUnder => "   sliding under a 0.9 m slab from a 5 m/s run, to standing".to_string(),
        Gait::Crawl => "   crawling on hands and knees".to_string(),
        Gait::Squeeze(flat) => if flat { "   squeezing sideways, the arms flat" } else { "   the side shuffle" }.to_string(),
        Gait::Lache => "   a lache from a 2.3 m bar to one 2 m ahead, from letting go to the catch".to_string(),
        Gait::FreeClimb => "   free climbing up a wall of holds".to_string(),
        Gait::RunLean => "   a 4 m/s run turning and gathering, leant".to_string(),
        Gait::Skid(round) => if round { "   a plant-and-turn from 5 m/s" } else { "   a skid stop from 5 m/s" }.to_string(),
        Gait::Onto => "   a standing jump onto a 0.2 m post 1.2 m ahead".to_string(),
        Gait::WallHand => "   a 1.2 m/s walk beside a wall, a hand on it".to_string(),
        Gait::Perch => "   crouching into a perch and up".to_string(),
        Gait::SpinJump => "   a standing jump turning round in the air".to_string(),
        Gait::Faith => "   a leap of faith from 6 m into a pile".to_string(),
        Gait::Vault(kind) => match kind {
            migera::character::anim::parkour::vault::VaultKind::Hop => "   hopping a 0.35 m rail in a 3.5 m/s run".to_string(),
            migera::character::anim::parkour::vault::VaultKind::Lazy => "   lazy vaulting a 0.9 m wall from a 3.5 m/s run".to_string(),
            migera::character::anim::parkour::vault::VaultKind::Speed => "   speed vaulting a 0.9 m wall from a 3.5 m/s run".to_string(),
        },
        Gait::LetGo(catch) => format!("   letting go of a hang and {}", if catch { "catching a ledge below" } else { "landing (3 m)" }),
        Gait::JumpShort => "   a jump falling short and catching the far ledge".to_string(),
        Gait::Leap(way) => {
            use migera::character::anim::parkour::hang::Leap;
            let way = match way {
                Leap::Up => "up to a ledge 0.8 m above",
                Leap::Aside(_) => "aside to the next ledge along",
                Leap::Back => "back to a wall behind",
                Leap::UpAside(_) => "up and aside to a ledge 0.6 m above",
                Leap::BackAside(_) => "back and aside to a wall behind",
            };
            format!("   leaping from a hang {way} and catching it")
        }
    };
    let gait = match (walls, away) {
        (0, _) => gait,
        (_, false) => format!("{gait}   held at walls ({walls} blocks)"),
        (_, true) => format!("{gait}   clear of walls ({walls} blocks)"),
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
/// `--walls N`: blocks the characters are kept off (see the timed loop).
fn walls() -> usize {
    let args: Vec<String> = std::env::args().collect();
    args.iter().position(|a| a == "--walls").and_then(|i| args.get(i + 1)).and_then(|v| v.parse().ok()).unwrap_or(0)
}

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
        Some("drop-down") => Gait::DropDown,
        Some("mantle") => Gait::Mantle,
        Some("vault") => Gait::Vault(migera::character::anim::parkour::vault::VaultKind::Speed),
        Some("wall-run") => Gait::WallRun,
        Some("wall-kick") => Gait::WallKick,
        Some("run-along") => Gait::RunAlong,
        Some("wall-slide") => Gait::Slide,
        Some("bar-swing") => Gait::BarSwing,
        Some("pole") => Gait::Pole,
        Some("beam") => Gait::Beam,
        Some("slide-under") => Gait::SlideUnder,
        Some("crawl") => Gait::Crawl,
        Some("squeeze") => Gait::Squeeze(true),
        Some("shuffle") => Gait::Squeeze(false),
        Some("lache") => Gait::Lache,
        Some("free-climb") => Gait::FreeClimb,
        Some("run-lean") => Gait::RunLean,
        Some("skid") => Gait::Skid(false),
        Some("plant-turn") => Gait::Skid(true),
        Some("onto") => Gait::Onto,
        Some("wall-hand") => Gait::WallHand,
        Some("perch") => Gait::Perch,
        Some("spin-jump") => Gait::SpinJump,
        Some("faith") => Gait::Faith,
        Some("vault-lazy") => Gait::Vault(migera::character::anim::parkour::vault::VaultKind::Lazy),
        Some("hop") => Gait::Vault(migera::character::anim::parkour::vault::VaultKind::Hop),
        Some("let-go") => Gait::LetGo(false),
        Some("catch") => Gait::LetGo(true),
        Some("jump-catch") => Gait::JumpShort,
        Some("leap-up") => Gait::Leap(migera::character::anim::parkour::hang::Leap::Up),
        Some("leap-aside") => Gait::Leap(migera::character::anim::parkour::hang::Leap::Aside(migera::character::anim::parkour::hang::Shimmy::Right)),
        Some("leap-back") => Gait::Leap(migera::character::anim::parkour::hang::Leap::Back),
        Some("leap-up-aside") => Gait::Leap(migera::character::anim::parkour::hang::Leap::UpAside(migera::character::anim::parkour::hang::Shimmy::Right)),
        Some("leap-back-aside") => Gait::Leap(migera::character::anim::parkour::hang::Leap::BackAside(migera::character::anim::parkour::hang::Shimmy::Right)),
        _ => Gait::None,
    };
    (characters.max(1), frames.max(1), gait)
}
