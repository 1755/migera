//! Scenario harness: builds traces from functions of time and measures what
//! a player would feel.
//!
//! A [`Scenario`] describes the target's path, the input and mode switches
//! as functions of *time*, never of frame number, so the same scenario can
//! be sampled at 30, 60 and 144 Hz and compared at common timestamps. That
//! is the only honest frame-rate-independence test: comparing frame N at
//! one rate with frame N at another compares different moments.
//!
//! [`Metrics`] summarise a run: worst roll, eye speed and acceleration, and
//! the pitch range. Later phases add line-of-sight and inside-collider
//! counts.

use super::clock::CameraClock;
use super::collision::ResolvedPose;
use super::probe::CameraProbe;
use super::input::CameraInput;
use super::pipeline::{CameraFrame, CameraOutput, TargetSample};
use super::stack::ModeRequest;
use super::trace::CameraTrace;
use bevy::prelude::*;

/// The target at time `t`: root position and whether it stands on ground.
pub type PathFn = Box<dyn Fn(f32) -> (Vec3, bool)>;
/// The input held at time `t`.
pub type InputFn = Box<dyn Fn(f32) -> CameraInput>;

pub struct Scenario {
    pub duration: f32,
    pub start_yaw: f32,
    pub path: PathFn,
    pub input: InputFn,
    /// Mode switches, each applied on the first frame ending at or after
    /// its time.
    pub requests: Vec<(f32, ModeRequest)>,
    /// Seconds of virtual time per second of real time (0 = paused).
    pub time_scale: f32,
}

impl Scenario {
    pub fn new(duration: f32, path: PathFn, input: InputFn) -> Self {
        Self { duration, start_yaw: 0.0, path, input, requests: Vec::new(), time_scale: 1.0 }
    }

    /// Samples the scenario into a trace at `hz`. Frame `i` covers
    /// `(i·dt, (i+1)·dt]`: the target is sampled at its end, the input held
    /// during it at its middle (so an input that changes on a frame boundary
    /// is attributed to the right frame at every rate).
    pub fn sample(&self, hz: f32) -> CameraTrace {
        let steps = (self.duration * hz).round() as usize;
        let dt = self.duration / steps as f32;
        let mut pending: Vec<&(f32, ModeRequest)> = self.requests.iter().collect();
        let frames = (0..steps)
            .map(|i| {
                let t = (i + 1) as f32 * dt;
                let (position, grounded) = (self.path)(t * self.time_scale);
                let mut requests = Vec::new();
                pending.retain(|(at, request)| {
                    if *at <= t + 1.0e-6 {
                        requests.push(request.clone());
                        false
                    } else {
                        true
                    }
                });
                CameraFrame {
                    clock: CameraClock { real_dt: dt, virtual_dt: dt * self.time_scale },
                    input: (self.input)(t - 0.5 * dt),
                    target: TargetSample { position, grounded, ..Default::default() },
                    requests,
                    goals: Vec::new(),
                }
            })
            .collect();
        CameraTrace { start_yaw: self.start_yaw, frames }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Metrics {
    pub frames: usize,
    /// Largest |camera right · world up|: 0 means level.
    pub max_roll: f32,
    pub max_eye_speed: f32,
    pub max_eye_acceleration: f32,
    pub min_pitch: f32,
    pub max_pitch: f32,
}

/// Measures a run. Frames flagged as cuts are excluded from speed and
/// acceleration, which a cut is allowed to break.
pub fn measure(trace: &CameraTrace, outputs: &[CameraOutput]) -> Metrics {
    let mut metrics = Metrics {
        frames: outputs.len(),
        min_pitch: f32::INFINITY,
        max_pitch: f32::NEG_INFINITY,
        ..Default::default()
    };
    let mut previous: Option<(Vec3, Option<Vec3>)> = None;
    for (frame, out) in trace.frames.iter().zip(outputs) {
        let right = out.pose.rotation * Vec3::X;
        metrics.max_roll = metrics.max_roll.max(right.y.abs());
        metrics.min_pitch = metrics.min_pitch.min(out.pitch);
        metrics.max_pitch = metrics.max_pitch.max(out.pitch);
        let dt = frame.clock.real_dt;
        if out.cut || dt <= 0.0 {
            previous = Some((out.pose.eye, None));
            continue;
        }
        if let Some((eye, velocity)) = previous {
            let v = (out.pose.eye - eye) / dt;
            metrics.max_eye_speed = metrics.max_eye_speed.max(v.length());
            if let Some(previous_v) = velocity {
                metrics.max_eye_acceleration =
                    metrics.max_eye_acceleration.max((v - previous_v).length() / dt);
            }
            previous = Some((out.pose.eye, Some(v)));
        } else {
            previous = Some((out.pose.eye, None));
        }
    }
    metrics
}

/// What collision achieved over a run.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CollisionMetrics {
    pub frames: usize,
    /// Frames with the eye's probe sphere inside geometry. Must be 0.
    pub inside_frames: usize,
    /// Frames where the sight line (origin → shoulder → eye) is blocked.
    pub blocked_frames: usize,
    /// The longest unbroken run of blocked frames, seconds: occlusion is
    /// allowed to last only `min_occlusion_time` before the camera acts.
    pub longest_blocked: f32,
    pub min_distance: f32,
}

/// Checks every resolved pose of a run against the geometry.
pub fn measure_collision(
    trace: &CameraTrace,
    resolved: &[ResolvedPose],
    probe: &dyn CameraProbe,
) -> CollisionMetrics {
    // A thin sight line, and a sphere a hair smaller than the probe so a
    // camera resting exactly on contact is not counted as inside.
    const SIGHT: f32 = 0.01;
    let clear = |from: Vec3, to: Vec3| {
        let step = to - from;
        match step.try_normalize() {
            Some(direction) => probe.sweep(from, direction, step.length(), SIGHT).is_none(),
            None => true,
        }
    };
    let mut metrics = CollisionMetrics { frames: resolved.len(), min_distance: f32::INFINITY, ..default() };
    let mut run = 0.0;
    for (frame, pose) in trace.frames.iter().zip(resolved) {
        if probe.overlaps(pose.eye, pose.probe_radius * 0.98) {
            metrics.inside_frames += 1;
        }
        if clear(pose.origin, pose.shoulder) && clear(pose.shoulder, pose.eye) {
            run = 0.0;
        } else {
            metrics.blocked_frames += 1;
            run += frame.clock.real_dt;
            metrics.longest_blocked = metrics.longest_blocked.max(run);
        }
        metrics.min_distance = metrics.min_distance.min(pose.distance);
    }
    metrics
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::input::CameraInputSettings;
    use crate::camera::pipeline::CameraConfig;
    use crate::camera::stack::ModeId;
    use crate::camera::trace::GoldenTrace;

    /// Walks along −Z at 3 m/s for two seconds, then turns onto +X at 4 m/s;
    /// grounded throughout. Straight segments meeting at a frame boundary of
    /// every tested rate, so the target really is linear within each frame.
    fn walk_and_turn(t: f32) -> (Vec3, bool) {
        let p = if t <= 2.0 {
            Vec3::new(0.0, 0.0, -3.0 * t)
        } else {
            Vec3::new(4.0 * (t - 2.0), 0.0, -6.0)
        };
        (p, true)
    }

    /// Stick held right for half a second, released, held up-left later;
    /// constant over intervals that are whole frames at every tested rate.
    fn some_look(t: f32) -> CameraInput {
        let stick = if t <= 0.5 {
            Vec2::new(0.6, 0.0)
        } else if (1.5..2.0).contains(&t) {
            Vec2::new(-0.4, 0.5)
        } else {
            Vec2::ZERO
        };
        CameraInput { look_stick: stick, ..Default::default() }
    }

    fn scenario() -> Scenario {
        let mut s = Scenario::new(6.0, Box::new(walk_and_turn), Box::new(some_look));
        s.requests.push((1.0, ModeRequest { id: ModeId::new("combat"), blend: 0.5 }));
        s.requests.push((3.0, ModeRequest { id: ModeId::new("explore"), blend: 0.5 }));
        s
    }

    fn eye_at(trace: &CameraTrace, outputs: &[CameraOutput], t: f32) -> Vec3 {
        let dt = trace.frames[0].clock.real_dt;
        let index = (t / dt).round() as usize - 1;
        outputs[index].pose.eye
    }

    #[test]
    fn the_camera_lands_in_the_same_place_at_30_60_and_144_hz() {
        let (settings, config) = (CameraInputSettings::default(), CameraConfig::default());
        let s = scenario();
        let runs: Vec<_> = [30.0, 60.0, 144.0]
            .iter()
            .map(|&hz| {
                let trace = s.sample(hz);
                let outputs = trace.replay(&settings, &config);
                (hz, trace, outputs)
            })
            .collect();
        let (_, fine_trace, fine) = &runs[2];
        // Every 1/6 s: the common frame boundaries of 30, 60 and 144 Hz,
        // which also lands samples inside the 0.5 s mode blends.
        for t in (1..=36).map(|i| i as f32 / 6.0) {
            let reference = eye_at(fine_trace, fine, t);
            for (hz, trace, outputs) in &runs[..2] {
                let eye = eye_at(trace, outputs, t);
                assert!(
                    eye.distance(reference) < 0.02,
                    "at t = {t} s the eye is {eye:?} at {hz} Hz but {reference:?} at 144 Hz \
                     ({:.4} m apart)",
                    eye.distance(reference),
                );
            }
        }
    }

    #[test]
    fn the_camera_never_rolls_through_a_whole_scenario() {
        let (settings, config) = (CameraInputSettings::default(), CameraConfig::default());
        let trace = scenario().sample(60.0);
        let metrics = measure(&trace, &trace.replay(&settings, &config));
        assert!(metrics.max_roll < 1.0e-5, "roll reached {}", metrics.max_roll);
    }

    #[test]
    fn mode_switches_keep_the_eye_velocity_continuous() {
        // A push blends with smoothstep weights, whose peak acceleration over
        // a move of length Δ in time T is 6Δ/T². A linear blend instead
        // steps the velocity at both ends: Δ/T per frame, ~90 m/s² here.
        let (settings, config) = (CameraInputSettings::default(), CameraConfig::default());
        let still = Scenario::new(3.0, Box::new(|_| (Vec3::ZERO, true)), Box::new(|_| CameraInput::default()));
        let mut switching = Scenario::new(3.0, Box::new(|_| (Vec3::ZERO, true)), Box::new(|_| CameraInput::default()));
        let blend = 0.4;
        switching.requests.push((1.0, ModeRequest { id: ModeId::new("combat"), blend }));
        let trace = switching.sample(60.0);
        let outputs = trace.replay(&settings, &config);
        let moved = outputs[50].pose.eye.distance(outputs[outputs.len() - 1].pose.eye);
        let bound = 1.15 * 6.0 * moved / (blend * blend);
        let metrics = measure(&trace, &outputs);
        assert!(
            metrics.max_eye_acceleration < bound,
            "a {blend} s mode blend over {moved:.3} m accelerated the eye at {} m/s² (smoothstep \
             bound {bound:.1})",
            metrics.max_eye_acceleration,
        );
        let calm = still.sample(60.0);
        let calm_metrics = measure(&calm, &calm.replay(&settings, &config));
        assert!(calm_metrics.max_eye_speed < 1.0e-4, "a still scene must hold the camera still");
    }

    #[test]
    fn a_paused_game_still_lets_the_player_look_but_freezes_the_pivot() {
        let (settings, config) = (CameraInputSettings::default(), CameraConfig::default());
        let mut paused = Scenario::new(
            2.0,
            Box::new(|t| (Vec3::new(0.0, 0.0, -3.0 * t), true)),
            // About 1.5 rad over the run: well clear of a ±π wrap.
            Box::new(|_| CameraInput { look_stick: Vec2::new(0.6, 0.0), ..Default::default() }),
        );
        paused.time_scale = 0.0;
        let trace = paused.sample(60.0);
        let outputs = trace.replay(&settings, &config);
        let (first, last) = (outputs[1], outputs[outputs.len() - 1]);
        assert!(
            crate::math::angle::angle_delta(first.yaw, last.yaw).abs() > 1.0,
            "look must keep working while paused: yaw {} → {}",
            first.yaw,
            last.yaw,
        );
        assert_eq!(first.pose.pivot, last.pose.pivot, "the pivot must not move while paused");
    }

    #[test]
    fn a_trace_round_trips_through_ron_and_replays_identically() {
        let (settings, config) = (CameraInputSettings::default(), CameraConfig::default());
        let trace = scenario().sample(30.0);
        let text = trace.to_ron().expect("serialize");
        let back = CameraTrace::from_ron(&text).expect("parse");
        assert_eq!(back, trace);
        assert_eq!(back.replay(&settings, &config), trace.replay(&settings, &config));
    }

    /// The playground's geometry as SDF boxes, plus a 2 cm fence.
    fn playground() -> crate::camera::probe::SdfProbe {
        let mut probe = crate::camera::probe::SdfProbe::default()
            .with_box(Vec3::new(0.0, -0.1, 0.0), Vec3::new(80.0, 0.2, 80.0))
            .with_box(Vec3::new(-6.0, 1.5, -4.0), Vec3::new(0.6, 3.0, 20.0))
            .with_box(Vec3::new(-1.4, 1.3, -20.0), Vec3::new(0.4, 2.6, 10.0))
            .with_box(Vec3::new(1.4, 1.3, -20.0), Vec3::new(0.4, 2.6, 10.0))
            .with_box(Vec3::new(0.0, 2.75, -20.0), Vec3::new(3.2, 0.3, 10.0))
            .with_box(Vec3::new(3.0, 1.25, 2.0), Vec3::new(0.02, 2.5, 3.0));
        for k in 0..6 {
            probe = probe.with_box(Vec3::new(5.0, 2.0, -2.0 - 3.0 * k as f32), Vec3::new(0.6, 4.0, 0.6));
        }
        probe
    }

    /// Walks the given waypoints in order at `speed`, then stands.
    fn route(points: Vec<Vec3>, speed: f32) -> PathFn {
        Box::new(move |t| {
            let mut left = t * speed;
            for pair in points.windows(2) {
                let length = pair[0].distance(pair[1]);
                if left <= length {
                    return (pair[0].lerp(pair[1], left / length), true);
                }
                left -= length;
            }
            (*points.last().unwrap(), true)
        })
    }

    #[test]
    fn a_camera_orbiting_through_the_playground_never_enters_geometry() {
        let (settings, config) = (CameraInputSettings::default(), CameraConfig::default());
        let probe = playground();
        // Into and through the roofed corridor, out west along the long
        // wall with the camera backed against it, then east past the
        // pillars and the thin fence, while the stick keeps orbiting and
        // nodding the camera.
        let path = route(
            vec![
                Vec3::new(0.0, 0.0, 4.0),
                Vec3::new(0.0, 0.0, -26.0),
                Vec3::new(0.0, 0.0, -14.0),
                Vec3::new(-5.3, 0.0, -10.0),
                Vec3::new(-5.3, 0.0, 4.0),
                Vec3::new(4.2, 0.0, 2.0),
                Vec3::new(4.2, 0.0, -18.0),
            ],
            2.0,
        );
        let look: InputFn = Box::new(|t| CameraInput {
            look_stick: Vec2::new(0.55, if ((t / 3.0) as usize).is_multiple_of(2) { 0.5 } else { -0.5 }),
            ..Default::default()
        });
        let trace = Scenario::new(40.0, path, look).sample(60.0);
        let resolved: Vec<_> =
            trace.replay_resolved(&settings, &config, &probe, 0.13).into_iter().map(|(_, r)| r).collect();
        let metrics = measure_collision(&trace, &resolved, &probe);
        assert_eq!(metrics.inside_frames, 0, "the eye entered geometry: {metrics:?}");
        let allowed = config.collision.min_occlusion_time + 2.0 / 60.0;
        assert!(
            metrics.longest_blocked <= allowed,
            "the character was hidden for {:.3} s at a stretch (allowed {allowed:.3}): {metrics:?}",
            metrics.longest_blocked,
        );
    }

    const GOLDEN: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/golden/camera/walk_turn_combat.camtrace.ron");

    /// Rewrites the golden trace from the current code. Run deliberately,
    /// after a behaviour change you have verified:
    /// `cargo test --release --lib camera::harness -- --ignored`
    #[test]
    #[ignore]
    fn regenerate_the_golden_trace() {
        let (settings, config) = (CameraInputSettings::default(), CameraConfig::default());
        let golden = scenario().sample(30.0).golden(&settings, &config, 6);
        let text = golden.to_ron().unwrap();
        std::fs::create_dir_all(std::path::Path::new(GOLDEN).parent().unwrap()).unwrap();
        std::fs::write(GOLDEN, text).unwrap();
    }

    #[test]
    fn the_golden_trace_replays_within_a_centimetre() {
        let text = std::fs::read_to_string(GOLDEN)
            .expect("missing golden trace; run the ignored regenerate_the_golden_trace test");
        let golden: GoldenTrace = ron::from_str(&text).expect("golden trace parses");
        let (settings, config) = (CameraInputSettings::default(), CameraConfig::default());
        let worst = golden.worst_eye_error(&settings, &config);
        assert!(worst < 0.01, "the camera drifted {worst} m from its recorded behaviour");
    }
}
