//! The orbit stage: yaw, pitch and zoom.
//!
//! State is orbit scalars (Nesky #3), never a world pose, so the camera
//! rotation is rebuilt as yaw·pitch each frame and cannot roll.
//!
//! Who moves yaw and pitch, in priority order (one owner per degree of
//! freedom; nothing fights):
//! 1. **A goal** ([`OrbitGoal`]: lock-on, dialog, aim assist). While a goal
//!    owns an axis, its base angle damps toward the goal and the player's
//!    input becomes an *offset* on top, which decays back once the player
//!    lets go. When the goal ends the offset folds into the base, so nothing
//!    jumps.
//! 2. **The recentre button**: a fast damp behind the character.
//! 3. **The player**, directly.
//! 4. **Auto-recentre**: behind the direction of travel, only after
//!    `recenter_delay` seconds without look input and only while moving,
//!    faster the faster the character goes (Nesky #23, #35).
//!
//! Everything here runs on *real* time, so the player can look around a
//! paused game. Stick look is integrated exactly over the frame (the rate
//! approaches the deflected rate exponentially, and the integral of that is
//! closed-form), so a held stick turns the camera the same at any frame rate.

use super::input::{CameraInput, CameraInputSettings};
use super::rig::OrbitShape;
use crate::math::angle::{
    angle_delta, damp, damp_angle, radial_deadzone, soft_limit_rate, stick_curve, wrap_angle,
};
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::f32::consts::LN_2;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
pub struct OrbitParams {
    /// Seconds without look input before auto-recentre may start.
    pub recenter_delay: f32,
    /// Half-life of auto-recentre at full speed, seconds.
    pub recenter_halflife: f32,
    /// Below this horizontal speed (m/s) auto-recentre never runs.
    pub recenter_min_speed: f32,
    /// At and above this speed auto-recentre runs at full strength.
    pub recenter_full_speed: f32,
    /// Half-life of the recentre button's swing, seconds.
    pub button_halflife: f32,
    /// Half-life of the player's offset decaying under a goal, seconds.
    pub goal_offset_halflife: f32,
    /// Most the player may push away from a goal, radians.
    pub goal_offset_max: f32,
    /// Look toward a pitch limit slows to zero over this band, radians.
    pub pitch_soft_band: f32,
    pub zoom_min: f32,
    pub zoom_max: f32,
    /// Zoom change per zoom step (multiplier on the mode's distance).
    pub zoom_step: f32,
    pub zoom_halflife: f32,
    /// Half-life of a shoulder swap, seconds.
    pub shoulder_halflife: f32,
    /// Half-life of the pitch easing under a low ceiling, seconds.
    pub ceiling_halflife: f32,
}

impl Default for OrbitParams {
    fn default() -> Self {
        Self {
            recenter_delay: 1.5,
            recenter_halflife: 0.8,
            recenter_min_speed: 0.5,
            recenter_full_speed: 3.0,
            button_halflife: 0.08,
            goal_offset_halflife: 0.6,
            goal_offset_max: 45f32.to_radians(),
            pitch_soft_band: 10f32.to_radians(),
            zoom_min: 0.6,
            zoom_max: 1.6,
            zoom_step: 0.1,
            zoom_halflife: 0.08,
            shoulder_halflife: 0.1,
            ceiling_halflife: 0.2,
        }
    }
}

/// A driver that wants the camera to face somewhere. Higher `priority` owns
/// an axis; `None` leaves that axis to lower priorities.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
pub struct OrbitGoal {
    pub priority: i32,
    pub yaw: Option<f32>,
    pub pitch: Option<f32>,
    pub halflife: f32,
}

/// What the orbit knows about the character and its drivers this frame.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct OrbitContext<'a> {
    /// Camera yaw that looks along the direction of travel, if moving.
    pub heading: Option<f32>,
    /// Horizontal speed, m/s.
    pub speed: f32,
    /// Camera yaw that looks the way the character faces.
    pub facing: Option<f32>,
    /// Goals active this frame (lock-on, dialog, assist).
    pub goals: &'a [OrbitGoal],
    /// The steepest pitch at which the full boom fits under the ceiling,
    /// from last frame's collision: the pitch eases under it rather than the
    /// boom crushing in.
    pub pitch_cap: Option<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub struct OrbitState {
    pub base_yaw: f32,
    pub base_pitch: f32,
    /// The player's offset from a goal, while one owns the axis.
    pub yaw_offset: f32,
    pub pitch_offset: f32,
    /// Current stick look rate (right, up), rad/s.
    pub stick_rate: Vec2,
    /// Seconds since the last look input.
    pub idle: f32,
    pub zoom: f32,
    pub zoom_target: f32,
    /// Which shoulder the camera sits over: +1 right, −1 left, sprung
    /// between them when swapped.
    pub shoulder_side: f32,
    pub shoulder_side_target: f32,
    /// The recentre button's swing is in progress.
    pub recentering: bool,
    pub yaw_goal_active: bool,
    pub pitch_goal_active: bool,
}

impl OrbitState {
    pub fn new(yaw: f32, pitch: f32) -> Self {
        Self {
            base_yaw: wrap_angle(yaw),
            base_pitch: pitch,
            yaw_offset: 0.0,
            pitch_offset: 0.0,
            stick_rate: Vec2::ZERO,
            idle: 0.0,
            zoom: 1.0,
            zoom_target: 1.0,
            shoulder_side: 1.0,
            shoulder_side_target: 1.0,
            recentering: false,
            yaw_goal_active: false,
            pitch_goal_active: false,
        }
    }

    pub fn yaw(&self) -> f32 {
        wrap_angle(self.base_yaw + self.yaw_offset)
    }

    pub fn pitch(&self) -> f32 {
        self.base_pitch + self.pitch_offset
    }

    /// Advances by `dt` seconds of real time.
    pub fn step(
        &mut self,
        input: &CameraInput,
        settings: &CameraInputSettings,
        shape: &OrbitShape,
        params: &OrbitParams,
        context: &OrbitContext,
        dt: f32,
    ) {
        // Look this frame as (right, up) radians: mouse plus integrated stick.
        let stick = radial_deadzone(input.look_stick, settings.stick_deadzone);
        let target_rate = Vec2::new(
            stick_curve(stick.x, settings.stick_curve),
            stick_curve(stick.y, settings.stick_curve),
        ) * settings.stick_max_rate;
        let stick_look = self.integrate_stick(target_rate, settings.stick_accel_halflife, dt);
        let mut look = input.look_delta + stick_look;
        if settings.invert_x {
            look.x = -look.x;
        }
        if settings.invert_y {
            look.y = -look.y;
        }
        let looking = input.look_delta != Vec2::ZERO || stick != Vec2::ZERO;
        if looking {
            self.idle = 0.0;
            self.recentering = false;
        } else {
            self.idle += dt;
        }
        if input.recenter {
            self.recentering = true;
        }

        // Looking right turns yaw negative (yaw is about +Y); looking up
        // lowers the elevation.
        let yaw_change = -look.x;
        let pitch_max = match context.pitch_cap {
            Some(cap) => shape.pitch_max.min(cap.max(shape.pitch_min)),
            None => shape.pitch_max,
        };
        let pitch_change = soft_limit_rate(
            self.pitch(),
            -look.y,
            shape.pitch_min,
            pitch_max,
            params.pitch_soft_band,
        );

        let goals = context.goals;
        let yaw_goal = goals.iter().filter(|g| g.yaw.is_some()).max_by_key(|g| g.priority);
        let pitch_goal = goals.iter().filter(|g| g.pitch.is_some()).max_by_key(|g| g.priority);
        // The part of this frame that lies past the recentre delay. Using it
        // instead of a whole frame once `idle` crosses the delay keeps the
        // start of a recentre from snapping to frame boundaries, which would
        // differ by frame rate.
        let settle_dt = (self.idle - params.recenter_delay).clamp(0.0, dt);

        // Yaw.
        match yaw_goal {
            Some(goal) => {
                if !self.yaw_goal_active {
                    self.base_yaw = self.yaw();
                    self.yaw_offset = 0.0;
                }
                self.base_yaw = damp_angle(self.base_yaw, goal.yaw.unwrap(), goal.halflife, dt);
                self.yaw_offset = (self.yaw_offset + yaw_change)
                    .clamp(-params.goal_offset_max, params.goal_offset_max);
                self.yaw_offset =
                    damp(self.yaw_offset, 0.0, params.goal_offset_halflife, settle_dt);
            }
            None => {
                if self.yaw_goal_active {
                    self.base_yaw = self.yaw();
                    self.yaw_offset = 0.0;
                }
                self.base_yaw = wrap_angle(self.base_yaw + yaw_change);
            }
        }
        self.yaw_goal_active = yaw_goal.is_some();

        // Pitch.
        match pitch_goal {
            Some(goal) => {
                if !self.pitch_goal_active {
                    self.base_pitch = self.pitch();
                    self.pitch_offset = 0.0;
                }
                self.base_pitch = damp(self.base_pitch, goal.pitch.unwrap(), goal.halflife, dt);
                self.pitch_offset = (self.pitch_offset + pitch_change)
                    .clamp(-params.goal_offset_max, params.goal_offset_max);
                self.pitch_offset =
                    damp(self.pitch_offset, 0.0, params.goal_offset_halflife, settle_dt);
            }
            None => {
                if self.pitch_goal_active {
                    self.base_pitch = self.pitch();
                    self.pitch_offset = 0.0;
                }
                self.base_pitch += pitch_change;
            }
        }
        self.pitch_goal_active = pitch_goal.is_some();

        // Recentring: only on axes no goal owns.
        if self.recentering {
            match context.facing {
                Some(facing) if !self.yaw_goal_active => {
                    self.base_yaw = damp_angle(self.base_yaw, facing, params.button_halflife, dt);
                    if !self.pitch_goal_active {
                        self.base_pitch =
                            damp(self.base_pitch, shape.pitch_default, params.button_halflife, dt);
                    }
                    if angle_delta(self.base_yaw, facing).abs() < 0.5f32.to_radians() {
                        self.recentering = false;
                    }
                }
                _ => self.recentering = false,
            }
        } else if let Some(heading) = context.heading {
            let span = (params.recenter_full_speed - params.recenter_min_speed).max(1.0e-3);
            // Only travel *away* from the camera pulls it round. Moving
            // sideways, recentring would turn "sideways" with it and the
            // character would spiral; moving toward the camera it would
            // flip the view. Full strength within ~25° of straight away,
            // none from ~60°.
            let away = angle_delta(self.base_yaw, heading).cos();
            let alignment = ((away - 0.5) / 0.4).clamp(0.0, 1.0);
            let strength = shape.recenter
                * alignment
                * ((context.speed - params.recenter_min_speed) / span).clamp(0.0, 1.0);
            if strength > 0.0 && settle_dt > 0.0 {
                let halflife = params.recenter_halflife / strength;
                if !self.yaw_goal_active {
                    self.base_yaw = damp_angle(self.base_yaw, heading, halflife, settle_dt);
                }
                if !self.pitch_goal_active {
                    self.base_pitch =
                        damp(self.base_pitch, shape.pitch_default, halflife, settle_dt);
                }
            }
        }

        // Hard pitch limits, on the total.
        let total = self.pitch().clamp(shape.pitch_min, shape.pitch_max);
        self.base_pitch = total - self.pitch_offset;
        // A ceiling eases the pitch down rather than snapping it.
        if pitch_max < shape.pitch_max && self.pitch() > pitch_max {
            self.base_pitch =
                damp(self.base_pitch, pitch_max - self.pitch_offset, params.ceiling_halflife, dt);
        }

        // Zoom.
        self.zoom_target = (self.zoom_target - input.zoom * params.zoom_step)
            .clamp(params.zoom_min, params.zoom_max);
        self.zoom = damp(self.zoom, self.zoom_target, params.zoom_halflife, dt);

        // Shoulder side.
        if input.shoulder_swap {
            self.shoulder_side_target = -self.shoulder_side_target;
        }
        self.shoulder_side =
            damp(self.shoulder_side, self.shoulder_side_target, params.shoulder_halflife, dt);
    }

    /// Moves the stick rate toward `target` (exponential approach) and
    /// returns the exact integral of the rate over the frame.
    fn integrate_stick(&mut self, target: Vec2, halflife: f32, dt: f32) -> Vec2 {
        if dt <= 0.0 {
            return Vec2::ZERO;
        }
        let lambda = LN_2 / halflife.max(1.0e-5);
        let keep = (-lambda * dt).exp();
        let start = self.stick_rate;
        self.stick_rate = target + (start - target) * keep;
        target * dt + (start - target) * ((1.0 - keep) / lambda)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::rig::ModeParams;

    fn shape() -> OrbitShape {
        ModeParams::exploration().orbit_shape()
    }

    fn stick(x: f32, y: f32) -> CameraInput {
        CameraInput { look_stick: Vec2::new(x, y), ..Default::default() }
    }

    fn run(
        orbit: &mut OrbitState,
        input: &CameraInput,
        context: &OrbitContext,
        goals: &[OrbitGoal],
        seconds: f32,
        hz: f32,
    ) {
        let steps = (seconds * hz).round() as usize;
        let context = OrbitContext { goals, ..*context };
        for _ in 0..steps {
            orbit.step(
                input,
                &CameraInputSettings::default(),
                &shape(),
                &OrbitParams::default(),
                &context,
                1.0 / hz,
            );
        }
    }

    #[test]
    fn pitch_never_leaves_its_limits_under_full_stick() {
        let shape = shape();
        for direction in [1.0, -1.0] {
            let mut orbit = OrbitState::new(0.0, 0.2);
            for _ in 0..600 {
                orbit.step(
                    &stick(0.3, direction),
                    &CameraInputSettings::default(),
                    &shape,
                    &OrbitParams::default(),
                    &OrbitContext::default(),
                    1.0 / 60.0,
                );
                let pitch = orbit.pitch();
                assert!(
                    pitch >= shape.pitch_min - 1.0e-5 && pitch <= shape.pitch_max + 1.0e-5,
                    "pitch {pitch} left [{}, {}]",
                    shape.pitch_min,
                    shape.pitch_max,
                );
            }
        }
    }

    #[test]
    fn a_held_stick_turns_the_same_at_any_frame_rate() {
        // Compared while held as well as after release: an Euler integral of
        // the accelerating rate errs one way on the ramp up and the other on
        // the ramp down, so the end alone can hide it.
        let turned = |hz: f32| {
            let mut orbit = OrbitState::new(0.0, 0.2);
            run(&mut orbit, &stick(0.7, 0.0), &OrbitContext::default(), &[], 0.5, hz);
            let held = orbit.yaw();
            run(&mut orbit, &stick(0.0, 0.0), &OrbitContext::default(), &[], 0.5, hz);
            (held, orbit.yaw())
        };
        let (a, b) = (turned(30.0), turned(144.0));
        assert!(angle_delta(a.0, b.0).abs() < 1.0e-4, "held yaw {} at 30 Hz vs {} at 144 Hz", a.0, b.0);
        assert!(angle_delta(a.1, b.1).abs() < 1.0e-4, "final yaw {} at 30 Hz vs {} at 144 Hz", a.1, b.1);
        assert!(a.1 < -0.3, "looking right must turn yaw negative, got {}", a.1);
    }

    #[test]
    fn mouse_look_does_not_depend_on_frame_time() {
        let total = Vec2::new(0.6, 0.2);
        let turned = |frames: usize, dt: f32| {
            let mut orbit = OrbitState::new(0.0, 0.2);
            let input = CameraInput { look_delta: total / frames as f32, ..Default::default() };
            for _ in 0..frames {
                orbit.step(
                    &input,
                    &CameraInputSettings::default(),
                    &shape(),
                    &OrbitParams::default(),
                    &OrbitContext::default(),
                    dt,
                );
            }
            (orbit.yaw(), orbit.pitch())
        };
        let a = turned(10, 1.0 / 30.0);
        let b = turned(10, 1.0 / 144.0);
        assert!((a.0 - b.0).abs() < 1.0e-6 && (a.1 - b.1).abs() < 1.0e-6, "{a:?} vs {b:?}");
        assert!((a.0 + 0.6).abs() < 1.0e-5, "0.6 rad right is yaw -0.6, got {}", a.0);
    }

    #[test]
    fn recentring_waits_for_the_delay_after_input() {
        let moving = OrbitContext { heading: Some(1.0), speed: 4.0, facing: Some(1.0), ..Default::default() };
        let mut orbit = OrbitState::new(0.0, 0.2);
        run(&mut orbit, &stick(0.0, 0.0), &moving, &[], 1.4, 60.0);
        assert!(orbit.yaw().abs() < 1.0e-6, "recentred before the delay: yaw {}", orbit.yaw());
        run(&mut orbit, &stick(0.0, 0.0), &moving, &[], 2.0, 60.0);
        assert!(orbit.yaw() > 0.5, "should swing toward the heading after the delay, {}", orbit.yaw());
    }

    #[test]
    fn recentring_starts_at_the_same_moment_at_any_frame_rate() {
        // A delay that ends mid-frame at 30 Hz: crediting the whole frame
        // that crosses it would start the swing up to 1/30 s early.
        let params = OrbitParams { recenter_delay: 1.45, ..OrbitParams::default() };
        let moving = OrbitContext { heading: Some(1.5), speed: 4.0, ..Default::default() };
        let yaw_at = |hz: f32| {
            let mut orbit = OrbitState::new(0.0, 0.2);
            for i in 0..(3.0 * hz).round() as usize {
                let t = (i as f32 + 0.5) / hz;
                let input = if t < 0.5 { stick(0.5, 0.0) } else { stick(0.0, 0.0) };
                orbit.step(&input, &CameraInputSettings::default(), &shape(), &params, &moving, 1.0 / hz);
            }
            orbit.yaw()
        };
        let (a, b) = (yaw_at(30.0), yaw_at(144.0));
        assert!(angle_delta(a, b).abs() < 2.0e-3, "yaw {a} at 30 Hz vs {b} at 144 Hz");
    }

    #[test]
    fn a_ceiling_cap_eases_the_pitch_down_and_blocks_looking_further_up() {
        let mut orbit = OrbitState::new(0.0, 0.8);
        let capped = OrbitContext { pitch_cap: Some(0.3), ..Default::default() };
        let mut previous = orbit.pitch();
        for _ in 0..90 {
            run(&mut orbit, &stick(0.0, -1.0), &capped, &[], 1.0 / 60.0, 60.0);
            let step = previous - orbit.pitch();
            assert!(step < 0.05, "the pitch must ease, not snap: dropped {step} rad in a frame");
            previous = orbit.pitch();
        }
        assert!(orbit.pitch() < 0.31, "pitch {} must settle under the 0.3 cap", orbit.pitch());
    }

    #[test]
    fn walking_sideways_or_toward_the_camera_does_not_pull_it_round() {
        // Camera at yaw 0; walking at yaw ±90° is sideways, at π toward it.
        for heading in [std::f32::consts::FRAC_PI_2, -std::f32::consts::FRAC_PI_2, std::f32::consts::PI] {
            let context = OrbitContext { heading: Some(heading), speed: 4.0, ..Default::default() };
            let mut orbit = OrbitState::new(0.0, 0.2);
            run(&mut orbit, &stick(0.0, 0.0), &context, &[], 5.0, 60.0);
            assert!(orbit.yaw().abs() < 1.0e-6, "heading {heading}: the camera swung to {}", orbit.yaw());
        }
    }

    #[test]
    fn standing_still_never_recentres() {
        let standing = OrbitContext { heading: Some(1.0), speed: 0.0, facing: Some(1.0), ..Default::default() };
        let mut orbit = OrbitState::new(0.0, 0.2);
        run(&mut orbit, &stick(0.0, 0.0), &standing, &[], 10.0, 60.0);
        assert!(orbit.yaw().abs() < 1.0e-6, "a standing character must not pull the camera");
    }

    #[test]
    fn the_recentre_button_swings_behind_the_character_quickly() {
        let standing = OrbitContext { facing: Some(2.0), ..Default::default() };
        let mut orbit = OrbitState::new(0.0, 0.2);
        let press = CameraInput { recenter: true, ..Default::default() };
        run(&mut orbit, &press, &standing, &[], 1.0 / 60.0, 60.0);
        run(&mut orbit, &stick(0.0, 0.0), &standing, &[], 1.0, 60.0);
        assert!(angle_delta(orbit.yaw(), 2.0).abs() < 0.01, "yaw {}", orbit.yaw());
    }

    #[test]
    fn a_goal_owns_yaw_and_player_input_becomes_a_decaying_offset() {
        let goal = OrbitGoal { priority: 10, yaw: Some(1.2), pitch: None, halflife: 0.1 };
        let mut orbit = OrbitState::new(0.0, 0.2);
        run(&mut orbit, &stick(0.0, 0.0), &OrbitContext::default(), &[goal], 1.0, 60.0);
        assert!(angle_delta(orbit.yaw(), 1.2).abs() < 0.01, "goal not reached: {}", orbit.yaw());
        // Push away; the goal keeps owning the base, the push is an offset.
        run(&mut orbit, &stick(-1.0, 0.0), &OrbitContext::default(), &[goal], 0.3, 60.0);
        assert!(orbit.yaw_offset > 0.1, "player push should show as an offset");
        // Let go: after the delay the offset decays back to the goal.
        run(&mut orbit, &stick(0.0, 0.0), &OrbitContext::default(), &[goal], 6.0, 60.0);
        assert!(angle_delta(orbit.yaw(), 1.2).abs() < 0.02, "offset should decay, yaw {}", orbit.yaw());
    }

    #[test]
    fn ending_a_goal_does_not_jump_the_camera() {
        let goal = OrbitGoal { priority: 10, yaw: Some(1.2), pitch: None, halflife: 0.1 };
        let mut orbit = OrbitState::new(0.0, 0.2);
        run(&mut orbit, &stick(-1.0, 0.0), &OrbitContext::default(), &[goal], 0.5, 60.0);
        let before = orbit.yaw();
        run(&mut orbit, &CameraInput::default(), &OrbitContext::default(), &[], 1.0 / 60.0, 60.0);
        assert!(angle_delta(before, orbit.yaw()).abs() < 0.08, "{before} → {}", orbit.yaw());
        assert_eq!(orbit.yaw_offset, 0.0, "the offset folds into the base");
    }

    #[test]
    fn the_higher_priority_goal_owns_the_axis() {
        let low = OrbitGoal { priority: 1, yaw: Some(-1.0), pitch: None, halflife: 0.05 };
        let high = OrbitGoal { priority: 5, yaw: Some(1.0), pitch: None, halflife: 0.05 };
        let mut orbit = OrbitState::new(0.0, 0.2);
        run(&mut orbit, &CameraInput::default(), &OrbitContext::default(), &[low, high], 1.0, 60.0);
        assert!(angle_delta(orbit.yaw(), 1.0).abs() < 0.01);
    }
}
