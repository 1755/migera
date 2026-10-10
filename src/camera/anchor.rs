//! The anchor stage: where the camera's pivot is.
//!
//! The pivot follows a goal point (the target's position plus its eye-height
//! offset) with a critically damped spring that is solved exactly over the
//! interval the goal moved in ([`spring_vec3_tracking`]). The follow is then
//! identical at any frame rate: a staircase spring would trail ~`v·dt/2`
//! further at 30 Hz than at 144 Hz.
//!
//! On top of the spring:
//! - **Leash.** The horizontal lag is capped, so the camera can't fall
//!   further behind the faster the character runs (Gothic's "zooms out when
//!   running"). The vertical lag has its own leash, so a fast fall can't
//!   leave the character below the frame.
//! - **Air deadband.** While airborne the pivot holds the take-off height
//!   until the goal leaves a band above it, and tracks any drop below it.
//!   Jumps don't bob the camera; falls are followed (Nesky #28, #45).
//! - **Teleport.** A goal jump past `teleport_distance` snaps the pivot and
//!   reports a cut.
//! - **Rebase.** When the anchor itself changes (a mount, a target switch, a
//!   death handing off to a ragdoll), the follow restarts on the new goal and
//!   the difference from the old pivot (position and velocity) becomes an
//!   offset that decays on its own, softer spring. That is inertialization:
//!   the hand-off is velocity-continuous, and its pace is the rebase spring's
//!   rather than the stiff follow spring's.
//! - **Look-ahead.** An optional offset along the goal's velocity, sprung.

use crate::math::spring::{
    spring_scalar_tracking, spring_vec3, spring_vec3_tracking, tracking_lag, SpringParams,
};
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
pub struct AnchorParams {
    /// Horizontal follow spring.
    pub horizontal: SpringParams,
    /// Vertical follow spring.
    pub vertical: SpringParams,
    /// Most the pivot may trail the goal horizontally, metres.
    pub leash: f32,
    /// Most the pivot may trail its vertical goal, metres: a fall at 10 m/s
    /// would otherwise leave the character below the frame.
    pub vertical_leash: f32,
    /// While airborne, how far the goal may rise above the take-off height
    /// before the pivot follows it up, metres.
    pub air_rise_band: f32,
    /// While airborne, how far the goal may drop below the take-off height
    /// before the pivot follows it down, metres. 0 tracks every fall.
    pub air_drop_band: f32,
    /// A goal jump larger than this in one frame is a teleport, metres.
    pub teleport_distance: f32,
    /// Seconds of goal velocity the pivot leads by; 0 disables look-ahead.
    pub look_ahead_time: f32,
    /// Cap on the look-ahead offset, metres.
    pub look_ahead_max: f32,
    pub look_ahead_spring: SpringParams,
    /// How the pivot travels from an old anchor to a new one.
    pub rebase: SpringParams,
}

impl Default for AnchorParams {
    fn default() -> Self {
        let follow = |halflife| SpringParams { halflife, damping_ratio: 1.0, max_speed: 1.0e3 };
        Self {
            horizontal: follow(0.035),
            vertical: follow(0.08),
            leash: 0.5,
            vertical_leash: 0.6,
            air_rise_band: 1.2,
            air_drop_band: 0.0,
            teleport_distance: 5.0,
            look_ahead_time: 0.0,
            look_ahead_max: 0.6,
            look_ahead_spring: follow(0.3),
            rebase: follow(0.25),
        }
    }
}

/// What the anchor stage reads about its target this frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
pub struct AnchorInput {
    /// The point the pivot follows, world space.
    pub goal: Vec3,
    pub grounded: bool,
    /// The anchor changed this frame: rebase instead of treating the jump as
    /// motion or a teleport.
    pub rebase: bool,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Reflect)]
pub struct PivotState {
    /// The followed point, before look-ahead.
    pub position: Vec3,
    pub velocity: Vec3,
    /// The goal last frame; `None` before the first frame.
    pub last_goal: Option<Vec3>,
    /// The vertical goal last frame (the held height while airborne).
    pub last_held_y: f32,
    /// Take-off height while airborne.
    pub takeoff_y: Option<f32>,
    pub look_ahead: Vec3,
    pub look_ahead_velocity: Vec3,
    /// The goal's velocity over the last frame.
    pub goal_velocity: Vec3,
    /// What is left of an anchor hand-off, decaying to zero.
    pub rebase_offset: Vec3,
    pub rebase_velocity: Vec3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnchorOutput {
    pub pivot: Vec3,
    /// The pivot snapped: downstream stages should snap too, and controls
    /// must not be remapped.
    pub cut: bool,
}

impl PivotState {
    /// The pivot including look-ahead and any hand-off still in progress.
    pub fn pivot(&self) -> Vec3 {
        self.position + self.look_ahead + self.rebase_offset
    }

    /// The pivot's world velocity, hand-off included (look-ahead excluded).
    pub fn pivot_velocity(&self) -> Vec3 {
        self.velocity + self.rebase_velocity
    }

    fn snap(&mut self, goal: Vec3) {
        *self = Self {
            position: goal,
            last_goal: Some(goal),
            last_held_y: goal.y,
            ..Self::default()
        };
    }

    /// Advances the pivot by `dt` seconds of *virtual* time.
    pub fn step(&mut self, input: &AnchorInput, params: &AnchorParams, dt: f32) -> AnchorOutput {
        let goal = input.goal;
        let Some(mut last_goal) = self.last_goal else {
            self.snap(goal);
            return AnchorOutput { pivot: self.pivot(), cut: true };
        };
        if input.rebase {
            // The new anchor has no history: start it where it is, carrying
            // the old anchor's velocity estimate so a mount moving with the
            // rider doesn't read as a stop. The follow restarts on the new
            // goal, and the old pivot's position and velocity, relative to
            // it, become the offset that decays.
            last_goal = goal - self.goal_velocity * dt;
            // Restart the follow at its steady trail behind the moving goal,
            // not on the goal itself, or the stiff follow spring yanks it.
            let v = self.goal_velocity;
            let steady = last_goal
                + Vec3::new(
                    tracking_lag(v.x, &params.horizontal),
                    tracking_lag(v.y, &params.vertical),
                    tracking_lag(v.z, &params.horizontal),
                );
            self.rebase_offset += self.position - steady;
            self.rebase_velocity += self.velocity - self.goal_velocity;
            self.position = steady;
            self.velocity = self.goal_velocity;
            self.last_held_y = last_goal.y;
            self.takeoff_y = None;
        } else if goal.distance(last_goal) > params.teleport_distance {
            self.snap(goal);
            return AnchorOutput { pivot: self.pivot(), cut: true };
        }
        if dt <= 0.0 {
            return AnchorOutput { pivot: self.pivot(), cut: false };
        }

        let goal_velocity = (goal - last_goal) / dt;

        // Horizontal: exact tracking of a goal moving linearly over the step.
        let flat = Vec3::new(1.0, 0.0, 1.0);
        let (horizontal, horizontal_velocity) = spring_vec3_tracking(
            self.position * flat,
            self.velocity * flat,
            last_goal * flat,
            goal_velocity * flat,
            &params.horizontal,
            dt,
        );

        // Vertical: the goal height, or the held take-off height in the air.
        let held_y = if input.grounded {
            self.takeoff_y = None;
            goal.y
        } else {
            let takeoff = self.takeoff_y.unwrap_or(last_goal.y);
            let held = takeoff.clamp(goal.y - params.air_rise_band, goal.y + params.air_drop_band);
            // Ratchet: once the goal has dragged the hold, it stays dragged.
            self.takeoff_y = Some(held);
            held
        };
        let (y, vy) = spring_scalar_tracking(
            self.position.y,
            self.velocity.y,
            self.last_held_y,
            (held_y - self.last_held_y) / dt,
            &params.vertical,
            dt,
        );

        let held_velocity = (held_y - self.last_held_y) / dt;
        let (y, vy) = if (y - held_y).abs() > params.vertical_leash {
            let leashed = held_y + (y - held_y).signum() * params.vertical_leash;
            // Keep the spring from pushing further out against the leash.
            let outward = (vy - held_velocity) * (y - held_y).signum();
            (leashed, if outward > 0.0 { held_velocity } else { vy })
        } else {
            (y, vy)
        };

        let mut position = Vec3::new(horizontal.x, y, horizontal.z);
        let mut velocity = Vec3::new(horizontal_velocity.x, vy, horizontal_velocity.z);

        // Leash the horizontal lag, removing the outward part of the
        // relative velocity so the spring doesn't keep pushing against it.
        let lag = (position - goal) * flat;
        if lag.length() > params.leash {
            let direction = lag.normalize();
            position = goal * flat + direction * params.leash + Vec3::Y * position.y;
            let relative = (velocity - goal_velocity) * flat;
            let outward = relative.dot(direction);
            if outward > 0.0 {
                velocity -= direction * outward;
            }
        }

        // Look-ahead along the goal's horizontal velocity.
        let lead = ((goal_velocity * flat) * params.look_ahead_time)
            .clamp_length_max(params.look_ahead_max);
        (self.look_ahead, self.look_ahead_velocity) = spring_vec3(
            self.look_ahead,
            self.look_ahead_velocity,
            lead,
            &params.look_ahead_spring,
            dt,
        );
        (self.rebase_offset, self.rebase_velocity) =
            spring_vec3(self.rebase_offset, self.rebase_velocity, Vec3::ZERO, &params.rebase, dt);

        self.position = position;
        self.velocity = velocity;
        self.last_goal = Some(goal);
        self.last_held_y = held_y;
        self.goal_velocity = goal_velocity;
        AnchorOutput { pivot: self.pivot(), cut: false }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grounded(goal: Vec3) -> AnchorInput {
        AnchorInput { goal, grounded: true, rebase: false }
    }

    #[test]
    fn the_first_frame_snaps_and_reports_a_cut() {
        let mut pivot = PivotState::default();
        let out = pivot.step(&grounded(Vec3::new(3.0, 1.6, -2.0)), &AnchorParams::default(), 1.0 / 60.0);
        assert!(out.cut);
        assert_eq!(out.pivot, Vec3::new(3.0, 1.6, -2.0));
    }

    #[test]
    fn the_leash_caps_horizontal_lag_while_running() {
        let params = AnchorParams::default();
        let mut pivot = PivotState::default();
        let dt = 1.0 / 60.0;
        let mut worst: f32 = 0.0;
        for i in 0..240 {
            let goal = Vec3::new(0.0, 1.6, -12.0 * i as f32 * dt); // a 12 m/s sprint
            pivot.step(&grounded(goal), &params, dt);
            worst = worst.max(((pivot.position - goal) * Vec3::new(1.0, 0.0, 1.0)).length());
        }
        assert!(worst <= params.leash + 1.0e-4, "lag reached {worst} m, leash {}", params.leash);
    }

    #[test]
    fn without_the_leash_a_sprint_would_trail_further() {
        // Proves the leash test above can fail: the spring alone trails past it.
        let params = AnchorParams { leash: 100.0, ..AnchorParams::default() };
        let mut pivot = PivotState::default();
        let dt = 1.0 / 60.0;
        let mut goal = Vec3::ZERO;
        for i in 0..240 {
            goal = Vec3::new(0.0, 1.6, -12.0 * i as f32 * dt);
            pivot.step(&grounded(goal), &params, dt);
        }
        assert!((pivot.position - goal).length() > AnchorParams::default().leash);
    }

    #[test]
    fn a_jump_inside_the_air_band_does_not_move_the_pivot_vertically() {
        let params = AnchorParams::default();
        let mut pivot = PivotState::default();
        let dt = 1.0 / 60.0;
        for _ in 0..30 {
            pivot.step(&grounded(Vec3::new(0.0, 1.6, 0.0)), &params, dt);
        }
        // A 0.8 m hop over 0.8 s, inside the 1.2 m band.
        for i in 1..=48 {
            let t = i as f32 * dt;
            let height = 4.0 * 0.8 * (t / 0.8) * (1.0 - t / 0.8);
            let input = AnchorInput { goal: Vec3::new(0.0, 1.6 + height, 0.0), grounded: false, rebase: false };
            pivot.step(&input, &params, dt);
            assert!(
                (pivot.position.y - 1.6).abs() < 1.0e-4,
                "frame {i}: pivot rose to {} during a hop", pivot.position.y,
            );
        }
    }

    #[test]
    fn a_fall_below_take_off_is_tracked() {
        let params = AnchorParams::default();
        let mut pivot = PivotState::default();
        let dt = 1.0 / 60.0;
        for _ in 0..30 {
            pivot.step(&grounded(Vec3::new(0.0, 5.0, 0.0)), &params, dt);
        }
        // Step off a ledge: free fall for one second.
        for i in 1..=60 {
            let t = i as f32 * dt;
            let input = AnchorInput { goal: Vec3::new(0.0, 5.0 - 4.9 * t * t, 0.0), grounded: false, rebase: false };
            pivot.step(&input, &params, dt);
        }
        assert!(pivot.position.y < 1.0, "the pivot must follow a 4.9 m fall, still at {}", pivot.position.y);
    }

    #[test]
    fn a_teleport_snaps_and_cuts() {
        let params = AnchorParams::default();
        let mut pivot = PivotState::default();
        pivot.step(&grounded(Vec3::ZERO), &params, 1.0 / 60.0);
        let out = pivot.step(&grounded(Vec3::new(50.0, 0.0, 0.0)), &params, 1.0 / 60.0);
        assert!(out.cut && out.pivot == Vec3::new(50.0, 0.0, 0.0));
    }

    #[test]
    fn an_anchor_switch_is_velocity_continuous() {
        let params = AnchorParams::default();
        let mut pivot = PivotState::default();
        let dt = 1.0 / 60.0;
        // Follow a target walking along -Z at 2 m/s.
        let mut previous_velocity = Vec3::ZERO;
        let mut previous_position = Vec3::ZERO;
        for i in 0..60 {
            pivot.step(&grounded(Vec3::new(0.0, 1.6, -2.0 * i as f32 * dt)), &params, dt);
            previous_velocity = pivot.pivot_velocity();
            previous_position = pivot.pivot();
        }
        // Switch to a mount standing 3 m to the side.
        let mount = Vec3::new(3.0, 2.2, -2.0);
        let out = pivot.step(&AnchorInput { goal: mount, grounded: true, rebase: true }, &params, dt);
        assert!(!out.cut, "a rebase is not a cut");
        let jump = pivot.pivot().distance(previous_position);
        assert!(jump < 0.06, "a rebase must not jump the pivot, moved {jump} m in one frame");
        // The stiff follow spring would accelerate at ~ω²·3 m ≈ 1200 m/s²;
        // the hand-off runs on the softer rebase spring.
        let accel = (pivot.pivot_velocity() - previous_velocity).length() / dt;
        assert!(accel < 50.0, "the hand-off must be gentle, accel {accel} m/s²");
        for _ in 0..240 {
            pivot.step(&grounded(mount), &params, dt);
        }
        assert!(pivot.pivot().distance(mount) < 0.01, "and settle on the new anchor");
    }

    #[test]
    fn a_paused_clock_freezes_the_pivot() {
        let params = AnchorParams::default();
        let mut pivot = PivotState::default();
        pivot.step(&grounded(Vec3::ZERO), &params, 1.0 / 60.0);
        pivot.step(&grounded(Vec3::new(0.0, 0.0, -0.05)), &params, 1.0 / 60.0);
        let frozen = pivot.position;
        for _ in 0..10 {
            pivot.step(&grounded(Vec3::new(0.0, 0.0, -0.05)), &params, 0.0);
        }
        assert_eq!(pivot.position, frozen);
    }
}
