//! The camera's input contract and the player's input preferences.
//!
//! [`CameraInput`] is the only way anything steers the camera. A device
//! mapping system writes it from mouse and gamepad; tests, trace replay and
//! AI write it directly. Nothing downstream reads a device.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

/// One frame of camera input.
///
/// Sign convention: `x` positive looks right, `y` positive looks up.
#[derive(Component, Debug, Clone, Default, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(Component)]
pub struct CameraInput {
    /// Mouse look this frame, radians (already scaled by mouse sensitivity).
    /// Never multiplied by `dt`: a mouse reports a displacement, not a rate.
    pub look_delta: Vec2,
    /// Raw stick deflection in `-1..=1`; shaped, scaled to a rate and
    /// integrated over `dt` by the orbit using [`CameraInputSettings`].
    pub look_stick: Vec2,
    /// Zoom steps this frame; positive zooms in.
    pub zoom: f32,
    /// Recentre behind the character (held or pressed this frame).
    pub recenter: bool,
    /// Lock-on toggle pressed this frame.
    pub lock_on: bool,
    /// Lock-on switch stick, `-1..=1`.
    pub switch: Vec2,
    /// Swap the shoulder side, pressed this frame.
    pub shoulder_swap: bool,
}

impl CameraInput {
    /// True if this frame carries any look or zoom intent.
    pub fn has_look(&self, deadzone: f32) -> bool {
        self.look_delta != Vec2::ZERO || self.look_stick.length() > deadzone || self.zoom != 0.0
    }
}

/// A player's input preferences, per camera so split-screen players each
/// have their own. Angles in radians.
#[derive(Component, Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(Component)]
pub struct CameraInputSettings {
    /// Radians of look per mouse count, used by the device mapping.
    pub mouse_sensitivity: f32,
    /// Look rate at full stick deflection, radians per second (yaw, pitch).
    pub stick_max_rate: Vec2,
    pub stick_deadzone: f32,
    /// Power-curve exponent for stick response; 1 is linear.
    pub stick_curve: f32,
    /// Half-life of the stick's look rate toward the deflected rate:
    /// acceleration and deceleration.
    pub stick_accel_halflife: f32,
    pub invert_x: bool,
    pub invert_y: bool,
    /// Comfort scale on camera shake; 0 removes it.
    pub shake_scale: f32,
}

impl Default for CameraInputSettings {
    fn default() -> Self {
        Self {
            mouse_sensitivity: 0.003,
            stick_max_rate: Vec2::new(200f32.to_radians(), 120f32.to_radians()),
            stick_deadzone: 0.15,
            stick_curve: 2.0,
            stick_accel_halflife: 0.04,
            invert_x: false,
            invert_y: false,
            shake_scale: 1.0,
        }
    }
}
