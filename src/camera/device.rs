//! Mouse, keyboard and gamepad → [`CameraInput`].
//!
//! Only cameras with a [`CameraDeviceInput`] are driven from devices; a
//! camera without one is steered entirely by whoever writes its
//! `CameraInput` (a test, a replay, an AI director).

use super::input::{CameraInput, CameraInputSettings};
use bevy::input::gamepad::{Gamepad, GamepadButton};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit};
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow};

/// When mouse movement turns the camera.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Reflect)]
pub enum MouseLook {
    Always,
    /// While this button is held (an editor-style camera).
    WhileHeld(MouseButton),
    /// While the primary window has grabbed the cursor.
    WhileGrabbed,
    Off,
}

/// Maps devices onto this camera's [`CameraInput`].
#[derive(Component, Debug, Clone, Reflect)]
#[reflect(Component)]
pub struct CameraDeviceInput {
    pub mouse_look: MouseLook,
    /// The gamepad driving this camera; `None` takes the first one found.
    pub gamepad: Option<Entity>,
    pub recenter_keys: Vec<KeyCode>,
    pub recenter_mouse: Option<MouseButton>,
    pub shoulder_swap_keys: Vec<KeyCode>,
}

impl Default for CameraDeviceInput {
    fn default() -> Self {
        Self {
            mouse_look: MouseLook::WhileGrabbed,
            gamepad: None,
            recenter_keys: vec![KeyCode::KeyR],
            recenter_mouse: Some(MouseButton::Middle),
            shoulder_swap_keys: vec![KeyCode::Tab],
        }
    }
}

/// Scroll in pixels per zoom step, for pixel-unit (touchpad) scrolling.
const PIXELS_PER_STEP: f32 = 40.0;

/// Skips itself in an app without Bevy's input plugin (a headless test).
#[allow(clippy::too_many_arguments)]
pub fn map_devices(
    mut cameras: Query<(&CameraDeviceInput, &CameraInputSettings, &mut CameraInput)>,
    motion: If<Res<AccumulatedMouseMotion>>,
    scroll: If<Res<AccumulatedMouseScroll>>,
    mouse: If<Res<ButtonInput<MouseButton>>>,
    keys: If<Res<ButtonInput<KeyCode>>>,
    gamepads: Query<(Entity, &Gamepad)>,
    cursor: Query<&CursorOptions, With<PrimaryWindow>>,
) {
    let grabbed = cursor.iter().any(|options| options.grab_mode != CursorGrabMode::None);
    for (device, settings, mut input) in &mut cameras {
        let looking = match device.mouse_look {
            MouseLook::Always => true,
            MouseLook::WhileHeld(button) => mouse.pressed(button),
            MouseLook::WhileGrabbed => grabbed,
            MouseLook::Off => false,
        };
        // Screen y grows downward; look up is positive.
        let look_delta = if looking {
            Vec2::new(motion.delta.x, -motion.delta.y) * settings.mouse_sensitivity
        } else {
            Vec2::ZERO
        };
        let mut zoom = match scroll.unit {
            MouseScrollUnit::Line => scroll.delta.y,
            MouseScrollUnit::Pixel => scroll.delta.y / PIXELS_PER_STEP,
        };
        let mut recenter = device.recenter_keys.iter().any(|key| keys.just_pressed(*key))
            || device.recenter_mouse.is_some_and(|button| mouse.just_pressed(button));
        let mut shoulder_swap = device.shoulder_swap_keys.iter().any(|key| keys.just_pressed(*key));

        let pad = match device.gamepad {
            Some(entity) => gamepads.get(entity).ok().map(|(_, pad)| pad),
            None => gamepads.iter().next().map(|(_, pad)| pad),
        };
        let mut look_stick = Vec2::ZERO;
        if let Some(pad) = pad {
            look_stick = pad.right_stick();
            recenter |= pad.just_pressed(GamepadButton::RightThumb);
            shoulder_swap |= pad.just_pressed(GamepadButton::LeftThumb);
            if pad.just_pressed(GamepadButton::DPadUp) {
                zoom += 1.0;
            }
            if pad.just_pressed(GamepadButton::DPadDown) {
                zoom -= 1.0;
            }
        }

        // Everything but `move_held`, which the movement controller owns.
        input.look_delta = look_delta;
        input.look_stick = look_stick;
        input.zoom = zoom;
        input.recenter = recenter;
        input.shoulder_swap = shoulder_swap;
    }
}
