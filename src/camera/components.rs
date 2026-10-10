//! The camera's ECS surface: what a game puts on entities and reads back.
//!
//! On the camera (a `Camera3d`):
//! - [`ThirdPersonCamera`]: which entity to follow and the tuning. It
//!   requires everything else, so `(Camera3d::default(),
//!   ThirdPersonCamera::follow(player))` is a complete camera.
//! - [`CameraInput`](super::CameraInput): the only input contract.
//! - [`CameraModeRequests`], [`CameraGoals`]: mode switches and orbit goals
//!   for this frame, written by gameplay before [`CameraSet::Rig`](super::CameraSet).
//! - [`CameraDesiredPose`]: the rig's pose before collision. A system
//!   ordered between `CameraSet::Rig` and `CameraSet::Collide` may edit it:
//!   the ECS form of a Cinemachine extension or an Unreal camera modifier.
//! - [`CameraView`]: the published result, including the yaw a player
//!   controller should steer against.
//!
//! On the followed entity: optionally [`CameraTarget`] (its pivot offset)
//! and [`CameraTargetState`] (grounded, facing), which the walker bridge
//! fills for a walking character.

use super::collision::ResolvedPose;
use super::pipeline::{CameraConfig, CameraOutput, CameraRig};
use super::orbit::OrbitGoal;
use super::rig::DesiredPose;
use super::stack::ModeRequest;
use super::trace::CameraTrace;
use super::{CameraClock, CameraInput, CameraInputSettings};
use bevy::prelude::*;

/// A third-person camera following `target`.
#[derive(Component, Debug, Clone, Reflect)]
#[reflect(Component)]
#[require(
    CameraInput,
    CameraInputSettings,
    CameraClock,
    CameraRigState,
    CameraModeRequests,
    CameraGoals,
    CameraDesiredPose,
    CameraResolvedPose,
    CameraView
)]
pub struct ThirdPersonCamera {
    /// The entity followed. Changing it hands the pivot over smoothly
    /// (an inertialized rebase), never a cut.
    pub target: Entity,
    pub config: CameraConfig,
}

impl ThirdPersonCamera {
    /// Follows `target` with the default tuning.
    pub fn follow(target: Entity) -> Self {
        Self { target, config: CameraConfig::default() }
    }
}

/// How the followed entity presents itself to the camera.
#[derive(Component, Debug, Clone, PartialEq, Reflect)]
#[reflect(Component)]
#[require(CameraTargetState)]
pub struct CameraTarget {
    /// The pivot's offset from the entity's root, world space: eye height.
    pub pivot_offset: Vec3,
    /// Colliders that belong to the target (its capsule, a carried shield)
    /// and so never block its camera. Ragdoll bodies are excluded by layer
    /// already.
    pub exclude: Vec<Entity>,
}

impl Default for CameraTarget {
    fn default() -> Self {
        Self { pivot_offset: Vec3::new(0.0, 1.6, 0.0), exclude: Vec::new() }
    }
}

/// A collider that never blocks or occludes any camera, whatever its
/// layers: for props that cannot change layer.
#[derive(Component, Debug, Clone, Copy, Default, Reflect)]
#[reflect(Component)]
pub struct CameraIgnore;

/// What the camera knows about its target beyond its transform. Filled by
/// a bridge (the walker's, `bridge.rs`) or by the game.
#[derive(Component, Debug, Clone, Copy, PartialEq, Reflect)]
#[reflect(Component)]
pub struct CameraTargetState {
    /// On the ground (true) or in the air: the pivot holds its height
    /// through a jump while airborne.
    pub grounded: bool,
    /// Camera yaw that looks the way the target faces, for the recentre
    /// button.
    pub facing_yaw: Option<f32>,
}

impl Default for CameraTargetState {
    fn default() -> Self {
        Self { grounded: true, facing_yaw: None }
    }
}

/// Mode switches for this frame, applied in order and then cleared.
#[derive(Component, Debug, Clone, Default, PartialEq, Reflect)]
#[reflect(Component)]
pub struct CameraModeRequests(pub Vec<ModeRequest>);

/// Orbit goals active this frame (lock-on, dialog, assist). Write them
/// every frame they apply; they are cleared once used.
#[derive(Component, Debug, Clone, Default, PartialEq, Reflect)]
#[reflect(Component)]
pub struct CameraGoals(pub Vec<OrbitGoal>);

/// The camera's own state: the pure pipeline's rig, built on first use.
#[derive(Component, Debug, Clone, Default, Reflect)]
#[reflect(Component)]
pub struct CameraRigState {
    pub rig: Option<CameraRig>,
    /// The target followed last frame, to detect a hand-off.
    pub last_target: Option<Entity>,
    pub output: Option<CameraOutput>,
}

/// The rig's pose this frame, before collision.
#[derive(Component, Debug, Clone, Copy, PartialEq, Reflect)]
#[reflect(Component)]
pub struct CameraDesiredPose(pub DesiredPose);

impl Default for CameraDesiredPose {
    fn default() -> Self {
        Self(DesiredPose {
            pivot: Vec3::ZERO,
            shoulder: Vec3::ZERO,
            eye: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            fov: std::f32::consts::FRAC_PI_4,
            distance: 0.0,
        })
    }
}

/// The pose after collision: what is written to the camera.
#[derive(Component, Debug, Clone, Copy, PartialEq, Reflect)]
#[reflect(Component)]
pub struct CameraResolvedPose(pub ResolvedPose);

impl Default for CameraResolvedPose {
    fn default() -> Self {
        Self(ResolvedPose::unresolved(&CameraDesiredPose::default().0))
    }
}

/// What the camera did this frame: read this, not the camera's
/// `Transform`, from gameplay.
#[derive(Component, Debug, Clone, Copy, PartialEq, Default, Reflect)]
#[reflect(Component)]
pub struct CameraView {
    pub eye: Vec3,
    pub rotation: Quat,
    /// Vertical FOV, radians.
    pub fov: f32,
    pub pivot: Vec3,
    /// The yaw the camera looks along.
    pub view_yaw: f32,
    pub pitch: f32,
    /// The yaw a player controller should map movement input against.
    /// Equal to `view_yaw`, except after a cut while the player keeps
    /// holding a direction: then it holds its pre-cut value, so a cut never
    /// turns a held "forward" into a different direction (Nesky #18).
    pub control_yaw: f32,
    /// `control_yaw` is holding a pre-cut value.
    pub latched: bool,
    /// Seconds the latch has held.
    pub latched_for: f32,
    /// The camera snapped this frame.
    pub cut_this_frame: bool,
    /// Boom length in use and the one the mode wants, metres.
    pub distance: f32,
    pub desired_distance: f32,
    /// How much to fade the target: 0 visible, 1 gone (the eye is inside
    /// its personal space).
    pub target_fade: f32,
    /// How far into the high fallback view, 0-1.
    pub fallback: f32,
}

impl CameraView {
    /// Horizontal forward for movement input (along `control_yaw`).
    pub fn control_forward(&self) -> Vec3 {
        Quat::from_rotation_y(self.control_yaw) * Vec3::NEG_Z
    }

    /// Horizontal right for movement input.
    pub fn control_right(&self) -> Vec3 {
        Quat::from_rotation_y(self.control_yaw) * Vec3::X
    }
}

/// Records every frame this camera consumes, for replay as a test
/// (`CameraTrace::replay`).
#[derive(Component, Debug, Clone, Default)]
pub struct CameraRecorder {
    pub trace: CameraTrace,
}

/// Settings for the control-yaw latch.
#[derive(Debug, Clone, Copy)]
pub struct LatchParams {
    /// After this long latched, `control_yaw` eases to the view even while
    /// the direction is still held, seconds.
    pub timeout: f32,
    /// Half-life of that easing, seconds.
    pub release_halflife: f32,
}

impl Default for LatchParams {
    fn default() -> Self {
        Self { timeout: 2.0, release_halflife: 0.3 }
    }
}

/// Advances the control-yaw latch one frame. Pure, so it is tested without a
/// `World`.
pub fn update_latch(
    view: &mut CameraView,
    view_yaw: f32,
    cut: bool,
    move_held: bool,
    params: &LatchParams,
    dt: f32,
) {
    use crate::math::angle::damp_angle;
    if cut && move_held && !view.latched {
        // Hold the yaw from before the cut.
        view.latched = true;
        view.latched_for = 0.0;
    }
    if view.latched {
        if !move_held {
            view.latched = false;
        } else {
            view.latched_for += dt;
            if view.latched_for > params.timeout {
                view.control_yaw =
                    damp_angle(view.control_yaw, view_yaw, params.release_halflife, dt);
            }
        }
    }
    if !view.latched {
        view.control_yaw = view_yaw;
    }
    view.view_yaw = view_yaw;
    view.cut_this_frame = cut;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cut_while_a_direction_is_held_keeps_the_control_yaw() {
        let params = LatchParams::default();
        let mut view = CameraView { control_yaw: 0.3, view_yaw: 0.3, ..default() };
        update_latch(&mut view, 2.5, true, true, &params, 1.0 / 60.0);
        assert_eq!(view.control_yaw, 0.3, "a held direction must not be remapped by a cut");
        for _ in 0..30 {
            update_latch(&mut view, 2.5, false, true, &params, 1.0 / 60.0);
        }
        assert_eq!(view.control_yaw, 0.3, "and keep holding while the direction is held");
        update_latch(&mut view, 2.5, false, false, &params, 1.0 / 60.0);
        assert_eq!(view.control_yaw, 2.5, "releasing the direction re-syncs to the view");
    }

    #[test]
    fn a_cut_with_nothing_held_re_syncs_at_once() {
        let mut view = CameraView { control_yaw: 0.3, ..default() };
        update_latch(&mut view, 2.5, true, false, &LatchParams::default(), 1.0 / 60.0);
        assert_eq!(view.control_yaw, 2.5);
    }

    #[test]
    fn a_long_hold_eases_back_to_the_view() {
        let params = LatchParams::default();
        let mut view = CameraView { control_yaw: 0.3, ..default() };
        update_latch(&mut view, 2.5, true, true, &params, 1.0 / 60.0);
        for _ in 0..(60.0 * (params.timeout + 3.0)) as usize {
            update_latch(&mut view, 2.5, false, true, &params, 1.0 / 60.0);
        }
        assert!((view.control_yaw - 2.5).abs() < 0.02, "control yaw {}", view.control_yaw);
    }

    #[test]
    fn without_a_cut_control_follows_the_view() {
        let mut view = CameraView::default();
        for i in 0..10 {
            update_latch(&mut view, i as f32 * 0.1, false, true, &LatchParams::default(), 1.0 / 60.0);
            assert_eq!(view.control_yaw, view.view_yaw);
        }
    }
}
