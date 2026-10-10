//! [`ThirdPersonCameraPlugin`]: the camera's systems and where they run.
//!
//! ```text
//! Update       CameraSet::Input     devices → CameraInput
//! PostUpdate   CameraTargetSources  (after physics writeback and the
//!                                    ragdoll's read-back: every system that
//!                                    last moves a target belongs here)
//!              CameraSet::Target    clocks, target bridges
//!              CameraSet::Rig       stack → anchor → orbit → rig pose
//!              CameraSet::Collide   (P2)
//!              CameraSet::Effects   (P5)
//!              CameraSet::Write     Transform, Projection, CameraView
//!              TransformSystems::Propagate
//! ```
//!
//! The rig reads its target's `Transform`, which is final for the frame by
//! then; `GlobalTransform` would still be last frame's. A follow camera run
//! in `Update`, or before the ragdoll's read-back, shows the character where
//! it was a frame ago (`the_camera_follows_a_ragdoll_read_back_in_the_same_frame`).

use super::bridge::bridge_walkers;
use super::components::{
    update_latch, CameraDesiredPose, CameraGoals, CameraModeRequests, CameraRecorder,
    CameraRigState, CameraTarget, CameraTargetState, CameraView, LatchParams, ThirdPersonCamera,
};
use super::device::{map_devices, CameraDeviceInput};
use super::pipeline::{CameraFrame, CameraRig, TargetSample};
use super::{CameraClock, CameraInput, CameraInputSettings};
use crate::character::anim::RagdollSet;
use avian3d::prelude::PhysicsSystems;
use bevy::prelude::*;

pub struct ThirdPersonCameraPlugin;

/// The camera's stages, in order. Order a system against these to extend
/// the camera: e.g. `.after(CameraSet::Rig).before(CameraSet::Collide)` to
/// edit [`CameraDesiredPose`].
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CameraSet {
    Input,
    Target,
    Rig,
    Collide,
    Effects,
    Write,
}

/// Every system that last moves a camera target in `PostUpdate` goes in
/// (or before) this set; the camera runs after it.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CameraTargetSources;

impl Plugin for ThirdPersonCameraPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<ThirdPersonCamera>()
            .register_type::<CameraTarget>()
            .register_type::<CameraTargetState>()
            .register_type::<CameraInput>()
            .register_type::<CameraInputSettings>()
            .register_type::<CameraClock>()
            .register_type::<CameraRigState>()
            .register_type::<CameraModeRequests>()
            .register_type::<CameraGoals>()
            .register_type::<CameraDesiredPose>()
            .register_type::<CameraView>()
            .register_type::<CameraDeviceInput>()
            .configure_sets(
                PostUpdate,
                CameraTargetSources.after(PhysicsSystems::Writeback).after(RagdollSet::ReadBack),
            )
            .configure_sets(
                PostUpdate,
                (
                    CameraSet::Target,
                    CameraSet::Rig,
                    CameraSet::Collide,
                    CameraSet::Effects,
                    CameraSet::Write,
                )
                    .chain()
                    .after(CameraTargetSources)
                    .before(TransformSystems::Propagate),
            )
            .add_systems(Update, map_devices.in_set(CameraSet::Input))
            .add_systems(PostUpdate, (update_clocks, bridge_walkers).in_set(CameraSet::Target))
            .add_systems(PostUpdate, run_rigs.in_set(CameraSet::Rig))
            .add_systems(PostUpdate, write_cameras.in_set(CameraSet::Write));
    }
}

/// Real and virtual frame times onto every camera.
fn update_clocks(
    mut clocks: Query<&mut CameraClock, With<ThirdPersonCamera>>,
    real: Res<Time<Real>>,
    virtual_time: Res<Time<Virtual>>,
) {
    let clock = CameraClock {
        real_dt: real.delta_secs(),
        virtual_dt: virtual_time.delta_secs(),
    };
    for mut c in &mut clocks {
        *c = clock;
    }
}

type RigQuery<'a> = (
    &'a ThirdPersonCamera,
    &'a mut CameraRigState,
    &'a mut CameraInput,
    &'a CameraInputSettings,
    &'a CameraClock,
    &'a mut CameraModeRequests,
    &'a mut CameraGoals,
    &'a mut CameraDesiredPose,
    Option<&'a mut CameraRecorder>,
);

/// One pipeline frame per camera: the target sampled, the rig stepped.
fn run_rigs(
    mut cameras: Query<RigQuery, With<Camera3d>>,
    targets: Query<(&Transform, Option<&CameraTarget>, Option<&CameraTargetState>)>,
) {
    for (camera, mut state, mut input, settings, clock, mut requests, mut goals, mut desired, recorder) in
        &mut cameras
    {
        let Ok((transform, target, target_state)) = targets.get(camera.target) else {
            continue;
        };
        let rebase = state.last_target.is_some_and(|last| last != camera.target);
        state.last_target = Some(camera.target);
        let sample = TargetSample {
            position: transform.translation,
            grounded: target_state.is_none_or(|s| s.grounded),
            facing_yaw: target_state.and_then(|s| s.facing_yaw),
            rebase,
            pivot_offset: target.map(|t| t.pivot_offset),
        };
        let frame = CameraFrame {
            clock: *clock,
            input: input.clone(),
            target: sample,
            requests: std::mem::take(&mut requests.0),
            goals: std::mem::take(&mut goals.0),
        };
        let start_yaw = sample.facing_yaw.unwrap_or(0.0);
        let rig = state.rig.get_or_insert_with(|| CameraRig::new(&camera.config, start_yaw));
        let output = rig.step(&frame, settings, &camera.config);
        if let Some(mut recorder) = recorder {
            if recorder.trace.frames.is_empty() {
                recorder.trace.start_yaw = start_yaw;
            }
            recorder.trace.frames.push(frame);
        }
        desired.0 = output.pose;
        state.output = Some(output);
        input.consume();
    }
}

type WriteQuery<'a> = (
    &'a CameraDesiredPose,
    &'a CameraRigState,
    &'a CameraInput,
    &'a CameraClock,
    &'a mut Transform,
    &'a mut Projection,
    &'a mut CameraView,
);

/// The pose onto the camera's `Transform` and `Projection`, and the
/// published [`CameraView`].
fn write_cameras(mut cameras: Query<WriteQuery, (With<ThirdPersonCamera>, With<Camera3d>)>) {
    for (desired, state, input, clock, mut transform, mut projection, mut view) in &mut cameras {
        let Some(output) = state.output else { continue };
        let pose = desired.0;
        *transform = Transform::from_translation(pose.eye).with_rotation(pose.rotation);
        if let Projection::Perspective(perspective) = projection.as_mut() {
            perspective.fov = pose.fov;
        }
        view.eye = pose.eye;
        view.rotation = pose.rotation;
        view.fov = pose.fov;
        view.pivot = pose.pivot;
        view.pitch = output.pitch;
        update_latch(
            &mut view,
            output.yaw,
            output.cut,
            input.move_held,
            &LatchParams::default(),
            clock.real_dt,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::time::TimeUpdateStrategy;
    use std::time::Duration;

    fn app() -> App {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin, ThirdPersonCameraPlugin))
            .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(1.0 / 60.0)));
        app
    }

    fn camera(app: &mut App, target: Entity) -> Entity {
        app.world_mut().spawn((Camera3d::default(), ThirdPersonCamera::follow(target))).id()
    }

    /// Teleports the target 10 m every frame (past the teleport distance,
    /// so the camera snaps and its pivot is exact) in `RagdollSet::ReadBack`,
    /// where a fallen ragdoll's root is carried.
    fn carry_in_read_back(mut targets: Query<&mut Transform, With<CameraTarget>>, mut frame: Local<f32>) {
        *frame += 1.0;
        for mut t in &mut targets {
            t.translation.x = 10.0 * *frame;
        }
    }

    #[test]
    fn the_camera_follows_a_ragdoll_read_back_in_the_same_frame() {
        let mut app = app();
        app.add_systems(PostUpdate, carry_in_read_back.in_set(RagdollSet::ReadBack));
        let target = app.world_mut().spawn((Transform::default(), CameraTarget::default())).id();
        let cam = camera(&mut app, target);
        for _ in 0..5 {
            app.update();
            let x = app.world().get::<Transform>(target).unwrap().translation.x;
            let pivot = app.world().get::<CameraView>(cam).unwrap().pivot;
            assert_eq!(pivot.x, x, "the camera saw the target a frame late: pivot {pivot:?}, target x {x}");
        }
    }

    #[test]
    fn camera_input_turns_the_camera_with_no_devices_and_is_consumed() {
        let mut app = app();
        let target = app.world_mut().spawn(Transform::default()).id();
        let cam = camera(&mut app, target);
        app.update();
        let before = app.world().get::<CameraView>(cam).unwrap().view_yaw;
        app.world_mut().get_mut::<CameraInput>(cam).unwrap().look_delta = Vec2::new(0.5, 0.0);
        app.update();
        let after = app.world().get::<CameraView>(cam).unwrap().view_yaw;
        assert!((after - (before - 0.5)).abs() < 1.0e-5, "0.5 rad right: {before} → {after}");
        app.update();
        let later = app.world().get::<CameraView>(cam).unwrap().view_yaw;
        assert_eq!(later, after, "a look delta is used once, not every frame");
    }

    #[test]
    fn a_third_person_camera_without_camera3d_is_left_alone() {
        // The shadow-map view carries a bare `Camera`; nothing here may
        // treat such an entity as the player's camera.
        let mut app = app();
        let target = app.world_mut().spawn(Transform::from_xyz(3.0, 0.0, 0.0)).id();
        let bare = app.world_mut().spawn((Camera::default(), Projection::default(), ThirdPersonCamera::follow(target))).id();
        app.update();
        assert_eq!(*app.world().get::<Transform>(bare).unwrap(), Transform::default());
        assert!(app.world().get::<CameraRigState>(bare).unwrap().rig.is_none());
    }

    #[test]
    fn a_cut_while_moving_latches_the_control_yaw_in_the_world() {
        let mut app = app();
        let target = app.world_mut().spawn(Transform::default()).id();
        let cam = camera(&mut app, target);
        app.update();
        app.update();
        app.world_mut().get_mut::<CameraInput>(cam).unwrap().move_held = true;
        let before = app.world().get::<CameraView>(cam).unwrap().control_yaw;
        // A teleport is a cut.
        app.world_mut().get_mut::<Transform>(target).unwrap().translation.x = 50.0;
        app.update();
        let view = *app.world().get::<CameraView>(cam).unwrap();
        assert!(view.cut_this_frame && view.latched, "a teleport while moving must latch: {view:?}");
        // The view turns; the held direction keeps its meaning.
        app.world_mut().get_mut::<CameraInput>(cam).unwrap().look_delta = Vec2::new(1.0, 0.0);
        app.update();
        let view = *app.world().get::<CameraView>(cam).unwrap();
        assert_eq!(view.control_yaw, before, "control yaw moved while latched: {view:?}");
        assert!((view.view_yaw - view.control_yaw).abs() > 0.5);
        app.world_mut().get_mut::<CameraInput>(cam).unwrap().move_held = false;
        app.update();
        let view = *app.world().get::<CameraView>(cam).unwrap();
        assert_eq!(view.control_yaw, view.view_yaw, "letting go re-syncs");
    }

    #[test]
    fn two_cameras_take_separate_inputs() {
        let mut app = app();
        let a = app.world_mut().spawn(Transform::default()).id();
        let b = app.world_mut().spawn(Transform::from_xyz(5.0, 0.0, 0.0)).id();
        let (cam_a, cam_b) = (camera(&mut app, a), camera(&mut app, b));
        app.update();
        app.world_mut().get_mut::<CameraInput>(cam_a).unwrap().look_delta = Vec2::new(0.4, 0.0);
        app.update();
        let (va, vb) = (*app.world().get::<CameraView>(cam_a).unwrap(), *app.world().get::<CameraView>(cam_b).unwrap());
        assert!((va.view_yaw + 0.4).abs() < 1.0e-5, "camera A turned {}", va.view_yaw);
        assert_eq!(vb.view_yaw, 0.0, "camera B must not see A's input");
        assert!(vb.pivot.x > 4.0, "and follows its own target");
    }

    #[test]
    fn switching_target_hands_over_without_a_cut() {
        let mut app = app();
        let a = app.world_mut().spawn(Transform::default()).id();
        let b = app.world_mut().spawn(Transform::from_xyz(3.0, 0.0, 0.0)).id();
        let cam = camera(&mut app, a);
        app.update();
        app.update();
        let before = app.world().get::<CameraView>(cam).unwrap().pivot;
        app.world_mut().get_mut::<ThirdPersonCamera>(cam).unwrap().target = b;
        app.update();
        let view = *app.world().get::<CameraView>(cam).unwrap();
        assert!(!view.cut_this_frame, "a target switch is a hand-off, not a cut");
        assert!(view.pivot.distance(before) < 0.1, "the pivot must not jump: {before} → {}", view.pivot);
        for _ in 0..240 {
            app.update();
        }
        let pivot = app.world().get::<CameraView>(cam).unwrap().pivot;
        assert!((pivot.x - 3.0).abs() < 0.01, "and arrives at the new target, pivot {pivot}");
    }
}
