//! The in-engine animation authoring studio.
//!
//! Feature-gated behind `anim_studio` so a consumer shipping a game never
//! links the editor.
//!
//! # Why an in-engine editor at all
//!
//! Pose data is judged by eye — does this idle read as a person or a
//! mannequin? — and this project's own history is that the judging is where
//! the time goes. Two authored poses once shipped with limbs 59-77% off the
//! rig's real bone lengths, past a green test suite, caught only by looking
//! at a screenshot.
//!
//! Editing a `.pose.ron` in a text editor and re-running the example puts a
//! recompile-and-relaunch between every tweak and seeing the result. The
//! hot-reload path already removes the recompile; this removes the rest, so
//! a drag on a slider and the rig moving are the same instant.
//!
//! # What lives where
//!
//! - [`edit`] — the editing model and its operations, as plain values and
//!   plain functions. No egui, fully unit-tested.
//! - [`save`] — RON serialization and file I/O, with real error types.
//! - [`pose_editor`] — the panel, a thin translation of clicks into the
//!   above.
//!
//! The split is the same one [`super::ragdoll`] and [`super::ragdoll_plugin`]
//! use, for the same reason: the interesting logic should be testable
//! without standing up the machinery it usually runs inside.

pub mod drag;
pub mod drag_plugin;
pub mod edit;
pub mod effector;
pub mod phase_editor;
pub mod pose_editor;
pub mod save;
pub mod timeline;
pub mod tuning;
pub mod tuning_editor;

use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

use super::plugin::{AnimPose, AnimTarget};
use crate::character::skeleton::Bone;
use edit::PoseEdit;

pub use edit::EditableRotation;
pub use save::{PoseFileError, POSE_DIRECTORY};

/// Everything the studio has open.
#[derive(Resource)]
pub struct StudioState {
    /// The pose being edited.
    pub edit: PoseEdit,
    /// Which named pose the load/save row is pointed at.
    pub pose_name: String,
    /// Whether the editor window is showing.
    pub open: bool,
    /// Last thing that happened, shown under the file row.
    pub status: String,
    /// Whether a save is awaiting confirmation.
    ///
    /// Saving overwrites authored data that may have taken real time to
    /// tune, so it takes two clicks rather than one.
    pub confirming_save: bool,
    /// Whether editing snaps the rig to the pose instead of springing.
    pub snap_while_editing: bool,
    /// Set by the panel, consumed by [`capture_from_rig`].
    pub capture_requested: bool,
    /// Whether procedural animation keeps running while the editor is open.
    ///
    /// Off by default. The phase oscillators breathe and shift weight
    /// continuously by design, which is right for a runtime character and
    /// wrong for authoring: a pose being tuned must hold still, or every
    /// judgement is made against a rig that has already moved on. It also
    /// makes a drag fight a moving target.
    ///
    /// Switchable because seeing the authored pose *with* the idle layer on
    /// top is the last check before calling it done.
    pub animation_playing: bool,
    /// Whether grabbing a limb's tip solves the whole chain rather than
    /// rotating one bone.
    ///
    /// On by default: dragging a hand to where it should be is what an
    /// author means by posing an arm. Switchable because the per-bone drag
    /// is the precise tool, and an effector overrides it on the four tips.
    pub effectors_enabled: bool,
    /// Whether joints can be grabbed and dragged in the viewport.
    ///
    /// On by default — it is the reason the studio is in-engine rather than
    /// a text editor. Switchable because the handles overlay the character
    /// and a final look at a pose wants them out of the way.
    pub drag_enabled: bool,
    /// Whether the phase-oscillator panel is showing.
    pub phase_open: bool,
    /// The oscillator layer being edited.
    pub phase_layer: super::phase::PhaseLayer,
    /// Which oscillator the detail controls address.
    pub selected_oscillator: Option<usize>,
    /// Which bone a newly added oscillator attaches to.
    pub phase_bone: Bone,
    /// Speed fed to the preview clock, so gait oscillators can be seen
    /// without a locomotion controller.
    pub phase_preview_speed: f32,
    /// Whether the phase panel has taken its layer from the rig.
    pub adopted_initial_phase: bool,
    /// Whether the timeline panel is showing.
    pub timeline_open: bool,
    /// The clip being authored.
    pub clip: super::clip::AnimClip,
    /// Where its playhead is.
    pub playback: super::clip::ClipPlayback,
    /// Which keyframe the contact controls address.
    pub selected_keyframe: Option<usize>,
    /// Which keyframe a drag is moving, if any.
    pub dragging_keyframe: Option<usize>,
    /// What the last foot-sliding removal achieved, shown next to its button.
    pub deslide_report: Option<super::slide::SlideReport>,
    /// Whether the spring tuning panel is showing.
    pub tuning_open: bool,
    /// The springs being tuned.
    pub springs: super::rig::BoneSet<super::math::SpringParams>,
    /// Which bone the tuning sliders address.
    pub tuning_bone: Bone,
    /// How far an edit reaches from that bone.
    pub tuning_scope: tuning::TuningScope,
    /// Whether the tuning panel has taken its values from the rig.
    ///
    /// Same hazard as [`Self::adopted_initial_pose`]: writing the studio's
    /// defaults onto a character on the first frame would replace whatever
    /// tuning it was configured with.
    pub adopted_initial_springs: bool,
    /// Whether the editor has taken its initial pose from the rig.
    ///
    /// The studio's default is the rest pose, and writing that onto a
    /// character on the first frame would replace whatever animation it was
    /// loaded with — live-caught doing exactly that, turning a
    /// `relaxed_stand` character into a T-pose the moment the editor
    /// opened. Adopting the rig's pose first makes opening the editor a
    /// read, not a write.
    pub adopted_initial_pose: bool,
}

impl Default for StudioState {
    fn default() -> Self {
        Self {
            edit: PoseEdit::default(),
            pose_name: "relaxed_stand".to_string(),
            open: true,
            status: String::new(),
            confirming_save: false,
            snap_while_editing: true,
            capture_requested: false,
            animation_playing: false,
            drag_enabled: true,
            effectors_enabled: true,
            // Both off by default: most authoring is single poses, and
            // four panels at once leaves little of the viewport to judge
            // the character by.
            phase_open: false,
            phase_layer: super::phase::PhaseLayer::standing_idle(),
            selected_oscillator: Some(0),
            phase_bone: Bone::Spine,
            phase_preview_speed: 0.0,
            adopted_initial_phase: false,
            timeline_open: false,
            clip: super::clip::AnimClip::new(true),
            playback: super::clip::ClipPlayback::stopped(),
            selected_keyframe: None,
            dragging_keyframe: None,
            deslide_report: None,
            tuning_open: true,
            springs: super::dho::default_springs(),
            tuning_bone: Bone::LeftArm,
            tuning_scope: tuning::TuningScope::Chain,
            adopted_initial_springs: false,
            adopted_initial_pose: false,
        }
    }
}

/// The authoring studio.
///
/// Requires `bevy_egui`'s `EguiPlugin`, and a character carrying
/// [`AnimTarget`] to edit. Add it only in a development build:
///
/// ```ignore
/// #[cfg(feature = "anim_studio")]
/// app.add_plugins(AnimStudioPlugin);
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct AnimStudioPlugin;

impl Plugin for AnimStudioPlugin {
    fn build(&self, app: &mut App) {
        // `--studio-timeline` opens the timeline at launch. Present so a
        // headless `--shot` run can capture it, which is the only way the
        // project's own verification rules can check a panel that defaults
        // to closed.
        let timeline_open = std::env::args().any(|argument| argument == "--studio-timeline");

        // `--studio-demo-clip` seeds a three-keyframe clip from the named
        // poses. Its purpose is verification: an empty timeline renders
        // its chrome but proves nothing about keyframes, scrubbing or
        // contact lanes, and those are most of what the panel is.
        let clip = if std::env::args().any(|argument| argument == "--studio-demo-clip") {
            demo_clip()
        } else {
            super::clip::AnimClip::new(true)
        };

        let phase_open = std::env::args().any(|argument| argument == "--studio-phase");

        app.insert_resource(StudioState {
            timeline_open,
            phase_open,
            clip,
            ..Default::default()
        })
            .init_resource::<drag_plugin::DragState>()
            .add_systems(
                EguiPrimaryContextPass,
                (
                    pose_editor::pose_editor_panel,
                    tuning_editor::tuning_panel,
                    timeline::timeline_panel,
                    phase_editor::phase_editor_panel,
                ),
            )
            .add_systems(
                Update,
                (
                    suspend_animation_while_editing,
                    capture_from_rig,
                    // Dragging runs after the panel has had the frame's
                    // pointer input, so `wants_pointer_input` reflects this
                    // frame rather than the last one — otherwise the first
                    // click on a slider also grabs a bone.
                    drag_plugin::drag_bones,
                    drag_plugin::draw_drag_handles,
                    phase_editor::advance_phase_preview,
                )
                    .chain(),
            );
    }
}

/// A three-keyframe clip over the named poses, for exercising the
/// timeline.
///
/// Deliberately built from poses that already exist rather than authored
/// here: what this needs to demonstrate is the timeline, and inventing new
/// pose data to do it would add a second thing that could be wrong.
///
/// The contacts describe a weight shift — both feet, then the right lifts,
/// then both again — so the lanes have something with structure to draw
/// rather than a solid bar.
fn demo_clip() -> super::clip::AnimClip {
    use super::clip::{Contacts, Keyframe};

    let mut clip = super::clip::AnimClip::new(true);

    clip.insert(Keyframe {
        time: 0.0,
        pose: super::poses::relaxed_stand(),
        contacts: Contacts::BOTH,
    });
    clip.insert(Keyframe {
        time: 0.8,
        pose: super::poses::wave(),
        contacts: Contacts { left: true, right: false },
    });
    clip.insert(Keyframe {
        time: 1.6,
        pose: super::poses::relaxed_stand(),
        contacts: Contacts::BOTH,
    });

    clip
}

/// What the studio took off a character so it would hold still, kept so it
/// can be put back.
///
/// Removing the components outright and re-inserting defaults would quietly
/// replace a consumer's tuned locomotion layer with a stock one the first
/// time someone opened the editor. Stashing the real values means
/// suppression is genuinely reversible.
#[derive(Component)]
pub struct SuspendedAnimation {
    phase: Option<super::phase::GaitPhase>,
    layer: Option<super::plugin::AnimPhaseLayer>,
    foot_ik: Option<super::plugin::AnimFootIk>,
}

/// Animated characters that have not yet been suspended, with whichever of
/// the interfering layers they happen to carry.
type RunningCharacters<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        Option<&'static super::phase::GaitPhase>,
        Option<&'static super::plugin::AnimPhaseLayer>,
        Option<&'static super::plugin::AnimFootIk>,
    ),
    (With<AnimTarget>, Without<SuspendedAnimation>),
>;

/// Holds the rig still while a pose is being authored, and restores it
/// afterwards.
///
/// Suppresses the phase oscillators (continuous breathing and weight
/// shift) and foot IK (a ground correction that keeps adjusting), so the
/// rig shows exactly the pose being edited and nothing else. Both are
/// correct at runtime and both actively interfere with authoring.
fn suspend_animation_while_editing(
    mut commands: Commands,
    studio: Res<StudioState>,
    active: RunningCharacters,
    suspended: Query<(Entity, &SuspendedAnimation)>,
) {
    // The phase editor is the one panel whose subject IS the procedural
    // motion, so opening it necessarily un-suspends. Suspending anyway
    // would leave every wave frozen and the editor unable to show the
    // thing it edits.
    let should_suspend = studio.open && !studio.animation_playing && !studio.phase_open;

    if should_suspend {
        for (entity, phase, layer, foot_ik) in &active {
            commands
                .entity(entity)
                .insert(SuspendedAnimation {
                    phase: phase.copied(),
                    layer: layer.cloned(),
                    foot_ik: foot_ik.cloned(),
                })
                .remove::<super::phase::GaitPhase>()
                .remove::<super::plugin::AnimPhaseLayer>()
                .remove::<super::plugin::AnimFootIk>();
        }
        return;
    }

    for (entity, stashed) in &suspended {
        let mut character = commands.entity(entity);
        character.remove::<SuspendedAnimation>();

        if let Some(phase) = stashed.phase {
            character.insert(phase);
        }
        if let Some(layer) = stashed.layer.clone() {
            character.insert(layer);
        }
        if let Some(foot_ik) = stashed.foot_ik.clone() {
            character.insert(foot_ik);
        }
    }
}

/// Copies the rig's current pose into the editor.
///
/// The point of this is capturing something the *runtime* produced that no
/// static pose describes — a moment mid-gait, a ragdoll settling — and
/// turning it into authored data. Without it the editor can only ever
/// refine poses that already exist.
///
/// Reads [`AnimPose`] (what is actually rendered) rather than
/// [`AnimTarget`] (what the springs are aiming at), because those differ
/// exactly when capturing is most interesting: mid-transition.
fn capture_from_rig(
    mut studio: ResMut<StudioState>,
    characters: Query<&AnimPose, With<AnimTarget>>,
) {
    if !studio.capture_requested {
        return;
    }
    studio.capture_requested = false;

    let Ok(animated) = characters.single() else {
        studio.status = "capture failed: no animated character in the scene".to_string();
        return;
    };

    let source = studio.edit.source.clone();
    studio.edit = PoseEdit::from_pose(&animated.pose());
    studio.edit.source = source;

    // A capture IS an edit — it replaces whatever was open. Marking it
    // dirty is what stops the next hot-reload or load from discarding it
    // without warning.
    studio.edit.set_dirty();
    studio.status = "captured the rig's current pose".to_string();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::poses;
    use crate::character::skeleton::Bone;

    /// A minimal app with the studio and one animated character.
    fn studio_app() -> (App, Entity) {
        let mut app = App::new();
        app.init_resource::<StudioState>();
        app.add_systems(Update, capture_from_rig);

        let character = app
            .world_mut()
            .spawn((
                AnimTarget::new(poses::rest()),
                AnimPose::settled_on(&poses::wave()),
            ))
            .id();

        (app, character)
    }

    #[test]
    fn capturing_copies_the_rendered_pose_into_the_editor() {
        // The rig is rendering `wave` while the editor holds the rest pose;
        // capturing must bring the wave across.
        let (mut app, _character) = studio_app();
        app.world_mut().resource_mut::<StudioState>().capture_requested = true;

        app.update();

        let studio = app.world().resource::<StudioState>();
        let captured = studio.edit.to_pose();
        let expected = poses::wave();

        for &bone in Bone::ALL.iter() {
            assert!(
                captured.rotation(bone).abs_diff_eq(expected.rotation(bone), 1.0e-4),
                "{} was not captured: {:?} against the rig's {:?}",
                bone.name(),
                captured.rotation(bone),
                expected.rotation(bone),
            );
        }
    }

    #[test]
    fn capturing_marks_the_pose_dirty() {
        // A capture replaces the open pose wholesale. If that did not count
        // as an unsaved change, a subsequent load would discard it with no
        // warning — which is the exact failure the dirty flag exists for.
        let (mut app, _character) = studio_app();
        app.world_mut().resource_mut::<StudioState>().capture_requested = true;

        app.update();

        assert!(
            app.world().resource::<StudioState>().edit.is_dirty(),
            "a captured pose is unsaved work and must be marked dirty",
        );
    }

    #[test]
    fn capturing_is_a_one_shot_request() {
        // The flag has to clear, or the editor would be overwritten by the
        // rig on every single frame and no edit would ever survive.
        let (mut app, _character) = studio_app();
        app.world_mut().resource_mut::<StudioState>().capture_requested = true;

        app.update();
        assert!(!app.world().resource::<StudioState>().capture_requested);

        // Now an edit must survive the next frames.
        app.world_mut()
            .resource_mut::<StudioState>()
            .edit
            .reset_bone(Bone::RightArm);
        app.update();
        app.update();

        let studio = app.world().resource::<StudioState>();
        assert_eq!(
            studio.edit.to_pose().rotation(Bone::RightArm),
            bevy::math::Quat::IDENTITY,
            "an edit made after a capture must not be overwritten by the rig",
        );
    }

    /// An app with the suspension system and one fully-equipped character.
    fn suspension_app() -> (App, Entity) {
        let mut app = App::new();
        app.init_resource::<StudioState>();
        app.add_systems(Update, suspend_animation_while_editing);

        let character = app
            .world_mut()
            .spawn((
                AnimTarget::new(poses::relaxed_stand()),
                AnimPose::settled_on(&poses::relaxed_stand()),
                super::super::phase::GaitPhase { speed: 1.4, ..Default::default() },
                super::super::plugin::AnimPhaseLayer(
                    super::super::phase::PhaseLayer::locomotion(),
                ),
                super::super::plugin::AnimFootIk::default(),
            ))
            .id();

        (app, character)
    }

    #[test]
    fn opening_the_editor_stops_the_procedural_animation() {
        // A pose being tuned has to hold still. The phase oscillators
        // breathe and shift weight continuously and foot IK keeps
        // re-correcting to the ground — both correct at runtime, both
        // actively interfering with authoring, and both making a drag chase
        // a moving target.
        let (mut app, character) = suspension_app();
        app.update();

        let entity = app.world().entity(character);
        assert!(
            entity.get::<super::super::phase::GaitPhase>().is_none(),
            "the gait clock should be suspended while the editor is open",
        );
        assert!(
            entity.get::<super::super::plugin::AnimPhaseLayer>().is_none(),
            "the phase layer should be suspended while the editor is open",
        );
        assert!(
            entity.get::<super::super::plugin::AnimFootIk>().is_none(),
            "foot IK should be suspended while the editor is open",
        );
    }

    #[test]
    fn re_enabling_playback_restores_the_characters_own_layers() {
        // Suppression must be reversible, and must restore what was
        // actually there: re-inserting stock defaults would quietly replace
        // a consumer's tuned locomotion layer the first time anyone opened
        // the editor.
        let (mut app, character) = suspension_app();
        app.update();

        app.world_mut().resource_mut::<StudioState>().animation_playing = true;
        app.update();

        let entity = app.world().entity(character);
        let phase = entity
            .get::<super::super::phase::GaitPhase>()
            .expect("the gait clock should come back");

        assert_eq!(
            phase.speed, 1.4,
            "the character's OWN gait speed must be restored, not a default",
        );
        assert!(entity.get::<super::super::plugin::AnimPhaseLayer>().is_some());
        assert!(entity.get::<super::super::plugin::AnimFootIk>().is_some());
        assert!(
            entity.get::<SuspendedAnimation>().is_none(),
            "the stash should be cleared once it has been restored",
        );
    }

    #[test]
    fn closing_the_editor_also_restores_playback() {
        // The other way out: a user who closes the window rather than
        // ticking the checkbox must not leave the character frozen.
        let (mut app, character) = suspension_app();
        app.update();

        app.world_mut().resource_mut::<StudioState>().open = false;
        app.update();

        assert!(
            app.world()
                .entity(character)
                .get::<super::super::phase::GaitPhase>()
                .is_some(),
            "closing the editor should give the character its animation back",
        );
    }

    #[test]
    fn suspension_is_idempotent() {
        // The system runs every frame. Suspending an already-suspended
        // character must not stash a second, empty snapshot over the real
        // one — that would lose the layers permanently.
        let (mut app, character) = suspension_app();
        for _ in 0..5 {
            app.update();
        }

        app.world_mut().resource_mut::<StudioState>().animation_playing = true;
        app.update();

        assert!(
            app.world()
                .entity(character)
                .get::<super::super::plugin::AnimPhaseLayer>()
                .is_some(),
            "repeated suspension must not lose the stashed layers",
        );
    }

    #[test]
    fn a_character_without_a_phase_layer_survives_suspension() {
        // Not every character has one. Stashing `None` and restoring
        // nothing has to be a clean no-op rather than inserting a default
        // the character never had.
        let mut app = App::new();
        app.init_resource::<StudioState>();
        app.add_systems(Update, suspend_animation_while_editing);

        let character = app
            .world_mut()
            .spawn((AnimTarget::new(poses::rest()), AnimPose::settled_on(&poses::rest())))
            .id();

        app.update();
        app.world_mut().resource_mut::<StudioState>().animation_playing = true;
        app.update();

        assert!(
            app.world()
                .entity(character)
                .get::<super::super::phase::GaitPhase>()
                .is_none(),
            "a character that never had a gait clock must not be given one",
        );
    }

    #[test]
    fn adopting_the_rigs_pose_is_a_read_not_a_write() {
        // The guard for a live-caught bug: the studio's default is the REST
        // pose, and the panel writes its edit onto the rig every time
        // something changes. On the first frame that combination replaced a
        // `relaxed_stand` character with a T-pose — opening the editor
        // silently destroyed the thing being edited.
        //
        // Exercised through `PoseEdit` rather than the panel because the
        // panel needs an egui context; what matters is that adoption
        // reproduces the rig's pose exactly, so the subsequent write is a
        // no-op.
        let rig_pose = poses::relaxed_stand();
        let adopted = PoseEdit::from_pose(&rig_pose).to_pose();

        for &bone in Bone::ALL.iter() {
            assert!(
                rig_pose.rotation(bone).abs_diff_eq(adopted.rotation(bone), 1.0e-5),
                "{} changed when the editor adopted the rig's pose: {:?} -> {:?} — \
                 opening the editor must not modify the character",
                bone.name(),
                rig_pose.rotation(bone),
                adopted.rotation(bone),
            );
        }
    }

    #[test]
    fn a_fresh_studio_has_not_adopted_a_pose_yet() {
        // The flag has to start false, or the adoption never runs and the
        // bug above returns.
        assert!(!StudioState::default().adopted_initial_pose);
    }

    #[test]
    fn capturing_with_no_character_reports_rather_than_panicking() {
        let mut app = App::new();
        app.init_resource::<StudioState>();
        app.add_systems(Update, capture_from_rig);
        app.world_mut().resource_mut::<StudioState>().capture_requested = true;

        app.update();

        let studio = app.world().resource::<StudioState>();
        assert!(
            studio.status.contains("no animated character"),
            "the failure should be reported to the user, got {:?}",
            studio.status,
        );
    }

    #[test]
    fn capturing_keeps_the_open_files_name() {
        // Otherwise "capture, then save" would lose the file it came from
        // and quietly fail with `NoPath`.
        let (mut app, _character) = studio_app();
        {
            let mut studio = app.world_mut().resource_mut::<StudioState>();
            studio.edit.source = Some("assets/anim/relaxed_stand.pose.ron".to_string());
            studio.capture_requested = true;
        }

        app.update();

        assert_eq!(
            app.world().resource::<StudioState>().edit.source.as_deref(),
            Some("assets/anim/relaxed_stand.pose.ron"),
        );
    }
}
