//! ECS wiring for viewport bone dragging.
//!
//! The maths is in [`super::drag`]; this reads the mouse, projects joints,
//! and applies the result. Kept apart for the same reason the rest of
//! `anim` splits this way — the interesting decisions should be testable
//! without a window.

use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use crate::character::anim::plugin::{AnimPose, AnimTarget};
use crate::character::skeleton::{Bone, HumanoidSkeleton};

/// Every bone's joint position, read from the rig's own real
/// `GlobalTransform`s.
///
/// Deliberately NOT `forward_kinematics` on the edited pose. That computes
/// positions on this crate's *synthetic* T-pose proportions, while the
/// character being edited is a real retargeted mesh with its own — so the
/// handles drifted off the upper body by a visible margin, floating clear
/// of the arms while the legs happened to line up.
///
/// Reading the real transforms is the same ground truth the skeleton
/// gizmos use, so a handle is by construction exactly where the joint
/// renders, on whatever rig is loaded.
fn joint_positions(
    skeleton: &HumanoidSkeleton,
    transforms: &Query<&GlobalTransform>,
) -> Vec<(Bone, Vec3)> {
    Bone::ALL
        .iter()
        .filter_map(|&bone| {
            let transform = transforms.get(skeleton.entity(bone)).ok()?;
            Some((bone, transform.translation()))
        })
        .collect()
}

/// The world frame a bone's authored pose delta actually rotates in.
///
/// # Why this is not just the parent's world rotation
///
/// A pose stores a rig-independent *delta*, and `retarget` renders it as
///
/// ```text
///   transform.rotation = rest_rotation(bone) * (bind⁻¹ · delta · bind)
/// ```
///
/// so by the time a delta reaches the screen it has been wrapped twice: by
/// the bone's own rest rotation, and by a conjugation into its bind frame.
/// The frame the delta therefore acts in is
///
/// ```text
///   parent_world · rest_rotation(bone) · bind(bone)
/// ```
///
/// Passing only `parent_world` — as this did originally — omits both. On a
/// synthetic rig those are identity and nothing goes wrong; on the real
/// retargeted mesh they are large multi-axis rotations (`LeftShoulder`'s
/// rest sits near pitch −66°, yaw 50°, roll −130°), and omitting them
/// **inverts one screen axis while leaving the other correct** — which is
/// exactly the reported "dragging up moves it down, sideways is fine".
///
/// `bind` is accumulated the same way `retarget` accumulates it, so the two
/// cannot disagree.
fn delta_frame(
    bone: Bone,
    skeleton: &HumanoidSkeleton,
    transforms: &Query<&GlobalTransform>,
) -> Quat {
    let parent_world = bone
        .parent()
        .and_then(|parent| transforms.get(skeleton.entity(parent)).ok())
        .map(|transform| transform.rotation())
        .unwrap_or(Quat::IDENTITY);

    parent_world * skeleton.rest_rotation(bone) * accumulated_bind(bone, skeleton)
}

/// A bone's bind rotation, accumulated down the chain exactly as
/// `retarget::accumulated_bind_rotations` does.
///
/// Duplicated rather than shared because that function is private to
/// `retarget` and returns the whole set; this needs one bone. The
/// duplication is guarded by a test that compares the two.
fn accumulated_bind(bone: Bone, skeleton: &HumanoidSkeleton) -> Quat {
    let mut chain = Vec::new();
    let mut current = Some(bone);
    while let Some(link) = current {
        chain.push(link);
        current = link.parent();
    }

    // Above `Hips` sits whatever ancestor chain the rig was spawned under —
    // for `puppet_base.gltf`, a Blender Z-up import correction composed
    // with the example's facing yaw. Starting from identity instead drops
    // it and puts every bone's frame out by that much.
    let mut accumulated = skeleton.hips_root_rotation();
    for &link in chain.iter().rev() {
        accumulated *= skeleton.rest_rotation(link);
    }
    accumulated
}

/// One bone's joint position, or the origin if it is not resolvable.
fn joint_position(
    bone: Bone,
    skeleton: &HumanoidSkeleton,
    transforms: &Query<&GlobalTransform>,
) -> Vec3 {
    transforms
        .get(skeleton.entity(bone))
        .map(|transform| transform.translation())
        .unwrap_or(Vec3::ZERO)
}

use super::drag::{
    aim_rotation, bone_for_handle, drag_target_on_view_plane, pick_joint, BoneDrag,
    GRAB_RADIUS_PIXELS,
};
use super::edit::EditableRotation;
use super::effector::Effector;
use super::StudioState;

/// What the pointer is doing to the rig.
#[derive(Resource, Default)]
pub struct DragState {
    /// The joint under the cursor, if any — drawn highlighted so the user
    /// knows what a click would grab before they commit to it.
    pub hovered: Option<Bone>,
    /// The drag in progress.
    pub active: Option<BoneDrag>,
    /// The effector being dragged, if the grab landed on a limb's tip.
    ///
    /// Kept separate from `active` because an effector drag needs nothing
    /// snapshotted: the solve is a pure function of the target, so
    /// re-solving from the live pose each frame is stable by construction
    /// rather than by carefully freezing state.
    pub effector: Option<super::effector::Effector>,
}

/// The rig's real geometry, so a solve uses the mesh's own proportions
/// rather than this crate's synthetic T-pose.
///
/// Offsets come from each bone's own local `Transform`, the same way
/// `ragdoll_plugin` reads them — a retargeted character's limb lengths are
/// its own, and solving against the synthetic ones would place a hand
/// where the synthetic rig's hand would go.
fn rig_geometry(
    skeleton: &HumanoidSkeleton,
    transforms: &Query<&Transform>,
) -> crate::character::anim::rig::RigGeometry {
    use crate::character::anim::rig::{BoneSet, RigGeometry};

    let offsets = BoneSet::from_fn(|bone| {
        if bone == Bone::Hips {
            return bone.t_pose_offset();
        }
        transforms
            .get(skeleton.entity(bone))
            .map(|transform| transform.translation)
            .unwrap_or_else(|_| bone.t_pose_offset())
    });

    RigGeometry::from_skeleton(skeleton, offsets)
}

/// Everything needed to turn the mouse into a world-space ray.
///
/// Grouped because they are always read together and never separately —
/// a cursor position means nothing without the camera that projects it.
#[derive(bevy::ecs::system::SystemParam)]
pub struct Pointer<'w, 's> {
    buttons: Res<'w, ButtonInput<MouseButton>>,
    windows: Query<'w, 's, &'static Window, With<PrimaryWindow>>,
    /// The scene camera, found by `With<Camera3d>`.
    ///
    /// The filter is load-bearing, not tidiness. Bevy's shadow mapping
    /// creates an internal view entity that also carries a bare `Camera`
    /// — the same trap that once made the gallery's whole egui UI render
    /// to an invisible camera (see `spawn_camera` there). A plain
    /// `Query<(&Camera, &GlobalTransform)>` therefore matches TWO entities,
    /// `single()` fails every frame, and this system returns before doing
    /// anything.
    ///
    /// That is not hypothetical: dragging shipped completely inert because
    /// of it, and the handles still drew (a different system, different
    /// query) so everything looked fine.
    cameras: Query<'w, 's, (&'static Camera, &'static GlobalTransform), With<Camera3d>>,
}

/// Highlights the joint under the cursor and starts or continues a drag.
///
/// Runs in `Update` and is deliberately a no-op whenever egui wants the
/// pointer, so dragging a slider in the editor panel cannot also yank the
/// character's arm.
pub fn drag_bones(
    mut studio: ResMut<StudioState>,
    mut drag: ResMut<DragState>,
    mut contexts: bevy_egui::EguiContexts,
    pointer: Pointer,
    transforms: Query<&GlobalTransform>,
    // Local transforms, for reading the rig's own bone offsets.
    locals: Query<&Transform>,
    mut characters: Query<(&mut AnimTarget, Option<&mut AnimPose>, &HumanoidSkeleton)>,
) -> Result {
    let Pointer { buttons, windows, cameras } = pointer;
    if !studio.open || !studio.drag_enabled {
        drag.hovered = None;
        drag.active = None;
        return Ok(());
    }

    // egui first: a click that landed on a panel belongs to the panel.
    // Without this, dragging the angle slider would simultaneously drag the
    // bone, and the two would fight.
    let egui_wants_pointer = contexts
        .ctx_mut()
        .map(|ctx| ctx.egui_wants_pointer_input())
        .unwrap_or(false);

    let Ok(window) = windows.single() else { return Ok(()) };
    let Some(cursor) = window.cursor_position() else {
        drag.hovered = None;
        return Ok(());
    };
    let Ok((camera, camera_transform)) = cameras.single() else { return Ok(()) };
    let Ok((mut target_component, mut animated, skeleton)) = characters.single_mut() else {
        return Ok(());
    };

    if buttons.just_released(MouseButton::Left) {
        drag.active = None;
    }

    let pose = studio.edit.to_pose();
    let camera_forward = camera_transform.forward();
    let projected: Vec<(Bone, Option<Vec2>, f32)> = joint_positions(skeleton, &transforms)
        .into_iter()
        .map(|(bone, world)| {
            let screen = camera.world_to_viewport(camera_transform, world).ok();
            let depth = (world - camera_transform.translation()).dot(*camera_forward);
            (bone, screen, depth)
        })
        .collect();

    // Hover, only when the pointer is free and no drag is running.
    if drag.active.is_none() {
        drag.hovered = (!egui_wants_pointer)
            .then(|| pick_joint(cursor, &projected, GRAB_RADIUS_PIXELS))
            .flatten()
            // The ROOT cannot be dragged (nothing above it to rotate), so
            // it must not highlight either — offering a grab that does
            // nothing is worse than offering none.
            .filter(|pick| bone_for_handle(pick.bone).is_some())
            .map(|pick| pick.bone);
    }

    // An effector drag: grabbing a limb's TIP solves the whole chain
    // rather than rotating one bone. Checked before the ordinary drag so
    // grabbing a hand poses the arm instead of rotating the wrist, which
    // is what "drag the hand there" means to an author.
    if studio.effectors_enabled
        && drag.active.is_none()
        && !egui_wants_pointer
        && buttons.just_pressed(MouseButton::Left)
        && let Some(handle) = drag.hovered
        && let Some(effector) = Effector::ALL.iter().copied().find(|e| e.tip == handle)
    {
        drag.effector = Some(effector);
        drag.hovered = Some(handle);
    }

    if let Some(effector) = drag.effector {
        if buttons.pressed(MouseButton::Left) {
            let Ok(ray) = camera.viewport_to_world(camera_transform, cursor) else {
                return Ok(());
            };
            let tip = joint_position(effector.tip, skeleton, &transforms);

            if let Some(target) = drag_target_on_view_plane(
                ray.origin,
                *ray.direction,
                tip,
                *camera_forward,
            ) {
                let mut edited = studio.edit.to_pose();
                super::effector::solve(
                    &mut edited,
                    effector,
                    target,
                    0.02,
                    &rig_geometry(skeleton, &locals),
                );

                // Write every bone the solve touched, since an effector
                // moves a whole chain rather than one bone.
                for bone in [effector.upper, effector.lower] {
                    studio
                        .edit
                        .set_rotation(bone, EditableRotation::from_quat(edited.rotation(bone)));
                }

                let pose = studio.edit.to_pose();
                target_component.pose = pose;
                if studio.snap_while_editing
                    && let Some(mut animated) = animated
                {
                    *animated = AnimPose::settled_on(&pose);
                }
            }
        } else {
            drag.effector = None;
        }

        return Ok(());
    }

    // Begin a drag.
    //
    // Everything the drag will need is captured HERE, once. See `BoneDrag`
    // for why none of it may be refreshed from the rig afterwards.
    if drag.active.is_none()
        && !egui_wants_pointer
        && buttons.just_pressed(MouseButton::Left)
        && let Some(handle) = drag.hovered
        && let Some(bone) = bone_for_handle(handle)
    {
        let start_world = joint_position(handle, skeleton, &transforms);

        // Where the cursor was pointing, in the joint's own view plane, at
        // the instant of the grab. The difference between that and the
        // joint is the offset the drag carries.
        let grab_offset = camera
            .viewport_to_world(camera_transform, cursor)
            .ok()
            .and_then(|ray| {
                drag_target_on_view_plane(
                    ray.origin,
                    *ray.direction,
                    start_world,
                    *camera_forward,
                )
            })
            .map(|under_cursor| start_world - under_cursor)
            .unwrap_or(Vec3::ZERO);

        drag.active = Some(BoneDrag {
            bone,
            handle,
            start_world,
            pivot_world: joint_position(bone, skeleton, &transforms),
            parent_world: delta_frame(bone, skeleton, &transforms),
            start_rotation: pose.rotation(bone),
            grab_offset,
        });
    }

    // Continue one.
    let Some(active) = drag.active else { return Ok(()) };

    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor) else {
        return Ok(());
    };

    // The joint tracks the cursor in the two dimensions the user can see
    // and keeps its depth in the one they cannot — see
    // `drag_target_on_view_plane`.
    let Some(under_cursor) = drag_target_on_view_plane(
        ray.origin,
        *ray.direction,
        active.start_world,
        *camera_forward,
    ) else {
        return Ok(());
    };

    // Carry the grab offset, so the joint keeps its distance from the
    // pointer rather than snapping onto it the instant the button goes
    // down.
    let world_target = under_cursor + active.grab_offset;

    // Every input is the snapshot from the grab — never the live rig.
    // Reading `current` or the parent rotation back each frame computes
    // this frame's correction against a pose that already contains the
    // last one, and the limb spins without bound.
    let rotation = aim_rotation(
        active.pivot_world,
        active.start_world,
        world_target,
        active.parent_world,
        active.start_rotation,
    );

    studio.edit.set_rotation(active.bone, EditableRotation::from_quat(rotation));

    let edited = studio.edit.to_pose();
    target_component.pose = edited;
    if studio.snap_while_editing
        && let Some(animated) = animated.as_mut()
    {
        **animated = AnimPose::settled_on(&edited);
    }

    Ok(())
}

/// Draws a marker on every joint, highlighting the one under the cursor.
///
/// Without this the grab targets are invisible and the tool is guesswork —
/// a user would have to sweep the cursor over the character hunting for a
/// joint that responds.
pub fn draw_drag_handles(
    mut gizmos: Gizmos,
    studio: Res<StudioState>,
    drag: Res<DragState>,
    transforms: Query<&GlobalTransform>,
    skeletons: Query<&HumanoidSkeleton>,
) {
    if !studio.open || !studio.drag_enabled {
        return;
    }

    let Ok(skeleton) = skeletons.single() else { return };

    for (bone, world) in joint_positions(skeleton, &transforms) {
        // The root is not draggable, so it gets no handle.
        if bone_for_handle(bone).is_none() {
            continue;
        }

        let dragging = drag.active.is_some_and(|active| active.handle == bone);
        let hovered = drag.hovered == Some(bone);

        // Effector tips are drawn larger and in a different colour, so it
        // is visible BEFORE clicking which handles pose a whole limb and
        // which rotate one bone. Two controls that look identical and
        // behave differently is the worst of both.
        let is_effector =
            studio.effectors_enabled && Effector::ALL.iter().any(|e| e.tip == bone);

        let (radius, color) = match (dragging, hovered, is_effector) {
            (true, _, _) => (0.032, Color::srgb(1.0, 0.85, 0.2)),
            (_, true, _) => (0.028, Color::srgb(0.4, 1.0, 0.6)),
            (_, _, true) => (0.024, Color::srgb(1.0, 0.6, 0.35)),
            _ => (0.014, Color::srgb(0.45, 0.65, 0.9)),
        };

        gizmos.sphere(Isometry3d::from_translation(world), radius, color);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::world::CommandQueue;

    /// An app with a real bone hierarchy, propagated.
    ///
    /// Uses `TransformPlugin` rather than a hand-assembled schedule:
    /// propagation depends on resources the plugin installs, and
    /// reconstructing that by hand breaks on the next Bevy upgrade.
    fn rig_app() -> (App, HumanoidSkeleton) {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin));

        let mut queue = CommandQueue::default();
        let mut commands = Commands::new(&mut queue, app.world());
        let (_root, skeleton) = crate::character::skeleton::tests::spawn_bare_bone_entities(
            &mut commands,
            Transform::IDENTITY,
        );
        queue.apply(app.world_mut());

        app.update();

        (app, skeleton)
    }

    #[test]
    fn handles_are_read_from_the_rigs_own_transforms() {
        // The guard for a live-caught bug: handles were computed by running
        // forward kinematics on this crate's SYNTHETIC T-pose proportions,
        // while the character on screen is a real retargeted mesh with its
        // own. The handles floated visibly clear of the arms while the legs
        // happened to line up, which is exactly the kind of partial wrongness
        // that reads as "close enough" at a glance.
        //
        // Reading the rig's real transforms makes a handle land on its joint
        // by construction, on whatever rig is loaded.
        let (mut app, skeleton) = rig_app();
        let mut state = app.world_mut().query::<&GlobalTransform>();
        let transforms = state.query(app.world());

        let positions = joint_positions(&skeleton, &transforms);
        assert_eq!(positions.len(), Bone::ALL.len(), "every bone should resolve");

        for (bone, position) in positions {
            let expected = transforms
                .get(skeleton.entity(bone))
                .expect("the bone entity exists")
                .translation();

            assert_eq!(
                position,
                expected,
                "{}'s handle must sit exactly on the rendered joint",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_single_joint_position_matches_the_full_list() {
        let (mut app, skeleton) = rig_app();
        let mut state = app.world_mut().query::<&GlobalTransform>();
        let transforms = state.query(app.world());

        for (bone, position) in joint_positions(&skeleton, &transforms) {
            assert_eq!(joint_position(bone, &skeleton, &transforms), position);
        }
    }

    #[test]
    fn dragging_a_joint_upward_moves_it_upward_on_a_retargeted_rig() {
        // The user-reported bug, as a test: "dragging up rotates it down,
        // horizontal is fine".
        //
        // A pure inversion of ONE screen axis is the signature of a missing
        // frame, not of broken rotation maths — and `aim_rotation` was
        // provably correct in isolation. What was missing is that a pose
        // stores a rig-independent DELTA, which `retarget` wraps twice
        // before it reaches the screen:
        //
        //     rest_rotation(bone) * (bind⁻¹ · delta · bind)
        //
        // The drag passed only the parent's world rotation, omitting both.
        // On a synthetic rig those are identity and nothing breaks; on the
        // real mesh they are large multi-axis rotations, and omitting them
        // flips an axis.
        //
        // So this runs on a REAL retargeted skeleton — the same fixture
        // `retarget`'s own tests use, for the same reason: a synthetic rig
        // is structurally blind to this entire bug class.
        use crate::character::anim::poses;
        use crate::character::anim::retarget::write_pose_to_skeleton;
        use crate::character::anim::studio::drag::aim_rotation;

        let (mut app, skeleton) = real_mesh_app();

        let pose = poses::relaxed_stand();
        write(&mut app, &skeleton, &pose);
        app.update();

        let handle = Bone::LeftForeArm;
        let bone = bone_for_handle(handle).expect("the elbow has a parent");

        let (pivot, start, frame) = {
            let mut state = app.world_mut().query::<&GlobalTransform>();
            let world = app.world();
            let transforms = state.query(world);
            (
                joint_position(bone, &skeleton, &transforms),
                joint_position(handle, &skeleton, &transforms),
                delta_frame(bone, &skeleton, &transforms),
            )
        };

        // Straight up, by a quarter of the bone's own length — well within
        // reach, so the aim is unambiguous.
        let reach = start.distance(pivot);
        let target = start + Vec3::Y * reach * 0.25;

        let rotation = aim_rotation(pivot, start, target, frame, pose.rotation(bone));

        let mut dragged = pose;
        dragged.set_rotation(bone, rotation);
        write(&mut app, &skeleton, &dragged);
        app.update();

        let after = {
            let mut state = app.world_mut().query::<&GlobalTransform>();
            let transforms = state.query(app.world());
            joint_position(handle, &skeleton, &transforms)
        };

        assert!(
            after.y > start.y,
            "dragging {} upward must move it UP, but it went from y={} to y={} — an \
             inverted axis means the delta's frame is wrong",
            handle.name(),
            start.y,
            after.y,
        );

        // And it should actually get there, not merely move the right way.
        assert!(
            after.distance(target) < 0.02,
            "the dragged joint should land near its target: {after:?} against {target:?}",
        );
    }

    #[test]
    fn dragging_follows_the_cursor_on_every_axis() {
        // The user reported vertical inverted while "horizontal looks
        // better and more predicted" — so a fix must be checked on BOTH,
        // or it risks trading one broken axis for another.
        //
        // Every direction is tested against the real retargeted rig, since
        // a synthetic one cannot exhibit the bug at all.
        use crate::character::anim::poses;
        use crate::character::anim::studio::drag::aim_rotation;

        for direction in [Vec3::Y, Vec3::NEG_Y, Vec3::X, Vec3::NEG_X, Vec3::Z, Vec3::NEG_Z] {
            let (mut app, skeleton) = real_mesh_app();
            let pose = poses::relaxed_stand();
            write(&mut app, &skeleton, &pose);
            app.update();

            let handle = Bone::LeftForeArm;
            let bone = bone_for_handle(handle).expect("the elbow has a parent");

            let (pivot, start, frame) = {
                let mut state = app.world_mut().query::<&GlobalTransform>();
                let world = app.world();
                let transforms = state.query(world);
                (
                    joint_position(bone, &skeleton, &transforms),
                    joint_position(handle, &skeleton, &transforms),
                    delta_frame(bone, &skeleton, &transforms),
                )
            };

            let reach = start.distance(pivot);
            let bone_axis = (start - pivot).normalize();

            // Skip directions that point along the bone's own axis. A joint
            // swings on a fixed-radius arc, so a target displaced along
            // that axis is simply not reachable — the limb aims at it and
            // stops at its own length, which is correct behaviour and not
            // something a "did it arrive" assertion can express.
            if bone_axis.dot(direction).abs() > 0.8 {
                continue;
            }

            // Project the displacement onto the sphere the joint actually
            // travels on, so the target is reachable by construction.
            let target =
                pivot + (start + direction * reach * 0.2 - pivot).normalize() * reach;

            let rotation = aim_rotation(pivot, start, target, frame, pose.rotation(bone));
            let mut dragged = pose;
            dragged.set_rotation(bone, rotation);
            write(&mut app, &skeleton, &dragged);
            app.update();

            let after = {
                let mut state = app.world_mut().query::<&GlobalTransform>();
                let world = app.world();
                let transforms = state.query(world);
                joint_position(handle, &skeleton, &transforms)
            };

            // Assert the joint LANDS ON the target, not merely that it
            // moved in roughly the right direction.
            //
            // "Moved the right way" is the weaker and more misleading
            // claim: a joint swinging on a fixed-length arc does not
            // travel parallel to a displacement that is perpendicular to
            // its own bone, so a direction test fails on a perfectly
            // correct drag. Reaching the target is the property that
            // actually means "the joint follows the cursor".
            assert!(
                after.distance(target) < 0.01,
                "dragging toward {direction:?} left the joint at {after:?} instead of \
                 its target {target:?} — it must follow the cursor on every axis",
            );
        }
    }

    /// An app carrying the real `puppet_base.gltf` bind data.
    ///
    /// The fixture's bind rotations are copied onto a rig spawned inside
    /// the app's own world. Swapping a prebuilt `World` into an `App`
    /// instead does not work: the plugins install their resources into
    /// whichever world was present when they were added, and the registry
    /// then refuses to re-add them.
    fn real_mesh_app() -> (App, HumanoidSkeleton) {
        use std::collections::HashMap;

        let (fixture, real) =
            crate::character::anim::retarget::tests::real_mesh_skeleton_world();
        drop(fixture);

        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin));

        let mut queue = bevy::ecs::world::CommandQueue::default();
        let mut commands = Commands::new(&mut queue, app.world());
        let (_root, bare) = crate::character::skeleton::tests::spawn_bare_bone_entities(
            &mut commands,
            Transform::IDENTITY,
        );
        queue.apply(app.world_mut());

        // Rebind the app's own entities with the fixture's REAL rest data.
        let bones: HashMap<Bone, Entity> = bare.iter().collect();
        let mut rest_rotations = HashMap::new();
        let mut rest_directions = HashMap::new();
        for &bone in Bone::ALL.iter() {
            rest_rotations.insert(bone, real.rest_rotation(bone));
            rest_directions.insert(bone, real.rest_direction(bone));
        }

        let skeleton = HumanoidSkeleton::for_other_rig(
            bones,
            rest_rotations,
            rest_directions,
            Vec3::ZERO,
            real.hips_root_rotation(),
            Vec3::ONE,
            Bone::Hips.t_pose_world_position(),
        );

        (app, skeleton)
    }

    fn write(
        app: &mut App,
        skeleton: &HumanoidSkeleton,
        pose: &crate::character::anim::rig::LocalPose,
    ) {
        let world = app.world_mut();
        let mut state = world.query::<&mut Transform>();
        let mut query = state.query_mut(world);
        crate::character::anim::retarget::write_pose_to_skeleton(skeleton, pose, &mut query);
    }

    #[test]
    fn a_fresh_drag_state_is_idle() {
        let state = DragState::default();
        assert!(state.hovered.is_none());
        assert!(state.active.is_none());
    }

    #[test]
    fn the_camera_query_ignores_bevys_internal_shadow_view() {
        // The bug that made dragging completely inert: Bevy's shadow
        // mapping creates an internal view entity carrying a bare `Camera`
        // with no render graph. An unfiltered
        // `Query<(&Camera, &GlobalTransform)>` therefore matches TWO
        // entities, `single()` fails every frame, and `drag_bones` returns
        // before doing anything — while the handles kept drawing from a
        // different system, so nothing looked wrong.
        //
        // This is the same trap that once made the gallery's entire egui UI
        // render to an invisible camera.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);

        // The real scene camera.
        app.world_mut().spawn((Camera3d::default(), Camera::default(), Transform::default()));
        // A bare `Camera`, standing in for the shadow view.
        app.world_mut().spawn((Camera::default(), Transform::default()));

        let mut unfiltered =
            app.world_mut().query::<(&Camera, &GlobalTransform)>();
        assert_eq!(
            unfiltered.iter(app.world()).count(),
            2,
            "test setup: an unfiltered camera query should match both entities",
        );

        let mut filtered = app
            .world_mut()
            .query_filtered::<(&Camera, &GlobalTransform), With<Camera3d>>();
        assert_eq!(
            filtered.iter(app.world()).count(),
            1,
            "filtering by Camera3d must find exactly the scene camera, or `single()` \
             fails and dragging silently does nothing",
        );
    }

    #[test]
    fn a_drag_actually_rotates_the_bone_it_grabbed() {
        // End-to-end through the real maths, with real rig geometry: pick a
        // joint, aim it somewhere else, and confirm the bone moved and the
        // joint followed.
        //
        // Written because the previous round verified only that HANDLES
        // rendered — which they did, from a different system — while
        // dragging itself was inert.
        use crate::character::anim::poses;
        use crate::character::anim::rig::forward_kinematics;
        use crate::character::anim::studio::drag::aim_rotation;

        let mut pose = poses::relaxed_stand();
        let before = forward_kinematics(&pose);

        // Grab the elbow; that rotates the upper arm.
        let handle = Bone::LeftForeArm;
        let bone = bone_for_handle(handle).expect("the elbow has a parent to rotate");
        assert_eq!(bone, Bone::LeftArm);

        let pivot = before[bone];
        let current = before[handle];

        // Aim a quarter turn around the pivot, so the requested motion is
        // within what the bone's own length can reach. Asking for a
        // displacement larger than the bone is a legitimate drag — the limb
        // aims at it and stops at its own length — but then the joint's
        // travel is bounded by geometry rather than by the request, which
        // makes a fixed distance threshold meaningless.
        let arm = current - pivot;
        let target = pivot + Quat::from_axis_angle(Vec3::X, 0.6) * arm;

        // The bone's PARENT's accumulated world rotation. Passing identity
        // here is wrong whenever the parent is itself rotated — which in
        // `relaxed_stand` it is — and produces a right-magnitude,
        // wrong-axis result that lands centimetres off a reachable target.
        let mut parent_world = Quat::IDENTITY;
        let mut chain = Vec::new();
        let mut walk = bone.parent();
        while let Some(link) = walk {
            chain.push(link);
            walk = link.parent();
        }
        for &link in chain.iter().rev() {
            parent_world *= pose.rotation(link);
        }

        let rotation = aim_rotation(pivot, current, target, parent_world, pose.rotation(bone));
        pose.set_rotation(bone, rotation);

        let after = forward_kinematics(&pose);

        assert!(
            after[handle].distance(before[handle]) > 0.05,
            "the dragged joint should have moved, but went {:?} -> {:?}",
            before[handle],
            after[handle],
        );
        // A reachable target must be reached, not merely approached — that
        // is the difference between aiming and drifting.
        assert!(
            after[handle].distance(target) < 1.0e-4,
            "the dragged joint should have landed ON a reachable target: {:?} against \
             {target:?}",
            after[handle],
        );
        // And the pivot itself must not move — rotating a bone moves its
        // children, never itself.
        assert!(
            after[bone].distance(before[bone]) < 1.0e-5,
            "rotating {} must not move {} itself",
            bone.name(),
            bone.name(),
        );
    }
}
