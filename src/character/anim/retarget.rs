//! Writing a rig-independent [`LocalPose`] onto a concrete skeleton — the
//! one part of the superseded module's retargeting that survives, distilled
//! to its essentials.
//!
//! # What this replaced, and why it got so much smaller
//!
//! `muscle::plugin::apply_solved_sim_to_skeleton` is ~440 lines and the most
//! bug-scarred function in the project. Reading it closely, it does two
//! separable jobs:
//!
//! 1. **Inferring a rotation from two solved particle positions.** This is
//!    where nearly all of the length and all of the documented bugs live:
//!    `Quat::from_rotation_arc` yields a direction with *arbitrary roll*, so
//!    the bind pose's own roll had to be guessed back (three strategies
//!    tried, all three real screenshot-caught bugs); an inferred rotation
//!    belongs to the *parent*, forcing a write-onto-parent rule; a parent
//!    with three children gets three disagreeing answers, needing a
//!    `chain_continuation_child` tie-breaker; and an inferred direction lags
//!    a position blend, needing a `swing_weight` ramp to hide it.
//! 2. **Composing a rig-independent delta onto a specific rig's bind pose.**
//!
//! Job 1 exists *only* because rotation was reconstructed rather than
//! authored. Authoring rotations natively does not solve those problems — it
//! removes them. There is nothing to infer, no roll to recover, no parent to
//! disambiguate, and no lag to ramp over.
//!
//! Job 2 is orthogonal to how the pose was produced and is fully
//! load-bearing: it is what lets one authored pose drive our synthetic rig,
//! a Quaternius mesh, and a Mixamo mesh unchanged. It is kept here, and the
//! four hard-won bind-pose accessors it depends on
//! (`rest_rotation`, `hips_root_rotation`, `hips_local_translation_for` and
//! its scale correction) stay exactly as they are in
//! [`crate::character::skeleton`].
//!
//! What remains is essentially one line per bone:
//!
//! ```text
//! local_rotation(bone) = skeleton.rest_rotation(bone) * pose.rotation(bone)
//! ```
//!
//! # The standing decision about rest twist
//!
//! `muscle::plugin` records a decision worth carrying forward rather than
//! rediscovering: **a rig's own bind-pose roll must not be re-applied on top
//! of an animated rotation.** Two attempts to do so (unconditionally, then
//! faded by how far the bone had swung) were both real bugs, because a
//! natural roll depends on the whole chain's pose, not on one bone's local
//! constant — `puppet_base.gltf`'s `RightArm` alone carries ~90° of rest
//! roll, and re-applying it after the arm had swung elsewhere visibly
//! twisted the skin across the body.
//!
//! Here the question cannot arise: `rest_rotation` already *is* the bind
//! pose including its roll, and the authored delta is measured relative to
//! it. Roll is authored data, never reconstructed. This note exists so the
//! lesson is not re-learned by someone adding a "helpful" twist correction.

use bevy::prelude::*;

use super::rig::LocalPose;
use crate::character::skeleton::{Bone, HumanoidSkeleton};

/// Writes `pose` onto `skeleton`'s bone entities.
///
/// Each bone's `Transform.rotation` becomes its own bind rotation composed
/// with the pose's rest-relative delta, so an all-identity pose reproduces
/// the rig's bind pose exactly — bit-for-bit, with no blending and no
/// special case.
///
/// Only `Hips` gets a `Transform.translation`; every other bone keeps its
/// fixed rest translation and inherits its world position from ancestor
/// rotations via Bevy's `ChildOf` propagation. `scale` is never touched.
///
/// Must run before `TransformSystems::Propagate`.
pub fn write_pose_to_skeleton(
    skeleton: &HumanoidSkeleton,
    pose: &LocalPose,
    transforms: &mut Query<&mut Transform>,
) {
    let bind = accumulated_bind_rotations(skeleton);

    for &bone in Bone::ALL.iter() {
        let Ok(mut transform) = transforms.get_mut(skeleton.entity(bone)) else {
            continue;
        };

        transform.rotation =
            skeleton.rest_rotation(bone) * delta_in_bone_frame(bind[bone.index()], pose.rotation(bone));

        if bone == Bone::Hips {
            // The root is the only bone with no parent to inherit a position
            // from, so its translation has to be written explicitly — and
            // converted out of world space into whatever local space this
            // particular rig's hip joint lives in. `hips_local_translation_for`
            // carries a real fix for a scaled ancestor chain (Blender's 0.01
            // FBX correction node); do not bypass it.
            transform.translation =
                skeleton.hips_local_translation_for(hips_world_position(skeleton, pose));
        }
    }
}

/// Each bone's accumulated bind rotation — the product of every
/// `rest_rotation` from the root down to and including that bone, seeded
/// with the rig's own ancestor rotation above `Hips`.
///
/// Indexed by [`Bone::index`]. Computed in one pass because [`Bone::ALL`]
/// lists every parent before its children.
///
/// This is the bone's orientation in world space when the rig is in its
/// bind pose, and it is exactly the frame an authored delta has to be
/// converted into — see [`delta_in_bone_frame`].
fn accumulated_bind_rotations(skeleton: &HumanoidSkeleton) -> [Quat; 22] {
    let mut accumulated = [Quat::IDENTITY; 22];

    for &bone in Bone::ALL.iter() {
        let parent_rotation = match bone.parent() {
            Some(parent) => accumulated[parent.index()],
            // Above `Hips` sits whatever ancestor chain the rig was spawned
            // under — for `puppet_base.gltf`, a Blender Z-up import
            // correction composed with the example's facing yaw.
            None => skeleton.hips_root_rotation(),
        };

        accumulated[bone.index()] = parent_rotation * skeleton.rest_rotation(bone);
    }

    accumulated
}

/// Re-expresses a world-axis rotation delta in a bone's own local frame.
///
/// # The frame mismatch this fixes
///
/// An authored delta says "turn 71 degrees about +Z", where +Z means the
/// world axis — poses are authored against this crate's synthetic T-pose,
/// whose bind rotations are all identity, so local and world axes coincide
/// there.
///
/// A real rig is different. `Transform.rotation` is interpreted in the
/// bone's *parent* local space, and a rendered bone's orientation is
/// `accumulated_bind(parent) * rest_rotation(bone) * delta`. So the delta
/// lands in the bone's own local frame — which, on `puppet_base.gltf`, is
/// rotated far from world axes (`pelvis` alone carries ~106 degrees about
/// X, and each clavicle a full four-component rotation).
///
/// Applying a world-axis delta there turns the right *amount* about the
/// wrong *axis*. Measured consequence: `relaxed_stand` rendered with both
/// arms raised overhead instead of hanging at the sides, while every unit
/// test passed.
///
/// The fix is the standard change of basis — conjugate the delta by the
/// bone's accumulated bind rotation:
///
/// ```text
/// delta_local = bind⁻¹ * delta_world * bind
/// ```
///
/// # How this was pinned down
///
/// Two earlier hypotheses were wrong, and both were killed by measurement
/// rather than argument, which is worth recording:
///
/// - *"The rigs' bone axes disagree, so align by `rest_direction`."*
///   `rest_direction` is a **parent-local** translation direction (a glTF
///   convention where bones run along local `+Y`), not a world direction;
///   conjugating a world vector against it compares two different spaces.
/// - *"The rigs' world-frame bone directions disagree."* Querying the live
///   rig over BRP settled it: in the bind pose the left arm runs
///   `(-1.000, 0.000, 0.030)`, against the synthetic T-pose's
///   `(-1.000, 0.000, 0.000)`. They **match**, so no direction alignment
///   was needed at all — the mismatch was in the frame the delta is
///   *applied* in, not the direction the bone points.
///
/// For a rig with no captured bind pose every `rest_rotation` is identity,
/// `bind` is identity, and this is a no-op — pinned by
/// `frame_conversion_is_a_no_op_on_a_rig_with_no_captured_bind_pose`.
#[inline]
fn delta_in_bone_frame(bind: Quat, delta: Quat) -> Quat {
    if delta == Quat::IDENTITY {
        // Identity in any frame is identity. Short-circuited so the rest
        // pose reproduces the bind pose bit-for-bit, with no float drift.
        return delta;
    }

    bind.inverse() * delta * bind
}

/// Where `pose` puts the root, in world space, before the conversion into
/// the target rig's own local hip space.
///
/// Split out so tests can assert the world-space intent independently of
/// each rig's local-space conversion.
#[inline]
pub fn hips_world_position(skeleton: &HumanoidSkeleton, pose: &LocalPose) -> Vec3 {
    // The root's own rest world position, displaced by the pose's root
    // translation AS IS: like the pose's rotations, the translation names
    // world axes, and `hips_local_translation_for` is what converts it into
    // the hips' parent's frame.
    //
    // This used to rotate it by `hips_root_rotation` first — a relic of when
    // TRAVEL lived in the root translation and had to be turned into the
    // character's facing. Composed with the conversion's inverse of that
    // same rest rotation, the two cancelled and the raw vector landed in
    // the parent's LOCAL axes; on a Z-up armature local +Y is horizontal.
    // Live, the gait's vertical bob moved the pelvis 63.4 mm along the
    // direction of travel and 0.0 mm up. Travel is on the entity now (see
    // the gallery's walk driver), so nothing needs the rotation.
    let _ = skeleton;
    Bone::Hips.t_pose_world_position() + pose.root_translation
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::character::anim::rig::LocalPose;
    use std::f32::consts::FRAC_PI_2;

    /// A world with one bare entity per bone and a plain (non-retargeted)
    /// skeleton. Reuses `skeleton`'s own test builder and the established
    /// `CommandQueue` pattern, so this stays in step with how the rest of
    /// the project stands up a bare rig.
    fn bare_skeleton_world() -> (World, HumanoidSkeleton) {
        let mut world = World::new();
        let mut queue = bevy::ecs::world::CommandQueue::default();
        let mut commands = Commands::new(&mut queue, &world);
        let (_root, skeleton) = crate::character::skeleton::tests::spawn_bare_bone_entities(
            &mut commands,
            Transform::IDENTITY,
        );
        queue.apply(&mut world);
        (world, skeleton)
    }

    fn write(world: &mut World, skeleton: &HumanoidSkeleton, pose: &LocalPose) {
        let mut state = world.query::<&mut Transform>();
        let mut query = state.query_mut(world);
        write_pose_to_skeleton(skeleton, pose, &mut query);
    }

    fn rotation_of(world: &mut World, skeleton: &HumanoidSkeleton, bone: Bone) -> Quat {
        world.get::<Transform>(skeleton.entity(bone)).unwrap().rotation
    }

    // -----------------------------------------------------------------
    // The retargeting path, against the REAL rig's bind pose
    //
    // These close a gap this project carried knowingly. Everything else in
    // this module is exercised on a synthetic skeleton whose every bind
    // rotation is identity — where the frame conversions this module exists
    // to perform are all no-ops. A rig with real bind rotations is where
    // they either work or do not, and until now that could only be checked
    // by running the game.
    //
    // Still NOT covered, deliberately: the glTF LOADER. These parse the
    // asset's JSON directly, so they verify the retargeting maths against
    // the bind pose the FILE declares — not that `bevy_gltf` reproduces that
    // bind pose when it loads it. That one genuinely needs the live app.
    // -----------------------------------------------------------------

    /// A world with one entity per bone, and a skeleton carrying the real
    /// rig's bind pose.
    fn real_rig_world() -> (World, HumanoidSkeleton) {
        use crate::character::anim::gltf_rig;

        let mut world = World::new();
        let mut entities = std::collections::HashMap::new();

        for &bone in Bone::ALL.iter() {
            entities.insert(bone, world.spawn(Transform::IDENTITY).id());
        }

        let skeleton = gltf_rig::real_skeleton(&gltf_rig::parsed_rig(), entities);
        (world, skeleton)
    }

    #[test]
    fn the_real_rigs_rest_pose_writes_its_own_bind_rotations() {
        // The same property the synthetic test asserts, on a rig where it is
        // not trivially true: `puppet_base` binds its foot about 70 degrees
        // off and its thigh about 164, so "the rest pose reproduces the bind
        // pose" is a real claim here rather than an identity.
        let (mut world, skeleton) = real_rig_world();

        write(&mut world, &skeleton, &LocalPose::REST);

        for &bone in Bone::ALL.iter() {
            let written = rotation_of(&mut world, &skeleton, bone);
            let expected = skeleton.rest_rotation(bone);

            assert!(
                written.abs_diff_eq(expected, 1.0e-6),
                "{} wrote {written:?} against its own bind rotation {expected:?}",
                bone.name(),
            );
        }
    }

    /// Every bone's world rotation as the RENDERER would compute it, by
    /// accumulating the local rotations `write_pose_to_skeleton` actually
    /// wrote.
    ///
    /// The point of going through the `World` rather than re-deriving the
    /// composition is that this is the real writer's output, not a model of
    /// it — a hand-composed "expected" chain is only ever a restatement of
    /// whichever convention its author believed in.
    fn rendered_world_rotations(
        world: &mut World,
        skeleton: &HumanoidSkeleton,
        pose: &LocalPose,
    ) -> crate::character::anim::rig::BoneSet<Quat> {
        use crate::character::anim::rig::BoneSet;

        write(world, skeleton, pose);

        let mut rendered = BoneSet::splat(Quat::IDENTITY);
        for &bone in Bone::ALL.iter() {
            let parent = match bone.parent() {
                Some(parent) => rendered[parent],
                None => skeleton.hips_root_rotation(),
            };
            rendered[bone] = parent * rotation_of(world, skeleton, bone);
        }

        rendered
    }

    #[test]
    fn the_world_rotations_agree_with_what_retargeting_actually_writes() {
        // The seam this project got wrong for a long time. `rig::
        // accumulate_world_rotations` is what every IK solver reasons about;
        // `write_pose_to_skeleton` is what the renderer draws. If they disagree
        // then a solver can hit its target perfectly and the character still
        // puts its hand somewhere else — which is exactly what happened:
        // arm IK solved a wrist to within 5e-8 m of a world target while the
        // live skeleton rendered it 0.36 m away.
        //
        // It went unnoticed because the disagreement is invisible to the one
        // question the legs ever ask. Conjugating a delta by a bind rotation
        // about the SAME axis is a no-op (parallel rotations commute), the leg
        // chain is bound about X, and hip pitch and knee bend are about X too.
        // So the deltas are chosen here to be about axes each bone's bind
        // rotation genuinely moves — otherwise this test passes vacuously.
        use crate::character::anim::gltf_rig;
        use crate::character::anim::rig::accumulate_world_rotations;

        let (mut world, skeleton) = real_rig_world();
        let rig = gltf_rig::puppet_base();

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, 0.4));
        pose.set_rotation(Bone::LeftForeArm, Quat::from_axis_angle(Vec3::X, -0.5));
        pose.set_rotation(Bone::LeftLeg, Quat::from_axis_angle(Vec3::Y, 0.3));
        pose.set_rotation(Bone::Spine1, Quat::from_axis_angle(Vec3::Y, 0.25));

        let solved = accumulate_world_rotations(&pose, &rig);
        let rendered = rendered_world_rotations(&mut world, &skeleton, &pose);

        for &bone in Bone::ALL.iter() {
            // Signed-free magnitude of the disagreement. `Quat::angle_between`
            // is avoided deliberately: it has a ~9.8e-4 rad precision floor
            // near identity, which is the regime this test lives in.
            let difference = solved[bone].inverse() * rendered[bone];
            let error = 2.0 * difference.w.abs().clamp(0.0, 1.0).acos();

            assert!(
                error < 5.0e-3,
                "{} : the solver thinks {:?}, the renderer draws {:?} \
                 — {:.3} degrees apart",
                bone.name(),
                solved[bone],
                rendered[bone],
                error.to_degrees(),
            );
        }
    }

    #[test]
    fn the_agreement_test_is_not_vacuous_on_the_axes_it_picks() {
        // Guards the test above against the failure mode that hid the bug for
        // months: if every delta's axis happens to be parallel to its bone's
        // bind axis, the conjugation is a no-op and the comparison holds no
        // matter which convention either side uses.
        //
        // So: assert that each chosen delta axis is one the bone's accumulated
        // bind rotation really does move. Measured on `puppet_base`, the
        // smallest of these is ~19 degrees.
        use crate::character::anim::gltf_rig;

        let (_, skeleton) = real_rig_world();
        let _ = gltf_rig::puppet_base();
        let bind = accumulated_bind_rotations(&skeleton);

        // `Spine1` is deliberately absent. Its accumulated bind on
        // `puppet_base` is only 1.29 degrees from identity, so it moves EVERY
        // axis by ~1.29 degrees and no delta through it can discriminate
        // between the two conventions. It stays in the agreement test above as
        // coverage that an almost-unbound bone still round-trips; it just
        // cannot carry the vacuity argument.
        for (bone, axis) in [
            (Bone::LeftArm, Vec3::Y),
            (Bone::LeftForeArm, Vec3::X),
            (Bone::LeftLeg, Vec3::Y),
        ] {
            let frame = bind[bone.index()];
            let moved = frame.inverse() * axis;
            let divergence = moved.angle_between(axis).to_degrees();

            assert!(
                divergence > 10.0,
                "{}'s bind rotation moves a {axis:?} delta by only {divergence:.2} degrees, \
                 so conjugating it is nearly a no-op and the agreement test proves nothing",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_delta_on_the_real_rig_is_conjugated_into_the_bones_own_frame() {
        // THE thing `delta_in_bone_frame` exists for: a pose's rotations are
        // authored in this crate's T-pose convention, so applying one
        // directly to a bone bound 70 degrees off gives the right angle
        // about the wrong axis.
        //
        // The axis matters, and picking it carelessly makes this test
        // vacuous. A delta about X conjugated by a bind rotation that is
        // ITSELF about X is a no-op — parallel rotations commute — and the
        // leg chain here is bound almost entirely about X. So the delta is
        // about Y, which that bind rotation genuinely moves.
        let (mut world, skeleton) = real_rig_world();

        let delta = Quat::from_axis_angle(Vec3::Y, 0.4);
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftFoot, delta);

        write(&mut world, &skeleton, &pose);

        let written = rotation_of(&mut world, &skeleton, Bone::LeftFoot);
        let naive = skeleton.rest_rotation(Bone::LeftFoot) * delta;

        assert!(
            !written.abs_diff_eq(naive, 1.0e-3),
            "the written rotation matches the UNCONJUGATED composition, so either \
             the frame conversion is not happening or this delta's axis is one the \
             bind rotation cannot move",
        );

        // And it is exactly the conjugated one.
        let bind = accumulated_bind_rotations(&skeleton);
        let expected = skeleton.rest_rotation(Bone::LeftFoot)
            * delta_in_bone_frame(bind[Bone::LeftFoot.index()], delta);

        assert!(written.abs_diff_eq(expected, 1.0e-6), "got {written:?}");
    }

    #[test]
    fn the_real_rigs_hip_translation_round_trips_through_its_own_frame() {
        // `hips_local_translation_for` converts a solved WORLD hip position
        // into whatever local frame this rig's hip joint translates in,
        // undoing the parent's rotation and scale.
        //
        // The round trip is the property. On `puppet_base` the parent
        // carries a real -90 degree rotation (the Z-up-to-Y-up correction),
        // so this is not the identity case.
        use crate::character::anim::gltf_rig;

        let (_, skeleton) = real_rig_world();
        let rig = gltf_rig::parsed_rig();

        for offset in [Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), Vec3::new(0.3, -0.1, 2.5)] {
            let world_position = Bone::Hips.t_pose_world_position() + offset;
            let local = skeleton.hips_local_translation_for(world_position);

            // The inverse of what the method does.
            let back = rig.hips_parent_rest_world_rotation
                * ((local - rig.hips_rest_local_translation)
                    * rig.hips_parent_rest_world_scale)
                + Bone::Hips.t_pose_world_position();

            assert!(
                back.abs_diff_eq(world_position, 1.0e-4),
                "offset {offset:?} converted to {local:?} and back to {back:?}",
            );
        }
    }

    #[test]
    fn a_scaled_parent_chain_is_divided_out_of_the_hip_translation() {
        // The documented once-live bug: Blender's FBX-cm-to-glTF-m
        // correction leaves a 0.01 scale above the hips, so a world-metre
        // delta must be divided by it to become the local units the hip
        // joint actually translates in. Without that the character moves
        // 100x too far.
        //
        // `puppet_base`'s own chain is unit-scaled, so this OVERRIDES the
        // scale rather than pretending the asset exercises the case — and
        // asserts that it is unit-scaled first, so the override cannot
        // silently mask a change.
        use crate::character::anim::gltf_rig;

        let mut rig = gltf_rig::parsed_rig();
        assert_eq!(
            rig.hips_parent_rest_world_scale,
            Vec3::ONE,
            "puppet_base is expected to be unit-scaled; if it is not, this test's \
             override is hiding something",
        );

        rig.hips_parent_rest_world_scale = Vec3::splat(0.01);

        let mut world = World::new();
        let mut entities = std::collections::HashMap::new();
        for &bone in Bone::ALL.iter() {
            entities.insert(bone, world.spawn(Transform::IDENTITY).id());
        }
        let skeleton = gltf_rig::real_skeleton(&rig, entities);

        let moved = Bone::Hips.t_pose_world_position() + Vec3::new(0.0, 0.0, -1.0);
        let local = skeleton.hips_local_translation_for(moved);
        let displacement = local - rig.hips_rest_local_translation;

        assert!(
            (displacement.length() - 100.0).abs() < 1.0,
            "a 1 m world move under a 0.01 parent scale should be 100 local units, \
             got {}",
            displacement.length(),
        );
    }

    #[test]
    fn the_real_rig_renders_its_bind_pose_upright_through_bevy_propagation() {
        // End to end through a real `World`: build the rig's own hierarchy,
        // write the rest pose, let Bevy propagate, and check the result is
        // person-shaped. This is the step that previously needed the game.
        //
        // The ROOT ENTITY carries the rig's own correction, and leaving it
        // at identity is what makes this fail: the offsets and bind
        // rotations are both glTF-local, in the asset's Z-up frame, so
        // without the -90 degree X node above them the whole rig renders
        // lying down — measured head y = 0.017 against foot y = 0.088, which
        // is CORRECT for a Z-up rig and wrong for a Y-up world.
        use crate::character::anim::gltf_rig;
        use bevy::prelude::*;

        let mut app = App::new();
        app.add_plugins(bevy::MinimalPlugins);
        app.add_plugins(bevy::transform::TransformPlugin);

        let rig = gltf_rig::parsed_rig();
        let geometry = gltf_rig::puppet_base();

        let world = app.world_mut();

        // Everything above `Hips` in the real asset, as one entity.
        let root = world
            .spawn(Transform::from_rotation(rig.hips_parent_rest_world_rotation))
            .id();

        let mut entities = std::collections::HashMap::new();
        for &bone in Bone::ALL.iter() {
            let parent = bone.parent().map(|p| entities[&p]).unwrap_or(root);
            let entity = world
                .spawn((
                    Transform::from_translation(geometry.offsets[bone]),
                    ChildOf(parent),
                ))
                .id();
            entities.insert(bone, entity);
        }

        let skeleton = gltf_rig::real_skeleton(&rig, entities.clone());

        {
            let mut state = world.query::<&mut Transform>();
            let mut query = state.query_mut(world);
            write_pose_to_skeleton(&skeleton, &LocalPose::REST, &mut query);
        }

        app.update();

        let world = app.world();
        let at = |bone: Bone| {
            world.get::<GlobalTransform>(entities[&bone]).unwrap().translation()
        };

        let (head, foot) = (at(Bone::Head), at(Bone::LeftFoot));

        assert!(
            head.y > foot.y,
            "the head ({}) should render above the foot ({})",
            head.y,
            foot.y,
        );
        assert!(
            (0.8..2.5).contains(&(head.y - foot.y)),
            "head-to-foot spans {} m, which is not a human-scale rig",
            head.y - foot.y,
        );

        // And left/right are genuinely mirrored, which catches a correction
        // that happens to stand the rig up while twisting it.
        let (left, right) = (at(Bone::LeftFoot), at(Bone::RightFoot));
        assert!(
            (left.x + right.x).abs() < 0.05,
            "the feet should straddle the centreline, got {} and {}",
            left.x,
            right.x,
        );
    }

    #[test]
    fn the_forward_kinematics_positions_are_the_ones_bevy_renders() {
        // The other half of
        // `the_world_rotations_agree_with_what_retargeting_actually_writes`,
        // and the half whose absence let a real bug through: that test compares
        // ROTATIONS, and rotations agreeing does not make positions agree.
        //
        // Positions are what an IK solver actually aims at — `solve_arm_on`
        // returns a position and every target is a point — so a solver that
        // reasons in a frame whose positions differ from the rendered ones puts
        // the hand somewhere else however right its rotations are. Measured
        // before this test existed: the FK chain had the left arm running along
        // -Y while the rendered skeleton had it along -X.
        use crate::character::anim::gltf_rig;
        use crate::character::anim::rig::forward_kinematics_on;
        use bevy::prelude::*;

        let mut app = App::new();
        app.add_plugins(bevy::MinimalPlugins);
        app.add_plugins(bevy::transform::TransformPlugin);

        let parsed = gltf_rig::parsed_rig();
        let rig = gltf_rig::puppet_base();
        let world = app.world_mut();

        let root = world
            .spawn(Transform::from_rotation(parsed.hips_parent_rest_world_rotation))
            .id();

        let mut entities = std::collections::HashMap::new();
        for &bone in Bone::ALL.iter() {
            let parent = bone.parent().map(|p| entities[&p]).unwrap_or(root);
            let entity = world
                .spawn((Transform::from_translation(rig.offsets[bone]), ChildOf(parent)))
                .id();
            entities.insert(bone, entity);
        }

        let skeleton = gltf_rig::real_skeleton(&parsed, entities.clone());

        // A pose with real rotations, on axes the binds genuinely move — the
        // rest pose would let a frame error hide.
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, 0.4));
        pose.set_rotation(Bone::LeftForeArm, Quat::from_axis_angle(Vec3::X, -0.5));
        pose.set_rotation(Bone::LeftLeg, Quat::from_axis_angle(Vec3::Y, 0.3));
        assert!(
            pose.rotation(Bone::LeftArm) != Quat::IDENTITY,
            "the pose has to be non-trivial or this test cannot fail",
        );

        {
            let mut state = world.query::<&mut Transform>();
            let mut query = state.query_mut(world);
            write_pose_to_skeleton(&skeleton, &pose, &mut query);
        }
        app.update();

        let world = app.world();
        let rendered = |bone: Bone| {
            world.get::<GlobalTransform>(entities[&bone]).unwrap().translation()
        };
        let solved = forward_kinematics_on(&pose, &rig);

        // Compared as offsets from the hip, not as absolute points: the two
        // frames deliberately place the root differently
        // (`write_pose_to_skeleton` puts the hips at
        // `Hips::t_pose_world_position()`, the FK chain at its own root), and
        // that difference is a translation the caller already calibrates out.
        // What must match is the SHAPE.
        let rendered_hip = rendered(Bone::Hips);
        let solved_hip = solved[Bone::Hips];

        for &bone in Bone::ALL.iter() {
            let from_render = rendered(bone) - rendered_hip;
            let from_solve = solved[bone] - solved_hip;
            let error = (from_render - from_solve).length();

            assert!(
                error < 1.0e-4,
                "{} : the renderer puts it {from_render:?} from the hip, the \
                 solver {from_solve:?} — {error} m apart",
                bone.name(),
            );
        }
    }

    #[test]
    fn the_rest_pose_writes_each_bones_own_bind_rotation_exactly() {
        // The property that makes "untouched bones render their own bind
        // pose" free rather than a special case: with an identity delta the
        // composition collapses to `rest_rotation` itself, bit-for-bit.
        let (mut world, skeleton) = bare_skeleton_world();

        write(&mut world, &skeleton, &LocalPose::REST);

        for &bone in Bone::ALL.iter() {
            let written = rotation_of(&mut world, &skeleton, bone);
            assert_eq!(
                written,
                skeleton.rest_rotation(bone),
                "{} should render its own bind rotation under the rest pose",
                bone.name(),
            );
        }
    }

    #[test]
    fn an_authored_delta_composes_onto_the_bones_own_bind_rotation() {
        let (mut world, skeleton) = bare_skeleton_world();

        let delta = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, delta);

        write(&mut world, &skeleton, &pose);

        let written = rotation_of(&mut world, &skeleton, Bone::LeftArm);
        let expected = skeleton.rest_rotation(Bone::LeftArm) * delta;

        assert!(
            written.abs_diff_eq(expected, 1.0e-6),
            "expected rest_rotation * delta = {expected:?}, got {written:?}",
        );
    }

    #[test]
    fn writing_a_pose_touches_only_rotation_except_at_the_root() {
        // Pins the "only Hips gets a translation, nobody gets a scale"
        // contract. A stray translation write would silently detach a bone
        // from its rest offset and break every length assumption downstream.
        let (mut world, skeleton) = bare_skeleton_world();

        let before: Vec<_> = Bone::ALL
            .iter()
            .map(|&bone| {
                let t = world.get::<Transform>(skeleton.entity(bone)).unwrap();
                (bone, t.translation, t.scale)
            })
            .collect();

        let mut pose = LocalPose::REST;
        for &bone in Bone::ALL.iter() {
            pose.set_rotation(bone, Quat::from_axis_angle(Vec3::X, 0.3));
        }
        write(&mut world, &skeleton, &pose);

        for (bone, translation, scale) in before {
            let after = world.get::<Transform>(skeleton.entity(bone)).unwrap();

            assert_eq!(after.scale, scale, "{}'s scale must never be written", bone.name());

            if bone != Bone::Hips {
                assert_eq!(
                    after.translation, translation,
                    "{}'s translation must be left at its rest offset",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn the_root_translation_is_written_to_hips() {
        let (mut world, skeleton) = bare_skeleton_world();

        let delta = Vec3::new(0.0, 0.0, -2.0);
        write(&mut world, &skeleton, &LocalPose::at_root(delta));

        let written = world.get::<Transform>(skeleton.entity(Bone::Hips)).unwrap().translation;
        let expected = Bone::Hips.t_pose_world_position() + delta;

        assert!(
            (written - expected).length() < 1.0e-5,
            "Hips should be written at {expected:?}, got {written:?}",
        );
    }

    #[test]
    fn a_rig_with_a_rotated_root_moves_the_root_in_its_own_frame() {
        // Guards the `hips_root_rotation` composition in
        // `hips_world_position`. A mesh whose root carries a 180-degree yaw
        // correction must still walk FORWARD when the pose says forward —
        // the documented failure mode here was a character translating
        // sideways (or backwards) once its rig gained a root correction.
        let (_, plain) = bare_skeleton_world();

        let forward = Vec3::new(0.0, 0.0, -1.0);
        let pose = LocalPose::at_root(forward);

        let plain_position = hips_world_position(&plain, &pose);
        assert!(
            (plain_position - (Bone::Hips.t_pose_world_position() + forward)).length() < 1.0e-5,
            "an unrotated rig should move straight along the pose's own root delta",
        );
    }

    #[test]
    fn every_bone_is_written_exactly_once() {
        // The superseded module needed skip rules (multi-child parents, a
        // Hips exemption) to avoid writing a bone twice with conflicting
        // values. Here the mapping is one-to-one by construction; this test
        // pins that, so a future refactor cannot quietly reintroduce the
        // ambiguity.
        let (mut world, skeleton) = bare_skeleton_world();

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::Y, 0.5));
        pose.set_rotation(Bone::Neck, Quat::from_axis_angle(Vec3::X, 0.2));
        pose.set_rotation(Bone::LeftUpLeg, Quat::from_axis_angle(Vec3::Z, 0.4));

        write(&mut world, &skeleton, &pose);

        // Each bone reflects ITS OWN delta, not a neighbour's — which is
        // what "written exactly once, by itself" means observationally.
        for &bone in Bone::ALL.iter() {
            let written = rotation_of(&mut world, &skeleton, bone);
            let expected = skeleton.rest_rotation(bone) * pose.rotation(bone);
            assert!(
                written.abs_diff_eq(expected, 1.0e-6),
                "{} should carry exactly its own authored delta",
                bone.name(),
            );
        }
    }

    /// The same world as [`bare_skeleton_world`], but carrying
    /// `puppet_base.gltf`'s own REAL bind pose — large, multi-axis
    /// rotations, nothing like identity (`LeftShoulder` carries a genuine
    /// four-component rotation; `RightArm` about 90 degrees of roll around
    /// its own bone axis).
    ///
    /// The values are BRP-captured from the live example rather than
    /// synthesized, because the retargeting bugs this composition has to
    /// avoid only appear against real bind data — a synthetic rig never
    /// exercises `rest_rotation` at all, so it is structurally blind to
    /// them. They were copied here rather than imported from the
    /// superseded module's fixture specifically so this coverage would
    /// survive that module's deletion, which it did.
    ///
    /// `pub(crate)` so the studio's drag tests can use it too. That is not
    /// convenience: a synthetic rig has identity rest rotations and is
    /// therefore structurally blind to every frame bug retargeting can
    /// have, so any test about frames needs real bind data or it proves
    /// nothing.
    pub(crate) fn real_mesh_skeleton_world() -> (World, HumanoidSkeleton) {
        use std::collections::HashMap;

        let (world, bare) = bare_skeleton_world();
        let bones: HashMap<Bone, Entity> = bare.iter().collect();

        let mut rest_rotations = HashMap::new();
        let mut rest_directions = HashMap::new();
        let mut insert = |bone: Bone, rotation: [f32; 4], direction: [f32; 3]| {
            rest_rotations.insert(
                bone,
                Quat::from_xyzw(rotation[0], rotation[1], rotation[2], rotation[3]),
            );
            rest_directions.insert(bone, Vec3::new(direction[0], direction[1], direction[2]));
        };

        insert(Bone::Hips, [0.800571, 0.0, 0.0, 0.5992379], [0.0, 0.045259636, 0.9989753]);
        insert(Bone::Spine, [-0.0918562, 0.0, 0.0, 0.9957723], [0.0, 1.0, -2.544752e-7]);
        insert(Bone::Spine1, [-0.039585352, 0.0, 0.0, 0.9992162], [0.0, 1.0, -1.3135752e-8]);
        insert(Bone::Spine2, [-0.12418899, 0.0, 0.0, 0.99225855], [0.0, 1.0, 0.0]);
        insert(
            Bone::LeftShoulder,
            [-0.5287374, -0.32910192, -0.41380805, 0.6639967],
            [0.16619363, 0.9194512, 0.35635543],
        );
        insert(
            Bone::LeftArm,
            [0.14980303, 0.69113076, -0.14938845, 0.6910719],
            [-0.043559864, 0.98098046, -0.1891556],
        );
        insert(
            Bone::LeftForeArm,
            [0.030732227, 4.1617506e-5, -1.2497959e-6, 0.9995277],
            [1.5071045e-7, 1.0, -2.0514012e-8],
        );
        insert(
            Bone::LeftHand,
            [-0.01560166, -4.1717584e-7, -9.0512575e-9, 0.9998783],
            [-6.208601e-7, 0.99999994, 1.1081928e-8],
        );
        insert(
            Bone::RightShoulder,
            [-0.5287374, 0.32910192, 0.41380805, 0.6639967],
            [-0.16619363, 0.9194512, 0.35635543],
        );
        insert(
            Bone::RightArm,
            [0.14980301, -0.69113076, 0.14938845, 0.6910719],
            [0.043559864, 0.98098046, -0.1891556],
        );
        insert(
            Bone::RightForeArm,
            [0.030732227, -4.167708e-5, 1.2516854e-6, 0.9995277],
            [-1.2293965e-7, 1.0, -1.141599e-8],
        );
        insert(
            Bone::RightHand,
            [-0.01560166, 4.7677682e-7, 8.121838e-9, 0.9998783],
            [6.2085496e-7, 0.99999994, 8.3419e-9],
        );

        // The real ancestor rotation above `Hips`: `puppet_base.gltf`'s own
        // Blender-Z-up glTF-import correction composed with the example's
        // 180-degree facing yaw. Also read directly via BRP.
        //
        // The two ~0.7071 components are MEASURED values that happen to sit
        // near `FRAC_1_SQRT_2`, not that constant — note they differ from
        // each other in the last digits. Substituting the exact constant
        // would quietly edit captured data, so clippy's suggestion is
        // declined here rather than applied.
        #[allow(clippy::approx_constant)]
        let hips_root_rotation =
            Quat::from_xyzw(3.0908616e-8, 0.70710677, 0.7071067, -3.090862e-8);

        let skeleton = HumanoidSkeleton::for_other_rig(
            bones,
            rest_rotations,
            rest_directions,
            Vec3::ZERO,
            hips_root_rotation,
            Vec3::ONE,
            Bone::Hips.t_pose_world_position(),
        );

        (world, skeleton)
    }

    #[test]
    fn a_real_bind_pose_is_reproduced_exactly_by_the_rest_pose() {
        // The load-bearing property of the whole composition, on a REAL rig:
        // an all-identity pose must render the artist's bind pose
        // bit-for-bit. Under the superseded position-inference path this was
        // not free — it needed a swing-weight ramp to approximate, because an
        // inferred direction carries no roll. Here it is exact.
        let (mut world, skeleton) = real_mesh_skeleton_world();

        write(&mut world, &skeleton, &LocalPose::REST);

        for &bone in Bone::ALL.iter() {
            let written = rotation_of(&mut world, &skeleton, bone);
            assert_eq!(
                written,
                skeleton.rest_rotation(bone),
                "{} must reproduce its real bind rotation exactly under the rest pose",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_real_rigs_bind_roll_is_preserved_under_an_authored_swing() {
        // The regression this module's doc comment warns about, stated as a
        // property. `RightArm`'s real bind pose carries ~90 degrees of roll
        // about its own axis. Re-applying that roll on top of an animated
        // rotation was tried twice in the superseded module and was a real,
        // screenshot-caught bug both times (it twisted the skin across the
        // body). Composing `rest * delta` keeps the roll exactly once — it
        // lives in `rest_rotation`, and the delta is measured relative to it.
        let (mut world, skeleton) = real_mesh_skeleton_world();

        let rest = skeleton.rest_rotation(Bone::RightArm);
        let swing = Quat::from_axis_angle(Vec3::X, -FRAC_PI_2);

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::RightArm, swing);
        write(&mut world, &skeleton, &pose);

        let written = rotation_of(&mut world, &skeleton, Bone::RightArm);

        // The delta is re-expressed in the rig's own frame before being
        // composed (see `delta_in_rig_frame`), so `written` is NOT literally
        // `rest * swing`. What must hold is that the *amount* of rotation is
        // unchanged: conjugation by `align` rotates the axis into the rig's
        // frame without altering the angle. An extra twist term — the bug
        // this guards — would change it.
        let applied = rest.inverse() * written;
        assert!(
            (applied.angle_between(Quat::IDENTITY) - FRAC_PI_2).abs() < 1.0e-4,
            "the rig should receive exactly the authored 90-degree swing, got {} degrees \
             — an extra twist term has crept back in",
            applied.angle_between(Quat::IDENTITY).to_degrees(),
        );

        // And the bind roll itself is still there, exactly once: an
        // identity delta must reproduce the bind pose bit-for-bit.
        write(&mut world, &skeleton, &LocalPose::REST);
        assert_eq!(
            rotation_of(&mut world, &skeleton, Bone::RightArm),
            rest,
            "the bind roll must be preserved exactly, not re-applied or faded",
        );
    }

    // `a_real_rigs_root_delta_is_applied_in_its_own_rotated_frame` used to
    // live here. It asserted `hips_world_position` shifted by
    // `hips_root_rotation * delta` — the implementation restated — and so
    // required a FORWARD delta to come out as world DOWN on this Z-up rig,
    // the very bug it was written to guard. Superseded by
    // `a_root_translation_moves_the_rendered_hips_along_the_poses_own_axes`,
    // which follows the write through the real parent chain to where the
    // hips actually render.

    /// `puppet_base` as a PARENTED hierarchy under a scene root, from its
    /// parsed bind pose — so a written pose can be followed all the way to
    /// where it renders, through the Z-up armature above the hips.
    fn real_rig_hierarchy() -> (World, HumanoidSkeleton, Entity) {
        use crate::character::anim::gltf_rig;

        let parsed = gltf_rig::parsed_rig();
        let rig = gltf_rig::puppet_base();
        let mut world = World::new();
        let armature = world
            .spawn(
                Transform::from_rotation(parsed.hips_parent_rest_world_rotation)
                    .with_scale(parsed.hips_parent_rest_world_scale),
            )
            .id();
        let mut entities = std::collections::HashMap::new();
        for &bone in Bone::ALL.iter() {
            let parent = bone.parent().map_or(armature, |parent| entities[&parent]);
            let translation =
                if bone == Bone::Hips { parsed.hips_rest_local_translation } else { rig.offsets[bone] };
            let entity = world
                .spawn((
                    Transform::from_translation(translation).with_rotation(rig.bind_rotations[bone]),
                    ChildOf(parent),
                ))
                .id();
            entities.insert(bone, entity);
        }
        let skeleton = gltf_rig::real_skeleton(&parsed, entities);
        (world, skeleton, armature)
    }

    /// Where `entity` renders: its transform composed with every ancestor's.
    fn rendered_position(world: &World, entity: Entity) -> Vec3 {
        let mut transform = *world.get::<Transform>(entity).unwrap();
        let mut current = entity;
        while let Some(parent) = world.get::<ChildOf>(current).map(ChildOf::parent) {
            transform = world.get::<Transform>(parent).unwrap().mul_transform(transform);
            current = parent;
        }
        transform.translation
    }

    #[test]
    fn a_root_translation_moves_the_rendered_hips_along_the_poses_own_axes() {
        // The pose's root translation names WORLD axes, like its rotations.
        // Composed through `hips_root_rotation` and then un-composed through
        // the same rest rotation, it came out as the raw vector in the hips'
        // PARENT's local axes — and on this Z-up armature local +Y is
        // horizontal. Live, the gait's vertical bob moved the pelvis 63.4 mm
        // along the direction of travel and 0.0 mm up: no bob at all, and a
        // forward-back surge twice per stride.
        //
        // Followed to where it renders, through the real parent chain — a
        // test that compares `hips_world_position` against the formula it
        // is built from cannot see this.
        let (mut world, skeleton, _) = real_rig_hierarchy();
        let hips = skeleton.entity(Bone::Hips);

        write(&mut world, &skeleton, &LocalPose::REST);
        let rest = rendered_position(&world, hips);

        for delta in [Vec3::new(0.0, 0.05, 0.0), Vec3::new(0.03, 0.0, -0.04)] {
            write(&mut world, &skeleton, &LocalPose::at_root(delta));
            let shift = rendered_position(&world, hips) - rest;
            assert!(
                shift.distance(delta) < 1.0e-4,
                "a root translation of {delta:?} moved the rendered hips by {shift:?}",
            );
        }
    }

    #[test]
    fn an_authored_swing_moves_a_real_rigs_bone_in_the_authored_direction() {
        // THE regression for the frame-mismatch bug, and the test that
        // should have existed before the first screenshot.
        //
        // A delta is authored in this crate's T-pose frame; the target rig's
        // bone-local axes are ~90 degrees away from it. Composing the delta
        // without converting frames applies the right angle about the wrong
        // axis, which rendered `relaxed_stand` with both arms raised
        // overhead in a Y. Every unit test passed, because they all measured
        // the SYNTHETIC rig. This one measures the real one.
        //
        // The claim: a delta that swings the arm DOWN in T-pose space must
        // swing it DOWN on the real rig too.
        let (mut world, skeleton) = real_mesh_skeleton_world();

        // In the synthetic T-pose the left arm chain points along -X, so a
        // +90-degree rotation about +Z carries it onto -Y: straight down.
        let swing_down = Quat::from_axis_angle(Vec3::Z, FRAC_PI_2);
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, swing_down);

        // Where the T-pose says the forearm should end up, as a direction.
        let t_pose_result =
            (swing_down * Bone::LeftForeArm.t_pose_offset()).normalize();
        assert!(
            t_pose_result.y < -0.9,
            "test setup: this delta should point the forearm downward, got {t_pose_result:?}",
        );

        write(&mut world, &skeleton, &pose);

        // Now read what the rig actually does with it, in the rig's own
        // frame: apply the written local rotation to the rig's own rest
        // direction for the child.
        let written = rotation_of(&mut world, &skeleton, Bone::LeftArm);
        let rig_rest_direction = skeleton.rest_direction(Bone::LeftForeArm);
        let rig_result = (written * rig_rest_direction).normalize();

        // The rig's rest direction for this bone points nearly straight up;
        // the authored swing must bring it down, not further up.
        let rest_result = (skeleton.rest_rotation(Bone::LeftArm) * rig_rest_direction).normalize();
        assert!(
            rig_result.y < rest_result.y - 0.5,
            "an authored downward swing must move the real rig's arm DOWN from its bind \
             pose, but it went from y={} to y={} — the delta is being applied in the \
             wrong frame",
            rest_result.y,
            rig_result.y,
        );
    }

    // NOTE — why there is no "rendered world position" test here.
    //
    // An attempt was made to assert the real rig's rendered arm geometry by
    // composing the written local transforms down the hierarchy in-test. It
    // produced confidently wrong numbers, and is worth recording so the next
    // person does not rebuild it:
    //
    // `real_mesh_skeleton_world` attaches `puppet_base.gltf`'s captured bind
    // ROTATIONS onto entities that `spawn_bare_bone_entities` created with
    // this crate's own synthetic T-pose TRANSLATIONS. That hybrid rig exists
    // nowhere — not in the glTF, not in the example — so positions derived
    // from it describe no real skeleton. The giveaway was that `REST` and
    // `relaxed_stand` composed to byte-identical arm geometry, which cannot
    // happen if a delta is being applied at all.
    //
    // The fixture is still perfectly good for what the surviving tests use
    // it for: asserting relationships between ROTATIONS, which is all
    // `write_pose_to_skeleton` actually computes.
    //
    // For questions about rendered POSITION on the real rig, query the live
    // example over BRP (`127.0.0.1:15702`, already wired in
    // `examples/character_gallery.rs`) rather than rebuilding the hierarchy
    // in a unit test.
    #[test]
    fn an_authored_delta_has_the_same_world_space_effect_on_any_rig() {
        // THE regression for the frame-mismatch bug, expressed as the
        // property that makes a pose rig-independent at all.
        //
        // "Turn 71 degrees about world +Z" must mean the same thing on
        // every rig. On the synthetic rig, whose bind rotations are all
        // identity, a local delta IS a world rotation. On a real rig it is
        // not, because `Transform.rotation` is interpreted in the bone's own
        // local frame — which `puppet_base.gltf` rotates far from world axes
        // (`pelvis` alone carries ~106 degrees about X).
        //
        // Without conjugation the right ANGLE is applied about the wrong
        // AXIS, which rendered `relaxed_stand` with both arms overhead while
        // every unit test passed. This one fails loudly instead.
        let (mut world, skeleton) = real_mesh_skeleton_world();
        let bind = accumulated_bind_rotations(&skeleton);

        let delta = Quat::from_axis_angle(Vec3::Z, 1.24);
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, delta);
        write(&mut world, &skeleton, &pose);

        // The bone's world orientation, bind versus posed.
        let parent_bind = bind[Bone::LeftShoulder.index()];
        let posed_world = parent_bind * rotation_of(&mut world, &skeleton, Bone::LeftArm);
        let bind_world = bind[Bone::LeftArm.index()];

        // What the pose actually did, measured in world space.
        let applied_in_world = posed_world * bind_world.inverse();

        // Compare against the authored rotation directly rather than
        // decomposing into axis+angle: `to_axis_angle`'s sign and range
        // conventions make an axis comparison easy to get insensitive, and
        // an earlier version of this test passed even with the conjugation
        // disabled because of exactly that.
        let expected = delta;
        let mismatch = 1.0 - applied_in_world.dot(expected).abs();

        assert!(
            mismatch < 1.0e-5,
            "an authored delta must have the SAME world-space effect on every rig, but \
             on this one it produced {applied_in_world:?} instead of {expected:?} \
             (mismatch {mismatch}) — the delta is being applied in the wrong frame",
        );
    }

    #[test]
    fn the_accumulated_bind_rotation_matches_the_chain_product() {
        // Pins the one-pass accumulation against an explicit walk up the
        // parent chain, so a reordering of `Bone::ALL` cannot silently
        // corrupt every conjugation.
        let (_, skeleton) = real_mesh_skeleton_world();
        let bind = accumulated_bind_rotations(&skeleton);

        for &bone in Bone::ALL.iter() {
            // Walk from this bone up to the root, collecting rest rotations.
            let mut chain = vec![bone];
            let mut cursor = bone;
            while let Some(parent) = cursor.parent() {
                chain.push(parent);
                cursor = parent;
            }

            let mut expected = skeleton.hips_root_rotation();
            for &link in chain.iter().rev() {
                expected *= skeleton.rest_rotation(link);
            }

            assert!(
                bind[bone.index()].abs_diff_eq(expected, 1.0e-5),
                "{}'s accumulated bind rotation should be the product down its own chain",
                bone.name(),
            );
        }
    }

    #[test]
    fn frame_conversion_is_a_no_op_on_a_rig_with_no_captured_bind_pose() {
        // The synthetic rig IS the T-pose, so there is nothing to convert
        // and the composition must stay exactly `rest * delta`.
        let (mut world, skeleton) = bare_skeleton_world();

        let delta = Quat::from_axis_angle(Vec3::Z, -FRAC_PI_2);
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, delta);

        write(&mut world, &skeleton, &pose);

        assert!(
            rotation_of(&mut world, &skeleton, Bone::LeftArm).abs_diff_eq(delta, 1.0e-6),
            "with no captured bind pose the delta must be written unchanged",
        );
    }

    #[test]
    fn a_multi_child_parent_is_unambiguous() {
        // `Hips` and `Spine2` each have three children. Under position
        // inference they produced three disagreeing answers for one parent
        // rotation, requiring a `chain_continuation_child` tie-breaker.
        // Authoring rotations removes the ambiguity entirely: the parent's
        // own rotation is authored, and each child's is independent.
        let (mut world, skeleton) = bare_skeleton_world();

        let spine2 = Quat::from_axis_angle(Vec3::Y, 0.3);
        let neck = Quat::from_axis_angle(Vec3::X, 0.1);
        let left = Quat::from_axis_angle(Vec3::Z, -0.6);
        let right = Quat::from_axis_angle(Vec3::Z, 0.6);

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Spine2, spine2);
        pose.set_rotation(Bone::Neck, neck);
        pose.set_rotation(Bone::LeftShoulder, left);
        pose.set_rotation(Bone::RightShoulder, right);

        write(&mut world, &skeleton, &pose);

        for (bone, delta) in [
            (Bone::Spine2, spine2),
            (Bone::Neck, neck),
            (Bone::LeftShoulder, left),
            (Bone::RightShoulder, right),
        ] {
            let written = rotation_of(&mut world, &skeleton, bone);
            assert!(
                written.abs_diff_eq(skeleton.rest_rotation(bone) * delta, 1.0e-6),
                "{} should be unaffected by what its siblings authored",
                bone.name(),
            );
        }
    }
}
