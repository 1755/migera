//! A standing stance with softly bent knees — the pose everything else
//! should build on, and the thing that makes foot IK solvable at all.
//!
//! # Why a bind pose cannot be stood on
//!
//! A rig is bound straight-legged: the hip socket sits at exactly
//! femur + shin + ankle above the sole. Measured on this project's own two
//! rigs, both agree to four decimals — `puppet_base.gltf` has its hip
//! socket at `0.9712` and a straight-leg reach of `0.9712`, and the
//! synthetic T-pose table is `0.940` and `0.940`. That is not a defect in
//! either; it is what a T-pose bind *is*.
//!
//! But it means the knee sits at exactly 180 degrees, which is a
//! singularity for inverse kinematics:
//!
//! - There is **zero horizontal reach budget**. A foot cannot move even a
//!   centimetre sideways without the leg needing to be longer than it is.
//! - The knee has no bend *direction*. At exactly straight, "bend forward"
//!   and "bend backward" are equally valid, so a solver has nothing to
//!   disambiguate with and will pick arbitrarily — or flip between them
//!   frame to frame.
//!
//! This is why earlier attempts at foot IK on this rig measured up to
//! **2.18x leg overstretch**: they were not mis-tuned, the pose they
//! started from had no solution space.
//!
//! # The fix is authored data, not rig geometry
//!
//! A real person standing relaxed carries 5-15 degrees of knee flexion.
//! Authoring that as a pose — rather than editing the rig's bone lengths —
//! is better in three ways:
//!
//! - **It works on any rig.** The stance is expressed as rotations, and
//!   rotations are rig-independent (see [`super::retarget`]). A taller or
//!   shorter character gets a correct stance from the same data.
//! - **It leaves the bind pose clean.** `t_pose_offset` stays the pure,
//!   straight-chain reference that the retargeting math and every bone
//!   length assumption depend on.
//! - **It is tunable without a recompile**, because it is a pose like any
//!   other and can live in a `.pose.ron` file.
//!
//! # The geometry
//!
//! Bending only the knee would swing the foot backward and lift the sole
//! off the ground. Three rotations are needed, and they must sum correctly:
//!
//! ```text
//! hip:   +flex/2    (thigh leans forward)
//! knee:  -flex      (shin folds back under it)
//! ankle: +flex/2    (foot re-levels against the ground)
//! ```
//!
//! With the hip and ankle each taking half the knee's angle, the shin ends
//! up back under the hip and the foot stays flat.
//!
//! Note what this symmetric split costs: the hip's `+flex/2` cancels half
//! the knee's `-flex`, so each segment ends up folded by `flex/2` rather
//! than `flex`, and the leg shortens by `L * (1 - cos(flex/2))` — about
//! half what a bare knee bend would give. That is the price of keeping the
//! shin vertical and the foot flat, and it is worth paying.
//!
//! The shortening is what buys reach, and it is deliberately small: 9
//! degrees of flex costs about 3 mm of height and yields roughly 0.055 m of
//! horizontal foot placement. Budget grows with the sine of the fold while
//! the crouch grows with its cosine, so the first few degrees are nearly
//! free and later ones are not — reaching 0.15 m would take past 20
//! degrees, which reads as a squat. Prefer a little foot sliding over a
//! permanently crouching character.

use bevy::math::{Quat, Vec3};

use super::rig::LocalPose;
use crate::character::skeleton::Bone;

/// The axis a knee bends about.
///
/// `+X` is the character's right (see [`Bone::t_pose_offset`]'s own
/// convention), and a positive rotation about it swings a bone **forward**
/// — toward `-Z`, the direction the rig faces. Both legs share the axis: a
/// knee is a hinge, and both knees hinge the same way.
///
/// # The sign, derived rather than recalled
///
/// This comment previously claimed the opposite, and the walk cycle built on
/// it ran backward. The derivation, so it can be checked rather than
/// trusted: a leg hangs along `-Y`, and
///
/// ```text
/// Rx(t) * (0, -L, 0) = (0, -L*cos(t), -L*sin(t))
/// ```
///
/// so a positive `t` drives `z` negative. `-Z` is forward. Measured the same
/// way on the real rig: `+0.3` rad moves `LeftFoot` from `z = +0.002` to
/// `z = -0.191`.
///
/// Because a knee only flexes one way, knee rotations here are NEGATIVE —
/// see [`stance_on`], which applies `-knee_flex` at the knee.
pub const KNEE_AXIS: Vec3 = Vec3::X;

/// How far the knees bend in the default standing stance, radians.
///
/// About 9 degrees: inside the 5-15 degree range a relaxed person actually
/// holds, and enough to buy roughly 0.24 m of horizontal foot placement on
/// a human-scale rig. Larger values buy more IK headroom at the cost of
/// looking like a crouch — the "avoid the dinosaur" warning from the
/// foot-locking literature, and the reason this is not simply maximised.
pub const DEFAULT_KNEE_FLEX: f32 = 0.16;

/// Builds a standing stance with both knees softly bent.
///
/// Composed **onto** `base`, so an authored upper-body pose keeps its arms
/// and spine and gains legs it can stand on.
///
/// `knee_flex` is the knee's own bend in radians; the hip and ankle each
/// take half of it in the opposite direction so the shin stays vertical and
/// the sole stays flat. See the module doc for why all three are needed.
pub fn stance_on(base: &LocalPose, knee_flex: f32) -> LocalPose {
    stance_on_rig(base, knee_flex, &super::rig::RigGeometry::default())
}

/// [`stance_on`], bending the knees the way THIS rig's own geometry says is
/// forward.
///
/// # Why the rig is needed to bend a knee
///
/// [`KNEE_AXIS`] is a constant, and the sign that swings a bone forward
/// about it is not: it depends on which way the rig faces, and rigs
/// disagree. The synthetic T-pose faces `-Z`; `puppet_base.gltf` faces
/// `+Z`. So the same rotation that bends a knee correctly on one bends it
/// BACKWARD on the other.
///
/// That is exactly what shipped. Measured on `puppet_base` before this
/// existed, as the knee's offset from the hip-to-ankle line along the rig's
/// own forward: `LocalPose::REST` sat at `+0.0169` (correct) while
/// `stance(REST)` sat at `-0.0098`, and the walk cycle composed on top
/// amplified it to `-0.19`. The character walked on backward-bending
/// knees.
///
/// It survived thirty-odd leg tests because every one of them measured the
/// UNSIGNED angle between thigh and shin, which is identical whichever way
/// the knee folds. See [`RigGeometry::knee_forward_offset`], which is the
/// signed measurement, and
/// `the_stance_bends_both_knees_forward_on_every_rig`, which asserts it.
pub fn stance_on_rig(
    base: &LocalPose,
    knee_flex: f32,
    rig: &super::rig::RigGeometry,
) -> LocalPose {
    let mut pose = *base;

    if knee_flex == 0.0 {
        return pose;
    }

    // Which way a positive rotation about `KNEE_AXIS` actually swings a
    // bone on this rig. `+1` reproduces exactly what this function did
    // before the rig was a parameter.
    let flex = knee_flex * facing_sign(rig);
    let half = flex * 0.5;

    for (hip, knee, ankle) in [
        (Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
        (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
    ] {
        compose(&mut pose, hip, Quat::from_axis_angle(KNEE_AXIS, half));
        compose(&mut pose, knee, Quat::from_axis_angle(KNEE_AXIS, -flex));
        compose(&mut pose, ankle, Quat::from_axis_angle(KNEE_AXIS, half));
    }

    pose
}

/// `+1` if this rig faces the way the poses here were authored, `-1` if it
/// faces the other way.
///
/// The one number every leg-posing path needs to stay correct on a rig that
/// was not exported facing [`AUTHORED_FORWARD`]. Derived from the rig's own
/// measured geometry, never assumed — see [`RigGeometry::forward`].
pub fn facing_sign(rig: &super::rig::RigGeometry) -> f32 {
    let forward = rig.forward();
    if forward == Vec3::ZERO {
        // Nothing to measure. Keep the authored convention rather than
        // picking a direction from noise.
        return 1.0;
    }

    let agreement = forward.dot(AUTHORED_FORWARD);
    if agreement.abs() < 1.0e-3 { 1.0 } else { agreement.signum() }
}

/// The facing every pose in this crate is authored against.
///
/// `-Z`, the synthetic T-pose's own convention — see
/// [`Bone::t_pose_offset`]. A rig facing the other way needs its leg angles
/// negated; [`facing_sign`] reports which.
pub const AUTHORED_FORWARD: Vec3 = Vec3::NEG_Z;

/// Shifts the pelvis by `shift` (horizontal, in the rig's frame) over feet
/// that stay exactly where they were, flat on the ground.
///
/// How a standing body sways (Winter §11.2.1): the pelvis carries the trunk
/// over the feet, each leg turning as a whole about its ankle, the trunk
/// upright. Each thigh takes the world turn that re-aims its hip-to-ankle
/// line at the ankle's old place, and each foot the opposite turn, so it
/// keeps its attitude on the ground.
///
/// A leg turned whole keeps its length, so the pelvis moves on a circle
/// about the ankles — an inverted pendulum — and rises or falls with the
/// shift. Leaving it level lifted the feet instead: the relaxed stance
/// stands with its hips ~5 cm ahead of its ankles, so a 4 cm forward lean
/// carried them 3.3 mm off the floor. The vertical is the mean the two legs
/// ask for; they differ only by the stance's small side-to-side asymmetry.
pub fn sway_over_feet(pose: &mut LocalPose, rig: &super::rig::RigGeometry, shift: Vec3) {
    use super::rig::{delta_after_world_turn, offset_from};
    let shift = Vec3::new(shift.x, 0.0, shift.z);
    if shift.length_squared() < 1.0e-12 {
        return;
    }
    let legs = [(Bone::LeftUpLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightFoot)]
        .map(|(socket, ankle)| {
            (socket, ankle, offset_from(pose, rig, Bone::Hips, ankle) - offset_from(pose, rig, Bone::Hips, socket))
        });
    // How far the pelvis must drop (negative: rise) for each leg to keep
    // its length under the shift: |leg − shift − v·Y| = |leg|.
    let drop = legs
        .iter()
        .map(|(_, _, leg)| {
            let shifted = *leg - shift;
            let horizontal = Vec3::new(shifted.x, 0.0, shifted.z).length_squared();
            shifted.y + (leg.length_squared() - horizontal).max(0.0).sqrt()
        })
        .sum::<f32>()
        * 0.5;
    let moved = shift + Vec3::Y * drop;
    for (socket, ankle, leg) in legs {
        let turn = Quat::from_rotation_arc(leg.normalize_or_zero(), (leg - moved).normalize_or_zero());
        pose.rotations[socket] = delta_after_world_turn(pose, rig, socket, turn);
        pose.rotations[ankle] = delta_after_world_turn(pose, rig, ankle, turn.inverse());
    }
    pose.root_translation += moved;
}

/// How far a full weight shift carries the pelvis toward the loaded foot,
/// metres.
///
/// Winter §11.2.1: side-to-side balance is the hips loading one leg and
/// unloading the other, which moves the pressure — and, held, the pelvis —
/// toward the loaded foot. A relaxed shift onto one leg brings the pelvis a
/// good part of the way over it; the feet here stand ~0.2 m apart.
pub const WEIGHT_SHIFT: f32 = 0.045;

/// How far a full weight shift drops the unloaded side of the pelvis,
/// radians (~4 degrees): the hip on the resting leg sags while the loaded
/// side's abductors hold it.
pub const WEIGHT_SHIFT_ROLL: f32 = 0.07;

/// Shifts the body's weight onto one leg: `onto` +1 is fully onto the
/// rig's left leg, −1 its right, 0 nothing.
///
/// A deliberate postural change, not quiet sway (Winter §11.2.1: a visible,
/// held shift is the hips loading one leg): the pelvis moves toward the
/// loaded foot and rolls down on the unloaded side, the spine rolls back so
/// the shoulders stay level, and both legs are re-solved to the ground where
/// the feet stood. The loaded leg straightens under the load; the unloaded
/// one, now too long for its lowered hip, relaxes its knee.
pub fn shift_weight(pose: &mut LocalPose, rig: &super::rig::RigGeometry, onto: f32) {
    use super::legik::{solve_leg_on, LegChain, LegIkConfig};
    use super::rig::{delta_after_world_turn, forward_kinematics_on, offset_from};

    let onto = onto.clamp(-1.0, 1.0);
    if onto.abs() < 1.0e-4 {
        return;
    }
    let before = forward_kinematics_on(pose, rig);
    let (left_toe, right_toe) = (before[Bone::LeftToeBase], before[Bone::RightToeBase]);
    let loaded = if onto > 0.0 { Bone::LeftUpLeg } else { Bone::RightUpLeg };
    let loaded_hip = offset_from(pose, rig, Bone::Hips, loaded);

    // Rolled about the rig's forward, so the unloaded hip goes down: a
    // positive turn about forward lifts the rig's left side.
    let roll = bevy::math::Quat::from_axis_angle(rig.forward(), onto * WEIGHT_SHIFT_ROLL);
    pose.rotations[Bone::Hips] = delta_after_world_turn(pose, rig, Bone::Hips, roll);
    pose.rotations[Bone::Spine] = delta_after_world_turn(pose, rig, Bone::Spine, roll.inverse());

    // Toward the loaded foot, and down by however much the roll lifted the
    // loaded hip, so the loaded leg can still reach the ground.
    let lifted = offset_from(pose, rig, Bone::Hips, loaded).y - loaded_hip.y;
    pose.root_translation += rig.left() * (onto * WEIGHT_SHIFT) - Vec3::Y * lifted.max(0.0);

    let config = LegIkConfig::default();
    solve_leg_on(pose, LegChain::LEFT, left_toe, &config, rig);
    solve_leg_on(pose, LegChain::RIGHT, right_toe, &config, rig);
}

/// The default standing stance on top of `base`.
pub fn stance(base: &LocalPose) -> LocalPose {
    stance_on(base, DEFAULT_KNEE_FLEX)
}

/// Multiplies a rotation onto whatever the pose already has for a bone,
/// rather than replacing it — so a stance layers onto an authored pose.
fn compose(pose: &mut LocalPose, bone: Bone, rotation: Quat) {
    pose.set_rotation(bone, pose.rotation(bone) * rotation);
}

/// How much horizontal room a foot has, given a hip height and the leg's
/// own straight reach.
///
/// This is the quantity that was zero before a stance was authored, and it
/// is what makes an IK target reachable: with the hip at `hip_height` above
/// the sole, a foot can move this far sideways before the leg would have to
/// stretch.
pub fn horizontal_reach_budget(straight_reach: f32, hip_height: f32) -> f32 {
    let squared = straight_reach * straight_reach - hip_height * hip_height;
    if squared <= 0.0 { 0.0 } else { squared.sqrt() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::{forward_kinematics, Side};

    #[test]
    fn a_weight_shift_loads_one_leg_and_rests_the_other() {
        use crate::character::anim::anthropometry::centre_of_mass;
        use crate::character::anim::gltf_rig::puppet_base_as_rendered;
        use crate::character::anim::gait::{leg_joints, sagittal_angles};
        use crate::character::anim::rig::forward_kinematics_on;

        let rig = puppet_base_as_rendered();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let left = rig.left();
        for onto in [1.0f32, -1.0] {
            let mut shifted = stood;
            shift_weight(&mut shifted, &rig, onto);
            let (a, b) = (forward_kinematics_on(&stood, &rig), forward_kinematics_on(&shifted, &rig));

            // The feet stay where they stood.
            for toe in [Bone::LeftToeBase, Bone::RightToeBase] {
                assert!(a[toe].distance(b[toe]) < 0.003, "{onto}: {} moved {}", toe.name(), a[toe].distance(b[toe]));
            }
            // The pelvis and the body's mass move toward the loaded foot.
            let pelvis = (b[Bone::Hips] - a[Bone::Hips]).dot(left) * onto;
            assert!(pelvis > 0.03, "{onto}: pelvis moved {pelvis} toward the loaded foot");
            let com = |p: &LocalPose| (p.root_translation + centre_of_mass(p, &rig)).dot(left);
            assert!((com(&shifted) - com(&stood)) * onto > 0.025, "{onto}: the centre of mass barely moved");
            // The resting knee bends more than the loaded one.
            let knee = |ankle| sagittal_angles(&shifted, &rig, leg_joints(ankle))[2];
            let (loaded, resting) =
                if onto > 0.0 { (knee(Bone::LeftFoot), knee(Bone::RightFoot)) } else { (knee(Bone::RightFoot), knee(Bone::LeftFoot)) };
            // Measured: pelvis 4.5 cm, centre of mass 3.4 cm; loaded knee
            // 14.1 degrees, resting 25.9.
            assert!(resting > loaded + 0.05, "{onto}: resting knee {resting} against loaded {loaded}");
            // The shoulders stay level.
            let tilt = (b[Bone::LeftArm].y - b[Bone::RightArm].y) - (a[Bone::LeftArm].y - a[Bone::RightArm].y);
            assert!(tilt.abs() < 0.01, "{onto}: shoulders tilted {tilt} m");
        }
    }

    #[test]
    fn the_stance_bends_both_knees_forward_on_every_rig() {
        // THE anatomical invariant, and the one this module's own tests
        // could not express: a knee bends FORWARD. Every other leg test in
        // this crate measured the unsigned angle between thigh and shin,
        // which is identical whichever way the knee folds — so a
        // backward-bending knee passed all of them, and was reported from a
        // screenshot instead.
        //
        // Checked on a rig with REAL bind rotations, because that is the
        // only place the bug can appear: the synthetic rig's binds are all
        // identity and it faces the same way the stance was authored
        // against, so it cannot disagree.
        //
        // Measured with the bug live on `puppet_base`: `LocalPose::REST`
        // sat at `+0.0169` while `stance(REST)` — whose entire job is
        // bending the knee — sat at `-0.0157`.
        use crate::character::anim::gltf_rig;
        use crate::character::anim::rig::RigGeometry;

        for (label, rig) in [
            ("synthetic", RigGeometry::default()),
            ("puppet_base", gltf_rig::puppet_base()),
        ] {
            for side in [Side::Left, Side::Right] {
                let stood = stance_on_rig(&LocalPose::REST, DEFAULT_KNEE_FLEX, &rig);

                // Measured by the FOLD — how the shin turns relative to the
                // thigh — rather than by the knee's offset from the
                // hip-to-ankle line. Both describe the same thing on a
                // bent leg, but the offset degenerates near full extension
                // where the knee lies on that line by definition, and a
                // standing stance is only about 9 degrees off straight.
                // See `RigGeometry::knee_fold_direction`.
                let fold = rig.knee_fold_direction(&stood, side);

                assert!(
                    fold < 0.0,
                    "{label} {side:?}: the stance folded the shin {fold} along the \
                     rig's own forward relative to the thigh — positive is a knee \
                     bending backward, which no human leg does",
                );
            }
        }
    }

    /// How far the leg reaches from its hip socket to the sole, measured
    /// from the rig itself rather than assembled by hand.
    ///
    /// # A naming trap worth knowing about
    ///
    /// Each bone entity sits where its bone *starts*, positioned by its
    /// parent's offset. So `LeftUpLeg` — "the thigh" — is at the point the
    /// thigh ENDS, i.e. the knee, and the hip socket is `Hips` itself.
    ///
    /// Two earlier versions of this helper got that wrong in different
    /// ways: one summed `LeftUpLeg`'s offset as though it were the femur
    /// (reporting 0.79 m of budget on a rig with none), the other measured
    /// from `LeftUpLeg` and found a 0.508 m leg. Deriving from the bind
    /// pose's own positions, between the two joints that really are the hip
    /// and the sole, removes the guesswork.
    fn straight_leg_reach() -> f32 {
        let rest = forward_kinematics(&LocalPose::REST);
        rest[Bone::Hips].y - rest[Bone::LeftToeBase].y
    }

    #[test]
    fn the_bind_pose_has_no_horizontal_reach_budget_at_all() {
        // The problem this module exists to solve, stated as a measurement.
        // If this ever stops being true the rig has changed, and the stance
        // should be re-derived rather than assumed.
        let budget =
            horizontal_reach_budget(straight_leg_reach(), hip_to_sole(&LocalPose::REST));

        assert!(
            budget < 0.05,
            "the bind pose should be effectively singular for IK, but reported \
             {budget} m of budget",
        );
    }

    /// How far the sole sits below the hip socket under `pose`.
    ///
    /// This is the quantity a stance actually changes. `Hips` is the rig's
    /// root and the hip socket is a fixed offset from it, so neither can
    /// move under a leg rotation — bending the knee raises the *sole*
    /// toward the hips rather than lowering the hips toward the ground.
    /// Same geometry either way, and this shrink is exactly the reach
    /// budget the IK stage needs.
    ///
    /// A consumer that wants the character to visibly crouch pairs the
    /// stance with a matching `root_translation`; that is a separate
    /// concern from having a solvable leg.
    fn hip_to_sole(pose: &LocalPose) -> f32 {
        let positions = forward_kinematics(pose);
        // `Hips` IS the hip socket — see `straight_leg_reach`'s own note on
        // why `LeftUpLeg` is the knee rather than the thigh's origin.
        positions[Bone::Hips].y - positions[Bone::LeftToeBase].y
    }

    #[test]
    fn a_stance_buys_real_horizontal_reach_budget() {
        // THE acceptance criterion for this phase: enough room to place a
        // foot somewhere other than directly under the hip.
        let standing = hip_to_sole(&stance(&LocalPose::REST));
        let budget = horizontal_reach_budget(straight_leg_reach(), standing);

        // 0.05 m rather than something larger, deliberately. Budget grows
        // with the sine of the fold while the crouch grows with its cosine,
        // so reach is cheap at first and expensive later: the default 9
        // degrees buys ~0.055 m, while 0.15 m would need past 20 degrees —
        // more than a relaxed person holds, and visibly a squat.
        //
        // That is the trade-off the foot-locking literature calls "avoid
        // the dinosaur": prefer a little foot sliding over a character who
        // permanently crouches. 0.055 m is enough to place a foot outside
        // the hip, measured against a bind pose that had essentially zero —
        // and `stance_on` takes a deeper flex where a pose genuinely needs
        // one.
        assert!(
            budget > 0.05,
            "a standing stance should give a foot at least 0.05 m of horizontal room, \
             got {budget} m (hip sits {standing} m above the sole)",
        );
    }

    #[test]
    fn a_stance_shortens_the_leg_without_tilting_the_foot_off_the_ground() {
        // The three-rotation construction exists precisely so the foot
        // stays flat. Bending only the knee would swing the sole backward
        // and tip it onto its heel.
        let rest = hip_to_sole(&LocalPose::REST);
        let standing = hip_to_sole(&stance(&LocalPose::REST));

        // Barely a millimetre, and that is the point: the shortening grows
        // with the square of the fold, so a relaxed stance costs almost no
        // height while still opening up the reach budget that matters.
        //
        // Note the symmetric hip/ankle split folds each segment by `flex/2`
        // rather than `flex` — the hip's `+flex/2` cancels half the knee's
        // `-flex` — so the shortening is `L * (1 - cos(flex/2))`, about
        // half what a pure knee bend would give. That is the price of
        // keeping the shin vertical and the foot flat, and it is worth it.
        assert!(
            rest - standing > 0.001,
            "the stance should shorten the hip-to-sole distance, but it changed by only \
             {} m",
            rest - standing,
        );

        // The foot must stay level: the toe and ankle should remain at
        // roughly the same height relative to each other.
        let posed = forward_kinematics(&stance(&LocalPose::REST));
        let bind = forward_kinematics(&LocalPose::REST);

        let posed_tilt = posed[Bone::LeftToeBase].y - posed[Bone::LeftFoot].y;
        let bind_tilt = bind[Bone::LeftToeBase].y - bind[Bone::LeftFoot].y;

        assert!(
            (posed_tilt - bind_tilt).abs() < 0.02,
            "the foot should stay as flat as it was bound, but its tilt changed by {} m",
            (posed_tilt - bind_tilt).abs(),
        );
    }

    #[test]
    fn a_stance_keeps_the_shin_close_to_vertical() {
        // The hip and ankle each take half the knee's bend specifically so
        // the shin ends up back under the hip. A shin leaning far off
        // vertical would read as a lunge, not a stand.
        let positions = forward_kinematics(&stance(&LocalPose::REST));

        let shin = (positions[Bone::LeftFoot] - positions[Bone::LeftLeg]).normalize();
        let lean = shin.dot(Vec3::NEG_Y).acos().to_degrees();

        assert!(
            lean < 15.0,
            "the shin should stay near vertical, but leans {lean} degrees",
        );
    }

    #[test]
    fn a_stance_bends_both_knees_by_the_authored_amount() {
        let flex = 0.2;
        let posed = stance_on(&LocalPose::REST, flex);

        for knee in [Bone::LeftLeg, Bone::RightLeg] {
            let angle = posed.rotation(knee).angle_between(Quat::IDENTITY);
            assert!(
                (angle - flex).abs() < 1.0e-5,
                "{} should bend by {flex} rad, got {angle}",
                knee.name(),
            );
        }
    }

    #[test]
    fn both_legs_bend_identically() {
        // An asymmetric stance would read as a limp. Idle asymmetry is
        // added later, deliberately, by the phase layer — not here.
        let posed = stance(&LocalPose::REST);

        for (left, right) in [
            (Bone::LeftUpLeg, Bone::RightUpLeg),
            (Bone::LeftLeg, Bone::RightLeg),
            (Bone::LeftFoot, Bone::RightFoot),
        ] {
            assert!(
                posed.rotation(left).abs_diff_eq(posed.rotation(right), 1.0e-6),
                "{} and {} should bend identically",
                left.name(),
                right.name(),
            );
        }
    }

    #[test]
    fn the_knee_bends_the_way_a_knee_actually_bends() {
        // A knee that hinges the wrong way is instantly, viscerally wrong,
        // and is the exact sign error that cost the superseded module real
        // debugging time. The heel must move BACKWARD, never forward.
        //
        // The rig faces -Z, so backward is +Z.
        let rest = forward_kinematics(&LocalPose::REST);
        let posed = forward_kinematics(&stance(&LocalPose::REST));

        let knee_shift = posed[Bone::LeftLeg].z - rest[Bone::LeftLeg].z;
        assert!(
            knee_shift < 0.0,
            "the knee should travel FORWARD (-Z) as it bends, but moved {knee_shift} in Z \
             — the bend axis sign is inverted",
        );
    }

    #[test]
    fn a_zero_flex_stance_is_exactly_the_base_pose() {
        let mut base = LocalPose::REST;
        base.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, 0.4));

        let posed = stance_on(&base, 0.0);

        for &bone in Bone::ALL.iter() {
            assert_eq!(
                posed.rotation(bone),
                base.rotation(bone),
                "{} should be untouched at zero flex",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_stance_leaves_the_upper_body_alone() {
        // It composes onto an authored pose, so the arms and spine it was
        // given must survive untouched.
        let mut base = LocalPose::REST;
        let arm = Quat::from_axis_angle(Vec3::Z, 1.2);
        base.set_rotation(Bone::LeftArm, arm);
        base.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::X, 0.3));

        let posed = stance(&base);

        assert_eq!(posed.rotation(Bone::LeftArm), arm, "the arm must be untouched");
        assert_eq!(
            posed.rotation(Bone::Spine),
            base.rotation(Bone::Spine),
            "the spine must be untouched",
        );
    }

    #[test]
    fn a_stance_preserves_every_bone_length() {
        // Free in rotation space, but worth pinning: this is the invariant
        // that the superseded position-space module could only approximate.
        let positions = forward_kinematics(&stance(&LocalPose::REST));

        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };
            let rest_length = bone.t_pose_offset().length();
            let posed_length = (positions[bone] - positions[parent]).length();

            assert!(
                (posed_length - rest_length).abs() < 1.0e-5,
                "{} should stay {rest_length} m from {}, got {posed_length}",
                bone.name(),
                parent.name(),
            );
        }
    }

    #[test]
    fn a_deeper_stance_buys_more_budget() {
        // Monotonic, so the knob behaves predictably when the IK stage
        // needs more room.
        let budget_at = |flex: f32| {
            horizontal_reach_budget(
                straight_leg_reach(),
                hip_to_sole(&stance_on(&LocalPose::REST, flex)),
            )
        };

        let shallow = budget_at(0.10);
        let default = budget_at(DEFAULT_KNEE_FLEX);
        let deep = budget_at(0.30);

        assert!(shallow < default, "{shallow} should be less than {default}");
        assert!(default < deep, "{default} should be less than {deep}");
    }

    #[test]
    fn the_default_flex_stays_in_the_range_a_person_actually_stands_at() {
        // Guards against quietly maximising this for IK headroom and
        // ending up with a permanent crouch.
        let degrees = DEFAULT_KNEE_FLEX.to_degrees();
        assert!(
            (5.0..=15.0).contains(&degrees),
            "the default knee flex should be a relaxed 5-15 degrees, got {degrees}",
        );
    }

    #[test]
    fn the_reach_budget_helper_is_zero_when_the_leg_is_exactly_straight() {
        assert_eq!(horizontal_reach_budget(0.94, 0.94), 0.0);
        assert_eq!(horizontal_reach_budget(0.94, 1.5), 0.0, "and clamps rather than NaN-ing");
        assert!(horizontal_reach_budget(0.94, 0.90) > 0.2);
    }
}
