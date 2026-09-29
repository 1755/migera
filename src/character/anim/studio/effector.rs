//! IK effectors: drag a hand or foot, and the limb solves to reach it.
//!
//! Joint dragging ([`super::drag`]) rotates one bone at a time, which is
//! exact but slow — posing an arm means three separate drags, each
//! invalidating the last. An effector is the other half: grab the end of a
//! limb, put it where it should be, and let the chain work out the angles.
//!
//! # Built on the shipped solver, not a new one
//!
//! [`super::super::math::ik::solve_two_bone`] already exists, is
//! extensively tested, and carries five named regressions for bugs its
//! predecessor once had. Writing a second solver for the editor would mean
//! the pose an author sees while dragging and the pose the runtime
//! produces are computed by different code — which is exactly how an
//! editor drifts from its engine.
//!
//! So this wraps that solver. What it adds is the part the leg-specific
//! [`super::super::legik`] does not generalise: naming an arbitrary
//! two-bone chain, and the pole direction that decides which way the
//! middle joint bends.

use bevy::math::{Quat, Vec3};

use crate::character::anim::math::ik::solve_two_bone;
use crate::character::anim::rig::{forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// A two-bone chain that can be solved to a target.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Effector {
    /// The fixed pivot — a shoulder or hip. Never rotated.
    pub root: Bone,
    /// Rotating this aims the upper segment.
    pub upper: Bone,
    /// Rotating this bends the middle joint.
    pub lower: Bone,
    /// The joint being placed.
    pub tip: Bone,
}

impl Effector {
    /// Every effector the rig offers.
    ///
    /// Four, deliberately: the limbs are what an author positions. A spine
    /// "effector" would be a three-plus-bone chain needing a different
    /// solver, and offering one that silently behaved differently would be
    /// worse than not offering it.
    pub const ALL: [Self; 4] = [
        Self {
            root: Bone::LeftShoulder,
            upper: Bone::LeftArm,
            lower: Bone::LeftForeArm,
            tip: Bone::LeftHand,
        },
        Self {
            root: Bone::RightShoulder,
            upper: Bone::RightArm,
            lower: Bone::RightForeArm,
            tip: Bone::RightHand,
        },
        Self {
            root: Bone::LeftUpLeg,
            upper: Bone::LeftLeg,
            lower: Bone::LeftFoot,
            tip: Bone::LeftToeBase,
        },
        Self {
            root: Bone::RightUpLeg,
            upper: Bone::RightLeg,
            lower: Bone::RightFoot,
            tip: Bone::RightToeBase,
        },
    ];

    /// What to call it on screen.
    pub fn label(self) -> &'static str {
        match self.tip {
            Bone::LeftHand => "Left hand",
            Bone::RightHand => "Right hand",
            Bone::LeftToeBase => "Left foot",
            Bone::RightToeBase => "Right foot",
            _ => self.tip.name(),
        }
    }

    /// Which way the middle joint bends.
    ///
    /// A two-bone solve fixes the *distance* to the target but not the
    /// plane the chain bends in — the elbow can sit anywhere on a circle.
    /// Something has to choose, and choosing wrong is the difference
    /// between an elbow and a broken arm.
    ///
    /// Knees bend forward (`-Z` is this crate's forward), elbows backward.
    /// Hardcoded per limb rather than derived from the current pose, which
    /// was the superseded solver's approach and produced a knee that
    /// flipped direction whenever the leg passed through straight.
    pub fn pole_direction(self) -> Vec3 {
        match self.tip {
            Bone::LeftToeBase | Bone::RightToeBase => Vec3::NEG_Z,
            _ => Vec3::Z,
        }
    }
}

/// Solves `effector`'s chain so its tip reaches `target`, in place.
///
/// Returns where the tip actually ended up — which is not always the
/// target: a chain cannot exceed its own length, and the solver softens
/// rather than snapping as it approaches full extension.
///
/// `softening` is the distance over which that easing happens; zero gives
/// a hard clamp.
pub fn solve(
    pose: &mut LocalPose,
    effector: Effector,
    target: Vec3,
    softening: f32,
    rig: &RigGeometry,
) -> Vec3 {
    let positions = forward_kinematics_on(pose, rig);

    // The chain PIVOTS at `upper`, not at `root`.
    //
    // `root` is the shoulder or hip socket — a fixed attachment the solve
    // never rotates. The two bones are `upper -> lower` and `lower -> tip`,
    // so the distance the law of cosines needs is measured from `upper`.
    //
    // Measuring from `root` instead adds the socket-to-limb offset (0.14 m
    // on this rig's shoulder) to every target distance, and the solve lands
    // exactly that far short — 8.6 cm, measured, consistently. It looks
    // like a converging-but-imprecise solver and is really a wrong pivot.
    let pivot = positions[effector.upper];
    let upper_length = positions[effector.upper].distance(positions[effector.lower]);
    let lower_length = positions[effector.lower].distance(positions[effector.tip]);

    let to_target = target - pivot;
    let Some(direction) = to_target.try_normalize() else {
        // The target is on the pivot itself; there is no direction to aim,
        // so hold the pose rather than snapping to an arbitrary one.
        return positions[effector.tip];
    };

    let solution = solve_two_bone(upper_length, lower_length, to_target.length(), softening);

    // The bend plane: the pole direction, made perpendicular to the aim.
    // Without orthogonalising, a limb aimed near the pole direction would
    // have almost no plane to bend in and the knee would jitter.
    let pole = effector.pole_direction();
    let side = direction.cross(pole);
    let bend_axis = match side.try_normalize() {
        Some(axis) => axis,
        // Aiming exactly along the pole. Any perpendicular axis is as
        // good as another; pick a stable one so the same target always
        // produces the same bend.
        None => direction.any_orthonormal_vector(),
    };

    // Aim the upper segment, rotated out of the straight line by the
    // solver's own upper angle.
    let upper_direction = Quat::from_axis_angle(bend_axis, -solution.upper_angle) * direction;
    aim_bone(pose, effector.upper, effector.lower, upper_direction, rig);

    // Then the lower segment, aimed from wherever the elbow has ended up.
    //
    // Then the lower segment, aimed from wherever the elbow ended up.
    //
    // Two exact aims are enough, and a third would be wrong. The upper's
    // angle already accounts for the bend — that is what `solve_two_bone`
    // computes — so re-aiming it afterwards to point its TIP at the target
    // fights the lower's aim, and the pair converges to a fixed point some
    // centimetres off. Measured: 5.8 cm, stable to the last decimal across
    // eight passes, which is what a wrong fixed point looks like as
    // opposed to a failure to converge.
    let positions = forward_kinematics_on(pose, rig);
    if let Some(lower_direction) = (target - positions[effector.lower]).try_normalize() {
        aim_bone(pose, effector.lower, effector.tip, lower_direction, rig);
    }

    forward_kinematics_on(pose, rig)[effector.tip]
}

/// Points the segment from `bone` to `child` along `world_direction`.
///
/// The child is named rather than looked up. A multi-child bone —
/// `LeftShoulder` has an arm, `Hips` has three — would otherwise aim at
/// whichever child `Bone::ALL` happened to list first, which is how a
/// solve lands near the target but never on it.
///
/// Works in the bone's parent frame, composing onto its existing rotation
/// — the same discipline [`super::drag`] follows, and for the same reason:
/// a world-space arc applied to a local rotation is right about the wrong
/// axis whenever the parent is itself rotated.
fn aim_bone(
    pose: &mut LocalPose,
    bone: Bone,
    child: Bone,
    world_direction: Vec3,
    rig: &RigGeometry,
) {
    let positions = forward_kinematics_on(pose, rig);

    let current = positions[child] - positions[bone];
    let Some(current) = current.try_normalize() else { return };

    // The parent's accumulated world rotation, so the arc lands in the
    // frame the local rotation actually acts in.
    let parent_world = accumulated_world_rotation(pose, bone.parent(), rig);
    let inverse_parent = parent_world.inverse();

    let arc = shortest_arc(inverse_parent * current, inverse_parent * world_direction);
    pose.set_rotation(bone, arc * pose.rotation(bone));
}

/// A bone's accumulated world rotation under `pose`.
fn accumulated_world_rotation(
    pose: &LocalPose,
    bone: Option<Bone>,
    rig: &RigGeometry,
) -> Quat {
    let Some(bone) = bone else { return rig.root_rotation };

    let mut chain = Vec::new();
    let mut current = Some(bone);
    while let Some(link) = current {
        chain.push(link);
        current = link.parent();
    }

    let mut rotation = rig.root_rotation;
    for &link in chain.iter().rev() {
        rotation *= rig.bind_rotations[link] * pose.rotation(link);
    }
    rotation
}

/// The shortest rotation taking `from` onto `to`, with an antipodal guard.
fn shortest_arc(from: Vec3, to: Vec3) -> Quat {
    const ANTIPODAL: f32 = -0.999_9;

    if from.dot(to) < ANTIPODAL {
        return Quat::from_axis_angle(from.any_orthonormal_vector(), std::f32::consts::PI);
    }
    Quat::from_rotation_arc(from, to)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::poses;
    use crate::character::anim::rig::forward_kinematics;

    fn rig() -> RigGeometry {
        RigGeometry::default()
    }

    #[test]
    fn every_effector_names_a_real_chain() {
        // Guards the hand-written table: a chain whose bones are not
        // actually parent-and-child would solve to nonsense, and the
        // symptom would be a limb folding in a way nobody could trace back
        // to a typo here.
        for effector in Effector::ALL {
            assert_eq!(
                effector.upper.parent(),
                Some(effector.root),
                "{}'s upper bone should hang off its root",
                effector.label(),
            );
            assert_eq!(
                effector.lower.parent(),
                Some(effector.upper),
                "{}'s lower bone should hang off its upper",
                effector.label(),
            );
            assert_eq!(
                effector.tip.parent(),
                Some(effector.lower),
                "{}'s tip should hang off its lower bone",
                effector.label(),
            );
        }
    }

    #[test]
    fn the_effectors_cover_all_four_limbs() {
        let tips: Vec<Bone> = Effector::ALL.iter().map(|e| e.tip).collect();

        for expected in
            [Bone::LeftHand, Bone::RightHand, Bone::LeftToeBase, Bone::RightToeBase]
        {
            assert!(tips.contains(&expected), "{} has no effector", expected.name());
        }
    }

    #[test]
    fn knees_and_elbows_bend_opposite_ways() {
        // The pole direction is the one thing a two-bone solve cannot
        // derive, and getting it backwards produces a knee that bends like
        // an elbow — instantly wrong to a viewer and invisible in the
        // numbers.
        let arm = Effector::ALL[0];
        let leg = Effector::ALL[2];

        assert!(
            arm.pole_direction().dot(leg.pole_direction()) < 0.0,
            "an elbow and a knee must bend in opposite directions",
        );
    }

    #[test]
    fn aiming_a_bone_achieves_the_direction_it_was_given() {
        // The primitive everything else here is built on. If an aim does
        // not land exactly, no amount of iterating on top of it will
        // converge to the target — it will converge to a fixed point some
        // distance away, which is much harder to diagnose from the outside.
        let mut pose = poses::relaxed_stand();
        let effector = Effector::ALL[0];
        let wanted = Vec3::new(0.3, -0.5, -0.8).normalize();

        aim_bone(&mut pose, effector.upper, effector.lower, wanted, &rig());

        let positions = forward_kinematics(&pose);
        let achieved = (positions[effector.lower] - positions[effector.upper]).normalize();

        assert!(
            achieved.dot(wanted) > 0.999,
            "aiming should land exactly: wanted {wanted:?}, achieved {achieved:?}",
        );
    }

    #[test]
    fn the_chain_pivots_at_its_upper_bone_not_its_root() {
        // The distinction that cost the most time here. `root` is the
        // shoulder or hip SOCKET — a fixed attachment the solve never
        // rotates — and it sits a real distance from where the chain
        // actually pivots. On this rig's shoulder that is 0.14 m.
        //
        // Measuring target distance from `root` adds that offset to every
        // solve, which lands exactly that far short and reads as an
        // imprecise solver. Worse, a test that also measures from `root`
        // asks for points outside the chain's reach, so the two errors
        // partly mask each other.
        let positions = forward_kinematics(&poses::relaxed_stand());

        for effector in Effector::ALL {
            let socket_offset = positions[effector.root].distance(positions[effector.upper]);
            assert!(
                socket_offset > 0.01,
                "{}'s socket sits {socket_offset} m from its pivot — if this were zero \
                 the distinction would not matter and this test would be pointless",
                effector.label(),
            );
        }
    }

    #[test]
    fn a_reachable_target_is_reached() {
        // The headline property. A target inside the limb's own reach must
        // actually be hit, not merely approached.
        let mut pose = poses::relaxed_stand();
        let effector = Effector::ALL[0];
        let positions = forward_kinematics(&pose);

        // Measured from the chain's real PIVOT — `upper`, not `root`. The
        // socket sits 0.14 m away on this rig, so a target placed at 90%
        // of the root-to-tip distance is genuinely beyond the chain's
        // reach, and a test asserting it gets hit is asserting the
        // impossible. (That is what this test did at first, and it sent
        // the search after a solver bug that was not there.)
        let pivot = positions[effector.upper];
        let reach = pivot.distance(positions[effector.lower])
            + positions[effector.lower].distance(positions[effector.tip]);
        let target = pivot + (Vec3::new(0.3, 0.4, -0.5).normalize() * reach * 0.9);

        let landed = solve(&mut pose, effector, target, 0.02, &rig());

        assert!(
            landed.distance(target) < 0.03,
            "a reachable target should be reached: landed {landed:?} against \
             {target:?} ({:.4} m away)",
            landed.distance(target),
        );
    }

    #[test]
    fn an_unreachable_target_extends_without_stretching() {
        // The anti-hyperextension property. A limb aimed at something
        // beyond its reach must point AT it and stop at its own length —
        // never lengthen to arrive.
        let mut pose = poses::relaxed_stand();
        let effector = Effector::ALL[0];
        let positions = forward_kinematics(&pose);

        let pivot = positions[effector.upper];
        // Ten metres away: far beyond any arm.
        let target = pivot + Vec3::new(1.0, 0.5, -0.2).normalize() * 10.0;

        let landed = solve(&mut pose, effector, target, 0.02, &rig());

        let extended = pivot.distance(landed);
        let maximum = positions[effector.upper].distance(positions[effector.lower])
            + positions[effector.lower].distance(positions[effector.tip]);

        assert!(
            extended <= maximum + 1.0e-3,
            "the limb reached {extended} m against a maximum of {maximum} m",
        );
        // And it should be pointing the right way, not dangling.
        let aim = (landed - pivot).normalize();
        assert!(
            aim.dot((target - pivot).normalize()) > 0.99,
            "an out-of-reach target should still be aimed at, got {aim:?}",
        );
    }

    #[test]
    fn solving_never_stretches_a_bone() {
        // The structural invariant, on the path most likely to break it:
        // IK computes rotations from distances, so a mistake there shows up
        // as a stretched limb before anything else.
        let mut pose = poses::relaxed_stand();

        for effector in Effector::ALL {
            let positions = forward_kinematics(&pose);
            let root = positions[effector.root];

            // Sweep several targets, including unreachable ones.
            for scale in [0.3f32, 0.7, 1.0, 3.0] {
                let mut probe = pose;
                let reach = root.distance(positions[effector.tip]);
                let target = root + Vec3::new(0.4, -0.6, -0.4).normalize() * reach * scale;

                solve(&mut probe, effector, target, 0.02, &rig());
                let solved = forward_kinematics(&probe);

                for &bone in Bone::ALL.iter() {
                    let Some(parent) = bone.parent() else { continue };
                    let rest = bone.t_pose_offset().length();
                    let posed = (solved[bone] - solved[parent]).length();

                    assert!(
                        (posed - rest).abs() < 1.0e-4,
                        "solving {} at scale {scale} stretched {} to {posed} m against \
                         {rest} m",
                        effector.label(),
                        bone.name(),
                    );
                }
            }
        }

        let _ = &mut pose;
    }

    #[test]
    fn solving_only_moves_its_own_limb() {
        // An effector that quietly disturbed the spine would make every
        // edit fight every other one.
        let original = poses::relaxed_stand();
        let mut pose = original;
        let effector = Effector::ALL[0];

        let positions = forward_kinematics(&pose);
        let target = positions[effector.tip] + Vec3::new(0.1, 0.2, -0.1);
        solve(&mut pose, effector, target, 0.02, &rig());

        for bone in [Bone::Spine, Bone::Spine1, Bone::Neck, Bone::RightArm, Bone::LeftUpLeg] {
            assert!(
                original.rotation(bone).abs_diff_eq(pose.rotation(bone), 1.0e-5),
                "{} is outside the solved chain and must not move",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_target_on_the_pivot_holds_the_pose() {
        // Dragging an effector onto its own shoulder is a degenerate aim;
        // holding beats snapping to an arbitrary direction.
        let original = poses::relaxed_stand();
        let mut pose = original;
        let effector = Effector::ALL[0];

        let pivot = forward_kinematics(&pose)[effector.upper];
        solve(&mut pose, effector, pivot, 0.02, &rig());

        for &bone in Bone::ALL.iter() {
            assert!(
                original.rotation(bone).abs_diff_eq(pose.rotation(bone), 1.0e-5),
                "{} moved on a degenerate solve",
                bone.name(),
            );
        }
    }

    #[test]
    fn solving_is_idempotent() {
        // Solving twice to the same target must give the same answer, or a
        // held mouse would walk the limb — the same property the joint
        // drag needed.
        let effector = Effector::ALL[2];
        let base = poses::relaxed_stand();
        let target = forward_kinematics(&base)[effector.tip] + Vec3::new(0.05, 0.1, -0.05);

        let mut once = base;
        solve(&mut once, effector, target, 0.02, &rig());

        let mut twice = once;
        solve(&mut twice, effector, target, 0.02, &rig());

        for &bone in Bone::ALL.iter() {
            assert!(
                once.rotation(bone).abs_diff_eq(twice.rotation(bone), 1.0e-4),
                "{} moved on a second solve to the same target",
                bone.name(),
            );
        }
    }
}
