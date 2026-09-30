//! Aiming a character's head at something.
//!
//! # Why this is spread across the spine
//!
//! Rotating only the head is the obvious implementation and it produces the
//! owl: a head that swivels independently of a body that has not noticed.
//! Real looking recruits the whole chain — the eyes lead, the head follows,
//! and past about 60 degrees the torso turns too.
//!
//! So a look is *distributed*: each bone in the chain contributes a share,
//! clamped to what that joint can actually do. The shares are fractions of
//! the total rather than absolute angles, so a target that needs 20 degrees
//! and one that needs 90 both come out proportioned rather than the second
//! saturating one joint and leaving the rest idle.
//!
//! # No eyes, and why
//!
//! `puppet_base.gltf` has an `Eyes` node, and it is a **skinned mesh, not a
//! joint** — `mesh=1, skin=0`, no children, and no eye joint anywhere in the
//! skin's 65. The eyes are geometry weighted to the head and physically
//! cannot move independently of it.
//!
//! Adding them would mean rewriting `JOINTS_0`/`WEIGHTS_0` binary data in a
//! 720 KB `.bin`, which is asset surgery on a downloaded model. This module
//! aims the head; eyes are a rig question rather than an animation one.
//!
//! # `Head` is a leaf, which makes gizmos lie
//!
//! On both rigs `Bone::Head` has no children — it is the top of the chain,
//! 0.083 m above `neck_01` on the real one. Rotating it therefore moves no
//! joint, and a gizmo view shows nothing happening.
//!
//! The head MESH is skinned to that joint, so it does turn. This is the
//! same trap as the earlier knee-visibility one, inverted: here the
//! verification needs `--show-real-mesh ON`, against the usual rule.

use bevy::math::{Quat, Vec3};

use super::rig::{forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// How far each joint may turn, and how much of a look it takes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LookAtConfig {
    /// Per-bone share of the total look, and that bone's own limit.
    ///
    /// Ordered from the base of the chain upward, so the spine turns before
    /// the head does when a target is far round.
    pub chain: [LookBone; 4],
    /// How quickly the look follows a moving target, as a half-life in
    /// seconds.
    ///
    /// A look that snaps reads as a machine noticing; one that eases reads
    /// as a person.
    pub halflife: f32,
    /// Beyond this angle from forward, the character gives up rather than
    /// contorting.
    ///
    /// A target directly behind is not something to look at without turning
    /// the whole body, which is a locomotion decision rather than this
    /// module's.
    pub max_angle: f32,
}

/// One joint's participation in a look.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LookBone {
    /// Which joint.
    pub bone: Bone,
    /// Its share of the total look, before clamping.
    pub share: f32,
    /// The furthest it may turn from its authored pose, radians.
    pub limit: f32,
}

impl Default for LookAtConfig {
    fn default() -> Self {
        Self {
            // Shares sum to 1.0, so a look inside every limit lands exactly
            // on target. Weighted toward the head, which is what actually
            // does most of the work at conversational angles; the spine
            // only earns its share once the head runs out of limit.
            chain: [
                LookBone { bone: Bone::Spine1, share: 0.10, limit: 0.30 },
                LookBone { bone: Bone::Spine2, share: 0.15, limit: 0.35 },
                LookBone { bone: Bone::Neck, share: 0.30, limit: 0.60 },
                LookBone { bone: Bone::Head, share: 0.45, limit: 0.70 },
            ],
            halflife: 0.12,
            // ~120 degrees. Past this a person turns their body, which is
            // not this module's decision to make.
            max_angle: 2.1,
        }
    }
}

/// A character's current look.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LookAt {
    /// Where to look, world space. `None` means look straight ahead.
    pub target: Option<Vec3>,
    /// The direction currently being looked along, in the character's own
    /// frame. Eased toward the target.
    current: Option<Vec3>,
}

impl LookAt {
    /// A character looking straight ahead.
    pub fn forward() -> Self {
        Self::default()
    }

    /// Looks at a world-space point, easing there from forward.
    pub fn at(target: Vec3) -> Self {
        Self { target: Some(target), current: None }
    }

    /// Looks at a world-space point, already settled on it.
    ///
    /// For a character that should BEGIN a scene looking somewhere, rather
    /// than turning to it on the first frame.
    pub fn settled_on(target: Vec3, from: Vec3) -> Self {
        let direction = (target - from).normalize_or_zero();

        Self {
            target: Some(target),
            current: (direction != Vec3::ZERO).then_some(direction),
        }
    }

    /// Stops looking at anything; the head returns to the authored pose.
    pub fn clear(&mut self) {
        self.target = None;
    }

    /// Eases the current look toward the target, and returns the direction
    /// to aim along — in the character's own frame, or `None` while there
    /// is nothing to look at and nothing left to ease back from.
    ///
    /// `head_position` and `facing` place the character in the world, so a
    /// world-space target becomes a body-relative direction.
    pub fn advance(
        &mut self,
        head_position: Vec3,
        facing: Quat,
        config: &LookAtConfig,
        dt: f32,
    ) -> Option<Vec3> {
        let wanted = self.target.and_then(|target| {
            let to_target = target - head_position;
            if to_target.length_squared() < 1.0e-8 {
                return None;
            }

            // Into the character's own frame, so the look is independent of
            // which way the body happens to be pointing.
            let local = facing.inverse() * to_target.normalize();

            // A target too far round is refused rather than contorted
            // toward. Turning the body is locomotion's decision.
            if local.dot(Vec3::NEG_Z).acos() > config.max_angle {
                None
            } else {
                Some(local)
            }
        });

        let goal = wanted.unwrap_or(Vec3::NEG_Z);

        self.current = Some(match self.current {
            None if wanted.is_none() => return None,

            // Acquiring a target from rest: ease from FORWARD rather than
            // adopting the goal outright.
            //
            // Snapping is what the first version did, and it is wrong for
            // the common case — a character noticing something mid-scene
            // should turn to it, not already be looking. A character that
            // should BEGIN looking somewhere gets that by constructing with
            // `settled_on`.
            None => ease_direction(Vec3::NEG_Z, goal, config.halflife, dt),

            Some(current) => {
                let eased = ease_direction(current, goal, config.halflife, dt);

                // Settled back to straight ahead with nothing to look at:
                // stop, so a character not looking at anything costs
                // nothing and writes nothing.
                if wanted.is_none() && eased.dot(Vec3::NEG_Z) > 0.9999 {
                    self.current = None;
                    return None;
                }

                eased
            }
        });

        self.current
    }
}

/// Moves `current` toward `goal` with an exponential half-life.
///
/// Slerped rather than lerped-and-normalised: a lerp between two widely
/// separated directions passes closer to the origin, so the eased direction
/// would swing wide before arriving.
fn ease_direction(current: Vec3, goal: Vec3, halflife: f32, dt: f32) -> Vec3 {
    if halflife <= 0.0 || dt <= 0.0 || !dt.is_finite() {
        return goal;
    }

    let decay = (-std::f32::consts::LN_2 * dt / halflife).exp();
    let t = 1.0 - decay;

    let from = Quat::from_rotation_arc(Vec3::NEG_Z, current.normalize_or_zero());
    let to = Quat::from_rotation_arc(Vec3::NEG_Z, goal.normalize_or_zero());

    (from.slerp(to, t) * Vec3::NEG_Z).normalize_or_zero()
}

/// Composes a look onto `pose`, aiming the head along `direction`.
///
/// `direction` is in the character's own frame — what [`LookAt::advance`]
/// returns. Each bone in the chain takes its share, clamped to its own
/// limit, and any share a clamped joint could not absorb is offered to the
/// ones above it.
pub fn apply(
    pose: &mut LocalPose,
    direction: Vec3,
    config: &LookAtConfig,
    rig: &RigGeometry,
) {
    let direction = direction.normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }

    // How far the whole chain has to turn, and about what.
    let total = Quat::from_rotation_arc(Vec3::NEG_Z, direction);
    let (axis, angle) = total.to_axis_angle();

    if angle < 1.0e-4 {
        return;
    }

    let mut remaining = angle;

    for link in config.chain.iter() {
        // This bone's share of what is LEFT, so a joint that could not take
        // its share passes the rest upward rather than dropping it.
        let wanted = (angle * link.share).min(remaining);
        let taken = wanted.min(link.limit);

        if taken <= 0.0 {
            continue;
        }

        // `axis` is a world axis, so this is a world-space correction and goes
        // through the same conjugation every other one here does — per bone,
        // because each link sits under the ones already turned above it. See
        // `legik::world_correction_frame`.
        //
        // The composition SIDE is load-bearing, not cosmetic: measured on
        // `aim_bone`, post-multiplying an otherwise-correct world-space delta
        // lands 0.073 m off on a 0.251 m bone.
        let delta = Quat::from_axis_angle(axis.normalize_or_zero(), taken);
        let frame = super::legik::world_correction_frame(pose, link.bone, rig);

        pose.set_rotation(
            link.bone,
            frame.inverse() * delta * frame * pose.rotation(link.bone),
        );

        remaining -= taken;
        if remaining <= 1.0e-4 {
            break;
        }
    }
}

/// Where the head sits under `pose`, for aiming from.
pub fn head_position(pose: &LocalPose, rig: &RigGeometry) -> Vec3 {
    forward_kinematics_on(pose, rig)[Bone::Head]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gltf_rig;
    use crate::character::anim::rig::accumulate_world_rotations;
    use crate::character::anim::stance::stance;

    const DT: f32 = 1.0 / 60.0;

    /// `puppet_base` as it is drawn, facing −Z: a look is a world-axis
    /// turn of the character's forward, so the fixture must face the way
    /// the character does.
    fn setup() -> (LocalPose, RigGeometry) {
        (stance(&LocalPose::REST), gltf_rig::puppet_base_as_rendered())
    }

    /// Where the face is pointing, in world space, under `pose`: the rig's
    /// forward carried by the head's turn since the rest pose.
    ///
    /// `Head` is a LEAF on both rigs, so there is no child joint whose
    /// position reveals the direction — it has to come from the bone's own
    /// accumulated rotation, which is what the skinned mesh follows. Not
    /// that rotation times −Z: the bone's own −Z is not the face. On the
    /// drawn rig it points out of the back of the head, and on plain
    /// `puppet_base()` (which faces away) it happened to start at −Z, so
    /// these tests passed while measuring the back of the head.
    fn head_direction(pose: &LocalPose, rig: &RigGeometry) -> Vec3 {
        let rest = accumulate_world_rotations(&LocalPose::REST, rig)[Bone::Head];
        accumulate_world_rotations(pose, rig)[Bone::Head] * rest.inverse() * rig.forward()
    }

    // -----------------------------------------------------------------
    // Aiming
    // -----------------------------------------------------------------

    #[test]
    fn looking_ahead_changes_nothing() {
        // The identity case. A character looking where it already faces
        // must not be perturbed, or every idle pose drifts.
        let (base, rig) = setup();

        let mut pose = base;
        apply(&mut pose, Vec3::NEG_Z, &LookAtConfig::default(), &rig);

        for &bone in Bone::ALL.iter() {
            assert!(
                pose.rotation(bone).abs_diff_eq(base.rotation(bone), 1.0e-5),
                "{} moved for a look straight ahead",
                bone.name(),
            );
        }
    }

    #[test]
    fn the_head_turns_toward_the_direction_it_is_given() {
        let (base, rig) = setup();
        let config = LookAtConfig::default();

        let before = head_direction(&base, &rig);

        // A modest look to the character's left and slightly up.
        let wanted = Vec3::new(-0.4, 0.2, -1.0).normalize();

        let mut pose = base;
        apply(&mut pose, wanted, &config, &rig);

        let after = head_direction(&pose, &rig);

        assert!(
            after.dot(wanted) > before.dot(wanted),
            "the head should turn toward {wanted:?}, but went from {before:?} to \
             {after:?}",
        );
    }

    #[test]
    fn a_look_recruits_the_whole_chain_rather_than_only_the_head() {
        // THE reason this is distributed. Rotating only the head produces
        // the owl: a head that swivels independently of a body that has not
        // noticed.
        let (base, rig) = setup();
        let config = LookAtConfig::default();

        let mut pose = base;
        apply(&mut pose, Vec3::new(-0.8, 0.0, -1.0).normalize(), &config, &rig);

        let mut moved = 0;
        for link in config.chain.iter() {
            if !pose.rotation(link.bone).abs_diff_eq(base.rotation(link.bone), 1.0e-4) {
                moved += 1;
            }
        }

        assert!(
            moved >= 3,
            "only {moved} of the chain's joints moved — the look is not being \
             distributed",
        );
    }

    #[test]
    fn no_joint_exceeds_its_own_limit() {
        // A clamp that only holds in aggregate would still let one joint
        // hyperextend while the others idle.
        let (base, rig) = setup();
        let config = LookAtConfig::default();

        // Far enough round that every joint is asked for more than it has.
        for direction in [
            Vec3::new(-1.0, 0.0, -0.2).normalize(),
            Vec3::new(1.0, 0.0, -0.2).normalize(),
            Vec3::new(0.0, 1.0, -0.2).normalize(),
        ] {
            let mut pose = base;
            apply(&mut pose, direction, &config, &rig);

            for link in config.chain.iter() {
                let delta = base.rotation(link.bone).inverse() * pose.rotation(link.bone);
                let (_, angle) = delta.to_axis_angle();

                assert!(
                    angle <= link.limit + 1.0e-3,
                    "{} turned {angle} rad against its {} limit, looking {direction:?}",
                    link.bone.name(),
                    link.limit,
                );
            }
        }
    }

    #[test]
    fn a_reachable_look_lands_on_target() {
        // The shares sum to 1.0, so a look inside every limit should arrive
        // exactly rather than falling short.
        let (base, rig) = setup();
        let config = LookAtConfig::default();

        let wanted = Vec3::new(-0.2, 0.1, -1.0).normalize();

        let mut pose = base;
        apply(&mut pose, wanted, &config, &rig);

        // The face, carried from where it faced before the look.
        let before = head_direction(&base, &rig);
        let achieved = head_direction(&pose, &rig);
        let turn = Quat::from_rotation_arc(rig.forward(), wanted);
        let error = achieved.angle_between(turn * before).to_degrees();
        assert!(error < 0.5, "a reachable look should arrive, but ended {error:.2} degrees off");
    }

    #[test]
    fn a_look_that_exceeds_the_chain_saturates_rather_than_contorting() {
        let (base, rig) = setup();
        let config = LookAtConfig::default();

        let mut pose = base;
        apply(&mut pose, Vec3::new(-1.0, 0.0, 0.3).normalize(), &config, &rig);

        // Every bone length preserved — a look cannot stretch a neck.
        let positions = forward_kinematics_on(&pose, &rig);
        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };
            let rest = rig.offsets[bone].length();
            let posed = (positions[bone] - positions[parent]).length();

            assert!(
                (posed - rest).abs() < 1.0e-4,
                "{} is {posed} against a rest length of {rest}",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_look_leaves_the_legs_and_arms_alone() {
        let (base, rig) = setup();

        let mut pose = base;
        apply(&mut pose, Vec3::new(-0.8, 0.2, -1.0).normalize(), &LookAtConfig::default(), &rig);

        for bone in [
            Bone::LeftUpLeg,
            Bone::RightUpLeg,
            Bone::LeftFoot,
            Bone::LeftHand,
            Bone::RightHand,
        ] {
            assert!(
                pose.rotation(bone).abs_diff_eq(base.rotation(bone), 1.0e-6),
                "{} moved for a look",
                bone.name(),
            );
        }
    }

    // -----------------------------------------------------------------
    // Easing
    // -----------------------------------------------------------------

    #[test]
    fn a_look_eases_toward_its_target_rather_than_snapping() {
        let mut look = LookAt::at(Vec3::new(-2.0, 1.5, -1.0));
        let config = LookAtConfig::default();

        let first = look
            .advance(Vec3::new(0.0, 1.5, 0.0), Quat::IDENTITY, &config, DT)
            .expect("a target should produce a direction");

        // One frame should have started the turn, not finished it.
        assert!(
            first.dot(Vec3::NEG_Z) > 0.5,
            "one frame moved the look all the way to {first:?}",
        );
    }

    #[test]
    fn a_look_reaches_its_target_direction() {
        let mut look = LookAt::at(Vec3::new(-2.0, 1.5, -1.0));
        let config = LookAtConfig::default();
        let head = Vec3::new(0.0, 1.5, 0.0);

        let mut direction = Vec3::NEG_Z;
        for _ in 0..120 {
            direction = look
                .advance(head, Quat::IDENTITY, &config, DT)
                .expect("still looking");
        }

        let wanted = (Vec3::new(-2.0, 1.5, -1.0) - head).normalize();
        assert!(
            direction.dot(wanted) > 0.999,
            "the look settled on {direction:?} rather than {wanted:?}",
        );
    }

    #[test]
    fn clearing_a_look_eases_back_to_forward_and_then_stops() {
        // A character not looking at anything should cost nothing and write
        // nothing — but it must EASE back rather than snapping.
        let mut look = LookAt::at(Vec3::new(-2.0, 1.5, -1.0));
        let config = LookAtConfig::default();
        let head = Vec3::new(0.0, 1.5, 0.0);

        for _ in 0..120 {
            look.advance(head, Quat::IDENTITY, &config, DT);
        }

        look.clear();

        let first_after = look
            .advance(head, Quat::IDENTITY, &config, DT)
            .expect("should still be easing back");
        assert!(
            first_after.dot(Vec3::NEG_Z) < 0.999,
            "clearing a look should ease back, not snap: {first_after:?}",
        );

        for _ in 0..600 {
            look.advance(head, Quat::IDENTITY, &config, DT);
        }

        assert_eq!(
            look.advance(head, Quat::IDENTITY, &config, DT),
            None,
            "once settled, a cleared look should stop producing a direction",
        );
    }

    #[test]
    fn a_look_is_expressed_in_the_characters_own_frame() {
        // The same world target seen from two headings must produce
        // different LOCAL directions — otherwise a turning character drags
        // its look around with it.
        let config = LookAtConfig::default();
        let head = Vec3::new(0.0, 1.5, 0.0);
        let target = Vec3::new(0.0, 1.5, -3.0);

        let mut ahead = LookAt::at(target);
        let mut turned = LookAt::at(target);

        let a = ahead
            .advance(head, Quat::IDENTITY, &config, 1.0)
            .expect("looking");
        let b = turned
            .advance(head, Quat::from_rotation_y(1.0), &config, 1.0)
            .expect("looking");

        assert!(
            a.angle_between(b) > 0.5,
            "a target dead ahead of one heading is off to the side of another, but \
             both produced {a:?} and {b:?}",
        );
    }

    #[test]
    fn a_target_too_far_round_is_refused() {
        // Past the limit a person turns their body, which is locomotion's
        // decision rather than this module's.
        let config = LookAtConfig::default();
        let head = Vec3::new(0.0, 1.5, 0.0);

        // Directly behind.
        let mut look = LookAt::at(Vec3::new(0.0, 1.5, 5.0));

        let direction = look.advance(head, Quat::IDENTITY, &config, 1.0);

        // Either nothing, or eased back toward forward — never aimed
        // backward.
        if let Some(direction) = direction {
            assert!(
                direction.z < 0.0,
                "a target behind should not aim the head backward, got {direction:?}",
            );
        }
    }

    #[test]
    fn a_target_at_the_head_is_safe() {
        let config = LookAtConfig::default();
        let head = Vec3::new(0.0, 1.5, 0.0);

        let mut look = LookAt::at(head);
        let direction = look.advance(head, Quat::IDENTITY, &config, DT);

        if let Some(direction) = direction {
            assert!(direction.is_finite(), "got {direction:?}");
        }
    }

    #[test]
    fn a_non_positive_timestep_is_safe() {
        let config = LookAtConfig::default();
        let head = Vec3::ZERO;

        let mut look = LookAt::at(Vec3::new(1.0, 0.0, -1.0));
        let a = look.advance(head, Quat::IDENTITY, &config, 0.0);
        let b = look.advance(head, Quat::IDENTITY, &config, f32::NAN);

        for direction in [a, b].into_iter().flatten() {
            assert!(direction.is_finite(), "got {direction:?}");
        }
    }

    #[test]
    fn the_chains_shares_sum_to_one() {
        // So a look inside every limit lands exactly on target rather than
        // falling short by whatever the shares happen to miss by.
        let total: f32 = LookAtConfig::default().chain.iter().map(|l| l.share).sum();

        assert!(
            (total - 1.0).abs() < 1.0e-5,
            "the chain's shares sum to {total} rather than 1.0",
        );
    }

    #[test]
    fn the_chain_runs_from_the_spine_upward() {
        // Ordering matters: the spine must be offered its share before the
        // head, so a far-round target turns the body rather than only the
        // neck.
        let chain = LookAtConfig::default().chain;

        for pair in chain.windows(2) {
            let (lower, upper) = (pair[0].bone, pair[1].bone);
            assert_eq!(
                upper.parent(),
                Some(lower),
                "{} should sit directly above {} in the chain",
                upper.name(),
                lower.name(),
            );
        }
    }
}
