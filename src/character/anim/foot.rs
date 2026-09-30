//! Where a foot meets the ground: its heel, the ball, and the toe tip.
//!
//! A walking foot is not planted at its ankle. Winter's stance (Appendix A;
//! section 11.3.1) rolls over rockers: the heel strikes with the toes up
//! (25 degrees, Table A.3(a)) and the foot pivots down about it, the foot
//! lies flat while the shank rotates over the ankle, the heel rises about
//! the ball, and in pre-swing the foot rolls onto the toes — the recording's
//! fifth-metatarsal marker climbs from 3.4 to 9.6 cm while its toe marker
//! stays down (Tables A.2(c)/(d), frames 63-70). The centre of pressure
//! travels 0.26 m heel to toe. Treating the ankle joint as the fixed point
//! makes all of that impossible: a foot that rolls must slide.
//!
//! So a foot has three contact points, carried rigidly in the ankle bone's
//! frame: the **heel**, under and behind the ankle; the **ball**, under the
//! toe joint; and the **tip**, under the end of the toes. Whichever are
//! lowest bear the weight.

use bevy::math::{Quat, Vec3};

use super::rig::{frame_from, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// How far the heel's contact lies behind the ankle, as a fraction of the
/// ankle-to-ball distance.
///
/// From Winter's foot markers at foot-flat (Tables A.2(c)/(d), frames
/// 40-50): the heel sits 0.060 m behind the lateral malleolus and the fifth
/// metatarsal head 0.094-0.099 m ahead of it. A fraction, so the heel scales
/// with the rig's own foot.
pub const HEEL_BEHIND_ANKLE: f32 = 0.61;

/// Height over which the load hands from one contact point to another,
/// metres. Well under the lift at a heel strike or a heel rise
/// (centimetres), so each rocker is a clean pivot, and above zero so the
/// hand-over through foot-flat is smooth.
pub const HANDOVER_HEIGHT: f32 = 0.004;

/// Height over which the BODY's weight hands from one foot to the other,
/// metres ([`support_height`], [`bearing`]). Wider than a single foot's
/// heel-to-toe hand-over, so the pelvis and root motion change feet
/// smoothly: at 4 mm the slow walk's root velocity stepped 0.15 m/s at each
/// heel strike.
pub const SUPPORT_HANDOVER: f32 = 0.01;

/// The contact points, heel to tip.
pub type Contacts = [Vec3; 3];

/// One foot's contact points, in its ankle bone's own frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sole {
    ankle: Bone,
    contacts: Contacts,
    /// The bind pose's up, in the ankle's frame: the sole's normal.
    up: Vec3,
}

/// The bones that carry one foot: ankle and toe.
pub fn foot_bones(ankle: Bone) -> (Bone, Bone) {
    match ankle {
        Bone::RightFoot => (Bone::RightFoot, Bone::RightToeBase),
        _ => (Bone::LeftFoot, Bone::LeftToeBase),
    }
}

impl Sole {
    /// Measured on the rig's bind pose, standing flat on the ground at `y = 0`
    /// — or on its foot's lowest joint, if one dips below that.
    ///
    /// A real asset binds its foot joints above its floor (`puppet_base`'s
    /// toe 15.2 mm, `character.glb`'s 4.9 mm), and the sole is the floor
    /// under them. The synthetic rig is stylised: its ankle sits exactly on
    /// `y = 0` with the toe 2 cm below it, so the floor there would put the
    /// "sole" above the toe; its sole is the plane of its lowest joint.
    pub fn of(rig: &RigGeometry, ankle: Bone) -> Self {
        let (ankle, toe) = foot_bones(ankle);
        let rest = LocalPose::REST;
        let world = forward_kinematics_on(&rest, rig);
        let (_, ankle_rotation) = frame_from(&rest, rig, Bone::Hips, ankle);
        let toe_rotation = super::rig::accumulate_world_rotations(&rest, rig)[toe];
        let (ankle_at, toe_at) = (world[ankle], world[toe]);
        let tip_at = toe_at + toe_rotation * rig.toe_end_offset(toe);

        let forward = rig.forward();
        let ahead = (toe_at - ankle_at).dot(forward);
        let floor = ankle_at.y.min(toe_at.y).min(tip_at.y).min(0.0);
        let ground = |p: Vec3| Vec3::new(p.x, floor, p.z);
        let heel = ground(ankle_at) - forward * (HEEL_BEHIND_ANKLE * ahead);

        let into_foot = |p: Vec3| ankle_rotation.inverse() * (p - ankle_at);
        Self {
            ankle,
            contacts: [heel, ground(toe_at), ground(tip_at)].map(into_foot),
            up: ankle_rotation.inverse() * Vec3::Y,
        }
    }

    /// Heel, ball and tip, relative to the hips, under `pose`.
    pub fn points(&self, pose: &LocalPose, rig: &RigGeometry) -> Contacts {
        let (ankle, rotation): (Vec3, Quat) = frame_from(pose, rig, Bone::Hips, self.ankle);
        self.contacts.map(|c| ankle + rotation * c)
    }
}

/// A foot's breadth as a fraction of its length: Winter §4.0.1 (Fig. 4.1),
/// breadth 0.055·H against length 0.152·H.
pub const FOOT_BREADTH_PER_LENGTH: f32 = 0.055 / 0.152;

/// How thick a foot's sole block is, metres: enough to stand on, thin
/// enough to stay under the ankle.
pub const SOLE_THICKNESS: f32 = 0.03;

/// A foot as a flat block standing on its sole, in its ankle bone's own
/// frame: a physics foot that can bear weight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SoleBox {
    /// The block's centre, in the ankle bone's frame.
    pub center: Vec3,
    /// Its orientation in that frame: `x` across the foot, `y` up from the
    /// sole, `z` from heel to toe tip.
    pub rotation: Quat,
    /// Its full size along those axes, metres.
    pub size: Vec3,
}

impl Sole {
    /// The foot as a block: heel to toe tip along the sole ([`Sole`]'s own
    /// contacts), [`FOOT_BREADTH_PER_LENGTH`] across, [`SOLE_THICKNESS`] up
    /// from the sole's plane.
    ///
    /// A single capsule from ankle to ball, which the ragdoll's foot used to
    /// be, has no heel and no flat underside: stood on, it rolled, and an
    /// unpinned ragdoll's feet skated 0.8 m in 2.5 s.
    pub fn block(&self) -> SoleBox {
        let [heel, _, tip] = self.contacts;
        let along = tip - heel;
        // The sole's own plane, in the ankle frame: the contacts lie on the
        // bind floor, so the plane's normal is the bind's up. Taken from the
        // three contacts' spread across the sole's length and the floor's
        // level: `up` is perpendicular to `along` and to the across axis.
        let length = along.length().max(1.0e-3);
        let forward = along / length;
        let up_guess = self.up;
        let across = up_guess.cross(forward).normalize_or_zero();
        let up = forward.cross(across).normalize_or_zero();
        let rotation = Quat::from_mat3(&bevy::math::Mat3::from_cols(across, up, forward));
        let size = Vec3::new(length * FOOT_BREADTH_PER_LENGTH, SOLE_THICKNESS, length);
        SoleBox { center: (heel + tip) * 0.5 + up * (SOLE_THICKNESS * 0.5), rotation, size }
    }
}

/// How the load is shared between the contact points, summing to 1: the
/// lowest carry it, handing over smoothly within [`HANDOVER_HEIGHT`].
pub fn shares(points: &Contacts) -> [f32; 3] {
    let lowest = points.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    let weights = points.map(|p| (-(p.y - lowest) / HANDOVER_HEIGHT).exp());
    let total: f32 = weights.iter().sum();
    weights.map(|w| w / total)
}

/// The height of a foot's weight-bearing contact: the [`shares`] blend.
pub fn lowest(points: &Contacts) -> f32 {
    let s = shares(points);
    points.iter().zip(s).map(|(p, s)| p.y * s).sum()
}

/// How far the weight-bearing contact moved between two poses.
///
/// The share-weighted DISPLACEMENT of the points — not the displacement of
/// a share-weighted point, which slides along the sole as the load hands
/// over: a centre of pressure moves, the body does not.
pub fn contact_moved(before: &Contacts, after: &Contacts) -> Vec3 {
    let middle = [0, 1, 2].map(|i| (before[i] + after[i]) * 0.5);
    let s = shares(&middle);
    (0..3).map(|i| (after[i] - before[i]) * s[i]).sum()
}

/// How much of the body's weight each planted foot bears: its share in
/// [`support_height`]'s soft maximum — its stance load, discounted by how
/// far below the supporting foot's need its own falls.
///
/// `heights` are the feet's weight-bearing contacts ([`lowest`]) relative to
/// the hips, so a HIGHER contact needs the body lower, and the body rests on
/// the foot that reaches lowest. THE SAME weights as the pelvis height,
/// deliberately: root motion once weighted by load times a separate
/// "touching" test, and at a heel strike the two disagreed about which foot
/// carried the body — the pelvis rode the trailing toe while root motion
/// half-followed the new heel, which slid 37 mm a stance.
pub fn bearing(loads: [f32; 2], heights: [f32; 2]) -> [f32; 2] {
    // A foot's need is how far the body must rise for it to touch: the
    // negation of its contact's height relative to the hips, up to a
    // constant shared by both feet on a symmetric rig.
    let needs = heights.map(|h| -h);
    support_shares(loads, needs)
}

/// Each foot's share in [`support_height`]'s soft maximum, summing to 1
/// (or all zero when no foot is loaded).
pub fn support_shares(loads: [f32; 2], needs: [f32; 2]) -> [f32; 2] {
    let total = loads[0] + loads[1];
    if total <= 1.0e-6 {
        return [0.0; 2];
    }
    let top = (0..2).filter(|&i| loads[i] > 0.0).map(|i| needs[i]).fold(f32::MIN, f32::max);
    let terms = [0, 1].map(|i| {
        if loads[i] > 0.0 { loads[i] * ((needs[i] - top) / SUPPORT_HANDOVER).exp() } else { 0.0 }
    });
    let sum = terms[0] + terms[1];
    terms.map(|t| t / sum)
}

/// How far the body must rise for its planted feet to stand on the ground,
/// given how far each would need it to on its own (`needs`) and their
/// stance loads.
///
/// A soft MAXIMUM, not an average: the body cannot sit so low that a
/// planted foot would pass through the floor, so it rests on whichever leg
/// holds it highest and the other foot, if the two disagree, is lifted
/// clear of the ground — which is what the trailing foot does in pre-swing.
/// An average pushed one foot under the floor and lifted the other in every
/// double support, and dropped the body 5 cm at each heel strike. Weighted
/// by load so a foot eases in at footfall and out at toe-off.
pub fn support_height(loads: [f32; 2], needs: [f32; 2]) -> Option<f32> {
    let total = loads[0] + loads[1];
    if total <= 1.0e-6 {
        return None;
    }
    let top = (0..2).filter(|&i| loads[i] > 0.0).map(|i| needs[i]).fold(f32::MIN, f32::max);
    let sum: f32 = (0..2).map(|i| loads[i] * ((needs[i] - top) / SUPPORT_HANDOVER).exp()).sum();
    Some(top + SUPPORT_HANDOVER * (sum / total).ln())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gltf_rig::puppet_base;

    #[test]
    fn a_standing_foot_rests_heel_ball_and_tip_on_the_ground() {
        let rig = puppet_base();
        let rest = LocalPose::REST;
        let world = forward_kinematics_on(&rest, &rig);
        let f = rig.forward();
        for ankle in [Bone::LeftFoot, Bone::RightFoot] {
            let [heel, ball, tip] = Sole::of(&rig, ankle).points(&rest, &rig).map(|p| world[Bone::Hips] + p);
            for p in [heel, ball, tip] {
                assert!(p.y.abs() < 1.0e-4, "{p}");
            }
            // Heel behind the ankle and ball ahead, by Winter's proportion;
            // the tip further ahead still.
            let behind = (world[ankle] - heel).dot(f);
            let ahead = (ball - world[ankle]).dot(f);
            assert!((behind / ahead - HEEL_BEHIND_ANKLE).abs() < 1.0e-3);
            assert!((tip - ball).dot(f) > 0.02);
        }
    }

    #[test]
    fn the_lowest_point_carries_the_load() {
        let at = |y: [f32; 3]| [Vec3::new(0.0, y[0], 0.0), Vec3::new(0.0, y[1], 0.1), Vec3::new(0.0, y[2], 0.2)];
        let heel_strike = shares(&at([0.0, 0.05, 0.07]));
        assert!(heel_strike[0] > 0.999);
        let pre_swing = shares(&at([0.2, 0.04, 0.0]));
        assert!(pre_swing[2] > 0.99);
        let flat = shares(&at([0.0, 0.0, 0.0]));
        assert!(flat.iter().all(|s| (s - 1.0 / 3.0).abs() < 1.0e-6));
        // A rigid foot translating without turning: every point moves alike.
        let moved = contact_moved(&at([0.0, 0.0, 0.0]), &at([0.0, 0.0, 0.0]).map(|p| p + Vec3::X));
        assert!((moved - Vec3::X).length() < 1.0e-6);
    }
}
