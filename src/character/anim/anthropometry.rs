//! Body segment parameters and the whole-body centre of mass.
//!
//! From Winter, *Biomechanics and Motor Control of Human Movement*, Table 4.1
//! (Dempster's cadaver data; printed p. 86, PDF 99): each segment's mass as a
//! fraction of body mass, and where its centre of mass (COM) sits along it
//! as a fraction of its length from the proximal end. The knowledge-base note
//! `docs/knowledge/biomechanics-winter/ch04-anthropometry/4.1-density-mass-inertial-properties/4.1.3-segment-mass-and-center-of-mass.md`
//! holds the full table and its reading notes.
//!
//! # The segments, on this rig
//!
//! Winter's rows are defined by anatomical landmarks; each maps onto rig
//! joints as follows, and the fractions sum to exactly 1:
//!
//! | Segment | Table 4.1 row | Proximal → distal on the rig | Mass | COM from proximal |
//! |---|---|---|---|---|
//! | trunk | "Trunk", greater trochanter / glenohumeral joint | mid-hip-sockets → mid-shoulder-joints | 0.497 | 0.50 |
//! | head and neck | C7–T1 / ear canal, COM **at** the ear canal | `Head` joint, raised by [`HEAD_COM_ABOVE_JOINT`] | 0.081 | — |
//! | upper arm ×2 | glenohumeral / elbow axis | `Arm` → `ForeArm` | 0.028 | 0.436 |
//! | forearm and hand ×2 | elbow axis / ulnar styloid | `ForeArm` → `Hand` | 0.022 | 0.682 |
//! | thigh ×2 | greater trochanter / femoral condyles | `UpLeg` → `Leg` | 0.100 | 0.433 |
//! | shank ×2 | femoral condyles / medial malleolus | `Leg` → `Foot` | 0.0465 | 0.433 |
//! | foot ×2 | lateral malleolus / head of metatarsal II | `Foot` → `ToeBase` | 0.0145 | 0.50 |
//!
//! Winter's "Trunk" row is used rather than his thorax/abdomen/pelvis split,
//! because its landmarks are rig joints; the split's fractions are ambiguous
//! about which length they multiply (the table's own footnote).

use bevy::math::Vec3;

use super::rig::{offset_from, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// Mass fractions of body mass, Table 4.1.
pub mod fraction {
    /// Head and neck.
    pub const HEAD_AND_NECK: f32 = 0.081;
    /// Trunk: thorax, abdomen and pelvis together.
    pub const TRUNK: f32 = 0.497;
    /// Thorax, C7–T1 to T12–L1.
    pub const THORAX: f32 = 0.216;
    /// Abdomen, T12–L1 to L4–L5.
    pub const ABDOMEN: f32 = 0.139;
    /// Pelvis, L4–L5 to the greater trochanter.
    pub const PELVIS: f32 = 0.142;
    /// One upper arm.
    pub const UPPER_ARM: f32 = 0.028;
    /// One forearm and hand.
    pub const FOREARM_AND_HAND: f32 = 0.022;
    /// One thigh.
    pub const THIGH: f32 = 0.100;
    /// One shank ("leg" in Winter).
    pub const SHANK: f32 = 0.0465;
    /// One foot.
    pub const FOOT: f32 = 0.0145;
}

/// Where a limb segment's mass sits and how it is spread, Table 4.1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LimbSegment {
    /// Mass as a fraction of body mass.
    pub mass: f32,
    /// The COM's distance from the proximal joint, as a fraction of the
    /// segment's length.
    pub com_from_proximal: f32,
    /// Radius of gyration about the COM, as a fraction of the segment's
    /// length: the segment's inertia about a TRANSVERSE axis through its COM
    /// is `mass · (ρ · L)²`. Table 4.1 is a 2D (sagittal) table, so this is
    /// the flexion/extension axis; the book gives no twist inertia.
    pub gyration_about_com: f32,
}

/// The limb segment `bone` starts, if its Table 4.1 row matches the span a
/// rig bone covers: the bone's joint to its child's.
///
/// Upper arm (glenohumeral to elbow), forearm-and-hand (elbow to ulnar
/// styloid, carrying the hand's mass — the hand is not a separate body),
/// thigh (trochanter to condyles), shank (condyles to malleolus), foot
/// (malleolus to the second metatarsal head, the rig's toe joint).
pub fn limb_segment(bone: Bone) -> Option<LimbSegment> {
    use fraction::*;
    let (mass, com_from_proximal, gyration_about_com) = match bone {
        Bone::LeftArm | Bone::RightArm => (UPPER_ARM, 0.436, 0.322),
        Bone::LeftForeArm | Bone::RightForeArm => (FOREARM_AND_HAND, 0.682, 0.468),
        Bone::LeftUpLeg | Bone::RightUpLeg => (THIGH, 0.433, 0.323),
        Bone::LeftLeg | Bone::RightLeg => (SHANK, 0.433, 0.302),
        Bone::LeftFoot | Bone::RightFoot => (FOOT, 0.50, 0.475),
        _ => return None,
    };
    Some(LimbSegment { mass, com_from_proximal, gyration_about_com })
}

/// How far above the `Head` joint the head-and-neck COM sits, as a fraction
/// of the neck-to-head joint distance, along that line.
///
/// Winter puts the COM at the ear canal. A Mixamo-style `Head` joint is the
/// skull base, and the ear canal lies a little above it; this is an
/// ESTIMATE, not a measurement. The head is 8.1% of the body, so a 3 cm
/// error here moves the whole-body COM by 2.4 mm.
pub const HEAD_COM_ABOVE_JOINT: f32 = 0.45;

/// The whole-body centre of mass under `pose`, relative to the hips, in the
/// rig's frame. Add the pose's `root_translation` for the rig's world.
///
/// Measure a pose authored in fixed world axes on a rig facing the way the
/// renderer shows it (`gltf_rig::puppet_base_as_rendered` for the gallery's
/// character): on the unturned rig such a pose puts the arms overhead.
pub fn centre_of_mass(pose: &LocalPose, rig: &RigGeometry) -> Vec3 {
    let at = |bone| offset_from(pose, rig, Bone::Hips, bone);
    let along = |from: Bone, to: Bone, t: f32| {
        let a = at(from);
        a + (at(to) - a) * t
    };
    let mid = |a: Bone, b: Bone| (at(a) + at(b)) * 0.5;

    let hips = mid(Bone::LeftUpLeg, Bone::RightUpLeg);
    let shoulders = mid(Bone::LeftArm, Bone::RightArm);
    let head = {
        let (neck, joint) = (at(Bone::Neck), at(Bone::Head));
        joint + (joint - neck) * HEAD_COM_ABOVE_JOINT
    };

    use fraction::*;
    let mut sum = (hips + shoulders) * 0.5 * TRUNK + head * HEAD_AND_NECK;
    for (proximal, distal) in [
        (Bone::LeftArm, Bone::LeftForeArm),
        (Bone::LeftForeArm, Bone::LeftHand),
        (Bone::RightArm, Bone::RightForeArm),
        (Bone::RightForeArm, Bone::RightHand),
        (Bone::LeftUpLeg, Bone::LeftLeg),
        (Bone::LeftLeg, Bone::LeftFoot),
        (Bone::LeftFoot, Bone::LeftToeBase),
        (Bone::RightUpLeg, Bone::RightLeg),
        (Bone::RightLeg, Bone::RightFoot),
        (Bone::RightFoot, Bone::RightToeBase),
    ] {
        if let Some(segment) = limb_segment(proximal) {
            sum += along(proximal, distal, segment.com_from_proximal) * segment.mass;
        }
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_segments_add_up_to_the_whole_body() {
        use fraction::*;
        let total = TRUNK + HEAD_AND_NECK + 2.0 * (UPPER_ARM + FOREARM_AND_HAND + THIGH + SHANK + FOOT);
        assert!((total - 1.0).abs() < 1.0e-6, "{total}");
        // The trunk row is the thorax, abdomen and pelvis rows together.
        assert!((THORAX + ABDOMEN + PELVIS - TRUNK).abs() < 1.0e-6);
    }

    #[test]
    fn a_standing_body_holds_its_mass_between_its_feet_just_above_the_hips() {
        // The result, not the formula: in the bind pose (arms out, legs
        // straight) the centre of mass lies on the body's midline, a little
        // above the hip joints, and — Winter's static stance, Example 5.1 —
        // a few centimetres ahead of the ankles.
        let rig = super::super::gltf_rig::puppet_base_as_rendered();
        let rest = LocalPose::REST;
        let com = centre_of_mass(&rest, &rig);
        let at = |bone| offset_from(&rest, &rig, Bone::Hips, bone);
        let hips = (at(Bone::LeftUpLeg) + at(Bone::RightUpLeg)) * 0.5;
        let ankles = (at(Bone::LeftFoot) + at(Bone::RightFoot)) * 0.5;
        assert!(com.dot(rig.left()).abs() < 0.01, "sideways {}", com.dot(rig.left()));
        assert!((0.0..0.15).contains(&(com.y - hips.y)), "{} above the hip joints", com.y - hips.y);
        let ahead = (com - ankles).dot(rig.forward());
        assert!((0.02..0.07).contains(&ahead), "{ahead} m ahead of the ankles");
    }
}
