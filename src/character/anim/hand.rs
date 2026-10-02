//! A relaxed hand: the fingers' resting curl.
//!
//! The animation drives 22 bones, and no finger is one of them, so a rig's
//! fingers kept their bind: a T-pose's flat, straight hand. Live, walking,
//! the middle finger bent 9° over its whole length, a hand held out stiff.
//! A relaxed hand is not flat. With nothing to hold, the fingers fall into
//! a curl that deepens from the index to the ring finger.
//!
//! [`RELAXED_FLEXION_DEGREES`] is Lee et al.'s measurement (*Relaxed hand
//! postures*, J. Ergonomics 44 spl., 2008; 15 men, Vicon): the forearm in
//! neutral, palm toward the thigh, the arm hanging (shoulder 0°). Each
//! finger is bent at its three joints toward the palm as the rig binds
//! ([`relax_hands`]); the thumb bends across the palm ([`bends_toward`]).
//!
//! A falling hand stays limp, and each finger gives way on its own where it
//! meets the floor: a palm pressed down straightens its fingers, knuckles
//! landing first curl them further. Curled and left alone, a fallen body's
//! fingertips went 45-80 mm into the floor. Straightening the whole hand
//! while down made it a board. So [`RelaxedHands`] keeps each finger's flat
//! and relaxed rotations, and [`finger_bent`] draws a finger anywhere from
//! flat (0) through relaxed (1) to further curled. The ragdoll's drawn pose
//! bends each finger in contact as little as keeps its tip out of the
//! ground, straighter when its palm faces the ground and curled further
//! when it faces away ([`palm_faces_ground`]), and lets it ease back to
//! relaxed at [`RELAX_RATE`]. Picked afresh each frame as the smaller bend,
//! a finger flipped from 86° curled to 8° in one frame.
//!
//! The palm's normal is read from the rig, not assumed: across the
//! knuckles (index to little) and along the middle finger, crossed in the
//! order that makes it point out of the palm on each side. On
//! `puppet_base`'s T-pose bind it points down on both hands, which is how
//! that bind holds its palms
//! (`the_palm_normal_points_down_on_the_t_pose_bind`).

use std::collections::HashMap;

use bevy::prelude::*;

use crate::character::skeleton::Side;
use crate::character::HumanoidSkeleton;

/// The five fingers, thumb first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Finger {
    Thumb,
    Index,
    Middle,
    Ring,
    Little,
}

impl Finger {
    pub const ALL: [Finger; 5] = [Finger::Thumb, Finger::Index, Finger::Middle, Finger::Ring, Finger::Little];

    /// The finger's three joints by name, then the leaf past its tip: the
    /// Unreal Mannequin's (`index_01_l` … `index_04_leaf_l`) and Mixamo's
    /// (`mixamorig:LeftHandIndex1` … `4`, prefixed or bare).
    pub fn joint_names(self, side: Side) -> [[String; 4]; 3] {
        let (ue, mixamo) = match self {
            Finger::Thumb => ("thumb", "Thumb"),
            Finger::Index => ("index", "Index"),
            Finger::Middle => ("middle", "Middle"),
            Finger::Ring => ("ring", "Ring"),
            Finger::Little => ("pinky", "Pinky"),
        };
        let (s, hand) = if side == Side::Right { ("r", "RightHand") } else { ("l", "LeftHand") };
        [
            [1, 2, 3, 4].map(|k| if k == 4 { format!("{ue}_04_leaf_{s}") } else { format!("{ue}_0{k}_{s}") }),
            [1, 2, 3, 4].map(|k| format!("mixamorig:{hand}{mixamo}{k}")),
            [1, 2, 3, 4].map(|k| format!("{hand}{mixamo}{k}")),
        ]
    }
}

/// How far each finger's joints bend in a relaxed hand, in degrees: knuckle
/// (MCP), middle (PIP) and end (DIP) joint. The thumb's are its base (CMC,
/// not measured, left straight), MCP and IP joints.
///
/// Lee et al., Table 3, neutral forearm, shoulder 0°. Their standard
/// deviations are 3-9° at most joints.
pub const RELAXED_FLEXION_DEGREES: [(Finger, [f32; 3]); 5] = [
    (Finger::Thumb, [0.0, 46.1, 8.5]),
    (Finger::Index, [28.4, 25.5, 13.1]),
    (Finger::Middle, [32.8, 30.1, 12.7]),
    (Finger::Ring, [24.7, 34.5, 11.7]),
    (Finger::Little, [16.6, 32.1, 15.6]),
];

/// One finger's bind, in the hand's frame.
#[derive(Debug, Clone, Copy)]
pub struct FingerBind {
    /// The frame the first joint hangs from: the hand itself, or a
    /// metacarpal bone between them on rigs that have one.
    pub base: (Quat, Vec3),
    /// Each joint's local rotation and translation.
    pub joints: [(Quat, Vec3); 3],
    /// The leaf past the tip, under the last joint.
    pub tip: Vec3,
}

impl FingerBind {
    /// Where the first joint (the knuckle) sits.
    pub fn knuckle(&self) -> Vec3 {
        self.base.1 + self.base.0 * self.joints[0].1
    }

    /// The direction of the segment the first joint carries.
    pub fn first_segment(&self) -> Vec3 {
        (self.base.0 * self.joints[0].0 * self.joints[1].1).normalize_or_zero()
    }
}

/// The palm's normal in the hand's frame, out of the palm: across the
/// knuckles from the little finger toward the index, along the middle
/// finger, crossed in the order that is "out of the palm" on `side`.
pub fn palm_normal(index: &FingerBind, middle: &FingerBind, little: &FingerBind, side: Side) -> Vec3 {
    let across = index.knuckle() - little.knuckle();
    let along = middle.first_segment();
    let normal = if side == Side::Right { across.cross(along) } else { along.cross(across) };
    normal.normalize_or_zero()
}

/// Which way `finger` bends, in the hand's frame: a finger toward the palm,
/// the thumb across it toward the little finger.
///
/// The thumb's bind already points out in front of the palm (on
/// `puppet_base`, 0.45 of its length along the palm's normal). Bent toward
/// the palm like the fingers, it stuck out sideways into the thigh; bent
/// across, it comes to lie along the index finger, as a relaxed thumb does.
pub fn bends_toward(finger: Finger, index: &FingerBind, little: &FingerBind, palm: Vec3) -> Vec3 {
    match finger {
        Finger::Thumb => (little.knuckle() - index.knuckle()).normalize_or_zero(),
        _ => palm,
    }
}

/// The finger's three joints' local rotations, each bent `degrees` toward
/// `palm` (in the hand's frame: the palm's normal for a finger, see
/// [`bends_toward`]).
///
/// Each joint turns about its own segment crossed with the palm's normal,
/// which carries the segment toward the palm; the axis is taken after the
/// joints above have bent, so a curled finger stays in its own plane.
pub fn curled(bind: &FingerBind, palm: Vec3, degrees: [f32; 3]) -> [Quat; 3] {
    let mut above = bind.base.0;
    let mut out = [Quat::IDENTITY; 3];
    for k in 0..3 {
        let (rotation, _) = bind.joints[k];
        let frame = above * rotation;
        let next = if k < 2 { bind.joints[k + 1].1 } else { bind.tip };
        let axis = (frame * next).cross(palm);
        let bend = if axis.length_squared() > 1.0e-12 {
            Quat::from_axis_angle((frame.inverse() * axis).normalize(), degrees[k].to_radians())
        } else {
            Quat::IDENTITY
        };
        out[k] = rotation * bend;
        above = frame * bend;
    }
    out
}

/// The finger's three joints' local rotations for a hand laid flat on the
/// floor: a finger straight (its bind), the thumb turned at its base into
/// the palm's plane.
///
/// The bind's thumb points out in front of the palm (on `puppet_base`, 0.45
/// of its length along the palm's normal), so on a palm laid flat its tip
/// went 76-83 mm into the floor, live, in a fall and in the get-up's
/// push-up.
pub fn flat(finger: Finger, bind: &FingerBind, palm: Vec3) -> [Quat; 3] {
    let out_of_palm = bind.first_segment().dot(palm);
    if finger != Finger::Thumb || out_of_palm <= 0.0 {
        return bind.joints.map(|(rotation, _)| rotation);
    }
    curled(bind, -palm, [out_of_palm.clamp(0.0, 1.0).asin().to_degrees(), 0.0, 0.0])
}

/// The furthest a finger curls past relaxed to keep its tip out of the
/// floor, as a multiple of its relaxed bend: about a loose fist.
pub const MOST_CURL: f32 = 1.8;

/// How fast a finger let go by the floor returns to relaxed, in bends per
/// second (a whole bend in 0.25 s). Snapped back, a finger leaving the
/// floor popped.
pub const RELAX_RATE: f32 = 4.0;

/// The fastest a finger bends against the floor, in bends per second (flat
/// to relaxed in 83 ms, five frames). Unlimited, a finger whose hand turned
/// its palm over in one frame jumped 106-139 mm across the hand; what a
/// limited finger cannot clear, the wrist turn lifts.
pub const BEND_RATE: f32 = 12.0;

/// One finger's joints: each joint's entity and its flat ([`flat`]) and
/// relaxed local rotations, the leaf past its tip, and the bend it is drawn
/// at ([`finger_bent`]).
#[derive(Debug, Clone, Copy)]
pub struct FingerJoints {
    pub joints: [(Entity, Quat, Quat); 3],
    pub tip: Entity,
    pub bend: f32,
}

/// A character's fingers, left hand then right.
#[derive(Component, Debug, Clone)]
pub struct RelaxedHands {
    pub fingers: [Vec<FingerJoints>; 2],
    /// Each hand's palm normal, in the hand's own frame ([`palm_normal`]).
    pub palms: [Vec3; 2],
    /// Whether each hand's fingers give way straighter (its palm toward the
    /// ground) rather than curled further; see [`palm_faces_ground`].
    pub straighten: [bool; 2],
}

/// How far a palm must face the ground (cosine to straight down) before its
/// fingers give way straighter, and away from it before they curl instead;
/// between the two a hand keeps its choice, so it does not flicker.
pub const PALM_DOWN: f32 = 0.2;

/// Whether a hand's fingers give way straighter: a palm pressed to the
/// floor flattens its fingers, the back of a hand or its knuckles landing
/// curls them. `palm` is the palm's normal in the world; `was` the last
/// choice.
///
/// Kept on one side whatever the hand did, a finger curled by a landing
/// stayed a fist under the get-up's flat palm, and the wrist turned 60° in
/// one frame to clear it.
pub fn palm_faces_ground(palm: Vec3, was: bool) -> bool {
    let down = -palm.y;
    if down > PALM_DOWN {
        true
    } else if down < -PALM_DOWN {
        false
    } else {
        was
    }
}

/// `finger`'s joints' local rotations at `bend`: 0 flat, 1 relaxed, more
/// curled further (each joint's turn from flat scaled alike).
pub fn finger_bent(finger: &FingerJoints, bend: f32) -> [Quat; 3] {
    finger.joints.map(|(_, flat, relaxed)| {
        let turn = (flat.inverse() * relaxed).normalize();
        // The short way round, so a scaled turn never goes the long way.
        let turn = if turn.w < 0.0 { -turn } else { turn };
        (flat * Quat::from_scaled_axis(turn.to_scaled_axis() * bend)).normalize()
    })
}

/// Curls the fingers of every humanoid that has just bound into
/// [`RELAXED_FLEXION_DEGREES`], and gives it [`RelaxedHands`]. A rig
/// without finger joints is left alone, as is any finger whose joints are
/// not all found.
pub fn relax_hands(
    mut commands: Commands,
    bound: Query<(Entity, &HumanoidSkeleton), Added<HumanoidSkeleton>>,
    children_of: Query<&Children>,
    child_of: Query<&ChildOf>,
    names: Query<&Name>,
    mut transforms: Query<&mut Transform>,
) {
    for (root, skeleton) in &bound {
        let mut relaxed = RelaxedHands { fingers: [Vec::new(), Vec::new()], palms: [Vec3::ZERO; 2], straighten: [true; 2] };
        let mut by_name: HashMap<&str, Entity> = HashMap::new();
        let mut stack = vec![root];
        while let Some(entity) = stack.pop() {
            if let Ok(children) = children_of.get(entity) {
                stack.extend(children.iter());
            }
            if let Ok(name) = names.get(entity) {
                by_name.insert(name.as_str(), entity);
            }
        }

        for (side, hand_bone) in [(Side::Left, crate::character::Bone::LeftHand), (Side::Right, crate::character::Bone::RightHand)] {
            let hand = skeleton.entity(hand_bone);
            let local = |entity: Entity| transforms.get(entity).map(|t| (t.rotation, t.translation)).ok();

            let mut fingers: Vec<(Finger, [Entity; 4], FingerBind)> = Vec::new();
            for finger in Finger::ALL {
                let Some(chain) = finger
                    .joint_names(side)
                    .iter()
                    .find_map(|names| names.iter().map(|n| by_name.get(n.as_str()).copied()).collect::<Option<Vec<_>>>())
                else {
                    continue;
                };
                // The frame between the hand and the first joint.
                let mut base = (Quat::IDENTITY, Vec3::ZERO);
                let mut between = Vec::new();
                let mut cursor = chain[0];
                let reached = loop {
                    let Ok(parent) = child_of.get(cursor).map(ChildOf::parent) else { break false };
                    if parent == hand {
                        break true;
                    }
                    between.push(parent);
                    cursor = parent;
                };
                if !reached {
                    continue;
                }
                for &node in between.iter().rev() {
                    let Some((rotation, translation)) = local(node) else { continue };
                    base = (base.0 * rotation, base.1 + base.0 * translation);
                }
                let (Some(j0), Some(j1), Some(j2), Some((_, tip))) = (local(chain[0]), local(chain[1]), local(chain[2]), local(chain[3])) else {
                    continue;
                };
                fingers.push((finger, [chain[0], chain[1], chain[2], chain[3]], FingerBind { base, joints: [j0, j1, j2], tip }));
            }

            let find = |wanted: Finger| fingers.iter().find(|(f, _, _)| *f == wanted).map(|(_, _, bind)| *bind);
            let (Some(index), Some(middle), Some(little)) = (find(Finger::Index), find(Finger::Middle), find(Finger::Little)) else {
                continue;
            };
            let palm = palm_normal(&index, &middle, &little, side);
            relaxed.palms[usize::from(side == Side::Right)] = palm;
            for (finger, joints, bind) in &fingers {
                let Some(&(_, degrees)) = RELAXED_FLEXION_DEGREES.iter().find(|(f, _)| f == finger) else { continue };
                let toward = bends_toward(*finger, &index, &little, palm);
                let bent = curled(bind, toward, degrees);
                let flattened = flat(*finger, bind, palm);
                for k in 0..3 {
                    if let Ok(mut transform) = transforms.get_mut(joints[k]) {
                        transform.rotation = bent[k];
                    }
                }
                relaxed.fingers[usize::from(side == Side::Right)].push(FingerJoints {
                    joints: [0, 1, 2].map(|k| (joints[k], flattened[k], bent[k])),
                    tip: joints[3],
                    bend: 1.0,
                });
            }
        }
        if relaxed.fingers.iter().any(|hand| !hand.is_empty()) {
            commands.entity(root).insert(relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gltf_rig::bind_node;

    /// One finger of `puppet_base` as it binds, and the hand's world bind
    /// rotation.
    fn finger(finger: Finger, side: Side) -> (FingerBind, Quat) {
        let names = &finger.joint_names(side)[0];
        let hand = if side == Side::Right { "hand_r" } else { "hand_l" };
        let (hand_rotation, _, above_hand) = bind_node(hand).unwrap();
        let joint = |k: usize| {
            let (rotation, translation, _) = bind_node(&names[k]).unwrap();
            (rotation, translation)
        };
        let bind = FingerBind { base: (Quat::IDENTITY, Vec3::ZERO), joints: [joint(0), joint(1), joint(2)], tip: joint(3).1 };
        (bind, above_hand * hand_rotation)
    }

    fn palm(side: Side) -> (Vec3, Quat) {
        let (index, hand) = finger(Finger::Index, side);
        let (middle, _) = finger(Finger::Middle, side);
        let (little, _) = finger(Finger::Little, side);
        (palm_normal(&index, &middle, &little, side), hand)
    }

    /// Which way `which` bends on `side`, in the hand's frame.
    fn toward(which: Finger, side: Side) -> Vec3 {
        let (index, _) = finger(Finger::Index, side);
        let (little, _) = finger(Finger::Little, side);
        bends_toward(which, &index, &little, palm(side).0)
    }

    /// Joint positions down the finger, in the hand's frame, with its
    /// joints' local rotations replaced by `rotations`.
    fn positions(bind: &FingerBind, rotations: [Quat; 3]) -> [Vec3; 4] {
        let (mut frame, mut at) = bind.base;
        let mut out = [Vec3::ZERO; 4];
        for k in 0..3 {
            at += frame * bind.joints[k].1;
            out[k] = at;
            frame *= rotations[k];
        }
        out[3] = at + frame * bind.tip;
        out
    }

    #[test]
    fn the_palm_normal_points_down_on_the_t_pose_bind() {
        // Independent of the cross product's order: a T-pose holds its
        // palms to the floor. A wrong order curls the fingers backward.
        for side in [Side::Left, Side::Right] {
            let (normal, hand) = palm(side);
            let world = hand * normal;
            assert!(world.y < -0.9, "{side:?} palm normal in the world: {world:?}");
        }
    }

    #[test]
    fn a_relaxed_finger_bends_each_joint_by_its_measured_angle_toward_the_palm() {
        for side in [Side::Left, Side::Right] {
            for &(which, degrees) in &RELAXED_FLEXION_DEGREES {
                let normal = toward(which, side);
                let (bind, _) = finger(which, side);
                let flat = positions(&bind, bind.joints.map(|(r, _)| r));
                let bent = positions(&bind, curled(&bind, normal, degrees));
                for k in 0..3 {
                    // Each segment's turn against its parent's, signed by
                    // how far toward the palm it went.
                    let before = flat[k + 1] - flat[k];
                    let after = bent[k + 1] - bent[k];
                    let parent_turn = if k == 0 {
                        0.0
                    } else {
                        (flat[k] - flat[k - 1]).angle_between(bent[k] - bent[k - 1]).to_degrees()
                    };
                    let turn = before.angle_between(after).to_degrees() - parent_turn;
                    // A straight bind finger: the turns add down the chain.
                    assert!(
                        (turn - degrees[k]).abs() < 2.5,
                        "{side:?} {which:?} joint {k}: bent {turn:.1} degrees, measured {:.1}",
                        degrees[k]
                    );
                }
                let tip_toward_palm = (bent[3] - flat[3]).dot(normal);
                if degrees.iter().sum::<f32>() > 0.0 {
                    assert!(tip_toward_palm > 0.01, "{side:?} {which:?}: the tip moved {tip_toward_palm:.3} m toward the palm");
                }
                // Bones keep their lengths.
                for k in 0..3 {
                    let (a, b) = ((flat[k + 1] - flat[k]).length(), (bent[k + 1] - bent[k]).length());
                    assert!((a - b).abs() < 1.0e-5);
                }
            }
        }
    }

    #[test]
    fn a_relaxed_thumb_comes_in_toward_the_index_finger_not_out_of_the_palm() {
        // Bent toward the palm like a finger, the thumb's tip went further
        // out of the palm and stuck sideways into the thigh (live).
        for side in [Side::Left, Side::Right] {
            let (normal, _) = palm(side);
            let (index, _) = finger(Finger::Index, side);
            let (thumb, _) = finger(Finger::Thumb, side);
            let degrees = RELAXED_FLEXION_DEGREES[0].1;
            let flat = positions(&thumb, thumb.joints.map(|(r, _)| r));
            let bent = positions(&thumb, curled(&thumb, toward(Finger::Thumb, side), degrees));
            let knuckle = index.knuckle();
            let lateral = |p: Vec3| {
                let off = p - knuckle;
                (off - normal * off.dot(normal)).length()
            };
            assert!(
                lateral(bent[3]) < lateral(flat[3]) && (bent[3] - flat[3]).dot(normal) < 0.01,
                "{side:?} thumb tip: {:.3} m beside the index knuckle (was {:.3}), {:.3} m further out of the palm",
                lateral(bent[3]),
                lateral(flat[3]),
                (bent[3] - flat[3]).dot(normal),
            );
        }
    }

    #[test]
    fn a_flat_hands_thumb_lies_in_the_palms_plane() {
        // The bind's thumb points out in front of the palm; on a palm laid
        // flat its tip went 76-83 mm into the floor.
        for side in [Side::Left, Side::Right] {
            let (normal, _) = palm(side);
            let (thumb, _) = finger(Finger::Thumb, side);
            let bind = positions(&thumb, thumb.joints.map(|(r, _)| r));
            let laid = positions(&thumb, flat(Finger::Thumb, &thumb, normal));
            let out = |p: [Vec3; 4]| (p[3] - p[0]).normalize().dot(normal);
            assert!(out(bind) > 0.3, "{side:?}: the bind thumb was expected out of the palm: {:.2}", out(bind));
            assert!(out(laid).abs() < 0.02, "{side:?}: a flat thumb out of the palm's plane by {:.2}", out(laid));
        }
    }

    #[test]
    fn a_finger_bent_runs_flat_relaxed_and_further() {
        let (bind, _) = finger(Finger::Middle, Side::Left);
        let (normal, _) = palm(Side::Left);
        let relaxed = curled(&bind, normal, RELAXED_FLEXION_DEGREES[2].1);
        let joints = FingerJoints {
            joints: [0, 1, 2].map(|k| (Entity::PLACEHOLDER, bind.joints[k].0, relaxed[k])),
            tip: Entity::PLACEHOLDER,
            bend: 1.0,
        };
        let at = |bend| finger_bent(&joints, bend);
        for k in 0..3 {
            assert!(at(0.0)[k].angle_between(bind.joints[k].0) < 1.0e-4);
            assert!(at(1.0)[k].angle_between(relaxed[k]) < 1.0e-4);
            // Half again past relaxed: each joint's turn from flat x1.5.
            let past = bind.joints[k].0.angle_between(at(1.5)[k]);
            assert!((past - 1.5 * bind.joints[k].0.angle_between(relaxed[k])).abs() < 1.0e-3);
        }
        // Further curled brings the tip further toward the palm.
        let tip = |bend| positions(&bind, at(bend))[3].dot(normal);
        assert!(tip(1.5) > tip(1.0) && tip(1.0) > tip(0.0));
    }

    #[test]
    fn a_palm_toward_the_ground_straightens_its_fingers_and_away_curls_them() {
        assert!(palm_faces_ground(Vec3::NEG_Y, false));
        assert!(!palm_faces_ground(Vec3::Y, true));
        // Near vertical, a hand keeps its last choice.
        let edge = Vec3::new(1.0, -0.1, 0.0).normalize();
        assert!(palm_faces_ground(edge, true) && !palm_faces_ground(edge, false));
    }

    #[test]
    fn the_hands_curl_as_mirror_images() {
        // Signed toward each hand's own palm, so a mirrored rig curls both
        // the same way: the tips' moves, mirrored across x, agree.
        let (_, left_hand) = palm(Side::Left);
        let (_, right_hand) = palm(Side::Right);
        for &(which, degrees) in &RELAXED_FLEXION_DEGREES {
            let tip_move = |side, hand: Quat| {
                let (bind, _) = finger(which, side);
                let flat = positions(&bind, bind.joints.map(|(r, _)| r));
                hand * (positions(&bind, curled(&bind, toward(which, side), degrees))[3] - flat[3])
            };
            let left = tip_move(Side::Left, left_hand);
            let right = tip_move(Side::Right, right_hand);
            let mirrored = Vec3::new(-right.x, right.y, right.z);
            assert!((left - mirrored).length() < 0.01, "{which:?}: left {left:?}, right {right:?}");
        }
    }
}
