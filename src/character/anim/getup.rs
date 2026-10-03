//! Getting up off the floor: the intermediate poses a fallen body rises
//! through, and which ones it takes.
//!
//! Two routes, by how the body lies:
//!
//! - **Face up:** lying → sitting → squatting → standing. VanSant (1988,
//!   *Phys Ther* 68:185) filmed 32 young adults rising from supine; the
//!   most common pattern was symmetrical throughout: a push with both
//!   arms, the trunk flexing forward, through sitting to a squat, then up.
//! - **Face down:** lying → hands and knees → half-kneeling → standing: the
//!   quadruped route of floor-to-stand studies (half-kneel then push up).
//! - **On the side:** lying → side-sitting → hands and knees →
//!   half-kneeling → standing: pushed up on the hand underneath onto the
//!   hip, legs folded to the other side, then over onto the knees. Side-sit
//!   to quadruped is a route floor-to-stand studies record (e.g. in older
//!   adults rising independently); it is chosen here because a body on its
//!   side would otherwise be read as face up or down and rolled 90° about
//!   its own length in one blend.
//!
//! Each key is authored as sagittal joint angles about the rig's own
//! measured `left` axis ([`RigGeometry::left`]), never an assumed one, then
//! *solved* on the rig so its contacts meet the floor together (knees and
//! hands, a foot and a knee, …), and set down on them. A pose delta here
//! names a world axis in the bind frame, and a bone's world rotation is the
//! product of the deltas from the root down to it, so angles about `left`
//! add along a chain: a foot stays flat when pelvis + hip + knee + ankle
//! sum to zero. Positive about `left` swings an upward segment's far end
//! forward (flexing the trunk) and a downward one's backward (bending the
//! knee); hip flexion is negative.

use bevy::math::{Quat, Vec3};

use super::foot::Sole;
use super::rig::{forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// How a fallen body lies: which way its chest faces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lying {
    /// Chest up (supine): it sits up.
    FaceUp,
    /// Chest down (prone): it pushes up onto hands and knees.
    FaceDown,
    /// On its side, the chest facing sideways: it pushes up on the hand
    /// underneath into a side-sit. `left_down`: lying on its left side.
    Side { left_down: bool },
}

impl Lying {
    /// How a body lies whose chest faces `chest_forward` and whose left
    /// points `chest_left` (world, unit): face up or down while the chest
    /// faces more than 45° from level, else on its side.
    pub fn of(chest_forward: Vec3, chest_left: Vec3) -> Self {
        let level = std::f32::consts::FRAC_1_SQRT_2;
        if chest_forward.y > level {
            Lying::FaceUp
        } else if chest_forward.y < -level {
            Lying::FaceDown
        } else {
            Lying::Side { left_down: chest_left.y < 0.0 }
        }
    }
}

/// One pose on the way up, and how long the move into it takes, seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GetUpKey {
    /// The pose, in the character's frame, standing on the floor at `y = 0`.
    pub pose: LocalPose,
    /// How long the blend into it from the previous pose takes.
    pub seconds: f32,
}

/// How long the move from the last key up to standing takes, seconds.
pub const STAND_SECONDS: f32 = 0.8;

/// The keys a body lying `lying` rises through, before standing. Timings are
/// choices: about 2.5 s from lying to standing, a young adult's unhurried
/// rise.
///
/// The keys are chained, each placed along the rig's forward so the contact
/// it shares with the next stays where it is: the squat's and the
/// half-kneel's front foot where the standing feet will be, the sitting
/// feet where the squat's are, the knee on hands and knees where the
/// half-kneel's is. Built each at the character's origin, the half-kneel's
/// front foot slid 0.34 m back as it stood.
pub fn keys(lying: Lying, rig: &RigGeometry) -> Vec<GetUpKey> {
    let standing = forward_kinematics_on(&LocalPose::REST, rig);
    let (first, second, first_on, second_on) = match lying {
        Lying::FaceUp => (sit(rig), squat(rig), Bone::LeftFoot, Bone::LeftFoot),
        Lying::FaceDown | Lying::Side { .. } => (quadruped(rig), half_kneel(rig), Bone::RightLeg, Bone::LeftFoot),
    };
    let second = placed(second, rig, second_on, standing[Bone::LeftFoot]);
    let first = placed(first, rig, first_on, forward_kinematics_on(&second, rig)[first_on]);
    let mut keys = vec![GetUpKey { pose: first, seconds: 0.9 }, GetUpKey { pose: second, seconds: 0.8 }];
    // On the side, the side-sit comes first, its propping hand where the
    // hands-and-knees hand on that side will be.
    if let Lying::Side { left_down } = lying {
        let hand = if left_down { Bone::LeftHand } else { Bone::RightHand };
        let sat = placed(side_sit(rig, left_down), rig, hand, forward_kinematics_on(&first, rig)[hand]);
        keys.insert(0, GetUpKey { pose: sat, seconds: 1.0 });
    }
    keys
}

/// `pose` moved along the rig's forward until `bone` is level with `at`.
pub(crate) fn placed(mut pose: LocalPose, rig: &RigGeometry, bone: Bone, at: bevy::math::Vec3) -> LocalPose {
    let forward = rig.forward();
    let off = (forward_kinematics_on(&pose, rig)[bone] - at).dot(forward);
    pose.root_translation -= forward * off;
    pose
}

/// How far a contact's joint sits above the floor when it bears weight,
/// metres. Choices, from the flesh around each joint: a knee on the floor,
/// a palm under the wrist, the seat under the hip joint.
pub(crate) const KNEE_CLEARANCE: f32 = 0.05;
pub(crate) const HAND_CLEARANCE: f32 = 0.03;
pub(crate) const SEAT_CLEARANCE: f32 = 0.10;

/// A joint that touches the floor in a key.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Contact {
    /// A foot, by its sole's lowest point ([`Sole`]).
    Foot(Bone),
    /// A joint, held its clearance above the floor.
    Joint(Bone, f32),
}

/// The contacts each key stands on; see [`contacts_of`].
fn contact_list(key: Key) -> Vec<Contact> {
    use Contact::*;
    match key {
        Key::Sit => vec![Joint(Bone::Hips, SEAT_CLEARANCE), Foot(Bone::LeftFoot), Foot(Bone::RightFoot), Joint(Bone::LeftHand, HAND_CLEARANCE), Joint(Bone::RightHand, HAND_CLEARANCE)],
        Key::Squat => vec![Foot(Bone::LeftFoot), Foot(Bone::RightFoot)],
        Key::Quadruped => vec![
            Joint(Bone::LeftLeg, KNEE_CLEARANCE),
            Joint(Bone::RightLeg, KNEE_CLEARANCE),
            Joint(Bone::LeftHand, HAND_CLEARANCE),
            Joint(Bone::RightHand, HAND_CLEARANCE),
        ],
        Key::HalfKneel => vec![Joint(Bone::RightLeg, KNEE_CLEARANCE), Foot(Bone::LeftFoot)],
        Key::SideSit { left_down } => {
            let hand = if left_down { Bone::LeftHand } else { Bone::RightHand };
            vec![
                Joint(Bone::Hips, SEAT_CLEARANCE),
                Joint(hand, HAND_CLEARANCE),
                Joint(Bone::LeftLeg, KNEE_CLEARANCE),
                Joint(Bone::RightLeg, KNEE_CLEARANCE),
            ]
        }
    }
}

/// The named keys, for tests and tools.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Key {
    Sit,
    Squat,
    Quadruped,
    HalfKneel,
    SideSit { left_down: bool },
}

impl Key {
    /// Every key.
    pub const ALL: [Key; 6] = [
        Key::Sit,
        Key::Squat,
        Key::Quadruped,
        Key::HalfKneel,
        Key::SideSit { left_down: true },
        Key::SideSit { left_down: false },
    ];

    /// The key's pose on `rig`.
    pub fn pose(self, rig: &RigGeometry) -> LocalPose {
        match self {
            Key::Sit => sit(rig),
            Key::Squat => squat(rig),
            Key::Quadruped => quadruped(rig),
            Key::HalfKneel => half_kneel(rig),
            Key::SideSit { left_down } => side_sit(rig, left_down),
        }
    }

    /// Whether the key is left/right symmetric.
    pub fn symmetric(self) -> bool {
        matches!(self, Key::Sit | Key::Squat | Key::Quadruped)
    }
}

/// What a key stands on.
pub fn contacts_of(key: Key) -> Vec<Contact> {
    contact_list(key)
}

/// How high `contact` is above the floor under `pose`, metres.
pub fn contact_height(pose: &LocalPose, rig: &RigGeometry, contact: Contact) -> f32 {
    let at = forward_kinematics_on(pose, rig);
    match contact {
        Contact::Joint(bone, clearance) => at[bone].y - clearance,
        Contact::Foot(ankle) => {
            let hips = at[Bone::Hips];
            Sole::of(rig, ankle).points(pose, rig).iter().map(|p| hips.y + p.y).fold(f32::MAX, f32::min)
        }
    }
}

/// Sets `pose` down so its lowest contact touches the floor.
pub(crate) fn set_down(pose: &mut LocalPose, rig: &RigGeometry, contacts: &[Contact]) {
    let lowest = contacts.iter().map(|&c| contact_height(pose, rig, c)).fold(f32::MAX, f32::min);
    pose.root_translation.y -= lowest;
}

/// A rotation about the rig's `left` by `degrees`.
pub(crate) fn about_left(rig: &RigGeometry, degrees: f32) -> Quat {
    Quat::from_axis_angle(rig.left(), degrees.to_radians())
}

/// The product of the deltas from the root down to `bone`: the world turn
/// everything below it rides.
pub(crate) fn carried(pose: &LocalPose, bone: Bone) -> Quat {
    let mut chain = Vec::new();
    let mut walker = Some(bone);
    while let Some(current) = walker {
        chain.push(current);
        walker = current.parent();
    }
    chain.iter().rev().fold(Quat::IDENTITY, |turn, &b| turn * pose.rotation(b))
}

/// Points both arms: each hangs straight down, then swings forward by
/// `forward_degrees` (negative: back), in the WORLD, whatever the trunk
/// above it has done. Elbows straight.
pub(crate) fn hang_arms(pose: &mut LocalPose, rig: &RigGeometry, forward_degrees: f32) {
    let forward = rig.forward();
    for (arm, down) in [(Bone::LeftArm, -1.0), (Bone::RightArm, 1.0)] {
        // Down from the T-pose: the left arm (along +left) turns about
        // `forward` by -90°, the right (along -left) by +90°.
        let world = about_left(rig, -forward_degrees) * Quat::from_axis_angle(forward, down * std::f32::consts::FRAC_PI_2);
        let above = carried(pose, arm.parent().unwrap_or(Bone::Hips));
        pose.set_rotation(arm, (above.inverse() * world).normalize());
    }
}

/// Turns `bone` so its segment, to `child`, points along `direction` in the
/// world, whatever its parents have done (`carried`). `rest` is the rig's
/// joints in its rest pose. For segments no sagittal angle describes: a
/// leg folded to the side.
pub(crate) fn aim(pose: &mut LocalPose, rest: &super::rig::BoneSet<Vec3>, bone: Bone, child: Bone, direction: Vec3) {
    let from = (rest[child] - rest[bone]).normalize();
    let world = Quat::from_rotation_arc(from, direction.normalize());
    let above = bone.parent().map_or(Quat::IDENTITY, |parent| carried(pose, parent));
    pose.set_rotation(bone, (above.inverse() * world).normalize());
}

/// How long a hand is, wrist to fingertip, per length of forearm: Winter's
/// hand 0.108 H over his forearm 0.146 H (§4.0.1).
pub const HAND_PER_FOREARM: f32 = 0.108 / 0.146;

/// Where `hand`'s fingertips are under `pose`: the hand carries on from the
/// wrist along the forearm's rest line, turned by the hand's own world turn.
/// The rigs have no finger joints this module knows.
pub fn fingertip(pose: &LocalPose, rig: &RigGeometry, hand: Bone) -> Vec3 {
    let forearm = hand.parent().expect("a hand hangs from its forearm");
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let at = forward_kinematics_on(pose, rig);
    let line = rest[hand] - rest[forearm];
    at[hand] + carried(pose, hand) * line * HAND_PER_FOREARM
}

/// Which way `hand`'s palm faces under `pose` (world, unit). The rest pose
/// is the bind's T-pose, palms down.
pub fn palm(pose: &LocalPose, rig: &RigGeometry, hand: Bone) -> Vec3 {
    carried(pose, hand) * rest_palm(rig, hand)
}

/// The palm's facing in the rest pose: down, squared to the forearm's line.
fn rest_palm(rig: &RigGeometry, hand: Bone) -> Vec3 {
    let forearm = hand.parent().expect("a hand hangs from its forearm");
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let line = (rest[hand] - rest[forearm]).normalize();
    (Vec3::NEG_Y - line * line.dot(Vec3::NEG_Y)).normalize()
}

/// Lays `hand`'s palm flat, facing down, its fingers along `fingers`
/// (horizontal), as a hand bearing weight on the floor is. The arm above
/// must be straight.
///
/// A hand set on the floor along its straight arm points its fingers
/// straight down through it: 18-21 cm, live, on hands and knees. Turning
/// the hand alone would twist the wrist by the forearm's whole turn, so the
/// turn about the arm's own line (pronation or supination) is shared
/// between the shoulder and the forearm, and the hand only bends back
/// (extends) from there. The wrist joint does not move.
pub(crate) fn palm_flat(pose: &mut LocalPose, rig: &RigGeometry, hand: Bone, fingers: Vec3) {
    let forearm = hand.parent().expect("a hand hangs from its forearm");
    let arm = forearm.parent().expect("a forearm hangs from its arm");
    let fingers = Vec3::new(fingers.x, 0.0, fingers.z).normalize();    let at =forward_kinematics_on(pose, rig);
    let line = (at[hand] - at[forearm]).normalize();
    // The bend back that lays the fingers along `fingers`, and the way the
    // palm must face before it so that it faces down after.
    let extend = Quat::from_rotation_arc(line, fingers);
    let wanted = extend.inverse() * Vec3::NEG_Y;
    // The palm as the hand would carry on straight from the forearm.
    let facing = carried(pose, forearm) * rest_palm(rig, hand);
    let facing = (facing - line * line.dot(facing)).normalize();
    let turn = line.dot(facing.cross(wanted)).atan2(facing.dot(wanted));
    let set_world = |pose: &mut LocalPose, bone: Bone, world: Quat| {
        let above = bone.parent().map_or(Quat::IDENTITY, |parent| carried(pose, parent));
        pose.set_rotation(bone, (above.inverse() * world).normalize());
    };
    // Half the turn at the shoulder, about the arm's own line; the forearm
    // rides it and adds the other half.
    let twist = Quat::from_axis_angle(line, 0.5 * turn);
    let forearm_world = twist * twist * carried(pose, forearm);
    set_world(pose, arm, twist * carried(pose, arm));
    set_world(pose, forearm, forearm_world);
    set_world(pose, hand, extend * forearm_world);
}

/// `a` turned toward `b` by `degrees` (both unit and at right angles).
pub(crate) fn toward(a: Vec3, b: Vec3, degrees: f32) -> Vec3 {
    let r = degrees.to_radians();
    a * r.cos() + b * r.sin()
}

/// A sagittal leg: hip, knee and ankle angles about `left`, degrees.
pub(crate) fn leg(pose: &mut LocalPose, rig: &RigGeometry, [hip, knee, ankle]: [Bone; 3], angles: [f32; 3]) {
    pose.set_rotation(hip, about_left(rig, angles[0]));
    pose.set_rotation(knee, about_left(rig, angles[1]));
    pose.set_rotation(ankle, about_left(rig, angles[2]));
}

pub(crate) const LEFT_LEG: [Bone; 3] = [Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot];
pub(crate) const RIGHT_LEG: [Bone; 3] = [Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot];

/// The value in `range` where `f` crosses zero, by bisection; `f` must
/// change sign across the range, else the nearer end.
pub(crate) fn solve(range: (f32, f32), f: impl Fn(f32) -> f32) -> f32 {
    let (mut low, mut high) = range;
    let (f_low, f_high) = (f(low), f(high));
    if f_low.signum() == f_high.signum() {
        return if f_low.abs() < f_high.abs() { low } else { high };
    }
    for _ in 0..32 {
        let middle = 0.5 * (low + high);
        if f(middle).signum() == f_low.signum() { low = middle } else { high = middle }
    }
    0.5 * (low + high)
}

/// Sitting up, VanSant's first stage from supine: on the seat with the
/// trunk still reclined 40° and propped on both hands behind, knees up and
/// feet flat in front.
///
/// Not upright: with the thighs level the knees are only seat-high, and a
/// shin cannot hang from them to the floor; with the trunk upright the
/// hands hung 27 cm short of it.
pub fn sit(rig: &RigGeometry) -> LocalPose {
    // All about `left`: the pelvis tips back 30°, the spine 10° more.
    let pelvis = -30.0;
    let build = |thigh: f32, arms_back: f32| {
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Hips, about_left(rig, pelvis));
        pose.set_rotation(Bone::Spine, about_left(rig, -10.0));
        pose.set_rotation(Bone::Neck, about_left(rig, 20.0));
        pose.set_rotation(Bone::Head, about_left(rig, 20.0));
        // `thigh` is the thigh's world angle (hip flexion is negative); the
        // shin leans 20° forward from the knee down to a flat foot.
        let hip = thigh - pelvis;
        let shin = -20.0;
        for bones in [LEFT_LEG, RIGHT_LEG] {
            leg(&mut pose, rig, bones, [hip, shin - thigh, -shin]);
        }
        hang_arms(&mut pose, rig, -arms_back);
        pose
    };
    // The thigh raised until the feet are flat on the floor level with the
    // seat, then the arms swung back until the hands are.
    let gap = |pose: &LocalPose, contact: Contact| {
        contact_height(pose, rig, contact) - contact_height(pose, rig, Contact::Joint(Bone::Hips, SEAT_CLEARANCE))
    };
    let thigh = solve((-170.0, -95.0), |t| gap(&build(t, 30.0), Contact::Foot(Bone::LeftFoot)));
    let arms_back = solve((0.0, 80.0), |a| gap(&build(thigh, a), Contact::Joint(Bone::LeftHand, HAND_CLEARANCE)));
    let mut pose = build(thigh, arms_back);
    // Propped behind, the fingers point out and back.
    let forward = rig.forward();
    palm_flat(&mut pose, rig, Bone::LeftHand, toward(rig.left(), -forward, 45.0));
    palm_flat(&mut pose, rig, Bone::RightHand, toward(-rig.left(), -forward, 45.0));
    set_down(&mut pose, rig, &contact_list(Key::Sit));
    pose
}

/// A deep squat, VanSant's last stage before standing: feet flat, knees
/// and hips deeply bent, arms reaching forward, the trunk leaning until the
/// centre of mass is over the feet.
pub fn squat(rig: &RigGeometry) -> LocalPose {
    let build = |lean: f32| {
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Spine, about_left(rig, lean * 0.5));
        pose.set_rotation(Bone::Spine1, about_left(rig, lean * 0.3));
        pose.set_rotation(Bone::Spine2, about_left(rig, lean * 0.2));
        pose.set_rotation(Bone::Head, about_left(rig, -lean * 0.6));
        // Thigh forward and a little up, shin leaning 35° forward over
        // a flat foot.
        let (hip, shin) = (-100.0, 35.0);
        for bones in [LEFT_LEG, RIGHT_LEG] {
            leg(&mut pose, rig, bones, [hip, shin - hip, -shin]);
        }
        hang_arms(&mut pose, rig, 70.0);
        pose
    };
    let forward = rig.forward();
    let off_feet = |pose: &LocalPose| {
        let at = forward_kinematics_on(pose, rig);
        let com = at[Bone::Hips] + super::anthropometry::centre_of_mass(pose, rig);
        let feet = [Bone::LeftFoot, Bone::RightFoot, Bone::LeftToeBase, Bone::RightToeBase].map(|b| at[b].dot(forward));
        com.dot(forward) - feet.iter().sum::<f32>() / 4.0
    };
    let lean = solve((0.0, 80.0), |l| off_feet(&build(l)));
    let mut pose = build(lean);
    set_down(&mut pose, rig, &contact_list(Key::Squat));
    pose
}

/// On hands and knees: thighs straight down to the knees, shins back along
/// the floor, arms straight down to the hands, the trunk pitched until the
/// hands and the knees both reach the floor.
pub fn quadruped(rig: &RigGeometry) -> LocalPose {
    let build = |pitch: f32| {
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Hips, about_left(rig, pitch));
        pose.set_rotation(Bone::Neck, about_left(rig, -(pitch - 20.0) * 0.4));
        pose.set_rotation(Bone::Head, about_left(rig, -(pitch - 20.0) * 0.4));
        for bones in [LEFT_LEG, RIGHT_LEG] {
            // Thigh down (hip = -pitch), shin horizontal back (knee 90°),
            // the foot on its toes.
            leg(&mut pose, rig, bones, [-pitch, 90.0, 40.0]);
        }
        hang_arms(&mut pose, rig, 0.0);
        pose
    };
    let level = |pose: &LocalPose| {
        contact_height(pose, rig, Contact::Joint(Bone::LeftHand, HAND_CLEARANCE))
            - contact_height(pose, rig, Contact::Joint(Bone::LeftLeg, KNEE_CLEARANCE))
    };
    let pitch = solve((50.0, 120.0), |p| level(&build(p)));
    let mut pose = build(pitch);
    for hand in [Bone::LeftHand, Bone::RightHand] {
        palm_flat(&mut pose, rig, hand, rig.forward());
    }
    set_down(&mut pose, rig, &contact_list(Key::Quadruped));
    pose
}

/// Half-kneeling: the right knee on the floor under an upright trunk, the
/// left foot flat in front with its shin vertical, hands resting forward.
pub fn half_kneel(rig: &RigGeometry) -> LocalPose {
    let build = |front_hip: f32| {
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Spine, about_left(rig, 5.0));
        leg(&mut pose, rig, RIGHT_LEG, [0.0, 90.0, 40.0]);
        leg(&mut pose, rig, LEFT_LEG, [front_hip, -front_hip, 0.0]);
        hang_arms(&mut pose, rig, 30.0);
        pose
    };
    let level = |pose: &LocalPose| {
        contact_height(pose, rig, Contact::Foot(Bone::LeftFoot))
            - contact_height(pose, rig, Contact::Joint(Bone::RightLeg, KNEE_CLEARANCE))
    };
    let front_hip = solve((-120.0, -40.0), |h| level(&build(h)));
    let mut pose = build(front_hip);
    set_down(&mut pose, rig, &contact_list(Key::HalfKneel));
    pose
}

/// Side-sitting, pushed up from lying on a side: the seat on the floor, the
/// trunk leaning over the straight arm underneath, its hand on the floor
/// beside the hip, both legs folded to the other side, knees down. The
/// other arm rests forward.
///
/// Authored as segment directions in the world (`aim`), not sagittal
/// angles: the legs fold sideways. `left_down`: lying on the left side.
pub fn side_sit(rig: &RigGeometry, left_down: bool) -> LocalPose {
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let (forward, up) = (rig.forward(), Vec3::Y);
    let under = if left_down { rig.left() } else { -rig.left() };
    let away = -under;
    let sides = |left: [Bone; 3], right: [Bone; 3]| if left_down { (left, right) } else { (right, left) };
    let (arm_under, arm_over) = sides(
        [Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand],
        [Bone::RightArm, Bone::RightForeArm, Bone::RightHand],
    );
    let (leg_under, leg_over) = sides(
        [Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot],
        [Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot],
    );
    let toe = |ankle: Bone| if ankle == Bone::LeftFoot { Bone::LeftToeBase } else { Bone::RightToeBase };
    // `lean`: the trunk over the arm underneath, degrees. The thighs: out
    // forward and to the far side, pitched down until the knee is down.
    let build = |lean: f32, under_pitch: f32, over_pitch: f32| {
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Hips, Quat::from_rotation_arc(up, toward(up, under, lean)));
        // The head kept nearer level than the trunk.
        pose.set_rotation(Bone::Neck, Quat::from_rotation_arc(up, toward(up, under, -0.6 * lean)));
        // The arm underneath straight down to the floor, a little out.
        let prop = toward(-up, under, 15.0);
        aim(&mut pose, &rest, arm_under[0], arm_under[1], prop);
        aim(&mut pose, &rest, arm_under[1], arm_under[2], prop);
        // The other rests down and forward, on the knees.
        let rest_on = toward(-up, forward, 35.0);
        aim(&mut pose, &rest, arm_over[0], arm_over[1], rest_on);
        aim(&mut pose, &rest, arm_over[1], arm_over[2], rest_on);
        // Underneath: thigh forward and a little toward the far side, the
        // shin across in front toward it, the foot on along the floor.
        let thigh = toward(toward(forward, away, 30.0), -up, under_pitch);
        let shin = toward(away, -forward, 20.0);
        aim(&mut pose, &rest, leg_under[0], leg_under[1], thigh);
        aim(&mut pose, &rest, leg_under[1], leg_under[2], shin);
        aim(&mut pose, &rest, leg_under[2], toe(leg_under[2]), shin);
        // On top: thigh out to the far side, the shin folded back beside
        // the seat.
        let thigh = toward(toward(away, forward, 25.0), -up, over_pitch);
        let shin = toward(-forward, away, 15.0);
        aim(&mut pose, &rest, leg_over[0], leg_over[1], thigh);
        aim(&mut pose, &rest, leg_over[1], leg_over[2], shin);
        aim(&mut pose, &rest, leg_over[2], toe(leg_over[2]), shin);
        pose
    };
    // Each knee pitched down level with the seat, then the trunk leaned
    // until the hand is.
    let gap = |pose: &LocalPose, contact: Contact| {
        contact_height(pose, rig, contact) - contact_height(pose, rig, Contact::Joint(Bone::Hips, SEAT_CLEARANCE))
    };
    // Each leaning moves the hips the knees hang from, so a few rounds.
    let (mut lean, mut under_pitch, mut over_pitch) = (15.0, 20.0, 20.0);
    for _ in 0..4 {
        under_pitch = solve((0.0, 60.0), |p| gap(&build(lean, p, over_pitch), Contact::Joint(leg_under[1], KNEE_CLEARANCE)));
        over_pitch = solve((0.0, 60.0), |p| gap(&build(lean, under_pitch, p), Contact::Joint(leg_over[1], KNEE_CLEARANCE)));
        lean = solve((0.0, 50.0), |l| gap(&build(l, under_pitch, over_pitch), Contact::Joint(arm_under[2], HAND_CLEARANCE)));
    }
    let mut pose = build(lean, under_pitch, over_pitch);
    // The propping hand's fingers point out and forward.
    palm_flat(&mut pose, rig, arm_under[2], toward(under, forward, 45.0));
    set_down(&mut pose, rig, &contact_list(Key::SideSit { left_down }));
    pose
}

/// How far a relaxed body's hips flex, degrees ([`relaxed`]): between a
/// fallen body's straight hips and the weightless crew's 31°.
pub const RELAXED_HIP_FLEXION: f32 = 15.0;

/// How far a relaxed body's knees flex, degrees: where Riener & Edrich's
/// passive knee moment is zero at [`RELAXED_HIP_FLEXION`] (14° with the hip
/// straight, 26° at 30°).
pub const RELAXED_KNEE_FLEXION: f32 = 20.0;

/// The pose an unconscious body's joints relax toward: where passive
/// muscle and connective tissue balance, with nothing loading them.
///
/// Mostly the neutral body posture measured in weightlessness, where
/// gravity loads nothing: the six STS-57 crew's medians (Mount et al. 2003,
/// NASA TM-2003-104805, Table 1). Hips out 10°, ankles 20° plantarflexed,
/// waist straight, neck 16° forward, shoulders 45° forward and 30° out,
/// elbows 70°. (NASA's standard posture, from Skylab, bends further.)
///
/// Not its legs. The knee's relaxed angle depends on the hip, two-joint
/// muscles crossing both: Riener & Edrich's passive knee moment is zero at
/// 14° of flexion with the hip straight, 26° at 30° of hip flexion, 43° at
/// 60°. Weightless, the crew's hips flexed 31° and knees 50° together; a
/// fallen body lies with its hips near straight. Relaxed toward those
/// (hips 31°, knees 50°), a body lying on its back drew its knees up
/// against gravity, and they toppled side to side for 8 s without rest.
/// So the legs relax to [`RELAXED_HIP_FLEXION`] and the knee's zero for it,
/// [`RELAXED_KNEE_FLEXION`].
///
/// Authored as segment directions (`aim`), so no joint's sign convention
/// enters.
pub fn relaxed(rig: &RigGeometry) -> LocalPose {
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let (forward, down, left) = (rig.forward(), Vec3::NEG_Y, rig.left());
    let mut pose = LocalPose::REST;
    // 16° of neck flexion, shared by the neck and the head.
    pose.set_rotation(Bone::Neck, about_left(rig, 8.0));
    pose.set_rotation(Bone::Head, about_left(rig, 8.0));
    let toe = |ankle: Bone| if ankle == Bone::LeftFoot { Bone::LeftToeBase } else { Bone::RightToeBase };
    for (side, [hip, knee, ankle]) in [(left, LEFT_LEG), (-left, RIGHT_LEG)] {
        let thigh = toward(toward(down, forward, RELAXED_HIP_FLEXION), side, 10.0);
        // The knee folds the shin back from the thigh, in the leg's plane.
        let back = (-forward - thigh * thigh.dot(-forward)).normalize();
        let shin = toward(thigh, back, RELAXED_KNEE_FLEXION);
        // The foot square to the shin, toes forward, then 20° toward its line.
        let square = (forward - shin * shin.dot(forward)).normalize();
        let foot = toward(square, shin, 20.0);
        aim(&mut pose, &rest, hip, knee, thigh);
        aim(&mut pose, &rest, knee, ankle, shin);
        aim(&mut pose, &rest, ankle, toe(ankle), foot);
    }
    for (side, [shoulder, elbow, wrist]) in
        [(left, [Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand]), (-left, [Bone::RightArm, Bone::RightForeArm, Bone::RightHand])]
    {
        let upper = toward(toward(down, forward, 45.0), side, 30.0);
        // The elbow folds the forearm forward from the upper arm.
        let ahead = (forward - upper * upper.dot(forward)).normalize();
        aim(&mut pose, &rest, shoulder, elbow, upper);
        aim(&mut pose, &rest, elbow, wrist, toward(upper, ahead, 70.0));
    }
    pose
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gltf_rig::{puppet_base, puppet_base_as_rendered};

    fn rigs() -> [(&'static str, RigGeometry); 2] {
        [("puppet_base", puppet_base()), ("as rendered", puppet_base_as_rendered())]
    }

    /// Where a contact meets the floor, horizontally.
    fn footprint(pose: &LocalPose, rig: &RigGeometry, contact: Contact) -> Vec<Vec3> {
        let at = forward_kinematics_on(pose, rig);
        match contact {
            Contact::Joint(bone, _) => vec![at[bone]],
            Contact::Foot(ankle) => Sole::of(rig, ankle).points(pose, rig).iter().map(|p| at[Bone::Hips] + *p).collect(),
        }
    }

    #[test]
    fn every_key_stands_on_all_its_contacts_and_nothing_goes_through_the_floor() {
        for (name, rig) in rigs() {
            for key in Key::ALL {
                let pose = key.pose(&rig);
                for contact in contacts_of(key) {
                    let height = contact_height(&pose, &rig, contact);
                    assert!(
                        (-1.0e-4..0.015).contains(&height),
                        "{name} {key:?}: {contact:?} is {:.1} mm off the floor",
                        height * 1e3
                    );
                }
                let at = forward_kinematics_on(&pose, &rig);
                for &bone in Bone::ALL.iter() {
                    assert!(at[bone].y > -0.01, "{name} {key:?}: {} is {:.0} mm under the floor", bone.name(), -at[bone].y * 1e3);
                }
            }
        }
    }

    #[test]
    fn every_hand_a_key_stands_on_lies_flat_palm_down() {
        // Set down along its straight arm, a hand pointed its fingers 18-21
        // cm into the floor, live. A weight-bearing hand lies flat on its
        // palm, the wrist bent back no further than it goes (about 90°
        // loaded), and no joint of the arm twists by more than half a turn
        // about its line.
        for (name, rig) in rigs() {
            for key in Key::ALL {
                let pose = key.pose(&rig);
                for contact in contacts_of(key) {
                    let Contact::Joint(hand @ (Bone::LeftHand | Bone::RightHand), _) = contact else { continue };
                    let at = forward_kinematics_on(&pose, &rig);
                    let tip = fingertip(&pose, &rig, hand);
                    assert!(tip.y > 0.0, "{name} {key:?}: {} fingertips {:.0} mm under the floor", hand.name(), -tip.y * 1e3);
                    assert!((tip.y - at[hand].y).abs() < 0.01, "{name} {key:?}: {} not level, tip {:.0} mm off the wrist", hand.name(), (tip.y - at[hand].y) * 1e3);
                    let facing = palm(&pose, &rig, hand);
                    assert!(facing.y < -0.99, "{name} {key:?}: {} palm faces {facing}", hand.name());
                    let forearm = hand.parent().unwrap();
                    let line = (at[hand] - at[forearm]).normalize();
                    let bent = line.angle_between(tip - at[hand]).to_degrees();
                    assert!(bent < 95.0, "{name} {key:?}: {} wrist bent back {bent:.0}°", hand.name());
                    // Bent back, not forward: the fingers go the way the
                    // back of the hand faced.
                    let straight = carried(&pose, forearm) * rest_palm(&rig, hand);
                    assert!(straight.dot(tip - at[hand]) < 0.0, "{name} {key:?}: {} wrist bent forward", hand.name());
                }
            }
        }
    }

    #[test]
    fn every_key_holds_its_centre_of_mass_over_what_it_stands_on() {
        // A pose to pass through, not a snapshot of a fall: each could be
        // held still. The COM's ground projection lies inside the box
        // around its contacts (2 cm margin).
        for (name, rig) in rigs() {
            for key in Key::ALL {
                let pose = key.pose(&rig);
                let at = forward_kinematics_on(&pose, &rig);
                let com = at[Bone::Hips] + crate::character::anim::anthropometry::centre_of_mass(&pose, &rig);
                let points: Vec<Vec3> = contacts_of(key).into_iter().flat_map(|c| footprint(&pose, &rig, c)).collect();
                let (lo, hi) = points.iter().fold((Vec3::MAX, Vec3::MIN), |(lo, hi), p| (lo.min(*p), hi.max(*p)));
                let margin = 0.02;
                assert!(
                    com.x > lo.x - margin && com.x < hi.x + margin && com.z > lo.z - margin && com.z < hi.z + margin,
                    "{name} {key:?}: COM at ({:.3}, {:.3}) outside its support x {:.3}..{:.3}, z {:.3}..{:.3}",
                    com.x, com.z, lo.x, hi.x, lo.z, hi.z
                );
            }
        }
    }

    #[test]
    fn every_key_faces_the_way_the_rig_does() {
        // Signed, along the rig's measured forward: an unsigned check is the
        // same for a pose and its mirror image.
        for (name, rig) in rigs() {
            let forward = rig.forward();
            let ahead = |pose: &LocalPose, front: Bone, back: Bone| {
                let at = forward_kinematics_on(pose, &rig);
                (at[front] - at[back]).dot(forward)
            };
            let quadruped = Key::Quadruped.pose(&rig);
            assert!(ahead(&quadruped, Bone::LeftHand, Bone::LeftLeg) > 0.3, "{name}: on hands and knees, the hands should be well ahead of the knees");
            assert!(ahead(&quadruped, Bone::Head, Bone::Hips) > 0.3, "{name}: on hands and knees, the head should be ahead of the hips");
            let squat = Key::Squat.pose(&rig);
            assert!(ahead(&squat, Bone::LeftLeg, Bone::LeftFoot) > 0.1, "{name}: squatting, the knee should be ahead of the ankle");
            let sit = Key::Sit.pose(&rig);
            assert!(ahead(&sit, Bone::LeftFoot, Bone::Hips) > 0.15, "{name}: sitting, the feet should be ahead of the seat");
            assert!(ahead(&sit, Bone::Hips, Bone::LeftHand) > 0.2, "{name}: sitting, the hands should prop it up from behind");
            let kneel = Key::HalfKneel.pose(&rig);
            assert!(ahead(&kneel, Bone::LeftFoot, Bone::RightLeg) > 0.2, "{name}: half-kneeling, the front foot should be ahead of the down knee");
            let at = forward_kinematics_on(&kneel, &rig);
            assert!(at[Bone::Head].y - at[Bone::Hips].y > 0.4, "{name}: half-kneeling, the trunk should be upright");
            for left_down in [true, false] {
                let at = forward_kinematics_on(&Key::SideSit { left_down }.pose(&rig), &rig);
                let under = if left_down { rig.left() } else { -rig.left() };
                let hand = if left_down { Bone::LeftHand } else { Bone::RightHand };
                let side = if left_down { "left" } else { "right" };
                assert!(
                    (at[hand] - at[Bone::Hips]).dot(under) > 0.15,
                    "{name}: side-sitting on the {side}, the hand underneath should prop it out on that side"
                );
                for knee in [Bone::LeftLeg, Bone::RightLeg] {
                    assert!((at[knee] - at[Bone::Hips]).dot(forward) > 0.1, "{name} {side}: the {} should be ahead of the seat", knee.name());
                }
                let feet = (at[Bone::LeftFoot] + at[Bone::RightFoot]) * 0.5;
                assert!((feet - at[Bone::Hips]).dot(under) < -0.1, "{name} {side}: the legs should fold to the other side");
                assert!(at[Bone::Head].y - at[Bone::Hips].y > 0.4, "{name} {side}: the trunk should be up");
            }
        }
    }

    #[test]
    fn the_two_side_sits_mirror_each_other_exactly() {
        use crate::character::anim::convert::{mirror_bone, mirrored};
        use crate::character::anim::math::quat_ext::neighborhood;
        let rig = puppet_base();
        assert!(rig.left().x.abs() > 0.999, "puppet_base's left is {}", rig.left());
        let flipped = mirrored(&Key::SideSit { left_down: true }.pose(&rig));
        let right = Key::SideSit { left_down: false }.pose(&rig);
        for &bone in Bone::ALL.iter() {
            let mirror = neighborhood(right.rotation(bone), flipped.rotation(bone));
            assert!(
                right.rotation(bone).abs_diff_eq(mirror, 1.0e-3),
                "{} on the right does not mirror {} on the left",
                bone.name(),
                mirror_bone(bone).name()
            );
        }
    }

    #[test]
    fn chained_keys_keep_their_shared_contacts_in_place() {
        // What stays planted from one key to the next does not slide: the
        // front foot from half-kneeling to standing, the feet from squatting
        // to standing and from sitting to squatting, the knee from hands and
        // knees to half-kneeling. Measured along the rig's forward.
        for (name, rig) in rigs() {
            let forward = rig.forward();
            let standing = forward_kinematics_on(&LocalPose::REST, &rig);
            for (lying, first_on, second_on) in
                [(Lying::FaceUp, Bone::LeftFoot, Bone::LeftFoot), (Lying::FaceDown, Bone::RightLeg, Bone::LeftFoot)]
            {
                let chain = keys(lying, &rig);
                let (first, second) = (forward_kinematics_on(&chain[0].pose, &rig), forward_kinematics_on(&chain[1].pose, &rig));
                let slid = (second[second_on] - standing[Bone::LeftFoot]).dot(forward).abs();
                assert!(slid < 1.0e-3, "{name} {lying:?}: {} slides {:.0} mm up to standing", second_on.name(), slid * 1e3);
                let slid = (first[first_on] - second[first_on]).dot(forward).abs();
                assert!(slid < 1.0e-3, "{name} {lying:?}: {} slides {:.0} mm between the keys", first_on.name(), slid * 1e3);
            }
            // On the side: the propping hand stays as the body goes over
            // onto hands and knees; the rest is the face-down route.
            for (left_down, hand) in [(true, Bone::LeftHand), (false, Bone::RightHand)] {
                let chain = keys(Lying::Side { left_down }, &rig);
                assert_eq!(chain.len(), 3);
                assert_eq!(chain[1..], keys(Lying::FaceDown, &rig)[..], "{name}: the side route should go on as face down");
                let (sat, quadruped) = (forward_kinematics_on(&chain[0].pose, &rig), forward_kinematics_on(&chain[1].pose, &rig));
                let slid = (sat[hand] - quadruped[hand]).dot(forward).abs();
                assert!(slid < 1.0e-3, "{name}: the {} slides {:.0} mm off the side-sit", hand.name(), slid * 1e3);
            }
        }
    }

    #[test]
    fn a_body_is_read_as_lying_on_its_side_only_when_its_chest_faces_sideways() {
        let (up, side) = (Vec3::Y, Vec3::X);
        let tilted = |degrees: f32| Quat::from_rotation_z(degrees.to_radians());
        // Chest forward rolled from straight up (face up) round to straight
        // down; its left goes with it.
        let read = |degrees: f32| Lying::of(tilted(degrees) * up, tilted(degrees) * side);
        assert_eq!(read(0.0), Lying::FaceUp);
        assert_eq!(read(40.0), Lying::FaceUp);
        assert_eq!(read(-40.0), Lying::FaceUp);
        // Rolled 90° about +Z: the chest faces -X, its left points up, so
        // it lies on its right side; the other way, on its left.
        assert_eq!(read(90.0), Lying::Side { left_down: false });
        assert_eq!(read(-90.0), Lying::Side { left_down: true });
        assert_eq!(read(180.0), Lying::FaceDown);
        assert_eq!(read(140.0), Lying::FaceDown);
    }

    #[test]
    #[ignore]
    fn probe_key_geometry() {
        let rig = puppet_base();
        for key in Key::ALL {
            let pose = key.pose(&rig);
            let at = forward_kinematics_on(&pose, &rig);
            let f = rig.forward();
            let show = |b: Bone| format!("{} f{:+.2} y{:.2}", b.name(), at[b].dot(f), at[b].y);
            println!("{key:?}: {}", [Bone::Hips, Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase, Bone::LeftHand, Bone::Head].map(show).join(" | "));
        }
    }

    #[test]
    fn the_relaxed_pose_has_the_neutral_body_postures_angles() {
        // STS-57's medians: elbow 70°, hip out 10°, shoulder 45° forward and
        // 30° out, ankle 20° plantarflexed; the hips and knees Riener and
        // Edrich's (`RELAXED_HIP_FLEXION`, `RELAXED_KNEE_FLEXION`). Signed
        // in the rig's own frame (forward, left), on both rigs.
        for (name, rig) in rigs() {
            let pose = relaxed(&rig);
            let at = forward_kinematics_on(&pose, &rig);
            let (forward, left) = (rig.forward(), rig.left());
            let angle = |a: Vec3, b: Vec3| a.angle_between(b).to_degrees();
            let segment = |from: Bone, to: Bone| (at[to] - at[from]).normalize();
            for (side, [hip, knee, ankle, toe], [shoulder, elbow, wrist]) in [
                (left, [Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase], [Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand]),
                (-left, [Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase], [Bone::RightArm, Bone::RightForeArm, Bone::RightHand]),
            ] {
                let (thigh, shin, foot) = (segment(hip, knee), segment(knee, ankle), segment(ankle, toe));
                let (upper, forearm) = (segment(shoulder, elbow), segment(elbow, wrist));
                let checks = [
                    ("knee flexion", angle(thigh, shin), RELAXED_KNEE_FLEXION),
                    ("hip flexion", thigh.dot(forward).atan2(-thigh.y).to_degrees(), RELAXED_HIP_FLEXION),
                    ("hip abduction", thigh.dot(side).asin().to_degrees(), 10.0),
                    // Between the shin and the foot: square is 90°,
                    // plantarflexed 20° is 110°.
                    ("ankle", 180.0 - angle(shin, foot), 110.0),
                    ("elbow flexion", angle(upper, forearm), 70.0),
                    ("shoulder abduction", upper.dot(side).asin().to_degrees(), 30.0),
                ];
                for (what, got, want) in checks {
                    assert!((got - want).abs() < 1.5, "{name}: {what} {got:.1}°, want {want}°");
                }
                // Each limb bends the right way: the shin back, the forearm
                // forward, the foot's toes ahead of the ankle.
                assert!(shin.dot(forward) < thigh.dot(forward), "{name}: the knee bends forward");
                assert!(forearm.dot(forward) > upper.dot(forward), "{name}: the elbow bends backward");
                assert!(foot.dot(forward) > 0.0, "{name}: the toes point back");
            }
        }
    }

    #[test]
    fn every_symmetric_key_mirrors_left_and_right_exactly() {
        use crate::character::anim::convert::{mirror_bone, mirrored};
        use crate::character::anim::math::quat_ext::neighborhood;
        let rig = puppet_base();
        // The mirror flips X; valid only where the rig's left is along X.
        assert!(rig.left().x.abs() > 0.999, "puppet_base's left is {}", rig.left());
        for key in Key::ALL.into_iter().filter(|k| k.symmetric()) {
            let pose = key.pose(&rig);
            let flipped = mirrored(&pose);
            for &bone in Bone::ALL.iter() {
                let original = pose.rotation(bone);
                let mirror = neighborhood(original, flipped.rotation(bone));
                assert!(
                    original.abs_diff_eq(mirror, 1.0e-4),
                    "{key:?}: {} does not mirror {}: {original:?} vs {mirror:?}",
                    bone.name(),
                    mirror_bone(bone).name()
                );
            }
        }
    }
}
