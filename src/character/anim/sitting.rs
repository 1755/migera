//! Sitting, on a chair or on the floor, and sitting down and standing up.
//!
//! Each sitting pose is solved on the rig, as the get-up's keys are
//! (`getup`): its contacts meet the seat or the floor together and nothing
//! goes under the floor. Sitting down and standing up go through key poses,
//! chained so that a contact two keys share stays where it is (the feet on
//! a chair, the seat on the floor), and blended per bone in the world
//! (`rig::blend_in_world`).
//!
//! # On a chair
//!
//! Standing up from a chair (Schenkman et al. 1990, *Phys Ther* 70:638)
//! goes in four phases: the trunk leans forward (flexion momentum), the
//! seat is left as the momentum carries the body over the feet (momentum
//! transfer), the hips and knees extend, and the body steadies. Young
//! adults rising from a standard chair took 1.9 s, the first three phases
//! 28 %, 18 % and 54 % of it, the knees ~75° flexed at seat-off, the trunk
//! travelling ~0.5 m forward (Marsh et al., Wake Forest, rising from two
//! chair types). Sitting down is the same path the other way, a little
//! slower, ~2.0 s (stand-to-sit in healthy adults).
//!
//! The feet stay where the character stood: a person stands in front of a
//! chair and sits back onto it. So the chair goes where the seated hips
//! land ([`seat_offset`]).
//!
//! # On the floor
//!
//! Down through a squat to sitting propped on the hands (the get-up's
//! face-up keys the other way), or for kneeling through a half-kneel; up
//! by the get-up's own routes (`getup::keys`).

use bevy::math::{Vec2, Vec3};

use super::getup::{
    self, about_left, aim, hang_arms, palm_flat, placed, set_down, solve, toward, Contact, HAND_CLEARANCE, KNEE_CLEARANCE,
    LEFT_LEG, RIGHT_LEG, SEAT_CLEARANCE,
};
use super::rig::{forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// A standard chair's seat height, metres (0.43-0.48 m).
pub const CHAIR_HEIGHT: f32 = 0.45;

/// A chair's seat as a sitting pose meets it: its height, and how much
/// further back from the feet than sitting upright puts them the hips go,
/// metres. A walk to a chair stops a few centimetres off its spot; the
/// shins swing forward or back to make that up (`approach`), as a person
/// sits with their feet a little further out or tucked in.
///
/// Across the chair the same: [`Seat::across`], how much further left of the
/// feet (the character's left), the hips go, the shins slanting sideways.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seat {
    pub height: f32,
    pub back: f32,
    pub across: f32,
}

/// The furthest a seat is moved back or forward, metres: a shin swung
/// ±20° from upright.
pub const SEAT_BACK_RANGE: f32 = 0.15;
/// The furthest it is moved across, metres: the shins slanting ±11°.
pub const SEAT_ACROSS_RANGE: f32 = 0.08;

impl Seat {
    /// A seat `height` high, the feet where sitting upright has them.
    pub fn at(height: f32) -> Self {
        Self { height, back: 0.0, across: 0.0 }
    }

    /// How much further forward, degrees, the shins swing to put the hips
    /// [`Seat::back`] further back over the feet.
    fn shin(&self, rig: &RigGeometry) -> f32 {
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let length = rest[Bone::LeftLeg].distance(rest[Bone::LeftFoot]);
        let back = self.back.clamp(-SEAT_BACK_RANGE, SEAT_BACK_RANGE);
        -(back / length).clamp(-0.9, 0.9).asin().to_degrees()
    }
}

/// How a body sits on a chair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChairPose {
    /// Upright, thighs level, shins down to flat feet, hands on the thighs.
    Upright,
    /// Leaning back against the backrest, the legs out in front.
    Reclined,
    /// The right knee crossed over the left, its foot hanging.
    LegsCrossed,
    /// Leaning forward, elbows on the knees, hands together in front.
    LeaningForward,
}

/// How a body sits on the floor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FloorPose {
    /// Cross-legged, each shin across in front, hands on the knees.
    CrossLegged,
    /// The legs out in front, leaning back on the hands behind.
    Propped,
    /// Knees up, feet flat, arms around the shins.
    HugKnees,
    /// On the left hip, the legs folded to the right, propped on the left
    /// hand (the get-up's side-sit).
    SideSit,
    /// Kneeling, sitting back on the heels, hands on the thighs (seiza).
    Kneeling,
}

/// A way of sitting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sitting {
    Chair(ChairPose),
    Floor(FloorPose),
}

impl Sitting {
    /// Every way of sitting.
    pub const ALL: [Sitting; 9] = [
        Sitting::Chair(ChairPose::Upright),
        Sitting::Chair(ChairPose::Reclined),
        Sitting::Chair(ChairPose::LegsCrossed),
        Sitting::Chair(ChairPose::LeaningForward),
        Sitting::Floor(FloorPose::CrossLegged),
        Sitting::Floor(FloorPose::Propped),
        Sitting::Floor(FloorPose::HugKnees),
        Sitting::Floor(FloorPose::SideSit),
        Sitting::Floor(FloorPose::Kneeling),
    ];

    /// Its name, `chair:upright` … `floor:kneeling`.
    pub fn name(self) -> &'static str {
        match self {
            Sitting::Chair(ChairPose::Upright) => "chair:upright",
            Sitting::Chair(ChairPose::Reclined) => "chair:reclined",
            Sitting::Chair(ChairPose::LegsCrossed) => "chair:crossed",
            Sitting::Chair(ChairPose::LeaningForward) => "chair:forward",
            Sitting::Floor(FloorPose::CrossLegged) => "floor:cross_legged",
            Sitting::Floor(FloorPose::Propped) => "floor:propped",
            Sitting::Floor(FloorPose::HugKnees) => "floor:hug",
            Sitting::Floor(FloorPose::SideSit) => "floor:side",
            Sitting::Floor(FloorPose::Kneeling) => "floor:kneeling",
        }
    }

    /// The way of sitting named `name` ([`Sitting::name`]).
    pub fn by_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|s| s.name() == name)
    }

    /// Whether it sits on a chair.
    pub fn on_chair(self) -> bool {
        matches!(self, Sitting::Chair(_))
    }
}

/// One pose on the way down or up, and how long the move into it takes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitKey {
    /// The pose, in the character's frame, on a floor at `y = 0`.
    pub pose: LocalPose,
    /// How long the blend into it from the previous pose takes, seconds.
    pub seconds: f32,
}

/// How long standing up from the last key takes, seconds: a chair rise's
/// extension and steadying (54 % of 1.9 s), a floor rise's last step.
pub fn stand_seconds(sitting: Sitting) -> f32 {
    if sitting.on_chair() { 1.0 } else { getup::STAND_SECONDS }
}

// ---------------------------------------------------------------------------
// Building blocks
// ---------------------------------------------------------------------------

/// The pelvis tipped `pelvis` degrees about `left` (positive forward, its
/// top over the thighs), the trunk leaning `lean` from vertical (positive
/// forward), shared down the spine, and the head held `head` from vertical.
fn trunk(pose: &mut LocalPose, rig: &RigGeometry, pelvis: f32, lean: f32, head: f32) {
    let spine = lean - pelvis;
    pose.set_rotation(Bone::Hips, about_left(rig, pelvis));
    pose.set_rotation(Bone::Spine, about_left(rig, spine * 0.5));
    pose.set_rotation(Bone::Spine1, about_left(rig, spine * 0.3));
    pose.set_rotation(Bone::Spine2, about_left(rig, spine * 0.2));
    let neck = head - lean;
    pose.set_rotation(Bone::Neck, about_left(rig, neck * 0.5));
    pose.set_rotation(Bone::Head, about_left(rig, neck * 0.5));
}

/// A sagittal leg by its segments' world angles about `left`, degrees from
/// hanging straight down (negative swings forward): the thigh, the shin,
/// and the foot (0 flat).
fn leg(pose: &mut LocalPose, rig: &RigGeometry, bones: [Bone; 3], pelvis: f32, [thigh, shin, foot]: [f32; 3]) {
    getup::leg(pose, rig, bones, [thigh - pelvis, shin - thigh, foot - shin]);
}

/// Both legs, alike.
fn legs(pose: &mut LocalPose, rig: &RigGeometry, pelvis: f32, angles: [f32; 3]) {
    for bones in [LEFT_LEG, RIGHT_LEG] {
        leg(pose, rig, bones, pelvis, angles);
    }
}

/// The arms' bones, left then right.
const ARMS: [[Bone; 3]; 2] = [[Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand], [Bone::RightArm, Bone::RightForeArm, Bone::RightHand]];

/// Puts `wrist`'s joint at `target` (the pose's frame), the elbow bending
/// toward `pole`: two segments, lengths kept (`aim`). Out of reach, the arm
/// points at it, straight.
fn reach(pose: &mut LocalPose, rig: &RigGeometry, [shoulder, elbow, wrist]: [Bone; 3], target: Vec3, pole: Vec3) {
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let at = forward_kinematics_on(pose, rig);
    let (start, upper, fore) = (at[shoulder], (at[elbow] - at[shoulder]).length(), (at[wrist] - at[elbow]).length());
    let to = target - start;
    let distance = to.length().clamp(1.0e-4, (upper + fore) * 0.999);
    let direction = to.normalize_or_zero();
    let cos = ((upper * upper + distance * distance - fore * fore) / (2.0 * upper * distance)).clamp(-1.0, 1.0);
    let bend = (pole - direction * direction.dot(pole)).normalize_or_zero();
    let elbow_at = start + (direction * cos + bend * (1.0 - cos * cos).sqrt()) * upper;
    aim(pose, &rest, shoulder, elbow, elbow_at - start);
    aim(pose, &rest, elbow, wrist, start + direction * distance - elbow_at);
}

/// The pose's joints.
fn joints(pose: &LocalPose, rig: &RigGeometry) -> BoneSet<Vec3> {
    forward_kinematics_on(pose, rig)
}

/// Each hand on its thigh, `along` of the way from hip to knee, the elbows
/// back and out.
fn hands_on_thighs(pose: &mut LocalPose, rig: &RigGeometry, along: f32) {
    let (up, left, forward) = (Vec3::Y, rig.left(), rig.forward());
    for (arm, (hip, knee), side) in [
        (ARMS[0], (Bone::LeftUpLeg, Bone::LeftLeg), left),
        (ARMS[1], (Bone::RightUpLeg, Bone::RightLeg), -left),
    ] {
        let at = joints(pose, rig);
        let target = at[hip].lerp(at[knee], along) + up * 0.08 + side * 0.02;
        reach(pose, rig, arm, target, side - forward * 0.5);
    }
}

/// Where a chair's seated hips stand, from where they stood: (forward,
/// left) metres in the character's frame, and the seat's height. Its seat
/// goes there, under the hips.
pub fn seat_offset(rig: &RigGeometry, stood: &LocalPose, chair: f32) -> (Vec2, f32) {
    let upright = chair_pose(ChairPose::Upright, rig, stood, Seat::at(chair));
    let off = joints(&upright, rig)[Bone::Hips] - joints(stood, rig)[Bone::Hips];
    (Vec2::new(off.dot(rig.forward()), off.dot(rig.left())), chair)
}

// ---------------------------------------------------------------------------
// The chair
// ---------------------------------------------------------------------------

/// The seat's contact: the hips joint, its clearance above the seat.
const SEAT: Contact = Contact::Joint(Bone::Hips, SEAT_CLEARANCE);

/// How high the seat contact is above the floor the feet stand on.
fn seat_height(pose: &LocalPose, rig: &RigGeometry) -> f32 {
    getup::contact_height(pose, rig, SEAT) - getup::contact_height(pose, rig, Contact::Foot(Bone::LeftFoot)).min(getup::contact_height(pose, rig, Contact::Foot(Bone::RightFoot)))
}

/// A seated chair pose: the trunk set, both legs with their shins at
/// `shin` (swung further for the seat's [`Seat::back`]), the thighs solved
/// so the seat meets `chair` with the feet flat on the floor, set down on
/// the feet.
fn on_chair(rig: &RigGeometry, pelvis: f32, lean: f32, head: f32, shin: f32, chair: Seat) -> LocalPose {
    let shin = shin + chair.shin(rig);
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let build = |thigh: f32| {
        let mut pose = LocalPose::REST;
        trunk(&mut pose, rig, pelvis, lean, head);
        legs(&mut pose, rig, pelvis, [thigh, shin, 0.0]);
        // Across: both ankles moved the other way under the hips, the
        // knees still forward, the feet kept flat and pointing as they were.
        let across = chair.across.clamp(-SEAT_ACROSS_RANGE, SEAT_ACROSS_RANGE);
        if across != 0.0 {
            for [hip, knee, ankle, toe] in [
                [Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase],
                [Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase],
            ] {
                let at = joints(&pose, rig);
                let foot = at[toe] - at[ankle];
                reach(&mut pose, rig, [hip, knee, ankle], at[ankle] - rig.left() * across, rig.forward());
                aim(&mut pose, &rest, ankle, toe, foot);
            }
        }
        hang_arms(&mut pose, rig, 10.0);
        pose
    };
    let thigh = solve((-125.0, -55.0), |t| seat_height(&build(t), rig) - chair.height);
    let mut pose = build(thigh);
    set_down(&mut pose, rig, &[Contact::Foot(Bone::LeftFoot), Contact::Foot(Bone::RightFoot)]);
    pose
}

/// `pose` moved over the floor until `bone` is over `at`, both ways: a
/// chair's seat across from the feet ([`Seat::across`]) placed along the
/// forward only (`getup::placed`) sat on its feet's middle again.
fn over(mut pose: LocalPose, rig: &RigGeometry, bone: Bone, at: Vec3) -> LocalPose {
    let off = joints(&pose, rig)[bone] - at;
    pose.root_translation -= Vec3::new(off.x, 0.0, off.z);
    pose
}

/// `pose` moved over the floor until its feet's middle is `stood`'s.
fn on_feet(pose: LocalPose, rig: &RigGeometry, stood: &LocalPose) -> LocalPose {
    let middle = |pose: &LocalPose| {
        let at = joints(pose, rig);
        (at[Bone::LeftFoot] + at[Bone::RightFoot]) * 0.5
    };
    let target = middle(stood);
    let now = middle(&pose);
    let mut pose = pose;
    pose.root_translation -= Vec3::new(now.x - target.x, 0.0, now.z - target.z);
    pose
}

/// A seated pose on a chair `chair` high, placed: feet where `stood` has
/// them, or for one whose legs move, its seat where the upright one's is.
pub fn chair_pose(how: ChairPose, rig: &RigGeometry, stood: &LocalPose, chair: Seat) -> LocalPose {
    let upright = || {
        // A slight backward tilt of the pelvis, the trunk upright, the
        // shins a little forward of the knees.
        let mut pose = on_chair(rig, -8.0, 0.0, 0.0, -5.0, chair);
        hands_on_thighs(&mut pose, rig, 0.6);
        on_feet(pose, rig, stood)
    };
    match how {
        ChairPose::Upright => upright(),
        ChairPose::Reclined => {
            // Against the backrest: the pelvis tipped back, the trunk 18°
            // behind vertical, the head kept level, the legs out.
            let mut pose = on_chair(rig, -22.0, -18.0, 0.0, -30.0, chair);
            hands_on_thighs(&mut pose, rig, 0.35);
            let seat = joints(&upright(), rig)[Bone::Hips];
            over(pose, rig, Bone::Hips, seat)
        }
        ChairPose::LeaningForward => {
            // The trunk 35° forward, the head raised to look ahead, the
            // elbows on the knees, the hands together in front of them.
            let mut pose = on_chair(rig, 10.0, 35.0, 5.0, -5.0, chair);
            let (forward, left, up) = (rig.forward(), rig.left(), Vec3::Y);
            let at = joints(&pose, rig);
            let between = (at[Bone::LeftLeg] + at[Bone::RightLeg]) * 0.5;
            for (arm, side) in [(ARMS[0], left), (ARMS[1], -left)] {
                let target = between + forward * 0.12 + up * 0.02 + side * 0.03;
                reach(&mut pose, rig, arm, target, -up + side * 0.5);
            }
            on_feet(pose, rig, stood)
        }
        ChairPose::LegsCrossed => {
            // The right knee over the left, its shin hanging down across
            // the left one, the foot relaxed; the right hand on the top knee.
            let mut pose = upright();
            let rest = forward_kinematics_on(&LocalPose::REST, rig);
            let (forward, left, up) = (rig.forward(), rig.left(), Vec3::Y);
            let at = joints(&pose, rig);
            let knee_over = at[Bone::LeftLeg] + up * 0.13 + forward * 0.04;
            aim(&mut pose, &rest, Bone::RightUpLeg, Bone::RightLeg, knee_over - at[Bone::RightUpLeg]);
            let shin = (-up * 0.9 + left * 0.25 + forward * 0.2).normalize();
            aim(&mut pose, &rest, Bone::RightLeg, Bone::RightFoot, shin);
            aim(&mut pose, &rest, Bone::RightFoot, Bone::RightToeBase, toward(forward, -up, 35.0));
            let at = joints(&pose, rig);
            reach(&mut pose, rig, ARMS[1], at[Bone::RightLeg] + up * 0.07, -left - forward * 0.5);
            pose
        }
    }
}

/// The keys from standing down onto a chair `chair` high, ending seated
/// `how`. The feet stay where `stood` has them throughout.
pub fn chair_down(how: ChairPose, rig: &RigGeometry, stood: &LocalPose, chair: Seat) -> Vec<SitKey> {
    // Half-way down: hips back, knees ~60°, the trunk leaning 35° to keep
    // the weight over the feet.
    let lowering = {
        let mut pose = LocalPose::REST;
        trunk(&mut pose, rig, 15.0, 35.0, 15.0);
        legs(&mut pose, rig, 15.0, [-45.0, 15.0, 0.0]);
        hang_arms(&mut pose, rig, 25.0);
        set_down(&mut pose, rig, &[Contact::Foot(Bone::LeftFoot), Contact::Foot(Bone::RightFoot)]);
        on_feet(pose, rig, stood)
    };
    // Touching down, still leaning forward, the legs as they will sit.
    let touching = {
        let mut pose = on_chair(rig, 5.0, 30.0, 15.0, -5.0, chair);
        hands_on_thighs(&mut pose, rig, 0.7);
        on_feet(pose, rig, stood)
    };
    let mut keys = vec![SitKey { pose: lowering, seconds: 0.9 }, SitKey { pose: touching, seconds: 0.6 }];
    let upright = chair_pose(ChairPose::Upright, rig, stood, chair);
    match how {
        ChairPose::Upright => keys.push(SitKey { pose: upright, seconds: 0.6 }),
        // Crossing a leg starts from sitting upright.
        ChairPose::LegsCrossed => {
            keys.push(SitKey { pose: upright, seconds: 0.5 });
            keys.push(SitKey { pose: chair_pose(how, rig, stood, chair), seconds: 0.8 });
        }
        _ => keys.push(SitKey { pose: chair_pose(how, rig, stood, chair), seconds: 0.8 }),
    }
    keys
}

/// The keys from seated `how` up to just before standing (the walker blends
/// the last key into its standing pose over [`stand_seconds`]): the trunk
/// leaning forward over the feet, then the seat left. 0.55 s and 0.35 s,
/// with the extension's 1.0 s: Marsh's 28/18/54 % of 1.9 s.
pub fn chair_up(how: ChairPose, rig: &RigGeometry, stood: &LocalPose, chair: Seat) -> Vec<SitKey> {
    let mut keys = Vec::new();
    // Uncrossed, or the legs drawn back under the seat, first.
    match how {
        ChairPose::Upright | ChairPose::LeaningForward => {}
        _ => keys.push(SitKey { pose: chair_pose(ChairPose::Upright, rig, stood, chair), seconds: 0.6 }),
    }
    let leaning = {
        let mut pose = on_chair(rig, 12.0, 42.0, 20.0, -5.0, chair);
        hang_arms(&mut pose, rig, 35.0);
        on_feet(pose, rig, stood)
    };
    // Seat-off: the hips 3 cm off the seat, the trunk at its furthest
    // forward, the knees ~75°.
    let seat_off = {
        let mut pose = on_chair(rig, 15.0, 45.0, 20.0, 5.0, Seat { height: chair.height + 0.03, ..chair });
        hang_arms(&mut pose, rig, 35.0);
        on_feet(pose, rig, stood)
    };
    keys.push(SitKey { pose: leaning, seconds: 0.55 });
    keys.push(SitKey { pose: seat_off, seconds: 0.35 });
    keys
}

// ---------------------------------------------------------------------------
// The floor
// ---------------------------------------------------------------------------

/// Kneeling legs: the thighs `thigh` and the shins `shin` (world angles
/// about `left`), set down on the knees, each foot's toes tucked under,
/// heel up, as the get-up's half-kneel holds its back foot, at the angle
/// that just puts them on the floor. At a fixed 130° the toes went 44 mm
/// into it.
fn kneeling_legs(pose: &mut LocalPose, rig: &RigGeometry, thigh: f32, shin: f32) {
    let knees = [Contact::Joint(Bone::LeftLeg, KNEE_CLEARANCE), Contact::Joint(Bone::RightLeg, KNEE_CLEARANCE)];
    let with = |foot: f32| {
        let mut posed = *pose;
        legs(&mut posed, rig, 0.0, [thigh, shin, foot]);
        set_down(&mut posed, rig, &knees);
        posed
    };
    let sole = |posed: &LocalPose| getup::contact_height(posed, rig, Contact::Foot(Bone::LeftFoot)).min(getup::contact_height(posed, rig, Contact::Foot(Bone::RightFoot)));
    *pose = with(solve((95.0, 178.0), |foot| sole(&with(foot))));
}

/// Kneeling up on both knees, thighs straight down, shins back along the
/// floor, toes tucked: the step between a half-kneel and sitting back on
/// the heels.
fn tall_kneel(rig: &RigGeometry) -> LocalPose {
    let mut pose = LocalPose::REST;
    trunk(&mut pose, rig, 0.0, 0.0, 0.0);
    hang_arms(&mut pose, rig, 5.0);
    kneeling_legs(&mut pose, rig, 0.0, 88.0);
    pose
}

/// `pose` with each foot whose sole goes under the floor tipped toes-up
/// just far enough that it does not. The get-up's side-sit folds the legs
/// with the feet on along the floor, a sole 42 mm into it: a key a rise
/// passes through, but not a pose to hold.
fn feet_on_floor(mut pose: LocalPose, rig: &RigGeometry) -> LocalPose {
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    for (ankle, toe) in [(Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightFoot, Bone::RightToeBase)] {
        let sole = |pose: &LocalPose| getup::contact_height(pose, rig, Contact::Foot(ankle));
        if sole(&pose) >= 0.0 {
            continue;
        }
        let at = joints(&pose, rig);
        let along = at[toe] - at[ankle];
        let level = Vec3::new(along.x, 0.0, along.z).normalize_or_zero();
        let tipped = |pitch: f32| {
            let mut tipped = pose;
            aim(&mut tipped, &rest, ankle, toe, toward(level, Vec3::Y, pitch));
            tipped
        };
        pose = tipped(solve((-30.0, 80.0), |pitch| sole(&tipped(pitch))));
    }
    pose
}

/// A seated floor pose, set down so its lowest contact touches the floor.
fn floor_pose(how: FloorPose, rig: &RigGeometry) -> LocalPose {
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let (forward, left, up, down) = (rig.forward(), rig.left(), Vec3::Y, Vec3::NEG_Y);
    let mut pose: LocalPose;
    match how {
        FloorPose::CrossLegged => {
            // Each thigh forward and 55° out; each shin back across in front
            // toward the other side, the right one in front, the foot along
            // the floor on its outer edge under the other knee. The thighs
            // tilted (`tilt`, degrees up) until the feet are level with the
            // seat, so both rest on the floor and the knees ride above it.
            let build = |tilt: f32| {
                let mut pose = LocalPose::REST;
                trunk(&mut pose, rig, 5.0, 10.0, 0.0);
                for (side, [hip, knee, ankle], ahead) in [(left, LEFT_LEG, 0.3), (-left, RIGHT_LEG, 0.45)] {
                    let toe = if ankle == Bone::LeftFoot { Bone::LeftToeBase } else { Bone::RightToeBase };
                    let thigh = toward(toward(forward, side, 55.0), up, tilt);
                    let shin = (-side * 0.9 + forward * ahead + down * 0.25).normalize();
                    aim(&mut pose, &rest, hip, knee, thigh);
                    aim(&mut pose, &rest, knee, ankle, shin);
                    aim(&mut pose, &rest, ankle, toe, (shin + forward * 0.6).normalize());
                }
                pose
            };
            let feet_above_seat = |pose: &LocalPose| {
                let sole = |foot| getup::contact_height(pose, rig, Contact::Foot(foot));
                sole(Bone::LeftFoot).min(sole(Bone::RightFoot)) - getup::contact_height(pose, rig, SEAT)
            };
            pose = build(solve((-20.0, 40.0), |t| feet_above_seat(&build(t))));
            let contacts = [
                SEAT,
                Contact::Joint(Bone::LeftLeg, KNEE_CLEARANCE),
                Contact::Joint(Bone::RightLeg, KNEE_CLEARANCE),
                Contact::Foot(Bone::LeftFoot),
                Contact::Foot(Bone::RightFoot),
            ];
            set_down(&mut pose, rig, &contacts);
            for (arm, knee, side) in [(ARMS[0], Bone::LeftLeg, left), (ARMS[1], Bone::RightLeg, -left)] {
                let at = joints(&pose, rig);
                reach(&mut pose, rig, arm, at[knee] + up * 0.07, side - forward * 0.3);
            }
        }
        FloorPose::Propped => {
            // The legs out along the floor, knees soft, feet relaxed; the
            // trunk 25° back over the hands, propped behind.
            // The thigh pitched until the heels rest level with the seat,
            // the knees 6° soft, the feet relaxed, toes up and a little
            // forward (a world angle of −65°: up is −90).
            let legs_out = |pose: &mut LocalPose, thigh: f32| legs(pose, rig, -15.0, [thigh, thigh + 6.0, -65.0]);
            let heels_on_floor = |thigh: f32| {
                let mut pose = LocalPose::REST;
                trunk(&mut pose, rig, -15.0, -25.0, -10.0);
                legs_out(&mut pose, thigh);
                let sole = |foot| getup::contact_height(&pose, rig, Contact::Foot(foot));
                sole(Bone::LeftFoot).min(sole(Bone::RightFoot)) - getup::contact_height(&pose, rig, SEAT)
            };
            let thigh = solve((-105.0, -70.0), heels_on_floor);
            let build = |arms_back: f32| {
                let mut pose = LocalPose::REST;
                trunk(&mut pose, rig, -15.0, -25.0, -10.0);
                legs_out(&mut pose, thigh);
                hang_arms(&mut pose, rig, -arms_back);
                pose
            };
            let gap = |pose: &LocalPose| {
                getup::contact_height(pose, rig, Contact::Joint(Bone::LeftHand, HAND_CLEARANCE)) - getup::contact_height(pose, rig, SEAT)
            };
            let arms_back = solve((0.0, 70.0), |a| gap(&build(a)));
            pose = build(arms_back);
            palm_flat(&mut pose, rig, Bone::LeftHand, toward(left, -forward, 45.0));
            palm_flat(&mut pose, rig, Bone::RightHand, toward(-left, -forward, 45.0));
            set_down(&mut pose, rig, &[SEAT, Contact::Foot(Bone::LeftFoot), Contact::Foot(Bone::RightFoot)]);
        }
        FloorPose::HugKnees => {
            // Knees up, feet flat near the seat, the trunk rounded forward,
            // the arms wrapped around the shins, the hands meeting in front.
            // The thighs raised until the flat feet are level with the seat.
            let build = |thigh: f32| {
                let mut pose = LocalPose::REST;
                trunk(&mut pose, rig, -10.0, 15.0, 0.0);
                legs(&mut pose, rig, -10.0, [thigh, -20.0, 0.0]);
                pose
            };
            let feet_above_seat = |pose: &LocalPose| {
                let sole = |foot| getup::contact_height(pose, rig, Contact::Foot(foot));
                sole(Bone::LeftFoot).min(sole(Bone::RightFoot)) - getup::contact_height(pose, rig, SEAT)
            };
            pose = build(solve((-175.0, -95.0), |t| feet_above_seat(&build(t))));
            set_down(&mut pose, rig, &[SEAT, Contact::Foot(Bone::LeftFoot), Contact::Foot(Bone::RightFoot)]);
            let at = joints(&pose, rig);
            let shins = (at[Bone::LeftLeg] + at[Bone::RightLeg] + at[Bone::LeftFoot] + at[Bone::RightFoot]) * 0.25;
            for (arm, side) in [(ARMS[0], left), (ARMS[1], -left)] {
                reach(&mut pose, rig, arm, shins + forward * 0.12 + up * 0.08 + side * 0.03, side + down * 0.3);
            }
        }
        FloorPose::SideSit => return feet_on_floor(getup::side_sit(rig, true), rig),
        FloorPose::Kneeling => {
            // On both knees, shins back along the floor, toes pointing back,
            // sitting on the heels; upright, hands on the thighs. The thighs
            // pitched until the seat rests just above the heels.
            // Sagittal angles, not directions: a foot aimed backward by the
            // shortest arc yawed round, its sole facing down 119 mm into the
            // floor. The toes tucked under, heels up (the half-kneel's back
            // foot, 130°): flat-topped feet (170°) had to turn through
            // pointing straight down on the way from the half-kneel, toes
            // through the floor. `pitch`: the thighs' angle below level; the
            // shins back along the floor.
            let build = |pitch: f32| {
                let mut pose = LocalPose::REST;
                trunk(&mut pose, rig, 0.0, 0.0, 0.0);
                kneeling_legs(&mut pose, rig, -(90.0 - pitch), 86.0);
                pose
            };
            let on_heels = |pose: &LocalPose| {
                let at = joints(pose, rig);
                getup::contact_height(pose, rig, SEAT) - at[Bone::LeftFoot].y - 0.03
            };
            pose = build(solve((15.0, 80.0), |p| on_heels(&build(p))));
            hands_on_thighs(&mut pose, rig, 0.65);
        }
    }
    pose
}

/// A seated floor pose, placed where the floor route down to it sits: its
/// seat where the propped sit's is, kneeling its right knee where the
/// half-kneel's is, the side-sit where the get-up's side route starts.
pub fn floor_pose_placed(how: FloorPose, rig: &RigGeometry) -> LocalPose {
    let pose = match how {
        FloorPose::Kneeling => placed(floor_pose(how, rig), rig, Bone::RightLeg, half_kneel_knee(rig)),
        _ => {
            let sit = getup::keys(getup::Lying::FaceUp, rig)[0].pose;
            placed(floor_pose(how, rig), rig, Bone::Hips, joints(&sit, rig)[Bone::Hips])
        }
    };
    toes_on_floor(pose, rig)
}

/// Floor keys with their toes bent onto the floor ([`toes_on_floor`]).
fn on_toes(keys: Vec<SitKey>, rig: &RigGeometry) -> Vec<SitKey> {
    keys.into_iter().map(|key| SitKey { pose: toes_on_floor(key.pose, rig), ..key }).collect()
}

/// Where the get-up's half-kneel, its front foot where standing feet are,
/// puts its back knee: kneeling keys put their right knee there.
fn half_kneel_knee(rig: &RigGeometry) -> Vec3 {
    joints(&getup::keys(getup::Lying::FaceDown, rig)[1].pose, rig)[Bone::RightLeg]
}

/// How far [`knees_aside`] tips the knees over, degrees.
const KNEES_ASIDE: f32 = 80.0;

/// How long the legs take to fold onto (or come up off) the side-sit's
/// hip, seconds.
const SIDE_FOLD_SECONDS: f32 = 1.2;

/// Sitting propped with the knees up (the get-up's sit, `sit`), both
/// thighs tipped `degrees` toward the side the side-sit folds its legs to
/// (its right): between the two, so the legs fall over rather than sweep
/// through the floor. Blended straight from one to the other, a knee went
/// so far under that the lift hoisted the body 23 cm and dropped it.
fn knees_aside(sit: &LocalPose, rig: &RigGeometry, degrees: f32) -> LocalPose {
    let tipped = |radians: f32| {
        let mut pose = *sit;
        let tip = bevy::math::Quat::from_axis_angle(rig.forward(), radians);
        for thigh in [Bone::LeftUpLeg, Bone::RightUpLeg] {
            pose.rotations[thigh] = super::rig::delta_after_world_turn(&pose, rig, thigh, tip);
        }
        pose
    };
    // Whichever way about `forward` carries the knees to the right on this
    // rig: a sign about an axis is a property of the rig's facing.
    let right = -rig.left();
    let knees = |pose: &LocalPose| joints(pose, rig)[Bone::LeftLeg] + joints(pose, rig)[Bone::RightLeg];
    let radians = degrees.to_radians();
    let radians = if (knees(&tipped(radians)) - knees(sit)).dot(right) > 0.0 { radians } else { -radians };
    // The feet turned flat to point out to that side: from the sit's toes
    // forward to the side-sit's toes back is half a turn, and the shortest
    // way round swung the top foot's toes through pointing straight down,
    // 336 mm into the floor. Through pointing aside it is two level quarter
    // turns.
    let mut pose = tipped(radians);
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    for (ankle, toe) in [(Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightFoot, Bone::RightToeBase)] {
        aim(&mut pose, &rest, ankle, toe, toward(right, rig.forward(), 20.0));
    }
    clear_floor(pose, rig)
}

/// The keys from standing down to sitting on the floor `how`. Kneeling: one
/// knee down, then the other, then back onto the heels. Otherwise through a
/// squat to sitting propped on the hands; the side-sit too (through the
/// get-up's hands and knees, a foot swept through the floor).
pub fn floor_down(how: FloorPose, rig: &RigGeometry) -> Vec<SitKey> {
    let seated = floor_pose_placed(how, rig);
    let keys = match how {
        FloorPose::Kneeling => {
            let half = getup::keys(getup::Lying::FaceDown, rig)[1].pose;
            let tall = placed(tall_kneel(rig), rig, Bone::RightLeg, half_kneel_knee(rig));
            // Onto one knee is the long step: at 1.0 s a toe moved 65 mm in
            // a frame (3.9 m/s).
            vec![SitKey { pose: half, seconds: 1.3 }, SitKey { pose: tall, seconds: 0.7 }, SitKey { pose: seated, seconds: 0.8 }]
        }
        _ => {
            // The face-up rise backward: squat, then sit propped.
            let up = getup::keys(getup::Lying::FaceUp, rig);
            let (sit, squat) = (up[0].pose, up[1].pose);
            let mut keys = vec![SitKey { pose: squat, seconds: 1.0 }, SitKey { pose: sit, seconds: 0.9 }];
            if how == FloorPose::SideSit {
                // The legs folded over slowly: they travel furthest.
                keys.push(SitKey { pose: knees_aside(&sit, rig, KNEES_ASIDE), seconds: 0.8 });
                keys.push(SitKey { pose: seated, seconds: SIDE_FOLD_SECONDS });
            } else {
                keys.push(SitKey { pose: seated, seconds: 0.8 });
            }
            keys
        }
    };
    on_toes(keys, rig)
}

/// The keys from sitting on the floor `how` up to just before standing.
pub fn floor_up(how: FloorPose, rig: &RigGeometry) -> Vec<SitKey> {
    let into = |keys: Vec<getup::GetUpKey>| keys.into_iter().map(|key| SitKey { pose: key.pose, seconds: key.seconds });
    let keys = match how {
        FloorPose::Kneeling => vec![
            SitKey { pose: placed(tall_kneel(rig), rig, Bone::RightLeg, half_kneel_knee(rig)), seconds: 0.7 },
            SitKey { pose: getup::keys(getup::Lying::FaceDown, rig)[1].pose, seconds: 0.9 },
        ],
        // Off the hip, the knees come up beside it first.
        FloorPose::SideSit => {
            let sit = getup::keys(getup::Lying::FaceUp, rig)[0].pose;
            std::iter::once(SitKey { pose: knees_aside(&sit, rig, KNEES_ASIDE), seconds: SIDE_FOLD_SECONDS })
                .chain(into(getup::keys(getup::Lying::FaceUp, rig)))
                .collect()
        }
        _ => into(getup::keys(getup::Lying::FaceUp, rig)).collect(),
    };
    on_toes(keys, rig)
}

// ---------------------------------------------------------------------------
// Between keys
// ---------------------------------------------------------------------------

/// Where `toe`'s tip is under `pose`: on from the toe joint as the toe
/// bone itself turns (the sole's own tip contact rides the foot, as if the
/// toes could not bend).
fn toe_tip(pose: &LocalPose, rig: &RigGeometry, toe: Bone) -> Vec3 {
    joints(pose, rig)[toe] + super::rig::accumulate_world_rotations(pose, rig)[toe] * rig.toe_end_offset(toe)
}

/// The heel's and the ball's height, the sole's first two contacts.
fn heel_and_ball(pose: &LocalPose, rig: &RigGeometry, ankle: Bone) -> f32 {
    let hips = joints(pose, rig)[Bone::Hips];
    super::foot::Sole::of(rig, ankle).points(pose, rig)[..2].iter().map(|p| hips.y + p.y).fold(f32::MAX, f32::min)
}

/// `pose` with each toe whose tip goes under the floor bent up at the ball
/// until it rests on it, as tucked toes do. Kept straight, the tucked toes
/// of a kneel went 71 mm into the floor, a side-sit's 57.
fn toes_on_floor(mut pose: LocalPose, rig: &RigGeometry) -> LocalPose {
    for toe in [Bone::LeftToeBase, Bone::RightToeBase] {
        if toe_tip(&pose, rig, toe).y >= 0.0 {
            continue;
        }
        let along = toe_tip(&pose, rig, toe) - joints(&pose, rig)[toe];
        let axis = along.cross(Vec3::Y);
        if axis.length_squared() < 1.0e-10 {
            continue;
        }
        let (axis, start) = (axis.normalize(), pose);
        let bent = |angle: f32| {
            let mut bent = start;
            bent.rotations[toe] = super::rig::delta_after_world_turn(&start, rig, toe, bevy::math::Quat::from_axis_angle(axis, angle));
            bent
        };
        pose = bent(solve((0.0, std::f32::consts::FRAC_PI_2 * 1.2), |angle| toe_tip(&bent(angle), rig, toe).y));
    }
    pose
}

/// The lowest any part a sitting body rests on goes, metres above the
/// floor: heels, balls, toe tips, knees, hands, the seat, each by its
/// clearance.
pub fn lowest_point(pose: &LocalPose, rig: &RigGeometry) -> f32 {
    let at = joints(pose, rig);
    [
        heel_and_ball(pose, rig, Bone::LeftFoot),
        heel_and_ball(pose, rig, Bone::RightFoot),
        toe_tip(pose, rig, Bone::LeftToeBase).y,
        toe_tip(pose, rig, Bone::RightToeBase).y,
        at[Bone::LeftToeBase].y,
        at[Bone::RightToeBase].y,
        at[Bone::LeftLeg].y - KNEE_CLEARANCE,
        at[Bone::RightLeg].y - KNEE_CLEARANCE,
        at[Bone::LeftHand].y - HAND_CLEARANCE,
        at[Bone::RightHand].y - HAND_CLEARANCE,
        getup::contact_height(pose, rig, SEAT),
    ]
    .into_iter()
    .fold(f32::MAX, f32::min)
}

/// `blended` (`from` blended `t` of the way to `to`) with each leg carried
/// by its foot: the ankle along the straight line between where the two
/// keys have it, the knee bending toward where they have it, blended, each
/// segment turned the least from how the blend left it, the foot as the
/// blend turned it.
///
/// Blended segment by segment, a leg went where its rotations took it, not
/// where its foot was going: into the side-sit a shin swung its foot
/// 258-348 mm under the floor between two keys that each had it on the
/// floor. A straight line between two points on or above the floor stays
/// there.
pub fn legs_by_their_feet(mut blended: LocalPose, from: &LocalPose, to: &LocalPose, t: f32, rig: &RigGeometry) -> LocalPose {
    let (a, b) = (joints(from, rig), joints(to, rig));
    let bind = super::rig::accumulate_bind_rotations(rig);
    for [hip, knee, ankle] in [LEFT_LEG, RIGHT_LEG] {
        let at = joints(&blended, rig);
        let world = super::rig::accumulate_world_rotations(&blended, rig);
        let foot_world = world[ankle];
        let target = a[ankle].lerp(b[ankle], t);
        let (thigh, shin) = ((at[knee] - at[hip]).length(), (at[ankle] - at[knee]).length());
        let to_target = target - at[hip];
        let distance = to_target.length().clamp(1.0e-4, (thigh + shin) * 0.999);
        let along = to_target.normalize_or_zero();
        let cos = ((thigh * thigh + distance * distance - shin * shin) / (2.0 * thigh * distance)).clamp(-1.0, 1.0);
        // Which way the knee bends: the way the plain blend has it bent
        // here, or where its knee lies within 1 cm of the line from hip to
        // ankle, the two keys' own bends blended. (Used to build keys only,
        // not every frame: as the blend's knee nears that line its side
        // turns fast, and solved per frame a knee whipped 149-260 mm.)
        let bend_of = |p: &BoneSet<Vec3>| {
            let line = (p[ankle] - p[hip]).normalize_or_zero();
            let off = p[knee] - p[hip];
            off - line * line.dot(off)
        };
        let own = bend_of(&at);
        let towards = if own.length() > 0.01 { own } else { bend_of(&a).normalize_or_zero().lerp(bend_of(&b).normalize_or_zero(), t) };
        let side = (towards - along * along.dot(towards)).normalize_or_zero();
        if side == Vec3::ZERO {
            continue;
        }
        let new_knee = at[hip] + (along * cos + side * (1.0 - cos * cos).sqrt()) * thigh;
        let new_ankle = at[hip] + along * distance;
        // The least turn of each segment onto its new line.
        let turn = |pose: &mut LocalPose, bone: Bone, from_dir: Vec3, to_dir: Vec3| {
            let arc = bevy::math::Quat::from_rotation_arc(from_dir.normalize(), to_dir.normalize());
            pose.rotations[bone] = super::rig::delta_after_world_turn(pose, rig, bone, arc);
        };
        turn(&mut blended, hip, at[knee] - at[hip], new_knee - at[hip]);
        let at = joints(&blended, rig);
        turn(&mut blended, knee, at[ankle] - at[knee], new_ankle - at[knee]);
        // The foot back to the blend's own turn.
        let parent = super::rig::accumulate_world_rotations(&blended, rig)[knee];
        blended.rotations[ankle] = super::rig::delta_from_world(ankle, parent, foot_world, rig, &bind);
    }
    blended
}

/// `keys` (each blended into from the one before, the first from `from`)
/// with a key added half-way through any move whose plain blend would
/// sweep a part more than 2 cm through the floor: the blend at its middle,
/// its legs carried by their feet ([`legs_by_their_feet`]), set clear of the
/// floor. Twice over, so a move may become four.
///
/// Built once, as keys, and not solved every frame: the legs' bend side
/// comes from the keys' own and can turn fast near a straight leg (a knee
/// whipped 149-260 mm in a frame solved per frame), while a plain blend
/// between fixed keys moves continuously whatever they are.
pub fn refined(from: &LocalPose, keys: Vec<SitKey>, rig: &RigGeometry) -> Vec<SitKey> {
    fn split(from: &LocalPose, key: SitKey, rig: &RigGeometry, depth: u32, out: &mut Vec<SitKey>) {
        let middle = super::rig::blend_in_world(from, &key.pose, 0.5, rig);
        let sweeps = (0..=8).any(|i| lowest_point(&super::rig::blend_in_world(from, &key.pose, i as f32 / 8.0, rig), rig) < -0.02);
        if depth == 0 || !sweeps {
            out.push(key);
            return;
        }
        let middle = SitKey { pose: clear_floor(legs_by_their_feet(middle, from, &key.pose, 0.5, rig), rig), seconds: key.seconds * 0.5 };
        split(from, middle, rig, depth - 1, out);
        split(&middle.pose, SitKey { seconds: key.seconds * 0.5, ..key }, rig, depth - 1, out);
    }
    let mut out = Vec::new();
    let mut previous = *from;
    for key in keys {
        split(&previous, key, rig, 2, &mut out);
        previous = key.pose;
    }
    out
}

/// How far under the floor a part may go before [`clear_floor`] acts,
/// metres: a foot resting on the floor is left alone.
const FLOOR_SLACK: f32 = 0.005;

/// `pose` lifted out of the floor by however far its lowest part
/// ([`lowest_point`]) goes under it. Two keys on the floor are each clear,
/// but a limb blended from one to the other can dip through it a little.
///
/// A lift only, which changes smoothly with the blend. Turning a foot up at
/// the ankle as well snapped: toes-up lowers the heel, so a flat foot a
/// millimetre under turned the full 90° (5.9 m/s), and over a foot pointing
/// straight down the turn's axis flipped (a toe moved 35 cm in a frame).
/// The keys are laid out so no foot sweeps far through the floor: kneeling
/// keeps its toes tucked, and the side-sit goes by the propped sit.
pub fn clear_floor(mut pose: LocalPose, rig: &RigGeometry) -> LocalPose {
    let under = -lowest_point(&pose, rig) - FLOOR_SLACK;
    if under > 0.0 {
        pose.root_translation.y += under;
    }
    pose
}

// ---------------------------------------------------------------------------
// Either
// ---------------------------------------------------------------------------

/// The seated pose `sitting`, placed (see [`chair_pose`],
/// [`floor_pose_placed`]).
pub fn seated(sitting: Sitting, rig: &RigGeometry, stood: &LocalPose, chair: Seat) -> LocalPose {
    match sitting {
        Sitting::Chair(how) => chair_pose(how, rig, stood, chair),
        Sitting::Floor(how) => floor_pose_placed(how, rig),
    }
}

/// The keys down from standing, ending seated.
pub fn sitting_down(sitting: Sitting, rig: &RigGeometry, stood: &LocalPose, chair: Seat) -> Vec<SitKey> {
    match sitting {
        Sitting::Chair(how) => chair_down(how, rig, stood, chair),
        Sitting::Floor(how) => floor_down(how, rig),
    }
}

/// The keys up from seated to just before standing.
pub fn standing_up(sitting: Sitting, rig: &RigGeometry, stood: &LocalPose, chair: Seat) -> Vec<SitKey> {
    match sitting {
        Sitting::Chair(how) => chair_up(how, rig, stood, chair),
        Sitting::Floor(how) => floor_up(how, rig),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gltf_rig::{puppet_base, puppet_base_as_rendered};
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn rigs() -> [(&'static str, RigGeometry); 2] {
        [("puppet_base", puppet_base()), ("as rendered", puppet_base_as_rendered())]
    }

    fn stood(rig: &RigGeometry) -> LocalPose {
        stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, rig)
    }

    /// A standard chair, sat on from where the walk meant to stop.
    const CHAIR: Seat = Seat { height: CHAIR_HEIGHT, back: 0.0, across: 0.0 };

    #[test]
    fn a_seat_off_where_the_feet_stand_is_met_by_the_shins_with_the_feet_kept() {
        // A walk to a chair stops a few centimetres off its spot; the seat
        // is that much further back, forward or across from the feet, and
        // the hips must land on it, at its height, the feet where they stood
        // and still flat.
        for (name, rig) in rigs() {
            let stood = stood(&rig);
            let standing = joints(&stood, &rig);
            let upright = joints(&chair_pose(ChairPose::Upright, &rig, &stood, CHAIR), &rig)[Bone::Hips];
            for (back, across) in [(-0.1, 0.0), (-0.05, 0.0), (0.05, 0.0), (0.1, 0.0), (0.0, -0.06), (0.0, 0.06), (0.08, 0.05), (-0.08, -0.05)] {
                let seat = Seat { back, across, ..CHAIR };
                let pose = chair_pose(ChairPose::Upright, &rig, &stood, seat);
                let at = joints(&pose, &rig);
                let moved = at[Bone::Hips] - upright;
                assert!((moved.dot(-rig.forward()) - back).abs() < 0.015, "{name} {seat:?}: the hips went {:.3} m back", moved.dot(-rig.forward()));
                assert!((moved.dot(rig.left()) - across).abs() < 0.01, "{name} {seat:?}: the hips went {:.3} m left", moved.dot(rig.left()));
                assert!((getup::contact_height(&pose, &rig, SEAT) - CHAIR_HEIGHT).abs() < 0.01, "{name} {seat:?}: off the seat's height");
                for (ankle, toe) in [(Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightFoot, Bone::RightToeBase)] {
                    let off = Vec2::new(at[ankle].x - standing[ankle].x, at[ankle].z - standing[ankle].z).length();
                    assert!(off < 0.01, "{name} {seat:?}: the {ankle:?} moved {:.0} mm", off * 1e3);
                    let foot = (at[toe] - at[ankle]).normalize();
                    let was = (standing[toe] - standing[ankle]).normalize();
                    assert!(foot.dot(was) > 0.995, "{name} {seat:?}: the {ankle:?} turned {:.1}°", foot.dot(was).clamp(-1.0, 1.0).acos().to_degrees());
                }
                assert!(lowest_point(&pose, &rig) > -0.005, "{name} {seat:?}: under the floor");
            }
        }
    }

    /// The lowest point of a pose's feet, toes, knees, hands and seat.
    fn lowest(pose: &LocalPose, rig: &RigGeometry) -> (f32, &'static str) {
        let at = joints(pose, rig);
        let mut worst = (f32::MAX, "");
        for (name, height) in [
            ("left sole", getup::contact_height(pose, rig, Contact::Foot(Bone::LeftFoot))),
            ("right sole", getup::contact_height(pose, rig, Contact::Foot(Bone::RightFoot))),
            ("seat", getup::contact_height(pose, rig, SEAT)),
            ("left knee", at[Bone::LeftLeg].y - KNEE_CLEARANCE),
            ("right knee", at[Bone::RightLeg].y - KNEE_CLEARANCE),
            ("left hand", at[Bone::LeftHand].y - HAND_CLEARANCE),
            ("right hand", at[Bone::RightHand].y - HAND_CLEARANCE),
            ("left toe", at[Bone::LeftToeBase].y),
            ("right toe", at[Bone::RightToeBase].y),
        ] {
            if height < worst.0 {
                worst = (height, name);
            }
        }
        worst
    }

    #[test]
    fn every_seated_pose_rests_on_its_contacts_and_nothing_goes_under_the_floor() {
        for (name, rig) in rigs() {
            let stood = stood(&rig);
            for how in Sitting::ALL {
                let pose = seated(how, &rig, &stood, CHAIR);
                let (low, what) = lowest(&pose, &rig);
                assert!(low > -0.005, "{name} {}: the {what} is {:.0} mm under the floor", how.name(), -low * 1e3);
                let seat = getup::contact_height(&pose, &rig, SEAT);
                match how {
                    Sitting::Chair(_) => assert!((seat - CHAIR_HEIGHT).abs() < 0.01, "{name} {}: seat at {seat:.3} m", how.name()),
                    // On the heels, the seat is the heels' height up.
                    Sitting::Floor(FloorPose::Kneeling) => assert!(seat > 0.05 && seat < 0.3, "{name} kneeling: seat at {seat:.3} m"),
                    Sitting::Floor(_) => assert!(seat.abs() < 0.01, "{name} {}: seat {seat:.3} m off the floor", how.name()),
                }
            }
        }
    }

    #[test]
    fn a_chair_keeps_the_feet_where_it_stood_and_its_seat_where_the_upright_one_is() {
        for (name, rig) in rigs() {
            let stood = stood(&rig);
            let feet = |pose: &LocalPose| {
                let at = joints(pose, &rig);
                [at[Bone::LeftFoot], at[Bone::RightFoot]]
            };
            let standing = feet(&stood);
            // And off where the walk meant to stop: the seat moved back and
            // across, the feet still kept.
            for chair in [CHAIR, Seat { back: -0.08, across: -0.04, ..CHAIR }, Seat { back: 0.06, across: 0.05, ..CHAIR }] {
                let seat = joints(&chair_pose(ChairPose::Upright, &rig, &stood, chair), &rig)[Bone::Hips];
                for how in [ChairPose::Upright, ChairPose::Reclined, ChairPose::LegsCrossed, ChairPose::LeaningForward] {
                    let keys = chair_down(how, &rig, &stood, chair).into_iter().chain(chair_up(how, &rig, &stood, chair));
                    for (i, key) in keys.enumerate() {
                        let at = joints(&key.pose, &rig);
                        // Reclined, the legs go out; crossed, the right foot lifts.
                        let moves = [how == ChairPose::Reclined, how == ChairPose::Reclined || how == ChairPose::LegsCrossed];
                        let is_seated = getup::contact_height(&key.pose, &rig, SEAT) < CHAIR_HEIGHT + 0.005;
                        for (side, (now, then)) in feet(&key.pose).into_iter().zip(standing).enumerate() {
                            if moves[side] && key.pose == chair_pose(how, &rig, &stood, chair) {
                                continue;
                            }
                            let off = Vec2::new(now.x - then.x, now.z - then.z).length();
                            assert!(off < 0.012, "{name} {chair:?} {how:?} key {i}: a foot {:.0} mm from where it stood", off * 1e3);
                        }
                        if is_seated {
                            let off = Vec2::new(at[Bone::Hips].x - seat.x, at[Bone::Hips].z - seat.z).length();
                            assert!(off < 0.03, "{name} {chair:?} {how:?} key {i}: seated {:.0} mm off the chair's seat", off * 1e3);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn sitting_upright_bends_the_knees_forward_by_about_a_right_angle() {
        // Signed, in the rig's own frame: the knee ahead of the hip and
        // the ankle, the shin hanging down from it.
        for (name, rig) in rigs() {
            let pose = chair_pose(ChairPose::Upright, &rig, &stood(&rig), CHAIR);
            let at = joints(&pose, &rig);
            let forward = rig.forward();
            for (hip, knee, ankle) in [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot)] {
                let (thigh, shin) = ((at[knee] - at[hip]).normalize(), (at[ankle] - at[knee]).normalize());
                assert!(thigh.dot(forward) > 0.8, "{name}: the thigh runs {:.2} forward", thigh.dot(forward));
                assert!(shin.y < -0.9, "{name}: the shin hangs {:.2} down", shin.y);
                let flexion = thigh.angle_between(shin).to_degrees();
                assert!((70.0..110.0).contains(&flexion), "{name}: the knee bends {flexion:.0}°");
            }
        }
    }

    #[test]
    fn standing_up_from_a_chair_takes_marshs_one_point_nine_seconds() {
        let rig = puppet_base();
        let keys = chair_up(ChairPose::Upright, &rig, &stood(&rig), CHAIR);
        let total: f32 = keys.iter().map(|key| key.seconds).sum::<f32>() + stand_seconds(Sitting::Chair(ChairPose::Upright));
        assert!((total - 1.9).abs() < 0.05, "{total:.2} s");
    }

    #[test]
    fn every_way_of_sitting_down_and_up_stays_out_of_the_floor_without_a_jump() {
        // The walker's posture stepped at 60 Hz: sit at 0 s, stand at 5 s.
        // Blended between floor keys, limbs swept through the floor (toes
        // 253 mm under, kneeling); turning feet up to clear it snapped a toe
        // 362 mm in a frame. Now the keys keep the feet out of the sweep, a
        // smooth lift takes the rest, and the fastest a joint moves is an
        // arm swung forward to rise from the floor (~3 m/s).
        //
        // On the rig as rendered only: the cycle ends in the standing pose,
        // `relaxed_stand`, authored in world axes, and `puppet_base` faces
        // away, so its standing hands are overhead (see the knowledge note on
        // that fixture); the arms swinging up to them moved 65 mm a frame.
        use crate::character::anim::walker::Posture;
        for (name, rig) in [("as rendered", puppet_base_as_rendered())] {
            let stood = stood(&rig);
            let dt = 1.0 / 60.0;
            for how in Sitting::ALL {
                let mut posture = Posture::Standing;
                let (mut lowest_seen, mut jump, mut previous): (f32, (f32, f32, Bone), Option<BoneSet<Vec3>>) = (f32::MAX, (0.0, 0.0, Bone::Hips), None);
                for frame in 0..(10.0 / dt) as usize {
                    let time = frame as f32 * dt;
                    let pose = posture.advance((time < 5.0).then_some(how), CHAIR, &stood, &rig, true, dt).map_or(stood, |(pose, _)| pose);
                    lowest_seen = lowest_seen.min(lowest_point(&pose, &rig));
                    let at = joints(&pose, &rig);
                    if let Some(was) = previous {
                        for &bone in Bone::ALL.iter() {
                            if at[bone].distance(was[bone]) > jump.0 {
                                jump = (at[bone].distance(was[bone]), time, bone);
                            }
                        }
                    }
                    previous = Some(at);
                }
                assert!(posture.is_standing(), "{name} {}: not standing again after 5 s", how.name());
                assert!(lowest_seen > -0.006, "{name} {}: {:.0} mm under the floor", how.name(), -lowest_seen * 1e3);
                assert!(jump.0 < 0.06, "{name} {}: {} moved {:.0} mm in a frame at {:.2} s", how.name(), jump.2.name(), jump.0 * 1e3, jump.1);
            }
        }
    }

    #[test]
    fn a_chair_sat_on_off_its_spot_keeps_the_planted_feet_still() {
        // The walker's posture stepped at 60 Hz onto a seat moved back and
        // across (a walk stopped off its spot): the feet planted through a
        // move stay where they were, frame by frame.
        use crate::character::anim::walker::Posture;
        let rig = puppet_base_as_rendered();
        let stood = stood(&rig);
        let dt = 1.0 / 60.0;
        for chair in [CHAIR, Seat { back: -0.08, across: -0.04, ..CHAIR }, Seat { back: 0.06, across: 0.05, ..CHAIR }] {
            let mut posture = Posture::Standing;
            let start = joints(&stood, &rig);
            let mut worst = 0.0_f32;
            for frame in 0..(8.0 / dt) as usize {
                let wanted = (frame as f32 * dt < 4.0).then_some(Sitting::Chair(ChairPose::Upright));
                let Some((pose, planted)) = posture.advance(wanted, chair, &stood, &rig, true, dt) else { continue };
                let at = joints(&pose, &rig);
                for (side, foot) in [Bone::LeftFoot, Bone::RightFoot].into_iter().enumerate() {
                    if planted[side] {
                        worst = worst.max(Vec2::new(at[foot].x - start[foot].x, at[foot].z - start[foot].z).length());
                    }
                }
            }
            assert!(worst < 0.005, "{chair:?}: a planted foot moved {:.1} mm", worst * 1e3);
        }
    }

    #[test]
    fn every_name_round_trips() {
        for how in Sitting::ALL {
            assert_eq!(Sitting::by_name(how.name()), Some(how));
        }
    }
}
