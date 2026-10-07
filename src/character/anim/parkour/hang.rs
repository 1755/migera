//! Grabbing a ledge and hanging from it: the first step of the parkour
//! design (`docs/knowledge/character-animation/parkour/`).
//!
//! Standing square to a wall under its edge, the body jumps (the standing
//! jump, [`Jump`]) just high enough for the hands to meet the lip a little
//! before the top of the flight, the arms reaching up for it through the
//! flight. At contact the hips keep their velocity and the arms take it:
//!
//! - **Radially** (the arms giving, then holding): a damped spring on the
//!   grip-to-hips distance ([`GIVE`]).
//! - **About the grip**: with a wall below to brace the feet on, the body
//!   settles out from it, the feet swinging onto the wall
//!   (`BRACED_*`); with none, a compound pendulum about the grip, its
//!   period about 2.4 s for a person (no damping data: the hanger damps it,
//!   [`SWING_DAMPING`]).
//!
//! Hanging, the hands hook over the lip (`hand::hooked`), palms against
//! the face, arms near straight ([`HANG_REACH`]), shoulders lifted toward
//! them (`armik::shoulder_lift`). There is no catch or braced-hang data;
//! the catch's give and the hang's shape are set from the rig's own
//! limits (`parkour-movement-data`).
//!
//! The body is posed from its hips each frame, as a ladder's climb is: the
//! trunk leant toward the grip, the arms solved to the lip, the legs to
//! their feet; the root rides the hips.

use bevy::math::{Quat, Vec3};

use super::geometry::Ledge;
use crate::character::anim::armik::{frame_turn, shoulder_lift, solve_arm_toward_from, turn_hand, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::hand::{hook_lip, HandGrip, FINGER_HALF_THICKNESS, GRIP_RADIUS};
use crate::character::anim::jump::{lead_of, Jump, JumpAsk, JumpPhase, GRAVITY, HIGHEST};
use crate::character::anim::math::SpringParams;
use crate::character::anim::rig::{accumulate_bind_rotations, accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

mod shimmy;
mod up;

pub use shimmy::Shimmy;

/// What a walker is asked to do with its ledge ([`Walker::hang`](crate::character::anim::Walker::hang)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HangAsk {
    /// Walk under it, jump for its lip, and hang from it.
    Grab,
    /// Hanging from it, climb up onto its top and stand there.
    ClimbUp,
    /// Hanging from it, shimmy along it while asked.
    Shimmy(Shimmy),
}

/// Each leg's socket, knee, ankle and toe: left, right.
const LEGS: [(Bone, Bone, Bone, Bone); 2] =
    [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase)];
const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
/// Which way each side lies along the character's left.
const SIGN: [f32; 2] = [1.0, -1.0];

/// Where the hips stand to jump for a ledge, metres out from its face, and
/// how far ahead the jump would land them, metres: it jumps up and in. Its
/// hips 0.35 m out, the walk up to it swung a hand 14 cm into the wall; at
/// 0.55 its stop still lands 11 cm long and a finger brushes it (5 cm). At
/// 0.65 (jumping in 0.3) the body is still short of the wall at the top of
/// the flight, and only a 1.9 m ledge was reached.
pub const SPOT_OUT: f32 = 0.55;
const JUMP_IN: f32 = 0.2;
/// How far the hands reach from their (lifted) shoulders as they meet the
/// lip, of the arm: the arms not quite straight, still able to give.
const CATCH_REACH: f32 = 0.9;
/// How much higher than just enough the jump goes, metres of the COM's
/// rise: the hands meet the lip a little before the top of the flight.
const CATCH_MARGIN: f32 = 0.04;
/// How far the hands reach from their shoulders hanging, of the arm: near
/// straight, the shoulders lifted.
const HANG_REACH: f32 = 0.94;
/// The arms' give at the catch: the grip-to-hips distance springs to its
/// hanging length at this rate, rad/s, and damping ratio.
const GIVE: (f32, f32) = (10.0, 0.9);
/// With a wall below: the hips settle this far out from its face, metres;
/// the legs reach the wall at this share of their length; the body
/// settles out from it at this rate, rad/s, critically damped.
const BRACED_OUT: f32 = 0.42;
const BRACED_LEG: f32 = 0.95;
const BRACED_SETTLE: f32 = 5.5;
/// How long the feet take to swing onto the wall after the catch, seconds.
const FEET_TO_WALL: f32 = 0.35;
/// The least wall below the edge to brace the feet on, beyond where they
/// go, metres.
const WALL_MARGIN: f32 = 0.1;
/// Free, the body's radius of gyration about its COM, metres, for the
/// compound pendulum's period (about 2.4 s for a person); and the share of
/// critical damping the hanger puts into the swing.
const GYRATION: f32 = 0.5;
const SWING_DAMPING: f32 = 0.2;
/// Free, the legs hang this share of their length below the hips, a little
/// bent; and take this long to come under the body after the catch.
const FREE_LEG: f32 = 0.97;
const LEGS_HANG: f32 = 0.3;
/// How far each foot is pitched toes-up against the wall, radians.
const FOOT_ON_WALL: f32 = 1.1;
/// The hands close onto the lip over this long before they reach it.
const CLOSING: f32 = 0.12;
/// Pressing on a top, each elbow's pole is out to its side by this much
/// for each unit back toward the hips: none, the elbows by the body (a
/// quarter out, they went 6.8 cm out of the shoulder-to-wrist line).
const PRESS_ELBOW_OUT: f32 = 0.0;
/// Sideways, the hips settle under the grip at this rate, rad/s.
const SIDEWAYS: f32 = 6.0;
/// Where a hand without known fingers hooks, metres along it from the
/// wrist: about the knuckles.
const GUESSED_KNUCKLES: f32 = 0.08;

/// How far the grip's middle stays from each end of `ledge` (`a`, `b`),
/// its hands `lane` either side of it: a hand's width from a bare end, and
/// as far as a hand keeps from a corner it may turn (`shimmy`).
fn grip_margins(ledge: &Ledge, others: &[Ledge], lane: f32) -> [f32; 2] {
    [0, 1].map(|end| lane + shimmy::hand_room(ledge, end, others).max(0.05))
}

/// The body as the standing pose has it, measured once.
#[derive(Debug, Clone)]
struct Body {
    stood: LocalPose,
    /// The hips joint, the pose's frame.
    hips: Vec3,
    /// Each hip socket, shoulder and clavicle's root, from the hips joint.
    sockets: [Vec3; 2],
    shoulders: [Vec3; 2],
    clavicles: [Vec3; 2],
    /// Each arm's and leg's length end to end.
    arms: [f32; 2],
    legs: [f32; 2],
    /// Each ankle from its ball, standing, the pose's frame; and each
    /// foot's world rotation.
    ankles: [Vec3; 2],
    attitudes: [Quat; 2],
    /// Each hand's bind rotation, and how it holds a bar in its rest frame.
    hand_binds: [Quat; 2],
    grips: [HandGrip; 2],
}

impl Body {
    fn of(stood: &LocalPose, rig: &RigGeometry) -> Self {
        let at = forward_kinematics_on(stood, rig);
        let world = accumulate_world_rotations(stood, rig);
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let binds = accumulate_bind_rotations(rig);
        let hips = at[Bone::Hips];
        Self {
            stood: *stood,
            hips,
            sockets: LEGS.map(|(socket, _, _, _)| at[socket] - hips),
            shoulders: ARMS.map(|arm| at[arm.shoulder] - hips),
            clavicles: CLAVICLES.map(|clavicle| at[clavicle] - hips),
            arms: ARMS.map(|arm| (at[arm.elbow] - at[arm.shoulder]).length() + (at[arm.wrist] - at[arm.elbow]).length()),
            legs: LEGS.map(|(socket, knee, ankle, _)| (at[knee] - at[socket]).length() + (at[ankle] - at[knee]).length()),
            ankles: LEGS.map(|(_, _, ankle, toe)| at[ankle] - at[toe]),
            attitudes: LEGS.map(|(_, _, ankle, _)| world[ankle]),
            hand_binds: ARMS.map(|arm| binds[arm.wrist]),
            // Until the fingers are known (`Hanging::set_grips`): the hand
            // on from the forearm, its rest palm down.
            grips: ARMS.map(|arm| {
                let along = (rest[arm.wrist] - rest[arm.elbow]).normalize_or(Vec3::NEG_Y);
                let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
                HandGrip { bar: along * GUESSED_KNUCKLES + palm * (GRIP_RADIUS + FINGER_HALF_THICKNESS), palm, along }
            }),
        }
    }
}

/// Where it is in a grab.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// Jumping for the lip, `t` seconds into the jump.
    Jumping,
    /// Hanging, caught `since` seconds ago.
    Hanging,
}

/// The hang's swing: the hips about the grip in the wall's plane square to
/// it, polar (`r` from the grip, `theta` out from straight below), and
/// along the edge.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Swing {
    r: f32,
    dr: f32,
    theta: f32,
    dtheta: f32,
    along: f32,
    dalong: f32,
}

/// A walker grabbing a ledge and hanging from it.
#[derive(Debug, Clone)]
pub struct Hanging {
    ledge: Ledge,
    body: Body,
    /// The rig it was measured on.
    rig: std::sync::Arc<RigGeometry>,
    /// The facing square to the wall (radians about `+Y`), and its turn.
    yaw: f32,
    turn: Quat,
    /// Where the root stood to jump, and how far the foot IK had the hips
    /// down (`AnimFootIk::pelvis_drop`).
    stood_at: Vec3,
    drop: f32,
    jump: Jump,
    /// When the hands meet the lip, seconds into the jump.
    catch_at: f32,
    phase: Phase,
    /// Seconds since the catch.
    since: f32,
    /// The grip's middle on the lip, and each hand's lip point.
    grip: Vec3,
    lips: [Vec3; 2],
    /// Feet braced on the wall, or the legs hanging free.
    braced: bool,
    /// Where the hips come to rest hanging, polar about the grip.
    rest: (f32, f32),
    swing: Swing,
    /// Each ankle as the hands caught the lip, and where it goes hanging:
    /// on the wall, braced; under the hips, free (the hips' frame there).
    caught_ankles: [Vec3; 2],
    wall_balls: [Vec3; 2],
    /// Each wrist as the push began, where its reach for the lip starts.
    reach_from: [Vec3; 2],
    /// Asked to climb up onto the top ([`Self::climb_up`]), and climbing.
    up_asked: bool,
    up: Option<up::ClimbUp>,
    /// Shimmying along the lip: the way asked this frame, and the step
    /// under way ([`Self::shimmy`]).
    shimmy_ask: Option<Shimmy>,
    step: Option<shimmy::Step>,
    /// How far it is pulled up from the hang's length, shimmying, metres.
    pulled: f32,
    /// The other ledges it may shimmy onto round a corner.
    others: Vec<Ledge>,
    /// How far apart the hands hold, a shoulder's width, metres.
    spread: f32,
}

impl Hanging {
    /// Where a walker's root stands to jump for `ledge`, facing `square`
    /// (radians about `+Y`), from near `near`: its hips [`SPOT_OUT`] out
    /// from the face, under the edge's nearest point, on the floor at
    /// `near`'s height.
    pub fn spot(ledge: &Ledge, others: &[Ledge], near: Vec3, square: f32, stood: &LocalPose, rig: &RigGeometry) -> Vec3 {
        let at = forward_kinematics_on(stood, rig);
        let hips = at[Bone::Hips];
        let lane = ARMS.map(|arm| (at[arm.shoulder] - hips).dot(rig.left()).abs());
        let margins = grip_margins(ledge, others, lane[0].max(lane[1])).map(|margin| margin.max(0.3));
        let edge = ledge.nearest_within(near, margins);
        let at = edge + ledge.out * SPOT_OUT - Quat::from_rotation_y(square) * hips;
        Vec3::new(at.x, near.y, at.z)
    }

    /// The facing that squares a rig facing `forward` to `ledge`'s wall.
    pub fn square(ledge: &Ledge, forward: Vec3) -> f32 {
        let heading = |d: Vec3| d.x.atan2(d.z);
        heading(-ledge.out) - heading(forward)
    }

    /// A grab of `ledge` by a walker standing at `root` facing `square`
    /// (radians about `+Y`), its hips `drop` below standing: `None` if a
    /// standing jump cannot bring its hands to the lip, or they reach it
    /// standing (a mantle, not a hang).
    pub fn grab(ledge: &Ledge, root: Vec3, square: f32, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let body = Body::of(stood, rig);
        let turn = Quat::from_rotation_y(square);
        let mut hanging = Self {
            ledge: *ledge,
            body,
            rig: std::sync::Arc::new(rig.clone()),
            yaw: square,
            turn,
            stood_at: root,
            drop,
            jump: Jump::plan(JumpAsk::up(0.1), stood, rig),
            catch_at: 0.0,
            phase: Phase::Jumping,
            since: 0.0,
            grip: Vec3::ZERO,
            lips: [Vec3::ZERO; 2],
            braced: false,
            rest: (1.0, 0.0),
            swing: Swing { r: 1.0, dr: 0.0, theta: 0.0, dtheta: 0.0, along: 0.0, dalong: 0.0 },
            caught_ankles: [Vec3::ZERO; 2],
            wall_balls: [Vec3::ZERO; 2],
            reach_from: [Vec3::ZERO; 2],
            up_asked: false,
            up: None,
            shimmy_ask: None,
            step: None,
            pulled: 0.0,
            others: Vec::new(),
            spread: 0.0,
        };
        hanging.place_hands(rig);
        hanging.plan_jump(stood, rig)?;
        hanging.plan_hang(rig);
        Some(hanging)
    }

    /// Takes the hand grips the rig's own fingers make (`RelaxedHands`, in
    /// each hand's own frame, on `rig`): the hands placed so they hook over
    /// the lip. Replanned, the jump and the hang reaching the hands as they
    /// now sit.
    ///
    /// Taken in the hand's own frame as if in its rest frame, the hands
    /// pointed out from the wall, the middle knuckles 13 cm off the lip.
    pub fn set_grips(&mut self, grips: [Option<HandGrip>; 2], stood: &LocalPose, rig: &RigGeometry) {
        if self.phase != Phase::Jumping || self.jump.elapsed() > 0.0 {
            return;
        }
        let binds = accumulate_bind_rotations(rig);
        for side in 0..2 {
            if let Some(grip) = grips[side] {
                let bind = binds[ARMS[side].wrist];
                self.body.grips[side] = HandGrip { bar: bind * grip.bar, palm: (bind * grip.palm).normalize(), along: (bind * grip.along).normalize() };
            }
        }
        if self.plan_jump(stood, rig).is_some() {
            self.plan_hang(rig);
        }
    }

    /// Each hand's lip point: the edge nearest the hips, a shoulder's width
    /// apart.
    fn place_hands(&mut self, rig: &RigGeometry) {
        let left = self.turn * rig.left();
        let lane = self.body.shoulders.map(|s| s.dot(rig.left()).abs());
        let margins = grip_margins(&self.ledge, &self.others, lane[0].max(lane[1]));
        self.grip = self.ledge.nearest_within(self.stood_at + self.turn * self.body.hips, margins);
        self.lips = [0, 1].map(|side| self.grip + left * SIGN[side] * lane[side]);
        self.spread = lane[0] + lane[1];
    }

    /// The hand's world turn hooked over the lip: the palm against the face,
    /// the fingers up over the edge.
    fn hook_turn(&self, side: usize) -> Quat {
        let grip = &self.body.grips[side];
        let back = self.turn.inverse();
        self.turn * frame_turn(grip.along, grip.palm, back * Vec3::Y, back * -self.hand_out(side))
    }

    /// Each wrist hooked over its lip point (shimmying, a moving hand's off
    /// the lip on its way to the next).
    fn wrists(&self) -> [Vec3; 2] {
        [0, 1].map(|side| self.lip_now(side) - self.hook_turn(side) * hook_lip(&self.body.grips[side]))
    }

    /// The shoulder joint of arm `side`, the hips at `hips` and the trunk
    /// turned `trunk` (the world), lifted toward a wrist at `wrist`; and how
    /// far the wrist is from it, of the arm.
    fn reach(&self, side: usize, hips: Vec3, trunk: Quat, wrist: Vec3) -> f32 {
        let (root, shoulder) = (hips + trunk * self.body.clavicles[side], hips + trunk * self.body.shoulders[side]);
        let lifted = root + shoulder_lift(root, shoulder, wrist, 0.85 * self.body.arms[side]) * (shoulder - root);
        (wrist - lifted).length() / self.body.arms[side]
    }

    /// Where the jump's pose frame is at `t`: where it stood, moved on by the
    /// jump's way forward (`Jump::travelled_at`, its root motion).
    fn jump_root(&self, t: f32) -> Vec3 {
        self.stood_at + self.turn * (self.rig.forward() * self.jump.travelled_at(t))
    }

    /// The jump's pose at `t`, and its hips in the world.
    fn jump_hips(&self, t: f32, stood: &LocalPose, rig: &RigGeometry) -> (LocalPose, Vec3) {
        let pose = self.jump.pose_at(t, stood, rig);
        let hips = forward_kinematics_on(&pose, rig)[Bone::Hips];
        (pose, self.jump_root(t) + self.turn * hips)
    }

    /// The farther of the two hands' reaches at the jump's `t`, the trunk as
    /// the jump has it.
    fn jump_reach(&self, t: f32, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        let (pose, hips) = self.jump_hips(t, stood, rig);
        let trunk = self.turn * accumulate_world_rotations(&pose, rig)[Bone::Hips] * accumulate_world_rotations(&self.body.stood, rig)[Bone::Hips].inverse();
        let wrists = self.wrists();
        (0..2).map(|side| self.reach(side, hips, trunk, wrists[side])).fold(0.0, f32::max)
    }

    /// The lowest standing jump whose hands reach the lip (within
    /// [`CATCH_REACH`]) at the top of its flight, [`CATCH_MARGIN`] higher;
    /// and when, on the way up, they first do. `None` past [`HIGHEST`], or
    /// if they reach it standing.
    fn plan_jump(&mut self, stood: &LocalPose, rig: &RigGeometry) -> Option<()> {
        // Reached standing where the jump would land it, a step in: a mantle.
        self.jump = Jump::plan(JumpAsk::forward(0.05, JUMP_IN), stood, rig);
        let landed = self.jump.duration();
        if self.jump_reach(landed, stood, rig) <= CATCH_REACH {
            return None;
        }
        let top = |jump: &Jump| {
            let start = jump.ends(JumpPhase::Push);
            let span = jump.ends(JumpPhase::Flight) - start;
            (0..=60).map(|k| start + span * k as f32 / 60.0).max_by(|a, b| jump.com_height_at(*a).total_cmp(&jump.com_height_at(*b))).unwrap_or(start)
        };
        let mut height = 0.05;
        loop {
            self.jump = Jump::plan(JumpAsk::forward(height, JUMP_IN), stood, rig);
            if self.jump_reach(top(&self.jump), stood, rig) <= CATCH_REACH {
                break;
            }
            height += 0.01;
            if height > HIGHEST {
                return None;
            }
        }
        self.jump = Jump::plan(JumpAsk::forward((height + CATCH_MARGIN).min(HIGHEST), JUMP_IN), stood, rig);
        let (start, apex) = (self.jump.ends(JumpPhase::Push), top(&self.jump));
        let mut t = start;
        while t < apex && self.jump_reach(t, stood, rig) > CATCH_REACH {
            t += 1.0 / 240.0;
        }
        self.catch_at = t.min(apex);
        Some(())
    }

    /// Braced or free, where the hips come to rest, and the swing as the
    /// hands catch the lip.
    fn plan_hang(&mut self, rig: &RigGeometry) {
        let out = self.ledge.out;
        let wrists = self.wrists();
        // The hips at `height` below the grip and `away` out from it, the
        // trunk leant toward the grip: the farther hand's reach.
        let reach_at = |away: f32, below: f32| {
            let hips = self.grip + out * away - Vec3::Y * below;
            let trunk = self.lean_turn(hips, rig);
            (0..2).map(|side| self.reach(side, hips, trunk, wrists[side])).fold(0.0, f32::max)
        };
        // How far below the grip the hips hang `away` out, the arms at
        // HANG_REACH.
        let below_for = |away: f32| {
            let (mut low, mut high) = (0.2, 2.5);
            for _ in 0..40 {
                let middle = 0.5 * (low + high);
                if reach_at(away, middle) > HANG_REACH { high = middle } else { low = middle }
            }
            low
        };
        // Braced: the hips out from the wall, each foot on the face where its
        // leg reaches at BRACED_LEG, toes up; if the wall goes down that far.
        let below = below_for(BRACED_OUT);
        let hips = self.grip + out * BRACED_OUT - Vec3::Y * below;
        let trunk = self.lean_turn(hips, rig);
        let balls = [0, 1].map(|side| self.wall_ball(side, hips + trunk * self.body.sockets[side], BRACED_LEG));
        let lowest = balls[0].y.min(balls[1].y);
        let braced = self.grip.y - lowest + WALL_MARGIN <= self.ledge.wall_below;
        let rest = if braced {
            let off = hips - self.grip;
            (off.length(), off.dot(out).atan2(-off.y))
        } else {
            (below_for(0.0), 0.0)
        };
        (self.braced, self.wall_balls, self.rest) = (braced, balls, rest);

        // The hips' state as the hands meet the lip.
        let (at, before, after) = (self.catch_at, self.catch_at - 1.0e-3, self.catch_at + 1.0e-3);
        let (pose, hips) = self.jump_hips(at, &self.body.stood, rig);
        let velocity = (self.jump_hips(after, &self.body.stood, rig).1 - self.jump_hips(before, &self.body.stood, rig).1) / 2.0e-3;
        let off = hips - self.grip;
        let along = self.ledge.along();
        let plane = off - along * off.dot(along);
        let r = plane.length();
        let radial = plane / r.max(1.0e-6);
        let theta = plane.dot(out).atan2(-plane.y);
        let tangential = (out * theta.cos() + Vec3::Y * theta.sin()).normalize();
        self.swing = Swing { r, dr: velocity.dot(radial), theta, dtheta: velocity.dot(tangential) / r.max(1.0e-3), along: off.dot(along), dalong: velocity.dot(along) };
        let at = forward_kinematics_on(&pose, rig);
        let caught = self.jump_root(self.catch_at);
        self.caught_ankles = LEGS.map(|(_, _, ankle, _)| caught + self.turn * at[ankle]);
        let push = self.jump.ends(JumpPhase::Down);
        let (pushing, _) = self.jump_hips(push, &self.body.stood, rig);
        let at = forward_kinematics_on(&pushing, rig);
        let pushed = self.jump_root(push);
        self.reach_from = ARMS.map(|arm| pushed + self.turn * at[arm.wrist]);
    }

    /// Where foot `side`'s ball goes on the wall's face, toes up, below its
    /// socket at `socket`, for the leg to reach it at `reach` of its length.
    fn wall_ball(&self, side: usize, socket: Vec3, reach: f32) -> Vec3 {
        let out = self.ledge.out;
        let ankle_from_ball = self.ankle_from_ball(side, self.toes_up(side));
        let on_face = |height: f32| Vec3::new(socket.x, height, socket.z) - out * self.ledge.out_of(Vec3::new(socket.x, height, socket.z));
        let (mut low, mut high) = (socket.y - 2.0 * self.body.legs[side], socket.y);
        for _ in 0..40 {
            let middle = 0.5 * (low + high);
            if (on_face(middle) + ankle_from_ball - socket).length() > reach * self.body.legs[side] { low = middle } else { high = middle }
        }
        on_face(high)
    }

    /// The trunk's world turn leant toward the grip from hips at `hips`:
    /// about the character's left by the line to the grip's angle from
    /// upright.
    fn lean_turn(&self, hips: Vec3, rig: &RigGeometry) -> Quat {
        Quat::from_axis_angle(self.turn * rig.left(), self.lean_at(hips)) * self.turn
    }

    /// How far the trunk leans toward the wall from upright, the hips at
    /// `hips`: toward the grip.
    fn lean_at(&self, hips: Vec3) -> f32 {
        let to = self.grip - hips;
        to.dot(-self.face().0).atan2(to.y)
    }

    /// Foot `side`'s ankle from its ball (the world), the foot turned to
    /// world rotation `attitude`: the standing pose's offset, turned by how
    /// far `attitude` is from the standing foot's world rotation.
    ///
    /// The standing foot's world rotation is `turn · attitudes`: taken as
    /// `attitudes` alone, the facing turn went in twice, which a half turn
    /// (squaring to none) hid on every wall faced so far; a quarter turn
    /// round a corner put the ankle 14 cm along the face and the toes 6.5 cm
    /// into the block.
    fn ankle_from_ball(&self, side: usize, attitude: Quat) -> Vec3 {
        attitude * (self.turn * self.body.attitudes[side]).inverse() * (self.turn * self.body.ankles[side])
    }

    /// Foot `side`'s standing world rotation pitched toes-up by
    /// [`FOOT_ON_WALL`] about the wall's horizontal line.
    fn toes_up(&self, side: usize) -> Quat {
        // Toes up: the toes, pointing at the wall, turn up about the axis
        // along the edge that carries `-out` toward `+Y`.
        let (out, along) = self.face();
        let axis = (-out).cross(Vec3::Y).normalize_or(along);
        Quat::from_axis_angle(axis, FOOT_ON_WALL) * self.turn * self.body.attitudes[side]
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        match self.phase {
            Phase::Jumping => {
                let left = self.catch_at - self.jump.elapsed();
                if dt < left {
                    self.jump.advance(dt);
                    return;
                }
                self.jump.advance(left.max(0.0));
                self.phase = Phase::Hanging;
                self.since = 0.0;
                self.step_swing(dt - left.max(0.0));
            }
            Phase::Hanging => match self.up.as_mut() {
                Some(up) => up.t += dt,
                None => {
                    self.advance_shimmy(dt);
                    self.step_swing(dt);
                    self.start_up();
                }
            },
        }
    }

    /// The swing on `dt`: the arms' give radially, braced settling out or
    /// free swinging about the grip, and sideways under it.
    fn step_swing(&mut self, dt: f32) {
        self.since += dt;
        let steps = ((dt / (1.0 / 480.0)).ceil() as usize).max(1);
        let h = dt / steps as f32;
        let (give, ratio) = GIVE;
        let (rest_r, rest_theta) = self.rest;
        // Shimmying, pulled up a little.
        let rest_r = rest_r - self.pulled;
        for _ in 0..steps {
            let s = &mut self.swing;
            let radial = -give * give * (s.r - rest_r) - 2.0 * ratio * give * s.dr;
            let angular = if self.braced {
                -BRACED_SETTLE * BRACED_SETTLE * (s.theta - rest_theta) - 2.0 * BRACED_SETTLE * s.dtheta
            } else {
                // A compound pendulum about the grip, the hanger damping it.
                let d = s.r.max(0.1);
                let omega = (GRAVITY * d / (d * d + GYRATION * GYRATION)).sqrt();
                -omega * omega * s.theta.sin() - 2.0 * SWING_DAMPING * omega * s.dtheta
            };
            let sideways = -SIDEWAYS * SIDEWAYS * s.along - 2.0 * SIDEWAYS * s.dalong;
            s.dr += radial * h;
            s.r += s.dr * h;
            s.dtheta += angular * h;
            s.theta += s.dtheta * h;
            s.dalong += sideways * h;
            s.along += s.dalong * h;
        }
    }

    /// The hips in the world now, hanging.
    fn hang_hips(&self) -> Vec3 {
        let s = &self.swing;
        let (out, along) = self.face();
        self.grip + along * s.along + out * (s.r * s.theta.sin()) - Vec3::Y * (s.r * s.theta.cos())
    }

    /// How far the hips are below standing as it jumps: the foot IK's drop,
    /// eased out over the jump's countermovement.
    fn sink(&self) -> f32 {
        match self.phase {
            Phase::Jumping => self.drop * (1.0 - smoothstep((self.jump.elapsed() / self.jump.ends(JumpPhase::Down).max(1.0e-3)).clamp(0.0, 1.0))),
            Phase::Hanging => 0.0,
        }
    }

    /// The pose now, on `rig` (the one it was measured on), in the walker's
    /// pose frame at [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self, rig: &RigGeometry) -> LocalPose {
        match (self.phase, self.up.as_ref()) {
            (Phase::Jumping, _) => self.jumping_pose(rig),
            (Phase::Hanging, Some(up)) => self.climbing_pose(up, rig),
            (Phase::Hanging, None) => self.hanging_pose(rig),
        }
    }

    /// Where the walker's root is now: under the hips as the standing pose
    /// has them.
    pub fn root(&self) -> Vec3 {
        match self.phase {
            Phase::Jumping => {
                let t = self.jump.elapsed();
                let pose = self.jump.pose_at(t, &self.body.stood, &self.rig);
                self.jump_root(t) + self.turn * (pose.root_translation - self.body.stood.root_translation) - Vec3::Y * self.sink()
            }
            Phase::Hanging => match self.up.as_ref() {
                Some(up) => self.up_root(up),
                None => self.hang_hips() - self.turn * self.body.hips,
            },
        }
    }

    /// The facing (radians about `+Y`): square to the wall.
    pub fn facing(&self) -> f32 {
        self.yaw
    }

    /// How closed each hand is on the lip, 0-1: closing as it arrives.
    pub fn grips(&self) -> [f32; 2] {
        if let Some(up) = self.up.as_ref() {
            return self.up_grips(up);
        }
        let to_catch = match self.phase {
            Phase::Jumping => self.catch_at - self.jump.elapsed(),
            Phase::Hanging => 0.0,
        };
        let closed = smoothstep((1.0 - to_catch / CLOSING).clamp(0.0, 1.0));
        [0, 1].map(|side| self.shimmy_grip(side).unwrap_or(closed))
    }

    /// Where it looks: at the lip, shimmying along it, and climbing up,
    /// ahead over the top.
    pub fn look(&self) -> Vec3 {
        match self.up.as_ref() {
            Some(up) => self.up_look(up),
            None => self.shimmy_look().unwrap_or(self.grip),
        }
    }

    /// Whether its hands hold the lip.
    pub fn is_hanging(&self) -> bool {
        self.phase == Phase::Hanging
    }

    /// Whether it hangs with its feet on the wall.
    pub fn is_braced(&self) -> bool {
        self.braced
    }

    /// Whether its feet are on the floor: jumping, until it leaves it.
    pub fn feet_down(&self) -> bool {
        self.phase == Phase::Jumping && !self.jump.airborne()
    }

    /// The ledge it holds or reaches for.
    pub fn ledge(&self) -> &Ledge {
        &self.ledge
    }

    /// The jump's pose, sunk by the foot IK's drop with the feet kept where
    /// the jump has them, and the arms reaching for the lip through the
    /// flight: the root moved to carry the hips, so the root rides them.
    fn jumping_pose(&self, rig: &RigGeometry) -> LocalPose {
        let t = self.jump.elapsed();
        let mut pose = self.jump.pose_at(t, &self.body.stood, rig);
        let shift = pose.root_translation - self.body.stood.root_translation;
        let feet = forward_kinematics_on(&pose, rig);
        pose.root_translation = self.body.stood.root_translation;
        // The feet kept on the floor where the jump has them, the hips sunk.
        let sink = self.sink();
        if sink > 0.0 {
            let back = self.turn.inverse();
            let hips = self.body.hips;
            for &(_, _, ankle, _) in &LEGS {
                // In the pose's frame at the moved root.
                let at = feet[ankle] - shift + back * (Vec3::Y * sink);
                place_ankle(&mut pose, rig, ankle, at - hips);
            }
        }
        // The arms up for the lip from the push's start, there as the hands
        // meet it, each wrist on the straight line from where it was then:
        // swung as the jump swings them, the hands went 16 cm into the wall.
        let from = self.jump.ends(JumpPhase::Down);
        if t > from {
            let reaching = smoothstep(((t - from) / (self.catch_at - from).max(1.0e-3)).clamp(0.0, 1.0));
            let root = self.root();
            let wrists = self.wrists();
            let targets = [0, 1].map(|side| self.reach_from[side].lerp(wrists[side], reaching));
            self.arms_to(&mut pose, rig, root, targets, [self.hook_turn(0), self.hook_turn(1)], [0.0; 2], reaching);
        }
        pose
    }

    /// The hang's pose: the trunk leant toward the grip, the legs braced on
    /// the wall or hanging (swinging there from where they were caught), the
    /// arms on the lip.
    fn hanging_pose(&self, rig: &RigGeometry) -> LocalPose {
        let hips = self.hang_hips();
        let root = hips - self.turn * self.body.hips;
        let back = self.turn.inverse();
        let lean = self.lean_at(hips);
        let mut pose = crate::character::anim::jump::upper(&self.body.stood, rig, lean, (0.0, 0.0));
        let trunk = self.lean_turn(hips, rig);
        // The legs: from where they were caught to the wall, or hanging.
        let moved = smoothstep((self.since / if self.braced { FEET_TO_WALL } else { LEGS_HANG }).clamp(0.0, 1.0));
        // Braced, off the wall going round a corner.
        let off_wall = self.feet_off_wall();
        for (side, &(_, _, ankle_bone, _)) in LEGS.iter().enumerate() {
            // Free: under the hips along the body, a little bent, the feet
            // leant with it.
            let hanging = || {
                let socket = hips + trunk * self.body.sockets[side];
                let down = (hips - self.grip).normalize_or(Vec3::NEG_Y);
                (socket + down * (FREE_LEG * self.body.legs[side]), trunk * self.body.attitudes[side])
            };
            let (target, attitude) = if self.braced {
                let attitude = self.toes_up(side);
                let on_wall = self.ball_now(side) + self.ankle_from_ball(side, attitude);
                if off_wall > 0.0 {
                    let (free, free_attitude) = hanging();
                    (on_wall.lerp(free, off_wall), attitude.slerp(free_attitude, off_wall))
                } else {
                    (on_wall, attitude)
                }
            } else {
                hanging()
            };
            let ankle = self.caught_ankles[side].lerp(target, moved);
            place_ankle(&mut pose, rig, ankle_bone, back * (ankle - root) - self.body.hips);
            let attitude = (self.turn * self.body.attitudes[side]).slerp(attitude, moved);
            let now = accumulate_world_rotations(&pose, rig)[ankle_bone];
            pose.rotations[ankle_bone] = delta_after_world_turn(&pose, rig, ankle_bone, (back * attitude) * now.inverse());
        }
        self.arms_to(&mut pose, rig, root, self.wrists(), [self.hook_turn(0), self.hook_turn(1)], self.elbows_tucked(), 1.0);
        pose
    }

    /// Where elbow `side` points hanging (the pose's frame): out to the side
    /// and back from the wall.
    fn hang_pole(&self, side: usize, rig: &RigGeometry) -> Vec3 {
        rig.left() * SIGN[side] * 0.7 + self.turn.inverse() * self.face().0 * 0.5 - Vec3::Y * 0.2
    }

    /// Where elbow `side` points pressing on the top (the pose's frame):
    /// back toward the hips, a little out. Pointing out as hanging, the
    /// elbows went 18.5 cm out past the shoulder-to-wrist line.
    fn press_pole(&self, side: usize, rig: &RigGeometry) -> Vec3 {
        rig.left() * SIGN[side] * PRESS_ELBOW_OUT + self.turn.inverse() * self.face().0
    }

    /// Each arm to its wrist at `wrists` (the world), turning its hand
    /// toward `turns` (the world) by `weight` (0 as the pose has it): the
    /// shoulder lifted toward it, the elbow out and back hanging, back
    /// pressing (`pressing`, 0-1 each).
    #[allow(clippy::too_many_arguments)]
    fn arms_to(&self, pose: &mut LocalPose, rig: &RigGeometry, root: Vec3, wrists: [Vec3; 2], turns: [Quat; 2], pressing: [f32; 2], weight: f32) {
        let back = self.turn.inverse();
        let to_pose = |p: Vec3| back * (p - root);
        let at = forward_kinematics_on(pose, rig);
        let targets = wrists.map(to_pose);
        for side in 0..2 {
            let lift = shoulder_lift(at[CLAVICLES[side]], at[ARMS[side].shoulder], targets[side], 0.85 * self.body.arms[side]);
            pose.rotations[CLAVICLES[side]] = delta_after_world_turn(pose, rig, CLAVICLES[side], lift);
        }
        let at = forward_kinematics_on(pose, rig);
        for side in 0..2 {
            let chain = ARMS[side];
            let pole = self.hang_pole(side, rig).lerp(self.press_pole(side, rig), pressing[side]).normalize();
            let (elbow, wrist) = solve_arm_toward_from(pose, &at, chain, targets[side], pole, rig);
            let wanted = back * turns[side];
            turn_hand(pose, rig, chain, self.body.hand_binds[side], wanted, weight, (wrist - elbow).normalize_or_zero());
        }
    }

    /// [`Self::pose`], each bone led ahead of its spring by how far the
    /// spring trails a steady motion (`jump::lead_of`): the grab as it will
    /// be that much later. The root is not sprung, so the rendered body is
    /// the grab's now.
    pub fn pose_led(&self, rig: &RigGeometry, springs: &BoneSet<SpringParams>) -> LocalPose {
        let mut pose = self.pose(rig);
        let mut posed: Vec<(f32, LocalPose)> = Vec::with_capacity(3);
        for bone in Bone::ALL {
            let lead = lead_of(&springs[bone]);
            if lead <= 1.0e-4 {
                continue;
            }
            pose.rotations[bone] = match posed.iter().find(|(at, _)| (at - lead).abs() < 1.0e-4) {
                Some((_, ahead)) => ahead.rotations[bone],
                None => {
                    let mut later = self.clone();
                    later.advance(lead);
                    let ahead = later.pose(rig);
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    pub(super) fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    const DT: f32 = 1.0 / 60.0;

    /// A wall 4 m wide, its face toward +Z at z = -0.5, `height` high, the
    /// wall reaching `below` down from its edge.
    pub(super) fn wall(height: f32, below: f32) -> Ledge {
        Ledge { wall_below: below, ..Ledge::wall(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 4.0, height, 1.0) }
    }

    /// A walker on its spot under `ledge`, grabbing it.
    pub(super) fn grabbing(ledge: &Ledge) -> Option<Hanging> {
        grabbing_among(ledge, &[], Vec3::new(0.2, 0.0, 1.0))
    }

    /// A walker on its spot under `ledge` from near `near`, among `others`
    /// (as the walker grabs), grabbing it.
    pub(super) fn grabbing_among(ledge: &Ledge, others: &[Ledge], near: Vec3) -> Option<Hanging> {
        let (stood, rig) = real_stood();
        let square = Hanging::square(ledge, rig.forward());
        let root = Hanging::spot(ledge, others, near, square, &stood, &rig);
        let mut hanging = Hanging::grab(ledge, root, square, 0.0, &stood, &rig)?;
        hanging.set_grips(crate::character::anim::hand::puppet_grips(), &stood, &rig);
        hanging.set_others(others);
        Some(hanging)
    }

    /// What a grab measured, frame by frame for `seconds`.
    #[derive(Debug, Default)]
    struct Measured {
        /// The most a hanging hand's wrist strays from its hook, metres; its
        /// lip point from the lip, metres; its palm from facing the face,
        /// radians.
        held: f32,
        lip_off: f32,
        palm_off: f32,
        /// The rig's own middle finger, hooked on the posed hand: the most its
        /// knuckle strays from the lip (up or down, in or out), metres; and
        /// the least its tip is both above the top and behind the face.
        knuckle_off: f32,
        tip_on_top: f32,
        /// The deepest any joint goes into the wall below the edge, metres.
        into_wall: f32,
        /// The hips' greatest acceleration from just before the hands meet
        /// the lip on, m/s² (the jump's own push before it is the jump's),
        /// and their speed at the end.
        catch_acceleration: f32,
        settled_speed: f32,
        /// The frame the hands met the lip.
        caught_at: usize,
        /// Braced: the most a foot's ball strays from the wall's face once on
        /// it, metres.
        feet_off: f32,
        /// Whether the hands caught the lip.
        caught: bool,
        /// The hips' path, frame by frame.
        hips: Vec<Vec3>,
    }

    fn grabbed(ledge: &Ledge, seconds: f32) -> Measured {
        let (_, rig) = real_stood();
        let mut hanging = grabbing(ledge).expect("in reach");
        let mut m = Measured { tip_on_top: f32::MAX, ..Default::default() };
        // Each hand's middle finger, hooked, in the hand's own frame.
        let middles = [crate::character::skeleton::Side::Left, crate::character::skeleton::Side::Right].map(|side| {
            let (bind, palm) = crate::character::anim::hand::puppet_finger(crate::character::anim::hand::Finger::Middle, side).expect("a middle finger");
            crate::character::anim::hand::finger_positions(&bind, crate::character::anim::hand::hooked(&bind, palm))
        });
        let frames = (seconds / DT).round() as usize;
        for _ in 0..frames {
            hanging.advance(DT);
            let pose = hanging.pose(&rig);
            let at = forward_kinematics_on(&pose, &rig);
            let turn = Quat::from_rotation_y(hanging.facing());
            let world = BoneSet::from_fn(|bone| hanging.root() + turn * at[bone]);
            m.hips.push(world[Bone::Hips]);
            if hanging.is_hanging() {
                if !m.caught {
                    m.caught_at = m.hips.len() - 1;
                }
                m.caught = true;
                let wrists = hanging.wrists();
                let rotations = accumulate_world_rotations(&pose, &rig);
                for side in 0..2 {
                    m.held = m.held.max((world[ARMS[side].wrist] - wrists[side]).length());
                    // The hand's own lip point on the lip, its palm to the face.
                    let hand = turn * rotations[ARMS[side].wrist] * hanging.body.hand_binds[side].inverse();
                    let grip = &hanging.body.grips[side];
                    m.lip_off = m.lip_off.max((world[ARMS[side].wrist] + hand * hook_lip(grip) - hanging.lips[side]).length());
                    m.palm_off = m.palm_off.max((hand * grip.palm).dot(-ledge.out).clamp(-1.0, 1.0).acos());
                    // The rig's own middle finger, hooked, on the hand as
                    // posed: its knuckle at the lip, its tip on the top.
                    let joints = &middles[side];
                    let placed = |p: Vec3| world[ARMS[side].wrist] + turn * rotations[ARMS[side].wrist] * p;
                    let (knuckle, tip) = (placed(joints[0]), placed(joints[3]));
                    m.knuckle_off = m.knuckle_off.max((knuckle.y - ledge.height()).abs().max(ledge.out_of(knuckle).abs()));
                    m.tip_on_top = m.tip_on_top.min((tip.y - ledge.height()).min(-ledge.out_of(tip)));
                }
                if hanging.is_braced() && hanging.since > FEET_TO_WALL {
                    for &(_, _, _, toe) in &LEGS {
                        m.feet_off = m.feet_off.max(ledge.out_of(world[toe]).abs());
                    }
                }
            }
            // Below the edge, where the wall is, nothing inside it.
            for bone in Bone::ALL {
                let p = world[bone];
                if p.y < ledge.height() - 0.02 && p.y > ledge.height() - ledge.wall_below {
                    m.into_wall = m.into_wall.max(-ledge.out_of(p));
                }
            }
        }
        m.catch_acceleration = m.hips[m.caught_at.saturating_sub(3)..].windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length()).fold(0.0, f32::max);
        m.settled_speed = m.hips.windows(2).last().map_or(0.0, |w| (w[1] - w[0]).length() / DT);
        m
    }

    /// A ledge 1.9-2.35 m up (`puppet_base`'s standing jump's reach, up and
    /// in) is caught and held: the hands hooked over the lip within a
    /// millimetre, the fingers over it, nothing through the wall, braced
    /// against a wall reaching the floor, free from a slab with none below;
    /// and the body comes to rest.
    #[test]
    fn a_ledge_in_reach_is_caught_and_hung_from() {
        for height in [1.95, 2.15, 2.35] {
            for (below, braced) in [(height, true), (0.15, false)] {
                let ledge = wall(height, below);
                let hanging = grabbing(&ledge).unwrap_or_else(|| panic!("{height} m: out of reach"));
                assert_eq!(hanging.is_braced(), braced, "{height} m, {below} m of wall: braced {}", hanging.is_braced());
                let m = grabbed(&ledge, if braced { 4.0 } else { 12.0 });
                assert!(m.caught, "{height} m: never caught");
                assert!(m.held < 1.0e-3, "{height} m braced {braced}: a wrist {:.4} m off its hook", m.held);
                assert!(m.lip_off < 1.0e-3 && m.palm_off < 0.02, "{height} m braced {braced}: a hand's lip {:.4} m off the lip, its palm {:.3} rad off the face", m.lip_off, m.palm_off);
                assert!(m.knuckle_off < 0.025 && m.tip_on_top > 0.0, "{height} m braced {braced}: a middle knuckle {:.4} m off the lip, its tip {:.4} m onto the top", m.knuckle_off, m.tip_on_top);
                assert!(m.into_wall < 1.0e-3, "{height} m braced {braced}: a joint {:.4} m into the wall", m.into_wall);
                assert!(m.settled_speed < 0.01, "{height} m braced {braced}: still moving at {:.3} m/s", m.settled_speed);
                if braced {
                    assert!(m.feet_off < 0.03, "{height} m: a foot {:.4} m off the wall", m.feet_off);
                }
            }
        }
    }

    /// The arms take the catch: the hips decelerate no harder than a soft
    /// landing's (3 body weights, Puddle and Maulder 2013), and the jump's
    /// path runs on into the hang without a jump in speed.
    #[test]
    fn the_catch_is_taken_softly() {
        for (below, name) in [(2.3, "braced"), (0.15, "free")] {
            let m = grabbed(&wall(2.3, below), 3.0);
            assert!(m.catch_acceleration < 3.0 * GRAVITY, "{name}: the hips accelerated {:.1} m/s² from the catch", m.catch_acceleration);
        }
    }

    /// Hanging free, the body swings about the grip as a compound pendulum:
    /// pushed out, its period is a person's, about 2.2-2.6 s.
    #[test]
    fn a_free_hang_swings_at_a_persons_period() {
        let mut hanging = grabbing(&wall(2.3, 0.15)).expect("in reach");
        hanging.advance(3.0);
        assert!(hanging.is_hanging());
        hanging.swing.theta = 0.2;
        hanging.swing.dtheta = 0.0;
        // Time between two crossings of straight below, the same way.
        let (mut t, mut crossings, mut last) = (0.0, Vec::new(), hanging.swing.theta);
        while crossings.len() < 3 && t < 10.0 {
            hanging.advance(1.0 / 480.0);
            t += 1.0 / 480.0;
            let now = hanging.swing.theta - hanging.rest.1;
            if last > 0.0 && now <= 0.0 {
                crossings.push(t);
            }
            last = now;
        }
        let period = crossings[2] - crossings[1];
        assert!((2.0..2.8).contains(&period), "a free hang swings in {period:.2} s");
    }

    /// Braced on a wall facing any way, each foot's ball is on its hold. The
    /// facing turn once went into the foot's offset twice, which a wall
    /// faced with a half turn (squaring to none) hid.
    #[test]
    fn braced_feet_are_on_their_holds_on_a_wall_facing_any_way() {
        let (_, rig) = real_stood();
        for out in [Vec3::Z, Vec3::X, Vec3::NEG_X, Vec3::new(1.0, 0.0, 1.0).normalize()] {
            let ledge = Ledge::wall(-out * 0.5, out, 4.0, 2.15, 1.0);
            let square = Hanging::square(&ledge, rig.forward());
            let root = Hanging::spot(&ledge, &[], out * 1.5, square, &real_stood().0, &rig);
            let mut hanging = Hanging::grab(&ledge, root, square, 0.0, &real_stood().0, &rig).expect("in reach");
            hanging.advance(4.0);
            assert!(hanging.is_braced());
            let pose = hanging.pose(&rig);
            let at = forward_kinematics_on(&pose, &rig);
            let turn = Quat::from_rotation_y(hanging.facing());
            for (side, &(_, _, _, toe)) in LEGS.iter().enumerate() {
                let off = (hanging.root() + turn * at[toe] - hanging.wall_balls[side]).length();
                assert!(off < 0.01, "a wall facing {out:?}: foot {side}'s ball {off:.4} m off its hold");
            }
        }
    }

    /// Too high for a standing jump, or low enough to reach standing, it is
    /// not grabbed.
    #[test]
    fn a_ledge_out_of_a_jumps_reach_or_reached_standing_is_not_grabbed() {
        assert!(grabbing(&wall(2.45, 2.45)).is_none(), "2.45 m grabbed");
        assert!(grabbing(&wall(1.85, 1.85)).is_none(), "1.85 m grabbed: a mantle");
    }
}
