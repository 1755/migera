//! Running up a wall: step 8 of the parkour design
//! (`docs/knowledge/character-animation/parkour/`), first part.
//!
//! Running at a wall, it takes off from one foot (a running leap,
//! [`Jump::from_run`]), plants the other on the face about a metre up, and
//! drives up off it for about a third of a second; then it flies on as a
//! fall ([`Falling`]) aimed at the wall's lip, which it catches if its hands
//! reach it, or lands at the wall's foot.
//!
//! Measured (`parkour-movement-data`): a wall climb comes in at 4.7 m/s, its
//! last ground step 1.17 m out, its first wall step 1.0 m up (Croft et al.
//! 2019). One foot's run up (Lawson 2015, not peer reviewed): the COM leaves
//! the floor rising 2.93 m/s, meets the wall 0.06 s later, leaves it 0.37 s
//! after that, 0.84 m higher, rising 1.55 m/s.
//!
//! - **Take-off**: the leap's own, its lead (free) foot brought onto its
//!   hold on the face as it meets it.
//! - **On the wall**: the hips on a Hermite curve from how the leap brought
//!   them to how they leave, the foot held on the face, the take-off leg's
//!   knee driven up, the arms swung up toward the lip.
//! - **Off it**: a fall from there at the hips' velocity, aimed at the lip
//!   and held off the wall.

use bevy::math::{Quat, Vec3};

use super::geometry::Ledge;
use super::Falling;
use crate::character::anim::armik::{solve_arm_toward_from, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::jump::{Jump, JumpAsk, JumpPhase, RunStart, ARMS_UP, GRAVITY};
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::anim::stance::{knee_toward, place_ankle};
use crate::character::skeleton::Bone;

/// Leaving the floor, the COM rises this fast, m/s (Lawson: 2.93).
const UP_SPEED: f32 = 2.9;
/// The take-off asked to land this far, toe to toe, metres: short, so it
/// brakes all a leap can.
const BRAKED_TO: f32 = 0.3;
/// From leaving the floor to the foot on the wall, seconds (Lawson: 0.06;
/// so short, at 4.5 m/s the lead foot came from the run's swing onto its
/// hold at 14 m/s about the hips).
const TO_WALL: f32 = 0.1;
/// The foot on the wall, seconds (Lawson: about 0.37).
const ON_WALL: f32 = 0.37;
/// The foot's ball on the face this high over the floor, metres (Croft:
/// the first wall step 1.0 m up).
const FOOT_UP: f32 = 1.0;
/// The foot pitched toes-up on the face, radians (as a braced hang's).
const TOES_UP: f32 = 1.1;
/// The lead foot comes onto its hold over this long before it meets the
/// wall, seconds (in 0.2 s, from a take-off 0.25 m far at 4.5 m/s, a toe
/// went 15 m/s about the hips).
const ONTO_WALL: f32 = 0.3;
/// The hips this far out from the face as the foot meets it, metres: where
/// the walker takes off from; and as they leave it, metres.
const HIPS_MEETING: f32 = 0.75;
const HIPS_LEAVING: f32 = 0.42;
/// The hips rise at most this much on the wall, metres (Lawson: the COM
/// 0.84); as far as the leg on the face reaches at this share of its
/// length.
const RISE: f32 = 0.84;
const LEAVE_LEG: f32 = 0.96;
/// The hold no farther from the socket than this share of the leg as the
/// foot meets it.
const MEETING_LEG: f32 = 0.98;
/// Leaving, pushed out from the face this fast, m/s (none: the fall's hold
/// off the wall keeps it out; pushed out 0.3 m/s, the hips drifted out of
/// the hands' reach of the lip); rising at least, and at most, this fast,
/// m/s (one foot's run up leaves at 1.55, Lawson; a hard one higher).
const PUSH_OUT: f32 = 0.0;
const LEAST_UP: f32 = 1.0;
const MOST_UP: f32 = 2.6;
/// The shoulders top out this far under the lip, metres (a leap's catch,
/// `hang::leap`).
const UNDER_LIP: f32 = 0.08;
/// The shoulders topping out this much lower still reach the lip, metres:
/// the hands reach 0.9 of the arm (0.45 m) from them, the hips 0.42 out
/// (measured: a 2.6 m wall caught, 2.7 m not).
const REACH_SLACK: f32 = 0.1;
/// The pose goes from the leap's to the wall's over this long, seconds.
const BLEND: f32 = 0.1;
/// On the wall the trunk leans this far toward it as the foot meets it,
/// upright leaving, radians; the arms swing up to this many times the
/// jump's arms up (nearly overhead): leant in 0.25 rad, the arms swung
/// forward-up 2.6 rad, a hand went 13-17 cm into the wall.
const LEAN: f32 = 0.0;
const ARMS: f32 = 1.45;
const ELBOWS_MID: f32 = 2.0;
/// The foot on the wall, its knee points up, this much out from the face
/// and (both knees) this much out to its own side for each unit up: up and
/// a little out from the face, at 4.5 m/s it went 2.7 cm in as the hips
/// came nearest.
const KNEE_OUT: f32 = 0.3;
const KNEE_ASIDE: f32 = 0.5;
/// The take-off leg's knee drives up: its ankle this far ahead of and
/// under the hips, metres, by this share of the time on the wall.
const DRIVE: (f32, f32) = (0.15, 0.45);
const DRIVE_BY: f32 = 0.7;
/// The driven ankle kept at least this far out from the face, metres; and
/// the wrists under the lip (a fall's `HANDS_OFF_WALL`).
const DRIVE_OFF: f32 = 0.25;
const HANDS_OFF: f32 = 0.1;
/// Leaving, the foot on the wall peels off it this fast, m/s.
const PEEL_OFF: f32 = 1.0;
/// The most the run meets the wall off square, radians.
pub const MOST_SLANT: f32 = 0.3;
/// The hips brake on the wall at most this hard, m/s² (3 g: a foot planted
/// on a wall at a run).
const MOST_BRAKE: f32 = 3.0 * GRAVITY;

const LEGS: [(Bone, Bone, Bone); 2] = [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot)];

/// Eased 0-1 over `span` from `from`.
fn ease(t: f32, from: f32, span: f32) -> f32 {
    smoothstep(((t - from) / span.max(1.0e-6)).clamp(0.0, 1.0))
}

/// A cubic Hermite from `a` (velocity `va`) to `b` (velocity `vb`) over
/// `span` seconds, at `t`: position and velocity.
fn hermite(a: Vec3, va: Vec3, b: Vec3, vb: Vec3, span: f32, t: f32) -> (Vec3, Vec3) {
    let s = (t / span).clamp(0.0, 1.0);
    let (s2, s3) = (s * s, s * s * s);
    let at = a * (2.0 * s3 - 3.0 * s2 + 1.0) + va * (span * (s3 - 2.0 * s2 + s)) + b * (3.0 * s2 - 2.0 * s3) + vb * (span * (s3 - s2));
    let rate = (a * (6.0 * s2 - 6.0 * s) + va * (span * (3.0 * s2 - 4.0 * s + 1.0)) + b * (6.0 * s - 6.0 * s2) + vb * (span * (3.0 * s2 - 2.0 * s))) / span;
    (at, rate)
}

/// The take-off from a run up a wall: a leap rising [`UP_SPEED`], asked to
/// land short ([`BRAKED_TO`]), so its plant leg brakes it as hard as a leap
/// does (`jump::leap::MOST_LOSS_PER_UP`). The wall met at the run's speed,
/// at 4.5 m/s the hips braked at 2.8 g on it; Lawson's run up meets the
/// wall at 2.35 m/s.
fn take_off(start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> Jump {
    Jump::from_run(JumpAsk::running(UP_SPEED * UP_SPEED / (2.0 * GRAVITY), BRAKED_TO), start, stood, rig)
}

/// A run up a wall, as planned at take-off.
#[derive(Debug, Clone)]
pub struct WallRun {
    /// The take-off leap (its frame: the pose's at `origin`, turned `turn`).
    jump: Jump,
    origin: Vec3,
    yaw: f32,
    turn: Quat,
    /// The wall (its lip and face), the foot IK's drop.
    wall: Ledge,
    drop: f32,
    stood: LocalPose,
    rig: std::sync::Arc<RigGeometry>,
    /// The leg whose foot goes on the wall (the lead).
    lead: usize,
    /// When the foot meets the wall and when it leaves, seconds into the
    /// jump; seconds into it now.
    contact: f32,
    leave: f32,
    t: f32,
    /// The leap's pose as the foot meets the wall; the hips then and their
    /// velocity, and leaving (the world).
    meeting: LocalPose,
    hips: (Vec3, Vec3),
    leaving: (Vec3, Vec3),
    /// The foot's ankle on the wall and its world rotation there.
    hold: (Vec3, Quat),
    /// The standing hips (the pose's frame).
    body_hips: Vec3,
}

impl WallRun {
    /// A run up `wall` (its lip ledge, its face reaching the floor) by a
    /// walker running at `start.speed`, its root at `origin` turned `yaw`
    /// as the foot of `start.leg` came down, its hips `drop` below
    /// standing: `None` if it runs at the wall more than [`MOST_SLANT`] off
    /// square, or the take-off is so far from or near the wall that its
    /// hips would not meet it right.
    #[allow(clippy::too_many_arguments)]
    pub fn plan(wall: &Ledge, origin: Vec3, yaw: f32, start: RunStart, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let turn = Quat::from_rotation_y(yaw);
        let forward = turn * rig.forward();
        if -forward.dot(wall.out) < MOST_SLANT.cos() {
            return None;
        }
        let jump = take_off(start, stood, rig);
        let contact = jump.ends(JumpPhase::Push) + TO_WALL;
        if contact >= jump.ends(JumpPhase::Flight) {
            return None;
        }
        let hips_at = |t: f32| origin + turn * (forward_kinematics_on(&jump.pose_at(t, stood, rig), rig)[Bone::Hips] + rig.forward() * jump.travelled_at(t));
        let meeting = jump.pose_at(contact, stood, rig);
        let meets = forward_kinematics_on(&meeting, rig);
        let meets_world = |bone: Bone| origin + turn * (meets[bone] + rig.forward() * jump.travelled_at(contact));
        let hips = meets_world(Bone::Hips);
        let velocity = (hips_at(contact + 1.0e-3) - hips_at(contact - 1.0e-3)) / 2.0e-3;
        // Met where the hips are near enough the wall, not into it.
        let out = wall.out_of(hips);
        if (out - HIPS_MEETING).abs() > MEETING_SLACK {
            return None;
        }
        let lead = 1 - start.leg;
        let stood_at = forward_kinematics_on(stood, rig);
        let stood_world = accumulate_world_rotations(stood, rig);
        // The foot's hold: its ball on the face under the lead socket, toes
        // up.
        let (socket, _, ankle) = LEGS[lead];
        let toe = crate::character::anim::foot::foot_bones(ankle).1;
        let along = wall.along();
        let lateral = (meets_world(socket) - wall.a).dot(along);
        let ball = wall.a + along * lateral + Vec3::Y * (origin.y + FOOT_UP - wall.a.y);
        let standing = turn * stood_world[ankle];
        let attitude = Quat::from_axis_angle((-wall.out).cross(Vec3::Y).normalize_or(along), TOES_UP) * standing;
        let ankle_at = ball + attitude * standing.inverse() * (turn * (stood_at[ankle] - stood_at[toe]));
        // The hold within the leg's reach as the foot meets it: taken off
        // 0.25 m far, the hips met the wall a metre out, the foot 1.7 cm
        // short of its hold.
        // The leg's length socket to ankle, straight in the rest pose: summed
        // by its segments, the synthetic rig's (shifted a joint: a 7 cm stub
        // for a shin) came out too short to reach any hold.
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let leg = (rest[ankle] - rest[socket]).length();
        if (meets_world(socket) - ankle_at).length() > MEETING_LEG * leg {
            return None;
        }
        // Leaving: the hips risen, nearer the wall, pushed out; rising as
        // fast as brings the hands to the lip near the top of the flight.
        let shoulders = 0.5 * (stood_at[Bone::LeftArm] + stood_at[Bone::RightArm]) - stood_at[Bone::Hips];
        // As high as the leg on its hold reaches (`LEAVE_LEG` of it), the
        // hips this far above the socket: rising Lawson's 0.84 m, the foot
        // was dragged off the face, the leg out of reach (the COM's rise is
        // helped by the ankle's push off the toes, which this pose has not).
        let socket_below = (stood_at[Bone::Hips] - stood_at[socket]).y;
        let socket_out = HIPS_LEAVING;
        let reach = LEAVE_LEG * leg;
        let reach_up = (reach * reach - socket_out * socket_out).max(0.0).sqrt();
        let rise = (ankle_at.y + reach_up + socket_below - hips.y).min(RISE);
        let leave_at = hips + wall.out * (HIPS_LEAVING - out) + Vec3::Y * rise;
        // The shoulders topping out just under the lip, as a leap's catch
        // plans (`hang::leap`): planned an arm's reach under it, the hands
        // never came within reach of it on the way down.
        let apex = wall.height() - UNDER_LIP - shoulders.y;
        let up = (2.0 * GRAVITY * (apex - leave_at.y).max(0.0)).sqrt().clamp(LEAST_UP, MOST_UP);
        let leaving = (leave_at, wall.out * PUSH_OUT + Vec3::Y * up);
        // Room on the wall to brake the run within `MOST_BRAKE`: taken off
        // 0.25 m nearer at 4.5 m/s, the hips braked at 3.5 g.
        let braking = (0..=40).map(|k| {
            let t = ON_WALL * k as f32 / 40.0;
            let (_, a) = hermite(hips, velocity, leaving.0, leaving.1, ON_WALL, t);
            let (_, b) = hermite(hips, velocity, leaving.0, leaving.1, ON_WALL, (t + 1.0e-3).min(ON_WALL));
            ((b - a) / 1.0e-3).length()
        });
        let most = braking.fold(0.0, f32::max);
        if most > MOST_BRAKE {
            return None;
        }
        Some(Self {
            jump,
            origin,
            yaw,
            turn,
            wall: *wall,
            drop,
            stood: *stood,
            rig: std::sync::Arc::new(rig.clone()),
            lead,
            contact,
            leave: contact + ON_WALL,
            t: 0.0,
            meeting,
            hips: (hips, velocity),
            leaving,
            hold: (ankle_at, attitude),
            body_hips: stood_at[Bone::Hips],
        })
    }

    /// How far before a wall's face (along the way of running) a run's
    /// foot best comes down to run up it from: its hips [`HIPS_MEETING`]
    /// out as the other foot meets the face.
    pub fn takeoff(start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        let jump = take_off(start, stood, rig);
        let contact = jump.ends(JumpPhase::Push) + TO_WALL;
        let hips = forward_kinematics_on(&jump.pose_at(contact, stood, rig), rig)[Bone::Hips];
        hips.dot(rig.forward()) + jump.travelled_at(contact) + HIPS_MEETING
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.t += dt;
        if self.t < self.contact {
            self.jump.advance(dt.min(self.contact - self.jump.elapsed()).max(0.0));
        }
    }

    /// Seconds into it.
    pub fn elapsed(&self) -> f32 {
        self.t
    }

    /// Whether its foot has left the wall: it flies on as a fall
    /// ([`Self::release`]).
    pub fn is_released(&self) -> bool {
        self.t >= self.leave
    }

    /// Whether its take-off foot is on the floor.
    pub fn feet_down(&self) -> [bool; 2] {
        if self.t < self.jump.ends(JumpPhase::Push) { [0, 1].map(|leg| leg != self.lead) } else { [false; 2] }
    }

    /// The facing (radians about `+Y`).
    pub fn facing(&self) -> f32 {
        self.yaw
    }

    /// The wall it runs up.
    pub fn wall(&self) -> &Ledge {
        &self.wall
    }

    /// The hips (the world) and their velocity on the wall at `t`.
    fn hips_on_wall(&self, t: f32) -> (Vec3, Vec3) {
        let ((a, va), (b, vb)) = (self.hips, self.leaving);
        hermite(a, va, b, vb, ON_WALL, t - self.contact)
    }

    /// The root now (the world).
    pub fn root(&self) -> Vec3 {
        self.root_at(self.t)
    }

    fn root_at(&self, t: f32) -> Vec3 {
        if t < self.contact {
            return self.origin + self.turn * (self.rig.forward() * self.jump.travelled_at(t));
        }
        self.hips_on_wall(t).0 - self.turn * self.body_hips
    }

    /// The pose now, on the rig it was planned on, in the walker's frame at
    /// [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self) -> LocalPose {
        self.pose_at(self.t)
    }

    fn pose_at(&self, t: f32) -> LocalPose {
        let rig = &*self.rig;
        let back = self.turn.inverse();
        if t < self.contact {
            // The leap's, its lead foot brought onto its hold.
            let mut pose = self.jump.pose_at(t, &self.stood, rig);
            // From no earlier than the take-off: a foot already part-way to
            // the hold at the first frame went 14 m/s.
            let from = (self.contact - ONTO_WALL).max(0.0);
            let w = ease(t, from, self.contact - from);
            if w > 0.0 {
                let root = self.root_at(t);
                let at = forward_kinematics_on(&pose, rig);
                let (socket, knee, ankle) = LEGS[self.lead];
                let hold = back * (self.hold.0 - root);
                place_ankle(&mut pose, rig, ankle, at[ankle].lerp(hold, w) - at[Bone::Hips]);
                // As on the wall: aimed ahead before and out after, the knee
                // turned round at 16 m/s as the foot met the face.
                knee_toward(&mut pose, rig, [socket, knee, ankle], self.wall_knee(), w);
                self.turn_foot(&mut pose, ankle, self.hold.1, w);
            }
            return pose;
        }
        let s = (t - self.contact) / ON_WALL;
        // The trunk upright from leaning in, the arms swung up, from the
        // leap's pose as the foot met the wall.
        // The elbows bent through the swing's middle, the hands coming up by
        // the body: swung up straight, mid-way they pointed at the wall and
        // went 7-12 cm into it.
        let (swing, elbow) = ARMS_UP;
        let s = s.clamp(0.0, 1.0);
        let arms = smoothstep(s) * ARMS;
        let bent = ELBOWS_MID * (std::f32::consts::PI * s).sin();
        let shaped = crate::character::anim::jump::upper(&self.stood, rig, LEAN * (1.0 - s), (swing * arms, elbow * arms + bent));
        let w = ease(t, self.contact, BLEND);
        let mut pose = shaped;
        for bone in Bone::ALL {
            pose.rotations[bone] = self.meeting.rotations[bone].slerp(shaped.rotations[bone], w);
        }
        pose.root_translation = self.stood.root_translation;
        let root = self.root_at(t);
        let hips = forward_kinematics_on(&pose, rig)[Bone::Hips];
        // The foot held on the wall, its knee up and a little out from it
        // (pointed toward it, it went 2.7 cm in).
        let (socket, knee, ankle) = LEGS[self.lead];
        place_ankle(&mut pose, rig, ankle, back * (self.hold.0 - root) - hips);
        knee_toward(&mut pose, rig, [socket, knee, ankle], self.wall_knee(), 1.0);
        self.turn_foot(&mut pose, ankle, self.hold.1, 1.0);
        // The take-off leg's knee driven up, from where the leap had it.
        let trail = 1 - self.lead;
        let (socket, knee, ankle) = LEGS[trail];
        let met = self.met_ankle(trail);
        let driven = self.hips_on_wall(t).0 + self.turn * (rig.forward() * DRIVE.0 - Vec3::Y * DRIVE.1);
        // Kept out from the face: taken off nearer, the hips came nearer it
        // and the driven knee went 2.7 cm in.
        let driven = driven + self.wall.out * (DRIVE_OFF - self.wall.out_of(driven)).max(0.0);
        let d = ease(t, self.contact, DRIVE_BY * ON_WALL);
        place_ankle(&mut pose, rig, ankle, back * (met.lerp(driven, d) - root) - hips);
        // Up and out to its side: ahead, it went 5 mm into the wall.
        let aside = rig.left() * if trail == 0 { 1.0 } else { -1.0 };
        knee_toward(&mut pose, rig, [socket, knee, ankle], Vec3::Y + aside * KNEE_ASIDE, d);
        // The hands kept off the face under the lip, as a fall keeps them
        // (taken off near, a hand swinging up went 2 cm in).
        let at = forward_kinematics_on(&pose, rig);
        for chain in [ArmChain::LEFT, ArmChain::RIGHT] {
            let wrist = root + self.turn * at[chain.wrist];
            let short = HANDS_OFF - self.wall.out_of(wrist);
            if short > 0.0 && wrist.y < self.wall.height() {
                let pole = (at[chain.elbow] - 0.5 * (at[chain.shoulder] + at[chain.wrist])).normalize_or(-rig.forward());
                solve_arm_toward_from(&mut pose, &at, chain, back * (wrist + self.wall.out * short - root), pole, rig);
            }
        }
        pose
    }

    /// Which way the foot on the wall points its knee (the pose's frame):
    /// up, a little out from the face and out to its side.
    fn wall_knee(&self) -> Vec3 {
        let rig = &*self.rig;
        let aside = rig.left() * if self.lead == 0 { 1.0 } else { -1.0 };
        Vec3::Y - rig.forward() * KNEE_OUT + aside * KNEE_ASIDE
    }

    /// Where leg `leg`'s ankle was as the foot met the wall (the world).
    fn met_ankle(&self, leg: usize) -> Vec3 {
        let at = forward_kinematics_on(&self.meeting, &self.rig);
        self.origin + self.turn * (at[LEGS[leg].2] + self.rig.forward() * self.jump.travelled_at(self.contact))
    }

    /// Turns `ankle`'s foot toward world rotation `attitude`, by `weight`.
    fn turn_foot(&self, pose: &mut LocalPose, ankle: Bone, attitude: Quat, weight: f32) {
        let rig = &*self.rig;
        let now = accumulate_world_rotations(pose, rig)[ankle];
        let wanted = self.turn.inverse() * attitude;
        let turn = Quat::IDENTITY.slerp(wanted * now.inverse(), weight);
        pose.rotations[ankle] = delta_after_world_turn(pose, rig, ankle, turn);
    }

    /// [`Self::pose`], each bone led ahead of its spring by how far the
    /// spring trails a steady motion (`jump::lead_of`), as the other moves
    /// pose: the run up as it will be that much later.
    pub fn pose_led(&self, springs: &crate::character::anim::rig::BoneSet<crate::character::anim::math::SpringParams>) -> LocalPose {
        let mut pose = self.pose();
        let mut posed: Vec<(f32, LocalPose)> = Vec::with_capacity(3);
        for bone in Bone::ALL {
            let lead = crate::character::anim::jump::lead_of(&springs[bone]);
            if lead <= 1.0e-4 {
                continue;
            }
            pose.rotations[bone] = match posed.iter().find(|(at, _)| (at - lead).abs() < 1.0e-4) {
                Some((_, ahead)) => ahead.rotations[bone],
                None => {
                    // Not past letting go: the fall poses itself from there.
                    let ahead = self.pose_at((self.t + lead).min(self.leave));
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }

    /// Whether its hands can reach the wall's lip from the top of its
    /// flight.
    pub fn reaches_lip(&self) -> bool {
        let rig = &*self.rig;
        let at = forward_kinematics_on(&self.stood, rig);
        let shoulders = 0.5 * (at[Bone::LeftArm] + at[Bone::RightArm]) - at[Bone::Hips];
        let (at, velocity) = self.leaving;
        let apex = at.y + velocity.y * velocity.y / (2.0 * GRAVITY);
        // The shoulders up to just under the lip, as planned.
        apex + shoulders.y >= self.wall.height() - UNDER_LIP - REACH_SLACK
    }

    /// Let go of the wall: the fall from here at the hips' velocity, aimed
    /// at the lip, held off the wall, landing on the ground `ground` finds.
    pub fn release(&self, ground: &dyn Fn(Vec3) -> Option<f32>) -> Falling {
        let rig = &*self.rig;
        let t = self.t.max(self.leave);
        let pose = self.pose_at(t);
        let root = self.root_at(t);
        let below = ground(root).unwrap_or(self.origin.y);
        let mut falling = Falling::off(root, self.yaw, self.leaving.1, &pose, below, self.drop, &self.stood, rig);
        // The legs and the trunk and arms as they move leaving.
        const READ: f32 = 0.01;
        let ankles = |t: f32| {
            let at = forward_kinematics_on(&self.pose_at(t), rig);
            LEGS.map(|(_, _, ankle)| self.turn * (at[ankle] - at[Bone::Hips]))
        };
        let (now, then) = (ankles(t), ankles(t - READ));
        // The foot on the wall peeling off it: coasting down the face as it
        // was held, its toes levelling went 1.4 cm into it.
        falling.coast_legs([0, 1].map(|side| (now[side] - then[side]) / READ + if side == self.lead { self.wall.out * PEEL_OFF } else { Vec3::ZERO }));
        let before = self.pose_at(t - READ);
        let mut ahead = pose;
        let on = 1.0 + Falling::COAST_AHEAD / READ;
        for bone in Bone::ALL {
            ahead.rotations[bone] = before.rotations[bone].slerp(pose.rotations[bone], on);
        }
        falling.coast_upper(ahead);
        falling.land_on(ground);
        falling.aim_at(self.wall);
        falling.against(&[self.wall], rig);
        falling
    }
}

/// How far the hips may be from [`HIPS_MEETING`] out as the foot meets the
/// wall, metres: the walker's take-off is a running step's spread off its
/// best.
const MEETING_SLACK: f32 = 0.35;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::BoneSet;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// What a run up a wall measured, frame by frame.
    #[derive(Debug, Default)]
    struct RanUp {
        /// The most the foot strays from its hold on the wall, metres.
        hold_off: f32,
        /// The deepest any joint goes into the wall (under its lip), metres,
        /// which and when.
        into: f32,
        deepest: Option<(Bone, f32)>,
        /// The fastest any joint moves, m/s, which and when.
        fastest: f32,
        fastest_bone: Option<(Bone, f32)>,
        /// The leap's own fastest before the wall, m/s.
        leap_fastest: f32,
        /// The hips' greatest acceleration on the wall, m/s².
        on_wall: f32,
        /// Caught the lip, and then the wrists' height under it; or landed.
        caught: bool,
        landed: bool,
        /// Whether the plan said the hands reach the lip.
        reaches: bool,
    }

    fn ran_up(height: f32, speed: f32, leg: usize, off: f32) -> Option<RanUp> {
        let (stood, rig) = real_stood();
        let start = RunStart { leg, speed };
        let wall = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 4.0, height, 1.0);
        let yaw = super::super::Hanging::square(&wall, rig.forward());
        let forward = Quat::from_rotation_y(yaw) * rig.forward();
        let origin = wall.a.with_y(0.0) - forward * (WallRun::takeoff(start, &stood, &rig) + off);
        let origin = Vec3::new(0.0, 0.0, origin.z);
        let mut run = WallRun::plan(&wall, origin, yaw, start, 0.0, &stood, &rig)?;
        let mut m = RanUp { reaches: run.reaches_lip(), ..Default::default() };
        let world = |pose: &LocalPose, root: Vec3, yaw: f32| {
            let at = forward_kinematics_on(pose, &rig);
            BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
        };
        let mut last: Option<BoneSet<Vec3>> = None;
        let mut hips = Vec::new();
        let mut t = 0.0;
        let measure = |now: BoneSet<Vec3>, t: f32, m: &mut RanUp, last: &mut Option<BoneSet<Vec3>>| {
            for bone in Bone::ALL {
                let p = now[bone];
                let into = -wall.out_of(p);
                if into > m.into && p.y < wall.height() && into < 1.0 {
                    (m.into, m.deepest) = (into, Some((bone, t)));
                }
                if let Some(before) = last.as_ref() {
                    // About the hips: over the ground a running foot already
                    // goes twice the run's speed.
                    let speed = ((p - now[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT;
                    if speed > m.fastest {
                        (m.fastest, m.fastest_bone) = (speed, Some((bone, t)));
                    }
                }
            }
            *last = Some(now);
        };
        let mut leap_last: Option<BoneSet<Vec3>> = None;
        while !run.is_released() {
            run.advance(DT);
            t += DT;
            let now = world(&run.pose(), run.root(), run.facing());
            if run.elapsed() >= run.contact && !run.is_released() {
                m.hold_off = m.hold_off.max((now[LEGS[run.lead].2] - run.hold.0).length());
                hips.push(now[Bone::Hips]);
            }
            // The leap's own, before the wall.
            if run.elapsed() < run.contact {
                let leap = world(&run.jump.pose_at(run.elapsed(), &stood, &rig), run.root(), run.facing());
                if let Some(before) = leap_last.as_ref() {
                    for bone in Bone::ALL {
                        m.leap_fastest = m.leap_fastest.max(((leap[bone] - leap[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT);
                    }
                }
                leap_last = Some(leap);
            }
            measure(now, t, &mut m, &mut last);
        }
        let mut falling = run.release(&|_| Some(0.0));
        loop {
            if falling.airborne() && falling.catches(&[wall], &rig).is_some() {
                m.caught = true;
                break;
            }
            if falling.is_done() {
                m.landed = true;
                break;
            }
            falling.advance(DT);
            t += DT;
            let now = world(&falling.pose(&rig), falling.root(), falling.facing());
            measure(now, t, &mut m, &mut last);
        }
        m.on_wall = hips.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length()).fold(0.0, f32::max);
        Some(m)
    }

    /// Running at 3.5 and 4.5 m/s at walls 2.3-3.4 m high, off either
    /// foot, from its best take-off and a quarter metre either side: the
    /// foot held on the face, nothing into the wall, no joint whipping round
    /// the hips, the hips braked on the wall within 3 g; it catches the lip
    /// up to 2.6 m, as planned, and higher lands at the wall's foot.
    #[test]
    fn it_runs_up_a_wall_and_catches_its_lip_or_lands() {
        for height in [2.3, 2.5, 2.6, 2.7, 3.4] {
            for speed in [3.5, 4.5] {
                for (leg, off) in [0, 1].into_iter().flat_map(|leg| [-0.25, -0.1, 0.0, 0.1, 0.25].map(|off| (leg, off))) {
                    let name = format!("{height} m wall, {speed} m/s, off the {} {off:+} m from its best", if leg == 0 { "left" } else { "right" });
                    // Farther than a tenth of a metre off its best, refused
                    // if there is no room to brake on the wall, or the hold
                    // is out of the leg's reach.
                    let Some(m) = ran_up(height, speed, leg, off) else {
                        assert!(off.abs() > 0.15, "{name}: not run up");
                        continue;
                    };
                    assert!(m.hold_off < 1.0e-3, "{name}: the foot {:.4} m off its hold", m.hold_off);
                    // The fall's own keep-off leaves a toe 1.6 mm in.
                    assert!(m.into < 2.0e-3, "{name}: {:?} {:.4} m into the wall", m.deepest, m.into);
                    // 14 m/s, by eye (a sprinter's swing foot goes about 10
                    // about the hips; pops were 16-42), or the leap's own.
                    assert!(m.fastest < 14.0_f32.max(m.leap_fastest + 0.01), "{name}: {:?} at {:.2} m/s about the hips, the leap's own {:.2}", m.fastest_bone, m.fastest, m.leap_fastest);
                    assert!(m.on_wall < 3.0 * GRAVITY, "{name}: the hips braked at {:.1} m/s² on the wall", m.on_wall);
                    assert_eq!(m.caught, m.reaches, "{name}: caught {}, planned to reach {}", m.caught, m.reaches);
                    assert_eq!(m.caught, height <= 2.6, "{name}: caught {}", m.caught);
                    assert!(m.caught || m.landed, "{name}: neither caught nor landed");
                }
            }
        }
    }

    /// Running at a wall too far off square, or taking off so far from or
    /// near it that the foot would not meet it right, it does not run up.
    #[test]
    fn a_wall_met_askew_or_from_the_wrong_spot_is_not_run_up() {
        let (stood, rig) = real_stood();
        let start = RunStart { leg: 0, speed: 4.0 };
        let wall = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 4.0, 2.5, 1.0);
        let yaw = super::super::Hanging::square(&wall, rig.forward());
        let best = WallRun::takeoff(start, &stood, &rig);
        let from = |off: f32| Vec3::new(0.0, 0.0, wall.a.z + best + off);
        assert!(WallRun::plan(&wall, from(0.0), yaw, start, 0.0, &stood, &rig).is_some(), "not run up from its best");
        assert!(WallRun::plan(&wall, from(0.0), yaw + 0.5, start, 0.0, &stood, &rig).is_none(), "run up 0.5 rad off square");
        assert!(WallRun::plan(&wall, from(1.0), yaw, start, 0.0, &stood, &rig).is_none(), "run up from 1 m too far");
        assert!(WallRun::plan(&wall, from(-0.6), yaw, start, 0.0, &stood, &rig).is_none(), "run up from 0.6 m too near");
    }
}
