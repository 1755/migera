//! A flagpole: step 17 of the parkour steps beyond the first ten (swinging
//! on fixtures). A pole sticking straight out of a wall, caught from a fall
//! or a jump coming by it; swung right round on it (the body a compound
//! pendulum about the grip, in the plane along the wall, driven over the
//! top as a gymnast's giant swing is); let go after the turn, going up and
//! forward, flung on as a fall.
//!
//! No flagpole data: the drive is set to bring a body caught at a run's
//! speed over the top in its first swing; the release is a bar release's
//! (`hang::swing`), early in the upswing.

use bevy::math::{Quat, Vec3};

use super::Falling;
use crate::character::anim::armik::{frame_turn, shoulder_lift, solve_arm_toward_from, turn_hand, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::hand::{HandGrip, FINGER_HALF_THICKNESS, GRIP_RADIUS};
use crate::character::anim::jump::{lead_of, GRAVITY};
use crate::character::anim::math::SpringParams;
use crate::character::anim::rig::{accumulate_bind_rotations, delta_after_world_turn, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// A flagpole's radius, metres.
pub const POLE_RADIUS: f32 = 0.025;

/// The hands hold no nearer either end than this, metres.
const END_MARGIN: f32 = 0.12;
/// How far the hands reach from their (lifted) shoulders, of the arm, and
/// the farthest an arm is solved to.
const REACH: f32 = 0.94;
const ARM_REACH: f32 = 0.98;
/// Caught: the hips within this much nearer or farther than hanging from
/// the pole, metres, and no more than this far round from straight below,
/// radians; the catch takes this long, seconds.
const CATCH_NEAR: f32 = 0.35;
const CATCH_FAR: f32 = 0.15;
const CATCH_ROUND: f32 = 1.2;
const CATCH: f32 = 0.4;
/// The body's radius of gyration about its centre of mass, metres (a free
/// hang's, `hang`), for the pendulum's length.
const GYRATION: f32 = 0.5;
/// How fast it goes over the top at least, rad/s, and the drive (rad²/s³
/// of the pendulum's energy a second) that gets it there.
const TOP_RATE: f32 = 2.0;
const DRIVE: f32 = 18.0;
/// How many turns it swings round, and how far past straight below on the
/// way up it lets go after them, radians.
const TURNS: f32 = 1.0;
const RELEASE: f32 = 0.6;
/// The legs piked ahead through the bottom by this, radians, at full speed.
const PIKE: f32 = 0.25;

const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
const LEGS: [(Bone, Bone); 2] = [(Bone::LeftUpLeg, Bone::LeftLeg), (Bone::RightUpLeg, Bone::RightLeg)];
const SIGN: [f32; 2] = [1.0, -1.0];
const GUESSED_KNUCKLES: f32 = 0.08;

/// The way arm `side`'s elbow bends, swinging (the body's frame, before it
/// turns round the pole): back from the body, a little out. Mostly out (as
/// a hang's), it lay along an arm reaching out along the pole as it was
/// caught, and the elbow flipped 37 cm in a frame.
fn elbow_way(side: usize, rig: &RigGeometry) -> Vec3 {
    (rig.left() * (SIGN[side] * 0.3) - rig.forward()).normalize()
}

/// An arm's swivel from its elbow at `elbow` to the way `pole`, about its
/// line from `shoulder` to `wrist`: the line, the elbow's way square to it,
/// and the angle from that to the pole's.
fn swivel(shoulder: Vec3, wrist: Vec3, elbow: Vec3, pole: Vec3) -> (Vec3, Vec3, f32) {
    let line = (wrist - shoulder).normalize_or(Vec3::Y);
    let to = swivel_to(shoulder, wrist, pole);
    let from = (elbow - shoulder - line * (elbow - shoulder).dot(line)).normalize_or(to);
    (line, from, line.dot(from.cross(to)).atan2(from.dot(to)))
}

/// The way `pole` square to an arm's line from `shoulder` to `wrist`.
fn swivel_to(shoulder: Vec3, wrist: Vec3, pole: Vec3) -> Vec3 {
    let line = (wrist - shoulder).normalize_or(Vec3::Y);
    (pole - line * pole.dot(line)).normalize_or(pole)
}

/// A flagpole: where its axis leaves the wall (the world), the way out
/// from the wall (level), and how long, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Flagpole {
    pub base: Vec3,
    pub out: Vec3,
    pub length: f32,
}

impl Flagpole {
    pub fn new(base: Vec3, out: Vec3, length: f32) -> Self {
        Self { base, out: Vec3::new(out.x, 0.0, out.z).normalize_or(Vec3::X), length }
    }

    /// The point `s` metres out along its axis.
    pub fn at(&self, s: f32) -> Vec3 {
        self.base + self.out * s
    }
}

/// The body as the standing pose has it, measured once.
#[derive(Debug, Clone)]
struct Body {
    stood: LocalPose,
    hips: Vec3,
    arms: [f32; 2],
    shoulders: [Vec3; 2],
    hand_binds: [Quat; 2],
    grips: [HandGrip; 2],
}

impl Body {
    fn of(stood: &LocalPose, rig: &RigGeometry) -> Self {
        let at = forward_kinematics_on(stood, rig);
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let binds = accumulate_bind_rotations(rig);
        Self {
            stood: *stood,
            hips: at[Bone::Hips],
            arms: ARMS.map(|arm| (at[arm.elbow] - at[arm.shoulder]).length() + (at[arm.wrist] - at[arm.elbow]).length()),
            shoulders: ARMS.map(|arm| at[arm.shoulder]),
            hand_binds: ARMS.map(|arm| binds[arm.wrist]),
            grips: ARMS.map(|arm| {
                let along = (rest[arm.wrist] - rest[arm.elbow]).normalize_or(Vec3::NEG_Y);
                let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
                HandGrip { bar: along * GUESSED_KNUCKLES + palm * (GRIP_RADIUS + FINGER_HALF_THICKNESS), palm, along }
            }),
        }
    }

    /// How far the hips hang under the pole's axis, metres.
    fn hang(&self) -> f32 {
        let shoulders = 0.5 * (self.shoulders[0] + self.shoulders[1]) - self.hips;
        shoulders.y + REACH * 0.5 * (self.arms[0] + self.arms[1]) + self.grips[0].bar.length()
    }

    fn half_width(&self) -> f32 {
        0.5 * (self.shoulders[0] - self.shoulders[1]).length()
    }
}

/// What a fall caught on the pole came with.
#[derive(Debug, Clone)]
struct Caught {
    /// The hips' distance from the axis, the pose, the facing, each wrist
    /// from the hips (the world's axes), and how far round it was caught.
    reach: f32,
    pose: LocalPose,
    yaw: f32,
    wrists: [Vec3; 2],
    phi: f32,
    /// Each elbow from the hips (the world's axes).
    elbows: [Vec3; 2],
    /// How fast the hips came in toward the pole's axis (negative) or out.
    radial: f32,
    /// Each arm's swivel from the fall's elbow to the swing's, as caught
    /// (later turns are taken the same way round).
    swivels: [f32; 2],
}

/// Swinging round a flagpole ([`Flagpole`]).
#[derive(Debug, Clone)]
pub struct Swinging {
    pole: Flagpole,
    body: Body,
    rig: RigGeometry,
    /// Where along the pole the hands hold, metres out.
    along: f32,
    /// The way it swings forward (level, square to the pole), its facing
    /// heading, and the pole's axis it turns about (`ahead × Y`).
    ahead: Vec3,
    yaw: f32,
    axis: Vec3,
    /// How far round from straight below (radians, unwrapped, positive
    /// toward `ahead`) and how fast.
    phi: f32,
    omega: f32,
    /// Seconds since the catch.
    t: f32,
    caught: Option<Box<Caught>>,
    released: bool,
}

impl Swinging {
    /// Caught from a fall: the hips at `hips` going at `velocity`, posed
    /// `pose`, facing `yaw`. `None` unless the hips are about as far from
    /// the pole's axis as they hang (within [`CATCH_NEAR`] nearer and
    /// [`CATCH_FAR`] farther), below it no more than [`CATCH_ROUND`] round,
    /// and along its length.
    #[allow(clippy::too_many_arguments)]
    pub fn caught(pole: &Flagpole, hips: Vec3, velocity: Vec3, pose: &LocalPose, yaw: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let body = Body::of(stood, rig);
        let hang = body.hang();
        let s = (hips - pole.base).dot(pole.out);
        let margin = END_MARGIN + body.half_width();
        if s < margin || s > pole.length - margin {
            return None;
        }
        let grip = pole.at(s);
        let off = hips - grip;
        let level = off - pole.out * off.dot(pole.out);
        let reach = level.length();
        if reach < hang - CATCH_NEAR || reach > hang + CATCH_FAR {
            return None;
        }
        // Forward is the way it is going across the pole (or facing, slow).
        let square = Vec3::Y.cross(pole.out).normalize();
        let going = velocity - pole.out * velocity.dot(pole.out);
        let facing = Quat::from_rotation_y(yaw) * rig.forward();
        let way = if going.with_y(0.0).length() > 0.3 { going.dot(square).signum() } else { facing.dot(square).signum() };
        let ahead = square * if way == 0.0 { 1.0 } else { way };
        let phi = level.dot(ahead).atan2(-level.y);
        if phi.abs() > CATCH_ROUND {
            return None;
        }
        let tangent = Vec3::Y * phi.sin() + ahead * phi.cos();
        let mut fall_pose = *pose;
        fall_pose.root_translation += body.hips - forward_kinematics_on(pose, rig)[Bone::Hips];
        let at = forward_kinematics_on(&fall_pose, rig);
        let wrists = ARMS.map(|arm| Quat::from_rotation_y(yaw) * (at[arm.wrist] - at[Bone::Hips]));
        let elbows = ARMS.map(|arm| Quat::from_rotation_y(yaw) * (at[arm.elbow] - at[Bone::Hips]));
        let axis = ahead.cross(Vec3::Y).normalize();
        // Each arm's swivel as caught, with the pose's own geometry then
        // (facing as the fall did, its wrists and elbows where it had them).
        let turn = Quat::from_rotation_y(yaw);
        let back = turn.inverse();
        let swung = Quat::from_axis_angle(axis, phi);
        let swivels = [0, 1].map(|side| {
            let shoulder = hips + turn * (Quat::from_axis_angle(back * axis, phi) * (body.shoulders[side] - body.hips));
            let pole = swung * (turn * elbow_way(side, rig));
            swivel(shoulder, hips + wrists[side], hips + elbows[side], pole).2
        });
        Some(Self {
            pole: *pole,
            rig: rig.clone(),
            along: s,
            ahead,
            yaw: crate::character::anim::approach::heading_of(ahead) - crate::character::anim::approach::heading_of(rig.forward()),
            axis,
            phi,
            omega: velocity.dot(tangent) / reach.max(0.3),
            t: 0.0,
            caught: Some(Box::new(Caught { reach, pose: fall_pose, yaw, wrists, phi, elbows, radial: velocity.dot(level / reach), swivels })),
            released: false,
            body,
        })
    }

    /// Each hand placed so its own fingers close round the pole.
    pub fn set_grips(&mut self, grips: [Option<HandGrip>; 2]) {
        for (side, grip) in crate::character::anim::hand::bound_grips(grips, &self.rig).into_iter().enumerate() {
            if let Some(grip) = grip {
                self.body.grips[side] = grip;
            }
        }
    }

    /// The pendulum's length (a compound pendulum's, about the grip).
    fn length(&self) -> f32 {
        let d = self.body.hang();
        (GYRATION * GYRATION + d * d) / d
    }

    /// Its energy, per the pendulum's inertia: rad²/s².
    fn energy(&self) -> f32 {
        0.5 * self.omega * self.omega + GRAVITY / self.length() * (1.0 - self.phi.cos())
    }

    /// Moves it on `dt` seconds: driven until it will go over the top fast
    /// enough, through its turns; let go past them on the way up.
    pub fn advance(&mut self, dt: f32) {
        if self.released {
            return;
        }
        let g = GRAVITY / self.length();
        let wanted = 0.5 * TOP_RATE * TOP_RATE + 2.0 * g;
        let steps = (dt / (1.0 / 240.0)).ceil().max(1.0) as usize;
        let h = dt / steps as f32;
        let end = std::f32::consts::TAU * TURNS + RELEASE;
        for _ in 0..steps {
            let driving = self.phi < std::f32::consts::TAU * TURNS && self.energy() < wanted;
            let drive = if driving { DRIVE / self.omega.abs().max(1.0) * self.omega.signum().max(0.0).max(if self.omega == 0.0 { 1.0 } else { 0.0 }) } else { 0.0 };
            self.omega += (-g * self.phi.sin() + drive) * h;
            self.phi += self.omega * h;
            self.t += h;
            // Let go at the frame's end: stopped mid-frame, the frame it let
            // go in went on less than a frame's time, and a toe swinging at
            // 12 m/s stepped 9 cm short.
            if self.phi >= end && self.omega > 0.0 {
                self.released = true;
            }
        }
    }

    /// The hips' distance from the axis now: from where caught to hanging,
    /// over the catch.
    fn reach(&self) -> f32 {
        let hang = self.body.hang();
        // Coming in or out as it was, to rest at the hang (eased from rest,
        // the radial speed it came with was dropped in a frame).
        match self.caught.as_ref() {
            Some(caught) => crate::character::anim::gait::hermite(caught.reach, hang, caught.radial * CATCH, 0.0, (self.t / CATCH).clamp(0.0, 1.0)),
            None => hang,
        }
    }

    /// Catching, how far through it (0-1, eased).
    fn catching(&self) -> f32 {
        smoothstep((self.t / CATCH).clamp(0.0, 1.0))
    }

    fn grip(&self) -> Vec3 {
        self.pole.at(self.along)
    }

    /// The hips now (the world).
    fn hips(&self) -> Vec3 {
        self.grip() + Quat::from_axis_angle(self.axis, self.phi) * (Vec3::NEG_Y * self.reach())
    }

    /// The hips' velocity now.
    pub fn hips_velocity(&self) -> Vec3 {
        let r = Quat::from_axis_angle(self.axis, self.phi) * (Vec3::NEG_Y * self.reach());
        (self.axis * self.omega).cross(r)
    }

    /// The facing turn now: caught, from the fall's to along the swing.
    fn turn(&self) -> Quat {
        let yaw = match self.caught.as_ref() {
            Some(caught) => caught.yaw + crate::character::anim::facing::shortest_angle(self.yaw - caught.yaw) * self.catching(),
            None => self.yaw,
        };
        Quat::from_rotation_y(yaw)
    }

    /// The walker's facing now.
    pub fn facing(&self) -> f32 {
        let (axis, angle) = self.turn().to_axis_angle();
        angle * axis.y.signum()
    }

    /// Where the walker's root is now.
    pub fn root(&self) -> Vec3 {
        self.hips() - self.turn() * self.body.hips
    }

    /// The pose now, on `rig`, in the walker's frame at [`Self::root`]
    /// turned [`Self::facing`].
    pub fn pose(&self, rig: &RigGeometry) -> LocalPose {
        self.posed(rig).0
    }

    /// [`Self::pose`], and where it put each wrist (the world).
    fn posed(&self, rig: &RigGeometry) -> (LocalPose, [Vec3; 2]) {
        let hips = self.hips();
        let turn = self.turn();
        let root = hips - turn * self.body.hips;
        let back = turn.inverse();
        // The body turned whole about the pole's axis.
        let mut pose = self.body.stood;
        let swing = Quat::from_axis_angle(back * self.axis, self.phi);
        pose.rotations[Bone::Hips] = delta_after_world_turn(&pose, rig, Bone::Hips, swing);
        // The legs piked ahead through the bottom, by the swing's speed.
        let pike = PIKE * (self.omega / 8.0).clamp(-1.0, 1.0) * self.phi.cos().max(0.0);
        for (thigh, _) in LEGS {
            pose.rotations[thigh] = delta_after_world_turn(&pose, rig, thigh, Quat::from_axis_angle(back * self.axis, pike));
        }
        // The hands round the pole, a shoulder's width apart, the palm the
        // way the body faces round it, the fingers on along the forearm.
        let points = [0, 1].map(|side| self.pole.at(self.along + SIGN[side] * self.body.half_width() * self.pole.out.dot(turn * rig.left()).signum()));
        let palm = Quat::from_axis_angle(self.axis, self.phi) * self.ahead;
        let shoulders = [0, 1].map(|side| hips + turn * (Quat::from_axis_angle(back * self.axis, self.phi) * (self.body.shoulders[side] - self.body.hips)));
        // Caught, the fall's pose eased out but for the arms, which are
        // solved throughout, each elbow bending first toward where the
        // fall had it, then the swing's way. Blended by turn, an elbow
        // flipped 20 cm in a frame half way through; solved to its moving
        // wrist alone, it took the swing's side at once (28 cm).
        let swung = Quat::from_axis_angle(self.axis, self.phi);
        let swing_poles = [0, 1].map(|side| swung * (turn * elbow_way(side, rig)));
        let mut fall_elbows = None;
        if let Some(caught) = self.caught.as_ref()
            && self.catching() < 1.0
        {
            let s = self.catching();
            let arms = [CLAVICLES[0], CLAVICLES[1], ARMS[0].shoulder, ARMS[1].shoulder, ARMS[0].elbow, ARMS[1].elbow, ARMS[0].wrist, ARMS[1].wrist];
            let blended = crate::character::anim::clip::blend(&caught.pose, &pose, s);
            for bone in Bone::ALL.into_iter().filter(|bone| !arms.contains(bone)) {
                pose.rotations[bone] = blended.rotations[bone];
            }
            let carried = Quat::from_axis_angle(self.axis, self.phi - caught.phi);
            fall_elbows = Some(([0, 1].map(|side| hips + carried * caught.elbows[side]), s));
        }
        // Each elbow's way: square to its arm's line, turned about that
        // line from where the fall had it to the swing's (blended straight
        // across, it passed near the line and an elbow swung 8 cm), the
        // turn kept the way it began (near half a turn, it flipped sides
        // half way and an elbow swung 42 cm).
        let poles_for = |wrists: &[Vec3; 2]| {
            [0, 1].map(|side| {
                // The swing's way turned back about the arm's line by the
                // swivel as caught, the less the further into the catch
                // (taken from the fall's elbow carried with the body each
                // frame, it passed near the line and the swivel jumped
                // 2.5 rad in a frame).
                let to = swivel_to(shoulders[side], wrists[side], swing_poles[side]);
                let pole = match (fall_elbows, self.caught.as_ref()) {
                    (Some((_, s)), Some(caught)) => {
                        let line = (wrists[side] - shoulders[side]).normalize_or(Vec3::Y);
                        Quat::from_axis_angle(line, -caught.swivels[side] * (1.0 - s)) * to
                    }
                    _ => to,
                };
                back * pole
            })
        };
        let unarmed = pose;
        let mut froms = shoulders;
        let mut wrists = [Vec3::ZERO; 2];
        for pass in 0..2 {
            pose = unarmed;
            let mut turns = [Quat::IDENTITY; 2];
            for side in 0..2 {
                let grip = &self.body.grips[side];
                let along = (points[side] - froms[side]).normalize_or(-palm.cross(self.axis));
                let hand = frame_turn(grip.along, grip.palm, along, palm);
                let on = points[side] - hand * grip.bar;
                // Caught: from where the fall had them, carried with the
                // hips, to the pole (put there at once, an arm swung 98 cm
                // in a frame).
                // Swept round the shoulder, not straight across: a hand by
                // the thigh going straight to the pole overhead passed its
                // shoulder and the elbow flipped (as the pole's get-on).
                wrists[side] = match self.caught.as_ref() {
                    Some(caught) => {
                        let s = self.catching();
                        let shoulder = shoulders[side];
                        // The fall's wrists carried round with the body as
                        // it swings (left on the world's axes, they slid
                        // across it, a hand 8 cm a frame).
                        let carried = Quat::from_axis_angle(self.axis, self.phi - caught.phi) * caught.wrists[side];
                        let (from, to) = (hips + carried - shoulder, on - shoulder);
                        let arc = Quat::IDENTITY.slerp(Quat::from_rotation_arc(from.normalize_or(Vec3::Y), to.normalize_or(Vec3::Y)), s);
                        shoulder + arc * from.normalize_or(Vec3::Y) * (from.length() + (to.length() - from.length()) * s)
                    }
                    None => on,
                };
                turns[side] = hand;
            }
            let poles = poles_for(&wrists);
            self.arms_to(&mut pose, rig, root, back, wrists, turns, poles, self.catching());
            if pass == 0 {
                let at = forward_kinematics_on(&pose, rig);
                froms = [0, 1].map(|side| root + turn * at[ARMS[side].elbow]);
            }
        }
        (pose, wrists)
    }

    /// Each arm to its wrist (the world), its elbow bending toward `poles`
    /// (the pose's frame), the hand turned to `turns`, both by `weight`
    /// (the shoulder's lift and the hand's turn).
    #[allow(clippy::too_many_arguments)]
    fn arms_to(&self, pose: &mut LocalPose, rig: &RigGeometry, root: Vec3, back: Quat, wrists: [Vec3; 2], turns: [Quat; 2], poles: [Vec3; 2], weight: f32) {
        let targets = wrists.map(|p| back * (p - root));
        let at = forward_kinematics_on(pose, rig);
        for side in 0..2 {
            let lift = Quat::IDENTITY.slerp(shoulder_lift(at[CLAVICLES[side]], at[ARMS[side].shoulder], targets[side], 0.85 * self.body.arms[side]), weight);
            pose.rotations[CLAVICLES[side]] = delta_after_world_turn(pose, rig, CLAVICLES[side], lift);
        }
        let at = forward_kinematics_on(pose, rig);
        for side in 0..2 {
            let chain = ARMS[side];
            let pole = poles[side];
            let off = targets[side] - at[chain.shoulder];
            let most = ARM_REACH * self.body.arms[side];
            let target = if off.length() > most { at[chain.shoulder] + off * (most / off.length()) } else { targets[side] };
            let (elbow, wrist) = solve_arm_toward_from(pose, &at, chain, target, pole, rig);
            turn_hand(pose, rig, chain, self.body.hand_binds[side], back * turns[side], weight, (wrist - elbow).normalize_or_zero());
        }
    }

    /// [`Self::pose`], each bone led ahead of its spring.
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

    /// Each wrist's place on the pole now (the world).
    pub fn wrists(&self, rig: &RigGeometry) -> [Vec3; 2] {
        self.posed(rig).1
    }

    /// How closed the hands are (closing over the catch, open let go).
    pub fn grips(&self) -> [f32; 2] {
        [if self.released { 0.0 } else { self.catching() }; 2]
    }

    /// Where it looks: ahead along the swing.
    pub fn look(&self) -> Vec3 {
        self.grip() + self.ahead * 3.0
    }

    /// How far round it has swung since straight below, radians.
    pub fn swung(&self) -> f32 {
        self.phi
    }

    /// Whether it has let go.
    pub fn is_released(&self) -> bool {
        self.released
    }

    /// Let go: the fall from the pose now, flung on at the hips' velocity,
    /// to the ground `ground` finds under it.
    pub fn release(&self, ground: &dyn Fn(Vec3) -> Option<f32>, floor: f32, stood: &LocalPose, rig: &RigGeometry) -> Falling {
        let root = self.root();
        let below = ground(root.with_y(root.y.min(floor + 0.05))).map_or(floor, |h| h.min(floor.max(h)));
        let mut falling = Falling::off(root, self.facing(), self.hips_velocity(), &self.pose(rig), below, 0.0, stood, rig);
        falling.land_on(ground);
        falling
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    fn world(pose: &LocalPose, root: Vec3, yaw: f32, rig: &RigGeometry) -> BoneSet<Vec3> {
        let at = forward_kinematics_on(pose, rig);
        BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
    }

    /// Run off a 1.5 m top at 3 and 4 m/s toward a flagpole 2.6-2.8 m up
    /// sticking out across the way, reaching: caught as the hips come by
    /// it, swung right round once (over the top at 1 rad/s at least), let
    /// go on the way up and on, and landed on the floor ahead; the hands on
    /// the pole within 1 mm after the catch, the wrists not bent back past
    /// 0.6 rad; every pose finite; no joint's step changing over 3 cm in a
    /// frame swinging, 7 cm as caught, 8 cm catching (the arms coming in
    /// from the fall's reach as the body swings at 4 rad/s).
    #[test]
    fn a_flagpole_is_caught_swung_round_and_let_go_of() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        let grips = crate::character::anim::hand::puppet_grips();
        let mut faults = Vec::new();
        for (speed, height, ahead) in [(3.0f32, 2.6f32, 0.9f32), (4.0, 2.8, 1.2), (3.0, 2.8, 0.8)] {
            let name = format!("{speed} m/s, pole {height} m up {ahead} m out");
            let top = 1.5;
            let pole = Flagpole::new(Vec3::new(0.75, top + height - 1.5, 0.0) + forward * ahead, Vec3::NEG_X, 1.5);
            let mut falling = Falling::off(Vec3::Y * top, 0.0, forward * speed, &stood, 0.0, 0.0, &stood, &rig);
            // Reaching up to catch, as a walker asked to catches.
            falling.reach(true);
            let mut frames: Vec<BoneSet<Vec3>> = vec![world(&stood, Vec3::Y * top, 0.0, &rig)];
            let mut swinging = None;
            for _ in 0..120 {
                falling.advance(DT);
                frames.push(world(&falling.pose(&rig), falling.root(), falling.facing(), &rig));
                if let Some(caught) = Swinging::caught(&pole, falling.hips(), falling.hips_velocity(), &falling.pose(&rig), falling.facing(), &stood, &rig) {
                    swinging = Some(caught);
                    break;
                }
            }
            let Some(mut swinging) = swinging else {
                faults.push(format!("{name}: never caught"));
                continue;
            };
            swinging.set_grips(grips);
            let (mut hand_off, mut flexed, mut kink, mut catch_kink, mut top_rate) = (0.0f32, 0.0f32, 0.0f32, 0.0f32, f32::MAX);
            let caught_at = frames.len();
            let mut kink_at = (0.0f32, 0.0f32);
            let mut catching_kink = 0.0f32;
            let mut t = 0.0;
            while !swinging.is_released() && t < 10.0 {
                swinging.advance(DT);
                t += DT;
                let pose = swinging.pose(&rig);
                if !Bone::ALL.iter().all(|&b| pose.rotations[b].is_finite()) {
                    faults.push(format!("{name}: NaN"));
                    break;
                }
                let now = world(&pose, swinging.root(), swinging.facing(), &rig);
                let n = frames.len();
                let step = Bone::ALL.iter().map(|&b| (now[b] - 2.0 * frames[n - 1][b] + frames[n - 2][b]).length()).fold(0.0, f32::max);
                if n < caught_at + 3 {
                    catch_kink = catch_kink.max(step)
                } else if t < CATCH {
                    catching_kink = catching_kink.max(step);
                } else if step > kink {
                    kink = step;
                    kink_at = (t, swinging.swung());
                }
                if t > CATCH {
                    for (side, wrist) in swinging.wrists(&rig).into_iter().enumerate() {
                        hand_off = hand_off.max((now[ARMS[side].wrist] - wrist).length());
                    }
                    for (_, flex) in crate::character::anim::hand::wrist_bend(&pose, &rig, &grips) {
                        flexed = flexed.min(flex);
                    }
                }
                if (swinging.swung() - std::f32::consts::PI).abs() < 0.2 {
                    top_rate = top_rate.min(swinging.omega);
                }
                frames.push(now);
            }
            let swung = swinging.swung();
            let mut falling = swinging.release(&|_| Some(0.0), 0.0, &stood, &rig);
            let released = falling.hips_velocity();
            let mut fell = 0.0;
            while !falling.is_done() && fell < 10.0 {
                falling.advance(DT);
                fell += DT;
            }
            eprintln!(
                "{name}: swung {swung:.2} rad in {t:.2} s, over the top at {top_rate:.2} rad/s, let go at {released:?}, hand off {hand_off:.5}, flexed {flexed:.2}, kink {kink:.4} at {kink_at:?} (catch {catch_kink:.4}, catching {catching_kink:.4}), landed {:?}",
                falling.root()
            );
            if !swinging.is_released() || swung < std::f32::consts::TAU {
                faults.push(format!("{name}: swung {swung:.2} rad, not round"));
            }
            if top_rate < 1.0 {
                faults.push(format!("{name}: over the top at {top_rate:.2} rad/s"));
            }
            if hand_off > 1.0e-3 {
                faults.push(format!("{name}: a hand {hand_off:.4} m off the pole"));
            }
            if flexed < -0.6 {
                faults.push(format!("{name}: a wrist bent back {flexed:.2} rad"));
            }
            if kink > 0.03 || catch_kink > 0.07 || catching_kink > 0.08 {
                faults.push(format!("{name}: a step changed {kink:.4} swinging, {catch_kink:.4} as caught, {catching_kink:.4} catching"));
            }
            if released.dot(forward) <= 0.0 || released.y <= 0.0 {
                faults.push(format!("{name}: let go at {released:?}, not up and on"));
            }
            if falling.root().y.abs() > 1.0e-3 || !falling.is_done() {
                faults.push(format!("{name}: did not land, at {:?}", falling.root()));
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }
}
