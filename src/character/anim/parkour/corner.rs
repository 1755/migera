//! A corner swing: step 17 of the parkour steps beyond the first ten
//! (swinging on fixtures). Running past a post at a building's corner, the
//! near hand catches it and the body swings round it on the arm, landing
//! on the far side running on the new way.
//!
//! It is a running leap (`Jump::from_run`, running on) whose flight is bent
//! round the post: through the flight the facing turns at the leap's speed
//! over the post's distance, so the way it travels (along its facing) is an
//! arc about the post; the near hand holds the post through the flight,
//! and the body banks toward it. Taken off so the flight begins with the
//! post beside the hips, its flight as long as the turn asks.

use bevy::math::{Quat, Vec3};

use super::pole::Pole;
use crate::character::anim::armik::{frame_turn, solve_arm_toward_from, turn_hand, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::hand::{HandGrip, FINGER_HALF_THICKNESS, GRIP_RADIUS};
use crate::character::anim::jump::{Jump, JumpAsk, JumpPhase, RunStart, GRAVITY};
use crate::character::anim::rig::{accumulate_bind_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// The post's axis this near the hips and this far, metres, as the flight
/// begins: the hand reaching it from the shoulder.
pub const NEAREST: f32 = 0.45;
pub const FARTHEST: f32 = 0.7;
/// The farthest the holding arm is solved to, of its length: straight, its
/// elbow had no steady side (0.75 m out, it swung 25 cm in a frame).
const ARM_REACH: f32 = 0.97;
/// The flight begins this near the post's being beside the hips, along the
/// way, metres (taken off from a footfall: the run's steps are paced).
pub const ABEAM: f32 = 0.25;
/// The rise a leap round a post may take, metres of the centre of mass.
const LOWEST: f32 = 0.05;
const HIGHEST: f32 = 0.5;
/// The hand takes the post over the take-off's last this long, seconds, and
/// lets it go over the flight's last this long (or half the flight).
const TAKING: f32 = 0.12;
const LETTING: f32 = 0.15;
/// The most the body banks toward the post, radians.
const MOST_BANK: f32 = 0.45;
const GUESSED_KNUCKLES: f32 = 0.08;
const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];

/// A swing round a post under way: the post, the hand that holds it (0
/// left, 1 right), and how far the facing turns through the flight
/// (radians, positive to the left).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CornerSwing {
    pub post: Pole,
    pub side: usize,
    pub turn: f32,
    /// The arm's grip, in the hand's rest frame (`hand::bound_grips`).
    grip: HandGrip,
}

/// The running leap round `post` from a run at `start.speed`, the foot of
/// `start.leg` just down with the root at `origin` facing `yaw` (the walker's),
/// turning `turn` radians (its sign: which way round, positive to the
/// left); `None` unless the post is on that side between [`NEAREST`] and
/// [`FARTHEST`] off the way and comes beside the hips within [`ABEAM`] of
/// where the flight begins.
#[allow(clippy::too_many_arguments)]
pub fn plan(post: &Pole, origin: Vec3, yaw: f32, start: RunStart, turn: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<(Jump, CornerSwing)> {
    let facing = Quat::from_rotation_y(yaw);
    let (forward, left) = (facing * rig.forward(), facing * rig.left());
    let to = (post.foot - origin).with_y(0.0);
    let (along, aside) = (to.dot(forward), to.dot(left));
    if aside.signum() != turn.signum() || !(NEAREST..=FARTHEST).contains(&aside.abs()) {
        return None;
    }
    // The flight as long as the turn at its speed takes: first planned to
    // find its speed, then for its height.
    let first = Jump::from_run(JumpAsk::running(0.15, 0.0), start, stood, rig);
    let seconds = turn.abs() * aside.abs() / first.speed().max(0.5);
    let height = (GRAVITY * seconds * seconds / 8.0).clamp(LOWEST, HIGHEST);
    let jump = Jump::from_run(JumpAsk::running(height, 0.0), start, stood, rig);
    let leaves = jump.travelled_at(jump.ends(JumpPhase::Push));
    if (along - leaves).abs() > ABEAM {
        return None;
    }
    let side = if turn > 0.0 { 0 } else { 1 };
    Some((jump, CornerSwing { post: *post, side, turn, grip: guessed_grip(side, rig) }))
}

/// A quintic ease, 0 to 1 with no speed nor acceleration at either end.
fn ease(u: f32) -> f32 {
    let u = u.clamp(0.0, 1.0);
    u * u * u * (u * (6.0 * u - 15.0) + 10.0)
}

/// How far ahead of where a run's foot comes down (the root there) the post
/// is best for a swing round it: beside the hips as the flight begins.
pub fn takeoff_ahead(start: RunStart, turn: f32, aside: f32, stood: &LocalPose, rig: &RigGeometry) -> f32 {
    let first = Jump::from_run(JumpAsk::running(0.15, 0.0), start, stood, rig);
    let seconds = turn.abs() * aside.abs() / first.speed().max(0.5);
    let height = (GRAVITY * seconds * seconds / 8.0).clamp(LOWEST, HIGHEST);
    let jump = Jump::from_run(JumpAsk::running(height, 0.0), start, stood, rig);
    jump.travelled_at(jump.ends(JumpPhase::Push))
}

fn guessed_grip(side: usize, rig: &RigGeometry) -> HandGrip {
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let arm = ARMS[side];
    let along = (rest[arm.wrist] - rest[arm.elbow]).normalize_or(Vec3::NEG_Y);
    let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
    HandGrip { bar: along * GUESSED_KNUCKLES + palm * (GRIP_RADIUS + FINGER_HALF_THICKNESS), palm, along }
}

impl CornerSwing {
    /// The holding hand placed so its own fingers close round the post.
    pub fn set_grips(&mut self, grips: [Option<HandGrip>; 2], rig: &RigGeometry) {
        if let Some(grip) = crate::character::anim::hand::bound_grips(grips, rig)[self.side] {
            self.grip = grip;
        }
    }

    /// How far the facing has turned `t` seconds into `jump`, radians:
    /// eased through its flight (turned at a steady rate from its start,
    /// the feet a metre out were flung 3-5 m/s round in a frame, a step
    /// changing 30 cm).
    pub fn turned_at(&self, jump: &Jump, t: f32) -> f32 {
        let (push, flight) = (jump.ends(JumpPhase::Push), jump.ends(JumpPhase::Flight));
        self.turn * ease((t - push) / (flight - push).max(1.0e-3))
    }


    /// How far the hand holds the post now (0-1): taken over the take-off's
    /// last [`TAKING`], held through the flight, let go over its last
    /// [`LETTING`], the post still beside it (let go after landing, the
    /// body run on past it, the arm reaching back to it turned over and a
    /// hand jumped 39 cm in a frame).
    pub fn holding(&self, jump: &Jump) -> f32 {
        let t = jump.elapsed();
        let (push, flight) = (jump.ends(JumpPhase::Push), jump.ends(JumpPhase::Flight));
        let letting = LETTING.min(0.5 * (flight - push));
        if t < push {
            smoothstep(((t - (push - TAKING)) / TAKING).clamp(0.0, 1.0))
        } else {
            1.0 - smoothstep(((t - (flight - letting)) / letting).clamp(0.0, 1.0))
        }
    }

    /// Whether it is done with the post (let go as it lands).
    pub fn is_done(&self, jump: &Jump) -> bool {
        jump.elapsed() >= jump.ends(JumpPhase::Flight)
    }

    /// `pose` (the jump's, its root at `root` turned `yaw`, the walker's)
    /// with the near hand on the post and the body banked toward it, by how
    /// far it holds ([`Self::holding`]).
    pub fn hold(&self, pose: &LocalPose, jump: &Jump, root: Vec3, yaw: f32, rig: &RigGeometry) -> LocalPose {
        self.held(pose, jump, root, yaw, rig).0
    }

    /// [`Self::hold`], and the wrist's place on the post it reaches for
    /// (the world).
    pub fn held(&self, pose: &LocalPose, jump: &Jump, root: Vec3, yaw: f32, rig: &RigGeometry) -> (LocalPose, Vec3) {
        let weight = self.holding(jump);
        let own = root + Quat::from_rotation_y(yaw) * forward_kinematics_on(pose, rig)[ARMS[self.side].wrist];
        if weight <= 0.0 {
            return (*pose, own);
        }
        let mut pose = *pose;
        let turn = Quat::from_rotation_y(yaw);
        let back = turn.inverse();
        // Banked toward the post as the turn would have it on its own
        // (`tan θ = v ω / g`, its mean rate), saturating smoothly at
        // `MOST_BANK`, rising and falling over the hold as `sin²`. Clamped,
        // the rate passed the clamp at once and the corner swung a toe
        // 23 cm in a frame; following the rate as it rose, the bank came
        // in over two frames and a toe swung 25 cm.
        let (push, flight) = (jump.ends(JumpPhase::Push), jump.ends(JumpPhase::Flight));
        let mean = self.turn / (flight - push).max(1.0e-3);
        let most = MOST_BANK * ((jump.speed() * mean / GRAVITY).atan() / MOST_BANK).tanh();
        let (from, to) = (push - TAKING, flight);
        let bank = most * (std::f32::consts::PI * ((jump.elapsed() - from) / (to - from)).clamp(0.0, 1.0)).sin().powi(2);
        pose.rotations[Bone::Hips] = delta_after_world_turn(&pose, rig, Bone::Hips, Quat::from_axis_angle(rig.forward(), -bank));
        // The hand round the post at the shoulder's height, the palm to its
        // axis, the fingers on along the forearm, round it.
        let chain = ARMS[self.side];
        let at = forward_kinematics_on(&pose, rig);
        let shoulder = root + turn * at[chain.shoulder];
        let axis = self.post.at(shoulder.y);
        let palm = (axis - shoulder).with_y(0.0).normalize_or(turn * rig.left());
        let own = root + turn * at[chain.wrist];
        // The fingers round the post level, the way the body goes: taken
        // along the line from the elbow to the post, square to the palm,
        // that line lay near the palm's and its rest flipped, the hand 33 cm
        // in a frame.
        let ahead = turn * rig.forward();
        let along = (ahead - palm * ahead.dot(palm)).normalize_or(Vec3::Y.cross(palm));
        let hand = frame_turn(self.grip.along, self.grip.palm, along, palm);
        let on = axis - hand * self.grip.bar;
        // The arm solved onto the post fully, then its bones blended from
        // the leap's own by the hold (the wrist and the elbow's way eased
        // instead, the arm passed through straight letting go, the post
        // behind it, and the elbow flipped 34 cm in a frame).
        let free = pose;
        let elbow_way = back * (Vec3::NEG_Y * 0.7 - palm * 0.3).normalize();
        let arm = (at[chain.elbow] - at[chain.shoulder]).length() + (at[chain.wrist] - at[chain.elbow]).length();
        let off = back * (on - root) - at[chain.shoulder];
        let target = at[chain.shoulder] + off * (off.length().min(ARM_REACH * arm) / off.length().max(1.0e-6));
        solve_arm_toward_from(&mut pose, &at, chain, target, elbow_way, rig);
        let at = forward_kinematics_on(&pose, rig);
        turn_hand(&mut pose, rig, chain, accumulate_bind_rotations(rig)[chain.wrist], back * hand, 1.0, (at[chain.wrist] - at[chain.elbow]).normalize_or_zero());
        // Along one arc, each sign set against the bone's rest (near half a
        // turn apart, the shortest arc flipped sides letting go: a hand
        // 37 cm in a frame).
        for bone in [chain.shoulder, chain.elbow, chain.wrist] {
            let rest = LocalPose::REST.rotations[bone];
            let toward = |q: Quat| if q.dot(rest) < 0.0 { -q } else { q };
            pose.rotations[bone] = super::fall::arc(toward(free.rotations[bone]), toward(pose.rotations[bone]), weight);
        }
        let wrist = root + turn * forward_kinematics_on(&pose, rig)[chain.wrist];
        let _ = own;
        (pose, if weight >= 1.0 { on } else { wrist })
    }
}

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

    /// Running at 3.5 and 4.5 m/s past posts 0.55-0.68 m to the left or the
    /// right, turning a quarter or a third of a turn round them: the hips
    /// keep near the post's distance through the flight (within 10 cm), the
    /// held hand on the post within 2 cm at full hold, the facing turned as
    /// asked (within 0.02 rad) and running on at the leap's speed; no
    /// joint's step changing over 1.5 cm more in a frame than the same leap
    /// turned unheld and what the turn accelerates a trailing toe by.
    #[test]
    fn a_corner_post_is_swung_round_on_one_arm() {
        let (stood, rig) = real_stood();
        let grips = crate::character::anim::hand::puppet_grips();
        let mut faults = Vec::new();
        for (speed, aside, turn) in [(3.5f32, 0.6f32, std::f32::consts::FRAC_PI_2), (4.5, 0.65, -std::f32::consts::FRAC_PI_2), (3.5, 0.55, -std::f32::consts::FRAC_PI_3), (4.5, 0.68, std::f32::consts::FRAC_PI_3)] {
            let name = format!("{speed} m/s, a post {aside} m aside turning {turn:.2}");
            let start = RunStart { leg: 0, speed };
            let ahead = takeoff_ahead(start, turn, aside, &stood, &rig);
            let post = Pole::new(rig.forward() * ahead + rig.left() * (aside * turn.signum()), 3.0);
            let Some((mut jump, mut swing)) = plan(&post, Vec3::ZERO, 0.0, start, turn, &stood, &rig) else {
                faults.push(format!("{name}: not planned"));
                continue;
            };
            swing.set_grips(grips, &rig);
            // The walker's motion: the jump's travel along the facing, the
            // facing turning as the swing has it.
            let (mut root, mut yaw) = (Vec3::ZERO, 0.0f32);
            let (mut radius, mut hand_off, mut kink, mut own) = ((f32::MAX, 0.0f32), 0.0f32, 0.0f32, 0.0f32);
            let mut frames: Vec<BoneSet<Vec3>> = Vec::new();
            let mut plain: Vec<BoneSet<Vec3>> = Vec::new();
            let mut plain_jump = jump.clone();
            let (mut plain_root, mut plain_yaw) = (Vec3::ZERO, 0.0f32);
            while !jump.is_done() {
                let (before, was) = (jump.travelled(), jump.elapsed());
                jump.advance(DT);
                yaw += swing.turned_at(&jump, jump.elapsed()) - swing.turned_at(&jump, was);
                root += Quat::from_rotation_y(yaw) * rig.forward() * (jump.travelled() - before);
                let (pose, target) = swing.held(&jump.pose(&stood, &rig), &jump, root, yaw, &rig);
                let at = forward_kinematics_on(&pose, &rig);
                let now = BoneSet::from_fn(|b| root + Quat::from_rotation_y(yaw) * at[b]);
                if jump.airborne() {
                    let d = (now[Bone::Hips] - post.foot).with_y(0.0).length();
                    radius = (radius.0.min(d), radius.1.max(d));
                }
                if swing.holding(&jump) >= 1.0 {
                    hand_off = hand_off.max((now[ARMS[swing.side].wrist] - target).length());
                }
                if frames.len() >= 2 {
                    let n = frames.len();
                    kink = Bone::ALL.iter().map(|&b| (now[b] - 2.0 * frames[n - 1][b] + frames[n - 2][b]).length()).fold(kink, f32::max);
                }
                frames.push(now);
                // The same leap turned but unheld, for its own change of
                // step and the turn's.
                let (before, was) = (plain_jump.travelled(), plain_jump.elapsed());
                plain_jump.advance(DT);
                plain_yaw += swing.turned_at(&plain_jump, plain_jump.elapsed()) - swing.turned_at(&plain_jump, was);
                plain_root += Quat::from_rotation_y(plain_yaw) * rig.forward() * (plain_jump.travelled() - before);
                let at = forward_kinematics_on(&plain_jump.pose(&stood, &rig), &rig);
                let now = BoneSet::from_fn(|b| plain_root + Quat::from_rotation_y(plain_yaw) * at[b]);
                if plain.len() >= 2 {
                    let n = plain.len();
                    own = Bone::ALL.iter().map(|&b| (now[b] - 2.0 * plain[n - 1][b] + plain[n - 2][b]).length()).fold(own, f32::max);
                }
                plain.push(now);
            }
            let runs_on = jump.resumes().map_or(0.0, |r| r.speed);
            eprintln!("{name}: radius {:.3}..{:.3} (aside {aside}), hand off {hand_off:.4}, turned {yaw:.3}, runs on {runs_on:.2}, kink {kink:.4} (own {own:.4})", radius.0, radius.1);
            if radius.0 < aside - 0.1 || radius.1 > aside + 0.1 {
                faults.push(format!("{name}: the hips {:.3}..{:.3} from the post", radius.0, radius.1));
            }
            if hand_off > 0.02 {
                faults.push(format!("{name}: the hand {hand_off:.4} off the post"));
            }
            if (yaw - turn).abs() > 0.02 {
                faults.push(format!("{name}: turned {yaw:.3}, not {turn:.3}"));
            }
            if runs_on < 0.5 * speed {
                faults.push(format!("{name}: runs on at {runs_on:.2}"));
            }
            // The leap's own change of step, and what the turn itself
            // accelerates a trailing toe 1.2 m from the turn's axis by
            // (`(ω² + α)·r·dt²`, its quintic's peak rate and acceleration
            // over the flight).
            let flight = jump.ends(JumpPhase::Flight) - jump.ends(JumpPhase::Push);
            let (rate, spin) = (1.875 * turn.abs() / flight, 5.774 * turn.abs() / (flight * flight));
            let turning = (rate * rate + spin) * 1.2 * DT * DT;
            if kink > own + turning + 0.015 {
                faults.push(format!("{name}: a step changed {kink:.4}, the leap's own {own:.4} and the turn's {turning:.4}"));
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }
}
