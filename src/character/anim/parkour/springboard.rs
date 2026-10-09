//! A springboard: step 12 of the parkour steps beyond the first ten (more
//! jumps). A sprung plank (or a flagpole's end) run onto: the take-off foot
//! comes down on its free end, which bends under the push and springs back,
//! and the running leap leaves as much higher as the board gives
//! (`Jump::from_board`). A little into its flight it is handed to a fall,
//! which lands on whatever it comes down on, or catches a ledge.
//!
//! A board yielding under the same push leaves the centre of mass's path as
//! it is: only the leg reaches further down after the foot. What it adds is
//! energy stored and given back, which a game's board does by fiat: its
//! give, metres of rise over a leap's own.

use bevy::math::Vec3;

use super::Falling;
use crate::character::anim::jump::{Board, Jump, JumpAsk, RunStart};
use crate::character::anim::rig::{forward_kinematics_on, LocalPose, RigGeometry};

/// A running leap off a board asks this rise of its own, metres of the
/// centre of mass, and the board adds its give.
pub const LEAP: f32 = 0.3;

/// A plank's give, metres of rise over the leap's own, and the most it
/// bends under the take-off foot, metres. The leg has 7-10 cm of slack
/// where the push is hardest (0.8 of the stance), which the foot sinking
/// with the board takes up.
pub const GIVE: f32 = 0.4;
pub const DIP: f32 = 0.04;

/// The take-off ankle is aimed this far back from the board's free end,
/// metres, so the foot's toes are on it, and comes down within this far of
/// there, along its way and across it; or the run goes past. Aimed at the
/// end itself, the foot came down 13 cm past it, over the drop, and the
/// foot's ground was the floor below.
pub const SPOT_BACK: f32 = 0.3;
pub const ON_BOARD: f32 = 0.15;

/// A springboard: its free end's top, where the take-off foot's ankle comes
/// down (the world), the way it launches along (level, from its fixed end
/// toward `at`), how long it is from its fixed end, metres; its give and
/// its most bend ([`GIVE`], [`DIP`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Springboard {
    pub at: Vec3,
    pub way: Vec3,
    pub length: f32,
    pub give: f32,
    pub dip: f32,
}

impl Springboard {
    /// A plank `length` long ending at `at`, launching along `way`.
    pub fn plank(at: Vec3, way: Vec3, length: f32) -> Self {
        Self { at, way: Vec3::new(way.x, 0.0, way.z).normalize_or(Vec3::NEG_Z), length, give: GIVE, dip: DIP }
    }

    /// Where the take-off ankle is aimed ([`SPOT_BACK`]).
    pub fn spot(&self) -> Vec3 {
        self.at - self.way * SPOT_BACK
    }

    /// How far `p` is from its [`Self::spot`] along its way (negative short
    /// of it), and across it, metres.
    pub fn off(&self, p: Vec3) -> (f32, f32) {
        let spot = self.spot();
        let d = Vec3::new(p.x - spot.x, 0.0, p.z - spot.z);
        (d.dot(self.way), d.cross(self.way).y)
    }

    /// Its fixed end, the world.
    pub fn root(&self) -> Vec3 {
        self.at - self.way * self.length
    }
}

/// The running leap off `board` from a run at `start.speed`, the foot of
/// `start.leg` just come down on it.
pub fn spring_leap(board: &Springboard, start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> Jump {
    Jump::from_board(JumpAsk::running(LEAP + board.give, 0.0), start, Board { give: board.give, dip: board.dip }, stood, rig)
}

/// How far ahead of the root its take-off ankle is as the foot comes down,
/// metres along the way of running (the jump's frame).
pub fn takeoff_ahead(board: &Springboard, start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> f32 {
    let jump = spring_leap(board, start, stood, rig);
    let ankle = [crate::character::skeleton::Bone::LeftFoot, crate::character::skeleton::Bone::RightFoot][start.leg];
    forward_kinematics_on(&jump.pose_at(0.0, stood, rig), rig)[ankle].dot(rig.forward())
}

/// Whether a leap off a board is handed to its fall now: in the air and
/// past its top, coming down (as a precision jump is). Handed over rising
/// toward a top higher than it left, the fall's drop, standing height to
/// standing height, was below nothing: its landing had no depth to brake
/// in, and went NaN. Planned from the top of the flight instead (for every
/// fall), a running jump's fall moved 1 cm off its path a frame on.
pub fn hands_over(jump: &Jump) -> bool {
    let t = jump.elapsed();
    jump.airborne() && jump.com_height_at(t + 1.0e-3) <= jump.com_height_at(t)
}

/// The fall a leap off a board is handed ([`hands_over`]): from the
/// walker's root `root` turned `yaw`, to the ground `ground` high, landing
/// on the first top along its flight (`land_on`'s `top`).
#[allow(clippy::too_many_arguments)]
pub fn spring_fall(jump: &Jump, root: Vec3, yaw: f32, ground: f32, top: &dyn Fn(Vec3) -> Option<f32>, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Falling {
    let mut falling = Falling::from_jump(jump, root, yaw, ground, drop, stood, rig);
    falling.land_on(top);
    falling
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::jump::{JumpPhase, GRAVITY};
    use crate::character::anim::rig::BoneSet;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
    use crate::character::skeleton::Bone;
    use bevy::math::Quat;

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// Off a plank at 4 and 5 m/s, either foot, onto the floor or a top
    /// 0.4-0.6 m higher: the board bends as far as asked, from and to rest,
    /// and the take-off ankle sinks with it within 2 mm (the plant leg
    /// reaching it); the COM rises the leap's own and the board's give
    /// (within 3 cm); handed to a fall at the top it flies on ballistically
    /// and lands where asked, every pose finite, no joint's step changing
    /// over 4 cm across the hand-over (the leap's own changes 8-11 cm at
    /// toe-off).
    #[test]
    fn a_springboard_bends_under_the_foot_and_throws_the_leap_higher() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        let mut faults = Vec::new();
        // Each run's speed, and the top it lands on: from how far past the
        // take-off root, how high (none: the floor it left).
        for (speed, onto) in [(4.0f32, None), (5.0, None), (4.0, Some((1.2f32, 0.6f32))), (5.0, Some((1.0, 0.4)))] {
            for leg in 0..2 {
                let start = RunStart { leg, speed };
                let name = format!("{speed} m/s off {leg} onto {onto:?}");
                let top = move |at: Vec3| onto.and_then(|(from, high)| (at.dot(forward) > from).then_some(high));
                let lands = onto.map_or(0.0, |(_, high)| high);
                let board = Springboard::plank(forward * 0.5, forward, 1.2);
                let world = |pose: &LocalPose, root: Vec3, yaw: f32| {
                    let at = forward_kinematics_on(pose, &rig);
                    BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
                };
                let ankle = [Bone::LeftFoot, Bone::RightFoot][leg];
                // The same leap off a board that does not bend: its own
                // change of step, to its hand-over (`from_run` would cap
                // its rise at a leap's).
                let unbent = || Jump::from_board(JumpAsk::running(LEAP + GIVE, 0.0), start, Board { give: GIVE, dip: 0.0 }, &stood, &rig);
                let own = {
                    let mut plain = unbent();
                    let mut frames = Vec::new();
                    while !hands_over(&plain) {
                        plain.advance(DT);
                        frames.push(world(&plain.pose(&stood, &rig), forward * plain.travelled(), 0.0));
                    }
                    frames.windows(3).map(|w| Bone::ALL.iter().map(|&b| (w[2][b] - 2.0 * w[1][b] + w[0][b]).length()).fold(0.0, f32::max)).fold(0.0, f32::max)
                };
                let mut jump = spring_leap(&board, start, &stood, &rig);
                let flat = unbent();
                // How far the ankle sank under the plain leap's, the board
                // most, and the most the ankle missed the board by (the
                // plant leg short of reaching it).
                let (mut deepest, mut sunk_most, mut missed) = (0.0f32, 0.0f32, 0.0f32);
                let mut frames = Vec::new();
                let mut first = None;
                while !hands_over(&jump) {
                    let t = jump.elapsed();
                    let at = forward_kinematics_on(&jump.pose_at(t, &stood, &rig), &rig);
                    let plain = forward_kinematics_on(&flat.pose_at(t, &stood, &rig), &rig);
                    if !jump.airborne() {
                        let sank = (plain[ankle] - plain[Bone::Hips]).y - (at[ankle] - at[Bone::Hips]).y + (plain[Bone::Hips].y - at[Bone::Hips].y);
                        deepest = deepest.max(sank);
                        sunk_most = sunk_most.max(jump.board_sunk());
                        missed = missed.max((sank - jump.board_sunk()).abs());
                    }
                    first.get_or_insert(jump.board_sunk());
                    jump.advance(DT);
                    frames.push(world(&jump.pose(&stood, &rig), forward * jump.travelled(), 0.0));
                }
                let left = jump.ends(JumpPhase::Push);
                let leaves = jump.com_height_at(left);
                // The COM's top: the leap's own to its hand-over at the top.
                let top_of = (0..=200).map(|k| jump.com_height_at(left + (jump.elapsed() - left) * k as f32 / 200.0)).fold(leaves, f32::max);
                let mut falling = spring_fall(&jump, forward * jump.travelled(), 0.0, 0.0, &top, 0.0, &stood, &rig);
                // The most a step changes over the hand-over (its first
                // frames) and after (the fall's own landing).
                let (mut kink, mut landing_kink, mut accel) = (0.0f32, 0.0f32, 0.0f32);
                // The fall's hips (the jump's swing about its COM, which the
                // fall takes the velocity of).
                let mut hips: Vec<Vec3> = Vec::new();
                let handed = frames.len();
                while !falling.is_done() {
                    falling.advance(DT);
                    let pose = falling.pose(&rig);
                    if !Bone::ALL.iter().all(|&b| pose.rotations[b].is_finite()) || !pose.root_translation.is_finite() {
                        faults.push(format!("{name}: NaN"));
                        break;
                    }
                    let now = world(&pose, falling.root(), falling.facing());
                    let n = frames.len();
                    let step = Bone::ALL.iter().map(|&b| (now[b] - 2.0 * frames[n - 1][b] + frames[n - 2][b]).length()).fold(0.0, f32::max);
                    if n < handed + 3 { kink = kink.max(step) } else { landing_kink = landing_kink.max(step) }
                    // The hips ballistic in the air: nothing (a top landed
                    // on, a wall) pushes them.
                    if falling.airborne() {
                        hips.push(now[Bone::Hips]);
                        let k = hips.len();
                        if k >= 3 {
                            accel = accel.max(((hips[k - 1].y - 2.0 * hips[k - 2].y + hips[k - 3].y) / (DT * DT) + GRAVITY).abs());
                        }
                    }
                    frames.push(now);
                }
                let rose = top_of - leaves;
                eprintln!(
                    "{name}: sunk {deepest:.3} (board {sunk_most:.3}, first {:.4}), missed {missed:.4}, rose {rose:.3} (asked {}), off g {accel:.3}, hand-over kink {kink:.4} (own {own:.4}), landing's {landing_kink:.4}, lands {:?}",
                    first.unwrap_or(0.0),
                    LEAP + GIVE,
                    falling.root()
                );
                if (sunk_most - DIP).abs() > 0.002 || first.unwrap_or(1.0) > 1.0e-4 {
                    faults.push(format!("{name}: the board bent {sunk_most:.3} at most, {:.4} at first, against {DIP}", first.unwrap_or(1.0)));
                }
                if missed > 2.0e-3 {
                    faults.push(format!("{name}: the ankle {missed:.4} m off the bent board"));
                }
                if accel > 0.05 {
                    faults.push(format!("{name}: in the air, the hips {accel:.3} m/s² off g"));
                }
                if (rose - (LEAP + GIVE)).abs() > 0.03 {
                    faults.push(format!("{name}: rose {rose:.3}, asked {}", LEAP + GIVE));
                }
                if kink > 0.04 {
                    faults.push(format!("{name}: handed over, a step changed {kink:.4}"));
                }
                if (falling.root().y - lands).abs() > 1.0e-3 {
                    faults.push(format!("{name}: landed at {:?}", falling.root()));
                }
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }
}
