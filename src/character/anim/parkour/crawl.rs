//! Crawling on hands and knees: step 10 of the parkour design, fourth
//! part. Asked to crawl, the walker gets down through the face-down
//! get-up's keys run backward (a half-kneel, then hands and knees), crawls
//! along its facing in a four-beat crawl (right hand, left knee, left hand,
//! right knee) while asked, and gets up through the same keys forward.
//!
//! Hands-and-knees crawling (Ma et al. 2017, `parkour-movement-data`): 0.28
//! to 0.69 m/s tried; slow, four-beat or diagonal pairs. Here 0.4 m/s,
//! four-beat, each limb down three quarters of the cycle.

use bevy::math::{Quat, Vec3};

use crate::character::anim::armik::{shoulder_lift, solve_arm_toward_from, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::getup;
use crate::character::anim::rig::{blend_in_world, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::sitting::{clear_floor, SitKey};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// The crawl's pace, m/s, and its stride, metres a cycle; crawling, the
/// body rides this much lower than hands and knees (the key's arms and
/// thighs straight down, a hand or knee 19 cm ahead or behind was out of
/// reach, the limb straightened, and its elbow's swivel jumped 5.6 cm).
pub const CRAWL_SPEED: f32 = 0.4;
const STRIDE: f32 = 0.4;
const DIP: f32 = 0.05;
/// The share of a cycle each limb is down.
const DUTY: f32 = 0.75;
/// Each limb's place in the cycle: left hand, right hand, left knee, right
/// knee (right hand, left knee, left hand, right knee).
const PHASES: [f32; 4] = [0.5, 0.0, 0.25, 0.75];
/// How high a hand and a knee lift on their way, metres.
const HAND_LIFT: f32 = 0.06;
const KNEE_LIFT: f32 = 0.04;
/// Into and out of crawling (the pace and the stride from and to none),
/// seconds.
const RAMP: f32 = 0.6;
/// Getting down: into the half-kneel, then onto hands and knees, seconds
/// (the sitting floor route's 1.3 into a half-kneel; the get-up's 0.9 out
/// of hands and knees). Getting up: the get-up's.
const INTO_HALF_KNEEL: f32 = 1.3;
const ONTO_HANDS: f32 = 0.9;

const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
const LEGS: [(Bone, Bone); 2] = [(Bone::LeftLeg, Bone::LeftFoot), (Bone::RightLeg, Bone::RightFoot)];
const SIGN: [f32; 2] = [1.0, -1.0];

/// Where it is in a crawl.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// Getting down, `t` seconds in.
    Down { t: f32 },
    /// Crawling: the cycle's phase (0-1) and how far into the crawl's pace
    /// it is (0-1).
    Crawling { cycle: f32, ramp: f32 },
    /// Getting up, `t` seconds in.
    Up { t: f32 },
    /// Stood up.
    Done,
}

/// Crawling on hands and knees, getting down to it and up from it.
#[derive(Debug, Clone)]
pub struct Crawling {
    rig: RigGeometry,
    stood: LocalPose,
    /// Hands and knees (the get-up's key), the keys down to it from
    /// standing and up from it to standing.
    base: LocalPose,
    down: Vec<SitKey>,
    up: Vec<SitKey>,
    /// Where it got down (the root) and its facing; how far it has crawled.
    start: Vec3,
    yaw: f32,
    travelled: f32,
    phase: Phase,
}

impl Crawling {
    /// Getting down to crawl from standing with the root at `root` turned
    /// `yaw` (on `rig`, standing `stood`).
    pub fn new(root: Vec3, yaw: f32, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let keys = getup::keys(getup::Lying::FaceDown, rig);
        // Cleared of the floor as the keys play it (left as built, the hips
        // dropped 1.6 cm as the crawl took over from the keys).
        let (base, half) = (clear_floor(keys[0].pose, rig), keys[1].pose);
        Self {
            rig: rig.clone(),
            stood: *stood,
            base,
            down: vec![SitKey { pose: half, seconds: INTO_HALF_KNEEL }, SitKey { pose: base, seconds: ONTO_HANDS }],
            up: vec![SitKey { pose: half, seconds: keys[1].seconds }, SitKey { pose: *stood, seconds: getup::STAND_SECONDS }],
            start: root,
            yaw,
            travelled: 0.0,
            phase: Phase::Down { t: 0.0 },
        }
    }

    /// Moves it on `dt` seconds, asked to crawl (`on`) or not: getting down
    /// goes on to crawling, crawling eases to a stop and gets up once not
    /// asked.
    pub fn advance(&mut self, on: bool, dt: f32) {
        let period = STRIDE / CRAWL_SPEED;
        self.phase = match self.phase {
            Phase::Down { t } if t + dt >= INTO_HALF_KNEEL + ONTO_HANDS => Phase::Crawling { cycle: 0.0, ramp: 0.0 },
            Phase::Down { t } => Phase::Down { t: t + dt },
            Phase::Crawling { cycle, ramp } => {
                let ramp = (ramp + if on { dt } else { -dt } / RAMP).clamp(0.0, 1.0);
                let ramp = if on { (ramp).min(1.0) } else { ramp };
                let eased = smoothstep(ramp);
                self.travelled += CRAWL_SPEED * eased * dt;
                let cycle = (cycle + eased * dt / period).rem_euclid(1.0);
                if !on && ramp <= 0.0 { Phase::Up { t: 0.0 } } else { Phase::Crawling { cycle, ramp } }
            }
            Phase::Up { t } if t + dt >= self.up.iter().map(|key| key.seconds).sum::<f32>() => Phase::Done,
            Phase::Up { t } => Phase::Up { t: t + dt },
            Phase::Done => Phase::Done,
        };
    }

    /// The facing turn.
    fn turn(&self) -> Quat {
        Quat::from_rotation_y(self.yaw)
    }

    /// The walker's root now: where it got down, moved on as far as it has
    /// crawled along its facing.
    pub fn root(&self) -> Vec3 {
        self.start + self.turn() * (self.rig.forward() * self.travelled)
    }

    /// The walker's facing.
    pub fn facing(&self) -> f32 {
        self.yaw
    }

    /// Whether it has stood up again.
    pub fn is_done(&self) -> bool {
        self.phase == Phase::Done
    }

    /// Whether it is on hands and knees, crawling or still.
    pub fn is_crawling(&self) -> bool {
        matches!(self.phase, Phase::Crawling { .. })
    }

    /// The pose now, on the rig it was made on, in the walker's frame at
    /// [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self) -> LocalPose {
        match self.phase {
            Phase::Down { t } => play(&self.stood, &self.down, t, &self.rig),
            Phase::Crawling { cycle, ramp } => self.crawling(cycle, smoothstep(ramp)),
            Phase::Up { t } => play(&self.base, &self.up, t, &self.rig),
            Phase::Done => self.stood,
        }
    }

    /// Where each limb (left hand, right hand, left knee, right knee) is
    /// `cycle` into the crawl, its stride `spread` (0-1) of the full: how
    /// far along the facing from its place in hands and knees, metres, and
    /// how high it is lifted. Down, it goes back at the body's pace, so it
    /// stays put on the floor; up, it swings forward.
    fn limbs(cycle: f32, spread: f32) -> [(f32, f32); 4] {
        [0, 1, 2, 3].map(|limb| {
            let phase = (cycle + PHASES[limb]).rem_euclid(1.0);
            let lift = if limb < 2 { HAND_LIFT } else { KNEE_LIFT };
            let reach = DUTY * STRIDE * spread;
            if phase < DUTY {
                (reach * (0.5 - phase / DUTY), 0.0)
            } else {
                let s = (phase - DUTY) / (1.0 - DUTY);
                (reach * (smoothstep(s) - 0.5), lift * spread * (std::f32::consts::PI * s).sin())
            }
        })
    }

    /// Hands and knees, each limb moved as [`Self::limbs`] has it: the arms
    /// reaching their hands, the legs their shins (the knee and the ankle
    /// moved together).
    fn crawling(&self, cycle: f32, spread: f32) -> LocalPose {
        let rig = &self.rig;
        let forward = rig.forward();
        let mut pose = self.base;
        let at = forward_kinematics_on(&pose, rig);
        // The body lowered, the limbs' places where hands and knees have
        // them on the floor: bent to reach them.
        pose.root_translation -= Vec3::Y * (DIP * spread);
        let hips = at[Bone::Hips] - Vec3::Y * (DIP * spread);
        let limbs = Self::limbs(cycle, spread);
        let moved = |p: Vec3, (along, up): (f32, f32)| p + forward * along + Vec3::Y * up;
        for (side, &(_, ankle)) in LEGS.iter().enumerate() {
            place_ankle(&mut pose, rig, ankle, moved(at[ankle], limbs[2 + side]) - hips);
        }
        let targets = [0, 1].map(|side| moved(at[ARMS[side].wrist], limbs[side]));
        let at = forward_kinematics_on(&pose, rig);
        for side in 0..2 {
            let arm = (at[ARMS[side].elbow] - at[ARMS[side].shoulder]).length() + (at[ARMS[side].wrist] - at[ARMS[side].elbow]).length();
            let lift = shoulder_lift(at[CLAVICLES[side]], at[ARMS[side].shoulder], targets[side], 0.85 * arm);
            pose.rotations[CLAVICLES[side]] = crate::character::anim::rig::delta_after_world_turn(&pose, rig, CLAVICLES[side], lift);
        }
        let at = forward_kinematics_on(&pose, rig);
        for side in 0..2 {
            // The elbow back, a little out (out to its side, the arms
            // spread wide as for a push-up).
            let pole = (rig.left() * (SIGN[side] * 0.3) - forward).normalize();
            solve_arm_toward_from(&mut pose, &at, ARMS[side], targets[side], pole, rig);
        }
        // Into the solved limbs with the stride: at none, hands and knees
        // exactly (solved there, the elbows swivelled their own way and the
        // forearms jumped 7 cm as the crawl took over from the keys).
        for bone in Bone::ALL {
            pose.rotations[bone] = self.base.rotations[bone].slerp(pose.rotations[bone], spread);
        }
        pose
    }

    /// Every joint in the world now.
    pub fn joints(&self) -> BoneSet<Vec3> {
        let (pose, root, turn) = (self.pose(), self.root(), self.turn());
        let at = forward_kinematics_on(&pose, &self.rig);
        BoneSet::from_fn(|bone| root + turn * at[bone])
    }
}

/// `keys` from `from`, `t` seconds in, as a posture plays them (`walker`):
/// each blended into in the world, eased, the feet down in both keys held
/// where they are, nothing under the floor.
fn play(from: &LocalPose, keys: &[SitKey], t: f32, rig: &RigGeometry) -> LocalPose {
    let mut left = t;
    let mut previous = *from;
    for key in keys {
        if left < key.seconds {
            let eased = smoothstep((left / key.seconds).clamp(0.0, 1.0));
            let mut blended = blend_in_world(&previous, &key.pose, eased, rig);
            let at = forward_kinematics_on(&blended, rig);
            let (was, will) = (forward_kinematics_on(&previous, rig), forward_kinematics_on(&key.pose, rig));
            let held: Vec<Vec3> = [Bone::LeftFoot, Bone::RightFoot]
                .into_iter()
                .filter(|&foot| {
                    let low = |pose: &LocalPose| getup::contact_height(pose, rig, getup::Contact::Foot(foot)) < 0.03;
                    was[foot].distance(will[foot]) < 0.03 && low(&previous) && low(&key.pose)
                })
                .map(|foot| was[foot].lerp(will[foot], eased) - at[foot])
                .collect();
            if !held.is_empty() {
                blended.root_translation += held.iter().sum::<Vec3>() / held.len() as f32;
            }
            return clear_floor(blended, rig);
        }
        left -= key.seconds;
        previous = key.pose;
    }
    previous
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// Getting down, crawling 4 s and getting up: continuous throughout, no
    /// joint whipping round nor under the floor; crawling, nothing higher
    /// than 0.85 m (under a 0.9 m gap), each limb down stays put on the
    /// floor, three limbs or more always down, and it covers its pace.
    #[test]
    fn it_gets_down_crawls_and_gets_up() {
        let (stood, rig) = real_stood();
        let dt = 1.0 / 60.0;
        for yaw in [0.0f32, 1.1] {
            let mut crawl = Crawling::new(Vec3::new(0.5, 0.0, -0.3), yaw, &stood, &rig);
            let mut frames = vec![crawl.joints(), crawl.joints()];
            let (mut t, mut fastest, mut kink, mut lowest, mut highest) = (0.0f32, 0.0f32, 0.0f32, f32::MAX, 0.0f32);
            let (mut slid, mut fewest_down) = (0.0f32, 4usize);
            let mut held: [Option<Vec3>; 4] = [None; 4];
            while !crawl.is_done() {
                crawl.advance(t < 2.2 + 4.0, dt);
                t += dt;
                let now = crawl.joints();
                let (a, b) = (frames[frames.len() - 2], frames[frames.len() - 1]);
                fastest = fastest.max(Bone::ALL.iter().map(|&bone| ((now[bone] - now[Bone::Hips]) - (b[bone] - b[Bone::Hips])).length() / dt).fold(0.0, f32::max));
                kink = kink.max(Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(0.0, f32::max));
                lowest = lowest.min(Bone::ALL.iter().map(|&bone| now[bone].y).fold(f32::MAX, f32::min));
                if let Phase::Crawling { cycle, ramp } = crawl.phase {
                    highest = highest.max(Bone::ALL.iter().map(|&bone| now[bone].y).fold(0.0, f32::max) + 0.12 * (bone_is_head_top(&now) as i32 as f32));
                    if ramp >= 1.0 {
                        let limbs = Crawling::limbs(cycle, 1.0);
                        let points = [now[ARMS[0].wrist], now[ARMS[1].wrist], now[Bone::LeftLeg], now[Bone::RightLeg]];
                        fewest_down = fewest_down.min(limbs.iter().filter(|(_, up)| *up == 0.0).count());
                        for limb in 0..4 {
                            match (limbs[limb].1 == 0.0, held[limb]) {
                                (true, Some(was)) => slid = slid.max((points[limb] - was).with_y(0.0).length()),
                                (true, None) => held[limb] = Some(points[limb]),
                                (false, _) => held[limb] = None,
                            }
                        }
                    }
                }
                frames.push(now);
            }
            let crawled = crawl.travelled;
            eprintln!("{yaw}: fastest {fastest:.2} m/s, kink {kink:.4} m, lowest {lowest:.3}, highest {highest:.2}, slid {slid:.4}, fewest down {fewest_down}, crawled {crawled:.2} m");
            assert!(fastest < 14.0, "{yaw}: a joint at {fastest:.1} m/s about the hips");
            assert!(kink < 0.03, "{yaw}: a joint's step changed {kink:.4} m in a frame");
            assert!(lowest > -0.01, "{yaw}: a joint {lowest:.3} m under the floor");
            assert!(highest < 0.85, "{yaw}: crawling, up to {highest:.2} m");
            assert!(slid < 0.01, "{yaw}: a limb down slid {slid:.4} m");
            assert!(fewest_down >= 3, "{yaw}: only {fewest_down} limbs down");
            assert!(crawled > CRAWL_SPEED * 3.0, "{yaw}: crawled only {crawled:.2} m in 4 s");
        }
    }

    /// Whether the head joint is the highest (its top 0.12 m above it).
    fn bone_is_head_top(at: &BoneSet<Vec3>) -> bool {
        Bone::ALL.iter().all(|&bone| at[bone].y <= at[Bone::Head].y + 1.0e-6)
    }
}
