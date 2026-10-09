//! Walking along a beam: step 10 of the parkour design, first part. On a
//! beam (a narrow top, [`Beam`]) the walk puts its feet nearly on the line
//! ([`BEAM_FEET`] of its step width), slows to a beam's pace, and holds
//! its arms out, swaying against the trunk ([`balance`]); the walker steers
//! back onto the beam's line.
//!
//! On beams 10-6 cm wide people walk at 0.82-0.69 m/s; with the arms free
//! the shoulders swing through 92-118° and the trunk bends sideways 61-100°
//! over a trial (da Silva Costa et al. 2022; Lambrich et al. 2025,
//! `parkour-movement-data`).

use bevy::math::{Quat, Vec3};

use super::Ledge;
use crate::character::anim::armik::ArmChain;
use crate::character::anim::rig::{delta_after_world_turn, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// A beam's width, metres, by default: 10 cm.
pub const BEAM_WIDTH: f32 = 0.1;
/// The pace on a beam, m/s (0.69-0.82 measured on 6-10 cm beams).
pub const BEAM_SPEED: f32 = 0.7;
/// The share of a walk's step width (13 cm on `puppet_base`) the feet keep
/// apart on a beam: their middles 2.6 cm either side of its line, the
/// swinging foot passing the planted one rather than through it.
pub const BEAM_FEET: f32 = 0.4;
/// Onto and off a beam, the balance eases in and out over this long,
/// seconds; the feet change their width in this many steps (each a walk
/// cycle built and cached).
pub const BEAM_EASE: f32 = 0.4;
pub const FEET_STEPS: f32 = 4.0;
/// The arms held out, radians up from hanging (75°), the elbows bent this
/// much; tilted against the trunk's sway by this much of it.
const ARMS_OUT: f32 = 1.3;
const ELBOWS: f32 = 0.35;
const ARMS_TILT: f32 = 2.0;
/// The trunk's sway sideways, radians, and its rate, Hz.
const SWAY: f32 = 0.08;
const SWAY_RATE: f32 = 0.55;
/// The root counts as on a beam this far either side of its edges, metres;
/// and from a step below its top (walking onto it beside its line, it is
/// steered onto it: held only once up there, a walker 9.5 cm off its line
/// walked its length straddling it, a foot on it and one on the floor) to
/// this far above.
const ON_MARGIN: f32 = 0.15;
const ON_HEIGHT: f32 = 0.15;

/// A beam: a narrow top from `a` to `b` (its line, at its top), `width`
/// across, standing on the floor (a block under it).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Beam {
    pub a: Vec3,
    pub b: Vec3,
    pub width: f32,
}

impl Beam {
    /// A beam [`BEAM_WIDTH`] wide from `a` to `b` (level, at its top).
    pub fn new(a: Vec3, b: Vec3) -> Self {
        Self { a, b: b.with_y(a.y), width: BEAM_WIDTH }
    }

    /// Along it, from `a` to `b`: level, unit.
    pub fn along(&self) -> Vec3 {
        (self.b - self.a).with_y(0.0).normalize_or(Vec3::Z)
    }

    /// Its top's height.
    pub fn height(&self) -> f32 {
        self.a.y
    }

    /// Its top as a ledge, for the walker's ground (`LedgeGround`): the edge
    /// along one side, the top as deep as the beam is wide.
    pub fn ledge(&self) -> Ledge {
        let along = self.along();
        let out = along.cross(Vec3::Y);
        let middle = 0.5 * (self.a + self.b);
        Ledge::wall(middle.with_y(0.0) + out * (0.5 * self.width), out, (self.b - self.a).with_y(0.0).length(), self.height(), self.width)
    }

    /// How far along it `point` is (from `a`), and how far off its line
    /// across (signed, the world's level).
    pub fn place(&self, point: Vec3) -> (f32, f32) {
        let along = self.along();
        let off = (point - self.a).with_y(0.0);
        (off.dot(along), off.dot(along.cross(Vec3::Y)))
    }

    /// Whether a root at `root` stands on it.
    pub fn holds(&self, root: Vec3) -> bool {
        let (along, across) = self.place(root);
        let length = (self.b - self.a).with_y(0.0).length();
        let below = self.height() - root.y;
        (0.0..=length).contains(&along) && across.abs() <= 0.5 * self.width + ON_MARGIN && (-ON_HEIGHT..=super::fall::STEP_DOWN + 0.01).contains(&below)
    }

    /// Its way nearer `forward` (level): along it or back.
    pub fn way_for(&self, forward: Vec3) -> Vec3 {
        let along = self.along();
        if forward.dot(along) >= 0.0 { along } else { -along }
    }
}

/// The feet's share of the step width on a beam `weight` (0-1) in, in
/// [`FEET_STEPS`] steps.
pub fn feet_apart(weight: f32) -> f32 {
    let stepped = (weight.clamp(0.0, 1.0) * FEET_STEPS).round() / FEET_STEPS;
    1.0 - stepped * (1.0 - BEAM_FEET)
}

/// How far the trunk sways sideways `t` seconds on (radians, toward the
/// rig's left).
pub fn sway_at(t: f32) -> f32 {
    SWAY * (std::f32::consts::TAU * SWAY_RATE * t).sin()
}

/// Balancing on a beam, `weight` (0-1) in, the trunk swaying `sway`
/// (radians toward the rig's left, [`sway_at`]): the trunk bent sideways by
/// it, the arms held out to the sides, the elbows a little bent, the arms
/// tilted against the sway.
pub fn balance(pose: &mut LocalPose, rig: &RigGeometry, weight: f32, sway: f32) {
    if weight <= 0.0 {
        return;
    }
    let (forward, left) = (rig.forward(), rig.left());
    // Sideways about the rig's forward: positive takes the up toward the
    // left (`Y` turned toward `left` is a turn about `left x Y`, which is
    // `-forward` on a right-handed rig, `+forward` on a mirrored one).
    let sideways = left.cross(Vec3::Y).normalize_or(forward);
    pose.rotations[Bone::Spine] = delta_after_world_turn(pose, rig, Bone::Spine, Quat::from_axis_angle(sideways, weight * sway));
    for (chain, sign) in [(ArmChain::LEFT, 1.0f32), (ArmChain::RIGHT, -1.0)] {
        // Up from hanging toward its own side: `-Y` toward `left * sign`.
        let axis = Vec3::NEG_Y.cross(left * sign).normalize_or(forward);
        // Tilted against the sway: the side it sways to raised, a seesaw.
        let out = ARMS_OUT + sign * ARMS_TILT * sway;
        pose.rotations[chain.shoulder] = delta_after_world_turn(pose, rig, chain.shoulder, Quat::from_axis_angle(axis, weight * out));
        // The elbow bent forward, the forearm swung toward the front (bent
        // about the arm's own lift, it folded up and the hand stood by the
        // head).
        let forward_bend = (left * sign).cross(forward).normalize_or(Vec3::Y);
        pose.rotations[chain.elbow] = delta_after_world_turn(pose, rig, chain.elbow, Quat::from_axis_angle(forward_bend, weight * ELBOWS));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gait::{walk_pose_on, GaitParams};
    use crate::character::anim::rig::forward_kinematics_on;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// A beam holds a root over it at its height, not one beside it, past
    /// its ends, or on the floor under it; its top as a ledge covers it.
    #[test]
    fn a_beam_holds_a_root_on_it() {
        let beam = Beam::new(Vec3::new(1.0, 0.4, 0.0), Vec3::new(1.0, 0.4, -4.0));
        assert!(beam.holds(Vec3::new(1.02, 0.4, -2.0)));
        assert!(beam.holds(Vec3::new(1.1, 0.15, -2.0)), "a step below beside its line");
        assert!(!beam.holds(Vec3::new(1.3, 0.4, -2.0)), "beside it");
        assert!(!beam.holds(Vec3::new(1.0, 0.4, 0.5)), "past its end");
        assert!(!beam.holds(Vec3::new(1.0, 0.0, -2.0)), "on the floor under it");
        let ledge = beam.ledge();
        for across in [-0.04, 0.0, 0.04] {
            let point = Vec3::new(1.0 + across, 0.4, -2.0);
            let back = -ledge.out_of(point);
            assert!((0.0..=ledge.depth).contains(&back) && (ledge.height() - 0.4).abs() < 1.0e-6, "{across}: off its top");
        }
    }

    /// Walking on a beam, at its pace, each foot's middle stays within the
    /// beam's half-width of the line under the hips through the whole cycle
    /// (walking, they are 6.5 cm out); and the feet never pass through each
    /// other.
    #[test]
    fn on_a_beam_the_feet_walk_on_its_line() {
        let (stood, rig) = real_stood();
        let left = rig.left();
        let worst = |feet_apart: f32| {
            let params = GaitParams { feet_apart, ..GaitParams::walking_on(BEAM_SPEED, &rig) };
            let mut worst: f32 = 0.0;
            let mut nearest = f32::MAX;
            for i in 0..60 {
                let pose = walk_pose_on(i as f32 / 60.0, &params, &stood, &rig);
                let at = forward_kinematics_on(&pose, &rig);
                let hips = at[Bone::Hips].dot(left);
                let feet = [Bone::LeftFoot, Bone::RightFoot].map(|ankle| at[ankle].dot(left) - hips);
                worst = worst.max(feet[0].abs()).max(feet[1].abs());
                nearest = nearest.min((feet[0] - feet[1]).abs());
            }
            (worst, nearest)
        };
        let (walking, _) = worst(1.0);
        let (on_beam, apart) = worst(BEAM_FEET);
        eprintln!("walking {walking:.4}, on a beam {on_beam:.4}, feet at least {apart:.4} apart");
        assert!(walking > 0.05, "walking, the feet only {walking:.3} m out");
        assert!(on_beam <= 0.5 * BEAM_WIDTH, "on a beam, a foot {on_beam:.4} m off its line");
        assert!(apart >= 0.03, "on a beam, the feet {apart:.4} m apart across");
    }

    /// Balancing, the hands are held out wide, the elbows below the hands'
    /// height no lower than the shoulders' by much, swaying against the
    /// trunk: the hand on the side the trunk sways to higher.
    #[test]
    fn on_a_beam_the_arms_are_held_out_against_the_sway() {
        let (stood, rig) = real_stood();
        let left = rig.left();
        let hands = |sway: f32| {
            let mut pose = stood;
            balance(&mut pose, &rig, 1.0, sway);
            let at = forward_kinematics_on(&pose, &rig);
            [ArmChain::LEFT, ArmChain::RIGHT].map(|chain| (at[chain.wrist] - at[Bone::Hips], at[chain.shoulder] - at[Bone::Hips]))
        };
        let level = hands(0.0);
        for (side, sign) in [(0, 1.0f32), (1, -1.0)] {
            let (wrist, shoulder) = level[side];
            assert!(sign * wrist.dot(left) > sign * shoulder.dot(left) + 0.35, "hand {side} only {:.2} m out from its shoulder", sign * (wrist - shoulder).dot(left));
            assert!(wrist.y > shoulder.y - 0.25, "hand {side} {:.2} m under its shoulder", shoulder.y - wrist.y);
        }
        let swayed = hands(SWAY);
        assert!(swayed[0].0.y > level[0].0.y + 0.02 && swayed[1].0.y < level[1].0.y - 0.02, "swayed left, the hands at {:.3} {:.3} against {:.3} {:.3}", swayed[0].0.y, swayed[1].0.y, level[0].0.y, level[1].0.y);
    }
}
