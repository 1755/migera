//! Windows: step 15 of the parkour steps beyond the first ten (overhangs
//! and windows), first part.
//!
//! A window is its sill, a ledge on the outside, and how high its opening
//! is over it ([`Window`]). Climbing in, a hang from the sill climbs up
//! through the opening (`Hanging::through_window`) and ends crouched on the
//! sill under the lintel, then drops into the room (`Hanging::off_window`).
//! Climbing out, a mantle from the room onto the sill
//! (`Hanging::mantle_through`) ends crouched on it facing out; it turns
//! round on the sill ([`SillTurn`]) and lowers itself down into a hang from
//! it (the climb in played backward, `Hanging::lower_down`).

use bevy::math::{Quat, Vec3};

use super::Ledge;
use crate::character::anim::gait::smoothstep;
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// Turning round on a sill, seconds; each foot steps over this share of
/// it, the second starting as far in, lifted this far, metres.
const TURN: f32 = 1.0;
const STEP: f32 = 0.55;
const SECOND_FROM: f32 = 0.45;
const LIFT: f32 = 0.05;

const LEGS: [(Bone, Bone, Bone); 2] = [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot)];

/// A window: its sill (a ledge on the outside, its depth the wall's
/// thickness), its opening's height over the sill, and the room's floor's
/// height, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Window {
    pub sill: Ledge,
    pub lintel: f32,
    pub room: f32,
}

impl Window {
    /// The sill seen from the room: its line in by the wall's thickness,
    /// facing in, its wall down to the room's floor.
    pub fn inner(&self) -> Ledge {
        let back = -self.sill.out * self.sill.depth;
        Ledge { a: self.sill.b + back, b: self.sill.a + back, out: -self.sill.out, depth: self.sill.depth, wall_below: self.sill.height() - self.room }
    }

    /// Whether `ledge` is this window's sill, from outside (`Some(true)`)
    /// or from the room (`Some(false)`).
    pub fn outside(&self, ledge: &Ledge) -> Option<bool> {
        let same = |a: Vec3, b: Vec3| (a - b).length() < 1.0e-3;
        if same(ledge.a, self.sill.a) && same(ledge.b, self.sill.b) {
            return Some(true);
        }
        let back = -self.sill.out * self.sill.depth;
        (same(ledge.a, self.sill.b + back) && same(ledge.b, self.sill.a + back)).then_some(false)
    }
}

/// One end of a turn on a sill: the pose, the walker's root and facing.
#[derive(Debug, Clone, Copy)]
pub struct Crouch {
    pub pose: LocalPose,
    pub root: Vec3,
    pub yaw: f32,
}

/// Turning round crouched on a sill, from one crouch to another facing the
/// other way: the trunk turned and moved between them, each foot stepping
/// in turn from where it was to where it goes, lifted a little.
#[derive(Debug, Clone)]
pub struct SillTurn {
    from: Crouch,
    to: Crouch,
    /// The hips and each ankle and foot's world rotation at either end (the
    /// world).
    hips: [Vec3; 2],
    ankles: [[Vec3; 2]; 2],
    feet: [[Quat; 2]; 2],
    /// The turn, radians (from one facing to the other, the short way).
    turn: f32,
    rig: RigGeometry,
    t: f32,
}

impl SillTurn {
    pub fn new(from: Crouch, to: Crouch, rig: &RigGeometry) -> Self {
        let world = |c: &Crouch| {
            let (at, rotations) = (forward_kinematics_on(&c.pose, rig), accumulate_world_rotations(&c.pose, rig));
            let turn = Quat::from_rotation_y(c.yaw);
            (c.root + turn * at[Bone::Hips], LEGS.map(|(_, _, ankle)| c.root + turn * at[ankle]), LEGS.map(|(_, _, ankle)| turn * rotations[ankle]))
        };
        let (a, b) = (world(&from), world(&to));
        Self {
            from,
            to,
            hips: [a.0, b.0],
            ankles: [a.1, b.1],
            feet: [a.2, b.2],
            turn: crate::character::anim::facing::shortest_angle(to.yaw - from.yaw),
            rig: rig.clone(),
            t: 0.0,
        }
    }

    pub fn advance(&mut self, dt: f32) {
        self.t = (self.t + dt).min(TURN);
    }

    pub fn is_done(&self) -> bool {
        self.t >= TURN
    }

    fn share(&self) -> f32 {
        smoothstep((self.t / TURN).clamp(0.0, 1.0))
    }

    /// The walker's facing now.
    pub fn facing(&self) -> f32 {
        self.from.yaw + self.turn * self.share()
    }

    /// The pose now and the walker's root.
    fn now(&self) -> (LocalPose, Vec3) {
        let rig = &self.rig;
        let w = self.share();
        let yaw = self.facing();
        let turn = Quat::from_rotation_y(yaw);
        let back = turn.inverse();
        let mut pose = self.from.pose;
        for bone in Bone::ALL {
            pose.rotations[bone] = self.from.pose.rotations[bone].slerp(self.to.pose.rotations[bone], w);
        }
        pose.root_translation = self.from.pose.root_translation.lerp(self.to.pose.root_translation, w);
        let at = forward_kinematics_on(&pose, rig);
        let hips = self.hips[0].lerp(self.hips[1], w);
        let root = hips - turn * at[Bone::Hips];
        // Each foot stepped in turn, the first, then the second from part
        // way through the first.
        let u = self.t / TURN;
        for (side, &(_, _, ankle)) in LEGS.iter().enumerate() {
            let start = if side == 0 { 0.0 } else { SECOND_FROM };
            let s = smoothstep(((u - start) / STEP).clamp(0.0, 1.0));
            let at_now = self.ankles[0][side].lerp(self.ankles[1][side], s) + Vec3::Y * (LIFT * (std::f32::consts::PI * s).sin());
            let at_pose = forward_kinematics_on(&pose, rig);
            place_ankle(&mut pose, rig, ankle, back * (at_now - root) - at_pose[Bone::Hips]);
            let wanted = self.feet[0][side].slerp(self.feet[1][side], s);
            let now = accumulate_world_rotations(&pose, rig)[ankle];
            pose.rotations[ankle] = delta_after_world_turn(&pose, rig, ankle, (back * wanted) * now.inverse());
        }
        (pose, root)
    }

    pub fn pose(&self) -> LocalPose {
        self.now().0
    }

    pub fn root(&self) -> Vec3 {
        self.now().1
    }
}
