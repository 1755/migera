//! Leaping from a hang: step 6 of the parkour design. Up to a higher ledge,
//! back off the wall (turning round in the air, landing or catching a
//! ledge behind), or sideways to the next ledge along.
//!
//! A leap is a launch in the hang and a flight. In the launch the hands
//! keep their hold and the hips go from where they hang to where they let
//! go, on a Hermite curve from the hang's own velocity to the release's: a
//! pull up for a leap up, a swing aside for one aside, a push off the wall
//! with the feet for one back. Let go, the flight is a fall
//! (`parkour::fall`), ballistic, aimed at the ledge leapt at; the release
//! is chosen so the shoulders come to just under its lip at the top of the
//! flight, where the hands catch it (as a jump to a ledge is planned to
//! meet it near its apex, step 1). With no ledge to catch, a leap back
//! falls and lands.
//!
//! There is no data for these leaps beyond a bar release's 73-157 ms window
//! (`parkour-movement-data`); the launch's time and reach are set by eye.

use bevy::math::Vec3;

use super::{Hanging, Ledge, Shimmy};
use crate::character::anim::jump::GRAVITY;
use crate::character::anim::rig::{forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// Which way to leap from a hang.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Leap {
    /// Up to a ledge above.
    Up,
    /// Back off the wall, turning round: catching a ledge behind, facing
    /// the wall, or landing.
    Back,
    /// Sideways to the next ledge along, the character's own left or right.
    Aside(Shimmy),
}

/// A launch under way: its plan, and how far into it.
#[derive(Debug, Clone)]
pub(super) struct Launch {
    t: f32,
    seconds: f32,
    /// The hips and their velocity as it began, and as it lets go.
    from: Vec3,
    from_velocity: Vec3,
    to: Vec3,
    velocity: Vec3,
    /// The ledge leapt at; and the turn in the air, radians.
    target: Option<Ledge>,
    spin: f32,
}

/// How long the launch takes, seconds: a pull, a swing or a push; the hands
/// letting go over its last part.
const LAUNCH: f32 = 0.35;
const LETTING_GO: f32 = 0.12;
/// Letting go, the legs' swing is read over the launch's last this long,
/// seconds.
const LEG_SWING_READ: f32 = 0.01;
/// Letting go, the hands come this far up off the lip, metres.
const HANDS_OFF_LIP: f32 = 0.05;
/// Where the hips let go from, metres: a leap up pulled up this far and out
/// from the face (rising straight up from under the lip, the shoulders and
/// knees went through it, 11 cm); aside swung this far along and up; back
/// pushed this far out from the wall and up.
const PULL_UP: f32 = 0.25;
const PULL_OUT: f32 = 0.25;
const SWING_ALONG: f32 = 0.15;
const SWING_UP: f32 = 0.15;
const PUSH_OUT: f32 = 0.2;
const PUSH_UP: f32 = 0.1;
/// At the top of the flight the shoulders are this far under the lip
/// leapt at, metres (the catch takes them falling past it, step 5).
const UNDER_LIP: f32 = 0.08;
/// Aside or back at a ledge no higher, the flight rises at least this
/// much, metres.
const LEAST_RISE: f32 = 0.2;
/// Caught, the hips this far out from the face, metres: as a braced hang
/// rests (at 0.3, the trunk leant to the grip put the shoulders 9.5 cm off
/// the wall and an elbow 3.9 cm into it).
const CATCH_OUT: f32 = 0.42;
/// Within reach: a ledge up 0.3-1.1 m above the lip held; aside, a gap of
/// up to 1.8 m to the next, its lip within 0.6 m up or down; behind, a face
/// 0.8-3.5 m away, its lip no more than 0.5 m higher.
const UP_REACH: (f32, f32) = (0.3, 1.1);
const ASIDE_GAP: f32 = 1.8;
const ASIDE_RISE: f32 = 0.6;
const BACK_REACH: (f32, f32) = (0.8, 3.5);
const BACK_RISE: f32 = 0.5;
/// Caught, the grip this far in from the ledge's end, metres.
const END_MARGIN: f32 = 0.35;
/// With nothing behind to catch, a leap back leaves at these speeds out and
/// up, m/s; and turns round over this long, seconds.
const BACK_SPEED: (f32, f32) = (2.2, 1.6);
const SPIN_TIME: f32 = 0.5;
/// The fastest it leaves across, and up, m/s (across at 2.5, a gap of 2 m
/// asked 4.9 m/s up, and was not leapt).
const FASTEST: f32 = 3.0;
const MOST_UP: f32 = 4.5;

impl Launch {
    /// The hips at `t` seconds in, and their velocity.
    fn hips_at(&self, t: f32) -> (Vec3, Vec3) {
        let s = (t / self.seconds).clamp(0.0, 1.0);
        let (a, b, va, vb) = (self.from, self.to, self.from_velocity * self.seconds, self.velocity * self.seconds);
        let (s2, s3) = (s * s, s * s * s);
        let at = a * (2.0 * s3 - 3.0 * s2 + 1.0) + va * (s3 - 2.0 * s2 + s) + b * (-2.0 * s3 + 3.0 * s2) + vb * (s3 - s2);
        let rate = (a * (6.0 * s2 - 6.0 * s) + va * (3.0 * s2 - 4.0 * s + 1.0) + b * (-6.0 * s2 + 6.0 * s) + vb * (3.0 * s2 - 2.0 * s)) / self.seconds;
        (at, rate)
    }
}

impl Hanging {
    /// Leaps `way` from the hang, at a ledge among the others about it
    /// within reach (back, with none behind, to land): `false`, and nothing
    /// done, if there is none to leap at, or it is not just hanging.
    pub fn leap(&mut self, way: Leap, rig: &RigGeometry) -> bool {
        if !self.is_hanging() || self.up.is_some() || self.step.is_some() || self.launch.is_some() {
            return false;
        }
        let (out, _) = self.face();
        let from = self.hang_hips();
        let from_velocity = self.hang_velocity();
        let shoulders = self.shoulders_over_hips(rig);
        let lip = self.ledge.nearest(self.grip, 0.0);
        let sideways = |shimmy: Shimmy| {
            let left = self.turn * rig.left();
            if shimmy == Shimmy::Left { left } else { -left }
        };
        let (to, target, spin) = match way {
            Leap::Up => {
                let Some(target) = self.others.iter().copied().filter(|ledge| ledge.out.dot(out) > 0.9).find(|ledge| {
                    let rise = ledge.height() - lip.y;
                    let near = ledge.nearest(lip, END_MARGIN);
                    (UP_REACH.0..=UP_REACH.1).contains(&rise) && (near - lip).with_y(0.0).length() < 0.6
                }) else {
                    return false;
                };
                (from + Vec3::Y * PULL_UP + out * PULL_OUT, Some(target), 0.0)
            }
            Leap::Aside(shimmy) => {
                let way = sideways(shimmy);
                let Some(target) = self
                    .others
                    .iter()
                    .copied()
                    .filter(|ledge| ledge.out.dot(out) > 0.9 && (ledge.height() - lip.y).abs() <= ASIDE_RISE)
                    .filter(|ledge| {
                        let near = ledge.nearest(lip, 0.0);
                        let gap = (near - lip).dot(way);
                        gap > 0.2 && gap <= ASIDE_GAP + END_MARGIN && ledge.out_of(lip).abs() < 0.5
                    })
                    .min_by(|a, b| (a.nearest(lip, 0.0) - lip).length().total_cmp(&(b.nearest(lip, 0.0) - lip).length()))
                else {
                    return false;
                };
                (from + way * SWING_ALONG + Vec3::Y * SWING_UP, Some(target), 0.0)
            }
            Leap::Back => {
                let push = from + out * PUSH_OUT + Vec3::Y * PUSH_UP;
                let target = self.others.iter().copied().filter(|ledge| ledge.out.dot(out) < -0.9 && ledge.height() <= lip.y + BACK_RISE).find(|ledge| {
                    let away = ledge.out_of(push);
                    (BACK_REACH.0..=BACK_REACH.1).contains(&away) && (ledge.nearest(push, END_MARGIN) - push).with_y(0.0).length() < BACK_REACH.1 + 0.5
                });
                (push, target, std::f32::consts::PI)
            }
        };
        let velocity = match target {
            Some(target) => {
                // Caught where the shoulders come to just under its lip, the
                // hips out from its face over the grip's spot on it: at the
                // top of the flight, or later falling past it if crossing
                // takes longer (caught only at the top, a ledge level along
                // was 5.9 m/s across away).
                let lip = target.nearest(to, END_MARGIN);
                let catch_height = lip.y - UNDER_LIP - shoulders.y;
                let catch = (lip + target.out * CATCH_OUT).with_y(catch_height);
                let rise = (catch_height - to.y).max(LEAST_RISE);
                let to_top = (2.0 * rise / GRAVITY).sqrt();
                let seconds = to_top.max((catch - to).with_y(0.0).length() / FASTEST);
                let up = (catch_height - to.y) / seconds + 0.5 * GRAVITY * seconds;
                if up > MOST_UP {
                    return false;
                }
                (catch - to).with_y(0.0) / seconds + Vec3::Y * up
            }
            None => out * BACK_SPEED.0 + Vec3::Y * BACK_SPEED.1,
        };
        self.launch = Some(Launch { t: 0.0, seconds: LAUNCH, from, from_velocity, to, velocity, target, spin });
        true
    }

    /// The shoulders' middle over the hips standing, in the world's axes.
    fn shoulders_over_hips(&self, rig: &RigGeometry) -> Vec3 {
        let at = forward_kinematics_on(&self.body.stood, rig);
        let arms = crate::character::anim::armik::ArmChain::LEFT;
        let other = crate::character::anim::armik::ArmChain::RIGHT;
        self.turn * (0.5 * (at[arms.shoulder] + at[other.shoulder]) - at[Bone::Hips])
    }

    /// Whether it is launching a leap ([`Self::leap`]).
    pub fn is_leaping(&self) -> bool {
        self.launch.is_some()
    }

    /// Whether the launch is done and it lets go ([`Self::release`]).
    pub fn is_released(&self) -> bool {
        self.launch.as_ref().is_some_and(|launch| launch.t >= launch.seconds)
    }

    /// The launch on `dt` seconds.
    pub(super) fn advance_launch(&mut self, dt: f32) -> bool {
        match self.launch.as_mut() {
            Some(launch) => {
                launch.t = (launch.t + dt).min(launch.seconds);
                true
            }
            None => false,
        }
    }

    /// Launching, the hips and their velocity.
    pub(super) fn launch_hips(&self) -> Option<(Vec3, Vec3)> {
        self.launch.as_ref().map(|launch| launch.hips_at(launch.t))
    }

    /// Launching, how far the hands have let go, 0-1: over the launch's last
    /// [`LETTING_GO`] (a bar's release window is 73-157 ms).
    pub(super) fn letting_go(&self) -> f32 {
        self.launch.as_ref().map_or(0.0, |launch| {
            let from = launch.seconds - LETTING_GO;
            crate::character::anim::gait::smoothstep(((launch.t - from) / LETTING_GO).clamp(0.0, 1.0))
        })
    }

    /// Letting go, the arms as they were as it began to (the pose's frame),
    /// which they go to from their hold on the lip, so the hands go with
    /// the body. Held to the lip to the end, the hands went from still to
    /// the body's speed the frame it let go, 7.8 cm; let go to follow the
    /// hips but still reached for, a near-straight arm's elbow jerked 2.3 cm
    /// pushing back off the wall.
    pub(super) fn letting_go_arms(&self, rig: &RigGeometry) -> Option<LocalPose> {
        let launch = self.launch.as_ref()?;
        let from = launch.seconds - LETTING_GO;
        if launch.t <= from {
            return None;
        }
        let mut began = self.clone();
        if let Some(launch) = began.launch.as_mut() {
            launch.t = from;
        }
        // The hands come up off the lip as they let go: held as they were
        // on it while the trunk straightened, the wrists dipped 8 mm into
        // its corner.
        let mut pose = began.pose(rig);
        let lifted = began.wrists().map(|wrist| wrist + Vec3::Y * HANDS_OFF_LIP);
        began.arms_to(&mut pose, rig, began.root(), lifted, [began.hook_turn(0), began.hook_turn(1)], began.elbows_tucked(), 1.0);
        Some(pose)
    }

    /// Let go: the flight (`parkour::fall`) from the hang as it is, at the
    /// release's velocity, aimed at the ledge leapt at (turning round, back),
    /// to the ground `ground` finds under it (its height at a point) or a
    /// top it comes down on; kept off `walls` it faces.
    pub fn release(&self, ground: &dyn Fn(Vec3) -> Option<f32>, walls: &[Ledge], stood: &LocalPose, rig: &RigGeometry) -> super::super::Falling {
        let launch = self.launch.as_ref().expect("released from a launch");
        let pose = self.pose(rig);
        let root = self.root();
        let below = ground(root).unwrap_or(0.0);
        let mut falling = super::super::Falling::off(root, self.yaw, launch.velocity, &pose, below, self.drop, stood, rig);
        // The legs as they swing in the launch's last moment: from rest, a
        // hanging toe's step changed 2.2 cm the frame it let go.
        let ankles = |hanging: &Hanging| {
            let at = forward_kinematics_on(&hanging.pose(rig), rig);
            super::LEGS.map(|(_, _, ankle, _)| self.turn * (at[ankle] - at[Bone::Hips]))
        };
        let mut earlier = self.clone();
        if let Some(launch) = earlier.launch.as_mut() {
            launch.t -= LEG_SWING_READ;
        }
        let (now, then) = (ankles(self), ankles(&earlier));
        falling.coast_legs([0, 1].map(|side| (now[side] - then[side]) / LEG_SWING_READ));
        // And the trunk and arms as they turn, carried on as far ahead as
        // the fall reads them (from rest, the head's step changed 2 cm).
        let before = earlier.pose(rig);
        let mut ahead = pose;
        let on = 1.0 + super::super::Falling::COAST_AHEAD / LEG_SWING_READ;
        for bone in Bone::ALL {
            ahead.rotations[bone] = before.rotations[bone].slerp(pose.rotations[bone], on);
        }
        falling.coast_upper(ahead);
        if launch.spin != 0.0 {
            falling.spin_round(launch.spin, SPIN_TIME, rig);
        }
        falling.land_on(ground);
        if let Some(target) = launch.target {
            falling.aim_at(target);
        }
        // Kept off the wall it faces, leaping at a ledge or not (up, the
        // feet left on the wall below went into it, 9 cm).
        falling.against(walls, rig);
        falling
    }

    /// The ledge leapt at, if any.
    pub fn leap_target(&self) -> Option<Ledge> {
        self.launch.as_ref().and_then(|launch| launch.target)
    }
}
