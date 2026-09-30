//! Standing balance after a push: the body as Winter's inverted pendulum,
//! and the step it takes when standing cannot catch it.
//!
//! Winter §11.2.1 (Eq. 11.3, after §5.2.9): the centre of pressure (COP)
//! under the feet steers the centre of mass (COM),
//!
//! ```text
//! COP − COM = −K · COM̈,   K = I / (W d) ≈ d / g
//! ```
//!
//! so standing balance is control of the COP. Front to back the ankles move
//! it; side to side the hips load one leg and unload the other. Quiet
//! standing is "stiffness, not reaction" (Winter et al., 1998): the COP runs
//! in phase with the COM and slightly wider, which always pushes the COM
//! back to the middle.
//!
//! [`Balance`] is that pendulum with a COP law `COP = COM + s·x + b·ẋ`: the
//! stiffness term Winter measured, plus damping for the reactive response to
//! an unexpected push, which he says the sensors stand by for. The law
//! makes `COM̈ = −(s·x + b·ẋ)/K`, a damped return at `ω² = s/K`. Winter
//! gives the structure, not a push response's gains: [`RECOVERY_FREQUENCY`]
//! and [`RECOVERY_DAMPING`] are this module's choice.
//!
//! # A push the feet cannot absorb: a step
//!
//! The COP cannot leave the feet ([`Support`]). When the capture point
//! `x + ẋ·√K` — where the COP would have to stand to stop the body — has
//! left them, standing cannot catch the body, and it steps: the foot on the
//! side the capture point left through (or the unloaded one, for a push
//! straight ahead or back) swings to where the capture point will be when
//! it lands, [`STEP_SECONDS`] later. Through the swing only the other foot
//! supports; on landing both do, and the pendulum brings the body to rest
//! over them. The trailing foot then joins it, and [`Balance::rebase`] hands
//! the distance travelled to the caller, who moves the character by it. The
//! capture point is a later formalisation of Winter's pendulum, not his; the
//! step's timing is a walking swing's, and its placement the pendulum's own
//! prediction.
//!
//! Axes are the rig's own, horizontal: `x` along [`RigGeometry::forward`],
//! `y` along [`RigGeometry::left`]. Positions are from the COM's rest place
//! over the feet as they stood.

use bevy::math::{Vec2, Vec3};

use super::foot::{Sole, FOOT_BREADTH_PER_LENGTH};
use super::rig::{offset_from, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// How fast a pushed body returns, rad/s: `ω` of the COP law. Critically
/// damped at this, a 0.2 m/s push peaks at 2.1 cm after ~0.3 s and is back
/// within 2 mm and 2 mm/s after ~1.8 s. Not Winter's number; see the module
/// docs.
pub const RECOVERY_FREQUENCY: f32 = 3.5;

/// The COP law's damping ratio. 1 returns without overshoot.
pub const RECOVERY_DAMPING: f32 = 1.0;

/// How far inside the feet the COP must stay, metres: a COP at the very
/// edge of the sole is a foot about to roll.
pub const SUPPORT_MARGIN: f32 = 0.01;

/// Winter's bound on the inverted-pendulum model: sway angles under 8°
/// (Winter et al., 1996).
pub const MAX_SWAY_ANGLE: f32 = 8.0 * std::f32::consts::PI / 180.0;

/// How long a recovery step swings, seconds: about a walking swing's
/// (Winter's stride, ~40 % swing of ~1 s), a little quicker for a stumble.
pub const STEP_SECONDS: f32 = 0.3;

/// How long the trailing foot takes to join the stepped one, seconds: an
/// unhurried step, once the body is caught.
pub const JOIN_SECONDS: f32 = 0.45;

/// The longest recovery step, metres of foot travel: a long walking step on
/// `puppet_base`'s legs is ~0.6 m, but the step starts from standing.
pub const MAX_STEP: f32 = 0.4;

/// The longest step a push may ask for and still be caught, metres: past
/// it the body falls ([`Balance::falls`]).
///
/// The model's own verdict, measured on `puppet_base`
/// (`a_push_past_a_catchable_step_falls`). Forward and back, a push is
/// caught while its planned step (from the whole push) asks ≤ 0.73 m, and
/// runs away past 0.87 m once the validity clamp is lifted. Sideways the
/// pendulum cannot tell: stepping off the far foot, the COM starts at the
/// 8° bound, and every side step there is caught by the clamp.
pub const MAX_CATCH: f32 = 0.8;

/// How high a stepping foot is lifted at mid-swing, metres.
pub const STEP_LIFT: f32 = 0.05;

/// How long a push takes to deliver, seconds: a shove, not a collision.
///
/// Delivered in one frame, a 0.8 m/s push moved the pelvis 13 mm a frame
/// while the legs' 0.015 s springs lagged it, and the rendered toes moved
/// fast enough for the foot locks to let go and re-plant 23 mm away.
pub const PUSH_SECONDS: f32 = 0.1;

/// Gravity, m/s².
const GRAVITY: f32 = 9.81;

/// Integration substeps per [`Balance::step`]: the pendulum's own unstable
/// pole is ~3 rad/s, well inside one 60 Hz frame, but a saturated COP
/// changes the dynamics abruptly and substeps keep that edge clean.
const SUBSTEPS: usize = 4;

/// An axis-aligned box in the balance axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Area {
    /// The most backward (`x`) and rightward (`y`) corner.
    pub min: Vec2,
    /// The most forward and leftward.
    pub max: Vec2,
}

impl Area {
    /// Whether `point` is inside.
    pub fn contains(&self, point: Vec2) -> bool {
        point.cmpge(self.min).all() && point.cmple(self.max).all()
    }

    /// This box moved by `by`.
    pub fn shifted(&self, by: Vec2) -> Self {
        Self { min: self.min + by, max: self.max + by }
    }

    /// The smallest box holding both.
    pub fn union(&self, other: &Self) -> Self {
        Self { min: self.min.min(other.min), max: self.max.max(other.max) }
    }

    /// Its middle.
    pub fn centre(&self) -> Vec2 {
        (self.min + self.max) * 0.5
    }
}

/// Where the COP can go: each foot's sole as it stood, relative to the
/// COM's rest place, less [`SUPPORT_MARGIN`].
///
/// A box per foot: heel to toe tip, and Winter's breadth across the sole's
/// centreline (§4.0.1). With both feet down the COP can be anywhere in the
/// box around both; a box rather than their hull, which with the feet side
/// by side is the same.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Support {
    /// The left and right feet, where they stood.
    pub feet: [Area; 2],
    /// How far the COM may stand from the middle of its support in either
    /// plane: Winter's pendulum is valid only within [`MAX_SWAY_ANGLE`].
    pub valid: f32,
}

impl Support {
    /// The support under `pose`, a standing pose, on `rig`: every `Sole`
    /// contact of each foot, relative to the pose's centre of mass.
    pub fn of(pose: &LocalPose, rig: &RigGeometry) -> Self {
        let com = super::anthropometry::centre_of_mass(pose, rig);
        let axes = |p: Vec3| Vec2::new((p - com).dot(rig.forward()), (p - com).dot(rig.left()));
        let feet = [Bone::LeftFoot, Bone::RightFoot].map(|ankle| {
            let points = Sole::of(rig, ankle).points(pose, rig).map(axes);
            let mut min = points[0].min(points[1]).min(points[2]);
            let mut max = points[0].max(points[1]).max(points[2]);
            let half_breadth = (max.x - min.x) * FOOT_BREADTH_PER_LENGTH * 0.5;
            min.y -= half_breadth;
            max.y += half_breadth;
            Area { min: min + Vec2::splat(SUPPORT_MARGIN), max: max - Vec2::splat(SUPPORT_MARGIN) }
        });
        Self { feet, valid: MAX_SWAY_ANGLE.tan() * pendulum_k(pose, rig) * GRAVITY }
    }

    /// Where the COP can go with the feet moved by `feet` (left, right),
    /// only those present bearing weight.
    pub fn under(&self, feet: [Option<Vec2>; 2]) -> Area {
        let mut area: Option<Area> = None;
        for (leg, at) in feet.iter().enumerate() {
            if let Some(at) = at {
                let foot = self.feet[leg].shifted(*at);
                area = Some(area.map_or(foot, |a| a.union(&foot)));
            }
        }
        area.unwrap_or(self.feet[0].union(&self.feet[1]))
    }
}

/// The pendulum constant `K = d/g` for `pose` on `rig`, s²: `d` the centre
/// of mass's height above the ankles, as a point mass. Winter's is somewhat
/// larger for a distributed body (~0.1 s² at d ≈ 0.9 m).
pub fn pendulum_k(pose: &LocalPose, rig: &RigGeometry) -> f32 {
    let ankles = 0.5
        * (offset_from(pose, rig, Bone::Hips, Bone::LeftFoot).y
            + offset_from(pose, rig, Bone::Hips, Bone::RightFoot).y);
    let height = super::anthropometry::centre_of_mass(pose, rig).y - ankles;
    height.max(0.0) / GRAVITY
}

/// A foot swinging to a new place.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Swing {
    /// Which foot: 0 the left, 1 the right.
    pub leg: usize,
    /// Where it swings from and to, as displacements from where it stood.
    pub from: Vec2,
    /// See `from`.
    pub to: Vec2,
    /// Seconds into the swing.
    pub elapsed: f32,
    /// How long the swing takes.
    pub duration: f32,
    /// Whether this is the trailing foot joining the stepped one.
    pub joining: bool,
}

impl Swing {
    /// How far through the swing, 0 to 1.
    pub fn progress(&self) -> f32 {
        if self.duration > 0.0 { (self.elapsed / self.duration).clamp(0.0, 1.0) } else { 1.0 }
    }

    /// The foot's displacement now: eased from `from` to `to`.
    pub fn at(&self) -> Vec2 {
        let t = self.progress();
        self.from + (self.to - self.from) * (t * t * (3.0 - 2.0 * t))
    }

    /// How high the foot is lifted now, metres: an arc peaking at
    /// [`STEP_LIFT`] mid-swing.
    pub fn lift(&self) -> f32 {
        let t = self.progress();
        4.0 * STEP_LIFT * t * (1.0 - t)
    }
}

/// A standing body's balance: where its centre of mass is, how it moves,
/// where the pressure under its feet is holding it, and any step it is
/// taking to stay up.
#[derive(bevy::ecs::component::Component, Debug, Clone, Copy, PartialEq, Default)]
pub struct Balance {
    /// The COM's offset from its rest place over the feet as they stood,
    /// metres.
    pub offset: Vec2,
    /// Its velocity, m/s.
    pub velocity: Vec2,
    /// Where the COP stood on the last step: the law's wish, clamped to the
    /// feet bearing weight.
    pub pressure: Vec2,
    /// Whether the capture point `offset + velocity·√K` is outside the feet
    /// bearing weight: standing cannot catch the body.
    pub needs_step: bool,
    /// Each foot's displacement from where it stood (left, right), metres.
    pub feet: [Vec2; 2],
    /// A foot swinging, if one is.
    pub swing: Option<Swing>,
    /// Set once both feet have stepped by the same distance and the body is
    /// caught: how far the character should move. See [`Balance::rebase`].
    pub travelled: Option<Vec2>,
    /// The foot the weight is moving onto before the other joins it, with
    /// both still down. Lifted with the COM between the feet, the trailing
    /// foot left it behind the front foot's heel on one foot, and the body
    /// fell back into a second step.
    pub transfer: Option<usize>,
    /// How long the last recovery step wanted to be, metres, before
    /// [`MAX_STEP`] clamped it.
    pub wanted_step: f32,
    /// Set once a push has asked for a step longer than [`MAX_CATCH`]: no
    /// step catches this body, and it is falling. The pendulum goes on
    /// posing a clamped step, so a character without physics still
    /// stumbles; one with a ragdoll hands it over (H2). Cleared by
    /// [`Balance::default`].
    pub falls: bool,
    /// The last swing to land, and the seconds since, for
    /// [`transition::LAND_HOLD`](super::transition::LAND_HOLD): see
    /// [`Balance::landing_spot`].
    landed: Option<(Swing, f32)>,
    /// Velocity still to be delivered by pushes under way, m/s.
    shove: Vec2,
    /// The rate it is delivered at, m/s².
    shove_rate: Vec2,
}

impl Balance {
    /// A push: a change in the COM's velocity, m/s (an impulse over the
    /// body's mass), delivered over [`PUSH_SECONDS`].
    pub fn push(&mut self, velocity: Vec2) {
        self.shove += velocity;
        self.shove_rate = self.shove / PUSH_SECONDS;
    }

    /// Whether the body is at rest over its feet as they stood, within
    /// `tolerance` metres and metres per second: no push under way, no step
    /// taken or to be handed over.
    pub fn is_settled(&self, tolerance: f32) -> bool {
        self.offset.length() < tolerance
            && self.velocity.length() < tolerance
            && self.shove == Vec2::ZERO
            && self.swing.is_none()
            && self.feet == [Vec2::ZERO; 2]
            && self.travelled.is_none()
            && self.landed.is_none()
    }

    /// Which feet are down (left, right): all but a swinging one. For the
    /// foot IK (`plugin::AnimFootIk::planted`).
    pub fn planted(&self) -> [bool; 2] {
        [0, 1].map(|leg| self.swing.is_none_or(|swing| swing.leg != leg))
    }

    /// The caller has moved the character by [`Balance::travelled`]: the
    /// feet stand where they are now, so everything is measured from there.
    pub fn rebase(&mut self) {
        if let Some(by) = self.travelled.take() {
            self.offset -= by;
            self.pressure -= by;
            self.feet = [Vec2::ZERO; 2];
            if let Some((swing, _)) = &mut self.landed {
                swing.from -= by;
                swing.to -= by;
            }
        }
    }

    /// Where the COM comes to rest over the feet bearing weight: where it
    /// stood, carried with them — on both, by their mean displacement; on
    /// one, over that foot's sole sideways.
    fn rest(&self, support: &Support) -> Vec2 {
        let over = |leg: usize, foot: Vec2| Vec2::new(foot.x, support.feet[leg].shifted(foot).centre().y);
        match self.bearing() {
            [Some(left), Some(right)] => match self.transfer {
                Some(0) => over(0, left),
                Some(_) => over(1, right),
                None => (left + right) * 0.5,
            },
            [Some(foot), None] => over(0, foot),
            [None, Some(foot)] => over(1, foot),
            [None, None] => Vec2::ZERO,
        }
    }

    /// The feet bearing weight now, displaced: the swinging one is not.
    fn bearing(&self) -> [Option<Vec2>; 2] {
        let mut feet = self.feet.map(Some);
        if let Some(swing) = self.swing {
            feet[swing.leg] = None;
        }
        feet
    }

    /// Advances the pendulum, and any step, by `dt` seconds on `support`,
    /// with constant `k` (s², [`pendulum_k`]).
    pub fn step(&mut self, support: &Support, k: f32, dt: f32) {
        if dt <= 0.0 || !dt.is_finite() || k <= 0.0 {
            return;
        }
        if let Some((_, since)) = &mut self.landed {
            *since += dt;
            if *since >= super::transition::LAND_HOLD {
                self.landed = None;
            }
        }
        let stiffness = k * RECOVERY_FREQUENCY * RECOVERY_FREQUENCY;
        let damping = 2.0 * RECOVERY_DAMPING * RECOVERY_FREQUENCY * k;
        let area = support.under(self.bearing());
        let rest = self.rest(support);
        let h = dt / SUBSTEPS as f32;
        for _ in 0..SUBSTEPS {
            // The push under way, a step's worth of it, never past what is left.
            let delivered = Vec2::new(
                (self.shove_rate.x * h).abs().min(self.shove.x.abs()) * self.shove.x.signum(),
                (self.shove_rate.y * h).abs().min(self.shove.y.abs()) * self.shove.y.signum(),
            );
            self.velocity += delivered;
            self.shove -= delivered;
            if self.shove.length_squared() < 1.0e-12 {
                self.shove = Vec2::ZERO;
            }
            let from_rest = self.offset - rest;
            let wanted = self.offset + from_rest * stiffness + self.velocity * damping;
            self.pressure = wanted.clamp(area.min, area.max);
            let acceleration = (self.offset - self.pressure) / k;
            self.velocity += acceleration * h;
            self.offset += self.velocity * h;
            // Outside the model — leaning more than it allows beyond the feet
            // bearing weight — held at its bound. Measured from the rest
            // point instead, it jumped when the rest moved onto a stepped
            // foot, and threw the COM 6 cm in a frame.
            let inside = self.offset.clamp(area.min, area.max);
            for axis in 0..2 {
                let lean = self.offset[axis] - inside[axis];
                if lean.abs() > support.valid {
                    self.offset[axis] = inside[axis] + lean.clamp(-support.valid, support.valid);
                    self.velocity[axis] = 0.0;
                }
            }
        }
        let capture = self.offset + self.velocity * k.sqrt();
        self.needs_step = !area.contains(capture);

        // A swing under way lands when it is done.
        if let Some(mut swing) = self.swing {
            swing.elapsed += dt;
            self.feet[swing.leg] = swing.at();
            if swing.progress() >= 1.0 {
                self.feet[swing.leg] = swing.to;
                self.swing = None;
                self.landed = Some((swing, 0.0));
                if swing.joining && self.feet[0].distance(self.feet[1]) < 1.0e-4 {
                    self.transfer = None;
                    self.travelled = Some(self.feet[0]);
                }
            } else {
                self.swing = Some(swing);
            }
            return;
        }
        if self.travelled.is_some() {
            return;
        }

        let caught = self.velocity.length() < 0.05 && (self.offset - rest).length() < 0.02;
        let (stepped, trailing) = if self.feet[0].length() > self.feet[1].length() { (0, 1) } else { (1, 0) };
        // Carried onto the stepped foot: its momentum will bring the COM
        // over that foot alone. Held down until the COM was over it, the
        // trailing leg kept the pelvis in reach of a foot 0.4 m away, and
        // the pelvis sank 14 cm forward, 21 cm sideways.
        let carried = {
            let margin = Vec2::splat(SUPPORT_MARGIN);
            let alone = support.feet[stepped].shifted(self.feet[stepped]);
            capture.cmpge(alone.min + margin).all() && capture.cmple(alone.max - margin).all()
        };
        if self.needs_step {
            self.transfer = None;
            // Planned for the whole push, the part still to come too:
            // planned mid-push, a 1.2 m/s shove asked no longer a step
            // than 0.8 m/s.
            self.plan_recovery_step(support, k, capture + self.shove * k.sqrt());
        } else if self.feet[0] != self.feet[1] && (caught || self.transfer.is_some() && carried) {
            if self.transfer.is_none() {
                // Caught between the feet: the weight moves onto the
                // stepped foot first, both still down...
                self.transfer = Some(stepped);
            } else {
                // ...and once it is there, the trailing foot joins it.
                self.swing = Some(Swing {
                    leg: trailing,
                    from: self.feet[trailing],
                    to: self.feet[stepped],
                    elapsed: 0.0,
                    duration: JOIN_SECONDS,
                    joining: true,
                });
            }
        }
    }

    /// Plans the step that catches a body whose capture point is at
    /// `capture`, outside its feet.
    fn plan_recovery_step(&mut self, support: &Support, k: f32, capture: Vec2) {
        let both = support.under(self.feet.map(Some));
        // The foot on the side the capture point left through; straight
        // ahead or back, the one not carrying the weight.
        let leg = if capture.y > both.max.y {
            0
        } else if capture.y < both.min.y || self.offset.y >= both.centre().y {
            1
        } else {
            0
        };
        let stance = 1 - leg;
        // Through the swing the COP stands on the other foot, and the
        // capture point runs away from it: `cp − p` grows as `e^{t/√K}`.
        let pressure = support.feet[stance].shifted(self.feet[stance]).centre();
        let landing = pressure + (capture - pressure) * (STEP_SECONDS / k.sqrt()).exp();
        // Put the swinging foot's middle there, keeping its side unless the
        // push is sideways.
        let foot = support.feet[leg].shifted(self.feet[leg]);
        let mut wanted = landing - foot.centre();
        if capture.y <= both.max.y && capture.y >= both.min.y {
            wanted.y = 0.0;
        }
        self.wanted_step = wanted.length();
        self.falls |= self.wanted_step > MAX_CATCH;
        let travel = wanted.clamp_length_max(MAX_STEP);
        self.swing = Some(Swing {
            leg,
            from: self.feet[leg],
            to: self.feet[leg] + travel,
            elapsed: 0.0,
            duration: STEP_SECONDS,
            joining: false,
        });
    }

    /// The COM's offset in the rig's frame, metres, horizontal.
    pub fn offset_on(&self, rig: &RigGeometry) -> Vec3 {
        on(rig, self.offset)
    }

    /// Poses the balance on `pose`, a standing one: each displaced or
    /// swinging foot placed where it is (its attitude kept), then the body
    /// carried over the feet to the COM's offset, the planted feet staying
    /// exactly where they are.
    ///
    /// Front to back, the pelvis carries the trunk over the ankles (Winter:
    /// the ankle strategy). Side to side, the hips load one leg and unload
    /// the other (the load/unload mechanism): the pelvis moves toward the
    /// loaded foot and drops on the unloaded side, like
    /// [`super::stance::shift_weight`], scaled continuously. The pelvis
    /// moves [`COM_PER_PELVIS`] further than the COM is to move.
    pub fn apply(&self, pose: &mut LocalPose, rig: &RigGeometry) {
        use super::stance::{move_pelvis_and_feet, MAX_HEEL_RISE, WEIGHT_SHIFT, WEIGHT_SHIFT_ROLL};
        let moved = [0, 1].map(|leg| {
            let lift = self.swing.filter(|swing| swing.leg == leg).map_or(0.0, |swing| swing.lift());
            on(rig, self.feet[leg]) + Vec3::Y * lift
        });
        let feet = self.bearing();
        let middle = {
            let present: Vec<Vec2> = feet.iter().flatten().copied().collect();
            present.iter().copied().sum::<Vec2>() / present.len().max(1) as f32
        };
        // +1 fully onto the left leg, as `shift_weight` counts it; one foot
        // down carries everything.
        let onto = match feet {
            [Some(_), None] => 1.0,
            [None, Some(_)] => -1.0,
            _ => ((self.offset.y - middle.y) / WEIGHT_SHIFT).clamp(-1.0, 1.0),
        };
        let roll = bevy::math::Quat::from_axis_angle(rig.forward(), onto * WEIGHT_SHIFT_ROLL);
        // A foot down bears some weight however far the body leans off it:
        // at none, the pelvis stopped keeping that leg in reach, and the
        // rear foot lifted 38 mm while the weight moved onto the stepped one.
        // The swinging foot bears none, but is placed on its arc here, so
        // its leg keeps the pelvis in reach of it too: let go, the pelvis
        // sprang up 132 mm in the frame the trailing foot lifted to join.
        let loads = [(0, 0.5 + 0.5 * onto), (1, 0.5 - 0.5 * onto)]
            .map(|(leg, load)| if feet[leg].is_some() { load.max(0.1) } else { 0.1 });
        // Where the feet have stepped the whole body has gone with them;
        // only the sway over them is the pendulum's, scaled by
        // `COM_PER_PELVIS`. Scaling all of it put the pelvis 60 mm ahead of
        // itself by the end of a 0.4 m step, and the hand-over popped it back.
        let carried = (self.feet[0] + self.feet[1]) * 0.5;
        let pelvis = carried + (self.offset - carried) / COM_PER_PELVIS;
        move_pelvis_and_feet(pose, rig, on(rig, pelvis), roll, loads, moved, MAX_HEEL_RISE);
    }

    /// Where the swinging foot's toe joint will stand, in `pose`'s frame
    /// (a standing pose, the feet where they stood), for the foot IK's
    /// landing (`plugin::AnimFootIk::landing`): `(left, spot, strength)`.
    ///
    /// The strength eases in over the first quarter of the swing: at full
    /// strength from lift-off the rendered ball, still 0.4 m from its spot,
    /// popped up [`LAND_LIFT`](super::transition::LAND_LIFT) in a frame.
    /// And it fades out over [`LAND_HOLD`](super::transition::LAND_HOLD)
    /// past the landing: the sprung foot is still arriving, and let go with
    /// the swing it dropped 21 mm in a frame; held full strength to the end
    /// of the hold, a sprung toe still 5 cm short dropped 20 mm there.
    pub fn landing_spot(&self, pose: &LocalPose, rig: &RigGeometry) -> Option<(bool, Vec3, f32)> {
        let (swing, strength) = match (self.swing, self.landed) {
            (Some(swing), _) => {
                let t = (swing.progress() / 0.25).min(1.0);
                (swing, t * t * (3.0 - 2.0 * t))
            }
            (None, Some((swing, since))) => {
                let t = (since / super::transition::LAND_HOLD).min(1.0);
                (swing, 1.0 - t * t * (3.0 - 2.0 * t))
            }
            (None, None) => return None,
        };
        let toe = if swing.leg == 0 { Bone::LeftToeBase } else { Bone::RightToeBase };
        let stood = pose.root_translation + offset_from(pose, rig, Bone::Hips, toe);
        Some((swing.leg == 0, stood + on(rig, swing.to), strength))
    }
}

/// A balance-axes vector in the rig's frame.
fn on(rig: &RigGeometry, v: Vec2) -> Vec3 {
    rig.forward() * v.x + rig.left() * v.y
}

/// How far the centre of mass moves per metre the pelvis is carried over
/// the planted feet by [`Balance::apply`], front to back and side to side:
/// under 1, since the legs pivot at the ankles and only the body above the
/// pelvis travels the whole way; less side to side, where the roll lowers
/// the unloaded side. Measured on `puppet_base` standing, pinned by
/// `the_com_moves_where_the_balance_puts_it`.
pub const COM_PER_PELVIS: Vec2 = Vec2::new(0.863, 0.810);

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;
    const K: f32 = 0.104;

    /// `puppet_base`-sized feet side by side, 23 cm apart, 26 cm long and
    /// 9.5 cm wide, the COM over their middle a little behind the balls.
    fn feet() -> Support {
        let foot = |y: f32| Area { min: Vec2::new(-0.07, y - 0.037), max: Vec2::new(0.17, y + 0.037) };
        Support { feet: [foot(0.115), foot(-0.115)], valid: 0.14 }
    }

    /// Runs `balance` for `seconds`, handing over any step travelled as the
    /// caller would; calls `each` after every frame.
    fn run(balance: &mut Balance, seconds: f32, mut each: impl FnMut(&Balance)) {
        for _ in 0..(seconds / DT) as usize {
            balance.step(&feet(), K, DT);
            each(balance);
            balance.rebase();
        }
    }

    #[test]
    fn a_push_sways_and_returns() {
        // A 0.2 m/s shove forward: absorbed by the feet, back at rest
        // within 2 s, no step.
        let mut balance = Balance::default();
        balance.push(Vec2::new(0.2, 0.0));
        let (mut furthest, mut settled_at, mut frame) = (0.0f32, None, 0);
        run(&mut balance, 4.0, |b| {
            furthest = furthest.max(b.offset.x);
            assert!(b.swing.is_none(), "a gentle push should not need a step");
            assert!(b.offset.x > -1.0e-3, "critically damped: no swing back past rest, got {}", b.offset.x);
            if settled_at.is_none() && b.is_settled(0.002) {
                settled_at = Some(frame);
            }
            frame += 1;
        });
        let settled = settled_at.expect("the body should come back to rest") as f32 * DT;
        assert!(settled < 2.0, "back at rest after {settled} s");
        assert!((0.018..0.025).contains(&furthest), "a critically damped return peaks at v/(ω·e) = 2.1 cm, got {furthest}");
    }

    #[test]
    fn the_cop_leads_the_com_with_the_opposite_sign() {
        // Winter's Eq. 11.3: COP − COM = −K·COM̈. Swaying forward and
        // slowing, the COP stands ahead of the COM.
        let mut balance = Balance::default();
        balance.push(Vec2::new(0.2, 0.0));
        let mut previous = balance.velocity;
        for _ in 0..30 {
            // Eq. 11.3 accounts for the COP and gravity only: while the push
            // itself is still acting, it does not hold.
            let pushed = balance.shove != Vec2::ZERO;
            balance.step(&feet(), K, DT);
            let acceleration = (balance.velocity - previous) / DT;
            previous = balance.velocity;
            if pushed || balance.shove != Vec2::ZERO {
                continue;
            }
            let error = balance.pressure - balance.offset;
            assert!(error.x * acceleration.x <= 0.0, "COP − COM {error} and COM̈ {acceleration} should oppose");
        }
    }

    #[test]
    fn a_push_the_feet_cannot_absorb_takes_a_step_and_is_caught() {
        // 0.6 m/s forward: the capture point leaves the feet, one foot steps
        // ahead, the other joins it, and the character has moved on. The COP
        // never leaves the feet bearing weight.
        let mut balance = Balance::default();
        balance.push(Vec2::new(0.6, 0.0));
        let (mut steps, mut moved, mut previous_swing) = (0, Vec2::ZERO, None);
        for _ in 0..(6.0 / DT) as usize {
            // The feet bearing weight through this frame's step.
            let area = feet().under(balance.bearing());
            balance.step(&feet(), K, DT);
            assert!(area.contains(balance.pressure), "the COP left the feet bearing weight: {}", balance.pressure);
            if balance.swing.is_some() && previous_swing.is_none() {
                steps += 1;
            }
            previous_swing = balance.swing;
            if let Some(by) = balance.travelled {
                moved += by;
            }
            balance.rebase();
        }
        assert_eq!(steps, 2, "a step, then the trailing foot joining it");
        assert!(moved.x > 0.1 && moved.y.abs() < 0.01, "the character should have stepped forward, moved {moved}");
        assert!(balance.is_settled(0.005), "caught and at rest: at {} moving {}", balance.offset, balance.velocity);
    }

    #[test]
    fn a_sideways_shove_steps_with_the_foot_on_that_side() {
        let mut balance = Balance::default();
        balance.push(Vec2::new(0.0, 0.7));
        let mut first = None;
        run(&mut balance, 6.0, |b| {
            if first.is_none() {
                first = b.swing;
            }
        });
        let first = first.expect("a 0.7 m/s sideways shove needs a step");
        assert_eq!(first.leg, 0, "pushed to its left, the left foot steps out");
        assert!(first.to.y > 0.05, "out to the left, went {}", first.to);
        assert!(balance.is_settled(0.005), "caught and at rest: at {} moving {}", balance.offset, balance.velocity);
    }

    #[test]
    fn a_sideways_push_within_the_feet_returns_without_a_step() {
        let mut balance = Balance::default();
        balance.push(Vec2::new(0.0, -0.15));
        run(&mut balance, 4.0, |b| assert!(b.swing.is_none(), "0.15 m/s sideways needs no step"));
        assert!(balance.is_settled(0.002), "still at {} moving {}", balance.offset, balance.velocity);
    }

    fn real_stood() -> (LocalPose, RigGeometry) {
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = crate::character::anim::gltf_rig::puppet_base();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    #[test]
    fn the_com_moves_where_the_balance_puts_it() {
        // `apply` moves the pelvis so the WHOLE-body centre of mass lands on
        // the balance's offset: measured by `centre_of_mass`, not assumed.
        use crate::character::anim::anthropometry::centre_of_mass;
        let (stood, rig) = real_stood();
        let com = |pose: &LocalPose| pose.root_translation + centre_of_mass(pose, &rig);
        for offset in [Vec2::new(0.03, 0.0), Vec2::new(-0.03, 0.0), Vec2::new(0.0, 0.03), Vec2::new(0.0, -0.03)] {
            let balance = Balance { offset, ..Default::default() };
            let mut pose = stood;
            balance.apply(&mut pose, &rig);
            let moved = com(&pose) - com(&stood);
            let horizontal = Vec3::new(moved.x, 0.0, moved.z);
            let off = (horizontal - balance.offset_on(&rig)).length();
            // 2 mm: a side-to-side sway also carries the COM 1.4 mm forward,
            // from rolling the pelvis over the loaded hip.
            assert!(off < 2.0e-3, "for {offset} the COM moved {horizontal:?}, {:.1} mm off", off * 1e3);
        }
    }

    fn soles(pose: &LocalPose, rig: &RigGeometry) -> [[Vec3; 3]; 2] {
        [Bone::LeftFoot, Bone::RightFoot].map(|ankle| {
            let hips = pose.root_translation + offset_from(pose, rig, Bone::Hips, Bone::Hips);
            Sole::of(rig, ankle).points(pose, rig).map(|p| p + hips)
        })
    }

    #[test]
    fn a_swaying_body_keeps_its_feet_planted() {
        // Every sole contact exactly where it stood, at the full support's
        // reach, in the target pose.
        let (stood, rig) = real_stood();
        let support = Support::of(&stood, &rig);
        let both = support.under([Some(Vec2::ZERO); 2]);
        let corners = [both.min, both.max, Vec2::new(both.max.x, both.min.y), Vec2::new(both.min.x, both.max.y)];
        for offset in corners.map(|c| c.clamp(Vec2::splat(-support.valid), Vec2::splat(support.valid))) {
            let mut pose = stood;
            Balance { offset, ..Default::default() }.apply(&mut pose, &rig);
            for (before, after) in soles(&stood, &rig).iter().zip(soles(&pose, &rig)) {
                for (a, b) in before.iter().zip(after) {
                    assert!((b - *a).length() < 1.0e-3, "at {offset} a sole contact moved {:.2} mm", (b - *a).length() * 1e3);
                }
            }
        }
    }

    #[test]
    fn a_stepping_foot_lands_where_the_step_puts_it_and_the_other_stays() {
        // Mid-step on the real rig: the stance foot exactly where it stood,
        // the stepping foot at its displacement, lifted by the arc.
        let (stood, rig) = real_stood();
        let swing = Swing { leg: 1, from: Vec2::ZERO, to: Vec2::new(0.25, 0.0), elapsed: 0.15, duration: 0.3, joining: false };
        let balance = Balance { offset: Vec2::new(0.05, 0.0), swing: Some(swing), feet: [Vec2::ZERO, swing.at()], ..Default::default() };
        let mut pose = stood;
        balance.apply(&mut pose, &rig);
        let (before, after) = (soles(&stood, &rig), soles(&pose, &rig));
        for (a, b) in before[0].iter().zip(after[0]) {
            assert!((b - *a).length() < 1.0e-3, "the stance foot moved {:.2} mm", (b - *a).length() * 1e3);
        }
        let wanted = on(&rig, swing.at()) + Vec3::Y * swing.lift();
        for (a, b) in before[1].iter().zip(after[1]) {
            assert!(((b - *a) - wanted).length() < 5.0e-3, "the stepping foot is {:.1} mm off", ((b - *a) - wanted).length() * 1e3);
        }
    }

    /// The gallery's standing loop on the real rig, headless: push, step the
    /// balance, hand any step travelled to the "entity", pose. Returns per
    /// frame each foot's sole contacts in the world, and the pelvis.
    fn replay(push: Vec2, seconds: f32) -> Vec<([[Vec3; 3]; 2], Vec3, Option<Swing>)> {
        replay_at(push, seconds, &[DT])
    }

    /// [`replay`] with frame times cycling through `dts`.
    fn replay_at(push: Vec2, seconds: f32, dts: &[f32]) -> Vec<([[Vec3; 3]; 2], Vec3, Option<Swing>)> {
        let (stood, rig) = real_stood();
        let support = Support::of(&stood, &rig);
        let k = pendulum_k(&stood, &rig);
        let mut balance = Balance::default();
        balance.push(push);
        let mut entity = Vec3::ZERO;
        let mut frames = Vec::new();
        let (mut time, mut frame) = (0.0, 0);
        while time < seconds {
            let dt = dts[frame % dts.len()];
            (time, frame) = (time + dt, frame + 1);
            balance.step(&support, k, dt);
            if let Some(by) = balance.travelled {
                entity += on(&rig, by);
                balance.rebase();
            }
            let mut pose = stood;
            balance.apply(&mut pose, &rig);
            let world = soles(&pose, &rig).map(|foot| foot.map(|p| p + entity));
            let pelvis = entity + pose.root_translation;
            frames.push((world, pelvis, balance.swing));
        }
        frames
    }

    #[test]
    fn a_push_past_a_catchable_step_falls() {
        // Measured with the validity clamp lifted: forward 0.8 m/s and back
        // 1.0 m/s are caught by their step (nothing discarded), forward
        // 1.0 and back 1.2 run away. `MAX_CATCH` sits between the steps
        // they ask for (0.73 / 0.71 caught, 0.90 / 0.87 not).
        let (stood, rig) = real_stood();
        let support = Support::of(&stood, &rig);
        let k = pendulum_k(&stood, &rig);
        let falls = |push: Vec2| {
            let mut balance = Balance::default();
            balance.push(push);
            for _ in 0..(3.0 / DT) as usize {
                balance.step(&support, k, DT);
                if balance.travelled.is_some() {
                    balance.rebase();
                }
            }
            balance.falls
        };
        for caught in [Vec2::new(0.6, 0.0), Vec2::new(0.8, 0.0), Vec2::new(0.0, 0.7), Vec2::new(-0.8, 0.0), Vec2::new(-1.0, 0.0)] {
            assert!(!falls(caught), "{caught} should be caught by a step");
        }
        for falling in [Vec2::new(1.0, 0.0), Vec2::new(-1.2, 0.0), Vec2::new(0.0, 1.0), Vec2::new(2.0, 0.0)] {
            assert!(falls(falling), "{falling} should fall");
        }
    }

    #[test]
    fn a_stumble_plans_the_same_steps_at_uneven_frame_times() {
        // Live frames ran 4-52 ms. The two steps (out, then join) must not
        // depend on them.
        for push in [Vec2::new(0.6, 0.0), Vec2::new(0.0, 0.7), Vec2::new(-0.8, 0.0)] {
            let frames = replay_at(push, 7.0, &[DT, 0.3 * DT, 2.5 * DT, DT, 0.6 * DT, 3.0 * DT]);
            let mut swings: Vec<Swing> = frames.iter().filter_map(|f| f.2).collect();
            swings.dedup_by(|a, b| a.leg == b.leg && a.to == b.to && a.from == b.from);
            assert_eq!(swings.len(), 2, "{push}: {swings:?}");
            assert!(!swings[0].joining && swings[1].joining && swings[0].leg != swings[1].leg, "{push}: {swings:?}");
            assert!(frames.last().unwrap().2.is_none(), "{push}: still stepping at the end");
        }
    }

    #[test]
    fn a_stumble_steps_cleanly_on_the_real_rig() {
        // The gallery's loop, headless (`replay`): forward, sideways and
        // backward pushes the feet cannot absorb. Live, the first version
        // popped: a pelvis 60 mm ahead of itself at the hand-over, a rear
        // foot lifting 42 mm in the wide stance, a stepped foot left out of
        // reach in the air, a COM thrown 6 cm when the weight moved onto the
        // stepped foot. Then, held flat, the rear foot sank the pelvis 14 cm
        // (21 cm sideways) and it sprang up 132 mm when that foot lifted.
        // Backward needs the harder push: the real soles reach 0.18 m behind
        // the COM and 0.11 m ahead, so 0.6 m/s back is caught in place.
        for (push, deepest) in [(Vec2::new(0.6, 0.0), 0.06), (Vec2::new(0.0, 0.7), 0.13), (Vec2::new(-0.8, 0.0), 0.06)] {
            let frames = replay(push, 7.0);
            let floor = frames[0].0.iter().flatten().map(|p| p.y).fold(f32::MAX, f32::min);
            let mut stepped = false;
            // Each planted foot's tip where it was set down, and its heel's
            // highest since.
            let mut tips: [Option<Vec3>; 2] = frames[0].0.map(|foot| Some(foot[2]));
            let mut risen: f32 = 0.0;
            for (frame, pair) in frames.windows(2).enumerate() {
                let ((before, pelvis_before, _), (after, pelvis_after, swing)) = (&pair[0], &pair[1]);
                for leg in 0..2 {
                    let swinging = swing.is_some_and(|s| s.leg == leg) || pair[0].2.is_some_and(|s| s.leg == leg);
                    stepped |= swinging;
                    if swinging {
                        tips[leg] = None;
                    } else {
                        // Down, the foot may roll about its tip
                        // (`MAX_HEEL_RISE`), smoothly; the tip stays put.
                        let tip = *tips[leg].get_or_insert(after[leg][2]);
                        assert!(
                            (after[leg][2] - tip).length() < 1.0e-3,
                            "{push}: the planted {} tip moved {:.1} mm at {:.2} s",
                            ["left", "right"][leg],
                            (after[leg][2] - tip).length() * 1e3,
                            (frame + 1) as f32 * DT
                        );
                        for (a, b) in before[leg].iter().zip(after[leg]) {
                            assert!((b - *a).length() < 5.0e-3, "{push}: a planted foot jumped {:.1} mm", (b - *a).length() * 1e3);
                        }
                        risen = risen.max(after[leg][0].y - floor);
                    }
                    for b in after[leg] {
                        assert!(b.y > floor - 1.0e-3, "{push}: a foot went {:.1} mm into the floor", (floor - b.y) * 1e3);
                    }
                }
                assert!(
                    (*pelvis_after - *pelvis_before).length() < 0.02,
                    "{push}: the pelvis jumped {:?} mm at {:.2} s (swing {:?})",
                    ((*pelvis_after - *pelvis_before) * 1e3).round(),
                    (frame + 1) as f32 * DT,
                    swing
                );
            }
            assert!(stepped, "{push}: should need a step");
            // Measured: 46 mm forward, 44 back, 115 in a 0.4 m side lunge
            // (feet 0.63 m apart, where leg length alone asks ~10 cm).
            // Without the heel rise, forward sank 69 mm.
            let sank = frames[0].1.y - frames.iter().map(|f| f.1.y).fold(f32::MAX, f32::min);
            assert!(sank < deepest, "{push}: the pelvis sank {:.0} mm", sank * 1e3);
            if push.x > 0.0 {
                // A lunge's rear foot rolls onto its toes (9 mm) rather
                // than squatting the body over it.
                assert!(risen > 0.005, "{push}: the rear heel rose only {:.1} mm", risen * 1e3);
            }
            // At rest, feet side by side as they stood.
            let (end, _, swing) = frames.last().unwrap();
            assert!(swing.is_none(), "{push}: still stepping at the end");
            let (left, right) = (end[0][1], end[1][1]);
            let (stood_left, stood_right) = (frames[0].0[0][1], frames[0].0[1][1]);
            assert!(((left - right) - (stood_left - stood_right)).length() < 2.0e-3, "{push}: the feet did not end side by side");
        }
    }

    #[test]
    fn the_real_feet_give_a_sensible_support() {
        // Heel behind and toes ahead of the COM, the feet either side,
        // each about Winter's breadth wide.
        let (stood, rig) = real_stood();
        let support = Support::of(&stood, &rig);
        let both = support.under([Some(Vec2::ZERO); 2]);
        assert!(both.min.x < -0.03 && both.max.x > 0.08, "front to back {} .. {}", both.min.x, both.max.x);
        assert!(both.min.y < -0.1 && both.max.y > 0.1, "side to side {} .. {}", both.min.y, both.max.y);
        let width = support.feet[0].max.y - support.feet[0].min.y;
        assert!((0.05..0.12).contains(&width), "a foot is {width} m wide");
    }
}
