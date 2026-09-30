//! Standing balance after a push: the body as Winter's inverted pendulum.
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
//! The COP cannot leave the feet ([`Support`]). A push the feet cannot
//! absorb saturates it, and the body can only be caught by stepping; this
//! module flags that ([`Balance::needs_step`]) rather than stepping. The flag
//! uses the capture point `x + ẋ·√K`, where the COP would have to stand to
//! stop the body: a later formalisation of the same pendulum, not Winter's.
//!
//! Axes are the rig's own, horizontal: `x` along [`RigGeometry::forward`],
//! `y` along [`RigGeometry::left`]. Offsets are from the COM's rest place
//! over the feet.

use bevy::math::{Vec2, Vec3};

use super::foot::Sole;
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

/// Gravity, m/s².
const GRAVITY: f32 = 9.81;

/// Integration substeps per [`Balance::step`]: the pendulum's own unstable
/// pole is ~3 rad/s, well inside one 60 Hz frame, but a saturated COP
/// changes the dynamics abruptly and substeps keep that edge clean.
const SUBSTEPS: usize = 4;

/// Where the COP can go: the feet, as a box in the balance axes, relative
/// to the COM's rest place, less [`SUPPORT_MARGIN`].
///
/// A box rather than the feet's hull: with the feet side by side the two
/// agree front to back, and side to side the box spans sole centreline to
/// sole centreline, inside the true outer edges. Conservative.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Support {
    /// The most backward (`x`) and rightward (`y`) the COP can stand.
    pub min: Vec2,
    /// The most forward and leftward.
    pub max: Vec2,
}

impl Support {
    /// The support under `pose`, a standing pose, on `rig`: every `Sole`
    /// contact of both feet, relative to the pose's centre of mass, and no
    /// further than [`MAX_SWAY_ANGLE`] of sway in either plane. The feet
    /// can reach further (`puppet_base`'s toes allow a 10° forward lean),
    /// but Winter's pendulum is only valid within 8°.
    pub fn of(pose: &LocalPose, rig: &RigGeometry) -> Self {
        let com = super::anthropometry::centre_of_mass(pose, rig);
        let axes = |p: Vec3| Vec2::new((p - com).dot(rig.forward()), (p - com).dot(rig.left()));
        let mut min = Vec2::splat(f32::MAX);
        let mut max = Vec2::splat(f32::MIN);
        for ankle in [Bone::LeftFoot, Bone::RightFoot] {
            for point in Sole::of(rig, ankle).points(pose, rig) {
                let at = axes(point);
                min = min.min(at);
                max = max.max(at);
            }
        }
        let valid = Vec2::splat(MAX_SWAY_ANGLE.tan() * pendulum_k(pose, rig) * GRAVITY);
        Self {
            min: (min + Vec2::splat(SUPPORT_MARGIN)).max(-valid),
            max: (max - Vec2::splat(SUPPORT_MARGIN)).min(valid),
        }
    }

    /// Whether `point` is inside.
    pub fn contains(&self, point: Vec2) -> bool {
        point.cmpge(self.min).all() && point.cmple(self.max).all()
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

/// A standing body's balance: where its centre of mass is, how it moves,
/// and where the pressure under its feet is holding it.
#[derive(bevy::ecs::component::Component, Debug, Clone, Copy, PartialEq, Default)]
pub struct Balance {
    /// The COM's offset from its rest place over the feet, metres.
    pub offset: Vec2,
    /// Its velocity, m/s.
    pub velocity: Vec2,
    /// Where the COP stood on the last step, relative to the COM's rest
    /// place: the law's wish, clamped to the feet.
    pub pressure: Vec2,
    /// Whether the last step's push is beyond what the feet can absorb:
    /// the capture point `offset + velocity·√K` has left the support.
    pub needs_step: bool,
    /// Velocity still to be delivered by pushes under way, m/s.
    shove: Vec2,
    /// The rate it is delivered at, m/s².
    shove_rate: Vec2,
}

/// How long a push takes to deliver, seconds: a shove, not a collision.
///
/// Delivered in one frame, a 0.8 m/s push moved the pelvis 13 mm a frame
/// while the legs' 0.015 s springs lagged it, and the rendered toes moved
/// fast enough for the foot locks to let go and re-plant 23 mm away.
pub const PUSH_SECONDS: f32 = 0.1;

impl Balance {
    /// A push: a change in the COM's velocity, m/s (an impulse over the
    /// body's mass), delivered over [`PUSH_SECONDS`].
    pub fn push(&mut self, velocity: Vec2) {
        self.shove += velocity;
        self.shove_rate = self.shove / PUSH_SECONDS;
    }

    /// Whether the body is at rest over its feet, within `tolerance`
    /// metres and metres per second, with no push under way.
    pub fn is_settled(&self, tolerance: f32) -> bool {
        self.offset.length() < tolerance && self.velocity.length() < tolerance && self.shove == Vec2::ZERO
    }

    /// Advances the pendulum by `dt` seconds on `support`, with constant
    /// `k` (s², [`pendulum_k`]).
    ///
    /// # A push the feet cannot absorb
    ///
    /// When the capture point leaves the support, standing still cannot
    /// catch the body: a person steps, and the step puts the pressure where
    /// the feet were not. There is no stepping yet, so this stands in for
    /// it: flagged ([`Balance::needs_step`]), the COP is no longer held to
    /// the feet and the body comes back on the same law. Holding it at the
    /// support's edge instead was measured: the edge is an equilibrium the
    /// law cannot leave (the COP clamped exactly under the COM), so the body
    /// hung there, and the clamp's jump moved the planted feet 32 mm in a
    /// frame. Past [`MAX_SWAY_ANGLE`], outside the model, it is held.
    pub fn step(&mut self, support: &Support, k: f32, dt: f32) {
        if dt <= 0.0 || !dt.is_finite() || k <= 0.0 {
            return;
        }
        let stiffness = k * RECOVERY_FREQUENCY * RECOVERY_FREQUENCY;
        let damping = 2.0 * RECOVERY_DAMPING * RECOVERY_FREQUENCY * k;
        let valid = MAX_SWAY_ANGLE.tan() * k * GRAVITY;
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
            self.needs_step = !support.contains(self.offset + self.velocity * k.sqrt());
            let wanted = self.offset + self.offset * stiffness + self.velocity * damping;
            self.pressure = if self.needs_step { wanted } else { wanted.clamp(support.min, support.max) };
            let acceleration = (self.offset - self.pressure) / k;
            self.velocity += acceleration * h;
            self.offset += self.velocity * h;
            for axis in 0..2 {
                if self.offset[axis].abs() > valid {
                    self.offset[axis] = self.offset[axis].clamp(-valid, valid);
                    self.velocity[axis] = 0.0;
                }
            }
        }
        self.needs_step = !support.contains(self.offset + self.velocity * k.sqrt());
    }

    /// The COM's offset in the rig's frame, metres, horizontal.
    pub fn offset_on(&self, rig: &RigGeometry) -> Vec3 {
        rig.forward() * self.offset.x + rig.left() * self.offset.y
    }

    /// Poses the offset on `pose`, a standing one: the body swaying over
    /// its planted feet, which stay exactly where they are.
    ///
    /// Front to back, the pelvis carries the trunk over the ankles (Winter:
    /// the ankle strategy). Side to side, the hips load one leg and unload
    /// the other (the load/unload mechanism): the pelvis moves toward the
    /// loaded foot and drops on the unloaded side, like
    /// [`super::stance::shift_weight`], scaled continuously. The pelvis
    /// moves [`COM_PER_PELVIS`] further than the COM is to move.
    pub fn apply(&self, pose: &mut LocalPose, rig: &RigGeometry) {
        use super::stance::{move_pelvis_over_feet, WEIGHT_SHIFT, WEIGHT_SHIFT_ROLL};
        if self.offset.length_squared() < 1.0e-12 {
            return;
        }
        // +1 fully onto the left leg, as `shift_weight` counts it.
        let onto = (self.offset.y / WEIGHT_SHIFT).clamp(-1.0, 1.0);
        let roll = bevy::math::Quat::from_axis_angle(rig.forward(), onto * WEIGHT_SHIFT_ROLL);
        let loads = [0.5 + 0.5 * onto, 0.5 - 0.5 * onto];
        let pelvis = self.offset / COM_PER_PELVIS;
        let shift = rig.forward() * pelvis.x + rig.left() * pelvis.y;
        move_pelvis_over_feet(pose, rig, shift, roll, loads);
    }
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

    /// `puppet_base`-sized feet side by side, 23 cm apart, COM over their
    /// middle a little behind the balls.
    fn feet() -> Support {
        Support { min: Vec2::new(-0.06, -0.10), max: Vec2::new(0.12, 0.10) }
    }

    #[test]
    fn a_push_sways_and_returns() {
        // A 0.2 m/s shove forward: absorbed by the feet, back at rest
        // within 2 s, never past the support.
        let mut balance = Balance::default();
        balance.push(Vec2::new(0.2, 0.0));
        let mut furthest: f32 = 0.0;
        let mut settled_at = None;
        for frame in 0..240 {
            balance.step(&feet(), K, DT);
            furthest = furthest.max(balance.offset.x);
            assert!(!balance.needs_step, "a gentle push should not need a step (frame {frame})");
            assert!(balance.offset.x > -1.0e-3, "critically damped: no swing back past rest, got {}", balance.offset.x);
            if settled_at.is_none() && balance.is_settled(0.002) {
                settled_at = Some(frame);
            }
        }
        assert!(furthest > 0.02, "the push should sway the body, went {furthest} m");
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
            // Per frame, not per substep: the sign is what is checked.
            let error = balance.pressure - balance.offset;
            assert!(error.x * acceleration.x <= 0.0, "COP − COM {error} and COM̈ {acceleration} should oppose");
        }
    }

    #[test]
    fn a_push_beyond_the_support_flags_a_step_and_still_comes_back() {
        // A 0.8 m/s shove cannot be absorbed standing: flagged. The COP
        // leaves the feet only while it is (the step's stand-in), and the
        // body comes back smoothly rather than hanging at the edge.
        let mut balance = Balance::default();
        balance.push(Vec2::new(0.8, 0.0));
        let (mut flagged, mut previous) = (false, balance.offset);
        for _ in 0..300 {
            balance.step(&feet(), K, DT);
            flagged |= balance.needs_step;
            if !balance.needs_step {
                assert!(balance.pressure.x <= feet().max.x + 1.0e-6, "the COP left the feet unflagged: {}", balance.pressure.x);
            }
            assert!((balance.offset - previous).length() < 0.8 * DT * 1.01, "the body jumped in a frame");
            previous = balance.offset;
        }
        assert!(flagged, "a 0.8 m/s shove cannot be absorbed standing");
        assert!(balance.is_settled(0.002), "it should come back, not hang: at {} moving {}", balance.offset, balance.velocity);
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

    #[test]
    fn a_swaying_body_keeps_its_feet_planted() {
        // Every sole contact exactly where it stood, at the full support's
        // reach, in the target pose.
        let (stood, rig) = real_stood();
        let support = Support::of(&stood, &rig);
        let soles = |pose: &LocalPose| {
            [Bone::LeftFoot, Bone::RightFoot].map(|ankle| {
                let hips = pose.root_translation + offset_from(pose, &rig, Bone::Hips, Bone::Hips);
                Sole::of(&rig, ankle).points(pose, &rig).map(|p| p + hips)
            })
        };
        for offset in [support.min, support.max, Vec2::new(support.max.x, support.min.y), Vec2::new(support.min.x, support.max.y)] {
            let mut pose = stood;
            Balance { offset, ..Default::default() }.apply(&mut pose, &rig);
            for (before, after) in soles(&stood).iter().zip(soles(&pose)) {
                for (a, b) in before.iter().zip(after) {
                    assert!((b - *a).length() < 1.0e-3, "at {offset} a sole contact moved {:.2} mm", (b - *a).length() * 1e3);
                }
            }
            // Winter's pendulum holds under 8° of sway, in each plane (his
            // A/P and M/L analyses are separate).
            let height = pendulum_k(&stood, &rig) * GRAVITY;
            for (plane, along) in [("front to back", offset.x), ("side to side", offset.y)] {
                let angle = along.abs().atan2(height);
                assert!(angle <= MAX_SWAY_ANGLE + 1.0e-4, "the support lets the body sway {:.1}° {plane}", angle.to_degrees());
            }
        }
    }

    #[test]
    fn the_real_feet_give_a_sensible_support() {
        // Heel behind and toes ahead of the COM, the feet either side.
        let (stood, rig) = real_stood();
        let support = Support::of(&stood, &rig);
        assert!(support.min.x < -0.03 && support.max.x > 0.08, "front to back {} .. {}", support.min.x, support.max.x);
        assert!(support.min.y < -0.08 && support.max.y > 0.08, "side to side {} .. {}", support.min.y, support.max.y);
    }

    #[test]
    fn a_sideways_push_returns_as_well() {
        let mut balance = Balance::default();
        balance.push(Vec2::new(0.0, -0.15));
        for _ in 0..240 {
            balance.step(&feet(), K, DT);
        }
        assert!(balance.is_settled(0.002), "still at {} moving {}", balance.offset, balance.velocity);
    }
}
