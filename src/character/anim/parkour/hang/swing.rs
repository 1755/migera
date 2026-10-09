//! Swinging on a bar: step 9 of the parkour design. Hanging free (from a
//! bar, `Ledge::bar`, or a ledge with no wall below), the hanger pumps the
//! swing up to a steady one ([`Hanging::pump`]) and lets go swinging
//! forward to fly on ([`Hanging::swing_release`]): landing, or catching a
//! ledge or bar ahead.
//!
//! Gymnasts' hips and shoulders flex just after the bottom of a swing and
//! extend just before its top (Yeadon and Hiley 2000): the legs pike ahead
//! through the bottom swinging forward and arch back swinging back. The
//! pump is modelled as a drive on the pendulum's energy toward a swing
//! [`AMPLITUDE`] out, started from a still hang by a push at the swing's own
//! rate; the legs' shape goes with it.

use bevy::math::Vec3;

use super::{leap::Launch, Hanging, Ledge};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::jump::GRAVITY;
use crate::character::anim::rig::RigGeometry;

/// The swing pumped up to, radians out from straight below: a strong
/// swing, as a lache wants. At 0.8 no bar level with the one held was
/// within a 1.5 m/s change of the flight from 1.2 m on; at 1.2, bars level
/// with it 1.2-2 m ahead are within 1 m/s, and lower ones further.
pub const AMPLITUDE: f32 = 1.2;
/// How hard the pump drives the swing's energy, per second; and from a
/// still hang, how hard it pushes at the swing's own rate, of gravity's
/// pull, fading out once the swing has this share of its energy.
const PUMP_RATE: f32 = 1.2;
const START: f32 = 0.15;
const STARTED: f32 = 0.2;
/// The legs pike ahead at most this far through the bottom swinging
/// forward, radians, and arch back this far swinging back.
const PIKE: f32 = 0.6;
const ARCH: f32 = 0.3;
/// Into and out of the pumping shape over this long, seconds.
const PUMP_EASE: f32 = 0.4;
/// Let go swinging forward, out between these shares of the swing ahead of
/// straight below (past the bottom, before the top).
const RELEASE: (f32, f32) = (0.3, 0.85);
/// The hands let go over this long, seconds (a bar release's 73-157 ms).
const LETTING_GO: f32 = 0.1;
/// At a ledge ahead, the release velocity may differ from the swing's by
/// this much at most, m/s, to catch it (the arms' pull or push letting
/// go); and it waits for a later moment in the window that needs this much
/// less.
const MOST_CHANGE: f32 = 1.5;
const BETTER_LATER: f32 = 0.05;
/// Where the shoulders come down past the lip to catch it, metres (out
/// from its face, under it): all well within the catch's reach of 0.9 of
/// the arm (`fall`); flying 0.05-1.2 s to get there.
const CATCH_AT: [(f32, f32); 5] = [(0.15, 0.15), (0.25, 0.25), (0.1, 0.3), (0.3, 0.1), (0.2, 0.35)];
const CATCH_STEP: f32 = 0.05;
const CATCH_STEPS: usize = 24;
/// The grip this far in from the ledge's end, metres.
const END_MARGIN: f32 = 0.35;
/// A lache's mark: its face this far ahead of the grip, metres, no more
/// than this much higher, and its nearest grip no further aside.
const LACHE_AHEAD: (f32, f32) = (0.8, 3.0);
const LACHE_RISE: f32 = 0.5;
const LACHE_ASIDE: f32 = 0.6;
/// With nothing ahead, a lache lets go to land once the swing is pumped
/// up to this share of [`AMPLITUDE`].
const PUMPED: f32 = 0.9;

/// The pump's angular acceleration on a swing at `theta`, `dtheta` (rad,
/// rad/s), the pendulum's natural rate `omega`, pumped for `since`
/// seconds: driving its energy to a swing [`AMPLITUDE`] out.
pub(super) fn pumped(theta: f32, dtheta: f32, omega: f32, since: f32) -> f32 {
    let energy = 0.5 * dtheta * dtheta + omega * omega * (1.0 - theta.cos());
    let wanted = omega * omega * (1.0 - AMPLITUDE.cos());
    let short = 1.0 - energy / wanted;
    // From still, pushed at the swing's own rate (at rest, the drive on its
    // speed has nothing to grow from), faded out as it gets going.
    let start = (1.0 - energy / (STARTED * wanted)).clamp(0.0, 1.0);
    PUMP_RATE * short * dtheta + START * GRAVITY * start * (omega * since).sin()
}

impl Hanging {
    /// Swings on (pumping the swing up, `on`) or lets it die down: hanging
    /// free only; braced, nothing.
    pub fn pump(&mut self, on: bool) {
        if on && !self.pumping && !self.braced {
            self.pumped_for = 0.0;
        }
        self.pumping = on && !self.braced;
    }

    /// Whether it is pumping a swing ([`Self::pump`]).
    pub fn is_pumping(&self) -> bool {
        self.pumping
    }

    /// How far out the swing goes now, radians: from its energy.
    pub fn swing_amplitude(&self) -> f32 {
        let s = &self.swing;
        let omega = self.swing_rate();
        let energy = 0.5 * s.dtheta * s.dtheta + omega * omega * (1.0 - s.theta.cos());
        (1.0 - energy / (omega * omega)).clamp(-1.0, 1.0).acos()
    }

    /// The free swing's natural rate, rad/s.
    fn swing_rate(&self) -> f32 {
        let d = self.swing.r.max(0.1);
        (GRAVITY * d / (d * d + super::GYRATION * super::GYRATION)).sqrt()
    }

    /// Hanging free from a bar, the hands' turn round it with the swing:
    /// by the line from the grip to the hips' angle from straight below.
    /// Hooked fixed as on a lip, at a 0.75 rad swing forward the wrists
    /// stayed out behind the bar and the arms stretched 7% past their
    /// length. About the lip point the body swings about, not the bar's
    /// middle 2 cm off it: about the middle, the arms still stretched 1%.
    pub(super) fn bar_roll(&self) -> Option<bevy::math::Quat> {
        if !self.ledge.is_bar() || self.braced {
            return None;
        }
        let (out, _) = self.face();
        let to = self.hang_hips() - self.grip;
        let angle = to.dot(out).atan2(-to.y);
        Some(bevy::math::Quat::from_axis_angle(out.cross(Vec3::Y).normalize_or(Vec3::X), angle))
    }

    /// The pump's ease into and out of its shape on `dt`, and its clock.
    pub(super) fn ease_pump(&mut self, dt: f32) {
        let wanted = if self.pumping { 1.0 } else { 0.0 };
        let step = dt / PUMP_EASE;
        self.pump = if self.pump < wanted { (self.pump + step).min(wanted) } else { (self.pump - step).max(wanted) };
        if self.pumping {
            self.pumped_for += dt;
        }
    }

    /// The legs' line `down` (from the hips along the body, the world)
    /// swung ahead or back with the swing: piked ahead through the bottom
    /// swinging forward, arched back swinging back.
    pub(super) fn swung_legs(&self, down: Vec3) -> Vec3 {
        let shape = smoothstep(self.pump.clamp(0.0, 1.0));
        if shape <= 0.0 {
            return down;
        }
        // Forward is toward the face (`-out`); swinging forward, the angle
        // out from straight below shrinks.
        let ahead = -self.swing.dtheta / (self.swing_rate() * AMPLITUDE);
        let angle = shape * if ahead > 0.0 { PIKE * ahead.min(1.0) } else { ARCH * ahead.max(-1.0) };
        let forward = -self.face().0;
        let forward = (forward - down * forward.dot(down)).normalize_or(forward);
        down * angle.cos() + forward * angle.sin()
    }

    /// Lets go of a pumped swing, swinging forward within its release window
    /// ([`RELEASE`]), flying on at the swing's velocity; at `target` (a
    /// ledge or bar ahead), only at the moment in the window a change of
    /// the least, and no more than [`MOST_CHANGE`], catches it (a lache).
    /// `false`, and nothing done, outside the window or before that moment,
    /// with `target` out of reach, or not swinging free.
    pub fn swing_release(&mut self, target: Option<Ledge>, rig: &RigGeometry) -> bool {
        if !self.is_hanging() || self.braced || self.up.is_some() || self.step.is_some() || self.launch.is_some() {
            return false;
        }
        let amplitude = self.swing_amplitude();
        let (theta, dtheta) = (self.swing.theta, self.swing.dtheta);
        // Forward is the angle shrinking, out ahead of straight below below
        // zero.
        let last = -RELEASE.1 * amplitude;
        if dtheta >= 0.0 || theta > -RELEASE.0 * amplitude || theta < last {
            return false;
        }
        let velocity = match target {
            Some(target) => {
                // Later in the window, at the swing's energy, a better moment?
                let omega = self.swing_rate();
                let energy = 0.5 * dtheta * dtheta + omega * omega * (1.0 - theta.cos());
                let change = |theta: f32, dtheta: f32| {
                    let (from, velocity) = self.swung_at(theta, dtheta);
                    self.catch_change(target, from, velocity, rig).0
                };
                let now = change(theta, dtheta);
                let later = (1..=8)
                    .map(|i| {
                        let at = theta + (last - theta) * i as f32 / 8.0;
                        change(at, -(2.0 * (energy - omega * omega * (1.0 - at.cos()))).max(0.0).sqrt())
                    })
                    .fold(f32::INFINITY, f32::min);
                if now > MOST_CHANGE || later + BETTER_LATER < now {
                    return false;
                }
                let from = self.hang_hips();
                Some(self.catch_change(target, from, self.hang_velocity(), rig).1)
            }
            None => None,
        };
        let from = self.hang_hips();
        let from_velocity = self.hang_velocity();
        let to = from + from_velocity * LETTING_GO - Vec3::Y * (0.5 * GRAVITY * LETTING_GO * LETTING_GO);
        let swung = from_velocity - Vec3::Y * (GRAVITY * LETTING_GO);
        self.launch = Some(Launch::released(from, from_velocity, to, velocity.unwrap_or(swung), target, LETTING_GO));
        self.pumping = false;
        true
    }

    /// Pumps the swing and lets go of it forward (a lache): at the nearest
    /// bar or ledge ahead among the others (facing the same way, its face
    /// [`LACHE_AHEAD`] in front of the grip, no more than [`LACHE_RISE`]
    /// higher), when the swing comes round to the moment that catches it
    /// ([`Self::swing_release`]); with none ahead, to land. `true` once let
    /// go; braced, nothing.
    pub fn lache(&mut self, rig: &RigGeometry) -> bool {
        if self.braced {
            return false;
        }
        self.pump(true);
        let (out, along) = self.face();
        let grip = self.grip;
        let target = self
            .others
            .iter()
            .copied()
            .filter(|ledge| ledge.out.dot(out) > 0.9 && ledge.height() - grip.y <= LACHE_RISE)
            .filter(|ledge| (LACHE_AHEAD.0..=LACHE_AHEAD.1).contains(&ledge.out_of(grip)) && (ledge.nearest(grip, END_MARGIN) - grip).dot(along).abs() < LACHE_ASIDE)
            .min_by(|a, b| a.out_of(grip).total_cmp(&b.out_of(grip)));
        // To land, from a swing pumped up, not the first small one.
        if target.is_none() && self.swing_amplitude() < PUMPED * AMPLITUDE {
            return false;
        }
        self.swing_release(target, rig)
    }

    /// The hips and their velocity swung to `theta`, `dtheta`.
    fn swung_at(&self, theta: f32, dtheta: f32) -> (Vec3, Vec3) {
        let (out, along) = self.face();
        let r = self.swing.r;
        let hips = self.grip + along * self.swing.along + out * (r * theta.sin()) - Vec3::Y * (r * theta.cos());
        (hips, (out * theta.cos() + Vec3::Y * theta.sin()) * (r * dtheta))
    }

    /// Letting go with the hips at `from` moving at `velocity`, the least
    /// change of the velocity they fly on at (from [`LETTING_GO`] on, m/s)
    /// that catches `target` falling past it, and that velocity: the
    /// shoulders coming down to one of [`CATCH_AT`] off its lip.
    fn catch_change(&self, target: Ledge, from: Vec3, velocity: Vec3, rig: &RigGeometry) -> (f32, Vec3) {
        let to = from + velocity * LETTING_GO - Vec3::Y * (0.5 * GRAVITY * LETTING_GO * LETTING_GO);
        let swung = velocity - Vec3::Y * (GRAVITY * LETTING_GO);
        let shoulders = self.shoulders_over_hips(rig);
        let lip = target.nearest(to, END_MARGIN);
        let mut best = (f32::INFINITY, swung);
        for (out, below) in CATCH_AT {
            let hips = lip + target.out * out - Vec3::Y * below - shoulders;
            for step in 1..=CATCH_STEPS {
                let seconds = step as f32 * CATCH_STEP;
                let up = (hips.y - to.y) / seconds + 0.5 * GRAVITY * seconds;
                // Caught only falling.
                if up >= GRAVITY * seconds {
                    continue;
                }
                let wanted = (hips - to).with_y(0.0) / seconds + Vec3::Y * up;
                let change = (wanted - swung).length();
                if change < best.0 {
                    best = (change, wanted);
                }
            }
        }
        best
    }
}
