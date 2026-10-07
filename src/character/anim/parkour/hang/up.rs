//! Climbing up from a hang onto the ledge's top: step 2 of the parkour
//! design.
//!
//! The hips are planned once as a path through the climb's shapes, each
//! measured from where the hands are, and the body is posed onto it each
//! frame, as the hang is:
//!
//! - **Pull**: the elbows bend, the chest comes up to the lip, the hands
//!   still hooked over it. Braced, each foot steps up the wall.
//! - **Turn over**: one hand, then the other, comes up over the lip and
//!   presses flat on the top; the shoulders come over the lip.
//! - **Press**: the arms straighten, the shoulders over the hands, the hips
//!   up past the lip, the trunk leant far over the top. The feet leave the
//!   wall.
//! - **Step on**: the lead foot comes up over the edge onto the top.
//! - **Stand up**: the other foot comes on beside it, the hands let go, and
//!   it stands, the foot IK's drop eased in, on the top where the walker
//!   stands on.
//!
//! There are no phase timings for a ledge climb-up or a muscle-up
//! (`parkour-movement-data`); the pull-up's elbow range is the proxy, and
//! the timings are set by eye.

use bevy::math::{Quat, Vec3};

use super::{Hanging, Phase, ARMS, LEGS, SIGN};
use crate::character::anim::armik::frame_turn;
use crate::character::anim::gait::smoothstep;
use crate::character::anim::hand::GRIP_RADIUS;
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// How long each part of the climb takes, seconds: pull, turn over,
/// press, step on, stand up. No data; set by eye.
const PARTS: [f32; 5] = [0.9, 0.4, 0.6, 0.5, 1.4];
/// End of the pull: the shoulders this far out from the face and below
/// the lip, metres; the trunk leant toward the wall, radians.
const PULLED: (f32, f32, f32) = (0.2, 0.1, 0.25);
/// End of the turn over: the shoulders this far out from the face and
/// above the lip, metres, the trunk leant over it, radians.
const TURNED: (f32, f32, f32) = (0.05, 0.15, 0.55);
/// End of the press: the shoulders above the hands by this share of the
/// arm, the trunk folded near flat over the top, radians: the hips high
/// enough for a knee to swing up round the edge. A knee swings round its
/// socket on the thigh's length, and leant 0.9 the sockets were too low
/// (0.23 m above the lip): coming up, the knee went into the corner.
const PRESSED: (f32, f32) = (0.88, 1.4);
/// The lead foot on: the hips this much further in over the top and up,
/// metres, the trunk leant, radians. In 0.1 and up 0.05, the hands were
/// out of reach as it came on.
const STEPPED: (f32, f32, f32) = (0.03, 0.0, 1.4);
/// Standing up, the hands let go over this share of it, once the hips
/// start in over the feet. Letting go only as it stood, the hands were
/// pulled 35 cm off the top first.
const LET_GO: (f32, f32) = (0.3, 0.6);
/// Standing up, the trail foot comes on over this share of it, from the
/// start; the hips held out from the face, rising a little on the lead leg,
/// until this share of it. With its socket carried in over the edge, the
/// foot still below the lip, the knee bent into the edge (3 cm), or
/// turned up out of it, backward.
const TRAIL_ON: (f32, f32) = (0.0, 0.5);
const HIPS_HELD: (f32, f32, f32) = (0.3, 0.05, 0.0);
/// Each hand presses on the top this far back from the lip, metres (the
/// palm's middle); the palm's middle is this far back from the knuckles.
const PRESS_BACK: f32 = 0.12;
const PALM_MIDDLE: f32 = 0.03;
/// Each hand comes up over the lip this far above the higher of where it
/// leaves and lands, metres.
const HAND_LIFT: f32 = 0.08;
/// Standing at the end, the hips this far back from the edge, metres; and
/// the least top that leaves room to stand on, metres.
const STAND_BACK: f32 = 0.3;
const LEAST_DEPTH: f32 = 0.55;
/// Braced, each foot steps up the wall through the pull to where its leg
/// reaches at this share of its length; out from the wall this much
/// between, metres.
const PUSH_LEG: f32 = 0.88;
/// Braced, each knee is turned out this share of the way to straight out
/// to the side (about 55° at the hip), clear of the wall: not turned, a
/// knee went 6 cm into it; turned out all the way, or out and back, the
/// thigh turned 90-116° at the hip.
const KNEES_OUT: f32 = 0.6;
/// Braced, a foot out of reach smears up the face to where its leg reaches
/// at this share of its length.
const SMEAR_LEG: f32 = 0.97;
const STEP_OUT: f32 = 0.08;
/// Free of the wall, the legs reach this share of their length from the
/// sockets, the feet this far out from them, metres, the knees clear of
/// the wall.
const DANGLE: f32 = 0.97;
const DANGLE_OUT: f32 = 0.2;
/// A foot coming up onto the top: its ankle this far out from the face and
/// below the lip, metres, the shin down the face and the knee over the lip,
/// by this share of its step; then lifted this far above the lip by this
/// share, metres.
const FOOT_FACE: (f32, f32, f32) = (0.15, 0.28, 0.4);
const FOOT_LIFT: (f32, f32) = (0.25, 0.7);
/// The legs go from where the hang had them to the climb's over this long,
/// seconds.
const SETTLE_IN: f32 = 0.25;
/// Asked mid-swing, the climb starts once the hips move slower than this,
/// m/s: begun at 1.05 m/s toward the wall, the pull had to turn the swing
/// round at 6.7 m/s².
const START_SPEED: f32 = 0.25;

/// A path's knot: when, where, and how fast.
#[derive(Debug, Clone, Copy)]
struct Knot {
    t: f32,
    at: Vec3,
    velocity: Vec3,
}

/// A path through knots, cubic between them, its acceleration continuous
/// across each (a clamped spline): from the first at its given velocity to
/// the last at rest. With each inner knot at the chord's velocity across
/// it, the acceleration jumped at the knots, 0.74 g at the crouch.
#[derive(Debug, Clone)]
struct Track(Vec<Knot>);

impl Track {
    fn through(points: &[(f32, Vec3)], start: Vec3) -> Self {
        let n = points.len() - 1;
        let mut velocity = vec![Vec3::ZERO; n + 1];
        velocity[0] = start;
        // Each inner knot's velocity making the acceleration either side of
        // it the same: tridiagonal, solved forward then back.
        if n >= 2 {
            let h = |i: usize| points[i + 1].0 - points[i].0;
            let (mut diagonal, mut right) = (vec![0.0; n], vec![Vec3::ZERO; n]);
            for i in 1..n {
                let (a, b) = (h(i - 1), h(i));
                diagonal[i] = 2.0 * (1.0 / a + 1.0 / b);
                right[i] = 3.0 * ((points[i].1 - points[i - 1].1) / (a * a) + (points[i + 1].1 - points[i].1) / (b * b));
            }
            right[1] -= start / h(0);
            for i in 2..n {
                let factor = (1.0 / h(i - 1)) / diagonal[i - 1];
                diagonal[i] -= factor / h(i - 1);
                right[i] = right[i] - right[i - 1] * factor;
            }
            for i in (1..n).rev() {
                let after = if i + 1 < n { velocity[i + 1] / h(i) } else { Vec3::ZERO };
                velocity[i] = (right[i] - after) / diagonal[i];
            }
        }
        Self((0..=n).map(|i| Knot { t: points[i].0, at: points[i].1, velocity: velocity[i] }).collect())
    }

    fn at(&self, t: f32) -> Vec3 {
        let knots = &self.0;
        if t <= knots[0].t {
            return knots[0].at;
        }
        for pair in knots.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if t <= b.t {
                let h = b.t - a.t;
                let s = (t - a.t) / h;
                let (s2, s3) = (s * s, s * s * s);
                return a.at * (2.0 * s3 - 3.0 * s2 + 1.0) + a.velocity * (h * (s3 - 2.0 * s2 + s)) + b.at * (3.0 * s2 - 2.0 * s3) + b.velocity * (h * (s3 - s2));
            }
        }
        knots[knots.len() - 1].at
    }
}

/// A cubic Bézier from `a` to `d` by `b` and `c`, at `s`.
fn bezier(a: Vec3, b: Vec3, c: Vec3, d: Vec3, s: f32) -> Vec3 {
    let r = 1.0 - s;
    a * (r * r * r) + b * (3.0 * r * r * s) + c * (3.0 * r * s * s) + d * (s * s * s)
}

/// Eased 0-1 across `(from, to)`.
fn across(t: f32, (from, to): (f32, f32)) -> f32 {
    smoothstep(((t - from) / (to - from).max(1.0e-6)).clamp(0.0, 1.0))
}

/// A climb up from a hang, as planned.
#[derive(Debug, Clone)]
pub(super) struct ClimbUp {
    /// Seconds since it began.
    pub(super) t: f32,
    /// When each part ends, seconds from the start.
    ends: [f32; 5],
    /// The hips' path, and the trunk's lean (in `x`).
    hips: Track,
    lean: Track,
    /// Each ankle, and foot's world rotation, as it began.
    from_ankles: [Vec3; 2],
    from_attitudes: [Quat; 2],
    /// Braced: each ankle on the wall as it began, and stepped up to.
    holds: Option<[[Vec3; 2]; 2]>,
    /// Each wrist hooked, and pressing on the top, and its hand's world turn
    /// pressing.
    hooks: [Vec3; 2],
    presses: [Vec3; 2],
    press_turns: [Quat; 2],
    /// Each ankle standing on the top at the end, and the root there.
    spots: [Vec3; 2],
    end_root: Vec3,
    /// The side whose hand turns over and foot steps on first.
    lead: usize,
}

impl Hanging {
    /// Asks it to climb up onto the top: `false` (still hanging) if it is
    /// not hanging yet, or the top has no room to stand. Swinging, it starts
    /// once the swing has slowed (`START_SPEED`), at the end of a swing.
    pub fn climb_up(&mut self) -> bool {
        if self.phase != Phase::Hanging || self.ledge.depth < LEAST_DEPTH {
            return false;
        }
        self.up_asked = true;
        self.start_up();
        true
    }

    /// Starts the climb up, if asked and the swing is slow enough.
    pub(super) fn start_up(&mut self) {
        if self.up_asked && self.up.is_none() && self.step.is_none() && self.hang_velocity().length() < START_SPEED {
            let rig = self.rig.clone();
            self.up = Some(self.plan_up(&rig));
        }
    }

    /// Whether it is climbing up.
    pub fn is_climbing_up(&self) -> bool {
        self.up.is_some()
    }

    /// Whether it has climbed up and stands on the top.
    pub fn is_done(&self) -> bool {
        self.up.as_ref().is_some_and(|up| up.t >= up.ends[4])
    }

    /// The trunk's world turn leant `lean` toward the wall.
    fn trunk_at(&self, lean: f32) -> Quat {
        Quat::from_axis_angle(self.turn * self.rig.left(), lean) * self.turn
    }

    /// The hips' velocity hanging now, in the world.
    fn hang_velocity(&self) -> Vec3 {
        let s = &self.swing;
        let (sin, cos) = s.theta.sin_cos();
        let (out, along) = self.face();
        along * s.dalong + out * (s.dr * sin + s.r * cos * s.dtheta) - Vec3::Y * (s.dr * cos - s.r * sin * s.dtheta)
    }

    fn plan_up(&self, rig: &RigGeometry) -> ClimbUp {
        let out = self.ledge.out;
        let mut ends = PARTS;
        for k in 1..5 {
            ends[k] += ends[k - 1];
        }
        let arm = 0.5 * (self.body.arms[0] + self.body.arms[1]);
        // The hips under shoulders at `at`, the trunk leant `lean`: from the
        // leant pose's own shoulders. The whole trunk turned by the lean about
        // the hips put the press's shoulders 6 cm too high, its arms already
        // straight.
        let under = |at: Vec3, lean: f32| {
            let leant = forward_kinematics_on(&crate::character::anim::jump::upper(&self.body.stood, rig, lean, (0.0, 0.0)), rig);
            at - self.turn * (0.5 * (leant[ARMS[0].shoulder] + leant[ARMS[1].shoulder]) - leant[Bone::Hips])
        };
        let lip = self.ledge.nearest(self.grip, 0.0);

        // Each hand pressing flat on the top: the palm's middle on it behind
        // the lip, the fingers pointing in.
        let back = self.turn.inverse();
        let press_turns = [0, 1].map(|side| {
            let grip = &self.body.grips[side];
            self.turn * frame_turn(grip.along, grip.palm, back * -out, back * Vec3::NEG_Y)
        });
        let presses = [0, 1].map(|side| {
            let grip = &self.body.grips[side];
            let palm = grip.bar - grip.palm * GRIP_RADIUS - grip.along * PALM_MIDDLE;
            self.lips[side] - out * PRESS_BACK - press_turns[side] * palm
        });

        // Standing on the top at the end.
        let stood = forward_kinematics_on(&self.body.stood, rig);
        let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
        let end_root = lip - out * STAND_BACK - flat(self.turn * self.body.hips);
        let end_root = Vec3::new(end_root.x, self.ledge.height(), end_root.z);
        let spots = LEGS.map(|(_, _, ankle, _)| end_root + self.turn * stood[ankle]);

        let start = self.hang_hips();
        let (pulled_out, pulled_below, pulled_lean) = PULLED;
        let (turned_out, turned_above, turned_lean) = TURNED;
        let (press_reach, pressed_lean) = PRESSED;
        let (stepped_in, stepped_up, stepped_lean) = STEPPED;
        let press_middle = 0.5 * (presses[0] + presses[1]);
        let pulled = under(lip + out * pulled_out - Vec3::Y * pulled_below, pulled_lean);
        let turned = under(lip + out * turned_out + Vec3::Y * turned_above, turned_lean);
        let pressed = under(press_middle + Vec3::Y * (press_reach * arm), pressed_lean);
        let stepped = pressed - out * stepped_in + Vec3::Y * stepped_up;
        let end_hips = end_root + self.turn * self.body.hips - Vec3::Y * self.drop;
        let (held_share, held_in, held_up) = HIPS_HELD;
        let held_at = ends[3] + held_share * (ends[4] - ends[3]);
        let held = stepped - out * held_in + Vec3::Y * held_up;
        let hips = Track::through(
            &[(0.0, start), (ends[0], pulled), (ends[1], turned), (ends[2], pressed), (ends[3], stepped), (held_at, held), (ends[4], end_hips)],
            self.hang_velocity(),
        );
        let lean = Track::through(
            &[
                (0.0, Vec3::X * self.lean_at(start)),
                (ends[0], Vec3::X * pulled_lean),
                (ends[1], Vec3::X * turned_lean),
                (ends[2], Vec3::X * pressed_lean),
                (ends[3], Vec3::X * stepped_lean),
                (held_at, Vec3::X * stepped_lean),
                (ends[4], Vec3::ZERO),
            ],
            Vec3::ZERO,
        );

        // Where the legs are now, as the hang poses them.
        let pose = self.hanging_pose(rig);
        let at = forward_kinematics_on(&pose, rig);
        let world = accumulate_world_rotations(&pose, rig);
        let root = self.root();
        let from_ankles = LEGS.map(|(_, _, ankle, _)| root + self.turn * at[ankle]);
        let from_attitudes = LEGS.map(|(_, _, ankle, _)| self.turn * world[ankle]);

        // Braced, each foot steps up to where its leg pushes from at the end
        // of the pull.
        let holds = self.braced.then(|| {
            let trunk = self.trunk_at(pulled_lean);
            [0, 1].map(|side| {
                let socket = pulled + trunk * self.body.sockets[side];
                [self.wall_ankle(side, self.wall_balls[side]), self.wall_ankle(side, self.wall_ball(side, socket, PUSH_LEG))]
            })
        });

        let hooks = self.wrists();
        ClimbUp { t: 0.0, ends, hips, lean, from_ankles, from_attitudes, holds, hooks, presses, press_turns, spots, end_root, lead: 0 }
    }

    /// The root, in the world, climbing up: under the hips as the standing
    /// pose has them, less the foot IK's drop as it stands.
    pub(super) fn up_root(&self, up: &ClimbUp) -> Vec3 {
        up.hips.at(up.t) - self.turn * self.body.hips + Vec3::Y * self.up_sink(up)
    }

    /// How far the hips are below standing as it stands up: the foot IK's
    /// drop, eased in.
    fn up_sink(&self, up: &ClimbUp) -> f32 {
        self.drop * across(up.t, (up.ends[3], up.ends[4]))
    }

    /// Each wrist and its hand's world turn, how far each has turned over
    /// onto the top (0-1), and how much the arms hold (letting go, standing
    /// up).
    fn hands_up(&self, up: &ClimbUp) -> ([Vec3; 2], [Quat; 2], [f32; 2], f32) {
        let (pulled, turned) = (up.ends[0], up.ends[1]);
        let span = turned - pulled;
        let mut wrists = up.hooks;
        let mut turns = [self.hook_turn(0), self.hook_turn(1)];
        let mut pressing = [0.0; 2];
        for side in 0..2 {
            let window = if side == up.lead { (pulled, pulled + 0.65 * span) } else { (pulled + 0.35 * span, turned) };
            let s = across(up.t, window);
            pressing[side] = s;
            if s <= 0.0 {
                continue;
            }
            let (from, to) = (up.hooks[side], up.presses[side]);
            // Up over the lip, out of the face.
            let over = |p: Vec3| {
                let p = Vec3::new(p.x, from.y.max(to.y) + HAND_LIFT, p.z);
                p + self.ledge.out * (0.06 - self.ledge.out_of(p)).max(0.0)
            };
            wrists[side] = bezier(from, over(from), over(to), to, s);
            turns[side] = turns[side].slerp(up.press_turns[side], s);
        }
        let (stepped, stood) = (up.ends[3], up.ends[4]);
        let holding = 1.0 - across(up.t, (stepped + LET_GO.0 * (stood - stepped), stepped + LET_GO.1 * (stood - stepped)));
        (wrists, turns, pressing, holding)
    }

    /// Foot `side`'s ankle and world rotation, and how far its knee turns
    /// out (0-1), before it comes onto the top: braced on the wall, then
    /// hanging below the hips.
    fn foot_below(&self, up: &ClimbUp, side: usize, t: f32) -> (Vec3, Quat, Vec3) {
        let hips = up.hips.at(t);
        let trunk = self.trunk_at(up.lean.at(t).x);
        let socket = hips + trunk * self.body.sockets[side];
        let down = ((DANGLE * self.body.legs[side]).powi(2) - DANGLE_OUT * DANGLE_OUT).max(0.0).sqrt();
        let hanging = (socket - Vec3::Y * down + self.ledge.out * DANGLE_OUT, self.turn * self.body.attitudes[side]);
        let (ankle, attitude, out) = match up.holds {
            Some(holds) => {
                // Up the wall through the pull, one foot then the other.
                let window = if side == up.lead { (0.05, 0.5) } else { (0.4, 0.85) };
                let s = across(t, (window.0 * up.ends[0], window.1 * up.ends[0]));
                let [from, to] = holds[side];
                let on_wall = from.lerp(to, s) + self.ledge.out * (STEP_OUT * (std::f32::consts::PI * s).sin());
                // Out of the leg's reach as the hips rise, it smears up the
                // face until it does reach: held where it was, the trail foot
                // was 12 cm short of its hold before it stepped.
                let reach = SMEAR_LEG * self.body.legs[side];
                let off = on_wall - socket;
                let flat = Vec3::new(off.x, 0.0, off.z).length();
                let on_wall = if off.length() > reach && flat < reach { Vec3::new(on_wall.x, socket.y - (reach * reach - flat * flat).sqrt(), on_wall.z) } else { on_wall };
                let leave = across(t, (up.ends[1], up.ends[2]));
                (on_wall.lerp(hanging.0, leave), self.toes_up(side).slerp(hanging.1, leave), 1.0 - leave)
            }
            None => (hanging.0, hanging.1, 0.0),
        };
        // From where the hang had it.
        let settle = across(t, (0.0, SETTLE_IN));
        let aside = self.turn * self.rig.left() * (SIGN[side] * KNEES_OUT);
        (up.from_ankles[side].lerp(ankle, settle), up.from_attitudes[side].slerp(attitude, settle), aside * (out * settle))
    }

    /// Foot `side`'s ankle and world rotation, and where its knee is turned
    /// (the world, as long as how far, 0-1): below, then up over the edge
    /// onto its spot on the top, the lead foot first.
    fn foot_up(&self, up: &ClimbUp, side: usize) -> (Vec3, Quat, Vec3) {
        let [_, _, pressed, stepped, stood] = up.ends;
        let window = if side == up.lead { (pressed, stepped) } else { (stepped + TRAIL_ON.0 * (stood - stepped), stepped + TRAIL_ON.1 * (stood - stepped)) };
        let standing = self.turn * self.body.attitudes[side];
        if up.t >= window.1 {
            return (up.spots[side], standing, Vec3::ZERO);
        }
        let (from, attitude, knee) = self.foot_below(up, side, up.t.min(window.0));
        if up.t <= window.0 {
            return (from, attitude, knee);
        }
        let s = ((up.t - window.0) / (window.1 - window.0)).clamp(0.0, 1.0);
        let to = up.spots[side];
        // Up the face until the shin hangs down it, the knee over the lip;
        // then straight up clear of it, and over onto its spot. Lifted
        // straight up from below, its toes, 14 cm ahead of the ankle, went
        // into the face 12 cm below the lip; brought up behind the hips, the
        // knee bent forward into the edge.
        let (face_out, face_below, face_at) = FOOT_FACE;
        let (lift, lifted_at) = FOOT_LIFT;
        let on_face = |out: f32, up: f32| {
            let p = Vec3::new(from.x, self.ledge.height() + up, from.z);
            p + self.ledge.out * (out - self.ledge.out_of(p))
        };
        let path = Track::through(&[(0.0, from), (face_at, on_face(face_out, -face_below)), (lifted_at, on_face(face_out, lift)), (1.0, to)], Vec3::ZERO);
        let attitude = if s < face_at { attitude.slerp(self.toes_up(side), across(s, (0.0, face_at))) } else { self.toes_up(side).slerp(standing, across(s, (face_at, 1.0))) };
        (path.at(s), attitude, knee * (1.0 - across(s, (0.0, face_at))))
    }

    /// The climb's pose: the trunk leant as planned on the hips' path, the
    /// legs to their feet, the arms to the lip or the top, letting go as it
    /// stands.
    pub(super) fn climbing_pose(&self, up: &ClimbUp, rig: &RigGeometry) -> LocalPose {
        let sink = self.up_sink(up);
        let root = self.up_root(up);
        let back = self.turn.inverse();
        let mut pose = crate::character::anim::jump::upper(&self.body.stood, rig, up.lean.at(up.t).x, (0.0, 0.0));
        // The hips joint is the root translation and the hips' own offset
        // from it: taken as the root translation alone, the feet went 0.95 m
        // above their marks.
        pose.root_translation.y -= sink;
        let hips = self.body.hips - Vec3::Y * sink;
        for (side, &leg) in LEGS.iter().enumerate() {
            let (ankle, attitude, knee) = self.foot_up(up, side);
            place_ankle(&mut pose, rig, leg.2, back * (ankle - root) - hips);
            // Against the wall the knee turned out to the side, not into it;
            // coming over the edge, up over it.
            if knee.length_squared() > 1.0e-8 {
                knee_toward(&mut pose, rig, leg, back * knee, knee.length().min(1.0));
            }
            let now = accumulate_world_rotations(&pose, rig)[leg.2];
            pose.rotations[leg.2] = delta_after_world_turn(&pose, rig, leg.2, (back * attitude) * now.inverse());
        }
        let (wrists, turns, pressing, holding) = self.hands_up(up);
        let free = pose;
        self.arms_to(&mut pose, rig, root, wrists, turns, pressing, 1.0);
        if holding < 1.0 {
            for arm in ARMS {
                for bone in [arm.shoulder, arm.elbow, arm.wrist] {
                    pose.rotations[bone] = free.rotations[bone].slerp(pose.rotations[bone], holding);
                }
            }
            for clavicle in super::CLAVICLES {
                pose.rotations[clavicle] = free.rotations[clavicle].slerp(pose.rotations[clavicle], holding);
            }
        }
        pose
    }

    /// How closed each hand is climbing up: hooked, opening as it comes up
    /// over the lip, open pressing.
    pub(super) fn up_grips(&self, up: &ClimbUp) -> [f32; 2] {
        let (pulled, turned) = (up.ends[0], up.ends[1]);
        let span = turned - pulled;
        [0, 1].map(|side| {
            let start = if side == up.lead { pulled } else { pulled + 0.35 * span };
            1.0 - across(up.t, (start, start + 0.15 * span))
        })
    }

    /// Where it looks climbing up: the lip, then ahead over the top.
    pub(super) fn up_look(&self, up: &ClimbUp) -> Vec3 {
        let ahead = up.end_root - self.ledge.out * 2.0 + Vec3::Y * 1.5;
        self.grip.lerp(ahead, across(up.t, (up.ends[0], up.ends[2])))
    }

    /// The ankle of foot `side` with its ball at `ball`, toes up on the wall.
    fn wall_ankle(&self, side: usize, ball: Vec3) -> Vec3 {
        ball + self.ankle_from_ball(side, self.toes_up(side))
    }
}

/// Turns `leg` (socket, knee, ankle, toe) about the line from its socket to
/// its ankle so its knee points as near `toward` (the pose's frame) as it
/// can, by `weight`; the foot turned back, keeping its attitude.
fn knee_toward(pose: &mut LocalPose, rig: &RigGeometry, (socket, knee, ankle, _): (Bone, Bone, Bone, Bone), toward: Vec3, weight: f32) {
    let at = forward_kinematics_on(pose, rig);
    let axis = (at[ankle] - at[socket]).normalize_or_zero();
    let square = |v: Vec3| (v - axis * v.dot(axis)).normalize_or_zero();
    let (now, wanted) = (square(at[knee] - at[socket]), square(toward));
    if axis == Vec3::ZERO || now == Vec3::ZERO || wanted == Vec3::ZERO {
        return;
    }
    let roll = Quat::from_axis_angle(axis, now.cross(wanted).dot(axis).atan2(now.dot(wanted)) * weight);
    pose.rotations[socket] = delta_after_world_turn(pose, rig, socket, roll);
    pose.rotations[ankle] = delta_after_world_turn(pose, rig, ankle, roll.inverse());
}

#[cfg(test)]
mod tests {
    use super::super::tests::{grabbing, real_stood, wall};
    use super::*;
    use crate::character::anim::rig::BoneSet;

    const DT: f32 = 1.0 / 60.0;

    /// What a climb up measured, frame by frame.
    #[derive(Debug, Default)]
    struct Measured {
        /// The most a hand strays from its hook, pulling, and from its press,
        /// pressing, metres.
        hooked_off: f32,
        pressed_off: f32,
        pressed_at: f32,
        /// Pressing, the most an elbow sits out to its side of the line from
        /// its shoulder to its wrist, metres.
        elbow_out: f32,
        /// When the hips accelerate most, seconds into the climb.
        accelerated_at: f32,
        /// The deepest any joint goes into the block (the wall below the
        /// edge, the top behind it), metres, and which.
        into_block: f32,
        deepest: Option<(Bone, f32)>,
        /// The most a bent knee's hinge is turned from standing's, in the
        /// pelvis's frame, radians: past a right angle, the knee bends
        /// backward or the thigh is turned round at the hip. In the thigh's
        /// own frame it was blind to a thigh turned about its own line, the
        /// knee 0.48 m behind the leg.
        knee_turned: f32,
        /// The most an ankle strays from where it is placed: braced on the
        /// wall through the pull, and on the top once it comes onto it,
        /// metres.
        wall_off: f32,
        landed_off: f32,
        /// The hips' greatest acceleration, m/s².
        acceleration: f32,
        /// At the end: the root from its spot on the top, each ankle from its
        /// own, metres; whether done.
        root_off: f32,
        ankles_off: f32,
        done: bool,
        /// The pose at the end.
        last: Option<LocalPose>,
    }

    /// How deep `p` is inside the block under `ledge`: in from the face and
    /// down from the top, the lesser; 0 outside it.
    fn inside(ledge: &super::super::Ledge, p: Vec3) -> f32 {
        let (into, down) = (-ledge.out_of(p), ledge.height() - p.y);
        if into > 0.0 && into < ledge.depth && down > 0.0 && down < ledge.wall_below { into.min(down) } else { 0.0 }
    }

    fn climbed(ledge: &super::super::Ledge) -> Measured {
        let (stood, rig) = real_stood();
        // Which way each knee folds standing, in the pelvis's frame: the
        // hinge, thigh across shin.
        let stood_folds = {
            let (at, pelvis) = (forward_kinematics_on(&stood, &rig), accumulate_world_rotations(&stood, &rig)[Bone::Hips]);
            LEGS.map(|(socket, knee, ankle, _)| (pelvis.inverse() * (at[knee] - at[socket]).cross(at[ankle] - at[knee])).normalize())
        };
        // Free, from mid-swing: the climb takes the hang's velocity on.
        let mut hanging = grabbing(ledge).expect("in reach");
        hanging.advance(if hanging.is_braced() { 3.0 } else { 1.5 });
        let mut hips = Vec::new();
        for _ in 0..2 {
            hanging.advance(DT);
            hips.push(hanging.hang_hips());
        }
        assert!(hanging.climb_up(), "would not climb up");
        // Swinging, it waits for the swing to slow.
        let mut waited = 0.0;
        while !hanging.is_climbing_up() {
            hanging.advance(DT);
            hips.push(hanging.hang_hips());
            waited += DT;
            assert!(waited < 3.0, "never started climbing up");
        }
        let mut m = Measured::default();
        let ends = hanging.up.as_ref().expect("climbing").ends;
        while !hanging.is_done() {
            hanging.advance(DT);
            let up = hanging.up.as_ref().expect("climbing");
            let pose = hanging.pose(&rig);
            let at = forward_kinematics_on(&pose, &rig);
            let turn = Quat::from_rotation_y(hanging.facing());
            let world = BoneSet::from_fn(|bone| hanging.root() + turn * at[bone]);
            hips.push(world[Bone::Hips]);
            let (wrists, _, _, holding) = hanging.hands_up(up);
            for side in 0..2 {
                let off = (world[ARMS[side].wrist] - wrists[side]).length();
                if up.t < ends[0] {
                    m.hooked_off = m.hooked_off.max(off);
                } else if up.t > ends[1] && holding >= 1.0 {
                    if off > m.pressed_off {
                        (m.pressed_off, m.pressed_at) = (off, up.t);
                    }
                    // The elbow out to its side of the shoulder-to-wrist line.
                    let (shoulder, elbow, wrist) = (world[ARMS[side].shoulder], world[ARMS[side].elbow], world[ARMS[side].wrist]);
                    let line = (wrist - shoulder).normalize_or_zero();
                    let bent = elbow - shoulder - line * (elbow - shoulder).dot(line);
                    m.elbow_out = m.elbow_out.max(bent.dot(turn * rig.left() * SIGN[side]));
                }
            }
            for side in 0..2 {
                let off = (world[LEGS[side].2] - hanging.foot_up(up, side).0).length();
                let landed = if side == up.lead { ends[3] } else { ends[3] + TRAIL_ON.1 * (ends[4] - ends[3]) };
                if up.holds.is_some() && up.t < ends[1] {
                    m.wall_off = m.wall_off.max(off);
                } else if up.t >= landed {
                    m.landed_off = m.landed_off.max(off);
                }
            }
            let pelvis = accumulate_world_rotations(&pose, &rig)[Bone::Hips];
            for (side, (socket, knee, ankle, _)) in LEGS.into_iter().enumerate() {
                let fold = (at[knee] - at[socket]).normalize_or_zero().cross((at[ankle] - at[knee]).normalize_or_zero());
                if fold.length() > KNEE_BENT {
                    m.knee_turned = m.knee_turned.max((pelvis.inverse() * fold).normalize().dot(stood_folds[side]).clamp(-1.0, 1.0).acos());
                }
            }
            for bone in Bone::ALL {
                let depth = inside(ledge, world[bone]);
                if depth > m.into_block {
                    (m.into_block, m.deepest) = (depth, Some((bone, up.t)));
                }
            }
            m.last = Some(pose);
        }
        for (k, w) in hips.windows(3).enumerate() {
            let a = ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length();
            if a > m.acceleration {
                (m.acceleration, m.accelerated_at) = (a, k as f32 * DT);
            }
        }
        let up = hanging.up.as_ref().expect("climbing");
        m.root_off = (hanging.root() - up.end_root).length();
        let pose = m.last.expect("posed");
        let at = forward_kinematics_on(&pose, &rig);
        let turn = Quat::from_rotation_y(hanging.facing());
        m.ankles_off = (0..2).map(|side| (hanging.root() + turn * at[LEGS[side].2] - up.spots[side]).length()).fold(0.0, f32::max);
        m.done = hanging.is_done();
        m
    }

    /// From a braced hang and a free one, on ledges across a jump's reach,
    /// it climbs onto the top: the hands held on the lip pulling and on the
    /// top pressing, nothing through the block, and standing still on the
    /// top at the end, its feet where the standing pose has them.
    #[test]
    fn it_climbs_up_from_either_hang_onto_the_top() {
        for height in [1.95, 2.35] {
            for below in [height, 0.15] {
                let ledge = wall(height, below);
                let m = climbed(&ledge);
                let name = format!("{height} m, {below} m of wall");
                assert!(m.done, "{name}: not done");
                assert!(m.hooked_off < 1.0e-3, "{name}: a hand {:.4} m off its hook pulling", m.hooked_off);
                assert!(m.pressed_off < 1.0e-3, "{name}: a hand {:.4} m off its press at {:.2} s", m.pressed_off, m.pressed_at);
                // Pressing, the elbows back by the body (pointing out as
                // hanging, 18.5 cm out; a quarter out, 6.8 cm).
                assert!(m.elbow_out < 0.03, "{name}: an elbow {:.4} m out to its side pressing", m.elbow_out);
                assert!(m.into_block < 1.0e-3, "{name}: {:?} {:.4} m into the block", m.deepest, m.into_block);
                assert!(m.knee_turned < KNEE_TURN, "{name}: a knee's hinge turned {:.2} rad from standing's", m.knee_turned);
                assert!(m.wall_off < 1.0e-3 && m.landed_off < 1.0e-3, "{name}: an ankle {:.4} m off the wall, {:.4} m off the top", m.wall_off, m.landed_off);
                assert!(m.root_off < 1.0e-4 && m.ankles_off < 1.0e-3, "{name}: the root {:.4} m off its spot, an ankle {:.4} m", m.root_off, m.ankles_off);
                assert!(m.acceleration < MOST_ACCELERATION, "{name}: the hips accelerated {:.1} m/s² at {:.2} s", m.acceleration, m.accelerated_at);
            }
        }
    }

    /// The hips' acceleration a climb up stays under, from the hang it
    /// starts from on, m/s²: no data; a pull and press well under 1 g
    /// (4.3 measured).
    const MOST_ACCELERATION: f32 = 6.0;
    /// A knee counts as bent past this sine of its fold; its hinge stays
    /// within this of standing's, radians: about as far as a hip turns a
    /// thigh out (the knees turned out on the wall, 0.94).
    const KNEE_BENT: f32 = 0.2;
    const KNEE_TURN: f32 = 1.0;

    /// Standing at the end, it is in the standing pose, its hips down by the
    /// foot IK's drop: the walker stands on from there.
    #[test]
    fn it_ends_in_the_standing_pose() {
        let (stood, rig) = real_stood();
        let m = climbed(&wall(2.15, 2.15));
        let pose = m.last.expect("posed");
        let (now, then) = (forward_kinematics_on(&pose, &rig), forward_kinematics_on(&stood, &rig));
        for bone in [Bone::Head, Bone::LeftHand, Bone::RightHand, Bone::LeftShoulder, Bone::RightShoulder] {
            assert!((now[bone] - then[bone]).length() < 0.01, "{bone:?} {:.4} m from standing", (now[bone] - then[bone]).length());
        }
    }

    /// With no room on the top to stand, it does not climb up.
    #[test]
    fn a_shallow_top_is_not_climbed() {
        let ledge = super::super::Ledge { depth: 0.3, ..wall(2.15, 2.15) };
        let mut hanging = grabbing(&ledge).expect("in reach");
        hanging.advance(3.0);
        assert!(!hanging.climb_up());
    }
}
