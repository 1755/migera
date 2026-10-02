//! Stage 4 — an active ragdoll that tracks the kinematic pose.
//!
//! # What this buys
//!
//! Stages 1-3 produce a pose. That pose is *correct* but it is not
//! *physical*: a character shoved by an explosion, leaned on by a falling
//! crate, or clipped by a projectile carries on holding it, because
//! nothing in the kinematic stack knows a force happened.
//!
//! Here the kinematic result becomes a **target** rather than an output. A
//! simulated skeleton of rigid bodies chases it through joint torques. Under
//! ordinary conditions the torques win and the character moves exactly as
//! animated. When something exceeds
//! [`max_torque`](super::math::pd::PdParams::max_torque), it does not — the
//! character yields, and then recovers as the controllers keep pulling.
//!
//! Both the break and the recovery are emergent. There is no flinch
//! animation, no get-up clip, and no state machine deciding when to hand
//! over to physics.
//!
//! # Why not avian's own joint motors
//!
//! avian 0.7's `AngularMotor` lives only on `RevoluteJoint` and
//! `PrismaticJoint`, and its target is a **scalar** angle. A shoulder needs
//! three degrees of freedom, and `SphericalJoint` — the joint actually
//! shaped like one — has no motor.
//!
//! Decomposing each ball joint into motorized revolutes is the obvious
//! workaround, and this project has already tried it: two independently
//! simulated rotational springs sharing an intermediate body fought each
//! other into a reproducible instability. The superseded `muscle` module
//! exists partly because of that.
//!
//! So the ball constraint is an **unmotorized** `SphericalJoint`, and
//! orientation comes from one [`pd_torque`] per joint. A single rotational
//! actuator per joint cannot fight a second one, because there is no second
//! one — the failure is structurally absent rather than tuned away.
//!
//! # The strength dial
//!
//! Per-joint [`RagdollStrength`] blends between tracking and limpness, and
//! it is continuous rather than a mode switch. That generality is borrowed
//! from Lugaru's own per-muscle `strength`, and it is strictly more
//! expressive than a binary animated/ragdoll flag: an arm can go slack
//! while the legs keep walking, and a stunned character can recover
//! gradually rather than snapping back.

use bevy::prelude::*;

use super::math::pd::{pd_torque_at, pd_torque_tracking, PdParams};
use super::rig::{BoneSet, LocalPose};
use crate::character::skeleton::Bone;

/// How much a joint is being driven, from limp to fully tracking.
///
/// `0.0` is a free ragdoll joint; `1.0` tracks the kinematic pose with the
/// full [`PdParams::max_torque`]. Intermediate values scale the torque cap,
/// which is what makes "stunned but not unconscious" expressible.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RagdollStrength(pub f32);

impl Default for RagdollStrength {
    fn default() -> Self {
        Self(1.0)
    }
}

impl RagdollStrength {
    /// Fully limp.
    pub const LIMP: Self = Self(0.0);
    /// Fully driven.
    pub const DRIVEN: Self = Self(1.0);

    /// The effective torque ceiling at this strength.
    pub fn scale(self, max_torque: f32) -> f32 {
        max_torque * self.0.clamp(0.0, 1.0)
    }
}

/// One character's simulated bodies and their tuning.
///
/// Absent by default: a character without this is purely kinematic, which
/// is what most of them should be most of the time.
#[derive(Component, Debug, Clone)]
pub struct Ragdoll {
    /// The simulated body standing in for each bone, where one exists.
    /// Leaf bones and bones too small to be worth simulating are `None`.
    pub bodies: BoneSet<Option<Entity>>,
    /// Per-joint controller tuning.
    pub params: BoneSet<PdParams>,
    /// Per-joint strength.
    pub strength: BoneSet<RagdollStrength>,
    /// Per-joint rotation limits, read when the joints are created.
    ///
    /// Changing this after spawn has no effect: avian's constraint owns the
    /// limits once the joint entity exists. Set it before building the
    /// ragdoll.
    pub limits: BoneSet<Option<JointLimits>>,
    /// Per-joint strength temporarily lost to hits. See [`Ragdoll::stun`].
    ///
    /// Kept apart from [`Ragdoll::strength`] so a hit never overwrites the
    /// strength the game chose: recovery returns each joint to whatever its
    /// dial says, not to a hardcoded "fully driven".
    pub stun: BoneSet<Stun>,
    /// How a hit spreads through the body and how quickly it wears off.
    pub stun_response: StunResponse,
    /// The pose last shown on screen: the animation blended with the
    /// simulation. `None` until the first read-back.
    ///
    /// Kept HERE, out of `AnimPose`'s spring state, for the reason that
    /// struct already documents for foot IK. Writing it back into the
    /// spring made next frame's "animated" pose contain last frame's
    /// simulation — the display blend became a low-pass filter toward the
    /// physics, and the PD targets chased the bodies instead of the
    /// animation. Latent while strength sat at exactly 0 or 1; the first
    /// hit to sweep strength through the values between froze a recovered
    /// forearm 55.9 degrees off its animation.
    pub displayed: Option<LocalPose>,
    /// Let go to fall, if it has been. See [`Ragdoll::fall`].
    pub fall: Option<Fall>,
    /// Each body's centre in its bone's own frame, as spawned: a bone is
    /// rigid, so this places a body on its bone in any pose (a rise's end
    /// sets the bodies back onto the animation with it).
    pub body_offsets: BoneSet<Vec3>,
    /// The poses a rise passes through, built on the rig when it starts
    /// ([`super::getup::keys`]).
    pub rise_keys: Vec<super::getup::GetUpKey>,
    /// Which feet and hands (left foot, right foot, left hand, right hand)
    /// move in the rise's current segment; only they may be tucked clear of
    /// the floor.
    pub rise_moving: [bool; 4],
    /// Each body's ball joint to its parent, by the child's bone.
    pub joints: BoneSet<Option<Entity>>,
    /// The hinges a fall turns the knees and elbows into, by the child's
    /// bone, as measured at spawn. See [`Hinge`].
    pub hinges: BoneSet<Option<Hinge>>,
    /// The hinge joints standing in for those ball joints while falling.
    pub hinge_joints: BoneSet<Option<Entity>>,
    /// Each bone's limit-only joints (`ragdoll_plugin::LimitOnly`), which
    /// share its main joint's anchors and so move them with it.
    pub limit_joints: BoneSet<Vec<Entity>>,
    /// Each body's position and rotation the last frame a fall was checked
    /// for rest: rest is judged by how the bodies move, not by their
    /// velocities (`ragdoll_plugin::REST_SPEED`).
    pub last_seen: BoneSet<Option<(Vec3, Quat)>>,
    /// Standing on its own feet ([`Ragdoll::stand_on_own_feet`]): the root
    /// is not pinned, gravity acts in full, and the joints carry the weight
    /// through joint torques (`joint_drive`).
    pub self_supporting: bool,
    /// Standing on its own feet, where the centre of mass rests over them:
    /// its horizontal offset from the planted feet's middle, world, taken
    /// when both were first planted (`joint_drive::carry_weight`).
    pub stand_rest: Option<Vec3>,
    /// How far the screen has gone over to the bodies standing on their
    /// own feet, `0..=1`: eased in and out over [`SWITCH_SECONDS`] as the
    /// body starts or stops carrying itself, so neither switch pops.
    pub stand_blend: f32,
    /// Whether a body standing on its own feet steps when its feet cannot
    /// catch it, rather than falling. On by default: it only acts where the
    /// body would otherwise fall, and it catches pushes about 0.2 m/s
    /// harder each way (see the knowledge note on a standing ragdoll).
    pub steps_on_own_feet: bool,
    /// A recovery step under way on its own feet
    /// (`joint_drive::carry_weight`).
    pub own_step: Option<OwnStep>,
    /// Standing on its own feet, the right ankle's place from the left's as
    /// the body first stood (horizontal, world): where a joining foot goes.
    pub feet_home: Option<Vec3>,
    /// Standing on its own feet, how high the hip joints stood over the
    /// ankles as the body first stood, metres: what a planted leg is solved
    /// to hold the hips at after a step.
    pub stand_height: Option<f32>,
    /// The leg the weight is moving onto before the other joins it, after a
    /// recovery step on its own feet.
    pub own_transfer: Option<usize>,
    /// Where the COM comes to rest after a recovery step on its own feet
    /// (horizontal `x`, `z`, world): the capture point as the step landed,
    /// held to until the feet are together again.
    pub own_rest: Option<Vec2>,
    /// How each drawn wrist (left, right) is turned to keep its fingertips
    /// out of the ground, in the hand's own frame, as last drawn: the next
    /// frame's turn changes from it as little as it can
    /// (`ragdoll_plugin::hold_clear`).
    pub wrist_lift: [Quat; 2],
    /// How each rising arm (left, right) is turned at the shoulder to keep
    /// its hand out of the ground, in the upper arm's own frame, as last
    /// drawn (`ragdoll_plugin::hold_clear`).
    pub arm_lift: [Quat; 2],
    /// How far each drawn knee (left, right) is folded to keep its foot out
    /// of the ground while rising, radians, signed about its hinge, as last
    /// drawn: the fold changes at a limited rate
    /// (`ragdoll_plugin::TUCK_DEEPEN_RATE`, `TUCK_RATE`).
    pub tucks: [f32; 2],
    /// How much of a relaxed body's passive joint tone a fall keeps
    /// (`passive`), `0..`: 1 the measured tone, 0 none (limbs free to
    /// their stops).
    pub passive_tone: f32,
}

/// A recovery step a body standing on its own feet is taking: one foot
/// swinging from where it stood to where the capture point will be.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OwnStep {
    /// Which leg: 0 the left, 1 the right.
    pub leg: usize,
    /// The ankle's joint point where it lifted, and where it lands, world.
    pub from: Vec3,
    /// See `from`.
    pub to: Vec3,
    /// Seconds into the swing, and how long it takes.
    pub elapsed: f32,
    /// See `elapsed`.
    pub duration: f32,
    /// The foot's world rotation as it lifted, flat on the floor: it is
    /// carried so through the swing, to land flat.
    pub foot: Quat,
    /// Whether it is a recovery step, its landing aimed afresh every frame
    /// at where the capture point will be (a join's landing is fixed).
    pub recovery: bool,
}

/// How long switching between a pinned root and standing on its own feet
/// takes, seconds: the screen's blend between the animation and the
/// bodies, and the pinned root's way back to the animation. A choice:
/// a little under a weight shift (Winter's MVC builds in ~200 ms, §9.0.6).
pub const SWITCH_SECONDS: f32 = 0.3;

/// A knee or elbow as the hinge it is: one bending axis fixed in the parent
/// segment (the femur's, the humerus's), within an anatomical range.
///
/// Standing, the ball joint stays: the pose controller holds the limb, and
/// a pose may roll a forearm 90 degrees (the wave), which a hinge would
/// lock. A limp body has no controller, and a ball joint let a knee fold
/// 159 degrees backward and 88 sideways in a fall. So a fall swaps each ball
/// joint for a hinge set up from where the limb is at that moment, its
/// roll and sideways tilt frozen, and the rise's end swaps it back.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hinge {
    /// The bending axis in the parent body's frame; bending about it by a
    /// positive angle is flexion.
    pub axis: Vec3,
    /// The allowed bend, radians, from the straight limb: (hyperextension,
    /// full flexion).
    pub range: (f32, f32),
    /// The parent's and the child's segment directions, each in its own
    /// body's frame: the bend is the angle between them about `axis`.
    pub segments: (Vec3, Vec3),
    /// The joint's anchor in each body's frame.
    pub anchors: (Vec3, Vec3),
}

/// A knee's bend, degrees: AAOS normal flexion is 0-135; a limp limb is
/// moved passively a little further, and a little past straight.
pub const KNEE_RANGE: (f32, f32) = (-5.0, 140.0);

/// An elbow's bend, degrees: AAOS normal flexion is 0-150.
pub const ELBOW_RANGE: (f32, f32) = (-5.0, 150.0);

impl Hinge {
    /// The bend now, radians, with the parent and child bodies at these
    /// world rotations.
    pub fn bend(&self, parent: Quat, child: Quat) -> f32 {
        let axis = parent * self.axis;
        let (p, c) = (parent * self.segments.0, child * self.segments.1);
        axis.dot(p.cross(c)).atan2(p.dot(c))
    }
}

/// A ragdoll let go to fall: its root is no longer pinned, gravity acts on
/// every body in full, each joint keeps only `tone` of its strength, and
/// the screen shows the simulation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fall {
    /// The strength every joint's pose controller keeps, `0..=1`.
    pub tone: f32,
    /// How strongly each joint resists motion, 1/s: avian's `JointDamping`
    /// on every joint, damping the two bodies' relative spin.
    pub damping: f32,
    /// Set once the root has been released: the hips body's centre in the
    /// hips bone's frame, which the display needs to place the skeleton
    /// on the body.
    pub root_offset: Option<Vec3>,
    /// How long every body has been slower than the rest bounds, seconds
    /// (see `ragdoll_plugin::REST_SPEED`).
    pub still_for: f32,
    /// Set once the body has come to rest, and is asleep.
    pub at_rest: bool,
    /// Getting back up, once asked ([`Ragdoll::get_up`]).
    pub rise: Option<Rise>,
    /// A velocity every body is given at release, m/s, world: the momentum
    /// of what knocked it over ([`Ragdoll::fall_moving`]).
    pub launch: Vec3,
}

/// A fallen body being handed back to animation. The bodies stay asleep
/// where they lie; the screen blends from them through the get-up keys
/// ([`Ragdoll::rise_keys`], chosen by how the body lies) to the animation,
/// and then the bodies are set onto the animated pose and the root is
/// pinned again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rise {
    /// Seconds since the rise was asked for, less its delay: negative
    /// while still lying.
    pub elapsed: f32,
    /// How the body lies, once read (the first frame of the rise).
    pub lying: Option<super::getup::Lying>,
    /// The turn about `+Y` the character should make, radians, to face the
    /// way it rises: toward its head face down, its feet face up.
    pub turn: f32,
    /// Set with `turn` for whatever owns the character's heading to apply
    /// once and clear. (The character's rotation is not the ragdoll's to
    /// write: in the gallery a facing controller rewrites it every frame.)
    pub turn_pending: bool,
}

/// How much pose-controller strength a falling body keeps by default: none.
/// At 0.15 the controller kept driving the limbs toward the standing pose
/// on the floor, and a forearm still moved at 0.57 m/s 3 s after the fall.
pub const FALL_TONE: f32 = 0.0;

/// How strongly a falling body's joints resist motion by default, 1/s: its
/// muscle tone, as resistance to motion rather than a pose to hold.
///
/// Chosen by sweep (`ragdoll_plugin::tests::probe_fall_damping`):
/// `puppet_base` carried at 1 m/s forward, backward and sideways, then let
/// go, at 12 substeps. Only 1-3/s brought every fall to rest (asleep by
/// 2.9-4.8 s). At 0 two of three never slept and the limbs whipped at
/// 7.4 m/s. At 10 the backward fall never slept: heavy damping turns a
/// collapse into a slow ooze, and live a hips-up body slid 12 mm/s for
/// seconds. 3 whips least (6.1 m/s) of those that rest.
pub const FALL_DAMPING: f32 = 3.0;

impl Default for Ragdoll {
    fn default() -> Self {
        Self {
            bodies: BoneSet::splat(None),
            params: default_joint_params(),
            strength: BoneSet::splat(RagdollStrength::DRIVEN),
            limits: default_joint_limits(),
            stun: BoneSet::splat(Stun::default()),
            stun_response: StunResponse::default(),
            displayed: None,
            fall: None,
            body_offsets: BoneSet::splat(Vec3::ZERO),
            rise_keys: Vec::new(),
            rise_moving: [true; 4],
            joints: BoneSet::splat(None),
            hinges: BoneSet::splat(None),
            hinge_joints: BoneSet::splat(None),
            limit_joints: BoneSet::from_fn(|_| Vec::new()),
            last_seen: BoneSet::splat(None),
            self_supporting: false,
            stand_rest: None,
            stand_blend: 0.0,
            steps_on_own_feet: true,
            own_step: None,
            feet_home: None,
            stand_height: None,
            own_transfer: None,
            own_rest: None,
            wrist_lift: [Quat::IDENTITY; 2],
            arm_lift: [Quat::IDENTITY; 2],
            tucks: [0.0; 2],
            passive_tone: 1.0,
        }
    }
}

impl Ragdoll {
    /// Lets the body stand on its own feet (plan step 4b): the root is
    /// released, gravity acts on every body in full, and every joint
    /// carries its load with a joint torque between its two bodies
    /// (`joint_drive::JointDrive`) instead of the pose controller's
    /// per-body acceleration, and balances over its feet by Winter's
    /// pendulum law (`joint_drive::carry_weight`). A push its feet cannot
    /// catch makes it fall, and the fall takes over as before. No way back
    /// to a pinned root yet (that is mode switching, step 4.5), and no
    /// step yet.
    pub fn stand_on_own_feet(&mut self) {
        self.self_supporting = true;
    }

    /// Back to a root pinned to the animation (step 4.5): the drives go,
    /// the root is pinned where the body stands and eased to the
    /// animation over [`SWITCH_SECONDS`], and the screen blends back to it.
    pub fn stop_standing_on_own_feet(&mut self) {
        self.self_supporting = false;
    }

    /// Whether the joints carry the body now: standing on its own feet and
    /// not falling.
    pub fn carries_itself(&self) -> bool {
        self.self_supporting && self.fall.is_none()
    }

    /// Sets every joint's strength at once — the whole-body dial between
    /// animated and limp.
    pub fn set_strength(&mut self, strength: f32) {
        self.strength = BoneSet::splat(RagdollStrength(strength));
    }

    /// Whether any joint is being driven at all.
    pub fn is_driven(&self) -> bool {
        self.strength.iter().any(|(_, s)| s.0 > 0.0)
    }

    /// The strength a joint actually has right now: its dial, less whatever
    /// a hit has knocked out of it.
    ///
    /// Every reader of strength — the torque ceiling and the read-back's
    /// display blend — must go through this rather than reading
    /// [`Ragdoll::strength`] directly, or a hit weakens one and not the
    /// other: a limb that physically yields but is still *shown* animated,
    /// or one shown limp while its controller holds it rigid.
    pub fn effective_strength(&self, bone: Bone) -> RagdollStrength {
        let mut dial = self.strength[bone].0.clamp(0.0, 1.0);
        if let Some(fall) = self.fall {
            dial = dial.min(fall.tone.clamp(0.0, 1.0));
        }
        let lost = self.stun[bone].limpness.clamp(0.0, 1.0);
        RagdollStrength(dial * (1.0 - lost))
    }

    /// How much of the simulation the screen shows at `bone`, `0..=1`:
    /// where the controller no longer enforces the animation, and all of
    /// it while falling, when the body is the character.
    pub fn shown(&self, bone: Bone) -> f32 {
        match self.fall {
            Some(_) => 1.0,
            None => {
                let driven = 1.0 - self.effective_strength(bone).0;
                let t = self.stand_blend.clamp(0.0, 1.0);
                driven + (1.0 - driven) * t * t * (3.0 - 2.0 * t)
            }
        }
    }

    /// Where a rise is: the segment (0 from lying to the first key, then
    /// key to key, the last up to standing) and how far through it, eased.
    /// `None` unless a rise has started moving.
    pub fn rise_segment(&self) -> Option<(usize, f32)> {
        let rise = self.fall?.rise?;
        if rise.elapsed <= 0.0 || rise.lying.is_none() {
            return None;
        }
        let mut left = rise.elapsed;
        let durations = self.rise_keys.iter().map(|key| key.seconds).chain([super::getup::STAND_SECONDS]);
        let count = self.rise_keys.len() + 1;
        for (segment, seconds) in durations.enumerate() {
            if left < seconds || segment + 1 == count {
                let t = if seconds > 0.0 { (left / seconds).clamp(0.0, 1.0) } else { 1.0 };
                return Some((segment, t * t * (3.0 - 2.0 * t)));
            }
            left -= seconds;
        }
        None
    }

    /// How long a rise takes once moving, seconds.
    pub fn rise_seconds(&self) -> f32 {
        self.rise_keys.iter().map(|key| key.seconds).sum::<f32>() + super::getup::STAND_SECONDS
    }

    /// How much of its own weight `bone`'s body carries against gravity,
    /// `0..=1`: its strength (see `ragdoll_plugin::support_own_weight`),
    /// and none while falling or standing on its own feet, when gravity
    /// acts in full and the joints carry the weight.
    pub fn carried_weight(&self, bone: Bone) -> f32 {
        if self.fall.is_some() || self.self_supporting { 0.0 } else { self.effective_strength(bone).0 }
    }

    /// Lets the body fall (H2): the root is released at the next
    /// `RagdollSet::Hit`, keeping the velocity it had, and every joint
    /// keeps `tone` of its strength (see [`Fall`]). A second call while
    /// already falling changes nothing.
    pub fn fall(&mut self, tone: f32, damping: f32) {
        self.fall_moving(tone, damping, Vec3::ZERO);
    }

    /// [`Ragdoll::fall`], the whole body moving at `velocity` (m/s, world)
    /// as it is let go: the push that toppled it. Without it a fall judged
    /// at the start of a push began from rest, and every push, whatever
    /// its direction, dropped the body the same way onto its back.
    pub fn fall_moving(&mut self, tone: f32, damping: f32, velocity: Vec3) {
        if self.fall.is_none() {
            self.fall = Some(Fall {
                tone,
                damping,
                root_offset: None,
                still_for: 0.0,
                at_rest: false,
                rise: None,
                launch: velocity,
            });
        }
    }

    /// Hands a fallen body back to animation (see [`Rise`]): after `delay`
    /// seconds lying, it rises through the get-up keys for how it lies.
    /// Only once it is at rest ([`Fall::at_rest`]); returns whether the
    /// rise began.
    pub fn get_up(&mut self, delay: f32) -> bool {
        match self.fall.as_mut() {
            Some(fall) if fall.at_rest && fall.rise.is_none() => {
                fall.rise = Some(Rise { elapsed: -delay.max(0.0), lying: None, turn: 0.0, turn_pending: false });
                self.rise_keys.clear();
                true
            }
            _ => false,
        }
    }

    /// Whether the body has been let go to fall.
    pub fn is_falling(&self) -> bool {
        self.fall.is_some()
    }

    /// Whether any joint is still recovering from a hit.
    pub fn is_stunned(&self) -> bool {
        self.stun.iter().any(|(_, stun)| stun.limpness > 0.0)
    }

    /// Knocks strength out of `struck` and, more weakly, out of the joints
    /// around it.
    ///
    /// `amount` is the fraction of strength the struck joint loses, `0..=1`.
    /// Each joint further away along the skeleton loses
    /// [`StunResponse::spread`] times as much as the one before it, so a
    /// blow to the forearm goes slack at the elbow, softens the shoulder a
    /// little, and leaves the legs alone.
    ///
    /// # Why a hit has to weaken the joint at all
    ///
    /// The read-back SHOWS the simulation only where a joint is weak
    /// (`slerp(animated, simulated, 1 - strength)`). At the default full
    /// strength a hit moves the body and the character is rendered exactly
    /// as animated anyway — the impulse is real and invisible. Losing
    /// strength is what makes the blow land on screen, and regaining it is
    /// the recovery.
    ///
    /// Repeated hits take the larger of the two losses rather than adding
    /// them: two jabs do not make a joint more than fully limp, and a light
    /// tap must not cut short a heavy blow's recovery.
    pub fn stun(&mut self, struck: Bone, amount: f32) {
        let amount = amount.clamp(0.0, 1.0);
        if amount <= 0.0 {
            return;
        }

        let response = self.stun_response;
        for (bone, stun) in self.stun.iter_mut() {
            let level = amount * response.spread.powi(joint_distance(struck, bone) as i32);
            // Below this a joint's loss is invisible, and leaving it at zero
            // keeps `is_stunned` from reporting a whole body as recovering
            // over a rounding error.
            if level < STUN_VISIBLE {
                continue;
            }

            stun.limpness = stun.limpness.max(level);
            stun.hold = stun.hold.max(response.hold_secs);
        }
    }

    /// Advances every joint's recovery by `dt` seconds.
    ///
    /// A stunned joint first *holds* its loss for
    /// [`StunResponse::hold_secs`] — the moment a real body spends yielding
    /// before it starts correcting — then regains strength linearly over
    /// [`StunResponse::recover_secs`].
    ///
    /// Linear in strength, not in pose: the PD controller is itself a
    /// critically-damped second-order system, so a steadily rising torque
    /// ceiling already produces a smooth return.
    pub fn recover(&mut self, dt: f32) {
        let response = self.stun_response;

        for (_, stun) in self.stun.iter_mut() {
            if stun.limpness <= 0.0 {
                continue;
            }

            // Time spent holding is time not spent recovering, within the
            // same step — otherwise the recovery would lag by up to a frame
            // each time a hold ran out.
            let held = stun.hold.min(dt).max(0.0);
            stun.hold -= held;
            let recovering = dt - held;

            stun.limpness = if response.recover_secs <= 0.0 {
                0.0
            } else {
                (stun.limpness - recovering / response.recover_secs).max(0.0)
            };

            if stun.limpness <= 0.0 {
                *stun = Stun::default();
            }
        }
    }
}

/// Strength a joint has temporarily lost to a hit.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Stun {
    /// Fraction of the joint's strength currently lost, `0..=1`.
    pub limpness: f32,
    /// Seconds left before this joint starts to recover.
    pub hold: f32,
}

/// How a hit spreads through the body and how long it lasts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StunResponse {
    /// How much of a joint's loss its neighbour one joint further away
    /// suffers, `0..1`. At `0.5`, a fully stunned elbow halves the shoulder
    /// and quarters the collarbone.
    pub spread: f32,
    /// How long a stunned joint stays at its loss before recovering.
    pub hold_secs: f32,
    /// How long a fully limp joint takes to regain its strength once the
    /// hold is over.
    pub recover_secs: f32,
}

impl Default for StunResponse {
    fn default() -> Self {
        // A quarter of a second of yielding and just over a second to
        // recover reads as a solid hit a person shrugs off — long enough to
        // see, short enough not to look like a knockout.
        Self { spread: 0.5, hold_secs: 0.25, recover_secs: 1.2 }
    }
}

/// A joint losing less than this to a hit is left untouched.
const STUN_VISIBLE: f32 = 0.02;

/// How many joints apart two bones are along the skeleton.
///
/// The path through their nearest common ancestor: a forearm and the
/// opposite forearm are six joints apart, not a straight-line distance —
/// which is how a blow actually travels through a body.
pub fn joint_distance(from: Bone, to: Bone) -> u32 {
    let mut up_from = 0;
    let mut ancestor = Some(from);

    while let Some(common) = ancestor {
        let mut up_to = 0;
        let mut walker = Some(to);
        while let Some(bone) = walker {
            if bone == common {
                return up_from + up_to;
            }
            up_to += 1;
            walker = bone.parent();
        }

        up_from += 1;
        ancestor = common.parent();
    }

    // Every bone descends from `Hips`, so the walk above always meets.
    unreachable!("{} and {} share no ancestor", from.name(), to.name())
}

/// Per-joint PD tuning, graded by how much each joint has to move.
///
/// A hip swings the whole upper body and can correct harder than a wrist.
/// Uniform limits either leave the heavy joints sagging or let the light
/// ones snap, so the defaults are graded — the same reasoning behind the
/// per-bone spring half-lives in [`super::dho`].
///
/// `max_torque` is an angular **acceleration** in rad/s², not a torque —
/// see [`PdParams::max_torque`] for why. For scale, correcting a
/// 90-degree error in about 0.15 s needs roughly 140 rad/s², so the
/// 30-400 band here spans "a neck nudging itself level" to "a hip catching
/// a stumble".
/// # Why no joint goes above 8 Hz
///
/// The frequency is bounded by the physics timestep, not by taste. A PD
/// controller integrated explicitly is stable only while `kd * dt < 2`, and
/// at avian's 64 Hz default with `damping_ratio = 1.0` that puts the
/// ceiling at about 10 Hz — where `kd * dt` reaches 1.96, i.e. 98% of the
/// hard limit.
///
/// The heavy joints were originally authored at 9-10 Hz, on the reasoning
/// that a hip should correct harder than a wrist. Measured on a jointed
/// chain, that reasoning was right about the intent and wrong about the
/// dial: the 10 Hz hip was the *worst*-behaved joint in the rig,
/// overshooting a 15-degree target to 29 degrees and vibrating at
/// 7.2 rad/s, precisely because it sat at the stability boundary.
///
/// "Corrects harder" is what `max_torque` expresses, and it still does —
/// the hip's ceiling is 20x the neck's. Frequency is how fast the
/// correction is *integrated*, which is a property of the solver, not of
/// the joint's role. Raising it past what the step can carry does not make
/// a joint stronger, it makes it unstable.
///
/// `every_default_joint_is_well_conditioned_for_the_physics_timestep`
/// enforces this, so a future edit that reaches for a higher frequency
/// fails loudly rather than shipping a joint that chatters.
///
/// # Why every ceiling is scaled by [`CEILING_SCALE`]
///
/// The graded values below were authored so arms would "yield sooner" to a
/// shove. They also could not produce the accelerations the character's
/// OWN animation demands: a ball joint carries no torque, so a forearm
/// holds its angle against being dragged by a swinging upper arm with its
/// own controller alone, and at the authored 40 rad/s² it saturated. A
/// saturated controller in a chain flails — measured live while walking,
/// forearms, feet and head swung 20-176 degrees off with the torso on
/// target. Swept headless (a walking root; a 1 Hz arm swing):
///
/// | ceilings | walk | swing |
/// |---|---|---|
/// | x1 | 39.8° (head) | 79.7° (forearm) |
/// | x3 | 0.9° | 113.9° (forearm) |
/// | x6 | 1.2° | 10.0° (the swung arm's own lag) |
///
/// That sweep ran on the synthetic rig. On the REAL rig walking its own
/// gait cycle, x6 still left the arms saturated: the upper arm's body swung
/// 71.7 degrees against a 57.3-degree target swing — overshoot, and 34
/// degrees of error. At x12 it swings 57.4, matching. What then remains,
/// ~19 degrees on the legs, does not move with authority: it is lag on the
/// fastest-swinging targets, not saturation.
///
/// Joint damping, tried alongside, changed nothing once the ceilings were
/// adequate, and velocity feedforward made the feet worse (24 → 80
/// degrees). Yielding to a blow no longer needs a low ceiling: a hit STUNS
/// the joints it lands on (see [`Ragdoll::stun`]), which removes their
/// ceiling for exactly as long as the blow should show.
pub const CEILING_SCALE: f32 = 12.0;

pub fn default_joint_params() -> BoneSet<PdParams> {
    let pd = |frequency_hz: f32, ceiling: f32| PdParams {
        frequency_hz,
        damping_ratio: 1.0,
        max_torque: ceiling * CEILING_SCALE,
    };
    BoneSet::from_fn(|bone| match bone {
        // The root and lower spine carry everything above them — expressed
        // through the torque ceiling, not the frequency.
        Bone::Hips | Bone::Spine => pd(8.0, 400.0),
        Bone::Spine1 | Bone::Spine2 => pd(8.0, 250.0),
        // Legs hold the character up, so they get the most authority.
        Bone::LeftUpLeg | Bone::RightUpLeg => pd(8.0, 300.0),
        Bone::LeftLeg | Bone::RightLeg => pd(8.0, 200.0),
        Bone::LeftFoot | Bone::RightFoot => pd(8.0, 80.0),
        // Arms are lighter and yield sooner, relative to the rest.
        Bone::LeftShoulder | Bone::RightShoulder => pd(8.0, 90.0),
        Bone::LeftArm | Bone::RightArm => pd(8.0, 70.0),
        Bone::LeftForeArm | Bone::RightForeArm => pd(7.0, 40.0),
        // A hand follows everything the arm does above it: its ceiling is
        // the forearm's and more. At the default 20, an arm swing left it
        // 54° behind.
        Bone::LeftHand | Bone::RightHand => pd(7.0, 60.0),
        Bone::Neck | Bone::Head => pd(7.0, 30.0),
        _ => pd(6.0, 20.0),
    })
}

/// How far a joint may rotate before the constraint stops it.
///
/// Expressed the way avian's `SphericalJoint` consumes it: a **swing**
/// half-angle away from the bone's own axis (a cone), and a **twist** range
/// about that axis. Both in radians.
///
/// # Why a cone plus a twist rather than three Euler ranges
///
/// Per-axis Euler limits on a ball joint are the standard trap: the
/// decomposition is order-dependent and gimbal-locks, so a limit that reads
/// correctly in the editor silently means something else at a different
/// pose. Swing-twist has neither problem — the cone is rotationally
/// symmetric about the bone axis, and the twist is a single well-defined
/// angle about it. It is also exactly the shape avian already solves, so
/// nothing has to be converted.
///
/// # Why these are not tighter
///
/// These are *anatomical stops*, not a pose authoring tool. A limit exists
/// to stop a knee bending backwards or a neck rotating through the spine —
/// things that read as broken instantly. Shaping motion within the
/// anatomical range is the PD controller's job, and tightening a limit to
/// do it means the controller spends its life pressed against a
/// constraint, which is precisely the configuration this module's design
/// notes warn about.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLimits {
    /// Half-angle of the allowed swing cone about the bone's own axis.
    pub swing_half_angle: f32,
    /// Allowed twist about the bone's own axis, as `(min, max)`. Usually
    /// asymmetric — a forearm pronates much further than it supinates.
    pub twist_range: (f32, f32),
}

impl JointLimits {
    /// A joint free to rotate anywhere. What every joint shipped as before
    /// limits existed.
    pub const FREE: Option<Self> = None;

    /// A symmetric cone with a symmetric twist, both in degrees — the
    /// common case, written in the unit anatomy references use.
    pub fn degrees(swing_half_angle: f32, twist: f32) -> Self {
        Self {
            swing_half_angle: swing_half_angle.to_radians(),
            twist_range: (-twist.to_radians(), twist.to_radians()),
        }
    }

    /// A cone with an asymmetric twist range, in degrees.
    pub fn degrees_asymmetric(swing_half_angle: f32, twist_min: f32, twist_max: f32) -> Self {
        Self {
            swing_half_angle: swing_half_angle.to_radians(),
            twist_range: (twist_min.to_radians(), twist_max.to_radians()),
        }
    }
}

/// Anatomical rotation limits per joint.
///
/// Deliberately generous. A real shoulder has a range that depends on
/// elevation, a real knee is a hinge with a few degrees of play, and none
/// of that is expressible as one cone — so each entry is the *loosest* stop
/// that still prevents a visibly impossible pose, not a faithful model of
/// the joint.
///
/// `Hips` is `None`: it is the root, so there is no parent to be limited
/// against.
///
/// # Centred on the bind pose, and checked against every shipped pose
///
/// Each cone is centred on the bind pose (`LocalPose::REST`), so a joint's
/// swing and twist are exactly its pose delta. That makes "the character's
/// own animation fits inside its joints" a property of the data, pinned by
/// `every_pose_the_character_holds_sits_inside_its_joint_limits_on_a_real_rig`
/// over every named pose and the walk and run cycles. Its first run found
/// 49 violations in this table as it then stood — `relaxed_stand`'s neck at
/// 41 degrees against a 35-degree cone, the run's knee at 86 against 80 and
/// its ankle at 54 against 35, the wave's collarbone and forearm twist —
/// every one a joint the controller would have driven into its stop
/// forever. The widened entries below are real anatomical ranges, not
/// numbers tuned to pass.
///
/// The hinges — knees and elbows — are the interesting case. A hinge is a
/// cone of near-zero radius plus a wide twist, but expressing it that way
/// puts the bend on the *twist* axis, which is the bone's own long axis and
/// therefore not where a knee bends at all. So they get a wide swing cone
/// (the bend) and a tight twist (the thing a knee genuinely cannot do), and
/// the one-directional-ness of a real hinge is left to the PD controller
/// and the authored pose rather than faked with a limit that would be on
/// the wrong axis.
pub fn default_joint_limits() -> BoneSet<Option<JointLimits>> {
    BoneSet::from_fn(|bone| match bone {
        // The root has no parent joint to limit.
        Bone::Hips => None,

        // Spine segments each contribute a little; the total across four
        // of them is what makes a torso bend.
        Bone::Spine | Bone::Spine1 | Bone::Spine2 => Some(JointLimits::degrees(30.0, 25.0)),

        // The neck turns much further than it tilts, but a cone cannot say
        // that, so the cone covers the tilt and the twist covers the turn.
        // Cervical flexion reaches ~50 degrees; `relaxed_stand` alone holds
        // the head 41 forward of bind.
        Bone::Neck => Some(JointLimits::degrees(50.0, 60.0)),
        // The head's body hangs from the upper torso across the neck, which
        // has no body of its own — so this joint carries the neck's range
        // and the head's together (50 + 20 of bend, 60 + 20 of turn).
        Bone::Head => Some(JointLimits::degrees(70.0, 80.0)),

        // The shoulder blade slides; it is not really a ball joint at all.
        // Elevation reaches ~45-50 degrees — the wave measures 48, twisted
        // 27.
        Bone::LeftShoulder | Bone::RightShoulder => Some(JointLimits::degrees(50.0, 30.0)),

        // The most mobile joint in the body, and the one where an
        // over-tight limit is most obvious.
        // The twist is the humerus rolling in its socket, ~90 degrees each
        // way. Now that the arm hangs from the upper torso through the
        // collarbone (which has no body), the run's arm swing measures a
        // 77.6-degree roll against the old 70-degree stop.
        //
        // The cone is centred out to the side and a little forward
        // (`ragdoll_plugin::anatomical_cone_centre`): up and down are 90
        // degrees from there, and an arm hanging and swung 60 back (AAOS
        // extension) is 104. 105 reaches all of them and not far behind
        // the back; the run's back swing measures 94.
        Bone::LeftArm | Bone::RightArm => Some(JointLimits::degrees(105.0, 90.0)),

        // Elbow: the bend is the swing cone (see the note above); the
        // asymmetric twist is real forearm pronation/supination, which
        // reaches ~90 degrees — the wave measures +90.7 on the character
        // as drawn. Until 2026-10-01 this was -95..85, fitted to -90.5
        // measured on plain `puppet_base()`, which faces away and reads
        // the twist with its sign flipped: live, the waving forearm held
        // 3.7 degrees short against the stop.
        Bone::LeftForeArm | Bone::RightForeArm => {
            Some(JointLimits::degrees_asymmetric(85.0, -85.0, 95.0))
        }

        Bone::LeftHand | Bone::RightHand => Some(JointLimits::degrees(45.0, 25.0)),

        // Hip: wide forward, much less back. A cone is symmetric, so it is
        // centred 45 degrees forward of straight down
        // (`ragdoll_plugin::anatomical_cone_centre`), and reaches AAOS's 120
        // of flexion and 30 of extension. Centred on the bind it allowed 75
        // each way.
        Bone::LeftUpLeg | Bone::RightUpLeg => Some(JointLimits::degrees(75.0, 40.0)),

        // Knee: swing is the bend, twist is near-zero because a knee that
        // twists is the classic broken-ragdoll tell. Flexion reaches ~135
        // degrees; the run's swing phase alone measures 86.
        Bone::LeftLeg | Bone::RightLeg => Some(JointLimits::degrees(130.0, 8.0)),

        // Ankle. Plantarflexion reaches ~50 degrees; the run's toe-off
        // measures 54 against the bind pose.
        Bone::LeftFoot | Bone::RightFoot => Some(JointLimits::degrees(60.0, 20.0)),

        // Toes barely articulate.
        Bone::LeftToeBase | Bone::RightToeBase => Some(JointLimits::degrees(25.0, 5.0)),
    })
}

/// The torque one joint should apply this step.
///
/// Split out from the ECS so the whole control law stays testable without
/// a physics world — the same discipline the rest of this module follows.
pub fn joint_torque(
    bone: Bone,
    current: Quat,
    target: Quat,
    angular_velocity: Vec3,
    ragdoll: &Ragdoll,
) -> Vec3 {
    joint_torque_at(bone, current, target, angular_velocity, ragdoll, 0.0)
}

/// [`joint_torque`], with the timestep it will be integrated over.
///
/// Prefer this wherever `dt` is known: it lets the controller clamp its
/// damping gain to what that timestep can integrate stably, which on a
/// jointed body is the difference between settling and vibrating. See
/// [`PdParams::stable_damping`].
pub fn joint_torque_at(
    bone: Bone,
    current: Quat,
    target: Quat,
    angular_velocity: Vec3,
    ragdoll: &Ragdoll,
    dt: f32,
) -> Vec3 {
    let params = ragdoll.params[bone];
    let strength = ragdoll.effective_strength(bone);

    // Strength scales the CEILING rather than the torque itself. Scaling
    // the torque would make a half-strength joint track a half-amplitude
    // pose — visibly wrong. Scaling the ceiling instead keeps it tracking
    // correctly until it runs out of strength, which is what "weakened"
    // actually looks like.
    let limited = PdParams { max_torque: strength.scale(params.max_torque), ..params };

    pd_torque_at(current, target, angular_velocity, &limited, dt)
}

/// [`joint_torque_at`] toward a target turning at `target_velocity` (world,
/// rad/s): see [`pd_torque_tracking`] for the lag this removes.
pub fn joint_torque_tracking(
    bone: Bone,
    current: Quat,
    target: Quat,
    angular_velocity: Vec3,
    target_velocity: Vec3,
    ragdoll: &Ragdoll,
    dt: f32,
) -> Vec3 {
    let params = ragdoll.params[bone];
    let strength = ragdoll.effective_strength(bone);
    let limited = PdParams { max_torque: strength.scale(params.max_torque), ..params };
    pd_torque_tracking(current, target, angular_velocity, target_velocity, &limited, dt)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn every_default_joint_is_well_conditioned_for_the_physics_timestep() {
        // A PD controller integrated explicitly is stable only while both
        // gains fit the step: `sqrt(kp) * dt < 2` and `kd * dt < 2`.
        //
        // `PdParams::stable_damping` clamps the second at runtime. Nothing
        // clamps the FIRST — a frequency too high for the timestep cannot
        // be rescued after the fact, it has to be authored lower. So this
        // asserts the shipped table is already inside both bounds, which
        // is the only place the check can live.
        //
        // avian's default physics rate is 64 Hz.
        const DT: f32 = 1.0 / 64.0;
        // Well inside the bound of 2.0, not merely under it: the bound is
        // derived for an isolated body, and a joint solver correcting the
        // same body adds stiffness this analysis does not see.
        const BUDGET: f32 = 1.2;

        for &bone in Bone::ALL.iter() {
            let params = default_joint_params()[bone];

            let stiffness_ratio = params.stiffness().sqrt() * DT;
            assert!(
                stiffness_ratio < BUDGET,
                "{}'s {} Hz is too stiff for a {DT} s step: sqrt(kp)*dt is \
                 {stiffness_ratio:.2}, over the {BUDGET} budget. Lower the frequency \
                 rather than relying on the damping clamp, which cannot fix this.",
                bone.name(),
                params.frequency_hz,
            );

            // The damping gain must fit the step WITHOUT the runtime clamp
            // having to intervene. If it does not, `stable_damping` will
            // quietly reduce it, and the authored `damping_ratio` stops
            // being what the joint actually gets — a table that lies about
            // itself.
            //
            // Budgeted at 1.6, the same 0.8 fraction the clamp uses, so
            // this test failing and the clamp engaging are the same event.
            let damping_ratio = params.damping() * DT;
            assert!(
                damping_ratio < 1.6,
                "{}'s damping gain is {damping_ratio:.2} x dt. `stable_damping` clamps \
                 at 1.6, so this joint would silently get less damping than its \
                 authored ratio of {} asks for — lower the frequency instead",
                bone.name(),
                params.damping_ratio,
            );
        }
    }

    #[test]
    fn a_driven_joint_pulls_toward_its_target() {
        let ragdoll = Ragdoll::default();
        let target = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);

        let torque =
            joint_torque(Bone::LeftArm, Quat::IDENTITY, target, Vec3::ZERO, &ragdoll);

        assert!(torque.length() > 1.0, "a driven joint should pull, got {torque:?}");
        assert!(
            torque.normalize().dot(Vec3::Y) > 0.99,
            "and should pull about the target's own axis, got {torque:?}",
        );
    }

    #[test]
    fn a_limp_joint_applies_no_torque() {
        // The passive end of the dial: a fully limp joint is a plain
        // ragdoll joint, indistinguishable from an unmotorized one.
        let mut ragdoll = Ragdoll::default();
        ragdoll.set_strength(0.0);

        let torque = joint_torque(
            Bone::LeftArm,
            Quat::IDENTITY,
            Quat::from_axis_angle(Vec3::Y, 1.0),
            Vec3::ZERO,
            &ragdoll,
        );

        assert_eq!(torque, Vec3::ZERO, "a limp joint must produce nothing");
    }

    #[test]
    fn strength_scales_the_ceiling_not_the_direction() {
        // A weakened joint still tracks correctly — it just gives out
        // sooner. Scaling the torque itself would make it track a
        // half-amplitude pose, which reads as a different animation rather
        // than a weaker character.
        let mut weak = Ragdoll::default();
        weak.set_strength(0.3);
        let strong = Ragdoll::default();

        let target = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);

        let weak_torque =
            joint_torque(Bone::LeftArm, Quat::IDENTITY, target, Vec3::ZERO, &weak);
        let strong_torque =
            joint_torque(Bone::LeftArm, Quat::IDENTITY, target, Vec3::ZERO, &strong);

        assert!(
            weak_torque.length() < strong_torque.length(),
            "a weakened joint should produce less torque",
        );
        assert!(
            weak_torque.normalize().dot(strong_torque.normalize()) > 0.999,
            "...but pull in exactly the same direction",
        );
    }

    #[test]
    fn strength_is_clamped_to_a_sensible_range() {
        assert_eq!(RagdollStrength(2.0).scale(100.0), 100.0, "above 1 clamps");
        assert_eq!(RagdollStrength(-1.0).scale(100.0), 0.0, "below 0 clamps");
        assert_eq!(RagdollStrength(0.5).scale(100.0), 50.0);
    }

    #[test]
    fn a_joint_never_exceeds_its_own_torque_limit() {
        // The cap is what lets an impact win. If it leaked, the ragdoll
        // would be kinematic in all but name.
        let ragdoll = Ragdoll::default();

        for bone in [Bone::Hips, Bone::LeftArm, Bone::Head, Bone::LeftForeArm] {
            let limit = ragdoll.params[bone].max_torque;

            let torque = joint_torque(
                bone,
                Quat::IDENTITY,
                Quat::from_axis_angle(Vec3::Y, 3.0),
                Vec3::splat(100.0),
                &ragdoll,
            );

            assert!(
                torque.length() <= limit + 1.0e-3,
                "{} produced {} N·m against a {limit} N·m limit",
                bone.name(),
                torque.length(),
            );
        }
    }

    #[test]
    fn heavier_joints_are_given_more_torque_than_lighter_ones() {
        // A hip moves the whole upper body; a forearm moves a hand.
        // Uniform gains leave the heavy joints sagging.
        let params = default_joint_params();

        assert!(
            params[Bone::Hips].max_torque > params[Bone::LeftArm].max_torque,
            "the hip should be stronger than the arm",
        );
        assert!(
            params[Bone::LeftArm].max_torque > params[Bone::LeftForeArm].max_torque,
            "the arm should be stronger than the forearm",
        );
        assert!(
            params[Bone::LeftUpLeg].max_torque > params[Bone::LeftFoot].max_torque,
            "the thigh should be stronger than the foot",
        );
    }

    #[test]
    fn every_joint_defaults_to_critical_damping() {
        // Stability first: a joint that oscillates is far more obvious than
        // one that settles slowly, so nothing ships underdamped by default.
        for (bone, params) in default_joint_params().iter() {
            assert_eq!(
                params.damping_ratio,
                1.0,
                "{} should default to critical damping",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_fresh_ragdoll_is_fully_driven() {
        let ragdoll = Ragdoll::default();
        assert!(ragdoll.is_driven(), "a ragdoll should start tracking its animation");

        let mut limp = Ragdoll::default();
        limp.set_strength(0.0);
        assert!(!limp.is_driven(), "...and report when it has gone fully limp");
    }

    #[test]
    fn one_joint_can_go_limp_while_the_others_keep_driving() {
        // The expressiveness a binary animated/ragdoll flag cannot reach: a
        // shoved arm goes slack while the legs keep walking.
        let mut ragdoll = Ragdoll::default();
        ragdoll.strength[Bone::LeftArm] = RagdollStrength::LIMP;

        let target = Quat::from_axis_angle(Vec3::Y, 1.0);

        assert_eq!(
            joint_torque(Bone::LeftArm, Quat::IDENTITY, target, Vec3::ZERO, &ragdoll),
            Vec3::ZERO,
            "the slack arm should produce nothing",
        );
        assert!(
            joint_torque(Bone::LeftUpLeg, Quat::IDENTITY, target, Vec3::ZERO, &ragdoll)
                .length()
                > 1.0,
            "...while the legs keep driving",
        );
        assert!(ragdoll.is_driven(), "and the ragdoll is still driven overall");
    }

    #[test]
    fn a_joint_at_its_target_is_left_alone() {
        let ragdoll = Ragdoll::default();
        let orientation = Quat::from_axis_angle(Vec3::Z, 0.4);

        let torque =
            joint_torque(Bone::Spine, orientation, orientation, Vec3::ZERO, &ragdoll);

        assert!(torque.length() < 1.0e-5, "got {torque:?}");
    }

    #[test]
    fn a_negated_target_produces_no_torque() {
        // Neighbourhooding, carried through the whole call path rather
        // than only tested at the bottom of it.
        let ragdoll = Ragdoll::default();
        let orientation = Quat::from_axis_angle(Vec3::Y, 0.9);

        let torque =
            joint_torque(Bone::Spine, orientation, -orientation, Vec3::ZERO, &ragdoll);

        assert!(
            torque.length() < 1.0e-3,
            "q and -q are the same orientation, got {torque:?}",
        );
    }

    #[test]
    fn joint_distance_walks_the_skeleton_not_the_air() {
        assert_eq!(joint_distance(Bone::LeftForeArm, Bone::LeftForeArm), 0);
        assert_eq!(joint_distance(Bone::LeftForeArm, Bone::LeftArm), 1);
        assert_eq!(joint_distance(Bone::LeftArm, Bone::LeftForeArm), 1);
        // Forearm -> arm -> shoulder -> Spine2 <- shoulder <- arm <- forearm.
        assert_eq!(joint_distance(Bone::LeftForeArm, Bone::RightForeArm), 6);
        // Symmetric, whichever end the walk starts from.
        for &a in Bone::ALL.iter() {
            for &b in Bone::ALL.iter() {
                assert_eq!(
                    joint_distance(a, b),
                    joint_distance(b, a),
                    "{} to {}",
                    a.name(),
                    b.name(),
                );
            }
        }
    }

    #[test]
    fn a_hit_weakens_the_struck_joint_most_and_distant_ones_not_at_all() {
        let mut ragdoll = Ragdoll::default();
        ragdoll.stun(Bone::LeftForeArm, 1.0);

        let strength = |bone| ragdoll.effective_strength(bone).0;

        assert_eq!(strength(Bone::LeftForeArm), 0.0, "the struck joint goes fully limp");
        assert!(
            (strength(Bone::LeftArm) - 0.5).abs() < 1.0e-6,
            "one joint away loses half as much, got {}",
            strength(Bone::LeftArm),
        );
        assert!(
            strength(Bone::LeftShoulder) > strength(Bone::LeftArm),
            "and the loss keeps shrinking with distance",
        );
        // Eight joints from the left forearm; nothing a leg would show.
        assert_eq!(strength(Bone::RightLeg), 1.0, "a far joint is untouched");
    }

    #[test]
    fn a_stunned_joint_holds_then_recovers_fully_on_schedule() {
        let mut ragdoll = Ragdoll::default();
        let response = ragdoll.stun_response;
        ragdoll.stun(Bone::Spine, 1.0);

        const DT: f32 = 1.0 / 60.0;
        let mut elapsed = 0.0;
        let mut previous = ragdoll.effective_strength(Bone::Spine).0;
        let mut samples = Vec::new();

        while elapsed < response.hold_secs + response.recover_secs + 0.5 {
            ragdoll.recover(DT);
            elapsed += DT;
            let now = ragdoll.effective_strength(Bone::Spine).0;
            assert!(now >= previous, "recovery must never lose strength: {previous} -> {now}");
            previous = now;
            samples.push((elapsed, now));
        }

        // Still fully limp partway through the hold.
        let mid_hold = samples.iter().find(|(t, _)| *t >= response.hold_secs * 0.5).unwrap();
        assert_eq!(mid_hold.1, 0.0, "the joint should still be limp during the hold");

        // Half recovered halfway through the recovery.
        let midpoint = response.hold_secs + response.recover_secs * 0.5;
        let mid_recovery = samples.iter().find(|(t, _)| *t >= midpoint).unwrap();
        assert!(
            (mid_recovery.1 - 0.5).abs() < 0.05,
            "halfway through recovery the joint should be about half strength, got {}",
            mid_recovery.1,
        );

        assert_eq!(previous, 1.0, "the joint should be back to full strength");
        assert!(!ragdoll.is_stunned(), "and nothing should still be recovering");
    }

    #[test]
    fn recovery_returns_to_the_games_dial_not_to_full_strength() {
        // A game that has set a character to half strength must get half
        // strength back after a hit, not a character that got STRONGER for
        // being punched.
        let mut ragdoll = Ragdoll::default();
        ragdoll.set_strength(0.4);
        ragdoll.stun(Bone::Neck, 1.0);
        assert_eq!(ragdoll.effective_strength(Bone::Neck).0, 0.0);

        ragdoll.recover(10.0);

        assert!((ragdoll.effective_strength(Bone::Neck).0 - 0.4).abs() < 1.0e-6);
        assert_eq!(ragdoll.strength[Bone::Neck], RagdollStrength(0.4), "the dial is untouched");
    }

    #[test]
    fn a_light_hit_does_not_cut_short_a_heavy_ones_recovery() {
        let mut ragdoll = Ragdoll::default();
        ragdoll.stun(Bone::LeftArm, 1.0);
        ragdoll.recover(ragdoll.stun_response.hold_secs + 0.1);
        let before = ragdoll.stun[Bone::LeftArm].limpness;

        ragdoll.stun(Bone::LeftArm, 0.2);

        assert_eq!(
            ragdoll.stun[Bone::LeftArm].limpness,
            before,
            "a weaker second hit must not reduce an existing loss",
        );
    }

    #[test]
    fn a_hit_weakens_the_torque_the_joint_can_produce() {
        // The stun has to reach the controller, not only the display: a
        // joint shown limp while its PD still holds full authority would
        // be rendered sagging and physically rigid at once.
        let mut ragdoll = Ragdoll::default();
        let target = Quat::from_axis_angle(Vec3::X, 1.0);
        let torque = |ragdoll: &Ragdoll| {
            joint_torque(Bone::LeftForeArm, Quat::IDENTITY, target, Vec3::ZERO, ragdoll).length()
        };

        let before = torque(&ragdoll);
        ragdoll.stun(Bone::LeftForeArm, 1.0);
        assert_eq!(torque(&ragdoll), 0.0, "a fully stunned joint exerts nothing");

        ragdoll.recover(100.0);
        assert!((torque(&ragdoll) - before).abs() < 1.0e-4, "and all of it comes back");
    }

    #[test]
    fn a_zero_hit_changes_nothing() {
        let mut ragdoll = Ragdoll::default();
        ragdoll.stun(Bone::Head, 0.0);
        assert!(!ragdoll.is_stunned());
    }
}
