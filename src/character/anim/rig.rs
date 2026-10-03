//! [`BoneSet`] — per-bone state as a flat `[T; 22]`, and the ordering
//! invariants the whole animation stack relies on.
//!
//! # Why an array rather than a `HashMap<Bone, T>`
//!
//! The superseded `muscle` module keys per-bone state by `Bone` in hash
//! maps, which costs a hash on every access — several times per bone per
//! frame — and forces it to rebuild four `Vec`s per character per frame
//! just to hand the solver contiguous slices.
//!
//! A 22-element array removes both costs, but the real argument is not
//! micro-optimization. Every stage here needs *whole-skeleton* access in a
//! single pass: forward kinematics accumulates parent-before-child, IK
//! spans a four-bone chain, phase layering writes across the spine. With a
//! flat array indexed by [`Bone::index`], all of that is straightforward
//! indexing into contiguous memory, and the entire solve stays a **pure
//! function over plain arrays** — callable from a unit test in
//! microseconds with no `World`, which is what makes the verification
//! discipline in this module's plan affordable.
//!
//! At 22 bones a `[Quat; 22]` is 352 bytes: one or two cache lines of real
//! work, fully prefetched.
//!
//! # The ordering invariant
//!
//! [`Bone::ALL`] is ordered so that **every bone appears after its own
//! parent**. Forward kinematics, rotation accumulation, and the rigid-chain
//! corrections all depend on it: each can walk `0..22` once, reading its
//! parent's already-final value, with no recursion, no sorting, and no
//! second pass.
//!
//! It is load-bearing enough to be pinned by a test
//! (`the_all_array_is_ordered_parent_before_child`) rather than left as a
//! convention — reordering `Bone::ALL` would otherwise silently produce
//! subtly wrong poses rather than an error.

use bevy::math::{Quat, Vec3};

use crate::character::skeleton::Bone;

/// How many bones the rig has. Every [`BoneSet`] is exactly this long.
pub const BONE_COUNT: usize = 22;

/// Per-bone state, stored flat and indexed by [`Bone::index`].
///
/// Indexable by `Bone` directly (`set[Bone::Head]`), so call sites read
/// like a map while costing an array index.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoneSet<T>(pub [T; BONE_COUNT]);

impl<T: Copy> BoneSet<T> {
    /// Every bone set to the same value.
    #[inline]
    pub const fn splat(value: T) -> Self {
        Self([value; BONE_COUNT])
    }
}

impl<T> BoneSet<T> {
    /// Builds a set by calling `f` for each bone, in [`Bone::ALL`] order.
    #[inline]
    pub fn from_fn(mut f: impl FnMut(Bone) -> T) -> Self {
        Self(std::array::from_fn(|i| f(Bone::ALL[i])))
    }

    /// Iterates `(bone, &value)` in [`Bone::ALL`] order — i.e. always
    /// parent before child.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = (Bone, &T)> {
        Bone::ALL.iter().copied().zip(self.0.iter())
    }

    /// Iterates `(bone, &mut value)` in [`Bone::ALL`] order.
    #[inline]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (Bone, &mut T)> {
        Bone::ALL.iter().copied().zip(self.0.iter_mut())
    }
}

impl<T> std::ops::Index<Bone> for BoneSet<T> {
    type Output = T;

    #[inline]
    fn index(&self, bone: Bone) -> &T {
        &self.0[bone.index()]
    }
}

impl<T> std::ops::IndexMut<Bone> for BoneSet<T> {
    #[inline]
    fn index_mut(&mut self, bone: Bone) -> &mut T {
        &mut self.0[bone.index()]
    }
}

impl<T: Default + Copy> Default for BoneSet<T> {
    fn default() -> Self {
        Self::splat(T::default())
    }
}

/// A complete pose: one local rotation per bone, plus the root's position.
///
/// The rotations are **deltas from the rest pose**, expressed in each
/// bone's own parent frame, and are deliberately **rig-independent** —
/// `Quat::IDENTITY` everywhere means "exactly the bind pose", whatever rig
/// that pose is later applied to. Composing onto a specific rig's own rest
/// rotations happens once, at write-back.
///
/// That independence is what lets one authored pose drive our synthetic
/// T-pose rig, a Quaternius mesh, and a Mixamo mesh without reauthoring.
///
/// Because a pose carries only rotations, **it cannot change a bone's
/// length**. The stretched-bone class of bug that the superseded
/// position-space module needed a dedicated invariant test to catch is
/// structurally impossible here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalPose {
    /// Per-bone local rotation delta from rest.
    pub rotations: BoneSet<Quat>,
    /// Root (`Hips`) translation, in the rig's own root space.
    pub root_translation: Vec3,
}

impl LocalPose {
    /// The rest pose: every bone exactly at its bind orientation.
    pub const REST: Self =
        Self { rotations: BoneSet::splat(Quat::IDENTITY), root_translation: Vec3::ZERO };

    /// A pose with the given root translation and no rotation deltas.
    #[inline]
    pub const fn at_root(root_translation: Vec3) -> Self {
        Self { rotations: BoneSet::splat(Quat::IDENTITY), root_translation }
    }

    /// The local rotation delta for one bone.
    #[inline]
    pub fn rotation(&self, bone: Bone) -> Quat {
        self.rotations[bone]
    }

    /// Sets one bone's local rotation delta.
    #[inline]
    pub fn set_rotation(&mut self, bone: Bone, rotation: Quat) {
        self.rotations[bone] = rotation;
    }
}

impl Default for LocalPose {
    fn default() -> Self {
        Self::REST
    }
}

/// Accumulates each bone's rest-relative rotation down the hierarchy.
///
/// Returns, per bone, the product of every rest-relative delta from the
/// root down to and including that bone — i.e. its orientation relative to
/// where the rest pose would have put it.
///
/// A single forward pass over `0..22` suffices because [`Bone::ALL`] is
/// ordered parent-before-child; see the module doc.
///
/// This is deliberately **not** a world-space transform: it knows nothing
/// about any particular rig's bind rotations. Those enter only at
/// write-back.
pub fn accumulate_rest_relative_rotations(pose: &LocalPose) -> BoneSet<Quat> {
    let mut accumulated = BoneSet::splat(Quat::IDENTITY);

    for &bone in Bone::ALL.iter() {
        accumulated[bone] = match bone.parent() {
            Some(parent) => accumulated[parent] * pose.rotations[bone],
            None => pose.rotations[bone],
        };
    }

    accumulated
}

/// Each bone's world position under `pose`, using this crate's own
/// synthetic T-pose proportions.
///
/// Rest-relative rotations are accumulated down the chain and applied to
/// each bone's rest offset, so a bone's distance from its parent is always
/// exactly `t_pose_offset().length()` — bone lengths are preserved by
/// construction, not by correction.
///
/// Intended for tests, gizmos, and the IK stage's own reasoning about
/// reach. Rendering goes through Bevy's transform propagation instead.
pub fn forward_kinematics(pose: &LocalPose) -> BoneSet<Vec3> {
    let accumulated = accumulate_rest_relative_rotations(pose);
    let mut positions = BoneSet::splat(Vec3::ZERO);

    for &bone in Bone::ALL.iter() {
        positions[bone] = match bone.parent() {
            Some(parent) => {
                // The rest offset, carried through the PARENT's accumulated
                // rotation: a bone's own rotation orients its children, not
                // itself.
                positions[parent] + accumulated[parent] * bone.t_pose_offset()
            }
            None => pose.root_translation + bone.t_pose_offset(),
        };
    }

    positions
}

/// A rig's own bone offsets and bind rotations — the geometry forward
/// kinematics needs in order to describe a *particular* skeleton.
///
/// # Why this is not just `Bone::t_pose_offset`
///
/// [`forward_kinematics`] uses this crate's synthetic T-pose table, which
/// is fine for anything that only cares about relationships *within* the
/// pose (bone lengths, symmetry, which way a limb points). Poses are
/// rig-independent, so that proxy is exactly right for them.
///
/// It is wrong the moment a computation involves the **world**. Ground
/// height is the motivating case: the synthetic rig puts a toe at
/// `z = -0.168` while `puppet_base.gltf` puts it near `z = 0.088`. On flat
/// ground the surface is the same at both, so the discrepancy cancels and
/// nothing is visibly wrong. On a slope the surface depends on `z`, the
/// solver lifts the foot by a height the real rig never needed, and — with
/// the hip fixed — the only way to reach it is to swing the whole leg
/// forward. Measured: a 72-degree swing, with every unit test passing.
///
/// So anything solving against the world has to know the real rig.
#[derive(Debug, Clone)]
pub struct RigGeometry {
    /// Each bone's translation from its parent, in the parent's frame.
    pub offsets: BoneSet<Vec3>,
    /// Each bone's bind rotation, in its parent's frame.
    pub bind_rotations: BoneSet<Quat>,
    /// The rotation of whatever the root hangs beneath.
    pub root_rotation: Quat,
    /// Where each toe's TIP sits relative to the toe joint, in the toe's own
    /// frame. See [`Self::toe_end_offset`].
    ///
    /// Indexed by bone for uniformity, but only the two `ToeBase` entries are
    /// ever read.
    pub toe_end_offsets: BoneSet<Vec3>,
}

impl Default for RigGeometry {
    /// This crate's own synthetic T-pose: the offsets from
    /// [`Bone::t_pose_offset`], with identity bind rotations.
    fn default() -> Self {
        let offsets = BoneSet::from_fn(|bone| bone.t_pose_offset());
        Self {
            toe_end_offsets: default_toe_end_offsets(&offsets),
            offsets,
            bind_rotations: BoneSet::splat(Quat::IDENTITY),
            root_rotation: Quat::IDENTITY,
        }
    }
}

/// How long a toe tip is, as a fraction of the toe bone itself, when the rig
/// does not supply a real one.
///
/// Measured from `puppet_base.gltf`, which *does* have the extra joint:
/// `ball_l` is 0.1591 m from the foot and `ball_leaf_l` a further 0.0789 m
/// beyond it — a ratio of 0.496. Rounded to a half, because the third digit
/// of one rig's proportions is not evidence about any other rig.
///
/// A fraction rather than an absolute length so a child or a giant gets a
/// proportionate tip instead of an adult-sized one.
pub const TOE_END_FRACTION: f32 = 0.5;

/// Estimates each toe's tip offset from the toe bone's own direction.
///
/// The fallback for a rig with no toe-end joint — which includes this crate's
/// synthetic one, and is the common case: of the three rigs in this repo only
/// `puppet_base.gltf` has the extra node. The tip continues straight along
/// the toe, which is what a toe does.
///
/// # The frame conversion is not optional
///
/// A bone's `offsets` entry lives in its **parent's** frame, while a tip
/// offset must live in the **toe's own** frame — that is the frame the toe's
/// accumulated rotation is applied to. On this crate's synthetic rig every
/// bind rotation is identity so the two coincide, and simply scaling the
/// parent-frame offset looks perfectly correct.
///
/// It is wrong on a real rig. `puppet_base.gltf` binds `ball_l` at
/// `[0, 0.973, -0.230, 0]` — about 180 degrees — so reusing the parent-frame
/// offset there points the tip backward, *into* the heel. Undoing the toe's
/// bind rotation is what makes the estimate mean the same thing on both.
fn toe_end_offsets_for(offsets: &BoneSet<Vec3>, bind_rotations: &BoneSet<Quat>) -> BoneSet<Vec3> {
    BoneSet::from_fn(|bone| match bone {
        Bone::LeftToeBase | Bone::RightToeBase => {
            // `offsets[toe]` points from the ankle to the toe joint, in the
            // ankle's frame; continuing along it is the direction the toe
            // already runs. Carried into the toe's own frame by the inverse of
            // the toe's bind rotation.
            bind_rotations[bone].inverse() * (offsets[bone] * TOE_END_FRACTION)
        }
        _ => Vec3::ZERO,
    })
}

/// The synthetic rig's tip offsets — identity bind rotations throughout.
fn default_toe_end_offsets(offsets: &BoneSet<Vec3>) -> BoneSet<Vec3> {
    toe_end_offsets_for(offsets, &BoneSet::splat(Quat::IDENTITY))
}

impl RigGeometry {
    /// Reads the geometry of a concrete skeleton.
    ///
    /// `offsets` come from the live bone entities' rest translations, and
    /// the bind rotations from the skeleton's own captured bind pose.
    pub fn from_skeleton(
        skeleton: &crate::character::skeleton::HumanoidSkeleton,
        offsets: BoneSet<Vec3>,
    ) -> Self {
        let bind_rotations = BoneSet::from_fn(|bone| skeleton.rest_rotation(bone));
        Self {
            toe_end_offsets: toe_end_offsets_for(&offsets, &bind_rotations),
            offsets,
            bind_rotations,
            root_rotation: skeleton.hips_root_rotation(),
        }
    }

    /// Overrides one toe's tip offset with a measured one, in the toe's own
    /// frame.
    ///
    /// Use this when the rig genuinely has a toe-end joint —
    /// `puppet_base.gltf`'s `ball_leaf_l`/`ball_leaf_r`, say — so the tip is
    /// read from the artist's own skeleton rather than estimated from
    /// [`TOE_END_FRACTION`].
    pub fn with_toe_end(mut self, toe: Bone, offset: Vec3) -> Self {
        self.toe_end_offsets[toe] = offset;
        self
    }

    /// Where a toe's tip sits relative to the toe joint, in the toe's frame.
    pub fn toe_end_offset(&self, toe: Bone) -> Vec3 {
        self.toe_end_offsets[toe]
    }

    /// Which way this rig faces, in its own bind pose — the ONE source of
    /// truth for direction on this rig.
    ///
    /// Measured from the left ankle to its toe, which is the bone pair that
    /// points along the facing by construction on any humanoid rig. Nothing
    /// in this crate should hardcode a facing: rigs disagree, and every
    /// place that assumed one has been a bug.
    ///
    /// ```text
    ///   synthetic T-pose   ankle -> toe  (0, -0.141, -0.990)   => -Z
    ///   puppet_base.gltf   ankle -> toe  (0, -0.448, +0.894)   => +Z
    /// ```
    ///
    /// The two face OPPOSITE ways, and `puppet_base` is not mirrored — it
    /// is upright and correctly handed (left hip at `x = +0.114`, right at
    /// `-0.114`, head at `y = +1.600`). A rig simply faces whichever way it
    /// was exported.
    ///
    /// Returns a horizontal unit vector: the ankle-to-toe direction has a
    /// downward component (the toe angles toward the floor) which is not
    /// part of the facing. Zero if the rig has no usable toe to measure.
    pub fn forward(&self) -> Vec3 {
        // The bind product down the one chain to the foot, root first — the
        // same value `accumulate_bind_rotations` gives the foot, without the
        // rest of the skeleton (120 ns a call that way, and the walk's
        // pelvis asks several times a frame).
        let mut chain = [Bone::Hips; 8];
        let mut length = 0;
        let mut walker = Some(Bone::LeftFoot);
        while let Some(current) = walker {
            chain[length] = current;
            length += 1;
            walker = current.parent();
        }
        let foot = chain[..length].iter().rev().fold(self.root_rotation, |bind, &link| bind * self.bind_rotations[link]);
        let toe = foot * self.offsets[Bone::LeftToeBase];
        Vec3::new(toe.x, 0.0, toe.z).normalize_or_zero()
    }

    /// This rig's own left, derived from [`Self::forward`].
    ///
    /// `up x forward` in Bevy's right-handed, Y-up world. Derived rather
    /// than stored so it cannot disagree with the facing.
    pub fn left(&self) -> Vec3 {
        Vec3::Y.cross(self.forward()).normalize_or_zero()
    }

    /// Where the knee must sit for the leg to bend like a human's: forward
    /// of the straight line from hip socket to ankle.
    ///
    /// Positive is anatomically correct, negative is a backward-bending
    /// knee — a bird's or a grasshopper's. Returns `0.0` for a straight leg
    /// and for a rig this cannot measure.
    ///
    /// # Why this exists as a named measurement
    ///
    /// Because the obvious check does not work. The angle between thigh and
    /// shin is **identical whichever way the knee folds**, so an unsigned
    /// measurement cannot tell a human leg from an insect's — and this
    /// crate had thirty-odd leg tests that all measured exactly that. They
    /// passed while the rendered character walked on backward knees, which
    /// was reported from a screenshot, twice, before any test noticed.
    ///
    /// Measured with that bug live on `puppet_base`: `LocalPose::REST` came
    /// out at `+0.0169` (correct) while `stance(REST)` — whose entire job is
    /// bending the knee — came out at `-0.0157`, and the walk cycle
    /// amplified it to `-0.19`.
    pub fn knee_forward_offset(&self, pose: &LocalPose, side: Side) -> f32 {
        let (socket, knee, ankle) = match side {
            Side::Left => (Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
            Side::Right => (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
        };

        let positions = forward_kinematics_on(pose, self);
        let midpoint = (positions[socket] + positions[ankle]) * 0.5;

        (positions[knee] - midpoint).dot(self.forward())
    }

    /// Which way the shin folds relative to the thigh — **negative is a
    /// human knee**, positive is a bird's.
    ///
    /// The companion to [`Self::knee_forward_offset`], and the one to trust
    /// on a nearly straight leg.
    ///
    /// # Why two measurements
    ///
    /// `knee_forward_offset` asks where the knee sits relative to the
    /// hip-to-ankle line, which is the right question when the leg has a
    /// real bend and reads directly as the shape a person sees. Near full
    /// extension it stops being reliable: the knee is on that line by
    /// definition, so the small residual is dominated by the hip's lateral
    /// placement rather than by the bend.
    ///
    /// Measured on the synthetic rig at 99.2% extension, late stance, the
    /// offset reads `-0.017` — apparently backward — while this measure
    /// reads `-0.34`, solidly human, at the same phases. The knee was
    /// always bending correctly; the offset was the wrong instrument for a
    /// straight leg.
    ///
    /// This one compares the two segment DIRECTIONS, so it stays meaningful
    /// however extended the leg is — it only degenerates when a segment has
    /// no length at all.
    pub fn knee_fold_direction(&self, pose: &LocalPose, side: Side) -> f32 {
        let (socket, knee, ankle) = match side {
            Side::Left => (Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
            Side::Right => (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
        };

        let positions = forward_kinematics_on(pose, self);
        let thigh = (positions[knee] - positions[socket]).normalize_or_zero();
        let shin = (positions[ankle] - positions[knee]).normalize_or_zero();

        if thigh == Vec3::ZERO || shin == Vec3::ZERO {
            return 0.0;
        }

        (shin - thigh).dot(self.forward())
    }
}

/// Which of a pair of limbs, for the rig-relative measurements above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// The character's own left.
    Left,
    /// The character's own right.
    Right,
}

/// How straight a leg is, as a fraction of its own full extension.
///
/// `1.0` is dead straight — the reach singularity, where the knee has no
/// meaningful bend direction. Below about `0.98` the leg has real slack and
/// its bend direction is a genuine property rather than numerical noise.
///
/// Exists so a direction assertion can scale its tolerance with how much
/// bend there is to have a direction at all, instead of picking a constant
/// that is either too loose near the singularity or too tight away from it.
pub fn leg_extension_fraction(pose: &LocalPose, rig: &RigGeometry, side: Side) -> f32 {
    let (socket, shin, ankle) = match side {
        Side::Left => (Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
        Side::Right => (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
    };

    // A bone's offset is measured from its PARENT, so the femur is
    // `offsets[shin]` and the shin is `offsets[ankle]` — the
    // LeftUpLeg-is-the-knee trap this rig's naming sets.
    let straight = rig.offsets[shin].length() + rig.offsets[ankle].length();
    if straight <= 1.0e-6 {
        return 1.0;
    }

    let positions = forward_kinematics_on(pose, rig);
    (positions[ankle] - positions[socket]).length() / straight
}

/// Where each toe's TIP sits in world space under `pose`, on a specific rig.
///
/// The tip is not a bone (see [`RigGeometry::toe_end_offsets`] for why it is
/// deliberately not a 23rd one), so it is not in the [`BoneSet`] that
/// [`forward_kinematics_on`] returns and has to be derived separately.
///
/// Returns `(left_tip, right_tip)`.
pub fn toe_end_positions(pose: &LocalPose, rig: &RigGeometry) -> (Vec3, Vec3) {
    let accumulated = accumulate_world_rotations(pose, rig);
    let positions = forward_kinematics_on(pose, rig);

    let tip = |toe: Bone| positions[toe] + accumulated[toe] * rig.toe_end_offsets[toe];

    (tip(Bone::LeftToeBase), tip(Bone::RightToeBase))
}

/// Each bone's full world rotation under `pose`, on a specific rig.
///
/// The rotation half of [`forward_kinematics_on`], which computes this
/// internally and then discards it. Exposed because anything positioning a
/// point in a bone's own frame — the toe tip, a held weapon, a foot's contact
/// plane — needs the bone's orientation, not just its position.
/// # The delta is applied about a WORLD axis, not a local one
///
/// A pose's rotations are authored against this crate's synthetic T-pose,
/// whose bind rotations are all identity — so "turn 40 degrees about +Y"
/// means the *world* +Y there, because local and world axes coincide. That
/// is the authoring contract, and a real rig has to reproduce it.
///
/// Composing `parent * bind * delta` does NOT: it applies the delta in the
/// bone's own local frame, which on `puppet_base.gltf` is rotated far from
/// world axes. Same angle, wrong axis. The delta has to be conjugated into
/// the bone's frame first, exactly as
/// [`retarget::write_pose_to_skeleton`](super::retarget::write_pose_to_skeleton)
/// does via `delta_in_bone_frame` — which makes the composition
/// `delta * accumulated_bind`, i.e. a world-axis turn applied on top of the
/// bone's bind orientation.
///
/// # How this was measured
///
/// This function shipped with the local-frame composition, and a comment in
/// [`forward_kinematics_on`] asserted it matched the retargeting path. It did
/// not. The discriminator is the residual `intent⁻¹ * actual`, where *intent*
/// is the synthetic rig's own answer (unambiguous — every bind is identity):
/// if a path honours the contract, that residual is a fixed per-bone
/// constant, independent of the pose. Run two different poses and compare:
///
/// ```text
///                  retarget      this function, BEFORE the fix
///     LeftArm      0.000 deg      32.816 deg
///     LeftForeArm  0.000 deg      39.011 deg
///     LeftLeg      0.000 deg      44.400 deg
/// ```
///
/// Pinned by `the_world_rotations_agree_with_what_retargeting_actually_writes`.
///
/// **Why the legs looked fine anyway.** Conjugation is a no-op when the
/// delta's axis is parallel to the bind's, because parallel rotations
/// commute — and the leg chain is bound almost entirely about X while every
/// leg delta the walk cycle produces (hip pitch, knee bend) is *also* about
/// X. Give `LeftLeg` a delta about Y instead and the same 34-degree error
/// appears. The legs were never immune; they were only ever asked the one
/// question the bug cannot get wrong.
pub fn accumulate_world_rotations(pose: &LocalPose, rig: &RigGeometry) -> BoneSet<Quat> {
    let bind = accumulate_bind_rotations(rig);
    let mut accumulated = BoneSet::splat(Quat::IDENTITY);

    for &bone in Bone::ALL.iter() {
        let parent_rotation = match bone.parent() {
            Some(parent) => accumulated[parent],
            None => rig.root_rotation,
        };

        accumulated[bone] = compose_world_rotation(parent_rotation, bone, pose, rig, bind[bone]);
    }

    accumulated
}

/// The pose delta that puts `bone` at `world`, given its parent's world
/// rotation — the exact inverse of [`accumulate_world_rotations`]'
/// composition:
///
/// ```text
///   world = parent_world * bind_local * (B⁻¹ * delta * B)
///   delta = B * ((parent_world * bind_local)⁻¹ * world) * B⁻¹
/// ```
///
/// where `B` is the bone's accumulated bind rotation
/// ([`accumulate_bind_rotations`]): a pose delta names a WORLD axis, and
/// `B` is what converts it into the bone's frame.
pub fn delta_from_world(bone: Bone, parent_world: Quat, world: Quat, rig: &RigGeometry, accumulated_bind: &BoneSet<Quat>) -> Quat {
    let local = (parent_world * rig.bind_rotations[bone]).inverse() * world;
    let bind = accumulated_bind[bone];
    bind * local * bind.inverse()
}

/// `from` blended `t` of the way to `to` per bone in the WORLD, then turned
/// back into local rotations: each segment takes the shortest way from
/// where it is to where `to` has it. The root translation is blended
/// linearly.
///
/// Blended locally instead, a limb rides its parents' swing as well as its
/// own: getting up, sitting up flung an arm out sideways, palm up, on its
/// way to the floor.
pub fn blend_in_world(from: &LocalPose, to: &LocalPose, t: f32, rig: &RigGeometry) -> LocalPose {
    use super::math::quat_ext::neighborhood;
    let bind = accumulate_bind_rotations(rig);
    let (a, b) = (accumulate_world_rotations(from, rig), accumulate_world_rotations(to, rig));
    let mut world = BoneSet::splat(Quat::IDENTITY);
    let mut out = *from;
    for &bone in Bone::ALL.iter() {
        world[bone] = a[bone].slerp(neighborhood(a[bone], b[bone]), t).normalize();
        let parent_world = bone.parent().map_or(rig.root_rotation, |parent| world[parent]);
        let local = delta_from_world(bone, parent_world, world[bone], rig, &bind);
        out.rotations[bone] = neighborhood(from.rotations[bone], local).normalize();
    }
    out.root_translation = from.root_translation.lerp(to.root_translation, t);
    out
}

/// One bone's world rotation from its parent's — THE composition, the only
/// copy of it. [`accumulate_world_rotations`] applies it to every bone and
/// [`offset_from`] to one chain; two copies of this drifted apart once (see
/// [`accumulate_world_rotations`]), so neither keeps its own.
fn compose_world_rotation(
    parent_rotation: Quat,
    bone: Bone,
    pose: &LocalPose,
    rig: &RigGeometry,
    accumulated_bind: Quat,
) -> Quat {
    let delta = pose.rotations[bone];
    let local_delta = if delta == Quat::IDENTITY {
        // Identity in any frame is identity. Short-circuited so the rest
        // pose is reproduced bit-for-bit, matching `delta_in_bone_frame`.
        delta
    } else {
        accumulated_bind.inverse() * delta * accumulated_bind
    };

    parent_rotation * rig.bind_rotations[bone] * local_delta
}

/// `bone`'s pose delta after turning the bone by `turn` in WORLD space, as
/// the pose currently stands.
///
/// # Why composing the turn onto the delta is not enough
///
/// A delta names a world axis relative to the bone's BIND frame and is
/// carried through every parent's delta above it:
/// `world = W(parent)·bind·(B⁻¹·δ·B)`. Multiplying a turn onto `δ` applies
/// it in the frame before the bone's own delta — fine for a leg, whose
/// deltas are small, and wrong for an arm the relaxed stance has already
/// rotated ~70 degrees down from its T-pose: the walk's arm swing, composed
/// that way, was carried round by the hang and swung the hands 37 mm and
/// 3 mm where a quarter of a metre was asked for.
///
/// Solving `W'(bone) = turn · W(bone)` for the delta gives
/// `δ' = (M⁻¹·turn·M)·δ` with `M = W(parent)·bind·B⁻¹` — the frame the
/// delta is really expressed in.
pub fn delta_after_world_turn(pose: &LocalPose, rig: &RigGeometry, bone: Bone, turn: Quat) -> Quat {
    // The chain down to the bone's parent, sharing the one composition.
    let mut chain = [Bone::Hips; 8];
    let mut length = 0;
    let mut walker = bone.parent();
    while let Some(current) = walker {
        chain[length] = current;
        length += 1;
        walker = current.parent();
    }
    chain[..length].reverse();

    let (mut parent_world, mut bind) = (rig.root_rotation, rig.root_rotation);
    for &link in &chain[..length] {
        bind *= rig.bind_rotations[link];
        parent_world = compose_world_rotation(parent_world, link, pose, rig, bind);
    }
    bind *= rig.bind_rotations[bone];

    let frame = parent_world * rig.bind_rotations[bone] * bind.inverse();
    (frame.inverse() * turn * frame) * pose.rotations[bone]
}

/// Where `bone`'s joint sits relative to `ancestor`'s under `pose`: the
/// difference [`forward_kinematics_on`] would give, computed along the one
/// chain from the root to `bone` instead of over the whole skeleton.
///
/// For a solver that needs one limb's position many times a frame. The gait
/// solves each stance thigh by iteration, and doing that on full-skeleton
/// forward kinematics — 22 bones and the bind accumulation, to learn where
/// one ankle is — took the gait from 27 to 70 µs per character per frame.
///
/// `ancestor` must lie on the chain from the root to `bone`; otherwise the
/// offset is measured from the root.
pub fn offset_from(pose: &LocalPose, rig: &RigGeometry, ancestor: Bone, bone: Bone) -> Vec3 {
    frame_from(pose, rig, ancestor, bone).0
}

/// [`offset_from`], plus `bone`'s world rotation — everything needed to
/// place a point fixed in the bone's own frame (a heel, the ball of a foot)
/// without whole-skeleton forward kinematics.
pub fn frame_from(pose: &LocalPose, rig: &RigGeometry, ancestor: Bone, bone: Bone) -> (Vec3, Quat) {
    // The chain, root first. Eight covers the deepest bone in the rig (a
    // hand: hips, three spine, shoulder, arm, forearm, hand).
    let mut chain = [Bone::Hips; 8];
    let mut length = 0;
    let mut walker = Some(bone);
    while let Some(current) = walker {
        chain[length] = current;
        length += 1;
        walker = current.parent();
    }
    chain[..length].reverse();

    let (mut rotation, mut bind) = (rig.root_rotation, rig.root_rotation);
    let mut position = pose.root_translation;
    let mut from = pose.root_translation;

    for &link in &chain[..length] {
        position += rotation * rig.offsets[link];
        bind *= rig.bind_rotations[link];
        rotation = compose_world_rotation(rotation, link, pose, rig, bind);
        if link == ancestor {
            from = position;
        }
    }

    (position - from, rotation)
}

/// Each bone's accumulated bind rotation — the product of every bind rotation
/// from the root down to and including that bone, seeded with the rig's own
/// root rotation.
///
/// This is the bone's world orientation in the rig's bind pose, and it is the
/// frame an authored world-axis delta is conjugated into; see
/// [`accumulate_world_rotations`]. It depends on the rig alone, never on the
/// pose — which is what lets an absolute world-space orientation be converted
/// into a delta by dividing it out.
///
/// The counterpart of `retarget::accumulated_bind_rotations`, which computes
/// the same product from a [`HumanoidSkeleton`](crate::character::skeleton::HumanoidSkeleton)'s
/// `rest_rotation`s rather than from a [`RigGeometry`]. The two agreeing is
/// what makes a solved pose render where it was solved to;
/// `retarget::tests::the_world_rotations_agree_with_what_retargeting_actually_writes`
/// pins it.
pub fn accumulate_bind_rotations(rig: &RigGeometry) -> BoneSet<Quat> {
    let mut bind = BoneSet::splat(Quat::IDENTITY);

    for &bone in Bone::ALL.iter() {
        let parent = match bone.parent() {
            Some(parent) => bind[parent],
            None => rig.root_rotation,
        };

        bind[bone] = parent * rig.bind_rotations[bone];
    }

    bind
}

/// Each bone's world position under `pose`, on a specific rig.
///
/// The rig-aware counterpart of [`forward_kinematics`]: same single
/// parent-before-child pass, but using the real offsets and bind rotations
/// rather than the synthetic T-pose. Use this whenever the result is
/// compared against anything in the world — see [`RigGeometry`] for why
/// the distinction matters.
pub fn forward_kinematics_on(pose: &LocalPose, rig: &RigGeometry) -> BoneSet<Vec3> {
    // The rotation accumulation is shared with `accumulate_world_rotations`
    // rather than repeated here. That sharing is load-bearing: a bone's world
    // rotation has to be the one `retarget::write_pose_to_skeleton` actually
    // writes, or a solver hits a target that the renderer then puts somewhere
    // else. Two copies of the composition DID drift apart once — see
    // `accumulate_world_rotations` for the measurement.
    let accumulated = accumulate_world_rotations(pose, rig);
    let mut positions = BoneSet::splat(Vec3::ZERO);

    for &bone in Bone::ALL.iter() {
        let (parent_position, parent_rotation) = match bone.parent() {
            Some(parent) => (positions[parent], accumulated[parent]),
            None => (pose.root_translation, rig.root_rotation),
        };

        positions[bone] = parent_position + parent_rotation * rig.offsets[bone];
    }

    positions
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn a_world_turn_turns_the_bone_in_the_world() {
        // On the real rig, on an arm already hanging from a posed spine and
        // shoulder — where composing the turn onto the delta goes wrong.
        let rig = crate::character::anim::gltf_rig::puppet_base();
        let pose = crate::character::anim::poses::relaxed_stand();
        let turn = Quat::from_axis_angle(Vec3::new(0.3, 0.2, 0.9).normalize(), 0.4);

        let before = accumulate_world_rotations(&pose, &rig);
        let mut turned = pose;
        turned.rotations[Bone::LeftArm] = delta_after_world_turn(&pose, &rig, Bone::LeftArm, turn);
        let after = accumulate_world_rotations(&turned, &rig);

        let expected = turn * before[Bone::LeftArm];
        assert!(
            after[Bone::LeftArm].angle_between(expected) < 1.0e-3,
            "the arm should turn by exactly `turn` in the world, {:.3} rad off",
            after[Bone::LeftArm].angle_between(expected),
        );
    }

    #[test]
    fn a_single_chain_agrees_with_whole_skeleton_forward_kinematics() {
        // `offset_from` shares the composition with the full pass but walks
        // its own chain, so the two must agree exactly — on the real rig,
        // whose binds are not identity, with deltas about several axes.
        let rig = crate::character::anim::gltf_rig::puppet_base();
        let mut pose = LocalPose::REST;
        pose.root_translation = Vec3::new(0.1, 0.9, -0.2);
        pose.set_rotation(Bone::Hips, Quat::from_euler(bevy::math::EulerRot::XYZ, 0.1, 0.2, -0.05));
        pose.set_rotation(Bone::Spine1, Quat::from_axis_angle(Vec3::Y, 0.25));
        pose.set_rotation(Bone::LeftUpLeg, Quat::from_axis_angle(Vec3::X, -0.4));
        pose.set_rotation(Bone::LeftLeg, Quat::from_axis_angle(Vec3::X, 0.6));
        pose.set_rotation(Bone::LeftFoot, Quat::from_axis_angle(Vec3::Z, 0.2));
        pose.set_rotation(Bone::RightForeArm, Quat::from_axis_angle(Vec3::Y, 0.7));

        let full = forward_kinematics_on(&pose, &rig);
        for (ancestor, bone) in [
            (Bone::Hips, Bone::LeftFoot),
            (Bone::Hips, Bone::RightHand),
            (Bone::Spine1, Bone::Head),
            (Bone::LeftUpLeg, Bone::LeftToeBase),
        ] {
            let chain = offset_from(&pose, &rig, ancestor, bone);
            let expected = full[bone] - full[ancestor];
            assert!(
                chain.distance(expected) < 1.0e-5,
                "{} from {}: chain {chain:?} against full {expected:?}",
                bone.name(),
                ancestor.name(),
            );
        }
    }

    #[test]
    fn the_default_rig_geometry_reproduces_the_synthetic_forward_kinematics() {
        // The anchor between the two implementations: on this crate's own
        // T-pose they must agree exactly, or one of them is wrong.
        let rig = RigGeometry::default();

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, 0.6));
        pose.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::X, -0.3));
        pose.root_translation = Vec3::new(0.2, 0.0, -1.0);

        let synthetic = forward_kinematics(&pose);
        let explicit = forward_kinematics_on(&pose, &rig);

        for &bone in Bone::ALL.iter() {
            assert!(
                (synthetic[bone] - explicit[bone]).length() < 1.0e-5,
                "{} differs: {:?} vs {:?}",
                bone.name(),
                synthetic[bone],
                explicit[bone],
            );
        }
    }

    #[test]
    fn a_rig_with_different_proportions_places_bones_differently() {
        // The whole point: a taller rig must produce different world
        // positions from the same rig-independent pose.
        let mut tall = RigGeometry::default();
        tall.offsets[Bone::LeftUpLeg] *= 1.5;

        let default_positions = forward_kinematics_on(&LocalPose::REST, &RigGeometry::default());
        let tall_positions = forward_kinematics_on(&LocalPose::REST, &tall);

        assert!(
            (default_positions[Bone::LeftUpLeg] - tall_positions[Bone::LeftUpLeg]).length()
                > 0.1,
            "a longer femur should move the knee",
        );
        assert_eq!(
            default_positions[Bone::Hips], tall_positions[Bone::Hips],
            "...without moving anything above it",
        );
    }

    #[test]
    fn a_bind_rotation_orients_a_rigs_bones_without_any_pose() {
        // Bind rotations are what make a real glTF rig differ from the
        // synthetic one even at rest.
        let mut rig = RigGeometry::default();
        rig.bind_rotations[Bone::LeftUpLeg] = Quat::from_axis_angle(Vec3::X, FRAC_PI_2);

        let positions = forward_kinematics_on(&LocalPose::REST, &rig);
        let plain = forward_kinematics_on(&LocalPose::REST, &RigGeometry::default());

        assert!(
            (positions[Bone::LeftLeg] - plain[Bone::LeftLeg]).length() > 0.1,
            "a bind rotation on the thigh should swing everything below it",
        );
    }

    #[test]
    fn every_bones_index_round_trips_through_the_all_array() {
        // Pins `Bone::index` against `Bone::ALL`. These are two hand-written
        // tables that MUST agree; if they drift, every BoneSet access reads
        // or writes the wrong bone — a silent, catastrophic failure that
        // would otherwise surface only as a mysteriously scrambled pose.
        for (expected, &bone) in Bone::ALL.iter().enumerate() {
            assert_eq!(
                bone.index(),
                expected,
                "{}'s index() says {} but it sits at position {expected} in Bone::ALL",
                bone.name(),
                bone.index(),
            );
        }
    }

    #[test]
    fn every_index_is_within_bounds_and_distinct() {
        let mut seen = [false; BONE_COUNT];
        for &bone in Bone::ALL.iter() {
            let index = bone.index();
            assert!(index < BONE_COUNT, "{}'s index {index} is out of bounds", bone.name());
            assert!(!seen[index], "index {index} is claimed by more than one bone");
            seen[index] = true;
        }
        assert!(seen.iter().all(|&s| s), "every index in 0..{BONE_COUNT} must be claimed");
    }

    #[test]
    fn the_all_array_is_ordered_parent_before_child() {
        // THE ordering invariant. Forward kinematics, rotation accumulation
        // and the IK chain solves all walk 0..22 exactly once and read their
        // parent's already-final value. Reordering Bone::ALL would break
        // every one of them silently, producing subtly wrong poses rather
        // than an error — so it is pinned here rather than left to
        // convention.
        for (position, &bone) in Bone::ALL.iter().enumerate() {
            if let Some(parent) = bone.parent() {
                assert!(
                    parent.index() < position,
                    "{} sits at position {position} but its parent {} is at {} — \
                     Bone::ALL must list every parent before its children",
                    bone.name(),
                    parent.name(),
                    parent.index(),
                );
            }
        }
    }

    #[test]
    fn exactly_one_bone_has_no_parent() {
        let roots: Vec<_> = Bone::ALL.iter().filter(|b| b.parent().is_none()).collect();
        assert_eq!(roots.len(), 1, "the rig must have exactly one root, found {roots:?}");
        assert_eq!(*roots[0], Bone::Hips, "the root must be Hips");
    }

    #[test]
    fn a_bone_set_indexes_by_bone_and_reads_back_what_was_written() {
        let mut set = BoneSet::splat(0.0f32);
        set[Bone::LeftHand] = 1.5;
        set[Bone::RightFoot] = -2.0;

        assert_eq!(set[Bone::LeftHand], 1.5);
        assert_eq!(set[Bone::RightFoot], -2.0);
        assert_eq!(set[Bone::Head], 0.0, "untouched bones keep the splatted value");
    }

    #[test]
    fn a_bone_set_iterates_in_all_order() {
        let set = BoneSet::from_fn(|bone| bone.index());
        let visited: Vec<_> = set.iter().map(|(bone, _)| bone).collect();

        assert_eq!(visited, Bone::ALL.to_vec(), "iteration must follow Bone::ALL exactly");

        for (bone, &value) in set.iter() {
            assert_eq!(value, bone.index(), "from_fn must pair each bone with its own slot");
        }
    }

    #[test]
    fn the_rest_pose_reproduces_the_t_pose_world_positions_exactly() {
        // Anchors the new forward kinematics against the existing,
        // independently-derived `t_pose_world_position`. If these disagree,
        // one of the two is wrong — and this catches it before any pose is
        // authored on top.
        let positions = forward_kinematics(&LocalPose::REST);

        for &bone in Bone::ALL.iter() {
            let expected = bone.t_pose_world_position();
            let actual = positions[bone];
            assert!(
                (actual - expected).length() < 1.0e-5,
                "{} should sit at its T-pose position {expected:?}, got {actual:?}",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_rotation_pose_can_never_change_a_bone_length() {
        // The structural guarantee that replaces the superseded module's
        // dedicated bone-length invariant test: because a pose carries only
        // rotations, every bone stays exactly its rest length no matter what
        // is authored. Checked here across a deliberately extreme pose.
        let mut pose = LocalPose::REST;
        for (i, &bone) in Bone::ALL.iter().enumerate() {
            let axis = match i % 3 {
                0 => Vec3::X,
                1 => Vec3::Y,
                _ => Vec3::Z,
            };
            pose.set_rotation(bone, Quat::from_axis_angle(axis, 0.7 * (i as f32 + 1.0)));
        }

        let positions = forward_kinematics(&pose);

        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };

            let rest_length = bone.t_pose_offset().length();
            let posed_length = (positions[bone] - positions[parent]).length();

            assert!(
                (posed_length - rest_length).abs() < 1.0e-5,
                "{} must stay exactly {rest_length} from {} under any rotation, got \
                 {posed_length}",
                bone.name(),
                parent.name(),
            );
        }
    }

    #[test]
    fn rotating_a_bone_moves_its_descendants_and_nothing_else() {
        // Pins the "a bone's rotation orients its CHILDREN" convention — the
        // exact relationship the superseded module needed a write-onto-parent
        // rule to express, and which is implicit and automatic here.
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, FRAC_PI_2));

        let rest = forward_kinematics(&LocalPose::REST);
        let posed = forward_kinematics(&pose);

        // The rotated bone itself does not move — its parent positions it.
        assert!(
            (posed[Bone::LeftArm] - rest[Bone::LeftArm]).length() < 1.0e-5,
            "rotating LeftArm must not move LeftArm itself",
        );

        // Its descendants do.
        for descendant in [Bone::LeftForeArm, Bone::LeftHand] {
            assert!(
                (posed[descendant] - rest[descendant]).length() > 0.05,
                "rotating LeftArm must move its descendant {}",
                descendant.name(),
            );
        }

        // Nothing on the other side, or up the spine, moves at all.
        for untouched in [Bone::RightHand, Bone::Head, Bone::LeftFoot, Bone::Spine2] {
            assert!(
                (posed[untouched] - rest[untouched]).length() < 1.0e-5,
                "rotating LeftArm must not move {}",
                untouched.name(),
            );
        }
    }

    #[test]
    fn root_translation_moves_every_bone_rigidly() {
        let delta = Vec3::new(1.0, -0.25, 3.0);
        let rest = forward_kinematics(&LocalPose::REST);
        let moved = forward_kinematics(&LocalPose::at_root(delta));

        for &bone in Bone::ALL.iter() {
            let shift = moved[bone] - rest[bone];
            assert!(
                (shift - delta).length() < 1.0e-5,
                "{} should shift by exactly {delta:?}, got {shift:?}",
                bone.name(),
            );
        }
    }

    #[test]
    fn accumulated_rotations_compose_down_a_chain() {
        let a = Quat::from_axis_angle(Vec3::Y, 0.4);
        let b = Quat::from_axis_angle(Vec3::X, -0.3);

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Spine, a);
        pose.set_rotation(Bone::Spine1, b);

        let accumulated = accumulate_rest_relative_rotations(&pose);

        assert!(
            accumulated[Bone::Spine1].abs_diff_eq(a * b, 1.0e-5),
            "Spine1's accumulated rotation should be Spine * Spine1",
        );
        assert!(
            accumulated[Bone::Spine2].abs_diff_eq(a * b, 1.0e-5),
            "an unrotated child inherits its parent's accumulation unchanged",
        );
    }

    // -----------------------------------------------------------------
    // The virtual toe end
    // -----------------------------------------------------------------

    #[test]
    fn a_toe_tip_extends_beyond_the_toe_joint_along_the_toe() {
        let rig = RigGeometry::default();
        let positions = forward_kinematics(&LocalPose::REST);
        let (left_tip, _) = toe_end_positions(&LocalPose::REST, &rig);

        let ankle = positions[Bone::LeftFoot];
        let toe = positions[Bone::LeftToeBase];

        // The tip must lie further along the ankle->toe direction than the
        // toe joint itself, which is what "the tip of the toe" means.
        let along = (toe - ankle).normalize();
        assert!(
            (left_tip - ankle).dot(along) > (toe - ankle).dot(along),
            "the tip {left_tip:?} should sit beyond the toe joint {toe:?}",
        );

        // And by the expected fraction of the toe's own length.
        let expected = toe.distance(ankle) * TOE_END_FRACTION;
        assert!(
            (left_tip.distance(toe) - expected).abs() < 1.0e-5,
            "the tip should sit {expected} m past the toe, got {}",
            left_tip.distance(toe),
        );
    }

    #[test]
    fn a_toe_tip_estimate_respects_a_real_rigs_bind_rotation() {
        // THE regression for a bug this file shipped mid-write: the estimate
        // scaled the toe's PARENT-frame offset and used it as a TOE-frame
        // one. On this crate's synthetic rig every bind rotation is identity,
        // so the two coincide and the mistake is invisible — every test above
        // passes either way.
        //
        // `puppet_base.gltf` binds its `ball_l` at roughly 180 degrees
        // (measured: [0, 0.973, -0.230, 0]), which turns the tip around to
        // point back into the heel. So this uses a real half-turn and asserts
        // the tip still lands FORWARD of the toe joint.
        let mut rig = RigGeometry::default();
        rig.bind_rotations[Bone::LeftToeBase] =
            Quat::from_axis_angle(Vec3::Y, std::f32::consts::PI);
        rig.toe_end_offsets =
            toe_end_offsets_for(&rig.offsets, &rig.bind_rotations);

        let positions = forward_kinematics_on(&LocalPose::REST, &rig);
        let (left_tip, _) = toe_end_positions(&LocalPose::REST, &rig);

        let ankle = positions[Bone::LeftFoot];
        let toe = positions[Bone::LeftToeBase];
        let along = (toe - ankle).normalize();

        assert!(
            (left_tip - ankle).dot(along) > (toe - ankle).dot(along),
            "with a half-turn bind rotation the tip must still extend FORWARD \
             past the toe joint, but {left_tip:?} fell behind {toe:?} — the \
             offset is being used in the wrong frame",
        );
    }

    #[test]
    fn a_measured_toe_end_overrides_the_estimate() {
        // A rig that genuinely has the joint should use it, not the guess.
        let measured = Vec3::new(0.0, 0.0789, 0.0);
        let rig = RigGeometry::default().with_toe_end(Bone::LeftToeBase, measured);

        assert_eq!(rig.toe_end_offset(Bone::LeftToeBase), measured);
        assert_ne!(
            rig.toe_end_offset(Bone::RightToeBase), measured,
            "overriding one toe must not touch the other",
        );
    }

    #[test]
    fn a_toe_tip_follows_the_toe_when_the_foot_rotates() {
        // The tip is defined in the toe's frame, so rotating anything above
        // it must carry the tip along rigidly.
        let rig = RigGeometry::default();

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftFoot, Quat::from_axis_angle(Vec3::X, 0.5));

        let rest_tip = toe_end_positions(&LocalPose::REST, &rig).0;
        let posed_tip = toe_end_positions(&pose, &rig).0;

        assert!(
            (posed_tip - rest_tip).length() > 0.01,
            "rotating the ankle should move the toe tip",
        );

        // ...and rigidly: its distance from the toe joint is unchanged.
        let posed = forward_kinematics_on(&pose, &rig);
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        assert!(
            ((posed_tip - posed[Bone::LeftToeBase]).length()
                - (rest_tip - rest[Bone::LeftToeBase]).length())
                .abs()
                < 1.0e-6,
            "the tip must stay a fixed distance from the toe joint",
        );
    }

    #[test]
    fn both_toe_tips_are_mirror_images_in_the_rest_pose() {
        let rig = RigGeometry::default();
        let (left, right) = toe_end_positions(&LocalPose::REST, &rig);

        assert!(
            (left.x + right.x).abs() < 1.0e-6,
            "the tips should mirror across X, got {left:?} and {right:?}",
        );
        assert!((left.y - right.y).abs() < 1.0e-6, "and sit at the same height");
        assert!((left.z - right.z).abs() < 1.0e-6, "and the same depth");
    }

    #[test]
    fn accumulated_world_rotations_match_the_composition_at_write_back() {
        // Pins the shared accumulation against a hand-composed chain, since
        // `forward_kinematics_on` now depends on it.
        //
        // The delta is conjugated into the bone's accumulated bind frame,
        // because an authored delta names a WORLD axis — see
        // `accumulate_world_rotations`. Composing `bind * delta` instead turns
        // the right amount about the wrong axis, and the version of this test
        // that hand-composed it that way could not catch the bug: it asserted
        // the implementation against a restatement of the implementation.
        // `the_world_rotations_agree_with_what_retargeting_actually_writes` is
        // the one that checks against an independent authority.
        let mut rig = RigGeometry { root_rotation: Quat::from_axis_angle(Vec3::Y, 0.3), ..Default::default() };
        rig.bind_rotations[Bone::Spine] = Quat::from_axis_angle(Vec3::X, 0.2);

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::Z, -0.4));

        let accumulated = accumulate_world_rotations(&pose, &rig);

        let hips_bind = rig.root_rotation * rig.bind_rotations[Bone::Hips];
        let spine_bind = hips_bind * rig.bind_rotations[Bone::Spine];
        let conjugate = |bind: Quat, delta: Quat| bind.inverse() * delta * bind;

        let expected = rig.root_rotation
            * rig.bind_rotations[Bone::Hips]
            * conjugate(hips_bind, pose.rotations[Bone::Hips])
            * rig.bind_rotations[Bone::Spine]
            * conjugate(spine_bind, pose.rotations[Bone::Spine]);

        assert!(
            accumulated[Bone::Spine].abs_diff_eq(expected, 1.0e-6),
            "got {:?}, expected {expected:?}",
            accumulated[Bone::Spine],
        );
    }

    #[test]
    fn a_world_axis_delta_turns_about_that_world_axis_on_a_bound_rig() {
        // The authoring contract, stated directly: a delta of 40 degrees about
        // world +Y has to turn the bone 40 degrees about world +Y, whatever
        // the rig's bind pose happens to be.
        //
        // This is the property the local-frame composition violated. It bound
        // `Spine` 90 degrees about X, which maps the bone's local +Y onto
        // world -Z, so a local-frame delta swung the chain about the wrong
        // axis entirely while still reporting the right angle.
        let mut rig = RigGeometry::default();
        rig.bind_rotations[Bone::Spine] = Quat::from_axis_angle(Vec3::X, FRAC_PI_2);

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::Y, 0.4));

        let bound = accumulate_world_rotations(&pose, &rig);
        let unbound = accumulate_world_rotations(&pose, &RigGeometry::default());

        // The delta's effect, isolated from the bind orientation: on both rigs
        // it must be the same world-space turn.
        let bind = rig.bind_rotations[Bone::Hips] * rig.bind_rotations[Bone::Spine];
        let effect_bound = bound[Bone::Spine] * bind.inverse();

        assert!(
            effect_bound.abs_diff_eq(unbound[Bone::Spine], 1.0e-5),
            "the bound rig turned the spine by {effect_bound:?}, the unbound one by {:?}",
            unbound[Bone::Spine],
        );

        // And it really is about +Y, not merely consistent between the two.
        let axis = effect_bound.to_scaled_axis().normalize();
        assert!(axis.abs_diff_eq(Vec3::Y, 1.0e-4), "turned about {axis:?}, not +Y");
    }

    #[test]
    fn the_rest_pose_accumulates_to_identity_everywhere() {
        let accumulated = accumulate_rest_relative_rotations(&LocalPose::REST);
        for &bone in Bone::ALL.iter() {
            assert!(
                accumulated[bone].abs_diff_eq(Quat::IDENTITY, 1.0e-6),
                "{} should accumulate to identity in the rest pose, got {:?}",
                bone.name(),
                accumulated[bone],
            );
        }
    }
}
