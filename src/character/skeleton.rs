//! Procedural T-pose humanoid rig: spawns a `Transform` hierarchy following
//! the Mixamo-compatible bone standard (see module doc), with a capsule mesh
//! per bone and a small sphere per joint so the rig is visible without
//! needing a skinned mesh asset. Each bone is `ChildOf` its parent, so moving
//! a bone's local `Transform` later (procedural animation) carries every
//! descendant with it for free via Bevy's normal transform propagation.

use bevy::prelude::*;

/// One entry per bone in the standard 15-bone-minimum humanoid hierarchy
/// (`research/MODEL_STANDARD.md` in the botica project, itself
/// Mixamo/Unity-Humanoid/UE-Mannequin compatible). `Spine2` and the
/// shoulders are included even though the doc lists them "optional" because
/// they make the T-pose read correctly (shoulders give the arms their
/// outward offset) and are present on nearly every real rig.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
    Center,
}

/// `Reflect` so the rig is introspectable over the Bevy Remote Protocol.
/// Components keyed by `Bone` (e.g. `JointTarget`) are otherwise invisible
/// to a live BRP query, which is the fastest way this project has to tell
/// "the value is wrong" from "the value is right and something downstream
/// ignores it".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Reflect)]
pub enum Bone {
    Hips,
    Spine,
    Spine1,
    Spine2,
    Neck,
    Head,
    LeftShoulder,
    LeftArm,
    LeftForeArm,
    LeftHand,
    RightShoulder,
    RightArm,
    RightForeArm,
    RightHand,
    LeftUpLeg,
    LeftLeg,
    LeftFoot,
    LeftToeBase,
    RightUpLeg,
    RightLeg,
    RightFoot,
    RightToeBase,
}

impl Bone {
    pub const ALL: [Bone; 22] = [
        Bone::Hips,
        Bone::Spine,
        Bone::Spine1,
        Bone::Spine2,
        Bone::Neck,
        Bone::Head,
        Bone::LeftShoulder,
        Bone::LeftArm,
        Bone::LeftForeArm,
        Bone::LeftHand,
        Bone::RightShoulder,
        Bone::RightArm,
        Bone::RightForeArm,
        Bone::RightHand,
        Bone::LeftUpLeg,
        Bone::LeftLeg,
        Bone::LeftFoot,
        Bone::LeftToeBase,
        Bone::RightUpLeg,
        Bone::RightLeg,
        Bone::RightFoot,
        Bone::RightToeBase,
    ];

    /// PascalCase name matching the Mixamo-compatible standard exactly
    /// (`mixamorig:` prefix stripped), so a future skinned glTF import can
    /// reuse animation code written against these names unchanged.
    pub fn name(self) -> &'static str {
        match self {
            Bone::Hips => "Hips",
            Bone::Spine => "Spine",
            Bone::Spine1 => "Spine1",
            Bone::Spine2 => "Spine2",
            Bone::Neck => "Neck",
            Bone::Head => "Head",
            Bone::LeftShoulder => "LeftShoulder",
            Bone::LeftArm => "LeftArm",
            Bone::LeftForeArm => "LeftForeArm",
            Bone::LeftHand => "LeftHand",
            Bone::RightShoulder => "RightShoulder",
            Bone::RightArm => "RightArm",
            Bone::RightForeArm => "RightForeArm",
            Bone::RightHand => "RightHand",
            Bone::LeftUpLeg => "LeftUpLeg",
            Bone::LeftLeg => "LeftLeg",
            Bone::LeftFoot => "LeftFoot",
            Bone::LeftToeBase => "LeftToeBase",
            Bone::RightUpLeg => "RightUpLeg",
            Bone::RightLeg => "RightLeg",
            Bone::RightFoot => "RightFoot",
            Bone::RightToeBase => "RightToeBase",
        }
    }

    /// Reverse of [`Bone::name`] — used by `character::anim::asset` to
    /// resolve the PascalCase bone names in a `.pose.ron` file against the
    /// rig, so a typo is reported rather than silently ignored.
    pub fn from_name(name: &str) -> Option<Bone> {
        Bone::ALL.iter().copied().find(|bone| bone.name() == name)
    }

    /// This bone's own dense index into [`Bone::ALL`].
    ///
    /// Lets `character::anim` store per-bone state as flat `[T; 22]` arrays
    /// indexed directly, instead of hashing a `Bone` on every access. The
    /// superseded position-space module (since deleted) hashed several
    /// times per bone per frame and rebuilt four `Vec`s per character per
    /// frame; an array index removes both costs and makes the whole solve a
    /// pure function over contiguous memory.
    ///
    /// `const` so it can be used in array initializers and const contexts.
    /// Kept in lockstep with `ALL` by
    /// `every_bones_index_round_trips_through_the_all_array`.
    pub const fn index(self) -> usize {
        match self {
            Bone::Hips => 0,
            Bone::Spine => 1,
            Bone::Spine1 => 2,
            Bone::Spine2 => 3,
            Bone::Neck => 4,
            Bone::Head => 5,
            Bone::LeftShoulder => 6,
            Bone::LeftArm => 7,
            Bone::LeftForeArm => 8,
            Bone::LeftHand => 9,
            Bone::RightShoulder => 10,
            Bone::RightArm => 11,
            Bone::RightForeArm => 12,
            Bone::RightHand => 13,
            Bone::LeftUpLeg => 14,
            Bone::LeftLeg => 15,
            Bone::LeftFoot => 16,
            Bone::LeftToeBase => 17,
            Bone::RightUpLeg => 18,
            Bone::RightLeg => 19,
            Bone::RightFoot => 20,
            Bone::RightToeBase => 21,
        }
    }

    /// Which side of the body this bone belongs to — used purely for debug
    /// color-coding (left/right mix-ups are one of the easiest pose bugs to
    /// make and one of the hardest to spot from a plain gray rig, since nothing
    /// else distinguishes mirrored bones at a glance).
    pub fn side(self) -> Side {
        match self {
            Bone::LeftShoulder
            | Bone::LeftArm
            | Bone::LeftForeArm
            | Bone::LeftHand
            | Bone::LeftUpLeg
            | Bone::LeftLeg
            | Bone::LeftFoot
            | Bone::LeftToeBase => Side::Left,
            Bone::RightShoulder
            | Bone::RightArm
            | Bone::RightForeArm
            | Bone::RightHand
            | Bone::RightUpLeg
            | Bone::RightLeg
            | Bone::RightFoot
            | Bone::RightToeBase => Side::Right,
            _ => Side::Center,
        }
    }

    /// This bone's parent in the standard hierarchy, or `None` for the root
    /// (`Hips`). Mirrors `research/MODEL_STANDARD.md`'s tree exactly.
    pub fn parent(self) -> Option<Bone> {
        match self {
            Bone::Hips => None,
            Bone::Spine => Some(Bone::Hips),
            Bone::Spine1 => Some(Bone::Spine),
            Bone::Spine2 => Some(Bone::Spine1),
            Bone::Neck => Some(Bone::Spine2),
            Bone::Head => Some(Bone::Neck),
            Bone::LeftShoulder => Some(Bone::Spine2),
            Bone::LeftArm => Some(Bone::LeftShoulder),
            Bone::LeftForeArm => Some(Bone::LeftArm),
            Bone::LeftHand => Some(Bone::LeftForeArm),
            Bone::RightShoulder => Some(Bone::Spine2),
            Bone::RightArm => Some(Bone::RightShoulder),
            Bone::RightForeArm => Some(Bone::RightArm),
            Bone::RightHand => Some(Bone::RightForeArm),
            Bone::LeftUpLeg => Some(Bone::Hips),
            Bone::LeftLeg => Some(Bone::LeftUpLeg),
            Bone::LeftFoot => Some(Bone::LeftLeg),
            Bone::LeftToeBase => Some(Bone::LeftFoot),
            Bone::RightUpLeg => Some(Bone::Hips),
            Bone::RightLeg => Some(Bone::RightUpLeg),
            Bone::RightFoot => Some(Bone::RightLeg),
            Bone::RightToeBase => Some(Bone::RightFoot),
        }
    }

    /// For a bone with MORE THAN ONE child (only `Hips` and `Spine2` in
    /// this rig — `Hips`: `Spine`/`LeftUpLeg`/`RightUpLeg`; `Spine2`:
    /// `Neck`/`LeftShoulder`/`RightShoulder`), the ONE child whose solved
    /// direction determines this bone's OWN rendered rotation — every
    /// other child's rotation is computed independently and never feeds
    /// back into this bone's own rotation at all. `None` for every
    /// single-child bone (its own only child is unambiguous, no picking
    /// needed) and for every leaf bone (no children at all).
    ///
    /// Naively averaging (e.g. quaternion slerp) a parent's rotation
    /// across ALL of its children is a known-bad approach in real
    /// procedural-animation/IK systems (Unity Animation Rigging, Unreal
    /// Control Rig, FABRIK multi-effector solvers all avoid it): it makes
    /// NONE of the children reach their target exactly, and reads as
    /// visible jitter whenever children disagree by any meaningful
    /// amount. The standard, cheap alternative instead treats the
    /// spine as its own independent chain and attaches lateral limbs via
    /// a fixed rest-pose-relative offset from whatever the spine chain
    /// already decided, letting each limb's OWN downstream swing chain
    /// (shoulder->arm->forearm->hand, hip->upleg->leg->foot) absorb the
    /// positional deviation instead of fighting for the shared parent's
    /// rotation. `Neck` (continuing the spine up from `Spine2`) and
    /// `Spine` (continuing it up from `Hips`) are exactly this rig's own
    /// spine-chain-continuation bones; `LeftShoulder`/`RightShoulder`/
    /// `LeftUpLeg`/`RightUpLeg` are the lateral attachments this
    /// deliberately excludes.
    pub fn chain_continuation_child(self) -> Option<Bone> {
        match self {
            Bone::Hips => Some(Bone::Spine),
            Bone::Spine2 => Some(Bone::Neck),
            _ => None,
        }
    }

    /// This bone's own T-pose WORLD position, computed by summing
    /// `t_pose_offset` up the `Bone::parent` chain from `Hips` (the rig's
    /// own fixed/pinned root — see `character::muscle::plugin`'s own
    /// `MuscleSim`, where `Hips` is the one `Joint::pinned` particle).
    /// Assumes the rig's root entity sits at the world origin with
    /// identity rotation, matching every current caller of
    /// `spawn_humanoid_debug_skeleton` (`Transform::IDENTITY`) — a rig
    /// spawned at a different root transform would need this adjusted.
    /// Deliberately a pure function of the STATIC rest-pose tree, not
    /// `GlobalTransform` — usable even before Bevy's own
    /// `TransformSystems::Propagate` has run for a just-spawned rig (see
    /// `MuscleSim`'s own spawn system for why that timing matters).
    pub fn t_pose_world_position(self) -> Vec3 {
        match self.parent() {
            Some(parent) => parent.t_pose_world_position() + self.t_pose_offset(),
            None => self.t_pose_offset(),
        }
    }

    /// T-pose local-space offset from this bone's parent joint (i.e. the
    /// parent-to-child translation baked into the parent's rest length).
    /// `pub`, not private: `character::muscle` reads rest lengths/
    /// directions when building its avian3d joint topology (each joint's
    /// anchor is derived directly from this), and consumers outside the
    /// crate (e.g. `examples/character_gallery.rs`'s own debug gizmos)
    /// legitimately need the same rest-pose data to compare "where a bone
    /// actually is" against "where the T-pose says it should be".
    /// Proportions target a real ~1.8m-tall adult per
    /// `MODEL_STANDARD.md`'s height requirement, summed and checked
    /// bone-by-bone (all figures floor-to-head, i.e. `Hips.y` + the
    /// upward spine/neck/head stack, since that's the actual
    /// standing-height sum this rig produces): legs 0.94m to the hip
    /// (thigh 0.45 + shin 0.42 + ankle-to-sole 0.07), torso 0.50m
    /// (Spine+Spine1+Spine2, hip line to shoulder line), neck 0.10m, head
    /// 0.24m (Neck joint to nominal skull-top — `Head` is a leaf bone
    /// here, see the module doc comment on the omitted
    /// `HeadTop_End`), totaling 0.94+0.50+0.10+0.24 = 1.78m. Arms extended
    /// horizontally sum to ~0.6m per side (shoulder 0.16 + arm 0.28 +
    /// forearm 0.26, overlapping slightly at each joint sphere).
    pub fn t_pose_offset(self) -> Vec3 {
        match self {
            // Root: hips sit at leg height off the ground (thigh + shin +
            // ankle-to-sole, see doc comment above) so the feet land at
            // y ~= 0 exactly, matching `LeftFoot`/`RightFoot`'s own downward
            // sum below.
            Bone::Hips => Vec3::new(0.0, 0.94, 0.0),

            Bone::Spine => Vec3::new(0.0, 0.17, 0.0),
            Bone::Spine1 => Vec3::new(0.0, 0.17, 0.0),
            Bone::Spine2 => Vec3::new(0.0, 0.16, 0.0),
            Bone::Neck => Vec3::new(0.0, 0.10, 0.0),
            Bone::Head => Vec3::new(0.0, 0.24, 0.0),

            // Shoulders: short lateral offset from the spine to where the
            // arm socket sits, so the T-pose arm starts at the shoulder line
            // rather than the spine's centerline. `MODEL_STANDARD.md`'s
            // coordinate system defines +X as the character's RIGHT side, so
            // `Right*` bones sit at positive X and `Left*` at negative X —
            // matching the front marker's own -Z-forward convention (see
            // `spawn_humanoid_debug_skeleton`'s doc comment): a character
            // facing -Z with +Y up has its right hand toward +X by the
            // right-hand rule (forward x up = (0,0,-1) x (0,1,0) = (1,0,0)).
            Bone::RightShoulder => Vec3::new(0.16, 0.10, 0.0),
            Bone::RightArm => Vec3::new(0.14, 0.0, 0.0),
            Bone::RightForeArm => Vec3::new(0.28, 0.0, 0.0),
            Bone::RightHand => Vec3::new(0.26, 0.0, 0.0),

            Bone::LeftShoulder => Vec3::new(-0.16, 0.10, 0.0),
            Bone::LeftArm => Vec3::new(-0.14, 0.0, 0.0),
            Bone::LeftForeArm => Vec3::new(-0.28, 0.0, 0.0),
            Bone::LeftHand => Vec3::new(-0.26, 0.0, 0.0),

            // Legs: straight down from the hips, standard Mixamo hip-width
            // separation. Same +X=right convention as the arms above.
            // Thigh (0.45) + shin (0.42) + ankle-to-sole (0.07) = 0.94,
            // matching `Hips`' own height above so the sole lands at
            // y ~= 0 exactly.
            Bone::RightUpLeg => Vec3::new(0.10, -0.45, 0.0),
            Bone::RightLeg => Vec3::new(0.0, -0.42, 0.0),
            Bone::RightFoot => Vec3::new(0.0, -0.07, 0.0),
            // Toe points along -Z (forward, matching this rig's own
            // declared forward axis — see the front marker in
            // `spawn_humanoid_debug_skeleton`), NOT +Z: a toe pointing +Z
            // would point backward relative to the character's own facing
            // direction, which is self-contradictory and was a real bug in
            // an earlier version of this rig.
            Bone::RightToeBase => Vec3::new(0.0, -0.02, -0.14),

            Bone::LeftUpLeg => Vec3::new(-0.10, -0.45, 0.0),
            Bone::LeftLeg => Vec3::new(0.0, -0.42, 0.0),
            Bone::LeftFoot => Vec3::new(0.0, -0.07, 0.0),
            Bone::LeftToeBase => Vec3::new(0.0, -0.02, -0.14),
        }
    }
}

/// Maps each spawned [`Bone`] to its entity, so later systems (procedural
/// animation, IK, debug UI) can look up "the LeftHand entity" without
/// re-walking the hierarchy or matching on names.
///
/// `other_rig` is `None` for a rig built from this crate's OWN T-pose
/// numbers (`spawn_humanoid_debug_skeleton`) — every method below falls
/// back to this crate's own conventions in that case (identity rest
/// rotation for every bone, `Bone::Hips` sitting directly under an
/// identity-transform root), exactly matching this struct's original,
/// single-rig behavior.
///
/// `Some` for a DIFFERENT skeleton MIRRORING the same
/// [`crate::character::muscle::plugin::MuscleSim`] (e.g. a real,
/// downloaded skinned character) whose own bind pose does not follow
/// either of those conventions — see [`OtherRigRestPose`]'s own doc
/// comment for the actual retargeting approach this requires (delta-
/// rotation retargeting: never reuse `MuscleSim`'s own world-space
/// directions on a different rig directly — its own bind pose lives in a
/// completely unrelated coordinate/rest-rotation convention, so mixing
/// the two is meaningless even though the numbers type-check. A real,
/// once-live bug from an earlier, wrong approach: composing a different
/// rig's own non-identity bind rotation as if it were this crate's own
/// T-pose-relative reference frame visibly collapsed the mirrored mesh
/// into a distorted heap, even though the debug skeleton's own bone
/// lengths were simultaneously exactly correct in the same frame).
#[derive(Component, Debug, Clone)]
pub struct HumanoidSkeleton {
    bones: std::collections::HashMap<Bone, Entity>,
    other_rig: Option<OtherRigRestPose>,
}

/// Everything needed to retarget a DIFFERENT rig's own bind pose onto
/// `MuscleSim`'s solved output, via DELTA-rotation retargeting: each
/// bone's rendered rotation is computed as `own_rest_rotation *
/// swing_delta`, where `swing_delta` is the SAME rig-independent
/// rotation-away-from-T-pose [`crate::character::muscle::plugin::
/// apply_solved_sim_to_skeleton`] already derives purely from
/// `MuscleSim`'s own T-pose-relative world-space solve (never touching
/// this rig's own bind pose at all) and `own_rest_rotation` is THIS rig's
/// own bind-pose local rotation for that same bone — composing a
/// rig-independent delta onto a rig-specific rest pose is what correctly
/// separates "how far the physics solve swung this bone" (portable across
/// any rig) from "what does this specific rig's own T-pose/rest look
/// like" (never portable, an earlier wrong approach's actual bug).
#[derive(Debug, Clone)]
struct OtherRigRestPose {
    rest_rotations: std::collections::HashMap<Bone, Quat>,
    /// Each mapped bone's own REST local translation direction (parent to
    /// child, normalized) — the `Quat::from_rotation_arc` "from" argument
    /// `apply_solved_sim_to_skeleton` needs to compute a swing rotation
    /// entirely self-consistently in THIS rig's own frame, never mixing
    /// in this crate's own T-pose direction constants (see that
    /// function's own doc comment for why composing a T-pose-relative
    /// delta onto a different rig's own rest rotation AFTER THE FACT is
    /// the wrong, fragile approach an earlier attempt used).
    rest_directions: std::collections::HashMap<Bone, Vec3>,
    /// `Bone::Hips`'s own mapped joint (e.g. `pelvis`) REST local
    /// translation, captured once before any animation write ever
    /// touches it.
    hips_rest_local_translation: Vec3,
    /// The world-space rotation of `Bone::Hips`'s own mapped joint's
    /// PARENT, captured once at rest (before any animation write) — used
    /// to convert `MuscleSim`'s own world-space `Bone::Hips` position
    /// into whatever LOCAL frame this specific rig's own hip joint
    /// actually translates in (its parent chain is very unlikely to sit
    /// at a plain identity transform the way the debug skeleton's own
    /// `Bone::Hips` does — e.g. this glTF's own `pelvis` sits under a
    /// `root` node carrying a real `-90°` axis-correction rotation).
    hips_parent_rest_world_rotation: Quat,
    /// The world-space SCALE of `Bone::Hips`'s own mapped joint's PARENT,
    /// captured once at rest — most real assets carry an identity scale
    /// here (`puppet_base.gltf` does), but a Mixamo FBX round-tripped
    /// through Blender's glTF exporter puts a real `0.01` scale node
    /// above the armature (Blender's own cm-to-m unit correction for an
    /// FBX authored in centimeters) — a real, once-live bug: `hips_local_
    /// translation_for`'s own world-to-local delta conversion divided out
    /// the parent's ROTATION but never its SCALE, so on a scaled parent
    /// chain the computed local delta stayed at METER magnitude while
    /// `hips_rest_local_translation` (captured directly from the node's
    /// own pre-scale `Transform.translation`) was already in that parent's
    /// own CENTIMETER-scale local units — adding the two together
    /// produced a nonsense translation dominated by the untouched rest
    /// value (live-caught via the HUD's own per-bone position readout
    /// showing `Hips pos (0.00, 0.28, -112.56)`, `Bone::Hips.z` never
    /// budging from its raw, unconverted centimeter rest value regardless
    /// of pose). Dividing `local_delta` by this scale (element-wise, right
    /// alongside the rotation un-rotate) fixes it for both the scaled
    /// (Mixamo/Blender) and unscaled (`puppet_base.gltf`) case uniformly
    /// — `Vec3::ONE` is a no-op division.
    hips_parent_rest_world_scale: Vec3,
    /// `MuscleSim`'s OWN rest reference for `Bone::Hips` —
    /// `Bone::Hips.t_pose_world_position()`, this crate's own fixed
    /// constant, NOT the mirrored rig's own scene-space world position
    /// (a real, once-live bug: the two are numerically close for this
    /// specific glTF but are NOT the same coordinate space in general,
    /// since every `MuscleSim` joint starts at THIS crate's own T-pose
    /// position regardless of which skeleton it's later mirrored onto —
    /// see `spawn_muscle_sim`'s own doc comment). Subtracted from
    /// `MuscleSim`'s own live `Bone::Hips` world position to get the pure
    /// MOVEMENT delta since rest, which is what actually needs converting
    /// into the mirrored rig's own local frame (an absolute position is
    /// meaningless to reuse directly across two differently authored
    /// rigs; only the DELTA the debug skeleton's own hip has moved by is
    /// portable).
    hips_rest_world_position: Vec3,
}

impl HumanoidSkeleton {
    pub fn entity(&self, bone: Bone) -> Entity {
        self.bones[&bone]
    }

    pub fn iter(&self) -> impl Iterator<Item = (Bone, Entity)> + '_ {
        self.bones.iter().map(|(&bone, &entity)| (bone, entity))
    }

    /// The REAL, actually-rendered GLOBAL rotation sitting above
    /// `Bone::Hips`'s own entity at spawn time — `Quat::IDENTITY` for the
    /// debug skeleton (its own root is a bare, unrotated `Transform`), or
    /// this specific mirrored rig's own real ancestor-chain rotation
    /// otherwise (e.g. a glTF importer's own Blender-Z-up correction node
    /// PLUS any additional rigid re-orientation applied to the mesh's own
    /// spawn root, like `examples/character_gallery.rs`'s `spawn_real_
    /// mesh` 180° facing-direction fix). `apply_solved_sim_to_skeleton`
    /// (`plugin.rs`) needs this to seed its own REAL-frame `global_
    /// rotation` bookkeeping correctly: that bookkeeping is built ENTIRELY
    /// from `rest_rotation`/swing composition and never otherwise reads
    /// this ancestor chain, so without this seed it silently omits
    /// whatever rotation sits above `Hips` -- a real, live bug (`Hips`'s
    /// own rendering is unaffected, since Bevy composes the real ancestor
    /// chain automatically regardless of what this crate's bookkeeping
    /// thinks, but every SWINGING descendant's `solved_direction`
    /// conversion, which explicitly divides by this bookkeeping's own
    /// `this_frame_rotation`, comes out silently wrong whenever this
    /// ancestor rotation is non-identity -- live-caught via screenshot:
    /// `wave_pose`'s `RightArm` swing rendered as barely-moved-from-T-pose
    /// once the mesh root gained its own 180° yaw correction, even though
    /// `MuscleSim` was solving the correct target position and an
    /// untouched bone like `Spine` kept rendering correctly, since only
    /// the untouched path is exempt from needing this frame at all).
    pub fn hips_root_rotation(&self) -> Quat {
        match &self.other_rig {
            Some(other) => other.hips_parent_rest_world_rotation,
            None => Quat::IDENTITY,
        }
    }

    /// This bone's own REST local rotation on THIS specific skeleton
    /// instance — `Quat::IDENTITY` for the debug skeleton (its own
    /// `spawn_humanoid_debug_skeleton` never rotates any bone's own
    /// transform, so identity genuinely IS its rest state) or a real,
    /// captured bind-pose rotation for a mirrored rig (see
    /// [`OtherRigRestPose`]'s own doc comment for how this composes with
    /// `MuscleSim`'s own rig-independent swing delta).
    pub fn rest_rotation(&self, bone: Bone) -> Quat {
        match &self.other_rig {
            Some(other) => other.rest_rotations.get(&bone).copied().unwrap_or(Quat::IDENTITY),
            None => Quat::IDENTITY,
        }
    }

    /// This bone's own REST local direction (parent-to-child, unit
    /// length) on THIS specific skeleton instance — this crate's own
    /// fixed [`Bone::t_pose_offset`] for the debug skeleton, or a real,
    /// captured bind-pose direction for a mirrored rig. See
    /// [`OtherRigRestPose::rest_directions`]'s own doc comment for why
    /// this must come from THIS skeleton's own rest pose, never a shared
    /// T-pose constant, for `apply_solved_sim_to_skeleton`'s swing
    /// computation to stay self-consistent per-rig.
    pub fn rest_direction(&self, bone: Bone) -> Vec3 {
        match &self.other_rig {
            Some(other) => other.rest_directions.get(&bone).copied().unwrap_or_else(|| bone.t_pose_offset().normalize_or_zero()),
            None => bone.t_pose_offset().normalize_or_zero(),
        }
    }

    /// Converts `MuscleSim`'s own world-space `Bone::Hips` position into
    /// whatever LOCAL translation this specific skeleton's own mapped hip
    /// joint needs, so `Transform.translation` (always local-space in
    /// Bevy) ends up correct once combined with its own parent chain's
    /// real transform — see [`OtherRigRestPose`]'s own doc comments for
    /// why a plain "assign the world position directly" (correct ONLY
    /// when the parent chain sits at a plain identity transform, true for
    /// the debug skeleton but not for a real character with its own
    /// non-trivial root hierarchy) breaks for a mirrored rig.
    pub fn hips_local_translation_for(&self, solved_world_position: Vec3) -> Vec3 {
        match &self.other_rig {
            None => solved_world_position,
            Some(other) => {
                let world_delta = solved_world_position - other.hips_rest_world_position;
                let rotated_delta = other.hips_parent_rest_world_rotation.inverse() * world_delta;
                // Un-scale too, not just un-rotate -- see `hips_parent_
                // rest_world_scale`'s own doc comment for the real bug
                // this fixes (a scaled parent chain, e.g. Blender's own
                // FBX-cm-to-glTF-m `0.01` correction node, left this
                // delta at world/meter magnitude while `hips_rest_local_
                // translation` was already in the parent's own, possibly
                // much larger, pre-scale local units).
                let local_delta = rotated_delta / other.hips_parent_rest_world_scale;
                other.hips_rest_local_translation + local_delta
            }
        }
    }

    /// Builds a skeleton retargeting a DIFFERENT rig's own bind pose —
    /// see [`HumanoidSkeleton`]'s own doc comment for why a real,
    /// downloaded character (different artist, different proportions,
    /// possibly a non-identity root rest rotation) needs this instead of
    /// the plain debug-skeleton constructor's own T-pose assumptions.
    pub fn for_other_rig(
        bones: std::collections::HashMap<Bone, Entity>,
        rest_rotations: std::collections::HashMap<Bone, Quat>,
        rest_directions: std::collections::HashMap<Bone, Vec3>,
        hips_rest_local_translation: Vec3,
        hips_parent_rest_world_rotation: Quat,
        hips_parent_rest_world_scale: Vec3,
        hips_rest_world_position: Vec3,
    ) -> Self {
        Self {
            bones,
            other_rig: Some(OtherRigRestPose {
                rest_rotations,
                rest_directions,
                hips_rest_local_translation,
                hips_parent_rest_world_rotation,
                hips_parent_rest_world_scale,
                hips_rest_world_position,
            }),
        }
    }
}

/// Tags a bone's own joint entity with which [`Bone`] it is, so debug
/// systems (gizmos, an egui inspector panel) can go entity -> bone without
/// walking through `HumanoidSkeleton`'s map or matching on `Name` strings.
#[derive(Component, Debug, Clone, Copy)]
pub struct BoneMarker(pub Bone);

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Spawns bare `Transform`/`ChildOf` bone entities (each carrying
    /// `BoneMarker`) with NO mesh/material geometry at all — the debug
    /// capsule/joint-sphere visualization this test builder used to
    /// include was removed along with `spawn_humanoid_debug_skeleton`
    /// itself once the real mesh's own glTF joints became the only
    /// skeleton this crate drives (see `plugin.rs`'s own module doc
    /// comment) — but unit tests still need SOME real, addressable ECS
    /// entity per bone to exercise `apply_solved_sim_to_skeleton`'s
    /// `Transform`-writing behavior against, hence this minimal
    /// `#[cfg(test)]`-only builder in place of the old one.
    pub(crate) fn spawn_bare_bone_entities(commands: &mut Commands, root_transform: Transform) -> (Entity, HumanoidSkeleton) {
        let root = commands.spawn((root_transform, Name::new("TestHumanoidRoot"))).id();
        let mut bones = std::collections::HashMap::new();
        for &bone in &Bone::ALL {
            let parent_entity = match bone.parent() {
                Some(parent_bone) => bones[&parent_bone],
                None => root,
            };
            let bone_entity = commands
                .spawn((Transform::from_translation(bone.t_pose_offset()), ChildOf(parent_entity), Name::new(bone.name()), BoneMarker(bone)))
                .id();
            bones.insert(bone, bone_entity);
        }
        let skeleton = HumanoidSkeleton { bones, other_rig: None };
        commands.entity(root).insert(skeleton.clone());
        (root, skeleton)
    }

    /// Regression test for a real, once-live bug: `HumanoidSkeleton::
    /// rest_rotation`/`rest_direction` must return exactly what a
    /// mirrored rig was built with (`for_other_rig`'s own arguments), not
    /// silently fall back to this crate's own T-pose constants for a
    /// bone that WAS actually captured. An earlier version of `plugin::
    /// apply_solved_sim_to_skeleton` read `skeleton.rest_direction(bone)`
    /// for use as `Quat::from_rotation_arc`'s "from" argument directly
    /// against `MuscleSim`'s own T-pose-relative `solved_direction` --
    /// mixing the mirrored rig's own captured (glTF bind-pose) direction
    /// with this crate's own T-pose-relative solved-direction convention
    /// is meaningless even though it type-checks (they're unrelated
    /// coordinate spaces) -- the fix moved to keeping the whole swing
    /// computation in `Bone::t_pose_offset`'s own convention throughout,
    /// and using `rest_rotation`/`rest_direction` ONLY at the very final
    /// step, composed onto the already-computed rig-independent swing.
    /// This test only guards that `rest_rotation`/`rest_direction`
    /// themselves still return the real captured values (the OTHER half
    /// of that fix, `plugin::apply_solved_sim_to_skeleton`'s own internal
    /// math, isn't unit-testable without spinning up a full ECS `World`
    /// — it's covered by live visual verification instead, see this
    /// crate's own commit history).
    #[test]
    fn mirrored_skeleton_rest_rotation_and_direction_return_the_captured_values_not_t_pose_fallback() {
        let captured_rotation = Quat::from_xyzw(-0.0919, 0.0, 0.0, 0.9958);
        let captured_direction = Vec3::new(0.3, 0.9, 0.1).normalize();

        let bones = std::collections::HashMap::from([(Bone::Hips, Entity::PLACEHOLDER), (Bone::Spine, Entity::PLACEHOLDER)]);
        let rest_rotations = std::collections::HashMap::from([(Bone::Spine, captured_rotation)]);
        let rest_directions = std::collections::HashMap::from([(Bone::Spine, captured_direction)]);
        let skeleton = HumanoidSkeleton::for_other_rig(bones, rest_rotations, rest_directions, Vec3::ZERO, Quat::IDENTITY, Vec3::ONE, Vec3::ZERO);

        assert_eq!(skeleton.rest_rotation(Bone::Spine), captured_rotation, "expected the mirrored rig's own captured rest rotation, not a T-pose fallback");
        assert_eq!(skeleton.rest_direction(Bone::Spine), captured_direction, "expected the mirrored rig's own captured rest direction, not Bone::t_pose_offset()");

        // A bone with NO captured entry (not every one of the 22 `Bone`
        // variants necessarily gets mapped) correctly falls back to this
        // crate's own T-pose constants, matching the debug skeleton's own
        // convention -- this fallback IS appropriate here, unlike inside
        // `apply_solved_sim_to_skeleton`'s own swing computation (see
        // this test's own doc comment).
        assert_eq!(skeleton.rest_rotation(Bone::Head), Quat::IDENTITY);
        assert_eq!(skeleton.rest_direction(Bone::Head), Bone::Head.t_pose_offset().normalize_or_zero());

        // The plain debug-skeleton constructor (`other_rig: None`) always
        // returns identity/T-pose regardless of which bone is asked.
        let debug_skeleton = HumanoidSkeleton { bones: std::collections::HashMap::from([(Bone::Spine, Entity::PLACEHOLDER)]), other_rig: None };
        assert_eq!(debug_skeleton.rest_rotation(Bone::Spine), Quat::IDENTITY);
        assert_eq!(debug_skeleton.rest_direction(Bone::Spine), Bone::Spine.t_pose_offset().normalize_or_zero());
    }
}

