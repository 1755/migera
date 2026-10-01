//! Rotation-space procedural character animation — a reusable Bevy plugin.
//!
//! This module replaces `character::muscle`'s position-space mass-spring
//! solver. The distinction is the whole design, so it is worth stating
//! precisely:
//!
//! - **`muscle` (superseded)** simulates joint *positions* as particles
//!   linked by distance constraints, then *derives* each bone's rendered
//!   rotation from the solved positions via `Quat::from_rotation_arc`.
//!   Rotation is never a simulated quantity.
//! - **`anim` (here)** authors and springs each bone's *local rotation*
//!   directly. Position is never simulated; it falls out of forward
//!   kinematics for free, via Bevy's own `ChildOf` transform propagation.
//!
//! # Why invert the primitive
//!
//! The position-space design was a deliberate, well-documented choice (see
//! `docs/knowledge/character-animation/lugaru-joint-muscle-system.md`) and
//! it worked. But four of its hardest, most-debugged problems exist *only*
//! because rotation was reconstructed rather than authored:
//!
//! - `from_rotation_arc` yields a direction with **arbitrary roll**, so the
//!   bind pose's own roll had to be guessed back. Three separate strategies
//!   for that were tried and all three were real, screenshot-caught bugs.
//! - A bone's inferred rotation is really its *parent's*, forcing a
//!   write-onto-parent rule and an off-by-one that bit repeatedly.
//! - A parent with three children (`Hips`, `Spine2`) gets three
//!   disagreeing inferred rotations, needing a `chain_continuation_child`
//!   tie-breaker.
//! - A segment's *direction* lags a position blend, needing a `swing_weight`
//!   ramp to hide the lag.
//!
//! Authoring rotations natively does not *solve* these; it makes them
//! **cease to exist**. Each bone owns exactly one local quaternion, written
//! to its own entity. Roll is authored data. Bone lengths become invariant
//! by construction, because a rotation cannot stretch a bone.
//!
//! # What survives from the old module
//!
//! The rig-binding half of the old retargeting — mapping our `Bone` enum
//! onto a foreign rig's bind pose (`HumanoidSkeleton::for_other_rig`,
//! `rest_rotation`, `hips_local_translation_for` and its scale correction)
//! — is orthogonal to how a pose was produced, and is kept verbatim in
//! `character::skeleton`. Any system emitting per-bone local rotation
//! deltas needs exactly that, unchanged.

pub mod anthropometry;
pub mod armik;
pub mod asset;
pub mod balance;
pub mod clip;
pub mod convert;
pub mod dho;
pub mod foot;
pub mod footlock;
pub mod ground;
pub mod facing;
pub mod gait;
pub mod getup;
/// A [`rig::RigGeometry`] parsed from a real glTF, for tests.
///
/// Test-only: it embeds a 31 KB asset and exists so tests can measure
/// against the rig the game ships rather than a hand-transcribed
/// approximation of it.
#[cfg(test)]
pub mod gltf_rig;
pub mod legik;
pub mod locomotion;
pub mod lookat;
pub mod pelvis;
pub mod slide;
pub mod math;
pub mod phase;
pub mod plugin;
pub mod poses;
pub mod ragdoll;
pub mod ragdoll_plugin;
pub mod reference;
pub mod retarget;
pub mod rig;
pub mod stance;
pub mod transition;
pub mod walk;
pub mod walk_balance;
#[cfg(feature = "anim_studio")]
pub mod studio;

pub use dho::{default_springs, DhoState};
pub use asset::{AnimAssetPlugin, PoseAsset};
pub use phase::{GaitPhase, PhaseClock, PhaseLayer, PhaseOscillator};
pub use plugin::{
    AnimArmIk, AnimPhaseLayer, AnimPlugin, AnimPose, AnimSet, AnimSprings, AnimTarget,
    AnimTargetAsset,
};
pub use poses::{relaxed_stand, rest, wave};
pub use ragdoll::{
    default_joint_limits, Fall, JointLimits, Ragdoll, RagdollStrength, Rise, Stun, StunResponse, FALL_DAMPING, FALL_TONE,
};
pub use ragdoll_plugin::{
    spawn_ragdoll, AnimRagdollPlugin, RagdollHit, RagdollSet, RagdollSpawnConfig,
};
pub use math::{InertializeCubic, Inertializer, RotationInertializer, SpringParams};
pub use retarget::write_pose_to_skeleton;
pub use rig::{BoneSet, LocalPose, BONE_COUNT};
