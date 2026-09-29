//! Standard humanoid skeleton for procedural character animation, built on
//! Bevy's ordinary `Transform` hierarchy and PBR mesh rendering (no SDF, no
//! custom GPU pipeline — see `docs/knowledge/` for why the from-scratch
//! renderer stopped being the target for characters). Procedural animation
//! is driven by [`anim`], which springs each bone's local **rotation**
//! toward an authored target and lets position fall out of Bevy's own
//! transform propagation (see that module's doc comment for why rotation
//! rather than position is the simulated quantity).
//!
//! The bone set/hierarchy/naming follows the Mixamo-compatible standard used
//! across the industry (Mixamo itself, Unity's Humanoid rig, UE's Mannequin
//! all converge on this shape) so that a real skinned glTF character can
//! later be dropped in without changing any animation code: PascalCase bone
//! names, Y-up/-Z-forward, T-pose rest, minimum 15-bone hierarchy. See
//! `Bone` for the exact list.

pub mod anim;
pub mod skeleton;

pub use anim::{relaxed_stand, rest, wave, AnimPlugin, AnimTarget, LocalPose};
pub use skeleton::{Bone, BoneMarker, HumanoidSkeleton, Side};
