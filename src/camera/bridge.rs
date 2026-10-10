//! The walker bridge: the only place the camera knows about
//! [`WalkerState`].
//!
//! It fills a walking character's [`CameraTargetState`] (grounded, facing)
//! each frame. The camera reads the character's position from its root
//! `Transform`, not from `WalkerState`: on a ragdoll fall the root is moved
//! with the fallen body in `PostUpdate` (`RagdollSet::ReadBack`), and
//! `follow_the_fallen_body` copies only its x/z into `locomotion.position`.

use super::components::CameraTargetState;
use crate::character::anim::WalkerState;
use bevy::prelude::*;

/// Grounded and facing from the walker's own state.
pub fn bridge_walkers(mut walkers: Query<(&WalkerState, &mut CameraTargetState)>) {
    for (state, mut target) in &mut walkers {
        let airborne = state.jump.as_ref().is_some_and(|jump| jump.airborne())
            || state.falling.as_ref().is_some_and(|falling| falling.airborne());
        target.grounded = !airborne;
        // The walker's yaw shares the camera's convention: about +Y, zero
        // along −Z.
        target.facing_yaw = Some(state.facing.yaw);
    }
}
