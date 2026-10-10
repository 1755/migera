//! The two clocks a camera runs on.
//!
//! Look input, recentre timers, mode blends and effects run on **real**
//! time: a paused or hit-stopped game must still let the player look around.
//! Follow springs run on **virtual** time, so the pivot freezes with the
//! world it follows.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Component, Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, Reflect)]
#[reflect(Component)]
pub struct CameraClock {
    /// Wall-clock seconds this frame.
    pub real_dt: f32,
    /// Game seconds this frame: 0 while paused, scaled in slow motion.
    pub virtual_dt: f32,
}

impl CameraClock {
    /// Both clocks advancing together, as in an unpaused game at speed 1.
    pub fn both(dt: f32) -> Self {
        Self { real_dt: dt, virtual_dt: dt }
    }
}
