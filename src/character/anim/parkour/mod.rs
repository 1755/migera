//! Parkour moves for a platformer, built against the level's geometry
//! (`docs/knowledge/character-animation/parkour/`): so far, grabbing a
//! ledge, hanging from it and climbing up onto it ([`hang`]).

pub mod fall;
pub mod geometry;
pub mod hang;

pub use fall::Falling;
pub use geometry::{Ledge, LedgeGround};
pub use hang::Hanging;
