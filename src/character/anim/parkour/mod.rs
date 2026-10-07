//! Parkour moves for a platformer, built against the level's geometry
//! (`docs/knowledge/character-animation/parkour/`): so far, grabbing a
//! ledge and hanging from it ([`hang`]).

pub mod geometry;
pub mod hang;

pub use geometry::Ledge;
pub use hang::Hanging;
