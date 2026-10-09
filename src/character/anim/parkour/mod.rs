//! Parkour moves for a platformer, built against the level's geometry
//! (`docs/knowledge/character-animation/parkour/`): so far, grabbing a
//! ledge, hanging from it and climbing up onto it ([`hang`]).

pub mod along;
pub mod beam;
pub mod crawl;
pub mod faith;
pub mod fall;
pub mod geometry;
pub mod hang;
pub mod holds;
pub mod lean;
pub mod monkey;
pub mod perch;
pub mod pole;
pub mod precision;
pub mod skid;
pub mod spin;
pub mod springboard;
pub mod squeeze;
pub mod teeter;
pub mod underslide;
pub mod vault;
pub mod wall;
pub mod wallhand;

pub use fall::Falling;
pub use geometry::{Ledge, LedgeGround};
pub use hang::Hanging;
pub use pole::{Pole, Poling};
