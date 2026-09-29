//! SDF world description: a procedural scene tree (see docs/knowledge/sdf-3d) that is
//! this demo's single source of truth for geometry, flattened into a GPU primitive
//! buffer every frame by `crate::raymarch` and evaluated live via sphere tracing.

pub mod assembly;
pub mod components;
pub mod primitives;
pub mod scene;
pub mod world;

pub use world::{AnimatedPillar, spawn_tile_cluster};
