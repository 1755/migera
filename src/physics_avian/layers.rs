//! Collision layers shared across systems.
//!
//! A collider's `CollisionLayers::memberships` says what it is; a query's
//! `SpatialQueryFilter` mask says what it looks for. Static world geometry
//! keeps avian's `LayerMask::DEFAULT` membership.

use avian3d::prelude::LayerMask;

/// Static terrain a walker's steering looks for: floor, walls, ramps.
pub const TERRAIN_LAYER: LayerMask = LayerMask(1 << 1);

/// Geometry the camera neither collides with nor counts as an occluder:
/// foliage, thin poles, fences, other characters' capsules. A camera that
/// bounced off every enemy's legs in a tight fight is the failure this
/// avoids (Dark Souls III, AC Syndicate).
pub const CAMERA_TRANSPARENT: LayerMask = LayerMask(1 << 2);

/// The ragdolls' layer bits (one per ragdoll, round-robin).
pub use crate::character::anim::ragdoll_plugin::RAGDOLL_LAYER_POOL;

/// What a camera's collision looks for by default: everything except
/// camera-transparent geometry and ragdoll bodies.
pub const CAMERA_BLOCKERS: LayerMask =
    LayerMask(LayerMask::ALL.0 & !CAMERA_TRANSPARENT.0 & !RAGDOLL_LAYER_POOL.0);
