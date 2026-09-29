//! Shape-keyed sample-point cache — Piece 5's answer to "sample points are
//! computed once per shape, not every frame." `sample_points.rs`'s own doc
//! comment confirms sample points are baked in LOCAL space and depend on
//! shape kind/parameters ONLY, never on a body's position/rotation/
//! velocity, so recomputing them every frame for every body is provably
//! wasted GPU work whenever a body's shape hasn't changed — the
//! overwhelming common case, since nothing in this codebase mutates
//! `PhysicsShape` after spawn today.
//!
//! Keyed by `PhysicsShapeGpu`'s raw bytes (via `bytemuck::bytes_of`), not a
//! derived `Hash`/`Eq` on the struct directly — `f32` has no `Eq`/`Hash`
//! impl (NaN inequality), and exact-bytes equality is exactly the
//! "identical shape kind and parameters" comparison this cache wants: two
//! shapes with bit-identical encodings always want identical sample
//! points, and a genuinely different parameter value (even a
//! floating-point ULP apart) simply misses and computes fresh, never
//! silently reusing a near-but-not-exact match.
//!
//! **Explicit tradeoff, not silently picked**: this cache never evicts — a
//! shape that stops being used by any body stays cached forever. Accepted
//! for v1 given realistic shape-kind/parameter cardinality in a real scene
//! is small even at 20,000 bodies (most bodies share a handful of shape
//! templates); revisit only if a future soak test shows unbounded growth
//! in a scene that spawns many genuinely distinct shape parameterizations
//! over its lifetime.

use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy::render::render_resource::PipelineCache;
use bevy::render::renderer::{RenderDevice, RenderQueue};

use super::buffers::{SamplePointsGpuState, ensure_sample_points_buffers};
use super::pass::{dispatch_physics_sample_points, read_buffer_sync};
use super::pipelines::SamplePointsGpuPipeline;
use super::types::{PhysicsShapeGpu, SamplePointsGpu};

/// Render-world resource: the shape-keyed cache itself. `None` entries are
/// never stored — a cache miss simply isn't in the map yet.
#[derive(Resource, Default)]
pub struct SamplePointsCache(HashMap<[u8; std::mem::size_of::<PhysicsShapeGpu>()], SamplePointsGpu>);

fn shape_key(shape: &PhysicsShapeGpu) -> [u8; std::mem::size_of::<PhysicsShapeGpu>()] {
    *bytemuck::bytes_of(shape).first_chunk().expect("PhysicsShapeGpu's byte size matches its own size_of exactly")
}

impl SamplePointsCache {
    pub fn get(&self, shape: &PhysicsShapeGpu) -> Option<SamplePointsGpu> {
        self.0.get(&shape_key(shape)).copied()
    }

    fn insert(&mut self, shape: PhysicsShapeGpu, points: SamplePointsGpu) {
        self.0.insert(shape_key(&shape), points);
    }
}

/// Diffs `shapes` against the cache's existing keys, dispatches
/// `physics_sample_points` for however many NEW (kind, params)
/// combinations appeared this call (one batched dispatch, not one per
/// shape), and inserts the results into the cache. Returns `false` if a
/// pipeline wasn't ready yet (same convention as every other dispatch
/// function in this port) — callers should treat this the same as "no
/// contact-gen input available this frame," not a hard error, since
/// pipeline-compile latency at startup is expected and transient.
///
/// Deliberately does NOT return the resolved per-body `SamplePointsGpu`
/// array itself — callers look those up via `SamplePointsCache::get`
/// AFTER calling this, so a cache hit (the common case, no new shapes)
/// never touches the GPU at all this frame.
pub fn ensure_sample_points_cached(render_device: &RenderDevice, render_queue: &RenderQueue, pipeline: &SamplePointsGpuPipeline, pipeline_cache: &PipelineCache, cache: &mut SamplePointsCache, shapes: &[PhysicsShapeGpu]) -> bool {
    let mut new_shapes = Vec::new();
    let mut seen_this_call = bevy::platform::collections::HashSet::new();
    for shape in shapes {
        let key = shape_key(shape);
        if cache.0.contains_key(&key) || !seen_this_call.insert(key) {
            continue;
        }
        new_shapes.push(*shape);
    }

    if new_shapes.is_empty() {
        return true;
    }

    if pipeline_cache.get_compute_pipeline(pipeline.pipeline).is_none() {
        return false;
    }

    let mut state = SamplePointsGpuState::default();
    ensure_sample_points_buffers(render_device, render_queue, &mut state, &new_shapes);
    dispatch_physics_sample_points(render_device, render_queue, pipeline, pipeline_cache, &state);

    let Some(buffers) = state.0.as_ref() else { return false };
    let results: Vec<SamplePointsGpu> = read_buffer_sync(render_device, render_queue, &buffers.outputs, new_shapes.len());
    for (shape, points) in new_shapes.into_iter().zip(results) {
        cache.insert(shape, points);
    }
    true
}
