//! Production non-blocking double-buffered readback — the one piece of
//! genuinely new infrastructure Piece 5 needed (every other piece of this
//! port's own math/dispatch plumbing already existed and is reused as-is).
//!
//! **Design, grounded in Bevy's own shipped precedent**: `bevy_render`'s
//! own `gpu_readback::GpuReadbackPlugin` (`bevy_render-0.19.1/src/gpu_readback.rs`)
//! is a complete, working, first-party non-blocking readback pattern for
//! this exact problem — not something this project has to invent from
//! scratch. Confirmed by reading its full source: `map_buffers` (in
//! `RenderSystems::Cleanup`, i.e. after this frame's command encoder is
//! already submitted) calls `slice.map_async(MapMode::Read, callback)` and
//! **never calls `RenderDevice::poll` itself, at any `PollType`** — the
//! callback fires because wgpu's native backends poll device callbacks
//! automatically as a side effect of other queue/device operations
//! happening every frame (submission, `Surface::present`, etc.).
//! `sync_readbacks` (an `ExtractSchedule` system — i.e. it runs once per
//! frame at the START of the NEXT frame's extract) does a non-blocking
//! `rx.try_recv()` and, if a result already arrived, applies it.
//!
//! This module follows that shape directly: `request_physics_readback`
//! (called at the end of this frame's dispatch chain, mirroring
//! `map_buffers`'s own placement) issues `map_async` and stores the
//! channel receiver PAIRED WITH the exact `Vec<Entity>` order that frame's
//! dispatch used (`PhysicsGpuReadbackState::pending`) — this pairing is
//! the actual "double buffering" that matters here, not just the
//! underlying GPU buffer's own ping-pong, since it's the only thing that
//! lets the next frame's apply system know which entity each result index
//! belongs to even if the entity SET changed in between.
//! `physics_gpu_apply_readback` (a main-world `PreUpdate` system, ordered
//! `.before(motion::update_previous_shape_transforms)` per that module's
//! own `PreUpdate`/`Update`/`PostUpdate` ordering contract) does the
//! non-blocking check and, on a hit, writes each body's result into its
//! own entity's `Transform`/`RigidBody` via `World::get_mut` — a plain
//! per-entity lookup, not a query over the CURRENT frame's body set, so a
//! despawned entity or one that lost its physics components between the
//! dispatch frame and this one is silently and correctly skipped
//! (`get_mut` returns `None` for a dead/mismatched entity, no panic, no
//! stale write). A newly-spawned entity that wasn't in the stored order
//! simply receives no write this frame — correct, since it wasn't part of
//! the dispatch that produced this result.
//!
//! Rejected: calling `RenderDevice::poll(PollType::Poll)` explicitly every
//! frame from a render-world system — `GpuReadbackPlugin`'s own working
//! code proves it's unnecessary, and adding an unneeded explicit poll call
//! risks diverging from a proven-correct, already-shipped pattern for no
//! benefit. If a future soak test ever shows the callback not firing
//! promptly under this project's own submission pattern, the fallback is
//! a single explicit `render_device.poll(PollType::Poll)` call
//! immediately after `map_async` in `request_physics_readback` — cheap
//! insurance, not built preemptively.

use std::sync::Mutex;
use std::sync::mpsc::{Receiver, TryRecvError, channel};

use bevy::prelude::*;
use bevy::render::render_resource::{Buffer, BufferAsyncError, BufferDescriptor, BufferUsages, CommandEncoderDescriptor, MapMode};
use bevy::render::renderer::{RenderDevice, RenderQueue};

use super::super::components::RigidBody;
use super::buffers::PhysicsGpuState;
use super::types::PhysicsBodyGpu;

/// One in-flight readback's own state: the entity order that dispatch
/// used, the channel the `map_async` callback will eventually fill, and
/// the staging buffer (kept alive until the result is consumed — dropping
/// it before the callback fires would be undefined per wgpu's own mapping
/// contract). `Receiver` is not `Sync`, but a `Resource` must be —
/// wrapped in a `Mutex` purely to satisfy that bound; this state is only
/// ever touched from render-world systems that already have exclusive
/// `&mut` access to the resource, so the mutex itself is never actually
/// contended.
type PendingReadback = (Vec<Entity>, Receiver<Result<(), BufferAsyncError>>, Buffer);

/// Render-world resource: the one in-flight readback, if any. `None`
/// means either no dispatch has ever run, or the previous readback was
/// already consumed by `sync_physics_gpu_readback`.
#[derive(Resource, Default)]
pub struct PhysicsGpuReadbackState {
    pending: Mutex<Option<PendingReadback>>,
}

/// Issues the non-blocking `map_async` request for this frame's
/// already-dispatched body-state buffer, pairing it with `dynamic_entities`
/// (the exact entity order this dispatch used) — called once, at the end
/// of the dispatch chain, after `dispatch_physics_apply_velocity` (the
/// substep loop's own last write into `PhysicsGpuState::bodies`) has
/// already submitted its command encoder. Overwrites any still-pending
/// readback from an earlier frame that was never consumed (can only
/// happen if `physics_gpu_apply_readback` somehow missed two frames in a
/// row, which would itself indicate a stall worth noticing elsewhere, not
/// a case this function needs to guard against specially).
pub fn request_physics_readback(render_device: &RenderDevice, render_queue: &RenderQueue, state: &PhysicsGpuState, dynamic_entities: Vec<Entity>, readback_state: &mut PhysicsGpuReadbackState) {
    let mut pending = readback_state.pending.lock().expect("readback state mutex should never be poisoned (never touched from more than one thread at a time)");

    let Some(buffers) = state.0.as_ref() else {
        *pending = None;
        return;
    };
    if dynamic_entities.is_empty() {
        *pending = None;
        return;
    }

    let byte_size = (dynamic_entities.len() as u64) * (std::mem::size_of::<PhysicsBodyGpu>() as u64);
    let staging = render_device.create_buffer(&BufferDescriptor {
        label: Some("physics_gpu_readback_staging"),
        size: byte_size,
        usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let mut encoder = render_device.create_command_encoder(&CommandEncoderDescriptor { label: Some("physics_gpu_readback_request_encoder") });
    // Only the DYNAMIC prefix of the bodies buffer is copied -- statics
    // never get a Transform write-back (see this module's own doc
    // comment), so there's no reason to transfer their bytes at all.
    encoder.copy_buffer_to_buffer(&buffers.bodies, 0, &staging, 0, byte_size);
    render_queue.submit(std::iter::once(encoder.finish()));

    let (tx, rx) = channel();
    let slice = staging.slice(..);
    slice.map_async(MapMode::Read, move |result| {
        // The receiver may already be gone (readback_state overwritten
        // before this callback fired, e.g. two frames' worth of dispatch
        // happened before either was consumed) -- dropping the result in
        // that case is correct, not an error.
        let _ = tx.send(result);
    });

    *pending = Some((dynamic_entities, rx, staging));
}

/// Main-world `PreUpdate` system: consumes `MainWorldPhysicsGpuResult`
/// (populated by `sync_physics_gpu_readback` below, which bridges a
/// render-world-produced result into the main world since `Extract<T>`
/// only flows main -> render, never the reverse) and — on a hit — writes
/// each body's result into its own entity's `Transform`/`RigidBody`.
/// Registered by `HybridRenderPlugin` (not `PhysicsPlugin`) alongside
/// `sync_physics_gpu_readback`, since the two are one conceptual unit
/// (the GPU path's own `Transform` write-back, the direct analogue of
/// `solve_world`'s own CPU write-back loop) even though this half runs in
/// `PreUpdate` on the main world. Must run
/// `.before(hybrid::motion::update_previous_shape_transforms)` (see this
/// module's own doc comment for why).
pub fn physics_gpu_apply_readback(world: &mut World) {
    let Some(result) = world.get_resource_mut::<MainWorldPhysicsGpuResult>().and_then(|mut r| r.0.take()) else {
        return;
    };
    let (entities, bytes) = result;
    let bodies: &[PhysicsBodyGpu] = bytemuck::cast_slice(&bytes);

    for (entity, body) in entities.iter().zip(bodies.iter()) {
        let Ok(mut entity_mut) = world.get_entity_mut(*entity) else {
            continue;
        };
        if let Some(mut transform) = entity_mut.get_mut::<Transform>() {
            transform.translation = body.position();
            transform.rotation = body.rotation();
        } else {
            continue;
        }
        if let Some(mut rigid_body) = entity_mut.get_mut::<RigidBody>() {
            rigid_body.linear_velocity = body.linear_velocity();
            rigid_body.angular_velocity = body.angular_velocity();
        }
    }
}

/// Main-world resource carrying a fully-resolved GPU readback result
/// across the render-world/main-world boundary, in the direction `Extract`
/// itself doesn't support (`Extract<T>` only flows main -> render).
/// Populated by `sync_physics_gpu_readback` (an `ExtractSchedule` system,
/// which — despite living in `ExtractSchedule` — has full `&mut World`
/// access to BOTH the render world it runs in AND (via
/// `Extract`-schedule's own documented main-world-mutation allowance,
/// the same mechanism `bevy_render::gpu_readback::sync_readbacks` itself
/// relies on via `ResMut<MainWorld>`) the main world, making it the
/// correct place to bridge a render-world-produced result back into a
/// main-world resource for `physics_gpu_apply_readback` to consume next
/// frame.
#[derive(Resource, Default)]
pub struct MainWorldPhysicsGpuResult(pub Option<(Vec<Entity>, Vec<u8>)>);

/// `ExtractSchedule` system (render-world side, but writes into
/// `MainWorld` — see `MainWorldPhysicsGpuResult`'s own doc comment):
/// non-blocking check via `try_recv()` on `PhysicsGpuReadbackState`'s
/// pending receiver; on a hit, maps the staging buffer's already-resolved
/// range (the callback already confirmed the map succeeded — `try_recv`
/// returning `Ok` IS the signal that `get_mapped_range` is now safe to
/// call), copies the bytes out, unmaps, and stashes the result plus its
/// paired entity order into `MainWorldPhysicsGpuResult`.
///
/// Confirmed via `SubApps::update`'s own source
/// (`bevy_app-0.19.1/src/sub_app.rs`): within one `App::update()` call,
/// the main app's OWN schedules (including `PreUpdate`, where
/// `physics_gpu_apply_readback` lives) run FIRST, and each sub-app's
/// `extract()` (which runs `ExtractSchedule`, hence this system) runs
/// AFTER. So a result written here during `update()` call N is only
/// visible to `physics_gpu_apply_readback` starting at call N+1's
/// `PreUpdate` — one extra `update()` of latency beyond the GPU's own
/// dispatch/readback latency, not the single frame the module doc
/// comment's own diagram might suggest at a glance. Not a bug: the
/// dispatch-to-apply latency being ">= 1 frame" rather than "exactly 1"
/// was already an accepted consequence of the design (see this module's
/// own doc comment on `PhysicsGpuReadbackState::pending` surviving until
/// consumed, however many frames that takes), confirmed correct by
/// Tier 1's own polling-based wiring test rather than a fixed frame-count
/// assertion (a fixed count was tried first and found too tight once this
/// exact ordering was accounted for).
pub fn sync_physics_gpu_readback(mut main_world: ResMut<bevy::render::MainWorld>, state: ResMut<PhysicsGpuReadbackState>) {
    let mut pending = state.pending.lock().expect("readback state mutex should never be poisoned (never touched from more than one thread at a time)");
    let Some((entities, rx, staging)) = pending.take() else { return };
    match rx.try_recv() {
        Ok(Ok(())) => {
            let data = staging.slice(..).get_mapped_range();
            let bytes = data.to_vec();
            drop(data);
            staging.unmap();
            if let Some(mut out) = main_world.get_resource_mut::<MainWorldPhysicsGpuResult>() {
                out.0 = Some((entities, bytes));
            }
        }
        Ok(Err(_)) => {
            // Buffer map failed -- drop this frame's result rather than
            // propagate a panic into the render schedule; the next
            // frame's dispatch will produce a fresh readback attempt.
        }
        Err(TryRecvError::Empty) => {
            // Not ready yet -- put it back and check again next frame.
            *pending = Some((entities, rx, staging));
        }
        Err(TryRecvError::Disconnected) => {
            // The map_async callback's sender was dropped without ever
            // sending -- can only happen if the staging buffer itself was
            // dropped first, which doesn't happen here (staging is moved
            // into `pending`, not dropped). Treat as "no result," the
            // same as a failed map.
        }
    }
}
