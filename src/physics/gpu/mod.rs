//! GPU port of the XPBD substep solver — see the plan's "Stage 3
//! remainder" section for the full design (buffer layout, pass breakdown,
//! CPU/GPU parity strategy, incremental landing order). This module tree
//! is render-world code: it's registered by `HybridRenderPlugin`
//! (`src/hybrid/mod.rs`), not `physics::integrate::PhysicsPlugin`, since
//! that plugin has only ever touched the main `App` — mirroring how
//! `ObjectGpu` lives under `src/hybrid/extract.rs` even though it mirrors
//! main-world ECS data, GPU-mirror structs live with the renderer that
//! consumes them, not the domain module they describe.
//!
//! Landed in 5 pieces (all done): predict-only buffers/dispatch (Piece 1),
//! contact-scatter atomics (Piece 2), full position-round apply (Piece 3),
//! velocity round + first CPU/GPU benchmark (Piece 4) — see
//! `parity_test.rs` for each piece's own isolated verification. The
//! broad-phase/contact-generation GPU port (a separate 4-piece sub-plan)
//! landed on top of this: GPU sample points, the Hillis-Steele scan, the
//! full spatial-hash broad-phase, and GPU contact generation.
//!
//! **Piece 5 (this module's newest code — `extract.rs`, `frame.rs`,
//! `readback.rs`, `sample_cache.rs`): end-to-end frame wiring.** This is
//! where every earlier piece's own independently-tested dispatch function
//! finally gets wired into a real per-frame system (`frame::dispatch_physics_gpu_frame`,
//! registered in `HybridRenderPlugin`'s `Render` schedule), behind the
//! `physics::integrate::PhysicsGpuEnabled` opt-in toggle (default OFF —
//! CPU `solve_world` remains the shipping default). `extract.rs` snapshots
//! the main-world physics body set into a render-world resource every
//! frame (`ExtractSchedule`); `sample_cache.rs` caches sample points by
//! shape identity rather than recomputing them every frame (they're
//! pose-independent); `readback.rs` implements the production
//! one-frame(+)-latency non-blocking readback, following
//! `bevy_render::gpu_readback::GpuReadbackPlugin`'s own shipped pattern
//! rather than inventing one from scratch.
//!
//! **Two real bugs found during this piece's own soak testing, both now
//! fixed** (see `frame.rs`'s own inline comments at each fix site for
//! full detail): the broad-phase's bucket-counts buffer and the contact-
//! generation cursor both need resetting EVERY substep (contacts and
//! broad-phase candidates are regenerated every substep, per
//! `solve_world`'s own doc comment on why), but the buffer-allocating
//! functions that also reset them (`ensure_broadphase_buffers`,
//! `ensure_contact_gen_buffers`) are only called once per frame to avoid
//! stomping GPU-computed state. Without the missing per-substep resets,
//! atomic accumulation compounded across substeps within a single frame,
//! corrupting the broad-phase's CSR ranges and producing contact counts
//! that grew unboundedly — measured as `dispatch_physics_contacts` itself
//! taking over 10 seconds by the 5th substep of a single frame, and
//! (before an earlier, separate fix in the same investigation) frequent
//! enough atomic contention to trigger a genuine AMD/radv GPU driver
//! context loss during extended runs. Also found and fixed: a leaked
//! `UniformBuffer` allocated fresh on every `dispatch_physics_extract_positions`
//! call (moved to a persistent field, matching every other per-dispatch
//! uniform in this port), and the per-substep synchronous contact-count
//! CPU readback replaced with a 4-byte GPU-to-GPU copy
//! (`copy_contact_count_into_scatter_uniform`) plus fixed-capacity
//! dispatch sizing, eliminating a full-device-drain `poll(wait_indefinitely())`
//! that ran 8 times per frame.
//!
//! **Re-scoped to visual-effects-only (2026-09)**, after `avian3d`
//! (`crate::physics_avian`) was adopted as this project's primary physics
//! engine (see `crate::physics`'s own module doc comment for the full
//! build-vs-buy comparison that motivated this): this module tree is no
//! longer "the GPU port of the primary solver," it's a GPU compute path
//! for large counts of simple, individually-inconsequential debris bodies
//! where GPU throughput is the actual goal. `extract.rs` now reads
//! `effects::GpuDebrisBody` instead of the demoted CPU engine's own
//! `RigidBody`/`Inertia`, and kinematic-body support was dropped entirely
//! (`BodyKind`, the dynamic-vs-kinematic contact-generation range — see
//! `extract.rs`'s own doc comment for why: interactive/moving-platform
//! bodies are now `avian3d`'s responsibility, a debris chunk has no
//! reason to ever be "kinematic"). Every dispatch function/WGSL shader
//! downstream of `extract.rs` needed NO changes — they only ever consume
//! `RenderPhysicsGpuFrame`'s own already-GPU-typed fields, never the CPU-
//! side ECS component types `extract.rs` itself queries.
//! `examples/physics_gpu_debris.rs` is this re-scoped path's own showcase.

pub mod buffers;
pub mod effects;
pub mod extract;
pub mod frame;
pub mod pass;
pub mod pipelines;
pub mod readback;
pub mod sample_cache;
pub mod types;

#[cfg(test)]
mod frame_test;
#[cfg(test)]
mod parity_test;
