---
title: Module and stage skeleton
description: Early structural map of src/hybrid — module roles, the MainPass-only ordering contract (after opaque, before transparent; never Prepass), the hit/shade and occupancy extension seams, the BVH evidence, and why an inner AABB was declined. Module map predates most features. Read before adding a stage to src/hybrid.
type: decision
status: stale
tags:
  - hybrid-renderer
  - render-pipeline
  - spatial-acceleration
  - bounding-volumes
updated: 2026-09-06
verified: 2026-09-28
code:
  - src/hybrid/mod.rs
  - src/hybrid/pipeline.rs
  - src/hybrid/pass.rs
  - src/hybrid/bvh.rs
aliases:
  - module map
  - stage ordering contract
  - inner AABB
  - hit/shade split
---

# Module and stage skeleton

> **Stale:** written 2026-09-06 when `src/hybrid` was mostly stubs. Since then
> soft shadows, three Bevy light types, reflections/refraction, DDGI and other
> GI methods, temporal accumulation, render scale, DoF and a post pass have
> landed, adding modules not listed below (`conetrace_ref`, `ddgi_ref`,
> `dof_ref`, `grain_ref`, `material`, `motion`, `post`, `radiance_cascades_ref`,
> `reflect_ref`, `refract_ref`, `taa_ref`, `temporal_ref`). `src/hybrid_legacy`
> and `docs/plan/hybrid-renderer-roadmap.md`, both cited below, were deleted.
> Still true (checked 2026-09-28): `hybrid_pass` runs in
> `Core3dSystems::MainPass`, nothing uses `Core3dSystems::Prepass`, and the
> inner-AABB reasoning holds. Use PROGRESS.md for what exists.

Contents: [Module map](#module-map) · [Stage ordering](#stage-ordering-contract) ·
[Extension seams](#two-extension-seams-documented-but-not-built) ·
[AABB/BVH](#aabbbvh-confirmed-real-win-with-an-honesty-caveat) ·
[Inner AABB](#inner-aabb-considered-and-declined) · [Naming](#naming-conventions) ·
[Future features](#future-features-with-no-structure-yet--deliberately) · [Related](#related)

The structural map of `src/hybrid` as durable knowledge — what's there,
why it's shaped this way, and where future work should land. See
[Self-shading vs. G-buffer-writer decision](./self-shading-vs-gbuffer-decision.md)
for the architecture choice this skeleton is built on.

## Module map

- **`mod.rs`** — `HybridRenderPlugin`, schedule registration.
- **`pass.rs`** — the one system registered into the schedule; owns the
  compute-dispatch-then-blit call sequence once `pipeline.rs` has real
  pipelines to dispatch.
- **`pipeline.rs`** — pipelines, bind-group layouts, buffers, scratch
  textures. The one connected pipeline `hybrid` owns end-to-end.
- **`extract.rs`** — main-world -> render-world extraction (scene data,
  debug flags, lights).
- **`scene.rs`** — ECS `SdfSceneRoot` trees -> flat object/record list for
  GPU upload.
- **`bvh.rs`** — acceleration structure over object AABBs.
- **`cpu_ref.rs`** — CPU-testable reference oracle for non-trivial march/
  shading math, proven with `cargo test` before porting to WGSL.

Each of `pipeline.rs`/`extract.rs`/`scene.rs`/`bvh.rs`/`cpu_ref.rs` mirrors
a same-named module in `src/hybrid_legacy` (the proven precedent for this
exact self-shading architecture) — port the *pattern*, not the code
verbatim, since the new implementation earns its own `cpu_ref`
verification this time rather than inheriting `hybrid_legacy`'s bug
history along with its shape.

## Stage ordering contract

`hybrid_pass` runs in `Core3d`/`Core3dSystems::MainPass`, ordered
`.after(main_opaque_pass_3d).before(main_transparent_pass_3d)`. This
mirrors `hybrid_legacy::pass::hybrid_pass`'s own documented reasoning:
`Core3dPlugin` chains only opaque->transparent by default, so anything
else registered into `MainPass` must pin itself against *both* ends of
that chain, or risk running before opaque content is drawn (composing
into a still-clearing target) or after transparent content expects to be
the last thing drawn.

This is deliberately **not** `Core3dSystems::Prepass` — that ordering
belongs to the G-buffer-writer architecture that was considered and set
aside (see the decision doc). `grep -rn "Core3dSystems::Prepass"
src/hybrid` should always return zero hits; if it doesn't, something has
drifted back toward that rejected direction without a fresh decision.

## Two extension seams, documented but not built

- **Hit/shade split**, inside `pipeline.rs`. Splitting primary-hit data
  (depth/t, normal, material id) from a separate shading dispatch is a
  measured, real win for this class of renderer —
  `docs/plan/hybrid-renderer-roadmap.md` recorded p50 148.8ms (brute
  force) -> p50 8.5ms after adding a BVH + this kind of restructuring at
  a far-orbit camera, a **17.6x** improvement (see the AABB/BVH section
  below for the honesty caveat on what that number bundles together).
  Fully compatible with self-shading — both dispatches would stay inside
  `hybrid`'s own pipeline, no Bevy G-buffer involved. Not split into
  separate `trace.rs`/`shade.rs` files yet, because the hit-buffer layout
  depends on a real feature (e.g. half-res secondary rays with bilateral
  upsample) that doesn't exist yet; when one does, the split lands as two
  dispatches inside `pipeline.rs` first, and the module split follows
  once the shape is proven, not before.
- **Occupancy-first-pass gate**, also inside `pipeline.rs`. See
  `docs/knowledge/hierarchical-volumes/occupancy-first-pass-design.md`'s
  three-level bitfield hierarchy (EMPTY/FULL/MIXED cells, GPU work-
  manifest + indirect dispatch) — would become an additional upstream
  binding the trace dispatch consumes, gating which pixels actually
  march. Doesn't change the stage boundary. Not built now; no concrete
  cell-size/query-API decision has been made for this renderer yet.

## AABB/BVH: confirmed real win, with an honesty caveat

A correct, complete AABB/BVH implementation (whole-ray "does this hit
anything at all" reject, then per-object marching bounded strictly to its
own `[near, far]` slab interval) is a large, real performance win here,
grounded in this project's own measured history — not a marginal
optimization or a nice-to-have. The 17.6x figure above bundles the BVH/
interval-skip change together with a fragment->compute migration and
marching-algorithm upgrades in the same roadmap stage, so it isn't a
clean BVH-only isolated A/B measurement. But the roadmap's own stated
reason for *why* far cameras cost so much more in the first place —
empty-space marching toward the horizon dominating cost — is exactly the
failure mode a whole-ray AABB reject test attacks directly, so the BVH/
AABB change is reasonably expected to be doing the bulk of that specific
win, even without an isolated measurement to prove the exact split.

See `docs/knowledge/aabb-acceleration/INDEX.md` and
`docs/knowledge/hierarchical-volumes/INDEX.md` for the construction/
traversal tradeoffs (median-split now, SAH as a later upgrade), and
`hybrid_legacy`'s own real bug history for a standing caution: three
separate bugs there were margin/bound-miscalibration bugs, not algorithm
bugs (a shadow march terminating at a slab's `far` bound before reaching
the true closest-approach point; an AO/indirect-light sample margin fixed
too short for how far those probes actually reach; marching a per-object
interval through a *merged* cross-object interval list instead of the
object's own slab). Any future "how far can this ray/probe legitimately
reach" cutoff must be an explicit parameter tied to the caller's real max
reach, never a hardcoded constant.

## Inner AABB: considered and declined

Whether adding an *inner* bound — a region guaranteed fully inside a
shape, letting a ray take one large guaranteed-safe jump on entry,
mirroring how an outer AABB lets a ray skip empty space — would add a
further large win was researched and **declined**, not overlooked.
Standard sphere tracing is already self-terminating: the moment a march
sample lands inside a solid shape, the SDF value crosses to non-positive
(or the march is defined to stop at `d < ε`), so a ray already stops in
O(1) steps on crossing the surface. It never marches *through* solid
interior the way it marches through empty exterior space, so there is no
wasted interior-marching cost for an inner bound to eliminate — the
technique's value proposition only pays off where marching actually
happens, and marching doesn't happen inside a hit solid under this
renderer's stop-on-hit model. The only place it could plausibly matter is
a narrower CSG subtraction/intersection case (a ray needing to know it's
"deep inside" one operand while still marching to find a different
operand's boundary), and nothing in this project's current scene
authoring concretely motivates building for that case. Per the project's
own documented caution — the reverted analytic-intersection tier, a real
regression from adding structure that didn't earn its keep — this is not
added to the skeleton or its future-work list without new evidence
changing the picture.

## Naming conventions

- `HybridRenderPlugin`, `hybrid_pass()` — already correct, carry forward.
- Future WGSL shaders: plain `hybrid_trace.wgsl`/`hybrid_blit.wgsl` are
  free to reuse (the legacy shaders are already prefixed
  `hybrid_legacy_*` in `assets/shaders/`, so there's no collision). If/
  when the hit/shade split happens, decide then whether it's two entry
  points in one file or a second file (`hybrid_shade.wgsl`) — correctly
  deferred, not decided now.
- Future resources: `HybridPipeline`, `HybridBuffers`,
  `HybridComputeBindGroup`, `HybridBlitBindGroup`, `HybridViewBindGroup`,
  `RenderHybridScene` — direct carry-overs of `hybrid_legacy`'s names, no
  collision since module paths disambiguate.

## Future features with no structure yet — deliberately

None of the following have a file, stub, or reserved name anywhere in
`src/hybrid` yet. This is deliberate, not an oversight — building
speculative structure for a feature with no concrete shape is exactly
what this project's own roadmap history (the reverted analytic-
intersection tier, a real 5-6x regression) warns against. Add structure
for each only once it has a concrete shape:

- **Soft shadows** — extends the eventual shading code inside
  `pipeline.rs`'s compute dispatch; no separate module.
- **Multiple lights** — extends `extract.rs` (query all real Bevy light
  component types, see
  [Bevy-native integration](./bevy-native-integration.md)) and the
  shading code; no separate module.
- **Multi-bounce reflections** — extends the shading code inside
  `pipeline.rs`; may motivate the hit/shade split above if reflection
  rays want to reuse hit data at reduced resolution.
- **Distance-based LODs at different resolutions** — extends `scene.rs`/
  `bvh.rs`; no splat/quad-tier revival without a fresh decision (see
  `scene.rs`'s own doc comment).
- **Upscalers** — a downstream post-process stage, not part of `hybrid`'s
  own pipeline; hooks into Bevy's own `EarlyPostProcess`/`PostProcess`
  sets via `FullscreenMaterialPlugin`, available regardless of which
  shading architecture `hybrid` uses.
- **Frame generation** — no KB documentation exists for this technique
  yet beyond TAA/motion-vector reprojection material; would need its own
  research pass before any structure is worth reserving.
- **AI-driven output improvements** — same post-process extension point
  as upscalers if it's a post-process filter; if it's something reading
  pre-shading buffers, Option B's freedom to define custom intermediate
  formats (see the decision doc) is what makes that tractable without a
  `bevy_pbr`-shaped constraint in the way.

## Related
- [Self-shading vs. G-buffer-writer decision](./self-shading-vs-gbuffer-decision.md) — prerequisite: the architecture this skeleton is built on.
- [Bevy-native integration](./bevy-native-integration.md) — deeper: how extraction and post-processing relate to Bevy.
- [Render scale subsumes half-res reflections](./performance-findings/render-scale-subsumes-half-res-reflections.md) — applies: the half-res secondary-ray feature the hit/shade seam was waiting for, measured and not built.
- [Occupancy first-pass design](../hierarchical-volumes/occupancy-first-pass-design.md) — deeper: the occupancy gate seam.
- [BVH deep dive](../hierarchical-volumes/bvh-deep-dive.md) — deeper: construction and traversal behind the BVH section.
- [Why the analytic tier was removed](../analytic-intersections/production-performance-finding.md) — prerequisite: the "structure that didn't earn its keep" precedent cited twice here.
