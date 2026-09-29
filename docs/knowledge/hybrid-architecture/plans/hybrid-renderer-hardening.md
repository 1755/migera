---
title: Hardening research for the tiered hybrid renderer (SDF / analytic / 3DGS)
description: August 2026 survey of pitfalls for a three-tier SDF/analytic/3DGS renderer: sphere-tracing accelerators, Lipschitz pruning, WGSL fma not fused, storage-texture access limits, BVH refit, tier pops. The M1–M6 tier plan is dropped; SDF, compute and BVH findings hold. Read before optimizing the march or BVH.
type: design
status: stale
tags:
  - hybrid-renderer
  - raymarching
  - spatial-acceleration
  - gpu-compute
  - state-of-the-art
  - lod
updated: 2026-08-23
verified: 2026-09-28
sources:
  - Keinert et al. 2014, Enhanced Sphere Tracing
  - Barbier et al. 2025, Lipschitz Pruning (CGF)
  - https://jms55.github.io/posts/2026-04-12-solari-bevy-0-19/
  - https://www.w3.org/TR/WGSL/
aliases:
  - M1-M6 milestones
  - three-tier renderer
  - enhanced sphere tracing
  - over-relaxation
  - WGSL fma
---

# Hardening research for the tiered hybrid renderer (SDF / analytic / 3DGS)

> **Stale:** the three-tier plan this survey feeds was never built in the
> rewrite. The analytic tier was retired after measuring 5–6x the cost of
> marching, no 3DGS tier exists in `src/hybrid`, `docs/plan/hybrid-renderer-roadmap.md`
> and `src/hybrid_legacy` were deleted, and on 2026-09-21 characters moved to
> Bevy's PBR pipeline. Sections 1 (SDF stepping), 4 (compute gotchas), 5 (BVH
> refit) and 7 (light transport) remain valid external research for the SDF
> renderer; sections 2, 3 and 6 and every *(Mx)* milestone tag are moot.
> Checked 2026-09-28 against PROGRESS.md: none of §1's step accelerators
> (over-relaxation, Lipschitz pruning) have been built.

Contents: [1 Near tier](#1-near-tier-sdf-sphere-tracing--sota-step-accelerators) ·
[2 Mid tier](#2-mid-tier-analyticquadric--precision--composition-hardening) ·
[3 Far tier](#3-far-tier-3dgs--lod-structure--sorting-pitfalls) ·
[4 Compute](#4-compute-execution-model--gotchas-that-will-bite-us) ·
[5 BVH](#5-bvh-maintenance--refitrebuild-policy-maps-to-our-two-level-plan) ·
[6 Tier transitions](#6-tier-transitions--pop-mitigation-is-genuinely-unsolved-sota-wide) ·
[7 Light transport](#7-light-transport-specifics-shadowsmulti-lightreflections) ·
[8 References](#8-reference-index-one-line-each) · [Related](#related)

External state-of-the-art survey (Aug 2026) feeding the rebuild defined in this repo:
three representation tiers (3DGS far ← analytic/quadric mid ← SDF near), BVH-class
acceleration, compute-resident rays, deep Bevy 0.19 integration. Each finding lists the
**gotcha/pitfall/lesson**, and where it lands in our milestone plan
(M1 primitives · M2 compute spine · M3 analytic tier · M4 light transport · M5 3DGS tier · M6 cutover).

Complementary to (does not repeat): `docs/knowledge/compute-shaders/raymarching-via-compute.md`
(compute-vs-fragment economics, phased pass split), `docs/knowledge/aabb-acceleration/*`
(slab tests), `docs/knowledge/hierarchical-volumes/*` (traversal algorithms),
`docs/knowledge/analytic-intersections/*` (intersectors, interval CSG),
`sdf-3dgs-bevy-integration/*` (bake + phase-item integration).

---

## 1. Near tier (SDF sphere tracing) — SOTA step accelerators

### 1.1 Enhanced Sphere Tracing (Keinert et al., STAG 2014)

https://diglib.eg.org/items/8ea5fa60-fe2f-4fef-8fd0-3783cb3200f0 — PDF mirror:
https://kev.town/raymarching-toolkit/media/enhanced_sphere_tracing.pdf

Five techniques, all directly applicable to our near tier:

1. **Safe over-relaxation**: multiply march steps by ω ∈ [1, 2] (they use ω = 1.6);
   valid because consecutive empty spheres must merely *overlap*. On failure detection
   (step overshoots), fall back to the last pre-jump position and finish classically.
   Cheap ~30-50% step-count reduction with zero tuning surface beyond ω. *(M2/M4)*
2. **Dynamic self-intersection prevention** for secondary rays: instead of a fixed
   epsilon, track a running minimum distance `t_min`; start secondary (shadow/
   reflection) marches at the hit point offset by the *pixel-footprint-scaled* error,
   controlling both precision and acne. *(M4)*
3. **Convex-bounding-volume optimization**: when an object sits inside a convex bound
   (our per-object AABBs!), enter at the slab-entry t and skip all interior empty-space
   stepping logic — Hart-style. This composes exactly with our BVH leaf intervals:
   march only inside `[t_near, t_far]`. *(M2)*
4. **Screen-space fallback candidate**: when max-steps is exhausted, pick the position
   of the *minimum recorded distance* along the ray instead of discarding the pixel —
   kills the thin green silhouette fringe artifact. We currently show sky; this is
   strictly better and free. *(M2)*
5. **Fixed-point discontinuity reduction**: 2-3 iterations of
   `p' = p - d·(f(p) − err(|p−o|))·n̂` after hit to remove stair-stepping at silhouettes;
   also reduces procedural-texture swimming. *(M2/M4, optional)*

### 1.2 Successor steppers

- **Accelerating Sphere Tracing** (Bálint & Valasek, EG Short 2018):
  https://diglib.eg.org/items/ce5ffffe-d78a-4d90-8ace-5184cc0f336c — replaces fixed ω
  with a *local linear approximation* of the field to compute the optimal next step;
  relaxation makes it safe for arbitrary geometry. Also: **multi-resolution rendering**
  (start at half/quarter res, refine only pixels still marching) — pairs perfectly with
  our compute internal-resolution plan *(M2)*.
- **Segment tracing** (Galin et al. 2020): computes *local Lipschitz bounds along ray
  segments*, taking longer steps than sphere tracing without backtracking — but requires
  per-primitive/per-operator Lipschitz knowledge (we have that: every primitive we ship
  is analytically Lipschitz-1; smin is too; domain repetition must clamp). Candidate
  upgrade once profiles show marching dominates *(M4+ stretch)*.

### 1.3 Lipschitz Pruning (Barbier et al., CGF 2025) — the big one for M2

https://onlinelibrary.wiley.com/doi/10.1111/cgf.70057

State of the art for exactly our shape: tree-based CSG SDFs of *thousands of
primitives, dynamically animated*, sphere-traced in real time (1080p on an RTX 4060,
with shadow rays + AO + AA):

- Precomputes **per-grid-cell pruned copies of the CSG tree** that provably evaluate to
  the same distance inside that cell — primitives whose influence region doesn't reach
  the cell are dropped. Compatible with hard AND smooth operators (unlike BlobTree
  bbox pruning, which needs compact support).
- **Far-field culling**: cells far from the zero isosurface collapse their subtree to a
  single constant that is a *lower bound* of the true distance — still a safe sphere-
  tracing bound, constant-time evaluation. This is the formal version of our bounding-
  sphere early-out, and it avoids our phantom-shell failure mode by construction
  (lower-bound constants, never fake distances).
- GPU build, 4-level grid hierarchy (4³ → 256³) — i.e., shallow-N³-tree topology,
  independently confirming our KB's VDB-style recommendation over deep octrees.
- Rebuilds on-the-fly per frame for animation ⇒ our `AnimGroup` rigid transforms fit
  (prune in rest pose, transform rays/groups like today).

Lesson for us: **the BVH/AABB layer and the pruned-tree layer are the same idea at two
granularities** — object-level AABB intervals for skipping whole objects (M2), cell-level
pruned trees for cheap `map()` evaluation inside an object (M4+). Design the GPU record
layout so both can share the same primitive encoding.

---

## 2. Mid tier (analytic/quadric) — precision & composition hardening

### 2.1 WGSL `fma` is NOT guaranteed fused — KB correction

WGSL spec (`fma`, §15.7.4 Floating Point Accuracy): implementations **may legally
implement `fma(e1,e2,e3)` as ordinary multiply followed by ordinary add** — i.e.,
two roundings, no fusion guarantee (Vulkan only got optional correct fusion via
`VK_KHR_shader_fma`, Oct 2025). Consequence: the Kahan/FMA discriminant trick from
`analytic-intersections/robustness-and-limits.md` **cannot be relied upon in WGSL**.
Use the *algebraic* fixes that don't need fusion: q-formulation (`x² = c/q`) and
NVIDIA's geometric reformulations; treat fma as a bonus, never a dependency. *(M3)*

Related C++-world lesson (KDAB FMA woes): compilers fusing `a*a - b*b` into
`FMA(a,a,-b*b)` can push `sqrt` negative where scalar math never would — i.e., even
"better" precision changes branch outcomes. Expect CPU-reference vs WGSL mismatches in
edge-case unit tests; compare with tolerances derived from ulp analysis, not exact bits.

### 2.2 Hybrid SDF+mesh production precedent

Interplay of Light "Deferred SDF rendering":
https://interplayoflight.wordpress.com/2017/12/12/deferred-signed-distance-field-rendering/
— SDF content written into a deferred g-buffer **with depth output**, sorting correctly
against rasterized meshes and unlocking stock SSR/SSAO/bloom. Confirms our composite
strategy: whatever evaluates hits (compute rays or analytic intervals), *writing depth*
is what buys coexistence with Bevy's mesh/splat passes and its post stack. *(M2)*

---

## 3. Far tier (3DGS) — LOD structure & sorting pitfalls

### 3.1 Octree-GS (TPAMI 2025) + Hierarchical 3DGS — canonical far-tier LOD

- Octree-GS: https://arxiv.org/abs/2403.17898 (code: https://github.com/city-super/Octree-GS)
- Kerbl et al., Hierarchical 3DGS: ACM TOG 43(4) 2024.

Key transferable facts: anchors in octree voxels; **LOD selected dynamically from
observation footprint** (projected size), not just distance; progressive coarse-to-fine
training keeps levels consistent; per-view rendered-Gaussian counts stay flat across
zoom-out trajectories. For our forward bake: generate 2-3 baked LOD densities per object
cluster, select by projected footprint each frame, and bias selection toward finer
levels near silhouettes (their "LOD bias" fights edge blur). *(M5)*

### 3.2 Sorting popping is measurable and fixable — StopThePop (Radl et al. 2024)

https://arxiv.org/html/2402.00525v3

Global mean-depth sort produces documented popping/view-inconsistency; per-pixel optimal
contribution depth `t_opt` + tiny insertion-sort window (16–24 entries) removes most
visible artifacts at modest cost. If our translucent splat path ever shows order pops,
this is the reference fix; opaque depth-tested content (our default) is immune. *(M5)*

### 3.3 Depth-tested splats ↔ opaque scene: confirmed pattern

Three independent implementations converge with our KB's correction-of-record
(`render-integration/sort-free-compositing.md`): mkkellogg/GaussianSplats3D explicitly
supports only depth-writing opaque objects alongside a normal scene ("transparent
objects more challenging"); rerun-io reached the same conclusion in design discussion;
Sort-free WSR (https://arxiv.org/html/2410.18931v1) positions WSR strictly as the
translucent/order-independent niche. New since our KB: **StochasticSplats** (stochastic
rasterization, unbiased MC estimator, sorting-free) — watch, not adopt. *(M5)*

### 3.4 Ecosystem references for wgpu-native splats

- mosure/bevy_gaussian_splatting (Apache/MIT): planar buffers, f16 clouds, GPU radix
  sort port (Fuchsia), LOD + temporal sorting experiments — API reference material, but
  currently tracks bevy 0.16-0.17, so expect extraction/phase API drift vs our 0.19.1.
- KeKsBoTer/web-splat + `wgpu_sort`: standalone WebGPU radix sort crate (indirect
  dispatch support) if we need GPU-side depth sorting for translucent splats. *(M5)*

---

## 4. Compute execution model — gotchas that will bite us

1. **Storage texture format matrix** (WebGPU/wgpu): `read_write` access exists ONLY for
   `r32float/r32sint/r32uint`; `rgba16float` bindings are single-access (write XOR read)
   per bind group entry. Practical consequences for the compute spine *(M2)*:
   - HDR color out: `texture_storage_2d<rgba16float, write>` + a *separate* sampled
     view/texture for the blit pass — do NOT assume in-place read-modify-write. The
     robust patterns are double-buffered ping-pong (Bevy's own game-of-life example) or
     `copy_texture_to_texture`.
   - Depth/ray-t buffer: r32float CAN be `read_write` — usable for Hi-Z style
     min/max mips built and consumed entirely in compute.
   - Claybook precedent (Aaltonen GDC 2018,
     https://media.gdcvault.com/gdc2018/presentations/Aaltonen_Sebastian_GPU_Based_Clay.pdf):
     DX11.1's missing typed UAV load on R8_UNORM forced them to indirect-dispatch tile
     copies into a temp buffer — the same class of constraint; check format access
     tables BEFORE designing an in-place update.
2. **Claybook measured facts worth stealing**: world marcher is *fetch-bandwidth bound*
   at 1024×1024×512 R8 with 5 mip levels (max step doubles per mip); sparse-virtual
   indirection cost measured at 13% slower steps — pay it only when residency demands;
   brush composition = small baked volumes + translate/rotate/uniform-scale only
   (non-uniform scale deliberately excluded — it breaks the distance property).
3. **Workgroup size for ray workloads**: Solari switched world-cache raytracing update
   groups from 1024 → 64 threads for a large win; matches our KB's single-wave guidance
   for fluctuating-loop kernels. Start 64-thread (or 8×8) groups; measure. *(M2)*
4. **Divergence economics**: naive per-pixel compute tiling measured ~2× slower than
   fragment shading in a controlled study (already in KB); the wins come from structural
   changes — work queues via atomic counters + indirect dispatch, half/quarter-res
   secondary passes with bilateral upsample, temporal reuse. Plan M2 to include the
   pass-split (hits pass → shading pass) from day one; retrofitting costs more. *(M2)*
5. **Windows TDR**: long single dispatches risk driver reset on Windows; budget
   dispatches well under the timeout and prefer many small indirect dispatches (Godot
   docs' warning is the canonical citation). *(M2)*
6. **Bevy 0.19 regression watch**: `prepare_preprocess_bind_groups` +33% frame cost
   reported in https://github.com/bevyengine/bevy/issues/24448 (GPU bin unpacking change).
   Profile before assuming our regressions are ours. *(M6)*

---

## 5. BVH maintenance — refit/rebuild policy (maps to our two-level plan)

Sources: NVIDIA RTX best practices (https://developer.nvidia.com/blog/rtx-best-practices);
Kopta et al., "Fast, Effective BVH Updates for Animated Scenes" (2012,
https://hwrt.cs.utah.edu/papers/hwrt_rotations.pdf); beefed.ai construction overview.

- **Refit-only degrades silently**: bounds stay valid but SAH quality collapses under
  motion ⇒ traversal cost balloons. Tree rotations folded into refit approach the
  quality of full SAH rebuild at <2× refit cost — the recommended default for animated
  scenes.
- **Two-level BLAS/TLAS model** is the industry answer and fits us exactly *(M2)*:
  - Bottom level: per-object BVH over the object's CSG leaves, rebuilt only on
    authored/topology change (SAH, offline-ish).
  - Top level: instances = (object AABB × current `GlobalTransform`), refit/rebuilt per
    frame cheaply — rigid `AnimGroup`/Transform animation never touches bottom levels.
- **Builder choice tradeoff**: Morton/LBVH builds ~an order of magnitude faster but up
  to ~85% slower traversal in pathological cases vs SAH. Static worlds: SAH. Hot-rebuild
  paths: LBVH. Measure effective rays/sec on representative primary+shadow+reflection
  ray mixes, not node count.
- **Layout**: wide nodes (BVH4/BVH8) in SoA with 16B-aligned bounds for whole-node
  cache-line fetches; stackless restart-trail (Laine 2010) if register pressure from
  the stack hurts occupancy in WGSL (no recursion, short-stack already planned).

---

## 6. Tier transitions — pop mitigation is genuinely unsolved SOTA-wide

Evidence that this needs explicit engineering time, not a hand-wave *(M3/M5)*:

- Solari 0.19 dev log (https://jms55.github.io/posts/2026-04-12-solari-bevy-0-19/):
  specular GI "lighting can pop or shift as the path termination crosses a LOD boundary…
  smoothing out these transitions is still an open problem" — even in Bevy's flagship RT
  renderer.
- Octree-GS handles it with progressive training + footprint-based continuous LOD bias;
  StopThePop removes order-pop but not representation-change-pop.

Hardening plan distilled from all sources:
1. **Match shading across tiers**: same PBR evaluate function fed by (material, normal,
   position) regardless of tier — pops then come only from geometry detail, not lighting
   discontinuities. Solari's BRDF-layering fix (double Fresnel `(1-F_L)(1-F_V)` on
   diffuse; Fresnel-weighted lobe selection; dedicated mirror BRDF instead of
   roughness-clamp hacks; VNDF sample validity rejection) should be THE shared shading
   kernel for all three tiers *(M4)*.
2. **Hysteresis bands** wider than worst-case per-frame camera translation × safety
   factor; switch on the *coarser* side while moving inward.
3. **Distance-based cross-fade window** (alpha blend old/new tier color over a short
   travel band) as the fallback where hysteresis alone still pops; splat↔geometry fades
   are alpha-friendly because both write depth-consistent colors.
4. **Light-leak analogue**: Solari's cache fix — force finest representation when
   `ray_t < cell/object size` — translates to: never let a mid/far-tier approximation
   shade a pixel whose ray travels less than the tier's feature size; escalate to the
   finer tier locally. *(M4)*

---

## 7. Light transport specifics (shadows/multi-light/reflections)

- Soft-shadow estimator (`res = min(res, k·h/t)`, k≈8–32): known artifacts and cures —
  self-shadow acne → origin offset along normal (Keinert's dynamic variant preferred);
  penumbra banding from oversized steps → clamp step length (also stabilizes the h/t
  ratio curve); directional parallel-ray striping → jitter origin along light dir +
  temporal accumulation (RTSDF, https://ar5iv.labs.arxiv.org/html/2210.06160);
  thin-geometry leaks → the bias itself widens penumbrae (accept, or special-case thin
  features); estimator widens penumbra but under-estimates umbra vs area-light truth —
  document as accepted approximation (Aaltonen's "penumbra widening"). *(M4)*
- Many lights: brute-force loop breaks down quickly (Solari's 441-emissive stress test
  needed RIS + is moving to world-space light grids — froxel binning's RT cousin, used
  shipped in DOOM: The Dark Ages and RE Requiem). Our scale (<32 lights, direct-only)
  stays a simple uniform loop now; keep the light list behind an interface so a grid
  binning pass can slot in later. Shadow-ray budget: cap shadow-casting lights per
  object/material flag rather than globally. *(M4)*
- Reflections: trace through the same BVH/tier evaluator as primaries (single code
  path), clamp bounce count to 1 initially; rough reflections → GGX VNDF sampling needs
  the validity check (reject z≤0 samples — Solari bug report above) and, if denoising/
  TAA enters later, remember mirror-guide-buffer issues (PSR technique). *(M4)*

---

## 8. Reference index (one line each)

- Keinert et al. 2014, Enhanced Sphere Tracing — over-relaxation, convex-bound accel,
  fallback candidate: https://diglib.eg.org/items/8ea5fa60-fe2f-4fef-8fd0-3783cb3200f0
- Bálint & Valasek 2018, Accelerating Sphere Tracing — linear-approx steps, progressive
  res: https://diglib.eg.org/items/ce5ffffe-d78a-4d90-8ace-5184cc0f336c
- Barbier et al. 2025, Lipschitz Pruning (CGF) — per-cell pruned CSG trees + far-field
  constant culling, GPU, animated: https://onlinelibrary.wiley.com/doi/10.1111/cgf.70057
- RTSDF 2022 — SDF soft shadows vs shadow maps, artifacts catalog:
  https://ar5iv.labs.arxiv.org/html/2210.06160
- Aaltonen 2018, Claybook GDC slides — volume-SDF production numbers, async compute,
  format/UAV traps: https://media.gdcvault.com/gdc2018/presentations/Aaltonen_Sebastian_GPU_Based_Clay.pdf
- Octree-GS TPAMI 2025 — anchor-octree LOD Gaussians:
  https://arxiv.org/abs/2403.17898
- Radl et al. 2024, StopThePop — per-pixel splat depth sorting:
  https://arxiv.org/html/2402.00525v3
- Sort-free GS via WSR 2024 — translucent niche confirmation:
  https://arxiv.org/html/2410.18931v1
- StochasticSplats — sorting-free stochastic rasterization (watchlist):
  https://ieeexplore.ieee.org/document/11444599/
- JMS55 Solari dev logs (0.18/0.19) — BRDF layering, light leaks, LOD-transition pops,
  WG sizing: https://jms55.github.io/posts/2026-04-12-solari-bevy-0-19/
- NVIDIA RTX best practices — refit degradation, update scheduling:
  https://developer.nvidia.com/blog/rtx-best-practices
- Kopta et al. 2012 — refit+tree rotations for animated BVHs:
  https://hwrt.cs.utah.edu/papers/hwrt_rotations.pdf
- WGSL spec fma accuracy clause — fusion not guaranteed: https://www.w3.org/TR/WGSL/
- WebGPU storage texture limits (read_write = r32* only):
  https://webgpufundamentals.org/webgpu/lessons/webgpu-storage-textures.html
- Interplay of Light, deferred SDF rendering — depth-output coexistence pattern:
  https://interplayoflight.wordpress.com/2017/12/12/deferred-signed-distance-field-rendering/
- bevy_gaussian_splatting / web-splat / wgpu_sort — wgpu splat ecosystem references.

## Related
- [Tier-transition popping](./tier-transition-popping.md) — deeper: the design synthesis that closes §6's open problem.
- [Why the analytic tier was removed](../../analytic-intersections/production-performance-finding.md) — contrast: the measurement that retired the mid tier of this plan.
- [migera moves characters to Bevy's PBR pipeline](../migera-pivot-to-bevy-pbr-for-characters.md) — contrast: the decision that ended the hybrid renderer as the main renderer.
- [Trace-pass bottleneck is not march steps](../performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — contrast: measured evidence that §1's step-count accelerators would not help on the dev GPU today.
- [Raymarching via compute](../../compute-shaders/raymarching-via-compute.md) — prerequisite: the compute-vs-fragment economics this survey builds on.
- [BVH deep dive](../../hierarchical-volumes/bvh-deep-dive.md) — deeper: construction and traversal behind §5.
