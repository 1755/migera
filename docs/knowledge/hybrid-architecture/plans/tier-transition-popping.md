---
title: "Tier-transition popping: artifact taxonomy, root principles, and the error-model solution"
description: Design for switching between SDF, analytic and 3DGS tiers without popping — six artifact classes, error-model-derived thresholds from the pixel cone, stateless evaluation vs hysteresis-damped residency, one shared shading kernel. Never built; the tiers no longer exist. Read only if a multi-representation LOD returns.
type: design
status: archived
tags:
  - lod
  - hybrid-renderer
  - global-illumination
  - sdf
  - 3dgs
updated: 2026-08-23
verified: 2026-09-28
sources:
  - Evans, Learning from Failure (SIGGRAPH 2015)
  - https://iquilezles.org/articles/smin/
  - Majercik et al. 2019, DDGI (JCGT)
  - Igehy 1999, Tracing Ray Differentials
aliases:
  - LOD popping
  - hysteresis
  - cone-based tier selection
  - ray differentials
---

# Tier-transition popping: artifact taxonomy, root principles, and the error-model solution

> **Archived:** the three tiers this designs transitions between were never
> built in the `src/hybrid` rewrite. The analytic tier was retired, no 3DGS
> tier exists, and `docs/plan/hybrid-renderer-roadmap.md` (cited below) was
> deleted. Two parts outlived the plan: §2's smooth-min facts (k is the blend
> thickness and the AABB inflation; polynomial smin underestimates distance, so
> far-field constants must be lower bounds), and §6's GI plan, whose G0
> (temporal accumulation) and G1 (DDGI probe grid with Chebyshev visibility)
> were built in a different form. See the GI notes under Related.

Contents: [1 Taxonomy](#1-artifact-taxonomy--popping-is-six-distinct-problems) ·
[2 Evidence](#2-external-evidence-base) · [3 Principles](#3-root-principles) ·
[4 Seams](#4-why-seams-stay-sub-pixel-by-construction-the-a4-argument) ·
[5 Hysteresis](#5-hysteresis-spec-residency-tier-only) ·
[6 GI coupling](#6-gi-coupling--irradiance-caching-is-also-a-transition-medicine) ·
[7 Metrics](#7-validation-metrics--debug-views) · [Related](#related)

Closes the "known-unsolved territory" from `hybrid-renderer-hardening.md` with a design-grade
synthesis for migera's three tiers (SDF near ← analytic mid ← 3DGS far) plus the GI layer
added to scope afterwards. Everything here is written to be folded directly into milestone
designs (see `docs/plan/hybrid-renderer-roadmap.md`, stages 3/5/6).

---

## 1. Artifact taxonomy — "popping" is six distinct problems

Naming them precisely matters because each has a different owner and fix:

| ID | Artifact | Root cause |
|----|----------|-----------|
| **A1** | Geometry/silhouette shift at switch | Representations disagree on surface position (sampling density, march epsilon, blend hardening) |
| **A2** | Shading discontinuity at switch | Different normals/material/lighting inputs per tier |
| **A3** | Temporal flip-flop | Camera oscillating across a hard threshold, no memory |
| **A4** | Spatial seam sweeping across an object | Adjacent pixels evaluate the *same object* in different tiers; mismatch visible along the iso-threshold line |
| **A5** | Secondary-ray inconsistency | Tier chosen by *camera* distance, but a shadow/reflection ray's quality need depends on distance from its own origin — near-camera objects get coarse hits inside reflection views; far-from-camera objects get wasteful fine hits in shadow queries |
| **A6** | Depth/compositing conflict during overlap windows | Both representations write depth/color in the transition frame set; z-fighting or double-blend |

## 2. External evidence base

- **Solari dev log 0.19** (jms55.github.io): specular-GI "lighting can pop or shift as the
  path termination crosses a LOD boundary… smoothing out these transitions is still an open
  problem." Even Bevy's flagship RT renderer ships with this unsolved ⇒ we must engineer it
  explicitly, not inherit it.
- **Octree-GS (TPAMI 2025)** keeps intra-splat LOD stable via *observation-footprint*-based
  level selection + silhouette "LOD bias" — footprint (projected size), not raw distance,
  is the stabilizing metric.
- **StopThePop (Radl 2024)** separates *order* pops (sorting, far tier internal) from
  representation-change pops; only the latter concerns us here.
- **Dreams "Learning from Failure" (Evans, SIGGRAPH 2015)** — closest ancestor of our
  architecture. Two load-bearing lessons:
  1. Every attempt to mix an alpha/OIT-composited representation with opaque geometry
     ("volumetric billboards", "hybrid rasterised gigavoxel cubes") died on **OIT overlap
     correctness**, not performance. The shipped game renders everything opaque,
     depth-written, one representation per surface point at any time.
  2. Distant sculptures *are* rendered as baked surfel/imposter proxies against near SDF —
     our three-tier shape is production-validated; the transition is handled by distance-
     driven proxy selection with matched prefiltered appearance ("refinement renderer"
     experiments: prefiltered/blurred look reads as intentional, masking residual mismatch).
- **iq, "Smooth-minimum" (iquilezles.org/articles/smin/)** — newer than our KB's
  combination-operators notes, with three facts we build on:
  1. With his normalization, parameter **k equals blend thickness in distance units AND is
     exactly the bounding-box expansion needed to bound the smooth union** — i.e., k is
     simultaneously the CSG-error scale, the AABB inflation constant for BVH leaves, and
     the analytic-hardening deviation bound. One constant, three duties.
  2. The DD family (incl. our unnormalized quadratic polynomial smin, whose deviation from
     `min` is ≤ **k/4**, active only where |a−b| < k) **underestimates distance along an
     infinitely extending Voronoi-edge band**. Safe for sphere tracing (conservative),
     but means "far field ≈ min()" never becomes exact ⇒ any far-field constant culling
     (Lipschitz-pruning style) must store *lower-bound* constants — consistent with our
     phantom-shell history.
  3. The Circular Geometrical smin has *local support* but **overestimates** distance in
     convex regions ⇒ produces holes when stepped. Forbidden as a marching function in
     this project, regardless of its locality appeal.

## 3. Root principles

**P1 — Split evaluation tier from residency tier.**
- *Evaluation tier*: chosen per ray-hit, instantaneous, stateless, from the cone/error rule
  (P2/P3). Owns image quality. Cannot flicker — it has no memory and varies continuously
  with view geometry.
- *Residency/work tier*: per object per frame (which splat LOD buffer is bound, whether the
  object is in the near-set dispatch mask, whether prune trees are rebuilt). Owns cost.
  This is the only place hysteresis lives.
All six artifacts come from conflating these. Once split, A3 reduces to cost-policy
damping, and quality decisions become pure functions of view geometry.

**P2 — Error-model-driven thresholds, not tuned distances.**
Every representation carries a computable worst-case geometric error against the SDF
source of truth:

```
r_cone(t)        = t · tan(½·hfov)/res_h                      // primary-ray cone radius
e_sdf(t)         = γ · r_cone(t)                              // γ ≈ 0.5–1, march tolerance
e_analytic(obj)  = 0                                          // fully exact-solvable tree
                 | Σ_active kᵢ/4                              // hardened soft blends (our poly-smin)
e_splat(obj,lod) = spacing_lod/2 + c·κ_max·spacing_lod²       // MEASURED AT BAKE:
                                                               // project splat centers onto the
                                                               // zero isosurface, record max|f|
```

Selection rule: **cheapest T ∈ {splat, analytic, sdf} with e_T ≤ α · r_cone(hit)** (α≈1).
Consequences worth stating explicitly:
- Thresholds are *derived*, survive resolution/DRS changes, and need no per-scene tuning.
- The SDF↔analytic switch lands where r_cone ≈ Σkᵢ/4 — i.e., close enough that the fillet
  flattening is sub-cone and therefore invisible. Analytic wins on cost everywhere its
  provable deviation fits under the cone; SDF is demanded exactly when the cone is tighter
  than the blend error.
- The analytic↔splat switch lands where r_cone ≈ e_splat(lod) — splat sampling error is
  by definition invisible once the pixel cone covers it.

**P3 — Cone consistency across ALL rays (kills A5).**
Primary, shadow, and reflection rays all carry a cone origin + growth rate (ray-differential
lineage, Igehy 1999):
- primary: r₀=0, growth tan(½hfov)/res;
- reflection: r₀ = primary cone radius at the hit, same growth;
- shadow: cone widens toward the light; visibility is forgiving — use α_shadow > α (≈1.5–2).
Tier selection consults the *local* cone at each candidate hit t. Camera distance never
enters secondary evaluation.

**P4 — One shading kernel shared by every tier (kills most of A2).**
A single `#import`-able WGSL library: procedural-pattern eval + PBR (with the Solari
layering fixes: diffuse scaled by `(1−F_L)(1−F_V)`, Fresnel-weighted lobe selection, VNDF
sample validity rejection) + shadow-ray traversal against the same BVH buffers. Both the
compute pass *and the splat fragment shader import it*. Splats carry material IDs + world
pos + normal per fragment — they can run identical per-pixel patterns/lighting/shadows.
Tiers then differ ONLY in how (position, normal) are delivered. Dynamic lights behave
identically at every distance.

**P5 — Single-depth-writer discipline (kills A6).**
Exactly one participant writes depth per pixel per frame; ordered composite:
opaque splats (raster, hardware depth) → compute ray pass (reads splat depth as max-range +
occlusion, writes nearer hits) → transparent. During a cross-fade window (P7) the outgoing
tier keeps rendering but drops its depth write (color-only). This is the direct lesson from
Dreams' failed OIT hybrids and our own WSR postmortem.

**P6 — Silhouette/graze escalation.**
Errors concentrate at grazing angles where cones traverse longest relative to feature size.
Multiply the allowed error by a grazing factor g(N·V) ∈ [1, ~3] (equivalently divide cone
growth), or escalate one tier finer when min-recorded-distance ≪ r_cone. Cheap: grazing
pixels are a minority; Octree-GS's silhouette bias is the intra-splat analogue.

**P7 — Transition cosmetics: cross-fade first, dither behind a flag.**
- Cross-fade band around each derived threshold: both tiers evaluate, weight =
  smoothstep over |r_cone − threshold| < band. Outgoing side color-only (P5). Band cost is
  bounded by band pixel coverage — measure, keep band ≈ 1–2 px of travel.
- Blue-noise threshold dithering (per-pixel threshold offset, spatiotemporal noise → TAA
  resolves) only becomes attractive once motion vectors/TAA exist; behind a flag until then.

**P8 — Validate with metrics, not eyes.**
See §7. Debug views ship in Stage 2, metrics run from Stage 3 onward.

## 4. Why seams stay sub-pixel by construction (the A4 argument)

Adjacent pixels have adjacent cones; thresholds are continuous functions of r_cone; so the
iso-threshold line sweeps smoothly. At the crossing point, the finer tier's own error ≈ the
coarser tier's error bound ≈ α·r_cone — meaning the maximum geometric disagreement between
tiers at the seam is bounded by ~2α·r_cone(threshold). Because thresholds sit where r_cone
matches representation error (P2), that bound is ≈ the coarser tier's own reconstruction
error at that distance — i.e., the seam cannot reveal more than what the coarser tier
already shows elsewhere. Residual shimmer is then sampling noise, not structure — the kind
temporal accumulation absorbs. This is the central reason to prefer derived thresholds over
hand-tuned ones: hand-tuned thresholds break this invariant invisibly.

## 5. Hysteresis spec (residency tier only)

```
enter_coarse:  d_eff > d_hi   (d_eff = projected-footprint-derived distance proxy)
return_fine:   d_eff < d_lo,  d_hi − d_lo ≥ v_cam_max · Δt_frame · N_safety (+ jitter margin)
state:         per-object u32 in render-world table; overrides via RenderTierOverride pin
flap guard:    minimum dwell time (frames) before re-evaluation after a flip
```

Cost flicker (dispatch mask size, splat LOD swaps) is what hysteresis dampens; image quality
never depends on this state (evaluation is stateless per P1) — worst case of a residency
mistake is wasted work or a stale LOD for one frame, not a visual pop.

## 6. GI coupling — irradiance caching is also a transition medicine

Indirect light queried from a position+normal-keyed cache (probe grid) is inherently
representation-agnostic: whichever tier produced the hit, the indirect term is the same
lookup. That removes A2 for the entire indirect lobe and shrinks A1's visibility (silhouette
shifts matter less when shading matches). Direct lighting remains per-kernel (P4 already
unifies it). Chosen plan (stage 6):

- **G0 — single-bounce cosine-weighted indirect in the compute pass + temporal
  accumulation** (ping-pong accumulation, disocclusion via depth reprojection). Noisy;
  serves as the correctness reference that G1 is validated against.
- **G1 — DDGI-style probe clipmap**: grid of probes storing L1 SH RGB + mean distance/
  variance (Chebyshev visibility weight), updated each frame by tracer rays through the SAME
  BVH/SDF evaluator stack (tier-aware like every other ray), trilinear query with normal-
  offset bias; scrollable clipmap for large worlds. Emissive rings feed it naturally —
  probe rays gather emissive hits with zero extra code.
- Shared-kernel hook from day one: the PBR library takes an `indirect_irradiance` input
  that is a constant ambient until stage 6 swaps implementations — no call-site churn.

References: Majercik et al. 2019 (DDGI); NVIDIA RTXGI docs; Solari's world-cache posts
(light-leak rule "force finest when ray_t < cell_size" adopted in P6's escalation and in
query biasing).

## 7. Validation metrics & debug views

Metrics (CSV per run, scripted cameras):
- `pop_events`: count of frame-to-frame silhouette-edge displacements > τ px within ROI;
  target 0 at default settings over ≥10 dolly loops.
- `seam_px`: total length of detected discontinuity lines along iso-threshold contours.
- `rmse_vs_finest`: vs a forced-finest-tier reference orbit (quality ceiling check).
- `band_pixel_pct`, `double_eval_ms`: cross-fade cost accounting.
- Standard set: per-pass GPU ms (timestamp queries), fps p50/p1, steps/pixel histogram,
  rays/pixel by type, BVH node visits, probe-update ms (stage 6).

Debug heatmaps (extend legacy `debug_flags` mechanism): tier-id per pixel, cone radius,
error-budget slack (e_T/r_cone), fade weight, probe irradiance (stage 6).

## References

- JMS55, Realtime Raytracing in Bevy 0.19 — https://jms55.github.io/posts/2026-04-12-solari-bevy-0-19/
- Ren et al., Octree-GS — https://arxiv.org/abs/2403.17898
- Radl et al., StopThePop — https://arxiv.org/html/2402.00525v3
- Evans, Learning from Failure (SIGGRAPH 2015) — https://media.lolrus.mediamolecule.com/AlexEvans_SIGGRAPH-2015.pdf
- iq, Smooth-minimum — https://iquilezles.org/articles/smin/
- Igehy, Tracing Ray Differentials (SIGGRAPH 1999)
- Majercik et al., Dynamic Diffuse Global Illumination with Ray-Traced Irradiance Fields (JCGT 2019)

## Related
- [Hardening research for the tiered hybrid renderer](./hybrid-renderer-hardening.md) — prerequisite: the survey whose open problem (§6 there) this design closes.
- [Hybrid GI temporal accumulation](../gi-and-lighting/hybrid-gi-temporal-accumulation.md) — applies: what G0's temporal accumulation became.
- [DDGI any-hit occlusion](../gi-and-lighting/ddgi-any-hit-occlusion.md) — applies: the DDGI probe grid that G1 became, and its measured occlusion cost.
- [Why the analytic tier was removed](../../analytic-intersections/production-performance-finding.md) — contrast: the measurement that removed the mid tier this design assumes.
- [Combination operators](../../sdf-3d/primitives-and-operators/combination-operators.md) — deeper: the smooth-min family §2 relies on.
