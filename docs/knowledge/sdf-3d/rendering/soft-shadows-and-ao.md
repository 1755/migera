---
title: Soft shadows and ambient occlusion from an SDF
description: Covers IQ's k-penumbra soft shadows, Aaltonen's anti-banding fix and 5-tap SDF AO, plus migera's src/hybrid findings — 1/t softness decay, slab-truncated marches, fixed PENUMBRA_REACH margin, k=2 via sweep, pad every BVH node, margin_fade. Read before touching trace_shadow or when shadows band, box or falsely darken.
type: concept
status: current
tags:
  - sdf
  - shadows
  - raymarching
  - hybrid-renderer
  - correctness
  - verification
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/hybrid/cpu_ref.rs
  - assets/shaders/hybrid_trace.wgsl
  - src/hybrid/extract.rs
sources:
  - https://iquilezles.org/articles/rmshadows/
  - commit 0f04320 (hybrid_legacy removed)
aliases:
  - penumbra
  - ambient occlusion
  - Aaltonen soft shadow
  - PENUMBRA_REACH
  - boxy shadow
---

# Soft shadows and ambient occlusion from an SDF

Contents: [Evolution of the technique](#soft-shadows-the-evolution-of-the-technique) ·
[Softness shrinks with distance](#softness-shrinks-with-distance-migeras-own-finding-not-in-iqs-or-aaltonens-material) ·
[Porting to src/hybrid](#porting-to-srchybrid-three-more-bugs-the-hybrid_legacy-port-didnt-warn-about) ·
[Interior penumbras](#handling-interior-penumbras-later-refinement) ·
[Ambient occlusion](#ambient-occlusion) · [Related](#related)

One of the most-cited practical advantages of SDF raymarching is that physically
plausible soft shadows and cheap ambient occlusion fall almost directly out of the same
distance-field machinery already used for primary visibility — no separate shadow-map
rasterization pass, no screen-space AO approximation from depth buffers.

## Soft shadows: the evolution of the technique

### Hard shadows (baseline)

A basic shadow ray marches from the shaded point toward the light using ordinary
[sphere tracing](./sphere-tracing.md); if it ever gets closer than a small epsilon to any
surface before reaching the light, the point is fully in shadow. This gives only hard,
binary shadow edges.

### Penumbra via the `k` parameter (the classic technique)

Inigo Quilez's widely-used soft shadow technique exploits information that's already
being computed during the shadow raymarch for free: at each step, the SDF value `h` tells
you how close the ray *almost* passed to an occluder even when it didn't actually hit
one. Converting that "near miss" distance into a penumbra darkening factor produces
continuous, physically-motivated soft shadows from a single ray per shaded point (not
multiple stochastic samples as area-light soft shadows would otherwise require):

```glsl
float softshadow(vec3 ro, vec3 rd, float mint, float maxt, float k) {
    float res = 1.0;
    float t = mint;
    for (int i = 0; i < 256 && t < maxt; i++) {
        float h = map(ro + rd*t);
        if (h < 0.001) return 0.0;
        res = min(res, k*h/t);
        t += h;
    }
    return res;
}
```

The parameter `k` controls shadow hardness and is conceptually tied to the inverse of the
light source's angular size: a larger `k` produces sharper, harder-edged shadows (a
smaller apparent light source); a smaller `k` produces broader, softer penumbras (a
larger light source).

### Fixing banding artifacts (Aaltonen's refinement)

The naive `k*h/t` formula above produces visible banding — discontinuities correlated
with the raymarch step positions themselves rather than a smooth function of geometry.
Sebastian Aaltonen's refinement estimates the true closest point *between* consecutive
marching samples (rather than only ever checking distance at exact step positions),
removing the banding at a modest extra cost per step:

```glsl
float softshadow(vec3 ro, vec3 rd, float mint, float maxt, float w) {
    float res = 1.0;
    float ph = 1e20;
    float t = mint;
    for (int i = 0; i < 256 && t < maxt; i++) {
        float h = map(ro + rd*t);
        if (h < 0.001) return 0.0;
        float y = h*h / (2.0*ph);
        float d = sqrt(h*h - y*y);
        res = min(res, d / (w*max(0.0, t-y)));
        ph = h;
        t += h;
    }
    return res;
}
```

### Softness shrinks with distance (migera's own finding, not in IQ's or Aaltonen's material)

(`src/hybrid_legacy` was deleted in commit 0f04320, 2026-09-12; its file and test names
below are history. The shipping implementation is `src/hybrid/cpu_ref.rs::trace_shadow`
and `assets/shaders/hybrid_trace.wgsl::trace_shadow`, verified 2026-09-28.)

Neither the classic `k*h/t` formula nor Aaltonen's `d/(w*(t-y))` refinement bounds how
the ratio behaves as `t` (distance already marched along the shadow ray) grows large
relative to the occluder's own size. Both are, dimensionally, `1/length` — for a **fixed**
`k`/`w`, the denominator scales linearly with `t` while the numerator (`h` or the
triangulated `d`) is bounded by roughly the occluder's own extent once the ray has passed
near it. The practical consequence, confirmed against migera's `src/hybrid` compute
raymarcher (a directional-light scene, one sphere on a ground plate) via a full-march
numeric simulator (not single-sample snapshots — see `src/hybrid/cpu_ref.rs`, the
permanent CPU reference oracle for this marching math, built out of this investigation):

- A shadow ray whose BVH candidate slab against a small compact occluder only starts
  (`obj_near`) at a large `t` — e.g. a ray that merely grazes the occluder's *bounding
  box* corner far from the ray's own origin, without ever converging on the occluder's
  true silhouette — reads dramatically darker than the identical physical gap would read
  if evaluated close to the ray's origin.
- Concretely: `k=12`, a ray whose true closest approach to a radius-1.2 sphere is `0.288`
  world units at `t≈2.36` gives `vis≈0.01` (effectively pitch black) — yet the *same*
  `k`, evaluated at `t≈0.9` for a genuinely-in-penumbra near-contact point with a larger
  gap (`h≈0.38`), gives `vis≈0.035`, barely brighter. No single `k` reconciles both: a `k`
  small enough to make the far/grazing case read plausibly lit (`k≈0.13-0.24` by direct
  calculation) makes the near-contact case read **fully lit** (`vis=1.0`), destroying the
  correctly-soft contact shadow.
- Visually, this produces a shadow whose penumbra near the occluder looks correctly soft
  and round, but whose outer reaches — precisely where a per-object BVH candidate's
  march terminates at its AABB slab boundary — cut off hard, tracing the **occluder's
  bounding box edges**, not its true (e.g. round) silhouette. This reads as a "boxy
  shadow" artifact and is easy to misdiagnose as an AABB-margin or candidate-clipping bug
  (migera's own investigation chased that exact false lead first, before a full-march
  numeric simulation revealed the real cause) — a targeted numeric simulation comparing a
  genuine near-contact ray against a genuine far-grazing ray, both against the exact same
  distance field, is what actually separates "this is a wrong-candidate-slab bug" from
  "this is the softness formula's own 1/t decay."

**Tried and reverted — a relaxing-hardness mitigation did not fix the visible artifact.**
An attempt was made to relax the effective hardness `k_eff` from the caller-tuned `k`
toward a much softer asymptotic value as `t` grows past a scene-scale-relative threshold
(smoothstep-blended so the transition has no visible kink), rather than using a single
fixed `k` for the whole ray:

```wgsl
let t_eff = max(t - y, 1e-4);
let relax_blend = clamp(t_eff / (RELAX_T0 * 3.0), 0.0, 1.0);
let relax_blend_s = relax_blend * relax_blend * (3.0 - 2.0 * relax_blend); // smoothstep
let k_eff = k + (K_FAR - k) * relax_blend_s;
let vis_sample = clamp(d / (k_eff * t_eff), 0.0, 1.0);
```

The full-march simulator showed this measurably brightening the far/grazing case
(near-parity with the near case in the simulator's test geometry: `0.10` vs `0.11`, vs.
`0.073` vs. `0.014` unrelaxed) without darkening the near-contact penumbra, and a
farther-still sanity check confirmed visibility kept improving monotonically with
genuine distance. **Despite that, the rendered result was judged still visibly wrong**
— the box-shaped edge was softer but still clearly present, and the numeric improvement
did not translate into an acceptable visual fix. This was reverted; `trace_shadow`
currently ships the plain, unmodified Aaltonen formula (`d / (k * max(t - y, 1e-4))`,
no relaxation). The lesson: a numeric simulator comparing isolated sample points is
useful for ruling out *wrong* hypotheses (it correctly showed the AABB-margin lead was a
dead end) but is not sufficient on its own to certify a fix — the rendered image is the
actual spec, and this fix was not re-validated against it carefully enough before being
reported as working.

**Follow-up investigation identified the actual mechanism, with a pinned regression
test.** The real bug is not (only) the formula's `1/t` decay — it's that the marching
loop's own bound is `t <= obj_far` (the BVH candidate slab's exit face), not the ray's
true limit (`max_t`, e.g. the light distance). A grazing ray whose slab only clips a
small corner of the occluder's AABB can be cut off after just ONE sample, taken far from
the object's true surface, before the soft-march formula (any formula) ever gets the
chance to converge or diverge. This is now a concrete, numerically pinned example:
probe `(729,380)` in the gallery sphere scene has `slab=[0.001, 0.4496]` — so short that
only the very first sample (`t=0.01, h=0.489`) fits before the slab boundary stops the
march, producing `vis=1.0` (fully lit) for a ray that is, per its own later true
closest-approach (`h≈0.30` at `t≈1.0`, found by letting the march run un-truncated),
genuinely NOT far from the sphere.

**Naively extending the march past `obj_far` is not a standalone fix — the two bugs are
coupled.** A follow-up experiment let the march continue to `max_t` with a divergence
early-out (stop once `h` grows well past the best value seen). This made the AABB-
truncated cases correctly darker/more-converged, but ALSO reintroduced the `1/t` decay
bug at a much larger scale: a ray verified to genuinely, monotonically diverge from the
sphere the whole way (`h` growing from `6` to `35` units, never converging) still read as
partially shadowed (`vis≈0.06`) once marched far enough, purely because `k*t` in the
denominator keeps growing while `h` stays large-but-finite. Extending the march widens
the *scope* over which the formula's own flaw applies, making a correctly-lit ray read
as falsely shadowed. **Any real fix has to address both together** — the march-bound
truncation AND the formula's distance-scaling — not one in isolation; a subsequent
attempt at a "true closest approach across the whole march, angular visibility from that
one point" reformulation also ran into a real problem (a purely angular test made even
the near-contact case read fully lit, since a modest absolute gap subtends a large angle
close to a small object) and was not resolved immediately.

**Resolved: both fixes landed together and are now validated in two independent
renderers.** The final fix combines the march-bound change above with a SECOND,
independently-necessary fix: padding every BVH leaf's AABB (not internal nodes) by
`margin = VIS_CUTOFF * k * scene_root_diagonal` before the ray/box slab test — without
this, a ray that never enters an occluder's *tight* AABB gets zero candidates for that
object at all, producing a polygonal (bounding-box-edge-shaped) shadow silhouette
regardless of the march-bound fix. `margin`'s derivation: the largest gap `h` between a
ray and an occluder for which `d/(k*t) == VIS_CUTOFF` at `t == scene_root_diagonal` (a
self-describing scene-scale reference requiring no per-scene tuning constant). Both
fixes shipped in `src/hybrid_legacy` first — see `hybrid_legacy_trace.wgsl::trace_shadow`
and its CPU reference `src/hybrid_legacy/cpu_ref.rs::trace_shadow`/
`shadow_candidate_margin`/`gather_candidates_padded`, cross-checked against real GPU
debug-buffer probes with three passing regression tests
(`real_near_contact_matches_shader_probe_630_405`,
`real_hard_hit_matches_shader_probe_600_380`,
`fixed_729_380_no_longer_truncated_by_short_aabb_slab` — the last one is the
`(729,380)` case above, its assertion flipped from the buggy `vis=1.0` behavior to the
fixed one once the real fix landed).

When `src/hybrid` (the fresh-start renderer that superseded `hybrid_legacy`) needed
shadows, this fixed/validated state — not the earlier buggy draft — was confirmed
directly (by reading both the code and this doc in full) before porting, since
`hybrid_legacy`'s shadow-quality investigation was part of the original motivation for
the rewrite. Ported into `src/hybrid/cpu_ref.rs::trace_shadow`/
`gather_candidates_padded` and `hybrid_trace.wgsl`, with its own
independent CPU-reference test suite reconstructing this same ground-plus-sphere scene
shape (see `PROGRESS.md`'s "Soft shadows — GI trajectory Stage A" entry).

### Porting to `src/hybrid`: three more bugs the `hybrid_legacy` port didn't warn about

Porting the fixed `hybrid_legacy` formula verbatim was necessary but not sufficient —
`src/hybrid` re-tested the technique at `--stress N` grid scale (tens to hundreds of
objects spread across a much larger world extent than `hybrid_legacy`'s original small
demo scene) and surfaced three additional, genuinely distinct bugs. None of these are
about the Aaltonen formula itself; all three are about how its free parameters
(`margin`, `max_t`, `k`) get derived relative to *scene* geometry versus *object*
geometry, and one is about the acceleration structure the shadow rays traverse. Each was
found by building a ground-truth CPU debug-scan test against the exact real demo scene
(same cell-center formula, same object half-extents, same light transform as
`examples/gallery.rs`) and asserting anomaly-detection bounds over dense point/angular
scans — **not** by iterating on screenshots. Two earlier fix attempts, made purely by
reasoning about the code and confirmed only by eyeballing a render, were later shown by
the ground-truth scans to not actually address the reported artifact; the eyeballed
"looks fixed" read was simply wrong. This is the methodological lesson worth keeping
above the three bugs themselves: **for a shading-correctness bug report on this
renderer, build the ground-truth CPU scan test first and get a numeric localization of
the anomaly, before attempting a fix** — a screenshot cannot distinguish "the artifact
moved" from "the artifact is gone," and confirms plausible-looking wrong fixes just as
readily as real ones.

1. **The margin/max_t reference distance must be a fixed, object-scale constant — never
   derived from scene extent or from a ray's own reach.** `hybrid_legacy`'s
   `margin = VIS_CUTOFF * k * scene_root_diagonal` is exactly right for a demo scene
   that's never resized, but `scene_root_diagonal` (the BVH root AABB's diagonal) grows
   with `--stress N`'s object count, silently shrinking the effective margin/max_t at
   the *same* `k` as more objects are added — the formula's self-describing intent
   ("no per-scene tuning constant") backfires once the "scene" itself becomes a
   parameter under test. A second attempt scaled the reference off each ray's own
   `max_t` (light distance) instead — still wrong, for the same reason: `max_t` is
   scene/light-placement-dependent, not object-scale-dependent. The fix that actually
   held: replace both with one **fixed constant sized to the renderer's own object
   scale**, independent of scene size and independent of ray reach
   (`src/hybrid/cpu_ref.rs`'s `PENUMBRA_REACH`, back-derived to match `hybrid_legacy`'s
   own validated margin at its `k`). General lesson: any formula parameter whose stated
   justification is "derived from the scene so it needs no tuning" is only safe if the
   thing it derives from truly doesn't change under the axis you intend to stress-test
   — check that explicitly, don't assume it from the derivation's own framing.

2. **A hardness/softness parameter (`k`) tuned for one renderer's demo scene is not
   automatically valid at a different object scale — re-derive it, don't copy it.**
   `hybrid_legacy`'s validated `k=12.0` default was tuned and tested only against its
   own small, fixed, never-stress-tested demo geometry. At `src/hybrid`'s actual
   `--stress N` object scale (objects ~0.8-2.4 units), the same `k=12` combined with the
   formula's inherent `1/t` distance decay (see "Softness shrinks with distance" above)
   produced confirmed false-darkening — genuine 1-unit-wide open gaps between unrelated
   objects reading ~95% shadowed with no occluder anywhere near the ray. Found the safe
   value with a real k-sweep test (`k ∈ {12,8,6,4,3,2,1.5,1}`, asserting worst-case
   visibility across the full stress grid) rather than guessing: `k<=2.0` shows zero
   false darkening across the whole grid; `k>=3.0` shows real false darkening. New
   default: `k=2.0`. Lesson: when porting a technique whose formula has a free
   "hardness" constant, re-run (or build) a sweep test against the *target* renderer's
   actual object/scene scale — don't carry the source renderer's tuned value forward as
   if it were formula-intrinsic.

3. **Padding must be applied to every BVH node, not just leaves — an internal node
   built from tight (unpadded) child bounds can prune a padded leaf's own valid
   candidate before the leaf test ever runs.** This is a distinct bug from #1/#2 above —
   it's about the acceleration structure, not the shadow formula's parameters — and
   produced a different symptom: not global false-darkening but a sharp, localized
   "cut" on one side of an otherwise-correct soft shadow (confirmed via a 64-sample
   angular ring-profile scan around the shadow silhouette: two rays ~5° apart, one
   found a valid padded candidate with correctly soft darkening, the immediate neighbor
   found *zero* candidates and stayed fully lit — an on/off cliff in candidate
   existence, not a soft transition). Root cause:
   `gather_candidates_padded`/`hybrid_legacy`'s original design pads leaf AABBs only
   (`pad = if is_leaf { margin } else { 0.0 }`) — deliberately, to keep internal-node
   culling tight. But an internal node's bounds are the union of its children's *tight*
   bounds, computed with no awareness that leaves get padded before the ray test. A ray
   whose true path only entered a leaf's padded margin region (missing the leaf's own
   tight box) can also miss that leaf's *parent's* tight box, and the whole subtree gets
   pruned during descent before the leaf's own (correctly padded) test is ever reached.
   The fix: pad **every** node uniformly by the same `margin`, both internal and leaf.
   This is provably still correct, not just a workaround that costs traversal
   tightness — since each leaf's own padding is bounded by `margin`, a parent box padded
   by the same `margin` still fully contains every padded descendant leaf bound, so no
   valid candidate can be pruned. General lesson for any BVH/acceleration-structure
   query that pads primitive bounds for a "near-miss" test (soft shadows here; likely
   relevant to Stage C's occupancy-grid secondary-ray work too): if leaves get padded
   before the geometric test, ancestor bounds built from the *unpadded* leaf geometry
   are not a valid conservative bound for the padded query, and must be padded too (or
   rebuilt from already-padded leaf bounds) — otherwise pruning during descent can
   silently discard leaves whose own (correct) padded test would have passed.

**A fourth, distinct issue found in the same investigation is not a bug in the shadow
technique at all: a hard visibility discontinuity at the padded-candidate-AABB
boundary itself.** A shaded point whose shadow ray falls just outside a padded
candidate's AABB gets `vis=1.0` unconditionally (the per-candidate loop body never
runs for that object); a point just inside gets a real computed `sample_vis` from the
Aaltonen formula — and nothing guarantees these two values are close to each other,
since "inside vs. outside a padded box" and "how dark the formula says this point is"
are unrelated conditions. Projected onto a curved (e.g. spherical) shaded surface, this
reads as a visible polygonal/hexagonal seam tracing the padded AABB's own edges —
confirmed by deliberately over-scaling the margin constant and watching the polygonal
edge grow proportionally larger. Fixed with an explicit continuity blend at the
boundary itself: `margin_fade = smoothstep(margin * 0.5, margin, h)`, blending
`sample_vis` toward `1.0` as the near-miss distance `h` approaches the padding margin,
so the candidate-existence boundary is smoothed rather than a hard cliff. General
lesson: **any padded/expanded acceleration-structure query used to approximate a
continuous field (like penumbra visibility) needs its own explicit continuity
smoothing at the pad boundary** — the padding fixes candidate existence (bug #3's
concern) but does nothing on its own to guarantee the *value* is continuous across
that boundary, and those are two separate correctness requirements.

**Tooling gap, not yet closed:** `src/hybrid` currently has no GPU debug-readback
tooling analogous to `hybrid_legacy`'s `src/hybrid_legacy/debug.rs` (a probe-buffer
mechanism that let specific pixel coordinates' full shading intermediate state be read
back and compared against the CPU reference, used heavily in the `hybrid_legacy`
investigation above). This investigation worked around the gap entirely with
scene-reconstructing CPU-side tests, which was sufficient here but is real
extra friction — worth building before the next hard-to-diagnose shading bug rather
than repeating the workaround indefinitely.

### Handling interior penumbras (later refinement)

A further refinement lets the ray continue marching *through* geometry it would otherwise
have hard-stopped at, tracking a signed (rather than clamped-to-zero) result, then
remapping through a smoothstep-like polynomial at the end — capturing subtler penumbra
detail from geometry the ray passes near even on the far side of a first occlusion. This
is a genuine quality/cost tradeoff most implementations only reach for when shadow
softness quality specifically matters (product visualization, hero shots) rather than
default real-time use.

## Ambient occlusion

The equivalent trick applies to ambient occlusion: sampling the SDF at a handful of small
fixed steps along the surface normal from a shaded point directly answers "how close is
the nearest other surface" without needing screen-space depth-buffer AO's inherent
limitations (no information about geometry outside the current view, banding from limited
sample counts in screen space). A simple weighted-sum approximation:

```glsl
float calcAO(vec3 pos, vec3 nor) {
    float occ = 0.0;
    float sca = 1.0;
    for (int i = 0; i < 5; i++) {
        float h = 0.01 + 0.12*float(i)/4.0;
        float d = map(pos + h*nor);
        occ += (h - d) * sca;
        sca *= 0.95;
    }
    return clamp(1.0 - 3.0*occ, 0.0, 1.0);
}
```

This is cheap (a handful of extra SDF evaluations per shaded point, no separate buffer or
blur pass) and, unlike screen-space AO, is not limited to what's visible on screen — it
naturally incorporates occluders behind or beside the camera's view, since it queries the
actual 3D scene function rather than a rasterized depth buffer.

## Why this matters relative to rasterization pipelines

In a conventional rasterized pipeline, soft shadows and AO require dedicated techniques
layered on top of the base renderer (shadow map filtering/PCSS for soft shadows,
screen-space or precomputed AO passes) — see how Bevy's rasterization pipeline handles
this in
[lighting-and-shadows](../../bevy-rendering/pbr-and-lighting/lighting-and-shadows.md) for
comparison. In an SDF raymarcher, both effects reuse the exact same "march toward a point
and read distance values along the way" primitive already needed for visibility, which is
part of why raymarched demos historically achieved visually rich lighting with
comparatively little code.

## When to dive in

- Implementing shadows/AO in a raymarcher for the first time → start with the classic `k`
  soft-shadow formula and the 5-tap AO loop above; add Aaltonen's anti-banding fix only if
  banding is actually visible in your scene.
- Seeing banding artifacts in soft shadows → apply the interpolated-closest-point fix
  rather than just increasing step count (which reduces but doesn't eliminate banding).
- Comparing SDF raymarching against a rasterization-based pipeline for lighting quality
  vs. cost → see
  [performance-characteristics](../performance-and-production/performance-characteristics.md)
  for the broader tradeoff analysis.

## Related
- [Sphere tracing](./sphere-tracing.md) — prerequisite: the march the shadow ray reuses.
- [Raymarching artifacts and fixes](./raymarching-artifacts-and-fixes.md) — deeper: symptom-first entries for the banding, boxy-shadow and false-darkening bugs above.
- [Shadow margin / VIS_CUTOFF leak](../../hybrid-architecture/gi-and-lighting/shadow-margin-vis-cutoff-leak.md) — same-trap: a later `trace_shadow` bug where margin_fade and VIS_CUTOFF returned a stale nonzero vis.
- [BVH deep dive](../../hierarchical-volumes/bvh-deep-dive.md#padded-expanded-queries-near-miss-soft-shadow-proximity) — deeper: why padded queries must pad every node.
- [DDGI any-hit occlusion](../../hybrid-architecture/gi-and-lighting/ddgi-any-hit-occlusion.md) — contrast: hard-occlusion rays in the same renderer.
- [Replicate the real frame loop in a unit test](../../engineering-practice/testing/replicate-the-real-frame-loop-in-a-unit-test.md) — same-trap: build the ground-truth CPU scan before trusting a screenshot.
- [A measurement of a broken system](../../engineering-practice/measurement/a-measurement-of-a-broken-system.md) — same-trap: `k=12` tuned on one scene did not survive a scale change.
- [Lighting and shadows in Bevy](../../bevy-rendering/pbr-and-lighting/lighting-and-shadows.md) — contrast: shadow maps/PCSS in a rasterizer.
