---
title: "Compositing: Weighted Sum Rendering vs. real depth-tested opaque rendering"
description: Records why Weighted Sum Rendering was wrong for opaque splats in migera's deleted renderer (depth-test-without-write needs a bias no constant can satisfy; three variants failed) and the fix, depth write plus alpha-test, with Transparent3d for translucent splats. Read before compositing splats or soft primitives.
type: lesson
status: archived
tags:
  - 3dgs
  - bevy
  - rasterization
  - render-pipeline
  - correctness
updated: 2026-08-31
sources:
  - commit eac219e (WSR replaced by two-pipeline opaque/translucent)
  - commit 7ce7c9e (opaque depth write + alpha-test restored)
  - commit 684490c (splat renderer removed)
aliases:
  - WSR
  - weighted sum rendering
  - order-independent transparency
  - OIT
  - depth bias
  - popping
---

# Compositing: Weighted Sum Rendering vs. real depth-tested opaque rendering

> **Archived (2026-09-28):** `src/splat/render.rs` and `SplatPipeline` were deleted in commit 684490c (2026-09-06); the lesson — opaque content needs a real depth write, not an order-independent blend with a tuned depth bias — still holds for any future splat or soft-primitive pass.

Contents: [One-sentence version](#the-one-sentence-version) · [What happened](#what-actually-happened-in-order-for-anyone-tempted-to-retread-this-path) · [When WSR is right](#when-wsr-is-still-the-right-answer) · [Opaque alternative](#proven-alternative-for-opaque-content-real-depth-tested-rendering) · [What is given up](#what-is-given-up-wsr-translucent-case-only) · [When to dive in](#when-to-dive-in)

> **Updated from a working, proven implementation** (the `migera` demo in this
> repository). The original version of this page recommended Weighted Sum Rendering
> (WSR) unconditionally as *the* answer for splat compositing in Bevy. Building a real
> demo with genuinely opaque, disjoint solid objects (a ground plane, several
> primitives, sampled from an SDF) proved that recommendation wrong for that content
> class, in a way that cost multiple rounds of debugging before the root cause was
> understood. This page now documents both the original reasoning (still correct for
> its actual target content) and the failure mode discovered — read the whole page
> before choosing an approach, not just the "when to dive in" section.
>
> **Second correction, found later (2026-08-31), same lesson repeated**: despite this
> page already recommending real depth-tested opaque rendering, `src/splat/render.rs`'s
> actual `SplatPipeline` had drifted back to `depth_write_enabled: false` with
> `BlendState::PREMULTIPLIED_ALPHA_BLENDING` — the exact depth-test-without-write,
> blend-based approach this page says to avoid for opaque content — with **no
> back-to-front instance sorting anywhere in the live render path** (a
> `create_depth_sorted_indices` helper existed but only in the *dead*, never-wired-in
> `src/splat/plugin.rs` extraction path). The visible symptom was reported as "too
> sparse," "far splats bigger than they should be," and "solid objects don't look
> solid" — i.e. exactly what you'd expect from unsorted alpha blending of soft Gaussian
> falloffs: overlapping splats compositing in arbitrary bake order instead of correct
> depth order, and a translucent/blended edge reading as gappy rather than solid.
> Fixed by switching to `blend: None` + `depth_write_enabled: true` +
> alpha-*test* (`discard` below a density threshold) in the fragment shader instead of
> alpha blending — see "Proven alternative for opaque content" below, now cross-checked
> against the actual shipped WGSL/pipeline code, not just this page's prose. The
> practical lesson: **a knowledge-base page describing a fix is not evidence the fix is
> still in the code** — re-verify against the current pipeline/shader source before
> relying on a "proven" claim here, the same discipline point 7 of the parent
> `sdf-3dgs-bevy-integration/INDEX.md` already asks for.

## The one-sentence version

**WSR solves a problem — popping artifacts from sorting translucent, reordering
Gaussians — that a scene made of opaque solid geometry does not have.** Applying it
anyway (specifically: depth-testing without depth-writing, which is what WSR requires
to stay order-independent) introduces a *different*, harder problem: same-surface
neighboring splats need a depth-tolerance bias to avoid rejecting each other, and that
bias has no single value that's correct at every camera angle and every surface
orientation — pushing the bias in world space along each splat's own normal projects
to a wildly different effective clip-space tolerance depending on the angle between
that normal and the view direction, so a value tuned to fix one artifact
(cross-object bleed-through) reliably breaks something else (a formerly-solid surface
going patchy/black) at some other camera angle. This was not a tuning mistake — three
independent bias formulations (world-space along the normal, then view-space linear
depth, at multiple magnitudes) were tried in the demo before the actual fix was
identified: **the technique itself was wrong for this content, not the constant.**

## What actually happened, in order (for anyone tempted to retread this path)

1. Built the demo with WSR: a `Splat3d` `BinnedPhaseItem`, an accumulation texture
   (`sum(alpha·w·color)`, `sum(alpha·w)`), a resolve pass dividing to get final color,
   and — because WSR alone has no occlusion between *distinct* objects at all, only
   order-independence *among* whatever contributes to one pixel — a separate depth-only
   prepass plus a depth *test* (not write) in the accumulate pass, so only near-surface
   splats would contribute to the sum.
2. That depth test needed a bias: splats sampled directly onto a curved/thin SDF
   surface (no deliberate thickness) naturally have slightly different true depths
   from their neighbors, and a bias-free test rejected valid same-surface
   contributions almost at random — a visibly patchy/net-like surface.
3. Fixing the bias for one problem (a torus tube rendering as mostly-invisible)
   broke another (the ground plane's whole interior going black) at a different
   camera angle, because the bias was applied in world space along each splat's own
   (varying) surface normal, which doesn't map to a consistent clip-space depth
   delta across different surface orientations relative to the camera.
4. Moving the bias to view-space linear depth (angle-independent by construction)
   fixed the systematic version of the bug, but a thin remaining artifact (a hairline
   seam at a hard geometric edge — see
   [world-representation](../architecture/world-representation.md) for why hard SDF
   edges are a genuine gradient discontinuity) still needed a separate fix
   (rounding the edge in the SDF itself, not a rendering-side patch).
5. **Stepping back and asking "does this content actually need WSR at all"** — not
   "how do I tune the bias better" — was the fix that actually ended the whack-a-mole.
   The scene's splats are all fully opaque (`alpha ≈ 1` at the core). Real single-pass
   opaque z-buffering (depth test *and* write, exactly like ordinary alpha-tested mesh
   geometry) needs zero bias constants, because it's the standard, universally-proven
   technique every rasterizer already uses for exactly this correctness problem.

## When WSR is still the right answer

The original reasoning below is **not wrong in general** — it's wrong as a default for
opaque content specifically. WSR remains the correct choice when:

- Content is genuinely translucent/soft-edged and *many* overlapping instances need to
  composite without a per-frame sort (the reference 3DGS use case: photographic
  radiance-field capture, soft volumetric detail, thousands of overlapping
  semi-transparent Gaussians where exact per-pixel ordering is both expensive to
  compute and visually unnecessary).
- The popping artifact (a visible discrete jump as two overlapping Gaussians' relative
  depth order flips) is an actual, observed problem for the target content — which
  requires genuine mutual overlap and reordering under camera motion, not just "the
  scene has more than one splat."

### Why the reference 3DGS rasterizer's sorting doesn't fit cleanly into Bevy's phase model

The reference 3DGS [tile-based rasterizer](../../3dgs/rendering-and-rasterization/tile-based-rasterizer.md)
depends on a **global per-tile depth sort** recomputed every frame as the camera moves —
this is precisely engineered as a standalone rendering technique, not designed to
interoperate with an *external* system of independently-ordered opaque/transparent phases
the way Bevy's `Core3dSystems::MainPass` composes `Opaque3d`/`AlphaMask3d` (binned,
order-independent by design since depth testing handles correctness) and `Transparent3d`
(globally sorted, back-to-front). For content that genuinely needs sort-based
correctness, WSR is still the right fix for this specific tension (see below) — it's
only wrong when the content doesn't need sort-based compositing (or any compositing
beyond a depth test) in the first place, which is the opaque case this page now leads
with.

### The WSR mechanism (unchanged, for translucent content)

**Sort-free Gaussian Splatting via Weighted Sum Rendering** approximates alpha blending
as a weighted **sum** rather than an ordered sequential composite. Because addition is
commutative, the compositing result no longer depends on evaluation order at all —
sorting the contributing Gaussians becomes entirely unnecessary, not just cheaper.

Concretely, WSR replaces the standard front-to-back compositing loop (see
[tile-based-rasterizer](../../3dgs/rendering-and-rasterization/tile-based-rasterizer.md)'s
`C = Σᵢ cᵢ αᵢ Tᵢ` formula, which depends on order through the transmittance term `Tᵢ =
Πⱼ<ᵢ(1-αⱼ)`) with a depth-weighted summation formulation where each Gaussian's
contribution is weighted by a function of its own depth/opacity alone, not by the
accumulated state of primitives composited *before* it in some chosen order. The
published result: competitive visual quality, an average 1.23x speedup on mobile GPUs,
37% lower memory usage, and elimination of the popping artifact specifically, since
there is no discrete reordering event left to cause one.

**What WSR does NOT give you**, which is the part the original version of this page
underweighted: order-independence *among the splats contributing to a pixel*, not
occlusion *between different surfaces*. A background object's splats and a foreground
object's splats at the same screen pixel still need a real depth relationship to
composite correctly, and WSR's additive math has no mechanism for that at all —
whatever depth-based gate you add on top (a prepass + depth test, as this demo tried)
re-introduces exactly the bias problem documented above, because "same-surface
tolerance" and "cross-object occlusion" pull the bias value in opposite directions and
there is no single constant that serves both correctly.

## Proven alternative for opaque content: real depth-tested rendering

This is what the demo actually ships with now, and the recommended default unless the
"when WSR is still the right answer" criteria above clearly apply:

- Splats whose material is opaque render through a pipeline with **real hardware depth
  test AND write** (`depth_write_enabled: true`, standard `GreaterEqual`
  reversed-Z compare), sharing Bevy's own Core3d depth buffer — the exact same
  z-buffering every opaque/alpha-tested mesh uses. The fragment shader alpha-tests
  (or, better — see below — drives alpha-to-coverage) instead of blending, so each
  visible fragment writes its own true depth and the hardware's nearest-wins
  comparison handles occlusion correctly by construction, both between distinct
  objects and between neighboring splats on the same surface. **Zero bias constants
  anywhere.**
- Splats whose material is genuinely translucent route to a **second** pipeline reusing
  Bevy's own `Transparent3d` phase directly — a real `SortedPhaseItem` with correct
  per-instance back-to-front ordering — depth-tested (not written) against the shared
  depth buffer the opaque pass already populated. `Transparent3d`'s fields turned out
  to be generic enough for non-mesh splat content without modification.
- **Pass ordering matters and is easy to get subtly wrong**: the custom opaque splat
  pass must be ordered both `.after(main_opaque_pass_3d)` *and*
  `.before(main_transparent_pass_3d)` explicitly. Bevy's `Core3dPlugin` only chains
  those two systems against *each other* via `.chain()`; that says nothing about
  where a third system in the same `Core3dSystems::MainPass` set falls relative to
  either one. Missing the `.before(main_transparent_pass_3d)` half let the scheduler
  run the opaque splat pass *after* the transparent pass in practice, which produced
  a translucent object appearing to be "hidden behind everything" (the transparent
  pass depth-tested against a not-yet-populated depth buffer and always won, then the
  opaque pass ran afterward and unconditionally overwrote color+depth on top of it).
- **Alpha-to-coverage, not alpha-test alone, for smooth opaque edges.** A binary
  alpha-test threshold (discard below X, treat above X as fully opaque) makes each
  splat's own disc edge a visible hard boundary against its neighbors — a
  stippled/blotchy surface texture. Setting `alpha_to_coverage_enabled: true` on the
  opaque pipeline's `MultisampleState` and letting the fragment shader's real Gaussian
  density drive MSAA sample coverage (instead of a hard discard) resolves neighboring
  splats' overlapping, fading discs into a smooth blended edge during MSAA resolve,
  with full depth correctness preserved (each *covered sample* still writes its own
  true depth).
- **Per-splat alpha for a translucent shell needs to account for layer stacking, not
  just "how see-through should this look."** A closed surface (e.g. a sphere) has no
  backface culling in this pipeline, so every view ray through it crosses at least two
  overlapping layers of splats (entry + exit surface), each itself built from several
  overlapping splats from the bake's own overlap factor. Real sequential alpha
  blending compounds those layers multiplicatively (`coverage ≈ 1 - (1-α)ⁿ` for `n`
  stacked layers) — a per-splat alpha picked to "look right" for one isolated splat
  reads as nearly opaque once several layers stack, and needs to be tuned much lower
  (an order of magnitude, empirically) than intuition suggests for the visible result
  to actually demonstrate translucency.

## What is given up (WSR, translucent case only)

WSR is an **approximation** of true depth-ordered alpha blending, not mathematically
identical to it — published results describe "competitive visual quality," not exact
equivalence. For genuinely soft, semi-transparent-edged organic content this
approximation is visually indistinguishable in practice per the published evaluation;
scenes depending on *exact*, sharply-defined depth ordering of highly-transparent
overlapping surfaces (e.g. stacked glass panes) are the harder case for any
order-independent compositing scheme, WSR included, and would need case-by-case
evaluation rather than an assumed-safe default.

## When to dive in

- Deciding how to composite a splat-rendered scene's opaque geometry → start with real
  depth-tested rendering (this page's "proven alternative" section), not WSR. This is
  the corrected default.
- Implementing genuinely translucent/soft splat content that needs order-independence
  → this page's WSR summary is still the right starting point, with the explicit
  caveat that it only solves same-pixel ordering among translucent contributors, not
  occlusion against opaque geometry (which still needs a real, separately-populated
  depth buffer to test against).
- Debugging a "some splats are invisible/patchy at some camera angles but not others"
  bug in a WSR-based pipeline → this is very likely the exact bias-angle-dependence
  failure mode documented above; the fix is very unlikely to be "tune the bias
  differently" and very likely to be "does this content need WSR at all."
- Understanding why `Splat3d` in [bevy-pipeline-integration](../architecture/bevy-pipeline-integration.md)
  is now two separate pipelines (one `BinnedPhaseItem`, one reusing `Transparent3d`)
  rather than a single order-independent phase → this page is the justification.

## Related

- [Native integration with Bevy's render pipeline](../architecture/bevy-pipeline-integration.md) — applies: the two-pipeline phase setup this lesson produced.
- [Why built-in Bevy screen-space effects work on splat-rendered pixels for free](./screen-space-effect-compatibility.md) — deeper: what depends on the depth the opaque pass writes.
- [The tile-based rasterizer](../../3dgs/rendering-and-rasterization/tile-based-rasterizer.md) — contrast: the reference global per-tile sort WSR was meant to avoid.
- [Common quality artifacts and their causes](../../3dgs/quality-and-artifacts/common-artifacts.md) — deeper: the popping artifact WSR removes for translucent content.
- [Render phases, PhaseItems, and GPU-driven batching](../../bevy-rendering/architecture/render-phases-and-batching.md) — prerequisite: binned vs. sorted phases.
- [Prepass, tonemapping, upscaling, deferred, and OIT](../../bevy-rendering/core-pipeline/passes-and-fullscreen-effects.md) — contrast: Bevy's own order-independent transparency.
