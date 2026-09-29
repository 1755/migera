---
title: Common raymarching artifacts and their causes
description: Symptom → cause → fix map for SDF raymarching — punch-through, floating surfaces, shadow banding, missed thin geometry, shadow acne, normal noise, and three migera BVH-march bugs (merged-interval chord cut, scale-dependent false shadow darkening, unpadded-ancestor cut). Read first when a raymarched image looks wrong.
type: guide
status: current
tags:
  - sdf
  - raymarching
  - troubleshooting
  - debugging
  - correctness
  - hybrid-renderer
updated: 2026-09-07
aliases:
  - punch-through
  - swiss cheese holes
  - shadow acne
  - overstepping
---

# Common raymarching artifacts and their causes

A practical troubleshooting reference connecting visible bugs back to their root cause in
the underlying SDF/raymarching theory covered elsewhere in this knowledge base.

## Punch-through / "Swiss cheese" holes

**Symptom**: rays pass through geometry that should be solid, especially at glancing
angles or in areas with heavy domain warping (twist, bend, displacement, aggressive
non-uniform scaling).

**Cause**: the field being raymarched has violated the Lipschitz-1 safety bound —
somewhere the SDF is *overestimating* distance, so a step taken at full length actually
travels through a surface without detecting it. See
[exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md).

**Fix**: apply a conservative step-damping multiplier (commonly `0.8`–`0.95`, tuned
empirically) to every step, especially after any twist/bend/displacement/non-uniform-scale
operator (see [modifier-operators](../primitives-and-operators/modifier-operators.md)).
This costs extra iterations but restores safety.

## Surface "floating" or offset from its true position

**Symptom**: geometry appears to sit slightly above or below where it should, most
visible where two surfaces should meet exactly (a shadow-casting object and its shadow's
contact point, for instance).

**Cause**: the raymarch hit epsilon (see [sphere-tracing](./sphere-tracing.md)) is too
large relative to scene scale, so the ray is accepted as "hit" before it has actually
reached the true zero level set.

**Fix**: reduce the epsilon, or better, scale it relative to distance traveled from the
camera (a fixed absolute epsilon becomes proportionally larger error at greater
distances) — but watch for the corresponding rise in iteration count.

## Banding in soft shadows or gradients

**Symptom**: visible discrete steps/bands in soft shadow penumbras rather than a smooth
gradient.

**Cause**: the naive soft-shadow formula (`k*h/t` at discrete step positions) is
discontinuous as a function of the actual occluder geometry — see
[soft-shadows-and-ao](./soft-shadows-and-ao.md) for why.

**Fix**: apply the interpolated-closest-point correction (Aaltonen's technique), not
simply more steps — more steps reduces band width but does not remove the underlying
discontinuity.

## Missed thin/grazing geometry, or holes at silhouette edges

**Symptom**: thin features (blades of grass, wires, sharp fins) disappear or flicker,
especially near the object's silhouette relative to the camera.

**Cause**: sphere tracing's convergence is only guaranteed given an *unbounded* number of
steps (see [sphere-tracing](./sphere-tracing.md)); rays that graze thin or steeply-angled
geometry take many small steps to resolve and can exhaust the iteration budget before
converging.

**Fix**: raise `MAX_STEPS` for affected scenes (with a direct performance cost), apply
overrelaxation to accelerate convergence for the common near-flat case, or — for
persistently thin geometry — consider whether a different representation (explicit
geometry for that specific asset) is more appropriate than forcing raymarching to resolve
arbitrarily thin features.

## Self-shadowing acne on curved surfaces

**Symptom**: shadow rays cast from a curved surface incorrectly detect the surface
itself as the first occluder, producing dark speckling/banding on lit surfaces that
should receive full light.

**Cause**: the shadow ray's starting point, evaluated via numerical normal estimation
(see [normal-estimation](./normal-estimation.md)), has some position/normal error; a ray
offset from a curved surface by too small a margin can immediately re-intersect that same
surface due to this error, especially where curvature is high.

**Fix**: offset the shadow ray's starting point along the normal by a small bias before
marching, and/or start the shadow raymarch's `mint` parameter slightly greater than zero
rather than at the exact surface point.

## Normal-estimation noise on sharp or displaced geometry

**Symptom**: shading flickers or looks faceted/unstable on sharp edges or on surfaces
with fine procedural displacement, especially under animation or camera movement.

**Cause**: numerical gradient estimation is inherently sensitive near points where the
true SDF gradient is discontinuous (sharp edges, medial-axis regions) or where the field
has high-frequency variation relative to the sample offset `h` (fine displacement); see
[normal-estimation](./normal-estimation.md).

**Fix**: tune the normal-estimation sample offset relative to feature scale — this is
often a different (larger) value than the raymarch hit epsilon, not the same constant
reused for both purposes, which is a common initial mistake.

## Flat "cut" or chord bitten out of a round silhouette, near where two objects touch

**Symptom**: a curved object (e.g. a sphere resting flush on a ground plate) renders
with a flat, straight-edged notch cut into its silhouette, only at specific camera
angles — other angles of the same scene render a perfectly round silhouette. Looks like
the object was clipped by an invisible box. Reads as an occlusion bug (the wrong object
"winning" the pixel) and is very tempting to explain away as *correct* occlusion between
a finite ground plate and a tangent sphere at a grazing angle — verify against real
ground truth (see the debugging note below) before accepting that explanation.

**Cause**: in a multi-object BVH-accelerated march, each object's per-step interval was
being walked through a **global merged interval list** (built by unioning all BVH
candidates' AABB slab hits, e.g. via a sorted-insert `is_push`-style merge) and then
clipped to the current object's own `[obj_near, obj_far]` slab. The merged list only
records the *union* of all objects' slabs, with no per-object memory — its insert/merge
logic can produce entries that, once clipped to one specific object's own bounds, leave a
gap exactly over that object's true surface crossing (most likely when a neighboring
object's slab starts or ends partway through this object's range, splitting the merged
list at a point that doesn't align with this object's own geometry). The march silently
skips that gap and never samples near the true surface there, so the object loses the
pixel to whichever other object's march resolves — with no error, warning, or missed-step
signal of any kind.

**Fix**: march each object over its own exact `[obj_near, obj_far]` candidate slab
directly — do not index through the merged global interval list at all for the per-object
step loop. The global list is only useful as a cheap "is anything at all along this ray"
early reject; it must never gate *how far* an individual object's march is allowed to
step, since only that object's own AABB slab is guaranteed to bound its own geometry.

## False shadow darkening that only appears at large scene/object counts

**Symptom**: soft shadows look correct in a small test scene but show real, incorrect
darkening (open ground reading as heavily shadowed with no occluder anywhere near the
ray) once the scene is stress-tested with many more objects spread over a larger world
extent — same lighting, same `k`, same code, only the object/scene count changed.

**Cause**: a shadow-margin or march-bound formula parameter was derived from a
scene-extent or ray-reach quantity (BVH root diagonal, a ray's own `max_t`) instead of a
fixed constant sized to the renderer's actual object scale. Both quantities grow
silently as more objects are added or spread out, shrinking the formula's effective
margin/reach at the same nominal `k` — see
[soft-shadows-and-ao](./soft-shadows-and-ao.md#porting-to-srchybrid-three-more-bugs-the-hybrid_legacy-port-didnt-warn-about)
for the full incident (also covers a related pitfall: a hardness constant `k` tuned and
validated only against one small demo scene is not automatically valid at a different
object scale).

**Fix**: replace any scene-extent- or ray-reach-derived reference distance in the
formula with a fixed constant tied to object scale; re-validate `k` with a real sweep
test against the target renderer's actual stress-test scale rather than carrying a
value forward from a different scene. Diagnose with a dense ground-truth CPU scan test
(every cell's own footprint, asserting no false darkening from unrelated objects) —
this class of bug is scale-dependent and very easy to miss by eye in a small test scene.

## Sharp on/off "cut" in an otherwise-smooth soft shadow silhouette

**Symptom**: a curved occluder's soft shadow looks correct and round almost everywhere,
but has a sharp, localized cut or flat edge on one side, distinct from the "flat chord
near a touching object" artifact above (this one is not near a touching neighbor and
does not track camera angle the same way) — an angular scan around the shadow's edge
shows a hard jump in visibility between two very close sample angles, not a gradual
transition.

**Cause**: a BVH acceleration query that pads leaf bounds before the ray test (as soft
shadows' candidate-margin fix requires — see
[soft-shadows-and-ao](./soft-shadows-and-ao.md)) but leaves internal/ancestor node
bounds tight/unpadded can prune a valid padded leaf during descent, because the
ancestor's tight bound has no knowledge that the leaf gets padded before its own test —
see [bvh-deep-dive](../../hierarchical-volumes/bvh-deep-dive.md#padded-expanded-queries-near-miss-soft-shadow-proximity)
for the general mechanism.

**Fix**: pad every BVH node in the padded query's descent path uniformly, not just
leaves.

## When to dive in

- Any visible raymarching artifact → start here to map the symptom to its likely root
  cause and the correct fix, rather than blindly increasing iteration counts or shrinking
  epsilons across the board (which usually masks rather than fixes the underlying issue,
  at real performance cost).

## Related
- [Exact vs. bound distance fields](../fundamentals/exact-vs-bound-sdfs.md) — prerequisite: the root cause of punch-through.
- [Sphere tracing](./sphere-tracing.md) — prerequisite: iteration cap and epsilon behaviour.
- [Normal estimation](./normal-estimation.md) — deeper: normal-noise entry.
- [Soft shadows and AO](./soft-shadows-and-ao.md) — deeper: the full investigation behind the shadow entries.
- [Glitch-free baked SDFs](../mesh-conversion/glitch-free-baked-sdfs.md) — deeper: the same artifacts specific to baked grids.
- [BVH deep dive](../../hierarchical-volumes/bvh-deep-dive.md) — deeper: per-object slabs and padded queries behind the BVH entries.
- [Hybrid renderer hardening](../../hybrid-architecture/plans/hybrid-renderer-hardening.md) — deeper: survey of sphere-tracing accelerators and robustness pitfalls for migera's SDF tier.
