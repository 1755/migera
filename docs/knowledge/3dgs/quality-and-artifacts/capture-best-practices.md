---
title: Capture best practices
description: Gives photo-capture guidance for 3DGS (view coverage and overlap, avoiding textureless/reflective/moving content that breaks SfM, consistent lighting, camera settings, checking SfM output before full training), since bad initialization is hard to fix later. Read before shooting any new capture.
type: guide
status: current
tags:
  - 3dgs
  - verification
  - prior-art
updated: 2026-08-15
aliases:
  - photogrammetry
  - COLMAP
  - view coverage
---

# Capture best practices

Because [floaters and related quality problems](./common-artifacts.md) trace back
disproportionately to initialization quality (see
[sfm-initialization](../optimization-and-training/sfm-initialization.md)), and because
post-hoc training-time fixes are documented as often insufficient once a bad
initialization has taken hold, capture quality is one of the highest-leverage places to
invest effort in a 3DGS pipeline — arguably higher-leverage than tuning training
hyperparameters after the fact.

## View coverage and overlap

Structure-from-Motion (COLMAP or equivalent) needs enough overlapping viewpoints of every
part of the scene to triangulate 3D points reliably, and 3DGS training itself benefits
from dense view coverage for the same underlying reason — regions seen from few angles
are under-constrained and prone to floaters or blurry, poorly-resolved detail. Practical
guidance following directly from this:

- Aim for substantial overlap between consecutive photos (a common rule of thumb from
  photogrammetry practice generally is 60-80% overlap between adjacent shots, though
  exact numbers depend on scene complexity and desired detail level).
- Capture from multiple heights/elevations when photographing a 3D volume, not just a
  single sweeping ring at one height — surfaces only visible from above or below a single
  capture ring will be under-covered.
- Avoid large gaps in angular coverage around any object or region of interest — a "hole"
  in viewing angle coverage becomes a corresponding hole or floater-prone region in the
  reconstruction.

## Avoiding known SfM failure modes

Since SfM feature matching is the foundation both pose estimation and initial point
placement depend on, avoid conditions known to break it:

- **Textureless or repetitive surfaces** (blank walls, tiled floors, repeated brick
  patterns) — SfM feature matching relies on locally distinctive visual features; large
  featureless or repetitive regions are classic failure cases producing sparse or
  incorrect points specifically in those areas.
- **Reflective or transparent surfaces** (glass, mirrors, polished metal) — these violate
  the implicit assumption that a matched feature corresponds to a fixed 3D point, since
  reflections/refractions change with viewing angle in ways unrelated to the underlying
  surface geometry.
- **Moving content during capture** (people, vehicles, wind-blown foliage) — SfM assumes
  a static scene; motion during capture can corrupt both pose estimation and point
  triangulation, not just produce local artifacts.

## Lighting consistency

Since 3DGS represents view-dependent appearance via spherical harmonics fit to the
specific training photos (see
[gaussian-primitive-parameters](../fundamentals/gaussian-primitive-parameters.md)),
inconsistent lighting across a capture session (e.g. capturing outdoors over a period
where the sun moves, or with a flash on some shots and not others) introduces genuine
ambiguity the optimizer must try to resolve — the same physical point may need to
represent two different "true" colors under different captured lighting, which SH
view-dependence isn't designed to encode (SH captures viewing-angle dependence, not
scene-relighting dependence — see
[state-of-the-art](../state-of-the-art/INDEX.md) for relighting-aware variants that
specifically address this separate problem). Consistent lighting throughout a capture
session avoids introducing this ambiguity in the first place.

## Camera settings

Standard photogrammetry-adjacent guidance applies: avoid extreme motion blur (which
corrupts feature matching), prefer consistent focal length/exposure settings across a
capture session where practical (variation can be handled by SfM/3DGS to a degree, but
consistency reduces ambiguity), and capture at resolution/quality sufficient for the
level of detail the final application requires, since 3DGS cannot invent detail beyond
what the input photos actually resolved.

## Post-capture verification before committing to full training

Since initialization problems are expensive to fix after the fact, it's worth verifying
SfM output quality (pose estimation converged for all/most images, point cloud density
and coverage looks reasonable, no obviously degenerate camera poses) before investing in
a full training run — catching a bad capture at this stage is far cheaper than
discovering the problem only after training has "baked in" floater traps that resist
correction.

## When to dive in

- Planning any new 3DGS capture → read this page before shooting, not after — the
  guidance here is specifically about preventing problems that are documented to be
  difficult to fix once training has started, rather than problems easily patched
  post-hoc.
- Diagnosing quality problems in an already-captured, already-trained scene → cross-
  reference against [common-artifacts](./common-artifacts.md) first to identify the
  specific symptom, then check whether this page's guidance was followed for the
  affected region of the capture.

## Related

- [Structure-from-Motion initialization](../optimization-and-training/sfm-initialization.md) — prerequisite: the SfM step these practices protect.
- [Common quality artifacts and their causes](./common-artifacts.md) — applies: the symptom map to check a finished capture against.
