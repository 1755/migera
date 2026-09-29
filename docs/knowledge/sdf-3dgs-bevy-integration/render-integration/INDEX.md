---
title: Render Integration
description: Archived record of how migera's deleted splat renderer composited in Bevy (depth-written opaque splats, Transparent3d for translucent) and which post effects work from ViewTarget alone vs. need a prepass. Read before compositing soft primitives in Bevy or checking post-process support for a custom pass.
type: index
status: current
tags:
  - 3dgs
  - bevy
  - post-processing
  - render-pipeline
updated: 2026-09-28
---

# Render Integration

Where "reuse Bevy's built-in tools with custom rendering" was made concrete: the
compositing technique for splats, and an audit of which built-in screen-space effects
work automatically. Both notes are archived — the splat renderer was removed in commits
22d3b91 (2026-08-16) and 684490c (2026-09-06) — but their lessons apply to any custom
Core3d pass.

## Archived / superseded

| Note | What it establishes | Read when |
|---|---|---|
| [Compositing: Weighted Sum Rendering vs. real depth-tested opaque rendering](./sort-free-compositing.md) | WSR is wrong for opaque content (depth bias with no correct constant); opaque splats need depth write plus alpha-test, translucent ones reuse `Transparent3d` | Compositing splats or other soft primitives, or tempted to tune a depth bias |
| [Why built-in Bevy screen-space effects work on splat-rendered pixels for free](./screen-space-effect-compatibility.md) | Tonemapping/bloom/vignette/FXAA/upscaling read only `ViewTarget` colour; TAA/SSAO/DOF need prepass depth, normals or motion vectors | Checking whether a post effect will work on a custom pass |
