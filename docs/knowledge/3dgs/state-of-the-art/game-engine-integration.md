---
title: Game engine integration and production status (2025-2026)
description: Assesses 3DGS production maturity as of early 2026: Unreal and Unity plugins reach ~60 fps under ~1M Gaussians on RTX 3060-class GPUs, but relighting is still research-only and physics needs extracted meshes. Read before committing splats to a game or VFX pipeline, or when needing relit or collidable splats.
type: research
status: current
tags:
  - 3dgs
  - integration
  - lighting
  - physics
  - state-of-the-art
updated: 2026-08-15
aliases:
  - Unreal Engine
  - Unity
  - relighting
  - XScene-UEPlugin
---

# Game engine integration and production status (2025-2026)

## Current maturity level

As of early 2026, both Unreal Engine and Unity have **mature plugin ecosystems** for
real-time 3DGS rendering, with frame rates around 60 fps achievable for scenes under
roughly 1 million Gaussians on mid-range consumer GPU hardware (NVIDIA RTX 3060 / AMD
RX 6600 class) — a concrete, current benchmark for what "production-viable real-time
splat rendering" means in practice on non-flagship hardware, as opposed to only being
demonstrable on top-tier GPUs.

## Unreal Engine

Multiple integration paths exist with differing maturity/support tradeoffs:

- **XScene-UEPlugin** (XVERSE) — free, open-source integration for Unreal Engine 5.3+.
- **Commercial plugins** (e.g. offerings on Epic's Fab marketplace) — offer additional
  stability and vendor support at the cost of licensing.

A practical consideration specific to Unreal integration: how splat content coexists
with Unreal's existing rendering technologies (Nanite virtualized geometry, traditional
PBR materials) — splats are a genuinely different rendering paradigm from either, so
production pipelines need to decide where splat content fits relative to conventional
mesh-based scene elements rather than treating it as a drop-in replacement for either.

## Unity

The most widely used solution is **UnityGaussianSplatting**, created by Aras
Pranckevičius (a former Unity engineer) — an actively maintained, open-source package
handling PLY import natively with GPU-accelerated sorting, supporting Unity 6 LTS across
PC/Mac/mobile platforms.

## Relighting: still an open production gap

**Relightable Gaussian Splatting research exists** (e.g. "Relightable 3D Gaussian,"
NeurIPS 2023; "LumiGauss," WACV 2025) — these decompose captured appearance into
material/lighting components (an inverse-rendering-style approach) so a trained scene can
be rendered under *novel* lighting conditions different from the capture's original
illumination, rather than being locked to appearance baked in at capture time. However,
as of the most recent available assessment, these techniques **require specialized
training pipelines and are not available in any commercial DCC (digital content
creation) tool** — meaning a production team wanting relightable splat content today
needs to adopt research-grade training code directly rather than relying on established
commercial tooling, a real practical gap between research capability and production
availability.

Standard commercial plugins do support **adding conventional dynamic lights that affect
the splat scene** to some degree — but shadow/occlusion behavior from these added lights
differs from mesh-based geometry's behavior (unsurprising, given rasterization-based
splat rendering's structural limitations around shadows discussed in
[ray-tracing-gaussians](../rendering-and-rasterization/ray-tracing-gaussians.md)), and
commercial plugin vendors are actively developing improved hybrid lighting solutions
rather than having fully solved this as of the current assessment.

## Physics and collision: meshes remain preferred

For objects requiring collision detection or physics simulation, **traditional meshes
remain the preferred representation**, not splats — 3DGS's explicit-primitive
representation (see [what-is-3dgs](../fundamentals/what-is-3dgs.md)) doesn't naturally
provide the clean surface/boundary information physics engines expect, unlike a mesh's
well-defined polygon boundaries. The practical guidance emerging from production
experience: 3DGS is best suited to representing real-world environments too complex to
model manually, where photorealistic lighting baked in from the real capture is
specifically desired — not as a general-purpose replacement for mesh geometry across all
use cases. Where physics/collision is needed on splat-captured content, extracting a mesh
(see [2d-gaussian-splatting](./2d-gaussian-splatting.md) for SuGaR-style mesh extraction)
and using that mesh for physics while retaining the splat representation for visual
rendering is the practical current pattern, rather than attempting physics directly
against the Gaussian representation.

## Practical scale limitations

Current commercial plugins handle moderate scenes well, but **very large environments
typically need to be split into sections** to manage memory — file sizes for complex
captures commonly reach 1 GB or more, with correspondingly significant GPU memory
requirements. This is the same underlying constraint discussed generally in
[large-scene-techniques](../performance-and-compression/large-scene-techniques.md) and
[compression-techniques](../performance-and-compression/compression-techniques.md),
manifesting here specifically as a practical game-engine-workflow concern (level
streaming / scene partitioning) rather than a purely technical rendering-cost discussion.

## Summary: where 3DGS fits in a production pipeline today

| Use case | Recommendation |
|---|---|
| Photorealistic capture of a real environment for visual backdrop/exploration | 3DGS is a strong, mature fit as of 2025-2026 |
| Objects/environments needing physics, collision, or gameplay interaction | Traditional mesh remains preferred; extract a mesh from splat data if needed |
| Content requiring relighting under novel/dynamic lighting conditions | Currently a research-only capability; not yet available in commercial DCC tooling |
| Very large environments (city-scale, open-world) | Requires scene partitioning/LOD/streaming; not a drop-in single-scene load |

## When to dive in

- Evaluating whether to adopt 3DGS in a game/VFX production pipeline today → this page's
  maturity assessment and the use-case table are the practical starting point for scoping
  what's realistic to ship versus what remains research-stage.
- Needing relighting or physics interaction with splat content specifically → understand
  these are current gaps requiring either research-grade tooling (relighting) or hybrid
  mesh-extraction workarounds (physics), not features to expect out-of-the-box from
  commercial plugins as of this writing.
- Planning for a large-scale environment → budget for scene partitioning/LOD work from
  the start rather than assuming a single monolithic scene load will scale.

## Related

- [Ray tracing Gaussians](../rendering-and-rasterization/ray-tracing-gaussians.md) — deeper: the route to shadows and reflections on mixed mesh+splat scenes.
- [Level-of-detail, streaming, and large-scene techniques](../performance-and-compression/large-scene-techniques.md) — applies: partitioning for large environments.
- [2D Gaussian Splatting and mesh extraction](./2d-gaussian-splatting.md) — applies: getting collision meshes out of splats.
- [Native integration with Bevy's render pipeline](../../sdf-3dgs-bevy-integration/architecture/bevy-pipeline-integration.md) — example: migera's own (archived) Bevy splat integration.
- [One SDF driving collision, physics, and lighting](../../sdf-3dgs-bevy-integration/live-editing/unified-physics-and-lighting.md) — contrast: sidestepping the relighting/physics gaps with an SDF source of truth.
