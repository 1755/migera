---
title: One SDF driving collision, physics, and lighting — not three separate systems
description: Designs using one SDF for collision (Claybook: penetration depth and normal for free), for moving results back onto splats (GASP, VR-GS), and for sphere-traced soft shadows/AO baked into splat attributes vs. computed per frame. Never built; archived. Read before wiring SDF collision or SDF lighting.
type: design
status: archived
tags:
  - sdf
  - 3dgs
  - physics
  - shadows
  - lighting
  - baking
updated: 2026-08-15
sources:
  - https://iquilezles.org/articles/rmshadows/
  - commit 684490c (splat renderer removed)
aliases:
  - SDF collision
  - ambient occlusion
  - soft shadows
  - Claybook
  - Dreams
  - GASP
---

# One SDF driving collision, physics, and lighting — not three separate systems

> **Archived (2026-09-28):** never built on splats (dropped in commit 684490c, 2026-09-06). migera did build an SDF-native rigid-body engine (`src/physics`) but demoted it in 2026-09 in favour of avian3d (`src/physics_avian`), and its marched SDF shadows live in the `src/hybrid` raymarcher.

Contents: [Why design it deliberately](#why-this-is-worth-designing-deliberately-not-letting-happen-by-accident) · [Physics and collision](#physics-and-collision-settled-prior-art-one-open-question) · [Lighting and shadows](#lighting-and-shadows-reuse-the-bakes-own-sdf-evaluator-decide-where-the-result-lives) · [Relighting research](#why-relighting-research-gsurf-discretized-sdf-papers-undersells-what-this-project-already-has) · [When to dive in](#when-to-dive-in)

## Why this is worth designing deliberately, not letting happen by accident

It's easy to end up with three unrelated systems that happen to read the same
primitive list: a physics engine with its own collision shapes, a lighting system with
its own shadow maps, and a splat renderer with its own baked geometry — kept in sync
only by convention, drifting the moment someone edits one without remembering the
others. The alternative this document develops: **the SDF stays the single source of
truth for all three**, and each consumer (physics, lighting, rendering) queries or bakes
from it directly, the same way [world-representation](../architecture/world-representation.md)
already establishes for geometry alone. This is not a hypothetical architecture —
it's how two shipped, 60fps-class games already worked.

## Physics and collision: settled prior art, one open question

[production-case-studies](../../sdf-3d/performance-and-production/production-case-studies.md)
already documents *Claybook*'s GPGPU physics solver running directly against its SDF
volume texture (`1024×1024×512`, 8-bit signed, 5 mip levels, 586MB), chosen specifically
because **signed distance gives penetration depth and contact normal for free** —
`|f(x)|` for `f(x) < 0` is exact penetration depth, and `∇f(x)` at the contact point is
the contact normal, with no separate normal computation needed the way mesh/convex-hull
collision requires. This solves the "thin shell tunneling" problem structurally: a
triangle mesh has no interior concept, so a fast-moving thin object can pass through it
between frames with no way to detect "how far it's already penetrated"; an SDF's
negative interior values make this trivial. Claybook's physics consumed roughly 25% of
total GPU frame time at a locked 60Hz on Xbox One, shipping down to Nintendo Switch too
(with fluid-particle-count and resolution cuts to hit a lower frame budget on weaker
hardware) — concrete evidence this is affordable within a real console frame budget, not
just a high-end tech demo. NVIDIA PhysX 5's GPU rigid-body pipeline independently
validates the same idea for concave collision shapes specifically to avoid convex
decomposition
([GPU Rigid Bodies docs](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/GPURigidBodies.html)).

**What's settled**: SDF-driven collision/physics works, is fast, and is architecturally
simple (query the same `distance()` function the bake already calls). **What's open**:
how the physics result gets back onto the *Gaussian splats*, which didn't exist as a
representation when Claybook or Dreams shipped. Two research threads are the closest
answers found:

- **GASP** (Gaussian Splatting for Physic-Based Simulations,
  [arXiv 2409.05819](https://arxiv.org/abs/2409.05819),
  [project page](https://waczjoan.github.io/GASP/),
  [code](https://github.com/waczjoan/GASP)) treats Gaussians as flat distributions
  **reducible to plain 3D points** for simulation purposes — any physics engine
  (demonstrated with Genesis, Blender, Taichi Elements, none of them Gaussian-aware)
  simulates the point positions directly, and Gaussian covariance/orientation is
  **re-derived afterward** from the simulated point neighborhood. This is the pattern
  to follow if this project's physics substrate stays a general-purpose engine (or a
  bespoke SDF-based solver, Claybook-style) rather than something MPM/Gaussian-native:
  physics owns point positions, splat parameters (orientation, scale) are a downstream
  derived view recomputed from those positions — structurally identical to how this
  project's `assemble_scene` already derives `Node` geometry from ECS `Transform`
  components each bake, just one layer further down (deriving splat *shape*, not just
  placement, from simulated point neighborhoods).
- **VR-GS**'s tet-cage embedding (already covered in
  [deformation-vs-rebake](./deformation-vs-rebake.md) for pure deformation) is the same
  mechanism applied to physics specifically: the cage *is* the physics simulation
  domain (XPBD constraints), and Gaussians ride along via interpolation weights — a
  workable alternative to GASP's point-reduction approach when a primitive's physical
  behavior is better modeled as a deformable cage than as free point-cloud dynamics
  (e.g. a squishable prop that should keep a roughly-constant volume, which XPBD
  volume-preservation constraints handle naturally and unconstrained point simulation
  does not).

No paper found couples an **SDF-based** physics solver (Claybook's approach) directly to
**Gaussian** splat output — that specific combination remains unclaimed territory, per
the scope note in [this topic's index](./INDEX.md). GASP's point-reduction pattern is
the most directly portable idea regardless of which physics solver sits underneath it.

## Lighting and shadows: reuse the bake's own SDF evaluator, decide where the result lives

Sphere-traced shadows and ambient occlusion need no new machinery beyond what
[sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) Steps 1-2 already call
millions of times per bake — the same `distance()` function, evaluated along a few
extra rays or offset points per splat:

- **Soft shadows** — the standard technique
  ([Inigo Quílez](https://iquilezles.org/articles/rmshadows/)) tracks the **minimum
  ratio of `SDF-value / distance-traveled`** observed while marching a ray from the
  splat's surface point toward a light:

  ```
  res = min(res, k * h / t)   // h = SDF value at current step, t = distance traveled so far
  ```

  scaled by `k` (inverse of the light's angular size, controlling penumbra softness) —
  a single-pass, closed-form soft shadow with no shadow-map geometry, directly
  evaluable at bake time per splat since position and the SDF evaluator are both
  already in scope. A refined variant (Sebastián Aaltonen) reduces banding artifacts
  by estimating true closest-approach distance between consecutive march steps via
  simple triangle geometry, worth adopting if banding shows up in practice.
- **Ambient occlusion** — cast a short sequence of steps outward along the splat's
  already-known surface normal (`∇f(p)`, no extra computation — this project's bake
  already computes this for splat orientation), and compare the actual SDF value at
  each step to the *expected* unoccluded distance; a deficit indicates nearby occluding
  geometry. Cheap, analytic, and — like the shadow term — fully evaluable at bake time
  using data the bake already has in hand.
- **Production validation at scale**: Unreal Engine 5's Lumen uses baked per-mesh
  distance fields for exactly this kind of sphere-traced "software ray tracing"
  (approximate GI/shadows/reflections), explicitly trading precision for the ability to
  fit inside a 60fps budget
  ([Lumen docs](https://dev.epicgames.com/documentation/unreal-engine/lumen-global-illumination-and-reflections-in-unreal-engine)) —
  citable evidence that SDF-sphere-traced lighting is production-proven at AAA
  real-time scale, not just a raymarching-demo trick.

**The genuinely open design decision is where the computed shadow/AO term is stored**,
not whether it can be computed cheaply:

- **Baked into a low-order spherical harmonic coefficient per splat** (SH degree 0, a
  flat scalar/color modulation — see
  [gaussian-primitive-parameters](../../3dgs/fundamentals/gaussian-primitive-parameters.md)
  for why SH degree 0 is just a constant color term) at bake time, alongside Step 3's
  existing appearance sourcing (see
  [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) Step 3) — cheap at
  render time (already-baked, no shader-side SDF evaluator needed at all), but static
  until the next re-bake, meaning a moving light or a moving shadow-caster won't update
  shadows on already-baked splats without triggering a re-bake. This is the natural
  choice for **static or slowly-changing lighting** (matching this project's existing
  "baked lighting response" strategy already named in
  [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md) Step 3's third
  sourcing option).
- **Computed per-frame in the splat fragment/vertex shader**, using a shader-side SDF
  evaluator — dynamic (correctly follows a moving light or moving occluders without
  needing a re-bake), but requires porting the CPU-side SDF evaluation used at bake
  time into WGSL as a second implementation of the same scene graph, with the drift
  risk that implies unless both share a single evaluation module (this project's
  planned [gpu-compute-baking](../baking-pipeline/gpu-compute-baking.md) direction
  already names this exact "one WGSL SDF-evaluation module shared between editor
  preview and production bake" goal for a different reason — reusing that same shared
  module for runtime shadow/AO evaluation is the natural way to avoid a second,
  independently-maintained SDF evaluator existing only for lighting).

A reasonable default for this project specifically: **bake at bake time** (first
bullet) for anything already covered by [streaming-and-invalidation](../baking-pipeline/streaming-and-invalidation.md)'s
periodic re-bake mechanism (this project's animated content — rings, pillar — already
re-bakes every `REANIMATE_INTERVAL_SECS`, so baked-in shadows on that content would
self-correct on the same cadence the geometry already refreshes on, no extra machinery
needed), reserving a real-time shader-side evaluator only if a future feature
specifically needs a fast-moving light or a shadow-caster that moves faster than the
re-bake cadence can track.

## Why relighting research (GSurf, discretized-SDF papers) undersells what this project already has

[sdf-to-gaussian-math](./sdf-to-gaussian-math.md) already notes that papers like GSurf
and the discretized-SDF relightable-assets work
([arXiv 2507.15629](https://arxiv.org/html/2507.15629)) store an SDF sample per Gaussian
specifically to *improve relighting quality* by getting better normals than
photogrammetric training alone would discover. Worth stating plainly here too: this
project already has **exact**, not discretized-and-interpolated, per-splat SDF value and
gradient at bake time — the relighting quality improvement those papers work to earn
through additional loss terms and discretized storage is a baseline property of this
project's pipeline already, not a future research contribution still to unlock. The
open work here is purely about *what to do with* that already-exact data (shadow/AO
computation and storage location, as above), not about *obtaining* it.

## When to dive in

- Adding collision/physics for SDF-authored geometry → Claybook's architecture
  (penetration depth from `|f(x)|`, contact normal from `∇f(x)`, direct GPU compute
  against the SDF) is the proven reference; GASP's point-reduction pattern is the
  concrete mechanism for keeping simulated physics results in sync with rendered
  Gaussian splats afterward.
- Adding dynamic shadows or AO to splat-rendered content → reuse the bake's existing
  SDF evaluator for the shadow-ratio and AO-deficit computations; decide bake-time vs.
  shader-time storage using the re-bake-cadence reasoning above before defaulting to
  the more expensive shader-time option.
- Wondering whether this project needs to catch up to recent SDF-for-relighting
  research → it doesn't; this project's exact analytic SDF access already exceeds what
  those papers work to approximate from photographs.

## Related

- [SDF as world description, 3DGS as world rendering](../architecture/world-representation.md) — prerequisite: the single-source-of-truth premise.
- [When to transform already-baked splats vs. re-bake from the SDF](./deformation-vs-rebake.md) — deeper: moving physics results onto splats.
- [Production case studies: Dreams and Claybook](../../sdf-3d/performance-and-production/production-case-studies.md) — deeper: Claybook's SDF physics solver.
- [Soft shadows and ambient occlusion from an SDF](../../sdf-3d/rendering/soft-shadows-and-ao.md) — deeper: the shadow and AO formulas reused here.
- [Game engine integration and production status (2025-2026)](../../3dgs/state-of-the-art/game-engine-integration.md) — contrast: production splats still need meshes for physics and cannot relight.