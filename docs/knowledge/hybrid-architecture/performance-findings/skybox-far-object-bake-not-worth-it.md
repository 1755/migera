---
title: A skybox or far-object bake would not speed up the hybrid renderer
description: Baking distant content into an amortized skybox was researched and not built; a ray that misses the scene already costs ~O(1) (~8–9 ms vs ~133–150 ms for a hit-heavy view at --stress 10000), and a far-objects-only bake risks shadow and occlusion leaks. Read before proposing a skybox or far-field cache for speed.
type: decision
status: current
tags:
  - performance
  - baking
  - spatial-acceleration
  - hybrid-renderer
updated: 2026-09-19
verified: 2026-09-28
code:
  - src/hybrid/bvh.rs
  - assets/shaders/hybrid_trace.wgsl
sources:
  - Claude memory skybox_far_object_bake_measured_not_implemented (2026-09-19)
  - commit 503ec26
  - PROGRESS.md "Performance investigation: skybox/far-object baking (measured, not implemented)" entry
aliases:
  - skybox bake
  - far-field cache
  - impostors
  - scene miss cost
---

# A skybox or far-object bake would not speed up the hybrid renderer

On 2026-09-19 the user asked to research baking distant objects into a skybox
texture refreshed over N frames (amortized like DDGI's probe relight). It was
**measured and not built**: rays that miss the scene are already nearly free,
and the far-objects-only variant carries real correctness risks for an unproven
gain.

## Context

The idea: primary rays that would march against distant geometry every frame
sample a cache instead, updated round-robin like `ddgi_probes_per_frame`.

## Decision

Not implemented, based on a direct measurement:
- At `--stress 10000`, a view where rays miss the whole scene costs ~8–9 ms of
  `gpu trace`, against ~133–150 ms for a near-orbit, hit-heavy view.
- A scene miss terminates after one root-node BVH slab test, independent of
  object count. This matches the earlier finding that march steps are not the
  bottleneck.

## Alternatives considered

- **Whole-scene skybox** (a real baked sky instead of the flat debug-magenta
  background). A visual feature, not a speed-up, since sky pixels are already
  nearly free. If built, reuse DDGI's `ddgi_probe_relight_start` round-robin.
- **Far-objects-only bake** (remove distant geometry from the live BVH, sample a
  cache). The version that might help, but its premise failed once the real
  cost driver was found (DDGI occlusion, ~48–52% of the frame). It also risks
  the `646a388` leak class: shadow rays, DDGI probes, cone traces and reflection
  rays all return **black on a miss** by design, for sealed-room safety.
  Removing far objects from the live scene loses the shadowing and occlusion
  they still provide to near objects.

## Consequences

- No skybox or far-field cache exists. The background stays a debug colour.

## Revisit when

- Re-measure "does a miss cost more than a hit" on the scene that motivates it
  before building anything for performance. The premise is true in many
  renderers but not in this one, because of its slab-test and best-t pruning.
- A real sky is wanted as a **visual** feature. Scope and justify it that way.

## Related
- [Trace-pass bottleneck is not march steps](./trace-pass-bottleneck-is-not-march-steps.md) — prerequisite: the earlier finding this measurement matches.
- [DDGI any-hit occlusion](../gi-and-lighting/ddgi-any-hit-occlusion.md) — deeper: the real dominant cost found in the same investigation.
- [DDGI probe-grid bounds wall-embedding leak](../gi-and-lighting/ddgi-probe-grid-bounds-wall-embedding-leak.md) — contrast: the leak class a far-object bake would risk reopening.
- [Ray–AABB slab test](../../aabb-acceleration/ray-aabb-slab-test.md) — deeper: why a scene miss costs one test.
- [A/B test a feature on the same input](../../engineering-practice/measurement/ab-test-on-the-same-input.md) — applies: measure the premise on this renderer before building.
