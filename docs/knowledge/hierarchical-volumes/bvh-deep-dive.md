---
title: BVH deep dive
description: Covers BVH construction (median, LBVH/Morton, bucketed SAH, SBVH), traversal (ordered stack, short-stack, ropes, wide nodes), GPU memory layout, and two migera-found traps - padded queries must pad every node, and fail-safe fallbacks need their own test. Read before changing src/hybrid/bvh.rs or BVH traversal in WGSL.
type: research
status: current
tags:
  - spatial-acceleration
  - bounding-volumes
  - ray-tracing
  - performance
  - correctness
  - testing
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/hybrid/bvh.rs
  - assets/shaders/hybrid_trace.wgsl
sources:
  - PBRT v3 ch.4.3 (pbr-book.org)
  - Lauterbach et al. 2009 (LBVH)
  - Popov et al., Object Partitioning Considered Harmful
  - Horn et al. (short-stack traversal)
  - commit 8f1bdeb
  - commit 942246c
  - commit f4885a2
aliases:
  - bounding volume hierarchy
  - SAH
  - LBVH
  - Morton code
  - short-stack traversal
  - BVH refit
---

# BVH deep dive

Sources: PBRT v3 ch.4.3 (pbr-book.org), CTU Prague spatial-acceleration course notes,
Popov et al. "Object Partitioning Considered Harmful", Lauterbach et al. 2009 (LBVH),
Garanzha et al. (parallel LBVH), Hunt/Marks/Strezh (stackless), Horn (short-stack),
Woop et al. Embree wide-BVH work, NVIDIA GPUOpen/Vulkan docs.

## Construction

**Quality ladder** (build cost rising):

1. **Median/equal-count split** - sort centroids on the widest axis, split in half.
   Fast, low quality; fine for tiny leaf counts.
2. **LBVH / Morton**: quantize centroids, interleave bits into Morton codes, radix-sort,
   split ranges where codes diverge. O(N), embarrassingly parallel, GPU-native
   (radix sort + one thread per node). Quality below SAH, degrades with mixed-size
   primitives (uses centroids only).
3. **SAH (bucketed)**: per node, per axis: sort/accumulate centroid-sorted primitive
   boxes into ~12 buckets; pick boundary minimizing
   `cost = C_trav + SA(L)/SA(P)*N_L*C_int + SA(R)/SA(P)*N_R*C_int`.
   The production standard for static CPU builds (~2x traversal speed vs naive).
4. **SBVH-style space-splitting mixes**: allow splitting primitives across children to
   cut box overlap - measurable gains on dense meshes.

**GPU build pipeline (LBVH)**: compute centroids -> morton encode -> radix sort by
code -> build hierarchy over sorted-code runs (each internal node = split of a code
range) -> compute AABBs bottom-up -> compact to pointerless linear layout.
Everything is fixed-size arrays and atomic counters - maps 1:1 onto WGSL compute.

## Traversal

- **Ordered descent with explicit stack** (the default): test both children's boxes,
  push far child, descend near child; prune when node t_enter >= closest hit so far.
  Stack depth = tree height (<=64 for any practical scene).
- **Short-stack** (Horn): fixed-size small stack; on overflow restart from root with a
  narrowed ray window. Cache-friendlier, slightly more tests. GVDB adapted exactly this
  for hierarchical DDA ("short stack of exit points").
- **Stackless via ropes** (kd-tree heritage): each leaf stores neighbor links; climb
  without stack. More memory traffic; rarely wins on modern GPUs vs short-stack.
- **Octree PUSH/ADVANCE/POP** (Laine & Karras SVO): incremental cube stepping with
  scale-indexed parent stack; the canonical sparse-octree ray walk (details in the
  hierarchical-grids doc).
- **Wide nodes** (BVH4/BVH8): fewer memory hops, SIMD-friendly child-box batch tests;
  what Embree/Hardware RT effectively use.

## Memory layout best practices

- Pointerless linear arrays (children as index ranges or first-child+count) - no
  pointers across PCIe/WGSL.
- Interleave node data SoA-style for coalescing; keep child boxes adjacent to nodes.
- Order leaves by Morton code for spatial locality of referenced primitives.
- Keep per-node footprint small (two boxes + two child refs = ~56 bytes for BVH2);
  compress normals/etc. out-of-band (SVO lesson).

## Padded/expanded queries (near-miss, soft-shadow, proximity)

A query that pads or expands leaf primitive bounds before the geometric test — e.g. a
soft-shadow ray's "near-miss" candidate gather, or any proximity/broad-phase query with
a margin — cannot pad leaves alone and leave internal (ancestor) node bounds tight.
Internal node bounds are normally computed as the union of children's own (tight)
bounds; if leaves get padded only at test time, the ancestor chain has no knowledge of
that padding and remains a bound on the *unpadded* geometry. A ray that only enters a
leaf's padded margin region (missing the leaf's tight box) can then also miss that
leaf's tight-bounded parent during descent, and the whole subtree is pruned before the
leaf's own correctly-padded test is ever reached — a silent false negative, not a
crash, so it reads as a shading/quality bug (a hard cliff between angularly-close rays)
rather than an obvious traversal failure. Fix: pad every node in the padded query's
descent uniformly by the same margin, both internal and leaf — still a valid
conservative bound, since a parent padded by `margin` fully contains any child padded
by `<= margin`. Real incident and full symptom description (a sharp "cut" artifact in
an otherwise-correct soft shadow, found via an angular ring-profile scan, not a
screenshot): [soft-shadows-and-ao.md](../sdf-3d/rendering/soft-shadows-and-ao.md#porting-to-srchybrid-three-more-bugs-the-hybrid_legacy-port-didnt-warn-about).

## Worst practices

- Rebuilding an SAH BVH every frame for dynamic content on CPU (build cost dominates;
  use refit-or-LBVH-on-GPU instead).
- Recursive traversal in WGSL (no recursion) - must be explicit-stack iterative.
- Testing children unordered when ordered descent is free (near-first halves visits).
- Ignoring box overlap: overlapping sibling boxes force double visits; consider
  space-splitting if geometry is dense/overlapping.
- Per-thread giant stacks in registers (occupancy death) - cap depth, use short-stack.
- **Trusting a doc comment as evidence a "fail-safe" fallback path is actually safe.**
  A pathological-input fallback (cell-count overflow, degenerate bounds, empty input)
  needs its own test that actually triggers the fallback condition and checks the
  query's real answer — a comment claiming "degrades to always-non-empty, the safe
  direction" is not proof the code does that; a boolean can be flipped (`vec![false]`
  where `vec![true]` was intended) and every other test can still pass, because tests
  built for the normal-input path never exercise the fallback branch at all. Real
  incident, migera's own occupancy-grid accelerator: found via a performance sweep
  that produced a suspiciously large "speedup" which was actually silently-broken
  correctness (the fallback reported "always empty" instead of "always occupied"),
  not caught by any prior test — see
  [soft-shadows-and-ao.md](../sdf-3d/rendering/soft-shadows-and-ao.md) and
  `PROGRESS.md`'s "Stage C" entry for the full repro and fix (an explicit boolean
  flag checked first and unconditionally, rather than relying on a synthetic
  degenerate grid's own geometry to resolve to the safe answer).

## Relevance to migera

`src/hybrid/bvh.rs` follows this note: bucketed SAH (12 buckets, all 3 axes, median
fallback for degenerate splits) and refit instead of per-frame rebuild (commit
8f1bdeb, 2026-09-06), and near-first ordered descent in `hybrid_trace.wgsl` (commit
942246c, 2026-09-07). Its GPU nodes are pointerless (`BvhNodeGpu`: six scalar bounds +
two child/object indices). Cone tracing's padded descent tightens the per-node pad
(`cone_slab_hit`) while keeping every node padded, per the section above.

## Related
- [Acceleration structures around AABBs](../aabb-acceleration/acceleration-structures.md) — prerequisite: where BVHs sit against grids, octrees and kd-trees.
- [Ray–AABB intersection: the slab method](../aabb-acceleration/ray-aabb-slab-test.md) — prerequisite: the per-node test and the `t_near` ordered descent sorts by.
- [Hierarchical grids & VDB-style trees](./hierarchical-grids-and-trees.md) — contrast: fixed-depth grid hierarchies and the SVO traversal referenced above.
- [Soft shadows and ambient occlusion from an SDF](../sdf-3d/rendering/soft-shadows-and-ao.md#porting-to-srchybrid-three-more-bugs-the-hybrid_legacy-port-didnt-warn-about) — example: the unpadded-internal-node shadow "cut" incident.
- [Module and stage skeleton](../hybrid-architecture/module-and-stage-skeleton.md) — applies: the measured AABB/BVH win in `src/hybrid` and the declined inner-AABB idea.
- [Occupancy first-pass design](./occupancy-first-pass-design.md) — contrast: the grid gate layered on this BVH, built twice and reverted with no measured win.
- [Same function both sides is a vacuous test](../engineering-practice/testing/same-function-both-sides-is-a-vacuous-test.md) — same-trap: a test suite that never exercises the path it claims to protect.
