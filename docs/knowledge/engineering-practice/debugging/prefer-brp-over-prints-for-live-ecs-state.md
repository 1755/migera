---
title: Prefer BRP over prints for live ECS state
description: character_gallery already serves the Bevy Remote Protocol on 127.0.0.1:15702; query world-space GlobalTransforms with world.query instead of adding eprintln! or env-var dumps. Gives live ground truth with zero source changes. Read before debugging any live pose, transform or retargeting question.
type: guide
status: current
tags:
  - debugging
  - ecs
  - bevy
  - tooling
updated: 2026-09-23
verified: 2026-09-28
code:
  - examples/character_gallery.rs
sources:
  - Claude memory prefer_brp_over_prints_for_live_ecs_state (2026-09-23)
aliases:
  - Bevy Remote Protocol
  - RemoteHttpPlugin
  - curl localhost:15702
  - world.query
---

# Prefer BRP over prints for live ECS state

For any "what does the live ECS state look like right now" question in
migera's Bevy examples, query the **Bevy Remote Protocol** (BRP) instead of
threading `eprintln!` or env-var-gated dump code through the source.
`examples/character_gallery.rs` already registers `RemotePlugin` +
`RemoteHttpPlugin` on `127.0.0.1:15702` for exactly this purpose.

## When to use

- Checking a live pose, a bone's world-space position or rotation, or a
  retargeting result.
- Checking where a ragdoll body actually is, or whether a value is animating.
- Not for things BRP cannot see: frame ordering and timing, values that are not
  reflected, or state that is not a queryable component. Use prints for those.

## Steps

0. **Kill stale instances first.** A leftover process holds the port and
   answers for an old build. See
   [Kill stale processes before trusting BRP](./kill-stale-processes-before-trusting-brp.md).
1. List the available methods. Names vary by Bevy version, so do not assume
   `bevy/list`; this version uses `world.query` and similar:
   `curl -s http://127.0.0.1:15702 -d '{"jsonrpc":"2.0","id":1,"method":"rpc.discover","params":{}}'`
2. Call `world.query` with `data.components` naming full Rust type paths, for
   example `bevy_transform::components::global_transform::GlobalTransform` and
   `bevy_ecs::name::Name`. Filter with `filter.with`. This returns real, live,
   **world-space** values.
3. Compute derived quantities (segment angles, distances) from those values in
   a scratch script, not by eyeballing a screenshot or a HUD line.

## Why this beats prints here

- The HUD/log line in `character_gallery.rs` prints each bone's
  `Transform.translation`/rotation, which is **local** space. That is nearly
  useless for checking a chain's world-space bend.
- A print-based dump (such as the `MIGERA_DUMP_REST` env-var gate) means edit,
  rebuild, rerun, then edit the diagnostic back out: extra round trips and a
  diff to remember to revert. BRP needs zero source changes and reads the
  process that is actually running.

## Related
- [Kill stale processes before trusting BRP](./kill-stale-processes-before-trusting-brp.md) — prerequisite: the silent failure mode of this recipe.
- [Cohesion tests miss free fall](../testing/cohesion-tests-miss-free-fall.md) — example: an absolute-position failure that one BRP query shows immediately.
- [Parse the asset, don't transcribe it](../testing/parse-the-asset-dont-transcribe-it.md) — example: parsed foot pitch checked against the live rig over BRP.
- [Full-strength read-back hides the physics](../../character-animation/ragdoll-and-physics/full-strength-readback-hides-the-physics.md) — applies: verifying ragdoll bodies against targets over BRP.
- [Gizmos need --show-real-mesh off](../../character-animation/animation-core/gizmos-need-show-real-mesh-off.md) — contrast: the visual ground-truth view, for when you need to see the shape rather than read numbers.
