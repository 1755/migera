---
title: Debugging lessons
description: Lessons on diagnosis — bisect your own code first, re-verify instead of recalling, check every consumer of changed state, query live ECS state over BRP, kill stale processes that answer for old builds, and measure each link of a long chain. Read when a diagnosis is stuck or before declaring a fix verified.
type: index
status: current
tags:
  - debugging
  - verification
  - tooling
updated: 2026-09-30
---

# Debugging lessons

How diagnoses in migera went wrong, and the method that got them back on
track. Two of these are guides for live inspection (BRP); the rest are traps.

**Before any live measurement:** read
[Kill stale processes before trusting BRP](./kill-stale-processes-before-trusting-brp.md),
then [Prefer BRP over prints](./prefer-brp-over-prints-for-live-ecs-state.md).

| Note | What it establishes | Read when |
|---|---|---|
| [Bisect your own code before grepping dependency source](./bisect-before-grepping-dependency-source.md) | Bisecting your own systems is bounded; the invisible egui panel was Bevy's shadow-view camera, not bevy_egui. | When a bug might be inside Bevy or a plugin. |
| [Verify, don't assert from memory](./verify-dont-assert-from-memory.md) | Re-run or re-open a check before citing it; the "visible" panel never was. | Before rebutting a bug report with a past observation. |
| [Grep other consumers before declaring a fix done](./grep-other-consumers-before-declaring-a-fix-done.md) | A fix to shared state is verified only when every reader (and every duplicated CPU/WGSL copy) is checked. | Before saying "fixed" on a change to shared state. |
| [Prefer BRP over prints for live ECS state](./prefer-brp-over-prints-for-live-ecs-state.md) | Query world-space transforms over BRP on port 15702 instead of adding prints; recipe included. | When debugging a live pose, transform or retarget. |
| [Kill stale processes before trusting BRP](./kill-stale-processes-before-trusting-brp.md) | A leftover process keeps the BRP port and serves an old build's state; a four-subsystem false diagnosis came from it. | Before any BRP measurement, and when live data and tests disagree. |
| [The symptom is far from the cause in the rig chain](./symptom-is-far-from-cause-in-the-rig-chain.md) | Downstream code reacting correctly to bad input moves the symptom far from the cause; measure link by link. | When a character pose looks wrong. |
| [Normalize what you read back from your own output](./normalize-what-you-read-back-from-your-own-output.md) | A value read back from your own write and fed through `inverse()` keeps its norm error forever; the ragdoll's hips drifted to 1.03, scaling the skeleton 6%. | When a loop reads what it wrote, or two rotation measurements contradict each other. |

## See also
- [Measurement lessons](../measurement/INDEX.md) — numbers that looked like evidence and weren't.
- [Hybrid renderer GI and lighting](../../hybrid-architecture/gi-and-lighting/INDEX.md) — a run of sealed-room leak hunts that apply these methods to the renderer.
