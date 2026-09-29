---
title: Kill stale processes before trusting BRP
description: A leftover character_gallery keeps BRP port 15702, so queries silently read an old build; a "backward knee" was bisected across four subsystems against a stale process. Kill every instance first, and A/B the parent commit in a worktree when live data and tests disagree. Read before any BRP measurement.
type: lesson
status: current
tags:
  - debugging
  - verification
  - tooling
updated: 2026-09-28
verified: 2026-09-28
code:
  - examples/character_gallery.rs
sources:
  - Claude memory kill_stale_processes_before_trusting_brp (2026-09-28)
aliases:
  - stale process
  - port 15702 in use
  - flexion_sign
  - git worktree A/B
---

# Kill stale processes before trusting BRP

BRP binds `127.0.0.1:15702`. A second `character_gallery` cannot take the
port, so **queries go to whichever instance is still alive**, often a build
from several edits ago. Nothing errors. The numbers just describe the wrong
binary. Kill every instance before measuring.

## What happened

A "knee bending backward by 0.15 m" was re-measured, bisected across four
subsystems (gait curves, retargeting, foot IK, phase layer), and used to
justify a whole `RigGeometry::flexion_sign` mechanism. All of it was measured
against a stale process. An A/B against the parent commit showed the knee
geometry was **identical** in both builds and had never been wrong. The
`flexion_sign` mechanism no longer exists in the code.

## Why it matters

The failure is silent and the data looks plausible, so it survives the
"measure, don't assume" discipline that is supposed to catch errors. Measuring
the wrong process is still measuring.

## How to apply

1. Before any BRP measurement:
   `for p in $(pgrep -f "examples/character_gallery"); do kill -9 "$p"; done`
2. Confirm with `ps -ef | grep -c "[c]haracter_gallery"` returning 0.
   Beware `pgrep -f` matching its own wrapper shell: it reports a PID that is
   not the app, which reads as "still running" forever.
3. Suspect a stale reading when you see either of these (both seen here):
   - a value **bit-identical across many samples** while the character should
     be animating;
   - a root translation stuck at `0.0` while travel should be advancing.
4. When a live measurement and a unit test disagree, **A/B the unmodified
   parent commit in a `git worktree`** before believing either. That is what
   settled this case.

## Evidence

- `examples/character_gallery.rs` documents the stale-process trap next to its
  BRP registration.
- `RigGeometry::flexion_sign` was removed after the A/B.

## Related
- [Prefer BRP over prints for live ECS state](./prefer-brp-over-prints-for-live-ecs-state.md) — applies: the recipe this lesson guards.
- [Unsigned measurements cannot see direction](../testing/unsigned-measurements-cannot-see-direction.md) — same-trap: the real knee-direction bug that the stale-process diagnosis was confused with.
- [Knee axis: positive swings forward](../../character-animation/rig-and-retargeting/knee-axis-positive-swings-forward.md) — example: the knee convention the false diagnosis tried to "fix".
- [A measurement of a broken system](../measurement/a-measurement-of-a-broken-system.md) — same-trap: a correct measurement of the wrong thing.
- [A/B test on the same input](../measurement/ab-test-on-the-same-input.md) — deeper: the parent-commit A/B is a control that differs in one thing.
- [Verify, don't assert from memory](./verify-dont-assert-from-memory.md) — same-trap: a check that looked fresh was not.
