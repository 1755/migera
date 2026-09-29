---
title: Verify, don't assert from memory
description: Before claiming an earlier screenshot or check shows something is fine, re-run or re-open it now; the "visible" egui panel had never been visible. Treat user pushback on "I already verified this" as a signal to re-verify immediately. Read before rebutting a bug report with a past observation.
type: lesson
status: current
tags:
  - verification
  - debugging
updated: 2026-09-21
verified: 2026-09-21
sources:
  - Claude memory verify_dont_assert_from_memory (2026-09-21)
aliases:
  - re-verify
  - user pushback
  - trusting recollection
---

# Verify, don't assert from memory

When re-asserting a past observation as still true, re-derive it: re-run the
check or re-open the file. Do not rely on remembering that it passed,
especially when the user contradicts the memory.

## What happened

On 2026-09-21 the user reported that the egui Controls panel in
`character_gallery` was not visible. The agent replied that the report was
probably a misunderstanding, because "the panel renders in my own `--shot`
screenshots throughout this session".

The user pushed back ("on your screenshots I also don't see it"). A fresh look
at an actual screenshot showed the panel was **not** there, and never had been
for this example. The confidence came from misremembered screenshots, possibly
conflated with `gallery.rs`, not from a re-check. The correction led directly
to a real, reproducible bug (a shadow-view camera capturing the egui context).

## Why it matters

Memory of "I checked this" is not a check. It feels like evidence, it is cheap
to repeat, and it is often about a different run, example or build.

## How to apply

- Before saying "this is fine, I saw it", re-open the screenshot or re-run the
  command.
- When a user contradicts a past verification, re-verify first and explain
  afterwards.
- Name the artifact (file, frame, command) that shows the claim, so the claim
  can be checked.

## Evidence

- The bug found once the claim was re-checked is described in
  [Bisect your own code before grepping dependency source](./bisect-before-grepping-dependency-source.md).

## Related
- [Bisect your own code before grepping dependency source](./bisect-before-grepping-dependency-source.md) — example: the root cause found once the claim was re-checked.
- [Kill stale processes before trusting BRP](./kill-stale-processes-before-trusting-brp.md) — same-trap: a "fresh" measurement that was really about an old build.
- [A measurement of a broken system](../measurement/a-measurement-of-a-broken-system.md) — deeper: even a real, recent measurement inherits the validity of what it measured.
- [migera moves characters to Bevy's PBR pipeline](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — example: the milestone during which this happened.
- [DDGI sealed-room light leak](../../hybrid-architecture/gi-and-lighting/ddgi-sealed-room-light-leak.md) — example: a "new regression" that was leftover debug code, settled by re-checking what was actually running.
