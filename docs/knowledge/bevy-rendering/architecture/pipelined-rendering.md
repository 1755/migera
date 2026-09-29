---
title: "Pipelined rendering: overlapping simulation and GPU submission"
description: Bevy 0.19.1's PipelinedRenderingPlugin hands the whole render SubApp back and forth to a dedicated render thread over capacity-1 channels, so frame N renders while frame N+1 simulates; extraction stays on the main thread. Read when debugging one-frame-late data, frame pacing, or thread-context assumptions.
type: reference
status: current
tags:
  - bevy
  - render-pipeline
  - ecs
  - performance
updated: 2026-08-15
verified: 2026-08-15
sources:
  - bevy_render-0.19.1/src/pipelined_rendering.rs
aliases:
  - PipelinedRenderingPlugin
  - render thread
  - one frame late
  - RenderAppChannels
---

# Pipelined rendering: overlapping simulation and GPU submission

## Why this exists

Running simulation (main-world update) and rendering (render-world update + GPU
submission) sequentially on one thread wastes CPU idle time: while the GPU
driver/backend is busy submitting or waiting, the CPU could already be simulating the
next frame. `PipelinedRenderingPlugin` overlaps frame N's rendering with frame N+1's
simulation on separate OS threads.

## How it works

The render `SubApp` is handed off to a dedicated **render thread**, and communication
happens via two bounded (capacity-1) `async_channel`s wrapped in `RenderAppChannels`. This
is a ping-pong hand-off of *ownership* of the entire render `SubApp` (and its `World`)
between threads — not a copy — which sidesteps needing locks on shared render-world
state.

Setup happens once, in `PipelinedRenderingPlugin::cleanup()` (called after all plugins are
built):

1. Two channels are created; the `RenderApp` sub-app is removed from the main `App` and
   immediately sent to the render thread — so at startup the render thread "owns" it
   first.
2. A background thread is spawned running a loop: block-receive a `SubApp` from the
   "app → render" channel, call `render_app.update()` (runs the actual `Render` schedule —
   GPU submission work), then send it back via the "render → app" channel.
3. A second, tiny sub-app (`RenderExtractApp`) is inserted into the *main* app, whose
   extract function is `renderer_extract` — it runs as an ordinary part of the main app's
   update sequence.

Each frame, `renderer_extract` (running on the **main thread**) drives the handshake:

1. It blocks (cooperatively, via a shared `MainThreadExecutor`) until the render thread
   sends back the `SubApp` from finishing the *previous* frame's rendering.
2. Once received, it calls `render_app.extract(world)` **on the main thread** — this is
   the `ExtractSchedule` step (entity sync + component/resource copying, see
   [render-app-and-extraction](./render-app-and-extraction.md)), run synchronously while
   the main world is available.
3. It immediately sends the (now-updated) `render_app` back to the render thread, which
   kicks off that frame's `Render` schedule (GPU work) — while the main thread proceeds to
   run the rest of its own schedule for the *next* frame's simulation.

This produces the timeline: frame 1 sim → (frame 1 render || frame 2 sim) → (frame 2
render || frame 3 sim) → ... Because `MainThreadExecutor` is shared (cloned into the
render world), any main-thread-only tasks the render world needs (certain windowing/
graphics APIs) can still run cooperatively on the main thread even while "the render app"
is conceptually busy on its own thread.

`RenderAppChannels::drop` guards shutdown: if the render `SubApp` is currently on the
render thread when the channel resource is dropped, it blocks waiting to receive it back
first — ensuring non-`Send` GPU resources created on the main thread are dropped on the
correct thread rather than leaking or crashing.

## The key mental model

**`ExtractSchedule` always runs on the main thread**, regardless of whether pipelined
rendering is enabled — that's why `Extract<...>` (see
[entity-sync-and-extraction-patterns](./entity-sync-and-extraction-patterns.md)) is safe
to use there. Extraction for frame N happens sandwiched between frame N-1's render-thread
completion and frame N's render-thread kickoff, not "at the same time" as anything else.

## When to dive in

- Debugging frame-pacing, input-latency, or "why did my resource change one frame late"
  issues → this file's timeline is the mental model to use; extraction for frame N is not
  concurrent with anything, but the *render* work for frame N-1 can still be finishing on
  the GPU thread while frame N's simulation logic runs.
- Writing code that assumes a particular main-thread-vs-render-thread execution context →
  confirm whether pipelined rendering is enabled in the target configuration (it's
  disabled on some platforms/backends by default).

## Related
- [The RenderApp split and the Extract step](./render-app-and-extraction.md) — prerequisite: the SubApp and extraction step whose ownership is handed between threads.
- [CPU-GPU data flow](../../compute-shaders/cpu-gpu-data-flow.md) — deeper: upload and asynchronous readback latency on top of the one-frame render lag described here.
