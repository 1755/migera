---
title: Camera collision and occlusion techniques
description: "How a third-person camera stays out of walls and keeps line of sight: sweep from a safe point inside the character, a near-plane-sized probe, snap-in/ease-out timing, minimum occlusion time, feelers, hills, ceilings, layers, avatar fade, top-down fallback. Read before building or tuning camera collision."
type: concept
status: current
tags:
  - camera
  - physics
  - correctness
  - prior-art
updated: 2026-10-10
code:
  - src/camera/collision.rs
  - src/camera/probe.rs
sources:
  - "Cinemachine Deoccluder / Decollider / ThirdPersonFollow docs: https://docs.unity3d.com/Packages/com.unity.cinemachine@3.1/manual/CinemachineDeoccluder.html"
  - "Lyra LyraCameraMode_ThirdPerson (PreventCameraPenetration, feelers): https://github.com/LeNidViolet/Lyra/tree/main/Source/LyraGame/Camera"
  - "OpenGothic camera.cpp (9-ray grid, fallback): https://raw.githubusercontent.com/Try/OpenGothic/master/common/camera.cpp"
  - "Nesky, 50 Game Camera Mistakes, items 5-13: https://gdcvault.com/play/1020460/50-Camera"
  - "Haigh-Hutchinson, Real-Time Cameras, navigation and occlusion excerpt: https://www.gamedeveloper.com/design/real-time-cameras---navigation-and-occlusion"
  - "Common problems article (Souls/AC bouncing, dithering, silhouettes): https://www.gamedeveloper.com/design/third-person-camera-view-in-games-a-record-of-the-most-common-problems-in-modern-games-solutions-taken-from-new-and-retro-games"
aliases:
  - camera penetration avoidance
  - deoccluder
  - spring arm collision
  - feelers
  - whiskers
  - safe pivot
---

# Camera collision and occlusion techniques

Camera collision is **two problems that share a sweep**:

- **Collision.** The eye or the near plane is inside geometry, so the view clips. Fix it
  *immediately*.
- **Occlusion.** Something stands between the eye and the character. Fix it *after a short
  delay*, so passing poles and foliage don't make the boom pump.

Both are resolved by shortening the boom along a sweep from a point
guaranteed to be free (a safe point inside the character). The shortened boom
eases back out slowly, after a hold. The rest of this note is refinements, each tied
to a known failure.

## The sweep

1. **Start inside the character, never at the feet or at the eye.**
   - Lyra traces from a SafeLoc inside the capsule.
   - Gothic traces from the offset pivot above the NPC.
   - Cinemachine ThirdPersonFollow traces down its shoulder→hand chain.

   If the character is pressed into a wall, even the safe point can be embedded. Walk a
   short chain of candidates (chest → head → above head) and take the first free one.
   Sweep capsule-centre → safe point → shoulder offset → eye, so a shoulder offset that
   points into a wall slides in instead of tunnelling.

   **Stop each slide a skin short of the hit**, and let a sweep that starts in contact
   ignore it while moving away. Otherwise every later sweep reads a hit at distance 0: see
   [a sweep from contact reads as a hit at zero](./a-sweep-from-contact-reads-as-a-hit-at-zero.md).
2. **Probe with a sphere at least as large as the near plane.**
   - The near-plane corners lie at distance
     `r = near · sqrt(1 + tan²(fovY/2)·(1 + aspect²))` from the eye.
   - Bevy's default `near` of 0.1 m with fovY 45° and 16:9 gives r ≈ 0.13 m.
   - A smaller probe lets corners clip even when the centre is clear.
   - Alternatives:
     - Gothic casts 9 rays to near-plane points, with 25 cm padding.
     - A box cast of the near-plane rectangle is exact.
   - Derive the radius from the live `Projection`, never as a constant, so a FOV change
     can't shrink it.
3. **Separate collision from occlusion by what moved.**
   - **Collision: snap in.** The camera's position at its current boom is inside
     geometry, *or the camera's own move since last frame passed through some*. The second
     case matters: a fast swing across a wall leaves the eye in free space on the far side,
     and only the path shows it went through.
   - **Occlusion: wait.** Something came between a camera in free space and the
     character. Pull in only after the line of sight has been blocked for a **minimum
     occlusion time**. Cinemachine has this; ≈0.1 s works; use 0 in combat.
   - migera's first version classed the swing as occlusion and let the camera sit behind
     the wall for the grace time.

## Timing: snap in, hold, ease out

- **Asymmetric response is the single biggest quality difference.**
  - Lyra blends in over 0.1 s and out over 0.15 s, with the main feeler snapping.
  - Cinemachine has Damping Into/From Collision, and Damping vs Damping When Occluded.
  - UE SpringArm and Godot SpringArm3D snap both ways, and Gothic eases both ways. Both
    behaviours are visible defects:
    - snapping out pops;
    - easing in shows the wall.
- **Hold before easing out.** Cinemachine's Smoothing Time is the minimum time held at
  the nearest point. Without it, walking along a colonnade makes the boom breathe in and
  out with every pillar.
- **Ease out with a frame-rate-independent damper**, never a per-frame lerp (see
  [camera damping](./camera-damping-is-exponential-not-a-per-frame-lerp.md)).

## Prediction

- **Feelers / whiskers.** Extra sweeps rotated off the main ray pull the camera in
  *before* the main ray is blocked. Their hits act as soft, weighted caps on distance.
  - Lyra's defaults are ±16° and ±32° yaw and ±20° pitch, with weights 0.5–1.
  - Journey reuses last frame's whisker results and swings yaw away from side occluders.
  - Swinging yaw automatically fights player intent, so make it optional; intent wins
    (Nesky #6).
- **Amortise.** Lyra re-traces a feeler that hit nothing only every 3–5 frames. One feeler
  per frame, round-robin, plus the main sweep every frame, keeps cost flat.
- **Velocity whiskers.** A feeler along the character's velocity anticipates the wall the
  player is running toward.

## Geometry-specific rules

- **Hills, not walls.** If the hit normal is mostly up, lift the eye over the ground
  (keep a clearance above the ground under the eye) instead of shortening the boom.
  Nesky #11; Cinemachine Decollider's terrain resolution.
- **Occluder behind the camera.** Pull forward; don't swing (Nesky #12).
- **Ceiling pinch.** A clearance probe above the eye should push a *soft floor on pitch*,
  so the camera tilts down under a low ceiling instead of being squeezed into the
  character.
- **Moving platforms and mounts.** Keep the pivot's lag as an *offset from the target*,
  added back each frame. A pivot stored in world space gets left behind by a moving base.
- **A shoulder against a wall: swap sides.**
  - With the character beside a wall and the camera turned along it, an over-the-shoulder
    offset toward the wall plus a boom angled slightly into it leaves the boom no room:
    0 m in migera's playground. Pulling in cannot fix that.
  - Ease over to the other shoulder while this side's boom is short and the other gives
    clearly more. Ease back once this side has room. In migera this triggers below 50% of
    the boom, needs a quarter boom more on the other side, and returns at 90%.
  - Judge by the *boom's* room, not by how far the shoulder slid: in that case the
    shoulder kept 64% of its offset while the boom had nothing.
  - TLOU and Gears-style shooters swap shoulders near cover [unverified detail].

## What must not block the camera

- **Characters.** Ignore NPCs, enemies and the player's own colliders, using collision
  layers and excluded entities. The Dark Souls III and AC Syndicate camera
  "bouncing off walls and enemy legs" in tight combat is this mistake.
- **Thin and small props.** Columns, poles, foliage and fences go on a camera-transparent
  layer, or are tagged ignorable for whiskers (Nesky #9). Cinemachine's Transparent Layers
  do the same.
- **Ragdoll bodies.** In migera they use a separate layer pool (`0xFFFF_0000`), which a
  camera blocker mask must exclude.

## When the boom gets too short

- **Fade the avatar.** Map the actual distance to an opacity, for example fully faded below
  ≈0.4 m, so the near plane never shows the inside of the body.
  - Lyra calls `OnCameraPenetratingTarget` below a threshold.
  - Skyrim has `fActorFadeOutLimit`.
  - Nesky #13.
- **Fade occluders instead of moving.** Dithered (screen-door) opacity keeps depth writes
  and avoids sorting. A silhouette or stencil can draw the character through walls
  (Mario Sunshine), and some games use a cut-out (For Honor). Witcher 3 is cited for
  dithering nearby occluders. How shipped games implement dithering is **unverified**.
  Use this per mode: pull-in suits exploration; fade suits combat, where the framing
  must not change.
- **Last-resort fallback.** If the boom stays below a minimum for a while, blend to a
  high, near-overhead view. Gothic switches to 1.5 m at 80° once collision forces the range
  under 0.8 m. Use hysteresis so the camera does not flip at the threshold.
  - Backed against a wall, the high boom at 70–80° still runs into the wall behind. Try a
    near-vertical boom too and take whichever reaches further, or the "high view" ends
    up half a metre above the head.

## Cost budget

Haigh-Hutchinson gives cameras a budget of roughly 5% of CPU, and recommends amortising
rays across frames with hysteresis statistics.

Measured in migera on 2026-10-10 with `camera_bench --collision`: 500 static avian boxes,
the whole pipeline including the boom, shoulder slide, one feeler, the ceiling and the
overlap tests. It came to about 9 µs per camera alone and 14 µs per camera at 64, against
0.2 µs without collision. Collision is almost all of the camera's cost, and still far
inside budget.

## Related

- [Third-person camera design](./third-person-camera-design.md) — applies: the `resolve_boom` stages implement this note in order.
- [A sweep from contact reads as a hit at zero](./a-sweep-from-contact-reads-as-a-hit-at-zero.md) — same-trap: why slides stop a skin short.
- [Gothic's ZenGin camera](./gothic-zengin-camera-modes-and-collision.md) — example: the ray grid and fallback, and the symmetric-timing defect.
- [Engine camera architectures compared](./engine-camera-architectures-compared.md) — deeper: the Deoccluder and Lyra feeler sources.
- [Fifty camera mistakes digest](./fifty-camera-mistakes-nesky-digest.md) — deeper: items 5–13 in their original grouping.
