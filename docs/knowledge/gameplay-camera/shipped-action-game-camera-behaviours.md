---
title: Shipped action-game camera behaviours
description: "What shipped action games do with their cameras: per-mode distances (Witcher 3, Skyrim), combat zoom-to-fit (AC3), auto-centring, lock-on (Zelda OoT, Souls), God of War's one-shot, TLOU2's camera options; unverified items marked. Read before choosing modes, lock-on or player options."
type: research
status: current
tags:
  - camera
  - prior-art
  - case-study
  - verification
updated: 2026-10-10
sources:
  - "Skyrim camera INI reference: https://stepmodifications.org/wiki/Guide:Skyrim_INI/Camera ; tweak: https://zenpad.wordpress.com/2017/03/13/tweaking-skyrim-third-person-view/"
  - "Witcher 3 next-gen camera options (search snippets; pages 403): https://vulkk.com/2022/12/14/how-to-change-the-camera-in-the-witcher-3-next-gen/ , https://primagames.com/tips/every-combat-change-in-the-witcher-3-remastered"
  - "Common camera problems in modern games: https://www.gamedeveloper.com/design/third-person-camera-view-in-games-a-record-of-the-most-common-problems-in-modern-games-solutions-taken-from-new-and-retro-games"
  - "AC3 combat camera (Wolfire GDC13 summary): https://www.wolfire.com/blog/2013/04/gdc13-summary-animation-bootcamp-part-4-6/"
  - "Zelda: Ocarina of Time (Z-targeting): https://en.wikipedia.org/wiki/The_Legend_of_Zelda:_Ocarina_of_Time"
  - "TLOU Part II accessibility: https://www.naughtydog.com/blog/the_last_of_us_part_ii_accessibility_features_detailed"
  - "Virtual camera systems / Mario 64: https://en.wikipedia.org/wiki/Virtual_camera_system ; https://www.vice.com/en/article/lights-camera-distraction-the-problem-with-virtual-camera-systems/"
  - "Eiserloh, Juicing Your Cameras With Math (GDC 2016): https://www.gdcvault.com/play/1023557/Math-for-Game-Programmers-Juicing ; implementation: https://kidscancode.org/godot_recipes/4.x/2d/screen_shake/index.html"
  - "Little Polygon on cameras: https://blog.littlepolygon.com/posts/cameras/"
  - "God of War one-shot (summary): https://gamerant.com/god-of-war-one-shot-unbroken-camera-challenging-film-technique/"
aliases:
  - Z-targeting
  - lock-on camera
  - trauma shake
  - camera accessibility options
  - fOverShoulderPosX
---

# Shipped action-game camera behaviours

Across shipped action games, the same few behaviours recur:
- **per-mode camera distances** (exploration, combat, mount);
- a **combat camera that zooms to fit the threats**;
- **auto-centring behind movement** that can be switched off;
- **lock-on** that frames player and target together;
- **player-facing options** for distance, FOV, shake and assistance.

What players complain about is just as consistent:
- cameras bouncing off enemies and walls in tight fights;
- cameras that snap back against the player's will;
- defaults that sit too far back.

Coverage of internals is thin. Items marked **[unverified]** come from memory, not a
fetched source.

## Per-mode distances and offsets

- **Witcher 3, next-gen 4.0 (2022):** separate Exploration, Combat and Horseback distance
  settings (Default/Close), plus an Automatic Camera Centering toggle. A **Dynamic** combat
  distance zooms out when several enemies surround Geralt and in for a single enemy; lock-on
  switching was made quicker. Sourced from snippets of 403'd pages. It is also cited for
  dithering nearby occluding geometry.
- **Skyrim:** `fOverShoulderPosX` 30, `fOverShoulderPosZ` −10 (Creation Engine units), with
  separate combat values. Mounts add height: `fOverShoulderHorseAddY` −300 and
  `fOverShoulderDragonAddY` −600. `fVanityModeMinDist`/`MaxDist` are 155/600, and an idle
  "vanity" orbit starts after `fAutoVanityModeDelay` 120 s. Wheel zoom is
  `fMouseWheelZoomSpeed` 0.8, `fMouseWheelZoomIncrement` 0.075. The avatar fades at
  `fActorFadeOutLimit`. The popular community tweak (`PosX` 37.5, `PosZ` 7.5) says players
  want a slightly wider, higher shoulder. Collision behaviour: not found.
- **Gothic:** about 30 named mode sets; see
  [Gothic's ZenGin camera](./gothic-zengin-camera-modes-and-collision.md).

## Combat framing

- **Assassin's Creed III:** the combat camera zooms dynamically "to be as close as it can
  possibly be without leaving important enemies out of the frame". Each finisher has its
  own authored camera.
- **Batman Arkham:** auto-centres during combat and sprint. Its combat camera shifts are
  cited as a nausea source.
- **Dark Souls III / AC Syndicate:** the sphere-collider camera "bounces off walls and enemy
  legs" in tight combat. The lesson: characters must not block the camera.
- **FromSoftware lock-on [unverified]:**
  - Targets are chosen from a view cone with a distance limit; switching uses a right-stick
    flick.
  - The lock breaks at distance or when line of sight is lost.
  - Pressing lock with no target recentres the camera behind the player.
  - Elden Ring exposes auto wall recovery, auto rotation and auto lock-on options.
  - The common complaint is large bosses filling the frame and the camera wedging into
    corners.

## Lock-on origin

**Zelda: Ocarina of Time** introduced Z-targeting, modelled on chanbara sword-fight films.
While targeting:
- the camera follows the target and Link always faces it;
- projectiles aim at it automatically;
- the view letterboxes and an arrow marks the target (the marker became Navi).

The rest of the time the camera was mostly AI-driven with limited player control.
Claim [unverified]: the idea of locking onto one attacker at a time came from a chanbara
show at Toei Kyoto Studio Park.

## Anti-patterns with names

- **Mario 64 snap-back:** the camera returns to where *it* likes after the player moves it.
  Automation overrode intent.
- **Walk-cycle bob and running wobble (Gears 4):** cited as nausea risks.
- **The Last Guardian:** the companion (Trico) fills the screen and occludes the view.
- **Gothic 1 Remake default:** off-centre and too far back, according to players; a centred
  option was added.

## Signature styles

- **God of War (2018):** a tight right-shoulder camera, presented as one unbroken shot of
  about 100 stitched long takes. There are no establishing fly-throughs; scale comes from
  Kratos looking up. No numbers available.
- **Mario 64:** the camera pre-rotates toward upcoming path turns, so it is aware of the
  level.
- **Solar Ash (Little Polygon):** tracks a ground-plane projection so jumps don't move the
  camera. A "leash" yaw term comes from the cross product of view direction and velocity.

## Shake

**Eiserloh's trauma model (GDC 2016):**
- Events add to `trauma` ∈ [0, 1], which decays linearly at about 0.8/s.
- Shake = `trauma²` (or `³`) × max × noise, using smooth noise (Perlin/OpenSimplex) with a
  separate seed per channel, not per-frame random values.
- Squaring makes small hits barely register while big ones stack.
- In 3D, prefer **rotational** shake (yaw/pitch/roll) over translation, which can push the
  camera into walls. This point is from memory of the talk.

## Player options: TLOU Part II (sourced)

- **Camera Assist:** turns the camera toward the movement direction; can be limited to
  horizontal or vertical only.
- **Lock-On Aim:** Off / On / Auto-Target, with a 1–10 strength. It targets centre mass,
  and the stick shifts to head or legs. Separate arc-throw lock-on.
- **Camera Shake** 1–10 and **Motion Blur** 1–10.
- **Camera Distance** −5…+5 and **FOV** −5…+5.
- **Dolly Zoom Effect** toggle and **Full Screen Effects** toggle.
- **Persistent Center Dot:** a motion-sickness aid.

Ghost of Tsushima and God of War Ragnarök offer similar sets [unverified].

## Relevance to migera

- Expose **per-mode distances** and a **dynamic combat distance** (target-group framing).
- **Auto-centre** with a delay and an off switch.
- **Lock-on** with cone selection, flick switching and break rules.
- **Ignore characters** in collision.
- Shake as **rotational trauma²** with a comfort scale.
- A TLOU2-style **option set** from the start: shake, distance and FOV offsets, assist
  strength, lock-on hold vs toggle, per-axis invert and sensitivity.

## Related

- [Fifty camera mistakes digest](./fifty-camera-mistakes-nesky-digest.md) — deeper: the principles behind most of these behaviours.
- [Third-person camera design](./third-person-camera-design.md) — applies: lock-on, target groups, effects and options.
- [Gothic's ZenGin camera](./gothic-zengin-camera-modes-and-collision.md) — example: a full per-mode table with numbers.
- [Camera collision and occlusion techniques](./camera-collision-and-occlusion-techniques.md) — deeper: why characters must not block the camera.
